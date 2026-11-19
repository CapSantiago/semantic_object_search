#!/usr/bin/env python3
"""Run one room cleaning request through the console's existing Nav2 stack."""
import argparse
import json
import math
from pathlib import Path
import signal
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_console import ROOT, write_json
from semantic_search.cleaning import (CENTER_CLEARANCE_M, LANE_SPACING_M, RETURN_POSITION_M,
                                      RETURN_YAW_RAD, SWEEP_WIDTH_M, TARGET_COVERAGE, RoomSweep)
from semantic_search.rooms import belongs_to


class StopRequested(Exception):
    pass


def yaw(rotation):
    return math.atan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                      1-2*(rotation.y*rotation.y+rotation.z*rotation.z))


class CleaningRun:
    def __init__(self, args, stopped):
        self.args, self.stopped = args, stopped
        self.node = Node('room_cleaning', parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self.node)
        self.client = ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        grid_qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid,'/map',self.receive_map,grid_qos)
        self.node.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.receive_costmap,grid_qos)
        self.node.create_subscription(Odometry,'/odom',self.receive_odom,qos_profile_sensor_data)
        self.started = time.monotonic()
        self.deadline = self.started+args.timeout
        self.rooms = json.loads(args.rooms.read_text())
        self.sweep = self.grid = self.costmap = self.start_pose = self.final_pose = None
        self.costmap_wall = None
        self.odom_wall = self.odom_stamp = self.stationary_since = None
        self.trace, self.goals = [], []
        self.distance = 0.
        self.stage = 'planning'
        self.settled = True
        self.return_state = 'not_attempted'
        self.return_errors = None
        self.next_report = self.next_sample = 0.
        self.planned_goals = self.finished_goals = 0
        self.skipped_lanes = 0
        self.last_pose_wall = self.started
        self.sample_gap = False

    def receive_map(self, message):
        if message.header.frame_id=='map' and self.grid is None:
            self.grid = message

    def receive_costmap(self, message):
        if message.header.frame_id=='map':
            self.costmap,self.costmap_wall = message,time.monotonic()

    def costmap_ready(self):
        return self.costmap_wall is not None and time.monotonic()-self.costmap_wall < 3.

    def available(self):
        self.wait_for(self.costmap_ready,3)
        source, live = self.grid.info,self.costmap.info
        same_grid = (
            (source.width,source.height)==(live.width,live.height)
            and abs(source.resolution-live.resolution)<1e-5
            and math.dist((source.origin.position.x,source.origin.position.y),
                          (live.origin.position.x,live.origin.position.y))<1e-4
            and abs(yaw(source.origin.orientation)-yaw(live.origin.orientation))<1e-4
        )
        if not same_grid:
            raise ValueError('导航代价地图与清扫底图的坐标格网不一致。')
        cost = np.asarray(self.costmap.data).reshape(live.height,live.width)
        return (cost>=0) & (cost<99)

    def receive_odom(self, message):
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        if self.odom_stamp is not None and stamp<=self.odom_stamp:
            return
        self.odom_stamp, self.odom_wall = stamp,time.monotonic()
        velocity = message.twist.twist
        slow = math.hypot(velocity.linear.x,velocity.linear.y)<.015 and abs(velocity.angular.z)<.04
        if slow:
            if self.stationary_since is None:
                self.stationary_since = stamp
        else:
            self.stationary_since = None

    def parked(self):
        return (self.stationary_since is not None and time.monotonic()-self.odom_wall<=.40
                and self.odom_stamp-self.stationary_since>=450_000_000)

    def pose(self):
        now = self.node.get_clock().now().nanoseconds
        if self.odom_wall is None or time.monotonic()-self.odom_wall>.60:
            return None
        try:
            transform = self.tf.lookup_transform('map','base_link',Time())
        except TransformException:
            return None
        age = now-Time.from_msg(transform.header.stamp).nanoseconds
        p = transform.transform.translation
        value = (p.x,p.y,yaw(transform.transform.rotation))
        if -1_500_000_000 <= age <= 750_000_000 and all(math.isfinite(v) for v in value):
            return value
        return None

    def tick(self):
        rclpy.spin_once(self.node,timeout_sec=.06)
        now = time.monotonic()
        if now>=self.next_sample:
            self.next_sample = now+.12
            current = self.pose()
            if current is None:
                self.sample_gap = True
            elif self.start_pose is not None:
                self.last_pose_wall = now
                stamp = self.node.get_clock().now().nanoseconds/1e9
                previous = self.trace[-1] if self.trace else None
                distance = math.dist(previous['pose'][:2],current[:2]) if previous else 0.
                connected = (previous is not None and not self.sample_gap and distance<=.30
                             and 0 < stamp-previous['sim_seconds']<=.6)
                self.distance += distance if connected else 0.
                self.sample_gap = False
                cleaning = self.stage in ('sweeping','supplementing')
                if cleaning and belongs_to(self.rooms,self.args.room,*current[:2]):
                    self.sweep.touch(*current[:2],stamp)
                if previous is None or stamp>previous['sim_seconds']:
                    self.trace.append(dict(pose=current,sim_seconds=stamp,stage=self.stage,connected=connected))
        if now>=self.next_report:
            self.next_report = now+1.
            write_json(self.args.output/'clean_progress.json',self.progress())

    def progress(self):
        step = max(1,math.ceil(len(self.trace)/350))
        preview, previous = [], -1
        for index in range(len(self.trace)):
            row = self.trace[index]
            if index % step and index != len(self.trace)-1 and row['stage']==self.trace[index-1]['stage']:
                continue
            connected = previous>=0 and all(sample['connected'] for sample in self.trace[previous+1:index+1])
            preview.append([round(row['pose'][0],3),round(row['pose'][1],3),row['stage'],connected])
            previous = index
        return dict(request_id=self.args.output.name,room=self.args.room,stage=self.stage,frame_id='map',
                    goals_total=self.planned_goals, goals_finished=self.finished_goals,
                    blocked_lanes=self.skipped_lanes,
                    coverage=self.sweep.metrics() if self.sweep else None, trace_preview=preview,
                    return_state=self.return_state)

    def wait_for(self, predicate, timeout, *, check_stop=True, require_pose=False):
        deadline = time.monotonic()+max(0.,timeout)
        while True:
            if check_stop and self.stopped:
                raise StopRequested()
            if predicate():
                return
            if time.monotonic()>=deadline or require_pose and time.monotonic()-self.last_pose_wall>1.5:
                raise TimeoutError('导航响应或定位数据超时。')
            self.tick()

    def go(self, target, deadline, *, timeout=40., purpose=None):
        if self.stopped:
            raise StopRequested()
        if not self.pose():
            raise ValueError('定位数据不可用，已停止清扫任务。')
        self.last_pose_wall = time.monotonic()
        if time.monotonic()>=deadline:
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y = map(float,target[:2])
        goal.pose.pose.orientation.z = math.sin(target[2]/2)
        goal.pose.pose.orientation.w = math.cos(target[2]/2)
        if self.stage in ('sweeping','supplementing'):
            goal.behavior_tree = str(ROOT/'src/semantic_search/config/navigate_cleaning_lane.xml')
        row = dict(stage=self.stage,target=list(target),status='sending')
        if purpose:
            row['purpose'] = purpose
        self.goals.append(row)
        self.settled = False
        sent = self.client.send_goal_async(goal)
        handle = action_result = None
        try:
            self.wait_for(sent.done,8)
            handle = sent.result()
            if not handle.accepted:
                self.settled = True
                row['status'] = 'rejected'
                return False
            action_result = handle.get_result_async()
            self.wait_for(action_result.done,min(timeout,deadline-time.monotonic()),require_pose=True)
        except (StopRequested, TimeoutError) as error:
            if handle is None:
                # The command may have reached Nav2 even if its response is late.
                self.wait_for(sent.done,2,check_stop=False)
                handle = sent.result()
            if handle.accepted:
                action_result = action_result or handle.get_result_async()
                if not action_result.done():
                    handle.cancel_goal_async()
                    self.wait_for(action_result.done,3,check_stop=False)
            self.settled = not handle.accepted or action_result.result().status in (
                GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_ABORTED,GoalStatus.STATUS_CANCELED)
            row['status'] = 'canceled' if isinstance(error,StopRequested) else 'timed_out'
            if isinstance(error,StopRequested) or not self.settled:
                raise
            return False
        result = action_result.result().status
        self.settled = result in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_ABORTED,GoalStatus.STATUS_CANCELED)
        row.update(status='succeeded' if result==GoalStatus.STATUS_SUCCEEDED else 'failed', action_status=result)
        if not self.settled:
            raise RuntimeError('导航动作终态未知。')
        return result==GoalStatus.STATUS_SUCCEEDED

    def entrances(self):
        if belongs_to(self.rooms,self.args.room,*self.start_pose[:2]):
            return []
        data = json.loads((ROOT/'scenes/aws_house/spatial_v4.json').read_text())
        result = []
        for edge in data['connections']:
            if self.args.room not in edge['rooms']:
                continue
            inside = edge['rooms'].index(self.args.room)
            x,y,_ = edge['poses'][inside]
            u,v,_ = edge['poses'][1-inside]
            col,row = self.sweep.index(x,y)
            if (0<=row<self.sweep.centers.shape[0] and 0<=col<self.sweep.centers.shape[1]
                    and self.sweep.centers[row,col]):
                result.append(((x,y,math.atan2(y-v,x-u)),(u,v,math.atan2(v-y,u-x))))
        if not result:
            raise ValueError('选定房间的已标定入口在当前地图中不可通行。')
        return sorted(result,key=lambda crossing:math.dist(crossing[0][:2],self.start_pose[:2]))

    def return_to_start(self, exit_pose):
        self.stage, self.return_state = 'returning','returning'
        current = self.pose()
        if exit_pose and current and belongs_to(self.rooms,self.args.room,*current[:2]):
            # Give the final approach a chance even if the doorway was blocked.
            exit_cutoff = min(time.monotonic()+30., self.deadline-30.)
            self.go(exit_pose,exit_cutoff,purpose='room_exit')
        returned = self.go(self.start_pose,self.deadline,timeout=50.,purpose='dock')
        current = self.pose()
        if not returned and current and math.dist(current[:2],self.start_pose[:2])<=.75:
            # Close to the start, align to the approach first, then restore
            # the measured original heading using the same Nav2 controller.
            x,y,_ = self.start_pose
            distance = math.dist(current[:2],(x,y))
            if distance>RETURN_POSITION_M:
                bearing = math.atan2(y-current[1],x-current[0])
                self.go((x,y,bearing),self.deadline,timeout=12.,purpose='dock_approach')
            returned = self.go(self.start_pose,self.deadline,timeout=12.,purpose='dock_alignment')
        self.wait_for(lambda:self.parked() and self.pose(),5)
        self.final_pose = self.pose()
        xy = math.dist(self.start_pose[:2],self.final_pose[:2])
        heading = abs(math.remainder(self.start_pose[2]-self.final_pose[2],2*math.pi))
        self.return_errors = dict(position_m=xy,yaw_rad=heading,stationary_seconds=(
            self.odom_stamp-self.stationary_since)/1e9)
        charging = returned and xy<=RETURN_POSITION_M and heading<=RETURN_YAW_RAD
        self.return_state = 'simulated_charging' if charging else 'return_incomplete'
        return charging

    def run(self):
        self.wait_for(lambda:self.grid is not None and self.pose() and self.parked()
                      and self.costmap_ready() and self.client.server_is_ready(),25)
        warmup = time.monotonic()+1.5
        self.wait_for(lambda:time.monotonic()>=warmup,2)
        self.start_pose = self.pose()
        if not self.start_pose or not self.parked():
            raise ValueError('起点定位不稳定或底盘还在移动。')
        info = self.grid.info
        p = info.origin.position
        self.sweep = RoomSweep(self.grid.data,info.width,info.height,info.resolution,
                              (p.x,p.y,yaw(info.origin.orientation)),self.rooms,self.args.room,self.start_pose)
        cutoff = self.deadline-min(100.,max(30.,self.args.timeout/2))
        entrances = self.entrances()
        self.tick()
        self.wait_for(self.pose,2)
        self.stage = 'entering'
        entered, exit_pose = not entrances,None
        for entrance,exit_target in entrances:
            if self.go(entrance,cutoff,purpose='room_entry'):
                entered,exit_pose = True,exit_target
                break
        if entered:
            route = self.sweep.route(self.pose() or self.start_pose,self.available())
            self.planned_goals = len(route)
            self.tick()
            self.wait_for(self.pose,2)
            self.stage = 'sweeping'
            for first,last in zip(route[::2],route[1::2]):
                if time.monotonic()>=cutoff:
                    break
                segments = self.sweep.lane_segments(first,last,self.available() & self.sweep.centers)
                if not segments:
                    self.skipped_lanes += 1
                for start,end in segments:
                    reached = self.go(start,cutoff)
                    pose = self.pose()
                    if reached or pose is not None and math.dist(pose[:2],start[:2])<=.18:
                        live = self.available()
                        col,row = self.sweep.index(*end[:2])
                        if live[row,col]:
                            self.go(end,cutoff)
                self.finished_goals += 2
            self.stage = 'supplementing'
            attempted = []
            for _ in range(10):
                current = self.pose()
                if (current is None or time.monotonic()>=cutoff or
                        self.sweep.metrics()['coverage_fraction']>=TARGET_COVERAGE):
                    break
                target = self.sweep.repair_goal(current,attempted,self.available())
                if target is None:
                    break
                attempted.append(target[:2])
                self.go(target,cutoff)
        charging = self.return_to_start(exit_pose)
        coverage = self.sweep.metrics()['coverage_fraction']
        if entered and charging and coverage>=TARGET_COVERAGE:
            return 'completed','清扫覆盖达到目标，已返回起点并进入模拟充电。'
        if not entered:
            return 'failed','未能进入选定房间。'
        if charging:
            return 'partial','清扫覆盖低于目标，已返回起点并进入模拟充电。'
        return 'partial','已保存清扫轨迹。覆盖率或返航停靠未达到目标，请查看结果。'

    def report(self, status, message):
        summary = self.progress()
        summary.update(status=status,message=message,elapsed_seconds=round(time.monotonic()-self.started,2),
            coverage_target=TARGET_COVERAGE,sweep_width_m=SWEEP_WIDTH_M,lane_spacing_m=LANE_SPACING_M,
            center_clearance_m=CENTER_CLEARANCE_M,distance_m=round(self.distance,3),
            initial_pose=self.start_pose,final_pose=self.final_pose,return_error=self.return_errors,
            return_tolerance=dict(position_m=RETURN_POSITION_M,yaw_rad=RETURN_YAW_RAD),
            navigation_settled=self.settled)
        write_json(self.args.output/'clean_result.json',summary)
        write_json(self.args.output/'clean_trace.json',dict(frame_id='map',samples=self.trace,goals=self.goals))
        if self.sweep:
            self.sweep.save_map(self.args.output/'clean_map.svg',self.trace,
                                self.rooms[self.args.room]['display_name']+'清扫轨迹')
        print(f'{message} 耗时 {summary["elapsed_seconds"]:.1f} 秒。',flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('room')
    parser.add_argument('rooms',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--timeout',type=float,default=480.)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    stopped = []
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda signum,frame:stopped.append(signum))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    task = CleaningRun(args,stopped)
    try:
        status,message = task.run()
    except StopRequested:
        status,message = 'canceled','清扫已取消。'
        if task.return_state=='returning':
            task.return_state = 'return_canceled'
        if task.settled:
            try:
                task.wait_for(task.parked,2,check_stop=False)
            except TimeoutError:
                task.settled = False
    except Exception as error:
        status,message = 'failed',str(error)
        if task.return_state=='returning':
            task.return_state = 'return_incomplete'
        if not task.settled:
            message += ' 导航终态未确认。'
    finally:
        try:
            task.report(status,message)
        finally:
            task.node.destroy_node()
            rclpy.try_shutdown()
    return 0 if status=='completed' else 1


if __name__=='__main__':
    raise SystemExit(main())
