"""Shared Nav2 action client, fresh pose sampling and terminal/physical-stop barrier."""
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, FollowPath
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry, Path as NavigationPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_console import ROOT, write_json


class StopRequested(Exception):
    pass


def yaw(rotation):
    return math.atan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                      1-2*(rotation.y*rotation.y+rotation.z*rotation.z))


def trace_preview(trace):
    step=max(1,math.ceil(len(trace)/350));preview=[];previous=-1
    for index,row in enumerate(trace):
        if index%step and index!=len(trace)-1 and row['stage']==trace[index-1]['stage']:
            continue
        connected=previous>=0 and all(s['connected'] for s in trace[previous+1:index+1])
        preview.append([round(row['pose'][0],3),round(row['pose'][1],3),row['stage'],connected])
        previous=index
    return preview


class NavigationRun:
    def __init__(self,args,stopped,*,node_name='service_return'):
        self.args,self.stopped=args,stopped
        self.node=Node(node_name,parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.tf=Buffer();self.listener=TransformListener(self.tf,self.node)
        self.client=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.path_client=ActionClient(self.node,FollowPath,'/follow_path')
        # Discover the refresh service during the existing client warm-up,
        # rather than creating its DDS endpoints at the final parked pose.
        self.localization_client=self.node.create_client(Empty,'/request_nomotion_update')
        self.node.create_subscription(Odometry,'/odom',self.receive_odom,qos_profile_sensor_data)
        self.started=time.monotonic();self.deadline=self.started+args.timeout
        self.start_pose=self.final_pose=None
        self.odom_wall=self.odom_stamp=self.stationary_since=None
        self.odom_distance=0.;self.previous_odom_pose=None
        self.odom_trace=[]
        self.trace,self.goals=[],[];self.distance=0.
        self.stage='planning';self.settled=True
        self.return_state='not_attempted';self.return_errors=None
        self.docking_localization_updates=[];self.latest_localization=None
        self.node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',
                                      self.receive_localization,qos_profile_sensor_data)
        self.next_report=self.next_sample=0.;self.last_pose_wall=self.started;self.sample_gap=False
        self.progress_filename='return_progress.json'
        self.last_plan=None
        self.node.create_subscription(NavigationPath,'/plan',self.receive_plan,1)

    def receive_localization(self,message):
        p=message.pose.pose.position;q=message.pose.pose.orientation
        values=[p.x,p.y,yaw(q),*[message.pose.covariance[i] for i in (0,7,35)]]
        if message.header.frame_id!='map' or not all(math.isfinite(v) for v in values):return
        self.latest_localization=dict(stamp_ns=Time.from_msg(message.header.stamp).nanoseconds,
            received_wall=time.monotonic(),pose=values[:3],variance=values[3:])

    def refresh_docking_localization(self,deadline):
        """Request one normal laser update, requiring new measured AMCL data."""
        event=dict(before=self.pose(),status='not_requested')
        self.docking_localization_updates.append(event)
        if not self.parked() or event['before'] is None:return False
        remaining=lambda:max(0.,min(3.,deadline-time.monotonic()))
        client=self.localization_client
        try:
            event['phase']='service_discovery'
            if not client.service_is_ready():
                self.wait_for(client.service_is_ready,remaining())
            event.update(requested_stamp_ns=self.node.get_clock().now().nanoseconds,
                         requested_wall=time.monotonic(),status='requested')
            future=client.call_async(Empty.Request())
            event['phase']='service_reply'
            self.wait_for(future.done,remaining());future.result()
            event['phase']='new_measurement'
            # The service acknowledgement alone does not prove a new scan
            # was consumed, or a new localized pose was published.
            self.wait_for(lambda:self.latest_localization is not None
                and self.latest_localization['stamp_ns']>=event['requested_stamp_ns']
                and self.latest_localization['received_wall']>=event['requested_wall']
                and self.pose() is not None and self.parked(),remaining())
            # AMCL postdates map->odom. A fresh message can arrive before
            # its correction is visible in the latest common base TF.
            measurement=dict(self.latest_localization)
            def coherent_pose():
                pose=self.pose()
                return (pose is not None and self.parked()
                    and math.dist(pose[:2],measurement['pose'][:2])<=.01
                    and abs(math.remainder(pose[2]-measurement['pose'][2],2*math.pi))<=.01)
            event['phase']='tf_coherence'
            self.wait_for(coherent_pose,remaining())
            event.update(status='updated',after=self.pose(),measurement=measurement)
            return True
        except (TimeoutError,RuntimeError) as error:
            event.update(status='unavailable',error=str(error));return False

    def receive_plan(self, message):
        if not message.poses:
            return
        pose=message.poses[-1].pose
        self.last_plan=dict(frame_id=message.header.frame_id,
            stamp_seconds=Time.from_msg(message.header.stamp).nanoseconds/1e9,
            endpoint=[pose.position.x,pose.position.y,yaw(pose.orientation)],
            received_wall=time.monotonic())


    def on_pose_sample(self,current,stamp):
        pass

    def progress(self):
        return dict(stage=self.stage,frame_id='map',trace_preview=trace_preview(self.trace),
                    return_state=self.return_state)

    def lane_position(self,target,purpose=None,recovery=False):
        return False

    def path_for_lane(self,target):
        raise ValueError('当前任务没有清扫参考路径。')

    def receive_odom(self, message):
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        if self.odom_stamp is not None and stamp<=self.odom_stamp:
            return
        self.odom_stamp, self.odom_wall = stamp,time.monotonic()
        p = message.pose.pose.position
        previous = getattr(self,'previous_odom_pose',None)
        increment=0.
        if previous is not None and 0<stamp-previous[0]<=600_000_000:
            distance = math.dist((p.x,p.y),previous[1:])
            if math.isfinite(distance) and distance<=.30:
                self.odom_distance = getattr(self,'odom_distance',0.)+distance
                increment=distance
        self.previous_odom_pose = (stamp,p.x,p.y)
        velocity = message.twist.twist
        if not hasattr(self,'odom_trace'):self.odom_trace=[]
        self.odom_trace.append(dict(stamp_ns=stamp,
            pose_xy=[v if math.isfinite(v) else None for v in (p.x,p.y)],
            distance_m=increment,
            velocity=[v if math.isfinite(v) else None for v in (velocity.linear.x,velocity.angular.z)]))
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
                self.on_pose_sample(current,stamp)
                if previous is None or stamp>previous['sim_seconds']:
                    self.trace.append(dict(pose=current,sim_seconds=stamp,stage=self.stage,connected=connected))
        if now>=self.next_report:
            self.next_report = now+1.
            write_json(self.args.output/self.progress_filename,self.progress())

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

    def go(self, target, deadline, *, timeout=40., purpose=None, recovery=False,
           lane=False, skip_if_close=False):
        if self.stopped:
            raise StopRequested()
        if not self.settled:
            raise RuntimeError('上次导航终态或停稳未确认，拒绝发送新目标。')
        if not self.pose():
            raise ValueError('定位数据不可用，已停止清扫任务。')
        self.last_pose_wall = time.monotonic()
        if time.monotonic()>=deadline:
            return False
        row = dict(stage=self.stage,target=list(target),status='sending',stop_confirmed=False,
                   timeout_seconds=timeout,
                   started_seconds=round(time.monotonic()-self.started,3))
        row['action_type']='follow_path' if lane else 'navigate_to_pose'
        if purpose:
            row['purpose'] = purpose
        self.goals.append(row)
        path=self.path_for_lane(target) if lane else None
        if lane:
            row['reference_path']=path
            if path is None:
                row.update(status='path_blocked',measured_pose=self.pose(),
                           stop_confirmed=self.parked(),
                           finished_seconds=round(time.monotonic()-self.started,3))
                return False
        current=self.pose()
        if (skip_if_close and self.stage in ('sweeping','supplementing') and not recovery
                and current is not None and self.parked() and math.dist(current[:2],target[:2])<=.12):
            row.update(status='already_reached',measured_pose=current,stop_confirmed=True,
                       finished_seconds=round(time.monotonic()-self.started,3))
            return True
        if lane:
            goal=FollowPath.Goal()
            goal.controller_id='CleaningPath'
            goal.goal_checker_id='cleaning_goal_checker'
            goal.path.header.frame_id='map'
            goal.path.header.stamp=self.node.get_clock().now().to_msg()
            for x,y,heading in path:
                pose=PoseStamped();pose.header=goal.path.header
                pose.pose.position.x,pose.pose.position.y=float(x),float(y)
                pose.pose.orientation.z=math.sin(heading/2)
                pose.pose.orientation.w=math.cos(heading/2)
                goal.path.poses.append(pose)
            client=self.path_client
        else:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y = map(float,target[:2])
            goal.pose.pose.orientation.z = math.sin(target[2]/2)
            goal.pose.pose.orientation.w = math.cos(target[2]/2)
            if self.stage in ('sweeping','supplementing') and purpose=='lane_start':
                # This waypoint only joins the next stripe. Its configured native
                # position checker avoids a redundant final heading; the next
                # FollowPath still turns with the collision-checked shim.
                goal.behavior_tree = str(ROOT/'src/semantic_search/config/navigate_cleaning_connector.xml')
                row['navigation_role']='cleaning_connector_position_only'
            elif self.stage in ('sweeping','supplementing') and not recovery:
                goal.behavior_tree = str(ROOT/'src/semantic_search/config/navigate_cleaning_lane.xml')
            elif (purpose in ('dock','dock_alignment','dock_approach')
                    and (getattr(self,'station',None)or{}).get('precision_docking',False)):
                goal.behavior_tree = str(ROOT/'src/semantic_search/config/navigate_docking.xml')
                row['navigation_role']='precision_docking_full_pose'
            client=self.client
        # Waiting for a fresh costmap and constructing a dense reference can
        # consume time. Recheck immediately before any action is sent.
        if self.stopped:
            row.update(status='canceled_before_send',stop_confirmed=self.parked(),
                       finished_seconds=round(time.monotonic()-self.started,3))
            raise StopRequested()
        if time.monotonic()>=deadline:
            row.update(status='deadline_before_send',stop_confirmed=self.parked(),
                       finished_seconds=round(time.monotonic()-self.started,3))
            return False
        if self.pose() is None:
            row.update(status='pose_unavailable_before_send',stop_confirmed=self.parked(),
                       finished_seconds=round(time.monotonic()-self.started,3))
            raise ValueError('路径准备期间定位数据失效，未发送导航目标。')
        self.settled = False
        sent = client.send_goal_async(goal)
        handle = action_result = None
        interrupted = None
        position_cancel = False
        def completion():
            # Task coverage may complete before the base reaches a stripe end.
            # The hook is used only for validated cleaning FollowPath actions.
            hook=getattr(self,'cleaning_completion',None)
            evidence=hook(target,purpose,recovery) if lane and callable(hook) else None
            if evidence is not None:return 'brush_completion',evidence
            if self.lane_position(target,purpose,recovery):
                return 'position_completion',dict(tolerance_m=.12,measured_pose=list(self.pose()))
            return None
        try:
            self.wait_for(sent.done,8)
            handle = sent.result()
            if not handle.accepted:
                self.settled = True
                row['status'] = 'rejected'
                return False
            action_result = handle.get_result_async()
            row['effective_timeout_seconds'] = max(0.,min(timeout,deadline-time.monotonic()))
            self.wait_for(lambda:action_result.done() or completion() is not None,
                          row['effective_timeout_seconds'],require_pose=True)
            if not action_result.done():
                # Stop only after the measured task completion criterion holds.
                # Cancel through Nav2 and confirm its terminal result before
                # another goal; entry, recovery and docking retain full poses.
                position_cancel = True
                handle.cancel_goal_async()
                self.wait_for(action_result.done,3,require_pose=True)
        except (StopRequested, TimeoutError) as error:
            interrupted = error
            row['status'] = 'canceled' if isinstance(error,StopRequested) else 'timed_out'
            if handle is None:
                # The command may have reached Nav2 even if its response is late.
                self.wait_for(sent.done,2,check_stop=False)
                handle = sent.result()
            if handle.accepted:
                action_result = action_result or handle.get_result_async()
                if not action_result.done():
                    handle.cancel_goal_async()
                    self.wait_for(action_result.done,3,check_stop=False)
            if not handle.accepted:
                self.settled = True
                if isinstance(error,StopRequested):
                    raise
                return False
        terminal = action_result.result() if action_result is not None and action_result.done() else None
        result = terminal.status if terminal is not None else None
        plan=getattr(self,'last_plan',None)
        if lane:
            row['plan_endpoint']=dict(frame_id='map',endpoint=list(path[-1]),
                source='validated_lane_reference',stamp_seconds=Time.from_msg(goal.path.header.stamp).nanoseconds/1e9)
        elif plan and plan['received_wall']>=self.started+row['started_seconds']:
            row['plan_endpoint']={k:v for k,v in plan.items() if k!='received_wall'}
            row['plan_endpoint']['age_wall_seconds']=time.monotonic()-plan['received_wall']
        row.update(action_status=result,terminal_pose=self.pose(),measured_pose=self.pose(),
                   terminal_seconds=round(time.monotonic()-self.started,3))
        if result not in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_ABORTED,GoalStatus.STATUS_CANCELED):
            row['status'] = 'unconfirmed'
            raise RuntimeError('导航动作终态未知。')
        if interrupted is None:
            row['status'] = 'succeeded' if result==GoalStatus.STATUS_SUCCEEDED else 'failed'
        # A parent action result can precede FollowPath stopping. Require a
        # new stationary interval after every accepted terminal result, even
        # timeout/abort. Keep this outside the cancellation exception handler:
        # a failed stop barrier must never be converted into a settled action.
        self.stationary_since = None
        try:
            self.wait_for(self.parked,2,check_stop=False,require_pose=True)
        except TimeoutError as error:
            row['stop_error'] = str(error)
            raise
        self.settled = True
        row.update(stop_confirmed=True,finished_seconds=round(time.monotonic()-self.started,3),
                   measured_pose=self.pose())
        if isinstance(interrupted,StopRequested) or self.stopped:
            raise StopRequested()
        if interrupted is not None:
            return False
        evidence=completion() if position_cancel else None
        if evidence is not None:row[evidence[0]]=evidence[1]
        brush_completed=result==GoalStatus.STATUS_CANCELED and 'brush_completion'in row
        position_completed = result==GoalStatus.STATUS_CANCELED and 'position_completion' in row
        row.update(status='succeeded' if result==GoalStatus.STATUS_SUCCEEDED else (
            'brush_covered' if brush_completed else 'position_reached' if position_completed else 'failed'),action_status=result,
            finished_seconds=round(time.monotonic()-self.started,3),measured_pose=self.pose())
        if (result==GoalStatus.STATUS_SUCCEEDED and not lane and purpose=='lane_start'
                and self.stage in ('sweeping','supplementing')):
            # NavFn may return a nearby reachable endpoint within its .5m
            # fallback. Native success at that endpoint is not arrival at the
            # requested stripe. Keep native status and the physical stop proof.
            current=self.pose()
            distance=math.inf if current is None else math.dist(current[:2],target[:2])
            row['requested_position_check']=dict(error_m=distance,tolerance_m=.12,passed=distance<=.12)
            if distance>.12:
                row['status']='adjusted_goal_unreached'
                return False
        return result==GoalStatus.STATUS_SUCCEEDED or position_completed or brush_completed
