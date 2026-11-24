#!/usr/bin/env python3
"""Run one room cleaning request through the console's existing Nav2 stack."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from scipy.ndimage import binary_dilation
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_console import ROOT, write_json
from semantic_search.cleaning import observed_return_speed, recent_translation_stopped
from semantic_search.cleaning import (CENTER_CLEARANCE_M, LANE_SPACING_M, RETURN_POSITION_M,
                                      RETURN_YAW_RAD, SWEEP_WIDTH_M, TARGET_COVERAGE, RoomSweep,
                                      TRACKING_MARGIN_CELLS, transit_timeout, coverage_accepted)
from semantic_search.rooms import belongs_to
from navigation_client import NavigationRun, StopRequested, yaw
from semantic_search.docking import dock_at, load_station


class CleaningRun(NavigationRun):
    def __init__(self,args,stopped):
        super().__init__(args,stopped,node_name='room_cleaning')
        self.progress_filename='clean_progress.json'
        grid_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(OccupancyGrid,'/map',self.receive_map,grid_qos)
        self.node.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.receive_costmap,grid_qos)
        self.rooms=json.loads(args.rooms.read_text())
        self.sweep=self.grid=self.costmap=None;self.costmap_wall=None
        self.planned_goals=self.finished_goals=0;self.skipped_lanes=0
        self.recoveries=[];self.replans=[]
        self.cleaning_stop_reason='not_started'
        self.navigation_events=[];self.navigation_arrays={};self.navigation_keys={}
        self.navigation_events_dropped=0;self.original_route=[]
        self.blocked_original_lanes=set()
        self.segment_attempts=[];self.coverage_observations=[]
        self.remaining_stop_reason='route_and_repairs_exhausted'
        self.cleaning_phases=[]
        station_file=getattr(args,'station_config',None)
        self.station=load_station(station_file,ROOT) if station_file else None
        self.dock_pose=tuple(self.station['map_pose']) if self.station else None
        self.return_label='充电处' if self.station else '本次起点'

    def on_pose_sample(self,current,stamp):
        if self.sweep is not None and self.stage in ('sweeping','supplementing'):
            self.sweep.touch(*current[:2],stamp)


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

    def path_for_lane(self,target):
        live=self.available()
        first=getattr(self,'lane_start',None)
        if first is None:
            path=self.sweep.lane_path(self.pose(),target,live)
        else:
            reference=self.sweep.lane_reference(self.pose(),first,target,live)
            path=(reference['path'] if reference is not None and
                  math.dist(reference['end'][:2],target[:2])<1e-6 else None)
        self.record_navigation('reference_validation',live,target=list(target),accepted=path is not None)
        return path

    def follow_lane(self,first,last,deadline,**options):
        """Revalidate after transit; the submitted target is the fresh segment end."""
        live=self.available()
        reference=self.sweep.lane_reference(self.pose(),first,last,live)
        target=reference['end'] if reference is not None else last
        self.record_navigation('lane_regeneration',live,requested_start=first,requested_end=last,
            selected_reference=None if reference is None else {k:v for k,v in reference.items() if k!='path'})
        self.lane_start=first
        goal_index=len(self.goals)
        try:
            return self.go(target,deadline,lane=True,skip_if_close=True,**options)
        finally:
            self.lane_start=None
            if len(self.goals)>goal_index:
                self.goals[goal_index]['nominal_lane']=dict(start=list(first),end=list(last))

    def record_navigation(self,kind,live,**details):
        """Keep decision masks in one bounded archive, never change coverage."""
        if len(self.navigation_events)>=256:
            self.navigation_events_dropped+=1
            return None
        if not self.navigation_arrays:
            self.navigation_arrays.update(static_map=self.sweep.occupancy.copy(),
                eligible=self.sweep.eligible.copy(),static_centers=self.sweep.transit_centers.copy(),
                planning_centers=self.sweep.planning_transit_centers.copy())
        def remember(prefix,array):
            array=np.asarray(array)
            digest=hashlib.sha256(prefix.encode()+array.tobytes()).hexdigest()
            if digest not in self.navigation_keys:
                key=f'{prefix}_{len(self.navigation_keys):03d}'
                self.navigation_keys[digest]=key;self.navigation_arrays[key]=array.copy()
            return self.navigation_keys[digest]
        current=self.pose()
        connected=self.sweep.connected_centers(current,live) if current is not None else np.zeros_like(live)
        possible=binary_dilation(connected,structure=self.sweep.disk) & self.sweep.eligible
        planned_possible=binary_dilation(connected & self.sweep.planning_centers,
                                        structure=self.sweep.disk) & self.sweep.eligible
        cost=np.asarray(self.costmap.data,dtype=np.int8).reshape(live.shape)
        blocked=[]
        for index,(first,last) in enumerate(zip(self.original_route[::2],self.original_route[1::2])):
            if not self.sweep.lane_segments(first,last,live & self.sweep.centers):
                blocked.append(index)
        self.blocked_original_lanes.update(blocked)
        self.skipped_lanes=len(self.blocked_original_lanes)
        event=dict(kind=kind,wall_seconds=time.monotonic()-self.started,
            sim_seconds=self.node.get_clock().now().nanoseconds/1e9,pose=current,
            costmap_stamp_seconds=Time.from_msg(self.costmap.header.stamp).nanoseconds/1e9,
            costmap_age_wall_seconds=time.monotonic()-self.costmap_wall,
            costmap_key=remember('cost',cost),covered_key=remember('covered',self.sweep.covered),
            connected_center_cells=int(connected.sum()),blocked_original_lane_ids=blocked,
            remaining_cells=self.sweep.eligible_cells-int(self.sweep.covered.sum()),
            current_mask_brush_potential_fraction=int((possible | self.sweep.covered).sum())/self.sweep.eligible_cells,
            planning_mask_brush_potential_fraction=int((planned_possible | self.sweep.covered).sum())/self.sweep.eligible_cells,
            **details)
        self.navigation_events.append(event)
        return event

    def observe_remaining(self,deadline):
        """Refresh current observations without clearing or relaxing obstacles."""
        if len(self.coverage_observations)>=2 or deadline-time.monotonic()<20.:
            return False
        current=self.remember_safe_arrival(None)
        if current is None:
            return False
        heading=current[2]+math.pi/2
        before=Time.from_msg(self.costmap.header.stamp).nanoseconds
        item=dict(from_pose=list(current),target=[*current[:2],heading])
        self.coverage_observations.append(item)
        reached=self.go(item['target'],deadline,timeout=12.,purpose='coverage_observation',recovery=True)
        if reached:
            self.wait_for(lambda:self.costmap_ready() and
                Time.from_msg(self.costmap.header.stamp).nanoseconds>before,3.,require_pose=True)
        item['status']='succeeded' if reached else 'failed'
        self.record_navigation('coverage_observation',self.available(),observation=item)
        return reached and self.remember_safe_arrival(None) is not None

    def next_lanes(self,current,live,pending,attempted,remaining_seconds=math.inf):
        """Keep the accepted baseline; budget reordering is an explicit experiment."""
        connected=self.sweep.connected_centers(current,live) & self.sweep.planning_centers
        if not connected.any():
            # An unsafe current cell says nothing about the queued stripes.
            # In particular, do not destructively pop/filter them below.
            self.remaining_stop_reason='selection_pose_not_navigable'
            return []
        if not getattr(getattr(self,'args',None),'budget_selection',False):
            while pending:
                lane_id,first,last=pending.pop(0)
                pieces=self.sweep.lane_segments(first,last,connected)
                if not pieces:
                    continue
                start,end=pieces[0]
                pending[:0]=[(lane_id,a,b) for a,b in pieces[1:]]
                gain=self.sweep.segment_gain(start,end)
                if gain*self.sweep.resolution**2<.04:
                    continue
                return [dict(start=start,end=end,predicted_new_cells=gain,
                             source='seeded_route',original_lane_id=lane_id)]
            required=self.sweep.required_lane_repairs(current,
                getattr(self,'original_route',[]),live,attempted)
            offset=getattr(self.sweep,'offset_lane_repairs',None)
            if not required and offset is not None:
                required=offset(current,getattr(self,'original_route',[]),live,attempted)
            if (required and self.sweep.metrics()['coverage_fraction']>=TARGET_COVERAGE
                    and not getattr(self,'local_remaining_pass',False)):
                return required
            choices=self.sweep.remaining_lanes(current,live,attempted,
                preferred_orientation=self.sweep.cross_orientation(getattr(self,'original_route',[])))
            for choice in choices:
                choice['source']='remaining_floor'
            if required:
                # Finish the seeded pass first. Below the area target, a short
                # reachable infill can be more useful than crossing the room
                # for an original stripe's tiny edge gap. Keep every original
                # stripe as the acceptance reference and reconsider it next
                # time; predicted gain never changes the measured brush mask.
                repair=required[0]
                repair_rate=repair['predicted_new_cells']/repair['estimated_seconds']
                if (choices and choices[0]['predicted_new_cells']/choices[0]['estimated_seconds']
                        >=1.25*repair_rate and
                        choices[0]['estimated_seconds']+2.<=remaining_seconds):
                    choices[0].update(selection_reason='reachable_infill_before_small_stripe_gap',
                        deferred_required_lane_id=repair.get('original_lane_id'),
                        deferred_required_predicted_cells=repair['predicted_new_cells'],
                        deferred_required_estimated_seconds=repair['estimated_seconds'])
                    return choices
                return required
            if not choices:
                self.remaining_stop_reason='remaining_lanes_exhausted'
            return choices
        distance=self.sweep.travel_distances(current,live)
        pieces=[(lane_id,a,b) for lane_id,first,last in pending
                for a,b in self.sweep.lane_segments(first,last,connected)
                if self.sweep.segment_gain(a,b)*self.sweep.resolution**2>=.04]
        pending[:]=pieces
        seeds=[];head=None
        for index,(lane_id,first,last) in enumerate(pieces):
            for reverse,(a,b) in enumerate(((first,last),(last,first))):
                choice=self.sweep.lane_value(current,a,b,distance)
                if choice is None:
                    continue
                choice.update(source='seeded_route',original_lane_id=lane_id,pending_index=index)
                if index==0 and reverse==0:
                    head=choice
                seeds.append(choice)
        repairs=self.sweep.remaining_lanes(current,live,attempted,distances=distance,budget=True)
        for choice in repairs:
            choice['source']='remaining_floor'
        affordable=[c for c in seeds+repairs if c['estimated_seconds']+2.<=remaining_seconds]
        if not affordable:
            self.remaining_stop_reason='insufficient_segment_budget' if seeds or repairs else 'remaining_lanes_exhausted'
            return []
        best=max(affordable,key=lambda c:c['score'])
        keep=(head is not None and head in affordable and best['score']<1.25*head['score'])
        chosen=dict(head if keep else best)
        chosen.update(selection_reason='keep_sweep_order' if keep else 'higher_gain_per_second',
                      available_seconds=None if math.isinf(remaining_seconds) else remaining_seconds,
                      queued_score=head['score'] if head else None)
        index=chosen.pop('pending_index',None)
        if index is not None:
            pending.pop(index)
            if not keep:
                chosen['source']='budget_seed'
        return [chosen]

    def sweep_remaining(self,deadline,*,seeded=True,seed_only=False):
        """Complete the sweep pass, regenerating reachable remaining floor afterwards."""
        safe_pose=self.remember_safe_arrival(None)
        if safe_pose is None:
            return False
        attempted=[];selection_pose_recoveries=0
        pending=([(i,a,b) for i,(a,b) in enumerate(zip(self.original_route[::2],self.original_route[1::2]))]
                 if seeded else [])
        for index in range(64):
            if time.monotonic()>=deadline or coverage_accepted(self.sweep,self.original_route):
                return True
            if seed_only and not pending:
                return True
            live=self.available();current=self.pose()
            if current is None:
                return False
            started=time.monotonic()
            choices=self.next_lanes(current,live,pending,attempted,deadline-time.monotonic())
            event=self.record_navigation('remaining_selection',live,candidates=choices,
                planning_wall_seconds=time.monotonic()-started)
            self.tick()
            if not choices:
                if getattr(self,'remaining_stop_reason',None)=='selection_pose_not_navigable':
                    if selection_pose_recoveries>=2 or not self.settled:
                        return False
                    selection_pose_recoveries+=1
                    self.record_navigation('selection_pose_recovery',live,
                        previous_anchor=safe_pose,attempt=selection_pose_recoveries,
                        limit=2,reason='current_pose_outside_safe_connected_centers')
                    if not self.recover_lane(safe_pose,deadline):
                        return False
                    safe_pose=self.remember_safe_arrival(safe_pose)
                    continue
                if (event is not None and event['planning_mask_brush_potential_fraction']<TARGET_COVERAGE
                        and self.observe_remaining(deadline)):
                    continue
                return True
            selected=choices[0];start,end=selected['start'],selected['end']
            reference_start,reference_end=start,end
            execution=dict(start=start,end=end,point_only=False,start_inset_m=0.,end_inset_m=0.)
            if getattr(self.sweep,'turning_radius_m',None)is not None:
                execution=self.sweep.execution_segment(start,end,live,
                    np.asarray(self.costmap.data,dtype=np.int8).reshape(live.shape),current=current)
            start,end=execution['start'],execution['end']
            attempted.append((start,end))
            before=int(self.sweep.covered.sum());odom_before=self.odom_distance
            item=dict(index=index,start=list(start),end=list(end),predicted_new_cells=selected['predicted_new_cells'],
                source=selected['source'],
                original_lane_id=selected.get('original_lane_id'),
                required_centerline_cells=selected.get('required_centerline_cells'),
                execution_segment=execution,
                orientation=selected.get('orientation'),
                preferred_orientation=selected.get('preferred_orientation'),
                goal_start_index=len(self.goals),odom_start_index=len(self.odom_trace))
            self.segment_attempts.append(item)
            self.planned_goals=max(self.planned_goals,2*len(self.segment_attempts))
            budget=self.sweep.connector_budget(current,start,live)
            item['connector_budget']=budget
            alignment=(self.sweep.connector_alignment_goal(current,start,end,live)
                if getattr(self.sweep,'align_connectors',False) else None)
            aligned=True
            if alignment is not None:
                item['connector_alignment']=alignment
                aligned=self.go(alignment['target'],deadline,timeout=20.,purpose='lane_start',recovery=True)
                alignment['status']='succeeded' if aligned else 'failed'
                if aligned:
                    safe_pose=self.remember_safe_arrival(safe_pose)
            main_goal_index=len(self.goals)
            reached=aligned and self.go(start,deadline,timeout=budget['timeout_seconds'],purpose='lane_start',recovery=True)
            if len(self.goals)>main_goal_index:
                self.goals[main_goal_index]['connector_budget']=budget
            ready=False
            if aligned and (reached or self.continue_from_partial_lane(start)):
                safe_pose=self.remember_safe_arrival(safe_pose)
                lane_goal_index=len(self.goals)
                finished=(execution['point_only']and reached and self.parked()and
                    self.sweep.spatial_coverage([reference_start,reference_end])['completed'])
                if not execution['point_only']:
                    finished=self.follow_lane(start,end,deadline)
                ready=finished or (len(self.goals)>lane_goal_index and
                    self.continue_from_partial_lane(self.goals[-1]['target']))
            item.update(status='continued' if ready else 'failed',goal_end_index=len(self.goals),
                odom_end_index=len(self.odom_trace),measured_odom_travel_m=self.odom_distance-odom_before,
                measured_new_cells=int(self.sweep.covered.sum())-before)
            self.finished_goals=2*len(self.segment_attempts)
            self.record_navigation('remaining_attempt',self.available(),attempt=item)
            if ready:
                safe_pose=self.remember_safe_arrival(safe_pose)
                actual_end=self.goals[-1]['target']
                if (selected['source'] in ('seeded_route','budget_seed') and
                        math.dist(actual_end[:2],reference_end[:2])>.14 and
                        (not(execution['start_inset_m']or execution['end_inset_m'])or
                         not self.sweep.spatial_coverage([reference_start,reference_end])['completed'])):
                    pending.insert(0,(selected['original_lane_id'],tuple(actual_end),reference_end))
                continue
            # A settled failure at a safe measured pose does not require a
            # different sweep orientation. Finish the seeded pass first;
            # regenerating a cross-room repair here can send the base to the
            # opposite wall and straight back to the next queued stripe.
            if (self.settled and self.goals[-1].get('stop_confirmed') and
                    (pending or self.goals[-1]['status']=='path_blocked')):
                current_safe=self.remember_safe_arrival(None)
                if current_safe is not None:
                    safe_pose=current_safe
                    item['continuation_policy']='next_seeded_lane' if pending else 'reselect_after_blocked_path'
                    continue
            live=self.available();current=self.pose()
            alternatives=self.sweep.remaining_lanes(current,live,attempted) if current is not None else []
            alternate_pending=[(j,c['start'],c['end']) for j,c in enumerate(alternatives)]
            previous_events=len(self.replans)
            resumed=self.replan_lane(alternate_pending,deadline,set())
            for recovery in self.replans[previous_events:]:
                attempted.extend((a['target'],a['end']) for a in recovery['attempts'])
            if resumed is not None:
                safe_pose=resumed
            elif not self.recover_lane(safe_pose,deadline):
                return False
            else:
                safe_pose=self.remember_safe_arrival(safe_pose)
        self.remaining_stop_reason='adaptive_attempt_limit'
        return True

    def save_navigation(self):
        if self.navigation_arrays:
            np.savez_compressed(self.args.output/'clean_navigation.npz',**self.navigation_arrays)
        write_json(self.args.output/'clean_plan.json',dict(frame_id='map',
            resolution_m=self.sweep.resolution if self.sweep else None,
            origin=self.sweep.origin if self.sweep else None,
            tracking_margin_cells=getattr(self.sweep,'tracking_margin_cells',TRACKING_MARGIN_CELLS),
            transit_clearance_m=getattr(self.sweep,'transit_clearance_m',CENTER_CLEARANCE_M),
            arrays_file='clean_navigation.npz' if self.navigation_arrays else None,
            original_route=self.original_route,events=self.navigation_events,
            events_dropped=self.navigation_events_dropped,
            note='Selection snapshots and hypothetical gains never count as physical cleaning.'))





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
                    spatial_coverage=self.sweep.spatial_coverage(self.original_route) if self.sweep else None,
                    return_state=self.return_state)



    def cleaning_completion(self,target,purpose=None,recovery=False):
        """Actual brush completion, never a replacement Nav2 goal tolerance."""
        first=getattr(self,'lane_start',None)
        if self.stage!='sweeping' or first is None or recovery or purpose in ('room_entry','dock','dock_alignment','lane_recovery'):
            return None
        if not (self.parked() or recent_translation_stopped(getattr(self,'odom_trace',()))) or not self.costmap_ready():
            return None
        now=self.node.get_clock().now().nanoseconds
        if (self.odom_wall is None or time.monotonic()-self.odom_wall>.4 or
                self.odom_stamp is None or not -100_000_000<=now-self.odom_stamp<=400_000_000):
            return None
        try:
            tf=self.tf.lookup_transform('map','base_link',Time())
            if not -100_000_000<=now-Time.from_msg(tf.header.stamp).nanoseconds<=400_000_000:
                return None
        except TransformException:
            return None
        pose=self.pose()
        if pose is None or not all(math.isfinite(v)for v in pose) or math.dist(pose[:2],target[:2])>self.sweep.radius:
            return None
        col,row=self.sweep.index(*pose[:2]);live=self.available()
        if not (0<=row<live.shape[0] and 0<=col<live.shape[1] and
                self.sweep.planning_centers[row,col] and live[row,col]):
            return None
        coverage=self.sweep.spatial_coverage([first,target])
        if not coverage['completed']:
            return None
        return dict(basis='measured_brush_stripe_centerline',radius_m=self.sweep.radius,
            start=list(first),end=list(target),measured_pose=list(pose),
            spatial_coverage=coverage)

    def lane_position(self, target, purpose=None, recovery=False):
        """Fresh physical position can finish a lane, never entry or docking."""
        if (not getattr(self.args,'position_goals',False) or
                self.stage not in ('sweeping','supplementing') or purpose is not None or recovery):
            return False
        pose = self.pose()
        return pose is not None and math.dist(pose[:2],target[:2])<=.12

    def recover_lane(self, safe_pose, deadline):
        """Use Nav2's normal recovery tree before issuing more cleaning goals.

        A fast lane tree intentionally has no recovery. After a collision-cost
        trap it must not keep aborting every later lane from that same pose.
        Recovery stays bounded and never directly drives or clears sensors.
        """
        # One bounded attempt follows each failed lane. Successful recovery
        # must not exhaust a global two-attempt quota and end a large room.
        # The finite lane list and cleaning deadline bound total work.
        if deadline-time.monotonic()<20.:
            return False
        current = self.pose()
        if current is None or safe_pose is None:
            return False
        source_was_navigable=self.remember_safe_arrival(None) is not None
        odom_source=getattr(self,'previous_odom_pose',None)
        legacy_distance = math.dist(current[:2],safe_pose[:2])
        record = dict(from_pose=list(current),target=list(safe_pose),status='running',
                      source='previous_arrival',source_was_navigable=source_was_navigable,previous_arrival=list(safe_pose),
                      previous_arrival_distance_m=legacy_distance)
        self.recoveries.append(record)
        # Preserve short/local restarts. A distant successful endpoint need
        # not force a complete traversal back along an already swept lane.
        try:
            sweep=getattr(self,'sweep',None)
            validate_target=getattr(sweep,'recovery_target_valid',None)
            if validate_target is not None and not validate_target(safe_pose,self.available()):
                # Native XY success may stop short of a formerly safe point.
                # Select a measured nearby target whose whole .12m arrival
                # disk is still navigable, without clearing any obstacles.
                candidate=sweep.recovery_anchor(current,self.trace,self.available(),
                    self.node.get_clock().now().nanoseconds/1e9,arrival_tolerance_m=.12)
                record['arrival_tolerance_m']=.12
                if candidate is None:
                    record.update(status='unavailable',reason='no_buffered_recovery_anchor')
                    return False
                safe_pose=candidate['pose']
                record.update(source='recent_trace_with_arrival_margin',trace_anchor=candidate,
                    target=list(safe_pose))
            elif getattr(self.args,'local_recovery',False) and legacy_distance > 1.:
                available = self.available()
                current = self.pose()
                if current is None:
                    record.update(status='unavailable',reason='no_fresh_pose')
                    return False
                record['from_pose'] = list(current)
                candidate = self.sweep.recovery_anchor(current,self.trace,available,
                    self.node.get_clock().now().nanoseconds/1e9,
                    **({'arrival_tolerance_m':.12} if validate_target is not None else {}))
                if candidate is not None:
                    safe_pose = candidate['pose']
                    record.update(source='recent_trace',trace_anchor=candidate,target=list(safe_pose))
                else:
                    record['fallback_reason'] = 'no_valid_recent_trace_anchor'
                    col,row = self.sweep.index(*safe_pose[:2])
                    height,width = available.shape
                    if not (0<=col<width and 0<=row<height and
                            available[row,col] and self.sweep.centers[row,col]
                            and (validate_target is None or validate_target(safe_pose,available))):
                        record.update(status='unavailable',reason='previous_arrival_blocked')
                        return False
        except Exception as error:
            record.update(status='failed',error=str(error))
            raise
        # A nearby anchor still requires turning, approach and full heading
        # alignment. Retain the old endpoint's time allowance rather than
        # shrinking it with Euclidean displacement; the task deadline bounds it.
        timeout = min(45.,20.+max(legacy_distance,math.dist(current[:2],safe_pose[:2]))/.16)
        record['timeout_seconds'] = timeout
        record['selected_distance_m'] = math.dist(current[:2],safe_pose[:2])
        record['trace_start_index'] = len(self.trace)
        record['odom_start_index'] = len(self.odom_trace)
        distance_before = self.distance
        odom_before = self.odom_distance
        started = time.monotonic()
        try:
            recovered = self.go(safe_pose,deadline,timeout=timeout,purpose='lane_recovery',recovery=True)
        except Exception as error:
            record.update(status='failed',error=str(error))
            raise
        finally:
            record['elapsed_wall_seconds'] = time.monotonic()-started
            record['measured_travel_m'] = self.distance-distance_before
            record['measured_odom_travel_m'] = self.odom_distance-odom_before
            record['trace_end_index'] = len(self.trace)
            record['odom_end_index'] = len(self.odom_trace)
        if recovered and self.remember_safe_arrival(None) is None:
            record.update(status='unsafe_arrival',reason='terminal_pose_not_navigable')
            return False
        if recovered and record['measured_odom_travel_m']<.20:
            odom_end=getattr(self,'previous_odom_pose',None)
            displacement=(math.dist(odom_source[1:],odom_end[1:])
                if odom_source is not None and odom_end is not None else 0.)
            grid=getattr(getattr(self,'sweep',None),'resolution',.05)
            minimum=max(.05,grid)
            net_minimum=max(.02,grid/2)
            record.update(measured_odom_displacement_m=displacement,
                          short_recovery_minimum_m=minimum,short_recovery_net_minimum_m=net_minimum)
            if (source_was_navigable or not math.isfinite(displacement) or displacement<net_minimum or
                    record['measured_odom_travel_m']<minimum):
                record.update(status='no_measured_progress',reason='recovery_requires_translation')
                return False
            record['confirmation']='unsafe_to_safe_with_measured_grid_translation'
        record['status'] = 'succeeded' if recovered else 'failed'
        return recovered

    def remember_safe_arrival(self, previous):
        """Only fresh measured navigable poses can replace a recovery anchor.

        A Nav2 success can precede settling or a localization correction. Keep
        the original action result, and independently check its measured pose.
        """
        live = self.available()
        pose = self.pose()
        usable = False
        cell = None
        if pose is not None:
            col,row = self.sweep.index(*pose[:2])
            cell = [col,row]
            height,width = live.shape
            usable = (0<=row<height and 0<=col<width and live[row,col]
                      and self.sweep.centers[row,col])
        check = dict(
            accepted=bool(usable),measured_pose=pose,cell=cell,
            basis='fresh_pose_static_centers_and_live_costmap')
        if self.goals:
            self.goals[-1]['recovery_anchor_check'] = check
        else:
            self.initial_anchor_check = check
        return pose if usable else previous

    def replan_lane(self, pending, deadline, excluded):
        """Try at most two other lanes after a settled failed segment.

        An action failure remains a failure. A new lane must actually move the
        base, finish its endpoint and pass the measured safety check to resume.
        """
        if len(self.replans)>=4 or deadline-time.monotonic()<25. or not self.settled:
            return None
        current = self.remember_safe_arrival(None)
        if current is None:
            return None
        attempted = [item['target'][:2] for event in self.replans for item in event['attempts']]
        choices = self.sweep.replan_options(current,pending,self.available(),excluded,attempted)
        event = dict(from_pose=list(current),attempts=[],status='unavailable')
        self.replans.append(event)
        for lane_id,start,end in choices:
            if deadline-time.monotonic()<20. or self.remember_safe_arrival(None) is None:
                break
            before = self.odom_distance
            item = dict(lane_id=lane_id,target=list(start),end=list(end),status='failed')
            item['odom_start_index']=len(self.odom_trace)
            event['attempts'].append(item)
            reached = self.go(start,deadline,purpose='lane_replan_start',recovery=True)
            finished = False
            if reached or self.continue_from_partial_lane(start):
                finished = self.follow_lane(start,end,deadline,purpose='lane_replan_end')
            safe = self.remember_safe_arrival(None)
            item['measured_odom_travel_m'] = self.odom_distance-before
            item['odom_end_index']=len(self.odom_trace)
            if finished and safe is not None and item['measured_odom_travel_m']>=.20:
                item['status'] = event['status'] = 'resumed'
                event['resumed_pose'] = list(safe)
                # Keep this original lane for its remaining segments, but do
                # not immediately repeat the just completed alternate segment.
                pending[:] = [p for p in pending if p[0]!=lane_id]+[p for p in pending if p[0]==lane_id]
                return safe
            if finished and item['measured_odom_travel_m']<.20:
                item['status'] = 'no_measured_progress'
        event['status'] = 'failed'
        return None

    def continue_from_partial_lane(self, target):
        """Continue a brush lane from a measured nearby terminal position.

        Keep the failed action status. This is not a successful Nav2 arrival
        and cannot satisfy entry/docking checks. It uses the existing .18 m
        lane-start rule, within the .25 m brush radius, at either lane end.
        """
        pose = self.pose()
        if not self.settled or pose is None or math.dist(pose[:2],target[:2])>.18:
            return False
        self.goals[-1]['continuation'] = dict(reason='continue_lane_from_partial_arrival',
            measured_pose=list(pose),distance_to_target_m=math.dist(pose[:2],target[:2]))
        return True

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

    def return_to_start(self,exit_pose):
        # The task's actual start remains separate from the configured dock.
        target=getattr(self,'dock_pose',None) or self.start_pose
        return dock_at(self,target,self.deadline,exit_pose=exit_pose,
            can_exit=lambda current:belongs_to(self.rooms,self.args.room,*current[:2]))

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
        self.return_reserve_seconds = self.sweep.return_reserve(self.dock_pose or self.start_pose,self.args.timeout)
        cutoff = self.deadline-self.return_reserve_seconds
        entrances = self.entrances()
        self.tick()
        self.wait_for(self.pose,2)
        self.stage = 'entering'
        entered, exit_pose = not entrances,None
        for entrance,exit_target in entrances:
            if self.go(entrance,cutoff,timeout=transit_timeout(self.pose() or self.start_pose,entrance),purpose='room_entry'):
                entered,exit_pose = True,exit_target
                break
        if entered:
            speed=observed_return_speed(self.odom_trace)
            self.return_budget_estimate=dict(source='initial_conservative_fallback',
                original_seconds=self.return_reserve_seconds,observed_speed=speed)
            if speed is not None:
                self.return_reserve_seconds=self.sweep.return_reserve(
                    self.dock_pose or self.start_pose,self.args.timeout,
                    travel_speed_mps=speed['selected_mps'])
                self.return_budget_estimate['source']='measured_ingress_cruising_median'
                cutoff=self.deadline-self.return_reserve_seconds
            self.return_budget_estimate['selected_seconds']=self.return_reserve_seconds
            route = self.sweep.route(self.pose() or self.start_pose,self.available())
            self.original_route = route
            self.planned_goals = len(route)
            self.record_navigation('initial_route',self.available())
            self.tick()
            self.wait_for(self.pose,2)
            self.stage = 'sweeping'
            supplement=getattr(self.args,'supplement_seconds',0.)
            if supplement:
                primary_cutoff=cutoff-supplement
                before=int(self.sweep.covered.sum())
                phase_start=time.monotonic()
                lane_navigation_ready=self.sweep_remaining(primary_cutoff,seed_only=True)
                self.cleaning_phases.append(dict(phase='primary',
                    elapsed_seconds=time.monotonic()-phase_start,
                    measured_new_cells=int(self.sweep.covered.sum())-before,
                    navigation_ready=lane_navigation_ready))
                # Re-enter only from a confirmed stopped pose. The new pass
                # keeps the original brush mask/stripe references and plans
                # from current live observations, without replaying the seed.
                if (not coverage_accepted(self.sweep,self.original_route) and self.settled
                        and time.monotonic()<cutoff and self.remember_safe_arrival(None) is not None):
                    cutoff=min(cutoff,time.monotonic()+supplement)
                    self.stage='supplementing'
                    before=int(self.sweep.covered.sum());phase_start=time.monotonic()
                    self.record_navigation('local_supplement_start',self.available(),
                        dedicated_budget_seconds=supplement,
                        actual_remaining_seconds=cutoff-time.monotonic())
                    self.local_remaining_pass=True
                    try:
                        lane_navigation_ready=self.sweep_remaining(cutoff,seeded=False)
                    finally:
                        self.local_remaining_pass=False
                    self.cleaning_phases.append(dict(phase='local_supplement',
                        elapsed_seconds=time.monotonic()-phase_start,
                        measured_new_cells=int(self.sweep.covered.sum())-before,
                        navigation_ready=lane_navigation_ready))
            else:
                lane_navigation_ready = self.sweep_remaining(cutoff)
            safe_pose = self.remember_safe_arrival(None)
            self.stage = 'supplementing'
            attempted = []
            for _ in range(10):
                current = self.pose()
                if (not lane_navigation_ready or current is None or time.monotonic()>=cutoff or
                        coverage_accepted(self.sweep,self.original_route)):
                    break
                target = self.sweep.repair_goal(current,attempted,self.available())
                if target is None:
                    break
                self.record_navigation('point_supplement',self.available(),target=target)
                attempted.append(target[:2])
                if self.go(target,cutoff):
                    safe_pose = self.remember_safe_arrival(safe_pose)
                else:
                    lane_navigation_ready = self.recover_lane(safe_pose,cutoff)
        if not entered:
            self.cleaning_stop_reason='room_entry_failed'
        elif coverage_accepted(self.sweep,self.original_route):
            self.cleaning_stop_reason='coverage_target_met'
        elif time.monotonic()>=cutoff:
            self.cleaning_stop_reason='cleaning_budget_exhausted'
        elif not lane_navigation_ready:
            self.cleaning_stop_reason=('recovery_budget_insufficient' if cutoff-time.monotonic()<20.
                                       else 'navigation_recovery_failed')
        elif self.pose() is None:
            self.cleaning_stop_reason='localization_unavailable'
        else:
            self.cleaning_stop_reason=self.remaining_stop_reason
        self.cleaning_time_remaining_seconds=max(0.,cutoff-time.monotonic())
        charging = self.return_to_start(exit_pose)
        coverage = self.sweep.metrics()['coverage_fraction']
        accepted=coverage_accepted(self.sweep,self.original_route)
        if entered and charging and accepted:
            return 'completed',f'覆盖达到80%且规划条带已实扫，已返回{self.return_label}并进入模拟充电。'
        if not entered:
            timed_out = any(goal.get('purpose')=='room_entry' and goal['status']=='timed_out'
                            for goal in self.goals)
            reason = '入房导航超时，尚未开始清扫' if timed_out else '入房导航失败，尚未开始清扫'
            docking = f'已返回{self.return_label}并进入模拟充电。' if charging else '已尝试返航，停靠未达标。'
            return 'failed',reason+'；'+docking
        reason={'cleaning_budget_exhausted':'本轮清扫时段已结束',
                'navigation_recovery_failed':'受阻后未能恢复清扫',
                'recovery_budget_insufficient':'受阻后剩余清扫时段不足以再恢复',
                'localization_unavailable':'清扫定位数据不可用',
                'remaining_lanes_exhausted':'当前可达未扫条带与补扫已执行完',
                'adaptive_attempt_limit':'本轮条带重选次数已到上限',
                'insufficient_segment_budget':'剩余清扫时段不足以完成下一段',
                'route_and_repairs_exhausted':'本轮路线与补扫已执行完'}.get(
                    self.cleaning_stop_reason,'清扫已结束')
        gap='规划条带仍有未扫区域' if coverage>=TARGET_COVERAGE else '覆盖仍低于目标'
        if charging:
            return 'partial',f'{reason}，{gap}；已返回{self.return_label}并进入模拟充电。'
        return 'partial',reason+'；已保存清扫轨迹，返航停靠未达标。'

    def report(self, status, message):
        summary = self.progress()
        summary.update(status=status,message=message,elapsed_seconds=round(time.monotonic()-self.started,2),
            coverage_target=TARGET_COVERAGE,sweep_width_m=SWEEP_WIDTH_M,lane_spacing_m=LANE_SPACING_M,
            center_clearance_m=CENTER_CLEARANCE_M,distance_m=round(self.distance,3),
            tracking_margin_cells=getattr(self.sweep,'tracking_margin_cells',TRACKING_MARGIN_CELLS),
            transit_clearance_m=getattr(self.sweep,'transit_clearance_m',CENTER_CLEARANCE_M),
            odom_distance_m=round(self.odom_distance,3),
            local_recovery_enabled=getattr(self.args,'local_recovery',False),
            lane_replans=self.replans,
            remaining_lane_attempts=self.segment_attempts,
            coverage_observations=self.coverage_observations,
            cleaning_phases=self.cleaning_phases,
            route_selection=('budget_aware_seed_and_remaining' if getattr(self.args,'budget_selection',False)
                             else 'seeded_route_then_reachable_remaining_lanes'),
            budget_selection_enabled=getattr(self.args,'budget_selection',False),
            initial_route_order=getattr(self.sweep,'route_order','adjacent') if self.sweep else None,
            navigation_masks_saved=bool(self.navigation_arrays),
            cleaning_stop_reason=getattr(self,'cleaning_stop_reason','not_started'),
            cleaning_time_remaining_seconds=getattr(self,'cleaning_time_remaining_seconds',None),
            initial_recovery_anchor_check=getattr(self,'initial_anchor_check',None),
            return_reserve_seconds=getattr(self,'return_reserve_seconds',None),
            return_budget_estimate=getattr(self,'return_budget_estimate',None),
            return_target_pose=self.dock_pose or self.start_pose,
            charging_station_id=self.station['id'] if self.station else None,
            initial_pose=self.start_pose,final_pose=self.final_pose,return_error=self.return_errors,
            return_tolerance=dict(position_m=RETURN_POSITION_M,yaw_rad=RETURN_YAW_RAD),
            navigation_settled=self.settled)
        summary['lane_recoveries'] = self.recoveries
        write_json(self.args.output/'clean_result.json',summary)
        write_json(self.args.output/'clean_trace.json',dict(frame_id='map',samples=self.trace,goals=self.goals,
                                                           odom_samples=self.odom_trace))
        self.save_navigation()
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
    parser.add_argument('--supplement-seconds',type=float,default=0.,
                        help='reserve up to180 seconds for a separate live remaining-floor pass')
    parser.add_argument('--position-goals',action='store_true',help='experimental position-only lane completion')
    parser.add_argument('--local-recovery',action='store_true',help='experimental recovery from recent measured trace')
    parser.add_argument('--budget-selection',action='store_true',help='experimental gain/time reordering; full-room validation failed')
    parser.add_argument('--station-config',type=Path,help='frozen map-bound charging facility configuration')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    if (not math.isfinite(args.supplement_seconds) or not 0<=args.supplement_seconds<=180
            or args.supplement_seconds>=args.timeout):
        parser.error('--supplement-seconds must be0–180 and below total timeout')
    stopped = []
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda signum,frame:stopped.append(signum))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    task = CleaningRun(args,stopped)
    try:
        status,message = task.run()
    except StopRequested:
        status,message = 'canceled','清扫已取消。'
        if task.stage!='returning':
            task.cleaning_stop_reason='canceled'
        if task.return_state=='returning':
            task.return_state = 'return_canceled'
        if task.settled:
            try:
                task.wait_for(task.parked,2,check_stop=False)
            except TimeoutError:
                task.settled = False
    except Exception as error:
        if task.stage!='returning':
            task.cleaning_stop_reason='execution_error'
        status,message = 'failed',str(error)
        if task.return_state=='returning':
            task.return_state = 'return_incomplete'
        if not task.settled:
            message += ' 导航终态或停稳未确认。'
    finally:
        try:
            task.report(status,message)
        finally:
            task.node.destroy_node()
            rclpy.try_shutdown()
    return 0 if status=='completed' else 1


if __name__=='__main__':
    raise SystemExit(main())
