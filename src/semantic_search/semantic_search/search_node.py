from dataclasses import asdict
from datetime import datetime, timezone
import json
from math import atan2, cos, isfinite, sin
from pathlib import Path
from time import monotonic
from uuid import uuid4

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray

from .semantic_map import LandmarkMap, parse_command, write_results
from .objects import admitted_aliases, validate_categories
from .active_observation import ActiveObservation
from .depth import foreground_depth, instance_geometry, compatible_size, truncated_box
from .rooms import belongs_to, validate_rooms, observation_views, observation_status, observation_coverage, inspection_coverage
from .sensor_qos import declare_sensor_qos


class Search(Node):
    def __init__(self):
        super().__init__('semantic_search')
        config = Path(get_package_share_directory('semantic_search')) / 'config'
        self.declare_parameter('rooms_file', str(config / 'rooms.json'))
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('scene_id', 'baseline')
        self.declare_parameter('target_categories', list(admitted_aliases()), ParameterDescriptor(read_only=True))
        self.target_categories = validate_categories(self.param('target_categories'))
        self.declare_parameter('active_observation', False, ParameterDescriptor(read_only=True))
        self.declare_parameter('memory_guidance', False)
        self.declare_parameter('memory_safe_approach', False)
        self.declare_parameter('memory_database', '', ParameterDescriptor(read_only=True))
        self.declare_parameter('memory_horizontal_fov', 1.02974, ParameterDescriptor(read_only=True))
        self.active_enabled = self.param('active_observation')
        self.declare_parameter('merge_radius', 0.30)
        self.declare_parameter('min_hits', 2)
        self.declare_parameter('dwell_seconds', 4.0)
        self.declare_parameter('observation_extra_seconds', 4.0)
        validation = self.validate_observation_parameters([
            self.get_parameter('dwell_seconds'), self.get_parameter('observation_extra_seconds')])
        if not validation.successful:
            raise ValueError(validation.reason)
        self.add_on_set_parameters_callback(self.validate_observation_parameters)
        self.declare_parameter('min_score', 0.35)
        self.declare_parameter('reject_clipped_detections', False)
        self.declare_parameter('validate_object_size', False)
        self.declare_parameter('optical_frame', 'camera_optical_frame')
        self.declare_parameter('depth_topic', '/rgbd/depth_image')
        self.declare_parameter('camera_info_topic', '/rgbd/camera_info')
        self.declare_parameter('detections_topic', '/yolo/detections')
        sensor_qos = declare_sensor_qos(self)
        self.rooms = json.loads(Path(self.param('rooms_file')).read_text(encoding='utf-8'))
        validate_rooms(self.rooms)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.bridge = CvBridge()
        self.grid = None
        self.costmap = None
        self.costmap_wall = None
        self.task = None
        self.goal = None
        self.phase = 'idle'
        self.request_id = None
        self.canceled = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/search/markers', qos)
        self.results = self.create_publisher(String, '/search/result', qos)
        self.progress = self.create_publisher(String, '/search/progress', 10)
        self.create_subscription(OccupancyGrid, '/map', self.set_map, qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.set_costmap, qos)
        self.create_subscription(String, '/search/command', self.on_command, 10)

        self.inputs = [
            Subscriber(self, DetectionArray, self.param('detections_topic'),
                       qos_profile=sensor_qos),
            Subscriber(self, Image, self.param('depth_topic'), qos_profile=sensor_qos),
            Subscriber(self, CameraInfo, self.param('camera_info_topic'),
                       qos_profile=sensor_qos),
        ]
        self.sync = ApproximateTimeSynchronizer(self.inputs, queue_size=20, slop=0.075)
        self.sync.registerCallback(self.on_observation)
        self.create_timer(0.2, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def validate_observation_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name in ('dwell_seconds', 'observation_extra_seconds'):
                value = parameter.value
                if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                        not isfinite(value) or value < 0):
                    return SetParametersResult(successful=False,
                        reason=f'{parameter.name} must be finite and nonnegative')
        return SetParametersResult(successful=True)

    def set_map(self, grid):
        self.grid = grid

    def set_costmap(self, grid):
        self.costmap, self.costmap_wall = grid, monotonic()

    @staticmethod
    def base_pose(transform):
        p,q = ((transform.translation,transform.rotation) if hasattr(transform,'translation')
               else (transform.position,transform.orientation))
        return (p.x,p.y,atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def observe_geometry(self, depth, info, stamp, detections=None):
        if not getattr(self,'active_enabled',False) or stamp.nanoseconds-self.geometry_stamp<500_000_000:
            return
        self.geometry_stamp = stamp.nanoseconds
        try:
            image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            base = self.base_pose(self.tf.lookup_transform('map','base_link',stamp).transform)
            optical = self.tf.lookup_transform('map',self.param('optical_frame'),stamp).transform
            if self.active is None:
                if self.grid is None:
                    raise ValueError('No occupancy map for active observation')
                i = self.grid.info
                origin = self.base_pose(i.origin)
                self.active = ActiveObservation(self.grid.data,i.width,i.height,i.resolution,
                                               origin,self.rooms,self.task[1],base)
            t,q = optical.translation,optical.rotation
            self.active.observe(image,info.k,[t.x,t.y,t.z],[q.x,q.y,q.z,q.w],base,
                                scale=.001 if depth.encoding=='16UC1' else 1.)
            if (detections is not None and self.task[2] in
                    ('cup','bottle','bowl','book','cell phone','vase','apple','banana','orange')):
                surfaces = []
                for det in detections.detections:
                    if det.class_name.lower()!='dining table' or not isfinite(det.score) or det.score<.5:
                        continue
                    b = det.bbox
                    if (not det.mask.data or det.mask.width!=depth.width or det.mask.height!=depth.height or
                            truncated_box(b.center.position.x,b.center.position.y,b.size.x,b.size.y,
                                          depth.width,depth.height)):
                        continue
                    g = instance_geometry(image,[(p.x,p.y) for p in det.mask.data],info.k,
                        [t.x,t.y,t.z],[q.x,q.y,q.z,q.w],.001 if depth.encoding=='16UC1' else 1.)
                    if (g is not None and .25<g[0][2]<1.25 and
                            belongs_to(self.rooms,self.task[1],*g[0][:2])):
                        surfaces.append(g[0])
                self.support_surfaces.observe(surfaces)
            self.active_error = None
        except (ValueError,TransformException,CvBridgeError) as error:
            self.active_error = str(error)

    def append_active_view(self):
        if not getattr(self,'active_enabled',False) or self.active is None:
            return False
        if monotonic()-self.started_wall>420.:
            self.active.reason = 'task_time_budget_exhausted'
            return False
        try:
            import numpy as np
            if self.costmap_wall is None or monotonic()-self.costmap_wall>3.:
                raise ValueError('No fresh global costmap for active observation')
            a,b = self.grid.info,self.costmap.info
            if (self.costmap.header.frame_id!='map' or (a.width,a.height)!=(b.width,b.height)
                or abs(a.resolution-b.resolution)>1e-5
                or any(abs(x-y)>1e-4 for x,y in zip(self.base_pose(a.origin),self.base_pose(b.origin)))):
                raise ValueError('Active observation costmap grid mismatch')
            transform = self.tf.lookup_transform('map','base_link',Time())
            age = self.get_clock().now().nanoseconds-Time.from_msg(transform.header.stamp).nanoseconds
            if not -1_500_000_000<=age<=750_000_000:
                raise ValueError('No fresh map pose for active observation')
            cost = np.asarray(self.costmap.data).reshape(b.height,b.width)
            pose = self.active.next_view(self.base_pose(transform.transform),self.views,
                                         (cost>=0)&(cost<99),
                                         [(item.id,item.x,item.y,item.z) for item in self.landmarks.items
                                          if item.hits<self.landmarks.min_hits],
                                         [(item.id,item.x,item.y,item.z) for item in self.support_surfaces.confirmed])
            if pose is None:
                return False
            self.views.append(list(pose))
            self.viewpoint_diagnostics.append(dict(index=self.view,pose=list(pose),
                source=self.active.decisions[-1].get('source','active_visibility'),
                reached=False,navigation_status=None,dwell_completed=False,extra_wait_started=False,
                waiting_for_sensor_data=False,extra_wait_seconds=0.,synchronized_messages=0,
                rejected_sensor_frames=0,synchronized_frames=0,tf_failures=0,invalid_target_depth=0,
                valid_projections=0))
            return True
        except (ValueError,TransformException) as error:
            self.active_error = str(error)
            self.active.reason = 'planning_data_unavailable'
            return False

    def on_command(self, msg):
        try:
            if not self.task:
                self.rooms = json.loads(Path(self.param('rooms_file')).read_text(encoding='utf-8'))
                validate_rooms(self.rooms)
            mode, room, category = parse_command(msg.data, self.rooms, getattr(self,'target_categories',None))
            request_id = json.loads(msg.data).get('request_id') if msg.data.lstrip().startswith('{') else None
        except (OSError, ValueError) as error:
            self.get_logger().warning(str(error))
            return
        if mode == 'cancel':
            if request_id is not None and request_id != self.request_id:
                return
            if self.task and self.phase == 'observing':
                self.finish('canceled')
            elif self.task and not self.canceled:
                self.canceled = True
                self.publish_progress('canceling')
                if self.goal:
                    self.goal.cancel_goal_async()
            return
        if self.task:
            self.get_logger().warning('已有任务进行中。')
            return
        if not self.nav.server_is_ready():
            self.get_logger().error('Nav2 navigate_to_pose 尚未就绪。')
            return

        self.task = (mode, room, category)
        self.request_id = request_id
        self.task_id = str(uuid4())
        self.command_text = msg.data
        self.canceled = False
        self.views = [list(pose) for pose in observation_views(self.rooms[room], category)]
        self.memory_guidance = dict(enabled=False,used=False,counts_from_history=False)
        if mode=='find' and self.param('memory_guidance'):
            try:
                from .semantic_memory import SemanticMemory, map_namespace
                from .memory_guidance import prioritize_find_views
                if self.grid is None or not self.param('memory_database'):
                    raise ValueError('Map or memory database unavailable')
                info=self.grid.info;p=info.origin.position;q=info.origin.orientation
                origin_yaw=atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                memory_grid=dict(available=True,frame_id=self.grid.header.frame_id,width=info.width,
                    height=info.height,resolution=info.resolution,
                    origin=dict(x=float(p.x),y=float(p.y),yaw=origin_yaw),
                    data=list(self.grid.data))
                namespace=map_namespace(memory_grid,self.rooms)
                memory=SemanticMemory(self.param('memory_database'))
                snapshot=memory.query(namespace,room=room,category=category)
                successful=memory.successful_viewpoints(namespace,snapshot['objects'],
                    root=Path(self.param('memory_database')).resolve().parent.parent)
                approach_context=None;approach_error=None
                try:
                    import numpy as np
                    if not self.param('memory_safe_approach'):
                        raise ValueError('Safe approach candidate disabled')
                    if self.costmap_wall is None or monotonic()-self.costmap_wall>3.:
                        raise ValueError('No fresh approach costmap')
                    a,b=self.grid.info,self.costmap.info
                    if (self.costmap.header.frame_id!='map' or (a.width,a.height)!=(b.width,b.height)
                            or abs(a.resolution-b.resolution)>1e-5
                            or any(abs(x-y)>1e-4 for x,y in zip(self.base_pose(a.origin),self.base_pose(b.origin)))):
                        raise ValueError('Approach grid mismatch')
                    transform=self.tf.lookup_transform('map','base_link',Time())
                    age=self.get_clock().now().nanoseconds-Time.from_msg(transform.header.stamp).nanoseconds
                    if not -1_500_000_000<=age<=750_000_000:raise ValueError('Approach pose stale')
                    cost=np.asarray(self.costmap.data).reshape(b.height,b.width)
                    approach_context=dict(rooms=self.rooms,room_id=room,grid=memory_grid,
                        available=(cost>=0)&(cost<99),current=self.base_pose(transform.transform))
                except (ValueError,TransformException) as error:
                    approach_error=str(error)
                self.views,self.memory_guidance=prioritize_find_views(self.views,self.rooms[room],snapshot['objects'],
                    half_fov=self.param('memory_horizontal_fov')/2,successful_views=successful,
                    approach_context=approach_context)
                if approach_error:self.memory_guidance['approach_unavailable']=approach_error
                for item in self.memory_guidance.get('verification_views',[]):
                    if item.get('basis')=='observed_memory_safe_approach':
                        item['planning'].update(costmap_stamp_ns=Time.from_msg(self.costmap.header.stamp).nanoseconds,
                            decision_stamp_ns=self.get_clock().now().nanoseconds,
                            costmap_received_age_seconds=monotonic()-self.costmap_wall)
                self.memory_guidance['namespace']=namespace
            except Exception as error:
                self.memory_guidance=dict(enabled=True,used=False,counts_from_history=False,
                    reason='history_unavailable',error=str(error))
        self.active = None
        self.active_error = None
        self.geometry_stamp = -1
        self.view = 0
        self.visited = 0
        self.visited_poses = []
        self.failed_views = []
        self.viewpoint_diagnostics = [dict(
            index=index, pose=pose, reached=False, navigation_status=None, dwell_completed=False,
            extra_wait_started=False, waiting_for_sensor_data=False, extra_wait_seconds=0.0,
            synchronized_messages=0, rejected_sensor_frames=0, synchronized_frames=0,
            tf_failures=0, invalid_target_depth=0, valid_projections=0)
            for index, pose in enumerate(self.views)]
        for item in self.memory_guidance.get('verification_views',[]):
            if item.get('basis')=='observed_memory_safe_approach':
                for view in self.viewpoint_diagnostics:
                    if view['pose']==item['pose']:view['source']='memory_safe_approach'
        self.frames = 0
        self.observations = []
        self.tf_failures = 0
        self.valid_projection = 0
        self.invalid_target_depth = 0
        self.started = self.get_clock().now().nanoseconds
        self.started_wall = monotonic()
        self.landmarks = LandmarkMap(self.param('merge_radius'), self.param('min_hits'))
        self.support_surfaces = LandmarkMap(.6,3)
        self.publish_markers([])
        self.get_logger().info(f'开始任务 {mode}: {category} in {room}')
        self.next_view()

    def publish_progress(self, stage, *, status=None, save_status=None):
        mode, room, category = self.task
        event = dict(version=1, request_id=self.request_id, task_id=self.task_id,
                     kind='search', stage=stage, status=status, save_status=save_status,
                     mode=mode, room=room, category=category, count=len(self.landmarks.confirmed),
                     viewpoint_index=self.view+1 if self.view < len(self.views) else None,
                     viewpoints_total=len(self.views), viewpoints_visited=self.visited,
                     viewpoints_failed=len(self.failed_views))
        if self.view<len(self.viewpoint_diagnostics):
            event['viewpoint_source'] = self.viewpoint_diagnostics[self.view].get('source','configured')
        config=getattr(self,'rooms',{}).get(room,{})
        event['memory_guidance']=getattr(self,'memory_guidance',dict(used=False))
        event['inspection_coverage']=inspection_coverage(config,self.viewpoint_diagnostics)
        if self.view<len(self.views):
            for key,poses in config.get('inspection_views',{}).items():
                if self.views[self.view] in poses:
                    event['inspection_area']=config.get('inspection_labels',{}).get(key,key)
        self.progress.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def next_view(self):
        task=getattr(self,'task',None)
        if (task and task[0]=='find' and self.landmarks.confirmed
                and self.view>0
                and self.viewpoint_diagnostics[self.view-1]['dwell_completed']
                and inspection_coverage(getattr(self,'rooms',{}).get(task[1],{}),
                                        self.viewpoint_diagnostics)['completed']):
            self.finish('found')
            return
        if self.view == len(self.views) and getattr(self,'active_enabled',False):
            self.append_active_view()
        if self.view == len(self.views):
            status = observation_status(self.task[0], self.frames, bool(self.landmarks.confirmed),
                self.valid_projection, self.tf_failures, self.invalid_target_depth, self.failed_views,
                self.viewpoint_diagnostics)
            if (getattr(self,'active_enabled',False) and status in ('completed','not_found')
                and (self.active is None or self.active_error
                     or self.active.summary()['observed_fraction']<self.active.target_fraction)):
                status = 'partial'
            self.finish(status)
            return

        x, y, yaw = self.views[self.view]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = sin(yaw / 2)
        goal.pose.pose.orientation.w = cos(yaw / 2)
        self.phase = 'navigating'
        self.publish_progress('navigating')
        self.nav.send_goal_async(goal).add_done_callback(self.on_goal)

    def on_goal(self, future):
        try:
            self.goal = future.result()
        except Exception as error:
            self.get_logger().error(f'发送导航目标失败：{error}')
            self.finish('navigation_failed')
            return
        if not self.goal.accepted:
            self.finish('canceled' if self.canceled else 'navigation_rejected')
            return
        self.goal.get_result_async().add_done_callback(self.on_navigation)
        if self.canceled:
            self.goal.cancel_goal_async()

    def on_navigation(self, future):
        self.goal = None
        try:
            status = future.result().status
        except Exception as error:
            self.get_logger().error(f'读取导航结果失败：{error}')
            self.finish('navigation_failed')
            return
        self.viewpoint_diagnostics[self.view]['navigation_status'] = status
        if self.canceled:
            self.viewpoint_diagnostics[self.view]['task_canceled'] = True
            self.finish('canceled')
        elif status != GoalStatus.STATUS_SUCCEEDED:
            self.failed_views.append(dict(index=self.view, pose=self.views[self.view],
                                          navigation_status=status))
            self.get_logger().warning(f'观察点 {self.view} 导航失败，继续检查其他视点。')
            self.view += 1
            self.next_view()
        else:
            self.viewpoint_diagnostics[self.view]['reached'] = True
            self.visited += 1
            self.visited_poses.append(self.views[self.view])
            self.phase = 'observing'
            self.observe_start = self.get_clock().now().nanoseconds
            self.observe_dwell_seconds = self.param('dwell_seconds')
            self.observe_extra_seconds = self.param('observation_extra_seconds')
            self.last_stamp = -1
            diagnostics = self.viewpoint_diagnostics[self.view]
            diagnostics['nominal_dwell_seconds'] = self.observe_dwell_seconds
            diagnostics['extra_wait_limit_seconds'] = self.observe_extra_seconds
            diagnostics['arrival_sim_seconds'] = self.observe_start / 1e9
            diagnostics['arrival_pose_lookup'] = 'latest'
            diagnostics['arrival_map_pose'] = None
            diagnostics['arrival_transform_stamp_ns'] = None
            diagnostics['arrival_pose_error'] = None
            try:
                arrival = self.tf.lookup_transform('map', 'base_link', Time())
                position, orientation = arrival.transform.translation, arrival.transform.rotation
                diagnostics['arrival_map_pose'] = dict(
                    frame_id='map', child_frame_id='base_link',
                    xyz=[position.x, position.y, position.z],
                    xyzw=[orientation.x, orientation.y, orientation.z, orientation.w])
                diagnostics['arrival_transform_stamp_ns'] = Time.from_msg(arrival.header.stamp).nanoseconds
            except TransformException as error:
                diagnostics['arrival_pose_error'] = str(error)
            self.publish_progress('observing')

    def tick(self):
        if self.phase != 'observing':
            return
        elapsed = (self.get_clock().now().nanoseconds - self.observe_start) / 1e9
        if elapsed < self.observe_dwell_seconds:
            return
        diagnostics = self.viewpoint_diagnostics[self.view]
        diagnostics['dwell_completed'] = True
        if not diagnostics['synchronized_frames'] and self.observe_extra_seconds > 0:
            diagnostics['extra_wait_started'] = True
        if diagnostics['extra_wait_started']:
            diagnostics['extra_wait_seconds'] = max(0.0, elapsed-self.observe_dwell_seconds)
        was_waiting = diagnostics['waiting_for_sensor_data']
        diagnostics['waiting_for_sensor_data'] = (
            not diagnostics['synchronized_frames'] and
            elapsed < self.observe_dwell_seconds+self.observe_extra_seconds)
        if diagnostics['waiting_for_sensor_data']:
            if not was_waiting:
                self.publish_progress('waiting_for_data')
            return
        self.view += 1
        self.next_view()

    def record_observation(self, field):
        """Keep existing task counters and the active viewpoint's counters in step."""
        self.viewpoint_diagnostics[self.view][field] += 1
        attribute = {'synchronized_frames': 'frames', 'valid_projections': 'valid_projection'}.get(field, field)
        setattr(self, attribute, getattr(self, attribute) + 1)

    def on_observation(self, detections, depth, info):
        if self.phase != 'observing':
            return
        diagnostics = self.viewpoint_diagnostics[self.view]
        if diagnostics['waiting_for_sensor_data'] or not diagnostics['synchronized_frames']:
            elapsed = (self.get_clock().now().nanoseconds-self.observe_start)/1e9
            if elapsed >= self.observe_dwell_seconds+self.observe_extra_seconds:
                self.tick()
                return
        stamp = Time.from_msg(detections.header.stamp)
        if stamp.nanoseconds < self.observe_start or stamp.nanoseconds <= self.last_stamp:
            return
        self.last_stamp = stamp.nanoseconds
        diagnostics = self.viewpoint_diagnostics[self.view]
        diagnostics['synchronized_messages'] += 1
        if (depth.width <= 0 or depth.height <= 0 or
                depth.width != info.width or depth.height != info.height):
            diagnostics['rejected_sensor_frames'] += 1
            return
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]
        if not (isfinite(fx) and isfinite(fy) and isfinite(cx) and isfinite(cy) and fx > 0 and fy > 0):
            diagnostics['rejected_sensor_frames'] += 1
            return
        if depth.encoding not in ('32FC1', '16UC1'):
            diagnostics['rejected_sensor_frames'] += 1
            return
        self.record_observation('synchronized_frames')
        if getattr(self,'active_enabled',False):
            self.observe_geometry(depth,info,stamp,detections)
        mode, room, category = self.task
        targets = [det for det in detections.detections
                   if det.class_name.lower() == category and det.score >= self.param('min_score')]
        if self.param('reject_clipped_detections') is True:
            complete=[]
            for det in targets:
                box=det.bbox
                if truncated_box(box.center.position.x,box.center.position.y,box.size.x,box.size.y,
                                 depth.width,depth.height):
                    diagnostics['clipped_detections']=diagnostics.get('clipped_detections',0)+1
                else:
                    complete.append(det)
            targets=complete
        if not targets:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        except CvBridgeError as error:
            self.record_observation('invalid_target_depth')
            self.get_logger().warning(f'读取深度图失败：{error}')
            return
        try:
            transform = self.tf.lookup_transform('map', self.param('optical_frame'), stamp)
        except TransformException:
            self.record_observation('tf_failures')
            return
        scale = 0.001 if depth.encoding == '16UC1' else 1.0
        points = []
        for target in targets:
            center = target.bbox.center.position
            if not (isfinite(center.x) and isfinite(center.y)):
                self.record_observation('invalid_target_depth')
                continue
            u, v = round(center.x), round(center.y)
            if not (0 <= u < depth.width and 0 <= v < depth.height):
                self.record_observation('invalid_target_depth')
                continue
            polygon = None
            if target.mask.data:
                if target.mask.width != depth.width or target.mask.height != depth.height:
                    self.record_observation('invalid_target_depth')
                    continue
                polygon = [(p.x, p.y) for p in target.mask.data]
            if polygon is not None:
                t, q = transform.transform.translation, transform.transform.rotation
                geometry = instance_geometry(image, polygon, info.k,
                    [t.x, t.y, t.z], [q.x, q.y, q.z, q.w], scale, category=category)
                mapped = geometry[0] if geometry is not None else None
                if mapped is None:
                    self.record_observation('invalid_target_depth')
                    continue
                self.record_observation('valid_projections')
                if self.param('validate_object_size') is True and not compatible_size(category,geometry[1]):
                    diagnostics['size_rejected_detections']=diagnostics.get('size_rejected_detections',0)+1
                    continue
                if belongs_to(self.rooms, room, mapped[0], mapped[1]):
                    points.append(mapped)
                continue
            sample = foreground_depth(image, center.x, center.y,
                                      target.bbox.size.x, target.bbox.size.y, scale, polygon)
            if sample is None:
                self.record_observation('invalid_target_depth')
                continue
            u, v, z = sample
            point = PointStamped()
            point.header.stamp = detections.header.stamp
            point.header.frame_id = self.param('optical_frame')
            point.point.x = (u - cx) * z / fx
            point.point.y = (v - cy) * z / fy
            point.point.z = z
            mapped = do_transform_point(point, transform).point
            self.record_observation('valid_projections')
            if belongs_to(self.rooms, room, mapped.x, mapped.y):
                points.append((mapped.x, mapped.y, mapped.z))
        before = len(self.landmarks.confirmed)
        point_ids = self.landmarks.observe(points)
        self.observations.append(dict(viewpoint=self.view, stamp_ns=stamp.nanoseconds,
                                      detections=len(targets), points=points,
                                      instance_ids=point_ids,
                                      confirmed_ids=[item.id for item in self.landmarks.confirmed]))
        # Keep observing for the configured dwell even after two hits confirm
        # an instance. next_view finishes find after this observation window;
        # later synchronized measurements still update its map estimate.
        if len(self.landmarks.confirmed) != before:
            self.publish_progress('observing')

    def publish_markers(self, objects):
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.action = Marker.DELETEALL
        markers = [clear]
        for item in objects:
            for ns, kind in [('objects', Marker.SPHERE), ('labels', Marker.TEXT_VIEW_FACING)]:
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.ns = ns
                marker.id = item['id']
                marker.type = kind
                marker.action = Marker.ADD
                marker.pose.position.x = item['x']
                marker.pose.position.y = item['y']
                marker.pose.position.z = item['z'] + (0.25 if ns == 'labels' else 0.0)
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.15
                marker.color.r, marker.color.g, marker.color.a = 1.0, 0.2, 1.0
                marker.text = str(item['id'])
                markers.append(marker)
        self.markers.publish(MarkerArray(markers=markers))

    def finish(self, status):
        if self.phase == 'observing' and self.view < len(self.viewpoint_diagnostics):
            diagnostics = self.viewpoint_diagnostics[self.view]
            if diagnostics['extra_wait_started']:
                elapsed = (self.get_clock().now().nanoseconds-self.observe_start)/1e9
                diagnostics['extra_wait_seconds'] = max(0.0, elapsed-self.observe_dwell_seconds)
            diagnostics['waiting_for_sensor_data'] = False
        mode, room, category = self.task
        confirmed = self.landmarks.confirmed
        if mode == 'find':
            confirmed = confirmed[:1]
        objects = [asdict(item) for item in confirmed]
        result = dict(scene_id=self.param('scene_id'), task_id=self.task_id, command=self.command_text,
                      frame_id='map', coordinate_unit='m',
                      mode=mode, room=room, category=category, status=status,
                      count=len(objects), objects=objects, viewpoints_visited=self.visited,
                      viewpoints_total=len(self.views), synchronized_frames=self.frames,
                      tf_failures=self.tf_failures, invalid_target_depth=self.invalid_target_depth,
                      valid_projections=self.valid_projection,
                      viewpoints=self.visited_poses, viewpoint_failures=self.failed_views,
                      viewpoint_diagnostics=self.viewpoint_diagnostics,
                      elapsed_seconds=(self.get_clock().now().nanoseconds-self.started)/1e9,
                      elapsed_wall_seconds=monotonic()-self.started_wall)
        result['coverage'] = observation_coverage(self.viewpoint_diagnostics)
        result['memory_guidance']=getattr(self,'memory_guidance',dict(used=False,counts_from_history=False))
        result['inspection_coverage'] = inspection_coverage(getattr(self,'rooms',{}).get(room,{}),self.viewpoint_diagnostics)
        if status in ('found','completed','not_found') and not result['inspection_coverage']['completed']:
            status='partial'
            result['status']=status
        if getattr(self,'active_enabled',False):
            result['geometric_coverage'] = self.active.summary() if self.active is not None else dict(
                stop_reason='no_valid_geometry',full_room_coverage_verified=False)
            result['geometric_coverage']['error'] = self.active_error
        result['clipped_detections'] = sum(v.get('clipped_detections',0) for v in self.viewpoint_diagnostics)
        result['size_rejected_detections'] = sum(v.get('size_rejected_detections',0) for v in self.viewpoint_diagnostics)
        if self.request_id is not None:
            result['request_id'] = self.request_id
        self.publish_progress('saving', status=status, save_status='pending')
        self.publish_markers(objects)
        self.results.publish(String(data=json.dumps(result, ensure_ascii=False)))
        if status == 'found':
            item = objects[0]
            self.get_logger().info(f'已找到，地图坐标为 ({item["x"]:.2f}, {item["y"]:.2f}, {item["z"]:.2f}) m。')
        elif status == 'completed':
            self.get_logger().info(f'统计完成：{room} 中确认 {len(objects)} 个 {category}。')
        elif status == 'not_found':
            self.get_logger().info('未找到指定物品。')
        self.get_logger().info(json.dumps(result, ensure_ascii=False))
        folder = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        save_status = 'failed'
        try:
            destination = Path(self.param('output_dir')) / folder
            write_results(destination, result, self.grid)
            (destination / 'observations.json').write_text(
                json.dumps(self.observations, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            if getattr(self,'active',None) is not None:
                (destination/'coverage_samples.json').write_text(
                    json.dumps(self.active.evidence(),ensure_ascii=False)+'\n',encoding='utf-8')
            save_status = 'saved' if self.grid is not None else 'partial'
        except OSError as error:
            self.get_logger().error(f'保存搜索结果失败：{error}')
        finally:
            self.publish_progress('finished', status=status, save_status=save_status)
            self.task = self.goal = None
            self.request_id = None
            self.phase = 'idle'


def main():
    rclpy.init()
    node = None
    try:
        node = Search()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

