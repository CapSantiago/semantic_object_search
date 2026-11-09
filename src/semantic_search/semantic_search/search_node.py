from dataclasses import asdict
from datetime import datetime, timezone
import json
from math import cos, isfinite, sin
from pathlib import Path
from uuid import uuid4

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray

from .semantic_map import LandmarkMap, parse_command, write_results
from .depth import foreground_depth, instance_centroid


class Search(Node):
    def __init__(self):
        super().__init__('semantic_search')
        config = Path(get_package_share_directory('semantic_search')) / 'config'
        self.declare_parameter('rooms_file', str(config / 'rooms.json'))
        self.declare_parameter('output_dir', 'results')
        self.declare_parameter('scene_id', 'baseline')
        self.declare_parameter('merge_radius', 0.30)
        self.declare_parameter('min_hits', 2)
        self.declare_parameter('dwell_seconds', 4.0)
        self.declare_parameter('min_score', 0.35)
        self.declare_parameter('optical_frame', 'camera_optical_frame')
        self.declare_parameter('depth_topic', '/rgbd/depth_image')
        self.declare_parameter('camera_info_topic', '/rgbd/camera_info')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.rooms = json.loads(Path(self.param('rooms_file')).read_text(encoding='utf-8'))
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.bridge = CvBridge()
        self.grid = None
        self.task = None
        self.goal = None
        self.phase = 'idle'
        self.canceled = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/search/markers', qos)
        self.results = self.create_publisher(String, '/search/result', qos)
        self.create_subscription(OccupancyGrid, '/map', self.set_map, qos)
        self.create_subscription(String, '/search/command', self.on_command, 10)

        self.inputs = [
            Subscriber(self, DetectionArray, self.param('detections_topic'),
                       qos_profile=qos_profile_sensor_data),
            Subscriber(self, Image, self.param('depth_topic'), qos_profile=qos_profile_sensor_data),
            Subscriber(self, CameraInfo, self.param('camera_info_topic'),
                       qos_profile=qos_profile_sensor_data),
        ]
        self.sync = ApproximateTimeSynchronizer(self.inputs, queue_size=20, slop=0.075)
        self.sync.registerCallback(self.on_observation)
        self.create_timer(0.2, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def set_map(self, grid):
        self.grid = grid

    def on_command(self, msg):
        try:
            if not self.task:
                self.rooms = json.loads(Path(self.param('rooms_file')).read_text(encoding='utf-8'))
            mode, room, category = parse_command(msg.data, self.rooms)
        except (OSError, ValueError) as error:
            self.get_logger().warning(str(error))
            return
        if mode == 'cancel':
            if self.task and self.phase == 'observing':
                self.finish('canceled')
            elif self.task and not self.canceled:
                self.canceled = True
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
        self.task_id = str(uuid4())
        self.command_text = msg.data
        self.canceled = False
        self.views = self.rooms[room]['views']
        self.view = 0
        self.visited = 0
        self.frames = 0
        self.observations = []
        self.tf_failures = 0
        self.valid_projection = 0
        self.invalid_target_depth = 0
        self.started = self.get_clock().now().nanoseconds
        self.landmarks = LandmarkMap(self.param('merge_radius'), self.param('min_hits'))
        self.publish_markers([])
        self.get_logger().info(f'开始任务 {mode}: {category} in {room}')
        self.next_view()

    def next_view(self):
        if self.view == len(self.views):
            if not self.frames:
                status = 'no_sensor_data'
            elif not self.valid_projection and (self.tf_failures or self.invalid_target_depth):
                status = 'localization_failed'
            else:
                status = 'completed' if self.landmarks.confirmed else 'not_found'
            self.finish(status)
            return

        x, y, yaw = self.views[self.view]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = sin(yaw / 2)
        goal.pose.pose.orientation.w = cos(yaw / 2)
        self.phase = 'navigating'
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
        if self.canceled:
            self.finish('canceled')
        elif status != GoalStatus.STATUS_SUCCEEDED:
            self.finish('navigation_failed')
        else:
            self.visited += 1
            self.phase = 'observing'
            self.observe_start = self.get_clock().now().nanoseconds
            self.last_stamp = -1

    def tick(self):
        if self.phase != 'observing':
            return
        elapsed = (self.get_clock().now().nanoseconds - self.observe_start) / 1e9
        if elapsed >= self.param('dwell_seconds'):
            self.view += 1
            self.next_view()

    def on_observation(self, detections, depth, info):
        if self.phase != 'observing':
            return
        stamp = Time.from_msg(detections.header.stamp)
        if stamp.nanoseconds < self.observe_start or stamp.nanoseconds <= self.last_stamp:
            return
        if depth.width != info.width or depth.height != info.height:
            return
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]
        if not (fx > 0 and fy > 0):
            return
        if depth.encoding not in ('32FC1', '16UC1'):
            return
        self.last_stamp = stamp.nanoseconds
        self.frames += 1
        mode, room, category = self.task
        targets = [det for det in detections.detections
                   if det.class_name.lower() == category and det.score >= self.param('min_score')]
        if not targets:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        except CvBridgeError as error:
            self.invalid_target_depth += 1
            self.get_logger().warning(f'读取深度图失败：{error}')
            return
        try:
            transform = self.tf.lookup_transform('map', self.param('optical_frame'), stamp)
        except TransformException:
            self.tf_failures += 1
            return
        scale = 0.001 if depth.encoding == '16UC1' else 1.0
        left, bottom, right, top = self.rooms[room]['bounds']
        points = []
        for target in targets:
            center = target.bbox.center.position
            if not (isfinite(center.x) and isfinite(center.y)):
                continue
            u, v = round(center.x), round(center.y)
            if not (0 <= u < depth.width and 0 <= v < depth.height):
                continue
            polygon = None
            if target.mask.data:
                if target.mask.width != depth.width or target.mask.height != depth.height:
                    self.invalid_target_depth += 1
                    continue
                polygon = [(p.x, p.y) for p in target.mask.data]
            if polygon is not None:
                t, q = transform.transform.translation, transform.transform.rotation
                mapped = instance_centroid(image, polygon, info.k,
                    [t.x, t.y, t.z], [q.x, q.y, q.z, q.w], scale)
                if mapped is None:
                    self.invalid_target_depth += 1
                    continue
                self.valid_projection += 1
                if left <= mapped[0] <= right and bottom <= mapped[1] <= top:
                    points.append(mapped)
                continue
            sample = foreground_depth(image, center.x, center.y,
                                      target.bbox.size.x, target.bbox.size.y, scale, polygon)
            if sample is None:
                self.invalid_target_depth += 1
                continue
            u, v, z = sample
            point = PointStamped()
            point.header.stamp = detections.header.stamp
            point.header.frame_id = self.param('optical_frame')
            point.point.x = (u - cx) * z / fx
            point.point.y = (v - cy) * z / fy
            point.point.z = z
            mapped = do_transform_point(point, transform).point
            self.valid_projection += 1
            if left <= mapped.x <= right and bottom <= mapped.y <= top:
                points.append((mapped.x, mapped.y, mapped.z))
        point_ids = self.landmarks.observe(points)
        self.observations.append(dict(viewpoint=self.view, stamp_ns=stamp.nanoseconds,
                                      detections=len(targets), points=points,
                                      instance_ids=point_ids,
                                      confirmed_ids=[item.id for item in self.landmarks.confirmed]))
        if mode == 'find' and self.landmarks.confirmed:
            self.finish('found')

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
                      viewpoints=self.views[:self.visited],
                      elapsed_seconds=(self.get_clock().now().nanoseconds-self.started)/1e9)
        self.task = self.goal = None
        self.phase = 'idle'
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
        try:
            destination = Path(self.param('output_dir')) / folder
            write_results(destination, result, self.grid)
            (destination / 'observations.json').write_text(
                json.dumps(self.observations, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        except OSError as error:
            self.get_logger().error(f'保存搜索结果失败：{error}')


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

