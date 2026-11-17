#!/usr/bin/env python3
"""Read-only ROS telemetry for the local console; ROS imports are lazy.

The monitor owns only its node/context, and never sends a robot command. All
freshness calculations use monotonic receipt time plus the observed /clock.
"""
from copy import deepcopy
import math
import os
import threading
import time


TOPICS = ('/clock', '/odom', '/scan', '/rgbd/obstacles', '/rgbd/image',
          '/rgbd/depth_image', '/rgbd/camera_info', '/yolo/detections')
SIMULATION_TOPICS = ('/clock', '/odom', '/scan', '/rgbd/obstacles')
PERCEPTION_TOPICS = ('/rgbd/image', '/rgbd/depth_image', '/rgbd/camera_info',
                     '/yolo/detections')
STACK_NODES = {'amcl', 'bt_navigator', 'controller_server', 'planner_server',
               'semantic_search', 'course_detector', 'robot_state_publisher',
               'fortress_depth_cloud_adapter'}


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def yaw_from_quaternion(rotation):
    x, y, z, w = (float(getattr(rotation, name)) for name in ('x', 'y', 'z', 'w'))
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def transform_pose(message):
    translation = message.transform.translation
    x, y = float(translation.x), float(translation.y)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError('Invalid map pose')
    return dict(x=x, y=y, yaw=yaw_from_quaternion(message.transform.rotation),
                frame_id=message.header.frame_id,
                stamp_seconds=stamp_seconds(message.header.stamp))


def map_payload(message):
    info = message.info
    width, height, resolution = int(info.width), int(info.height), float(info.resolution)
    if width <= 0 or height <= 0 or width*height != len(message.data):
        raise ValueError('Map dimensions and data length disagree')
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Map resolution must be positive and finite')
    x, y = float(info.origin.position.x), float(info.origin.position.y)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError('Invalid map origin')
    return dict(available=True, width=width, height=height, resolution=resolution,
                origin=dict(x=x, y=y, yaw=yaw_from_quaternion(info.origin.orientation)),
                data=[int(value) for value in message.data], frame_id=message.header.frame_id,
                stamp_seconds=stamp_seconds(message.header.stamp))


class TelemetryStore:
    """Thread-safe data model also exercised without starting ROS or Gazebo."""
    def __init__(self, now=time.monotonic, freshness_seconds=3.0):
        self.now = now
        self.freshness_seconds = freshness_seconds
        self.lock = threading.RLock()
        self.started_wall = now()
        self.running = False
        self.error = None
        self.topics = {name: dict(messages=0, received=None, stamp=None) for name in TOPICS}
        self.clock_seconds = None
        self.clock_advanced_wall = None
        self.pose = None
        self.pose_received = None
        self.map = None
        self.map_received = None
        self.lifecycle = {}
        self.graph_wall = None
        self.graph_polls = 0
        self.nodes = []
        self.foreign_publishers = {}
        self.command_subscribers = 0

    def receive(self, topic, stamp):
        now = self.now()
        if not math.isfinite(stamp):
            return
        with self.lock:
            entry = self.topics[topic]
            entry.update(messages=entry['messages']+1, received=now, stamp=stamp)
            if topic == '/clock':
                if self.clock_seconds is not None and stamp > self.clock_seconds:
                    self.clock_advanced_wall = now
                elif self.clock_seconds is not None and stamp < self.clock_seconds:
                    self.clock_advanced_wall = None
                self.clock_seconds = stamp

    def receive_pose(self, pose):
        with self.lock:
            self.pose = pose
            self.pose_received = self.now()

    def receive_map(self, payload):
        with self.lock:
            self.map = payload
            self.map_received = self.now()

    def receive_lifecycle(self, name, state_id, label):
        with self.lock:
            self.lifecycle[name] = dict(id=int(state_id), label=label, received=self.now())

    def receive_graph(self, nodes, publishers, subscribers):
        with self.lock:
            self.nodes = sorted(nodes)
            self.foreign_publishers = publishers
            self.command_subscribers = subscribers
            self.graph_wall = self.now()
            self.graph_polls += 1

    def _age(self, when, now):
        return None if when is None else max(0.0, now-when)

    def snapshot(self):
        with self.lock:
            now = self.now()
            topics = {}
            for name, row in self.topics.items():
                age = self._age(row['received'], now)
                stamp_age = (None if self.clock_seconds is None or row['stamp'] is None
                             else self.clock_seconds-row['stamp'])
                topics[name] = dict(age_seconds=age, messages=row['messages'],
                                    stamp_seconds=row['stamp'], stamp_age_seconds=stamp_age,
                                    fresh=(age is not None and age <= self.freshness_seconds
                                           and (name == '/clock' or stamp_age is not None
                                                and -.5 <= stamp_age <= self.freshness_seconds)))
            clock_age = self._age(self.clock_advanced_wall, now)
            clock_advancing = clock_age is not None and clock_age <= self.freshness_seconds
            graph_age = self._age(self.graph_wall, now)
            graph_fresh = graph_age is not None and graph_age <= self.freshness_seconds
            simulation_ready = (self.running and clock_advancing
                                and all(topics[name]['fresh'] for name in SIMULATION_TOPICS))
            pose = deepcopy(self.pose)
            pose_age = self._age(self.pose_received, now)
            pose_stamp_age = (None if pose is None or self.clock_seconds is None
                              else self.clock_seconds-pose['stamp_seconds'])
            pose_fresh = (pose is not None and pose_age is not None and pose_age <= 1.5
                          and pose['frame_id'] == 'map' and pose_stamp_age is not None
                          and -1.5 <= pose_stamp_age <= self.freshness_seconds)
            lifecycle = {}
            for name in ('amcl', 'bt_navigator'):
                row = self.lifecycle.get(name)
                age = self._age(row['received'], now) if row else None
                lifecycle[name] = dict(id=row['id'] if row else None,
                                       label=row['label'] if row else 'unavailable',
                                       age_seconds=age,
                                       active=bool(row and row['id'] == 3 and age <= 5))
            navigation_ready = (simulation_ready and pose_fresh
                                and all(row['active'] for row in lifecycle.values()))
            perception_ready = (simulation_ready and graph_fresh and self.command_subscribers > 0
                                and all(topics[name]['fresh'] for name in PERCEPTION_TOPICS))
            foreign_nodes = [name for name in self.nodes if name.rsplit('/', 1)[-1] in STACK_NODES]
            foreign = bool(foreign_nodes or any(self.foreign_publishers.values())
                           or self.command_subscribers)
            return dict(connected=bool(self.running and graph_fresh
                                       and any(row['fresh'] for row in topics.values())),
                        monitor_running=self.running, error=self.error,
                        discovery_ready=bool(graph_fresh and self.graph_polls >= 2
                                             and now-self.started_wall >= 2),
                        simulation_ready=simulation_ready, navigation_ready=navigation_ready,
                        perception_ready=perception_ready, clock_advancing=clock_advancing,
                        clock_seconds=self.clock_seconds, topics=topics,
                        pose=pose if pose_fresh else None, pose_fresh=pose_fresh,
                        pose_age_seconds=pose_age, pose_stamp_age_seconds=pose_stamp_age,
                        lifecycle=lifecycle, command_subscribers=self.command_subscribers,
                        discovered_nodes=list(self.nodes), foreign_stack=foreign,
                        foreign_nodes=foreign_nodes,
                        foreign_publishers=dict(self.foreign_publishers))

    def map_snapshot(self):
        with self.lock:
            if self.map is None:
                return dict(available=False, age_seconds=None)
            result = deepcopy(self.map)
            result['age_seconds'] = self._age(self.map_received, self.now())
            # A static map is intentionally latched; its age alone is not failure.
            result['source'] = '/map'
            return result


class Monitor:
    def __init__(self):
        self.store = TelemetryStore()
        self._stop = threading.Event()
        self._thread = None
        self._thread_lock = threading.Lock()

    def start(self):
        with self._thread_lock:
            if self._thread is not None:
                raise RuntimeError('Monitor has already started')
            self.store.started_wall = time.monotonic()
            self._thread = threading.Thread(target=self._run, name='robot-telemetry', daemon=True)
            self._thread.start()
        return self

    def snapshot(self):
        return self.store.snapshot()

    def map_snapshot(self):
        return self.store.map_snapshot()

    def foreign_stack_present(self):
        return self.snapshot()['foreign_stack']

    def close(self):
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise RuntimeError('ROS telemetry thread did not stop')

    def _run(self):
        context = node = executor = None
        try:
            import rclpy
            from lifecycle_msgs.srv import GetState
            from nav_msgs.msg import OccupancyGrid, Odometry
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.parameter import Parameter
            from rclpy.qos import DurabilityPolicy, QoSProfile
            from rclpy.signals import SignalHandlerOptions
            from rclpy.time import Time
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
            from semantic_search.sensor_qos import sensor_qos_profile
            from tf2_ros import Buffer, TransformException, TransformListener
            from yolo_msgs.msg import DetectionArray

            context = Context()
            rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = Node('robot_console_monitor_'+str(os.getpid()), context=context,
                        parameter_overrides=[Parameter('use_sim_time', value=True)])
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            reliable = sensor_qos_profile('reliable')
            best_effort = sensor_qos_profile('best_effort')
            subscriptions = []
            for topic, kind in [('/clock', Clock), ('/odom', Odometry), ('/scan', LaserScan),
                                ('/rgbd/obstacles', PointCloud2), ('/rgbd/image', Image),
                                ('/rgbd/depth_image', Image), ('/rgbd/camera_info', CameraInfo),
                                ('/yolo/detections', DetectionArray)]:
                def receive(message, name=topic):
                    stamp = message.clock if name == '/clock' else message.header.stamp
                    self.store.receive(name, stamp_seconds(stamp))
                # The adapter publishes obstacles BEST_EFFORT; all image bridge
                # outputs and detector results in this candidate are RELIABLE.
                qos = best_effort if topic == '/rgbd/obstacles' else reliable
                subscriptions.append(node.create_subscription(kind, topic, receive, qos))

            def receive_map(message):
                try:
                    self.store.receive_map(map_payload(message))
                except ValueError as error:
                    with self.store.lock:
                        self.store.error = 'map: '+str(error)
            subscriptions.append(node.create_subscription(
                OccupancyGrid, '/map', receive_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            services = {name: node.create_client(GetState, '/'+name+'/get_state')
                        for name in ('amcl', 'bt_navigator')}
            pending = {}
            last_graph = last_pose = last_services = -math.inf
            with self.store.lock:
                self.store.running = True
            while not self._stop.is_set() and context.ok():
                executor.spin_once(timeout_sec=.05)
                now = time.monotonic()
                if now-last_graph >= .5:
                    names = [('/'+namespace.strip('/')+'/'+name).replace('//', '/')
                             for name, namespace in node.get_node_names_and_namespaces()
                             if name != node.get_name() and not name.startswith('transform_listener_impl_')]
                    publishers = {topic: node.count_publishers(topic)
                                  for topic in ('/clock', '/odom', '/scan', '/rgbd/image',
                                                '/yolo/detections', '/search/result')}
                    self.store.receive_graph(names, publishers, node.count_subscribers('/search/command'))
                    last_graph = now
                if now-last_pose >= .2:
                    try:
                        self.store.receive_pose(transform_pose(
                            buffer.lookup_transform('map', 'base_link', Time())))
                    except (TransformException, ValueError):
                        pass  # Existing pose expires; stale pose never satisfies readiness.
                    last_pose = now
                for name, (future, sent) in list(pending.items()):
                    if future.done():
                        try:
                            response = future.result().current_state
                            self.store.receive_lifecycle(name, response.id, response.label)
                        except Exception:
                            pass  # State expires if a response fails or a service disappears.
                        del pending[name]
                    elif now-sent >= 2:
                        future.cancel()
                        del pending[name]
                if now-last_services >= 1:
                    for name, client in services.items():
                        if name not in pending and client.service_is_ready():
                            pending[name] = (client.call_async(GetState.Request()), now)
                    last_services = now
        except Exception as error:
            with self.store.lock:
                self.store.error = f'{type(error).__name__}: {error}'
        finally:
            with self.store.lock:
                self.store.running = False
            if executor is not None:
                executor.shutdown(timeout_sec=1)
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.try_shutdown()
