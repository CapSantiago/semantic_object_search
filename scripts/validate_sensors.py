#!/usr/bin/env python3
"""Inspect real simulator messages and TF; optionally verify a short base motion."""
import argparse
import json
import math
from pathlib import Path
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener
from semantic_search.sensor_qos import sensor_qos_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='results/validation')
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--motion', action='store_true')
    parser.add_argument('--sensor-reliability', choices=('best_effort', 'reliable'), default='best_effort')
    args = parser.parse_args()
    sensor_qos = sensor_qos_profile(args.sensor_reliability)
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('validate_sensors')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    messages, counts, stamps = {}, {}, {}
    def receive(topic, msg):
        messages[topic] = msg
        counts[topic] = counts.get(topic, 0) + 1
        stamp = msg.clock if topic == '/clock' else msg.header.stamp
        stamps.setdefault(topic, []).append(stamp.sec + stamp.nanosec / 1e9)
    subscriptions = []
    for topic, kind in [('/clock', Clock), ('/odom', Odometry), ('/scan', LaserScan),
                        ('/rgbd/image', Image), ('/rgbd/depth_image', Image),
                        ('/rgbd/camera_info', CameraInfo)]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, t=topic: receive(t, msg), sensor_qos))
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    report = {'sensor_reliability': args.sensor_reliability,
              'topics': {t: {'messages': n, 'sim_hz': (n-1)/(stamps[t][-1]-stamps[t][0])
                            if n > 1 and stamps[t][-1] > stamps[t][0] else 0}
                         for t, n in counts.items()}, 'tf': {}}
    bridge = CvBridge()
    import cv2
    for topic in ('/rgbd/image', '/rgbd/depth_image'):
        if topic not in messages:
            continue
        msg = messages[topic]
        data = bridge.imgmsg_to_cv2(msg, 'passthrough')
        report['topics'][topic].update(frame=msg.header.frame_id, encoding=msg.encoding,
                                      width=msg.width, height=msg.height)
        if topic.endswith('depth_image'):
            valid = data[np.isfinite(data) & (data > 0)]
            report['topics'][topic].update(valid_fraction=float(valid.size/data.size),
                minimum=float(valid.min()) if valid.size else None,
                maximum=float(valid.max()) if valid.size else None)
            np.save(folder / 'depth.npy', data)
        else:
            cv2.imwrite(str(folder / 'rgb.png'), bridge.imgmsg_to_cv2(msg, 'bgr8'))
    if '/scan' in messages:
        scan = messages['/scan']
        valid = [x for x in scan.ranges if math.isfinite(x)]
        report['topics']['/scan'].update(frame=scan.header.frame_id,
            samples=len(scan.ranges), minimum=min(valid) if valid else None,
            maximum=max(valid) if valid else None)
    if '/rgbd/camera_info' in messages:
        info = messages['/rgbd/camera_info']
        report['topics']['/rgbd/camera_info'].update(frame=info.header.frame_id, k=list(info.k))
    for parent, child in [('odom', 'base_footprint'), ('base_footprint', 'base_scan'),
                          ('base_link', 'camera_optical_frame')]:
        try:
            tf = buffer.lookup_transform(parent, child, Time()).transform
            report['tf'][f'{parent}->{child}'] = {
                'xyz': [tf.translation.x, tf.translation.y, tf.translation.z],
                'xyzw': [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]}
        except Exception as error:
            report['tf'][f'{parent}->{child}'] = {'error': str(error)}
    if args.motion and '/odom' in messages and '/scan' in messages:
        pub = node.create_publisher(Twist, '/cmd_vel', 10)
        before = messages['/odom'].pose.pose.position
        command = Twist()
        command.linear.x = 0.1
        start = time.monotonic()
        try:
            while time.monotonic() - start < 3:
                scan = messages['/scan']
                front = [r for i, r in enumerate(scan.ranges)
                         if math.cos(scan.angle_min+i*scan.angle_increment) > 0.9
                         and math.isfinite(r)]
                if front and min(front) < 0.4:
                    break
                pub.publish(command)
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            for _ in range(5):
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
        after = messages['/odom'].pose.pose.position
        report['motion_distance_m'] = math.hypot(after.x-before.x, after.y-before.y)
    report['passed'] = (len(counts) == 6
        and all(n > 1 for n in counts.values())
        and all('error' not in tf for tf in report['tf'].values())
        and report['topics'].get('/rgbd/depth_image', {}).get('valid_fraction', 0) > 0
        and (not args.motion or report.get('motion_distance_m', 0) > 0.05))
    (folder / 'sensors.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
