#!/usr/bin/env python3
"""Capture a synchronized RGB/depth/intrinsics frame and its map transform."""
import argparse
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from yolo_msgs.msg import DetectionArray
from semantic_search.sensor_qos import sensor_qos_profile
from semantic_search.objects import OBJECTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--fixed-frame', default='map', help='Use odom for sensor-only calibration without AMCL')
    parser.add_argument('--skip-frames', type=int, default=0, help='Discard initial synchronized frames after a pose change')
    parser.add_argument('--category', choices=list(OBJECTS),
                        help='Wait for an online detection and include its synchronized RGB-D frame')
    parser.add_argument('--min-instances', type=int, default=1,
                        help='Minimum simultaneous detections when --category is used')
    parser.add_argument('--timeout', type=float, default=20)
    parser.add_argument('--sensor-reliability', choices=['best_effort','reliable'], default='best_effort')
    args = parser.parse_args()
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('capture_rgbd')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    bridge = CvBridge()
    done = []
    received = [0]
    def capture(rgb, depth, info, detections=None):
        received[0] += 1
        if received[0] <= args.skip_frames:
            return
        targets = [d for d in detections.detections if d.class_name == args.category and d.score >= .35] if detections else []
        if args.category and len(targets) < args.min_instances:
            return
        try:
            tf = buffer.lookup_transform(args.fixed_frame, info.header.frame_id, Time.from_msg(rgb.header.stamp)).transform
        except Exception:
            return
        cv2.imwrite(str(folder/'rgb.png'), bridge.imgmsg_to_cv2(rgb, 'bgr8'))
        np.save(folder/'depth.npy', bridge.imgmsg_to_cv2(depth, 'passthrough'))
        (folder/'frame.json').write_text(json.dumps(dict(
            stamp_ns=Time.from_msg(rgb.header.stamp).nanoseconds,
            frame=info.header.frame_id, k=list(info.k), depth_encoding=depth.encoding,
            matched_detections=[dict(category=d.class_name,score=d.score,
                xywh=[d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.size.x,d.bbox.size.y])
                for d in targets],
            fixed_frame=args.fixed_frame,
            transform=dict(xyz=[tf.translation.x, tf.translation.y, tf.translation.z],
                xyzw=[tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])), indent=2)+'\n')
        done.append(True)
    sensor_qos = sensor_qos_profile(args.sensor_reliability)
    subs = [Subscriber(node, kind, topic, qos_profile=sensor_qos)
            for kind, topic in [(Image, '/rgbd/image'), (Image, '/rgbd/depth_image'),
                                (CameraInfo, '/rgbd/camera_info')]]
    if args.category:
        subs.append(Subscriber(node, DetectionArray, '/yolo/detections', qos_profile=sensor_qos))
    sync = ApproximateTimeSynchronizer(subs, queue_size=30, slop=.075)
    sync.registerCallback(capture)
    started = time.monotonic()
    while not done and time.monotonic()-started < args.timeout:
        rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()
    if not done:
        raise TimeoutError('No matching synchronized frame with requested TF/category')
    print(folder)


if __name__ == '__main__':
    main()
