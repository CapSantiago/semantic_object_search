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
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    args = parser.parse_args()
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('capture_rgbd')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    bridge = CvBridge()
    done = []
    def capture(rgb, depth, info):
        try:
            tf = buffer.lookup_transform('map', info.header.frame_id, Time.from_msg(rgb.header.stamp)).transform
        except Exception:
            return
        cv2.imwrite(str(folder/'rgb.png'), bridge.imgmsg_to_cv2(rgb, 'bgr8'))
        np.save(folder/'depth.npy', bridge.imgmsg_to_cv2(depth, 'passthrough'))
        (folder/'frame.json').write_text(json.dumps(dict(
            stamp_ns=Time.from_msg(rgb.header.stamp).nanoseconds,
            frame=info.header.frame_id, k=list(info.k), depth_encoding=depth.encoding,
            transform=dict(xyz=[tf.translation.x, tf.translation.y, tf.translation.z],
                xyzw=[tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])), indent=2)+'\n')
        done.append(True)
    subs = [Subscriber(node, kind, topic, qos_profile=qos_profile_sensor_data)
            for kind, topic in [(Image, '/rgbd/image'), (Image, '/rgbd/depth_image'),
                                (CameraInfo, '/rgbd/camera_info')]]
    sync = ApproximateTimeSynchronizer(subs, queue_size=30, slop=.075)
    sync.registerCallback(capture)
    started = time.monotonic()
    while not done and time.monotonic()-started < 20:
        rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()
    if not done:
        raise TimeoutError('No synchronized frame with map TF')
    print(folder)


if __name__ == '__main__':
    main()
