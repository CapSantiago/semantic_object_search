#!/usr/bin/env python3
"""Read-only TF/odometry/velocity trace for arrival and recovery diagnostics."""
import argparse
import json
from math import atan2
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(q):
    return atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=600)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('record_navigation_state',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(); listener = TransformListener(buffer, node)
    latest = {}; samples = []; failures = 0
    def odom(message):
        p = message.pose.pose
        latest['odom'] = dict(stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                             pose=[p.position.x,p.position.y,yaw(p.orientation)],
                             velocity=[message.twist.twist.linear.x,message.twist.twist.angular.z])
    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', lambda m:latest.update(
        cmd_vel=dict(received_wall=time.monotonic(),velocity=[m.linear.x,m.angular.z])),10)
    start = time.monotonic(); next_sample = start
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if now < next_sample: continue
            next_sample = now+.2
            row = dict(wall_elapsed=now-start,sim_seconds=node.get_clock().now().nanoseconds/1e9,
                       **latest)
            try:
                tf = buffer.lookup_transform('map','base_link',Time())
                t = tf.transform
                row['map_pose'] = [t.translation.x,t.translation.y,yaw(t.rotation)]
                row['tf_stamp'] = tf.header.stamp.sec+tf.header.stamp.nanosec/1e9
            except TransformException:
                failures += 1
            samples.append(row)
    except KeyboardInterrupt:
        pass
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(method='Measured latest map TF plus odometry; no simulator truth',
            sample_period_wall_seconds=.2,tf_lookup_failures=failures,samples=samples),indent=2)+'\n')
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__': main()
