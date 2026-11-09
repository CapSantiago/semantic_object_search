#!/usr/bin/env python3
"""Drive a known, static simulation route with odometry feedback for SLAM."""
import argparse
import json
from math import atan2, cos, hypot, isfinite, sin
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', default='scripts/mapping_route.json')
    parser.add_argument('--output', default='results/validation/mapping_drive.json')
    args = parser.parse_args()
    route = json.loads(Path(args.route).read_text())
    rclpy.init()
    node = Node('mapping_drive')
    state = {}
    def odom(msg):
        state['odom'] = msg
    def scan(msg):
        state['scan'] = msg
    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    visited = []
    started = time.monotonic()
    try:
        while not all(t in state for t in ('scan', 'odom')):
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() - started > 15:
                raise RuntimeError('No odometry or scan')
        for target in route:
            deadline = time.monotonic() + 90
            while True:
                rclpy.spin_once(node, timeout_sec=.05)
                pose = state['odom'].pose.pose
                dx, dy = target[0]-pose.position.x, target[1]-pose.position.y
                q = pose.orientation
                yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                distance = hypot(dx, dy)
                error = atan2(sin(atan2(dy, dx)-yaw), cos(atan2(dy, dx)-yaw))
                if distance < .09:
                    if len(target) < 3:
                        break
                    error = atan2(sin(target[2]-yaw), cos(target[2]-yaw))
                    if abs(error) < .08:
                        break
                cmd = Twist()
                cmd.angular.z = max(-.7, min(.7, 1.6*error))
                if abs(error) < .25 and distance >= .09:
                    scan_msg = state['scan']
                    forward = [r for i, r in enumerate(scan_msg.ranges)
                               if cos(scan_msg.angle_min+i*scan_msg.angle_increment) > .94
                               and isfinite(r)]
                    if forward and min(forward) < .30:
                        raise RuntimeError(f'Obstacle before waypoint {target}: {min(forward):.3f} m')
                    cmd.linear.x = min(.26, .7*distance)
                pub.publish(cmd)
                if time.monotonic() > deadline:
                    raise RuntimeError(f'Waypoint timeout: {target}')
            pub.publish(Twist())
            visited.append(target)
            print(f'Reached {target}', flush=True)
        status = 'completed'
    except Exception as error:
        status = str(error)
        raise
    finally:
        for _ in range(5):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=.05)
        Path(args.output).write_text(json.dumps(dict(status=status, visited=visited,
            wall_seconds=time.monotonic()-started), indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
