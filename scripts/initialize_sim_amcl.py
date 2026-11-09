#!/usr/bin/env python3
"""Initialize AMCL from simulator odometry (valid for this aligned course map)."""
from math import atan2, cos, sin
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState


def main():
    rclpy.init()
    node = Node('initialize_sim_amcl')
    state = []
    node.create_subscription(Odometry, '/odom', lambda m: state.append(m), qos_profile_sensor_data)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    client = node.create_client(GetState, '/amcl/get_state')
    started = time.monotonic()
    active = False
    pending = None
    while not state or not pub.get_subscription_count() or not active:
        rclpy.spin_once(node, timeout_sec=.1)
        if client.service_is_ready() and pending is None:
            pending = client.call_async(GetState.Request())
        if pending is not None and pending.done():
            active = pending.result().current_state.id == 3
            pending = None
        if time.monotonic()-started > 20:
            raise RuntimeError('Odometry or active AMCL unavailable')
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = state[-1].header.stamp
    pose = state[-1].pose.pose
    msg.pose.pose.position.x, msg.pose.pose.position.y = pose.position.x, pose.position.y
    q = pose.orientation
    yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = sin(yaw/2), cos(yaw/2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = .01
    for _ in range(4):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.1)
    print(f'AMCL initialized: {pose.position.x:.3f}, {pose.position.y:.3f}, {yaw:.3f}')
    # AMCL can be active while Nav2 is still activating its costmaps and BT.
    # Returning before this point lets the first search goal be rejected.
    navigator = node.create_client(GetState, '/bt_navigator/get_state')
    pending = None
    deadline = time.monotonic() + 45
    while True:
        rclpy.spin_once(node, timeout_sec=.1)
        if navigator.service_is_ready() and pending is None:
            pending = navigator.call_async(GetState.Request())
        if pending is not None and pending.done():
            if pending.result().current_state.id == 3:
                break
            pending = None
        if time.monotonic() > deadline:
            raise RuntimeError('AMCL initialized but Nav2 navigator did not become active')
    print('Nav2 navigator active; ready for search commands')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
