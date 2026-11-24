#!/usr/bin/env python3
"""Initialize AMCL from simulator odometry (valid for this aligned course map)."""
from collections import deque
from math import atan2, cos, sin
import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from semantic_search.docking import load_station, world_pose_to_map


def fresh_odometry(message,received_wall,now_wall,sim_seconds):
    p=message.pose.pose.position;q=message.pose.pose.orientation
    stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
    return (message.header.frame_id=='odom' and 0<=now_wall-received_wall<=.4 and
        stamp>0 and -.1<=sim_seconds-stamp<=.4 and
        all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) and
        .99<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.01)


def wait_until_active(node, client, ready, timeout, error):
    """Retry a lost DDS reply within the original startup deadline."""
    deadline = time.monotonic()+timeout
    request = None
    retry_at = 0.
    active = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node,timeout_sec=.10)
        now = time.monotonic()
        if request is not None and request.done():
            response = request.result()
            active = response is not None and response.current_state.id==3
            request = None
        if active and ready():
            return
        if not active and now>=retry_at and client.service_is_ready():
            if request is not None:
                client.remove_pending_request(request)
                request.cancel()
            request = client.call_async(GetState.Request())
            retry_at = now+1.5
    raise RuntimeError(error)


def main():
    rclpy.init()
    node = Node('initialize_sim_amcl',parameter_overrides=[Parameter('use_sim_time',value=True)])
    state = deque(maxlen=1)
    node.create_subscription(Odometry, '/odom', lambda m: state.append((m,time.monotonic())), qos_profile_sensor_data)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    client = node.create_client(GetState, '/amcl/get_state')
    wait_until_active(node,client,lambda:state and pub.get_subscription_count() and
                      fresh_odometry(state[-1][0],state[-1][1],time.monotonic(),node.get_clock().now().nanoseconds/1e9),20,
                      'Odometry or active AMCL unavailable')
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = state[-1][0].header.stamp
    pose = state[-1][0].pose.pose
    q = pose.orientation
    yaw = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    position=(pose.position.x,pose.position.y,yaw)
    station_file=os.environ.get('SEMANTIC_CHARGING_CONFIG')
    if station_file:
        station=load_station(station_file,Path(__file__).resolve().parents[1])
        position=world_pose_to_map(station,position)
    msg.pose.pose.position.x,msg.pose.pose.position.y,yaw=position
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = sin(yaw/2), cos(yaw/2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = .01
    for _ in range(4):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.1)
    print(f'AMCL initialized: {position[0]:.3f}, {position[1]:.3f}, {yaw:.3f}')
    # AMCL can be active while Nav2 is still activating its costmaps and BT.
    # Returning before this point lets the first search goal be rejected.
    navigator = node.create_client(GetState, '/bt_navigator/get_state')
    wait_until_active(node,navigator,lambda:True,45,
                      'AMCL initialized but Nav2 navigator did not become active')
    print('Nav2 navigator active; ready for search commands')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
