#!/usr/bin/env python3
"""Start non-autostart Nav2 after DDS discovery, initializing aligned simulation AMCL."""
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('start_development_navigation')
    clients = [node.create_client(ManageLifecycleNodes, '/lifecycle_manager_'+name+'/manage_nodes')
               for name in ('localization','navigation')]
    received = set()
    subscriptions = [node.create_subscription(kind, topic,
        lambda message, name=topic: received.add(name), qos_profile_sensor_data)
        for topic, kind in [('/clock', Clock), ('/odom', Odometry),
                            ('/scan', LaserScan), ('/rgbd/obstacles', PointCloud2)]]
    try:
        deadline = time.monotonic()+30
        while len(received) < len(subscriptions) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if len(received) < len(subscriptions):
            missing = {'/clock','/odom','/scan','/rgbd/obstacles'}-received
            raise RuntimeError('Simulator bridge data unavailable before Nav2 startup: '+
                               ', '.join(sorted(missing)))
        for client in clients:
            if not client.wait_for_service(timeout_sec=30):
                raise TimeoutError('Nav2 lifecycle manager unavailable')
        # Both servers and every managed node have started before the first transition.
        deadline = time.monotonic()+4
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        request = ManageLifecycleNodes.Request(); request.command=0
        local = clients[0].call_async(request)
        deadline = time.monotonic()+45
        while not local.done() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        if not local.done() or not local.result().success:
            raise RuntimeError('Localization startup failed')
        nav = clients[1].call_async(request)
        # Initial pose is needed while Nav2 activates costmaps. The existing helper
        # waits for bt_navigator active before returning, so startup cannot race tasks.
        subprocess.run([sys.executable,str(Path(__file__).with_name('initialize_sim_amcl.py'))],
                       check=True,timeout=90)
        deadline=time.monotonic()+30
        while not nav.done() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        if not nav.done() or not nav.result().success:
            raise RuntimeError('Navigation startup failed')
        print('Localization and navigation active after staged DDS startup',flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
