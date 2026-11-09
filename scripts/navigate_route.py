#!/usr/bin/env python3
"""Visit explicit map-frame poses through Nav2, recording actual action results."""
import argparse
import json
from math import atan2, cos, sin
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--route')
    inputs.add_argument('--pose', nargs=3, type=float)
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=180.)
    args = parser.parse_args()
    route = json.loads(Path(args.route).read_text()) if args.route else [args.pose]
    rclpy.init()
    node = Node('navigate_validation_route', parameter_overrides=[Parameter('use_sim_time', value=True)])
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    report = dict(route=route, visited=[], status='running')
    active = None
    started = time.monotonic()

    def wait(future, seconds):
        end = time.monotonic() + seconds
        while not future.done():
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= end:
                raise TimeoutError(f'Nav2 response exceeded {seconds} seconds')
        return future.result()

    try:
        if not client.wait_for_server(timeout_sec=45.):
            raise RuntimeError('NavigateToPose is not ready')
        # Allow request/response endpoint discovery to settle on Humble/Fast DDS
        # before the first goal from this short-lived validation client.
        discovery_deadline = time.monotonic() + 2.
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        previous = (0., 0.)
        for index, target in enumerate(route):
            yaw = target[2] if len(target) > 2 else atan2(target[1]-previous[1], target[0]-previous[0])
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, target[:2])
            goal.pose.pose.orientation.z = sin(yaw/2)
            goal.pose.pose.orientation.w = cos(yaw/2)
            print(f'Goal {index+1}/{len(route)}: {target}', flush=True)
            active = wait(client.send_goal_async(goal), 15.)
            if not active.accepted:
                raise RuntimeError(f'Nav2 rejected {target}')
            result = wait(active.get_result_async(), args.timeout)
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Nav2 status {result.status} at {target}')
            report['visited'].append(target)
            previous = target[:2]
            active = None
            print(f'Reached {target}', flush=True)
        report['status'] = 'completed'
    except BaseException as error:
        report['status'] = f'{type(error).__name__}: {error}'
        if active is not None:
            try:
                wait(active.cancel_goal_async(), 5.)
            except Exception:
                pass
        raise
    finally:
        report['wall_seconds'] = time.monotonic()-started
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
