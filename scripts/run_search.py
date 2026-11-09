#!/usr/bin/env python3
"""Send one text task and wait for its new, persisted search result."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command')
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--output', default='results/validation/task_result.json')
    args = parser.parse_args()
    rclpy.init()
    node = Node('search_acceptance_client')
    received, old_ids = [], set()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/search/result', lambda m: received.append(json.loads(m.data)), qos)
    pub = node.create_publisher(String, '/search/command', 10)
    start = time.monotonic()
    while time.monotonic()-start < 2:
        rclpy.spin_once(node, timeout_sec=.1)
    old_ids.update(row.get('task_id') for row in received)
    received.clear()
    if not pub.get_subscription_count():
        raise RuntimeError('No search command subscriber')
    pub.publish(String(data=args.command))
    start = time.monotonic()
    result = None
    try:
        while time.monotonic()-start < args.timeout:
            rclpy.spin_once(node, timeout_sec=.1)
            for row in received:
                if row.get('task_id') not in old_ids and row.get('command') == args.command:
                    result = row
                    break
            if result is not None:
                break
        if result is None:
            pub.publish(String(data='取消'))
            raise TimeoutError('Search timed out; cancellation sent')
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result['status'] in ('found', 'completed') else 1)


if __name__ == '__main__':
    main()
