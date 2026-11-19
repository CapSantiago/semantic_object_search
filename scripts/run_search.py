#!/usr/bin/env python3
"""Send one text/JSON task and save its actual /search/result message."""
import argparse
import json
import math
from pathlib import Path
import signal
import time


def save_json(path, value):
    if path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command')
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--output', default='results/validation/task_result.json')
    parser.add_argument('--initial-state-output')
    parser.add_argument('--diagnostics-output')
    parser.add_argument('--pose-timeout', type=float, default=10)
    parser.add_argument('--discovery-timeout', type=float, default=15,
                        help='maximum command-subscriber wait after the pose/warmup preamble')
    parser.add_argument('--cancel-wait', type=float, default=5)
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--result-topic', default='/search/result')
    parser.add_argument('--command-topic', default='/search/command')
    parser.add_argument('--exclude-task-id', action='append', default=[],
                        help='previous task ID to ignore, including delayed live messages; repeatable')
    args = parser.parse_args()
    cancel_command = '取消'
    if args.command.lstrip().startswith('{'):
        try:
            request = json.loads(args.command)
        except ValueError:
            request = None
        if isinstance(request, dict) and isinstance(request.get('request_id'), str):
            cancel_command = json.dumps(dict(mode='cancel', request_id=request['request_id']))
    if (not all(math.isfinite(value) for value in
                (args.timeout, args.discovery_timeout, args.pose_timeout, args.cancel_wait))
            or args.timeout <= 0 or args.discovery_timeout < 0 or args.pose_timeout < 0 or args.cancel_wait < 0):
        parser.error('timeouts must be finite; task timeout positive, other waits nonnegative')

    # Lazy imports keep --help and offline matrix validation independent of ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from std_msgs.msg import String

    diagnostic = dict(command=args.command, result_topic=args.result_topic, command_topic=args.command_topic,
                      result_qos_durability='volatile', excluded_task_ids=sorted(set(args.exclude_task_id)),
                      ignored_task_ids=[], pre_command_task_ids=[],
                      discovery_timeout_seconds=args.discovery_timeout,
                      discovery_wait_wall_seconds=0.0, command_subscribers_before_publish=0,
                      command_sent=False, cancellation_sent=False, cancellation_observed=False,
                      outcome='starting', errors=[])
    stopped = []
    previous_handlers = {}
    node = None
    result = None
    started = time.monotonic()
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[sig] = signal.signal(sig, lambda signum, frame: stopped.append(signum))
        node = Node('search_acceptance_client')
        received, old_ids = [], set(args.exclude_task_id)

        def receive(message):
            try:
                row = json.loads(message.data)
                if not isinstance(row, dict):
                    raise ValueError('result is not an object')
                if row.get('task_id') in old_ids:
                    if row['task_id'] not in diagnostic['ignored_task_ids']:
                        diagnostic['ignored_task_ids'].append(row['task_id'])
                    return
                received.append(row)
            except (ValueError, TypeError) as error:
                diagnostic['errors'].append(str(error))

        def next_result():
            return next((row for row in received if row.get('task_id')
                         and row['task_id'] not in old_ids and row.get('command') == args.command), None)

        # Each client wants a future result. Retained history may arrive after the
        # discovery preamble and otherwise masquerade as a repeated text task.
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(String, args.result_topic, receive, qos)
        pub = node.create_publisher(String, args.command_topic, 10)
        odometry, transform, transform_error = [], None, None
        if args.initial_state_output:
            from nav_msgs.msg import Odometry
            from rosidl_runtime_py.convert import message_to_ordereddict
            from tf2_ros import Buffer, TransformListener, TransformException
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            node.create_subscription(Odometry, '/odom', lambda msg: odometry.__setitem__(slice(None), [msg]),
                                     qos_profile_sensor_data)
        start = time.monotonic()
        while not stopped and time.monotonic()-start < max(2, args.pose_timeout if args.initial_state_output else 2):
            rclpy.spin_once(node, timeout_sec=.1)
            if args.initial_state_output:
                try:
                    transform = buffer.lookup_transform(args.map_frame, args.base_frame, Time())
                    transform_error = None
                except TransformException as error:
                    transform_error = str(error)
            if time.monotonic()-start >= 2 and (not args.initial_state_output or (transform and odometry)):
                break
        # DDS matching can finish after the two-second warmup. This budget is
        # independent of the task timeout, which starts only after publication.
        discovery_start = time.monotonic()
        def serving_nodes():
            # A recorder can subscribe to commands without executing them.
            # Require the same discovered node to expose both protocol ends.
            def identities(endpoints):
                return {(item.node_namespace,item.node_name) for item in endpoints
                        if item.node_name != '_NODE_NAME_UNKNOWN_'}
            return sorted(identities(node.get_subscriptions_info_by_topic(args.command_topic)) &
                          identities(node.get_publishers_info_by_topic(args.result_topic)))
        def discovered():
            return bool(pub.get_subscription_count() and node.count_publishers(args.result_topic)
                        and serving_nodes())
        while not stopped and not discovered() and time.monotonic()-discovery_start < args.discovery_timeout:
            rclpy.spin_once(node, timeout_sec=.1)
        diagnostic['discovery_wait_wall_seconds'] = time.monotonic()-discovery_start
        diagnostic['command_subscribers_before_publish'] = pub.get_subscription_count()
        diagnostic['serving_nodes_before_publish'] = serving_nodes()
        diagnostic['result_publishers_before_publish'] = node.count_publishers(args.result_topic)
        diagnostic['pre_command_task_ids'] = sorted({row['task_id'] for row in received if row.get('task_id')})
        old_ids.update(diagnostic['pre_command_task_ids'])
        received.clear()
        if args.initial_state_output:
            # Use the latest pose after any discovery wait, rather than saving
            # the preamble's earlier transform as the command's initial pose.
            try:
                transform = buffer.lookup_transform(args.map_frame, args.base_frame, Time())
                transform_error = None
            except TransformException as error:
                transform, transform_error = None, str(error)
            save_json(args.initial_state_output, dict(
                captured_unix_seconds=time.time(), source='observed_ros_messages_before_command',
                map_pose=dict(source='tf', target_frame=args.map_frame, source_frame=args.base_frame,
                              available=transform is not None,
                              transform=message_to_ordereddict(transform) if transform else None,
                              error=transform_error),
                odom=dict(source='/odom', available=bool(odometry),
                          message=message_to_ordereddict(odometry[-1]) if odometry else None,
                          error=None if odometry else 'No odometry received before command'),
                limitation='Latest received TF and odometry; timestamps retained, freshness not certified.'))
        if stopped:
            diagnostic['outcome'] = 'interrupted_before_command'
            return 130
        if not discovered():
            diagnostic['outcome'] = 'discovery_timeout'
            diagnostic['errors'].append(f'No matching command/result search node within {args.discovery_timeout:g}s after preamble')
            print(json.dumps(diagnostic, ensure_ascii=False))
            return 2
        pub.publish(String(data=args.command))
        diagnostic['command_sent'] = True
        diagnostic['command_sent_unix_seconds'] = time.time()
        start = time.monotonic()
        while not stopped and time.monotonic()-start < args.timeout:
            rclpy.spin_once(node, timeout_sec=.1)
            result = next_result()
            if result is not None:
                break
        diagnostic['outcome'] = 'result_received' if result else ('interrupted' if stopped else 'timeout')
        if result is None:
            pub.publish(String(data=cancel_command))
            diagnostic['cancellation_sent'] = True
            deadline = time.monotonic() + args.cancel_wait
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                result = next_result()
                if result is not None:
                    diagnostic['cancellation_observed'] = result.get('status') == 'canceled'
                    break
        if result is not None:
            save_json(args.output, result)
            print(json.dumps(result, ensure_ascii=False, indent=2))
        if diagnostic['outcome'] == 'timeout':
            return 124
        if stopped:
            return 130
        return 0 if result['status'] in ('found', 'completed') else 1
    except Exception as error:
        diagnostic['outcome'] = 'client_error'
        diagnostic['errors'].append(f'{type(error).__name__}: {error}')
        # A client-side failure after publishing still requires cancellation.
        if node is not None and diagnostic['command_sent'] and result is None:
            try:
                pub.publish(String(data=cancel_command))
                diagnostic['cancellation_sent'] = True
                rclpy.spin_once(node, timeout_sec=.2)
            except Exception as cancel_error:
                diagnostic['errors'].append(f'cancel: {cancel_error}')
        print(json.dumps(diagnostic, ensure_ascii=False))
        return 2
    finally:
        diagnostic['elapsed_client_wall_seconds'] = time.monotonic()-started
        diagnostic['signals'] = stopped
        try:
            save_json(args.diagnostics_output, diagnostic)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
