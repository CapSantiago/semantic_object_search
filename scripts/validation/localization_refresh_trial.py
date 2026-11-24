#!/usr/bin/env python3
"""One actual count with stationary AMCL no-motion updates, without pose hints.

This is an explicit diagnostic candidate, not the default robot behavior. The
only localization input requested is another ordinary laser measurement update.
"""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
import urllib.request

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

from target_stability import evaluate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--room', required=True)
    parser.add_argument('--category', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(status='starting', pid=os.getpid(), requests=[], samples=0,
                  method='One UI count; one ordinary AMCL laser update per stationary observation, no initialpose or truth')
    active = 'c041_localization_refresh'

    def api(path, body=None):
        request = urllib.request.Request('http://127.0.0.1:8767'+path,
            data=None if body is None else json.dumps(body).encode(), headers={'Content-Type': 'application/json'})
        return json.load(urllib.request.urlopen(request, timeout=10))

    def save():
        path = args.output/'report.tmp'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        path.replace(args.output/'report.json')

    state = api('/api/state')
    assert state['ready'] and not state['busy'], 'Requires an idle managed console'
    session = Path(state['output_directory'])
    meta = json.loads((session/'session.json').read_text())
    assert str(meta['config']['domain']) == os.environ.get('ROS_DOMAIN_ID')
    report.update(session=str(session), configuration=meta['config'])
    rclpy.init()
    node = Node('stationary_amcl_refresh_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = node.create_client(Empty, '/request_nomotion_update')
    progress = {}
    motion = {}
    localization = {}
    pending = {}
    raw = (args.output/'transforms.jsonl').open('x', buffering=1)

    def odom(message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        v = message.twist.twist
        slow = math.hypot(v.linear.x, v.linear.y) < .01 and abs(v.angular.z) < .03
        since = motion.get('since', stamp) if slow else None
        motion.update(wall=time.monotonic(), stamp=stamp, since=stamp if slow and since is None else since)

    def stage(message):
        value = json.loads(message.data)
        if value.get('request_id') == active:
            progress.clear()
            progress.update(value)

    def amcl(message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        localization.update(stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                            xy=[p.x, p.y], quaternion=[q.x, q.y, q.z, q.w], covariance=list(message.pose.covariance))

    def sample():
        now = time.monotonic()
        row = dict(wall=now, sim_seconds=node.get_clock().now().nanoseconds/1e9,
                   stage=progress.get('stage'), view=progress.get('viewpoint_index'), amcl=dict(localization))
        for key, target, source in [('map_base', 'map', 'base_link'), ('odom_base', 'odom', 'base_link')]:
            try:
                tf = buffer.lookup_transform(target, source, Time())
                p, q = tf.transform.translation, tf.transform.rotation
                row[key] = dict(stamp=tf.header.stamp.sec+tf.header.stamp.nanosec/1e9,
                    xyz=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w])
            except Exception:
                pass
        raw.write(json.dumps(row)+'\n')
        report['samples'] += 1
        view = progress.get('viewpoint_index')
        if (progress.get('stage') != 'observing' or view in pending or not motion
                or now-motion['wall'] > .4 or motion['since'] is None
                or motion['stamp']-motion['since'] < .45):
            return
        event = dict(view=view, requested_sim_seconds=row['sim_seconds'], before_amcl=dict(localization))
        report['requests'].append(event)
        pending[view] = client.call_async(Empty.Request())

        def acknowledged(future):
            try:
                future.result()
                event.update(acknowledged=True, acknowledged_sim_seconds=node.get_clock().now().nanoseconds/1e9)
            except Exception as error:
                event.update(acknowledged=False, error=str(error))
        pending[view].add_done_callback(acknowledged)

    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(String, '/search/progress', stage, 10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl, qos_profile_sensor_data)
    timer = node.create_timer(.1, sample)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    sent = False
    try:
        assert client.wait_for_service(timeout_sec=15), 'AMCL no-motion update unavailable'
        report['status'] = 'running'
        save()
        api('/api/task', dict(request_id=active, mode='count', room=args.room, category=args.category))
        sent = True
        until = time.monotonic()+650
        while time.monotonic() < until:
            state = api('/api/state')
            assert state['output_directory'] == str(session)
            task = state.get('task') or {}
            assert task.get('id') == active
            report['last_task'] = task
            save()
            if task['state'] in ('completed', 'partial', 'failed', 'canceled'):
                sent = False
                result = evaluate(task, dict(room=args.room, category=args.category, position_limit_m=.20), session, meta)
                report.update(status='finished', evaluation=result,
                              all_refreshes_acknowledged=bool(report['requests']) and
                                  all(x.get('acknowledged') is True for x in report['requests']))
                break
            time.sleep(.3)
        else:
            raise TimeoutError('Localization diagnostic watchdog')
    except BaseException as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
        if sent:
            try:
                state = api('/api/state')
                if state['busy'] and (state.get('task') or {}).get('id') == active:
                    api('/api/cancel', {})
            except Exception:
                pass
        raise
    finally:
        executor.shutdown(timeout_sec=5)
        worker.join(timeout=5)
        raw.close()
        save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
