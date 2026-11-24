#!/usr/bin/env python3
"""Read-only DWB decisions and critic scores for bounded navigation diagnosis.

No commands, parameter changes or simulator truth. Short-circuited trajectory
scores are explicitly marked incomplete and must not be compared as full costs.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from dwb_msgs.msg import LocalPlanEvaluation
from nav_msgs.msg import Path as NavPath
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=180.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 1800:
        parser.error('Use a bounded duration of at most 1800 seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    stopped = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *unused: stopped.append(True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('dwb_score_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    plans = {}
    report = dict(status='running', pid=os.getpid(), samples=0, stopped_choices=0,
                  no_legal_choice_samples=0,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  method='Actual published DWB evaluation, at most 5 Hz; best and positive alternatives',
                  limitations=['Controller scoring only; not physical contact or room coverage acceptance.',
                               'Partial costs from short-circuit scoring are not complete candidate costs.',
                               'Latest TF is diagnostic context, not guaranteed evaluation-stamp synchronization.'])
    raw = (args.output/'scores.jsonl').open('x', buffering=1)
    last_sample = -math.inf

    def plan(message, key):
        if message.poses:
            p = message.poses[-1].pose.position
            plans[key] = dict(frame=message.header.frame_id, endpoint=[p.x, p.y],
                              stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                              poses=len(message.poses))

    def evaluation(message):
        nonlocal last_sample
        now = time.monotonic()
        if now-last_sample < .2 or not message.twists or message.best_index >= len(message.twists):
            return
        last_sample = now
        best = message.twists[message.best_index]
        chosen_valid = best.total >= 0 and all(s.raw_score >= 0 for s in best.scores)
        # With no legal trajectory, upstream leaves best_index at zero and
        # publishes an illegal diagnostic sample there; it selected no command.
        expected = {s.name for s in best.scores} if chosen_valid else set()

        def pack(score):
            v = score.traj.velocity
            poses = score.traj.poses
            return dict(velocity=[v.x, v.y, v.theta], total=score.total,
                        full_critic_set=bool(expected) and {s.name for s in score.scores} == expected,
                        nonnegative_costs=all(s.raw_score >= 0 for s in score.scores),
                        endpoint=[poses[-1].x, poses[-1].y, poses[-1].theta] if poses else None,
                        critics=[dict(name=s.name, raw=s.raw_score, scale=s.scale,
                                      weighted=s.raw_score*s.scale) for s in score.scores])

        positive = [s for s in message.twists if s.traj.velocity.x > .001 and s.total >= 0]
        full = [s for s in positive if {c.name for c in s.scores} == expected
                and all(c.raw_score >= 0 for c in s.scores)]
        alternatives = sorted(full, key=lambda s: s.total)[:2]
        if not alternatives:
            alternatives = sorted(positive, key=lambda s: s.total)[:2]
        row = dict(wall=now, stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                   frame=message.header.frame_id, evaluated=len(message.twists),
                   chosen_valid=chosen_valid,
                   best=pack(best), positive_alternatives=[pack(s) for s in alternatives],
                   plans=dict(plans))
        try:
            tf = buffer.lookup_transform('map', 'base_link', Time())
            p = tf.transform.translation
            row['latest_map_xy'] = [p.x, p.y]
            endpoint = plans.get('received', {}).get('endpoint')
            if endpoint:
                row['distance_to_received_endpoint_m'] = math.dist([p.x, p.y], endpoint)
        except Exception as error:
            row['tf_error'] = type(error).__name__
        raw.write(json.dumps(row, allow_nan=False)+'\n')
        report['samples'] += 1
        if not chosen_valid:
            report['no_legal_choice_samples'] += 1
        elif abs(best.traj.velocity.x) < .001 and abs(best.traj.velocity.theta) < .03:
            report['stopped_choices'] += 1

    node.create_subscription(LocalPlanEvaluation, '/evaluation', evaluation, qos_profile_sensor_data)
    for key, topic in [('received', '/received_global_plan'), ('transformed', '/transformed_global_plan')]:
        node.create_subscription(NavPath, topic, lambda m, k=key: plan(m, k), qos_profile_sensor_data)
    started = time.monotonic()
    try:
        while not stopped and time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
        report['status'] = 'interrupted' if stopped else 'finished'
    except BaseException as error:
        report.update(status='failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        report['elapsed_wall_seconds'] = time.monotonic()-started
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        raw.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
