#!/usr/bin/env python3
r"""Run a declared text/JSON task matrix against an already running ROS stack.

Example (paths to --config and --weights are operator-declared runtime inputs)::

    python3 scripts/run_search_matrix.py tests/scenarios/tabletop_repeat_matrix.json \
      --output results/development/repeats_001 --first-start warm \
      --subsequent-start warm --startup-note 'Existing isolated stack, no reset' \
      --config src/semantic_search/config/rooms_aws_house_v1.json \
      --config src/semantic_search/config/nav2_depth.yaml --weights yolov8n-seg.pt

No simulation, localization reset, restart, or ground-truth input is performed.
Expectations are consumed only by the post-result evaluator. Each output directory
must be new. A timeout, cancellation, or broken client stops the entire batch.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


def save(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def fingerprint(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(), bytes=path.stat().st_size)


def load_matrix(path):
    matrix = json.loads(path.read_text(encoding='utf-8'))
    cases = matrix.get('cases', [])
    if not cases:
        raise ValueError('matrix must contain cases')
    seen = set()
    for case in cases:
        name = case.get('id', '')
        if not re.fullmatch(r'[a-zA-Z0-9_-]+', name) or name in seen:
            raise ValueError('case IDs must be unique safe directory names')
        seen.add(name)
        repeat = case.get('repeats', matrix.get('repeats', 1))
        if type(repeat) is not int or repeat < 1:
            raise ValueError(f'{name}: repeats must be a positive integer')
        command = case.get('command')
        if isinstance(command, dict):
            if set(command) != {'mode', 'room', 'category'} or command['mode'] not in ('find', 'count'):
                raise ValueError(f'{name}: JSON commands accept only mode, room, category (find/count)')
            if not all(isinstance(value, str) and value for value in command.values()):
                raise ValueError(f'{name}: command values must be nonempty strings')
        elif not isinstance(command, str) or not command.strip():
            raise ValueError(f'{name}: command must be text or a task object')
        expected = case.get('evaluation', {})
        if set(expected) - {'status', 'count'}:
            raise ValueError(f'{name}: evaluation only accepts status and count')
        if 'count' in expected and (type(expected['count']) is not int or expected['count'] < 0):
            raise ValueError(f'{name}: expected count must be a nonnegative integer')
    return matrix


def read_artifact(path, errors):
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(value, dict):
            raise ValueError('expected JSON object')
        return value
    except (OSError, ValueError) as error:
        errors.append(f'{path.name}: {error}')
        return None


def evaluate(result, expected):
    """Called only after execution; no expectations are passed to the ROS client."""
    checks = {key: result.get(key) == value for key, value in expected.items()}
    return dict(expected=expected, checks=checks, passed=all(checks.values()) if checks else None)


def run_client(argv, log, deadline, cancel_wait, stopped):
    """Own and reap exactly this client's process group, including on interruption."""
    if stopped:
        return dict(returncode=None, watchdog_timeout=False, interrupted=True)
    with log.open('w', encoding='utf-8') as stream:
        process = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        watchdog = False
        try:
            expires = time.monotonic() + deadline
            while process.poll() is None and not stopped and time.monotonic() < expires:
                try:
                    process.wait(timeout=.1)
                except subprocess.TimeoutExpired:
                    pass
            watchdog = process.poll() is None and not stopped
        finally:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=cancel_wait + 3)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()
    return dict(returncode=process.returncode, watchdog_timeout=watchdog, interrupted=bool(stopped))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('matrix', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', action='append', default=[], help='case ID; repeat to select several')
    parser.add_argument('--limit', type=int, help='maximum task executions after case filtering')
    parser.add_argument('--timeout', type=float, default=480)
    parser.add_argument('--pose-timeout', type=float, default=10)
    parser.add_argument('--discovery-timeout', type=float, default=15)
    parser.add_argument('--cancel-wait', type=float, default=5)
    parser.add_argument('--result-topic', default='/search/result')
    parser.add_argument('--command-topic', default='/search/command')
    parser.add_argument('--first-start', choices=('cold', 'warm', 'unknown'), required=True)
    parser.add_argument('--subsequent-start', choices=('warm', 'unknown'), required=True,
                        help='explicit declaration for subsequent tasks; runner never restarts the stack')
    parser.add_argument('--startup-note', required=True, help='operator description of stack start/reset history')
    parser.add_argument('--config', type=Path, action='append', required=True, help='runtime config file; repeatable')
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--client', type=Path, default=Path(__file__).with_name('run_search.py'),
                        help='compatible Python client (also permits offline test doubles)')
    args = parser.parse_args(argv)
    if (not all(math.isfinite(value) for value in
                (args.timeout, args.discovery_timeout, args.pose_timeout, args.cancel_wait))
            or args.timeout <= 0 or args.discovery_timeout < 0 or args.pose_timeout < 0 or args.cancel_wait < 0
            or (args.limit is not None and args.limit < 1)):
        parser.error('timeouts must be finite; task timeout/limit positive, other waits nonnegative')
    try:
        matrix = load_matrix(args.matrix)
        unknown = set(args.case) - {case['id'] for case in matrix['cases']}
        if unknown:
            raise ValueError(f'unknown case IDs: {sorted(unknown)}')
        artifacts = dict(matrix=fingerprint(args.matrix), client=fingerprint(args.client),
                         configs=[fingerprint(path) for path in args.config], weights=fingerprint(args.weights))
        args.output.mkdir(parents=True, exist_ok=False)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    schedule = [(case, repeat) for case in matrix['cases'] if not args.case or case['id'] in args.case
                for repeat in range(1, case.get('repeats', matrix.get('repeats', 1)) + 1)]
    if args.limit:
        schedule = schedule[:args.limit]
    report = dict(schema_version=1, started_utc=datetime.now(timezone.utc).isoformat(), status='running',
                  artifacts=artifacts, matrix=matrix, planned_runs=len(schedule), runs=[],
                  startup=dict(first=args.first_start, subsequent=args.subsequent_start, note=args.startup_note,
                               source='operator_declaration', stack_restarted_by_runner=False),
                  environment={key: os.environ.get(key) for key in ('ROS_DOMAIN_ID', 'IGN_PARTITION', 'RMW_IMPLEMENTATION')},
                  topics=dict(result=args.result_topic, command=args.command_topic),
                  client_timeouts=dict(task_seconds=args.timeout, pose_seconds=args.pose_timeout,
                                       discovery_seconds=args.discovery_timeout, cancel_wait_seconds=args.cancel_wait),
                  limitations=['Config/weights paths are operator declarations, not live-node parameter verification.',
                               'Map TF is localization output; odometry is measured ROS input, not a map-pose substitute.',
                               'No full-room coverage, cold-start completeness, collision or G0 certification.'])
    report_path = args.output / 'report.json'
    save(report_path, report)
    stopped = []
    handlers = {sig: signal.signal(sig, lambda signum, frame: stopped.append(signum))
                for sig in (signal.SIGINT, signal.SIGTERM)}
    seen_task_ids = set()
    try:
        for index, (case, repeat) in enumerate(schedule, 1):
            if stopped:
                report['status'] = 'interrupted'
                break
            folder = args.output / f'{index:03d}_{case["id"]}_{repeat:02d}'
            folder.mkdir()
            command = case['command'] if isinstance(case['command'], str) else json.dumps(case['command'], ensure_ascii=False)
            row = dict(case=case['id'], repeat=repeat, command=command, directory=str(folder.resolve()),
                       startup_state=args.first_start if index == 1 else args.subsequent_start,
                       started_utc=datetime.now(timezone.utc).isoformat())
            report['runs'].append(row)
            save(report_path, report)
            began = time.monotonic()
            # Rehash each trial to retain configuration drift instead of mislabelling it.
            row['artifacts'] = dict(configs=[fingerprint(path) for path in args.config], weights=fingerprint(args.weights))
            client_argv = [sys.executable, str(args.client.resolve()), command, '--timeout', str(args.timeout),
                           '--output', str(folder/'result.json'), '--initial-state-output', str(folder/'initial_state.json'),
                           '--diagnostics-output', str(folder/'diagnostics.json'), '--pose-timeout', str(args.pose_timeout),
                           '--discovery-timeout', str(args.discovery_timeout),
                           '--cancel-wait', str(args.cancel_wait), '--result-topic', args.result_topic,
                           '--command-topic', args.command_topic]
            row['excluded_task_ids'] = sorted(seen_task_ids)
            for task_id in row['excluded_task_ids']:
                client_argv.extend(['--exclude-task-id', task_id])
            row['client_watchdog_seconds'] = args.timeout + max(2, args.pose_timeout) + args.discovery_timeout + args.cancel_wait + 10
            row['client'] = run_client(client_argv, folder/'client.log', row['client_watchdog_seconds'], args.cancel_wait, stopped)
            row['elapsed_client_wall_seconds'] = time.monotonic()-began
            errors = []
            result = read_artifact(folder/'result.json', errors)
            row['initial_state'] = read_artifact(folder/'initial_state.json', errors)
            row['diagnostics'] = read_artifact(folder/'diagnostics.json', errors)
            row['result'] = result
            if result is not None:
                if result.get('command') != command or not result.get('task_id') or result['task_id'] in seen_task_ids:
                    errors.append('result command/task_id does not identify a fresh matching task')
                seen_task_ids.add(result.get('task_id'))
                required = ('status', 'count', 'elapsed_seconds', 'elapsed_wall_seconds',
                            'synchronized_frames', 'valid_projections', 'tf_failures')
                missing = [key for key in required if key not in result]
                if missing:
                    errors.append(f'incomplete result metrics: {missing}')
                row['metrics'] = {key: result.get(key) for key in required}
                row['evaluation'] = evaluate(result, case.get('evaluation', {}))
            diagnostic = row['diagnostics'] or {}
            aborted = (row['client']['watchdog_timeout'] or row['client']['interrupted']
                       or diagnostic.get('outcome') != 'result_received'
                       or diagnostic.get('cancellation_sent') or (result or {}).get('status') == 'canceled')
            # Nonzero is normal for genuine not_found/partial/failed search results;
            # it is never ignored for successful statuses or unrelated client errors.
            valid_exit = result is not None and row['client']['returncode'] == (0 if result.get('status') in ('found', 'completed') else 1)
            if not valid_exit:
                errors.append('client return code inconsistent with the received result')
            row['errors'] = errors
            row['passed'] = not aborted and not errors and row.get('evaluation', {}).get('passed') is True
            save(folder/'trial.json', row)
            save(report_path, report)
            print(json.dumps(dict(run=index, case=case['id'], repeat=repeat, status=(result or {}).get('status'),
                                  passed=row['passed']), ensure_ascii=False), flush=True)
            if aborted or errors:
                report['status'] = 'interrupted' if stopped else 'aborted'
                break
        else:
            report['status'] = 'passed' if all(row['passed'] for row in report['runs']) else 'failed'
    except Exception as error:
        report['status'] = 'aborted'
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        report['finished_utc'] = datetime.now(timezone.utc).isoformat()
        report['executed_runs'] = len(report['runs'])
        report['unexecuted_runs'] = len(schedule)-len(report['runs'])
        save(report_path, report)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
