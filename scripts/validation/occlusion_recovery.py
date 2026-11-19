#!/usr/bin/env python3
"""Reversible opaque-screen test in an idle managed AWS simulation.

Scene edits are evaluation fixtures only; search receives ordinary view poses.
The original rooms parameter and the temporary model are restored on exit.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from run_search_matrix import run_client, save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=5)
    args = parser.parse_args()
    session = json.loads((args.session / 'session.json').read_text())
    if str(session['config']['domain']) != os.environ.get('ROS_DOMAIN_ID'):
        parser.error('ROS domain does not match session')
    if args.repeats < 1:
        parser.error('repeats must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, IGN_PARTITION=session['config']['ign_partition'])
    stopped = []
    handlers = {sig: signal.signal(sig, lambda s, f: stopped.append(s))
                for sig in (signal.SIGINT, signal.SIGTERM)}
    report = dict(status='running', session=str(args.session), cases=[], restored=False,
                  fixture='Opaque screen at kitchen table; not a search input')

    def parameter(value):
        result = subprocess.run(['ros2', 'param', 'set', '--no-daemon', '--spin-time', '3',
            '/semantic_search', 'rooms_file', str(value)], capture_output=True, text=True, timeout=30)
        if result.returncode or 'successful' not in result.stdout:
            raise RuntimeError('rooms_file change rejected: ' + result.stdout + result.stderr)

    result = subprocess.run(['ros2', 'param', 'get', '--no-daemon', '--spin-time', '3',
        '/semantic_search', 'rooms_file'], capture_output=True, text=True, check=True, timeout=30)
    prefix = 'String value is: '
    if not result.stdout.startswith(prefix):
        raise RuntimeError('Unexpected rooms_file type')
    original = result.stdout.removeprefix(prefix).strip()
    rooms = json.loads(Path(original).read_text())
    front = [6.6, -.4, 1.57079633]
    side = [5.1, .8, 0.]
    fixtures = {}
    for kind, poses in [('front', [front]), ('side', [side]), ('both', [front, side])]:
        value = json.loads(json.dumps(rooms))
        value['kitchen']['category_views']['cup'] = poses
        path = args.output / (kind + '_rooms.json')
        save(path, value)
        fixtures[kind] = path.resolve()
    name = 'acceptance_screen_' + uuid.uuid4().hex
    sdf = args.output / 'screen.sdf'
    sdf.write_text(f'''<sdf version="1.8"><model name="{name}"><static>true</static>
      <pose>6.4 0.5 0.9 0 0 0</pose><link name="screen">
      <collision name="collision"><geometry><box><size>0.5 0.06 0.4</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.5 0.06 0.4</size></box></geometry>
      <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual></link></model></sdf>\n''')
    attempted_create = False

    def service(action, request_type, request):
        response = subprocess.run(['ign', 'service', '-s', '/world/aws_house_r0_v1/' + action,
            '--reqtype', 'ignition.msgs.' + request_type, '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '10000', '--req', request], env=env, capture_output=True, text=True, timeout=20)
        report.setdefault('services', []).append(dict(action=action, returncode=response.returncode,
            stdout=response.stdout, stderr=response.stderr))
        if response.returncode or 'data: true' not in response.stdout:
            raise RuntimeError('Simulation service failed: ' + action)

    def trial(kind, views, status, count, capture=False):
        if stopped:
            raise InterruptedError('Stop requested')
        parameter(fixtures[views])
        folder = args.output / f'{len(report["cases"])+1:03d}_{kind}'
        folder.mkdir()
        argv = [sys.executable, str(ROOT / 'scripts/run_search.py'), '去厨房统计杯子',
                '--timeout', '240', '--output', str(folder / 'result.json'),
                '--diagnostics-output', str(folder / 'diagnostics.json')]
        client = run_client(argv, folder / 'client.log', 280, 5, stopped)
        result = json.loads((folder / 'result.json').read_text()) if (folder / 'result.json').exists() else {}
        checks = dict(status=result.get('status') == status, count=result.get('count') == count,
                      observed=result.get('coverage', {}).get('observation_data_complete') is True)
        if kind.startswith('recovery'):
            checks['first_view_no_object'] = result.get('viewpoint_diagnostics', [{}])[0].get('valid_projections') == 0
        row = dict(kind=kind, result=result, client=client, checks=checks, passed=all(checks.values()))
        report['cases'].append(row)
        save(args.output / 'report.json', report)
        print(json.dumps(dict(kind=kind, passed=row['passed'], status=result.get('status'), count=result.get('count'))), flush=True)
        if capture:
            subprocess.run([sys.executable, str(ROOT / 'scripts/capture_rgbd.py'), str(folder / 'capture'),
                '--skip-frames', '4', '--timeout', '30', '--sensor-reliability', 'reliable'], check=True, timeout=40)
        if not row['passed']:
            raise RuntimeError('Occlusion trial failed: ' + kind)

    try:
        trial('baseline_front', 'front', 'completed', 1, True)
        trial('baseline_side', 'side', 'completed', 1, True)
        attempted_create = True
        service('create', 'EntityFactory', 'sdf_filename: ' + json.dumps(str(sdf.resolve())) + ' allow_renaming: false')
        time.sleep(1)
        for index in range(1, args.repeats + 1):
            trial(f'blocked_{index}', 'front', 'not_found', 0, index == 1)
            trial(f'recovery_{index}', 'both', 'completed', 1, index == 1)
        report['status'] = 'passed'
    except BaseException as error:
        report.update(status='interrupted' if stopped else 'failed', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        try:
            if attempted_create:
                service('remove', 'Entity', 'name: ' + json.dumps(name) + ' type: MODEL')
        except Exception as error:
            report.update(status='failed', cleanup_error=str(error))
            raise
        finally:
            try:
                parameter(original)
                report['restored'] = not attempted_create or report['services'][-1]['action'] == 'remove' and 'data: true' in report['services'][-1]['stdout']
            except Exception as error:
                report.update(status='failed', cleanup_error=str(error))
                raise
            finally:
                save(args.output / 'report.json', report)
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)


if __name__ == '__main__':
    main()
