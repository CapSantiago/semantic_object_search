"""Readiness may delay dispatch, but must never erase a later sensor outage."""
import json
import runpy
import sys
import types
from pathlib import Path

import pytest


def run_observer(monkeypatch, tmp_path, *, never_ready=False):
    elapsed = [0.0]
    callbacks = {}
    ns = types.SimpleNamespace

    class Node:
        def __init__(self, *args, **kwargs):
            pass

        def create_subscription(self, kind, topic, callback, qos):
            callbacks[topic] = callback

        def get_clock(self):
            return ns(now=lambda: ns(nanoseconds=int(
                (100 + elapsed[0]) * 1e9) if elapsed[0] >= 3.2 else 0))

        def destroy_node(self):
            pass

    def spin_once(node, timeout_sec):
        elapsed[0] += .1
        if never_ready or elapsed[0] < 3.2:
            return
        stamp = 100 + elapsed[0]
        message = ns(header=ns(stamp=ns(sec=int(stamp), nanosec=int((stamp % 1)*1e9))))
        callbacks['/scan'](message)
        if not 6 <= elapsed[0] <= 6.7:
            callbacks['/rgbd/obstacles'](message)

    modules = {
        'rclpy': dict(init=lambda: None, shutdown=lambda: None, spin_once=spin_once),
        'rclpy.node': dict(Node=Node),
        'rclpy.parameter': dict(Parameter=lambda *a, **kw: None),
        'rclpy.qos': dict(QoSProfile=ns, qos_profile_sensor_data=ns(reliability=1)),
        'sensor_msgs': {}, 'sensor_msgs.msg': dict(PointCloud2=object, LaserScan=object),
        'std_msgs': {}, 'std_msgs.msg': dict(String=object),
    }
    for name, attrs in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr('time.monotonic', lambda: elapsed[0])
    monkeypatch.setattr('signal.signal', lambda *args: None)
    output, ready = tmp_path/'trace.json', tmp_path/'ready.json'
    monkeypatch.setattr(sys, 'argv', ['cloud_continuity.py', '--seconds', '12' if never_ready else '9',
                                    '--output', str(output), '--ready-file', str(ready), '--journal', str(tmp_path/'stream.jsonl')])
    script = Path(__file__).resolve().parents[1]/'scripts/validation/cloud_continuity.py'
    return lambda: runpy.run_path(str(script), run_name='__main__'), output, ready


def test_readiness_retains_startup_and_all_later_outage_checks(monkeypatch, tmp_path):
    run, output, ready = run_observer(monkeypatch, tmp_path)
    run()
    report = json.loads(output.read_text())
    handoff = json.loads(ready.read_text())
    assert 3.2 <= handoff['wall'] < 3.4
    assert report['measurement_started_wall'] == handoff['wall']
    assert any(not sample['ready'] for sample in report['startup_checks'])
    assert any(not sample['ready'] and sample['wall'] > 6 for sample in report['checks'])
    assert report['summary']['stale_checks'] > 0
    assert report['checks'][-1]['ready'] is True
    assert report['startup_error'] is None
    journal = [json.loads(line) for line in (tmp_path/'stream.jsonl').read_text().splitlines()]
    checks = [{k: v for k, v in row.items() if k != 'kind'} for row in journal if row['kind'] == 'check']
    assert checks == report['checks']
    assert any(row['kind'] == 'startup' and not row['ready'] for row in journal)


def test_no_native_readiness_times_out_without_dispatch_handoff(monkeypatch, tmp_path):
    run, output, ready = run_observer(monkeypatch, tmp_path, never_ready=True)
    with pytest.raises(SystemExit, match='within 10 seconds'):
        run()
    report = json.loads(output.read_text())
    assert not ready.exists()
    assert report['measurement_started_wall'] is None
    assert report['checks'] == []
    assert len(report['startup_checks']) >= 100
    assert report['startup_error']
