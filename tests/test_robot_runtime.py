"""Real local subprocess checks for ownership and cleanup; no ROS stack starts."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('robot_runtime', ROOT/'scripts/robot_runtime.py')
runtime_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime_module)
Runtime = runtime_module.Runtime


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.runtimes = []
        self.addCleanup(self.close_runtimes)

    def close_runtimes(self):
        for runtime in reversed(self.runtimes):
            runtime.close(graceful_seconds=.15)

    def make_runtime(self, **kwargs):
        options = dict(domain=71, headless=True, check_dependencies=False)
        options.update(kwargs)
        runtime = Runtime(self.root, **options)
        self.runtimes.append(runtime)
        return runtime

    def wait_file(self, path, timeout=3):
        deadline = time.monotonic()+timeout
        while not path.exists() and time.monotonic()<deadline:
            time.sleep(.02)
        self.assertTrue(path.exists(), str(path))

    def test_duplicate_owner_is_rejected_and_close_releases_locks(self):
        first = self.make_runtime()
        with self.assertRaisesRegex(RuntimeError, '已有一键运行实例'):
            self.make_runtime()
        first.close(graceful_seconds=0)
        second = self.make_runtime()
        self.assertNotEqual(first.session_dir, second.session_dir)
        self.assertNotEqual(first.env['IGN_PARTITION'], second.env['IGN_PARTITION'])
        self.assertEqual(second.env['ROS_DOMAIN_ID'], '71')
        self.assertEqual(second.snapshot()['simulation']['state'], 'not_started')
        self.assertEqual(second.snapshot()['rviz']['state'], 'disabled')

    def test_dependency_and_display_failure_release_locks(self):
        with self.assertRaisesRegex(RuntimeError, '运行资源缺失'):
            self.make_runtime(check_dependencies=True)
        self.assertFalse((self.root/'results/sessions').exists())
        with self.assertRaisesRegex(RuntimeError, '--headless'):
            self.make_runtime(headless=False, env={})
        runtime = self.make_runtime()
        self.assertTrue(runtime.session_dir.is_dir())

    def test_different_projects_cannot_own_the_same_ros_domain(self):
        first = self.make_runtime()
        second_root = self.root/'second_project'
        second_root.mkdir()
        with self.assertRaisesRegex(RuntimeError, '已有一键运行实例'):
            Runtime(second_root, domain=71, headless=True, check_dependencies=False)
        # The failed second domain lock must also release its project lock.
        second = Runtime(second_root, domain=72, headless=True, check_dependencies=False)
        self.runtimes.append(second)
        self.assertTrue(first.session_dir.is_dir())
        self.assertEqual(second.domain, 72)

    def test_existing_session_is_not_modified(self):
        existing = self.root/'already_saved'
        existing.mkdir()
        (existing/'result.json').write_text('keep original')
        with self.assertRaises(FileExistsError):
            self.make_runtime(session_dir=existing)
        self.assertEqual(sorted(p.name for p in existing.iterdir()), ['result.json'])
        self.assertEqual((existing/'result.json').read_text(), 'keep original')
        runtime = self.make_runtime()
        self.assertNotEqual(runtime.session_dir, existing)

    def test_explicit_fixture_is_used_and_fingerprinted(self):
        rooms=self.root/'rooms.json'
        rooms.write_text(json.dumps({'test':dict(aliases=['测试'],bounds=[0,0,2,2],views=[[1,1,0]])}))
        layout=self.root/'layout.json'
        layout.write_text(json.dumps(dict(targets=[])))
        runtime=self.make_runtime(rooms_file='rooms.json',layout_file='layout.json')
        self.assertIn('rooms_file:='+str(rooms),runtime.command('search'))
        self.assertIn('layout_file:=layout.json',runtime.command('simulation'))
        receipt=json.loads((runtime.session_dir/'session.json').read_text())
        self.assertEqual(receipt['config']['rooms'],'rooms.json')
        self.assertIn('rooms.json',receipt['source_sha256'])

    def test_invalid_fixture_releases_runtime_locks(self):
        with self.assertRaises(ValueError):
            self.make_runtime(rooms_file='../missing.json')
        runtime=self.make_runtime()
        self.assertEqual(runtime.config['rooms'],'src/semantic_search/config/rooms_aws_house_v4.json')

    def test_current_candidate_arguments_are_explicit(self):
        runtime = self.make_runtime()
        simulation = runtime.command('simulation')
        self.assertIn('gui:=false', simulation)
        self.assertIn('layout_file:=scenes/aws_house/layout_r0_yellow_v1.json', simulation)
        self.assertIn('rooms_file:='+str(self.root/'src/semantic_search/config/rooms_aws_house_v4.json'),
                      runtime.command('search'))
        for component in ('search','detector'):
            self.assertIn('sensor_reliability:=reliable', runtime.command(component))
        self.assertIn('device:=cuda:0',runtime.command('detector'))
        self.assertEqual(runtime.env['SEMANTIC_DETECTOR_DEVICE'],'cuda:0')
        self.assertIn('output_dir:='+str(runtime.task_output_dir), runtime.command('search'))
        with self.assertRaises(ValueError):
            runtime.command('rviz')
        with self.assertRaises(ValueError):
            runtime.start_process('../escape', [sys.executable, '-c', 'pass'])

    def test_cpu_fallback_is_explicit_and_recorded(self):
        runtime=self.make_runtime(device='cpu')
        self.assertIn('device:=cpu',runtime.command('detector'))
        self.assertEqual(runtime.env['SEMANTIC_DETECTOR_DEVICE'],'cpu')
        receipt=json.loads((runtime.session_dir/'session.json').read_text())
        self.assertEqual(receipt['config']['detector']['python_env'],'.venv')
        self.assertFalse(receipt['config']['detector']['half'])

    def test_invalid_device_cannot_acquire_locks(self):
        with self.assertRaises(ValueError):self.make_runtime(device='invalid')
        self.make_runtime(device='cpu')

    def test_logs_exit_codes_and_failed_exec_are_preserved(self):
        runtime = self.make_runtime()
        process = runtime.start_process('success', [sys.executable, '-c', 'print("actual child output",flush=True)'])
        self.assertEqual(process.wait(timeout=3), 0)
        self.assertIn('actual child output', runtime.log_tail('success'))
        self.assertEqual(runtime.snapshot()['success']['returncode'], 0)
        with self.assertRaises(RuntimeError):
            runtime.start_process('success', [sys.executable, '-c', 'pass'])
        with self.assertRaises(FileNotFoundError):
            runtime.start_process('missing', ['/definitely/missing/robot/executable'])
        receipt = runtime.close(graceful_seconds=.1)
        self.assertEqual(receipt['status'], 'stopped')
        self.assertEqual(runtime.close(), receipt)
        self.assertTrue((runtime.session_dir/'shutdown.json').is_file())

    def test_signal_only_targets_named_owned_client(self):
        runtime = self.make_runtime()
        ready = runtime.session_dir/'ready'
        code = ('import signal,time;from pathlib import Path;'
                'signal.signal(signal.SIGINT,lambda *args:exit(23));'
                f'Path({str(ready)!r}).write_text("ready");'
                'time.sleep(60)')
        client = runtime.start_process('task_client', [sys.executable, '-c', code])
        other = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'])
        try:
            self.wait_file(ready)
            self.assertTrue(runtime.signal_process('task_client'))
            self.assertEqual(client.wait(timeout=3), 23)
            self.assertIsNone(other.poll())
            runtime.close(graceful_seconds=.1)
            self.assertIsNone(other.poll(), 'cleanup touched an unrelated child')
        finally:
            other.terminate()
            other.wait(timeout=3)

    def test_detached_descendants_are_cleaned_after_parent_exits(self):
        runtime = self.make_runtime()
        child_pid = runtime.session_dir/'detached.pid'
        child_code = 'import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);time.sleep(60)'
        parent_code = ('import subprocess,sys;from pathlib import Path;'
                       f'child=subprocess.Popen([sys.executable,"-c",{child_code!r}],start_new_session=True);'
                       f'Path({str(child_pid)!r}).write_text(str(child.pid))')
        parent = runtime.start_process('detaching_parent', [sys.executable, '-c', parent_code])
        parent.wait(timeout=3)
        self.wait_file(child_pid)
        pid = int(child_pid.read_text())
        self.assertIsNotNone(runtime_module._identity(pid, runtime.marker))
        receipt = runtime.close(graceful_seconds=.1)
        self.assertEqual(receipt['status'], 'stopped')
        self.assertIsNone(runtime_module._identity(pid, runtime.marker))
        self.assertTrue(any(row['pid']==pid and row['signal']=='SIGTERM' for row in receipt['signals']))

    def test_pid_identity_mismatch_is_never_signalled(self):
        runtime = self.make_runtime()
        process = runtime.start_process('sleeper', [sys.executable, '-c', 'import time;time.sleep(60)'])
        identity = runtime_module._identity(process.pid, runtime.marker)
        self.assertIsNotNone(identity)
        wrong = dict(identity, start_ticks=identity['start_ticks']+1)
        self.assertFalse(runtime_module._signal(wrong, runtime.marker, signal.SIGTERM))
        self.assertIsNone(process.poll())

    def test_registration_write_failure_reaps_owned_processes_and_releases_locks(self):
        runtime = self.make_runtime()
        ready = runtime.session_dir / 'registered_child.pid'
        parent_code = (
            'import subprocess,sys,time;from pathlib import Path;'
            'child=subprocess.Popen([sys.executable,"-c","import time;time.sleep(60)"],'
            'start_new_session=True);'
            f'Path({str(ready)!r}).write_text(str(child.pid));time.sleep(60)')

        def failed_write():
            self.wait_file(ready)
            raise OSError('simulated registration write failure')

        runtime._write_processes = failed_write
        with self.assertRaisesRegex(OSError, 'simulated registration write failure'):
            runtime.start_process('unpersisted_client', [sys.executable, '-c', parent_code])
        row = runtime.snapshot()['unpersisted_client']
        self.assertEqual(row['state'], 'exited')
        self.assertIsNone(runtime_module._identity(row['pid'], runtime.marker))
        self.assertIsNone(runtime_module._identity(int(ready.read_text()), runtime.marker))
        self.assertEqual(json.loads((runtime.session_dir / 'shutdown.json').read_text())['status'], 'stopped')
        with self.assertRaisesRegex(RuntimeError, 'already closed'):
            runtime.start_process('retry', [sys.executable, '-c', 'pass'])
        replacement = self.make_runtime()
        self.assertNotEqual(replacement.session_dir, runtime.session_dir)

    def test_transient_identity_lookup_does_not_disable_cancellation(self):
        runtime = self.make_runtime()
        original_identity = runtime_module._identity
        lookups = []

        def delayed_identity(pid, marker):
            lookups.append(pid)
            return None if len(lookups) == 1 else original_identity(pid, marker)

        ready = runtime.session_dir / 'cancel_ready'
        code = ('import signal,time;from pathlib import Path;'
                'signal.signal(signal.SIGINT,lambda *args:exit(23));'
                f'Path({str(ready)!r}).write_text("ready");time.sleep(60)')
        with patch.object(runtime_module, '_identity', side_effect=delayed_identity):
            child = runtime.start_process('cancel_client', [sys.executable, '-c', code])
        self.assertGreaterEqual(len(lookups), 2)
        self.wait_file(ready)
        # Also exercise recovery after an unusually long initial /proc delay.
        runtime._processes['cancel_client']['identity'] = None
        self.assertTrue(runtime.signal_process('cancel_client', signal.SIGINT))
        self.assertEqual(child.wait(timeout=3), 23)


if __name__ == '__main__':
    unittest.main()
