"""Failure/cancel/shutdown tests using fake processes; no ROS nodes are started."""
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('robot_console', ROOT/'scripts/robot_console.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Process:
    returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        if self.returncode is None:
            raise subprocess.TimeoutExpired('fake client', timeout)
        return self.returncode


class Monitor:
    def snapshot(self):
        return dict(simulation_ready=True,navigation_ready=True,perception_ready=True)

    def close(self):
        pass


class Runtime:
    def __init__(self, folder):
        self.session_dir=Path(folder)
        self.process=Process()
        self.closed=False
        self.signals=[]
        self.started=[]
        self.fail_start=False
        self.deliver_int=False
        self.deliver_term=True

    def snapshot(self):
        return {name:dict(state='exited' if self.closed else 'running')
                for name in ('simulation','navigation','detector','search')}

    def start_process(self,name,argv):
        self.started.append(name)
        if self.fail_start:
            raise OSError('process registry unavailable')
        return self.process

    def signal_process(self,name,sig):
        self.signals.append(sig)
        if sig==signal.SIGINT:
            return self.deliver_int
        if self.deliver_term:
            self.process.returncode=-int(sig)
            return True
        return False

    def close(self):
        self.closed=True
        self.process.returncode=-15
        return {'status':'stopped'}


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        folder=tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.runtime=Runtime(folder.name)
        self.console=module.Console(self.runtime,Monitor())
        self.console.phase='ready'

    def make_job(self):
        folder=self.runtime.session_dir/'requests/example1'
        folder.mkdir(parents=True)
        (folder/'request.json').write_text('{}')
        (folder/'diagnostics.json').write_text('{"command_sent":true}')
        row=dict(id='example1',kind='search',command='find',state='running',result=None,error=None,
                 artifacts=[],save_status='pending',_started=time.monotonic(),_signature='find',
                 _folder=folder,_process_name='request_example1',_process=self.runtime.process)
        self.console.jobs[row['id']]=self.console.job=row
        return row

    def test_request_write_failure_is_terminal_and_never_launches(self):
        with patch.object(module,'write_json',side_effect=OSError('disk full')):
            with self.assertRaises(module.ApiError):
                self.console.submit(dict(request_id='example1', command='去厨房找杯子'))
        self.assertEqual(self.console.job['state'],'failed')
        self.assertFalse(self.console.snapshot()['busy'])
        self.assertFalse(self.console.uncertain)
        self.assertFalse(self.runtime.closed)
        self.assertEqual(self.runtime.started,[])

    def test_launch_exception_closes_session_and_blocks_new_tasks(self):
        self.runtime.fail_start=True
        with self.assertRaises(module.ApiError):
            self.console.submit(dict(request_id='example1', command='去厨房找杯子'))
        self.assertTrue(self.runtime.closed)
        self.assertTrue(self.console.uncertain)
        self.assertEqual(self.console.job['state'],'failed')
        self.assertFalse(self.console.snapshot()['ready'])

    def test_watcher_launch_failure_also_cleans_up_started_process(self):
        with patch.object(module.threading.Thread,'start',side_effect=RuntimeError('no threads')):
            with self.assertRaises(module.ApiError):
                self.console.submit(dict(request_id='example1', command='去厨房找杯子'))
        self.assertTrue(self.runtime.closed)
        self.assertTrue(self.console.uncertain)
        self.assertIsNotNone(self.runtime.process.poll())

    def test_uppercase_request_id_has_valid_internal_process_name(self):
        with patch.object(module.threading,'Thread') as thread:
            self.console.submit(dict(request_id='TEST1234', command='去厨房找杯子'))
        self.assertRegex(self.runtime.started[0],r'^request_[a-f0-9]{32}$')
        self.assertIn('TEST1234',self.console.watchers)
        thread.return_value.start.assert_called_once()

    def test_failed_sigint_has_independent_deadline_and_escalates(self):
        row=self.make_job()
        self.console.cancel_timeout=0
        self.console.cancel()
        self.assertFalse(row['cancel_signal_delivered'])
        self.assertTrue(self.console.uncertain)
        self.console._watch_job(row)
        self.assertEqual(self.runtime.signals,[signal.SIGINT,signal.SIGTERM])
        self.assertEqual(row['state'],'failed')
        self.assertTrue((row['_folder']/'console_result.json').is_file())

    def test_kill_failure_finalizes_instead_of_losing_watcher_exception(self):
        row=self.make_job()
        self.runtime.deliver_term=False
        self.console.cancel_timeout=0
        self.console.cancel()
        self.console._watch_job(row)
        self.assertEqual(self.runtime.signals,[signal.SIGINT,signal.SIGTERM,signal.SIGKILL])
        self.assertEqual(row['state'],'failed')
        self.assertIn('停止期限',row['error'])
        self.assertTrue(self.console.uncertain)

    def test_shutdown_joins_watcher_again_after_runtime_cleanup(self):
        row=self.make_job()
        watcher=Mock()
        def join(timeout):
            if self.runtime.closed:
                self.console._watch_job(row)
        watcher.join.side_effect=join
        self.console.watchers[row['id']]=watcher
        self.console.cancel_timeout=0
        self.console.close()
        self.assertEqual(watcher.join.call_count,2)
        final=json.loads((self.runtime.session_dir/'console_final.json').read_text())
        self.assertFalse(final['busy'])
        self.assertEqual(final['task']['state'],'failed')
        self.assertTrue((row['_folder']/'console_result.json').is_file())

    def test_shutdown_missing_receipt_stays_final_after_late_watcher(self):
        row=self.make_job()
        self.console.watchers[row['id']]=Mock()
        self.console.cancel_timeout=0
        self.console.close()
        self.assertTrue(row['_finalized'])
        self.assertTrue(self.console.uncertain)
        error=row['error']
        self.runtime.process.returncode=0
        (row['_folder']/'client_result.json').write_text('{"status":"found"}')
        self.console._watch_job(row)
        self.assertEqual(row['state'],'failed')
        self.assertEqual(row['error'],error)


if __name__=='__main__':
    unittest.main()
