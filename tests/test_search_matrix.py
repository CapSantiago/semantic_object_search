"""Exercise the matrix's evidence boundary using child processes, without ROS."""
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

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / 'scripts/run_search_matrix.py'

# This double only models the file/exit-code contract, never ROS task execution.
FAKE_CLIENT = r'''
import argparse, json, os, signal, time
from pathlib import Path
p = argparse.ArgumentParser()
p.add_argument('command')
for option in ('output', 'initial-state-output', 'diagnostics-output', 'timeout', 'pose-timeout', 'cancel-wait',
               'result-topic', 'command-topic', 'discovery-timeout'):
    p.add_argument('--' + option)
p.add_argument('--exclude-task-id', action='append', default=[])
a = p.parse_args()
def save(path, value):
    Path(path).write_text(json.dumps(value))
save(a.initial_state_output, {'map_pose': {'available': True, 'transform': {'header': {'frame_id': 'map'}}},
                              'odom': {'available': True, 'message': {'header': {'frame_id': 'odom'}}}})
d = {'outcome': 'result_received', 'command_sent': True, 'cancellation_sent': False,
     'excluded_task_ids': a.exclude_task_id, 'discovery_timeout_seconds': float(a.discovery_timeout)}
if a.command == 'hang':
    Path(a.output).with_name('child.pid').write_text(str(os.getpid()))
    def stop(*unused):
        d.update(outcome='interrupted', cancellation_sent=True)
        save(a.diagnostics_output, d)
        raise SystemExit(130)
    signal.signal(signal.SIGINT, stop)
    while True:
        time.sleep(.1)
if a.command == 'timeout':
    d.update(outcome='timeout', cancellation_sent=True, cancellation_observed=False)
    save(a.diagnostics_output, d)
    raise SystemExit(124)
status = 'not_found' if a.command == 'empty' else ('canceled' if a.command == 'cancel' else 'completed')
result = dict(task_id='repeated-id' if a.command == 'replay' else Path(a.output).parent.name,
              command=a.command, status=status,
              count=0 if a.command in ('empty', 'wrong_count') else 1,
              elapsed_seconds=2.5, elapsed_wall_seconds=3.5, synchronized_frames=8,
              valid_projections=13, tf_failures=0, objects=[])
save(a.output, result)
save(a.diagnostics_output, d)
raise SystemExit(1 if a.command == 'bad_exit' or status in ('not_found', 'canceled') else 0)
'''


class MatrixTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.client = self.root/'client.py'
        self.client.write_text(FAKE_CLIENT)
        self.config = self.root/'config.json'
        self.config.write_text('{"runtime":"fixture"}')
        self.weights = self.root/'model.pt'
        self.weights.write_bytes(b'offline weight fingerprint fixture')
        self.output = self.root/'output'

    def command(self, cases, extra=()):
        matrix = self.root/'matrix.json'
        matrix.write_text(json.dumps(dict(repeats=2, cases=cases)))
        return [sys.executable, str(RUNNER), str(matrix), '--output', str(self.output),
                '--client', str(self.client), '--config', str(self.config), '--weights', str(self.weights),
                '--first-start', 'cold', '--subsequent-start', 'warm', '--startup-note', 'Explicit test declaration',
                '--timeout', '1', '--pose-timeout', '0', '--cancel-wait', '0', *extra]

    def run_cases(self, cases, extra=()):
        completed = subprocess.run(self.command(cases, extra), capture_output=True, text=True, timeout=15)
        report = json.loads((self.output/'report.json').read_text())
        return completed, report

    @staticmethod
    def case(name, command, status='completed', count=1):
        return dict(id=name, command=command, evaluation=dict(status=status, count=count))

    def test_wrong_counts_are_preserved_and_nonzero_empty_results_are_evaluated(self):
        completed, report = self.run_cases([self.case('wrong', 'wrong_count'),
                                             self.case('empty', 'empty', 'not_found', 0)])
        self.assertEqual(completed.returncode, 1)
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['executed_runs'], 4)
        self.assertEqual([row['passed'] for row in report['runs']], [False, False, True, True])
        self.assertEqual([row['startup_state'] for row in report['runs']], ['cold', 'warm', 'warm', 'warm'])
        row = report['runs'][0]
        self.assertEqual(row['result']['count'], 0)
        self.assertEqual(row['evaluation']['expected']['count'], 1)
        self.assertEqual(row['metrics']['synchronized_frames'], 8)
        self.assertEqual(row['metrics']['valid_projections'], 13)
        self.assertEqual(row['metrics']['elapsed_wall_seconds'], 3.5)
        self.assertEqual(len(row['artifacts']['weights']['sha256']), 64)
        self.assertEqual(row['artifacts']['configs'][0]['path'], str(self.config))
        self.assertEqual(row['initial_state']['map_pose']['transform']['header']['frame_id'], 'map')
        self.assertEqual(row['initial_state']['odom']['message']['header']['frame_id'], 'odom')
        self.assertTrue((Path(row['directory'])/'client.log').exists())

    def test_success_result_with_failed_client_is_not_reported_as_passed(self):
        completed, report = self.run_cases([self.case('broken', 'bad_exit')])
        self.assertEqual(completed.returncode, 1)
        self.assertEqual(report['status'], 'aborted')
        self.assertEqual(report['executed_runs'], 1)
        self.assertFalse(report['runs'][0]['passed'])
        self.assertIn('return code', ' '.join(report['runs'][0]['errors']))

    def test_prior_ids_are_forwarded_and_replayed_success_cannot_pass(self):
        completed, report = self.run_cases([self.case('replay', 'replay'), self.case('later', 'ok')])
        self.assertEqual(completed.returncode, 1)
        self.assertEqual(report['status'], 'aborted')
        self.assertEqual(report['executed_runs'], 2)
        self.assertEqual(report['unexecuted_runs'], 2)
        first, repeated = report['runs']
        self.assertTrue(first['passed'])
        self.assertEqual(repeated['diagnostics']['excluded_task_ids'], ['repeated-id'])
        self.assertTrue(repeated['evaluation']['passed'])
        self.assertFalse(repeated['passed'])
        self.assertIn('fresh matching task', ' '.join(repeated['errors']))

    def test_discovery_budget_is_forwarded_and_included_in_watchdog(self):
        completed, report = self.run_cases([self.case('ok', 'ok')], ['--limit', '1', '--discovery-timeout', '7'])
        self.assertEqual(completed.returncode, 0)
        row = report['runs'][0]
        self.assertEqual(row['diagnostics']['discovery_timeout_seconds'], 7)
        self.assertEqual(report['client_timeouts']['discovery_seconds'], 7)
        self.assertGreaterEqual(row['client_watchdog_seconds'], 1 + 2 + 7)

    def test_unbounded_timeouts_are_rejected_before_starting_children(self):
        for script in (RUNNER, ROOT/'scripts/run_search.py'):
            for option in ('timeout', 'pose-timeout', 'discovery-timeout', 'cancel-wait'):
                for value in ('nan', 'inf', '-inf'):
                    with self.subTest(script=script.name, option=option, value=value):
                        argv = (self.command([self.case('ok', 'ok')]) if script == RUNNER
                                else [sys.executable, str(script), 'unused'])
                        result = subprocess.run([*argv, f'--{option}={value}'],
                                                capture_output=True, text=True, timeout=5)
                        self.assertEqual(result.returncode, 2)
                        self.assertIn('must be finite', result.stderr)

    def test_zero_discovery_budget_is_allowed(self):
        completed, report = self.run_cases([self.case('ok', 'ok')], ['--limit', '1', '--discovery-timeout', '0'])
        self.assertEqual(completed.returncode, 0)
        self.assertEqual(report['client_timeouts']['discovery_seconds'], 0)

    def test_timeout_without_result_retains_diagnostics_and_stops(self):
        completed, report = self.run_cases([self.case('timeout', 'timeout'), self.case('later', 'ok')])
        self.assertEqual(completed.returncode, 1)
        self.assertEqual(report['status'], 'aborted')
        self.assertEqual(report['executed_runs'], 1)
        self.assertEqual(report['unexecuted_runs'], 3)
        row = report['runs'][0]
        self.assertIsNone(row['result'])
        self.assertEqual(row['diagnostics']['outcome'], 'timeout')
        self.assertTrue(row['diagnostics']['cancellation_sent'])
        self.assertFalse(row['diagnostics']['cancellation_observed'])

    def test_canceled_result_stops_even_when_evaluation_expected_it(self):
        _, report = self.run_cases([self.case('cancel', 'cancel', 'canceled')])
        self.assertEqual(report['status'], 'aborted')
        self.assertEqual(report['executed_runs'], 1)
        self.assertTrue(report['runs'][0]['evaluation']['passed'])
        self.assertFalse(report['runs'][0]['passed'])

    def test_case_filter_limit_json_command_and_existing_output_protection(self):
        cases = [self.case('skip', 'wrong_count'), self.case('json', dict(mode='count', room='kitchen', category='cup'))]
        argv = self.command(cases, ['--case', 'json', '--limit', '1'])
        completed = subprocess.run(argv, capture_output=True, text=True, timeout=15)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report_text = (self.output/'report.json').read_text()
        report = json.loads(report_text)
        self.assertEqual(report['planned_runs'], 1)
        self.assertEqual(json.loads(report['runs'][0]['command']), cases[1]['command'])
        repeated = subprocess.run(argv, capture_output=True, text=True, timeout=15)
        self.assertEqual(repeated.returncode, 2)
        self.assertEqual((self.output/'report.json').read_text(), report_text)

    def test_sigterm_cancels_reaps_client_and_does_not_start_next_trial(self):
        process = subprocess.Popen(self.command([self.case('hang', 'hang')]), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.addCleanup(lambda: process.poll() is None and process.kill())
        pid_file = self.output/'001_hang_01/child.pid'
        deadline = time.monotonic()+5
        while not pid_file.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertTrue(pid_file.exists())
        child_pid = int(pid_file.read_text())
        process.send_signal(signal.SIGTERM)
        process.communicate(timeout=6)
        with self.assertRaises(ProcessLookupError):
            os.kill(child_pid, 0)
        report = json.loads((self.output/'report.json').read_text())
        self.assertEqual(report['status'], 'interrupted')
        self.assertEqual(report['executed_runs'], 1)
        self.assertTrue(report['runs'][0]['diagnostics']['cancellation_sent'])

    def test_watchdog_kills_and_reaps_an_unresponsive_client(self):
        spec = importlib.util.spec_from_file_location('matrix', RUNNER)
        runner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runner)
        client = self.root/'unresponsive.py'
        pid_file = self.root/'unresponsive.pid'
        client.write_text('import os, signal, time\nfrom pathlib import Path\n'
                          f'Path({str(pid_file)!r}).write_text(str(os.getpid()))\n'
                          'signal.signal(signal.SIGINT, signal.SIG_IGN)\ntime.sleep(60)\n')
        outcome = runner.run_client([sys.executable, str(client)], self.root/'watchdog.log', .2, 0, [])
        self.assertTrue(outcome['watchdog_timeout'])
        self.assertEqual(outcome['returncode'], -signal.SIGKILL)
        with self.assertRaises(ProcessLookupError):
            os.kill(int(pid_file.read_text()), 0)


if __name__ == '__main__':
    unittest.main()
