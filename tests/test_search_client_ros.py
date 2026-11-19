"""Opt-in DDS regression on domain 69 and unique, test-only topic names.

Run after sourcing scripts/env.sh with SEMANTIC_ROS_DOMAIN_ID=69, then set
SEMANTIC_RUN_ROS_CLIENT_TESTS=1. This test starts no simulator or search server
and never publishes to /search/command or /search/result.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('SEMANTIC_RUN_ROS_CLIENT_TESTS') == '1',
                     'explicit isolated ROS client test opt-in required')
class SearchClientRosTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '69', 'tests require isolated domain 69')
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        self.ros, self.String, self.durability = rclpy, String, DurabilityPolicy
        rclpy.init()
        self.node = rclpy.create_node('search_client_fixture_' + uuid4().hex[:8])
        self.addCleanup(self.cleanup_ros)
        self.topic = '/matrix_client_test_' + uuid4().hex
        self.publisher = self.node.create_publisher(
            String, self.topic+'/result', QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.commands = []
        self.command_subscription = self.node.create_subscription(String, self.topic+'/command',
                                      lambda msg: self.commands.append((msg.data, time.monotonic())), 10)
        evidence = os.environ.get('SEMANTIC_CLIENT_TEST_EVIDENCE_DIR')
        if evidence:
            Path(evidence).mkdir(parents=True, exist_ok=True)
            self.folder = Path(tempfile.mkdtemp(prefix=self._testMethodName+'_', dir=evidence))
        else:
            temporary = tempfile.TemporaryDirectory()
            self.addCleanup(temporary.cleanup)
            self.folder = Path(temporary.name)

    def cleanup_ros(self):
        self.node.destroy_node()
        self.ros.try_shutdown()

    def publish(self, task_id, command, status='completed'):
        row = dict(task_id=task_id, command=command, status=status, count=1)
        self.publisher.publish(self.String(data=json.dumps(row, ensure_ascii=False)))

    def exercise(self, command, fresh_result):
        # This unlisted cached result must not be delivered to a VOLATILE reader.
        self.publish('cached-unlisted-id', command)
        output, diagnostics = self.folder/'result.json', self.folder/'diagnostics.json'
        argv = [sys.executable, str(ROOT/'scripts/run_search.py'), command,
                '--output', str(output), '--diagnostics-output', str(diagnostics),
                '--timeout', '1.5', '--cancel-wait', '2',
                '--exclude-task-id', 'previous-task-id',
                '--command-topic', self.topic+'/command', '--result-topic', self.topic+'/result']
        with (self.folder/'client.log').open('w') as stream:
            client = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT)
            sent_old = sent_new = sent_cancel = verified_wait = False
            qos_verified = False
            deadline = time.monotonic()+12
            try:
                while client.poll() is None and time.monotonic() < deadline:
                    self.ros.spin_once(self.node, timeout_sec=.03)
                    for info in self.node.get_subscriptions_info_by_topic(self.topic+'/result'):
                        self.assertEqual(info.qos_profile.durability, self.durability.VOLATILE)
                        qos_verified = True
                    requested = next((stamp for text, stamp in self.commands if text == command), None)
                    if requested is None:
                        continue
                    elapsed = time.monotonic()-requested
                    if elapsed > .15 and not sent_old:
                        # Delayed live replay arrives after the client has sent its command.
                        self.publish('previous-task-id', command)
                        sent_old = True
                    if elapsed > .45 and not verified_wait:
                        self.assertFalse(output.exists(), 'client returned an old task result')
                        self.assertIsNone(client.poll(), 'client exited before receiving a fresh result')
                        verified_wait = True
                    if fresh_result and elapsed > .8 and not sent_new:
                        self.publish('new-task-id', command)
                        sent_new = True
                    if any(text == '取消' for text, _ in self.commands) and not sent_cancel:
                        self.publish('new-task-id', command, 'canceled')
                        sent_cancel = True
                self.assertIsNotNone(client.poll(), 'client did not finish within test deadline')
                client.wait(timeout=1)
            finally:
                if client.poll() is None:
                    client.send_signal(signal.SIGINT)
                    try:
                        client.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        client.kill()
                        client.wait()
        self.assertTrue(qos_verified)
        self.assertTrue(sent_old and verified_wait)
        actual = json.loads(output.read_text())
        detail = json.loads(diagnostics.read_text())
        self.assertEqual(actual['task_id'], 'new-task-id')
        self.assertEqual(actual['command'], command)
        self.assertEqual(detail['ignored_task_ids'], ['previous-task-id'])
        self.assertEqual(detail['pre_command_task_ids'], [])
        self.assertEqual(detail['result_qos_durability'], 'volatile')
        metadata = dict(domain=69, command_topic=self.topic+'/command', result_topic=self.topic+'/result',
                        command=command, received_commands=[text for text, _ in self.commands],
                        previous_live_id_ignored=True, cached_id_not_received=True,
                        waited_for_new_result=True, actual_subscription_volatile=qos_verified,
                        client_returncode=client.returncode, client_reaped=client.poll() is not None)
        (self.folder/'test_evidence.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
        return client.returncode, actual, detail

    def test_delayed_old_id_is_ignored_until_new_result_and_json_is_unchanged(self):
        command = '{ "mode": "count", "room": "kitchen", "category": "cup" }'
        returncode, actual, detail = self.exercise(command, fresh_result=True)
        self.assertEqual(returncode, 0)
        self.assertEqual(actual['status'], 'completed')
        self.assertEqual(detail['outcome'], 'result_received')
        self.assertFalse(detail['cancellation_sent'])

    def test_only_replayed_old_id_times_out_and_cancels_instead_of_passing(self):
        returncode, actual, detail = self.exercise('去厨房统计杯子', fresh_result=False)
        self.assertEqual(returncode, 124)
        self.assertEqual(actual['status'], 'canceled')
        self.assertEqual(detail['outcome'], 'timeout')
        self.assertTrue(detail['cancellation_sent'])
        self.assertTrue(detail['cancellation_observed'])

    def discovery_exercise(self, create_subscriber, interrupt=False, spectator=False):
        self.node.destroy_subscription(self.command_subscription)
        if spectator:
            observer=self.ros.create_node('passive_command_observer_'+uuid4().hex[:8])
            observer.create_subscription(self.String,self.topic+'/command',lambda msg:None,10)
            self.addCleanup(observer.destroy_node)
        output, diagnostics = self.folder/'result.json', self.folder/'diagnostics.json'
        command = '去厨房找杯子'
        discovery_timeout = 4 if create_subscriber or interrupt else .7
        argv = [sys.executable, str(ROOT/'scripts/run_search.py'), command,
                '--output', str(output), '--diagnostics-output', str(diagnostics),
                '--timeout', '.6', '--discovery-timeout', str(discovery_timeout), '--cancel-wait', '0',
                '--command-topic', self.topic+'/command', '--result-topic', self.topic+'/result']
        appeared = subscriber_created = signaled = None
        with (self.folder/'client.log').open('w') as stream:
            client = subprocess.Popen(argv, stdout=stream, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+12
            published = False
            try:
                while client.poll() is None and time.monotonic() < deadline:
                    self.ros.spin_once(self.node, timeout_sec=.03)
                    now = time.monotonic()
                    if appeared is None and self.node.count_publishers(self.topic+'/command'):
                        appeared = now
                    if create_subscriber and appeared is not None and now-appeared > 3 and subscriber_created is None:
                        self.assertIsNone(client.poll(), 'client exited before delayed DDS matching')
                        self.assertFalse(output.exists())
                        self.command_subscription = self.node.create_subscription(self.String, self.topic+'/command',
                            lambda msg: self.commands.append((msg.data, time.monotonic())), 10)
                        subscriber_created = now
                    if interrupt and appeared is not None and now-appeared > 2.5 and signaled is None:
                        client.send_signal(signal.SIGTERM)
                        signaled = now
                    if self.commands and not published:
                        self.assertTrue(create_subscriber)
                        self.assertEqual(self.commands[0][0], command)
                        self.publish('discovery-new-task', command, 'found')
                        published = True
                self.assertIsNotNone(client.poll(), 'discovery wait exceeded bounded test deadline')
                client.wait(timeout=1)
            finally:
                if client.poll() is None:
                    client.kill()
                    client.wait()
        detail = json.loads(diagnostics.read_text())
        if create_subscriber:
            self.assertEqual(client.returncode, 0)
            self.assertEqual(json.loads(output.read_text())['task_id'], 'discovery-new-task')
            self.assertTrue(detail['command_sent'])
            self.assertGreater(detail['discovery_wait_wall_seconds'], .5)
            self.assertGreaterEqual(detail['command_subscribers_before_publish'], 1)
            self.assertGreaterEqual(self.commands[0][1], subscriber_created)
        else:
            self.assertEqual(client.returncode, 130 if interrupt else 2)
            self.assertFalse(output.exists())
            self.assertFalse(detail['command_sent'])
            self.assertFalse(detail['cancellation_sent'])
            self.assertEqual(detail['outcome'], 'interrupted_before_command' if interrupt else 'discovery_timeout')
            if spectator:
                self.assertGreaterEqual(detail['command_subscribers_before_publish'],1)
                self.assertEqual(detail['serving_nodes_before_publish'],[])
            else:
                self.assertEqual(detail['command_subscribers_before_publish'], 0)
            if not interrupt:
                self.assertGreaterEqual(detail['discovery_wait_wall_seconds'], discovery_timeout)
                self.assertLess(detail['discovery_wait_wall_seconds'], discovery_timeout + .3)
        metadata = dict(domain=69, command_topic=self.topic+'/command', result_topic=self.topic+'/result',
                        scenario='delayed_subscriber' if create_subscriber else ('interrupted_discovery' if interrupt else 'missing_subscriber'),
                        subscriber_creation_delay_after_publisher_discovery=None if subscriber_created is None else subscriber_created-appeared,
                        command_sent=detail['command_sent'], outcome=detail['outcome'], client_returncode=client.returncode,
                        discovery_wait_wall_seconds=detail['discovery_wait_wall_seconds'],
                        task_timeout_seconds=.6, discovery_timeout_seconds=discovery_timeout,
                        received_commands=[text for text, _ in self.commands], client_reaped=client.poll() is not None)
        (self.folder/'test_evidence.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')

    def test_command_subscriber_created_after_three_seconds_still_succeeds(self):
        self.discovery_exercise(create_subscriber=True)

    def test_missing_command_subscriber_exits_without_sending_a_task(self):
        self.discovery_exercise(create_subscriber=False)

    def test_signal_during_discovery_exits_without_command_or_cancel(self):
        self.discovery_exercise(create_subscriber=False, interrupt=True)

    def test_passive_observer_does_not_replace_delayed_search_server(self):
        self.discovery_exercise(create_subscriber=True, spectator=True)

    def test_passive_observer_alone_does_not_receive_task_or_cancel(self):
        self.discovery_exercise(create_subscriber=False, spectator=True)


if __name__ == '__main__':
    unittest.main()
