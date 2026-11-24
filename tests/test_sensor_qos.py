"""Use real RMW QoS checks without initializing ROS or creating a node."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

try:
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSCompatibility, QoSProfile,
        ReliabilityPolicy, qos_check_compatible, qos_profile_sensor_data,
    )
    from semantic_search.sensor_qos import declare_sensor_qos, sensor_qos_profile
except ModuleNotFoundError as error:
    if error.name.split('.')[0] not in {'rclpy', 'rcl_interfaces'}:
        raise
    sensor_qos_profile = None


@unittest.skipIf(sensor_qos_profile is None, 'Source scripts/env.sh for ROS QoS/message libraries')
class SensorQoSTests(unittest.TestCase):
    def test_latest_frame_queue_preserves_reliability_and_global_default(self):
        chosen = sensor_qos_profile('reliable',depth=1)
        self.assertEqual(chosen.depth,1)
        self.assertEqual(chosen.reliability,ReliabilityPolicy.RELIABLE)
        self.assertEqual(chosen.history,HistoryPolicy.KEEP_LAST)
        self.assertEqual(sensor_qos_profile().depth,5)
        for value in (0,-1,True,1.5):
            with self.assertRaises(ValueError):sensor_qos_profile(depth=value)

    def test_input_freshness_rejects_queued_old_and_invalid_frames(self):
        from semantic_search.sensor_qos import current_sensor_stamp
        self.assertTrue(current_sensor_stamp(10.,10.2))
        self.assertFalse(current_sensor_stamp(10.,10.4))
        self.assertFalse(current_sensor_stamp(10.3,10.))
        self.assertFalse(current_sensor_stamp(0.,0.))
        self.assertFalse(current_sensor_stamp(float('nan'),10.))

    def test_default_matches_sensor_preset_and_reliable_changes_only_reliability(self):
        baseline = sensor_qos_profile()
        reliable = sensor_qos_profile('reliable')
        self.assertEqual(baseline, qos_profile_sensor_data)
        self.assertEqual(reliable.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(reliable.history, HistoryPolicy.KEEP_LAST)
        self.assertEqual(reliable.depth, 5)
        self.assertEqual(reliable.durability, DurabilityPolicy.VOLATILE)
        for policy in ['history', 'depth', 'durability', 'deadline', 'lifespan', 'liveliness',
                       'liveliness_lease_duration', 'avoid_ros_namespace_conventions']:
            self.assertEqual(getattr(reliable, policy), getattr(qos_profile_sensor_data, policy))

    def test_profiles_do_not_mutate_global_sensor_preset_or_other_subscribers(self):
        original = sensor_qos_profile()
        chosen = sensor_qos_profile('reliable')
        self.assertIsNot(chosen, qos_profile_sensor_data)
        self.assertIsNot(original, chosen)
        chosen.depth = 17
        chosen.durability = DurabilityPolicy.TRANSIENT_LOCAL
        chosen.reliability = ReliabilityPolicy.BEST_EFFORT
        self.assertEqual(qos_profile_sensor_data, original)
        self.assertEqual(sensor_qos_profile('reliable').depth, 5)
        self.assertEqual(sensor_qos_profile('reliable').durability, DurabilityPolicy.VOLATILE)

    def test_real_qos_compatibility_accepts_reliable_publishers_but_rejects_best_effort_to_reliable(self):
        for publisher_reliability, selected, compatible in [
                (ReliabilityPolicy.BEST_EFFORT, 'best_effort', True),
                (ReliabilityPolicy.RELIABLE, 'best_effort', True),
                (ReliabilityPolicy.RELIABLE, 'reliable', True),
                (ReliabilityPolicy.BEST_EFFORT, 'reliable', False)]:
            with self.subTest(publisher=publisher_reliability, subscription=selected):
                publisher = QoSProfile(depth=10, reliability=publisher_reliability,
                                       durability=DurabilityPolicy.VOLATILE)
                status, reason = qos_check_compatible(publisher, sensor_qos_profile(selected))
                # Sensor-data SYSTEM_DEFAULT liveliness can produce WARNING.
                # Preserve it instead of altering product policy to force OK.
                if compatible:
                    self.assertNotEqual(status, QoSCompatibility.ERROR, reason)
                else:
                    self.assertEqual(status, QoSCompatibility.ERROR)

    def test_parameter_accepts_startup_override_and_real_rclpy_descriptor_rejects_live_change(self):
        node = SimpleNamespace(declare_parameter=Mock(return_value=SimpleNamespace(value='reliable')))
        profile = declare_sensor_qos(node)
        self.assertEqual(profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(node.declare_parameter.call_args.args, ('sensor_reliability', 'best_effort'))
        descriptor = node.declare_parameter.call_args.kwargs['descriptor']
        self.assertTrue(descriptor.read_only)
        result = Node._apply_descriptor(
            node, Parameter('sensor_reliability', value='best_effort'), descriptor)
        self.assertFalse(result.successful)
        self.assertIn('read-only', result.reason)

    def test_unknown_startup_values_are_rejected_without_falling_back(self):
        for value in ['', 'RELIABLE', 'sensor_data', None, []]:
            with self.subTest(value=value):
                node = SimpleNamespace(declare_parameter=Mock(return_value=SimpleNamespace(value=value)))
                with self.assertRaisesRegex(ValueError, 'sensor_reliability.*best_effort.*reliable'):
                    declare_sensor_qos(node)


if __name__ == '__main__':
    unittest.main()
