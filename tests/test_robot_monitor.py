"""Readiness must reflect observed messages and state, not live processes."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as N
import unittest


SPEC = importlib.util.spec_from_file_location('robot_monitor',
                                             Path(__file__).resolve().parents[1]/'scripts/robot_monitor.py')
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class MonitorStoreTests(unittest.TestCase):
    def setUp(self):
        self.wall = 0.0
        self.store = module.TelemetryStore(now=lambda: self.wall)
        self.store.running = True

    def receive_all(self, clock):
        for topic in module.TOPICS:
            self.store.receive(topic, clock)

    def complete(self):
        self.receive_all(10)
        self.wall = .2
        self.receive_all(10.2)
        self.store.receive_graph(['/semantic_search'], {'/clock': 1}, 1)
        self.store.receive_pose(dict(x=1, y=2, yaw=0, frame_id='map', stamp_seconds=10.2))
        for name in ('amcl', 'bt_navigator'):
            self.store.receive_lifecycle(name, 3, 'active')

    def test_first_or_paused_clock_cannot_make_simulation_ready(self):
        self.receive_all(10)
        self.assertFalse(self.store.snapshot()['simulation_ready'])
        self.wall = .2
        self.receive_all(10.2)
        self.assertTrue(self.store.snapshot()['simulation_ready'])
        self.wall = 4
        self.receive_all(10.2)  # Repeated data still arrives but simulation time is frozen.
        self.assertFalse(self.store.snapshot()['simulation_ready'])
        self.assertFalse(self.store.snapshot()['clock_advancing'])

    def test_navigation_and_perception_require_their_own_live_inputs(self):
        self.complete()
        actual = self.store.snapshot()
        self.assertTrue(actual['simulation_ready'])
        self.assertTrue(actual['navigation_ready'])
        self.assertTrue(actual['perception_ready'])
        self.store.receive_lifecycle('bt_navigator', 2, 'inactive')
        self.assertFalse(self.store.snapshot()['navigation_ready'])
        self.assertTrue(self.store.snapshot()['perception_ready'])
        self.store.receive_graph(['/semantic_search'], {'/clock': 1}, 0)
        self.assertFalse(self.store.snapshot()['perception_ready'])

    def test_late_replayed_sensor_stamp_is_not_fresh(self):
        self.complete()
        self.store.receive('/rgbd/depth_image', 1)
        actual = self.store.snapshot()
        self.assertFalse(actual['perception_ready'])
        self.assertFalse(actual['topics']['/rgbd/depth_image']['fresh'])

    def test_ready_after_mode_switch_depends_on_control_publishers(self):
        self.complete()
        nav = {'/map': ['/map_server'],
               '/cmd_vel': ['/behavior_server'] * 5 + ['/velocity_smoother']}
        self.store.receive_graph(['/amcl'], {'/map': 1, '/cmd_vel': 6}, 1, nav)
        self.assertTrue(self.store.snapshot()['navigation_control_ready'])
        mixed = {**nav, '/cmd_vel': nav['/cmd_vel'] + ['/mapping_motion']}
        self.store.receive_graph(['/amcl'], {'/map': 1, '/cmd_vel': 7}, 1, mixed)
        self.assertFalse(self.store.snapshot()['navigation_control_ready'])

        slam = {'/map': ['/slam_toolbox'], '/cmd_vel': ['/mapping_motion']}
        self.store.receive_map({'available': True})
        self.store.receive_mapping_motion('{"status":"idle"}')
        self.store.receive_graph(['/slam_toolbox', '/mapping_motion'], {'/map': 1, '/cmd_vel': 1}, 0, slam)
        self.assertTrue(self.store.snapshot()['mapping_ready'])
        wrong_owner = {**slam, '/cmd_vel': ['/velocity_smoother']}
        self.store.receive_graph(['/slam_toolbox', '/mapping_motion'], {'/map': 1, '/cmd_vel': 1}, 0, wrong_owner)
        self.assertFalse(self.store.snapshot()['mapping_ready'])

    def test_stale_pose_lifecycle_and_clock_reset_fail_closed(self):
        self.complete()
        self.wall = 6
        self.receive_all(16)
        self.store.receive_graph(['/semantic_search'], {'/clock': 1}, 1)
        actual = self.store.snapshot()
        self.assertTrue(actual['simulation_ready'])
        self.assertFalse(actual['navigation_ready'])
        self.assertIsNone(actual['pose'])
        self.assertFalse(actual['lifecycle']['amcl']['active'])
        self.store.receive('/clock', 0)
        self.assertFalse(self.store.snapshot()['simulation_ready'])

    def test_discovery_requires_a_window_and_detects_existing_publishers(self):
        self.store.receive_graph([], {}, 0)
        self.assertFalse(self.store.snapshot()['discovery_ready'])
        self.wall = 2.1
        self.store.receive_graph([], {}, 0)
        self.assertTrue(self.store.snapshot()['discovery_ready'])
        self.assertFalse(self.store.snapshot()['foreign_stack'])
        self.store.receive_graph(['/some_other_monitor'], {'/clock': 1}, 0)
        self.assertTrue(self.store.snapshot()['foreign_stack'])
        self.store.receive_graph(['/amcl'], {}, 0)
        self.assertTrue(self.store.snapshot()['foreign_stack'])

    def test_static_map_age_does_not_make_it_disappear_and_callers_cannot_mutate(self):
        self.store.receive_map(dict(available=True, data=[0, 100], width=2, height=1))
        self.wall = 100
        result = self.store.map_snapshot()
        self.assertTrue(result['available'])
        self.assertEqual(result['age_seconds'], 100)
        result['data'][0] = 50
        self.assertEqual(self.store.map_snapshot()['data'][0], 0)

    def test_monitor_close_invalidates_readiness_without_discarding_results(self):
        self.complete()
        self.store.running = False
        actual = self.store.snapshot()
        self.assertFalse(actual['connected'])
        self.assertFalse(actual['simulation_ready'])
        self.assertFalse(actual['navigation_ready'])
        self.assertFalse(actual['perception_ready'])


class MapSerializationTests(unittest.TestCase):
    def sample_map(self):
        origin = N(position=N(x=-3, y=4),
                   orientation=N(x=0, y=0, z=math.sin(math.pi/4), w=math.cos(math.pi/4)))
        return N(info=N(width=2, height=2, resolution=.05, origin=origin),
                 data=[-1, 0, 50, 100], header=N(frame_id='map', stamp=N(sec=4, nanosec=500000000)))

    def test_map_retains_rotated_origin_and_row_order(self):
        result = module.map_payload(self.sample_map())
        self.assertEqual(result['data'], [-1, 0, 50, 100])
        self.assertEqual(result['origin']['x'], -3)
        self.assertAlmostEqual(result['origin']['yaw'], math.pi/2)
        self.assertEqual(result['stamp_seconds'], 4.5)

    def test_inconsistent_or_nonfinite_maps_are_rejected(self):
        message = self.sample_map()
        message.data.pop()
        with self.assertRaisesRegex(ValueError, 'dimensions'):
            module.map_payload(message)
        message = self.sample_map()
        message.info.resolution = float('nan')
        with self.assertRaisesRegex(ValueError, 'resolution'):
            module.map_payload(message)


if __name__ == '__main__':
    unittest.main()
