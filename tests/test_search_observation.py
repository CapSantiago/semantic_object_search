"""Callback regressions without starting ROS nodes; source scripts/env.sh for imports."""
import json
from tempfile import TemporaryDirectory
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

from semantic_search.rooms import observation_coverage

try:
    from semantic_search.search_node import Search
    from builtin_interfaces.msg import Time as TimeMessage
    from tf2_ros import TransformException
except ModuleNotFoundError as error:
    if error.name.split('.')[0] not in {
            'action_msgs', 'ament_index_python', 'cv_bridge', 'geometry_msgs', 'message_filters',
            'nav2_msgs', 'nav_msgs', 'rclpy', 'sensor_msgs', 'std_msgs', 'tf2_geometry_msgs',
            'tf2_ros', 'visualization_msgs', 'yolo_msgs'}:
        raise
    Search = None


def viewpoint(index=0, **changes):
    view = dict(index=index, pose=[0., 0., 0.], reached=False, navigation_status=None,
                dwell_completed=False, synchronized_messages=0, rejected_sensor_frames=0,
                synchronized_frames=0, tf_failures=0, invalid_target_depth=0, valid_projections=0,
                extra_wait_started=False, waiting_for_sensor_data=False, extra_wait_seconds=0.0)
    view.update(changes)
    return view


@unittest.skipIf(Search is None, 'Source scripts/env.sh to load ROS message types; no ROS node is started')
class SearchObservationTests(unittest.TestCase):
    def test_integer_json_coordinates_are_valid_ros_goal_coordinates(self):
        node=SimpleNamespace(view=0,views=[[0,0,0]],phase='idle',nav=Mock(),on_goal=Mock(), publish_progress=Mock(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:TimeMessage())))
        Search.next_view(node)
        goal=node.nav.send_goal_async.call_args.args[0]
        self.assertEqual(goal.pose.pose.position.x,0.)
        self.assertEqual(goal.pose.pose.position.y,0.)

    @staticmethod
    def waiting_node(extra=4., previous_data=True):
        now = SimpleNamespace(nanoseconds=5_000_000_000)
        diagnostics = [viewpoint(reached=True, dwell_completed=True,
                                  synchronized_frames=2, valid_projections=2)] if previous_data else []
        index = len(diagnostics)
        diagnostics.append(viewpoint(index, reached=True))
        node = SimpleNamespace(
            phase='observing', observe_start=1_000_000_000, observe_dwell_seconds=4.,
            request_id=None, publish_progress=Mock(),
            observe_extra_seconds=extra, last_stamp=-1, view=index,
            views=[row['pose'] for row in diagnostics], viewpoint_diagnostics=diagnostics,
            get_clock=lambda: SimpleNamespace(now=lambda: now),
            frames=2 if previous_data else 0, valid_projection=2 if previous_data else 0,
            tf_failures=0, invalid_target_depth=0, task=('count', 'kitchen', 'cup'),
            landmarks=SimpleNamespace(confirmed=[object()] if previous_data else []),
            failed_views=[], finish=Mock(), param=lambda key: .35)
        node.record_observation = MethodType(Search.record_observation, node)
        node.next_view = MethodType(Search.next_view, node)
        node.tick = MethodType(Search.tick, node)
        return node, now

    @staticmethod
    def empty_observation(second):
        return (SimpleNamespace(header=SimpleNamespace(stamp=TimeMessage(sec=second)), detections=[]),
                SimpleNamespace(width=640, height=480, encoding='32FC1'),
                SimpleNamespace(width=640, height=480, k=[500., 0., 320., 0., 500., 240., 0., 0., 1.]))

    def test_delayed_frame_ends_extra_wait_without_extending_all_viewpoints(self):
        node, now = self.waiting_node()
        Search.tick(node)
        self.assertTrue(node.viewpoint_diagnostics[1]['waiting_for_sensor_data'])
        node.finish.assert_not_called()
        now.nanoseconds = 6_000_000_000
        Search.on_observation(node, *self.empty_observation(6))
        Search.tick(node)
        node.finish.assert_called_once_with('completed')
        self.assertEqual(node.viewpoint_diagnostics[1]['extra_wait_seconds'], 1.)
        self.assertFalse(node.viewpoint_diagnostics[1]['waiting_for_sensor_data'])
        self.assertEqual(node.viewpoint_diagnostics[0]['extra_wait_seconds'], 0.)

    def test_permanent_sensor_dropout_stops_at_extra_limit(self):
        for previous, expected in [(True, 'partial'), (False, 'no_sensor_data')]:
            with self.subTest(previous=previous):
                node, now = self.waiting_node(previous_data=previous)
                Search.tick(node)
                now.nanoseconds = 8_900_000_000
                Search.tick(node)
                node.finish.assert_not_called()
                now.nanoseconds = 9_000_000_000
                Search.tick(node)
                node.finish.assert_called_once_with(expected)
                self.assertEqual(node.viewpoint_diagnostics[-1]['extra_wait_seconds'], 4.)
                self.assertFalse(node.viewpoint_diagnostics[-1]['waiting_for_sensor_data'])

    def test_frame_arriving_after_extra_deadline_cannot_rescue_expired_observation(self):
        for prior_tick in [False, True]:
            with self.subTest(prior_tick=prior_tick):
                node, now = self.waiting_node()
                if prior_tick:
                    Search.tick(node)
                now.nanoseconds = 9_100_000_000
                Search.on_observation(node, *self.empty_observation(9))
                node.finish.assert_called_once_with('partial')
                self.assertEqual(node.viewpoint_diagnostics[1]['synchronized_frames'], 0)

    def test_zero_extra_restores_nominal_dwell_without_data(self):
        node, now = self.waiting_node(extra=0.)
        Search.tick(node)
        node.finish.assert_called_once_with('partial')
        self.assertFalse(node.viewpoint_diagnostics[1]['extra_wait_started'])
        self.assertEqual(node.viewpoint_diagnostics[1]['extra_wait_seconds'], 0.)

    def test_target_depth_failure_has_frames_and_does_not_trigger_extra_wait(self):
        node, now = self.waiting_node(previous_data=False)
        node.frames = node.viewpoint_diagnostics[0]['synchronized_frames'] = 1
        node.invalid_target_depth = node.viewpoint_diagnostics[0]['invalid_target_depth'] = 1
        Search.tick(node)
        node.finish.assert_called_once_with('localization_failed')
        self.assertFalse(node.viewpoint_diagnostics[0]['extra_wait_started'])

    def test_observation_time_parameters_reject_negative_or_nonfinite_values(self):
        for name in ['dwell_seconds', 'observation_extra_seconds']:
            for value in [-1., float('nan'), float('inf'), True, '4']:
                with self.subTest(name=name, value=value):
                    result = Search.validate_observation_parameters(
                        None, [SimpleNamespace(name=name, value=value)])
                    self.assertFalse(result.successful)
                    self.assertIn(name, result.reason)
            for value in [0., 4.]:
                self.assertTrue(Search.validate_observation_parameters(
                    None, [SimpleNamespace(name=name, value=value)]).successful)

    def test_cancel_during_extra_wait_finishes_immediately_and_records_actual_wait(self):
        node, now = self.waiting_node()
        Search.tick(node)
        now.nanoseconds = 7_500_000_000
        with TemporaryDirectory() as output_dir:
            node.param = dict(scene_id='test', output_dir=output_dir).__getitem__
            node.rooms = {}
            node.landmarks = SimpleNamespace(confirmed=[])
            node.task_id, node.command_text = 'test', 'test'
            node.visited, node.visited_poses = 2, node.views
            node.started, node.started_wall = 0, 0
            node.publish_markers, node.results = Mock(), Mock()
            node.get_logger, node.grid, node.observations = lambda: Mock(), None, []
            node.finish = MethodType(Search.finish, node)
            Search.on_command(node, SimpleNamespace(data='取消'))
            result = json.loads(node.results.publish.call_args.args[0].data)
            self.assertEqual(result['status'], 'canceled')
            self.assertEqual(result['viewpoint_diagnostics'][1]['extra_wait_seconds'], 2.5)
            self.assertFalse(result['viewpoint_diagnostics'][1]['waiting_for_sensor_data'])
            self.assertEqual(node.phase, 'idle')

    def test_navigation_arrival_only_becomes_observation_complete_after_dwell(self):
        now = SimpleNamespace(nanoseconds=1_000_000_000)
        node = SimpleNamespace(view=0, views=[[0., 0., 0.]], visited=0, visited_poses=[],
                               viewpoint_diagnostics=[viewpoint()], canceled=False, goal=None,
                               get_clock=lambda: SimpleNamespace(now=lambda: now),
                               param=lambda key: 4., next_view=Mock(), tf=Mock(), publish_progress=Mock())
        node.tf.lookup_transform.return_value = SimpleNamespace(
            transform=SimpleNamespace(translation=SimpleNamespace(x=.2, y=.3, z=.1),
                                      rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)),
            header=SimpleNamespace(stamp=TimeMessage(sec=0, nanosec=900_000_000)))
        Search.on_navigation(node, SimpleNamespace(result=lambda: SimpleNamespace(status=4)))
        self.assertTrue(node.viewpoint_diagnostics[0]['reached'])
        self.assertFalse(node.viewpoint_diagnostics[0]['dwell_completed'])
        self.assertEqual(node.viewpoint_diagnostics[0]['arrival_map_pose']['xyz'], [.2, .3, .1])
        self.assertEqual(node.viewpoint_diagnostics[0]['arrival_sim_seconds'], 1.)
        self.assertEqual(node.viewpoint_diagnostics[0]['arrival_pose_lookup'], 'latest')
        self.assertEqual(node.viewpoint_diagnostics[0]['arrival_transform_stamp_ns'], 900_000_000)
        node.viewpoint_diagnostics[0]['synchronized_frames'] = 1
        now.nanoseconds = 4_900_000_000
        Search.tick(node)
        self.assertFalse(node.viewpoint_diagnostics[0]['dwell_completed'])
        now.nanoseconds = 5_000_000_000
        Search.tick(node)
        self.assertTrue(node.viewpoint_diagnostics[0]['dwell_completed'])
        node.next_view.assert_called_once()

    def test_arrival_pose_lookup_failure_does_not_undo_successful_navigation(self):
        node = SimpleNamespace(view=0, views=[[0., 0., 0.]], visited=0, visited_poses=[],
                               viewpoint_diagnostics=[viewpoint()], canceled=False, goal=None,
                               get_clock=lambda: SimpleNamespace(
                                   now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)), tf=Mock(),
                               param=lambda key: 4., publish_progress=Mock())
        node.tf.lookup_transform.side_effect = TransformException('No map transform')
        Search.on_navigation(node, SimpleNamespace(result=lambda: SimpleNamespace(status=4)))
        self.assertEqual(node.phase, 'observing')
        self.assertTrue(node.viewpoint_diagnostics[0]['reached'])
        self.assertIsNone(node.viewpoint_diagnostics[0]['arrival_map_pose'])
        self.assertEqual(node.viewpoint_diagnostics[0]['arrival_pose_error'], 'No map transform')

    def test_task_cancel_racing_navigation_result_does_not_report_navigation_failure(self):
        for status in [5, 4, 6]:
            with self.subTest(status=status):
                node = SimpleNamespace(view=0, viewpoint_diagnostics=[viewpoint()],
                                       canceled=True, goal=None, finish=Mock())
                Search.on_navigation(node, SimpleNamespace(result=lambda: SimpleNamespace(status=status)))
                node.finish.assert_called_once_with('canceled')
                self.assertTrue(node.viewpoint_diagnostics[0]['task_canceled'])
                self.assertEqual(observation_coverage(node.viewpoint_diagnostics)['incomplete_viewpoints'],
                                 [dict(index=0, reasons=['canceled'])])

    def test_fresh_empty_frames_rejections_and_target_failures_stay_at_their_view(self):
        node = SimpleNamespace(phase='observing', observe_start=1_000_000_000, last_stamp=-1,
                               observe_dwell_seconds=4., observe_extra_seconds=4.,
                               get_clock=lambda: SimpleNamespace(
                                   now=lambda: SimpleNamespace(nanoseconds=2_000_000_000)),
                               view=1, viewpoint_diagnostics=[viewpoint(), viewpoint(1)],
                               frames=0, valid_projection=0, tf_failures=0, invalid_target_depth=0,
                               task=('count', 'kitchen', 'cup'), param=lambda key: .35)
        node.record_observation = MethodType(Search.record_observation, node)
        detections = SimpleNamespace(header=SimpleNamespace(stamp=TimeMessage(sec=2)),
                                     detections=[])
        depth = SimpleNamespace(width=640, height=480, encoding='32FC1')
        info = SimpleNamespace(width=640, height=480, k=[500., 0., 320., 0., 500., 240., 0., 0., 1.])
        Search.on_observation(node, detections, depth, info)
        Search.on_observation(node, detections, depth, info)  # Same stamp must not inflate counts.
        self.assertEqual(node.frames, 1)
        self.assertEqual(node.viewpoint_diagnostics[0]['synchronized_frames'], 0)
        self.assertEqual(node.viewpoint_diagnostics[1]['synchronized_frames'], 1)
        detections.header.stamp.sec = 3
        depth.width = 320
        Search.on_observation(node, detections, depth, info)
        self.assertEqual(node.frames, 1)
        self.assertEqual(node.viewpoint_diagnostics[1]['synchronized_messages'], 2)
        self.assertEqual(node.viewpoint_diagnostics[1]['rejected_sensor_frames'], 1)
        for field, aggregate in [('tf_failures', 'tf_failures'),
                                 ('invalid_target_depth', 'invalid_target_depth'),
                                 ('valid_projections', 'valid_projection')]:
            node.record_observation(field)
            self.assertEqual(getattr(node, aggregate), 1)
            self.assertEqual(node.viewpoint_diagnostics[0][field], 0)
            self.assertEqual(node.viewpoint_diagnostics[1][field], 1)

    def test_route_end_uses_per_view_data_even_when_task_total_contains_frames(self):
        node = SimpleNamespace(view=2, views=[[0., 0., 0.], [1., 0., 0.]],
                               task=('count', 'kitchen', 'cup'), frames=5,
                               landmarks=SimpleNamespace(confirmed=[object()]),
                               valid_projection=2, tf_failures=0, invalid_target_depth=0,
                               failed_views=[], finish=Mock(), viewpoint_diagnostics=[
                                   viewpoint(reached=True, dwell_completed=True,
                                             synchronized_frames=5, valid_projections=2),
                                   viewpoint(1, reached=True, dwell_completed=True)])
        Search.next_view(node)
        node.finish.assert_called_once_with('partial')

    def test_zero_or_negative_image_extent_cannot_complete_an_otherwise_observed_route(self):
        for width, height in [(0, 0), (0, 480), (640, 0), (-1, 480), (640, -1)]:
            with self.subTest(width=width, height=height):
                node = SimpleNamespace(
                    phase='observing', observe_start=1_000_000_000, last_stamp=-1,
                    observe_dwell_seconds=4., observe_extra_seconds=4.,
                    get_clock=lambda: SimpleNamespace(
                        now=lambda: SimpleNamespace(nanoseconds=2_000_000_000)),
                    view=0, views=[[0., 0., 0.], [1., 0., 0.]],
                    viewpoint_diagnostics=[viewpoint(reached=True, dwell_completed=True),
                                           viewpoint(1, reached=True, dwell_completed=True)],
                    frames=0, valid_projection=0, tf_failures=0, invalid_target_depth=0,
                    task=('count', 'kitchen', 'cup'), param=lambda key: .35,
                    landmarks=SimpleNamespace(confirmed=[]), failed_views=[], finish=Mock())
                node.record_observation = MethodType(Search.record_observation, node)
                detections = SimpleNamespace(header=SimpleNamespace(stamp=TimeMessage(sec=2)),
                                             detections=[])
                depth = SimpleNamespace(width=640, height=480, encoding='32FC1')
                info = SimpleNamespace(width=640, height=480,
                                       k=[500., 0., 320., 0., 500., 240., 0., 0., 1.])
                Search.on_observation(node, detections, depth, info)
                node.view = 1
                detections.header.stamp.sec = 3
                depth.width = info.width = width
                depth.height = info.height = height
                Search.on_observation(node, detections, depth, info)
                self.assertEqual(node.frames, 1)
                self.assertEqual(node.viewpoint_diagnostics[0]['synchronized_frames'], 1)
                self.assertEqual(node.viewpoint_diagnostics[1]['synchronized_frames'], 0)
                self.assertEqual(node.viewpoint_diagnostics[1]['rejected_sensor_frames'], 1)
                node.view = 2
                Search.next_view(node)
                node.finish.assert_called_once_with('partial')

    def test_cancel_at_last_arrival_serializes_unfinished_observation(self):
        with TemporaryDirectory() as output_dir:
            params = dict(scene_id='test', output_dir=output_dir)
            node = SimpleNamespace(task=('count', 'kitchen', 'cup'), phase='observing', view=0,
                                   landmarks=SimpleNamespace(confirmed=[]), request_id=None, publish_progress=Mock(),
                                   param=params.__getitem__, task_id='test', command_text='test',
                                   visited=1, views=[[0., 0., 0.]], frames=0, tf_failures=0,
                                   invalid_target_depth=0, valid_projection=0,
                                   visited_poses=[[0., 0., 0.]], failed_views=[],
                                   viewpoint_diagnostics=[viewpoint(reached=True, navigation_status=4)],
                                   get_clock=lambda: SimpleNamespace(
                                       now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
                                   started=0, started_wall=0, publish_markers=Mock(),
                                   results=Mock(), get_logger=lambda: Mock(), grid=None, observations=[])
            Search.finish(node, 'canceled')
            result = json.loads(node.results.publish.call_args.args[0].data)
            self.assertEqual(result['status'], 'canceled')
            self.assertTrue(result['coverage']['arrival_plan_completed'])
            self.assertFalse(result['coverage']['observation_plan_completed'])
            self.assertFalse(result['coverage']['observation_data_complete'])
            self.assertEqual(result['coverage']['incomplete_viewpoints'],
                             [dict(index=0, reasons=['dwell_incomplete', 'no_sensor_data'])])
            self.assertEqual(result['viewpoint_diagnostics'][0]['synchronized_frames'], 0)


if __name__ == '__main__':
    unittest.main()
