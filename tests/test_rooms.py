import json
from pathlib import Path
import unittest

from semantic_search.rooms import (
    belongs_to, contains, room_at, validate_rooms, observation_views, observation_status,
    observation_coverage,
)

ROOT = Path(__file__).resolve().parents[1]


class RoomTests(unittest.TestCase):
    @staticmethod
    def viewpoint(index=0, **changes):
        view = dict(index=index, reached=True, navigation_status=4, dwell_completed=True,
                    synchronized_frames=5, valid_projections=2, tf_failures=0, invalid_target_depth=0)
        view.update(changes)
        return view

    def test_one_view_without_data_cannot_complete_count_or_report_absence(self):
        views = [self.viewpoint(), self.viewpoint(1, synchronized_frames=0, valid_projections=0)]
        coverage = observation_coverage(views)
        self.assertTrue(coverage['arrival_plan_completed'])
        self.assertTrue(coverage['observation_plan_completed'])
        self.assertFalse(coverage['observation_data_complete'])
        self.assertFalse(coverage['full_room_coverage_verified'])
        self.assertEqual(coverage['viewpoints_observed'], 1)
        self.assertEqual(coverage['incomplete_viewpoints'], [dict(index=1, reasons=['no_sensor_data'])])
        self.assertEqual(observation_status('count', 5, True, 2, 0, 0, [], views), 'partial')
        views[0]['valid_projections'] = 0
        self.assertEqual(observation_status('count', 5, False, 0, 0, 0, [], views), 'partial')
        self.assertEqual(observation_status('find', 5, False, 0, 0, 0, [], views), 'partial')

    def test_arrival_or_early_find_is_not_completed_observation(self):
        views = [self.viewpoint(dwell_completed=False)]
        coverage = observation_coverage(views)
        self.assertTrue(coverage['arrival_plan_completed'])
        self.assertFalse(coverage['observation_plan_completed'])
        self.assertTrue(coverage['observation_data_complete'])
        self.assertEqual(coverage['incomplete_viewpoints'], [dict(index=0, reasons=['dwell_incomplete'])])
        self.assertEqual(observation_status('find', 5, True, 2, 0, 0, [], views), 'found')
        self.assertEqual(observation_status('count', 5, True, 2, 0, 0, [], views), 'partial')

    def test_all_reached_views_without_data_is_no_sensor_data_even_with_unreachable_view(self):
        failed = [dict(index=0, navigation_status=6)]
        views = [self.viewpoint(reached=False, dwell_completed=False, navigation_status=6,
                                synchronized_frames=0, valid_projections=0),
                 self.viewpoint(1, synchronized_frames=0, valid_projections=0)]
        self.assertEqual(observation_status('count', 0, False, 0, 0, 0, failed, views), 'no_sensor_data')
        self.assertEqual(observation_status('count', 0, False, 0, 0, 0, failed, views[:1]),
                         'navigation_failed')
        self.assertFalse(observation_coverage(views)['observation_plan_completed'])

    def test_local_projection_failures_cannot_hide_behind_other_view(self):
        for error in ['tf_failures', 'invalid_target_depth']:
            with self.subTest(error=error):
                views = [self.viewpoint(), self.viewpoint(1, valid_projections=0, **{error: 5})]
                coverage = observation_coverage(views)
                self.assertFalse(coverage['observation_data_complete'])
                self.assertEqual(coverage['incomplete_viewpoints'],
                                 [dict(index=1, reasons=['localization_failed'])])
                self.assertEqual(observation_status('count', 10, True, 2, 5, 0, [], views), 'partial')

    def test_empty_detection_frames_are_valid_observations_and_empty_plan_is_not_complete(self):
        views = [self.viewpoint(valid_projections=0)]
        coverage = observation_coverage(views)
        self.assertTrue(coverage['observation_plan_completed'])
        self.assertTrue(coverage['observation_data_complete'])
        self.assertEqual(observation_status('count', 5, False, 0, 0, 0, [], views), 'not_found')
        self.assertEqual(observation_status('count', 5, True, 2, 0, 0, [], [self.viewpoint()]), 'completed')
        self.assertFalse(observation_coverage([])['arrival_plan_completed'])
        self.assertFalse(observation_coverage([])['observation_plan_completed'])
        self.assertFalse(observation_coverage([])['observation_data_complete'])

    def test_unreached_navigation_results_distinguish_cancel_abort_and_unknown_arrival(self):
        for status, task_canceled, reason in [
                (None, False, 'not_visited'), (4, False, 'not_visited'),
                (5, False, 'canceled'), (4, True, 'canceled'),
                (6, True, 'canceled'), (6, False, 'navigation_failed')]:
            with self.subTest(status=status, task_canceled=task_canceled):
                coverage = observation_coverage([self.viewpoint(
                    reached=False, dwell_completed=False, navigation_status=status,
                    task_canceled=task_canceled, synchronized_frames=0, valid_projections=0)])
                self.assertEqual(coverage['incomplete_viewpoints'], [dict(index=0, reasons=[reason])])
                self.assertFalse(coverage['observation_plan_completed'])

    def test_unreachable_view_is_not_reported_as_absence_or_total_count(self):
        failed=[{'index':0,'navigation_status':6}]
        self.assertEqual(observation_status('count',0,False,0,0,0,failed),'navigation_failed')
        self.assertEqual(observation_status('count',5,True,5,0,0,failed),'partial')
        self.assertEqual(observation_status('count',5,False,0,0,0,failed),'partial')
        self.assertEqual(observation_status('find',5,True,5,0,0,failed),'found')
        self.assertEqual(observation_status('count',5,True,5,0,0,[]),'completed')
        self.assertEqual(observation_status('count',5,False,0,1,0,[]),'localization_failed')

    def test_tabletop_route_does_not_replace_legacy_or_chair_route(self):
        room=dict(views=[[0,0,0]],category_views={'cup':[[1,1,1]]})
        self.assertEqual(observation_views(room,'cup'),[[1,1,1]])
        self.assertEqual(observation_views(room,'chair'),[[0,0,0]])
        self.assertEqual(observation_views({'views':[[2,2,2]]},'cup'),[[2,2,2]])

    def test_concave_room_excludes_building_notch(self):
        rooms = json.loads((ROOT/'src/semantic_search/config/rooms_aws_house_v1.json').read_text())
        validate_rooms(rooms)
        self.assertTrue(belongs_to(rooms, 'kitchen', 7, 1))
        self.assertFalse(belongs_to(rooms, 'kitchen', 8, -5.3))
        self.assertIsNone(room_at(rooms, float('nan'), 0))

    def test_shared_boundary_has_unique_owner_independent_of_order(self):
        rooms = dict(b={'bounds':[1,0,2,1], 'polygon':[[1,0],[2,0],[2,1],[1,1]]},
                     a={'bounds':[0,0,1,1], 'polygon':[[0,0],[1,0],[1,1],[0,1]]})
        self.assertEqual(room_at(rooms, 1, .5), 'a')
        self.assertEqual(room_at(dict(reversed(list(rooms.items()))), 1, .5), 'a')
        self.assertFalse(belongs_to(rooms, 'b', 1, .5))

    def test_holes_exclude_boundary_and_interior(self):
        room = dict(bounds=[0,0,4,4], polygon=[[0,0],[4,0],[4,4],[0,4]],
                    holes=[[[1,1],[2,1],[2,2],[1,2]]])
        self.assertFalse(contains(room, 1, 1.5))
        self.assertFalse(contains(room, 1.5, 1.5))
        self.assertTrue(contains(room, 3, 3))

    def test_legacy_rectangle_preserves_inclusive_boundary(self):
        rooms = dict(a={'bounds':[0,0,1,1]}, b={'bounds':[1,0,2,1]})
        self.assertTrue(belongs_to(rooms, 'a', 1, .5))
        self.assertTrue(belongs_to(rooms, 'b', 1, .5))

    def test_self_intersection_rejected(self):
        with self.assertRaises(ValueError):
            validate_rooms({'bad':dict(bounds=[0,0,4,4], views=[],
                polygon=[[0,0],[4,3],[0,4],[3,0]])})


if __name__ == '__main__':
    unittest.main()
