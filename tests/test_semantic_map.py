import json
from math import pi, sin, cos
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest
from xml.etree import ElementTree as ET

from semantic_search.semantic_map import LandmarkMap, parse_command, write_results


class AssociationTests(unittest.TestCase):
    def test_repeated_views_do_not_inflate_count(self):
        landmarks = LandmarkMap(.3, 2)
        for x in [1.0, 1.04, .97, 1.03, .99]:
            landmarks.observe([(x, 2.0, .4)])
        self.assertEqual(len(landmarks.confirmed), 1)
        self.assertEqual(landmarks.confirmed[0].hits, 5)

    def test_close_simultaneous_instances_stay_separate(self):
        landmarks = LandmarkMap(.3, 2)
        landmarks.observe([(1., 2., .4), (1.2, 2., .4)])
        ids = landmarks.observe([(1.19, 2., .4), (1.01, 2., .4)])
        self.assertEqual(ids, [2, 1])
        self.assertEqual(len(landmarks.confirmed), 2)
        self.assertEqual([item.hits for item in landmarks.items], [2, 2])

    def test_assignment_does_not_create_track_when_both_can_match(self):
        landmarks = LandmarkMap(.3, 2)
        landmarks.observe([(0., 0., .4), (.35, 0., .4)])
        # Greedy nearest-first steals the only match available to the left point.
        landmarks.observe([(.1, 0., .4), (-.2, 0., .4)])
        self.assertEqual(len(landmarks.items), 2)
        self.assertEqual(len(landmarks.confirmed), 2)

    def test_one_frame_cannot_confirm_or_double_update_a_track(self):
        landmarks = LandmarkMap(.3, 3)
        landmarks.observe([(1., 2., .4)])
        landmarks.observe([(1., 2., .4), (1.1, 2., .4)])
        self.assertEqual(max(item.hits for item in landmarks.items), 2)
        self.assertEqual(landmarks.confirmed, [])

    def test_new_task_has_fresh_ids(self):
        old, new = LandmarkMap(.3, 1), LandmarkMap(.3, 1)
        old.observe([(1., 1., 0.), (2., 2., 0.)])
        new.observe([(3., 3., 0.)])
        self.assertEqual([item.id for item in new.confirmed], [1])


class OutputTests(unittest.TestCase):
    def test_rotated_map_coordinates_and_json(self):
        grid = NS(info=NS(width=20, height=20, resolution=.5, origin=NS(
            position=NS(x=1., y=2.), orientation=NS(x=0., y=0., z=sin(pi/4), w=cos(pi/4)))),
            data=[0]*400)
        result = {'frame_id': 'map', 'coordinate_unit': 'm',
                  'objects': [{'id': 7, 'x': 1., 'y': 3., 'z': .4}]}
        with TemporaryDirectory() as folder:
            write_results(folder, result, grid)
            self.assertEqual(json.loads((Path(folder)/'result.json').read_text()), result)
            root = ET.parse(Path(folder)/'semantic_map.svg').getroot()
            circle = root.find('{*}circle')
            self.assertAlmostEqual(float(circle.get('cx')), 2.)
            self.assertAlmostEqual(float(circle.get('cy')), 20.)
            self.assertEqual(root.find('{*}text').text, '7')

    def test_constrained_command_parsing(self):
        rooms = {'kitchen': {'aliases': ['厨房', 'kitchen']},
                 'living': {'aliases': ['客厅', 'living room']}}
        self.assertEqual(parse_command('去厨房找椅子', rooms), ('find', 'kitchen', 'chair'))
        self.assertEqual(parse_command('count chairs in kitchen', rooms), ('count', 'kitchen', 'chair'))
        self.assertEqual(parse_command('取消', rooms), ('cancel', None, None))
        with self.assertRaises(ValueError):
            parse_command('去厨房和客厅找椅子', rooms)

    def test_structured_task_has_the_same_execution_fields_as_text(self):
        rooms = {'kitchen': {'aliases': ['厨房', 'kitchen']}}
        for mode, text in [('find', '去厨房找椅子'), ('count', '去厨房统计椅子')]:
            task = json.dumps(dict(mode=mode, room='kitchen', category='chair'))
            self.assertEqual(parse_command(task, rooms), parse_command(text, rooms))
        self.assertEqual(parse_command('{"mode":"cancel"}', rooms), ('cancel', None, None))

    def test_structured_task_rejects_unsupported_or_mistyped_fields(self):
        rooms = {'kitchen': {'aliases': ['厨房', 'kitchen']}}
        invalid = [
            {'mode': 'drive', 'room': 'kitchen', 'category': 'chair'},
            {'mode': 'find', 'room': 'unknown', 'category': 'chair'},
            {'mode': 'find', 'room': 'kitchen', 'category': 'cola_brand'},
            {'mode': 'find', 'room': ['kitchen'], 'category': 'chair'},
            {'mode': 'find', 'room': 'kitchen', 'category': {'name': 'chair'}},
            {'mode': 'find', 'room': 'kitchen'},
            {'mode': 'find', 'room': 'kitchen', 'category': 'chair', 'speed': 2},
            {'mode': 'cancel', 'room': ['kitchen']},
        ]
        for task in invalid:
            with self.subTest(task=task), self.assertRaises(ValueError):
                parse_command(json.dumps(task), rooms)

    def test_malformed_structured_task_is_not_treated_as_text(self):
        with self.assertRaises(ValueError):
            parse_command('{"mode":"count", "room":"kitchen", "category":"chair"',
                          {'kitchen': {'aliases': ['厨房', 'kitchen']}})


if __name__ == '__main__':
    unittest.main()
