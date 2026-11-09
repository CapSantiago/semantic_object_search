import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from semantic_search.aws_world import import_aws_house
from semantic_search.scenes import load_scene

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/semantic_search/config'


class SceneTests(unittest.TestCase):
    def test_scene_configuration_cannot_mix_maps_rooms_or_results(self):
        baseline = load_scene('baseline', ROOT, CONFIG)
        house = load_scene('aws_house', ROOT, CONFIG)
        for key in ('world', 'map', 'rooms', 'route', 'output'):
            self.assertNotEqual(baseline[key], house[key])
        self.assertEqual(baseline.get('detector_iou', .5), .5)
        self.assertEqual(house['detector_iou'], .3)
        self.assertEqual(json.loads(Path(baseline['rooms']).read_text()),
                         json.loads((CONFIG / 'rooms.json').read_text()))
        with self.assertRaises(ValueError):
            load_scene('typo', ROOT, CONFIG)

    def test_accepted_baseline_is_byte_for_byte_preserved(self):
        manifest = json.loads((ROOT / 'scenes/baseline/preservation.json').read_text())
        for filename, expected in manifest['sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT / filename).read_bytes()).hexdigest(), expected, filename)
        for extension in ('yaml', 'pgm'):
            self.assertEqual((ROOT / f'scenes/baseline/maps/course_house.{extension}').read_bytes(),
                             (ROOT / f'maps/course_house.{extension}').read_bytes())

    def test_aws_adapter_preserves_instances_and_repairs_all_texture_paths(self):
        assets = ROOT / 'third_party/aws_small_house'
        world = ET.Element('world', name='aws_house')
        ET.SubElement(world, 'light', name='sun', type='directional')
        with tempfile.TemporaryDirectory() as cache:
            truth = import_aws_house(world, assets, cache)
            self.assertEqual(len(truth), 87)
            self.assertEqual(sum('Chair' in t['asset'] for t in truth), 8)
            self.assertEqual(len({m.get('name') for m in world.findall('model')}), 87)
            lights = [light.get('name') for light in world.findall('light')]
            self.assertEqual(len(lights), 9)
            self.assertEqual(len(set(lights)), len(lights))
            self.assertFalse(world.findall('.//inertial'))
            self.assertFalse(world.findall('.//visual/meta'))
            self.assertFalse(world.find("model[@name='FloorB_01_001']").findall('.//collision'))
            ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
            for mesh in world.findall('.//mesh/uri'):
                self.assertTrue(Path(mesh.text).is_file(), mesh.text)
                for texture in ET.parse(mesh.text).findall('.//c:image/c:init_from', ns):
                    self.assertTrue(Path(texture.text).is_file(), texture.text)


if __name__ == '__main__':
    unittest.main()
