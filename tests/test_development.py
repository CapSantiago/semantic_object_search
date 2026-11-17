import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

from semantic_search.development import add_targets, apply_sensor_rig, rig_frames
from semantic_search.rooms import room_at

ROOT = Path(__file__).resolve().parents[1]


class DevelopmentTests(unittest.TestCase):
    def test_depth_obstacles_keep_original_planning_and_control_algorithms(self):
        config = ROOT/'src/semantic_search/config'
        original = yaml.safe_load((config/'nav2.yaml').read_text())
        candidate = yaml.safe_load((config/'nav2_depth.yaml').read_text())
        self.assertEqual(original['planner_server'],candidate['planner_server'])
        self.assertEqual(original['controller_server'],candidate['controller_server'])
        for namespace,layer in [('local_costmap','voxel_layer'),('global_costmap','obstacle_layer')]:
            p=candidate[namespace][namespace]['ros__parameters'][layer]
            self.assertIn('depth',p['observation_sources'].split())
            self.assertEqual(p['depth']['topic'],'/rgbd/obstacles')
            self.assertEqual(p['depth']['data_type'],'PointCloud2')
            self.assertGreater(p['depth']['min_obstacle_height'],0)

    def test_sensor_transform_and_physical_components_agree(self):
        robot = ET.fromstring('<model><link name="base_link"><visual name="rgbd_mast"/></link>'
                              '<link name="camera_rgb_frame"><pose>0 0 0 0 0 0</pose></link></model>')
        apply_sensor_rig(robot)
        description = ET.parse(ROOT/'src/semantic_search/config/tb3_frames.urdf').getroot()
        rig_frames(description)
        xyz = [float(v) for v in description.find("joint[@name='camera_joint']/origin").get('xyz').split()]
        rpy = [float(v) for v in description.find("joint[@name='camera_joint']/origin").get('rpy').split()]
        self.assertEqual(xyz+rpy, [float(v) for v in robot.find("link[@name='camera_rgb_frame']/pose").text.split()])
        for name in ['camera_rgb_frame', 'rgbd_support']:
            link = robot.find(f"link[@name='{name}']")
            self.assertTrue(link.findall('collision'))
            self.assertGreater(float(link.findtext('inertial/mass')), 0)
            for axis in ['ixx','iyy','izz']:
                self.assertGreater(float(link.findtext('inertial/inertia/'+axis)), 0)
        self.assertFalse(robot.findall("link/visual[@name='rgbd_mast']"))
        mast_x = float(robot.find("link[@name='rgbd_support']/pose").text.split()[0])
        self.assertLess(mast_x+.025/2, xyz[0]-.02)

    def test_r0_targets_use_local_assets_and_calibrated_support_heights(self):
        layout = json.loads((ROOT/'scenes/aws_house/layout_r0_v1.json').read_text())
        rooms = json.loads((ROOT/'src/semantic_search/config/rooms_aws_house_v1.json').read_text())
        world = ET.Element('world')
        with tempfile.TemporaryDirectory() as cache:
            add_targets(world, ROOT, layout, cache)
            self.assertEqual(len(world.findall('model')), 6)
            for target, model in zip(layout['targets'], world.findall('model')):
                self.assertEqual(room_at(rooms, *target['pose'][:2]), target['room'])
                bottom = target['pose'][2]+layout['assets'][target['asset']]['min'][2]
                self.assertAlmostEqual(bottom-target['support_z_m'], .001)
                self.assertEqual(model.findtext('static'), 'true')
                for uri in model.findall('.//mesh/uri'):
                    path = Path(uri.text)
                    self.assertTrue(path.is_file())
                    if path.suffix == '.obj':
                        mtl = next(line.split(maxsplit=1)[1] for line in path.read_text().splitlines()
                                   if line.startswith('mtllib '))
                        material = path.parent/mtl
                        for line in material.read_text().splitlines():
                            if line.startswith('map_'):
                                self.assertTrue((material.parent/line.split(maxsplit=1)[1]).is_file())


if __name__ == '__main__':
    unittest.main()
