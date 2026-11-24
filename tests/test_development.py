import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

from semantic_search.development import add_targets, apply_sensor_rig, rig_frames, add_contact_diagnostics
from semantic_search.rooms import room_at

ROOT = Path(__file__).resolve().parents[1]


class DevelopmentTests(unittest.TestCase):
    def test_contact_diagnostics_reference_existing_collisions_without_changing_physics(self):
        robot=ET.fromstring('<model><link name="base"><collision name="body"><geometry><box><size>1 1 1</size></box></geometry></collision></link><link name="visual_only"/></model>')
        before=ET.tostring(robot.find('link/collision'))
        add_contact_diagnostics(robot)
        sensor=robot.find('link/sensor')
        self.assertEqual(sensor.findtext('contact/collision'),'body')
        self.assertEqual(sensor.findtext('contact/topic'),'/robot/contacts')
        self.assertEqual(ET.tostring(robot.find('link/collision')),before)
        self.assertIsNone(robot.find("link[@name='visual_only']/sensor"))

    def test_ycb_texture_filename_whitespace_is_adapted_without_changing_source(self):
        from semantic_search.development import prepare_obj
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);mesh=folder/'textured.obj';material=folder/'textured.mtl'
            mesh.write_text('mtllib textured.mtl  \nv 0 0 0\n')
            material.write_text('newmtl material_0\nmap_Kd texture_map.png  \n')
            (folder/'texture_map.png').write_bytes(b'texture')
            derived=prepare_obj(mesh,folder,folder/'cache')
            material_name=derived.read_text().splitlines()[0].split(maxsplit=1)[1]
            texture_name=(derived.parent/material_name).read_text().splitlines()[1].split(maxsplit=1)[1]
            self.assertEqual((derived.parent/texture_name).read_bytes(),b'texture')
            self.assertTrue(material.read_text().endswith('texture_map.png  \n'))

    def test_multiple_obj_assets_with_same_names_keep_distinct_materials_and_textures(self):
        from semantic_search.development import prepare_obj
        with tempfile.TemporaryDirectory() as temporary:
            folder=Path(temporary);records=[]
            for name in ('apple','bowl'):
                source=folder/name;source.mkdir()
                (source/'model.obj').write_text('mtllib model.mtl\nusemtl material_0\nv 0 0 0\n')
                (source/'model.mtl').write_text('newmtl material_0\nmap_Kd texture.png\n')
                (source/'texture.png').write_bytes(name.encode())
                derived=prepare_obj(source/'model.obj',source,folder/'cache')
                lines=derived.read_text().splitlines()
                material=lines[1].split(maxsplit=1)[1]
                mtl=(derived.parent/lines[0].split(maxsplit=1)[1]).read_text().splitlines()
                self.assertEqual(mtl[0],'newmtl '+material)
                texture=mtl[1].split(maxsplit=1)[1]
                self.assertEqual((derived.parent/texture).read_bytes(),name.encode())
                records.append((material,texture,derived.name))
            for a,b in zip(records[0],records[1]):
                self.assertNotEqual(a,b)

    def test_phone_adaptation_retains_geometry_without_subt_or_thermal_extras(self):
        world=ET.Element('world')
        original=(ROOT/'third_party/target_assets/mobile_phone/model.sdf').read_bytes()
        layout=dict(targets=[dict(id='phone',asset='mobile_phone',category='cell phone',pose=[0,0,.9,0,0,0])])
        with tempfile.TemporaryDirectory() as cache:
            add_targets(world,ROOT,layout,cache)
        model=world.find('model')
        self.assertTrue(model.findall('.//collision/geometry/mesh/uri'))
        self.assertTrue(model.findall('.//visual/geometry/mesh/uri'))
        self.assertFalse(model.findall('include'))
        self.assertFalse(model.findall('.//visual/plugin'))
        self.assertEqual((ROOT/'third_party/target_assets/mobile_phone/model.sdf').read_bytes(),original)

    def test_additional_furniture_reuses_existing_project_asset(self):
        world=ET.Element('world')
        directory='third_party/aws_small_house/models/aws_robomaker_residential_ChairA_01'
        layout=dict(assets={'extra_chair':dict(directory=directory)},targets=[
            dict(id='validation_chair',asset='extra_chair',category='chair',pose=[-4,-3,0,0,0,0])])
        with tempfile.TemporaryDirectory() as cache:
            truth=add_targets(world,ROOT,layout,cache)
        self.assertEqual(truth,layout['targets'])
        self.assertEqual(world.find('model').get('name'),'validation_chair')
        self.assertTrue(all(Path(uri.text).is_file() for uri in world.findall('.//mesh/uri')))

    def test_depth_obstacles_keep_original_planning_and_control_algorithms(self):
        config = ROOT/'src/semantic_search/config'
        original = yaml.safe_load((config/'nav2.yaml').read_text())
        candidate = yaml.safe_load((config/'nav2_depth.yaml').read_text())
        self.assertEqual(original['planner_server'],candidate['planner_server'])
        # Progress and cleaning arrival checks are now explicit service fixes;
        # DWB and ordinary arrival tolerances stay. C040 explicitly raises
        # only the development linear speed cap; the original profile remains.
        old=original['controller_server']['ros__parameters'].copy()
        new=candidate['controller_server']['ros__parameters'].copy()
        for key in ('goal_checker_plugins','progress_checker','cleaning_goal_checker','docking_goal_checker','controller_plugins','CleaningPath','DockingPath'):
            old.pop(key,None);new.pop(key,None)
        ordinary=new['FollowPath'].copy()
        self.assertEqual(ordinary['plugin'],'nav2_rotation_shim_controller::RotationShimController')
        self.assertEqual(ordinary['primary_controller'],old['FollowPath']['plugin'])
        ordinary['plugin']=ordinary.pop('primary_controller')
        for key in ('max_vel_x','max_speed_xy'):
            self.assertEqual(ordinary[key],.35)
            ordinary[key]=old['FollowPath'][key]
        for key in ('angular_dist_threshold','angular_disengage_threshold','forward_sampling_distance',
                    'rotate_to_heading_angular_vel','max_angular_accel','simulate_ahead_time',
                    'rotate_to_goal_heading','closed_loop'):
            ordinary.pop(key,None)
        new['FollowPath']=ordinary
        self.assertEqual(old,new)
        for namespace in ['local_costmap','global_costmap']:
            costmap=candidate[namespace][namespace]['ros__parameters']
            self.assertEqual(costmap['plugins'],['static_layer','laser_layer','depth_layer','inflation_layer'])
            self.assertEqual(costmap['laser_layer']['observation_sources'],'scan')
            self.assertEqual(costmap['depth_layer']['observation_sources'],'depth depth_clear')
            self.assertEqual(costmap['laser_layer']['combination_method'],1)
            self.assertEqual(costmap['depth_layer']['combination_method'],1)
            self.assertGreaterEqual(costmap['robot_radius'],(.1965**2+.1325**2)**.5)
            p=costmap['depth_layer']
            self.assertEqual(p['depth']['topic'],'/rgbd/obstacles')
            self.assertEqual(p['depth']['data_type'],'PointCloud2')
            self.assertGreater(p['depth']['min_obstacle_height'],0)
            self.assertTrue(p['depth']['marking']);self.assertFalse(p['depth']['clearing'])
            clear=p['depth_clear']
            self.assertEqual(clear['topic'],'/rgbd/clearing')
            self.assertFalse(clear['marking']);self.assertTrue(clear['clearing'])
            self.assertEqual(clear['min_obstacle_height'],-8.)
            self.assertEqual(clear['max_obstacle_height'],8.)
            self.assertEqual(clear['raytrace_max_range'],4.)
            self.assertEqual(clear['expected_update_rate'],.4)
            self.assertEqual(p['depth']['min_obstacle_height'],.08)
            self.assertEqual(p['depth']['max_obstacle_height'],1.1)

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
