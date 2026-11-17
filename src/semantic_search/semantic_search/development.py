"""Opt-in R0 layout and complete RGB-D rig; archived scenes stay separate."""
from copy import deepcopy
import json
import hashlib
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


def _box(link, name, xyz, size, color):
    for kind in ('visual', 'collision'):
        part = ET.SubElement(link, kind, name=name+'_'+kind)
        ET.SubElement(part, 'pose').text = ' '.join(map(str, xyz))+' 0 0 0'
        geometry = ET.SubElement(part, 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = ' '.join(map(str, size))
        if kind == 'visual':
            ET.SubElement(ET.SubElement(part, 'material'), 'diffuse').text = color


def _inertia(link, mass, size, center=(0, 0, 0)):
    for old in link.findall('inertial'):
        link.remove(old)
    item = ET.SubElement(link, 'inertial')
    ET.SubElement(item, 'pose').text = ' '.join(map(str, center))+' 0 0 0'
    ET.SubElement(item, 'mass').text = str(mass)
    tensor = ET.SubElement(item, 'inertia')
    x, y, z = size
    for tag, value in [('ixx', mass*(y*y+z*z)/12), ('iyy', mass*(x*x+z*z)/12),
                       ('izz', mass*(x*x+y*y)/12), ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        ET.SubElement(tensor, tag).text = str(value)


def apply_sensor_rig(robot, height=1.0, pitch=.25):
    """Camera x forward, positive pitch looks down; same pose must feed ROS TF."""
    if not .3 <= height <= 1.3 or not 0 <= pitch <= .6:
        raise ValueError('Candidate rig height/pitch outside supported calibration range')
    base = robot.find("link[@name='base_link']")
    for old in base.findall("visual[@name='rgbd_mast']"):
        base.remove(old)
    camera = robot.find("link[@name='camera_rgb_frame']")
    camera.find('pose').text = f'0.069 -0.047 {height} 0 {pitch} 0'
    housing = [.035, .09, .03]
    _box(camera, 'rgbd_housing', [-.02, 0, 0], housing, '.08 .09 .10 1')
    _inertia(camera, .072, housing, [-.02, 0, 0])
    # A separate physical mast and mounting plate, rigidly attached to the base.
    mast = ET.SubElement(robot, 'link', name='rgbd_support')
    center = (height+.18)/2
    # Keep the column behind the optical origin so downward rays cannot hit it.
    ET.SubElement(mast, 'pose').text = f'0.029 -0.047 {center} 0 0 0'
    size = [.025, .025, height-.18]
    _box(mast, 'mast', [0, 0, 0], size, '.15 .17 .19 1')
    _box(mast, 'mount_plate', [0, 0, .18-center], [.10, .07, .012], '.2 .2 .2 1')
    # Box approximation for combined mast/plate mass; explicit positive inertia.
    _inertia(mast, .25, [.10, .07, height-.18])
    joint = ET.SubElement(robot, 'joint', name='rgbd_support_joint', type='fixed')
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'rgbd_support'


def prepare_obj(path, asset_folder, cache_dir):
    """Repair Fuel OBJ/MTL texture paths in derived files, preserving originals."""
    if path.suffix.lower() != '.obj':
        return path
    folder = Path(cache_dir)/hashlib.sha256(str(path).encode()).hexdigest()[:16]
    folder.mkdir(parents=True, exist_ok=True)
    lines = []
    for line in path.read_text().splitlines():
        if line.startswith('mtllib '):
            original = path.parent/line.split(maxsplit=1)[1]
            material = []
            for value in original.read_text().splitlines():
                if value.startswith('map_'):
                    key, filename = value.split(maxsplit=1)
                    texture = original.parent/filename
                    if not texture.is_file():
                        texture = asset_folder/'materials/textures'/filename
                    if not texture.is_file():
                        raise FileNotFoundError(texture)
                    # Fortress OBJLoader concatenates paths, even when mtllib is absolute.
                    # Keep the whole OBJ/MTL/texture bundle relative inside an ASCII cache.
                    shutil.copyfile(texture, folder/texture.name)
                    value = key+' '+texture.name
                material.append(value)
            derived = folder/original.name
            derived.write_text('\n'.join(material)+'\n')
            line = 'mtllib '+derived.name
        lines.append(line)
    target = folder/path.name
    target.write_text('\n'.join(lines)+'\n')
    return target


def add_targets(world, root, layout, cache_dir):
    truth = []
    for target in layout['targets']:
        folder = root/'third_party/target_assets'/target['asset']
        model = deepcopy(ET.parse(folder/'model.sdf').getroot().find('model'))
        model.set('name', target['id'])
        for tag in ('pose', 'static'):
            for node in model.findall(tag):
                model.remove(node)
        ET.SubElement(model, 'pose').text = ' '.join(map(str, target['pose']))
        ET.SubElement(model, 'static').text = 'true'
        for uri in model.findall('.//mesh/uri'):
            path = uri.text
            if path.startswith('model://'):
                path = path.removeprefix('model://').split('/', 1)[1]
            resolved = (folder/path).resolve()
            if not resolved.is_file():
                raise FileNotFoundError(resolved)
            uri.text = str(prepare_obj(resolved, folder, cache_dir))
        # The legacy Gazebo material script is unavailable in Fortress.
        for material in model.findall('.//visual/material'):
            for script in material.findall('script'):
                material.remove(script)
            if material.find('diffuse') is None:
                ET.SubElement(material, 'diffuse').text = '.9 .9 .9 1'
            if target['asset'] == 'plastic_cup':
                material.find('diffuse').text = '.8 .12 .08 1'
                ET.SubElement(material, 'ambient').text = '.3 .04 .02 1'
        world.append(model)
        truth.append(dict(target))
    return truth


def build_world(root, output, base_builder, height=1.0, pitch=.25, layout_file=None):
    upstream = root/'third_party/tb3_fortress/turtlebot3'
    assets = root/'third_party/aws_small_house'
    original = base_builder(upstream, output, assets)
    document = ET.parse(output)
    world = document.getroot().find('world')
    world.set('name', 'aws_house_r0_v1')
    robot = world.find("model[@name='turtlebot3_waffle']")
    if robot is None:
        robot = next(m for m in world.findall('model') if m.find("link[@name='camera_rgb_frame']") is not None)
    apply_sensor_rig(robot, height, pitch)
    layout_path = root/(layout_file or 'scenes/aws_house/layout_r0_v1.json')
    layout = json.loads(layout_path.read_text())
    targets = add_targets(world, root, layout, output.parent/'target_meshes')
    document.write(output, encoding='utf-8', xml_declaration=True)
    return dict(purpose='evaluation_only_never_runtime_search_input', frame_id='world',
                original_instances=len(original), targets=targets,
                layout_file=str(layout_path.relative_to(root)),
                layout_sha256=hashlib.sha256(layout_path.read_bytes()).hexdigest())


def rig_frames(description, height=1.0, pitch=.25):
    joint = description.find("joint[@name='camera_joint']/origin")
    joint.set('xyz', f'0.069 -0.047 {height}')
    joint.set('rpy', f'0 {pitch} 0')
    ET.SubElement(description, 'link', name='rgbd_support')
    support = ET.SubElement(description, 'joint', name='rgbd_support_joint', type='fixed')
    ET.SubElement(support, 'parent', link='base_link')
    ET.SubElement(support, 'child', link='rgbd_support')
    ET.SubElement(support, 'origin', xyz=f'0.029 -0.047 {(height+.18)/2}')
