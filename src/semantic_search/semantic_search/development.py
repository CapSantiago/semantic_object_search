"""Opt-in R0 layout and complete RGB-D rig; archived scenes stay separate."""
from copy import deepcopy
import ctypes
import json
import hashlib
import math
import shutil
import struct
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


def add_contact_diagnostics(robot):
    """Expose measured simulation contacts; never an input to task decisions."""
    for link in robot.findall('link'):
        # Normal wheel/caster floor contacts arrive at every physics step.
        # Monitor the body/rig without adding that traffic to the control bridge.
        if link.get('name') in {'wheel_left_link','wheel_right_link',
                                'caster_back_left_link','caster_back_right_link'}:
            continue
        collisions=link.findall('collision')
        if not collisions:
            continue
        sensor=ET.SubElement(link,'sensor',name='robot_contact',type='contact')
        ET.SubElement(sensor,'always_on').text='true'
        ET.SubElement(sensor,'update_rate').text='30'
        contact=ET.SubElement(sensor,'contact')
        for collision in collisions:
            ET.SubElement(contact,'collision').text=collision.get('name')
        ET.SubElement(contact,'topic').text='/robot/contacts'


def prepare_obj(path, asset_folder, cache_dir):
    """Repair Fuel OBJ/MTL texture paths in derived files, preserving originals."""
    if path.suffix.lower() != '.obj':
        return path
    namespace = hashlib.sha256(str(path).encode()).hexdigest()[:16]
    folder = Path(cache_dir)/namespace
    folder.mkdir(parents=True, exist_ok=True)
    lines = []
    for line in path.read_text().splitlines():
        if line.startswith('mtllib '):
            original = path.parent/line.split(maxsplit=1)[1].strip()
            material = []
            for value in original.read_text().splitlines():
                if value.startswith('newmtl '):
                    value = 'newmtl '+namespace+'_'+value.split(maxsplit=1)[1].strip()
                elif value.startswith('map_') or value.startswith(('bump ','disp ','decal ')):
                    key, filename = value.split(maxsplit=1)
                    filename = filename.strip()
                    texture = original.parent/filename
                    if not texture.is_file():
                        texture = asset_folder/'materials/textures'/filename
                    if not texture.is_file():
                        raise FileNotFoundError(texture)
                    # Fortress OBJLoader concatenates paths, even when mtllib is absolute.
                    # Keep the whole OBJ/MTL/texture bundle relative inside an ASCII cache.
                    # Ogre resources can be shared across mesh directories.
                    # Google/YCB assets commonly reuse material_0/texture.png;
                    # isolate both material IDs and texture resource names.
                    texture_key = hashlib.sha256(str(texture.resolve()).encode()).hexdigest()[:16]
                    texture_name = texture_key+'_'+texture.name
                    shutil.copyfile(texture, folder/texture_name)
                    value = key+' '+texture_name
                material.append(value)
            derived = folder/(namespace+'_'+original.name)
            derived.write_text('\n'.join(material)+'\n')
            line = 'mtllib '+derived.name
        elif line.startswith('usemtl '):
            line = 'usemtl '+namespace+'_'+line.split(maxsplit=1)[1].strip()
        lines.append(line)
    target = folder/(namespace+'_'+path.name)
    target.write_text('\n'.join(lines)+'\n')
    return target


def prepare_glb(path, cache_dir):
    """Use installed Assimp for glTF geometry; preserve the pinned source.

    Fortress common4 cannot load GLB. This static adapter supports embedded
    glTF2 images, converts +Y-up to +Z-up, and uses a diffuse OBJ fallback for
    PBR materials. It is not a general animated glTF renderer.
    """
    path = Path(path)
    namespace = hashlib.sha256(path.read_bytes()).hexdigest()[:16]
    folder = Path(cache_dir)/('glb_'+namespace)
    destination = folder/'model.obj'
    if (folder/'complete.json').is_file():
        return destination
    folder.mkdir(parents=True,exist_ok=True)
    data = path.read_bytes()
    if len(data)<28 or len(data)>64*1024*1024:
        raise ValueError('Invalid static GLB size')
    magic,version,size = struct.unpack_from('<4sII',data)
    if magic!=b'glTF' or version!=2 or size!=len(data):
        raise ValueError('Expected a complete glTF2 binary')
    chunks,offset = {},12
    while offset<len(data):
        length,kind = struct.unpack_from('<I4s',data,offset);offset+=8
        if offset+length>len(data) or kind in chunks:
            raise ValueError('Invalid GLB chunk bounds')
        chunks[kind] = data[offset:offset+length];offset+=length
    document = json.loads(chunks[b'JSON'])
    payload = chunks[b'BIN\x00']
    textures = {}
    for index,item in enumerate(document.get('images',[])):
        if 'bufferView' not in item or item.get('mimeType') not in ('image/png','image/jpeg'):
            raise ValueError('Only embedded PNG/JPEG glTF images are supported')
        view = document['bufferViews'][item['bufferView']]
        start,length = view.get('byteOffset',0),view['byteLength']
        if view.get('buffer',0)!=0 or min(start,length)<0 or start+length>len(payload):
            raise ValueError('Embedded image outside GLB binary buffer')
        name = f'texture_{index}'+('.png' if item['mimeType']=='image/png' else '.jpg')
        (folder/name).write_bytes(payload[start:start+length]);textures['*'+str(index)] = name
    library = ctypes.CDLL('libassimp.so.5')
    library.aiImportFile.argtypes=[ctypes.c_char_p,ctypes.c_uint]
    library.aiImportFile.restype=ctypes.c_void_p
    library.aiExportScene.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_char_p,ctypes.c_uint]
    library.aiExportScene.restype=ctypes.c_int
    library.aiReleaseImport.argtypes=[ctypes.c_void_p]
    library.aiGetErrorString.restype=ctypes.c_char_p
    scene = library.aiImportFile(str(path.resolve()).encode(),0x108)  # Triangulate + PreTransformVertices.
    if not scene:
        raise ValueError('Assimp import failed: '+str(library.aiGetErrorString()))
    try:
        if library.aiExportScene(scene,b'obj',str(destination.resolve()).encode(),0)!=0:
            raise ValueError('Assimp export failed: '+str(library.aiGetErrorString()))
    finally:
        library.aiReleaseImport(scene)
    lines = []
    for line in destination.read_text().splitlines():
        if line.startswith(('v ','vn ')):
            key,x,y,z,*tail = line.split()
            line = f'{key} {x} {-float(z):.10g} {y}'+(' '+' '.join(tail) if tail else '')
        lines.append(line)
    destination.write_text('\n'.join(lines)+'\n')
    for material in folder.glob('*.mtl'):
        lines = []
        for line in material.read_text().splitlines():
            if line.startswith(('map_','bump ','disp ','decal ')):
                key,filename = line.split(maxsplit=1)
                if filename.strip() not in textures:
                    raise ValueError('Unsupported external glTF texture reference')
                line = key+' '+textures[filename.strip()]
            elif line.startswith('Ke '):
                # OBJ has lost the PBR emissive texture; constant white emission
                # would erase the real albedo. Keep a diffuse-only fallback.
                line = 'Ke 0 0 0'
            lines.append(line)
        material.write_text('\n'.join(lines)+'\n')
    (folder/'complete.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(data).hexdigest(),
        assimp_version=[library.aiGetVersionMajor(),library.aiGetVersionMinor(),library.aiGetVersionPatch()],
        axis_conversion='glTF +Y up to simulator +Z up',material='diffuse fallback; PBR emission disabled'),indent=2)+'\n')
    return destination


def add_targets(world, root, layout, cache_dir):
    truth = []
    for target in layout['targets']:
        asset = layout.get('assets', {}).get(target['asset'], {})
        folder = (root/asset.get('directory', 'third_party/target_assets/'+target['asset'])).resolve()
        folder.relative_to(root.resolve())
        model = deepcopy(ET.parse(folder/'model.sdf').getroot().find('model'))
        # Fuel's Samsung asset includes a SubT proximity detector and a thermal
        # visual plugin. Neither is part of the phone's RGB-D geometry. Adapt
        # only these known extras, retaining the original staged source files.
        for include in list(model.findall('include')):
            if (include.findtext('uri') or '').rstrip('/').endswith('/Artifact Proximity Detector'):
                model.remove(include)
        for visual in model.findall('.//visual'):
            for plugin in list(visual.findall('plugin')):
                if plugin.get('filename') == 'ignition-gazebo-thermal-system':
                    visual.remove(plugin)
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
            if resolved.suffix.lower()=='.glb':
                resolved = prepare_glb(resolved,cache_dir)
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


def camera_spec(robot, width=640, height=480, hfov=1.02974):
    """Configure one aligned simulated RGB-D sensor; do not invent a hardware brand."""
    if ((width,height) not in ((640,480),(1280,960)) or isinstance(hfov,bool)
            or not math.isfinite(hfov) or not .8<=hfov<=1.6):
        raise ValueError('Unsupported RGB-D resolution or horizontal field of view')
    camera = robot.find("link[@name='camera_rgb_frame']/sensor/camera")
    if camera is None:
        raise ValueError('Robot has no RGB-D camera configuration')
    camera.find('image/width').text = str(width)
    camera.find('image/height').text = str(height)
    camera.find('horizontal_fov').text = str(hfov)


def add_charging_station(world,robot,station):
    """Add the configured facility and spawn; it supplies no perception answer."""
    pose=robot.find('pose')
    if pose is None:pose=ET.SubElement(robot,'pose')
    values=[float(x) for x in (pose.text or '0 0 0 0 0 0').split()]
    values[0],values[1],values[5]=station['world_spawn_pose']
    pose.text=' '.join(map(str,values))
    data=station['model'];x,y,a=data['pose'];sx,sy,sz=data['size']
    model=ET.SubElement(world,'model',name=data['name'])
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=f'{x} {y} 0 0 0 {a}'
    link=ET.SubElement(model,'link',name='charger')
    _box(link,'stand',(0,0,sz/2),(sx,sy,sz),'.06 .34 .36 1')
    _box(link,'indicator',(sx/2+.003,0,sz*.8),(.006,.18,.035),'.18 .9 .45 1')
    visual=ET.SubElement(link,'visual',name='docking_pad')
    dx,dy=values[0]-x,values[1]-y
    px,py=math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy
    ET.SubElement(visual,'pose').text=f'{px} {py} .006 0 0 {values[5]-a}'
    ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size').text='.9 .8 .012'
    ET.SubElement(ET.SubElement(visual,'material'),'diffuse').text='.12 .65 .55 .65'


def apply_scene_model_poses(world, poses):
    """Design-only independent layouts; original assets/worlds remain intact."""
    if not isinstance(poses,dict):
        raise ValueError('场景变化位姿必须为物品名称到位姿的字典。')
    applied={}
    for name,pose in poses.items():
        matches=[model for model in world.findall('model') if model.get('name')==name]
        if (len(matches)!=1 or name in ('turtlebot3_waffle','waffle','ground_plane','home_dock')
                or len(pose)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in pose)):
            raise ValueError('场景变化布局必须指定现有物品及有限六维位姿。')
        node=matches[0].find('pose')
        if node is None:node=ET.SubElement(matches[0],'pose')
        node.text=' '.join(map(str,pose));applied[name]=list(pose)
    return applied


def build_world(root, output, base_builder, height=1.0, pitch=.25, layout_file=None,
                image_width=640,image_height=480,camera_hfov=1.02974,charging_file=None):
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
    add_contact_diagnostics(robot)
    camera_spec(robot,image_width,image_height,camera_hfov)
    if charging_file:
        from .docking import load_station
        add_charging_station(world,robot,load_station(charging_file,root))
    layout_path = root/(layout_file or 'scenes/aws_house/layout_r0_v1.json')
    layout = json.loads(layout_path.read_text())
    scene_poses=apply_scene_model_poses(world,layout.get('scene_model_poses',{}))
    targets = add_targets(world, root, layout, output.parent/'target_meshes')
    document.write(output, encoding='utf-8', xml_declaration=True)
    return dict(purpose='evaluation_only_never_runtime_search_input', frame_id='world',
                original_instances=len(original), targets=targets,
                scene_model_poses=scene_poses,
                resource_adaptations=[json.loads(p.read_text()) for p in
                    (output.parent/'target_meshes').glob('glb_*/complete.json')],
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
