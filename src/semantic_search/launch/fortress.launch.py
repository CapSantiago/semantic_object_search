"""Adapt the pinned Onicc TurtleBot3 assets at launch time."""
import os
import json
import sys
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.actions import LogInfo, TimerAction
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from semantic_search.aws_world import import_aws_house
from semantic_search.scenes import load_scene


def box_link(model, name, position, size, color):
    link = ET.SubElement(model, 'link', name=name)
    ET.SubElement(link, 'pose').text = f'{position} 0 0 0'
    for kind in ('collision', 'visual'):
        part = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(part, 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = size
        if kind == 'visual':
            ET.SubElement(ET.SubElement(part, 'material'), 'diffuse').text = color


def static_model(world, name, x, y):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} 0 0 0 0'
    return model


def chair_model(asset, name, x, y):
    model = ET.parse(asset / 'model.sdf').getroot().find('model')
    model.set('name', name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} 0 0 0 0'
    for uri in model.findall('.//mesh/uri'):
        uri.text = str((asset / 'meshes' / Path(uri.text).name).resolve())
    return model


def make_robot(upstream, camera_height=0.30):
    source = upstream / 'models/turtlebot3/model.sdf'
    robot = ET.parse(source).getroot().find('model')
    for uri in robot.findall('.//mesh/uri'):
        mesh = source.parent / 'meshes' / Path(uri.text).name
        uri.text = str(mesh.resolve())

    camera_link = robot.find("./link[@name='camera_rgb_frame']")
    camera_link.find('pose').text = f'0.069 -0.047 {camera_height:.2f} 0 0 0'
    sensor = camera_link.find('sensor')
    sensor.set('type', 'rgbd_camera')
    sensor.remove(sensor.find('camera_info_topic'))
    sensor.find('topic').text = '/rgbd'
    sensor.find('ignition_frame_id').text = 'camera_optical_frame'
    sensor.find('update_rate').text = '8'
    camera = sensor.find('camera')
    camera.remove(camera.find('noise'))
    camera.find('image/width').text = '640'
    camera.find('image/height').text = '480'
    camera.find('clip/far').text = '8.0'
    ET.SubElement(camera, 'optical_frame_id').text = 'camera_optical_frame'
    return robot


def make_world(upstream, destination, chair_asset=None):
    """Original baseline geometry and sensor settings, preserved for regression."""
    root = ET.parse(upstream / 'models/worlds/world_only.sdf').getroot()
    root.set('version', '1.8')
    world = root.find('world')
    world.set('name', 'course_house')
    robot = make_robot(upstream)
    world.append(robot)

    walls = [
        (-4, 0, '.12 6 2'), (4, 0, '.12 6 2'),
        (0, -3, '8 .12 2'), (0, 3, '8 .12 2'),
        (0, 1.85, '.12 2.3 2'), (0, -1.85, '.12 2.3 2'),
        (2.35, 0, '3.3 .12 2'),
    ]
    for index, (x, y, size) in enumerate(walls):
        model = static_model(world, f'wall_{index}', x, y)
        box_link(model, 'wall', '0 0 1', size, '0.78 0.74 0.69 1')

    # Simple scene fixtures. Replace or augment these with realistic detector targets on Ubuntu.
    for index, (x, y) in enumerate([(1.5, 1.8), (2.45, 2.25), (-2.6, 1.0), (3.1, -2.2)]):
        if chair_asset is not None:
            world.append(chair_model(chair_asset, f'chair_{index}', x, y))
            continue
        chair = static_model(world, f'chair_{index}', x, y)
        box_link(chair, 'seat', '0 0 .34', '.34 .34 .04', '.25 .14 .08 1')
        box_link(chair, 'back', '0 .15 .55', '.34 .04 .40', '.25 .14 .08 1')
        for leg, (dx, dy) in enumerate([(-.13, -.13), (-.13, .13), (.13, -.13), (.13, .13)]):
            box_link(chair, f'leg_{leg}', f'{dx} {dy} .16', '.05 .05 .32', '.19 .10 .05 1')
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)


def make_aws_world(upstream, destination, asset_root):
    root = ET.parse(upstream / 'models/worlds/world_only.sdf').getroot()
    root.set('version', '1.8')
    world = root.find('world')
    world.set('name', 'aws_house')
    world.find('physics/max_step_size').text = '0.002'
    ground = world.find("model[@name='ground_plane']")
    ground.find('link').remove(ground.find('link/visual'))
    robot = make_robot(upstream, camera_height=0.65)
    # A visual mast makes the scene-specific raised RGB-D mounting explicit.
    base = robot.find("link[@name='base_link']")
    visual = ET.SubElement(base, 'visual', name='rgbd_mast')
    ET.SubElement(visual, 'pose').text = '0.069 -0.047 0.39 0 0 0'
    ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = '.025 .025 .52'
    ET.SubElement(ET.SubElement(visual, 'material'), 'diffuse').text = '.12 .14 .16 1'
    world.append(robot)
    truth = import_aws_house(world, asset_root, Path(destination).parent / 'meshes')
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)
    return truth


def setup(context):
    scene_id = LaunchConfiguration('scene').perform(context)
    project_root = Path(LaunchConfiguration('project_root').perform(context)).expanduser()
    scene = load_scene(scene_id, project_root)
    upstream = Path(LaunchConfiguration('upstream_dir').perform(context)).expanduser().resolve()
    if not (upstream / 'models/turtlebot3/model.sdf').is_file():
        raise RuntimeError(f'Pass upstream_dir pointing to the cloned tb3_fortress/turtlebot3: {upstream}')
    cache = tempfile.TemporaryDirectory(prefix='semantic_house_')
    world_path = Path(cache.name) / f'{scene["world"]}.sdf'
    asset_arg = LaunchConfiguration('chair_asset').perform(context)
    asset = Path(asset_arg).expanduser().resolve() if asset_arg else None
    if asset is not None and not (asset / 'model.sdf').is_file():
        raise RuntimeError(f'Chair asset missing: {asset}')
    camera_height = 0.30
    if scene_id == 'baseline':
        make_world(upstream, world_path, asset)
    else:
        asset_root = project_root / 'third_party/aws_small_house'
        if not (asset_root / 'worlds/small_house.world').is_file():
            raise RuntimeError('AWS scene missing: see scripts/fetch_aws_house.sh')
        truth = make_aws_world(upstream, world_path, asset_root)
        (Path(cache.name) / 'scene_truth.json').write_text(json.dumps(truth, indent=2))
        camera_height = 0.65
    config = Path(get_package_share_directory('semantic_search')) / 'config'
    gazebo_args = ['ign', 'gazebo', '-r', '-v', '3']
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        gazebo_args.append('-s')
    gazebo_args.append(str(world_path))
    description = ET.fromstring((config / 'tb3_frames.urdf').read_text())
    description.find("joint[@name='camera_joint']/origin").set('xyz', f'0.069 -0.047 {camera_height:.2f}')
    if scene_id == 'aws_house' and LaunchConfiguration('gui').perform(context).lower() == 'true':
        gui_default = Path('/usr/share/ignition/ignition-gazebo6/gui/gui.config')
        if gui_default.exists():
            # Fortress's GUI config has multiple top-level XML elements.
            gui = gui_default.read_text().replace('<?xml version="1.0"?>', '')
            gui = gui.replace('<camera_pose>-6 0 6 0 0.5 0</camera_pose>',
                              '<camera_pose>0 -10.8 15 0 0.9273 1.5708</camera_pose>')
            gui_path = Path(cache.name) / 'aws_gui.config'
            gui_path.write_text(gui)
            gazebo_args.extend(['--gui-config', str(gui_path)])

    def from_sim(topic, ros_type, gz_type):
        return f'{topic}@{ros_type}[ignition.msgs.{gz_type}'

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            from_sim('/odom', 'nav_msgs/msg/Odometry', 'Odometry'),
            from_sim('/odom/tf', 'tf2_msgs/msg/TFMessage', 'Pose_V'),
            from_sim('/clock', 'rosgraph_msgs/msg/Clock', 'Clock'),
            from_sim('/scan', 'sensor_msgs/msg/LaserScan', 'LaserScan'),
            from_sim('/rgbd/image', 'sensor_msgs/msg/Image', 'Image'),
            from_sim('/rgbd/depth_image', 'sensor_msgs/msg/Image', 'Image'),
            from_sim('/rgbd/camera_info', 'sensor_msgs/msg/CameraInfo', 'CameraInfo'),
        ],
        remappings=[('/odom/tf', '/tf')],
    )
    actions = [
        LogInfo(msg=f'Scene [{scene_id}]: {scene["label"]}; world={scene["world"]}'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: cache.cleanup())])),
        ExecuteProcess(cmd=gazebo_args, output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True,
                          'robot_description': ET.tostring(description, encoding='unicode')}]),
        bridge,
    ]
    if scene_id == 'aws_house' and LaunchConfiguration('gui').perform(context).lower() == 'true':
        actions.append(TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=[sys.executable, '-m', 'semantic_search.scene_view'], output='screen')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='aws_house'),
        DeclareLaunchArgument('project_root', default_value=os.getcwd()),
        DeclareLaunchArgument('gui', default_value='true', description='Start Gazebo GUI'),
        DeclareLaunchArgument('chair_asset', default_value=os.path.join(
            os.getcwd(), 'third_party', 'fuel_chair'),
            description='Fuel Chair asset directory; empty string uses original geometric fixtures'),
        DeclareLaunchArgument('upstream_dir', default_value=os.path.join(
            os.getcwd(), 'third_party', 'tb3_fortress', 'turtlebot3')),
        OpaqueFunction(function=setup),
    ])
