"""Separate candidate layout/rig. Build a fresh map before navigation use."""
import json
import runpy
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from semantic_search.development import build_world, rig_frames


def setup(context):
    root = Path(LaunchConfiguration('project_root').perform(context)).resolve()
    share = Path(get_package_share_directory('semantic_search'))
    original = runpy.run_path(str(share/'launch/fortress.launch.py'))
    cache = tempfile.TemporaryDirectory(prefix='semantic_r0_')
    output = Path(cache.name)/'aws_house_r0_v1.sdf'
    height = float(LaunchConfiguration('camera_height').perform(context))
    pitch = float(LaunchConfiguration('camera_pitch').perform(context))
    layout_file = LaunchConfiguration('layout_file').perform(context)
    truth = build_world(root, output, original['make_aws_world'], height, pitch, layout_file)
    truth['camera_rig'] = dict(height_m=height,pitch_rad=pitch)
    evidence = root/LaunchConfiguration('evidence_dir').perform(context)
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence/'scene_truth.json').write_text(json.dumps(truth, indent=2)+'\n')
    description = ET.parse(share/'config/tb3_frames.urdf').getroot()
    rig_frames(description,height,pitch)
    bridge = [
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/rgbd/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/rgbd/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        '/rgbd/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked']
    command = ['ign', 'gazebo', '-r', '-v', '3']
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        command.append('-s')
    command.append(str(output))
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: cache.cleanup())])),
            ExecuteProcess(cmd=command, output='screen'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'use_sim_time': True, 'robot_description': ET.tostring(description, encoding='unicode')}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridge,
                 remappings=[('/odom/tf', '/tf'),('/rgbd/points','/rgbd/points_raw')], output='screen'),
            Node(package='semantic_search',executable='cloud_adapter',
                 parameters=[{'use_sim_time':True}],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('project_root'),
                              DeclareLaunchArgument('gui', default_value='false'),
                              DeclareLaunchArgument('camera_height', default_value='1.0'),
                              DeclareLaunchArgument('camera_pitch', default_value='0.25'),
                              DeclareLaunchArgument('layout_file', default_value='scenes/aws_house/layout_r0_v1.json'),
                              DeclareLaunchArgument('evidence_dir', default_value='results/development/r0_v1'),
                              OpaqueFunction(function=setup)])
