from pathlib import Path
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from semantic_search.scenes import load_scene


def setup(context):
    scene = load_scene(LaunchConfiguration('scene').perform(context),
                       LaunchConfiguration('project_root').perform(context))
    rooms = LaunchConfiguration('rooms_file').perform(context) or scene['rooms']
    output = LaunchConfiguration('output_dir').perform(context) or scene['output']
    iou = LaunchConfiguration('iou_threshold').perform(context)
    iou = float(iou) if iou else scene.get('detector_iou', .5)
    return [
        LogInfo(msg=f'Search scene [{scene["id"]}]: {scene["label"]}; rooms={rooms}'),
        Node(package='semantic_search', executable='search', output='screen',
             parameters=[{'use_sim_time': True, 'rooms_file': rooms,
                          'output_dir': output, 'scene_id': scene['id']}]),
        Node(executable=LaunchConfiguration('detector_python'),
             arguments=['-m', 'semantic_search.detector'], output='screen',
             condition=IfCondition(LaunchConfiguration('with_detector')),
             parameters=[{'use_sim_time': True, 'model': LaunchConfiguration('model'),
                          'iou_threshold': iou,
                          'device': ParameterValue(LaunchConfiguration('device'), value_type=str)}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='aws_house'),
        DeclareLaunchArgument('project_root', default_value=os.getcwd()),
        DeclareLaunchArgument('rooms_file', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('with_detector', default_value='false'),
        DeclareLaunchArgument('model', default_value='yolov8n-seg.pt'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('iou_threshold', default_value=''),
        DeclareLaunchArgument('detector_python', default_value=(
            str(Path(os.environ['VIRTUAL_ENV']) / 'bin/python')
            if 'VIRTUAL_ENV' in os.environ else sys.executable)),
        OpaqueFunction(function=setup),
    ])
