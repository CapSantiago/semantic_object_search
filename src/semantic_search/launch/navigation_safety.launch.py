"""Opt-in upstream collision filtering and a final freshness watchdog."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('semantic_search'))/'config/collision_monitor.yaml'
    return LaunchDescription([
        Node(package='nav2_collision_monitor',executable='collision_monitor',
             name='collision_monitor',parameters=[str(config)],output='screen'),
        Node(package='semantic_search',executable='lifecycle_startup',output='screen'),
        Node(package='semantic_search',executable='velocity_guard',
             parameters=[dict(use_sim_time=True)],output='screen')])
