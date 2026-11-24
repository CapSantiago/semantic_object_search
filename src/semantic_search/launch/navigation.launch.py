"""Nav2 bringup with explicit goal checkers; original planners/recoveries retained."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Humble's YAML parser corrupts escaped non-ASCII path values. env.sh
    # already creates this checkout-specific ASCII alias for ROSIDL/Nav2.
    config = PathJoinSubstitution([EnvironmentVariable('SEMANTIC_WS'),'src','semantic_search','config'])
    params = RewrittenYaml(source_file=LaunchConfiguration('params_file'),root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml':PathJoinSubstitution([config,'navigate_service.xml']),
            'default_nav_through_poses_bt_xml':PathJoinSubstitution([config,'navigate_service_through_poses.xml'])},
        convert_types=True)
    names = [('use_sim_time','True'),('autostart','False'),('use_composition','False'),
             ('map',''),('params_file','')]
    arguments = {name:LaunchConfiguration(name) for name,_ in names}
    arguments['params_file'] = params
    return LaunchDescription([*(DeclareLaunchArgument(name,default_value=value) for name,value in names),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),'launch','bringup_launch.py'])),launch_arguments=arguments.items())])
