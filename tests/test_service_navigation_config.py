"""Configuration contracts preventing empty-ID failures with multiple checkers."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_native_cleaning_and_ordinary_goals_select_their_own_registered_checker():
    for name in ('nav2_depth.yaml','nav2_safety.yaml'):
        config = yaml.safe_load((ROOT/'src/semantic_search/config'/name).read_text())
        params = config['controller_server']['ros__parameters']
        assert set(params['goal_checker_plugins']) == {'general_goal_checker','cleaning_goal_checker','docking_goal_checker'}
        assert params['docking_goal_checker']==dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=.05,yaw_goal_tolerance=.1,stateful=True)
        assert params['general_goal_checker']['xy_goal_tolerance'] == .12
        assert params['general_goal_checker']['yaw_goal_tolerance'] == .1
        assert params['cleaning_goal_checker'] == dict(plugin='nav2_controller::PositionGoalChecker',
            xy_goal_tolerance=.12,stateful=False)
        assert params['progress_checker'] == dict(plugin='nav2_controller::PoseProgressChecker',
            required_movement_radius=.2,required_movement_angle=.5,movement_time_allowance=10.)
    for name,expected in [('navigate_cleaning_lane.xml','cleaning_goal_checker'),
                          ('navigate_cleaning_connector.xml','cleaning_goal_checker'),
                          ('navigate_service.xml','general_goal_checker'),
                          ('navigate_docking.xml','docking_goal_checker'),
                          ('navigate_service_through_poses.xml','general_goal_checker')]:
        tree = ET.parse(ROOT/'src/semantic_search/config'/name)
        actions = tree.findall('.//FollowPath')
        assert actions and all(x.get('goal_checker_id')==expected for x in actions)


def test_cleaning_connector_keeps_normal_navfn_replanning_and_recovery_actions():
    config=ROOT/'src/semantic_search/config'
    original=ET.parse(config/'navigate_service.xml')
    connector=ET.parse(config/'navigate_cleaning_connector.xml')
    follow=connector.find('.//FollowPath')
    assert follow.get('controller_id')=='CleaningPath'
    assert follow.get('goal_checker_id')=='cleaning_goal_checker'
    follow.set('controller_id','FollowPath');follow.set('goal_checker_id','general_goal_checker')
    assert ET.tostring(connector.getroot())==ET.tostring(original.getroot())


def test_rewritten_bt_paths_preserve_the_project_ascii_alias(monkeypatch,tmp_path):
    import importlib.util
    import os
    import pytest
    pytest.importorskip('launch')
    from launch import LaunchContext
    from launch_ros.substitutions import FindPackageShare
    path=ROOT/'src/semantic_search/launch/navigation.launch.py'
    spec=importlib.util.spec_from_file_location('service_navigation_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setenv('SEMANTIC_WS','/tmp/service_ascii_alias')
    monkeypatch.setattr(FindPackageShare,'perform',lambda self,context:'/tmp/installed_package')
    context=LaunchContext();context.environment['SEMANTIC_WS']='/tmp/service_ascii_alias'
    description=module.generate_launch_description()
    include=description.entities[-1]
    arguments=dict(include.launch_arguments)
    for name in ('nav2_depth.yaml','nav2_safety.yaml','nav2_body_footprint.yaml'):
        context.launch_configurations['params_file']=str(ROOT/'src/semantic_search/config'/name)
        rewritten=arguments['params_file'].perform(context)
        text=Path(rewritten).read_text()
        # Native rcl_yaml_param_parser rejects aliases, even for shared lists
        # which have equal values after Python's safe_load succeeds.
        assert not any(isinstance(token,yaml.tokens.AliasToken) for token in yaml.scan(text))
        config=yaml.safe_load(text)['bt_navigator']['ros__parameters']
        assert config['default_nav_to_pose_bt_xml'].startswith('/tmp/service_ascii_alias/')
        assert config['default_nav_to_pose_bt_xml'].isascii()
        Path(rewritten).unlink()


def test_local_control_and_behaviors_share_continuous_frame_while_global_goals_use_map():
    # Global goals keep map; local sensor history and recoveries use odom
    # in the default repair so localization corrections do not move stored
    # local obstacles. The archived safety candidate retains its own profile.
    for name in ('nav2_depth.yaml','nav2_safety.yaml'):
        config=yaml.safe_load((ROOT/'src/semantic_search/config'/name).read_text())
        local=config['local_costmap']['local_costmap']['ros__parameters']
        global_=config['global_costmap']['global_costmap']['ros__parameters']
        behavior=config['behavior_server']['ros__parameters']
        assert global_['global_frame']=='map'
        expected='odom' if name=='nav2_depth.yaml' else 'map'
        assert local['global_frame']==behavior['global_frame']==expected
        assert local['static_layer'].get('enabled',True) is (name!='nav2_depth.yaml')
        assert local['plugins'][0]=='static_layer'
        assert local['static_layer']['map_subscribe_transient_local'] is True
        controller=config['controller_server']['ros__parameters']
        assert controller['FollowPath']['xy_goal_tolerance']==controller['cleaning_goal_checker']['xy_goal_tolerance']==.12
        speed=.35 if name=='nav2_depth.yaml' else .26
        assert controller['FollowPath']['max_vel_x']==speed
        assert controller['CleaningPath']['max_speed_xy']==speed
        smoother=config['velocity_smoother']['ros__parameters']
        assert smoother['max_velocity'][0]==speed
        assert smoother['min_velocity'][0]==-speed
        assert local['robot_radius']==global_['robot_radius']


def test_short_lane_turns_use_the_native_collision_checked_shim_and_original_dwb_limits():
    for name in ('nav2_depth.yaml','nav2_safety.yaml'):
        params=yaml.safe_load((ROOT/'src/semantic_search/config'/name).read_text())['controller_server']['ros__parameters']
        assert set(params['controller_plugins'])=={'FollowPath','CleaningPath','DockingPath'}
        ordinary,lane=params['FollowPath'],params['CleaningPath']
        assert ordinary.get('primary_controller',ordinary['plugin'])=='dwb_core::DWBLocalPlanner'
        if name=='nav2_depth.yaml':
            assert ordinary['plugin']=='nav2_rotation_shim_controller::RotationShimController'
            assert ordinary['rotate_to_goal_heading'] is False
        assert lane['plugin']=='nav2_rotation_shim_controller::RotationShimController'
        assert lane['primary_controller']==ordinary.get('primary_controller',ordinary['plugin'])
        assert lane['rotate_to_goal_heading'] is False
        assert 0<lane['forward_sampling_distance']<.35-.12
        assert lane['angular_disengage_threshold']==.1
        assert lane['angular_dist_threshold']==.4
        assert lane['simulate_ahead_time']>0
        assert lane['critics']==[critic for critic in ordinary['critics'] if critic!='RotateToGoal']
        for key,value in ordinary.items():
            if key not in ('plugin','critics'):
                assert lane[key]==value,(name,key)
    tree=ET.parse(ROOT/'src/semantic_search/config/navigate_cleaning_lane.xml')
    assert tree.find('.//FollowPath').get('controller_id')=='CleaningPath'
    assert tree.find('.//FollowPath').get('goal_checker_id')=='cleaning_goal_checker'
    for name in ('navigate_service.xml','navigate_service_through_poses.xml'):
        assert all(x.get('controller_id')=='FollowPath' for x in ET.parse(ROOT/'src/semantic_search/config'/name).findall('.//FollowPath'))


def test_rgbd_recovery_observes_without_erasing_hidden_real_obstacles():
    for name in ('navigate_service.xml','navigate_service_through_poses.xml',
                 'navigate_cleaning_connector.xml'):
        tree=ET.parse(ROOT/'src/semantic_search/config'/name)
        assert not tree.findall('.//ClearEntireCostmap')
        for recovery_name in ('ComputePathToPose','ComputePathThroughPoses','FollowPath'):
            for recovery in tree.findall(f'.//RecoveryNode[@name="{recovery_name}"]'):
                retry=list(recovery)[-1]
                assert retry.tag=='Wait' and float(retry.get('wait_duration'))>=1.
        assert tree.find('.//Spin') is not None
        assert tree.find('.//BackUp') is not None
        assert tree.find('.//Sequence[@name="ObserveKeepingObstacles"]') is not None


def test_precision_checker_is_registered_in_body_candidate_without_changing_its_lane_tolerance():
    params=yaml.safe_load((ROOT/'src/semantic_search/config/nav2_body_footprint.yaml').read_text())['controller_server']['ros__parameters']
    assert 'docking_goal_checker' in params['goal_checker_plugins']
    assert params['docking_goal_checker']==dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=.05,yaw_goal_tolerance=.1,stateful=True)
    assert params['cleaning_goal_checker']['xy_goal_tolerance']==.04


def test_precision_docking_controller_and_goal_checker_share_xy_threshold():
    for name in ('nav2_depth.yaml','nav2_safety.yaml','nav2_body_footprint.yaml'):
        params=yaml.safe_load((ROOT/'src/semantic_search/config'/name).read_text())['controller_server']['ros__parameters']
        expected=dict(params['FollowPath'],xy_goal_tolerance=.05,rotate_to_goal_heading=True)
        assert params['DockingPath']==expected
        assert params['DockingPath']['xy_goal_tolerance']==params['docking_goal_checker']['xy_goal_tolerance']
    action=ET.parse(ROOT/'src/semantic_search/config/navigate_docking.xml').find('.//FollowPath')
    assert action.get('controller_id')=='DockingPath' and action.get('goal_checker_id')=='docking_goal_checker'
