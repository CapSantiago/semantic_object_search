"""The explicit body candidate must enclose real collisions and check its edges."""
import json
import math
from pathlib import Path

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


def test_candidate_encloses_all_audited_body_collision_vertices():
    config=yaml.safe_load((ROOT/'src/semantic_search/config/nav2_body_footprint.yaml').read_text())
    hull=json.loads((ROOT/'tests/fixtures/cleaning_rear_access.json').read_text())['body_collision_hull_xy']
    maps=[config[key][key]['ros__parameters'] for key in ('global_costmap','local_costmap')]
    for index,costmap in enumerate(maps):
        vertices=np.asarray(json.loads(costmap['footprint']))
        assert vertices.shape==((4,2) if index==0 else (8,2))
        for point in hull:
            edges=np.roll(vertices,-1,axis=0)-vertices
            offsets=np.asarray(point)-vertices
            assert np.all(edges[:,0]*offsets[:,1]-edges[:,1]*offsets[:,0]>=-1e-10)
        assert costmap['footprint_padding']==(.01 if index==0 else .03)
        assert 'robot_radius' not in costmap
    # NavFn's translation guidance must not inherit the camera-side .08m
    # minimum radius of the true asymmetric body.
    rectangle=np.asarray(json.loads(maps[0]['footprint']))
    assert min(abs(rectangle[:,0]))>=.20 and min(abs(rectangle[:,1]))>=.18
    for key in ('FollowPath','CleaningPath','DockingPath'):
        controller=config['controller_server']['ros__parameters'][key]
        assert 'ObstacleFootprint' in controller['critics']
        assert 'BaseObstacle' not in controller['critics']
        assert controller['ObstacleFootprint.scale']>0


def test_candidate_keeps_baseline_motion_localization_and_obstacle_inputs():
    config=ROOT/'src/semantic_search/config'
    baseline=yaml.safe_load((config/'nav2_depth.yaml').read_text())
    candidate=yaml.safe_load((config/'nav2_body_footprint.yaml').read_text())
    assert candidate['planner_server']['ros__parameters']['GridBased']['plugin']=='service_navfn/CellCenteredNavfn'
    candidate['planner_server']['ros__parameters']['GridBased']['plugin']=baseline['planner_server']['ros__parameters']['GridBased']['plugin']
    global_layers=candidate['global_costmap']['global_costmap']['ros__parameters']
    for layer,plugin in [('laser_layer','OdomObstacleLayer'),('depth_layer','OdomVoxelLayer')]:
        assert global_layers[layer]['plugin']=='service_navfn/'+plugin
        global_layers[layer]['plugin']=baseline['global_costmap']['global_costmap']['ros__parameters'][layer]['plugin']
    assert candidate['controller_server']['ros__parameters']['cleaning_goal_checker']['xy_goal_tolerance']==.04
    candidate['controller_server']['ros__parameters']['cleaning_goal_checker']['xy_goal_tolerance']=baseline['controller_server']['ros__parameters']['cleaning_goal_checker']['xy_goal_tolerance']
    # Remember sensed local obstacles in continuous odometry, so later AMCL
    # corrections cannot move the robot into observations stored in old map TF.
    local=candidate['local_costmap']['local_costmap']['ros__parameters']
    assert local['global_frame']=='odom'
    assert candidate['behavior_server']['ros__parameters']['global_frame']=='odom'
    assert candidate['global_costmap']['global_costmap']['ros__parameters']['global_frame']=='map'
    # Do not reintroduce AMCL-projected prior edges into the continuous local
    # collision grid; the global prior and both real sensor layers stay active.
    assert local['static_layer'].pop('enabled') is False
    assert baseline['local_costmap']['local_costmap']['ros__parameters']['static_layer'].pop('enabled',False) is False
    assert {'laser_layer','depth_layer','inflation_layer'}<=set(local['plugins'])
    assert candidate['global_costmap']['global_costmap']['ros__parameters']['static_layer'].get('enabled',True)
    local['global_frame']=baseline['local_costmap']['local_costmap']['ros__parameters']['global_frame']
    candidate['behavior_server']['ros__parameters']['global_frame']=baseline['behavior_server']['ros__parameters']['global_frame']
    # The high camera sees low feet before they reach a 3m rolling window.
    # Preserve every marked 2.5m observation plus the sensor's XY offset.
    assert min(local['width'],local['height'])/2>=local['depth_layer']['depth']['obstacle_max_range']+.1
    assert local['footprint_padding']>=local['resolution']/2
    local['width']=baseline['local_costmap']['local_costmap']['ros__parameters']['width']
    local['height']=baseline['local_costmap']['local_costmap']['ros__parameters']['height']
    for namespace in ('global_costmap','local_costmap'):
        # The archived explicit body candidate has not adopted the default's
        # new ground-only clearing source; do not silently upgrade its status.
        depth=baseline[namespace][namespace]['ros__parameters']['depth_layer']
        assert depth['observation_sources']=='depth depth_clear'
        clear=depth.pop('depth_clear')
        assert clear['marking'] is False and clear['clearing'] is True
        assert depth['depth']['clearing'] is False
        depth['observation_sources']='depth';depth['depth']['clearing']=True
        costmap=candidate[namespace][namespace]['ros__parameters']
        costmap.pop('footprint');costmap.pop('footprint_padding');costmap['robot_radius']=.25
        assert costmap['depth_layer'].pop('footprint_clearing_enabled') is False
        assert costmap['depth_layer']['depth']['min_obstacle_height']==.02
        costmap['depth_layer']['depth']['min_obstacle_height']=.08
    for key in ('FollowPath','CleaningPath','DockingPath'):
        controller=candidate['controller_server']['ros__parameters'][key]
        controller['critics']=['BaseObstacle' if c=='ObstacleFootprint' else c for c in controller['critics']]
        # BaseObstacle uses scale directly; FootprintCritic multiplies by
        # local resolution (Humble headers verified), so compare effective units.
        effective=controller.pop('ObstacleFootprint.scale')*candidate['local_costmap']['local_costmap']['ros__parameters']['resolution']
        original=baseline['controller_server']['ros__parameters'][key]['BaseObstacle.scale']
        assert math.isclose(effective,original,rel_tol=1e-12)
        controller['BaseObstacle.scale']=original
    assert candidate==baseline
