"""Inset commanded centres within brush reach; never shorten acceptance lines."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from semantic_search.cleaning import RoomSweep

ROOT=Path(__file__).resolve().parents[1]


def fixture():
    path=Path(__file__).parent/'fixtures/cleaning_turn_endpoint.npz'
    z=np.load(path);m=json.loads(path.with_suffix('.json').read_text());h,w=z['occupancy'].shape
    s=RoomSweep(z['occupancy'].ravel(),w,h,m['resolution'],m['origin'],m['rooms'],'gym',m['initial_pose'])
    return s,m,z['cost'],(z['cost']>=0)&(z['cost']<99)


def test_real_failed_turn_gets_safe_inset_and_keeps_full_original_brush_reference():
    s,m,cost,live=fixture();original=[m['first'],m['last']];before=s.metrics();eligible=s.eligible.copy()
    v=s.execution_segment(*original,live,cost)
    assert v['start_inset_m']==pytest.approx(.15) and v['end_inset_m']==0.
    assert v['reference_start']==original[0] and v['reference_end']==original[1]
    assert not v['point_only'] and s.lane_path(v['start'],v['end'],live) is not None
    assert s.metrics()==before and np.array_equal(s.eligible,eligible)
    count=math.ceil(math.dist(v['start'][:2],v['end'][:2])/.02)+1
    for i,p in enumerate(np.linspace(v['start'][:2],v['end'][:2],count)):
        s.touch(*p,1.+i*.1)
    assert s.spatial_coverage(original)['completed']


def test_current_body_geometry_and_native_tolerance_fit_the_declared_turn_circle():
    cfg=yaml.safe_load((ROOT/'src/semantic_search/config/nav2_body_footprint.yaml').read_text())
    local=cfg['local_costmap']['local_costmap']['ros__parameters']
    points=np.asarray(json.loads(local['footprint']));points+=np.sign(points)*local['footprint_padding']
    native=cfg['controller_server']['ros__parameters']['cleaning_goal_checker']['xy_goal_tolerance']
    room=json.loads((ROOT/'src/semantic_search/config/rooms_aws_house_v12_body_transit.json').read_text())['gym']
    assert np.linalg.norm(points,axis=1).max()+native<=room['cleaning_turning_radius_m']
    assert .15+native+local['resolution']*math.sqrt(2)/2<.25


def test_unknown_target_never_becomes_a_new_execution_endpoint():
    s,m,cost,live=fixture();v=s.execution_segment(m['first'],m['last'],live,cost)
    x,y=s.index(*v['start'][:2]);cost=cost.copy();cost[y,x]=-1;live=live.copy();live[y,x]=False
    changed=s.execution_segment(m['first'],m['last'],live,cost)
    assert changed['start_inset_m']==0. or live[s.index(*changed['start'][:2])[1],s.index(*changed['start'][:2])[0]]
    assert not s.covered.any()


def test_large_entry_turn_is_inset_even_before_low_obstacle_is_seen():
    s,m,cost,live=fixture();cost=np.where(s.free,0,100);live=s.free.copy()
    v=s.execution_segment(m['first'],m['last'],live,cost,current=(0.,1.3,math.pi/2))
    assert v['approach_turn'] and v['start_inset_m']==pytest.approx(.15)
    assert not s.covered.any()
