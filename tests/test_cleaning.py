import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from semantic_search.cleaning import RoomSweep, lane_groups, parse_clean_command


ROOMS = {
    'living': {'bounds': [0, 0, 2.4, 1.6], 'aliases': ['客厅']},
    'bedroom': {'bounds': [0, 1.6, 2.4, 2.0], 'aliases': ['卧室']},
}


def test_cleaning_commands_select_one_room_and_refuse_mixed_intent():
    assert parse_clean_command('请帮我去客厅扫地。', ROOMS) == 'living'
    assert parse_clean_command('打扫一下卧室', ROOMS) == 'bedroom'
    assert parse_clean_command('去客厅找杯子', ROOMS) is None
    for text in ('不要清扫卧室', '清扫客厅和卧室', '扫地', '清扫客厅再找杯子'):
        with pytest.raises(ValueError):
            parse_clean_command(text, ROOMS)


def test_planning_preserves_unreachable_floor_and_does_not_count_it_as_cleaned():
    grid = np.zeros((40,48),dtype=np.int8)
    grid[:,24] = 100
    grid[4:8,4:8] = -1
    sweep = RoomSweep(grid,48,40,.05,(0,0,0),ROOMS,'living',(.6,.8,0))
    route = sweep.route((.6,.8,0))
    assert route and all(0 < x < 1.2 for x,_,_ in route)
    assert sweep.metrics()['coverage_fraction']==0
    assert sweep.eligible_cells < sweep.floor_cells
    assert not sweep.eligible[6,6]
    assert not sweep.eligible[:,24:].any()
    assert sweep.repair_goal((.6,.8,0),[])


def test_coverage_uses_position_measurements_and_distinguishes_later_revisits():
    sweep = RoomSweep(np.zeros((40,48)),48,40,.05,(0,0,0),ROOMS,'living',(.8,.8,0))
    sweep.touch(.8,.8,1.)
    first = sweep.metrics()
    assert 0 < first['covered_cells'] < first['eligible_cells']
    sweep.touch(.8,.8,1.2)
    assert first==sweep.metrics()
    sweep.touch(1.5,1.,1.5)
    assert sweep.metrics()['covered_cells'] > first['covered_cells']
    sweep.touch(.8,.8,3.)
    assert sweep.metrics()['repeated_area_m2']==first['covered_area_m2']


def test_boustrophedon_groups_keep_each_side_of_an_obstacle_together():
    lanes = [(0,0,50),(5,0,25),(5,30,50),(10,0,26),(10,30,50),(15,0,50)]
    assert lane_groups(lanes,5)==[
        [lanes[0]], [lanes[1],lanes[3]], [lanes[2],lanes[4]], [lanes[5]]]


def test_live_costmap_splits_free_lanes_without_reducing_the_coverage_denominator():
    sweep = RoomSweep(np.zeros((40,48)),48,40,.05,(0,0,0),ROOMS,'living',(.8,.8,0))
    available = sweep.centers.copy()
    available[:,22:26] = False
    first = (*sweep.point(8,18),0.)
    last = (*sweep.point(38,18),0.)
    expected = sweep.eligible_cells
    pieces = sweep.lane_segments(first,last,available)
    assert len(pieces) == 2
    assert sweep.index(*pieces[0][1][:2]) == (21,18)
    assert sweep.index(*pieces[1][0][:2]) == (26,18)
    reverse = sweep.lane_segments(last,first,available)
    assert reverse == [(b,a) for a,b in pieces[::-1]]
    assert sweep.eligible_cells == expected


def test_return_uses_exit_then_recovers_near_start_without_relaxing_docking_limits():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun

    poses = iter(((1.3,.6,0.),(.4,.1,2.),(.06,0.,.30)))
    results = iter((True,False,True,True))
    commands = []

    def go(target, deadline, *, purpose, **kwargs):
        commands.append((target,purpose))
        return next(results)

    worker = SimpleNamespace(
        pose=lambda: next(poses), go=go, rooms=ROOMS,args=SimpleNamespace(room='living'),
        start_pose=(0.,0.,0.),deadline=time.monotonic()+90,
        wait_for=lambda *args: None, parked=lambda: True,
        odom_stamp=501_000_000,stationary_since=0)
    exit_pose = (1.30,1.70,math.pi/2)
    assert not CleaningRun.return_to_start(worker,exit_pose)
    assert worker.return_state == 'return_incomplete'
    assert worker.return_errors == pytest.approx(dict(position_m=.06,yaw_rad=.30,stationary_seconds=.501))
    assert [purpose for _,purpose in commands] == [
        'room_exit','dock','dock_approach','dock_alignment']
    assert commands[0][0] == exit_pose
    assert commands[1][0] == commands[-1][0] == worker.start_pose
