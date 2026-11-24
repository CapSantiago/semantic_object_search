"""Measured recovery anchors must not bridge gaps or change coverage."""
import math

import numpy as np
import pytest

from semantic_search.cleaning import RoomSweep


@pytest.fixture
def sweep():
    rooms = {'living': {'bounds': [0., 0., 5., 5.], 'aliases': ['客厅']}}
    return RoomSweep(np.zeros((100, 100)), 100, 100, .05, (0., 0., 0.),
                     rooms, 'living', (2., 2., 0.))


def trail(points, start=10., step=.2):
    return [dict(pose=(*point, .7), sim_seconds=start+index*step, connected=index > 0)
            for index, point in enumerate(points)]


def test_recovery_selects_latest_valid_retreat_and_preserves_yaw_and_coverage(sweep):
    trace = trail([(1., 2.), (1.25, 2.), (1.5, 2.), (1.75, 2.), (2., 2.)])
    before = sweep.metrics()
    anchor = sweep.recovery_anchor((2.05, 2., -.2), trace, sweep.centers.copy(), 10.9)
    assert anchor == dict(pose=(1.5, 2., .7), trace_index=2, sim_seconds=10.4,
                          retreat_path_m=pytest.approx(.55))
    assert sweep.metrics() == before
    assert not sweep.covered.any()


@pytest.mark.parametrize('problem', ['gap', 'clock_reverse', 'time_gap', 'position_jump'])
def test_recovery_does_not_cross_discontinuous_trace(sweep, problem):
    trace = trail([(1.25, 2.), (1.5, 2.), (1.75, 2.), (2., 2.)])
    if problem == 'gap':
        trace[-2]['connected'] = False
    elif problem == 'clock_reverse':
        trace[-3]['sim_seconds'] = trace[-2]['sim_seconds']+.1
    elif problem == 'time_gap':
        trace[-3]['sim_seconds'] = trace[-2]['sim_seconds']-1.
    else:
        trace[-3]['pose'] = (1.3, 2., .7)
    assert sweep.recovery_anchor((2., 2., 0.), trace, sweep.centers, 10.7) is None


@pytest.mark.parametrize('current,now', [((2., 2., 0.), 12.), ((2.31, 2., 0.), 10.7),
                                         ((2., 2., 0.), 10.5)])
def test_recovery_requires_latest_sample_to_connect_to_current_pose(sweep, current, now):
    trace = trail([(1.25, 2.), (1.5, 2.), (1.75, 2.), (2., 2.)])
    assert sweep.recovery_anchor(current, trace, sweep.centers, now) is None


def test_recovery_skips_newly_blocked_endpoints_and_rejects_all_blocked(sweep):
    trace = trail([(1.25, 2.), (1.5, 2.), (1.75, 2.), (2., 2.)])
    available = sweep.centers.copy()
    col, row = sweep.index(1.5, 2.)
    available[row, col] = False
    anchor = sweep.recovery_anchor((2., 2., 0.), trace, available, 10.7)
    assert anchor['pose'] == (1.25, 2., .7)
    assert sweep.recovery_anchor((2., 2., 0.), trace, np.zeros_like(available), 10.7) is None
    sweep.centers[:] = False
    assert sweep.recovery_anchor((2., 2., 0.), trace, np.ones_like(available), 10.7) is None


def test_recovery_limits_accumulated_retreat_not_only_endpoint_distance(sweep):
    # A historical loop stays within .4 m of the current pose until the final
    # candidate, but reaching that candidate backwards would exceed 1.5 m.
    backward = [(2., 2.), (2.2, 2.), (2.2, 2.2), (2., 2.2), (2., 2.),
                (1.8, 2.), (1.8, 1.8), (2., 1.8), (2., 2.), (1.75, 2.), (1.5, 2.)]
    trace = trail(backward[::-1])
    assert sweep.recovery_anchor((2., 2., 0.), trace, sweep.centers, 12.1) is None


def test_recovery_rejects_old_trace_despite_stationary_continuity(sweep):
    points = [(1.5, 2.), (1.75, 2.)]+[(2., 2.)]*160
    trace = trail(points)
    assert sweep.recovery_anchor((2., 2., 0.), trace, sweep.centers, trace[-1]['sim_seconds']+.1) is None


@pytest.mark.parametrize('bad_value', [math.nan, math.inf])
def test_recovery_rejects_nonfinite_samples_and_current_time(sweep, bad_value):
    trace = trail([(1.5, 2.), (1.75, 2.), (2., 2.)])
    assert sweep.recovery_anchor((2., 2., 0.), trace, sweep.centers, bad_value) is None
    trace[-2]['pose'] = (bad_value, 2., 0.)
    assert sweep.recovery_anchor((2., 2., 0.), trace, sweep.centers, 10.5) is None


def test_recorded_native_success_can_stop_in_blocked_cell_so_anchor_needs_arrival_margin():
    import json
    from pathlib import Path
    path=Path(__file__).parent/'fixtures/cleaning_recovery_arrival'
    data=json.loads(path.with_suffix('.json').read_text())
    arrays=np.load(path.with_suffix('.npz'))
    h,w=arrays['static_map'].shape
    room=RoomSweep(arrays['static_map'],w,h,data['grid']['resolution'],data['grid']['origin'],
        data['rooms'],'gym',data['initial_pose'])
    live=(arrays['cost']>=0)&(arrays['cost']<99)
    before=room.metrics()
    assert not room.recovery_target_valid(data['previous_anchor'],live,arrival_tolerance_m=.12)
    anchor=room.recovery_anchor(data['unsafe_current'],data['trace'],live,data['now_sim_seconds'],
        arrival_tolerance_m=.12)
    assert anchor is not None
    assert room.recovery_target_valid(anchor['pose'],live,arrival_tolerance_m=.12)
    # Every grid cell that a .12m arrival may occupy must remain traversable.
    for angle in np.linspace(0.,2*math.pi,73):
        x=anchor['pose'][0]+.12*math.cos(angle);y=anchor['pose'][1]+.12*math.sin(angle)
        c,r=room.index(x,y)
        assert live[r,c] and room.centers[r,c]
    assert room.metrics()==before and room.eligible_cells==11156


def test_arrival_margin_rejects_unknown_static_obstacles_and_outside_map(sweep):
    live=np.ones_like(sweep.centers)
    target=(2.,2.,0.)
    assert sweep.recovery_target_valid(target,live,arrival_tolerance_m=.12)
    c,r=sweep.index(2.1,2.);live[r,c]=False
    assert not sweep.recovery_target_valid(target,live,arrival_tolerance_m=.12)
    assert not sweep.recovery_target_valid((.01,2.,0.),live,arrival_tolerance_m=.12)
    assert not sweep.recovery_target_valid((math.nan,2.,0.),live,arrival_tolerance_m=.12)


def test_recovery_replaces_recorded_tolerance_trap_with_buffered_measured_anchor():
    pytest.importorskip('rclpy')
    import json
    import time
    from pathlib import Path
    from types import SimpleNamespace
    from run_cleaning import CleaningRun
    path=Path(__file__).parent/'fixtures/cleaning_recovery_arrival'
    data=json.loads(path.with_suffix('.json').read_text());arrays=np.load(path.with_suffix('.npz'))
    h,w=arrays['static_map'].shape
    room=RoomSweep(arrays['static_map'],w,h,data['grid']['resolution'],data['grid']['origin'],
        data['rooms'],'gym',data['initial_pose'])
    live=(arrays['cost']>=0)&(arrays['cost']<99)
    state={'pose':data['unsafe_current']};sent=[]
    worker=SimpleNamespace(sweep=room,pose=lambda:state['pose'],available=lambda:live,
        recoveries=[],goals=[dict(status='path_blocked',action_status=None)],
        trace=data['trace'],odom_trace=[],odom_distance=0.,distance=0.,
        previous_odom_pose=None,args=SimpleNamespace(local_recovery=False),
        remember_safe_arrival=lambda previous:state['pose'] if sent else None,
        node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(
            nanoseconds=int(data['now_sim_seconds']*1e9)))))
    def go(target,deadline,**options):
        sent.append(target)
        assert room.recovery_target_valid(target,live)
        worker.odom_distance+=math.dist(state['pose'][:2],target[:2]);state['pose']=target
        return True
    worker.go=go
    before=room.metrics()
    assert CleaningRun.recover_lane(worker,data['previous_anchor'],time.monotonic()+60.)
    assert len(sent)==1 and sent[0]!=data['previous_anchor']
    assert worker.recoveries[-1]['source']=='recent_trace_with_arrival_margin'
    assert worker.goals==[dict(status='path_blocked',action_status=None)]
    assert room.metrics()==before
