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


def test_failed_lane_recovers_through_nav2_with_a_budget_before_new_lanes():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun

    calls = []
    def go(target,deadline,**options):
        calls.append((target,options))
        return False
    worker = SimpleNamespace(recoveries=[],pose=lambda:(1.,2.,0.),go=go,distance=0.,odom_distance=0.,odom_trace=[],trace=[],args=SimpleNamespace(local_recovery=False),remember_safe_arrival=lambda prior:(1.,2.,0.))
    safe = (.5,2.,0.)
    assert not CleaningRun.recover_lane(worker,safe,time.monotonic()+40.)
    assert calls[0][0] == safe
    assert calls[0][1]['recovery'] is True
    assert calls[0][1]['purpose']=='lane_recovery'
    assert worker.recoveries[0]['status']=='failed'
    worker.recoveries = [dict(status='succeeded')]*2
    def moved(*args,**kwargs):
        worker.odom_distance+=.5
        return True
    worker.go = moved
    assert CleaningRun.recover_lane(worker,safe,time.monotonic()+40.)
    assert worker.recoveries[-1]['status']=='succeeded'
    worker.recoveries.clear()
    assert not CleaningRun.recover_lane(worker,safe,time.monotonic()+10.)
    assert len(calls)==1


def test_partial_lane_continuation_preserves_failure_and_requires_a_known_terminal_pose():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    worker = SimpleNamespace(pose=lambda:(1.15,2.,.3),settled=True,
                             goals=[dict(status='failed',action_status=6)])
    assert CleaningRun.continue_from_partial_lane(worker,(1.,2.,0.))
    assert worker.goals[-1]['status']=='failed'
    assert worker.goals[-1]['continuation']['distance_to_target_m']==pytest.approx(.15)
    worker.settled = False
    assert not CleaningRun.continue_from_partial_lane(worker,(1.,2.,0.))
    worker.settled = True
    assert not CleaningRun.continue_from_partial_lane(worker,(.8,2.,0.))


def test_brush_overlapping_a_room_counts_only_its_actual_intersection():
    sweep = RoomSweep(np.zeros((40,48)),48,40,.05,(0,0,0),ROOMS,'living',(.8,.8,0))
    eligible = sweep.eligible_cells
    sweep.touch(1.2,1.7,1.)  # Base outside living, brush partly inside.
    assert sweep.metrics()['covered_cells']>0
    assert not sweep.covered[32:,:].any()
    assert sweep.eligible_cells==eligible


def test_position_only_completion_is_limited_to_fresh_cleaning_lane_poses():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    worker = SimpleNamespace(stage='sweeping',pose=lambda:(1.10,2.,2.9),args=SimpleNamespace(position_goals=False))
    assert not CleaningRun.lane_position(worker,(1.,2.,0.))
    worker.args.position_goals = True
    assert CleaningRun.lane_position(worker,(1.,2.,0.))
    for purpose in ('room_entry','dock','dock_alignment','lane_recovery'):
        assert not CleaningRun.lane_position(worker,(1.,2.,0.),purpose)
    assert not CleaningRun.lane_position(worker,(1.,2.,0.),recovery=True)
    worker.stage = 'returning'
    assert not CleaningRun.lane_position(worker,(1.,2.,0.))
    worker.stage = 'sweeping'
    worker.pose = lambda:None
    assert not CleaningRun.lane_position(worker,(1.,2.,0.))
    worker.pose = lambda:(1.13,2.,0.)
    assert not CleaningRun.lane_position(worker,(1.,2.,0.))


@pytest.mark.parametrize('terminal_available',[True,False])
def test_lane_position_cancel_waits_for_terminal_and_physical_stop(terminal_available):
    pytest.importorskip('rclpy')
    from rclpy.time import Time
    from run_cleaning import CleaningRun
    events = []
    state = dict(pose=(0.,2.,0.),canceled=False,parked=False)
    result = SimpleNamespace(done=lambda:state['canceled'] and terminal_available,
        result=lambda:SimpleNamespace(status=5) if state['canceled'] and terminal_available else None)
    def cancel():
        state['canceled'] = True
        events.append('cancel')
    handle = SimpleNamespace(accepted=True,get_result_async=lambda:result,cancel_goal_async=cancel)
    def send(goal):
        events.append('send')
        return SimpleNamespace(done=lambda:True,result=lambda:handle)
    def wait(predicate,timeout,**kwargs):
        if predicate():return
        if state['canceled'] and not terminal_available:raise TimeoutError('terminal missing')
        if not state['canceled']:
            state['pose'] = (1.10,2.,2.9)
        else:
            state['parked'] = True
            events.append('fresh_stopped_odometry')
        assert predicate()
    worker = SimpleNamespace(stopped=[],started=time.monotonic(),stage='sweeping',goals=[],settled=True,
        args=SimpleNamespace(position_goals=True),
        pose=lambda:state['pose'],parked=lambda:state['parked'],client=SimpleNamespace(send_goal_async=send),
        node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time())),wait_for=wait)
    worker.lane_position = lambda target,purpose,recovery:CleaningRun.lane_position(worker,target,purpose,recovery)
    if terminal_available:
        assert CleaningRun.go(worker,(1.,2.,0.),time.monotonic()+20)
        assert events==['send','cancel','fresh_stopped_odometry']
        assert worker.goals[-1]['status']=='position_reached'
        assert worker.goals[-1]['action_status']==5
    else:
        with pytest.raises(TimeoutError):
            CleaningRun.go(worker,(1.,2.,0.),time.monotonic()+20)
        assert not worker.settled
        assert events.count('send')==1
        assert 'fresh_stopped_odometry' not in events


@pytest.mark.parametrize('ending', ['timeout', 'abort', 'position', 'success'])
@pytest.mark.parametrize('stop_available', [True, False])
def test_terminal_action_requires_fresh_stop_before_recovery(ending, stop_available):
    """A Nav2 result may precede the child's stop; never send over that gap."""
    pytest.importorskip('rclpy')
    from rclpy.time import Time
    from run_cleaning import CleaningRun

    events = []
    state = dict(terminal=ending in ('abort','success'), stopped=False, canceled=False)
    result = SimpleNamespace(done=lambda: state['terminal'],
        result=lambda: SimpleNamespace(status=6 if ending == 'abort' else 4 if ending == 'success' else 5))
    def cancel():
        events.append('cancel')
        state.update(terminal=True, canceled=True)
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result, cancel_goal_async=cancel)
    def send(goal):
        events.append('send')
        return SimpleNamespace(done=lambda: True, result=lambda: handle)
    def wait(predicate, timeout, **kwargs):
        if predicate():
            return
        if not state['terminal']:
            raise TimeoutError('navigation deadline')
        events.append('wait_for_stop')
        if not stop_available:
            raise TimeoutError('fresh stopped odometry unavailable')
        state['stopped'] = True
        assert predicate()
    worker = SimpleNamespace(stopped=[], started=time.monotonic(), stage='sweeping', goals=[], settled=True,
        args=SimpleNamespace(position_goals=ending == 'position'), stationary_since=0,
        pose=lambda: (1.10, 2., 0.), parked=lambda: state['stopped'],
        client=SimpleNamespace(send_goal_async=send),
        node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: Time())), wait_for=wait)
    worker.lane_position = lambda target, purpose, recovery: CleaningRun.lane_position(worker, target, purpose, recovery)
    if stop_available:
        assert CleaningRun.go(worker, (1., 2., 0.), time.monotonic()+20) == (ending in ('position','success'))
        assert worker.settled
        assert worker.goals[-1]['stop_confirmed'] is True
        assert worker.goals[-1]['timeout_seconds'] == 40.
        assert worker.goals[-1]['effective_timeout_seconds'] <= 20.
        assert worker.goals[-1]['status'] == {'timeout': 'timed_out', 'abort': 'failed', 'position': 'position_reached','success':'succeeded'}[ending]
        assert events[-1] == 'wait_for_stop'
    else:
        with pytest.raises(TimeoutError, match='odometry'):
            CleaningRun.go(worker, (1., 2., 0.), time.monotonic()+20)
        assert not worker.settled
        assert worker.goals[-1]['stop_confirmed'] is False
        assert 'odometry' in worker.goals[-1]['stop_error']
        with pytest.raises(RuntimeError):
            CleaningRun.go(worker, (0., 2., 0.), time.monotonic()+20, recovery=True)
        assert events.count('send') == 1


def test_parked_rejects_stale_or_moving_odometry():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    worker = SimpleNamespace(stationary_since=0, odom_stamp=500_000_000, odom_wall=time.monotonic()-1.)
    assert not CleaningRun.parked(worker)
    worker.odom_wall = time.monotonic()
    assert CleaningRun.parked(worker)
    worker.stationary_since = None
    assert not CleaningRun.parked(worker)


@pytest.mark.parametrize('user_cancel', [False, True])
def test_late_goal_acceptance_is_canceled_and_stopped_before_return(user_cancel):
    pytest.importorskip('rclpy')
    from rclpy.time import Time
    from run_cleaning import CleaningRun, StopRequested
    events, stopped = [], []
    state = dict(accepted=False, terminal=False, parked=False)
    result = SimpleNamespace(done=lambda: state['terminal'], result=lambda: SimpleNamespace(status=5))
    def cancel():
        events.append('cancel')
        state['terminal'] = True
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result, cancel_goal_async=cancel)
    sent = SimpleNamespace(done=lambda: state['accepted'], result=lambda: handle)
    def wait(predicate, timeout, **options):
        if predicate():
            return
        if not events:
            events.append('interrupted')
            if user_cancel:
                stopped.append(True)
                raise StopRequested()
            raise TimeoutError('late acceptance')
        assert options.get('check_stop') is False
        if not state['accepted']:
            events.append('late_acceptance')
            state['accepted'] = True
        else:
            events.append('fresh_stop')
            state['parked'] = True
        assert predicate()
    worker = SimpleNamespace(stopped=stopped, started=time.monotonic(), stage='sweeping', goals=[], settled=True,
        args=SimpleNamespace(position_goals=False), pose=lambda: (1., 2., 0.),
        parked=lambda: state['parked'], client=SimpleNamespace(send_goal_async=lambda goal: sent),
        node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: Time())), wait_for=wait)
    worker.lane_position = lambda *args: False
    if user_cancel:
        with pytest.raises(StopRequested):
            CleaningRun.go(worker, (4., 2., 0.), time.monotonic()+20)
    else:
        assert not CleaningRun.go(worker, (4., 2., 0.), time.monotonic()+20)
    assert events == ['interrupted', 'late_acceptance', 'cancel', 'fresh_stop']
    assert worker.settled and worker.goals[-1]['stop_confirmed']
    assert worker.goals[-1]['action_status'] == 5


@pytest.mark.parametrize('local_anchor, fallback_free', [(True, True), (False, True), (False, False)])
def test_long_recovery_uses_one_live_checked_goal_and_records_measured_travel(local_anchor, fallback_free):
    pytest.importorskip('rclpy')
    from rclpy.time import Time
    from run_cleaning import CleaningRun
    calls = []
    available = np.ones((2, 8), dtype=bool)
    available[0, 0] = fallback_free
    candidate = dict(pose=(3.5, 0., 0.), trace_index=4, sim_seconds=9., retreat_path_m=.5)
    def select(current, trace, live, now):
        assert live is available and now == 10.
        return candidate if local_anchor else None
    worker = SimpleNamespace(recoveries=[], pose=lambda: (4., 0., 0.), distance=12., odom_distance=12., odom_trace=[], trace=[],
        args=SimpleNamespace(local_recovery=True),
        available=lambda: available, node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10))),
        sweep=SimpleNamespace(recovery_anchor=select, index=lambda x,y: (0,0), centers=np.ones_like(available)))
    def go(target, deadline, **options):
        calls.append((target, options))
        worker.distance += .6
        worker.odom_distance += .6
        return True
    worker.go = go
    worker.remember_safe_arrival = lambda prior: worker.pose()
    assert CleaningRun.recover_lane(worker, (0.,0.,0.), time.monotonic()+50.) == (local_anchor or fallback_free)
    record = worker.recoveries[-1]
    if local_anchor or fallback_free:
        assert len(calls) == 1
        assert calls[0][1]['recovery'] is True
        assert calls[0][0] == (candidate['pose'] if local_anchor else (0.,0.,0.))
        assert record['source'] == ('recent_trace' if local_anchor else 'previous_arrival')
        assert record['measured_travel_m'] == pytest.approx(.6)
        assert record['previous_arrival_distance_m'] == 4.
        assert record['timeout_seconds'] == 45.
        assert record['trace_start_index'] == record['trace_end_index'] == 0
    else:
        assert not calls
        assert record['status'] == 'unavailable'
        assert record['reason'] == 'previous_arrival_blocked'


def test_local_recovery_experiment_requires_explicit_opt_in():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    calls = []
    worker = SimpleNamespace(recoveries=[], pose=lambda: (4., 0., 0.), distance=0., odom_distance=0., odom_trace=[], trace=[],
        args=SimpleNamespace(local_recovery=False),
        remember_safe_arrival=lambda prior:(0.,0.,0.))
    def moved(target,deadline,**options):
        calls.append((target,options));worker.odom_distance+=.5
        return True
    worker.go=moved
    assert CleaningRun.recover_lane(worker, (0., 0., 0.), time.monotonic()+50.)
    assert len(calls) == 1 and calls[0][0] == (0., 0., 0.)
    assert worker.recoveries[-1]['source'] == 'previous_arrival'


@pytest.mark.parametrize('condition', ['valid', 'static_blocked', 'live_blocked', 'outside', 'stale_pose'])
def test_only_measured_navigable_arrivals_replace_recovery_anchors(condition):
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    live = np.ones((4,4), dtype=bool)
    centers = live.copy()
    if condition == 'static_blocked': centers[1,1] = False
    if condition == 'live_blocked': live[1,1] = False
    measured = None if condition == 'stale_pose' else ((-1.,1.,0.) if condition == 'outside' else (1.,1.,0.))
    previous = (2.,2.,0.)
    worker = SimpleNamespace(pose=lambda:measured,available=lambda:live,
        sweep=SimpleNamespace(centers=centers,index=lambda x,y:(int(x),int(y))),
        goals=[dict(status='succeeded',action_status=4)])
    selected = CleaningRun.remember_safe_arrival(worker,previous)
    assert selected == (measured if condition == 'valid' else previous)
    assert worker.goals[-1]['action_status'] == 4
    assert worker.goals[-1]['recovery_anchor_check']['accepted'] == (condition == 'valid')
    worker.goals.clear()
    assert CleaningRun.remember_safe_arrival(worker,None) == (measured if condition == 'valid' else None)
    assert worker.initial_anchor_check['accepted'] == (condition == 'valid')


def test_successful_recovery_at_an_unsafe_pose_does_not_claim_escape():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    calls = []
    worker = SimpleNamespace(recoveries=[],pose=lambda:(1.,2.,0.),distance=0.,odom_distance=0.,odom_trace=[],trace=[],
        args=SimpleNamespace(local_recovery=False),remember_safe_arrival=lambda prior:None,
        go=lambda target, deadline, **options:calls.append(target) or True)
    assert not CleaningRun.recover_lane(worker,(1.,2.,0.),time.monotonic()+40)
    assert len(calls) == 1
    assert worker.recoveries[-1]['status'] == 'unsafe_arrival'


@pytest.mark.parametrize('initial', [(.1606,0.,0.),(0.,0.,.22)])
def test_successful_dock_reply_does_not_skip_measured_error_recovery(initial):
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    state = dict(pose=initial)
    commands = []
    def go(target, deadline, *, purpose, **options):
        commands.append(purpose)
        if purpose == 'dock_approach':
            state['pose'] = (.08,0.,target[2])
        elif purpose == 'dock_alignment':
            state['pose'] = (.08,0.,.05)
        return True  # Parent action succeeds while measured pose is still bad.
    worker = SimpleNamespace(pose=lambda:state['pose'],go=go,rooms=ROOMS,
        args=SimpleNamespace(room='living'),start_pose=(0.,0.,0.),deadline=time.monotonic()+120.,
        wait_for=lambda *args:None,parked=lambda:True,odom_stamp=500_000_000,stationary_since=0)
    assert CleaningRun.return_to_start(worker,None)
    assert commands == (['dock','dock_approach','dock_alignment'] if initial[0] else ['dock','dock_alignment'])
    assert worker.return_state == 'simulated_charging'
    assert worker.return_errors['position_m'] <= .15 and worker.return_errors['yaw_rad'] <= .15


def test_room_transit_budget_allows_the_actual_kitchen_to_bedroom_distance():
    from semantic_search.cleaning import transit_timeout
    start = (8.807870555967726,2.119846779673029,-.5974358423)
    entry = (-2.9,-.5,math.pi)
    assert 80 < transit_timeout(start,entry) <= 180
    assert transit_timeout((0.,0.,0.),(0.,1.3,0.)) == 40.
    assert transit_timeout((0.,0.),(100.,0.)) == 180.
    with pytest.raises(ValueError):
        transit_timeout((float('nan'),0.),(1.,0.))


def test_distant_start_reserves_return_time_without_changing_cleanable_cells():
    sweep = RoomSweep(np.zeros((40,48)),48,40,.05,(0,0,0),ROOMS,'living',(.8,.8,0))
    before = sweep.metrics()
    near = sweep.return_reserve((.8,.8,0.),720.)
    far = sweep.return_reserve((15.,0.,0.),720.)
    assert near == 100. and near < far <= 210.
    assert sweep.return_reserve((15.,0.,0.),135.) == 67.5
    assert sweep.metrics() == before
