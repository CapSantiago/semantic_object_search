"""Remaining-floor selection must never grant planned coverage or cross a live barrier."""
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from semantic_search.cleaning import RoomSweep


def room():
    rooms={'room':dict(bounds=[0.,0.,4.,3.])}
    return RoomSweep(np.zeros(80*60),80,60,.05,(0.,0.,0.),rooms,'room',(1.,1.,0.))


def test_remaining_lanes_shift_off_a_blocked_original_stripe_and_preserve_floor():
    sweep=room();pose=(1.,1.,0.);live=np.ones_like(sweep.centers)
    original=sweep.route(pose,live)
    first,last=original[:2]
    col,row=sweep.index(*first[:2]);ec,er=sweep.index(*last[:2])
    if col==ec:live[:,col]=False
    else:live[row,:]=False
    choices=sweep.remaining_lanes(pose,live)
    assert choices
    before=sweep.metrics()
    for choice in choices:
        assert sweep.lane_path(choice['start'],choice['end'],live) is not None
        assert sweep.segment_gain(choice['start'],choice['end'])==choice['predicted_new_cells']
        assert not sweep.attempted_segment(choice['start'],choice['end'],[(first,last)])
    assert sweep.metrics()==before and not sweep.covered.any()


def test_current_connected_component_and_geodesic_distance_control_transit():
    sweep=room();live=np.ones_like(sweep.centers);live[:45,40]=False
    pose=(*sweep.point(38,20),0.)
    distances=sweep.travel_distances(pose,live)
    assert distances[20,42]>2.5  # Nearby across a barrier requires a long detour.
    live[:,40]=False
    assert math.isinf(sweep.travel_distances(pose,live)[20,42])
    choices=sweep.remaining_lanes(pose,live)
    assert choices and all(p[0]<2. for c in choices for p in (c['start'],c['end']))
    live[20,38]=False
    assert not sweep.remaining_lanes(pose,live)


@pytest.mark.parametrize('budget', [False, True])
def test_temporarily_unsafe_current_cell_preserves_unexecuted_stripes(budget):
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    sweep=room();pose=(1.,1.,0.);live=np.ones_like(sweep.centers)
    route=sweep.route(pose,live)
    pending=[(i,a,b) for i,(a,b) in enumerate(zip(route[::2],route[1::2]))]
    original=list(pending);before=sweep.metrics()
    col,row=sweep.index(*pose[:2]);live[row,col]=False
    worker=SimpleNamespace(sweep=sweep,args=SimpleNamespace(budget_selection=budget))
    assert CleaningRun.next_lanes(worker,pose,live,pending,[],100.)==[]
    assert pending==original
    assert worker.remaining_stop_reason=='selection_pose_not_navigable'
    assert sweep.metrics()==before and not sweep.covered.any()


def test_unrecoverable_selection_pose_is_a_failure_not_exhausted_floor():
    pytest.importorskip('rclpy')
    import time
    from run_cleaning import CleaningRun
    a,b=(1.,1.,0.),(1.,2.,0.)
    recoveries=[]
    worker=SimpleNamespace(original_route=[a,b],pose=lambda:a,available=lambda:None,settled=True,
        tick=lambda:None,remember_safe_arrival=lambda previous:a,
        record_navigation=lambda *args,**kwargs:None,
        recover_lane=lambda anchor,deadline:recoveries.append(anchor) or False)
    worker.sweep=SimpleNamespace(metrics=lambda:dict(coverage_fraction=0.))
    def select(current,live,pending,attempted,remaining):
        worker.remaining_stop_reason='selection_pose_not_navigable'
        assert pending==[(0,a,b)]
        return []
    worker.next_lanes=select
    assert not CleaningRun.sweep_remaining(worker,time.monotonic()+90.)
    assert recoveries==[a]


@pytest.mark.parametrize('mode', ['retry_original_queue', 'bounded_repeated_recovery'])
def test_selection_recovery_retains_queue_and_cannot_repeat_indefinitely(mode):
    pytest.importorskip('rclpy')
    import time
    from run_cleaning import CleaningRun
    from navigation_client import StopRequested
    a,b=(1.,1.,0.),(1.,2.,0.)
    recoveries=[];selections=[]
    worker=SimpleNamespace(original_route=[a,b],pose=lambda:a,available=lambda:None,settled=True,
        tick=lambda:None,remember_safe_arrival=lambda previous:a,
        record_navigation=lambda *args,**kwargs:None,
        recover_lane=lambda anchor,deadline:recoveries.append(anchor) or True)
    worker.sweep=SimpleNamespace(metrics=lambda:dict(coverage_fraction=0.))
    def select(current,live,pending,attempted,remaining):
        assert pending==[(0,a,b)]
        selections.append(list(pending))
        if recoveries and mode=='retry_original_queue':
            raise StopRequested()  # Cancellation at the next planning boundary.
        worker.remaining_stop_reason='selection_pose_not_navigable'
        return []
    worker.next_lanes=select
    if mode=='retry_original_queue':
        with pytest.raises(StopRequested):
            CleaningRun.sweep_remaining(worker,time.monotonic()+90.)
        assert recoveries==[a] and len(selections)==2
    else:
        assert not CleaningRun.sweep_remaining(worker,time.monotonic()+90.)
        assert recoveries==[a,a] and len(selections)==3


def test_connector_budget_accounts_for_observed_detour_and_keeps_a_finite_cap():
    from semantic_search.cleaning import transit_timeout
    sweep=room();live=np.ones_like(sweep.centers);live[:45,40]=False
    current=(*sweep.point(38,20),0.);target=(*sweep.point(42,20),0.)
    budget=sweep.connector_budget(current,target,live)
    assert budget['observed_path_distance_m']>2.5
    assert transit_timeout(current,target)<budget['timeout_seconds']<=180.
    live[:,40]=False
    blocked=sweep.connector_budget(current,target,live)
    assert blocked['observed_path_distance_m'] is None and blocked['source']=='euclidean_fallback'
    assert blocked['timeout_seconds']==transit_timeout(current,target)


def test_remaining_gain_uses_only_measured_uncovered_floor_and_excludes_attempts():
    sweep=room();live=np.ones_like(sweep.centers);pose=(1.,1.,0.)
    before=sweep.eligible.copy()
    sweep.covered[:,:40]=sweep.eligible[:,:40]  # A recorded already-cleaned region.
    choices=sweep.remaining_lanes(pose,live)
    assert choices and all(c['predicted_new_cells']>0 for c in choices)
    attempted=[(c['end'],c['start']) for c in choices]
    alternatives=sweep.remaining_lanes(pose,live,attempted)
    assert alternatives and all(not sweep.attempted_segment(c['start'],c['end'],attempted)
                                for c in alternatives)
    assert np.array_equal(sweep.eligible,before)
    sweep.covered[:]=sweep.eligible
    assert sweep.remaining_lanes(pose,live)==[]


def test_segment_gain_matches_physical_samples_and_does_not_fill_diagonal_rectangle():
    sweep=room()
    first=(*sweep.point(20,20),0.);last=(*sweep.point(50,20),0.)
    predicted=sweep.segment_gain(first,last)
    for col in range(20,51):sweep.touch(*sweep.point(col,20),float(col))
    assert int(sweep.covered.sum())==predicted
    assert sweep.segment_gain(first,last)==0
    sweep=room();diagonal=(*sweep.point(50,50),0.)
    assert sweep.segment_gain(first,diagonal)<sweep.segment_gain(first,last)*2


def test_decision_archive_reloads_raw_masks_without_counting_the_plan(tmp_path):
    pytest.importorskip('rclpy')
    from builtin_interfaces.msg import Time
    from run_cleaning import CleaningRun
    sweep=room();raw=np.zeros_like(sweep.occupancy);raw[30,30]=99
    worker=SimpleNamespace(sweep=sweep,navigation_arrays={},navigation_keys={},navigation_events=[],
        navigation_events_dropped=0,original_route=sweep.route((1.,1.,0.)),started=0.,
        blocked_original_lanes=set(),skipped_lanes=0,
        pose=lambda:(1.,1.,0.),node=SimpleNamespace(get_clock=lambda:SimpleNamespace(
            now=lambda:SimpleNamespace(nanoseconds=4_000_000_000))),
        costmap=SimpleNamespace(data=raw.ravel(),header=SimpleNamespace(stamp=Time(sec=4))),
        costmap_wall=0.,args=SimpleNamespace(output=tmp_path))
    before=sweep.metrics()
    event=CleaningRun.record_navigation(worker,'selection',(raw>=0)&(raw<99))
    CleaningRun.record_navigation(worker,'same_selection',(raw>=0)&(raw<99))
    CleaningRun.save_navigation(worker)
    data=np.load(tmp_path/'clean_navigation.npz',allow_pickle=False)
    saved=json.loads((tmp_path/'clean_plan.json').read_text())
    assert np.array_equal(data[event['costmap_key']],raw)
    assert not data[event['covered_key']].any()
    assert saved['events'][0]['costmap_key']==saved['events'][1]['costmap_key']
    assert len(data.files)==6  # Four fixed masks and two deduplicated live masks.
    assert sweep.metrics()==before


def test_reference_joins_original_stripe_instead_of_cutting_adjacent_blocked_cell():
    sweep=room();live=np.ones_like(sweep.centers)
    first=(*sweep.point(20,10),math.pi/2);last=(*sweep.point(20,28),math.pi/2)
    current=(*sweep.point(22,10),0.)
    live[18,21]=False
    assert sweep.lane_path(current,last,live) is None
    reference=sweep.lane_reference(current,first,last,live)
    assert reference is not None and reference['end']==last
    joined=reference['path'].index(reference['join'])
    assert all(p[0]==first[0] for p in reference['path'][joined:])
    assert max(math.dist(a[:2],b[:2]) for a,b in zip(reference['path'],reference['path'][1:]))<=.025+1e-10
    assert not sweep.covered.any()


def test_reference_rechecks_new_barrier_after_transit_and_trims_actual_goal():
    sweep=room();live=np.ones_like(sweep.centers)
    first=(*sweep.point(20,10),math.pi/2);last=(*sweep.point(20,28),math.pi/2)
    current=(*sweep.point(22,10),0.)
    assert sweep.lane_reference(current,first,last,live)['end']==last
    live[20,:]=False
    reference=sweep.lane_reference(current,first,last,live)
    assert reference is not None and sweep.index(*reference['end'][:2])==(20,19)
    assert all(sweep.index(*p[:2])[1]<20 for p in reference['path'])
    assert not sweep.covered.any()
    assert sweep.lane_reference((*sweep.point(26,10),0.),first,last,live) is None


def test_follow_lane_submits_clipped_goal_and_preserves_original_request():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    sweep=room();live=np.ones_like(sweep.centers);live[20,:]=False
    first=(*sweep.point(20,10),math.pi/2);last=(*sweep.point(20,28),math.pi/2)
    worker=SimpleNamespace(sweep=sweep,available=lambda:live,pose=lambda:(*sweep.point(22,10),0.),
        record_navigation=lambda *args,**kwargs:None,goals=[])
    def go(target,deadline,**options):
        assert target!=last and sweep.index(*target[:2])==(20,19)
        assert options['lane'] and options['skip_if_close'] and worker.lane_start==first
        worker.goals.append(dict(target=list(target),status='succeeded'))
        return True
    worker.go=go
    assert CleaningRun.follow_lane(worker,first,last,123.)
    assert worker.lane_start is None
    assert worker.goals[0]['nominal_lane']==dict(start=list(first),end=list(last))


def test_valid_direct_reference_keeps_its_interior_clearance():
    sweep=room();live=np.ones_like(sweep.centers)
    first=(*sweep.point(20,10),math.pi/2);last=(*sweep.point(20,28),math.pi/2)
    current=(*sweep.point(22,10),0.)
    ref=sweep.lane_reference(current,first,last,live)
    assert ref['source']=='direct_to_fresh_end'
    assert ref['path']==sweep.lane_path(current,last,live)
    assert not sweep.covered.any()


def test_planning_margin_preserves_original_floor_and_keeps_tracking_space():
    sweep=room();eligible=sweep.eligible.copy()
    assert sweep.planning_centers.any()
    assert not (sweep.planning_centers & ~sweep.centers).any()
    assert int(sweep.planning_centers.sum())<int(sweep.centers.sum())
    for x,y,h in sweep.route((1.,1.,0.)):
        col,row=sweep.index(x,y)
        assert all(sweep.transit_centers[r,c] for c,r in [(col-1,row),(col+1,row),(col,row-1),(col,row+1)])
    assert np.array_equal(sweep.eligible,eligible) and not sweep.covered.any()


def test_seed_pass_keeps_planned_order_before_regenerating_repairs():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    sweep=room();live=np.ones_like(sweep.centers);worker=SimpleNamespace(sweep=sweep)
    a=(*sweep.point(20,10),0.);b=(*sweep.point(40,10),0.)
    c=(*sweep.point(40,17),math.pi);d=(*sweep.point(20,17),math.pi)
    pending=[(7,a,b),(8,c,d)]
    first=CleaningRun.next_lanes(worker,a,live,pending,[])[0]
    assert first['source']=='seeded_route' and first['original_lane_id']==7
    assert first['start']==a and first['end']==b and len(pending)==1
    second=CleaningRun.next_lanes(worker,b,live,pending,[])[0]
    assert second['original_lane_id']==8 and not pending
    repairs=CleaningRun.next_lanes(worker,(1.,1.,0.),live,pending,[])
    assert repairs and all(c['source']=='remaining_floor' for c in repairs)
    assert not sweep.covered.any()


def test_seed_pass_rechecks_barrier_without_changing_denominator():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    sweep=room();live=np.ones_like(sweep.centers);live[20,:]=False
    a=(*sweep.point(20,10),math.pi/2);b=(*sweep.point(20,28),math.pi/2)
    worker=SimpleNamespace(sweep=sweep);before=sweep.metrics()
    choice=CleaningRun.next_lanes(worker,a,live,[(2,a,b)],[])[0]
    assert sweep.index(*choice['end'][:2])==(20,19)
    assert sweep.metrics()==before


def test_high_gain_repair_can_run_before_low_gain_seed_without_losing_seed():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    sweep=room();live=np.ones_like(sweep.centers);worker=SimpleNamespace(sweep=sweep,args=SimpleNamespace(budget_selection=True))
    current=(*sweep.point(14,12),0.)
    tiny=(*sweep.point(65,40),0.);tail=(*sweep.point(70,40),0.)
    repair_start=(*sweep.point(14,14),0.);repair_end=(*sweep.point(46,14),0.)
    sweep.remaining_lanes=lambda current,live,attempted,**kw:[sweep.lane_value(current,repair_start,repair_end,kw['distances'])]
    pending=[(8,tiny,tail)];before=sweep.metrics()
    chosen=CleaningRun.next_lanes(worker,current,live,pending,[],100.)[0]
    assert chosen['source']=='remaining_floor' and chosen['selection_reason']=='higher_gain_per_second'
    assert pending==[(8,tiny,tail)] and sweep.metrics()==before


def test_budget_selection_uses_a_fitting_segment_and_preserves_unfinished_work():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    sweep=room();live=np.ones_like(sweep.centers);worker=SimpleNamespace(sweep=sweep,args=SimpleNamespace(budget_selection=True))
    current=(*sweep.point(14,12),0.)
    far=(*sweep.point(60,30),0.);end=(*sweep.point(70,30),0.)
    sweep.remaining_lanes=lambda *args,**kw:[]
    pending=[(2,far,end)];before=sweep.metrics()
    assert CleaningRun.next_lanes(worker,current,live,pending,[],2.)==[]
    assert worker.remaining_stop_reason=='insufficient_segment_budget'
    assert pending==[(2,far,end)] and sweep.metrics()==before


def test_later_seed_is_promoted_when_connection_cost_makes_it_more_efficient():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    sweep=room();live=np.ones_like(sweep.centers);worker=SimpleNamespace(sweep=sweep,args=SimpleNamespace(budget_selection=True))
    current=(*sweep.point(14,12),0.)
    far=(*sweep.point(60,30),0.);end=(*sweep.point(70,30),0.)
    near=(*sweep.point(14,14),0.);long=(*sweep.point(46,14),0.)
    sweep.remaining_lanes=lambda *args,**kw:[]
    pending=[(2,far,end),(9,near,long)]
    choice=CleaningRun.next_lanes(worker,current,live,pending,[],100.)[0]
    assert choice['source']=='budget_seed' and choice['original_lane_id']==9
    assert pending==[(2,far,end)]


def test_successful_recovery_without_physical_translation_cannot_restart_sweep():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    import time
    worker=SimpleNamespace(recoveries=[],pose=lambda:(1.,2.,0.),distance=0.,odom_distance=0.,odom_trace=[],
        trace=[],args=SimpleNamespace(local_recovery=False),remember_safe_arrival=lambda previous:(1.,2.,0.))
    original=dict(status='succeeded',action_status=4);worker.goals=[original.copy()]
    worker.go=lambda *args,**kwargs:True
    assert not CleaningRun.recover_lane(worker,(1.,2.,0.),time.monotonic()+40.)
    assert worker.recoveries[-1]['status']=='no_measured_progress'
    assert worker.recoveries[-1]['measured_odom_travel_m']==0.
    assert worker.goals==[original]


@pytest.mark.parametrize('terminal', ['safe_stop', 'unsafe_pose', 'unsettled', 'unconfirmed_stop'])
def test_failed_stripe_keeps_seed_pass_only_from_a_safe_stopped_pose(terminal):
    """A failure must not insert a horizontal room crossing before the next stripe."""
    pytest.importorskip('rclpy')
    import time
    from run_cleaning import CleaningRun
    a,b,c,d=(1.,1.,0.),(1.,2.,0.),(1.35,2.,0.),(1.35,1.,0.)
    commands=[];repairs=[]
    worker=SimpleNamespace(original_route=[a,b,c,d],segment_attempts=[],goals=[],
        odom_trace=[],odom_distance=0.,settled=True,planned_goals=0,finished_goals=0,
        replans=[],pose=lambda:b,available=lambda:None,tick=lambda:None,
        record_navigation=lambda *args,**kwargs:None,
        continue_from_partial_lane=lambda target:False,
        recover_lane=lambda *args:False)
    worker.sweep=SimpleNamespace(covered=np.zeros((1,1),dtype=bool),
        metrics=lambda:dict(coverage_fraction=0.),
        connector_budget=lambda *args:dict(timeout_seconds=40.),
        remaining_lanes=lambda *args:repairs.append('regenerated') or [])
    def safe(previous):
        return None if worker.goals and terminal=='unsafe_pose' else b
    worker.remember_safe_arrival=safe
    def select(current,live,pending,attempted,remaining):
        if not pending:return []
        lane_id,start,end=pending.pop(0)
        return [dict(start=start,end=end,predicted_new_cells=1,source='seeded_route',original_lane_id=lane_id)]
    worker.next_lanes=select
    def go(target,deadline,**options):
        commands.append(target);worker.goals.append(dict(target=target,status='succeeded'))
        return True
    worker.go=go
    def follow(first,last,deadline):
        commands.append(last)
        failed=len(commands)==2
        worker.goals.append(dict(target=last,status='failed' if failed else 'succeeded',
                                 stop_confirmed=terminal!='unconfirmed_stop'))
        if failed and terminal=='unsettled':worker.settled=False
        return not failed
    worker.follow_lane=follow
    worker.replan_lane=lambda *args:repairs.append('replan')
    completed=CleaningRun.sweep_remaining(worker,time.monotonic()+120.)
    assert worker.goals[1]['status']=='failed' and not worker.sweep.covered.any()
    if terminal=='safe_stop':
        assert completed and commands==[a,b,c,d] and repairs==[]
        assert worker.segment_attempts[0]['continuation_policy']=='next_seeded_lane'
    else:
        assert not completed and commands==[a,b]
        assert repairs==['regenerated','replan']
        assert 'continuation_policy' not in worker.segment_attempts[0]


def test_residual_stripes_do_not_bridge_two_distant_covered_gaps():
    sweep=room();sweep.covered[:]=sweep.eligible
    sweep.covered[24:28,15:20]=False;sweep.covered[24:28,60:65]=False
    live=np.zeros_like(sweep.centers);live[24:26,10:70]=True
    route=sweep.route((*sweep.point(20,24),0.),live,uncovered_only=True)
    assert route
    assert all(math.dist(a[:2],b[:2])<1.5 for a,b in zip(route[::2],route[1::2]))


def test_perpendicular_supplement_preserves_actual_coverage_and_live_obstacles():
    sweep=room();live=np.ones_like(sweep.centers);before=sweep.metrics()
    original=[(1.,1.,0.),(3.,1.,0.)]
    orientation=sweep.cross_orientation(original)
    assert orientation=='vertical'
    choices=sweep.remaining_lanes((1.,1.,0.),live,preferred_orientation=orientation)
    assert choices and all(abs(c['end'][0]-c['start'][0])<1e-8 for c in choices)
    for c in choices:
        assert sweep.lane_path(c['start'],c['end'],live) is not None
        assert c['orientation']=='vertical' and c['preferred_orientation']=='vertical'
    assert sweep.metrics()==before and not sweep.covered.any()


def test_cross_orientation_falls_back_when_only_short_vertical_space_remains():
    sweep=room();live=np.zeros_like(sweep.centers);live[24:26,15:65]=True
    choices=sweep.remaining_lanes((*sweep.point(20,24),0.),live,preferred_orientation='vertical')
    assert choices and all(c['orientation']=='horizontal'for c in choices)
    assert not sweep.covered.any()


@pytest.mark.parametrize('mode',['unsafe_to_safe','historical_transition','safe_to_safe','zero','subgrid','loop','low_net','unsafe_end'])
def test_short_recovery_requires_real_unsafe_to_safe_grid_transition(mode):
    pytest.importorskip('rclpy')
    import time
    from run_cleaning import CleaningRun
    state={'after':False}
    worker=SimpleNamespace(sweep=SimpleNamespace(resolution=.05),args=SimpleNamespace(local_recovery=False),
        pose=lambda:(1.,1.,0.),recoveries=[],trace=[],odom_trace=[],goals=[],distance=0.,odom_distance=0.,
        previous_odom_pose=(1.,1.,1.))
    def check(previous):
        usable=(state['after']and mode!='unsafe_end')or mode=='safe_to_safe'
        return (1.,1.,0.)if usable else None
    worker.remember_safe_arrival=check
    def go(*args,**kwargs):
        state['after']=True
        shift=0. if mode=='zero' else .03 if mode=='subgrid'else .08
        net=0. if mode=='loop'else .01 if mode=='low_net'else .0324242260857 if mode=='historical_transition'else shift
        worker.previous_odom_pose=(2.,1.+net,1.)
        worker.odom_distance=shift;worker.distance=shift
        return True
    worker.go=go
    result=CleaningRun.recover_lane(worker,(1.1,1.,0.),time.monotonic()+60)
    assert result==(mode in ('unsafe_to_safe','historical_transition'))
    record=worker.recoveries[-1]
    if result:assert record['confirmation']=='unsafe_to_safe_with_measured_grid_translation'
    else:assert record['status']in ('no_measured_progress','unsafe_arrival')


def test_return_budget_uses_observed_ingress_with_same_bounds_and_coverage():
    from semantic_search.cleaning import observed_return_speed
    sweep=room();before=sweep.metrics();start=(12.,-3.,0.)
    assert observed_return_speed([{'velocity':[0.,0.]}]*100)is None
    assert observed_return_speed([{'velocity':[.35,0.]}]*19)is None
    speed=observed_return_speed([{'velocity':[.35,0.]}]*100+[{'velocity':[None,0.]}])
    assert speed['selected_mps']==.35 and speed['samples']==100
    baseline=sweep.return_reserve(start,720.)
    measured=sweep.return_reserve(start,720.,travel_speed_mps=speed['selected_mps'])
    assert 100.<=measured<baseline<=360.
    assert sweep.return_reserve(start,135.,travel_speed_mps=.26)==67.5
    assert sweep.return_reserve(start,720.,travel_speed_mps=.10)>=baseline
    assert sweep.metrics()==before


@pytest.mark.parametrize('value',[0.,-.1,.36,math.nan,math.inf])
def test_return_reserve_rejects_invalid_speed_estimates(value):
    with pytest.raises(ValueError):room().return_reserve((1.,1.,0.),720.,travel_speed_mps=value)



def test_return_speed_estimate_ignores_rare_fast_samples():
 from semantic_search.cleaning import observed_return_speed
 rows=[dict(velocity=[.1,0.])]*95+[dict(velocity=[.35,0.])]*5
 assert observed_return_speed(rows)['selected_mps']==.1
