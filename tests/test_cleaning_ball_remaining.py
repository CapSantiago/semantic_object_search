"""Recorded ball run: broad reachable infill must not wait for tiny edge repairs."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from semantic_search.cleaning import RoomSweep, coverage_accepted


def recorded_selection():
    z=np.load(Path(__file__).parent/'fixtures/cleaning_ball_remaining.npz')
    m=json.loads((Path(__file__).parent/'fixtures/cleaning_ball_remaining.json').read_text())
    h,w=z['occupancy'].shape
    sweep=RoomSweep(z['occupancy'].ravel(),w,h,m['resolution'],tuple(m['origin']),
                    m['rooms'],m['room'],m['initial_pose'])
    sweep.covered[:]=z['covered']
    live=(z['cost']>=0)&(z['cost']<99)
    worker=SimpleNamespace(sweep=sweep,original_route=z['original_route'].tolist())
    return worker,m['current'],live,m['attempted']


def test_recorded_infill_selected_without_crediting_or_dropping_stripes():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    worker,current,live,attempted=recorded_selection();sweep=worker.sweep
    before=sweep.covered.copy();native=live.copy();route=list(worker.original_route)
    original=sweep.required_lane_repairs(current,route,live,attempted)[0]
    assert original['original_lane_id']==5 and original['predicted_new_cells']==88
    chosen=CleaningRun.next_lanes(worker,current,live,[],attempted,64.)[0]
    assert chosen['source']=='remaining_floor' and chosen['predicted_new_cells']==213
    assert chosen['selection_reason']=='reachable_infill_before_small_stripe_gap'
    assert chosen['deferred_required_lane_id']==5
    assert sweep.lane_path(chosen['start'],chosen['end'],live) is not None
    assert np.array_equal(sweep.covered,before) and np.array_equal(live,native)
    assert worker.original_route==route and not coverage_accepted(sweep,route)
    # The original gap is still eligible for a later repair, not marked done.
    assert sweep.required_lane_repairs(current,route,live,attempted)[0]==original


def test_short_window_keeps_original_repair_instead_of_overscheduling_infill():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    worker,current,live,attempted=recorded_selection()
    chosen=CleaningRun.next_lanes(worker,current,live,[],attempted,12.)[0]
    assert chosen['source']=='required_stripe_repair' and chosen['original_lane_id']==5


def test_infill_does_not_preempt_unfinished_seed_pass():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    worker,current,live,attempted=recorded_selection()
    floor=worker.sweep.remaining_lanes(current,live,attempted)[0]
    pending=[(123,floor['start'],floor['end'])]
    chosen=CleaningRun.next_lanes(worker,current,live,pending,attempted,64.)[0]
    assert chosen['source']=='seeded_route' and chosen['original_lane_id']==123


def test_separate_supplement_replans_without_replaying_seed_or_crediting_coverage():
    pytest.importorskip('rclpy')
    import time
    from run_cleaning import CleaningRun
    worker,current,live,attempted=recorded_selection()
    before=worker.sweep.covered.copy();seen=[]
    worker.pose=lambda:current
    worker.remember_safe_arrival=lambda previous:current
    worker.available=lambda:live
    worker.tick=lambda:None
    worker.record_navigation=lambda *args,**kwargs:None
    def select(pose,mask,pending,attempted,remaining):
        seen.append(list(pending))
        return []
    worker.next_lanes=select
    worker.observe_remaining=lambda deadline:False
    assert CleaningRun.sweep_remaining(worker,time.monotonic()+60,seeded=False)
    assert seen==[[]]
    assert np.array_equal(before,worker.sweep.covered)
    seen.clear()
    worker.original_route=[]
    assert CleaningRun.sweep_remaining(worker,time.monotonic()+60,seed_only=True)
    assert not seen


def test_real_above80_supplement_can_finish_larger_infill_before_small_gap():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    base=Path(__file__).parent/'fixtures/cleaning_local_supplement_80'
    z=np.load(base.with_suffix('.npz'));m=json.loads(base.with_suffix('.json').read_text());h,w=z['occupancy'].shape
    sweep=RoomSweep(z['occupancy'].ravel(),w,h,m['resolution'],tuple(m['origin']),m['rooms'],m['room'],m['initial_pose'])
    sweep.covered[:]=z['covered'];live=(z['cost']>=0)&(z['cost']<99)
    worker=SimpleNamespace(sweep=sweep,original_route=z['original_route'].tolist())
    before=sweep.covered.copy();mask=live.copy()
    original=CleaningRun.next_lanes(worker,m['current'],live,[],m['attempted'],100.)[0]
    assert original['source']=='required_stripe_offset_repair' and original['predicted_new_cells']==21
    worker.local_remaining_pass=True
    choice=CleaningRun.next_lanes(worker,m['current'],live,[],m['attempted'],100.)[0]
    assert choice['source']=='remaining_floor' and choice['predicted_new_cells']==75
    assert sweep.lane_path(choice['start'],choice['end'],live) is not None
    assert np.array_equal(sweep.covered,before) and np.array_equal(live,mask)
    assert not coverage_accepted(sweep,worker.original_route)
