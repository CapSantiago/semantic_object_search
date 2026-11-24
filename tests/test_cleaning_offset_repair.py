"""Real frozen inflated-stripe gap: keep the centre safe, credit only real brush."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.ndimage import binary_dilation
from semantic_search.cleaning import RoomSweep


def frozen_gap():
    path=Path(__file__).parent/'fixtures/cleaning_utility_offset.npz'
    z=np.load(path);m=json.loads(path.with_suffix('.json').read_text());h,w=z['occupancy'].shape
    sweep=RoomSweep(z['occupancy'].ravel(),w,h,m['resolution'],tuple(m['origin']),m['rooms'],m['room'],m['initial_pose'])
    sweep.covered[:]=z['covered'];live=(z['cost']>=0)&(z['cost']<99)
    return sweep,m['current'],z['original_route'].tolist(),live


def test_offset_brush_can_cover_forbidden_centres_without_entering_them():
    sweep,current,route,live=frozen_gap();before=sweep.covered.copy();native=live.copy()
    assert sweep.spatial_coverage(route)['covered_lanes']==25
    assert not sweep.required_lane_repairs(current,route,live)
    choices=sweep.offset_lane_repairs(current,route,live)
    assert choices and choices[0]['original_lane_id']==18
    choice=choices[0]
    assert choice['source']=='required_stripe_offset_repair' and choice['required_centerline_cells']==4
    assert np.array_equal(sweep.covered,before) and np.array_equal(live,native)
    path=sweep.lane_path(choice['start'],choice['end'],live)
    assert path is not None
    for pose in path:
        col,row=sweep.index(*pose[:2]);assert live[row,col] and sweep.planning_centers[row,col]
    # Unit geometry replay only; onsite acceptance must still use measured TF.
    for index,pose in enumerate(path):sweep.touch(*pose[:2],float(index+1))
    assert sweep.spatial_coverage(route)['completed']
    assert np.array_equal(live,native)


def test_no_offset_if_missing_floor_is_outside_all_safe_brush_reach():
    sweep,current,route,live=frozen_gap();assert not sweep.offset_lane_repairs(current,route,np.zeros_like(live))
    missing=np.zeros_like(live)
    for first,last in zip(route[::2],route[1::2]):
        a,b=sweep.index(*first[:2]),sweep.index(*last[:2]);n=max(abs(a[0]-b[0]),abs(a[1]-b[1]))+1
        cells=np.rint(np.linspace(a,b,n)).astype(int);cells=cells[~sweep.covered[cells[:,1],cells[:,0]]];missing[cells[:,1],cells[:,0]]=True
    blocked=live & ~binary_dilation(missing,iterations=sweep.reach+2)
    assert not sweep.offset_lane_repairs(current,route,blocked)


def test_offset_retries_are_bounded_and_completion_stops_planning():
    sweep,current,route,live=frozen_gap();choice=sweep.offset_lane_repairs(current,route,live)[0]
    attempted=[(choice['start'],choice['end'])]*2
    later=sweep.offset_lane_repairs(current,route,live,attempted,limit=2)
    assert len(later)<=2
    assert not any(sweep.attempted_segment(c['start'],c['end'],[(choice['start'],choice['end'])])for c in later)
    sweep.covered[:]=sweep.eligible;assert not sweep.offset_lane_repairs(current,route,live)


def test_executor_prefers_original_stripe_offset_before_general_area_repairs():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    sweep,current,route,live=frozen_gap();worker=SimpleNamespace(sweep=sweep,original_route=route)
    choices=CleaningRun.next_lanes(worker,current,live,[],[])
    assert choices and choices[0]['source']=='required_stripe_offset_repair'
    assert choices[0]['required_centerline_cells']==4
