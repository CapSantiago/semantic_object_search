"""A one-cell original-stripe gap must not disappear behind area thresholds."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
from semantic_search.cleaning import RoomSweep


def isolated_real_gap():
    path=Path(__file__).parent/'fixtures/cleaning_required_stripe.npz'
    z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text())
    occupancy=z['occupancy'];h,w=occupancy.shape;route=z['original_route'].tolist()
    sweep=RoomSweep(occupancy.ravel(),w,h,meta['resolution'],tuple(meta['origin']),
                    meta['rooms'],'gym',meta['initial_pose'])
    # The actual recorded lane 9 had 43/44 cells brushed. Isolate that real
    # missing cell from other uncovered floor to test the area/space conflict.
    first,last=route[18:20];a,b=sweep.index(*first[:2]),sweep.index(*last[:2])
    count=max(abs(a[0]-b[0]),abs(a[1]-b[1]))+1
    cells=np.rint(np.linspace(a,b,count)).astype(int)
    missing=[(x,y)for x,y in cells if not z['covered'][y,x]]
    assert len(missing)==1
    x,y=missing[0];sweep.covered[:]=sweep.eligible;sweep.covered[y,x]=False
    return sweep,route,missing[0]


def test_required_original_cell_bypasses_area_threshold_without_granting_coverage():
    sweep,route,cell=isolated_real_gap();before=sweep.covered.copy();metrics=sweep.metrics()
    assert metrics['coverage_fraction']>.99 and not sweep.spatial_coverage(route)['completed']
    choices=sweep.required_lane_repairs(route[0],route,sweep.free)
    assert choices
    chosen=choices[0]
    assert chosen['source']=='required_stripe_repair' and chosen['original_lane_id']==9
    assert chosen['predicted_new_cells']==1 and chosen['predicted_new_cells']*sweep.resolution**2<.04
    assert math.dist(chosen['start'][:2],chosen['end'][:2])>=.14
    assert sweep.lane_path(chosen['start'],chosen['end'],sweep.free) is not None
    assert np.array_equal(sweep.covered,before) and sweep.metrics()==metrics
    sweep.touch(*chosen['start'][:2],1.);sweep.touch(*chosen['end'][:2],2.)
    assert sweep.covered[cell[1],cell[0]] and sweep.spatial_coverage(route)['completed']


def test_required_repairs_remain_blocked_by_live_obstacle_and_bounded_retries():
    sweep,route,(x,y)=isolated_real_gap();live=sweep.free.copy();live[y,x]=False
    assert not sweep.required_lane_repairs(route[0],route,live)
    choices=sweep.required_lane_repairs(route[0],route,sweep.free)
    assert choices
    attempted=[(choices[0]['start'],choices[0]['end'])]*2
    assert not sweep.required_lane_repairs(route[0],route,sweep.free,attempted)


def test_executor_selects_required_gap_after_seed_pass_before_area_repairs():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from run_cleaning import CleaningRun
    sweep,route,_=isolated_real_gap();before=sweep.covered.copy()
    # Current area-only regeneration really returns no candidate for this gap.
    assert not sweep.remaining_lanes(route[0],sweep.free)
    worker=SimpleNamespace(sweep=sweep,original_route=route)
    selected=CleaningRun.next_lanes(worker,route[0],sweep.free,[],[])
    assert selected and selected[0]['source']=='required_stripe_repair'
    assert selected[0]['predicted_new_cells']==1
    assert np.array_equal(before,sweep.covered)
