import numpy as np

from semantic_search.cleaning import RoomSweep, TARGET_COVERAGE, coverage_accepted


def test_eighty_percent_cannot_hide_a_missed_stripe_and_a_plan_never_cleans():
    sweep=RoomSweep(np.zeros(80*60),80,60,.05,(0,0,0),
        {'r':dict(bounds=[0,0,4,3])},'r',(1,1,0))
    route=sweep.route((1,1,0))
    assert TARGET_COVERAGE==.80 and not coverage_accepted(sweep,route)
    assert sweep.spatial_coverage(route)['covered_lanes']==0
    sweep.covered[:]=sweep.eligible
    c,r=sweep.index(*route[0][:2]);sweep.covered[r,c]=False
    assert sweep.metrics()['coverage_fraction']>.99
    spatial=sweep.spatial_coverage(route)
    assert not spatial['completed'] and spatial['incomplete_lanes'][0]['lane_id']==0
    assert not coverage_accepted(sweep,route)
    sweep.touch(*route[0][:2],1.)
    assert coverage_accepted(sweep,route)
    assert sweep.spatial_coverage(route)['covered_lanes']==len(route)//2
    assert not coverage_accepted(sweep,[])


def test_later_blockage_does_not_erase_unvisited_original_stripe_cells():
    sweep=RoomSweep(np.zeros(80*60),80,60,.05,(0,0,0),
        {'r':dict(bounds=[0,0,4,3])},'r',(1,1,0))
    route=sweep.route((1,1,0));before=sweep.spatial_coverage(route)
    live=np.zeros_like(sweep.centers)
    assert sweep.remaining_lanes((1,1,0),live)==[]
    assert sweep.spatial_coverage(route)==before
    assert not before['completed']
