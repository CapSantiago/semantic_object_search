import math

import numpy as np

from semantic_search.cleaning import RoomSweep, coarse_then_infill


def stripes(n):
    return [(float(x),y,math.pi/2 if x%2==0 else -math.pi/2)
            for x in range(n) for y in ([0.,3.] if x%2==0 else [3.,0.])]


def segments(route):
    return {frozenset((tuple(a[:2]),tuple(b[:2]))) for a,b in zip(route[::2],route[1::2])}


def test_first_pass_reaches_the_far_side_before_infill_without_losing_stripes():
    original=stripes(9);route=coarse_then_infill(original,(-.2,0,0))
    assert segments(route)==segments(original) and len(route)==len(original)
    assert [a[0] for a in route[::2]]==[0.,2.,4.,6.,8.,7.,5.,3.,1.]
    for first,last in zip(route[::2],route[1::2]):
        yaw=math.atan2(last[1]-first[1],last[0]-first[0])
        assert first[2]==last[2]==yaw
    # Adjacent coarse stripes must alternate travel direction, not return
    # diagonally down the full preceding stripe just to preserve its old yaw.
    assert route[1][1]==route[2][1]


def test_room_option_preserves_floor_and_default_route_and_never_grants_coverage():
    room=dict(bounds=[0.,0.,4.,3.]);args=(np.zeros(80*60),80,60,.05,(0.,0.,0.))
    old=RoomSweep(*args,{'r':room},'r',(1.,1.,0.))
    candidate=RoomSweep(*args,{'r':dict(room,cleaning_route_order='coarse_then_infill')},'r',(1.,1.,0.))
    baseline=old.route((1.,1.,0.));selected=candidate.route((1.,1.,0.))
    assert selected==coarse_then_infill(baseline,(1.,1.,0.))
    assert np.array_equal(candidate.eligible,old.eligible)
    assert candidate.metrics()['covered_cells']==0
    assert candidate.route((1.,1.,0.),uncovered_only=True)==old.route((1.,1.,0.),uncovered_only=True)
