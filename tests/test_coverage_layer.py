from types import SimpleNamespace
import hashlib
import numpy as np
import pytest
from semantic_search.coverage_layer import CoverageDisplay,encode_runs


def decode(runs,n):
    m=np.zeros(n,dtype=bool)
    for first,size in runs:m[first:first+size]=True
    return m


def test_display_roundtrip_matches_actual_mask_and_never_expands_denominator():
    grid=np.array([[0,0,-1],[100,0,0]],dtype=np.int8)
    eligible=grid==0;s=SimpleNamespace(occupancy=grid,eligible=eligible,resolution=.05,origin=(1.,2.,.1))
    view=CoverageDisplay(s);covered=np.array([[True,False,False],[False,False,True]])
    raw=view.snapshot(covered,stamp_seconds=3.,blocked=np.ones_like(grid),cost_stamp_seconds=2.9)
    assert np.array_equal(decode(raw['covered_runs'],6).reshape(grid.shape),covered)
    assert np.array_equal(decode(raw['eligible_runs'],6).reshape(grid.shape),eligible)
    assert raw['covered_cells']==2 and raw['eligible_cells']==4
    assert np.array_equal(decode(raw['blocked_runs'],6).reshape(grid.shape),eligible)
    assert raw['data_sha256']==hashlib.sha256(grid.tobytes()).hexdigest()
    assert encode_runs(np.zeros(6,dtype=bool))==[]
    with pytest.raises(ValueError):view.snapshot(np.ones_like(grid),stamp_seconds=4.)


def test_overlay_digest_matches_actual_monitor_grid_payload():
    from robot_monitor import map_payload
    from types import SimpleNamespace as NS
    grid=np.array([[0,-1],[100,0]],dtype=np.int8)
    message=NS(info=NS(width=2,height=2,resolution=.05,
        origin=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),
        data=list(grid.ravel()),header=NS(frame_id='map',stamp=NS(sec=1,nanosec=0)))
    sweep=NS(occupancy=grid,eligible=grid==0,resolution=.05,origin=(0.,0.,0.))
    assert map_payload(message)['data_sha256']==CoverageDisplay(sweep).base['data_sha256']
