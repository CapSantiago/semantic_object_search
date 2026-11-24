"""Coverage references retain their shape and cannot cut occupied corners."""
import math

import numpy as np
import pytest

from semantic_search.cleaning import RoomSweep


def sweep():
    rooms={'room':dict(bounds=[-2.,-2.,2.,2.])}
    return RoomSweep(np.zeros(80*80),80,80,.05,(-2.,-2.,0.),rooms,'room',(0.,0.,0.))


def test_dense_lane_keeps_requested_reference_and_original_floor_denominator():
    room=sweep();before=room.eligible.copy()
    path=room.lane_path((-.8,0.,0.),(.8,0.,0.),np.ones_like(room.centers))
    assert path[0]==(-.8,0.,0.) and path[-1]==(.8,0.,0.)
    assert all(y==0. for x,y,heading in path)
    assert max(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:]))<=.025+1e-10
    assert np.array_equal(room.eligible,before)
    assert not room.covered.any()  # A plan cannot count as physical cleaning.


def test_one_live_blocked_cell_rejects_the_entire_reference():
    room=sweep();live=np.ones_like(room.centers)
    col,row=room.index(0.,0.);live[row,col]=False
    assert room.lane_path((-.8,0.,0.),(.8,0.,0.),live) is None


def test_diagonal_reference_cannot_cut_a_blocked_grid_corner():
    room=sweep();live=np.ones_like(room.centers)
    a,b=room.point(40,40),room.point(41,41)
    live[40,41]=False
    assert room.lane_path((*a,0.),(*b,0.),live) is None


def test_unknown_or_static_unsafe_start_and_nonfinite_pose_are_rejected():
    room=sweep();live=np.ones_like(room.centers)
    assert room.lane_path((-1.95,0.,0.),(.8,0.,0.),live) is None
    assert room.lane_path((math.nan,0.,0.),(.8,0.,0.),live) is None
    assert room.lane_path(None,(.8,0.,0.),live) is None


def test_explicit_zero_tracking_margin_preserves_base_safety_and_coverage_denominator():
    before=sweep()
    rooms={'room':dict(bounds=[-2.,-2.,2.,2.],cleaning_tracking_margin_cells=0)}
    after=RoomSweep(np.zeros(80*80),80,80,.05,(-2.,-2.,0.),rooms,'room',(0.,0.,0.))
    assert before.tracking_margin_cells==1 and after.tracking_margin_cells==0
    assert np.array_equal(after.centers,before.centers)
    assert np.array_equal(after.eligible,before.eligible)
    assert after.metrics()==before.metrics() and not after.covered.any()
    assert np.array_equal(after.planning_transit_centers,after.transit_centers)
    assert after.planning_centers.sum()>before.planning_centers.sum()>0
    # An unknown live cell still rejects the path at either margin setting.
    live=np.ones_like(after.centers);c,r=after.index(0.,0.);live[r,c]=False
    assert after.lane_path((-.8,0.,0.),(.8,0.,0.),live) is None


@pytest.mark.parametrize('value',[-1,2,.5,False,'0'])
def test_invalid_tracking_margin_is_rejected_before_planning(value):
    rooms={'room':dict(bounds=[-2.,-2.,2.,2.],cleaning_tracking_margin_cells=value)}
    with pytest.raises(ValueError,match='跟踪余量'):
        RoomSweep(np.zeros(80*80),80,80,.05,(-2.,-2.,0.),rooms,'room',(0.,0.,0.))


def test_polygon_transit_candidate_rejoins_actual_arrival_without_changing_reference_floor():
    import json
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    fixture=json.loads((root/'tests/fixtures/cleaning_rear_access.json').read_text())
    grid=fixture['grid']
    arrays=np.load(root/'tests/fixtures/cleaning_rear_access.npz')
    rooms=json.loads((root/'src/semantic_search/config/rooms_aws_house_v6.json').read_text())
    rooms['gym']['cleaning_tracking_margin_cells']=0
    def build():
        return RoomSweep(arrays['static_map'],grid['width'],grid['height'],grid['resolution'],
                         (*grid['origin'],0.),rooms,'gym',(-1.8,-4.,0.))
    baseline=build();rooms['gym']['cleaning_transit_clearance_m']=.22;candidate=build()
    assert baseline.eligible_cells==candidate.eligible_cells==11156
    assert np.array_equal(baseline.centers,candidate.centers)
    assert np.array_equal(baseline.planning_centers,candidate.planning_centers)
    assert np.array_equal(baseline.eligible,candidate.eligible)
    live=(arrays['cost']>=0)&(arrays['cost']<99)
    measured=tuple(fixture['measured_arrival'])
    first=tuple(fixture['first'])
    last=tuple(fixture['last'])
    assert baseline.lane_reference(measured,first,last,live) is None
    reference=candidate.lane_reference(measured,first,last,live)
    assert reference is not None and reference['end']==pytest.approx(last)
    assert not candidate.covered.any()
    blocked=live.copy();c,r=candidate.index(*measured[:2]);blocked[r,c]=False
    assert candidate.lane_reference(measured,first,last,blocked) is None


@pytest.mark.parametrize('value',[.1,.219,.321,float('nan'),False,'0.22'])
def test_invalid_transit_candidate_rejects_unbounded_clearance(value):
    rooms={'room':dict(bounds=[-2.,-2.,2.,2.],cleaning_transit_clearance_m=value)}
    with pytest.raises(ValueError,match='连接净空'):
        RoomSweep(np.zeros(80*80),80,80,.05,(-2.,-2.,0.),rooms,'room',(0.,0.,0.))


def test_recorded_south_connector_aligns_before_entering_the_narrow_stripe():
    import json
    from pathlib import Path
    path=Path(__file__).parent/'fixtures/cleaning_connector_alignment'
    data=json.loads(path.with_suffix('.json').read_text());arrays=np.load(path.with_suffix('.npz'))
    h,w=arrays['static_map'].shape
    room=RoomSweep(arrays['static_map'],w,h,data['grid']['resolution'],data['grid']['origin'],
        data['rooms'],'gym',data['initial_pose'])
    live=(arrays['cost']>=0)&(arrays['cost']<99);before=room.metrics()
    alignment=room.connector_alignment_goal(data['current'],data['start'],data['end'],live)
    assert alignment is not None
    assert alignment['target']==pytest.approx((data['start'][0],data['current'][1],data['start'][2]))
    assert .04<alignment['lateral_error_m']<.18
    assert room.lane_path(data['current'],alignment['target'],live) is not None
    assert room.recovery_target_valid(alignment['target'],live,arrival_tolerance_m=.04)
    assert room.eligible_cells==11156 and room.metrics()==before
    blocked=live.copy();c,r=room.index(*alignment['target'][:2]);blocked[r,c]=False
    assert room.connector_alignment_goal(data['current'],data['start'],data['end'],blocked) is None
    room.align_connectors=False
    assert room.connector_alignment_goal(data['current'],data['start'],data['end'],live) is None


def test_alignment_does_not_insert_long_cross_room_or_already_aligned_moves():
    room=sweep();room.align_connectors=True;live=np.ones_like(room.centers)
    assert room.connector_alignment_goal((-.8,0.,0.),(-.8,.8,0.),(-.8,1.2,0.),live) is None
    assert room.connector_alignment_goal((-.2,0.,0.),(.8,.8,0.),(.8,1.2,0.),live) is None
    assert room.connector_alignment_goal((.7,.7,0.),(.8,.8,0.),(.8,1.2,0.),live) is None
