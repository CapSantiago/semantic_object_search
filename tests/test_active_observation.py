import math

import numpy as np
from scipy.spatial.transform import Rotation

from semantic_search.active_observation import ActiveObservation


def observer(grid=None, **options):
    grid = np.zeros((80,100)) if grid is None else grid
    rooms = {'room':dict(bounds=[0,0,5,4],aliases=['房间'])}
    return ActiveObservation(grid,100,80,.05,(0,0,0),rooms,'room',(1,1,0),**options)


def measure(active, depth):
    # Optical forward -> map x, optical right -> map -y.
    rotation = Rotation.from_euler('xyz',[-math.pi/2,0,-math.pi/2]).as_quat()
    active.observe(depth,[40,0,31.5,0,40,23.5,0,0,1],[1,1,1.2],rotation,[1,1,0])


def test_measured_depth_updates_coverage_and_predictions_never_mark_samples_observed():
    active = observer(max_views=1)
    assert active.next_view((1,1,0),[],np.ones((80,100),bool)) is None
    assert active.reason == 'no_measured_camera'
    measure(active,np.full((48,64),np.nan))
    assert not active.seen.any()
    measure(active,np.full((48,64),2.))
    initial = active.seen.copy()
    assert 0 < initial.sum() < len(initial)
    pose = active.next_view((1,1,0),[(1,1,0)],np.ones((80,100),bool))
    assert pose is not None
    assert np.array_equal(active.seen,initial)
    assert active.next_view(pose,[],np.ones((80,100),bool)) is None
    assert active.reason == 'view_budget_exhausted'
    assert not active.evidence()['full_room_coverage_verified']


def test_new_views_respect_live_connectivity_and_do_not_cross_an_unobserved_barrier():
    active = observer()
    measure(active,np.full((48,64),1.))
    available = np.ones((80,100),bool)
    available[:,40:45] = False
    pose = active.next_view((1,1,0),[],available)
    assert pose is not None and pose[0]<2.
    available[:,:] = False
    assert active.next_view((1,1,0),[],available) is None
    assert active.reason == 'current_pose_not_in_safe_component'


def test_predicted_line_of_sight_stops_at_occupied_and_unknown_cells():
    grid = np.zeros((80,100))
    grid[20:50,40] = 100
    grid[50:,40] = -1
    active = observer(grid)
    points = np.array([[1.5,1.4,.85],[3,1.4,.85],[3,3,.85]])
    assert active._line_of_sight((1,1.4),points).tolist() == [True,False,False]


def test_completed_sample_target_stops_planning_but_does_not_claim_complete_recognition():
    active = observer()
    measure(active,np.full((48,64),3.))
    active.seen[:] = True
    assert active.next_view((1,1,0),[],np.ones((80,100),bool)) is None
    assert active.summary()['observed_fraction']==1.
    assert active.reason == 'sample_target_reached'
    assert not active.summary()['full_room_coverage_verified']


def test_tentative_measured_target_can_request_two_closer_views_without_confirming_it():
    active = observer()
    measure(active,np.full((48,64),3.))
    active.seen[:] = True
    before = active.seen.copy()
    available = np.ones((80,100),bool)
    target = [(7,3.,2.,.75)]
    attempted = []
    for _ in range(2):
        pose = active.next_view((1,1,0),attempted,available,target)
        assert pose is not None
        assert .8<=math.dist(pose[:2],target[0][1:3])<=2.2
        assert active.decisions[-1]['source']=='tentative_object_recheck'
        assert active.decisions[-1]['tentative_id']==7
        assert np.array_equal(active.seen,before)
        attempted.append(pose)
    assert active.next_view((1,1,0),attempted,available,target) is None
    assert active.summary()['tentative_recheck_views']==2


def test_tentative_target_recheck_keeps_live_safe_connectivity():
    active = observer()
    measure(active,np.full((48,64),3.))
    available = np.ones((80,100),bool)
    available[:,40:] = False
    pose = active.next_view((1,1,0),[],available,[(9,3.,2.,.75)])
    assert pose is not None and pose[0]<2.
    available[:] = False
    assert active.next_view((1,1,0),[],available,[(9,3.,2.,.75)]) is None


def test_measured_support_inspection_does_not_claim_a_target_or_change_observed_coverage():
    active = observer()
    measure(active,np.full((48,64),3.))
    before = active.seen.copy()
    pose = active.next_view((1,1,0),[],np.ones((80,100),bool),supports=[(4,3.,2.,.6)])
    assert pose is not None
    decision = active.decisions[-1]
    assert decision['source']=='measured_support_surface'
    assert decision['tentative_id'] is None
    assert active.summary()['tentative_recheck_views']==0
    assert active.summary()['support_surface_views']==1
    assert np.array_equal(before,active.seen)
