"""Historical approach safety and reproducibility without ROS or object truth."""
import copy
import math
import numpy as np

from semantic_search.memory_approach import propose_approach, verify_approach
from semantic_search.memory_guidance import prioritize_find_views
from semantic_search.semantic_memory import map_namespace


def fixture():
    rooms={'room':{'bounds':[0.,0.,6.,6.],'views':[[4.8,4.8,-2.],[4.9,3.8,-3.]]}}
    grid=dict(available=True,frame_id='map',width=120,height=120,resolution=.05,
              origin=dict(x=0.,y=0.,yaw=0.),data=[0]*14400)
    prior=dict(id='observed_record',state='last_observed',x=4.,y=4.,z=.4)
    return rooms,grid,prior,np.ones((120,120),dtype=bool),[.625,.625,0.]


def test_safe_approach_is_derived_from_grids_and_keeps_all_fallbacks():
    rooms,grid,prior,live,current=fixture();before=list(grid['data']);mask=live.copy()
    views=rooms['room']['views'];context=dict(rooms=rooms,room_id='room',grid=grid,available=live,current=current)
    ordered,evidence=prioritize_find_views(views,rooms['room'],[prior],approach_context=context)
    item=evidence['verification_views'][0]
    assert item['basis']=='observed_memory_safe_approach' and item['pose']==ordered[0]
    assert len(ordered)==len(views)+1 and all(v in ordered for v in views)
    assert evidence['counts_from_history'] is False
    assert item['planning']['clearance_m']>=.35
    assert 1<=math.dist(item['pose'][:2],[prior['x'],prior['y']])<=2
    assert item['planning']['estimated_approach_seconds']<.9*item['planning']['fastest_configured_estimate_seconds']
    assert verify_approach(item,views,rooms,'room',map_namespace(grid,rooms))
    assert grid['data']==before and np.array_equal(live,mask)


def test_unsafe_current_cell_and_unreachable_views_produce_no_shortcut():
    rooms,grid,prior,live,current=fixture();views=rooms['room']['views']
    live[12,12]=False
    assert propose_approach(views,rooms,'room',[prior],grid,live,current) is None
    live[:]=True;live[:,60]=False
    assert propose_approach(views,rooms,'room',[prior],grid,live,current) is None


def test_unknown_barrier_cannot_be_used_as_free_space():
    rooms,grid,prior,live,current=fixture()
    data=np.array(grid['data']).reshape(120,120);data[:,60]=-1;grid['data']=data.ravel().tolist()
    assert propose_approach(rooms['room']['views'],rooms,'room',[prior],grid,live,current) is None


def test_tampered_pose_empty_pose_and_wrong_map_are_rejected():
    rooms,grid,prior,live,current=fixture();views=rooms['room']['views']
    item=propose_approach(views,rooms,'room',[prior],grid,live,current)
    changed=copy.deepcopy(item);changed['pose'][0]+=.02
    assert not verify_approach(changed,views,rooms,'room')
    changed=copy.deepcopy(item);changed['pose']=[]
    assert not verify_approach(changed,views,rooms,'room')
    assert not verify_approach(item,views,rooms,'room','unrelated_namespace')


def test_mandatory_inspection_and_invalid_history_keep_existing_plan():
    rooms,grid,prior,live,current=fixture();room=rooms['room'];views=room['views']
    room['inspection_views']={'surface':[views[0]]}
    assert propose_approach(views,rooms,'room',[prior],grid,live,current) is None
    room.pop('inspection_views');prior['classification_conflict']=True
    assert propose_approach(views,rooms,'room',[prior],grid,live,current) is None
