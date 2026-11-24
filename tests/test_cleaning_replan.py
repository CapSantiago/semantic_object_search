import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from semantic_search.cleaning import RoomSweep

ROOMS={'living':dict(bounds=[0.,0.,2.4,1.6],aliases=['客厅'])}


def test_live_connectivity_filters_new_routes_without_reducing_eligible_ground():
    sweep=RoomSweep(np.zeros((40,48)),48,40,.05,(0,0,0),ROOMS,'living',(.6,.8,0.))
    before=sweep.metrics();live=np.ones((40,48),dtype=bool);live[:,24]=False
    pending=[(1,(.6,.5,0.),(.6,1.2,0.)),(2,(1.8,.5,0.),(1.8,1.2,0.))]
    choices=sweep.replan_options((.6,.8,0.),pending,live)
    assert choices and all(row[0]==1 for row in choices)
    assert len(choices)<=2
    assert sweep.metrics()==before
    assert not sweep.connected_centers((1.21,.8,0.),live).any()


@pytest.mark.parametrize('mode',['success','failed','no_motion','unsafe','unsettled'])
def test_replan_requires_new_safe_stopped_pose_and_measured_odom_progress(mode):
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    commands=[];state={'pose':(.6,.8,0.)}
    choices=[(2,(.9,.6,0.),(1.4,.6,0.)),(3,(.9,1.,0.),(1.4,1.,0.))]
    original_failure=dict(status='failed',action_status=6)
    worker=SimpleNamespace(replans=[],settled=mode!='unsettled',odom_distance=0.,odom_trace=[],goals=[original_failure.copy()],
        available=lambda:np.ones((2,2),dtype=bool),
        remember_safe_arrival=lambda old:None if mode=='unsafe' else state['pose'],
        sweep=SimpleNamespace(replan_options=lambda *args:choices),continue_from_partial_lane=lambda target:False)
    def go(target,deadline,**options):
        commands.append(options['purpose'])
        if mode=='failed':return False
        if mode!='no_motion':
            worker.odom_distance+=math.dist(state['pose'][:2],target[:2]);state['pose']=target
        return True
    worker.go=go
    worker.follow_lane=lambda first,last,deadline,**options:go(last,deadline,**options)
    pending=[(2,choices[0][1],choices[0][2]),(3,choices[1][1],choices[1][2])]
    output=CleaningRun.replan_lane(worker,pending,time.monotonic()+120.,{1})
    assert worker.goals==[original_failure]
    if mode=='success':
        assert output==choices[0][2]
        assert worker.replans[-1]['status']=='resumed'
        assert commands==['lane_replan_start','lane_replan_end']
        assert pending[-1][0]==2
    else:
        assert output is None
        assert len(commands)<=4
        if mode in ('unsafe','unsettled'):assert not commands
        else:assert len(worker.replans[-1]['attempts'])==2
    worker.replans=[dict(attempts=[])]*4;commands.clear()
    assert CleaningRun.replan_lane(worker,pending,time.monotonic()+120.,{1}) is None
    assert not commands
