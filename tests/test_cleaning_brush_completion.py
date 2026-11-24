import math,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from rclpy.time import Time
from semantic_search.cleaning import RoomSweep
from navigation_client import NavigationRun as Nav
from run_cleaning import CleaningRun as Clean

def worker():
 sweep=RoomSweep(np.zeros(80*60),80,60,.05,(0.,0.,0.),{'room':dict(bounds=[0.,0.,4.,3.])},'room',(1.,1.,0.))
 now=Time(seconds=10);w=NS(stage='sweeping',parked=lambda:True,costmap_ready=lambda:True,available=lambda:np.ones_like(sweep.centers),lane_start=(1.,1.,0.),sweep=sweep,odom_wall=time.monotonic(),odom_stamp=now.nanoseconds,node=NS(get_clock=lambda:NS(now=lambda:now)),tf=NS(lookup_transform=lambda *args:NS(header=NS(stamp=now.to_msg()))),pose=lambda:(1.,1.8,math.pi/2))
 return w

def test_brush_requires_entire_actual_stripe_and_near_fresh_end():
 w=worker();end=(1.,2.,0.);before=w.sweep.metrics();assert Clean.cleaning_completion(w,end)is None
 for y in np.arange(1.,1.81,.05):w.sweep.touch(1.,y,y)
 evidence=Clean.cleaning_completion(w,end);assert evidence and evidence['spatial_coverage']['completed']
 metrics=w.sweep.metrics();assert metrics!=before
 assert Clean.cleaning_completion(w,end) and w.sweep.metrics()==metrics
 col,row=w.sweep.index(1.,1.5);w.sweep.covered[row,col]=False
 assert Clean.cleaning_completion(w,end)is None

@pytest.mark.parametrize('mode',['old_odom','old_tf','returning','dock','recovery','far','no_pose','nan','not_parked','blocked','outside_planning','stale_cost'])
def test_brush_rejects_stale_and_non_cleaning_states(mode):
 w=worker();w.sweep.covered[:]=w.sweep.eligible;kwargs={}
 if mode=='old_odom':w.odom_wall-=.5
 if mode=='old_tf':w.tf=NS(lookup_transform=lambda *args:NS(header=NS(stamp=Time(seconds=9).to_msg())))
 if mode=='returning':w.stage='returning'
 if mode=='dock':kwargs['purpose']='dock'
 if mode=='recovery':kwargs['recovery']=True
 if mode=='far':w.pose=lambda:(1.,1.7,0.)
 if mode=='no_pose':w.pose=lambda:None
 if mode=='not_parked':w.parked=lambda:False
 if mode=='blocked':w.available=lambda:np.zeros_like(w.sweep.centers)
 if mode=='outside_planning':w.sweep.planning_centers[:]=False
 if mode=='stale_cost':w.costmap_ready=lambda:False
 if mode=='nan':w.pose=lambda:(math.nan,1.8,0.)
 assert Clean.cleaning_completion(w,(1.,2.,0.),**kwargs)is None

@pytest.mark.parametrize('mode',['normal','lost_evidence','no_terminal','still_moving','native_abort'])
def test_brush_cancellation_keeps_native_status_and_fresh_stop(mode):
 state=dict(canceled=False,parked=False);events=[]
 result=NS(done=lambda:mode=='native_abort'or state['canceled']and mode!='no_terminal',result=lambda:NS(status=6 if mode=='native_abort'else 5))
 def cancel():state['canceled']=True;events.append('cancel')
 handle=NS(accepted=True,get_result_async=lambda:result,cancel_goal_async=cancel)
 client=NS(send_goal_async=lambda goal:NS(done=lambda:True,result=lambda:handle))
 def wait(predicate,timeout,**kwargs):
  if predicate():return
  if mode in ('no_terminal','still_moving'):raise TimeoutError(mode)
  state['parked']=True;events.append('fresh_stop');assert predicate()
 w=NS(stopped=[],started=time.monotonic(),stage='sweeping',goals=[],settled=True,pose=lambda:(1.,1.8,0.),parked=lambda:state['parked'],node=NS(get_clock=lambda:NS(now=lambda:Time(seconds=10))),wait_for=wait,path_client=client,client=client,path_for_lane=lambda target:[(1.,1.,0.),(1.,2.,0.)],lane_position=lambda *args:False)
 w.cleaning_completion=lambda *args:None if mode=='lost_evidence'and state['parked']else dict(basis='actual_coverage',spatial_coverage=dict(completed=True))
 if mode in ('no_terminal','still_moving'):
  with pytest.raises(TimeoutError):Nav.go(w,(1.,2.,0.),time.monotonic()+30,lane=True)
  assert not w.settled
 else:
  assert Nav.go(w,(1.,2.,0.),time.monotonic()+30,lane=True)==(mode=='normal')
  row=w.goals[-1];assert row['action_status']==(6 if mode=='native_abort'else 5) and row['stop_confirmed']
  assert row['status']==('brush_covered'if mode=='normal'else'failed')
  assert 'position_completion'not in row


@pytest.mark.parametrize('mode',['spin_stopped_translation','moving_translation','gapped_history'])
def test_covered_stripe_may_stop_spinning_but_not_moving_or_missing_odometry(mode):
 w=worker();w.parked=lambda:False;w.sweep.covered[:]=w.sweep.eligible
 w.odom_trace=[dict(stamp_ns=10_000_000_000-i*20_000_000,velocity=[.02 if mode=='moving_translation'else 0.,.3])for i in range(26)][::-1]
 if mode=='gapped_history':w.odom_trace=w.odom_trace[:1]+w.odom_trace[-1:]
 assert bool(Clean.cleaning_completion(w,(1.,2.,0.)))==(mode=='spin_stopped_translation')
