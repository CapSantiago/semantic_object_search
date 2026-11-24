"""No motion goal may be sent when preparation consumed its deadline."""
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
import navigation_client as client


def fixture(monkeypatch, *, preparation='normal', parked=True, stage='sweeping'):
    clock=[100.];sent=[];stopped=[]
    monkeypatch.setattr(client,'time',NS(monotonic=lambda:clock[0]))
    def prepare(target):
        if preparation=='late':clock[0]=102.
        if preparation=='stop':stopped.append(True)
        if preparation=='no_pose':worker.pose=lambda:None
        return None if preparation=='blocked' else [(0.,0.,0.),target]
    def send(goal):
        sent.append((clock[0],goal))
        return NS(done=lambda:True,result=lambda:NS(accepted=False))
    worker=NS(stopped=stopped,settled=True,pose=lambda:(0.,0.,0.),started=90.,
        stage=stage,goals=[],path_for_lane=prepare,parked=lambda:parked,
        node=NS(get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time()))),
        client=NS(send_goal_async=send),path_client=NS(send_goal_async=send),
        wait_for=lambda predicate,*args,**kwargs:predicate())
    return worker,sent


def test_path_preparation_cannot_send_after_deadline(monkeypatch):
    worker,sent=fixture(monkeypatch,preparation='late')
    assert client.NavigationRun.go(worker,(1.,0.,0.),101.,lane=True) is False
    assert not sent
    assert worker.settled  # No new action was accepted.


def test_stop_during_path_preparation_cannot_send_a_goal(monkeypatch):
    worker,sent=fixture(monkeypatch,preparation='stop')
    with pytest.raises(client.StopRequested):
        client.NavigationRun.go(worker,(1.,0.,0.),101.,lane=True)
    assert not sent and worker.settled


def test_pose_lost_during_path_preparation_cannot_send_a_goal(monkeypatch):
    worker,sent=fixture(monkeypatch,preparation='no_pose')
    with pytest.raises(ValueError):
        client.NavigationRun.go(worker,(1.,0.,0.),101.,lane=True)
    assert not sent and worker.settled


def test_blocked_reference_cannot_be_skipped_as_already_reached(monkeypatch):
    worker,sent=fixture(monkeypatch,preparation='blocked')
    assert client.NavigationRun.go(worker,(.1,0.,0.),101.,lane=True,skip_if_close=True) is False
    assert not sent and worker.goals[-1]['status']=='path_blocked'
    assert 'action_status' not in worker.goals[-1]


def test_fresh_stationary_lane_position_needs_no_new_action(monkeypatch):
    worker,sent=fixture(monkeypatch)
    assert client.NavigationRun.go(worker,(.1,0.,0.),101.,lane=True,skip_if_close=True) is True
    assert not sent and worker.goals[-1]['status']=='already_reached'
    assert 'action_status' not in worker.goals[-1]


def test_moving_base_cannot_bypass_action_and_stop_protocol(monkeypatch):
    worker,sent=fixture(monkeypatch,parked=False)
    assert client.NavigationRun.go(worker,(.1,0.,0.),101.,lane=True,skip_if_close=True) is False
    assert len(sent)==1 and worker.goals[-1]['status']=='rejected'


def test_docking_must_keep_full_pose_even_inside_xy_tolerance(monkeypatch):
    worker,sent=fixture(monkeypatch,stage='returning')
    assert client.NavigationRun.go(worker,(.1,0.,3.),101.,skip_if_close=True) is False
    assert len(sent)==1 and worker.goals[-1]['status']=='rejected'


def test_precision_docking_is_explicit_and_never_uses_position_only_completion(monkeypatch):
    worker,sent=fixture(monkeypatch,stage='returning')
    worker.station={'precision_docking':True}
    assert client.NavigationRun.go(worker,(.1,0.,3.),101.,skip_if_close=True,purpose='dock') is False
    assert len(sent)==1
    assert sent[0][1].behavior_tree.endswith('/navigate_docking.xml')
    assert worker.goals[-1]['navigation_role']=='precision_docking_full_pose'
    assert sent[0][1].pose.pose.orientation.z != 0.


def test_normal_return_keeps_original_checker_without_explicit_candidate(monkeypatch):
    worker,sent=fixture(monkeypatch,stage='returning')
    worker.station={}
    assert client.NavigationRun.go(worker,(.1,0.,3.),101.,purpose='dock') is False
    assert len(sent)==1 and not sent[0][1].behavior_tree
