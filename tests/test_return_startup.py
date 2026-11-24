"""A newly started return client must allow DDS replies to be discovered."""
from types import SimpleNamespace

import pytest


def test_return_waits_before_sending_and_keeps_fresh_pose_stop_and_server_checks(monkeypatch):
    pytest.importorskip('rclpy')
    import run_return
    clock={'now':0.,'pose':True,'parked':True,'server':True};calls=[]
    monkeypatch.setattr(run_return.time,'monotonic',lambda:clock['now'])
    worker=SimpleNamespace(started=0.,deadline=240.,target_pose=(-1.8,-4.,0.),
        pose=lambda:(-1.8,-4.,0.) if clock['pose'] else None,
        parked=lambda:clock['parked'],client=SimpleNamespace(server_is_ready=lambda:clock['server']))
    def wait(predicate,timeout):
        assert timeout==25. and worker.settled is False
        for t in [0.,1.99]:clock['now']=t;assert not predicate()
        clock['now']=2.
        for field in ['pose','parked','server']:
            clock[field]=False;assert not predicate();clock[field]=True
        assert predicate();calls.append('ready')
    worker.wait_for=wait
    def dock(w,target,deadline):
        assert w.settled and clock['now']>=2. and deadline==240.
        calls.append('dock');return True
    monkeypatch.setattr(run_return,'dock_at',dock)
    assert run_return.ReturnRun.run(worker)[0]=='completed'
    assert calls==['ready','dock'] and worker.startup_ready_after_seconds==2.
