"""Acceptance must fail for incomplete batches or mismatched saved artifacts."""
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def test_arrival_audit_rejects_missing_tf_and_pose_error():
    from scripts.validation.audit_search_results import audit_arrivals
    limits=dict(xy_m=.2,yaw_rad=.15)
    view=dict(reached=True,navigation_status=4,pose=[0.,0.,0.],
        arrival_map_pose=dict(frame_id='map',child_frame_id='base_link',
            xyz=[.1,0.,0.],xyzw=[0.,0.,0.,1.]))
    assert audit_arrivals([view,dict(reached=False)],limits)[0]['passed']
    view['arrival_map_pose']['xyz'][0]=.21
    assert not audit_arrivals([view],limits)[0]['passed']
    view['arrival_map_pose']['xyz'][0]=0.
    view['arrival_map_pose']['xyzw']=[0.,0.,1.,0.]
    assert not audit_arrivals([view],limits)[0]['passed']
    del view['arrival_map_pose']
    assert not audit_arrivals([view],limits)[0]['passed']


def test_audit_rejects_incomplete_batches_and_wrong_map_labels(tmp_path):
    def save(name,value):
        p=tmp_path/name;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(json.dumps(value));return p
    result=dict(task_id='unique',mode='find',category='cup',room='room',status='found',
        count=1,objects=[dict(id=1,x=1.,y=1.,z=.5,hits=1)],frame_id='map',coordinate_unit='m',
        synchronized_frames=1,elapsed_wall_seconds=1.,viewpoint_diagnostics=[dict(pose=[0,0,0])])
    saved=save('tasks/one/result.json',result)
    save('tasks/one/observations.json',[dict(instance_ids=[1])])
    svg=saved.with_name('semantic_map.svg');svg.write_text('<svg><text>1</text></svg>')
    matrix=dict(status='passed',planned_runs=1,unexecuted_runs=0,runs=[dict(case='find',repeat=1,
        result=result,client=dict(returncode=0),diagnostics=dict(outcome='result_received'),
        evaluation=dict(passed=True),errors=[])])
    save('matrix.json',matrix)
    save('rooms.json',{'room':dict(bounds=[0,0,2,2],views=[[0,0,0]])})
    save('layout.json',dict(targets=[dict(id='cup',category='cup',room='room',asset='cup',pose=[1,1,.5,0,0,0])],
        assets=dict(cup=dict(min=[-.05,-.05,0],max=[.05,.05,.1]))))
    save('furniture.json',dict(instances=[]))
    save('criteria.json',dict(position_limits_m=dict(cup=.2)))
    argv=[sys.executable,str(ROOT/'scripts/validation/audit_search_results.py')]
    for key,name in [('report','matrix.json'),('tasks','tasks'),('rooms','rooms.json'),
        ('layout','layout.json'),('furniture','furniture.json'),('criteria','criteria.json'),('output','audit.json')]:
        argv += ['--'+key,str(tmp_path/name)]
    def run():return subprocess.run(argv,capture_output=True,text=True)
    assert run().returncode==0
    save('rooms.json',{'room':dict(bounds=[0,0,2,2],views=[[1,0,0]])})
    assert run().returncode==1
    assert not json.loads((tmp_path/'audit.json').read_text())['cases'][0]['checks']['configured_route']
    save('rooms.json',{'room':dict(bounds=[0,0,2,2],views=[[0,0,0]])})
    svg.write_text('<svg><text>2</text></svg>')
    assert run().returncode==1
    assert not json.loads((tmp_path/'audit.json').read_text())['cases'][0]['checks']['svg_ids']
    svg.write_text('<svg><text>1</text></svg>')
    matrix.update(status='aborted',planned_runs=2,unexecuted_runs=1);save('matrix.json',matrix)
    assert run().returncode==1
    assert not json.loads((tmp_path/'audit.json').read_text())['matrices'][0]['complete']
