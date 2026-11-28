#!/usr/bin/env python3
"""Serial real UI acceptance; scene truth is read only after each saved result."""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

from scipy.optimize import linear_sum_assignment

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/semantic_search'))
from semantic_search.rooms import observation_views,room_at
from audit_search_results import audit_arrivals,target_reference


def docked(value):
    error=value.get('return_error') or {}
    return (value.get('return_state')=='simulated_charging' and value.get('navigation_settled') is True
            and error.get('position_m',math.inf)<=.15 and error.get('yaw_rad',math.inf)<=.15
            and error.get('stationary_seconds',0)>=.45)


def configured_route_matches(result,room,category,*,rooms=None,room_id=None):
    """Find may add one evidenced heading at a configured site; count may not."""
    baseline=observation_views(room,category)
    actual=result.get('viewpoints',[])
    if result.get('mode')=='count':return actual==baseline
    guidance=result.get('memory_guidance')or{}
    extras=guidance.get('verification_views')or[]
    if len(extras)>1 or guidance.get('added_viewpoints',0)!=len(extras):return False
    additions=[]
    for item in extras:
        pose=item.get('pose',[]);source=item.get('source_configured_pose')
        if (not guidance.get('enabled')or not guidance.get('used')or guidance.get('counts_from_history')is not False
                or item.get('memory_id')not in guidance.get('memory_ids',[])
                or source not in baseline or len(pose)!=3
                or not all(type(v)in(int,float)and math.isfinite(v)for v in pose)):
            return False
        if item.get('basis')=='observed_memory_safe_approach':
            from semantic_search.memory_approach import verify_approach
            if rooms is None or not verify_approach(item,baseline,rooms,room_id,guidance.get('namespace'),guidance.get('historical_successful_views')):
                return False
        elif pose[:2]!=source[:2]:return False
        additions.append(pose)
    diagnostics=result.get('viewpoint_diagnostics')or[]
    planned=[v['pose']for v in diagnostics]if diagnostics else actual
    if Counter(map(tuple,planned))!=Counter(map(tuple,baseline+additions)):return False
    if result.get('status')=='found':
        # A one-object find legitimately returns before visiting every fallback.
        # The complete plan remains in diagnostics; only reached poses belong
        # in the public visited-view list.
        return bool(diagnostics)and actual==[v['pose']for v in diagnostics if v.get('reached')]
    return Counter(map(tuple,actual))==Counter(map(tuple,baseline+additions))


def evaluate(task,case,session,meta):
    """The caller must supply a finalized task, never a pending prediction."""
    result=task.get('result') or {};objects=result.get('objects',[])
    layout=json.loads((session/meta['configuration_snapshots'][meta['config']['layout']]).read_text())
    rooms=json.loads((session/meta['configuration_snapshots'][meta['config']['rooms']]).read_text())
    reference=[target_reference(t,layout['assets']) for t in layout['targets']
               if t['room']==case['room'] and t['category']==case['category']]
    if case['category'] in ('sports ball','chair'):
        prefix='Ball_' if case['category']=='sports ball' else 'Chair'
        for obj in json.loads((ROOT/'scenes/aws_house/ground_truth.json').read_text())['instances']:
            if not obj['name'].startswith(prefix):continue
            pose=layout.get('scene_model_poses',{}).get(obj['name'],obj['pose'])
            if room_at(rooms,*pose[:2])==case['room']:reference.append(dict(id=obj['name'],xy=pose[:2]))
    expected=min(1,len(reference)) if result.get('mode')=='find' else len(reference)
    costs=[[math.dist([o['x'],o['y']],t['xy']) for t in reference] for o in objects];errors=[]
    if costs and reference:
        a,b=linear_sum_assignment(costs)
        errors=[dict(id=objects[i]['id'],reference=reference[j]['id'],xy_error_m=costs[i][j]) for i,j in zip(a,b)]
    arrivals=audit_arrivals(result.get('viewpoint_diagnostics',[]),dict(xy_m=.20,yaw_rad=.15))
    checks=dict(task_completed=task['state']=='completed',count=result.get('count')==expected,
        position=len(errors)==expected and all(x['xy_error_m']<=case.get('position_limit_m',.20) for x in errors),
        unique_ids=len({o['id'] for o in objects})==len(objects),
        map_metres=result.get('frame_id')=='map' and result.get('coordinate_unit')=='m',
        room=all(room_at(rooms,o['x'],o['y'])==case['room'] for o in objects),
        required_surfaces=result.get('inspection_coverage',{}).get('completed') is True,
        measured_arrivals=bool(arrivals) and all(v['passed'] for v in arrivals),
        saved=task.get('save_status')=='saved',memory_saved=task.get('memory',{}).get('status')=='saved',
        return_passed=docked(task.get('return') or {}),
        configured_route=configured_route_matches(result,rooms[case['room']],case['category'],rooms=rooms,room_id=case['room']))
    if result.get('mode')=='count' or not expected:
        coverage=result.get('coverage') or {}
        checks.update(full_observation_plan=coverage.get('observation_plan_completed') is True,
            full_observation_data=coverage.get('observation_data_complete') is True)
    else:checks['found']=result.get('status')=='found'
    if result.get('mode')=='count':checks['no_history_count']=result.get('memory_guidance',{}).get('counts_from_history') is False
    matches=[]
    for path in (session/'tasks').glob('*/result.json'):
        saved=json.loads(path.read_text())
        if saved.get('task_id')==result.get('task_id'):matches.append((path,saved))
    checks['persisted_result']=len(matches)==1 and matches[0][1]==result
    if len(matches)==1:
        folder=matches[0][0].parent
        frames=json.loads((folder/'observations.json').read_text())
        hits=Counter(i for frame in frames for i in frame['instance_ids'])
        checks['observed_hits']=all(hits[o['id']]==o['hits'] for o in objects)
        labels=[e.text for e in ET.parse(folder/'semantic_map.svg').getroot().iter() if e.tag.rsplit('}',1)[-1]=='text']
        checks['svg_ids']=sorted(labels)==sorted(str(o['id'])for o in objects)
    return dict(passed=all(checks.values()),expected=expected,actual=result.get('count'),checks=checks,
        position_limit_m=case.get('position_limit_m',.20),errors=errors,arrivals=arrivals)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--tag',required=True)
    parser.add_argument('--count-repeats',type=int,default=5)
    parser.add_argument('--find-repeats',type=int,default=1)
    parser.add_argument('--port',type=int,default=8767)
    args=parser.parse_args()
    if min(args.count_repeats,args.find_repeats)<1:parser.error('Positive repetition counts required')
    args.output.mkdir(parents=True,exist_ok=False);cases=json.loads(args.cases.read_text())
    report=dict(status='starting',pid=os.getpid(),tag=args.tag,cases=[],skipped=[],
        criteria=dict(count_repeats=args.count_repeats,find_repeats=args.find_repeats,arrival_xy_m=.20,
                      arrival_yaw_rad=.15,selected_repeated_case='full_count',case_definitions=cases),
        driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    stopped=False;active=None
    def stop(*unused):
        nonlocal stopped
        stopped=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    def save():
        path=args.output/'report.tmp';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');path.replace(args.output/'report.json')
    def api(path,body=None):
        request=urllib.request.Request(f'http://127.0.0.1:{args.port}'+path,data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json'})
        return json.load(urllib.request.urlopen(request,timeout=6))
    def wait(predicate,seconds):
        end=time.monotonic()+seconds;last_save=0
        while time.monotonic()<end:
            if stopped:raise InterruptedError('Acceptance interrupted')
            state=api('/api/state');value=predicate(state)
            if value:return value
            if state['phase']=='failed':raise RuntimeError(state.get('error'))
            if time.monotonic()-last_save>4:report['last_task']=state.get('task');save();last_save=time.monotonic()
            time.sleep(.25)
        raise TimeoutError('Acceptance watchdog')
    failed=set()
    try:
        state=wait(lambda s:s if s['ready'] and not s['busy'] else None,220)
        session=Path(state['output_directory']);meta=json.loads((session/'session.json').read_text())
        report.update(status='running',session=str(session),configuration=meta['config'],source_sha256=meta['source_sha256']);save()
        for repeat in range(max(args.count_repeats,args.find_repeats)):
            for case in cases:
                for mode,limit in [('find',args.find_repeats),('count',args.count_repeats)]:
                    if repeat>=limit:continue
                    key=(case['room'],case['category'],mode)
                    if key in failed:
                        report['skipped'].append(dict(case=case,mode=mode,repeat=repeat+1,reason='prior_failure_requires_repair'));save();continue
                    state=api('/api/state');assert state['ready'] and not state['busy'] and state['output_directory']==str(session),'Unexpected owner/task state'
                    active=f'{args.tag}_{len(report["cases"]):03d}'
                    api('/api/task',dict(request_id=active,mode=mode,room=case['room'],category=case['category']))
                    def done(s):
                        task=s.get('task')or{}
                        return task if task.get('id')==active and task['state'] in ('completed','partial','failed','canceled') else None
                    task=wait(done,750);active=None
                    row=dict(case=case,mode=mode,repeat=repeat+1,task=task);report['cases'].append(row);save()
                    row['evaluation']=evaluate(task,case,session,meta);save()
                    print(case['room'],case['category'],mode,repeat+1,row['evaluation'],flush=True)
                    assert row['evaluation']['checks']['return_passed'],'Return failure: do not continue'
                    if not row['evaluation']['passed']:failed.add(key)
        report.update(status='finished',all_checks_passed=not failed and not report['skipped']);save()
    except BaseException as error:
        report.update(status='interrupted' if stopped else 'failed',error=f'{type(error).__name__}: {error}');save()
        if active:
            try:
                state=api('/api/state')
                if state['busy'] and state.get('task',{}).get('id')==active:api('/api/cancel',{})
            except Exception:pass
        raise
    finally:save()


if __name__=='__main__':main()
