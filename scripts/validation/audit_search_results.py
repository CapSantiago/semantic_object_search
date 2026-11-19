#!/usr/bin/env python3
"""Post-run acceptance audit; scene fixtures are never supplied to the robot."""
import argparse
from collections import Counter
import hashlib
import json
from math import atan2, cos, hypot, isfinite, sin
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from scipy.optimize import linear_sum_assignment

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/semantic_search'))
from semantic_search.rooms import room_at, observation_views


def read(path):return json.loads(Path(path).read_text())


def audit_arrivals(views, limits):
    """Check measured arrivals, including a find task's visited prefix only."""
    rows=[]
    for index, view in enumerate(views):
        if not view.get('reached'):
            continue
        row=dict(index=index,passed=False)
        try:
            pose=view['arrival_map_pose']; xyz=pose['xyz']; q=pose['xyzw']; target=view['pose']
            if (pose['frame_id']!='map' or pose['child_frame_id']!='base_link'
                or len(xyz)!=3 or len(q)!=4 or len(target)!=3
                or not all(isfinite(v) for v in xyz+q+target)
                or abs(sum(v*v for v in q)-1.)>.01):
                raise ValueError('Invalid measured arrival transform')
            yaw=atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
            xy=hypot(xyz[0]-target[0],xyz[1]-target[1])
            angle=abs(atan2(sin(yaw-target[2]),cos(yaw-target[2])))
            row.update(xy_error_m=xy,yaw_error_rad=angle,
                passed=view.get('navigation_status')==4 and xy<=limits['xy_m'] and angle<=limits['yaw_rad'])
        except (KeyError,TypeError,ValueError) as error:
            row['error']=str(error)
        rows.append(row)
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,action='append',required=True)
    p.add_argument('--tasks',type=Path,action='append',required=True)
    p.add_argument('--rooms',type=Path,required=True)
    p.add_argument('--layout',type=Path,required=True)
    p.add_argument('--furniture',type=Path,required=True)
    p.add_argument('--criteria',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();rooms=read(args.rooms);layout=read(args.layout);criteria=read(args.criteria)
    truth=[]
    for t in layout['targets']:
        a=layout['assets'][t['asset']]
        if any(abs(v)>1e-9 for v in t['pose'][3:]):
            raise ValueError('Rotate asset bounds before auditing nonzero-angle fixtures')
        truth.append(dict(id=t['id'],category=t['category'],room=t['room'],
            xy=[t['pose'][i]+(a['min'][i]+a['max'][i])/2 for i in (0,1)]))
    for t in read(args.furniture)['instances']:
        if t['name'].startswith('Chair'):
            truth.append(dict(id=t['name'],category='chair',room=room_at(rooms,*t['pose'][:2]),xy=t['pose'][:2]))
    saved={}
    for directory in args.tasks:
        for path in directory.glob('*/result.json'):
            saved.setdefault(read(path)['task_id'],[]).append(path)
    cases=[];seen=set();matrices=[]
    for report_path in args.report:
        matrix=read(report_path)
        matrices.append(dict(path=str(report_path),complete=matrix.get('status') in ('passed','failed')
            and matrix.get('unexecuted_runs')==0 and len(matrix['runs'])==matrix['planned_runs']))
        for row in matrix['runs']:
            r=row.get('result')
            expected_exit=0 if (r or {}).get('status') in ('found','completed') else 1
            checks=dict(client_ok=not row.get('errors') and row.get('client',{}).get('returncode')==expected_exit
                        and (row.get('diagnostics') or {}).get('outcome')=='result_received',
                        expected_result=row.get('evaluation',{}).get('passed') is True)
            record=dict(case=row['case'],repeat=row['repeat'],report=str(report_path),checks=checks)
            if r is None:
                checks['result_present']=False;record['passed']=False;cases.append(record);continue
            matches=saved.get(r['task_id'],[])
            checks.update(unique_task_id=r['task_id'] not in seen,
                configured_route=[v.get('pose') for v in r.get('viewpoint_diagnostics',[])]
                    == observation_views(rooms[r['room']],r['category']),
                unique_object_ids=len({o['id'] for o in r['objects']})==r['count'],
                map_metres=r['frame_id']=='map' and r['coordinate_unit']=='m',
                correct_room=all(room_at(rooms,o['x'],o['y'])==r['room'] for o in r['objects']),
                synchronized_data=r['synchronized_frames']>0,persisted_result=len(matches)==1)
            seen.add(r['task_id'])
            if 'arrival_audit_limits' in criteria:
                arrivals=audit_arrivals(r.get('viewpoint_diagnostics',[]),criteria['arrival_audit_limits'])
                record['arrival_evaluation']=arrivals
                checks['measured_arrivals']=bool(arrivals) and all(v['passed'] for v in arrivals)
            if r['status'] in ('completed','not_found'):
                checks['observed_route']=r['coverage']['observation_plan_completed'] and r['coverage']['observation_data_complete']
            if len(matches)==1:
                result_path=matches[0];folder=result_path.parent
                try:
                    checks['persisted_result'] &= read(result_path)==r
                    labels=[e.text for e in ET.parse(folder/'semantic_map.svg').getroot().iter() if e.tag.rsplit('}',1)[-1]=='text']
                    checks['svg_ids']=sorted(labels)==sorted(str(o['id']) for o in r['objects'])
                    hits=Counter(i for f in read(folder/'observations.json') for i in f['instance_ids'])
                    checks['observation_hits']=all(hits[o['id']]==o['hits'] for o in r['objects'])
                    record['saved_result']=str(result_path)
                    record['sha256']=hashlib.sha256(result_path.read_bytes()).hexdigest()
                except (OSError,ValueError,ET.ParseError) as e:
                    checks['artifacts_readable']=False;record['artifact_error']=str(e)
            reference=[t for t in truth if t['category']==r['category'] and t['room']==r['room']]
            expected=min(1,len(reference)) if r['mode']=='find' else len(reference)
            checks['fixture_count']=r['count']==expected
            errors=[]
            if r['objects'] and reference:
                costs=[[hypot(o['x']-t['xy'][0],o['y']-t['xy'][1]) for t in reference] for o in r['objects']]
                rows,cols=linear_sum_assignment(costs)
                errors=[dict(id=r['objects'][i]['id'],reference=reference[j]['id'],xy_error_m=costs[i][j]) for i,j in zip(rows,cols)]
                checks['position']=len(errors)==r['count'] and all(e['xy_error_m']<=criteria['position_limits_m'][r['category']] for e in errors)
            record.update(task_id=r['task_id'],status=r['status'],count=r['count'],xy_evaluation=errors,
                          elapsed_wall_seconds=r['elapsed_wall_seconds'],passed=all(checks.values()))
            cases.append(record)
    report=dict(status='passed' if cases and all(c['passed'] for c in cases)
                and all(m['complete'] for m in matrices) else 'failed',
        method='Post-run matrix expectations, saved JSON/SVG/observations, room ownership and frozen XY criteria',
        criteria=criteria,matrices=matrices,cases=cases,total=len(cases),passed=sum(c['passed'] for c in cases),
        limitations=['Does not prove full-room coverage or voice/cleaning acceptance.',
                    'Fixture/world comparison assumes archived map alignment; not independent metrology.'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(status=report['status'],total=report['total'],passed=report['passed'],
        failures=[dict(case=c['case'],repeat=c['repeat'],checks=[k for k,v in c['checks'].items() if not v]) for c in cases if not c['passed']]),ensure_ascii=False))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
