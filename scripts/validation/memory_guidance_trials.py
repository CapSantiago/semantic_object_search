#!/usr/bin/env python3
"""Counterbalanced actual find trials with incremental independent odometry."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from uuid import uuid4
import signal
import statistics
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters,SetParameters
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
import urllib.request

from target_stability import evaluate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--room',required=True)
    parser.add_argument('--category',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--pairs',type=int,default=2)
    parser.add_argument('--port',type=int,default=8767)
    parser.add_argument('--parameter-probe',action='store_true',help='Read/write the unchanged current value, without a robot task')
    parser.add_argument('--guided-only',action='store_true',help='One new guided trial; compare to an independently preserved baseline afterwards')
    args=parser.parse_args()
    if not 1<=args.pairs<=3:parser.error('Use one to three bounded pairs')
    args.output.mkdir(parents=True,exist_ok=False)
    contacts=[];run_token=uuid4().hex[:8]
    tool_source=Path(__file__).read_bytes()
    (args.output/'validation_tool.py').write_bytes(tool_source)
    report=dict(status='starting',pid=os.getpid(),room=args.room,category=args.category,trials=[],pairs=[],contacts=contacts,
                validation_tool_sha256=hashlib.sha256(tool_source).hexdigest(),validation_tool_snapshot='validation_tool.py',run_token=run_token)
    dependency=Path(evaluate.__code__.co_filename)
    (args.output/'target_stability_snapshot.py').write_bytes(dependency.read_bytes())
    report['validation_dependency_sha256']={str(dependency):hashlib.sha256(dependency.read_bytes()).hexdigest()}
    stopped=False;active=None;phase='initializing';last_sample=0.;samples=[];latest={};original=None
    def stop(*unused):
        nonlocal stopped
        stopped=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('memory_guidance_trial_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    raw=(args.output/'odom.jsonl').open('x',buffering=1)
    def odom(message):
        nonlocal last_sample
        now=time.monotonic();p=message.pose.pose.position;v=message.twist.twist
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        latest.update(wall=now,velocity=[v.linear.x,v.angular.z],stamp=stamp)
        if now-last_sample<.09:return
        last_sample=now
        row=dict(wall=now,stamp=stamp,xy=[p.x,p.y],velocity=[v.linear.x,v.angular.z],
                 source_age_seconds=node.get_clock().now().nanoseconds/1e9-stamp,task=active,phase=phase)
        raw.write(json.dumps(row)+'\n');samples.append(row)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    def contact(message):
        for item in message.contacts:
            names=[item.collision1.name,item.collision2.name]
            if any('ground_plane'in n for n in names)or all('waffle'in n for n in names):continue
            contacts.append(dict(task=active,stamp=node.get_clock().now().nanoseconds/1e9,names=names))
    node.create_subscription(Contacts,'/robot/contacts',contact,10)
    # Humble's executor runs on another thread. Keep parameter clients alive
    # and in its wait set before spinning, rather than creating/destroying them
    # concurrently with an active executor.
    clients={GetParameters:node.create_client(GetParameters,'/semantic_search/get_parameters'),
             SetParameters:node.create_client(SetParameters,'/semantic_search/set_parameters')}
    executor=SingleThreadedExecutor();executor.add_node(node)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if stopped:raise InterruptedError('Memory trials interrupted')
            time.sleep(.025)
    def save():
        path=args.output/'report.tmp';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');path.replace(args.output/'report.json')
    def api(path,body=None):
        request=urllib.request.Request(f'http://127.0.0.1:{args.port}'+path,data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json'})
        return json.load(urllib.request.urlopen(request,timeout=6))
    def parameter(value=None,allow_interrupt=False):
        service=GetParameters if value is None else SetParameters
        client=clients[service]
        assert client.wait_for_service(timeout_sec=15),'Missing parameter service; use matching project env.sh'
        request=service.Request()
        if value is None:request.names=['memory_guidance']
        else:request.parameters=[Parameter('memory_guidance',value=value).to_parameter_msg()]
        future=client.call_async(request);end=time.monotonic()+10
        while not future.done() and time.monotonic()<end:
            if allow_interrupt:time.sleep(.025)
            else:spin(.025)
        assert future.done(),'Parameter timeout'
        result=future.result()
        if value is None:return result.values[0].bool_value
        assert result.results[0].successful
    try:
        state=api('/api/state');assert state['ready'] and not state['busy']
        session=Path(state['output_directory']);meta=json.loads((session/'session.json').read_text())
        assert str(meta['config']['domain'])==os.environ.get('ROS_DOMAIN_ID'),'Use the matching ROS domain'
        report.update(session=str(session),configuration=meta['config'],source_sha256=meta['source_sha256'])
        original=parameter();report['original_memory_guidance']=original;spin(1.)
        assert node.count_publishers('/robot/contacts')>0,'Missing native body contact source'
        assert latest and time.monotonic()-latest['wall']<.4 and abs(latest['velocity'][0])<.01
        if args.parameter_probe:
            parameter(original)
            assert parameter()==original
            report.update(status='parameter_probe_passed',no_task_sent=True);save();return
        report['status']='running';save()
        for pair in range(args.pairs):
            for enabled in ((True,)if args.guided_only else ((False,True) if pair%2==0 else (True,False))):
                state=api('/api/state');assert state['ready'] and not state['busy'] and state['output_directory']==str(session)
                history=api('/api/memory');parameter(enabled)
                active=f'memory_{run_token}_{pair}_{str(enabled).lower()}';phase='search';started=time.monotonic();result_seen=None
                contact_start=len(contacts)
                api('/api/task',dict(request_id=active,mode='find',room=args.room,category=args.category))
                end=time.monotonic()+550;last_save=0;contact_cancel=False
                while time.monotonic()<end:
                    state=api('/api/state');task=state.get('task')or{}
                    if state['busy']and len(contacts)>contact_start and not contact_cancel:
                        api('/api/cancel',{});contact_cancel=True
                    if task.get('id')==active:
                        if task.get('result') and result_seen is None:result_seen=time.monotonic();phase='returning'
                        if task['state'] in ('completed','partial','failed','canceled'):break
                    if state['phase']=='failed':raise RuntimeError(state.get('error'))
                    if time.monotonic()-last_save>4:report['last_task']=task;save();last_save=time.monotonic()
                    spin(.2)
                else:raise TimeoutError('Find trial watchdog')
                task_id=active;active=None
                evaluation=evaluate(task,dict(room=args.room,category=args.category,position_limit_m=.20),session,meta)
                points=[s for s in samples if s['task']==task_id and s['phase']=='search']
                distance=0.;jumps=0;max_gap=0.
                for a,b in zip(points,points[1:]):
                    gap=b['stamp']-a['stamp'];d=math.dist(a['xy'],b['xy']);max_gap=max(max_gap,gap)
                    if 0<gap<=.5 and d<=.3:distance+=d
                    else:jumps+=1
                quality=(len(points)>=10 and jumps==0 and max_gap<=.5 and
                         all(-.1<=s['source_age_seconds']<=.4 for s in points))
                result=task.get('result')or{};guidance=result.get('memory_guidance')or{}
                row=dict(pair=pair,enabled=enabled,history_before=history,task=task,evaluation=evaluation,
                    search_started_wall=started,result_seen_wall=result_seen,search_odom_distance_m=distance,
                    odometry_quality=dict(passed=quality,samples=len(points),jumps=jumps,max_gap_seconds=max_gap),
                    no_recorded_body_contact=len(contacts)==contact_start,contact_cancel=contact_cancel)
                report['trials'].append(row);save()
                assert evaluation['passed'] and quality and len(contacts)==contact_start,'Correctness/return/measurement/contact check failed'
                assert guidance.get('enabled')==enabled and guidance.get('counts_from_history') is False
                if enabled:assert guidance.get('used') and guidance.get('order_changed'),'No actual history reordering'
                print(pair,enabled,result['elapsed_wall_seconds'],result['viewpoints_visited'],distance,flush=True)
            trials=[t for t in report['trials'] if t['pair']==pair]
            if args.guided_only:continue
            baseline=next(t for t in trials if not t['enabled']);guided=next(t for t in trials if t['enabled'])
            a=baseline['task']['result'];b=guided['task']['result']
            report['pairs'].append(dict(pair=pair,seconds_saved=a['elapsed_wall_seconds']-b['elapsed_wall_seconds'],
                measured_search_distance_saved_m=baseline['search_odom_distance_m']-guided['search_odom_distance_m'],
                baseline_views=a['viewpoints_visited'],guided_views=b['viewpoints_visited']));save()
        if args.guided_only:
            report.update(status='guided_trial_finished',benefit_not_computed=True);save();return
        report.update(status='finished',time_benefit_all_pairs=all(p['seconds_saved']>0 for p in report['pairs']),
            distance_benefit_all_pairs=all(p['measured_search_distance_saved_m']>0 for p in report['pairs']),
            mean_seconds_saved=statistics.mean(p['seconds_saved']for p in report['pairs']),
            limitations=['Same unchanged layout/dock/profile; natural live costmaps/arrivals vary.',
                         'Both modes save real observations; history before each trial retained, not an identical frozen-memory causal experiment.',
                         'Distance is an independently sampled odometry polyline for the search stage only; return excluded.',
                         'Counterbalanced limited pairs do not establish gains for all categories/rooms.']);save()
    except BaseException as error:
        report.update(status='interrupted' if stopped else 'failed',error=f'{type(error).__name__}: {error}');save()
        if active:
            try:
                state=api('/api/state')
                if state['busy'] and state.get('task',{}).get('id')==active:api('/api/cancel',{})
            except Exception:pass
        raise
    finally:
        if original is not None:
            try:
                parameter(original,allow_interrupt=True);report['parameter_restored']=True
            except Exception as error:report['parameter_restore_error']=str(error)
        save();executor.shutdown(timeout_sec=5);worker.join(timeout=5)
        raw.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
