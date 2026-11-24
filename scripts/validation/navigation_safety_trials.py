#!/usr/bin/env python3
"""Measure sensor-outage and cancellation stops during managed Gazebo tasks.

Only the verified session cloud adapter can be paused. Movement is requested
through the console, never by publishing velocities. Cached odometry cannot
satisfy the stop check. Run exclusively beside an idle protected session.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import urllib.request
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from robot_runtime import OWNER_VARIABLE,_owned,_signal
from semantic_search.rooms import room_at


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--url',default='http://127.0.0.1:8767')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repeats',type=int,default=5)
    parser.add_argument('--cases',nargs='+',choices=('outage','cancel'),default=['outage','cancel'])
    args = parser.parse_args()
    if not args.url.startswith('http://127.0.0.1:') or not 1<=args.repeats<=20:
        parser.error('Use a local console and 1–20 repetitions')
    session = json.loads((args.session/'session.json').read_text())
    if (str(session['config']['domain'])!=os.environ.get('ROS_DOMAIN_ID')
        or not session['config'].get('navigation_safety')):
        parser.error('ROS domain must match an explicitly protected session')
    marker = (OWNER_VARIABLE+'='+session['config']['ign_partition'].removeprefix('semantic_session_')).encode()
    identities = []
    for row in _owned(marker):
        argv = (Path('/proc')/str(row['pid'])/'cmdline').read_bytes().split(b'\0')
        if any(value.endswith(b'/cloud_adapter') for value in argv):
            identities.append(row)
    if len(identities)!=1:
        raise RuntimeError('Expected one verified owned cloud adapter')
    adapter = identities[0]
    rooms = json.loads((ROOT/session['config']['rooms']).read_text())
    args.output.mkdir(parents=True,exist_ok=False)
    report = dict(status='running',session=str(args.session),planned=args.repeats*len(args.cases),cases=[],
                  method='Console task, verified process pause/cancel, fresh measured odometry and guarded velocity',
                  limitation='Sensor-outage and cancellation subset; not complete obstacle avoidance acceptance')
    def save():
        temporary = args.output/'report.json.tmp'
        temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        temporary.replace(args.output/'report.json')
    def api(path,body=None):
        request = urllib.request.Request(args.url+path,data=json.dumps(body).encode() if body is not None else None,
                                         headers={'Content-Type':'application/json'})
        value = json.load(urllib.request.urlopen(request,timeout=12))
        if 'session_id' in value and value['session_id']!=args.session.name:
            raise RuntimeError('Console session changed')
        return value

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init()
    node = Node('navigation_safety_acceptance')
    latest,rows = {},[]
    phase = 'initializing'
    def odometry(message):
        latest['odom'] = dict(received=time.monotonic(),
            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
            velocity=[message.twist.twist.linear.x,message.twist.twist.angular.z])
    node.create_subscription(Odometry,'/odom',odometry,qos_profile_sensor_data)
    node.create_subscription(Twist,'/cmd_vel_safe',lambda m:latest.update(
        output=dict(received=time.monotonic(),velocity=[m.linear.x,m.angular.z])),10)
    node.create_subscription(String,'/safety/state',lambda m:latest.update(
        safety=json.loads(m.data)),10)
    def sample(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)
            rows.append(dict(wall=time.monotonic(),phase=phase,**latest))
    def moving():
        odom = latest.get('odom',{})
        v = odom.get('velocity',[0.,0.])
        return time.monotonic()-odom.get('received',-math.inf)<.2 and (abs(v[0])>.05 or abs(v[1])>.2)
    def stop_checks(event,delay):
        stopped = [r for r in rows if r['phase']=='interrupted' and r['wall']-event>=delay]
        fresh = bool(stopped) and all(r['wall']-r.get('odom',{}).get('received',-math.inf)<.2 for r in stopped)
        distinct = {r.get('odom',{}).get('stamp') for r in stopped}
        return dict(fresh_odometry=fresh and len(distinct)>=5,
                    measured_stop=fresh and all(abs(r['odom']['velocity'][0])<.01
                        and abs(r['odom']['velocity'][1])<.03 for r in stopped),
                    zero_output=bool(stopped) and all(r['wall']-r.get('output',{}).get('received',-math.inf)<.2
                        and max(abs(x) for x in r['output']['velocity'])<.001 for r in stopped))
    def interrupted(*unused):
        raise KeyboardInterrupt()
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,interrupted)
    active = None
    save()
    try:
        sample(2.)
        for repeat in range(1,args.repeats+1):
            for kind in args.cases:
                waiting = time.monotonic()
                state = api('/api/state')
                while not state['ready'] and time.monotonic()-waiting<20.:
                    if state.get('error'):
                        break
                    sample(.2)
                    state = api('/api/state')
                if not state['ready']:
                    report['not_ready_state'] = state
                    raise RuntimeError('Session is busy or not ready')
                pose = state['telemetry']['pose']
                room = 'living' if room_at(rooms,pose['x'],pose['y'])=='kitchen' else 'kitchen'
                request = dict(request_id=uuid4().hex,mode='find',room=room,category='cup')
                rows.clear()
                record = dict(kind=kind,repeat=repeat,request=request,status='running',
                              readiness_wait_seconds=time.monotonic()-waiting)
                report['cases'].append(record)
                active = request['request_id']
                api('/api/task',request)
                phase = 'waiting_motion'
                deadline = time.monotonic()+65
                while not moving() and time.monotonic()<deadline:
                    sample(.1)
                if not moving():
                    raise RuntimeError('No fresh measured motion before injection')
                phase = 'healthy'
                sample(.25)
                event = time.monotonic()
                phase = 'interrupted'
                if kind=='outage':
                    if not _signal(adapter,marker,signal.SIGSTOP):
                        raise RuntimeError('Cloud adapter ownership changed')
                    sample(1.5)
                    checks = stop_checks(event,.9)
                    checks['stale_cloud_reported'] = any('cloud_stale' in r.get('safety',{}).get('reasons',[])
                                                        for r in rows if r['phase']=='interrupted')
                    if not _signal(adapter,marker,signal.SIGCONT):
                        raise RuntimeError('Cloud adapter could not be resumed')
                else:
                    api('/api/cancel',{})
                    sample(4.)
                    checks = stop_checks(event,2.)
                phase = 'resumed'
                sample(2.)
                if kind=='outage':
                    checks['guard_recovered'] = any(r.get('safety',{}).get('ready') is True
                                                    for r in rows if r['phase']=='resumed')
                deadline = time.monotonic()+480
                while time.monotonic()<deadline:
                    state = api('/api/state')
                    task = state['task']
                    if task['id']!=active:
                        raise RuntimeError('Another task replaced validation task')
                    if task['state'] in ('completed','partial','failed','canceled'):
                        break
                    sample(.5)
                else:
                    raise TimeoutError('Task did not finish after injection')
                checks['task_result'] = (task['state']=='canceled' if kind=='cancel' else
                    task['state']=='completed' and (task.get('result') or {}).get('status')=='found'
                    and task['result'].get('count')==1)
                record.update(status='passed' if all(checks.values()) else 'failed',checks=checks,
                              task=task,injection_wall=event)
                trace = args.output/f'{repeat:02d}_{kind}_trace.json'
                trace.write_text(json.dumps(rows,ensure_ascii=False)+'\n')
                record['trace'] = str(trace)
                active = None
                save()
                print(json.dumps(dict(kind=kind,repeat=repeat,checks=checks),ensure_ascii=False),flush=True)
                if record['status']!='passed':
                    raise RuntimeError('Stop or recovery checks failed')
        report['status'] = 'passed'
    except BaseException as error:
        report.update(status='interrupted' if isinstance(error,KeyboardInterrupt) else 'failed',error=str(error))
        (args.output/'interrupted_trace.json').write_text(json.dumps(rows,ensure_ascii=False)+'\n')
        raise
    finally:
        _signal(adapter,marker,signal.SIGCONT)
        if active:
            try:
                api('/api/cancel',{})
            except Exception as error:
                report['cancel_error'] = str(error)
        save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
