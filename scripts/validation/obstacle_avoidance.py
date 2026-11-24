#!/usr/bin/env python3
"""Unmapped obstacle trials with calibrated Gazebo contact instrumentation.

Fixture geometry is evaluation data only. Nav2 receives ordinary scan/depth,
and every movement is a console search request. A dropped calibration sphere
must trigger the fixture sensor before a no-contact result can be accepted.
"""
import argparse
import hashlib
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/semantic_search'))
from semantic_search.rooms import room_at


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--url',default='http://127.0.0.1:8767')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cases',nargs='+',choices=('static','low','overhang'),default=['static','low','overhang'])
    p.add_argument('--repeats',type=int,default=5)
    p.add_argument('--resume-report',type=Path,action='append',default=[],
                   help='reuse passed cases from restored reports in the same session; repeatable')
    args = p.parse_args()
    meta = json.loads((args.session/'session.json').read_text())
    if (not args.url.startswith('http://127.0.0.1:') or not 1<=args.repeats<=10
        or str(meta['config']['domain'])!=os.environ.get('ROS_DOMAIN_ID')
        or not meta['config'].get('navigation_safety')):
        p.error('Use a matching protected local session and 1–10 repetitions')
    args.output.mkdir(parents=True,exist_ok=False)
    rooms = json.loads((ROOT/meta['config']['rooms']).read_text())
    environment = dict(os.environ,IGN_PARTITION=meta['config']['ign_partition'])
    token = 'validation_'+uuid4().hex
    contact_topic = '/'+token+'/contacts'
    manager = token+'_manager'
    models = []
    bridge = None
    active = None
    report = dict(status='running',session=str(args.session),planned=len(args.cases)*args.repeats,
        cases=[],services=[],restored=False,method='Live unmapped fixtures, calibrated simulator contact sensor, real console navigation',
        driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Contacts cover the test fixture, not every household collision.',
                     'Search tasks exercise the combined NavFn/DWB/safety pipeline; this is not an isolated controller comparison.'])
    skipped = set()
    sources = []
    for previous_path in args.resume_report:
        previous = json.loads(previous_path.read_text())
        if not previous.get('restored') or Path(previous['session']).resolve()!=args.session.resolve():
            raise ValueError('Only a restored report from the same session can be resumed')
        for case in previous['cases']:
            if (case['case'] in args.cases and 1<=case['repeat']<=args.repeats
                    and case['status']=='passed' and all(case['checks'].values())):
                skipped.add((case['case'],case['repeat']))
        sources.append(dict(path=str(previous_path),sha256=hashlib.sha256(previous_path.read_bytes()).hexdigest()))
    if sources:
        report.update(resumed_sources=sources,reused_passed_cases=sorted(skipped),planned=report['planned']-len(skipped))
    def save():
        temporary = args.output/'report.json.tmp'
        temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        temporary.replace(args.output/'report.json')
    def service(action,kind,request,name):
        for attempt in range(1,4):
            done = subprocess.run(['ign','service','-s','/world/aws_house_r0_v1/'+action,
                '--reqtype','ignition.msgs.'+kind,'--reptype','ignition.msgs.Boolean',
                '--timeout','5000','--req',request],env=environment,capture_output=True,text=True,timeout=8)
            row = dict(action=action,model=name,attempt=attempt,returncode=done.returncode,
                       stdout=done.stdout,stderr=done.stderr[-500:])
            report['services'].append(row)
            save()
            if done.returncode==0 and 'data: true' in done.stdout:
                return
            # A missed acknowledgement does not prove create/remove failed.
            # Verify the unique owned model name before any safe repeat.
            scene = subprocess.run(['ign','service','-s','/world/aws_house_r0_v1/scene/info',
                '--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene',
                '--timeout','5000','--req',''],env=environment,capture_output=True,text=True,timeout=8)
            if scene.returncode==0 and re.search(r'(?m)^model \{',scene.stdout):
                present = re.search(r'(?m)^\s*name: '+re.escape(json.dumps(name))+r'\s*$',scene.stdout) is not None
                row['verified_model_present'] = present
                row['scene_sha256'] = hashlib.sha256(scene.stdout.encode()).hexdigest()
                save()
                if present==(action=='create'):
                    return
            time.sleep(.1)
        raise RuntimeError('Fixture service failed: '+action)
    def create(name,sdf):
        file = args.output/(name+'.sdf')
        file.write_text(sdf)
        models.append(name)  # Cleanup also runs after an ambiguous create reply.
        service('create','EntityFactory','sdf_filename: '+json.dumps(str(file.resolve()))+' allow_renaming: false',name)
    def remove(name):
        service('remove','Entity','name: '+json.dumps(name)+' type: MODEL',name)
        models.remove(name)
    def api(path,body=None):
        request = urllib.request.Request(args.url+path,data=json.dumps(body).encode() if body is not None else None,
                                         headers={'Content-Type':'application/json'})
        value = json.load(urllib.request.urlopen(request,timeout=12))
        if value.get('session_id')!=args.session.name:
            raise RuntimeError('Console session changed')
        return value

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from nav2_msgs.srv import ClearEntireCostmap
    from ros_gz_interfaces.msg import Contacts
    rclpy.init()
    node = Node('obstacle_avoidance_acceptance')
    contacts,trajectory = [],[]
    contact_count = 0
    def receive(message):
        nonlocal contact_count
        if message.contacts:
            contact_count += len(message.contacts)
            if len(contacts)<64:
                contacts.append(dict(wall=time.monotonic(),pairs=[
                    [c.collision1.name,c.collision2.name] for c in message.contacts]))
    def odometry(message):
        pose = message.pose.pose.position
        trajectory.append(dict(wall=time.monotonic(),frame=message.header.frame_id,
            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
            xy=[pose.x,pose.y],velocity=[message.twist.twist.linear.x,message.twist.twist.angular.z]))
    node.create_subscription(Contacts,contact_topic,receive,10)
    node.create_subscription(Odometry,'/odom',odometry,qos_profile_sensor_data)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)
    def ready():
        end = time.monotonic()+20
        while True:
            state = api('/api/state')
            if state['ready']:
                return state
            if state.get('error') or time.monotonic()>=end:
                raise RuntimeError('Console did not become ready')
            spin(.2)
    def clear():
        for topic in ('/global_costmap/clear_entirely_global_costmap','/local_costmap/clear_entirely_local_costmap'):
            client = node.create_client(ClearEntireCostmap,topic)
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Costmap reset unavailable')
            spin(.3)  # Discovery can precede the service response-reader match.
            succeeded = False
            for attempt in range(1,4):
                future = client.call_async(ClearEntireCostmap.Request())
                end = time.monotonic()+5
                while not future.done() and time.monotonic()<end:
                    spin(.05)
                succeeded = future.done() and future.result() is not None
                report.setdefault('costmap_resets',[]).append(dict(topic=topic,attempt=attempt,confirmed=succeeded))
                if succeeded:
                    break
                future.cancel()  # Reset is idempotent; retain missed acknowledgements.
            if not succeeded:
                raise RuntimeError('Costmap reset failed')
            node.destroy_client(client)
        spin(.5)
    def interrupted(signum,*unused):
        report.setdefault('received_signals',[]).append(dict(signal=signum,wall=time.monotonic()))
        raise KeyboardInterrupt()
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,interrupted)
    save()
    try:
        ready()
        clear()
        create(manager,f'''<sdf version="1.8"><model name="{manager}"><static>true</static>
          <link name="manager"/><plugin filename="ignition-gazebo-contact-system" name="gz::sim::systems::Contact"/>
          </model></sdf>''')
        bridge_log = (args.output/'contact_bridge.log').open('w')
        bridge = subprocess.Popen(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge',
            contact_topic+'@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '--ros-args','-r','__node:='+token+'_bridge'],env=environment,
            stdout=bridge_log,stderr=subprocess.STDOUT)
        spin(2.)
        for case in args.cases:
            for repeat in range(1,args.repeats+1):
                if (case,repeat) in skipped:
                    continue
                state = ready()
                x,y = [(2.7,-.20),(3.1,-.25),(3.5,-.30)][(repeat-1)%3]
                height,z = {'static':(.60,.30),'low':(.12,.06),'overhang':(.25,.90)}[case]
                name = token+'_'+case+'_'+str(repeat)
                pose = state['telemetry']['pose']
                if math.dist((pose['x'],pose['y']),(x,y))<1.1:
                    raise RuntimeError('Robot too close to create test obstacle')
                record = dict(case=case,repeat=repeat,status='running',fixture=dict(x=x,y=y,z=z,size=[.45,.85,height]))
                report['cases'].append(record)
                save()
                create(name,f'''<sdf version="1.8"><model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>
                  <link name="box"><collision name="body"><geometry><box><size>.45 .85 {height}</size></box></geometry></collision>
                  <visual name="body"><geometry><box><size>.45 .85 {height}</size></box></geometry>
                  <material><diffuse>.8 .2 .15 1</diffuse></material></visual>
                  <sensor name="contact" type="contact"><always_on>true</always_on><contact><collision>body</collision>
                  <topic>{contact_topic}</topic></contact></sensor></link></model></sdf>''')
                spin(.5)
                contacts.clear()
                contact_count = 0
                ball = name+'_calibration'
                create(ball,f'''<sdf version="1.8"><model name="{ball}"><pose>{x} {y} {z+height/2+.3} 0 0 0</pose>
                  <link name="sphere"><inertial><mass>.05</mass><inertia><ixx>.000072</ixx><iyy>.000072</iyy><izz>.000072</izz></inertia></inertial>
                  <collision name="body"><geometry><sphere><radius>.06</radius></sphere></geometry></collision></link></model></sdf>''')
                spin(2.)
                calibration = list(contacts)
                record['calibration_contacts'] = calibration
                record['calibration_contact_count'] = contact_count
                remove(ball)
                spin(1.)
                if not calibration:
                    raise RuntimeError('Contact instrumentation did not detect calibration sphere')
                contacts.clear()
                contact_count = 0
                trajectory.clear()
                state = ready()
                pose = state['telemetry']['pose']
                room = 'living' if room_at(rooms,pose['x'],pose['y'])=='kitchen' else 'kitchen'
                request = dict(request_id=uuid4().hex,mode='find',room=room,category='cup')
                record['request'] = request
                active = request['request_id']
                api('/api/task',request)
                end = time.monotonic()+480
                while time.monotonic()<end:
                    spin(.3)
                    task = api('/api/state')['task']
                    if task['id']!=active:
                        raise RuntimeError('Another task replaced validation task')
                    if contacts:
                        record['first_contact_response'] = 'cancel_task'
                        api('/api/cancel',{})
                        stop_deadline = time.monotonic()+10
                        while time.monotonic()<stop_deadline:
                            spin(.1)
                            latest = api('/api/state')
                            task = latest['task']
                            if not latest['busy']:
                                break
                        break
                    if task['state'] in ('completed','partial','failed','canceled'):
                        break
                else:
                    raise TimeoutError('Obstacle traversal did not finish')
                active = None
                checks = dict(calibrated_sensor=True,no_fixture_contacts=not contacts,
                    contact_bridge_alive=bridge.poll() is None,
                    task_result=task['state']=='completed' and (task.get('result') or {}).get('status')=='found'
                                and task['result'].get('count')==1)
                record.update(checks=checks,status='passed' if all(checks.values()) else 'failed',task=task,
                              contacts=list(contacts),contact_count=contact_count,contact_samples_limit=64)
                file = args.output/(case+'_'+str(repeat)+'_trajectory.json')
                file.write_text(json.dumps(trajectory)+'\n')
                record['trajectory'] = str(file)
                remove(name)
                clear()
                save()
                print(json.dumps(dict(case=case,repeat=repeat,checks=checks)),flush=True)
                if record['status']!='passed':
                    raise RuntimeError('Obstacle trial failed')
        report['status'] = 'passed'
    except BaseException as error:
        report.update(status='interrupted' if isinstance(error,KeyboardInterrupt) else 'failed',error=str(error))
        if report['cases'] and report['cases'][-1]['status']=='running':
            report['cases'][-1].update(status=report['status'],error=type(error).__name__+': '+str(error))
        raise
    finally:
        if active:
            try:
                api('/api/cancel',{})
                end = time.monotonic()+10
                while api('/api/state')['busy'] and time.monotonic()<end:
                    spin(.1)
                report['cleanup_cancel_state'] = api('/api/state')['task']['state']
            except Exception as error:
                report['cancel_error'] = str(error)
        for name in list(reversed(models)):
            try:
                remove(name)
            except Exception as error:
                report.update(status='failed',cleanup_error=str(error))
        if bridge:
            bridge.terminate()
            try:
                bridge.wait(timeout=3)
            except subprocess.TimeoutExpired:
                bridge.kill();bridge.wait(timeout=3)
            bridge_log.close()
        report['restored'] = not models
        save()
        node.destroy_node()
        rclpy.try_shutdown()
    if report['status']!='passed':
        raise RuntimeError('Validation cleanup failed')


if __name__=='__main__':
    main()
