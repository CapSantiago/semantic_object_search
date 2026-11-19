#!/usr/bin/env python3
"""Serial, reversible fault tests on an idle managed robot session.

Run with its ROS domain after env.sh. No UI or other client may submit tasks
until this driver exits. Only the verified session detector may be paused.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from robot_runtime import _owned, _signal, OWNER_VARIABLE
from run_search_matrix import run_client, save


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--repeats',type=int,default=5)
    args=p.parse_args()
    session=json.loads((args.session/'session.json').read_text())
    if str(session['config']['domain'])!=os.environ.get('ROS_DOMAIN_ID'):
        p.error('ROS domain does not match session')
    if args.repeats<1:p.error('repeats must be positive')
    args.output.mkdir(parents=True,exist_ok=False)
    marker=(OWNER_VARIABLE+'='+session['config']['ign_partition'].removeprefix('semantic_session_')).encode()
    detectors=[r for r in _owned(marker) if b'semantic_search.detector' in
               (Path('/proc')/str(r['pid'])/'cmdline').read_bytes().split(b'\0')]
    if len(detectors)!=1:raise RuntimeError('Expected exactly one verified owned detector')
    detector=detectors[0];stopped=[]
    handlers={sig:signal.signal(sig,lambda s,f:stopped.append(s)) for sig in (signal.SIGINT,signal.SIGTERM)}
    def parameter(name,value):
        result=subprocess.run(['ros2','param','set','--no-daemon','--spin-time','3','/semantic_search',name,str(value)],
            capture_output=True,text=True,timeout=30)
        if result.returncode or 'successful' not in result.stdout:
            raise RuntimeError('Parameter change rejected: '+name+' '+result.stdout+result.stderr)
    def get_parameter(name):
        result=subprocess.run(['ros2','param','get','--no-daemon','--spin-time','3','/semantic_search',name],
            capture_output=True,text=True,check=True,timeout=30)
        if not result.stdout.startswith('String value is: '):raise RuntimeError('Unexpected parameter type')
        return result.stdout.removeprefix('String value is: ').strip()
    original_rooms=get_parameter('rooms_file');original_frame=get_parameter('optical_frame')
    rooms=json.loads(Path(original_rooms).read_text())
    front=[6.6,-.4,1.57079633]
    fixtures={}
    for kind in ('front','neighbor','unreachable'):
        r=json.loads(json.dumps(rooms))
        r['kitchen'].setdefault('category_views',{})['cup']=[front]
        if kind=='neighbor':r['gym'].setdefault('category_views',{})['cup']=[front]
        if kind=='unreachable':r['kitchen']['category_views']['cup']=[[50.,50.,0.],front]
        path=args.output/(kind+'_rooms.json');save(path,r);fixtures[kind]=path.resolve()
    report=dict(status='running',session=str(args.session),repeats=args.repeats,
        restored=False,cases=[],scope='Live neighbor filtering, unreachable view, TF failure, detector outage, cancellation, isolated invalid-depth input and recovery')
    paused=False
    test_topics=None
    def trial(kind,index,command,status,count,timeout=180):
        if stopped:raise InterruptedError('Stop requested')
        folder=args.output/f'{len(report["cases"])+1:03d}_{kind}_{index}';folder.mkdir()
        argv=[sys.executable,str(ROOT/'scripts/run_search.py'),command,'--timeout',str(timeout),
              '--output',str(folder/'result.json'),'--diagnostics-output',str(folder/'diagnostics.json')]
        if test_topics:
            argv.extend(['--command-topic',test_topics+'/command','--result-topic',test_topics+'/result'])
        client=run_client(argv,folder/'client.log',timeout+40,5,stopped)
        result=json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {}
        checks=dict(status=result.get('status')==status,count=result.get('count')==count)
        if kind=='neighbor':checks['outside_target_seen']=result.get('valid_projections',0)>0 and result.get('tf_failures')==0
        if kind=='unreachable':checks['failure_recorded']=bool(result.get('viewpoint_failures'))
        if kind=='tf_failure':checks['tf_failure_recorded']=result.get('tf_failures',0)>0
        if kind=='depth_failure':
            checks.update(invalid_depth_recorded=result.get('invalid_target_depth',0)>0,
                          tf_intact=result.get('tf_failures')==0,
                          no_valid_projection=result.get('valid_projections')==0)
        if kind=='cancel':
            diagnostic=json.loads((folder/'diagnostics.json').read_text())
            checks['cancel_acknowledged']=diagnostic.get('cancellation_observed') is True
            subprocess.run([sys.executable,str(ROOT/'scripts/record_navigation_state.py'),
                '--seconds','4','--output',str(folder/'stop_trace.json')],check=True,timeout=20)
            trace=json.loads((folder/'stop_trace.json').read_text())
            samples=[s['odom']['velocity'] for s in trace['samples'] if s['wall_elapsed']>=2 and 'odom' in s]
            checks['stopped']=bool(samples) and all(abs(v[0])<.01 and abs(v[1])<.01 for v in samples)
        row=dict(kind=kind,repeat=index,checks=checks,passed=all(checks.values()),client=client,
                 result=result,directory=str(folder))
        report['cases'].append(row);save(args.output/'report.json',report)
        print(json.dumps(dict(kind=kind,repeat=index,status=result.get('status'),passed=row['passed'])),flush=True)
        if stopped:raise InterruptedError('Stop requested')
        if not row['passed']:raise RuntimeError('Fault test failed: '+kind)
    try:
        for i in range(1,args.repeats+1):
            parameter('rooms_file',fixtures['neighbor'])
            trial('neighbor',i,'去健身区统计杯子','not_found',0)
            parameter('rooms_file',fixtures['unreachable'])
            trial('unreachable',i,'去厨房统计杯子','partial',1)
            parameter('rooms_file',fixtures['front'])
            parameter('optical_frame','acceptance_missing_camera_frame')
            trial('tf_failure',i,'去厨房找杯子','localization_failed',0)
            parameter('optical_frame',original_frame)
            trial('tf_recovery',i,'去厨房找杯子','found',1)
            if not _signal(detector,marker,signal.SIGSTOP):raise RuntimeError('Detector identity changed')
            paused=True;time.sleep(2)
            try:trial('sensor_outage',i,'去厨房找杯子','no_sensor_data',0)
            finally:
                if not _signal(detector,marker,signal.SIGCONT):raise RuntimeError('Cannot resume detector')
                paused=False
            time.sleep(2)
            trial('sensor_recovery',i,'去厨房找杯子','found',1)
            trial('cancel',i,'去杂物间找椅子','canceled',0,timeout=2)
        # Feed the unchanged Search implementation a separate depth stream.
        # Live RGB detections, intrinsics, TF, Nav2 and Gazebo remain real;
        # only this test node's depth bytes are replaced by invalid zeros.
        # The normal search command/result topics and sensor stream stay idle.
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image
        from semantic_search.sensor_qos import sensor_qos_profile
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        relay=Node('acceptance_depth_relay')
        test_topics='/acceptance_depth_search'
        publisher=relay.create_publisher(Image,test_topics+'/depth',sensor_qos_profile('reliable'))
        state=dict(corrupt=True,invalid_frames=0,valid_frames=0)
        def depth_frame(message):
            if state['corrupt']:
                message=copy.copy(message)
                message.data=bytes(len(message.data))
                state['invalid_frames']+=1
            else:
                state['valid_frames']+=1
            publisher.publish(message)
        relay.create_subscription(Image,'/rgbd/depth_image',depth_frame,sensor_qos_profile('reliable'))
        relay_stop=threading.Event();relay_errors=[]
        def spin_relay():
            try:
                while not relay_stop.is_set():rclpy.spin_once(relay,timeout_sec=.1)
            except Exception as error:relay_errors.append(str(error))
        thread=threading.Thread(target=spin_relay,daemon=True);thread.start()
        child=None
        with (args.output/'depth_search.log').open('w') as log:
            try:
                child=subprocess.Popen([sys.executable,'-c',
                    'from semantic_search.search_node import main; main()',
                    '--ros-args','-r','__node:=acceptance_depth_search',
                    '-r','/search/command:='+test_topics+'/command',
                    '-r','/search/result:='+test_topics+'/result',
                    '-r','/search/markers:='+test_topics+'/markers',
                    '-p','use_sim_time:=true','-p','sensor_reliability:=reliable',
                    '-p','scene_id:='+session['config']['scene_id']+'_depth_test',
                    '-p','rooms_file:='+str(fixtures['front']),
                    '-p','output_dir:='+str((args.output/'depth_tasks').resolve()),
                    '-p','depth_topic:='+test_topics+'/depth',
                    '-p','reject_clipped_detections:=true','-p','validate_object_size:=true'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                for i in range(1,args.repeats+1):
                    state['corrupt']=True;time.sleep(1)
                    trial('depth_failure',i,'去厨房找杯子','localization_failed',0)
                    state['corrupt']=False;time.sleep(1)
                    trial('depth_recovery',i,'去厨房找杯子','found',1)
                    if relay_errors:raise RuntimeError('Depth relay failed: '+str(relay_errors))
            finally:
                if child is not None:
                    for sig,timeout in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,3)):
                        if child.poll() is not None:break
                        child.send_signal(sig)
                        try:child.wait(timeout=timeout)
                        except subprocess.TimeoutExpired:pass
                relay_stop.set();thread.join(timeout=5)
                relay.destroy_node();rclpy.try_shutdown()
                report['depth_fixture']=dict(**state,errors=relay_errors,
                    child_stopped=child is None or child.poll() is not None,relay_stopped=not thread.is_alive())
                test_topics=None
                if thread.is_alive() or child is not None and child.poll() is None:
                    raise RuntimeError('Depth test fixture did not stop')
        report['status']='passed'
    except BaseException as error:
        report.update(status='interrupted' if stopped else 'failed',error=f'{type(error).__name__}: {error}')
        raise
    finally:
        try:
            if paused and not _signal(detector,marker,signal.SIGCONT):raise RuntimeError('Detector resume failed')
            parameter('optical_frame',original_frame);parameter('rooms_file',original_rooms)
            report['restored']=True
        finally:
            save(args.output/'report.json',report)
            for sig,handler in handlers.items():signal.signal(sig,handler)


if __name__=='__main__':main()
