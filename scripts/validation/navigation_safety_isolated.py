#!/usr/bin/env python3
"""Test the real collision filter and final watchdog in an isolated ROS domain.

Run after env.sh with a free ROS_DOMAIN_ID. Only private probe velocity topics
are used. This does not move Gazebo or replace the physical avoidance matrix.
"""
import argparse,hashlib,json,math,os,subprocess,time,signal,sys
import yaml
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,TransformStamped
from sensor_msgs.msg import LaserScan,Image,PointCloud2,PointField
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from tf2_ros import TransformBroadcaster
from semantic_search.sensor_qos import sensor_qos_profile
ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--repeats',type=int,default=5)
args=parser.parse_args()
if not 1<=args.repeats<=20:parser.error('repeats must be 1–20')
if os.environ.get('ROS_DOMAIN_ID','') in ('','66','67'):parser.error('Select a free isolated ROS domain, such as ROS_DOMAIN_ID=71')
args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
config=ROOT/'src/semantic_search/config/collision_monitor.yaml'
params=yaml.safe_load(config.read_text())['collision_monitor']['ros__parameters']
params.update(use_sim_time=False,cmd_vel_in_topic='/probe/raw',cmd_vel_out_topic='/probe/safe')
params['scan']['topic']='/probe/scan';params['depth']['topic']='/probe/cloud'
parameters=args.output/'parameters.yaml'
parameters.write_text(yaml.safe_dump({'collision_monitor_probe':{'ros__parameters':params}}))
rclpy.init();n=Node('isolated_final_velocity_probe');tf=TransformBroadcaster(n);phase='startup';rows=[]
publishers={k:n.create_publisher(t,'/probe/'+k,sensor_qos_profile('reliable') if k=='depth' else qos_profile_sensor_data) for k,t in [('scan',LaserScan),('depth',Image),('cloud',PointCloud2),('odom',Odometry)]};cmd=n.create_publisher(Twist,'/probe/raw',10)
n.create_subscription(Twist,'/probe/guarded',lambda m:rows.append(dict(phase=phase,x=m.linear.x,w=m.angular.z)),10)
base=['/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor','--ros-args','-r','__node:=collision_monitor_probe','--params-file',str(parameters)]
remaps={'/cmd_vel_collision_filtered':'/probe/safe','/cmd_vel_safe':'/probe/guarded','/safety/state':'/probe/guard_state','/scan':'/probe/scan','/rgbd/depth_image':'/probe/depth','/rgbd/obstacles':'/probe/cloud','/odom':'/probe/odom','/collision_monitor/get_state':'/collision_monitor_probe/get_state'}
argv=[sys.executable,'-m','semantic_search.velocity_guard','--ros-args','-p','use_sim_time:=false']
for a,b in remaps.items():argv += ['-r',a+':='+b]
def interrupted(*unused):raise KeyboardInterrupt()
for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,interrupted)
logs=[(args.output/name).open('w') for name in ['monitor_guard_probe.log','guard_probe.log']];children=[];checks=[]
def wait(f,seconds):
 end=time.monotonic()+seconds
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
 assert f.done() and f.result().success,'transition failed'
try:
 end=time.monotonic()+2.
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 foreign=[name for name in n.get_node_names() if name!=n.get_name() and not name.startswith('_ros2cli')]
 if foreign:raise RuntimeError('Isolated domain already contains nodes: '+str(foreign))
 for a,l in zip([base,argv],logs):children.append(subprocess.Popen(a,stdout=l,stderr=subprocess.STDOUT))
 c=n.create_client(ChangeState,'/collision_monitor_probe/change_state');assert c.wait_for_service(timeout_sec=15)
 for value in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
  r=ChangeState.Request();r.transition.id=value;wait(c.call_async(r),10)
 phases=[('fresh',.1),('stale_scan',0.),('resumed',.1),('invalid_depth',0.),('resumed_2',.1),('stale_tf',0.),('reverse',-.1),('stale_command',0.),('obstacle',0.),('recovered',.1)]
 for trial,(phase,expected) in enumerate(phases*args.repeats):
  rows.clear()
  end=time.monotonic()+1.6
  while time.monotonic()<end:
   stamp=n.get_clock().now().to_msg()
   if phase!='stale_tf':
    t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_link';t.header.stamp=stamp;t.transform.rotation.w=1.;tf.sendTransform(t)
   if phase!='stale_scan':
    s=LaserScan();s.header.frame_id='base_link';s.header.stamp=stamp;s.angle_min=-math.pi;s.angle_max=math.pi;s.angle_increment=math.pi/180;s.range_min=.05;s.range_max=20.;s.ranges=[10.]*361
    if phase=='obstacle':s.ranges[180]=.2
    publishers['scan'].publish(s)
   d=Image();d.header.frame_id='base_link';d.header.stamp=stamp;d.width=16;d.height=16;d.encoding='32FC1';d.step=64;d.data=np.full((16,16),np.nan if phase=='invalid_depth' else 2.,np.float32).tobytes();publishers['depth'].publish(d)
   p=PointCloud2();p.header.frame_id='base_link';p.header.stamp=stamp;p.height=1;p.point_step=12;p.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate('xyz')];publishers['cloud'].publish(p)
   o=Odometry();o.header.frame_id='odom';o.child_frame_id='base_link';o.header.stamp=stamp;publishers['odom'].publish(o)
   if phase!='stale_command':
    m=Twist();m.linear.x=-.1 if phase=='reverse' else .1;cmd.publish(m)
   rclpy.spin_once(n,timeout_sec=.03);time.sleep(.02)
  tail=[r for r in rows if r['phase']==phase][-6:]
  passed=bool(tail) and all(abs(r['x']-expected)<.001 and abs(r['w'])<.001 for r in tail)
  checks.append(dict(repeat=trial//len(phases)+1,phase=phase,expected_x=expected,passed=passed,last_outputs=tail))
 report=dict(method='Real Humble CollisionMonitor + project final guard, private topics in isolated domain; no simulated robot velocity topic',status='passed' if all(x['passed'] for x in checks) else 'failed',cases=checks,domain=os.environ['ROS_DOMAIN_ID'],config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),guard_sha256=hashlib.sha256((ROOT/'src/semantic_search/semantic_search/velocity_guard.py').read_bytes()).hexdigest(),source='src/semantic_search/semantic_search/velocity_guard.py',limitation='Does not replace actual simulator navigation and obstacle trials')
 (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(status=report['status'],checks=[dict(phase=x['phase'],passed=x['passed']) for x in checks])),flush=True)
except BaseException as error:
 (args.output/'report.json').write_text(json.dumps(dict(status='interrupted' if isinstance(error,KeyboardInterrupt) else 'failed',error=str(error),cases=checks),indent=2)+'\n')
 raise
finally:
 for p in reversed(children):
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:p.kill();p.wait(timeout=3)
 for log in logs:log.close()
 n.destroy_node();rclpy.try_shutdown()
if report['status']!='passed':raise SystemExit(1)
