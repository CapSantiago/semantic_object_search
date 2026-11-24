#!/usr/bin/env python3
"""Bounded local Nav2/cleaning-client diagnostic, never a room acceptance run.

Uses the same action/stop/docking backend. One map-selected segment is checked
against both margins and revalidated on current sensor costmaps. No scene truth.
"""
import argparse,hashlib,json,math,os,signal,threading,time,urllib.request
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters,SetParameters
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import IsPathValid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan,PointCloud2
from ros_gz_interfaces.msg import Contacts
from navigation_client import NavigationRun,StopRequested,yaw
from semantic_search.cleaning import RoomSweep
from semantic_search.docking import dock_at


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',type=Path,required=True);p.add_argument('--route',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--cloud-min-period',type=float,choices=(.1,.25));p.add_argument('--preflight-only',action='store_true');p.add_argument('--local-margin-zero',action='store_true');p.add_argument('--approach-speed',type=float,choices=(.12,.35),default=.12);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 (a.output/'probe_source.py').write_bytes(Path(__file__).read_bytes())
 def api():return json.load(urllib.request.urlopen('http://127.0.0.1:8767/api/state',timeout=2))
 state=api();assert state['ready']and not state['busy']and Path(state['output_directory'])==a.session.resolve()
 meta=json.loads((a.session/'session.json').read_text());assert str(meta['config']['domain'])==os.environ.get('ROS_DOMAIN_ID');route=json.loads(a.route.read_text());first,last=route['candidate']['first'],route['candidate']['last'];native_regression=route.get('mode')in ('native_waypoint_regression','native_waypoints_only');stopped=[];finished=threading.Event()
 regression_speed=route.get('regression_speed_mps',.12)
 if regression_speed not in (.12,.35):raise ValueError('Unsupported regression speed')
 for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *unused:stopped.append('signal'))
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
 class Probe(NavigationRun):
  def path_for_lane(self,target):
   live=available();reference=self.sweep.lane_reference(self.pose(),first,last,live)
   if reference is None or math.dist(reference['end'][:2],last[:2])>.01:return None
   return reference['path']
  def on_pose_sample(self,current,stamp):
   if self.stage=='sweeping':self.sweep.touch(*current[:2],stamp)
  def tick(self):
   super().tick()
   if self.stage in ('approaching','sweeping'):
    now=time.monotonic();sim=observer.get_clock().now().nanoseconds/1e9
    for key in ('scan','depth'):
     row=fresh.get(key)
     if row is None or now-row['wall']>.4 or not -.1<=sim-row['stamp']<=.4:
      if not stopped:
       report.setdefault('stale_events',[]).append(dict(sensor=key,wall=now,sim=sim,last=row))
       stopped.append('stale_'+key)
 original_period=None
 worker=Probe(SimpleNamespace(output=a.output,timeout=360.),stopped,node_name='local_cleaning_clearance_probe');worker.sweep=None;fresh={};maps={};contacts=[];message_count=0;limits=[];steps=[]
 report=dict(status='initializing',pid=os.getpid(),session=str(a.session.resolve()),source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),scope='One explicit local segment; not full-room coverage acceptance',startup_freshness_window_seconds=2.,contact_monitor='Event-only Fortress contact sensor, endpoint and server log checked; no heartbeat expected; excludes wheels/casters',route=route,steps=steps,contacts=contacts,stop_reasons=stopped,speed_limits=limits)
 def save():
  report.update(goals=worker.goals,trace=worker.trace,odom=worker.odom_trace,contact_messages=message_count,latest_sensors=fresh)
  temp=a.output/'report.tmp';temp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');temp.replace(a.output/'report.json')
 def contact(m):
  nonlocal message_count
  message_count+=1;fresh['contacts']=dict(wall=time.monotonic(),stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)
  for c in m.contacts:
   names=[c.collision1.name,c.collision2.name]
   if any('ground_plane'in n for n in names)or all('waffle'in n for n in names):continue
   contacts.append(dict(sim_seconds=worker.node.get_clock().now().nanoseconds/1e9,names=names,depths=list(c.depths),map_pose=worker.pose()))
   if 'nonfloor_contact'not in stopped:stopped.append('nonfloor_contact')
 observer=Node('clearance_sensor_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
 observer.create_subscription(Contacts,'/robot/contacts',contact,10)
 for key,topic,kind in [('scan','/scan',LaserScan),('depth','/rgbd/obstacles',PointCloud2)]:
  def receive(m,key=key):
   fresh[key]=dict(wall=time.monotonic(),stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)
   if key=='depth':maps['cloud']=m
  observer.create_subscription(kind,topic,receive,QoSProfile(depth=1,reliability=qos_profile_sensor_data.reliability))
 qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 for key,topic in [('map','/map'),('cost','/global_costmap/costmap')]:
  worker.node.create_subscription(OccupancyGrid,topic,lambda m,k=key:maps.update({k:m,'wall_'+k:time.monotonic()}),qos)
 worker.node.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m:maps.update(local_cost=m,wall_local_cost=time.monotonic()),qos_profile_sensor_data)
 publisher=worker.node.create_publisher(SpeedLimit,'/speed_limit',10)
 def limit(value):
  m=SpeedLimit();m.header.stamp=worker.node.get_clock().now().to_msg();m.percentage=False;m.speed_limit=value
  for _ in range(3):publisher.publish(m);rclpy.spin_once(worker.node,timeout_sec=.04)
  limits.append(dict(value_mps=value,sim_seconds=worker.node.get_clock().now().nanoseconds/1e9))
 def cloud_period(value=None, *, cleanup=False):
  service=GetParameters if value is None else SetParameters
  client=worker.node.create_client(service,'/fortress_depth_cloud_adapter/'+('get_parameters'if value is None else'set_parameters'))
  assert client.wait_for_service(timeout_sec=8)
  req=service.Request()
  if value is None:req.names=['min_period']
  else:req.parameters=[Parameter('min_period',value=value).to_parameter_msg()]
  future=client.call_async(req);worker.wait_for(future.done,8,check_stop=not cleanup)
  response=future.result();worker.node.destroy_client(client)
  if value is None:return response.values[0].double_value
  assert response.results[0].successful,response.results[0].reason
 def available():
  worker.wait_for(lambda:'cost'in maps and time.monotonic()-maps['wall_cost']<3.,3.)
  grid,cost=maps['map'],maps['cost'];assert grid.header.frame_id==cost.header.frame_id=='map'
  assert (grid.info.width,grid.info.height)==(cost.info.width,cost.info.height)
  assert abs(grid.info.resolution-cost.info.resolution)<1e-5 and math.dist([grid.info.origin.position.x,grid.info.origin.position.y],[cost.info.origin.position.x,cost.info.origin.position.y])<1e-4
  data=np.asarray(cost.data).reshape(cost.info.height,cost.info.width);return(data>=0)&(data<99)
 def observed_cloud(name):
  msg=maps['cloud'];assert msg.point_step==12 and not msg.is_bigendian
  tr=worker.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
  q=tr.rotation;v=tr.translation;points=np.frombuffer(msg.data,dtype='<f4').reshape(-1,3)
  xyz=points@Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix().T+np.array([v.x,v.y,v.z])
  bounds=route.get('observed_roi',[-.1,3.65,.9,4.6]);roi=(xyz[:,0]>=bounds[0])&(xyz[:,0]<=bounds[2])&(xyz[:,1]>=bounds[1])&(xyz[:,1]<=bounds[3])
  np.savez_compressed(a.output/name,map_xyz=xyz[roi])
  band=route.get('height_band',[.02,.08])
  return dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,roi_points=int(roi.sum()),
              low_points=int((roi&(xyz[:,2]>=.02)&(xyz[:,2]<.08)).sum()),
              band=band,band_points=int((roi&(xyz[:,2]>=band[0])&(xyz[:,2]<band[1])).sum()))
 def owner_watch():
  while not finished.wait(.4):
   try:
    s=api()
    if s['busy']or Path(s['output_directory'])!=a.session.resolve():stopped.append('external_task_or_owner_change');return
   except OSError:stopped.append('console_unavailable');return
 watch=threading.Thread(target=owner_watch,daemon=True)
 sensor_executor=SingleThreadedExecutor();sensor_executor.add_node(observer);sensor_thread=threading.Thread(target=sensor_executor.spin,daemon=True);sensor_thread.start()
 fresh_since=None
 fresh_longest=0.
 fresh_resets=0
 def sensors_ready():
  nonlocal fresh_since,fresh_longest,fresh_resets
  now=time.monotonic();sim=observer.get_clock().now().nanoseconds/1e9
  valid=all(key in fresh and now-fresh[key]['wall']<.3 and -.1<=sim-fresh[key]['stamp']<.3 for key in ('scan','depth'))
  if not valid:
   if fresh_since is not None:fresh_resets+=1
   fresh_since=None
  elif fresh_since is None:fresh_since=now
  duration=now-fresh_since if fresh_since is not None else 0.
  fresh_longest=max(fresh_longest,duration)
  report['startup_freshness']=dict(current_window_seconds=duration,longest_window_seconds=fresh_longest,resets=fresh_resets,required_seconds=2.)
  return duration>=2.
 def startup_ready():
  checks=dict(pose=worker.pose()is not None,parked=worker.parked(),navigate_server=worker.client.server_is_ready(),follow_server=worker.path_client.server_is_ready(),map='map'in maps,costmap='cost'in maps,contact_publisher=worker.node.count_publishers('/robot/contacts')>0,speed_limit_subscriber=publisher.get_subscription_count()>0,continuous_fresh_sensors=sensors_ready())
  report['startup_checks']=checks
  return all(checks.values())
 try:
  if a.cloud_min_period is not None:
   original_period=cloud_period();cloud_period(a.cloud_min_period);report['cloud_period_change']=dict(before=original_period,candidate=a.cloud_min_period,restored=False);save()
  worker.wait_for(startup_ready,30.)
  if a.preflight_only:
   report.update(status='preflight_passed',scope='No movement: startup checks only, not local passage or room acceptance');return
  worker.start_pose=worker.pose();grid=maps['map'];g=grid.info;p0=g.origin.position;rooms=json.loads((a.session/meta['configuration_snapshots'][meta['config']['rooms']]).read_text())
  if a.local_margin_zero:rooms['gym']['cleaning_tracking_margin_cells']=0
  report['local_margin_override']=a.local_margin_zero
  worker.sweep=RoomSweep(grid.data,g.width,g.height,g.resolution,(p0.x,p0.y,yaw(g.origin.orientation)),rooms,'gym',worker.start_pose)
  baseline_rooms=json.loads(json.dumps(rooms));baseline_rooms['gym']['cleaning_tracking_margin_cells']=1;baseline=RoomSweep(grid.data,g.width,g.height,g.resolution,(p0.x,p0.y,yaw(g.origin.orientation)),baseline_rooms,'gym',worker.start_pose)
  assert worker.sweep.tracking_margin_cells==0 and np.array_equal(worker.sweep.eligible,baseline.eligible)and np.array_equal(worker.sweep.centers,baseline.centers)
  report.update(eligible_cells=worker.sweep.eligible_cells,reference_centers_unchanged=True,transit_clearance_m=worker.sweep.transit_clearance_m,tracking_margin_cells=0,contact_publishers=worker.node.count_publishers('/robot/contacts'),status='running')
  live=available()
  def masks(name):
   arrays=dict(static_map=np.asarray(grid.data).reshape(g.height,g.width),
       cost=np.asarray(maps['cost'].data).reshape(g.height,g.width),centers=worker.sweep.centers,
       candidate_planning=worker.sweep.planning_centers,baseline_planning=baseline.planning_centers,
       eligible=worker.sweep.eligible)
   item=dict(file=name,pose=worker.pose(),sim_seconds=worker.node.get_clock().now().nanoseconds/1e9,
       global_stamp=maps['cost'].header.stamp.sec+maps['cost'].header.stamp.nanosec/1e9,
       global_age_wall=time.monotonic()-maps['wall_cost'])
   local=maps.get('local_cost')
   if local is not None:
    arrays.update(local_cost=np.asarray(local.data).reshape(local.info.height,local.info.width),
        local_origin=np.array([local.info.origin.position.x,local.info.origin.position.y,yaw(local.info.origin.orientation)]),
        local_resolution=np.array(local.info.resolution))
    item.update(local_stamp=local.header.stamp.sec+local.header.stamp.nanosec/1e9,
                local_age_wall=time.monotonic()-maps['wall_local_cost'],local_frame=local.header.frame_id)
   for key in ('cost','local_cost'):
    message=maps.get(key)
    if message is None:continue
    try:
     transform=worker.tf.lookup_transform(message.header.frame_id,'base_link',Time.from_msg(message.header.stamp)).transform
     item[key+'_pose_at_stamp']=[transform.translation.x,transform.translation.y,yaw(transform.rotation)]
    except Exception as error:item[key+'_pose_error']=str(error)
   np.savez_compressed(a.output/name,**arrays);report.setdefault('grid_snapshots',[]).append(item)
   report['grid']=dict(resolution=g.resolution,origin=[p0.x,p0.y],width=g.width,height=g.height)
  masks('initial_masks.npz')
  points=worker.sweep.lane_path(first,last,live)
  report['initial_live_segment_accepted']=points is not None
  if route.get('approach_waypoints'):
   points=worker.sweep.lane_path(first,last,np.ones_like(live))
  assert points,'Current safety map rejects fixed local segment; no motion sent'
  both={label:all(sweep.planning_centers[sweep.index(x,y)[1],sweep.index(x,y)[0]]for x,y,_ in points)for label,sweep in [('baseline',baseline),('candidate',worker.sweep)]};report['static_planning_acceptance']=both
  if not native_regression:assert not both['baseline']and both['candidate']
  save()
  watch.start();limit(a.approach_speed);worker.stage='approaching';ok=True
  for index,target in enumerate(route.get('approach_waypoints',[])):
   live=available();c,r=worker.sweep.index(*target[:2])
   if not (0<=r<live.shape[0]and 0<=c<live.shape[1]and worker.sweep.transit_centers[r,c]and live[r,c]):
    report['staging_rejection']=dict(index=index,target=target);ok=False;break
   precise=route.get('precision_cleaning_connectors',False) and index>0
   if precise:worker.stage='sweeping'
   ok=worker.go(target,worker.deadline,timeout=65.,purpose='lane_start' if precise else 'local_staging',recovery=True)
   steps.append(dict(kind='staging',index=index,passed=ok));masks('staging_arrival_%02d.npz'%index);save()
   if not ok:break
   seconds=route.get('observe_after_staging',{}).get(str(index),0.)
   if seconds:
    end=time.monotonic()+seconds;next_sample=0.;rows=[]
    while time.monotonic()<end:
     rclpy.spin_once(worker.node,timeout_sec=.02);worker.tick()
     if time.monotonic()>=next_sample:
      live=available();cost=np.asarray(maps['cost'].data).reshape(live.shape)
      rows.append(dict(sim_seconds=worker.node.get_clock().now().nanoseconds/1e9,
        costs=[int(cost[worker.sweep.index(*v[:2])[1],worker.sweep.index(*v[:2])[0]])for v in route.get('known_unsafe_goals',[])]));next_sample=time.monotonic()+.5
    item=dict(index=index,seconds=seconds,rows=rows)
    try:item['cloud']=observed_cloud('staging_%02d_points.npz'%index)
    except Exception as e:item['cloud_error']=str(e)
    report.setdefault('staging_observations',[]).append(item);masks('staging_%02d_masks.npz'%index);save()
  if ok and native_regression:
   end=time.monotonic()+2.
   while time.monotonic()<end:rclpy.spin_once(worker.node,timeout_sec=.02);worker.tick()
   live=available();masks('observed_regression_masks.npz');checks=[]
   for target in route.get('known_unsafe_goals',[]):
    c,r=worker.sweep.index(*target[:2]);check=dict(target=target,center_rejected=not bool(live[r,c]),cost=int(np.asarray(maps['cost'].data).reshape(live.shape)[r,c]))
    client=worker.node.create_client(IsPathValid,'/is_path_valid');assert client.wait_for_service(timeout_sec=3.)
    request=IsPathValid.Request();request.path.header.frame_id='map';request.path.header.stamp=worker.node.get_clock().now().to_msg()
    pose=PoseStamped();pose.header=request.path.header;pose.pose.position.x,pose.pose.position.y=target[:2];pose.pose.orientation.z=math.sin(target[2]/2);pose.pose.orientation.w=math.cos(target[2]/2);request.path.poses=[pose]
    future=client.call_async(request);worker.wait_for(future.done,3.);check['native_footprint_valid']=future.result().is_valid;worker.node.destroy_client(client)
    check['rejected']=check['center_rejected']or not check['native_footprint_valid'];checks.append(check)
   report['known_unsafe_goal_checks']=checks;ok=(bool(checks)or route.get('mode')=='native_waypoints_only')and all(c['center_rejected']if route.get('require_center_rejection')else c['rejected']for c in checks)
   try:
    observation=observed_cloud('observed_obstacle_points.npz');report['observed_cloud']=observation;report['observed_low_points']=observation['low_points']
   except Exception as e:report['observed_point_error']=str(e)
   save()
   if ok:
    limit(regression_speed)
    for index,target in enumerate(route.get('native_regression_targets',[])):
     live=available();c,r=worker.sweep.index(*target[:2])
     if not worker.sweep.transit_centers[r,c]or not live[r,c]:ok=False;report['regression_target_rejected']=target;break
     precise=route.get('precision_cleaning_connectors',False)
     if precise:worker.stage='sweeping'
     ok=worker.go(target,worker.deadline,timeout=45.,purpose='lane_start' if precise else 'low_chair_regression',recovery=True)
     steps.append(dict(kind='regression',index=index,passed=ok));masks('regression_%02d_masks.npz'%index);save()
     if not ok:break
  if ok and not native_regression:
   live=available();masks('before_local_masks.npz')
   if route.get('derive_execution_segment',False):
    report['original_reference']=[list(first),list(last)]
    execution=worker.sweep.execution_segment(first,last,live,np.asarray(maps['cost'].data).reshape(live.shape),current=worker.pose())
    report['execution_segment']=execution;first,last=execution['start'],execution['end']
   ok=worker.sweep.lane_path(first,last,live)is not None
   report['local_segment_revalidated_after_staging']=ok;save()
  if ok and not native_regression:
   limit(.12)
   precise=route.get('precision_cleaning_approach',False)
   if precise:worker.stage='sweeping'
   ok=worker.go(first,worker.deadline,timeout=65.,purpose='lane_start'if precise else'local_approach',recovery=True);steps.append(dict(kind='approach',passed=ok));masks('approach_terminal.npz');save()
  if ok and not native_regression:
   speed=route.get('local_strip_speed_mps',.12)
   if speed not in(.12,.35):raise ValueError('Unsupported local strip speed')
   limit(speed)
   worker.stage='sweeping';ok=worker.go(last,worker.deadline,timeout=45.,lane=True,purpose='local_strip');steps.append(dict(kind='local_strip',passed=ok));masks('strip_terminal.npz');save()
   if 'original_reference'in report:
    report['original_reference_spatial_coverage']=worker.sweep.spatial_coverage(report['original_reference'])
    ok=ok and report['original_reference_spatial_coverage']['completed'];save()
  worker.stage='returning';limit(0.);charging=dock_at(worker,tuple(state['charging_station']['map_pose']),worker.deadline)if worker.settled else False
  masks('return_terminal.npz')
  report.update(status='passed'if ok and charging and not contacts else'failed',return_passed=charging,return_error=worker.return_errors,local_coverage=worker.sweep.metrics())
 except StopRequested:report.update(status='contact_stopped'if contacts else'interrupted')
 except BaseException as e:report.update(status='failed',error=f'{type(e).__name__}: {e}')
 finally:
  finished.set();worker.stage='cleanup'
  try:
   if not worker.settled:raise RuntimeError('Unconfirmed navigation; retain low speed until owned stack stops')
   limit(0.);report['speed_limit_restored']=True
   if original_period is not None:
    cloud_period(original_period,cleanup=True);report['cloud_period_change']['restored']=True
  except Exception as e:report['restore_limit_error']=str(e)
  report.update(navigation_settled=worker.settled,final_pose=worker.pose(),speed_limit_restored=report.get('speed_limit_restored',False))
  save();sensor_executor.shutdown(timeout_sec=5);sensor_thread.join(timeout=5);observer.destroy_node();worker.node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
