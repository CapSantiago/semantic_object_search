#!/usr/bin/env python3
"""Read-only synchronized velocity, guard, TF and costmap sampling.

This never publishes a command or reads simulator object truth. Run beside a
normal console request; use the same sampling load for a before/after pair.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.msg import VoxelGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=120.)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--navigation-log',type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0:
        parser.error('seconds must be finite and positive')
    args.output.mkdir(parents=True,exist_ok=False)
    stopped = []
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *unused:stopped.append(True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('motion_profile',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer = Buffer(); listener = TransformListener(buffer,node)
    values = {}; maps = {}
    sensors = {}
    node.create_subscription(VoxelGrid,'/local_costmap/voxel_grid',
        lambda m:sensors.update(voxel=m),qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/rgbd/obstacles',
        lambda m:sensors.update(cloud=m),qos_profile_sensor_data)
    def velocity(key,msg):
        values[key] = (time.monotonic(),[msg.linear.x,msg.linear.y,msg.angular.z])
    for key,topic in [('requested','/cmd_vel'),('filtered','/cmd_vel_collision_filtered'),('safe','/cmd_vel_safe')]:
        node.create_subscription(Twist,topic,lambda m,k=key:velocity(k,m),1)
    def status(msg):
        try:values['guard'] = (time.monotonic(),json.loads(msg.data))
        except ValueError:pass
    node.create_subscription(String,'/safety/state',status,1)
    def cloud_status(msg):
        try:values['cloud_adapter']=(time.monotonic(),json.loads(msg.data))
        except ValueError:pass
    node.create_subscription(String,'/rgbd/cloud_adapter_state',cloud_status,1)
    def odometry(msg):
        p=msg.pose.pose.position;v=msg.twist.twist
        values['odom'] = (time.monotonic(),dict(pose=[p.x,p.y,yaw(msg.pose.pose.orientation)],
            xyz=[p.x,p.y,p.z],quaternion=[msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,msg.pose.pose.orientation.w],
            velocity=[v.linear.x,v.linear.y,v.angular.z],stamp=Time.from_msg(msg.header.stamp).nanoseconds/1e9))
    node.create_subscription(Odometry,'/odom',odometry,qos_profile_sensor_data)
    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key,topic in [('global','/global_costmap/costmap'),('local','/local_costmap/costmap')]:
        node.create_subscription(OccupancyGrid,topic,lambda m,k=key:maps.update({k:(time.monotonic(),m)}),qos)
    def transform(target,source):
        try:
            t=buffer.lookup_transform(target,source,Time());p=t.transform.translation
            q=t.transform.rotation
            return dict(pose=[p.x,p.y,yaw(q)],xyz=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],
                stamp=Time.from_msg(t.header.stamp).nanoseconds/1e9)
        except Exception:return None
    rows=[];started=time.monotonic();started_epoch=time.time();next_sample=started
    next_log=started;log_position=args.navigation_log.stat().st_size if args.navigation_log else 0
    failures=0
    try:
        with (args.output/'samples.jsonl').open('x') as output:
            while not stopped and time.monotonic()-started<args.seconds:
                rclpy.spin_once(node,timeout_sec=.02)
                now=time.monotonic()
                if now<next_sample:continue
                next_sample=now+.1
                row=dict(wall_seconds=now-started,sim_seconds=node.get_clock().now().nanoseconds/1e9)
                for key,(received,value) in values.items():
                    row[key]=dict(age_wall_seconds=now-received,value=value)
                row['map_to_odom']=transform('map','odom')
                row['odom_to_base']=transform('odom','base_link')
                row['map_to_base']=transform('map','base_link')
                row['costmaps']={}
                for key,(received,m) in maps.items():
                    pose=row['map_to_base' if m.header.frame_id=='map' else 'odom_to_base']
                    if pose is None:continue
                    p=m.info.origin.position;a=yaw(m.info.origin.orientation);x,y=pose['pose'][:2]
                    c=math.floor((math.cos(a)*(x-p.x)+math.sin(a)*(y-p.y))/m.info.resolution)
                    r=math.floor((-math.sin(a)*(x-p.x)+math.cos(a)*(y-p.y))/m.info.resolution)
                    row['costmaps'][key]=dict(frame=m.header.frame_id,age_wall_seconds=now-received,
                        stamp=Time.from_msg(m.header.stamp).nanoseconds/1e9,cell=[c,r],
                        cost=int(m.data[r*m.info.width+c]) if 0<=c<m.info.width and 0<=r<m.info.height else None)
                output.write(json.dumps(row,separators=(',',':'))+'\n');rows.append(row)
                if args.navigation_log and now>=next_log:
                    next_log=now+1.
                    with args.navigation_log.open() as source:
                        source.seek(log_position);new=source.read();log_position=source.tell()
                    if any(event in new for event in ('Goal failed','Failed to make progress')) and failures<8:
                        failures+=1;arrays={};metadata={}
                        for key,(received,m) in maps.items():
                            arrays[key]=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width)
                            p=m.info.origin.position
                            metadata[key]=dict(width=m.info.width,height=m.info.height,resolution=m.info.resolution,
                                origin=[p.x,p.y,yaw(m.info.origin.orientation)],frame_id=m.header.frame_id,
                                stamp=Time.from_msg(m.header.stamp).nanoseconds/1e9,age_wall_seconds=now-received)
                        voxel=sensors.get('voxel')
                        if voxel is not None:
                            arrays['voxel']=np.asarray(voxel.data,dtype=np.uint32).reshape(voxel.size_y,voxel.size_x)
                            metadata['voxel']=dict(origin=[voxel.origin.x,voxel.origin.y,voxel.origin.z],
                                resolution=[voxel.resolutions.x,voxel.resolutions.y,voxel.resolutions.z],
                                stamp=Time.from_msg(voxel.header.stamp).nanoseconds/1e9,frame_id=voxel.header.frame_id)
                        cloud=sensors.get('cloud')
                        if cloud is not None and cloud.point_step==12 and not cloud.is_bigendian:
                            # Compact XYZ adapter output; preserve source stamp and
                            # both exact observation transforms, never simulator truth.
                            arrays['cloud_xyz']=np.frombuffer(cloud.data,dtype='<f4').reshape(-1,3)
                            metadata['cloud']=dict(frame_id=cloud.header.frame_id,
                                stamp=Time.from_msg(cloud.header.stamp).nanoseconds/1e9,transforms={})
                            for frame in ('map','odom'):
                                try:
                                    t=buffer.lookup_transform(frame,cloud.header.frame_id,Time.from_msg(cloud.header.stamp))
                                    p=t.transform.translation;q=t.transform.rotation
                                    metadata['cloud']['transforms'][frame]=dict(xyz=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w])
                                except Exception as error:metadata['cloud']['transforms'][frame]=dict(error=str(error))
                        np.savez_compressed(args.output/f'failure_{failures:02d}.npz',**arrays)
                        (args.output/f'failure_{failures:02d}.json').write_text(json.dumps(dict(
                            sample=row,maps=metadata,new_log=new,
                            note='First sample after a new Goal failed or Failed to make progress log, at most1s later; not a controller-internal synchronized costmap.'),indent=2)+'\n')
    finally:
        node.destroy_node();rclpy.try_shutdown()
    def active(row,key):
        x=row.get(key) or {};v=x.get('value')
        return (x.get('age_wall_seconds',math.inf)<=.3 and v is not None and
                (math.hypot(*v[:2])>.015 or abs(v[2])>.04))
    requested=filtered=blocked=stale_blocked=odom_distance=0.
    transitions=0;previous_blocked=False
    for prev,row in zip(rows,rows[1:]):
        dt=row['wall_seconds']-prev['wall_seconds'];wanted=active(row,'requested');middle=active(row,'filtered')
        requested+=dt if wanted else 0.;filtered+=dt if wanted and middle else 0.
        zero=wanted and middle and not active(row,'safe');blocked+=dt if zero else 0.
        guard=(row.get('guard') or {}).get('value') or {};reasons=guard.get('reasons') or []
        stale_blocked+=dt if zero and any('stale' in x for x in reasons) else 0.
        transitions+=int(zero and not previous_blocked);previous_blocked=zero
        before=(prev.get('odom') or {}).get('value');after=(row.get('odom') or {}).get('value')
        if before and after and 0<after['stamp']-before['stamp']<=.5:
            distance=math.dist(before['pose'][:2],after['pose'][:2])
            if distance<=.3:odom_distance+=distance
    def quantiles(values):
        return {k:float(np.percentile(values,q)) for k,q in [('p50',50),('p95',95),('p99',99),('max',100)]} if values else None
    speeds=[abs(row['odom']['value']['velocity'][0]) for row in rows if active(row,'requested') and 'odom' in row]
    report=dict(scope=__doc__,requested_seconds=args.seconds,elapsed_wall_seconds=time.monotonic()-started,
        started_wall_epoch=started_epoch,failed_goal_snapshots=failures,
        sample_count=len(rows),requested_nonzero_seconds=requested,filtered_nonzero_seconds=filtered,
        filtered_nonzero_but_safe_zero_seconds=blocked,stale_reason_correlated_zero_seconds=stale_blocked,
        filtered_to_safe_zero_transitions=transitions,odom_distance_m=odom_distance,odom_linear_speed_mps=quantiles(speeds),
        limitations=['10Hz arrival-aligned sampling, not exact controller-internal timing; sub-sample stops can be missed.',
                     'Guard reason correlation uses its last status message; warnings alone are not counted as a physical stop.',
                     'A zero output while the filtered command is nonzero can be intentional data protection; never disable it to improve this metric.'])
    (args.output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()
