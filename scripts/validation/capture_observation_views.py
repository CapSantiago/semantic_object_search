#!/usr/bin/env python3
"""Read-only RGB-D capture at successful, stationary Nav2 observation poses.

Run beside a serial search matrix. This node never sends navigation commands.
Each saved frame retains measured map TF, arrival errors and action status.
"""
import argparse
import json
from math import atan2, hypot, sin, cos
from pathlib import Path
import signal
import time

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from semantic_search.sensor_qos import sensor_qos_profile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=14400)
    args=parser.parse_args()
    poses=json.loads(args.route.read_text())
    if not poses or any(len(p)!=3 or not np.isfinite(p).all() for p in poses):
        parser.error('Expected finite x/y/yaw poses')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    node=Node('acceptance_view_recorder',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node);bridge=CvBridge()
    report=dict(status='running',method='Read-only capture during successful stationary Nav2 poses',
        source_route=str(args.route),poses=poses,limits=dict(xy_m=.20,yaw_rad=.15,stationary_wall_seconds=.25),
        captures={},planned=len(poses),limitations=[
            'Reference poses are matched to measured TF; action status supplies an ID/status, not the goal request pose.',
            'Visibility uses the measured optical TF, not the reference pose.'])
    stopped=[]
    handlers={sig:signal.signal(sig,lambda s,f:stopped.append(s)) for sig in (signal.SIGINT,signal.SIGTERM)}
    current=dict(status=None,stationary_since=None,odom_wall=0.)
    started=time.monotonic()
    def save():
        temporary=args.output/'report.json.tmp'
        temporary.write_text(json.dumps(report,indent=2)+'\n');temporary.replace(args.output/'report.json')
    def action(message):
        if message.status_list:
            latest=max(message.status_list,key=lambda v:v.goal_info.stamp.sec*10**9+v.goal_info.stamp.nanosec)
            current['status']=latest.status
            current['goal_id']=[int(value) for value in latest.goal_info.goal_id.uuid]
    def odometry(message):
        now=time.monotonic();v=message.twist.twist
        if abs(v.linear.x)<.01 and abs(v.linear.y)<.01 and abs(v.angular.z)<.01:
            if current['stationary_since'] is None:current['stationary_since']=now
        else:current['stationary_since']=None
        current['odom_wall']=now
    node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',action,
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry,'/odom',odometry,qos_profile_sensor_data)
    def frame(rgb,depth,info):
        now=time.monotonic()
        if (current['status']!=4 or current['stationary_since'] is None
            or now-current['stationary_since']<.25 or now-current['odom_wall']>.5):return
        stamp=Time.from_msg(rgb.header.stamp)
        if abs(node.get_clock().now().nanoseconds-stamp.nanoseconds)>500_000_000:return
        try:
            base=buffer.lookup_transform('map','base_link',stamp).transform
            optical=buffer.lookup_transform('map',info.header.frame_id,stamp).transform
        except Exception:return
        q=base.rotation;p=base.translation
        yaw=atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        for index,pose in enumerate(poses):
            key=f'{index:03d}'
            if key in report['captures']:continue
            xy=hypot(p.x-pose[0],p.y-pose[1]);dyaw=abs(atan2(sin(yaw-pose[2]),cos(yaw-pose[2])))
            if xy>.20 or dyaw>.15:continue
            folder=args.output/key;folder.mkdir()
            cv2.imwrite(str(folder/'rgb.png'),bridge.imgmsg_to_cv2(rgb,'bgr8'))
            np.save(folder/'depth.npy',bridge.imgmsg_to_cv2(depth,'passthrough'))
            t,r=optical.translation,optical.rotation
            value=dict(stamp_ns=stamp.nanoseconds,frame=info.header.frame_id,k=list(info.k),
                depth_encoding=depth.encoding,fixed_frame='map',transform=dict(xyz=[t.x,t.y,t.z],xyzw=[r.x,r.y,r.z,r.w]),
                reference_pose=pose,measured_base_pose=[p.x,p.y,yaw],position_error_m=xy,yaw_error_rad=dyaw,
                navigation_status=current['status'],goal_id=current.get('goal_id'),
                stationary_wall_seconds=now-current['stationary_since'])
            (folder/'frame.json').write_text(json.dumps(value,indent=2)+'\n')
            report['captures'][key]=value
            save();print(f'Captured {len(report["captures"])}/{len(poses)}: {pose}',flush=True)
    subscriptions=[Subscriber(node,kind,topic,qos_profile=sensor_qos_profile('reliable'))
        for kind,topic in ((Image,'/rgbd/image'),(Image,'/rgbd/depth_image'),(CameraInfo,'/rgbd/camera_info'))]
    sync=ApproximateTimeSynchronizer(subscriptions,queue_size=30,slop=.075);sync.registerCallback(frame)
    save()
    try:
        while len(report['captures'])<len(poses) and not stopped and time.monotonic()-started<args.timeout:
            rclpy.spin_once(node,timeout_sec=.1)
        report['status']='completed' if len(report['captures'])==len(poses) else 'interrupted' if stopped else 'timeout'
    except BaseException as error:
        report.update(status='failed',error=f'{type(error).__name__}: {error}')
        raise
    finally:
        report['elapsed_wall_seconds']=time.monotonic()-started
        report['missing']=[i for i in range(len(poses)) if f'{i:03d}' not in report['captures']]
        save();node.destroy_node();rclpy.try_shutdown()
        for sig,handler in handlers.items():signal.signal(sig,handler)
    return 0 if report['status']=='completed' else 1


if __name__=='__main__':raise SystemExit(main())
