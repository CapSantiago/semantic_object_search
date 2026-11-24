#!/usr/bin/env python3
"""Read-only multi-class survey during actual stationary search observations.

No commands, simulator truth, or memory writes. Counts are provisional findings
from the executed views, not category admission or a complete room inventory.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from cv_bridge import CvBridge,CvBridgeError
from message_filters import ApproximateTimeSynchronizer,Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from yolo_msgs.msg import DetectionArray
from semantic_search.depth import instance_geometry,truncated_box,compatible_size
from semantic_search.objects import OBJECTS
from semantic_search.rooms import belongs_to
from semantic_search.semantic_map import LandmarkMap
from semantic_search.sensor_qos import sensor_qos_profile

ROOT = Path(__file__).resolve().parents[2]


def floor_proposal(depth,k,scale,transform):
    """Fit a visible low plane in measured base coordinates; never apply it."""
    v,u = np.mgrid[0:depth.shape[0]:12,0:depth.shape[1]:12]
    z = depth[::12,::12].astype(float)*scale
    valid = np.isfinite(z)&(z>.1)&(z<5.)
    u,v,z = u[valid],v[valid],z[valid]
    points = np.column_stack(((u-k[2])*z/k[0],(v-k[5])*z/k[4],z))
    if len(points)<150:return dict(available=False,reason='not_enough_valid_depth')
    t,q = transform.translation,transform.rotation
    points = Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points)+[t.x,t.y,t.z]
    points = points[(np.linalg.norm(points[:,:2],axis=1)<3.5)&(points[:,2]>-.10)&(points[:,2]<.10)]
    if len(points)<150:return dict(available=False,reason='not_enough_low_plane_samples')
    matrix = np.column_stack((points[:,:2],np.ones(len(points))))
    fit = least_squares(lambda p:matrix@p-points[:,2],[0.,0.,float(np.median(points[:,2]))],
                        loss='soft_l1',f_scale=.002)
    params = fit.x
    for _ in range(3):
        keep = abs(matrix@params-points[:,2])<.003
        if keep.sum()<150:return dict(available=False,reason='not_enough_planar_inliers')
        params = np.linalg.lstsq(matrix[keep],points[keep,2],rcond=None)[0]
    normal = np.array([-params[0],-params[1],1.]);normal /= np.linalg.norm(normal)
    inliers = points[keep]
    residual = abs(matrix[keep]@params-inliers[:,2])
    height = float(-params[2]/math.sqrt(1+params[0]**2+params[1]**2))
    span = np.ptp(inliers[:,:2],axis=0)
    return dict(available=True,method='sensor_only_low_plane_robust_fit',points=len(points),
        inliers=int(keep.sum()),inlier_fraction=float(keep.mean()),
        base_height_m=height,
        zero_heading_roll_pitch_rad=[math.atan2(float(normal[1]),float(normal[2])),-math.asin(float(normal[0]))],
        residual_p95_m=float(np.quantile(residual,.95)),xy_span_m=span.tolist(),
        quality_checks=dict(inlier_fraction=bool(keep.mean()>=.7),spatial_span=bool(min(span)>.4),
                            plausible_height=bool(0<height<.1),small_tilt=bool(normal[2]>math.cos(.1))),
        applied_to_runtime=False,heading_observable=False,
        limitation='Low horizontal surface assumed; requires multi-view and physical validation before calibration use.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=1800)
    parser.add_argument('--capture-views',action='store_true',help='retain one synchronized RGB-D frame per actual observation view')
    parser.add_argument('--categories',nargs='+',choices=tuple(OBJECTS),help='只读采样指定类别，减少并行诊断负载')
    args = parser.parse_args()
    selected=args.categories or list(OBJECTS)
    meta = json.loads((args.session/'session.json').read_text())
    if str(meta['config']['domain'])!=os.environ.get('ROS_DOMAIN_ID') or not 0<args.timeout<=14400:
        parser.error('Use the matching ROS domain and a bounded positive timeout')
    args.output.mkdir(parents=True,exist_ok=False)
    rooms = json.loads((ROOT/meta['config']['rooms']).read_text())
    report = dict(status='running',session=str(args.session),runtime_config=meta['config'],
        source_sha256=meta['source_sha256'],driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        method='Passive synchronized YOLO masks, actual depth, map TF and stationary odometry during search observing stages',
        selected_categories=selected,
        criteria=dict(score=.35,merge_radius_m=.30,min_hits=3,clipping_margin_px=2),tasks={},
        limitations=['Provisional measurements from executed views; not formal category admission.',
                     'Uses the same visible-surface geometry and association as the search baseline.',
                     'No full-room coverage or physical identity guarantee.'])
    rejected = Counter()
    trackers = {}
    progress = {}
    last_progress = last_save = 0.
    last_stamp = -1
    motion = dict(wall=-math.inf,stamp=-math.inf,stationary=False)
    stopped = []
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda s,f:stopped.append(s))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('passive_object_survey',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer = Buffer();listener = TransformListener(buffer,node);bridge = CvBridge()
    def save():
        nonlocal last_save
        for task_id,maps in trackers.items():
            report['tasks'][task_id]['categories'] = {name:dict(count=len(items.confirmed),
                objects=[asdict(o) for o in items.confirmed],
                tentative_objects=[asdict(o) for o in items.items if o.hits<items.min_hits])
                for name,items in maps.items()}
        report['rejected_frames_or_detections'] = dict(rejected)
        p = args.output/'report.json.tmp';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');p.replace(args.output/'report.json')
        last_save = time.monotonic()
    def stage(message):
        nonlocal progress,last_progress
        try:value = json.loads(message.data)
        except (ValueError,TypeError):return
        if value.get('room') not in rooms or not value.get('task_id'):return
        progress,last_progress = value,time.monotonic()
        key = value['task_id']
        if key not in trackers:
            trackers[key] = {name:LandmarkMap(.30,3) for name in selected}
            report['tasks'][key] = dict(request_id=value.get('request_id'),requested_room=value['room'],
                requested_category=value.get('category'),requested_mode=value.get('mode'),frames=[],views=[],
                task_status=None,captures={},floor_proposals={})
        if value['stage']=='finished':
            report['tasks'][key]['task_status'] = value.get('status')
        save()
    def odom(message):
        stamp = Time.from_msg(message.header.stamp).nanoseconds/1e9
        if stamp<=motion['stamp']:return
        v = message.twist.twist
        motion.update(wall=time.monotonic(),stamp=stamp,
            stationary=abs(v.linear.x)<.01 and abs(v.linear.y)<.01 and abs(v.angular.z)<.03)
    node.create_subscription(String,'/search/progress',stage,10)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    def observe(detections,depth,info,rgb=None):
        nonlocal last_stamp
        wall = time.monotonic()
        if progress.get('stage')!='observing' or wall-last_progress>12.:return
        if (not motion['stationary'] or wall-motion['wall']>.4
                or not -.1<=node.get_clock().now().nanoseconds/1e9-motion['stamp']<=.4):
            rejected['not_stationary'] += 1;return
        stamp = Time.from_msg(detections.header.stamp)
        if stamp.nanoseconds<=last_stamp or not -.1<=(node.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9<=.5:
            rejected['stamp'] += 1;return
        if (depth.encoding not in ('32FC1','16UC1') or min(depth.width,depth.height)<=0
            or depth.width!=info.width or depth.height!=info.height
            or not all(math.isfinite(k) for k in info.k) or min(info.k[0],info.k[4])<=0):
            rejected['image_or_intrinsics'] += 1;return
        try:
            t = buffer.lookup_transform('map',info.header.frame_id,stamp).transform
            image = bridge.imgmsg_to_cv2(depth,'passthrough')
        except (TransformException,CvBridgeError,ValueError):
            rejected['tf_or_depth'] += 1;return
        last_stamp = stamp.nanoseconds
        room = progress['room'];task_id = progress['task_id']
        xyz = [t.translation.x,t.translation.y,t.translation.z]
        q = [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
        points = {name:[] for name in selected}
        for d in detections.detections:
            category = d.class_name.lower()
            if category not in selected or not math.isfinite(d.score) or d.score<.35:continue
            b = d.bbox
            if truncated_box(b.center.position.x,b.center.position.y,b.size.x,b.size.y,depth.width,depth.height):
                rejected['clipped'] += 1;continue
            if not d.mask.data or d.mask.width!=depth.width or d.mask.height!=depth.height:
                rejected['mask'] += 1;continue
            geometry = instance_geometry(image,[(p.x,p.y) for p in d.mask.data],info.k,xyz,q,
                                         .001 if depth.encoding=='16UC1' else 1.,category=category)
            if geometry is None or not compatible_size(category,geometry[1]):
                rejected['projection_or_size'] += 1;continue
            if belongs_to(rooms,room,*geometry[0][:2]):points[category].append(geometry[0])
        observed = {}
        for name,items in points.items():
            if items:
                ids = trackers[task_id][name].observe(items)
                observed[name] = dict(points=items,instance_ids=ids)
        record = report['tasks'][task_id]
        if len(record['frames'])>=12000:
            raise RuntimeError('Survey frame budget exceeded')
        view = progress.get('viewpoint_index')
        if view not in record['views']:record['views'].append(view)
        key = str(view)
        if key not in record['floor_proposals']:
            try:
                relative = buffer.lookup_transform('base_link',info.header.frame_id,stamp).transform
                record['floor_proposals'][key] = dict(stamp_ns=stamp.nanoseconds,
                    **floor_proposal(image,info.k,.001 if depth.encoding=='16UC1' else 1.,relative))
            except TransformException:
                rejected['floor_base_tf'] += 1
        if rgb is not None and key not in record['captures']:
            import cv2
            base = buffer.lookup_transform('map','base_link',stamp).transform
            folder = args.output/'captures'/task_id/key;folder.mkdir(parents=True)
            if not cv2.imwrite(str(folder/'rgb.png'),bridge.imgmsg_to_cv2(rgb,'bgr8')):
                raise OSError('Could not save survey RGB evidence')
            np.save(folder/'depth.npy',image)
            frame = dict(stamp_ns=stamp.nanoseconds,frame=info.header.frame_id,k=list(info.k),
                rgb_stamp_ns=Time.from_msg(rgb.header.stamp).nanoseconds,
                depth_stamp_ns=Time.from_msg(depth.header.stamp).nanoseconds,
                depth_encoding=depth.encoding,fixed_frame='map',
                transform=dict(xyz=xyz,xyzw=q),
                measured_base_transform=dict(xyz=[base.translation.x,base.translation.y,base.translation.z],
                    xyzw=[base.rotation.x,base.rotation.y,base.rotation.z,base.rotation.w]),
                detections=[dict(category=d.class_name,score=float(d.score),
                    xywh=[d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.size.x,d.bbox.size.y],
                    mask=[[p.x,p.y] for p in d.mask.data]) for d in detections.detections])
            (folder/'frame.json').write_text(json.dumps(frame,indent=2)+'\n')
            record['captures'][key] = str(folder.relative_to(args.output))
        record['frames'].append(dict(stamp_ns=stamp.nanoseconds,viewpoint=view,observed=observed))
        if wall-last_save>1.:save()
    subscriptions = [Subscriber(node,kind,topic,qos_profile=sensor_qos_profile('reliable'))
        for kind,topic in ((DetectionArray,'/yolo/detections'),(Image,'/rgbd/depth_image'),(CameraInfo,'/rgbd/camera_info'))]
    if args.capture_views:
        subscriptions.append(Subscriber(node,Image,'/rgbd/image',qos_profile=sensor_qos_profile('reliable')))
    synchronizer = ApproximateTimeSynchronizer(subscriptions,queue_size=20,slop=.075)
    synchronizer.registerCallback(observe)
    started = time.monotonic();save()
    try:
        while not stopped and time.monotonic()-started<args.timeout:
            rclpy.spin_once(node,timeout_sec=.1)
        report['status'] = 'stopped' if stopped else 'time_budget_reached'
    except BaseException as error:
        report.update(status='failed',error=str(error));raise
    finally:
        report['elapsed_seconds'] = round(time.monotonic()-started,2)
        save();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':
    main()
