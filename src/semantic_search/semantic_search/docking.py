"""Configured service facility and shared measured docking, without object truth."""
import json
import math
from pathlib import Path
import time

from .cleaning import RETURN_POSITION_M, RETURN_YAW_RAD, transit_timeout
from .map_resources import resolve_map, same_map


def load_station(path, root, *, check_map=True, active_map=None):
    root=Path(root).resolve();path=Path(path)
    if not path.is_absolute():path=root/path
    if not path.resolve().is_relative_to(root):
        raise ValueError('充电配置必须位于项目目录内。')
    station=json.loads(path.read_text())
    if type(station.get('precision_docking',False)) is not bool:
        raise ValueError('停靠精度候选标志必须为布尔值。')
    if station.get('version')!=1 or station.get('frame_id')!='map' or station.get('simulation_only') is not True:
        raise ValueError('充电配置版本、坐标系或模拟充电标志无效。')
    for key in ('map_pose','world_spawn_pose'):
        pose=station.get(key)
        if not isinstance(pose,list) or len(pose)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in pose):
            raise ValueError('充电/出生位姿必须是有限的x、y、yaw。')
    if not station.get('id') or not station.get('room') or not station.get('display_name'):
        raise ValueError('充电配置缺少标识或房间。')
    model=station.get('model') or {}
    if not model.get('name'):
        raise ValueError('充电配置缺少场景模型。')
    for key in ('pose','size'):
        values=model.get(key)
        if not isinstance(values,list) or len(values)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in values):
            raise ValueError('充电模型尺寸与位姿无效。')
        if key=='size' and not all(0<v<=2 for v in values):raise ValueError('充电模型尺寸无效。')
    if check_map:
        resource=resolve_map(root,yaml_path=station['map_yaml'])
        for key,digest in [('yaml',resource['requested_yaml_sha256']),('image',resource['image_sha256'])]:
            if digest!=station['map_sha256'][key]:
                raise ValueError('充电位姿绑定的地图版本不一致。')
        if active_map is not None and not same_map(resource,active_map):
            raise ValueError('充电位姿绑定的地图版本与当前载入地图不一致。')
    return station


def world_pose_to_map(station, current):
    """Apply the fixed map/world alignment to current odometry, including restarts."""
    m,w=station['map_pose'],station['world_spawn_pose']
    angle=m[2]-w[2];c,s=math.cos(angle),math.sin(angle)
    return (m[0]+c*(current[0]-w[0])-s*(current[1]-w[1]),
            m[1]+s*(current[0]-w[0])+c*(current[1]-w[1]),
            math.remainder(current[2]+angle,2*math.pi))


def dock_at(worker, target, deadline, *, exit_pose=None, can_exit=None):
    """Use the shared Nav2 client, its terminal/stop barrier and actual tolerances."""
    worker.stage,worker.return_state='returning','returning'
    current=worker.pose()
    if exit_pose and current and (can_exit is None or can_exit(current)):
        timeout=transit_timeout(current,exit_pose)
        reserve=transit_timeout(exit_pose,target,minimum=30.)+24.
        cutoff=min(time.monotonic()+timeout,deadline-reserve)
        if cutoff>time.monotonic():
            worker.go(exit_pose,cutoff,timeout=timeout,purpose='room_exit')
    timeout=transit_timeout(current or target,target,minimum=50.)
    returned=worker.go(target,deadline,timeout=timeout,purpose='dock')
    current=worker.pose()
    measured=(current is not None and math.dist(current[:2],target[:2])<=RETURN_POSITION_M and
              abs(math.remainder(current[2]-target[2],2*math.pi))<=RETURN_YAW_RAD)
    if (not returned or not measured) and current and math.dist(current[:2],target[:2])<=.75:
        x,y,_=target
        if math.dist(current[:2],target[:2])>RETURN_POSITION_M:
            bearing=math.atan2(y-current[1],x-current[0])
            worker.go((x,y,bearing),deadline,timeout=12.,purpose='dock_approach')
        returned=worker.go(target,deadline,timeout=12.,purpose='dock_alignment')
    worker.wait_for(lambda:worker.parked() and worker.pose(),5)
    worker.final_pose=worker.pose()
    if worker.final_pose is None:raise ValueError('停靠后定位不可用。')
    xy=math.dist(target[:2],worker.final_pose[:2])
    heading=abs(math.remainder(target[2]-worker.final_pose[2],2*math.pi))
    worker.return_errors=dict(position_m=xy,yaw_rad=heading,
        stationary_seconds=(worker.odom_stamp-worker.stationary_since)/1e9)
    charging=bool(returned and xy<=RETURN_POSITION_M and heading<=RETURN_YAW_RAD)
    worker.return_state='simulated_charging' if charging else 'return_incomplete'
    return charging
