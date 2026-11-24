import copy
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from semantic_search.docking import load_station, world_pose_to_map
from semantic_search.development import add_charging_station

ROOT=Path(__file__).resolve().parents[1]


def test_station_is_bound_to_both_map_files_and_rejects_modified_image(tmp_path):
    station=json.loads((ROOT/'src/semantic_search/config/charging_station.json').read_text())
    station['map_yaml']='map/house.yaml';folder=tmp_path/'map';folder.mkdir()
    (folder/'house.yaml').write_text('image: house.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    (folder/'house.pgm').write_bytes(b'original-map-image')
    station['map_sha256']={key:hashlib.sha256((folder/name).read_bytes()).hexdigest()
        for key,name in [('yaml','house.yaml'),('image','house.pgm')]}
    path=tmp_path/'station.json';path.write_text(json.dumps(station))
    assert load_station(path,tmp_path)['map_pose']==[-1.8,-4.,0.]
    (folder/'house.pgm').write_bytes(b'changed-map-image')
    with pytest.raises(ValueError,match='地图版本'):load_station(path,tmp_path)


def test_world_odometry_is_not_double_shifted_and_mode_restart_uses_current_pose():
    station=dict(map_pose=[-1.8,-4.,0.],world_spawn_pose=[-1.8,-4.,0.])
    assert world_pose_to_map(station,(-1.8,-4.,0.))==pytest.approx((-1.8,-4.,0.))
    assert world_pose_to_map(station,(2.,.7,.4))==pytest.approx((2.,.7,.4))
    rotated=dict(map_pose=[4.,5.,math.pi/2],world_spawn_pose=[1.,2.,0.])
    assert world_pose_to_map(rotated,(2.,2.,0.))==pytest.approx((4.,6.,math.pi/2))


def test_station_binding_must_match_the_selected_navigation_map(tmp_path):
    from semantic_search.map_resources import resolve_map
    station=json.loads((ROOT/'src/semantic_search/config/charging_station.json').read_text())
    for name,origin in [('old',[0,0,0]),('new',[1,0,0])]:
        folder=tmp_path/name;folder.mkdir()
        (folder/'house.yaml').write_text('image: house.pgm\nresolution: 0.05\norigin: '+json.dumps(origin)+'\n')
        (folder/'house.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    station['map_yaml']='old/house.yaml';old=resolve_map(tmp_path,yaml_path=station['map_yaml'])
    station['map_sha256']={'yaml':old['yaml_sha256'],'image':old['image_sha256']}
    file=tmp_path/'station.json';file.write_text(json.dumps(station))
    assert load_station(file,tmp_path,active_map=old)['id']==station['id']
    with pytest.raises(ValueError,match='当前载入地图'):
        load_station(file,tmp_path,active_map=resolve_map(tmp_path,yaml_path='new/house.yaml'))


def test_station_model_and_spawn_share_config_without_changing_scene_objects():
    station=load_station(ROOT/'src/semantic_search/config/charging_station.json',ROOT)
    world=ET.fromstring('<world><model name="waffle"><pose>0 0 0 0 0 0</pose></model><model name="chair"/></world>')
    robot=world.find('model');old_chair=ET.tostring(world.find("model[@name='chair']"))
    add_charging_station(world,robot,station)
    pose=list(map(float,robot.find('pose').text.split()))
    assert [pose[0],pose[1],pose[5]]==station['world_spawn_pose']
    assert world.find("model[@name='home_charging_station']/static").text=='true'
    assert ET.tostring(world.find("model[@name='chair']"))==old_chair
    assert world.find(".//collision[@name='docking_pad']") is None


def test_cleaning_uses_fixed_dock_without_overwriting_its_actual_start():
    pytest.importorskip('rclpy')
    from run_cleaning import CleaningRun
    poses=iter([(9.5,0.,0.),(9.96,0.,0.),(9.96,0.,0.)]);commands=[]
    worker=SimpleNamespace(start_pose=(0.,0.,0.),dock_pose=(10.,0.,0.),
        pose=lambda:next(poses),go=lambda target,*a,**k:commands.append(target) or True,
        deadline=time.monotonic()+120,wait_for=lambda *a,**k:None,parked=lambda:True,
        odom_stamp=500_000_000,stationary_since=0)
    assert CleaningRun.return_to_start(worker,None)
    assert commands==[(10.,0.,0.)] and worker.start_pose==(0.,0.,0.)
    assert worker.return_errors['position_m']==pytest.approx(.04)


def test_localization_rejects_stale_wrong_frame_or_invalid_quaternion():
    pytest.importorskip('rclpy')
    from nav_msgs.msg import Odometry
    from initialize_sim_amcl import fresh_odometry
    msg=Odometry();msg.header.frame_id='odom';msg.header.stamp.sec=10;msg.pose.pose.orientation.w=1.
    assert fresh_odometry(msg,20.,20.1,10.1)
    assert not fresh_odometry(msg,20.,20.5,10.1)
    assert not fresh_odometry(msg,20.,20.1,10.5)
    msg.header.frame_id='wrong';assert not fresh_odometry(msg,20.,20.1,10.1)
    msg.header.frame_id='odom';msg.pose.pose.orientation.w=0.
    assert not fresh_odometry(msg,20.,20.1,10.1)
