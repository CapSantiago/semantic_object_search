"""Independent recorded native geometry, invalid-depth safety and startup gating."""
import json
from pathlib import Path
from types import MethodType,SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from semantic_search.cloud_projection import depth_xyz,projected_clouds,verify_native_projection
from semantic_search.clouds import clearing_cloud,sample_cloud


def recorded_pair():
    f=Path(__file__).parent/'fixtures/depth_native_fortress.npz'
    z=np.load(f);m=json.loads(f.with_suffix('.json').read_text());stride=m['sampling_stride']
    image=np.full((m['height'],m['width']),np.nan,dtype='<f4')
    native=np.full((m['height'],m['width'],3),np.nan,dtype='<f4')
    image[::stride,::stride]=z['depth_sample'];native[::stride,::stride]=z['native_xyz']
    header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0),frame_id=m['depth_frame'])
    depth=SimpleNamespace(header=header,data=image.tobytes(),width=m['width'],height=m['height'],
        step=m['width']*4,is_bigendian=False,encoding='32FC1')
    fields=[SimpleNamespace(name=name,offset=i*4,datatype=7,count=1)for i,name in enumerate(('x','y','z'))]
    cloud=SimpleNamespace(header=header,data=native.tobytes(),width=m['width'],height=m['height'],
        point_step=12,row_step=m['width']*12,fields=fields,is_bigendian=False)
    info=SimpleNamespace(header=header,width=m['width'],height=m['height'],k=m['k'],d=m['distortion'])
    return m,depth,cloud,info,z['native_xyz'].reshape(-1,3)


def project(depth,info,stride):
    return depth_xyz(depth.data,depth.width,depth.height,depth.step,depth.is_bigendian,
                     depth.encoding,info.k,stride)


def test_recorded_native_marking_and_clearing_geometry_are_preserved():
    m,depth,cloud,info,native=recorded_pair();xyz,rays=project(depth,info,m['sampling_stride'])
    check=verify_native_projection(native,xyz)
    assert check['finite_points']==15256 and check['maximum_error_m']<.000002
    marked,cleared,count=projected_clouds(xyz,rays,8.)
    expected=sample_cloud(cloud.data,cloud.width,cloud.height,cloud.point_step,cloud.row_step,
        cloud.fields,stride=m['sampling_stride'],compact=True)
    expected_clear,expected_count=clearing_cloud(cloud.data,cloud.width,cloud.height,
        cloud.point_step,cloud.row_step,cloud.fields,False,info.k,8.,m['sampling_stride'])
    assert count==expected_count==3944
    assert np.allclose(np.frombuffer(marked,'<f4'),np.frombuffer(expected,'<f4'),atol=.000002)
    assert np.allclose(np.frombuffer(cleared,'<f4'),np.frombuffer(expected_clear,'<f4'),atol=.000002)


def test_native_mismatch_and_false_no_return_are_not_calibration():
    m,d,c,i,native=recorded_pair();xyz,_=project(d,i,m['sampling_stride'])
    broken=xyz.copy();index=np.flatnonzero(np.isfinite(native).all(axis=1))[0];broken[index,0]+=.01
    with pytest.raises(ValueError,match='coordinates'):verify_native_projection(native,broken)
    index=np.flatnonzero(np.isposinf(native[:,0]))[0];broken=xyz.copy();broken[index]=np.nan
    with pytest.raises(ValueError,match='no-return'):verify_native_projection(native,broken)


def test_nan_zero_negative_and_negative_inf_never_become_free_rays():
    image=np.array([[1.]*8+[np.inf,np.nan,0.,-np.inf,-1.]],dtype='<f4')
    xyz,rays=depth_xyz(image.tobytes(),13,1,52,False,'32FC1',[100.,0.,6.5,0.,100.,.5,0.,0.,1.],1)
    marked,cleared,count=projected_clouds(xyz,rays,8.)
    assert len(marked)//12==8 and len(cleared)//12==9 and count==1
    assert np.isfinite(np.frombuffer(cleared,'<f4')).all()
    _,_,count=projected_clouds(xyz[1:],rays[1:],8.);assert count==0
    _,cleared,count=projected_clouds(xyz,rays,0.);assert not cleared and count==0


def test_depth_padding_endianness_encoding_and_truncation():
    data=np.array([[1000,2000,99],[3000,4000,99]],dtype='>u2').tobytes()
    xyz,_=depth_xyz(data,2,2,6,True,'16UC1',[10.,0.,1.,0.,10.,1.,0.,0.,1.],1)
    assert np.array_equal(xyz[:,0],[1.,2.,3.,4.])
    with pytest.raises(ValueError,match='Truncated'):depth_xyz(data[:-3],2,2,6,True,'16UC1',[1.]*9)
    with pytest.raises(ValueError,match='encoding'):depth_xyz(data,2,2,6,True,'rgb8',[1.]*9)


def calibration_node():
    pytest.importorskip('rclpy')
    from semantic_search.cloud_adapter import Adapter
    m,d,c,i,native=recorded_pair();now=SimpleNamespace(nanoseconds=1_100_000_000)
    parameters={'maximum_input_age':.25,'sampling_stride':m['sampling_stride']}
    node=SimpleNamespace(calibration_verified=False,calibration_pairs=0,calibration_last_stamp=None,
        calibration_signature=None,calibration_metrics=None,calibration_last_error=None,
        get_parameter=lambda key:SimpleNamespace(value=parameters[key]),
        get_clock=lambda:SimpleNamespace(now=lambda:now),depth_signature=Adapter.depth_signature)
    node.project_depth=MethodType(Adapter.project_depth,node)
    return Adapter,node,now,d,c,i


def test_three_distinct_exact_stamp_native_pairs_are_required():
    Adapter,node,now,depth,cloud,info=calibration_node()
    for _ in range(3):Adapter.calibrate_depth(node,cloud,depth,info)
    assert node.calibration_pairs==1 and not node.calibration_verified
    for second in (2,3):
        cloud.header.stamp.sec=second;now.nanoseconds=second*10**9+100_000_000
        Adapter.calibrate_depth(node,cloud,depth,info)
    assert node.calibration_pairs==3 and node.calibration_verified
    assert node.calibration_metrics['maximum_error_m']<.000002


def test_unmatched_calibration_stamp_cannot_bootstrap_output():
    Adapter,node,now,depth,cloud,info=calibration_node()
    depth.header=SimpleNamespace(frame_id=depth.header.frame_id,stamp=SimpleNamespace(sec=1,nanosec=1))
    Adapter.calibrate_depth(node,cloud,depth,info)
    assert not node.calibration_verified and node.calibration_pairs==0
    assert 'exact source stamps'in node.calibration_last_error


def test_changed_intrinsics_after_calibration_publish_neither_marking_nor_clearing():
    Adapter,node,now,depth,cloud,info=calibration_node()
    node.calibration_verified=True;node.calibration_signature=Adapter.depth_signature(depth,info)
    node.received=node.stale=node.clearing_errors=0;node.last=None;node.camera_info=info
    node.publisher=Mock();node.clearing_publisher=Mock();node.report=Mock();node.release_calibration_streams=Mock()
    info.k[0]*=1.01
    Adapter.receive_depth(node,depth)
    node.publisher.publish.assert_not_called();node.clearing_publisher.publish.assert_not_called()
    assert node.clearing_errors==1 and 'calibration changed'in node.calibration_last_error
