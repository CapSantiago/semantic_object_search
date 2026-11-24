"""A confirmed find must finish its observation window before leaving."""
from types import MethodType,SimpleNamespace
from unittest.mock import Mock,patch

import numpy as np
import pytest

pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time as TimeMessage
from semantic_search.search_node import Search


def diagnostic(index=0,**changes):
    row=dict(index=index,pose=[float(index),0.,0.],reached=True,navigation_status=4,
        dwell_completed=False,synchronized_messages=0,rejected_sensor_frames=0,
        synchronized_frames=2,valid_projections=2,tf_failures=0,invalid_target_depth=0,
        extra_wait_started=False,waiting_for_sensor_data=False,extra_wait_seconds=0.)
    row.update(changes)
    return row


def test_find_keeps_sampling_confirmed_target_before_nominal_dwell():
    now=SimpleNamespace(nanoseconds=2_000_000_000)
    node=SimpleNamespace(phase='observing',observe_start=1_000_000_000,
        observe_dwell_seconds=4.,observe_extra_seconds=4.,last_stamp=-1,view=0,
        viewpoint_diagnostics=[diagnostic()],frames=2,valid_projection=2,
        tf_failures=0,invalid_target_depth=0,task=('find','gym','sports ball'),
        rooms={'gym':dict(bounds=[-5.,-5.,5.,5.])},observations=[],
        get_clock=lambda:SimpleNamespace(now=lambda:now),
        param={'min_score':.35,'reject_clipped_detections':True,'validate_object_size':True,
               'optical_frame':'camera_optical_frame'}.__getitem__,
        bridge=Mock(),tf=Mock(),finish=Mock(),publish_progress=Mock(),
        landmarks=SimpleNamespace(confirmed=[SimpleNamespace(id=1)],observe=Mock(return_value=[1])))
    node.record_observation=MethodType(Search.record_observation,node)
    node.bridge.imgmsg_to_cv2.return_value=np.ones((480,640),dtype=np.float32)
    node.tf.lookup_transform.return_value=SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.,y=0.,z=0.),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    target=SimpleNamespace(class_name='sports ball',score=.8,
        bbox=SimpleNamespace(center=SimpleNamespace(position=SimpleNamespace(x=320.,y=240.)),size=SimpleNamespace(x=60.,y=60.)),
        mask=SimpleNamespace(width=640,height=480,data=[SimpleNamespace(x=300.,y=220.),SimpleNamespace(x=330.,y=220.),SimpleNamespace(x=320.,y=260.)]))
    detections=SimpleNamespace(header=SimpleNamespace(stamp=TimeMessage(sec=2)),detections=[target])
    depth=SimpleNamespace(width=640,height=480,encoding='32FC1')
    info=SimpleNamespace(width=640,height=480,k=[500.,0.,320.,0.,500.,240.,0.,0.,1.])
    with patch('semantic_search.search_node.instance_geometry',return_value=((1.,1.,.3),(.2,.2,.2))):
        Search.on_observation(node,detections,depth,info)
    node.finish.assert_not_called()
    assert len(node.observations)==1 and node.valid_projection==3
    assert not node.viewpoint_diagnostics[0]['dwell_completed']


def test_find_finishes_at_dwell_without_another_navigation_goal():
    now=SimpleNamespace(nanoseconds=4_900_000_000)
    node=SimpleNamespace(phase='observing',observe_start=1_000_000_000,
        observe_dwell_seconds=4.,observe_extra_seconds=4.,view=0,
        views=[[0.,0.,0.],[1.,0.,0.]],viewpoint_diagnostics=[diagnostic(),diagnostic(1,reached=False)],
        task=('find','gym','sports ball'),rooms={'gym':{}},
        landmarks=SimpleNamespace(confirmed=[object()]),finish=Mock(),nav=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:now))
    node.next_view=MethodType(Search.next_view,node)
    Search.tick(node);node.finish.assert_not_called()
    now.nanoseconds=5_000_000_000;Search.tick(node)
    node.finish.assert_called_once_with('found');node.nav.send_goal_async.assert_not_called()
    assert node.viewpoint_diagnostics[0]['dwell_completed']and node.view==1


def test_found_object_does_not_skip_unvisited_required_surface():
    views=[[0.,0.,0.],[1.,0.,0.]]
    node=SimpleNamespace(view=1,views=views,
        viewpoint_diagnostics=[diagnostic(dwell_completed=True),diagnostic(1,reached=False)],
        task=('find','kitchen','cup'),landmarks=SimpleNamespace(confirmed=[object()]),
        rooms={'kitchen':dict(inspection_views={'table':[views[0]],'worktop':[views[1]]})},
        finish=Mock(),nav=Mock(),publish_progress=Mock(),on_goal=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:TimeMessage())))
    Search.next_view(node)
    node.finish.assert_not_called();node.nav.send_goal_async.assert_called_once()
