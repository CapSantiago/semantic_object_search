"""Isolated ROS action/odometry protocol tests; no physical robot is driven.

The fixture deliberately returns a parent action result before its simulated
child velocity stops. It tests the production client, not Nav2 or Gazebo motion.
"""
import os
import threading
import time
from types import SimpleNamespace
import uuid

import pytest


@pytest.mark.skipif(os.environ.get('SEMANTIC_RUN_ROS_CLIENT_TESTS') != '1',
                    reason='requires explicit isolated ROS protocol run')
@pytest.mark.parametrize('ending', ['abort', 'timeout', 'success','shifted_success'])
@pytest.mark.parametrize('odometry', ['delayed_stop', 'moving', 'stale'])
@pytest.mark.parametrize('action_kind', ['navigate', 'follow'])
def test_terminal_result_precedes_stopped_odometry(ending, odometry, action_kind):
    assert os.environ.get('ROS_DOMAIN_ID') == '69'
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from nav2_msgs.action import NavigateToPose, FollowPath
    from nav_msgs.msg import Odometry
    from run_cleaning import CleaningRun

    context = rclpy.Context()
    rclpy.init(context=context)
    suffix = uuid.uuid4().hex[:10]
    server_node = rclpy.create_node('clean_stop_server_'+suffix, context=context)
    client_node = rclpy.create_node('clean_stop_client_'+suffix, context=context)
    client_executor = SingleThreadedExecutor(context=context)
    client_executor.add_node(client_node)
    prefix = '/clean_stop_test_'+suffix
    action_type=FollowPath if action_kind=='follow' else NavigateToPose
    state = dict(terminal_wall=None, goals=0)
    worker = CleaningRun.__new__(CleaningRun)
    worker.node = client_node
    worker.stopped, worker.goals = [], []
    worker.started = worker.last_pose_wall = time.monotonic()
    worker.stage, worker.settled = 'sweeping', True
    worker.args = SimpleNamespace(position_goals=False)
    worker.odom_wall = worker.odom_stamp = worker.stationary_since = None
    # The protocol fixture has a fixed pose; localization is not under test.
    worker.pose = lambda: (1., 2., 0.)
    worker.path_for_lane=lambda target:[(1.,2.,0.),target]
    target=(4.,2.,0.)
    if ending=='brush':
        import numpy as np
        from semantic_search.cleaning import RoomSweep
        target=(1.2,2.,0.)
        worker.lane_start=(1.,2.,0.)
        worker.sweep=RoomSweep(np.zeros(120*80),120,80,.05,(0.,0.,0.),
            {'room':dict(bounds=[0.,0.,6.,4.])},'room',(1.,2.,0.))
        worker.sweep.covered[:]=worker.sweep.eligible
        worker.costmap_ready=lambda:True
        worker.available=lambda:np.ones_like(worker.sweep.centers)
        worker.tf=SimpleNamespace(lookup_transform=lambda *args:SimpleNamespace(
            header=SimpleNamespace(stamp=client_node.get_clock().now().to_msg())))

    def tick():
        client_executor.spin_once(timeout_sec=.02)
        worker.last_pose_wall = time.monotonic()
    worker.tick = tick
    client_node.create_subscription(Odometry, prefix+'/odom', worker.receive_odom, 10)
    publisher = server_node.create_publisher(Odometry, prefix+'/odom', 10)
    def publish():
        terminal = state['terminal_wall']
        if terminal is not None and odometry == 'stale':
            return
        message = Odometry()
        message.header.stamp = server_node.get_clock().now().to_msg()
        message.twist.twist.linear.x = (0. if terminal is None and ending=='brush' else 0. if terminal is not None and
            odometry == 'delayed_stop' and time.monotonic()-terminal >= .30 else .1)
        if terminal is None and ending=='brush':message.twist.twist.angular.z=.3
        publisher.publish(message)
    server_node.create_timer(.02, publish)
    def execute(goal):
        state['goals'] += 1
        state['request']=goal.request
        if ending in ('timeout','brush'):
            deadline = time.monotonic()+5.
            while not goal.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(.01)
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
        elif ending in ('success','shifted_success'):
            goal.succeed()
        else:
            goal.abort()
        state['terminal_wall'] = time.monotonic()
        return action_type.Result()
    server = ActionServer(server_node, action_type, prefix+'/navigate', execute_callback=execute,
        cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
    worker.client = ActionClient(client_node, action_type, prefix+'/navigate')
    worker.path_client=worker.client
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(server_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert worker.client.wait_for_server(timeout_sec=5.)
        worker.wait_for(lambda: worker.odom_wall is not None, 5.)
        if ending=='brush':worker.wait_for(lambda:len(worker.odom_trace)>25,5.)
        if odometry == 'delayed_stop':
            assert worker.go(target, time.monotonic()+8., timeout=.25,
                             lane=action_kind=='follow',purpose='lane_start' if ending=='shifted_success' else None) == (
                                 ending in ('success','brush') or ending=='shifted_success' and action_kind=='follow')
            assert worker.settled and worker.goals[-1]['stop_confirmed']
            assert time.monotonic()-state['terminal_wall'] >= .70
            assert worker.parked()
            expected={'timeout':'timed_out','abort':'failed','success':'succeeded','brush':'brush_covered',
                      'shifted_success':'adjusted_goal_unreached' if action_kind=='navigate' else 'succeeded'}[ending]
            assert worker.goals[-1]['status'] == expected
            if ending=='shifted_success' and action_kind=='navigate':
                assert worker.goals[-1]['action_status']==4
                assert worker.goals[-1]['requested_position_check']==dict(error_m=3.,tolerance_m=.12,passed=False)
        else:
            with pytest.raises(TimeoutError):
                worker.go(target, time.monotonic()+8., timeout=.25,lane=action_kind=='follow')
            assert not worker.settled
            assert not worker.goals[-1]['stop_confirmed']
            with pytest.raises(RuntimeError):
                worker.go((1., 2., 0.), time.monotonic()+8., recovery=True)
            assert state['goals'] == 1
        if action_kind=='follow':
            request=state['request']
            assert request.controller_id=='CleaningPath'
            assert request.goal_checker_id=='cleaning_goal_checker'
            assert request.path.header.frame_id=='map'
            assert request.path.poses[-1].pose.position.x==target[0]
    finally:
        executor.shutdown(timeout_sec=6.)
        thread.join(timeout=6.)
        client_executor.shutdown()
        worker.client.destroy()
        server.destroy()
        client_node.destroy_node()
        server_node.destroy_node()
        context.shutdown()


@pytest.mark.skipif(os.environ.get('SEMANTIC_RUN_ROS_CLIENT_TESTS') != '1',
                    reason='requires explicit isolated ROS protocol run')
@pytest.mark.parametrize('odometry',['delayed_stop','moving','stale'])
def test_brush_cancel_preserves_real_terminal_and_stop_barrier(odometry):
    test_terminal_result_precedes_stopped_odometry('brush',odometry,'follow')
