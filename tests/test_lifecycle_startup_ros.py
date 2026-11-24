"""Real ROS protocol regression for a lifecycle transition whose ACK is late.

Use isolated domain 69 and SEMANTIC_RUN_ROS_CLIENT_TESTS=1. The service fixture
is a state machine, not a collision monitor or a physical avoidance test.
"""
import os
import subprocess
import sys
import threading
import time

import pytest


@pytest.mark.skipif(os.environ.get('SEMANTIC_RUN_ROS_CLIENT_TESTS')!='1',
                    reason='requires explicit isolated ROS protocol run')
def test_startup_checks_actual_state_after_a_late_transition_ack():
    assert os.environ.get('ROS_DOMAIN_ID')=='69'
    import rclpy
    from lifecycle_msgs.srv import GetState,ChangeState

    rclpy.init()
    node = rclpy.create_node('late_lifecycle_ack_fixture')
    observed = {'state':1,'transitions':[]}
    stopped = threading.Event()
    def get(request,response):
        response.current_state.id = observed['state']
        return response
    def change(request,response):
        transition = request.transition.id
        observed['transitions'].append(transition)
        if (observed['state'],transition)==(1,1):
            observed['state'] = 2
            time.sleep(2.6)  # Startup's request expires after 2 seconds.
            response.success = True
        elif (observed['state'],transition)==(2,3):
            observed['state'] = 3
            response.success = True
        return response
    node.create_service(GetState,'/collision_monitor/get_state',get)
    node.create_service(ChangeState,'/collision_monitor/change_state',change)
    def spin():
        while not stopped.is_set():
            rclpy.spin_once(node,timeout_sec=.05)
    worker = threading.Thread(target=spin)
    worker.start()
    try:
        process = subprocess.run([sys.executable,'-m','semantic_search.lifecycle_startup'],
                                 timeout=20,capture_output=True,text=True)
        assert process.returncode==0,process.stdout+process.stderr
        assert observed=={'state':3,'transitions':[1,3]}
        assert 'acknowledged=False' in process.stdout+process.stderr
    finally:
        stopped.set();worker.join(timeout=4)
        node.destroy_node();rclpy.try_shutdown()
