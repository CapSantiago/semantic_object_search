"""Bounded lifecycle startup with state verification after missed replies.

Only configures/activates during startup. Later deactivation stays stopped;
velocity_guard independently checks live lifecycle state and command freshness.
"""
import time


def main():
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState, GetState

    rclpy.init()
    node = rclpy.create_node('initialize_navigation_safety')
    state = node.create_client(GetState, '/collision_monitor/get_state')
    change = node.create_client(ChangeState, '/collision_monitor/change_state')
    deadline = time.monotonic()+40.
    attempts = {1: 0, 2: 0}

    def call(client, request):
        future = client.call_async(request)
        end = min(deadline,time.monotonic()+2.)
        while not future.done() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
        if future.done():
            return future.result()
        future.cancel()
        return None

    try:
        # Let response-reader discovery catch up with service discovery.
        warmup = time.monotonic()+2.
        while time.monotonic()<warmup:
            rclpy.spin_once(node,timeout_sec=.05)
        while time.monotonic()<deadline:
            if not state.service_is_ready() or not change.service_is_ready():
                rclpy.spin_once(node,timeout_sec=.1)
                continue
            observed = call(state,GetState.Request())
            if observed is None:
                continue
            current = observed.current_state.id
            if current==3:
                node.get_logger().info('CollisionMonitor active; verified after lifecycle transitions')
                return
            if current in attempts:
                attempts[current] += 1
                if attempts[current]>3:
                    raise RuntimeError('CollisionMonitor could not complete lifecycle startup')
                request = ChangeState.Request()
                request.transition.id = (Transition.TRANSITION_CONFIGURE if current==1
                                         else Transition.TRANSITION_ACTIVATE)
                response = call(change,request)
                node.get_logger().info(f'Lifecycle state {current}, transition {request.transition.id}, '
                                       f'acknowledged={response is not None and response.success}')
                # Always observe actual state next. A missing ACK is not a
                # reason to repeat a transition that has already completed.
            rclpy.spin_once(node,timeout_sec=.1)
        raise TimeoutError('CollisionMonitor startup exceeded 40 seconds')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
