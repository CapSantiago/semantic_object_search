"""Startup-only reliability selection for image and synchronized sensor inputs."""
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


def sensor_qos_profile(reliability='best_effort', *, depth=None):
    if reliability not in ('best_effort', 'reliable'):
        raise ValueError('sensor_reliability must be "best_effort" or "reliable"')
    # Construct a new profile with every sensor-data policy preserved; never
    # mutate the process-wide preset used by other ROS subscriptions.
    profile = QoSProfile(**qos_profile_sensor_data.get_c_qos_profile().to_dict())
    profile.reliability = (ReliabilityPolicy.RELIABLE if reliability == 'reliable'
                           else ReliabilityPolicy.BEST_EFFORT)
    if depth is not None:
        if type(depth) is not int or depth<=0:
            raise ValueError('Sensor queue depth must be a positive integer')
        profile.depth = depth
    return profile


def declare_sensor_qos(node, *, depth=None):
    parameter = node.declare_parameter(
        'sensor_reliability', 'best_effort', descriptor=ParameterDescriptor(
            read_only=True,
            description='Sensor input reliability: best_effort or reliable; restart to change.'))
    return sensor_qos_profile(parameter.value,depth=depth)


def current_sensor_stamp(stamp, now, maximum_age=.25):
    """Reject old/future work without changing the sensor's original stamp."""
    import math
    return (all(math.isfinite(v) for v in (stamp,now,maximum_age)) and stamp>0 and
            maximum_age>0 and -.1<=now-stamp<=maximum_age)
