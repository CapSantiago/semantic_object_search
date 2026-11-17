"""Startup-only reliability selection for image and synchronized sensor inputs."""
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


def sensor_qos_profile(reliability='best_effort'):
    if reliability not in ('best_effort', 'reliable'):
        raise ValueError('sensor_reliability must be "best_effort" or "reliable"')
    # Construct a new profile with every sensor-data policy preserved; never
    # mutate the process-wide preset used by other ROS subscriptions.
    profile = QoSProfile(**qos_profile_sensor_data.get_c_qos_profile().to_dict())
    profile.reliability = (ReliabilityPolicy.RELIABLE if reliability == 'reliable'
                           else ReliabilityPolicy.BEST_EFFORT)
    return profile


def declare_sensor_qos(node):
    parameter = node.declare_parameter(
        'sensor_reliability', 'best_effort', descriptor=ParameterDescriptor(
            read_only=True,
            description='Sensor input reliability: best_effort or reliable; restart to change.'))
    return sensor_qos_profile(parameter.value)
