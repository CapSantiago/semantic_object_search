"""Fortress RGB-D cloud remains X-forward, even when its header says optical.

Use camera_link (X forward/Y left/Z up) for this verified simulator only.
Do not apply this frame correction blindly to real RGB-D drivers.
"""
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from .clouds import sample_cloud


class Adapter(Node):
    def __init__(self):
        super().__init__('fortress_depth_cloud_adapter')
        self.declare_parameter('input_topic','/rgbd/points_raw')
        self.declare_parameter('output_topic','/rgbd/obstacles')
        self.declare_parameter('frame_id','camera_link')
        self.last = None
        self.publisher = self.create_publisher(PointCloud2,self.get_parameter('output_topic').value,
                                               qos_profile_sensor_data)
        self.create_subscription(PointCloud2,self.get_parameter('input_topic').value,self.receive,
                                 qos_profile_sensor_data)

    def receive(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if self.last is not None and 0 <= stamp-self.last < .25:
            return
        try:
            data = sample_cloud(msg.data,msg.width,msg.height,msg.point_step,msg.row_step,
                                msg.fields,msg.is_bigendian)
        except ValueError as error:
            self.get_logger().error(str(error)); return
        self.last = stamp
        output = PointCloud2()
        output.header = deepcopy(msg.header)
        output.header.frame_id = self.get_parameter('frame_id').value
        output.height=1;output.width=len(data)//msg.point_step
        output.fields=msg.fields;output.is_bigendian=msg.is_bigendian
        output.point_step=msg.point_step;output.row_step=len(data)
        output.data=data;output.is_dense=True
        self.publisher.publish(output)


def main():
    rclpy.init();node=Adapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
