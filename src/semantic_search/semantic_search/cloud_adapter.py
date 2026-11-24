"""Fortress RGB-D cloud remains X-forward, even when its header says optical.

Use camera_link (X forward/Y left/Z up) for this verified simulator only.
Do not apply this frame correction blindly to real RGB-D drivers.
"""
from copy import deepcopy
import json
import time
import numpy as np
import rclpy
from message_filters import Subscriber,TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField,CameraInfo,Image
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from .clouds import sample_cloud,clearing_cloud,_sample_records
from .cloud_projection import depth_xyz,projected_clouds,verify_native_projection
from .sensor_qos import current_sensor_stamp, sensor_qos_profile


class Adapter(Node):
    def __init__(self):
        super().__init__('fortress_depth_cloud_adapter')
        self.declare_parameter('input_topic','/rgbd/points_raw')
        self.declare_parameter('output_topic','/rgbd/obstacles')
        self.declare_parameter('frame_id','camera_link')
        self.declare_parameter('input_reliability','best_effort')
        self.declare_parameter('min_period',.25)
        self.declare_parameter('maximum_input_age',.25)
        self.declare_parameter('compact_xyz',False)
        self.declare_parameter('sampling_stride',4,ParameterDescriptor(read_only=True))
        self.declare_parameter('clearing_max_depth',0.,ParameterDescriptor(read_only=True))
        self.declare_parameter('clearing_output_topic','/rgbd/clearing',ParameterDescriptor(read_only=True))
        self.declare_parameter('input_source','native_cloud',ParameterDescriptor(read_only=True))
        self.declare_parameter('depth_topic','/rgbd/depth_image',ParameterDescriptor(read_only=True))
        self.input_source=self.get_parameter('input_source').value
        if self.input_source not in ('native_cloud','depth_image'):
            raise ValueError('Unsupported cloud input source')
        if self.input_source=='depth_image'and not self.get_parameter('compact_xyz').value:
            raise ValueError('Depth-image input requires compact XYZ output')
        self.calibration_verified=False
        self.calibration_pairs=0
        self.calibration_last_stamp=None
        self.calibration_signature=None
        self.calibration_metrics=None
        self.calibration_last_error=None
        self.calibration_filters=[]
        self.calibration_sync=None
        self.input_payload_bytes=0
        self.camera_info=None
        self.no_return_rays=0
        self.clearing_errors=0
        self.last = None
        self.received = self.published = self.stale = 0
        self.next_status = 0.
        self.status = self.create_publisher(String,'/rgbd/cloud_adapter_state',1)
        self.publisher = self.create_publisher(PointCloud2,self.get_parameter('output_topic').value,
                                               sensor_qos_profile(depth=1))
        self.clearing_publisher=self.create_publisher(PointCloud2,self.get_parameter('clearing_output_topic').value,sensor_qos_profile(depth=1))
        self.create_subscription(CameraInfo,'/rgbd/camera_info',lambda m:setattr(self,'camera_info',m),sensor_qos_profile('reliable',depth=1))
        if self.input_source=='native_cloud':
            self.create_subscription(PointCloud2,self.get_parameter('input_topic').value,self.receive,
                                     sensor_qos_profile(self.get_parameter('input_reliability').value,depth=1))
        else:
            self.create_subscription(Image,self.get_parameter('depth_topic').value,self.receive_depth,
                                     sensor_qos_profile('reliable',depth=1))
            self.calibration_filters=[Subscriber(self,kind,topic,qos_profile=sensor_qos_profile('reliable',depth=1))
                for kind,topic in ((PointCloud2,self.get_parameter('input_topic').value),
                    (Image,self.get_parameter('depth_topic').value),(CameraInfo,'/rgbd/camera_info'))]
            self.calibration_sync=TimeSynchronizer(self.calibration_filters,queue_size=4)
            self.calibration_sync.registerCallback(self.calibrate_depth)

    @staticmethod
    def depth_signature(msg,info):
        if (info is None or (msg.width,msg.height)!=(info.width,info.height)
                or not msg.header.frame_id or msg.header.frame_id!=info.header.frame_id
                or any(not np.isfinite(v)or abs(v)>1e-12 for v in info.d)):
            raise ValueError('Depth camera calibration, dimensions or frame differ')
        return (msg.width,msg.height,msg.encoding,msg.header.frame_id,tuple(info.k),tuple(info.d))

    def project_depth(self,msg,info):
        return depth_xyz(msg.data,msg.width,msg.height,msg.step,msg.is_bigendian,msg.encoding,
                         info.k,self.get_parameter('sampling_stride').value)

    def calibrate_depth(self,native,depth,info):
        if self.calibration_verified:return
        stamp=native.header.stamp.sec+native.header.stamp.nanosec/1e9
        now=self.get_clock().now().nanoseconds/1e9
        if not current_sensor_stamp(stamp,now,self.get_parameter('maximum_input_age').value):return
        try:
            if native.header.stamp!=depth.header.stamp or native.header.stamp!=info.header.stamp:
                raise ValueError('Native calibration requires exact source stamps')
            stamp_ns=native.header.stamp.sec*10**9+native.header.stamp.nanosec
            if self.calibration_last_stamp is not None and stamp_ns<=self.calibration_last_stamp:return
            signature=self.depth_signature(depth,info)
            if native.header.frame_id!=depth.header.frame_id or (native.width,native.height)!=(depth.width,depth.height):
                raise ValueError('Native calibration frame or dimensions differ')
            projected,_=self.project_depth(depth,info)
            _,sampled=_sample_records(native.data,native.width,native.height,native.point_step,
                native.row_step,native.fields,native.is_bigendian,self.get_parameter('sampling_stride').value)
            xyz=np.column_stack([sampled[name]for name in ('x','y','z')])
            metrics=verify_native_projection(xyz,projected)
            if signature!=self.calibration_signature:self.calibration_pairs=0
            self.calibration_signature=signature
            self.calibration_pairs+=1;self.calibration_metrics=metrics;self.calibration_last_stamp=stamp_ns
            self.calibration_last_error=None
            self.calibration_verified=self.calibration_pairs>=3
        except ValueError as error:
            self.calibration_pairs=0;self.calibration_last_stamp=None;self.calibration_last_error=str(error)

    def release_calibration_streams(self):
        # Run outside TimeSynchronizer's callback/lock. The native bridge is
        # lazy in this mode, so removing its last subscriber stops bulk data.
        for subscriber in self.calibration_filters:self.destroy_subscription(subscriber.sub)
        self.calibration_filters=[]
        if self.calibration_sync is not None:
            for queue in self.calibration_sync.queues:queue.clear()
            self.calibration_sync=None

    def xyz_message(self,header,data):
        output=PointCloud2();output.header=deepcopy(header)
        output.header.frame_id=self.get_parameter('frame_id').value
        output.height=1;output.width=len(data)//12
        output.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                       for i,name in enumerate(('x','y','z'))]
        output.point_step=12;output.row_step=len(data);output.is_bigendian=False
        output.is_dense=True;output.data=data
        return output

    def receive_depth(self,msg):
        started=time.monotonic();self.received+=1;self.input_payload_bytes=len(msg.data)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        now=self.get_clock().now().nanoseconds/1e9
        if not current_sensor_stamp(stamp,now,self.get_parameter('maximum_input_age').value):
            self.stale+=1;self.report(stamp,started,processed=False);return
        if not self.calibration_verified:
            self.report(stamp,started,processed=False);return
        self.release_calibration_streams()
        if self.last is not None and 0<=stamp-self.last<self.get_parameter('min_period').value:return
        try:
            info=self.camera_info
            if self.depth_signature(msg,info)!=self.calibration_signature:
                raise ValueError('Depth calibration changed after native verification')
            info_stamp=info.header.stamp.sec+info.header.stamp.nanosec/1e9
            if not current_sensor_stamp(info_stamp,now,.5):raise ValueError('Depth camera info stale')
            xyz,rays=self.project_depth(msg,info)
            marked,cleared,count=projected_clouds(xyz,rays,self.get_parameter('clearing_max_depth').value)
        except ValueError as error:
            self.calibration_last_error=str(error);self.clearing_errors+=1
            self.report(stamp,started,processed=False);return
        self.calibration_last_error=None
        self.last=stamp;self.publisher.publish(self.xyz_message(msg.header,marked))
        if self.get_parameter('clearing_max_depth').value>0 and self.clearing_publisher.get_subscription_count()>0:
            self.clearing_publisher.publish(self.xyz_message(msg.header,cleared))
        self.no_return_rays=count;self.published+=1;self.report(stamp,started,processed=True)

    def receive(self, msg):
        started = time.monotonic()
        self.received += 1
        self.input_payload_bytes=len(msg.data)
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        now = self.get_clock().now().nanoseconds/1e9
        if not current_sensor_stamp(stamp,now,self.get_parameter('maximum_input_age').value):
            self.stale += 1
            self.report(stamp,started,processed=False)
            return
        if self.last is not None and 0 <= stamp-self.last < self.get_parameter('min_period').value:
            return
        try:
            data = sample_cloud(msg.data,msg.width,msg.height,msg.point_step,msg.row_step,
                                msg.fields,msg.is_bigendian,
                                stride=self.get_parameter('sampling_stride').value,
                                compact=self.get_parameter('compact_xyz').value)
        except ValueError as error:
            self.get_logger().error(str(error)); return
        self.last = stamp
        output = PointCloud2()
        output.header = deepcopy(msg.header)
        output.header.frame_id = self.get_parameter('frame_id').value
        output.height=1
        if self.get_parameter('compact_xyz').value:
            output.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                           for i,name in enumerate(('x','y','z'))]
            output.point_step=12;output.is_bigendian=False
        else:
            output.fields=msg.fields;output.is_bigendian=msg.is_bigendian
            output.point_step=msg.point_step
        output.width=len(data)//output.point_step;output.row_step=len(data)
        output.data=data;output.is_dense=True
        self.publisher.publish(output)
        if self.get_parameter('clearing_max_depth').value>0 and self.clearing_publisher.get_subscription_count()>0:
            try:
                info=self.camera_info
                if info is None or info.width!=msg.width or info.height!=msg.height:
                    raise ValueError('Clearing camera info unavailable or dimensions differ')
                info_stamp=info.header.stamp.sec+info.header.stamp.nanosec/1e9
                if not current_sensor_stamp(info_stamp,now,.5)or any(abs(v)>1e-12 for v in info.d):
                    raise ValueError('Clearing camera info stale or distorted')
                clearing,count=clearing_cloud(msg.data,msg.width,msg.height,msg.point_step,msg.row_step,msg.fields,
                    msg.is_bigendian,info.k,self.get_parameter('clearing_max_depth').value,
                    self.get_parameter('sampling_stride').value)
                free=PointCloud2();free.header=deepcopy(output.header);free.height=1;free.width=len(clearing)//12
                free.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)for i,name in enumerate(('x','y','z'))]
                free.point_step=12;free.row_step=len(clearing);free.is_bigendian=False;free.is_dense=True;free.data=clearing
                self.clearing_publisher.publish(free);self.no_return_rays=count
            except ValueError:
                self.clearing_errors+=1  # No unverified rays are published.
        self.published += 1
        self.report(stamp,started,processed=True)

    def report(self, stamp, started, *, processed):
        wall = time.monotonic()
        if wall<self.next_status:
            return
        self.next_status = wall+1.
        self.status.publish(String(data=json.dumps(dict(received=self.received,published=self.published,
            input_source=self.input_source,input_payload_bytes=self.input_payload_bytes,
            native_calibration_required=self.input_source=='depth_image',
            native_calibration_verified=self.calibration_verified,
            native_calibration_pairs=self.calibration_pairs,native_calibration_metrics=self.calibration_metrics,
            native_calibration_last_error=self.calibration_last_error,
            dropped_stale=self.stale,processed=processed,stamp_seconds=stamp,
            clearing_no_return_rays=self.no_return_rays,clearing_rejected_frames=self.clearing_errors,
            source_age_seconds=self.get_clock().now().nanoseconds/1e9-stamp,
            processing_wall_seconds=wall-started))))


def main():
    rclpy.init();node=Adapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
