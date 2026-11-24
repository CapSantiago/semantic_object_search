"""Final velocity lease after Nav2 CollisionMonitor; stale data always stops.

Humble 1.1.20 CollisionMonitor discards expired sources and can pass velocity
with no valid source. This adapter supplies the missing freshness watchdog.
Obstacle geometry and slowdown remain in the upstream component.
"""
import math


class VelocityLease:
    sources = ('scan','depth','cloud','odom')

    def __init__(self, sensor_timeout=.4, command_timeout=.3):
        self.sensor_timeout,self.command_timeout = sensor_timeout,command_timeout
        self.inputs = {}
        self.command = None
        self.clock = self.advanced = None

    def source(self, name, stamp, wall):
        if name not in self.sources or not math.isfinite(stamp) or stamp<=0:
            return False
        if name in self.inputs and stamp<=self.inputs[name][0]:
            return False
        self.inputs[name] = (stamp,wall)
        return True

    def velocity(self, value, wall):
        self.command = None
        if len(value)==3 and all(math.isfinite(x) for x in value) and max(abs(x) for x in value[:2])<=.5 and abs(value[2])<=2.:
            self.command = (tuple(value),wall)

    def evaluate(self, sim, wall, *, transform_ready, filter_active):
        if self.clock is not None and sim<self.clock:
            self.inputs.clear()
            self.command = None
            self.advanced = None
        if self.clock is None or sim>self.clock:
            self.advanced = wall
        self.clock = sim
        reasons = []
        if sim<=0 or self.advanced is None or wall-self.advanced>self.sensor_timeout:
            reasons.append('clock_stale')
        for name in self.sources:
            value = self.inputs.get(name)
            if value is None or wall-value[1]>self.sensor_timeout or not -.10<=sim-value[0]<=self.sensor_timeout:
                reasons.append(name+'_stale')
        if not transform_ready:
            reasons.append('odometry_tf_stale')
        if not filter_active:
            reasons.append('collision_monitor_inactive')
        ready = not reasons
        if self.command is None or wall-self.command[1]>self.command_timeout:
            reasons.append('command_stale')
        return (self.command[0] if not reasons else (0.,0.,0.)),ready,reasons

    def ages(self, sim, wall):
        return {name:dict(sim_seconds=sim-value[0],wall_seconds=wall-value[1])
                for name,value in self.inputs.items()}


def main():
    import json
    import signal
    import time
    import numpy as np
    import rclpy
    from rclpy.clock import Clock,ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan,Image,PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer,TransformListener,TransformException
    from .sensor_qos import sensor_qos_profile

    class Guard(Node):
        def __init__(self):
            super().__init__('velocity_guard')
            self.lease = VelocityLease()
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer,self)
            self.pub = self.create_publisher(Twist,'/cmd_vel_safe',1)
            self.status = self.create_publisher(String,'/safety/state',10)
            self.client = self.create_client(GetState,'/collision_monitor/get_state')
            self.pending = None
            self.pending_since = None
            self.filter_checked = self.next_check = self.next_state = 0.
            self.filter_active = False
            self.last_reasons = None
            self.output_count = self.blocked_count = 0
            self.create_subscription(Twist,'/cmd_vel_collision_filtered',self.receive_velocity,1)
            for name,kind,topic in (('scan',LaserScan,'/scan'),('depth',Image,'/rgbd/depth_image'),
                                   ('cloud',PointCloud2,'/rgbd/obstacles'),('odom',Odometry,'/odom')):
                qos = sensor_qos_profile('reliable' if name=='depth' else 'best_effort',depth=1)
                self.create_subscription(kind,topic,lambda m,n=name:self.receive(n,m),qos)
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(.05,self.step,clock=self.wall_clock)

        def receive_velocity(self, message):
            self.lease.velocity((message.linear.x,message.linear.y,message.angular.z),time.monotonic())

        def receive(self, name, message):
            stamp = Time.from_msg(message.header.stamp)
            try:
                if name=='scan':
                    values = np.asarray(message.ranges)
                    valid = np.isposinf(values) | (np.isfinite(values)&(values>=message.range_min)&(values<=message.range_max))
                    if (len(values)<8 or not 0<message.range_min<message.range_max
                        or not math.isfinite(message.angle_increment) or message.angle_increment<=0
                        or valid.mean()<.75):
                        return
                elif name=='depth':
                    size = {'32FC1':4,'16UC1':2}.get(message.encoding)
                    if (size is None or message.width<=0 or message.height<=0
                        or message.step<message.width*size or len(message.data)<message.step*message.height):
                        return
                    dtype = ('>' if message.is_bigendian else '<')+('f4' if size==4 else 'u2')
                    depth = np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                                       strides=(message.step,size))[::16,::16]
                    if np.mean(np.isfinite(depth)&(depth>0))<.02:
                        return
                elif name=='cloud' and message.width>0:
                    if (message.height<=0 or message.point_step<12 or message.row_step<message.width*message.point_step
                        or len(message.data)<message.height*message.row_step):
                        return
                if not message.header.frame_id:
                    return
                if name in ('scan','depth','cloud'):
                    self.buffer.lookup_transform('base_link',message.header.frame_id,stamp)
                self.lease.source(name,stamp.nanoseconds/1e9,time.monotonic())
            except (ValueError,TransformException):
                return

        def stop(self):
            self.pub.publish(Twist())

        def step(self):
            wall,sim = time.monotonic(),self.get_clock().now().nanoseconds/1e9
            if self.pending is not None and self.pending.done():
                try:
                    self.filter_active = self.pending.result().current_state.id==3
                    self.filter_checked = wall
                except Exception:
                    self.filter_active = False
                self.pending = None
            elif self.pending is not None and wall-self.pending_since>2.:
                self.pending.cancel()
                self.pending = None
                self.filter_active = False
            if wall>=self.next_check and self.pending is None and self.client.service_is_ready():
                self.pending = self.client.call_async(GetState.Request())
                self.pending_since = wall
                self.next_check = wall+1.
            tf_ready = False
            try:
                tf = self.buffer.lookup_transform('odom','base_link',Time())
                age = sim-Time.from_msg(tf.header.stamp).nanoseconds/1e9
                tf_ready = -.1<=age<=.4
            except TransformException:
                pass
            value,ready,reasons = self.lease.evaluate(sim,wall,transform_ready=tf_ready,
                filter_active=self.filter_active and wall-self.filter_checked<3.)
            message = Twist()
            message.linear.x,message.linear.y,message.angular.z = value
            self.pub.publish(message)
            self.output_count += 1
            requested = self.lease.command is not None and any(abs(x)>1e-6 for x in self.lease.command[0])
            self.blocked_count += int(requested and bool(reasons))
            if wall>=self.next_state or reasons!=self.last_reasons:
                self.next_state = wall+.25
                self.last_reasons = list(reasons)
                self.status.publish(String(data=json.dumps(dict(ready=ready,reasons=reasons,
                    output=list(value),filter_active=self.filter_active,stamp_seconds=sim,
                    source_ages=self.lease.ages(sim,wall),output_samples=self.output_count,
                    blocked_nonzero_command_samples=self.blocked_count))))

    stopped = []
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *args:stopped.append(True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Guard()
    try:
        while not stopped:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.stop()
        rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
