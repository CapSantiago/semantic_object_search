"""Read-only, depth-one cloud/scan continuity trace (no large raw-cloud subscription)."""
import argparse,json,time,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,LaserScan
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=45);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
rclpy.init();n=Node('cloud_continuity_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
stopped=[]
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *unused:stopped.append(True))
rows={};statuses=[];start=time.monotonic();latest={};checks=[]
for key,topic,kind in [('cloud','/rgbd/obstacles',PointCloud2),('scan','/scan',LaserScan)]:
 rows[key]=[]
 def cb(m,k=key):
  r=dict(wall=time.monotonic()-start,stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,sim=n.get_clock().now().nanoseconds/1e9);rows[k].append(r);latest[k]=r
 n.create_subscription(kind,topic,cb,QoSProfile(depth=1,reliability=qos_profile_sensor_data.reliability))
n.create_subscription(String,'/rgbd/cloud_adapter_state',lambda m:statuses.append(json.loads(m.data)),1)
while time.monotonic()-start<a.seconds and not stopped:
 rclpy.spin_once(n,timeout_sec=.01)
 now=time.monotonic()-start;sim=n.get_clock().now().nanoseconds/1e9
 if now>=3:checks.append(dict(wall=now,ready=all(k in latest and now-latest[k]['wall']<=.4 and -.1<=sim-latest[k]['stamp']<=.4 for k in ('scan','cloud'))))
summary={}
for k,rs in rows.items():
 measured=[r for r in rs if r['wall']>=3];gaps=[b['wall']-x['wall']for x,b in zip(measured,measured[1:])];ages=[r['sim']-r['stamp']for r in measured]
 summary[k]=dict(count=len(measured),max_gap=max(gaps,default=None),gaps_over_point4=sum(x>.4 for x in gaps),max_source_age=max(ages,default=None))
summary['fresh_checks']=len(checks);summary['stale_checks']=sum(not x['ready']for x in checks)
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(dict(summary=summary,rows=rows,statuses=statuses,checks=checks),indent=2));print(json.dumps(summary))
n.destroy_node();rclpy.shutdown()
