#!/usr/bin/env python3
"""Send one request to the mapping gate and wait for its terminal receipt."""
import argparse
import json
from pathlib import Path
import signal
import time
from uuid import uuid4

import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command')
    parser.add_argument('request_id')
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    stopped = []
    result = []
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda signum,frame:stopped.append(signum))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mapping_command_client')
    pub = node.create_publisher(String,'/mapping/command',10)

    def receive(message):
        row = json.loads(message.data)
        if row.get('request_id') == args.request_id:
            result.append(row)

    def send(**command):
        pub.publish(String(data=json.dumps(command)))

    def wait(seconds, predicate):
        deadline = time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        return predicate()

    node.create_subscription(String,'/mapping/result',receive,10)
    try:
        ready = wait(8,lambda:stopped or (
            any(p.node_name=='mapping_motion' for p in node.get_publishers_info_by_topic('/mapping/result'))
            and pub.get_subscription_count()))
        if stopped or not ready:
            row = dict(status='canceled' if stopped else 'rejected',
                       message='指令未发送。' if stopped else '运动保护节点尚未就绪。')
        else:
            send(mode=args.command,id=args.request_id,source='console')
            wait(59,lambda:result or stopped)
            if not result:
                send(mode='stop',id=uuid4().hex,source='console',target=args.request_id)
                wait(4,lambda:result)
            row = result[0] if result else dict(status='unconfirmed',
                message='未收到运动结束回执。请停止本次运行。')
        args.output.write_text(json.dumps(row,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(row,ensure_ascii=False),flush=True)
        return 0 if row['status']=='completed' else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    raise SystemExit(main())
