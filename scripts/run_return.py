#!/usr/bin/env python3
"""Return a completed service task to its configured charging facility."""
import argparse
import math
from pathlib import Path
import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions

from navigation_client import NavigationRun, StopRequested, trace_preview
from robot_console import ROOT, write_json
from semantic_search.docking import dock_at, load_station


class ReturnRun(NavigationRun):
    def __init__(self,args,stopped):
        super().__init__(args,stopped)
        self.station=load_station(args.station_config,ROOT)
        self.target_pose=tuple(self.station['map_pose'])

    def run(self):
        # The search result can precede the last child controller stopping.
        # Local action-server discovery can precede matching its response
        # reader. Keep spinning before the first goal, as the search client does.
        discovery_ready_at=self.started+2.
        self.settled=False
        self.wait_for(lambda:time.monotonic()>=discovery_ready_at and self.pose()
                      and self.parked() and self.client.server_is_ready(),25.)
        self.settled=True
        self.startup_ready_after_seconds=time.monotonic()-self.started
        self.start_pose=self.pose()
        charging=dock_at(self,self.target_pose,self.deadline)
        return ('completed','已返回充电处并进入模拟充电。') if charging else ('partial','返航已停止，停靠未达标。')

    def progress(self):
        result=super().progress()
        result.update(request_id=self.args.output.name,charging_station_id=self.station['id'],
                      target_pose=list(self.target_pose))
        return result

    def report(self,status,message):
        result=dict(status=status,message=message,charging_station_id=self.station['id'],frame_id='map',
            initial_pose=self.start_pose,return_target_pose=self.target_pose,final_pose=self.final_pose,
            return_state=self.return_state,return_error=self.return_errors,navigation_settled=self.settled,
            elapsed_seconds=round(time.monotonic()-self.started,2),distance_m=round(self.distance,3),
            discovery_warmup_seconds=2.,
            startup_ready_after_seconds=getattr(self,'startup_ready_after_seconds',None),
            odom_distance_m=round(self.odom_distance,3),trace_preview=trace_preview(self.trace))
        write_json(self.args.output/'return_result.json',result)
        write_json(self.args.output/'return_trace.json',dict(frame_id='map',samples=self.trace,goals=self.goals,
                                                           odom_samples=self.odom_trace))
        print(message,flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('station_config',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--timeout',type=float,default=240.)
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout<=0:parser.error('timeout must be finite and positive')
    stopped=[]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda sig,frame:stopped.append(sig))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    task=None;status='failed';message='返航未启动。'
    try:
        task=ReturnRun(args,stopped);status,message=task.run()
    except StopRequested:
        status,message='canceled','返航已停止。'
        if task:
            task.return_state='return_canceled'
            if not task.settled and not task.goals:
                try:
                    task.wait_for(lambda:task.parked() and task.pose(),2.,check_stop=False)
                    task.settled=True
                except TimeoutError:pass
    except Exception as error:
        message=str(error)
        if task:task.return_state='return_incomplete'
    finally:
        if task:
            try:task.report(status,message)
            finally:task.node.destroy_node()
        rclpy.try_shutdown()
    return 0 if status=='completed' else 130 if status=='canceled' else 1


if __name__=='__main__':raise SystemExit(main())
