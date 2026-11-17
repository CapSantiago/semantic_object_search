#!/usr/bin/env python3
"""Simulator placement-based camera calibration, explicitly not a navigation test."""
import argparse
import json
from math import sin, cos
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
VIEWS = [('kitchen_front', [6.6, -.4, 1.57079633]),
         ('kitchen_side', [5.0, .9, 0]),
         ('living_front', [1.2, -.5, -1.57079633]),
         ('living_side', [2.7, -1.8, 3.14159265]),
         ('bedroom_side', [-8.3, .9, 2.1]),
         ('bedroom_oblique', [-8.25, 1.15, 2.4])]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='results/development/r0_v1b/views')
    args = parser.parse_args()
    folder = ROOT/args.output; folder.mkdir(parents=True, exist_ok=True)
    records = []
    for name, (x, y, yaw) in VIEWS:
        request = (f'name: "waffle", position: {{x: {x}, y: {y}, z: 0.02}}, '
                   f'orientation: {{z: {sin(yaw/2)}, w: {cos(yaw/2)}}}')
        response = subprocess.run(['ign','service','-s','/world/aws_house_r0_v1/set_pose',
            '--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean',
            '--timeout','5000','--req',request], text=True, capture_output=True, check=True)
        if 'data: true' not in response.stdout:
            raise RuntimeError('Simulator placement rejected: '+response.stdout+response.stderr)
        destination = folder/name
        subprocess.run([sys.executable, str(ROOT/'scripts/capture_rgbd.py'), str(destination),
                        '--fixed-frame','odom','--skip-frames','4'], check=True)
        records.append(dict(name=name, commanded_world_pose=[x,y,.02,0,0,yaw],
                            frame_file=str((destination/'frame.json').relative_to(ROOT))))
        (folder/'views.json').write_text(json.dumps(dict(
            method='simulator_set_pose_for_visibility_calibration_not_autonomous_navigation',
            complete=len(records) == len(VIEWS),
            views=records), indent=2)+'\n')
    print('Captured calibration views:',len(records))


if __name__ == '__main__':
    main()
