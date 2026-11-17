#!/usr/bin/env python3
"""Measure collision meshes and define a candidate R0 layout; never edit assets."""
import hashlib
import argparse
import itertools
import json
from math import cos, sin
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def transform(pose):
    x, y, z, roll, pitch, yaw = pose
    cr, sr, cp, sp, cy, sy = cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = [x, y, z]
    return matrix


def pose(element):
    node = element.find('pose')
    if node is not None and node.get('relative_to'):
        raise ValueError('Relative SDF poses need explicit frame resolution')
    return [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]


def mesh_tool():
    source = ROOT/'tools/mesh_bounds.cpp'
    folder = ROOT/'downloads/layout_tools'; folder.mkdir(parents=True, exist_ok=True)
    binary = folder/'mesh_bounds'
    stamp = folder/'mesh_bounds.sha256'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if not binary.exists() or not stamp.exists() or stamp.read_text() != digest:
        flags = shlex.split(subprocess.check_output(
            ['pkg-config', '--cflags', '--libs', 'ignition-common4-graphics'], text=True))
        subprocess.run(['g++', str(source), '-O2', '-o', str(binary), *flags], check=True)
        stamp.write_text(digest)
    return binary


def collision_bounds(model_file, binary, world_pose=None):
    """SDF model pose intentionally supplied by layout, not upstream template pose."""
    model = ET.parse(model_file).getroot().find('model')
    points, sources = [], {}
    for link in model.findall('link'):
        for part in link.findall('collision'):
            mesh = part.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Expected mesh collision in '+str(model_file))
            uri = mesh.findtext('uri')
            if uri.startswith('model://'):
                relative = uri.removeprefix('model://').split('/', 1)[1]
            else:
                relative = uri
            path = (model_file.parent/relative).resolve()
            box = json.loads(subprocess.check_output([str(binary), str(path)], text=True))
            scale = np.array([float(v) for v in mesh.findtext('scale', '1 1 1').split()])
            matrix = transform(world_pose or [0]*6) @ transform(pose(link)) @ transform(pose(part))
            for corner in itertools.product(*zip(box[:3], box[3:])):
                points.append((matrix @ np.r_[np.array(corner)*scale, 1])[:3])
            sources[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    if not points:
        raise ValueError('No collision mesh in '+str(model_file))
    points = np.asarray(points)
    return dict(min=points.min(axis=0).tolist(), max=points.max(axis=0).tolist(),
                source_sha256=sources)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cup-asset',choices=['blue_mug','yellow_mug','plastic_cup'],default='blue_mug')
    parser.add_argument('--output',default='scenes/aws_house/layout_r0_v1.json')
    args=parser.parse_args()
    binary = mesh_tool()
    truth = json.loads((ROOT/'scenes/aws_house/ground_truth.json').read_text())['instances']
    selected = ['KitchenTable_01_001', 'CoffeeTable_01_001', 'ReadingDesk_01_001']
    surfaces = {}
    for name in selected:
        item = next(i for i in truth if i['name'] == name)
        path = ROOT/'third_party/aws_small_house/models'/item['asset']/'model.sdf'
        bounds = collision_bounds(path, binary, item['pose'])
        surfaces[name] = dict(bounds, top_z_m=bounds['max'][2],
                              method='world AABB of simulator collision mesh; flat top pending contact check')
    assets = {}
    for name in [args.cup_asset, 'mustard_bottle']:
        path = ROOT/'third_party/target_assets'/name/'model.sdf'
        bounds = collision_bounds(path, binary)
        assets[name] = dict(bounds, dimensions_m=(np.array(bounds['max'])-bounds['min']).tolist())
    placements = []
    for room, table, xy in [('kitchen', selected[0], [6.55, .95]),
                             ('living', selected[1], [1.20, -1.90]),
                             ('bedroom', selected[2], [-8.98, 2.06])]:
        for index, asset in enumerate(assets):
            support_z = surfaces[table]['top_z_m']
            offset = (index-.5)*.30
            placements.append(dict(id=f'{room}_{asset}', category='bottle' if asset=='mustard_bottle' else 'cup',
                asset=asset, room=room, support=table, support_z_m=support_z,
                pose=[xy[0]+(offset if room != 'bedroom' else 0),
                      xy[1]+(offset if room == 'bedroom' else 0),
                      support_z-assets[asset]['min'][2]+.001, 0,0,0]))
    report = dict(version='aws_house_r0_layout_v1', status='candidate_mesh_calibrated_not_contact_or_vision_validated',
                  purpose='scene_design_and_evaluation_only', frame_id='world', coordinate_unit='m',
                  measurement_backend='ignition-common4 MeshManager (Fortress system library)',
                  surfaces=surfaces, assets=assets, targets=placements,
                  limitations=['AABB top is a candidate flat support surface, not a physical contact test.',
                               'Texture appearance and YOLO detectability are unverified.',
                               'Old map is reference only; new layout needs sensor/navigation verification.'])
    output = ROOT/args.output
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(surfaces={k:v['top_z_m'] for k,v in surfaces.items()},
                         assets={k:v['dimensions_m'] for k,v in assets.items()}, targets=len(placements))))


if __name__ == '__main__':
    main()
