#!/usr/bin/env python3
"""Check room ownership and grid connectivity on a fixed saved map, without ROS."""
import argparse
import hashlib
import json
from math import cos, floor, sin
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/semantic_search'))
from semantic_search.rooms import contains, room_at, validate_rooms
from semantic_search.map_resources import resolve_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', default='scenes/aws_house/spatial_v1.json')
    parser.add_argument('--output', default='results/development/p1/spatial_validation.json')
    parser.add_argument('--plot', default='results/development/p1/spatial_plan.png')
    args = parser.parse_args()
    plan = json.loads((ROOT / args.plan).read_text())
    rooms = json.loads((ROOT / plan['rooms_file']).read_text())
    validate_rooms(rooms)
    resource = resolve_map(ROOT, yaml_path=plan['map_file'])
    metadata = resource['metadata']
    image = np.asarray(Image.open(resource['image']))
    probability = (255-image.astype(float))/255
    if metadata.get('negate', 0):
        probability = 1-probability
    free = probability < metadata['free_thresh']
    resolution = metadata['resolution']
    # Pad with obstacles so distance cannot extend beyond the known map border.
    clearance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]*resolution
    radius = .22
    safe = free & (clearance >= radius + resolution/2)
    components, _ = label(safe)  # four-connected, no diagonal obstacle cutting
    ox, oy, yaw = metadata['origin']

    def cell(pose):
        dx, dy = pose[0]-ox, pose[1]-oy
        col = floor((cos(yaw)*dx+sin(yaw)*dy)/resolution)
        row = free.shape[0]-1-floor((-sin(yaw)*dx+cos(yaw)*dy)/resolution)
        if 0 <= row < safe.shape[0] and 0 <= col < safe.shape[1]:
            return row, col
        return None

    home = cell(plan['home_pose'])
    home_component = int(components[home]) if home else 0
    checks = {'map_version': all(hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == v
                                 for p, v in plan['map_sha256'].items()),
              'home_clearance': home_component > 0}
    portals = []
    for connection in plan['connections']:
        ends = []
        for room, pose in zip(connection['rooms'], connection['poses']):
            c = cell(pose)
            ends.append(dict(room=room, pose=pose, owner=room_at(rooms, *pose[:2]),
                             clearance_m=float(clearance[c]) if c else 0,
                             connected_to_home=bool(c and home_component > 0 and
                                                    components[c] == home_component)))
        valid = all(e['owner'] == e['room'] and e['connected_to_home'] for e in ends)
        checks['portal_'+'_'.join(connection['rooms'])] = valid
        portals.append(dict(endpoints=ends, passed=valid))
    # Detect overlapping room interiors using map cell centres, including occupied cells.
    overlap_cells = 0
    for row in range(free.shape[0]):
        for col in range(free.shape[1]):
            lx, ly = (col+.5)*resolution, (free.shape[0]-row-.5)*resolution
            x, y = ox+cos(yaw)*lx-sin(yaw)*ly, oy+sin(yaw)*lx+cos(yaw)*ly
            overlap_cells += sum(contains(room, x, y) for room in rooms.values()) > 1
    checks['no_room_cell_overlap'] = overlap_cells == 0
    observations = []
    for name, room in rooms.items():
        routes = [('default', room['views'])]+list(room.get('category_views',{}).items())
        for category, poses in routes:
            for pose in poses:
                c = cell(pose)
                observations.append(dict(room=name, category=category, pose=pose,
                    clearance_m=float(clearance[c]) if c else 0,
                    connected_to_home=bool(c and home_component > 0 and components[c] == home_component)))
    checks['observation_poses_connected'] = all(p['connected_to_home'] for p in observations)
    report = dict(status='passed' if all(checks.values()) else 'failed', checks=checks,
                  frame_id='map', radius_m=radius, grid_margin_m=resolution/2,
                  portals=portals, overlapping_cell_centres=overlap_cells,
                  observation_poses=observations,
                  validation='archived_static_map_only_not_Nav2_execution',
                  unvalidated=['new layout collision map', 'dynamic obstacles',
                               'sensor visibility', 'full room coverage', 'charging'])
    output = ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.transforms import Affine2D
        fig, ax = plt.subplots(figsize=(12, 8))
        extent = [0, free.shape[1]*resolution, 0, free.shape[0]*resolution]
        transform = Affine2D().rotate(yaw).translate(ox, oy)+ax.transData
        ax.imshow(image, cmap='gray', origin='upper', extent=extent, transform=transform,
                  vmin=0, vmax=255)
        for name, room in rooms.items():
            p = room['polygon']+[room['polygon'][0]]
            ax.plot(*zip(*p), label=name)
        for connection in plan['connections']:
            ax.plot(*zip(*(p[:2] for p in connection['poses'])), 'mo-', markersize=4)
        ax.plot(*plan['home_pose'][:2], 'g*', markersize=12, label='initial pose')
        ax.set(xlim=(-10,10), ylim=(-6,6), xlabel='Map x (m)', ylabel='Map y (m)',
               title='Candidate room polygons and entry connections (static map check)')
        ax.set_aspect('equal'); ax.legend(); ax.grid(alpha=.2)
        plot = ROOT / args.plot
        plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(plot, dpi=150); plt.close(fig)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
