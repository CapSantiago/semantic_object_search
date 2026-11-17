#!/usr/bin/env python3
"""Inventory existing scene-design inputs; never feed truth to runtime search."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='scenes/aws_house/planning_inventory.json')
    args = parser.parse_args()
    sources = {
        'scene': 'scenes/aws_house/ground_truth.json',
        'rooms': 'src/semantic_search/config/rooms_aws_house.json',
        'assets': 'resources/assets.lock.json',
    }
    data = {key: json.loads((ROOT / path).read_text()) for key, path in sources.items()}
    instances = data['scene']['instances']
    assert len({item['name'] for item in instances}) == len(instances)
    rooms = data['rooms']
    supports = ('KitchenTable', 'CoffeeTable', 'ReadingDesk', 'NightStand',
                'CookingBench', 'BalconyTable')
    objects = []
    for item in instances:
        x, y = item['pose'][:2]
        candidates = []
        for name, room in rooms.items():
            xmin, ymin, xmax, ymax = room['bounds']
            if xmin <= x <= xmax and ymin <= y <= ymax:
                candidates.append(name)
        objects.append(dict(item, room_candidates_from_origin=candidates,
                            support_candidate=item['name'].startswith(supports),
                            support_surface_z_m=None))
    report = dict(
        purpose='scene_design_only_not_runtime_perception_or_search_input',
        source_sha256={path: hashlib.sha256((ROOT / path).read_bytes()).hexdigest()
                       for path in sources.values()},
        frame='world', position_unit='m', angle_unit='rad',
        coordinate_note='Existing map alignment was validated for the archived layout only; '
                        'new geometry requires separate map/extrinsic verification.',
        limitations=['Room candidates use model origins and legacy rectangular bounds; '
                     'they do not define final polygons or object ownership.',
                     'Model origin height is not measured support-surface height.',
                     'No physics, navigation, collision, visibility or inference was tested.'],
        rooms=rooms, instances=objects,
        staged_targets=[{key: a[key] for key in
                        ('id', 'category', 'directory', 'version', 'model_sdf_sha256', 'status')}
                        for a in data['assets']['assets']],
        totals=dict(instances=len(objects), distinct_assets=len({i['asset'] for i in objects}),
                    chairs=sum(i['name'].startswith('Chair') for i in objects),
                    support_candidates=sum(i['support_candidate'] for i in objects),
                    staged_targets=len(data['assets']['assets'])))
    output = Path(args.output)
    if not output.is_absolute():
        output = ROOT / output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report['totals'], ensure_ascii=False))


if __name__ == '__main__':
    main()
