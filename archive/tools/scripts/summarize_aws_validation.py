#!/usr/bin/env python3
"""Check saved AWS deployment evidence; scene truth is used only for evaluation."""
import hashlib
import itertools
import json
from math import hypot
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1]
    evidence = root / 'results/aws_house/validation'
    final = evidence / 'final_run'
    def read(path):
        return json.loads(path.read_text(encoding='utf-8'))
    task3, task4 = read(final/'task3.json'), read(final/'task4.json')
    sensors = read(final/'sensors/sensors.json')
    baseline = read(root/'results/baseline/validation/sensors/sensors.json')
    preservation = read(root/'scenes/baseline/preservation.json')
    truth = [m for m in read(root/'scenes/aws_house/ground_truth.json')['instances']
             if 'Chair' in m['asset'] and m['pose'][0] > 4.8]
    objects = task4['objects']
    errors = []
    if len(objects) == len(truth) == 4:
        def distance(i, j):
            return hypot(objects[i]['x']-truth[j]['pose'][0],
                         objects[i]['y']-truth[j]['pose'][1])
        matches = min(itertools.permutations(range(4)),
                      key=lambda p: sum(distance(i, j) for i, j in enumerate(p)))
        errors = [dict(id=objects[i]['id'], truth=truth[j]['name'],
                       distance_to_model_origin_xy_m=distance(i, j))
                  for i, j in enumerate(matches)]
    artifacts, svg_matches = [], []
    for task in (task3, task4):
        folders = [p for p in (root/'results/aws_house/tasks').iterdir()
                   if (p/'result.json').is_file()
                   and read(p/'result.json')['task_id'] == task['task_id']]
        if len(folders) != 1:
            raise RuntimeError(f'Persisted result missing or ambiguous: {task["task_id"]}')
        folder = folders[0]
        labels = [e.text for e in ET.parse(folder/'semantic_map.svg').getroot().iter()
                  if e.tag.rsplit('}', 1)[-1] == 'text']
        svg_matches.append(sorted(labels) == sorted(str(o['id']) for o in task['objects']))
        artifacts.append(str(folder.relative_to(root)))
    markers = read(final/'markers.json')
    ids = sorted(o['id'] for o in objects)
    views = read(root/'src/semantic_search/config/rooms_aws_house.json')['kitchen']['views']
    test_log = (final/'tests.log').read_text()
    test_count = re.search(r'Ran (\d+) tests', test_log)
    checks = dict(
        build='2 packages finished' in (final/'build.log').read_text(),
        regression_tests=bool(test_count and int(test_count[1]) >= 15 and '\nOK\n' in test_log),
        baseline_preserved=all(hashlib.sha256((root/p).read_bytes()).hexdigest() == h
                               for p, h in preservation['sha256'].items()),
        baseline_sensors=baseline['passed'], sensors=sensors['passed'],
        mapping=read(evidence/'mapping_summary.json')['waypoints'] == 30,
        scene_identity=task3['scene_id'] == task4['scene_id'] == 'aws_house',
        task3=task3['status'] == 'found' and task3['count'] == 1,
        task4=task4['status'] == 'completed' and task4['count'] == 4,
        viewpoints=task4['viewpoints_total'] == task4['viewpoints_visited'] == len(views)
                   and task4['viewpoints'] == views,
        tf=task3['tf_failures'] == task4['tf_failures'] == 0,
        xy_evaluation=bool(errors and max(e['distance_to_model_origin_xy_m'] for e in errors) < .4),
        svg_labels=all(svg_matches),
        rviz_markers=markers['sphere_ids'] == markers['label_ids'] == ids,
    )
    summary = dict(scene_id='aws_house', status='passed' if all(checks.values()) else 'failed',
                   checks=checks, task3=task3, task4=task4, xy_evaluation=errors,
                   artifacts=artifacts, regression_tests=int(test_count[1]) if test_count else 0,
                   limitations=['Fixed layout and chair category; not a repeated-run benchmark',
                                'Speech, GPU inference, cup/bottle recognition unvalidated'],
                   calibration_history=['task4_initial.json', 'task4_threshold05.json',
                                        'task4_side_only.json'])
    (evidence/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(status=summary['status'], checks=checks, artifacts=artifacts),
                     ensure_ascii=False, indent=2))
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
