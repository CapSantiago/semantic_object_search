#!/usr/bin/env python3
"""Check persisted real-run results, including their SVG IDs and sensor diagnostics."""
import json
from pathlib import Path
import re
from xml.etree import ElementTree as ET


def main():
    base = Path('results/validation')
    cases = {
        'task3_final': ('found', 1),
        'task4_final': ('completed', 2),
        'task4_cross_view_final': ('completed', 2),
        'task4_close_pair_final': ('completed', 2),
        'task4_stationary_final': ('completed', 1),
        'first_chair_view': ('completed', 2),
    }
    results = {}
    all_results = {}
    for file in Path('results').glob('*/*/result.json'):
        result = json.loads(file.read_text())
        if 'task_id' in result:
            all_results[result['task_id']] = file.parent
    for name, (status, count) in cases.items():
        file = base / f'{name}.json'
        if not file.exists():
            results[name] = {'passed': False, 'error': 'result missing'}
            continue
        result = json.loads(file.read_text())
        folder = all_results.get(result['task_id'])
        svg_ids = set()
        if folder is not None and (folder/'semantic_map.svg').exists():
            root = ET.parse(folder/'semantic_map.svg').getroot()
            svg_ids = {int(element.text) for element in root.findall('{*}text')}
        expected_ids = {item['id'] for item in result['objects']}
        observations = json.loads((folder/'observations.json').read_text()) if folder else []
        results[name] = dict(
            passed=(result['status'] == status and result['count'] == count
                    and result['tf_failures'] == 0 and result['valid_projections'] > 0
                    and svg_ids == expected_ids and len(svg_ids) == count),
            status=result['status'], count=result['count'], expected_count=count,
            synchronized_frames=result['synchronized_frames'],
            elapsed_seconds=result['elapsed_seconds'],
            object_hits=[item['hits'] for item in result['objects']],
            observations_per_view={i: sum(o['viewpoint'] == i for o in observations)
                                   for i in range(result['viewpoints_total'])},
            artifact_dir=str(folder), svg_ids=sorted(svg_ids))
    sensors = json.loads((base/'sensors.json').read_text())
    final_sensors = json.loads((base/'sensors_final/sensors.json').read_text())
    fresh_start = json.loads((base/'fresh_start/sensors.json').read_text())
    markers = json.loads((base/'markers_final.json').read_text())
    final_result = json.loads((base/'task4_final.json').read_text())
    labels = {m['id']: m['text'] for m in markers if m['namespace'] == 'labels'}
    markers_passed = (labels == {o['id']: str(o['id']) for o in final_result['objects']}
                      and len(markers) == 1 + 2*final_result['count'])
    mapping = json.loads((base/'mapping_drive_final.json').read_text())
    build_passed = '2 packages finished' in (base/'build_final.log').read_text()
    unit_log = (base/'unit_after.log').read_text()
    tests_passed = unit_log.rstrip().endswith('OK')
    test_count = int(re.search(r'Ran (\d+) tests', unit_log)[1])
    summary = dict(passed=(sensors['passed'] and final_sensors['passed'] and fresh_start['passed']
                          and markers_passed
                          and mapping['status'] == 'completed' and build_passed and tests_passed
                          and all(r['passed'] for r in results.values())),
                   sensors_passed=sensors['passed'], final_sensors_passed=final_sensors['passed'],
                   fresh_start_passed=fresh_start['passed'], rviz_markers_passed=markers_passed,
                   build_passed=build_passed, unit_tests_passed=tests_passed, unit_test_count=test_count,
                   mapping=dict(status=mapping['status'], waypoints=len(mapping['visited']),
                                wall_seconds=mapping['wall_seconds']), cases=results)
    (base/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    raise SystemExit(0 if summary['passed'] else 1)


if __name__ == '__main__':
    main()
