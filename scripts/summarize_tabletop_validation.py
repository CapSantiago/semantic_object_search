#!/usr/bin/env python3
"""Audit persisted tabletop trials. Layout truth is evaluation-only input."""
import argparse
from collections import Counter
import hashlib
import json
from math import hypot
from pathlib import Path
import xml.etree.ElementTree as ET

from scipy.optimize import linear_sum_assignment


def read(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--layout', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--result-pattern', default='task[34]_*.json',
                        help='Result glob relative to --results (e.g. repeat_matrix/*/result.json)')
    args = parser.parse_args()
    layout = read(args.layout)
    persisted = {}
    for path in (args.results/'tasks').glob('*/result.json'):
        persisted.setdefault(read(path)['task_id'], []).append(path.parent)
    cases = []
    for path in sorted(args.results.glob(args.result_pattern)):
        result = read(path)
        category, room = result['category'], result['room']
        if category not in ('cup','bottle'):
            raise ValueError('Tabletop evaluator excludes base-scene furniture: '+category)
        truth = [t for t in layout['targets'] if t['category'] == category and t['room'] == room]
        expected = min(1, len(truth)) if result['mode'] == 'find' else len(truth)
        expected_status = ('found' if result['mode'] == 'find' else 'completed') if expected else 'not_found'
        folders = persisted.get(result['task_id'], [])
        checks = dict(status=result['status'] == expected_status, count=result['count'] == expected,
                      unique_ids=len({o['id'] for o in result['objects']}) == result['count'],
                      tf=result['tf_failures'] == 0,
                      synchronized_data=result['synchronized_frames'] > 0,
                      persisted_result=len(folders) == 1,
                      projections=not expected or result['valid_projections'] >= 2)
        if len(folders) == 1:
            folder = folders[0]
            checks['persisted_result'] &= read(folder/'result.json') == result
            labels = [e.text for e in ET.parse(folder/'semantic_map.svg').getroot().iter()
                      if e.tag.rsplit('}',1)[-1] == 'text']
            checks['svg_ids'] = sorted(labels) == sorted(str(o['id']) for o in result['objects'])
            hits = Counter(i for frame in read(folder/'observations.json') for i in frame['instance_ids'])
            checks['observed_hits'] = all(hits[o['id']] == o['hits'] for o in result['objects'])
        errors = []
        if truth and result['objects']:
            costs = [[hypot(o['x']-t['pose'][0], o['y']-t['pose'][1]) for t in truth]
                     for o in result['objects']]
            rows, cols = linear_sum_assignment(costs)
            errors = [dict(id=result['objects'][i]['id'], target=truth[j]['id'],
                           distance_to_model_origin_xy_m=costs[i][j]) for i,j in zip(rows,cols)]
        cases.append(dict(case=str(path.relative_to(args.results)), source=str(path),
                          source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                          checks=checks, passed=all(checks.values()), result=result,
                          xy_evaluation=errors))
    checks = dict(has_cases=bool(cases), all_case_checks=bool(cases) and all(c['passed'] for c in cases),
                  unique_task_ids=len({c['result']['task_id'] for c in cases}) == len(cases))
    report = dict(status='passed' if all(checks.values()) else 'failed',
                  scope='Fixed tabletop fixtures; not G0, speech or full-room coverage certification',
                  runtime_config=read(args.results/'runtime_config.json'),
                  checks=checks, layout=str(args.layout),
                  layout_sha256=hashlib.sha256(args.layout.read_bytes()).hexdigest(), cases=cases,
                  limitations=['Model-origin XY comparison assumes this project map/world alignment; not independent sensor accuracy.',
                               'No collision/contact-sensor proof; inspect actual navigation/recovery logs separately.',
                               'Repeat/cold-start completeness and full-room coverage are not inferred from these checks.'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(status=report['status'],cases=len(cases), checks=checks,
                         failed=[c['case'] for c in cases if not c['passed']]),ensure_ascii=False))
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
