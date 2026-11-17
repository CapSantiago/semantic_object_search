#!/usr/bin/env python3
"""Evaluate unchanged detector/depth on calibration frames; not a search task."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/semantic_search'))
from semantic_search.depth import instance_centroid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', type=Path)
    args = parser.parse_args()
    model = YOLO(str(ROOT/'yolov8n-seg.pt'))
    base = args.captures
    manifest = json.loads((base/'views.json').read_text())
    if manifest.get('complete') is False:
        raise RuntimeError('Capture batch is still in progress; wait for all viewpoints')
    views = manifest['views']
    records = []
    for view in views:
        folder = base/view['name']
        frame = json.loads((folder/'frame.json').read_text())
        depth = np.load(folder/'depth.npy')
        prediction = model.predict(str(folder/'rgb.png'), device='cpu', conf=.35,
                                   iou=.30, verbose=False)[0]
        prediction.save(filename=str(folder/'detections.jpg'))
        detections = []
        for i, box in enumerate(prediction.boxes):
            category = model.names[int(box.cls)]
            point = None
            if category in ('cup','bottle') and prediction.masks is not None:
                centroid = instance_centroid(depth, prediction.masks.xy[i].tolist(), frame['k'],
                    frame['transform']['xyz'], frame['transform']['xyzw'],
                    .001 if frame['depth_encoding'] == '16UC1' else 1.)
                point = [float(v) for v in centroid] if centroid is not None else None
            detections.append(dict(category=category, score=float(box.conf),
                xyxy=box.xyxy.tolist()[0], point=point, frame_id=frame['fixed_frame']))
        valid = sorted({d['category'] for d in detections if d['point'] is not None})
        records.append(dict(view=view['name'], valid_target_categories=valid,
            depth_valid_fraction=float(np.mean(np.isfinite(depth)&(depth>0))), detections=detections))
        print(view['name'],valid,flush=True)
    room_hits = {room: sorted({c for view in records if view['view'].startswith(room)
                             for c in view['valid_target_categories']})
                 for room in ('kitchen','living','bedroom')}
    report = dict(method='single RGB-D frame per simulator-placed calibration viewpoint',
        model='yolov8n-seg.pt',device='cpu',confidence=.35,nms_iou=.30,
        views=records,room_category_hits=room_hits,
        all_rooms_have_cup_and_bottle=all(set(c)=={'cup','bottle'} for c in room_hits.values()),
        limitations=['Not autonomous navigation, repeated-run reliability, tracking or task counting.',
                     'Projection uses measured odom TF; no new map/world accuracy claim.',
                     'Some viewpoints may fail due to occlusion; failures retained.'])
    (base.parent/'visual_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':
    main()
