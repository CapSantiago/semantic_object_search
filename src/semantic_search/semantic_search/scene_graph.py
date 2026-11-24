"""Adapt sensor-backed memory to MIT-SPARK's Spark-DSG representation.

This adapter uses the upstream graph API. Centroids and room membership are
available now; bounding boxes, CLIP features and moved-object reidentification
require further sensing and are deliberately not fabricated here.
"""
import argparse
from datetime import datetime
from importlib.metadata import version
import json
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from .objects import OBJECTS
from .rooms import room_at
from .semantic_memory import SemanticMemory

GRAPH_ADAPTER_VERSION = 2


def build_scene_graph(snapshot, rooms):
    import spark_dsg as dsg
    if snapshot.get('truncated'):
        raise ValueError('查询结果被截断，不能导出为完整语义图。')
    graph = dsg.DynamicSceneGraph()
    graph.metadata.set(dict(namespace=snapshot['namespace'], frame_id='map', coordinate_unit='m',
        library='MIT-SPARK/Spark-DSG', library_version=version('spark-dsg'),
        adapter_version=GRAPH_ADAPTER_VERSION,
        source_revision=snapshot.get('revision'),
        observation_basis='robot_search_results', geometry='observed_centroids',
        association='spatial_baseline', physical_identity_verified=False))
    building = dsg.NodeSymbol('B', 0)
    building_attrs = dsg.SemanticNodeAttributes()
    building_attrs.name = '家庭'
    graph.add_node(dsg.DsgLayers.BUILDINGS, building, building_attrs)
    room_nodes = {}
    for index, (key, room) in enumerate(sorted(rooms.items())):
        attrs = dsg.RoomNodeAttributes()
        attrs.name = room.get('display_name', room.get('aliases', [key])[0])
        if 'polygon' in room:
            xy = np.mean(room['polygon'], axis=0)
        else:
            xmin, ymin, xmax, ymax = room['bounds']
            xy = ((xmin+xmax)/2, (ymin+ymax)/2)
        attrs.position = [float(xy[0]), float(xy[1]), 0.]
        attrs.metadata.set(dict(room_id=key, geometry={name: room[name]
            for name in ('polygon', 'holes', 'bounds') if name in room}, source='configured_semantic_rooms'))
        node = dsg.NodeSymbol('R', index)
        graph.add_node(dsg.DsgLayers.ROOMS, node, attrs)
        graph.insert_edge(building, node)
        room_nodes[key] = node
    used_ids = set()
    for item in snapshot['objects']:
        index = int(item['id'][:14], 16)
        if index in used_ids:
            raise ValueError('场景图节点 ID 冲突，保留原记忆等待处理。')
        used_ids.add(index)
        attrs = dsg.ObjectNodeAttributes()
        attrs.name = OBJECTS[item['category']].name
        attrs.semantic_label = OBJECTS[item['category']].coco_id
        attrs.position = [item['x'], item['y'], item['z']]
        attrs.last_update_time_ns = int(datetime.fromisoformat(item['last_seen']).timestamp()*1e9)
        actual_room = room_at(rooms, item['x'], item['y'])
        attrs.metadata.set(dict(memory_id=item['id'], category=item['category'], room=actual_room,
            reported_room=item['room'], state=item['state'], last_seen=item['last_seen'],
            category_admitted=OBJECTS[item['category']].admitted,
            classification_conflict=item.get('classification_conflict',False),
            association=item['association'], observation_tasks=item['observation_tasks'],
            evidence=item['evidence'], task_id=item['task_id'], physical_identity_verified=False))
        node = dsg.NodeSymbol('O', index)
        graph.add_node(dsg.DsgLayers.OBJECTS, node, attrs)
        if actual_room in room_nodes:
            graph.insert_edge(room_nodes[actual_room], node)
    return graph


def export_scene_graph(snapshot, rooms, output):
    import spark_dsg as dsg
    graph = build_scene_graph(snapshot, rooms)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.scene-graph-', dir=output.parent) as tmp:
        temporary = Path(tmp)/'graph.json'
        graph.save(temporary, False)
        restored = dsg.DynamicSceneGraph.load(temporary)
        if restored.num_nodes() != graph.num_nodes() or restored.num_edges() != graph.num_edges():
            raise ValueError('上游场景图序列化重载检查失败。')
        temporary.replace(output)
    return dict(library='spark-dsg', version=version('spark-dsg'),
                nodes=graph.num_nodes(), edges=graph.num_edges(), objects=len(snapshot['objects']))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database', required=True)
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--rooms', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    snapshot = SemanticMemory(args.database).query(args.namespace, limit=500)
    snapshot['namespace'] = args.namespace
    rooms = json.loads(Path(args.rooms).read_text())
    print(json.dumps(export_scene_graph(snapshot, rooms, args.output), ensure_ascii=False))


if __name__ == '__main__':
    main()
