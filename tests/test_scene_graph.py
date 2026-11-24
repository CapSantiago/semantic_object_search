"""Exercise the real upstream Spark-DSG API and its serialization contract."""
import json

import pytest

from semantic_search.scene_graph import build_scene_graph, export_scene_graph


@pytest.fixture
def dsg():
    # ROS launch-testing inspects modules during directory collection. A module-
    # level skip can accidentally suppress the entire project test directory.
    return pytest.importorskip('spark_dsg')


def snapshot():
    return dict(namespace='map-test', revision='1:2026-11-21', truncated=False, objects=[dict(
        id='0123456789abcdef0123456789abcdef', category='cup', room='wrong-claimed-room',
        x=1., y=1., z=.8, last_seen='2026-11-21T02:00:00+00:00', state='last_observed',
        association='spatial', observation_tasks=2, evidence='tasks/source/result.json', task_id='source')])


def rooms():
    return {'kitchen':dict(bounds=[0,0,2,2], aliases=['厨房']),
            'living':dict(bounds=[3,0,5,2], aliases=['客厅'])}


def test_real_library_roundtrip_preserves_sensor_evidence_and_room_membership(tmp_path, dsg):
    output = tmp_path/'graph.json'
    receipt = export_scene_graph(snapshot(), rooms(), output)
    assert receipt['nodes'] == 4 and receipt['edges'] == 3 and receipt['objects'] == 1
    restored = dsg.DynamicSceneGraph.load(output)
    objects = list(restored.get_layer(dsg.DsgLayers.OBJECTS).nodes)
    assert len(objects) == 1
    attrs = objects[0].attributes
    assert attrs.semantic_label == 41
    assert list(attrs.position) == [1., 1., .8]
    metadata = attrs.metadata.get()
    assert metadata['room'] == 'kitchen'
    assert metadata['reported_room'] == 'wrong-claimed-room'
    assert metadata['evidence'] == 'tasks/source/result.json'
    assert not metadata['physical_identity_verified']
    assert not metadata['classification_conflict']
    assert restored.metadata.get()['adapter_version']==2
    assert restored.metadata.get()['source_revision'] == '1:2026-11-21'
    assert json.loads(output.read_text())['metadata']['library'] == 'MIT-SPARK/Spark-DSG'


def test_record_node_ids_are_stable_when_other_records_are_added(dsg):
    first = build_scene_graph(snapshot(), rooms())
    data = snapshot()
    new = dict(data['objects'][0], id='1123456789abcdef0123456789abcdef', x=4., room='living')
    data['objects'].insert(0,new)
    second = build_scene_graph(data, rooms())
    original = list(first.get_layer(dsg.DsgLayers.OBJECTS).nodes)[0].id.value
    assert second.has_node(original)
    assert len(list(second.get_layer(dsg.DsgLayers.OBJECTS).nodes)) == 2


def test_incomplete_export_never_replaces_existing_graph(tmp_path, dsg):
    output = tmp_path/'graph.json'
    export_scene_graph(snapshot(), rooms(), output)
    old = output.read_bytes()
    data = snapshot(); data['truncated'] = True
    with pytest.raises(ValueError,match='截断'):
        export_scene_graph(data,rooms(),output)
    assert output.read_bytes() == old
