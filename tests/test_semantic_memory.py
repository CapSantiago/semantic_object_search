"""Memory correctness under repeats, ambiguity, stale evidence and map changes."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy

import pytest

from semantic_search.semantic_memory import SemanticMemory, map_namespace


def grid():
    return dict(available=True, frame_id='map', width=2, height=2, resolution=.05,
                origin=dict(x=0., y=0., yaw=0.), data=[0, 0, -1, 100], stamp_seconds=1)


def result(task='one', xs=(0.,), **changes):
    value = dict(task_id=task, category='cup', room='kitchen', mode='count', status='completed',
                 frame_id='map', coordinate_unit='m', count=len(xs),
                 objects=[dict(id=i+1, x=x, y=1., z=.8, hits=3) for i, x in enumerate(xs)],
                 coverage=dict(observation_plan_completed=True, observation_data_complete=True,
                               full_room_coverage_verified=False))
    value.update(changes)
    return value


def ingest(memory, value, namespace='map-a', second=0):
    return memory.ingest(namespace, value, observed_at=f'2026-11-21T10:00:{second:02}+08:00',
                         evidence=f'results/sessions/test/tasks/{value["task_id"]}/result.json')


@pytest.fixture
def memory(tmp_path):
    return SemanticMemory(tmp_path/'memory.sqlite3')


def test_namespace_uses_geometry_and_ownership_not_calibration_or_timestamps():
    rooms = {'kitchen': dict(bounds=[-1, -1, 1, 1], views=[[0, 0, 0]])}
    original = grid()
    key = map_namespace(original, rooms)
    original['stamp_seconds'] = 400
    rooms['kitchen']['views'] = [[.5, 0, 1]]
    assert map_namespace(original, rooms) == key
    for field, value in [('data', [0, 0, 0, 100]), ('origin', dict(x=1., y=0., yaw=0.))]:
        changed = deepcopy(original)
        changed[field] = value
        assert map_namespace(changed, rooms) != key
    rooms['kitchen']['bounds'][0] = -2
    assert map_namespace(original, rooms) != key


def test_restart_and_replay_do_not_duplicate_or_inflate_hits(memory):
    ingest(memory, result())
    first = memory.query('map-a')['objects'][0]
    restored = SemanticMemory(memory.path)
    assert ingest(restored, result())['status'] == 'already_saved'
    same = restored.query('map-a')['objects'][0]
    assert same == first
    ingest(restored, result('two', (.02,)), second=1)
    updated = restored.query('map-a')['objects'][0]
    assert updated['id'] == first['id']
    assert updated['observation_tasks'] == 2
    assert updated['last_seen'] == '2026-11-21T02:00:01.000000+00:00'
    assert updated['evidence'].endswith('/two/result.json')


def test_guidance_reads_confirmed_instance_views_and_missing_evidence_is_a_fallback(memory,tmp_path):
    import json
    value=result()
    value['viewpoint_diagnostics']=[dict(index=0,pose=[0.,0.,0.],reached=True),
                                  dict(index=1,pose=[1.,0.,1.],reached=True)]
    ingest(memory,value)
    records=memory.query('map-a')['objects']
    path=tmp_path/'results/sessions/test/tasks/one/observations.json'
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps([dict(viewpoint=0,instance_ids=[99])]*3+
                               [dict(viewpoint=1,instance_ids=[1])]*2))
    successful=memory.successful_viewpoints('map-a',records,root=tmp_path)
    assert len(successful)==1 and successful[0]['pose']==[1.,0.,1.]
    assert successful[0]['memory_id']==records[0]['id'] and successful[0]['frames']==2
    assert memory.successful_viewpoints('other-map',records,root=tmp_path)==[]
    path.unlink()
    assert memory.successful_viewpoints('map-a',records,root=tmp_path)==[]
    assert memory.query('map-a')['objects']==records


def test_simultaneous_neighbors_keep_one_to_one_records(memory):
    ingest(memory, result(xs=(0., .2)))
    old = {round(row['x'], 2): row['id'] for row in memory.query('map-a')['objects']}
    ingest(memory, result('two', (.19, .01)), second=1)
    rows = memory.query('map-a')['objects']
    assert len(rows) == 2
    assert {round(row['x'], 2): row['id'] for row in rows} == {.01: old[0.], .19: old[.2]}


def test_ambiguous_association_is_explicit_and_does_not_merge_neighbors(memory):
    ingest(memory, result(xs=(0., .2)))
    ingest(memory, result('ambiguous', (.1,)), second=1)
    rows = memory.query('map-a')['objects']
    assert len(rows) == 3
    assert sum(row['association'] == 'ambiguous' for row in rows) == 1
    assert sum(row['state'] == 'needs_verification' for row in rows) == 2


def test_movement_creates_unverified_identity_and_empty_count_does_not_delete(memory):
    ingest(memory, result())
    old = memory.query('map-a')['objects'][0]
    ingest(memory, result('moved', (2.,)), second=1)
    rows = memory.query('map-a')['objects']
    assert len(rows) == 2
    assert next(row for row in rows if row['id'] == old['id'])['state'] == 'needs_verification'
    ingest(memory, result('empty', ()), second=2)
    rows = memory.query('map-a')['objects']
    assert len(rows) == 2
    assert all(row['state'] == 'needs_verification' for row in rows)


def test_partial_or_find_empty_does_not_mark_old_records_missing(memory):
    ingest(memory, result())
    for i, changes in enumerate((dict(status='partial'), dict(mode='find', status='not_found')), 1):
        ingest(memory, result(str(i), (), **changes), second=i)
    assert memory.query('map-a')['objects'][0]['state'] == 'last_observed'


def test_changed_task_id_content_and_out_of_order_import_are_rejected(memory):
    ingest(memory, result())
    with pytest.raises(ValueError, match='内容变化'):
        ingest(memory, result(xs=(.1,)))
    ingest(memory, result('empty', ()), second=10)
    with pytest.raises(ValueError, match='旧观测'):
        ingest(memory, result('late'), second=5)
    assert memory.query('map-a')['objects'][0]['observation_tasks'] == 1


@pytest.mark.parametrize('changes', [dict(frame_id='odom'), dict(count=3),
    dict(objects=[dict(id=1, x=float('nan'), y=0., z=.8, hits=2)]),
    dict(objects=[dict(id=1, x=True, y=0., z=.8, hits=2)]),
    dict(objects=[dict(id=1, x=0., y=0., z=.8, hits=0)]), dict(category='dumbbell')])
def test_invalid_data_cannot_partially_modify_memory(memory, changes):
    ingest(memory, result())
    with pytest.raises(ValueError):
        ingest(memory, result('invalid', **changes), second=1)
    assert memory.query('map-a')['objects'][0]['observation_tasks'] == 1


def test_maps_categories_rooms_and_limits_are_separate(memory):
    ingest(memory, result())
    ingest(memory, result('bottle-task', category='bottle'))
    ingest(memory, result(), namespace='map-b')
    ingest(memory, result('bedroom', room='bedroom'), second=1)
    assert memory.query('map-a')['total'] == 3
    assert memory.query('map-b')['total'] == 1
    assert memory.query('map-a', room='kitchen', category='cup')['total'] == 1
    limited = memory.query('map-a', limit=2)
    assert limited['total'] == 3 and len(limited['objects']) == 2 and limited['truncated']


def test_cross_class_history_is_flagged_even_in_a_filtered_query_without_changing_results(memory):
    ingest(memory,result('orange',category='orange'))
    assert not memory.query('map-a')['objects'][0]['classification_conflict']
    ingest(memory,result('apple',(.02,),category='apple'),second=1)
    restored = SemanticMemory(memory.path)
    rows = restored.query('map-a',category='apple')['objects']
    assert len(rows)==1 and rows[0]['classification_conflict']
    assert restored.query('map-a',category='orange')['objects'][0]['classification_conflict']
    assert rows[0]['category']=='apple' and rows[0]['state']=='last_observed'
    # Nearby furniture can overlap portable-object surface centroids.
    ingest(memory,result('table',(2.,),category='dining table'),second=2)
    ingest(memory,result('cup',(2.,),category='cup'),second=3)
    assert not restored.query('map-a',category='cup')['objects'][0]['classification_conflict']
    assert not restored.query('map-b')['objects']


def test_concurrent_replay_is_atomic(memory):
    with ThreadPoolExecutor(max_workers=6) as pool:
        outcomes = list(pool.map(lambda _: ingest(memory, result()), range(12)))
    assert sum(item['status'] == 'saved' for item in outcomes) == 1
    assert memory.query('map-a')['objects'][0]['observation_tasks'] == 1


def test_complete_empty_count_marks_history_for_verification_without_changing_evidence(memory):
    ingest(memory, result())
    before = memory.query('map-a')['objects'][0]
    empty = result('real_empty_status', (), status='not_found')
    assert ingest(memory, empty, second=1)['objects'] == 0
    after = memory.query('map-a')['objects'][0]
    assert after['state'] == 'needs_verification'
    assert after['last_checked'] == '2026-11-21T02:00:01.000000+00:00'
    assert {k: v for k, v in after.items() if k not in ('state', 'last_checked')} == {
        k: v for k, v in before.items() if k not in ('state', 'last_checked')}
    assert empty['count'] == 0 and empty['objects'] == []
    assert ingest(memory, empty, second=1)['status'] == 'already_saved'
    assert memory.query('map-a')['objects'][0] == after


@pytest.mark.parametrize('coverage', [
    dict(observation_plan_completed=False, observation_data_complete=True),
    dict(observation_plan_completed=True, observation_data_complete=False), {},
])
def test_empty_count_without_complete_observation_preserves_history(memory, coverage):
    ingest(memory, result())
    before = memory.query('map-a')['objects'][0]
    ingest(memory, result('incomplete_empty', (), status='not_found', coverage=coverage), second=1)
    assert memory.query('map-a')['objects'][0] == before
