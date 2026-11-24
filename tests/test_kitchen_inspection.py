import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from semantic_search.rooms import inspection_coverage, observation_views, validate_rooms, room_at
from semantic_search.semantic_memory import map_namespace

ROOT=Path(__file__).resolve().parents[1]


def configured():
    return json.loads((ROOT/'src/semantic_search/config/rooms_aws_house_v5.json').read_text())


def diagnostics(room):
    return [dict(index=i,pose=pose,reached=True,navigation_status=4,dwell_completed=True,
        synchronized_frames=2,valid_projections=0,tf_failures=0,invalid_target_depth=0)
        for i,pose in enumerate(observation_views(room,'cup'))]


def test_both_areas_precede_the_original_route_for_every_category():
    rooms=configured();validate_rooms(rooms);room=rooms['kitchen']
    required=[p for v in room['inspection_views'].values() for p in v]
    for category in ['cup','bottle','chair','bowl','book']:
        route=observation_views(room,category)
        assert route[:3]==required and len(route)==len({tuple(p) for p in route})
    old={'views':[[0,0,0]],'category_views':{'cup':[[1,1,1]]}}
    assert observation_views(old,'cup')==[[1,1,1]]


def test_worktop_is_semantically_in_kitchen_with_an_isolated_memory_namespace():
    old=json.loads((ROOT/'src/semantic_search/config/rooms_aws_house_v4.json').read_text())
    new=configured()
    for xy in [(8.7,-5.1),(9.,-3.05)]:
        assert room_at(old,*xy) is None
        assert room_at(new,*xy)=='kitchen'
    grid=dict(available=True,frame_id='map',width=1,height=1,resolution=.05,origin=[0,0,0],data=[0])
    assert map_namespace(grid,old)!=map_namespace(grid,new)


@pytest.mark.parametrize('change',[dict(reached=False),dict(dwell_completed=False),
    dict(synchronized_frames=0),dict(tf_failures=1)])
def test_inspection_cannot_be_completed_by_arrival_without_usable_observations(change):
    room=configured()['kitchen'];views=diagnostics(room)
    assert inspection_coverage(room,views)['completed']
    views[2].update(change);coverage=inspection_coverage(room,views)
    assert not coverage['completed']
    assert coverage['groups'][0]['completed'] and not coverage['groups'][1]['completed']


def test_confirmed_find_waits_for_the_worktop_before_returning():
    from semantic_search.search_node import Search
    from builtin_interfaces.msg import Time
    rooms=configured();room=rooms['kitchen'];views=diagnostics(room)
    node=SimpleNamespace(task=('find','kitchen','cup'),rooms=rooms,
        landmarks=SimpleNamespace(confirmed=[object()]),views=observation_views(room,'cup'),
        viewpoint_diagnostics=views,view=1,active_enabled=False,finish=Mock(),
        nav=Mock(),on_goal=Mock(),publish_progress=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=Time)))
    views[2]['dwell_completed']=False
    Search.next_view(node)
    node.finish.assert_not_called();node.nav.send_goal_async.assert_called_once()
    views[2]['dwell_completed']=True
    Search.next_view(node)
    node.finish.assert_called_once_with('found')
