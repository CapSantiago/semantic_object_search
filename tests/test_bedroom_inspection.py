import json
from pathlib import Path

from semantic_search.rooms import inspection_coverage, observation_views, room_at, validate_rooms
from semantic_search.semantic_memory import map_namespace

ROOT=Path(__file__).resolve().parents[1]


def test_bedroom_has_distributed_views_with_required_desk_and_nightstand():
    rooms=json.loads((ROOT/'src/semantic_search/config/rooms_aws_house_v6.json').read_text())
    old=json.loads((ROOT/'src/semantic_search/config/rooms_aws_house_v5.json').read_text())
    validate_rooms(rooms)
    assert rooms['kitchen']==old['kitchen']
    for category in ['cup','bottle','chair','book']:
        route=observation_views(rooms['bedroom'],category)
        assert len(route)==6 and len({tuple(p[:2]) for p in route})==5
        assert route[:3]==[p for v in rooms['bedroom']['inspection_views'].values() for p in v]
    grid=dict(available=True,frame_id='map',width=1,height=1,resolution=.05,origin=[0,0,0],data=[0])
    assert map_namespace(grid,old)==map_namespace(grid,rooms)


def test_nightstand_placement_stays_in_bedroom_and_does_not_supply_runtime_counts():
    rooms=json.loads((ROOT/'src/semantic_search/config/rooms_aws_house_v6.json').read_text())
    layout=json.loads((ROOT/'scenes/aws_house/layout_service_surfaces_v1.json').read_text())
    target=next(t for t in layout['targets'] if t['id']=='bedroom_nightstand_yellow_mug')
    assert target['support']=='NightStand_01_001' and room_at(rooms,*target['pose'][:2])=='bedroom'
    assert layout['purpose']=='scene_design_and_evaluation_only'
    assert not inspection_coverage(rooms['bedroom'],[])['completed']
