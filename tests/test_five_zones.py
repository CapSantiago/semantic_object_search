"""Five-zone migration preserves coverage and supplies one semantic partition."""
import json
from pathlib import Path
from semantic_search.rooms import room_at, validate_rooms, observation_views
from semantic_search.semantic_map import parse_command

ROOT = Path(__file__).resolve().parents[1]
def rooms(version):
    return json.loads((ROOT/f'src/semantic_search/config/rooms_aws_house_v{version}.json').read_text())

def test_split_preserves_original_union_and_other_rooms():
    old, new = rooms(3), rooms(4)
    validate_rooms(new)
    assert set(new) == {'kitchen','living','bedroom','utility','gym'}
    for ix in range(-95, 96):
        for iy in range(-57, 57):
            x,y = ix/10+.017,iy/10+.017
            before,after = room_at(old,x,y),room_at(new,x,y)
            expected = ('gym' if y>1 else 'living') if before=='living' else before
            assert after==expected, (x,y,before,after)

def test_shared_boundary_has_exactly_one_owner():
    r=rooms(4)
    assert room_at(r,0,1-1e-5)=='living'
    assert room_at(r,0,1)=='gym'
    assert room_at(r,0,1+1e-5)=='gym'
    assert room_at(r,0,0)=='living'

def test_new_aliases_and_unchanged_tabletop_routes():
    old,new=rooms(3),rooms(4)
    for alias in ('健身区','运动区','健身房'):
        assert parse_command('去'+alias+'统计瓶子',new)==('count','gym','bottle')
    assert parse_command('去杂物间找杯子',new)==('find','utility','cup')
    assert parse_command('去餐区找杯子',new)==('find','kitchen','cup')
    for room in ('kitchen','living','bedroom'):
        for category in ('cup','bottle'):
            assert observation_views(old[room],category)==observation_views(new[room],category)

def test_labels_and_gym_views_are_owned_by_their_region():
    r=rooms(4)
    for name,room in r.items():
        assert room_at(r,*room['label_position'])==name
        assert room['display_name'] and len(room['color'])==7
    assert all(room_at(r,*pose[:2])=='gym' for pose in r['gym']['views'])
