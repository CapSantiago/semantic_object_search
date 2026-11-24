from collections import Counter
import math

from semantic_search.memory_guidance import prioritize_find_views


def test_history_prioritizes_a_view_but_keeps_every_fallback_pose():
    views=[[0,0,math.pi],[0,0,0],[2,0,math.pi]]
    records=[dict(id='memory1',x=1.,y=0.,z=.3,state='last_observed')]
    ordered,evidence=prioritize_find_views(views,{},records)
    assert ordered[0]==views[1]
    assert Counter(map(tuple,ordered))==Counter(map(tuple,views))
    assert evidence['order_changed'] and evidence['memory_ids']==['memory1']
    assert evidence['counts_from_history'] is False
    # The policy cannot mutate history into a currently confirmed detection.
    assert records==[dict(id='memory1',x=1.,y=0.,z=.3,state='last_observed')]


def test_two_mandatory_surfaces_stay_complete_and_first():
    views=[[0,0,math.pi],[2,0,math.pi],[0,0,0],[3,0,0]]
    room=dict(inspection_views=dict(table=views[:1],worktop=views[1:2]))
    records=[dict(id='memory1',x=1.,y=0.,z=.3,state='last_observed')]
    ordered,evidence=prioritize_find_views(views,room,records)
    assert ordered[:2]==views[:2]
    assert Counter(map(tuple,ordered))==Counter(map(tuple,views))


def test_missing_conflicting_or_invalid_history_keeps_the_original_route():
    views=[[0,0,math.pi],[0,0,0]]
    for records in ([],[dict(id='bad',x=float('nan'),y=0.,z=.3,state='last_observed')],
                    [dict(id='bad',x=1.,y=0.,z=.3,state='last_observed',classification_conflict=True)]):
        ordered,evidence=prioritize_find_views(views,{},records)
        assert ordered==views and not evidence['used']


def test_real_historical_view_can_outrank_a_geometric_bearing_guess():
    views=[[0,0,0],[0,1,-.5],[2,0,math.pi]]
    records=[dict(id='m',x=1.,y=0.,z=.3,state='last_observed')]
    successes=[dict(memory_id='m',task_id='past',pose=views[2],frames=3)]
    ordered,e=prioritize_find_views(views,{},records,successful_views=successes)
    assert ordered[0]==views[2] and e['historical_successful_views']==successes
    assert Counter(map(tuple,ordered))==Counter(map(tuple,views))
    assert not e['counts_from_history']


def test_far_success_can_be_verified_close_up_at_an_existing_site():
    # Recorded gym history had a far successful view, while a nearer calibrated
    # position faced away. Reorient that position instead of adding a location.
    views=[[0.,1.2,1.57],[1.5,4.9,-2.8],[-1.4,2.6,.6]]
    history=[dict(id='last',x=2.7148357,y=4.3285725,z=.2859,state='last_observed'),
             dict(id='old',x=.9,y=3.77,z=.29,state='needs_verification')]
    ordered,e=prioritize_find_views(views,{},history,
        successful_views=[dict(memory_id='last',pose=views[2],task_id='past',frames=2)])
    assert ordered[0][:2]==views[1][:2]
    assert abs(ordered[0][2]-math.atan2(history[0]['y']-4.9,history[0]['x']-1.5))<1e-9
    assert Counter(map(tuple,ordered[1:]))==Counter(map(tuple,views))
    assert e['added_viewpoints']==1 and len(e['verification_views'])==1
    assert e['verification_views'][0]['memory_id']=='last'
    assert .8<=e['verification_views'][0]['distance_m']<=2.2
    assert not e['counts_from_history']


def test_bearing_recheck_does_not_skip_any_mandatory_surface_or_fallback():
    views=[[0,0,math.pi],[2,0,math.pi],[0,2,0]]
    room=dict(inspection_views=dict(table=[views[0]],counter=[views[1]]))
    history=[dict(id='m',x=3.,y=.6,z=.8,state='last_observed')]
    ordered,e=prioritize_find_views(views,room,history)
    assert ordered[:2]==views[:2] and ordered[2][:2]==views[1][:2]
    assert len(ordered)==len(views)+1
    assert Counter(map(tuple,views))<=Counter(map(tuple,ordered))
    assert e['verification_views'][0]['source_configured_pose']==views[1]


def test_distant_history_cannot_invent_a_nearer_navigation_position():
    views=[[0,0,0],[1,0,math.pi]]
    ordered,e=prioritize_find_views(views,{},[dict(id='far',x=8.,y=8.,z=.8,state='last_observed')])
    assert Counter(map(tuple,ordered))==Counter(map(tuple,views))
    assert e['added_viewpoints']==0 and e['verification_views']==[]


def test_acceptance_rejects_dropped_fallback_or_invented_location(monkeypatch):
    from pathlib import Path
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'scripts/validation'))
    from target_stability import configured_route_matches
    views=[[0,0,math.pi],[2,0,math.pi],[0,2,0]]
    room=dict(views=views)
    plan,evidence=prioritize_find_views(views,room,[dict(id='m',x=3.,y=.6,z=.8,state='last_observed')])
    result=dict(mode='find',viewpoints=plan,memory_guidance=evidence)
    assert configured_route_matches(result,room,'cup')
    assert not configured_route_matches(dict(result,viewpoints=plan[:-1]),room,'cup')
    invalid=[list(v)for v in plan];invalid[0][0]+=10
    assert not configured_route_matches(dict(result,viewpoints=invalid),room,'cup')
    assert not configured_route_matches(dict(result,mode='count'),room,'cup')


def test_real_find_can_stop_at_three_views_while_full_fallback_is_retained(monkeypatch):
    import copy
    import json
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts/validation'))
    from target_stability import configured_route_matches
    fixture=json.loads((root/'tests/fixtures/memory_find_early_result.json').read_text())
    result,room=fixture['result'],fixture['room']
    assert len(result['viewpoints'])==3 and len(result['viewpoint_diagnostics'])==11
    assert configured_route_matches(result,room,'sports ball')
    missing=copy.deepcopy(result);missing['viewpoint_diagnostics'].pop()
    assert not configured_route_matches(missing,room,'sports ball')
    assert not configured_route_matches(dict(result,status='not_found'),room,'sports ball')
