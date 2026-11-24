"""Source agreement, measured geometry, and honest separation of map layers."""
import copy
import json
import xml.etree.ElementTree as ET

from PIL import Image
import pytest

from semantic_search.view_export import export_view


def grid():
    return dict(available=True,frame_id='map',width=3,height=2,resolution=1.,
                origin=dict(x=0.,y=0.,yaw=0.),data=[0,0,100,0,0,0],memory_namespace='same_map')


def state():
    return dict(session_id='export_session',mode='navigation',rooms_frame_id='map',
                rooms={'room':dict(bounds=[0,0,3,2],display_name='<房间 & 测试>')})


def memory():
    return dict(available=True,truncated=False,namespace='same_map',revision='1:stamp',objects=[dict(
        id='record_one',category='cup',room='room',x=.5,y=.5,z=.8,
        state='needs_verification',task_id='previous',evidence='result.json')])


def cleaning():
    return dict(static_map=[[0,0,100],[0,0,0]],eligible=[[True,True,False],[False,False,False]],
                covered=[[True,False,False],[False,False,False]],
                result=dict(frame_id='map',room='room',status='partial',coverage=dict(floor_cells=5,eligible_cells=2,covered_cells=1,coverage_fraction=.5)),
                plan=dict(frame_id='map',origin=[0.,0.,0.],resolution_m=1.),
                trace=dict(frame_id='map',samples=[
                    dict(pose=[0.,0.,0.],connected=False,stage='sweeping'),
                    dict(pose=[2.,0.,0.],connected=False,stage='sweeping'),
                    dict(pose=[2.,1.,0.],connected=True,stage='sweeping')]))


def svg(folder,view):
    return ET.parse(folder/(view+'.svg')).getroot()


@pytest.mark.parametrize('view',['occupancy','rooms','semantic'])
def test_shared_map_and_historical_records_preserved_without_input_mutation(tmp_path,view):
    base=grid();history=memory();before=copy.deepcopy((base,history));out=tmp_path/view
    paths=export_view(out,view,base,state(),memory=history)
    data=json.loads(paths[-1].read_text());assert data['occupancy']==before[0]
    assert (base,history)==before
    assert Image.open(paths[0]).size==(1400,1140)
    if view=='semantic':
        assert data['memory']==history
        assert '当前确认' in ''.join(svg(out,view).itertext())
        rings=[n for n in svg(out,view)if n.tag.endswith('circle')]
        assert rings[0].get('fill')=='white' # Pending records are not confirmed markers.
    with pytest.raises(FileExistsError):export_view(out,view,base,state(),memory=history)


@pytest.mark.parametrize('change',[dict(namespace='another_map'),dict(truncated=True),dict(available=False)])
def test_wrong_map_or_truncated_history_never_produces_a_complete_view(tmp_path,change):
    m=memory();m.update(change)
    with pytest.raises(ValueError):export_view(tmp_path/'bad','semantic',grid(),state(),memory=m)
    assert not (tmp_path/'bad').exists()


def test_observation_uses_measured_arrival_not_nominal_goal_and_keeps_unvisited(tmp_path):
    task=dict(kind='search',result=dict(frame_id='map',viewpoint_diagnostics=[
        dict(pose=[0.,0.,0.],reached=True,dwell_completed=True,synchronized_frames=3,
             arrival_map_pose=dict(frame_id='map',xyz=[1.,1.,0.],xyzw=[0.,0.,0.,1.])),
        dict(pose=[2.,1.,0.],reached=False)]))
    paths=export_view(tmp_path/'obs','observations',grid(),state(),task=task)
    root=svg(tmp_path/'obs','observations');circles=[n for n in root if n.tag.endswith('circle')]
    green=next(n for n in circles if n.get('fill')=='#168278')
    # scale405, left92.5, top175; nominal goal maps to(92.5,985).
    assert float(green.get('cx'))==497.5 and float(green.get('cy'))==580.
    assert len(circles)==3 # Both nominal sites, only one measured observation.
    del task['result']['viewpoint_diagnostics'][0]['arrival_map_pose']
    with pytest.raises(ValueError,match='实测'):export_view(tmp_path/'invalid','observations',grid(),state(),task=task)
    assert not (tmp_path/'invalid').exists()


def test_cleaning_trace_keeps_discontinuities_and_actual_coverage_denominator(tmp_path):
    clean=cleaning();out=tmp_path/'trace';export_view(out,'clean_trace',grid(),state(),cleaning=clean)
    lines=[n for n in svg(out,'clean_trace')if n.tag.endswith('polyline')and n.get('stroke')=='#168278']
    assert len(lines)==1 # No fabricated connection from (0,0) to (2,0).
    data=json.loads((out/'snapshot.json').read_text());assert data['cleaning']==clean
    export_view(tmp_path/'coverage','clean_coverage',grid(),state(),cleaning=clean)
    clean['covered'][0][1]=True
    with pytest.raises(ValueError,match='报告不一致'):export_view(tmp_path/'bad','clean_coverage',grid(),state(),cleaning=clean)
    assert not (tmp_path/'bad').exists()


def test_cleaning_other_map_or_noneligible_credit_is_rejected(tmp_path):
    clean=cleaning();clean['static_map'][0][2]=0
    with pytest.raises(ValueError,match='混用地图'):export_view(tmp_path/'wrong','clean_coverage',grid(),state(),cleaning=clean)
    clean=cleaning();clean['covered'][1][0]=True
    with pytest.raises(ValueError,match='预测覆盖'):export_view(tmp_path/'wrong2','clean_coverage',grid(),state(),cleaning=clean)


def test_relations_require_actual_current_graph_edge_and_matching_record_fields(tmp_path):
    m=memory();item=m['objects'][0];graph=dict(metadata=dict(namespace='same_map',source_revision='1:stamp'),
        nodes=[dict(id=1,attributes=dict(name='家庭',metadata={})),dict(id=10,attributes=dict(metadata=dict(room_id='room'))),
               dict(id=20,attributes=dict(position=[.5,.5,.8],metadata=dict(item,memory_id=item['id'])))],
        edges=[dict(source=1,target=10),dict(source=10,target=20)])
    export_view(tmp_path/'relations','relations',grid(),state(),memory=m,graph=graph)
    graph['edges']=[dict(source=1,target=10)]
    with pytest.raises(ValueError,match='假关系'):export_view(tmp_path/'missing','relations',grid(),state(),memory=m,graph=graph)
    assert not (tmp_path/'missing').exists()
    graph['edges']=[dict(source=10,target=20)]
    with pytest.raises(ValueError,match='家庭'):export_view(tmp_path/'missing_root','relations',grid(),state(),memory=m,graph=graph)
    assert not (tmp_path/'missing_root').exists()


def test_unrelated_room_floor_is_not_painted_as_excluded_cleaning_area(tmp_path):
    current=state();current['rooms']['room']['bounds']=[0,0,2,1]
    clean=cleaning();clean['result']['coverage']['floor_cells']=2
    paths=export_view(tmp_path/'scope','clean_coverage',grid(),current,cleaning=clean)
    with Image.open(paths[0])as image:
        # Cell(0,1) is known free but outside the cleaning room: preserve base white.
        assert image.getpixel((295,377))==(248,248,248)


@pytest.mark.parametrize('frame,fresh,expected',[('map',True,True),('odom',True,False),('map',False,False)])
def test_robot_source_marker_respects_rotated_base_frame_and_freshness(tmp_path,frame,fresh,expected):
    import math
    base=grid();base['origin']['yaw']=math.pi/2
    current=state();current['telemetry']=dict(pose_fresh=fresh,pose=dict(frame_id=frame,x=-1.,y=1.,yaw=0.))
    paths=export_view(tmp_path/'pose','rooms',base,current)
    data=json.loads(paths[-1].read_text());markers=[n for n in svg(tmp_path/'pose','rooms')if n.tag.endswith('circle')and n.get('fill')=='#0b6058']
    assert bool(markers)==expected
    assert (data['robot_pose']is not None)==expected
    if expected:
        assert float(markers[0].get('cx'))==pytest.approx(497.5)
        assert float(markers[0].get('cy'))==pytest.approx(580.)
