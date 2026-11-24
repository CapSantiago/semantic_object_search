"""Export geometry and evidence must agree with native map frames and times."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest

from semantic_search.map_export import compose_raster, export_costmap


def grid(width=2, height=2, **changes):
    return dict(available=True, frame_id='map', width=width, height=height, resolution=1.,
                origin=dict(x=0., y=0., yaw=0.), data=[0]*(width*height), **changes)


def native():
    return grid(source='/global_costmap/costmap', fresh=True, stamp_seconds=10.)


def state():
    return dict(session_id='test_session', mode='navigation',
                task=dict(id='clean_01', kind='clean', state='running',
                          progress=dict(trace_preview=[[0.,0.,'sweeping',True],[1.,1.,'sweeping',True]])),
                telemetry=dict(clock_seconds=10.5, pose_fresh=False,
                               navigation_path=dict(fresh=False,frame_id='map',stamp_seconds=0.,points=[[0,0],[1,1]])))


def test_rotated_shifted_native_cells_are_projected_on_shared_base():
    base = grid()
    cost = native()
    # Rotated native cell (0,0) has world centre (1.5,.5), bottom right.
    cost.update(origin=dict(x=2.,y=0.,yaw=math.pi/2), data=[100,0,0,0])
    raster = np.asarray(compose_raster(base,cost)).astype(int)
    red = (raster[:,:,0] > raster[:,:,1]+40)
    assert red.tolist() == [[False,False],[False,True]]
    cost['frame_id']='odom'
    with pytest.raises(ValueError,match='坐标系'):compose_raster(base,cost)


@pytest.mark.parametrize('changes',[dict(fresh=False),dict(stamp_seconds=7.9),dict(stamp_seconds=11.),dict(source='/local_costmap/costmap')])
def test_no_export_with_stale_or_wrong_source(tmp_path,changes):
    costs=native();costs.update(changes)
    with pytest.raises(ValueError):export_costmap(tmp_path/'invalid',grid(),costs,state())
    assert not (tmp_path/'invalid').exists()


def test_bundle_preserves_sources_and_omits_stale_plan(tmp_path):
    costs=native();costs['data']=[-1,98,99,100]
    current=state();current['task']['id']='<task & id>'
    output=tmp_path/'bundle'
    paths=export_costmap(output,grid(),costs,current,captured_at='2026-11-23T12:00:00+00:00')
    bundle=json.loads((output/'snapshot.json').read_text())
    assert bundle['costmap']==costs and bundle['occupancy']==grid()
    assert bundle['navigation_path'] is None and len(bundle['trace_preview'])==2
    assert bundle['robot_pose'] is None
    svg=ET.parse(output/'navigation_cost.svg')
    assert '<task & id>' in ''.join(svg.getroot().itertext())
    with Image.open(output/'navigation_cost.png')as png:
        assert png.size==(1400,1140) and png.format=='PNG'
    assert len(paths)==3 and all(p.is_file()for p in paths)
    with pytest.raises(FileExistsError):export_costmap(output,grid(),costs,current)


def test_sampled_actual_trace_does_not_join_disconnected_localization_segments(tmp_path):
    current=state();current['task']['progress']['trace_preview']=[
        [0.,0.,'sweeping',False],[1.,0.,'sweeping',True],
        [1.,1.,'sweeping',False],[0.,1.,'sweeping',True]]
    export_costmap(tmp_path/'gaps',grid(),native(),current)
    root=ET.parse(tmp_path/'gaps/navigation_cost.svg').getroot()
    raster=next(n for n in root if n.tag.endswith('image'))
    bottom=float(raster.get('y'))+float(raster.get('height'))
    lines=[n for n in root if n.tag.endswith('polyline')and n.get('stroke')=='#168278'
           and max(float(p.split(',')[1])for p in n.get('points').split())<=bottom]
    assert len(lines)==2
    assert lines[0].get('points').split()[-1]!=lines[1].get('points').split()[0]
