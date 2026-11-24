#!/usr/bin/env python3
"""Evaluate measured view coverage; save unobserved map samples for inspection."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/semantic_search'))
from semantic_search.rooms import room_at, observation_views
from semantic_search.visibility import visible_samples
from semantic_search.map_resources import resolve_map


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('captures',type=Path)
    p.add_argument('--rooms',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--spacing',type=float,default=.2)
    p.add_argument('--manifest',type=Path,help='JSON mapping capture folder names to commanded map poses')
    p.add_argument('--plot',type=Path,help='Optional PNG of observed/unobserved samples by route and height')
    p.add_argument('--map',type=Path,help='Explicit map version; default is the registered AWS map')
    args=p.parse_args()
    if not .05<=args.spacing<=1.:p.error('spacing must be 0.05–1 m')
    rooms=json.loads(args.rooms.read_text())
    manifest=json.loads(args.manifest.read_text()) if args.manifest else None
    captures=[]
    paths=[args.captures/name/'frame.json' for name in manifest] if manifest is not None else args.captures.rglob('frame.json')
    for path in sorted(paths):
        frame=json.loads(path.read_text())
        if frame['fixed_frame']!='map':raise ValueError('Coverage requires map-frame captures')
        depth=np.load(path.with_name('depth.npy'),allow_pickle=False)
        captures.append((frame,depth,manifest[path.parent.name] if manifest is not None else None))
    if not captures:raise ValueError('No measured frames')
    report=dict(method='Measured RGB-D frustum and first-surface occlusion',
        rooms_sha256=hashlib.sha256(args.rooms.read_bytes()).hexdigest(),
        capture_count=len(captures),spacing_m=args.spacing,max_optical_depth_m=4.,
        surface_tolerance_m=.05,full_room_coverage_verified=False,rooms={},
        limitations=['Geometric visibility is not detection probability or proof of absence.',
          'Samples include room furniture footprints; blocked/unmeasured samples remain unobserved.',
          'Finite height planes and sampled pixels do not prove continuous volume coverage.',
          'A find task can finish early; route visibility assumes every configured view was executed.'])
    report['basis']='per_category_configured_views' if manifest is not None else 'all_captures_union_not_a_task_route'
    plot_samples={}
    for name,room in rooms.items():
        xmin,ymin,xmax,ymax=room['bounds']
        xy=[(x,y) for x in np.arange(xmin+args.spacing/2,xmax,args.spacing)
            for y in np.arange(ymin+args.spacing/2,ymax,args.spacing) if room_at(rooms,x,y)==name]
        routes={category:observation_views(room,category) for category in ('chair','cup','bottle')} if manifest is not None else {'all_captures':None}
        report['rooms'][name]={}
        for category,poses in routes.items():
            selected=[(f,d) for f,d,pose in captures if poses is None or any(np.allclose(pose,p,atol=1e-8,rtol=0) for p in poses)]
            missing=[] if poses is None else [p for p in poses if not any(
                np.allclose(pose,p,atol=1e-8,rtol=0) for _,_,pose in captures)]
            planes=[]
            for height in (.15,.45,.85):
                points=np.array([(x,y,height) for x,y in xy]);seen=np.zeros(len(xy),dtype=bool)
                for frame,depth in selected:
                    seen |= visible_samples(points,depth,frame['k'],frame['transform']['xyz'],
                        frame['transform']['xyzw'],depth_scale=.001 if frame['depth_encoding']=='16UC1' else 1.)
                planes.append(dict(height_m=height,samples=len(xy),visible=int(seen.sum()),
                    fraction=float(seen.mean()),unobserved_xy=points[~seen,:2].round(3).tolist()))
                if args.plot:plot_samples[(name,category,height)]=(points,seen)
            report['rooms'][name][category]=dict(capture_count=len(selected),
                missing_configured_poses=missing,route_captures_complete=not missing,planes=planes)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        from PIL import Image
        cjk=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
        if cjk.exists():
            font_manager.fontManager.addfont(str(cjk))
            plt.rcParams['font.family']=font_manager.FontProperties(fname=str(cjk)).get_name()
            plt.rcParams['axes.unicode_minus']=False
        resource=resolve_map(ROOT,yaml_path=args.map) if args.map else resolve_map(ROOT,map_id='aws_house_v1')
        metadata=resource['metadata']
        if abs(metadata['origin'][2])>1e-9:raise ValueError('Plot currently requires an axis-aligned map')
        occupancy=np.array(Image.open(resource['image']))
        x,y,_=metadata['origin'];resolution=metadata['resolution']
        extent=[x,x+occupancy.shape[1]*resolution,y,y+occupancy.shape[0]*resolution]
        categories=['chair','cup','bottle'] if manifest is not None else ['all_captures']
        figure,axes=plt.subplots(len(categories),3,figsize=(15,4.2*len(categories)),squeeze=False)
        for row,category in enumerate(categories):
            for column,height in enumerate((.15,.45,.85)):
                axis=axes[row,column]
                axis.imshow(occupancy,cmap='gray',vmin=0,vmax=255,extent=extent,origin='upper')
                for name,room in rooms.items():
                    points,seen=plot_samples[(name,category,height)]
                    axis.scatter(points[seen,0],points[seen,1],s=2,c='#16a085',alpha=.65)
                    axis.scatter(points[~seen,0],points[~seen,1],s=3,c='#e67e22',alpha=.7)
                    polygon=np.asarray(room['polygon']+[room['polygon'][0]])
                    axis.plot(polygon[:,0],polygon[:,1],color='#374151',linewidth=.6)
                    label=room.get('label_position',polygon[:-1].mean(axis=0))
                    axis.text(*label,room.get('display_name',name) if cjk.exists() else name,ha='center',fontsize=7,bbox=dict(facecolor='white',alpha=.7,edgecolor='none'))
                title=({'chair':'椅子','cup':'杯子','bottle':'瓶子'}.get(category,category)+f'观察路线 | 高度 {height:.2f} m') if cjk.exists() else f'{category} route | height {height:.2f} m'
                axis.set_title(title)
                axis.set_aspect('equal');axis.set_ylabel('map y (m)')
                if row==len(categories)-1:axis.set_xlabel('map x (m)')
                axis.set_xlim(min(r['bounds'][0] for r in rooms.values())-.4,max(r['bounds'][2] for r in rooms.values())+.4)
                axis.set_ylim(min(r['bounds'][1] for r in rooms.values())-.4,max(r['bounds'][3] for r in rooms.values())+.4)
        incomplete=any(not route['route_captures_complete'] for routes in report['rooms'].values() for route in routes.values())
        coverage_note='部分配置视点未采集；' if incomplete else ''
        figure.suptitle(('RGB-D实测可见性：绿色为已观察采样点，橙色为未观察采样点\n'
            +coverage_note+'未观察部分包含家具与遮挡；几何可见不等于识别完整。') if cjk.exists() else
            ('Measured RGB-D visibility: green = observed; orange = unobserved\n'
            'Unobserved includes furniture and occlusion. Geometric visibility does not prove detection completeness.'),fontsize=11)
        figure.tight_layout(rect=(0,0,1,.95),h_pad=2.);args.plot.parent.mkdir(parents=True,exist_ok=True)
        figure.savefig(args.plot,dpi=180);plt.close(figure)
    print(json.dumps({r:{c:[round(p['fraction'],3) for p in v['planes']] for c,v in routes.items()}
        for r,routes in report['rooms'].items()}))


if __name__=='__main__':main()
