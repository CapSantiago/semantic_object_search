"""Evidence views on one observed occupancy grid; no simulator data is consumed."""
import base64
from datetime import datetime, timezone
from html import escape
import io
import json
import math
from pathlib import Path
import re

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .map_export import _grid
from .objects import OBJECTS
from .rooms import belongs_to

VIEWS = {'occupancy':'占据栅格底图', 'rooms':'房间分区图', 'semantic':'物品语义与记忆图',
         'observations':'搜索观察点与朝向', 'clean_trace':'实际清扫轨迹图',
         'clean_coverage':'实际清扫覆盖图', 'relations':'家庭—房间—物品关系图'}
COLORS = {'cup':'#087f8c','bottle':'#346fca','chair':'#7972ab','bowl':'#bc6c25',
          'book':'#815bc0','sports ball':'#bc7a0c','apple':'#cd4d58'}
PHASES = {'sweeping':'#168278','supplementing':'#168278','returning':'#8168ad'}


def finite(values):
    return all(type(v) in (int,float) and math.isfinite(v) for v in values)


class Sheet:
    """Record the same drawing operations for PNG and SVG."""
    def __init__(self, width=1400, height=1140):
        self.width,self.height=width,height;self.ops=[];self.raster=None
    def text(self,x,y,text,size=21,color='#263846'):
        self.ops.append(('text',(x,y,str(text),size,color)))
    def line(self,points,color,width=3,dashed=False):
        if len(points)>1:self.ops.append(('line',(points,color,width,dashed)))
    def circle(self,x,y,r,color,filled=True):
        self.ops.append(('circle',(x,y,r,color,filled)))
    def rect(self,x,y,w,h,color):
        self.ops.append(('rect',(x,y,w,h,color)))
    def save(self,folder,view,bundle):
        # Validate/serialize and render before creating any artifact directory.
        encoded=json.dumps(bundle,ensure_ascii=False,allow_nan=False)+'\n'
        image=Image.new('RGB',(self.width,self.height),'white');draw=ImageDraw.Draw(image);fonts={}
        svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" viewBox="0 0 {self.width} {self.height}">','<rect width="100%" height="100%" fill="white"/>',f'<metadata>{escape(json.dumps({k:bundle[k]for k in ("schema","view","session_id","captured_at")},ensure_ascii=False))}</metadata>']
        if self.raster:
            raster,box=self.raster;x,y,w,h=box;image.paste(raster.resize((w,h),Image.NEAREST),(round(x),round(y)));stream=io.BytesIO();raster.save(stream,format='PNG')
            svg.append(f'<image x="{x}" y="{y}" width="{w}" height="{h}" image-rendering="pixelated" href="data:image/png;base64,{base64.b64encode(stream.getvalue()).decode()}"/>')
        for kind,args in self.ops:
            if kind=='text':
                x,y,text,size,color=args
                if size not in fonts:
                    try:fonts[size]=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',size)
                    except OSError as error:raise ValueError('缺少中文导出字体fonts-noto-cjk。')from error
                draw.text((x,y),text,font=fonts[size],fill=color);svg.append(f'<text x="{x}" y="{y}" dominant-baseline="text-before-edge" font-family="Noto Sans CJK SC,sans-serif" font-size="{size}" fill="{color}">{escape(text)}</text>')
            elif kind=='line':
                points,color,width,dashed=args
                for a,b in zip(points,points[1:]):
                    distance=math.dist(a,b)
                    if not dashed:draw.line([a,b],fill=color,width=width)
                    elif distance:
                        for start in np.arange(0,distance,14):
                            u,v=start/distance,min(start+8,distance)/distance;draw.line([(a[0]+(b[0]-a[0])*u,a[1]+(b[1]-a[1])*u),(a[0]+(b[0]-a[0])*v,a[1]+(b[1]-a[1])*v)],fill=color,width=width)
                coords=' '.join(f'{x:.3f},{y:.3f}'for x,y in points);dash=' stroke-dasharray="8 6"'if dashed else ''
                svg.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"{dash}/>')
            elif kind=='circle':
                x,y,r,color,filled=args;draw.ellipse((x-r,y-r,x+r,y+r),fill=color if filled else 'white',outline=color,width=2)
                svg.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{color if filled else "white"}" stroke="{color}" stroke-width="2"/>')
            elif kind=='rect':
                x,y,w,h,color=args;draw.rectangle((x,y,x+w,y+h),fill=color);svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{color}"/>')
        svg.append('</svg>');png=io.BytesIO();image.save(png,format='PNG');folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
        paths=[folder/(view+'.png'),folder/(view+'.svg'),folder/'snapshot.json'];paths[0].write_bytes(png.getvalue());paths[1].write_text('\n'.join(svg));paths[2].write_text(encoded);return paths


def _room_polygon(room):
    if 'polygon'in room:return room['polygon']
    x1,y1,x2,y2=room['bounds'];return [[x1,y1],[x1,y2],[x2,y2],[x2,y1]]


def _clean_layers(grid,cleaning,rooms):
    if not cleaning:raise ValueError('请选择本会话已保存的清扫任务。')
    result,plan,trace=cleaning['result'],cleaning['plan'],cleaning['trace']
    if (result.get('frame_id')!='map'or plan.get('frame_id')!='map'or trace.get('frame_id')!='map'
            or plan.get('origin')!=[grid['origin'][k]for k in ('x','y','yaw')]
            or plan.get('resolution_m')!=grid['resolution']):raise ValueError('清扫记录与当前底图坐标不一致。')
    base=_grid(grid);eligible=np.asarray(cleaning['eligible']);covered=np.asarray(cleaning['covered']);original=np.asarray(cleaning['static_map'])
    if (eligible.shape!=base.shape or covered.shape!=base.shape or original.shape!=base.shape
            or eligible.dtype!=bool or covered.dtype!=bool or not np.array_equal(original,base)
            or np.any(covered&~eligible)):raise ValueError('清扫栅格不匹配，拒绝混用地图或预测覆盖。')
    coverage=result.get('coverage')or{}
    if (int(eligible.sum())!=coverage.get('eligible_cells')or int(covered.sum())!=coverage.get('covered_cells')):raise ValueError('保存的实际刷盘栅格与覆盖报告不一致。')
    room=result.get('room')
    if room not in rooms:raise ValueError('清扫缺少原房间分区。')
    floor=np.zeros(base.shape,dtype=bool);origin=grid['origin'];c,s=math.cos(origin['yaw']),math.sin(origin['yaw']);r=grid['resolution']
    for y,x in np.argwhere(base==0):
        dx,dy=(int(x)+.5)*r,(int(y)+.5)*r
        floor[y,x]=belongs_to(rooms,room,origin['x']+c*dx-s*dy,origin['y']+s*dx+c*dy)
    if int(floor.sum())!=coverage.get('floor_cells')or np.any(eligible&~floor):raise ValueError('清扫房间范围与原始地面分母不一致。')
    return floor,eligible,covered


def _relations(sheet,memory,graph,rooms):
    if (not graph or graph.get('metadata',{}).get('namespace')!=memory['namespace']
            or graph.get('metadata',{}).get('source_revision')!=memory['revision']):raise ValueError('场景图与当前物品记忆修订不一致。')
    nodes={n['id']:n for n in graph['nodes']};parents={e['target']:e['source']for e in graph['edges']}
    room_nodes={n['attributes'].get('metadata',{}).get('room_id'):n for n in nodes.values()if n['attributes'].get('metadata',{}).get('room_id')}
    root_ids={parents.get(room_nodes[k]['id'])for k in rooms if k in room_nodes}
    if (set(rooms)-set(room_nodes)or len(root_ids)!=1 or next(iter(root_ids))not in nodes):
        raise ValueError('场景图缺少家庭—房间归属边，不能生成假关系。')
    root=nodes[next(iter(root_ids))]
    objects={n['attributes'].get('metadata',{}).get('memory_id'):n for n in nodes.values()if n['attributes'].get('metadata',{}).get('memory_id')}
    groups={key:[]for key in rooms}
    for item in memory['objects']:
        node=objects.get(item['id']);attrs=(node or{}).get('attributes',{});meta=attrs.get('metadata',{})
        if (not node or attrs.get('position')!=[item[k]for k in ('x','y','z')]
                or any(meta.get(k)!=item.get(k)for k in ('state','task_id','evidence'))):raise ValueError('场景图物品字段与记录不一致。')
        parent=nodes.get(parents.get(node['id']),{});room=parent.get('attributes',{}).get('metadata',{}).get('room_id')
        if room not in groups:raise ValueError('物品缺少实际场景图房间归属边，不能生成假关系。')
        groups[room].append(item)
    y=185;root_y=185+sum(max(65,len(v)*42)for v in groups.values())/2
    sheet.rect(45,root_y-24,140,48,'#153c4c');sheet.text(77,root_y-17,root['attributes'].get('name','家庭'),25,'white')
    for key,items in groups.items():
        block=max(65,len(items)*42);cy=y+block/2;sheet.line([(185,root_y),(350,cy)],'#b9cbd4',2);sheet.text(370,cy-16,rooms[key].get('display_name',key),22)
        for i,item in enumerate(items):
            iy=y+i*42+21;pending=item['state']=='needs_verification'or item.get('classification_conflict');color='#bc7a0c'if pending else COLORS.get(item['category'],'#557684');sheet.line([(530,cy),(630,iy)],'#cbd9df',2)
            sheet.circle(642,iy,5,color,not pending);sheet.text(660,iy-15,f"{OBJECTS[item['category']].name} · M-{item['id'][:6]} · {'待核验'if pending else'最后观测'}",19,color)
        y+=block
    sheet.text(55,sheet.height-110,'连线直接取自当前真实Spark-DSG房间归属边；不表示支撑、邻近或移动后的物理身份。',19)


def export_view(folder,view,grid,state,*,memory=None,task=None,cleaning=None,graph=None,filters=None,captured_at=None):
    if view not in VIEWS:raise ValueError('未知地图导出视图。')
    base=_grid(grid)
    if grid.get('frame_id')!='map'or state.get('rooms_frame_id')!='map'or state.get('mode')=='mapping':raise ValueError('请在同一map的定位导航模式导出。')
    if view in ('semantic','relations'):
        if (not memory or not memory.get('available')or memory.get('truncated')
                or not grid.get('memory_namespace')or memory.get('namespace')!=grid['memory_namespace']):raise ValueError('物品记忆缺失、截断或地图版本不一致。')
    if view=='observations'and(not task or task.get('kind')!='search'or (task.get('result')or{}).get('frame_id')!='map'or not (task.get('result')or{}).get('viewpoint_diagnostics')):raise ValueError('请选择本会话已保存的map寻物或计数任务。')
    rooms=state.get('rooms')or{}
    if view.startswith('clean_'):floor,eligible,covered=_clean_layers(grid,cleaning,rooms)
    height=max(1140,350+sum(max(65,sum(v['room']==k for v in memory['objects'])*42)for k in rooms))if view=='relations' else 1140
    sheet=Sheet(height=height);timestamp=captured_at or datetime.now(timezone.utc).isoformat();task=task or{};result=task.get('result')or{}
    bundle=dict(schema='shared_map_view_v1',view=view,captured_at=timestamp,session_id=state.get('session_id'),occupancy=grid,rooms=rooms,rooms_file=state.get('rooms_file'),memory=memory if view in ('semantic','relations')else None,scene_graph=graph if view=='relations'else None,task=task,filters=filters or{},clock_seconds=state.get('telemetry',{}).get('clock_seconds'),cleaning=cleaning,notes=['One observed occupancy base, derived views only.','History and last-observed records are not a live inventory.','Search headings/visible samples do not prove full-room recognition coverage.'])
    bundle['time_basis']='captured_at is snapshot export time; each source retains its original stamp, revision and session; no synchronized interpolation'
    telemetry=state.get('telemetry')or{};robot=telemetry.get('pose')if telemetry.get('pose_fresh')else None
    if robot and(robot.get('frame_id')!='map'or robot.get('fresh')is False or not finite([robot.get(k)for k in ('x','y','yaw')])):robot=None
    bundle['robot_pose']=robot
    sheet.text(55,20,VIEWS[view]+' · 证据快照',32);sheet.text(55,68,f"导出 {timestamp}  |  frame=map / m  |  分辨率 {grid['resolution']:.3f} m",19);sheet.text(55,100,'来源会话 '+str(state.get('session_id','—')),18)
    sheet.text(55,130,('任务 '+str(task.get('id'))+' · '+str(task.get('state'))+' · '+str(result.get('status','')) )if task else '无所选任务；静态分区为配置，物品位置来自实际观测',18)
    if view=='relations':
        _relations(sheet,memory,graph,rooms);return sheet.save(folder,view,bundle)
    shade=np.where(base<0,205,np.where(base>=65,35,248)).astype(np.uint8);rgb=np.repeat(shade[:,:,None],3,axis=2)
    if view=='clean_coverage':
        rgb[floor&~eligible]=(220,228,237);rgb[eligible]=(255,242,198);rgb[covered]=(173,225,207)
    scale=min(1290/grid['width'],810/grid['height']);w,h=round(grid['width']*scale),round(grid['height']*scale);left=(1400-w)/2;top=175;sheet.raster=(Image.fromarray(rgb[::-1],'RGB'),(left,top,w,h));origin=grid['origin'];c,s=math.cos(origin['yaw']),math.sin(origin['yaw'])
    def project(point):
        dx,dy=point[0]-origin['x'],point[1]-origin['y'];return(left+(c*dx+s*dy)/grid['resolution']*scale,top+(grid['height']-(-s*dx+c*dy)/grid['resolution'])*scale)
    if view!='occupancy':
        for key,room in rooms.items():
            polygon=_room_polygon(room);color=room.get('color','#557684');color=color if re.fullmatch('#[0-9a-fA-F]{6}',color)else'#557684'
            sheet.line([project(p)for p in polygon+[polygon[0]]],color,2)
            if view=='rooms':
                xy=room.get('label_position',np.mean(polygon,axis=0));x,y=project(xy);sheet.text(x-35,y-14,room.get('display_name',key),23,color)
    if view=='semantic':
        for item in memory['objects']:
            if not finite([item.get(k)for k in ('x','y','z')]):raise ValueError('物品位置不是有效map坐标。')
            x,y=project([item['x'],item['y']]);pending=item['state']=='needs_verification'or item.get('classification_conflict');color='#bc7a0c'if pending else COLORS.get(item['category'],'#557684');sheet.circle(x,y,7,color,not pending)
            if not pending:sheet.text(x+10,y-20,f"{OBJECTS[item['category']].name} M-{item['id'][:6]}",17,color)
        selected=(task.get('kind')=='search' and result.get('frame_id')=='map'
                  and all(result.get(k)==v for k,v in (filters or{}).items()))
        if selected:
            for item in result.get('objects',[]):
                if not finite([item.get(k)for k in ('x','y','z')]):raise ValueError('任务物品位置无效。')
                x,y=project([item['x'],item['y']]);sheet.circle(x,y,10,'#225d82',False);sheet.text(x+12,y+6,'#'+str(item['id']),19,'#225d82')
        sheet.text(55,1000,f"历史记录 {len(memory['objects'])} 条；空心橙圈=待核验，实心分类色=最后观测（并非当前确认）。",21)
        sheet.text(55,1035,'标签优先显示最后观测记录；全部ID、位置、时间和证据保存在原始JSON。',20)
        if selected:sheet.text(55,1070,f"蓝圈 #编号 = 所选任务确认 {result.get('count',0)} 个；M-编号是跨任务空间记录，两者不是物理身份。",19)
    elif view=='observations':
        sites={};valid=0
        for d in result['viewpoint_diagnostics']:
            pose=d['pose'];key=tuple(pose[:2]);sites.setdefault(key,len(sites)+1);nominal=project(pose);sheet.circle(*nominal,4,'#889298',False)
            if not(d.get('reached')and d.get('dwell_completed')and d.get('synchronized_frames',0)>0):continue
            arrival=d.get('arrival_map_pose')or{};xyz,q=arrival.get('xyz',[]),arrival.get('xyzw',[])
            if arrival.get('frame_id')!='map'or len(xyz)!=3 or len(q)!=4 or not finite(xyz+q)or not .99<=sum(v*v for v in q)<=1.01:raise ValueError('观察缺少有效实测到达位姿，不能把目标点当实走。')
            yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2));xy=project(xyz);end=project([xyz[0]+.55*math.cos(yaw),xyz[1]+.55*math.sin(yaw)]);sheet.line([xy,end],'#168278',3);sheet.circle(*xy,5,'#168278');valid+=1
        for xy,i in sites.items():
            x,y=project(xy);sheet.text(x+12,y-32,'P'+str(i),19)
        sheet.text(55,1000,f"配置 {len(sites)} 处 / {len(result['viewpoint_diagnostics'])} 方向；有效观察 {valid}。灰圈=预设，绿点/短线=实测到达与朝向。",20)
        sheet.text(55,1035,'仅展示已执行观察，不等于全房可见覆盖或识别无漏检；未执行/缺数据的回退点保持灰色。',19)
    elif view.startswith('clean_'):
        previous=None
        for row in cleaning['trace']['samples']:
            pose=row['pose']
            if not finite(pose):raise ValueError('轨迹包含无效坐标。')
            if previous is not None and row.get('connected'):sheet.line([project(previous['pose']),project(pose)],PHASES.get(row.get('stage'),'#889298'),3)
            previous=row
        samples=cleaning['trace']['samples']
        if samples:
            start=project(samples[0]['pose']);end=project(samples[-1]['pose']);sheet.circle(*start,6,'#557684');sheet.circle(*end,10,'#8168ad',False);sheet.text(start[0]+13,start[1]-27,'S 起点',17,'#557684');sheet.text(end[0]+13,end[1]+6,'E 终点',17,'#8168ad')
        coverage=cleaning['result']['coverage'];sheet.text(55,1000,f"原分母 {coverage['eligible_cells']} 格；实扫 {coverage['covered_cells']} 格 / {coverage['coverage_fraction']*100:.2f}%；终态 {cleaning['result']['status']}",21)
        sheet.text(55,1035,'灰=前往房间，绿=清扫，紫=返航；定位断点不连线；仅实际轨迹计覆盖。',20)
        if view=='clean_coverage':sheet.text(55,1070,'浅绿=实扫，浅黄=可扫未覆盖，灰蓝=原可达模型排除地面；不因动态障碍缩小分母。',19)
    else:sheet.text(55,1000,'白=已知可通行，黑=占据，灰=未知；房间边界来自配置，不是新增墙体。',21)
    if robot and view in ('occupancy','rooms','semantic'):
        xy=project([robot['x'],robot['y']]);end=project([robot['x']+.4*math.cos(robot['yaw']),robot['y']+.4*math.sin(robot['yaw'])]);sheet.circle(*xy,8,'#0b6058');sheet.line([xy,end],'#0b6058',4);sheet.text(xy[0]+12,xy[1]+5,'R 源位置',17,'#0b6058')
    sheet.text(55,1105,'快照不自动更新；不同源保留各自原始时间，不插值、不使用场景真值补图。',18)
    return sheet.save(folder,view,bundle)
