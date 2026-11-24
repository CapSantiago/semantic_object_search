"""Evidence snapshots of native global navigation costs on the shared map."""
import base64
from datetime import datetime, timezone
from html import escape
import io
import json
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont


COLORS = {'occupied': '#cf2f34', 'inscribed': '#e57320', 'inflated': '#f5b132',
          'unknown': '#6f7d8b', 'plan': '#346fca', 'trace': '#168278', 'robot': '#0b6058'}


def _grid(value):
    if not value.get('available'):
        raise ValueError('地图尚不可用。')
    width, height = value.get('width'), value.get('height')
    if (type(width) is not int or type(height) is not int or min(width, height) <= 0
            or width * height > 16000000):
        raise ValueError('地图尺寸无效。')
    origin = value.get('origin') or {}
    numbers = [value.get('resolution'), *(origin.get(k) for k in ('x', 'y', 'yaw'))]
    if not all(isinstance(n, (float, int)) and math.isfinite(n) for n in numbers) or numbers[0] <= 0:
        raise ValueError('地图坐标无效。')
    data = np.asarray(value.get('data', []))
    if data.size != width * height or not np.issubdtype(data.dtype, np.number):
        raise ValueError('地图栅格不完整。')
    if not np.all(np.isfinite(data)) or np.any((data < -1) | (data > 100)):
        raise ValueError('地图编码无效。')
    return data.reshape(height, width)


def compose_raster(grid, costs):
    """Sample cell centres using each native origin, yaw and resolution."""
    occupancy, native = _grid(grid), _grid(costs)
    if grid.get('frame_id') != 'map' or costs.get('frame_id') != grid['frame_id']:
        raise ValueError('仅导出同一map坐标系的全局代价图。')
    rows, cols = np.indices(occupancy.shape)
    x, y = (cols + .5) * grid['resolution'], (rows + .5) * grid['resolution']
    origin, target = grid['origin'], costs['origin']
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    dx, dy = origin['x'] + c*x - s*y - target['x'], origin['y'] + s*x + c*y - target['y']
    c, s = math.cos(target['yaw']), math.sin(target['yaw'])
    cx, cy = np.floor((c*dx+s*dy)/costs['resolution']).astype(int), np.floor((-s*dx+c*dy)/costs['resolution']).astype(int)
    valid = (cx >= 0) & (cx < costs['width']) & (cy >= 0) & (cy < costs['height'])
    sampled = np.full(occupancy.shape, -2, dtype=int)
    sampled[valid] = native[cy[valid], cx[valid]]
    base = np.where(occupancy < 0, 205, np.where(occupancy >= 65, 35, 248)).astype(np.uint8)
    rgb = np.repeat(base[:, :, None], 3, axis=2)
    rgba = np.zeros((*sampled.shape, 4), dtype=np.uint8)
    for mask, color, alpha in ((sampled == -1, (111,125,139), 100),
                               (sampled == 100, (207,47,52), 220),
                               (sampled == 99, (229,115,32), 200)):
        rgba[mask] = (*color, alpha)
    mask = (sampled > 0) & (sampled < 99)
    rgba[mask, :3] = (245,177,50)
    rgba[mask, 3] = np.rint(35 + sampled[mask]*1.1).astype(np.uint8)
    alpha = rgba[:, :, 3:4].astype(float) / 255
    rgb = np.rint(rgb*(1-alpha) + rgba[:, :, :3]*alpha).astype(np.uint8)
    return Image.fromarray(rgb[::-1], 'RGB')


def export_costmap(folder, grid, costs, state, captured_at=None):
    """Write one immutable source bundle plus PNG/SVG; never mutate navigation."""
    telemetry = state.get('telemetry') or {}
    clock, stamp = telemetry.get('clock_seconds'), costs.get('stamp_seconds')
    if (state.get('mode') == 'mapping' or not costs.get('fresh') or costs.get('source') != '/global_costmap/costmap'
            or not isinstance(clock, (int, float)) or not isinstance(stamp, (int, float))
            or not math.isfinite(clock) or not math.isfinite(stamp) or not -.1 <= clock-stamp <= 2.5):
        raise ValueError('全局代价图未就绪或已过期，请等待新帧后导出。')
    raster = compose_raster(grid, costs)
    timestamp = captured_at or datetime.now(timezone.utc).isoformat()
    task = state.get('task') or {}
    progress = task.get('progress') or task.get('result') or {}
    plan = telemetry.get('navigation_path') or {}
    plan_ok = (plan.get('fresh') and plan.get('frame_id') == 'map'
               and isinstance(plan.get('stamp_seconds'), (float,int))
               and -.1 <= clock-plan['stamp_seconds'] <= 2.5)
    trace = progress.get('trace_preview', []) if task.get('kind') == 'clean' else []
    pose = telemetry.get('pose') if telemetry.get('pose_fresh') else None
    if pose and pose.get('frame_id') != 'map':
        pose = None
    captured = dict(schema='navigation_cost_snapshot_v1', captured_at=timestamp,
                    session_id=state.get('session_id'), mode=state.get('mode'),
                    task=dict(id=task.get('id'), state=task.get('state'), kind=task.get('kind')),
                    clock_seconds=clock, occupancy=grid, costmap=costs,
                    navigation_path=plan if plan_ok else None, robot_pose=pose,
                    trace_preview=trace, trace_sampling='UI measured samples, not full-resolution trajectory',
                    rooms_file=state.get('rooms_file'),
                    notes=['Native global cost encoding; not object classes.',
                           'Snapshot time; not a continuously updated map.'])
    width, height = 1400, 1140
    margin, top, area_h = 55, 155, 815
    scale = min((width-2*margin)/grid['width'], area_h/grid['height'])
    image_w, image_h = round(grid['width']*scale), round(grid['height']*scale)
    left = (width-image_w)/2
    origin = grid['origin']; c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    def project(point):
        dx,dy = point[0]-origin['x'], point[1]-origin['y']
        return left+(c*dx+s*dy)/grid['resolution']*scale, top+(grid['height']-(-s*dx+c*dy)/grid['resolution'])*scale
    operations = []
    def text(x,y,value,size=21,color='#263846'):
        operations.append(('text',(x,y,value,size,color)))
    def line(points,color,stroke=3,dashed=False):
        if len(points)>1:operations.append(('line',(points,color,stroke,dashed)))
    text(55,22,'导航代价地图 · 瞬时快照',32)
    text(55,68,f"采集：{timestamp}  |  frame=map  |  分辨率 {grid['resolution']:.3f} m",19)
    text(55,99,f"源：/global_costmap/costmap  sim={stamp:.3f}s  |  会话：{state.get('session_id','—')}",18)
    text(55,126,f"任务：{task.get('id','无')}  ·  {task.get('state','idle')}  |  单位 m；底图原点朝向 {origin['yaw']:.3f} rad",18)
    def points(values):
        return [project(v) for v in values if isinstance(v,(list,tuple)) and len(v)>=2
                and all(isinstance(x,(int,float)) and math.isfinite(x) for x in v[:2])]
    if plan_ok:line(points(plan.get('points',[])),COLORS['plan'],4,True)
    segment=[]
    for row in trace:
        if not isinstance(row,(list,tuple)):
            line(points(segment),COLORS['trace'],4);segment=[];continue
        if len(row)<4 or row[3]is not True:
            line(points(segment),COLORS['trace'],4);segment=[]
        segment.append(row)
    line(points(segment),COLORS['trace'],4)
    if pose and all(isinstance(pose.get(k),(int,float)) and math.isfinite(pose[k]) for k in ('x','y','yaw')):
        x,y=project((pose['x'],pose['y'])); end=project((pose['x']+.4*math.cos(pose['yaw']),pose['y']+.4*math.sin(pose['yaw'])))
        operations.append(('circle',(x,y,7,COLORS['robot'])))
        line([(x,y),end],COLORS['robot'],5)
    labels=[('occupied','障碍占据 100'),('inscribed','禁行膨胀 99'),('inflated','通行代价 1–98'),('unknown','未知 -1')]
    for i,(key,label) in enumerate(labels):
        x=55+i*328;operations.append(('rect',(x,995,22,22,COLORS[key])));text(x+32,988,label,22)
    line([(55,1044),(100,1044)],COLORS['plan'],4,True);text(113,1028,'当前规划（有效时）',21)
    line([(460,1044),(505,1044)],COLORS['trace'],4);text(518,1028,'清扫实测轨迹（界面采样）',21)
    text(55,1072,'白/灰/黑为共用占据底图；颜色表达导航代价。动态障碍与膨胀并非物品语义类别。',20)
    text(55,1103,'快照保留采集时刻的有效帧；路线及机器人分别按自身源戳记录，未进行同步插值。',18)
    output=Image.new('RGB',(width,height),'white');output.paste(raster.resize((image_w,image_h),Image.NEAREST),(round(left),top))
    draw=ImageDraw.Draw(output)
    fonts={}
    def font(size):
        if size not in fonts:
            try:
                fonts[size]=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',size)
            except OSError as error:
                raise ValueError('缺少中文导出字体，请安装fonts-noto-cjk。') from error
        return fonts[size]
    buffer=io.BytesIO();raster.save(buffer,format='PNG')
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
         '<rect width="100%" height="100%" fill="white"/>',
         f'<metadata>{escape(json.dumps({k:captured[k] for k in ("schema","session_id","captured_at","task")},ensure_ascii=False))}</metadata>',
         f'<image x="{left:.3f}" y="{top}" width="{image_w}" height="{image_h}" image-rendering="pixelated" href="data:image/png;base64,{base64.b64encode(buffer.getvalue()).decode()}"/>']
    for kind,args in operations:
        if kind=='text':
            x,y,value,size,color=args;draw.text((x,y),value,font=font(size),fill=color)
            svg.append(f'<text x="{x}" y="{y}" dominant-baseline="text-before-edge" font-family="Noto Sans CJK SC,sans-serif" font-size="{size}" fill="{color}">{escape(value)}</text>')
        elif kind=='line':
            vertices,color,stroke,dashed=args
            # Dash both output formats; do not depict native plans as actual travel.
            for a,b in zip(vertices,vertices[1:]):
                distance=math.dist(a,b)
                if not dashed:draw.line([a,b],fill=color,width=stroke)
                elif distance>0:
                    for start in np.arange(0,distance,14):
                        u,v=start/distance,min(start+8,distance)/distance
                        draw.line([(a[0]+(b[0]-a[0])*u,a[1]+(b[1]-a[1])*u),(a[0]+(b[0]-a[0])*v,a[1]+(b[1]-a[1])*v)],fill=color,width=stroke)
            coordinates=' '.join(f'{x:.3f},{y:.3f}'for x,y in vertices)
            dash=' stroke-dasharray="8 6"'if dashed else ''
            svg.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{stroke}"{dash}/>')
        elif kind=='rect':
            x,y,w,h,color=args;draw.rectangle((x,y,x+w,y+h),fill=color)
            svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{color}"/>')
        elif kind=='circle':
            x,y,r,color=args;draw.ellipse((x-r,y-r,x+r,y+r),fill=color)
            svg.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{r}" fill="{color}"/>')
    svg.append('</svg>')
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'snapshot.json').write_text(json.dumps(captured,ensure_ascii=False,allow_nan=False)+'\n')
    output.save(folder/'navigation_cost.png')
    (folder/'navigation_cost.svg').write_text('\n'.join(svg))
    return [folder/'navigation_cost.png',folder/'navigation_cost.svg',folder/'snapshot.json']
