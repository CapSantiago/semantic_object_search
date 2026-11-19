"""Room coverage from an occupancy grid and measured map-frame poses."""
from base64 import b64encode
from html import escape
import math
import re

import numpy as np
from scipy.ndimage import binary_dilation, convolve, distance_transform_edt, label

from .rooms import belongs_to


SWEEP_WIDTH_M = .50
LANE_SPACING_M = .35
CENTER_CLEARANCE_M = .32
TARGET_COVERAGE = .85
RETURN_POSITION_M = .15
RETURN_YAW_RAD = .15


def parse_clean_command(text, rooms):
    """Return a single room for a cleaning command, or None for other tasks."""
    action = r'(?:清扫|打扫(?:卫生)?|扫地|扫个地|扫一下)'
    if not re.search(action, text):
        return None
    text = re.sub(r'[，。！？,.!?]+$', '', text.strip()).strip()
    for room_id, room in rooms.items():
        aliases = '|'.join(re.escape(name) for name in (*room['aliases'], room_id))
        destination = rf'(?:{aliases})'
        prefix = r'(?:请(?:你)?\s*)?(?:帮我\s*)?'
        for phrase in (rf'(?:去|到|把)?\s*{destination}\s*{action}(?:一下)?',
                       rf'{action}(?:一下)?\s*{destination}'):
            if re.fullmatch(prefix+phrase, text, flags=re.IGNORECASE):
                return room_id
    raise ValueError('请只指定一个要清扫的房间，例如“请去健身区扫地”。')


def runs(row):
    edges = np.flatnonzero(np.diff(np.pad(row.astype(np.int8), 1)))
    return zip(edges[::2], edges[1::2])


def lane_groups(intervals, pitch):
    """Continue a cell while its free interval has a one-to-one successor."""
    groups, previous = [], []
    for y in sorted({row for row,_,_ in intervals}):
        row_intervals = [lane for lane in intervals if lane[0]==y]
        if previous and y-previous[0][0][0]!=pitch:
            previous = []
        connections = [[index for index,((_,a,b),_) in enumerate(previous)
                        if left <= b and right >= a]
                       for _,left,right in row_intervals]
        current = []
        for lane, parents in zip(row_intervals,connections):
            if len(parents)==1 and sum(parents[0] in matches for matches in connections)==1:
                group = previous[parents[0]][1]
            else:
                group = []
                groups.append(group)
            group.append(lane)
            current.append((lane,group))
        previous = current
    return groups


class RoomSweep:
    def __init__(self, data, width, height, resolution, origin, rooms, room_id, start):
        if not (0 < width*height <= 2_000_000 and .02 <= resolution <= .10):
            raise ValueError('清扫地图的尺寸或分辨率超出支持范围。')
        self.resolution = resolution
        self.origin = origin
        yaw = origin[2]
        self.cosine, self.sine = math.cos(yaw), math.sin(yaw)
        self.occupancy = np.asarray(data, dtype=np.int8).reshape(height,width)
        self.free = self.occupancy == 0
        clearance = distance_transform_edt(np.pad(self.free, 1))[1:-1,1:-1]*resolution
        navigable = clearance >= CENTER_CLEARANCE_M + resolution/2
        components, _ = label(navigable)
        x,y = self.index(*start[:2])
        reach = math.ceil(.25/resolution)
        near = [(math.hypot(col-x,row-y), int(components[row,col]))
                for row in range(max(0,y-reach), min(height,y+reach+1))
                for col in range(max(0,x-reach), min(width,x+reach+1))
                if components[row,col]]
        if not near or min(near)[0]*resolution > .25:
            raise ValueError('起点附近没有可安全通行的已知区域。')
        reachable = components == min(near)[1]
        region = np.zeros((height,width), dtype=bool)
        xmin,ymin,xmax,ymax = rooms[room_id]['bounds']
        corners = [self.index(ix,iy) for ix in (xmin,xmax) for iy in (ymin,ymax)]
        for row in range(max(0,min(y for _,y in corners)), min(height,max(y for _,y in corners)+1)):
            for col in range(max(0,min(x for x,_ in corners)), min(width,max(x for x,_ in corners)+1)):
                region[row,col] = belongs_to(rooms,room_id,*self.point(col,row))
        self.floor = self.free & region
        self.centers = reachable & region
        self.radius = SWEEP_WIDTH_M/2
        self.reach = math.ceil(self.radius/resolution)
        offset = np.arange(-self.reach,self.reach+1)
        self.disk = np.hypot(offset[:,None],offset[None,:])*resolution <= self.radius
        self.eligible = self.floor & binary_dilation(self.centers, structure=self.disk)
        self.floor_cells = int(self.floor.sum())
        self.eligible_cells = int(self.eligible.sum())
        if not self.eligible_cells:
            raise ValueError('选定房间没有与起点连通的可清扫地面。')
        self.covered = np.zeros_like(region)
        self.repeated = np.zeros_like(region)
        self.last_touch = np.full(region.shape,-np.inf)

    def index(self, x, y):
        dx,dy = x-self.origin[0],y-self.origin[1]
        return (math.floor((self.cosine*dx+self.sine*dy)/self.resolution),
                math.floor((-self.sine*dx+self.cosine*dy)/self.resolution))

    def point(self, col, row):
        x,y = (col+.5)*self.resolution,(row+.5)*self.resolution
        return x*self.cosine-y*self.sine+self.origin[0], x*self.sine+y*self.cosine+self.origin[1]

    def route(self, start, available=None):
        """Choose horizontal or vertical lanes using a coverage/travel estimate."""
        available = self.centers if available is None else self.centers & available
        if not available.any():
            raise ValueError('当前代价地图中没有可通行的清扫路线。')
        pitch = max(1,round(LANE_SPACING_M/self.resolution))
        short = math.ceil(.20/self.resolution)
        best_score, best_route = None, []
        for transpose, centers in ((False,available),(True,available.T)):
            rows = np.flatnonzero(centers.any(axis=1))
            for offset in range(pitch):
                lane_rows = range(int(rows[0])+offset,int(rows[-1])+1,pitch)
                intervals = [(int(y),int(a),int(b)-1) for y in lane_rows
                             for a,b in runs(centers[y]) if b-a >= short]
                if not intervals:
                    continue
                planned = np.zeros_like(centers)
                for row,a,b in intervals:
                    planned[row,a:b+1] = True
                predicted = binary_dilation(planned.T if transpose else planned,structure=self.disk)
                missing = self.eligible_cells-int((predicted & self.eligible).sum())
                groups = []
                for lanes in lane_groups(intervals,pitch):
                    options = []
                    for order in (lanes,lanes[::-1]):
                        for flip in (False,True):
                            options.append(self.lane_poses(order,transpose,flip))
                    groups.append(options)
                route, previous, length = [], start[:2], 0.
                while groups:
                    _,index,chosen = min(
                        (math.dist(previous,option[0][:2]),i,option)
                        for i,options in enumerate(groups) for option in options)
                    route.extend(chosen)
                    length += math.dist(previous,chosen[0][:2])
                    length += sum(math.dist(a[:2],b[:2]) for a,b in zip(chosen,chosen[1:]))
                    previous = chosen[-1][:2]
                    groups.pop(index)
                score = missing*self.resolution**2 + .045*(length + len(intervals)*.45)
                if best_score is None or score < best_score:
                    best_score,best_route = score,route
        if not best_route:
            raise ValueError('选定房间没有足够长的可清扫路线。')
        return best_route

    def lane_poses(self, lanes, transpose, flip):
        route = []
        for index,(row,a,b) in enumerate(lanes):
            if (index % 2 == 1) != flip:
                a,b = b,a
            first = self.point(row,a) if transpose else self.point(a,row)
            last = self.point(row,b) if transpose else self.point(b,row)
            yaw = math.atan2(last[1]-first[1],last[0]-first[0])
            route.extend([(*first,yaw),(*last,yaw)])
        return route

    def lane_segments(self, first, last, available):
        """Split a planned lane where the current costmap blocks the centre."""
        col,row = self.index(*first[:2])
        end_col,end_row = self.index(*last[:2])
        vertical = col==end_col
        a,b = (row,end_row) if vertical else (col,end_col)
        low,high = sorted((a,b))
        cells = available[low:high+1,col] if vertical else available[row,low:high+1]
        segments = [(i+low,j+low-1) for i,j in runs(cells)
                    if (j-i-1)*self.resolution >= .14]
        if a>b:
            segments = [(end,start) for start,end in reversed(segments)]
        def center(index):
            return self.point(col,index) if vertical else self.point(index,row)
        return [((*center(start),first[2]),(*center(end),last[2])) for start,end in segments]

    def touch(self, x, y, stamp):
        """Mark only eligible cells inside the physical sweep width."""
        col,row = self.index(x,y)
        height,width = self.eligible.shape
        u,v = max(0,col-self.reach),max(0,row-self.reach)
        right,bottom = min(width,col+self.reach+1),min(height,row+self.reach+1)
        if u>=right or v>=bottom:
            return
        region = np.s_[v:bottom,u:right]
        gy,gx = np.mgrid[v:bottom,u:right]
        dx,dy = x-self.origin[0],y-self.origin[1]
        local_x = self.cosine*dx+self.sine*dy
        local_y = -self.sine*dx+self.cosine*dy
        hit = (np.hypot((gx+.5)*self.resolution-local_x,
                        (gy+.5)*self.resolution-local_y) <= self.radius) & self.eligible[region]
        visited = self.last_touch[region]
        self.repeated[region] |= hit & np.isfinite(visited) & (stamp-visited > 1.0)
        visited[hit] = stamp
        self.covered[region] |= hit

    def repair_goal(self, current, attempted, available=None):
        remaining = self.eligible & ~self.covered
        gain = convolve(remaining.astype(np.int32),self.disk.astype(np.int32),mode='constant')
        centers = self.centers if available is None else self.centers & available
        gain[~centers] = 0
        for x,y in attempted:
            col,row = self.index(x,y)
            span = math.ceil(.25/self.resolution)
            gain[max(0,row-span):row+span+1,max(0,col-span):col+span+1] = 0
        # Prefer the closest of the best repairs to avoid repeated room crossings.
        best = np.argwhere(gain==gain.max())
        if gain.max()*self.resolution**2 < .02:
            return None
        row,col = min(best,key=lambda cell:math.dist(current[:2],self.point(cell[1],cell[0])))
        x,y = self.point(col,row)
        return x,y,math.atan2(y-current[1],x-current[0])

    def metrics(self):
        area = self.resolution**2
        swept = int(self.covered.sum())
        repeated = int(self.repeated.sum())
        return dict(floor_cells=self.floor_cells, eligible_cells=self.eligible_cells,
                    covered_cells=swept, uncovered_cells=self.eligible_cells-swept,
                    excluded_floor_cells=self.floor_cells-self.eligible_cells,
                    coverage_fraction=swept/self.eligible_cells,
                    covered_area_m2=round(swept*area,4),
                    repeated_area_m2=round(repeated*area,4),
                    repeated_coverage_fraction=repeated/self.eligible_cells)

    def save_map(self, path, trace, title):
        import cv2

        height,width = self.occupancy.shape
        rgb = np.full((height,width,3),[196,204,210],dtype=np.uint8)
        rgb[self.free] = [250,250,249]
        rgb[self.occupancy>0] = [55,65,81]
        rgb[self.floor] = [209,220,225]
        rgb[self.eligible] = [253,222,157]
        rgb[self.covered] = [148,221,194]
        encoded, bitmap = cv2.imencode('.png',rgb[::-1,:,::-1])
        if not encoded:
            raise ValueError('无法生成清扫地图的栅格底图。')
        bitmap = b64encode(bitmap.tobytes()).decode()
        paths = []
        previous = None
        for row in trace:
            x,y = self.index(*row['pose'][:2])
            # The trace is split at missing or discontinuous localization.
            paths.append(f"{'L' if previous is not None and row['connected'] else 'M'}{x+.5},{height-y-.5}")
            previous = row
        path.write_text(
            f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
            f'viewBox="0 0 {width} {height}"><title>{escape(title)}</title>'
            f'<image width="{width}" height="{height}" xlink:href="data:image/png;base64,{bitmap}"/>'
            f'<path d="{" ".join(paths)}" fill="none" stroke="#164e63" stroke-width="1.7" '
            f'stroke-linejoin="round"/></svg>\n')
