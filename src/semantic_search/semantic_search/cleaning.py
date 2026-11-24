"""Room coverage from an occupancy grid and measured map-frame poses."""
from base64 import b64encode
from html import escape
import math
import re

import numpy as np
from scipy.ndimage import (binary_dilation, binary_erosion, convolve,
                           distance_transform_edt, generate_binary_structure, label)
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from .rooms import belongs_to


SWEEP_WIDTH_M = .50
LANE_SPACING_M = .35
CENTER_CLEARANCE_M = .32
TRACKING_MARGIN_CELLS = 1
TARGET_COVERAGE = .80
RETURN_POSITION_M = .15
RETURN_YAW_RAD = .15


def coverage_accepted(sweep, route):
    """A global percentage cannot hide an unvisited planned stripe."""
    return (sweep.metrics()['coverage_fraction']>=TARGET_COVERAGE and
            sweep.spatial_coverage(route)['completed'])


def transit_timeout(start, target, minimum=40., *, path_distance_m=None):
    """Bound a room transit with distance and detour/turning allowance.

    This is a wall-clock budget estimate, not a planned path length. Nav2's
    progress/recovery checks and the overall cleaning deadline still apply.
    """
    distance = math.dist(start[:2], target[:2])
    if path_distance_m is not None:
        if not math.isfinite(path_distance_m) or path_distance_m<0:
            raise ValueError('可达路径距离必须为有限非负数。')
        distance=max(distance,path_distance_m)
    if not math.isfinite(distance) or not 0 < minimum <= 180.:
        raise ValueError('导航距离与最短预算必须有限且有效。')
    return min(180., max(minimum, 20.+1.5*distance/.16))


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


def coarse_then_infill(route, start):
    """Visit alternate existing stripes outwards, then infill on the way back.

    Only order/direction changes. Nav2 and live path checks still execute every
    transition; neither predicted nor traversed connectors pre-fill coverage.
    """
    pairs=list(zip(route[::2],route[1::2]))
    order=list(range(0,len(pairs),2))+list(reversed(range(1,len(pairs),2)))
    result=[];previous=start[:2]
    for index in order:
        first,last=pairs[index]
        if math.dist(previous,last[:2])<math.dist(previous,first[:2]):first,last=last,first
        yaw=math.atan2(last[1]-first[1],last[0]-first[0])
        result.extend([(first[0],first[1],yaw),(last[0],last[1],yaw)])
        previous=last[:2]
    return result


def recent_translation_stopped(odometry):
    """Require .45s of low linear speed with fresh, continuous odometry stamps."""
    if not odometry:
        return False
    latest=odometry[-1]['stamp_ns'];previous=None
    for row in reversed(odometry):
        stamp=row['stamp_ns'];velocity=row.get('velocity',[])
        if previous is not None and stamp>=previous:
            continue
        if previous is not None and previous-stamp>100_000_000:
            return False
        if not velocity or velocity[0] is None or not math.isfinite(velocity[0]) or abs(velocity[0])>=.015:
            return False
        if latest-stamp>=450_000_000:
            return True
        previous=stamp
    return False


def observed_return_speed(odometry):
    """Conservative cruising-speed estimate from actual ingress, not scene truth."""
    values=[]
    for row in odometry:
        velocity=row.get('velocity',[])
        if velocity and velocity[0] is not None and math.isfinite(velocity[0]) and abs(velocity[0])>.04:
            values.append(abs(velocity[0]))
    if len(values)<20:
        return None
    median=float(np.median(values))
    return dict(samples=len(values),median_mps=median,
                selected_mps=min(.35,median),maximum_mps=.35)


class RoomSweep:
    def __init__(self, data, width, height, resolution, origin, rooms, room_id, start):
        self.align_connectors=rooms[room_id].get('cleaning_connector_alignment',False)
        self.turning_radius_m=rooms[room_id].get('cleaning_turning_radius_m')
        if (self.turning_radius_m is not None and
                (type(self.turning_radius_m)not in(int,float)or
                 not math.isfinite(self.turning_radius_m)or not .30<=self.turning_radius_m<=.50)):
            raise ValueError('转向检查半径必须在0.30至0.50米之间。')
        if type(self.align_connectors) is not bool:
            raise ValueError('清扫连接对正开关必须为布尔值。')
        self.tracking_margin_cells=rooms[room_id].get('cleaning_tracking_margin_cells',TRACKING_MARGIN_CELLS)
        if type(self.tracking_margin_cells) is not int or self.tracking_margin_cells not in (0,1):
            raise ValueError('清扫附加跟踪余量仅支持0或1格，0格仅供显式局部验证。')
        self.route_order=rooms[room_id].get('cleaning_route_order','adjacent')
        if self.route_order not in ('adjacent','coarse_then_infill'):
            raise ValueError('清扫条带顺序无效。')
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
        reference_centers = navigable & reachable
        # A polygon-aware Nav2 candidate may connect existing sweep centres
        # through narrower observed free space. This is only a connectivity
        # filter: native full-footprint collision checks still execute motion.
        # The reference floor and stripe centres retain their original .32m.
        self.transit_clearance_m=rooms[room_id].get('cleaning_transit_clearance_m',CENTER_CLEARANCE_M)
        if (type(self.transit_clearance_m) not in (int,float)
                or not math.isfinite(self.transit_clearance_m)
                or not .22<=self.transit_clearance_m<=CENTER_CLEARANCE_M):
            raise ValueError('显式连接净空仅支持0.22–0.32m，须配合完整轮廓碰撞检查。')
        transit=clearance>=self.transit_clearance_m+resolution/2
        transit_components,_=label(transit)
        # Reuse an actual reference component cell near the start, rather
        # than selecting a disconnected newly opened pocket.
        source_row,source_col=np.argwhere(reference_centers)[0]
        self.transit_centers=transit & (transit_components==transit_components[source_row,source_col])
        # SciPy iterations=0 erodes to convergence; zero means no extra margin.
        def margin(mask):
            return (binary_erosion(mask,structure=generate_binary_structure(2,1),
                iterations=self.tracking_margin_cells) if self.tracking_margin_cells else mask.copy())
        self.planning_transit_centers=margin(self.transit_centers)
        reference_planning=margin(reference_centers)
        region = np.zeros((height,width), dtype=bool)
        xmin,ymin,xmax,ymax = rooms[room_id]['bounds']
        corners = [self.index(ix,iy) for ix in (xmin,xmax) for iy in (ymin,ymax)]
        for row in range(max(0,min(y for _,y in corners)), min(height,max(y for _,y in corners)+1)):
            for col in range(max(0,min(x for x,_ in corners)), min(width,max(x for x,_ in corners)+1)):
                region[row,col] = belongs_to(rooms,room_id,*self.point(col,row))
        self.floor = self.free & region
        self.centers = reference_centers & region
        self.planning_centers=reference_planning & region
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

    def return_reserve(self, start, task_seconds, *, travel_speed_mps=None):
        """Reserve return time from the room, retaining the short-run bounds."""
        rows,cols = np.nonzero(self.centers)
        corners = [self.point(col,row) for col in (int(cols.min()),int(cols.max()))
                   for row in (int(rows.min()),int(rows.max()))]
        if travel_speed_mps is None:
            estimate = max(transit_timeout(point,start) for point in corners)+30.
        else:
            if not math.isfinite(travel_speed_mps) or not 0<travel_speed_mps<=.35:
                raise ValueError('返航速度估计必须大于0且不超过0.35m/s。')
            estimate=max(min(180.,max(40.,20.+1.5*math.dist(point,start[:2])/travel_speed_mps))
                         for point in corners)+30.
        maximum = max(30.,task_seconds/2)
        original = min(100.,maximum)
        return min(maximum,max(original,estimate))

    def route(self, start, available=None, *, uncovered_only=False, attempted=(), orientation=None):
        """Choose horizontal or vertical lanes using a coverage/travel estimate."""
        available = self.planning_centers if available is None else self.planning_centers & available
        remaining=self.eligible & ~self.covered if uncovered_only else self.eligible
        if uncovered_only and (not remaining.any() or not available.any()):
            return []
        if not available.any():
            raise ValueError('当前代价地图中没有可通行的清扫路线。')
        gains=convolve(remaining.astype(np.int32),self.disk.astype(np.int32),mode='constant') if uncovered_only else None
        pitch = max(1,round(LANE_SPACING_M/self.resolution))
        short = math.ceil(.20/self.resolution)
        best_score, best_route = None, []
        for transpose, centers in ((False,available),(True,available.T)):
            if orientation is not None and orientation != ('vertical' if transpose else 'horizontal'):
                continue
            rows = np.flatnonzero(centers.any(axis=1))
            for offset in range(pitch):
                lane_rows = range(int(rows[0])+offset,int(rows[-1])+1,pitch)
                intervals = [(int(y),int(a),int(b)-1) for y in lane_rows
                             for a,b in runs(centers[y]) if b-a >= short]
                if uncovered_only:
                    trimmed=[];values=gains.T if transpose else gains
                    for y,a,b in intervals:
                        # Separate remaining pockets instead of joining them
                        # across a long strip of already-cleaned floor.
                        for left,right in runs(values[y,a:b+1]>0):
                            first,last=a+int(left),a+int(right)-1
                            if (last-first)*self.resolution>=.14:
                                endpoints=self.lane_poses([(y,first,last)],transpose,False)
                                if not self.attempted_segment(*endpoints,attempted):
                                    trimmed.append((y,first,last))
                    intervals=trimmed
                if not intervals:
                    continue
                planned = np.zeros_like(centers)
                for row,a,b in intervals:
                    planned[row,a:b+1] = True
                predicted = binary_dilation(planned.T if transpose else planned,structure=self.disk)
                predicted_cells=int((predicted & remaining).sum())
                missing = int(remaining.sum())-predicted_cells
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
                # Regeneration first retains the reachable uncovered footprint;
                # travel then ranks equivalent .35m-pitch alternatives. Only
                # physical touch() calls may update the coverage masks.
                score = (-predicted_cells,length+len(intervals)*.45) if uncovered_only else (
                    missing*self.resolution**2 + .045*(length + len(intervals)*.45))
                if best_score is None or score < best_score:
                    best_score,best_route = score,route
        if not best_route:
            if uncovered_only:
                return []
            raise ValueError('选定房间没有足够长的可清扫路线。')
        return (coarse_then_infill(best_route,start)
                if self.route_order=='coarse_then_infill' and not uncovered_only else best_route)

    def spatial_coverage(self,route):
        """Check actual brush coverage of every original stripe centreline.

        Plans grant no coverage. Keep the initial stripe cells even if a later
        obstacle blocks them, so unfinished areas remain explicit in a partial
        result instead of silently shrinking the acceptance denominator.
        """
        if len(route)%2:
            raise ValueError('清扫条带必须包含成对端点。')
        lanes=[]
        for lane_id,(first,last) in enumerate(zip(route[::2],route[1::2])):
            c,r=self.index(*first[:2]);ec,er=self.index(*last[:2])
            if c!=ec and r!=er:
                raise ValueError('清扫条带必须与栅格坐标轴对齐。')
            cells=[(c,y) for y in range(min(r,er),max(r,er)+1)] if c==ec else [
                (x,r) for x in range(min(c,ec),max(c,ec)+1)]
            h,w=self.covered.shape
            valid=all(0<=y<h and 0<=x<w for x,y in cells)
            covered=sum(bool(self.covered[y,x]) for x,y in cells) if valid else 0
            lanes.append(dict(lane_id=lane_id,centerline_cells=len(cells),
                covered_centerline_cells=covered,completed=valid and covered==len(cells)))
        return dict(basis='measured_brush_coverage_of_original_stripe_centerlines',
            completed=bool(lanes) and all(lane['completed'] for lane in lanes),
            planned_lanes=len(lanes),covered_lanes=sum(lane['completed'] for lane in lanes),
            incomplete_lanes=[lane for lane in lanes if not lane['completed']])

    def travel_distances(self,current,available):
        """Map-only geodesic connector estimates; NavFn still executes transit."""
        usable=self.transit_centers & available
        col,row=self.index(*current[:2])
        distances=np.full(usable.shape,np.inf)
        if not (0<=row<usable.shape[0] and 0<=col<usable.shape[1] and usable[row,col]):
            return distances
        indices=np.full(usable.shape,-1,dtype=np.int32)
        indices[usable]=np.arange(int(usable.sum()))
        edges=[]
        for first,last in ((indices[:,:-1],indices[:,1:]),(indices[:-1,:],indices[1:,:])):
            valid=(first>=0)&(last>=0)
            edges.append((first[valid],last[valid]))
        a=np.concatenate([x for x,y in edges]);b=np.concatenate([y for x,y in edges])
        graph=coo_matrix((np.ones(len(a)),(a,b)),shape=(int(usable.sum()),)*2).tocsr()
        distances[usable]=dijkstra(graph,directed=False,indices=int(indices[row,col]))*self.resolution
        return distances

    def segment_gain(self,first,last):
        """Hypothetical brush footprint gain, never physical coverage credit."""
        a,b=self.index(*first[:2]),self.index(*last[:2])
        if a>b:
            first,last=last,first
        left,right=sorted((a[0],b[0]));bottom,top=sorted((a[1],b[1]))
        x0,x1=max(0,left-self.reach),min(self.free.shape[1],right+self.reach+1)
        y0,y1=max(0,bottom-self.reach),min(self.free.shape[0],top+self.reach+1)
        gy,gx=np.mgrid[y0:y1,x0:x1]
        def local(pose):
            x,y=pose[0]-self.origin[0],pose[1]-self.origin[1]
            return self.cosine*x+self.sine*y,-self.sine*x+self.cosine*y
        ax,ay=local(first);bx,by=local(last)
        px,py=(gx+.5)*self.resolution,(gy+.5)*self.resolution
        vx,vy=bx-ax,by-ay
        length_squared=vx*vx+vy*vy
        fraction=np.clip(((px-ax)*vx+(py-ay)*vy)/max(1e-12,length_squared),0.,1.)
        dx=px-(ax+fraction*vx)
        dy=py-(ay+fraction*vy)
        hit=np.hypot(dx,dy)<=self.radius
        return int((hit & self.eligible[y0:y1,x0:x1] & ~self.covered[y0:y1,x0:x1]).sum())

    @staticmethod
    def attempted_segment(first,last,attempted):
        return any((math.dist(first[:2],a[:2])<=.20 and math.dist(last[:2],b[:2])<=.20)
                   or (math.dist(first[:2],b[:2])<=.20 and math.dist(last[:2],a[:2])<=.20)
                   for a,b in attempted)

    def lane_value(self,current,first,last,distances):
        """Estimate one connector plus brush segment without granting coverage."""
        heading=math.atan2(last[1]-first[1],last[0]-first[0])
        first=(*first[:2],heading);last=(*last[:2],heading)
        col,row=self.index(*first[:2]);connector=float(distances[row,col])
        if not math.isfinite(connector):
            return None
        gain=self.segment_gain(first,last)
        if gain*self.resolution**2<.04:
            return None
        length=math.dist(first[:2],last[:2])
        approach=math.atan2(first[1]-current[1],first[0]-current[0]) if connector>.12 else current[2]
        turns=abs(math.remainder(approach-current[2],2*math.pi))+abs(math.remainder(heading-approach,2*math.pi))
        seconds=5.+(connector+length)/.20+turns/.6
        return dict(start=first,end=last,predicted_new_cells=gain,
            connector_distance_m=connector,length_m=length,
            estimated_seconds=seconds,score=gain*self.resolution**2/seconds)

    def connector_alignment_goal(self,current,first,last,available):
        """Join a nearby stripe axis in open space before a long connector.

        The projected point uses only the planned stripe and current observed
        free grid. It grants no coverage and cannot cross a blocked corner.
        """
        if (not self.align_connectors or current is None or
                not all(math.isfinite(v)for v in (*current,*first,*last))):
            return None
        dx,dy=last[0]-first[0],last[1]-first[1];length=math.hypot(dx,dy)
        if length<.14:
            return None
        ux,uy=dx/length,dy/length
        along=(current[0]-first[0])*ux+(current[1]-first[1])*uy
        if along>=-LANE_SPACING_M:
            return None
        target=(first[0]+along*ux,first[1]+along*uy,math.atan2(uy,ux))
        lateral=math.dist(current[:2],target[:2])
        if (not .04<lateral<=.18 or self.lane_path(current,target,available) is None
                or not self.recovery_target_valid(target,available,arrival_tolerance_m=.04)):
            return None
        return dict(target=target,lateral_error_m=lateral,longitudinal_distance_m=-along,
            position_tolerance_m=.04,source='observed_free_local_stripe_axis_join')

    def execution_segment(self,first,last,available,costs,*,current=None):
        """Leave turning room within the brush reach of the unchanged stripe.

        This adjusts commanded body centres only; the original stripe remains
        the acceptance reference. Native collision/stop checks and real brush
        repaint must still establish that reference coverage.
        """
        result=dict(start=first,end=last,reference_start=first,reference_end=last,
                    point_only=False,start_inset_m=0.,end_inset_m=0.)
        if self.turning_radius_m is None or costs is None:
            return result
        costs=np.asarray(costs)
        if costs.shape!=self.covered.shape or available.shape!=costs.shape:
            raise ValueError('转向检查必须使用同一底图尺寸。')
        blocked=(costs<0)|(costs>=100)|~self.free
        radius=float(self.turning_radius_m);res=self.resolution;h,w=costs.shape
        def safe(target):
            col,row=self.index(*target[:2]);reach=math.ceil(radius/res)+1
            if not(0<=row<h and 0<=col<w and self.planning_centers[row,col]and available[row,col]):
                return False
            x0,x1=max(0,col-reach),min(w,col+reach+1);y0,y1=max(0,row-reach),min(h,row+reach+1)
            dx,dy=target[0]-self.origin[0],target[1]-self.origin[1]
            gx=self.cosine*dx+self.sine*dy;gy=-self.sine*dx+self.cosine*dy
            if gx-radius<0 or gy-radius<0 or gx+radius>w*res or gy+radius>h*res:
                return False
            # Exact distance to occupied whole cell squares, not just centres.
            xs=(np.arange(x0,x1)+.5)*res;ys=(np.arange(y0,y1)+.5)*res
            near=np.hypot(np.maximum(abs(xs[None,:]-gx)-res/2,0),
                          np.maximum(abs(ys[:,None]-gy)-res/2,0))<=radius
            return not bool((near&blocked[y0:y1,x0:x1]).any())
        length=math.dist(first[:2],last[:2])
        if length<1e-9:
            return result
        ux,uy=(last[0]-first[0])/length,(last[1]-first[1])/length
        approach_turn=False
        if current is not None and math.dist(current[:2],first[:2])>.35:
            approach=math.atan2(first[1]-current[1],first[0]-current[0])
            approach_turn=abs(math.remainder(first[2]-approach,2*math.pi))>.4
        def inward(point,sign,forced=False):
            # A large turn at entry needs room even when the forward camera
            # has not observed that endpoint yet. Do not rely on unknown low
            # feet having been marked before the robot approaches them.
            candidates=(min(.15,length/2),) if forced else(0.,.05,.10,.15)
            for inset in candidates:
                if inset>length/2:
                    break
                target=(point[0]+sign*inset*ux,point[1]+sign*inset*uy,first[2])
                if safe(target):return target,inset
            return point,0.
        start,si=inward(first,1,approach_turn);end,ei=inward(last,-1)
        if (si or ei)and math.dist(start[:2],end[:2])<.14:
            middle=((first[0]+last[0])/2,(first[1]+last[1])/2,first[2])
            # .15m + .04m native body checker + half cell diagonal < .25m brush.
            if length<=.30 and safe(middle):
                start=end=middle;si=ei=length/2;result['point_only']=True
            else:
                return result
        if (si or ei)and not result['point_only'] and self.lane_path(start,end,available)is None:
            return result
        result.update(start=start,end=end,start_inset_m=si,end_inset_m=ei,
                      turning_radius_m=radius,approach_turn=approach_turn,
                      source='measured_obstacle_turning_room_within_brush_reach')
        return result

    def connector_budget(self,current,target,available):
        """Bound a cleaning connector using the observed navigable detour."""
        distances=self.travel_distances(current,available)
        col,row=self.index(*target[:2])
        distance=float(distances[row,col])
        if not math.isfinite(distance):
            distance=None
        return dict(timeout_seconds=transit_timeout(current,target,path_distance_m=distance),
                    observed_path_distance_m=distance,
                    source='current_mask_four_neighbor_distance' if distance is not None else 'euclidean_fallback')

    def cross_orientation(self,route):
        """Perpendicular to the dominant seeded stripe axis in this grid."""
        horizontal=vertical=0
        for first,last in zip(route[::2],route[1::2]):
            a,b=self.index(*first[:2]),self.index(*last[:2])
            horizontal+=abs(a[0]-b[0]);vertical+=abs(a[1]-b[1])
        return ('vertical' if horizontal>=vertical else 'horizontal') if horizontal+vertical else None

    def required_lane_repairs(self,current,route,available,attempted=(),limit=4):
        """Repair measured gaps on original stripes, even below area-gain cutoffs.

        Keep blocked cells and exhausted retries visibly incomplete. A proposed
        repair never changes the brush mask or the original acceptance route.
        """
        needed={v['lane_id']for v in self.spatial_coverage(route)['incomplete_lanes']}
        if not needed:
            return []
        connected=self.connected_centers(current,available)&self.planning_centers
        distance=self.travel_distances(current,available)
        ranked=[];minimum_steps=math.ceil(.14/self.resolution)
        for lane_id,(first,last) in enumerate(zip(route[::2],route[1::2])):
            if lane_id not in needed:
                continue
            for a,b in self.lane_segments(first,last,connected):
                start,finish=self.index(*a[:2]),self.index(*b[:2])
                count=max(abs(start[0]-finish[0]),abs(start[1]-finish[1]))+1
                cells=np.rint(np.linspace(start,finish,count)).astype(int)
                missing=~self.covered[cells[:,1],cells[:,0]]
                for low,high in runs(missing):
                    left=max(0,low-2);right=min(count-1,high+1)
                    if right-left<minimum_steps:
                        right=min(count-1,left+minimum_steps)
                        left=max(0,right-minimum_steps)
                    if right-left<minimum_steps:
                        continue
                    p=self.point(*cells[left]);q=self.point(*cells[right])
                    heading=math.atan2(q[1]-p[1],q[0]-p[0]);p=(*p,heading);q=(*q,heading)
                    if sum(self.attempted_segment(p,q,[old])for old in attempted)>=2:
                        continue
                    gain=self.segment_gain(p,q)
                    if gain==0:
                        continue
                    required=int(missing[left:right+1].sum())
                    for a,b in ((p,q),(q,p)):
                        heading=math.atan2(b[1]-a[1],b[0]-a[0]);a=(*a[:2],heading);b=(*b[:2],heading)
                        col,row=self.index(*a[:2]);connector=float(distance[row,col])
                        if not math.isfinite(connector):
                            continue
                        length=math.dist(a[:2],b[:2]);turn=abs(math.remainder(heading-current[2],2*math.pi))
                        seconds=3.+(connector+length)/.20+turn/.6
                        ranked.append(dict(start=a,end=b,source='required_stripe_repair',
                            original_lane_id=lane_id,required_centerline_cells=required,
                            predicted_new_cells=gain,connector_distance_m=connector,length_m=length,
                            estimated_seconds=seconds,score=required/seconds))
        selected=[]
        for item in sorted(ranked,key=lambda v:(-v['score'],v['connector_distance_m'])):
            if self.attempted_segment(item['start'],item['end'],[(v['start'],v['end'])for v in selected]):
                continue
            selected.append(item)
            if len(selected)>=limit:
                break
        return selected


    def offset_lane_repairs(self,current,route,available,attempted=(),limit=4):
        """Reach an original stripe with the brush from a safe adjacent path."""
        from .cleaning_repairs import offset_lane_repairs
        return offset_lane_repairs(self,current,route,available,attempted,limit)

    def remaining_lanes(self,current,available,attempted=(),limit=4,*,distances=None,budget=False,
                        preferred_orientation=None):
        """Regenerate .35m-pitch lanes and rank their measured-uncovered gain."""
        connected=self.connected_centers(current,available) & self.planning_centers
        route=self.route(current,connected,uncovered_only=True,attempted=attempted,
                         orientation=preferred_orientation)
        if not route and preferred_orientation is not None:
            route=self.route(current,connected,uncovered_only=True,attempted=attempted)
        if not route:
            return []
        distance=self.travel_distances(current,available) if distances is None else distances
        ranked=[]
        for first,last in zip(route[::2],route[1::2]):
            gain=self.segment_gain(first,last)
            if gain*self.resolution**2<.04:
                continue
            for start,end in ((first,last),(last,first)):
                if budget:
                    choice=self.lane_value(current,start,end,distance)
                    if choice is not None:
                        ranked.append(choice)
                else:
                    heading=math.atan2(end[1]-start[1],end[0]-start[0])
                    start=(*start[:2],heading);end=(*end[:2],heading)
                    col,row=self.index(*start[:2]);connector=float(distance[row,col])
                    if not math.isfinite(connector):
                        continue
                    length=math.dist(start[:2],end[:2])
                    turn=abs(math.remainder(heading-current[2],2*math.pi))
                    seconds=3.+(connector+length)/.20+turn/.6
                    ranked.append(dict(start=start,end=end,predicted_new_cells=gain,
                        connector_distance_m=connector,length_m=length,
                        estimated_seconds=seconds,score=gain*self.resolution**2/seconds))
        selected=[]
        for item in sorted(ranked,key=lambda x:(-x['score'],x['connector_distance_m'])):
            a,b=self.index(*item['start'][:2]),self.index(*item['end'][:2])
            item.update(orientation='horizontal' if abs(a[0]-b[0])>=abs(a[1]-b[1]) else 'vertical',
                        preferred_orientation=preferred_orientation)
            if self.attempted_segment(item['start'],item['end'],
                                      [(x['start'],x['end']) for x in selected]):
                continue
            selected.append(item)
            if len(selected)>=limit:
                break
        if not selected and preferred_orientation is not None:
            selected=self.remaining_lanes(current,available,attempted,limit,
                distances=distance,budget=budget)
            for item in selected:
                item['preferred_orientation']=preferred_orientation
        return selected

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

    def connected_centers(self, current, available):
        """Keep currently connected candidates; the coverage denominator stays fixed."""
        live = self.transit_centers & available
        col,row = self.index(*current[:2])
        if not (0<=row<live.shape[0] and 0<=col<live.shape[1] and live[row,col]):
            return np.zeros_like(live)
        components,_ = label(live)
        return (components==components[row,col]) & self.centers

    def lane_path(self, current, target, available):
        """Keep a brush segment as a dense reference, checking every crossed cell.

        The mask is for navigation only; the original eligible floor remains
        unchanged. Check both side cells at a diagonal grid crossing so a
        reference cannot cut between two blocked corners.
        """
        if current is None or not all(math.isfinite(v) for v in (*current,*target)):
            return None
        usable=self.transit_centers & available
        distance=math.dist(current[:2],target[:2])
        count=max(1,math.ceil(distance/(self.resolution/2)))
        heading=math.atan2(target[1]-current[1],target[0]-current[0])
        path=[];previous=None
        for fraction in np.linspace(0.,1.,count+1):
            x=current[0]+(target[0]-current[0])*fraction
            y=current[1]+(target[1]-current[1])*fraction
            col,row=self.index(x,y)
            cells=[(col,row)]
            if previous is not None and col!=previous[0] and row!=previous[1]:
                cells.extend([(col,previous[1]),(previous[0],row)])
            if any(not (0<=r<usable.shape[0] and 0<=c<usable.shape[1] and usable[r,c])
                   for c,r in cells):
                return None
            path.append((float(x),float(y),float(heading)))
            previous=col,row
        path[-1]=tuple(target)
        return path

    def lane_reference(self,current,first,last,available):
        """Join the intended stripe locally, then stop before a newly blocked gap.

        A measured arrival may be off the stripe by the normal XY tolerance.
        A direct diagonal to its distant end can cross an adjacent obstacle.
        Both the local join and the retained stripe use lane_path's full checks.
        """
        if current is None or not all(math.isfinite(v) for v in (*current,*first,*last)):
            return None
        connected=self.connected_centers(current,available)
        for start,end in self.lane_segments(first,last,connected):
            dx,dy=end[0]-start[0],end[1]-start[1]
            fraction=np.clip(((current[0]-start[0])*dx+(current[1]-start[1])*dy)/
                             max(1e-12,dx*dx+dy*dy),0.,1.)
            join=(start[0]+float(fraction)*dx,start[1]+float(fraction)*dy,start[2])
            if math.dist(current[:2],join[:2])>.18:
                continue
            direct=self.lane_path(current,end,available)
            if direct is not None:
                return dict(path=direct,start=start,end=end,join=join,source='direct_to_fresh_end')
            approach=self.lane_path(current,join,available)
            stripe=self.lane_path(join,end,available)
            if approach is not None and stripe is not None:
                return dict(path=approach+stripe[1:],start=start,end=end,join=join,
                            source='rejoin_nominal_stripe')
        return None

    def replan_options(self, current, pending, available, excluded=(), attempted=()):
        """Rank two different remaining lane starts using current sensor costs."""
        connected = self.connected_centers(current,available)
        options = []
        for lane_id,first,last in pending:
            if lane_id in excluded:
                continue
            for start,end in self.lane_segments(first,last,connected):
                for a,b in ((start,end),(end,start)):
                    if any(math.dist(a[:2],old)<.25 for old in attempted):
                        continue
                    options.append((math.dist(current[:2],a[:2]),lane_id,a,b))
        result=[]
        for _,lane_id,start,end in sorted(options):
            if any(math.dist(start[:2],a[:2])<.25 for _,a,_ in result):
                continue
            result.append((lane_id,start,end))
            if len(result)==2:
                break
        return result

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
        centers = self.planning_centers if available is None else self.connected_centers(current,available) & self.planning_centers
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

    def recovery_target_valid(self, target, available, *, arrival_tolerance_m=.12):
        """Keep a recovery arrival disk out of blocked/unknown navigation cells.

        This buffers only a recovery endpoint, never the coverage mask or
        the whole dynamic map. Nav2 still validates and executes the path.
        """
        if not math.isfinite(arrival_tolerance_m) or arrival_tolerance_m<0:
            raise ValueError('恢复到点余量必须为有限非负数。')
        if (target is None or not all(math.isfinite(v) for v in target)
                or available.shape!=self.centers.shape):
            return False
        dx,dy=target[0]-self.origin[0],target[1]-self.origin[1]
        x,y=self.cosine*dx+self.sine*dy,-self.sine*dx+self.cosine*dy
        radius=arrival_tolerance_m;res=self.resolution
        left,bottom=math.floor((x-radius)/res),math.floor((y-radius)/res)
        right,top=math.floor((x+radius)/res)+1,math.floor((y+radius)/res)+1
        h,w=available.shape
        if left<0 or bottom<0 or right>w or top>h:
            return False
        rows,cols=np.mgrid[bottom:top,left:right]
        # Distance to the entire cell square, rather than just its centre.
        near_x=np.maximum(np.abs((cols+.5)*res-x)-res/2,0.)
        near_y=np.maximum(np.abs((rows+.5)*res-y)-res/2,0.)
        touched=np.hypot(near_x,near_y)<=radius+1e-12
        # Match the independent post-recovery arrival predicate, which still
        # requires the original conservative reference centres.
        usable=self.centers[bottom:top,left:right]&available[bottom:top,left:right]
        return bool(np.all(usable[touched]))

    def recovery_anchor(self, current, trace, available, now_sim_seconds, *, arrival_tolerance_m=0.):
        """Find a nearby measured anchor on the latest continuous trace.

        This only selects an endpoint. Nav2 still checks and executes the
        recovery path using current obstacles; past traversal is not proof
        that the corridor remains free. Coverage masks are never modified.
        """
        if (not trace or not math.isfinite(now_sim_seconds) or len(current) != 3
                or not all(math.isfinite(v) for v in current)):
            return None
        if available.shape != self.centers.shape:
            raise ValueError('恢复选点的代价地图与清扫地图尺寸不一致。')

        def sample(index):
            try:
                pose = tuple(float(v) for v in trace[index]['pose'])
                stamp = float(trace[index]['sim_seconds'])
                if len(pose) == 3 and all(math.isfinite(v) for v in (*pose, stamp)):
                    return pose, stamp
            except (KeyError, TypeError, ValueError):
                pass
            return None

        latest = sample(len(trace)-1)
        if latest is None:
            return None
        previous, previous_stamp = latest
        path_length = math.dist(current[:2], previous[:2])
        if not 0 <= now_sim_seconds-previous_stamp <= .6 or path_length > .30:
            return None
        for index in range(len(trace)-1, -1, -1):
            value = sample(index)
            if value is None:
                break
            pose, stamp = value
            if not 0 <= now_sim_seconds-stamp <= 30.:
                break
            if index < len(trace)-1:
                # A row's connected flag refers to its edge from the older
                # row. Never bridge a missing TF sample or clock jump.
                step = math.dist(previous[:2], pose[:2])
                if (trace[index+1].get('connected') is not True
                        or not 0 < previous_stamp-stamp <= .6 or step > .30):
                    break
                path_length += step
            if path_length > 1.5:
                break
            distance = math.dist(current[:2], pose[:2])
            col, row = self.index(*pose[:2])
            if (.4 <= distance <= .85 and 0 <= row < self.centers.shape[0]
                    and 0 <= col < self.centers.shape[1]
                    and self.centers[row, col] and available[row, col]
                    and (arrival_tolerance_m==0. or self.recovery_target_valid(
                        pose,available,arrival_tolerance_m=arrival_tolerance_m))):
                return dict(pose=pose, trace_index=index, sim_seconds=stamp,
                            retreat_path_m=path_length)
            previous, previous_stamp = pose, stamp
        return None

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
