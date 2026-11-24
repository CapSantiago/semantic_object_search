"""Short parallel/perpendicular brush repairs from measured safe centre cells."""
import math
import numpy as np
from scipy.ndimage import binary_dilation

def offset_lane_repairs(sweep,current,route,available,attempted=(),limit=4):
    needed={v['lane_id'] for v in sweep.spatial_coverage(route)['incomplete_lanes']}
    if not needed:return []
    usable=sweep.connected_centers(current,available)&sweep.planning_centers
    distance=sweep.travel_distances(current,available);ranked=[];seen=set()
    steps=max(3,math.ceil(.30/sweep.resolution))
    for lane_id,(first,last) in enumerate(zip(route[::2],route[1::2])):
        if lane_id not in needed:continue
        a,b=sweep.index(*first[:2]),sweep.index(*last[:2]);n=max(abs(a[0]-b[0]),abs(a[1]-b[1]))+1
        cells=np.rint(np.linspace(a,b,n)).astype(int);cells=cells[~sweep.covered[cells[:,1],cells[:,0]]]
        missing=np.zeros_like(usable);missing[cells[:,1],cells[:,0]]=True
        rr,cc=np.nonzero(usable & binary_dilation(missing,structure=sweep.disk))
        for col,row in zip(cc,rr):
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                for lo,hi in [(0,steps),(-steps,0),(-steps//2,steps//2)]:
                    start=(int(col+lo*dx),int(row+lo*dy));end=(int(col+hi*dx),int(row+hi*dy));key=(lane_id,tuple(sorted((start,end))))
                    if key in seen:continue
                    seen.add(key)
                    h,w=usable.shape
                    if not all(0<=c<w and 0<=r<h and usable[r,c]for c,r in [start,end]):continue
                    segment=np.array(end)-start;fractions=np.clip(((cells-start)@segment)/max(float(segment@segment),1),0,1);dist=np.linalg.norm(cells-(np.array(start)+fractions[:,None]*segment),axis=1)*sweep.resolution
                    required=int((dist<=sweep.radius+1e-9).sum())
                    if not required:continue
                    xy1=sweep.point(*start);xy2=sweep.point(*end);yaw=math.atan2(xy2[1]-xy1[1],xy2[0]-xy1[0]);p=(*xy1,yaw);q=(*xy2,yaw)
                    if sum(sweep.attempted_segment(p,q,[old])for old in attempted)>=2:continue
                    if sweep.lane_path(p,q,available)is None:continue
                    gain=sweep.segment_gain(p,q)
                    if not gain:continue
                    connector=float(distance[start[1],start[0]])
                    if not math.isfinite(connector):continue
                    seconds=3+(connector+math.dist(xy1,xy2))/.20+abs(math.remainder(yaw-current[2],2*math.pi))/.6
                    ranked.append(dict(start=p,end=q,source='required_stripe_offset_repair',original_lane_id=lane_id,required_centerline_cells=required,predicted_new_cells=gain,connector_distance_m=connector,length_m=math.dist(xy1,xy2),estimated_seconds=seconds,score=required/seconds,brush_margin_m=float(sweep.radius-dist[dist<=sweep.radius+1e-9].max())))
    selected=[]
    for item in sorted(ranked,key=lambda v:(-v['score'],-v['brush_margin_m'],v['connector_distance_m'])):
        if sweep.attempted_segment(item['start'],item['end'],[(v['start'],v['end'])for v in selected]):continue
        selected.append(item)
        if len(selected)>=limit:break
    return selected
