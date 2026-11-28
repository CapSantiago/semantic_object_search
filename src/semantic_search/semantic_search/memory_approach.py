"""One bounded historical-position approach, using observed grids and Nav2.

Reuse RoomSweep's map geometry/geodesic distances and SciPy clearance. This
predicts a safe verification site, never current object presence or counts.
"""
import base64
import math
import zlib

import numpy as np
from scipy.ndimage import distance_transform_edt, label

from .cleaning import RoomSweep


def _packed(array):
    return base64.b64encode(zlib.compress(np.asarray(array,dtype=np.int8).tobytes())).decode('ascii')


def _legacy_approach(views, rooms, room_id, records, grid, available, current):
    if rooms[room_id].get('inspection_views') or not views or grid.get('frame_id')!='map':
        return None
    width,height=grid['width'],grid['height']
    if width*height>2_000_000 or np.shape(available)!=(height,width):
        return None
    origin=grid['origin'];origin=(origin['x'],origin['y'],origin.get('yaw',0.))
    sweep=RoomSweep(grid['data'],width,height,grid['resolution'],origin,rooms,room_id,current)
    live=np.asarray(available,dtype=bool)
    clearance=distance_transform_edt(np.pad(sweep.free,1))[1:-1,1:-1]*sweep.resolution
    safe=(clearance>=.35)&live
    components,_=label(safe)
    col,row=sweep.index(*current[:2])
    if not(0<=row<height and 0<=col<width) or not components[row,col]:
        return None
    component=components[row,col]
    distance=sweep.travel_distances(current,safe)
    def estimate(pose):
        c,r=sweep.index(*pose[:2])
        if not(0<=r<height and 0<=c<width):return math.inf
        return float(distance[r,c])/.25+abs(math.remainder(pose[2]-current[2],2*math.pi))/.6
    source=min(views,key=estimate);baseline=estimate(source)
    if not math.isfinite(baseline):return None
    best=None
    usable=[v for v in records if not v.get('classification_conflict') and
            v.get('state') in ('last_observed','needs_verification') and
            all(type(v.get(k))in(int,float)and math.isfinite(v[k])for k in ('x','y','z'))]
    for prior in usable[:8]:
        for radius in (1.2,1.5,1.8):
            for angle in np.arange(-math.pi,math.pi,math.pi/12):
                c,r=sweep.index(prior['x']+radius*math.cos(angle),prior['y']+radius*math.sin(angle))
                if not(0<=r<height and 0<=c<width and sweep.centers[r,c]
                       and components[r,c]==component):continue
                x,y=sweep.point(c,r);reach=math.hypot(prior['x']-x,prior['y']-y)
                if not 1.0<=reach<=2.0:continue
                # Predict LOS through known map cells, allowing the final
                # target surface. Actual RGB-D must confirm after arrival.
                clear=True
                for fraction in np.linspace(0,max(0,1-.2/reach),80):
                    cc,rr=sweep.index(x+fraction*(prior['x']-x),y+fraction*(prior['y']-y))
                    if not(0<=rr<height and 0<=cc<width and sweep.free[rr,cc]):
                        clear=False;break
                if not clear:continue
                pose=[x,y,math.atan2(prior['y']-y,prior['x']-x)]
                cost=estimate(pose)
                if cost>=baseline*.9:continue
                rank=cost*(1. if prior['state']=='last_observed' else 4.)
                if best is None or rank<best[0]:best=(rank,cost,pose,prior,r,c,reach)
    if best is None:return None
    _,cost,pose,prior,r,c,reach=best
    return dict(pose=pose,memory_id=prior['id'],prior_state=prior['state'],
        prior_xy=[prior['x'],prior['y']],prior_xyz=[prior[k]for k in ('x','y','z')],
        distance_m=reach,source_configured_pose=list(source),
        basis='observed_memory_safe_approach',
        planning=dict(room=room_id,current_pose=list(current),clearance_m=float(clearance[r,c]),
            minimum_clearance_m=.35,estimated_approach_seconds=cost,
            fastest_configured_estimate_seconds=baseline,visibility='predicted_2d_only',
            grid={k:grid[k]for k in ('frame_id','width','height','resolution','origin')},
            occupancy_zlib_base64=_packed(sweep.occupancy),available_zlib_base64=_packed(live)))


def propose_approach(views, rooms, room_id, records, grid, available, current, *, successful_views=None):
    """Stay near a genuinely successful observation; never chase a bare XY guess."""
    if rooms[room_id].get('inspection_views') or not views or grid.get('frame_id')!='map':return None
    width,height=grid['width'],grid['height']
    if width*height>2_000_000 or np.shape(available)!=(height,width):return None
    origin=grid['origin'];origin=tuple(origin.get(k,0.)for k in ('x','y','yaw'))
    sweep=RoomSweep(grid['data'],width,height,grid['resolution'],origin,rooms,room_id,current)
    clearance=distance_transform_edt(np.pad(sweep.free,1))[1:-1,1:-1]*sweep.resolution
    safe=(clearance>=.35)&np.asarray(available,dtype=bool)
    components,_=label(safe);c,r=sweep.index(*current[:2])
    if not(0<=r<height and 0<=c<width)or not components[r,c]:return None
    component=components[r,c];travel=sweep.travel_distances(current,safe)
    def estimate(pose):
        c,r=sweep.index(*pose[:2])
        if not(0<=r<height and 0<=c<width):return math.inf
        return float(travel[r,c])/.25+abs(math.remainder(pose[2]-current[2],2*math.pi))/.6
    source=min(views,key=estimate);baseline=estimate(source)
    if not math.isfinite(baseline):return None
    def obstruction(pose,prior):
        reach=math.dist(pose[:2],[prior['x'],prior['y']]);blocked=0
        for f in np.linspace(0,max(0,1-.2/reach),80):
            c,r=sweep.index(pose[0]+f*(prior['x']-pose[0]),pose[1]+f*(prior['y']-pose[1]))
            if not(0<=r<height and 0<=c<width)or sweep.occupancy[r,c]<0:return math.inf
            if not sweep.free[r,c]:
                # Allow only the same nearby support-surface projection as
                # the actual successful view; not an intervening obstacle.
                if reach*(1-f)>.6:return math.inf
                blocked+=1
        return blocked*reach/80
    best=None
    for prior in records[:8]:
        if (prior.get('state')!='last_observed' or prior.get('classification_conflict') or
            not all(type(prior.get(k))in(int,float)and math.isfinite(prior[k])for k in ('x','y','z'))):continue
        refs=[v for v in (successful_views or []) if v.get('memory_id')==prior['id'] and
              v.get('task_id') and v.get('frames',0)>=2 and len(v.get('pose',[]))==3 and
              all(type(x)in(int,float)and math.isfinite(x)for x in v['pose'])]
        for ref in refs[:8]:
            anchor=ref['pose'];radius=math.dist(anchor[:2],[prior['x'],prior['y']])
            if not .8<=radius<=2.2:continue
            blocked=obstruction(anchor,prior)
            if not math.isfinite(blocked):continue
            bearing=math.atan2(anchor[1]-prior['y'],anchor[0]-prior['x'])
            for ratio in (.8,.9,1.):
                for angle in np.arange(-math.pi,math.pi,math.pi/24):
                    if abs(math.remainder(angle-bearing,2*math.pi))>math.pi/4:continue
                    c,r=sweep.index(prior['x']+radius*ratio*math.cos(angle),prior['y']+radius*ratio*math.sin(angle))
                    if not(0<=r<height and 0<=c<width and sweep.centers[r,c]and components[r,c]==component):continue
                    x,y=sweep.point(c,r);reach=math.hypot(x-prior['x'],y-prior['y'])
                    if not .8<=reach<=radius+sweep.resolution/2 or math.dist([x,y],anchor[:2])>.75:continue
                    pose=[x,y,math.atan2(prior['y']-y,prior['x']-x)];cost=estimate(pose)
                    occlusion=obstruction(pose,prior)
                    if occlusion>blocked+sweep.resolution/2 or cost>=baseline-1.:continue
                    if best is None or cost<best[0]:best=(cost,pose,prior,r,c,reach,ref,blocked,occlusion)
    if best is None:return None
    cost,pose,prior,r,c,reach,ref,blocked,occlusion=best
    return dict(pose=pose,memory_id=prior['id'],prior_state=prior['state'],prior_xy=[prior['x'],prior['y']],
        prior_xyz=[prior[k]for k in ('x','y','z')],distance_m=reach,source_configured_pose=list(source),
        basis='observed_memory_safe_approach',planning=dict(policy_version=2,room=room_id,current_pose=list(current),
        clearance_m=float(clearance[r,c]),minimum_clearance_m=.35,estimated_approach_seconds=cost,
        fastest_configured_estimate_seconds=baseline,minimum_predicted_saving_seconds=1.,
        reference_successful_view=ref,reference_blocked_length_m=blocked,candidate_blocked_length_m=occlusion,
        visibility='near_successful_view_same_or_better_range_and_2d_occlusion_prediction_only',
        grid={k:grid[k]for k in ('frame_id','width','height','resolution','origin')},
        occupancy_zlib_base64=_packed(sweep.occupancy),available_zlib_base64=_packed(safe)))


def verify_approach(item,views,rooms,room_id,namespace=None,successful_views=None):
    """Recompute the submitted site from retained observed grids, not a flag."""
    try:
        planning=item['planning'];grid=dict(planning['grid']);n=grid['width']*grid['height']
        if not 0<n<=2_000_000 or planning['room']!=room_id:return False
        if len(item['pose'])!=3 or not all(type(v)in(int,float)and math.isfinite(v)for v in item['pose']):return False
        def unpack(key):
            decoder=zlib.decompressobj();raw=decoder.decompress(base64.b64decode(planning[key]),n+1)
            if len(raw)!=n or decoder.unconsumed_tail or not decoder.eof:raise ValueError('Grid size mismatch')
            return np.frombuffer(raw,dtype=np.int8).reshape(grid['height'],grid['width'])
        grid['data']=unpack('occupancy_zlib_base64').ravel().tolist()
        if namespace is not None:
            from .semantic_memory import map_namespace
            if map_namespace(dict(grid,available=True),rooms)!=namespace:return False
        available=unpack('available_zlib_base64')
        if not np.isin(available,[0,1]).all():return False
        prior=dict(id=item['memory_id'],state=item['prior_state'],
                   **dict(zip(('x','y','z'),item['prior_xyz'])))
        if planning.get('policy_version',1)==1:
            expected=_legacy_approach(views,rooms,room_id,[prior],grid,available.astype(bool),planning['current_pose'])
        else:
            ref=planning['reference_successful_view']
            if successful_views is not None and ref not in successful_views:return False
            expected=propose_approach(views,rooms,room_id,[prior],grid,available.astype(bool),planning['current_pose'],successful_views=[ref])
        return (expected is not None and
                abs(expected['planning']['clearance_m']-planning['clearance_m'])<1e-8 and
                all(abs(a-b)<1e-8 for a,b in zip(expected['pose'],item['pose'])))
    except (ValueError,TypeError,KeyError,OverflowError,zlib.error):
        return False
