"""Verify history at existing sites and retain every configured find fallback.

No history is inserted into a current LandmarkMap. All configured viewpoints
remain as fallback, and mandatory inspection surfaces keep their full prefix.
"""
import math


def _bearing_verification(views, records, half_fov):
    """At most one new heading, at the nearest already configured nearby site.

    Reuse the active-observation focus range (.8--2.2 m). A historical point
    suggests a viewing direction only; it cannot establish a current detection.
    """
    sites = list(dict.fromkeys(tuple(p[:2]) for p in views))
    for item in sorted(records, key=lambda r: r['state'] != 'last_observed'):
        nearby = [(math.hypot(item['x']-x, item['y']-y), (x,y)) for x,y in sites]
        nearby = [v for v in nearby if .8 <= v[0] <= 2.2]
        if not nearby:
            continue
        distance, (x,y) = min(nearby, key=lambda v: v[0])
        yaw = math.atan2(item['y']-y, item['x']-x)
        existing = [p for p in views if p[:2] == [x,y]]
        if any(abs(math.remainder(p[2]-yaw,2*math.pi)) <= half_fov*.6 for p in existing):
            # A suitable nearby direction already exists; simply prioritize it.
            return None
        return dict(pose=[x,y,yaw], memory_id=item['id'], prior_state=item['state'],
                    prior_xy=[item['x'],item['y']], distance_m=distance,
                    source_configured_pose=existing[0],
                    basis='historical_bearing_at_existing_calibrated_site')
    return None


def prioritize_find_views(views, room, records, *, half_fov=.515,successful_views=None,approach_context=None):
    baseline=[list(p) for p in views]
    required={tuple(p) for group in room.get('inspection_views',{}).values() for p in group}
    history=[]
    for item in records:
        xyz=[item.get(k) for k in ('x','y','z')]
        if (item.get('classification_conflict') or item.get('state') not in
                ('last_observed','needs_verification') or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in xyz)):
            continue
        history.append(item)
    evidence=dict(enabled=True,used=bool(history),order_changed=False,
        basis='nearby_historical_bearing_then_successful_view_and_geometry_prior',
        memory_ids=[item['id'] for item in history],baseline_viewpoints=baseline,
        fallback='all_configured_views_retained',counts_from_history=False,
        verification_views=[], added_viewpoints=0)
    if not history:
        evidence['reason']='no_usable_history'
        return baseline,evidence
    trusted={item['id']:(1. if item['state']=='last_observed' else .25) for item in history}
    observed=[v for v in (successful_views or []) if v.get('memory_id') in trusted]
    evidence['historical_successful_views']=observed
    def observed_score(pose):
        return sum(trusted[v['memory_id']] for v in observed
                   if len(v['pose'])==3 and all(abs(a-b)<1e-6 for a,b in zip(pose,v['pose'])))
    def score(pose):
        value=0.
        for item in history:
            dx,dy=item['x']-pose[0],item['y']-pose[1]
            distance=math.hypot(dx,dy)
            angle=abs(math.atan2(math.sin(math.atan2(dy,dx)-pose[2]),
                                 math.cos(math.atan2(dy,dx)-pose[2])))
            if .5<=distance<=4.5 and angle<half_fov:
                trust=1. if item['state']=='last_observed' else .25
                value+=trust*(1-angle/half_fov)/(1+distance)
        return value
    prefix=[p for p in baseline if tuple(p) in required]
    optional=[(i,p) for i,p in enumerate(baseline) if tuple(p) not in required]
    verification=None
    if approach_context is not None:
        from .memory_approach import propose_approach
        verification=propose_approach(baseline,records=history,**approach_context)
    if verification is None:
        verification=_bearing_verification(baseline,history,half_fov)
    ordered=prefix+([verification['pose']]if verification else [])+[
        p for _,p in sorted(optional,key=lambda item:(-observed_score(item[1]),-score(item[1]),item[0]))]
    if verification and verification.get('basis')=='observed_memory_safe_approach':
        evidence['basis']='historical_position_and_observed_map_safe_approach'
        evidence['predicted_time_only']=True
    evidence.update(order_changed=ordered!=baseline,
        verification_views=[verification]if verification else [],
        added_viewpoints=int(verification is not None),
        predicted_view_scores=[dict(configured_index=i,score=score(p)) for i,p in enumerate(baseline)])
    return ordered,evidence
