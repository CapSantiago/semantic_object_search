"""Room geometry shared by search and future coverage tasks (map metres)."""
from math import isfinite

EPS = 1e-9


def _on_segment(p, a, b):
    cross = (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0])
    return (abs(cross) <= EPS and
            min(a[0], b[0])-EPS <= p[0] <= max(a[0], b[0])+EPS and
            min(a[1], b[1])-EPS <= p[1] <= max(a[1], b[1])+EPS)


def polygon_contains(polygon, x, y):
    """Ray crossing including the boundary; supports concave simple polygons."""
    if not (isfinite(x) and isfinite(y)):
        return False
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if _on_segment((x, y), a, b):
            return True
        if (a[1] > y) != (b[1] > y):
            if x < a[0] + (y-a[1])*(b[0]-a[0])/(b[1]-a[1]):
                inside = not inside
    return inside


def contains(room, x, y):
    if not (isfinite(x) and isfinite(y)):
        return False
    if 'polygon' in room:
        return (polygon_contains(room['polygon'], x, y) and
                not any(polygon_contains(hole, x, y) for hole in room.get('holes', [])))
    xmin, ymin, xmax, ymax = room['bounds']
    return xmin <= x <= xmax and ymin <= y <= ymax


def room_at(rooms, x, y):
    """Stable ownership for polygon boundaries/overlaps, independent of JSON order."""
    return next((key for key in sorted(rooms) if contains(rooms[key], x, y)), None)


def belongs_to(rooms, target, x, y):
    # Preserve inclusive legacy rectangle semantics for archived scenes.
    if not any('polygon' in r for r in rooms.values()):
        return contains(rooms[target], x, y)
    return room_at(rooms, x, y) == target


def observation_views(room, category):
    """Optional tabletop category views; archived configurations keep their route."""
    route=room.get('category_views', {}).get(category, room['views'])
    if not room.get('inspection_views'):return route
    ordered=[p for poses in room['inspection_views'].values() for p in poses]+route
    return [p for i,p in enumerate(ordered) if p not in ordered[:i]]


def inspection_coverage(room, diagnostics):
    """Configured areas require actual arrival, dwell and usable sensor input."""
    groups=[]
    for name,poses in room.get('inspection_views',{}).items():
        matching=[[v for v in diagnostics if all(abs(a-b)<1e-6 for a,b in zip(v['pose'],pose))]
                  for pose in poses]
        complete=all(any(observation_coverage([v])['viewpoints_observed']==1 for v in views)
                     for views in matching)
        groups.append(dict(id=name,name=room.get('inspection_labels',{}).get(name,name),
                           completed=complete,required_views=len(poses)))
    return dict(required=bool(groups),completed=all(g['completed'] for g in groups),groups=groups)


def observation_coverage(viewpoints):
    """Separate route execution from data availability, never infer room coverage."""
    incomplete = []
    observed = 0
    for view in viewpoints:
        reasons = []
        if not view['reached']:
            # action_msgs/GoalStatus: 5 = CANCELED, 6 = ABORTED. A non-null
            # result alone does not prove failure (success can race a cancel).
            if view.get('task_canceled', False) or view['navigation_status'] == 5:
                reasons.append('canceled')
            elif view['navigation_status'] == 6:
                reasons.append('navigation_failed')
            else:
                reasons.append('not_visited')
        else:
            if not view['dwell_completed']:
                reasons.append('dwell_incomplete')
            if not view['synchronized_frames']:
                reasons.append('no_sensor_data')
            elif not view['valid_projections'] and (view['tf_failures'] or view['invalid_target_depth']):
                reasons.append('localization_failed')
        if reasons:
            incomplete.append(dict(index=view['index'], reasons=reasons))
        else:
            observed += 1
    arrived = bool(viewpoints) and all(view['reached'] for view in viewpoints)
    plan_completed = arrived and all(view['dwell_completed'] for view in viewpoints)
    data_complete = arrived and all(
        view['synchronized_frames'] > 0 and
        (view['valid_projections'] > 0 or not (view['tf_failures'] or view['invalid_target_depth']))
        for view in viewpoints)
    return dict(arrival_plan_completed=arrived, observation_plan_completed=plan_completed,
                observation_data_complete=data_complete, viewpoints_observed=observed,
                incomplete_viewpoints=incomplete, full_room_coverage_verified=False,
                basis='configured_viewpoints_only')


def observation_status(mode, frames, confirmed, projections, tf_errors, depth_errors, failed_views,
                       viewpoint_diagnostics=None):
    """Do not turn an unreachable view into an absence or a total-room count."""
    if mode == 'find' and confirmed:
        return 'found'
    if not frames:
        if viewpoint_diagnostics is not None and any(view['reached'] for view in viewpoint_diagnostics):
            return 'no_sensor_data'
        return 'navigation_failed' if failed_views else 'no_sensor_data'
    if not projections and (tf_errors or depth_errors):
        return 'localization_failed'
    if failed_views:
        return 'partial'
    if viewpoint_diagnostics is not None:
        coverage = observation_coverage(viewpoint_diagnostics)
        if not (coverage['observation_plan_completed'] and coverage['observation_data_complete']):
            return 'partial'
    return 'completed' if confirmed else 'not_found'


def validate_rooms(rooms):
    """Reject malformed geometry before starting a robot task."""
    if not rooms:
        raise ValueError('Room configuration is empty')
    for key, room in rooms.items():
        bounds = room['bounds']
        if (len(bounds) != 4 or not all(isfinite(v) for v in bounds) or
                bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
            raise ValueError(f'{key}: invalid bounds')
        for polygon in ([room['polygon']] if 'polygon' in room else []) + room.get('holes', []):
            if len(polygon) < 3 or any(len(p) != 2 or not all(isfinite(v) for v in p) for p in polygon):
                raise ValueError(f'{key}: invalid polygon')
            area = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(polygon, polygon[1:]+polygon[:1]))
            if abs(area) <= EPS:
                raise ValueError(f'{key}: zero polygon area')
            edges = list(zip(polygon, polygon[1:]+polygon[:1]))
            for i, (a, b) in enumerate(edges):
                if a == b:
                    raise ValueError(f'{key}: duplicate polygon vertex')
                for j, (c, d) in enumerate(edges):
                    if j <= i+1 or (i == 0 and j == len(edges)-1):
                        continue
                    def cross(p, q, r):
                        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
                    crossing = cross(a,b,c)*cross(a,b,d) < 0 and cross(c,d,a)*cross(c,d,b) < 0
                    if crossing or any((_on_segment(c,a,b), _on_segment(d,a,b),
                                        _on_segment(a,c,d), _on_segment(b,c,d))):
                        raise ValueError(f'{key}: self-intersecting polygon')
        categories = room.get('category_views', {})
        if not isinstance(categories, dict) or any(not isinstance(k,str) or not v for k,v in categories.items()):
            raise ValueError(f'{key}: invalid category observation views')
        inspection=room.get('inspection_views',{})
        if (not isinstance(inspection,dict) or any(not isinstance(k,str) or not k or
                not isinstance(v,list) or not v for k,v in inspection.items())):
            raise ValueError(f'{key}: invalid required inspection views')
        for pose in room['views'] + [p for views in categories.values() for p in views] + [p for views in inspection.values() for p in views]:
            if len(pose) != 3 or not all(isfinite(v) for v in pose):
                raise ValueError(f'{key}: invalid observation pose')
