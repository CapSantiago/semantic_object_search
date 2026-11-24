"""Audit live moving-obstacle stops without requiring the robot to approach it.

This is a dynamics evidence check, not a replacement for independent task,
contact, sensor-age or docking acceptance. Fixture positions only evaluate the
already executed trial and never enter robot control.
"""
import math


def audit_moving_yield(report, trajectory):
    begin = report.get('actor_trigger_wall', math.inf)
    clear = report.get('actor_clear_wall', -math.inf)
    previous_plan = report.get('pre_trigger_plan') or {}
    points = previous_plan.get('points') or []
    fixture = report.get('fixture') or {}
    centre = fixture.get('centre') or []
    size = fixture.get('size') or []
    diagnostics = ((report.get('task') or {}).get('result') or {}).get('viewpoint_diagnostics') or []
    goals = [v['pose'][:2] for v in diagnostics]
    if points:
        goals.append(points[-1][:2])  # Also exclude a normal room-entry stop.
    path_blocked = bool(len(centre) >= 2 and len(size) >= 2 and any(
        all(abs(p[i] - centre[i]) <= size[i] / 2 for i in range(2)) for p in points))
    observations = [v for v in report.get('obstacle_observations', []) if begin <= v['wall'] <= clear]
    marked = [v for v in observations if v['centre_cost'] >= 99]
    first_mark = min((v['wall'] for v in marked), default=math.inf)
    transition = any(0 <= v['centre_cost'] < 99 and v['wall'] < first_mark for v in observations)
    groups = []
    for row in trajectory:
        if not begin <= row['wall'] <= clear:
            continue
        eligible = (abs(row['v']) < .05 and goals and 'map_pose' in row
                    and min(math.dist(row['map_pose'][:2], goal) for goal in goals) > .3)
        if not eligible:
            if groups and groups[-1] and groups[-1][-1] is not None:
                groups.append([])
            continue
        if not groups or not groups[-1] or row['stamp'] - groups[-1][-1]['stamp'] > .2:
            groups.append([])
        groups[-1].append(row)
    stops = []
    uncorroborated_stops = []
    for group in groups:
        if not group:
            continue
        first, last = group[0], group[-1]
        duration = last['stamp'] - first['stamp']
        # Native costs must have appeared by the end of the stop and be from
        # that sim-time window. A stale occupied cell cannot prove a response.
        seen = any(v['wall'] <= last['wall'] and abs(last['stamp'] - v['stamp']) <= 1.5 for v in marked)
        if duration >= .2 and seen:
            stops.append(dict(start_wall=first['wall'], end_wall=last['wall'],
                duration_sim_seconds=duration,
                minimum_goal_distance_m=min(math.dist(r['map_pose'][:2], g) for r in group for g in goals),
                minimum_odom_distance_to_nominal_crossing_m=min(math.dist(r['odom_xy'], centre) for r in group)))
        elif duration >= .2:
            uncorroborated_stops.append(dict(start_wall=first['wall'],end_wall=last['wall'],
                duration_sim_seconds=duration,
                reason='No occupied snapshot from this sim-time window within1.5s by the end of the stop.'))
    resumed = []
    for row in trajectory:
        if row['wall'] <= clear:
            continue
        if abs(row['v']) <= .05:
            if resumed and resumed[-1]: resumed.append([])
            continue
        if not resumed or not resumed[-1] or row['stamp'] - resumed[-1][-1]['stamp'] > .2:
            resumed.append([])
        resumed[-1].append(row)
    resume_duration = max((g[-1]['stamp'] - g[0]['stamp'] for g in resumed if g), default=0.)
    checks = dict(actor_crossing_finished=math.isfinite(begin) and clear > begin,
        robot_was_moving_before_fixture=report.get('trigger_velocity_mps', 0.) > .08,
        published_path_fresh_at_trigger=bool(points) and 0 <= begin - previous_plan.get('wall', -math.inf) <= 2.,
        obstacle_on_actual_published_path=path_blocked,
        native_cost_changed_to_occupied=transition and bool(marked),
        measured_near_stop_away_from_any_goal=bool(stops),
        resumed_after_actor_clear=resume_duration >= .45)
    return dict(yield_evidence_passed=all(checks.values()), checks=checks, stops=stops,
        uncorroborated_stops=uncorroborated_stops,
        resume_duration_sim_seconds=resume_duration,
        thresholds=dict(near_stop_mps=.05, stop_duration_seconds=.2, resume_seconds=.45, goal_exclusion_m=.3),
        scope='Dynamics evidence only; combine with task correctness, calibrated contact, full sensor and dock gates.',
        proximity_note='No upper approach-distance requirement: stopping earlier is not a failure. Native obstacle transition, actual path and goal exclusion provide context.')
