"""Explain recorded search decisions without inferring absence or physical identity."""


def guidance_trace(guidance, diagnostics, *, count=0):
    if not guidance.get('used'):
        return None
    successes = guidance.get('historical_successful_views', [])
    rows = []
    for view in diagnostics:
        pose = view.get('pose', [])
        sources = [s for s in successes if s.get('pose') == pose]
        rows.append(dict(index=view['index'], pose=pose,
            historical_sources=[{k:s.get(k) for k in ('memory_id','task_id','frames')} for s in sources],
            reached=view.get('reached', False),
            dwell_completed=view.get('dwell_completed', False),
            valid_projections=view.get('valid_projections', 0),
            navigation_status=view.get('navigation_status')))
    return dict(basis=guidance.get('basis'), order_changed=guidance.get('order_changed', False),
        fallback=guidance.get('fallback'), history=guidance.get('historical_records', []),
        live_confirmed_count=count, counts_from_history=False, views=rows,
        physical_identity_verified=False)
