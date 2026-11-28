from semantic_search.memory_trace import guidance_trace


def test_no_history_does_not_claim_guidance():
    assert guidance_trace(dict(used=False), []) is None


def test_history_is_evidence_never_current_count_or_absence():
    history = dict(id='old', state='needs_verification', x=1., y=2.)
    guidance = dict(used=True, historical_records=[history],
        historical_successful_views=[dict(memory_id='old', task_id='actual-task',
            frames=31, pose=[2.,3.,0.])], fallback='all_configured_views_retained')
    views = [dict(index=0, pose=[2.,3.,0.], reached=True,
                  dwell_completed=True, valid_projections=0),
             dict(index=1, pose=[1.,0.,1.], reached=False)]
    trace = guidance_trace(guidance, views)
    assert trace['live_confirmed_count'] == 0
    assert trace['history'] == [history]
    assert trace['views'][0]['historical_sources'][0]['task_id'] == 'actual-task'
    assert trace['views'][0]['valid_projections'] == 0
    assert trace['views'][1]['historical_sources'] == []
    assert not trace['counts_from_history'] and not trace['physical_identity_verified']
    assert guidance_trace(guidance, views, count=1)['live_confirmed_count'] == 1
