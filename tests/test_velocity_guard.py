import math

import pytest

from semantic_search.velocity_guard import VelocityLease


def lease():
    value = VelocityLease()
    for name in value.sources:
        assert value.source(name,10.,1.)
    value.velocity((.2,0.,.3),1.)
    return value


def evaluate(value, sim=10.1, wall=1.1, **options):
    return value.evaluate(sim,wall,transform_ready=options.get('tf',True),
                          filter_active=options.get('active',True))


def test_fresh_filtered_velocity_passes_unchanged_and_idle_stops_without_losing_sensor_readiness():
    value = lease()
    assert evaluate(value)==((.2,0.,.3),True,[])
    for name in value.sources:
        value.source(name,10.35,1.35)
    output,ready,reasons = evaluate(value,10.35,1.35)
    assert output==(0.,0.,0.) and ready and reasons==['command_stale']


@pytest.mark.parametrize('source',VelocityLease.sources)
def test_each_missing_or_replayed_sensor_blocks_new_velocity(source):
    value = lease()
    evaluate(value)
    for name in value.sources:
        value.source(name,10.6,1.6) if name!=source else value.source(name,10.,1.6)
    value.velocity((.2,0.,.3),1.6)
    output,ready,reasons = evaluate(value,10.6,1.6)
    assert output==(0.,0.,0.) and not ready and source+'_stale' in reasons


@pytest.mark.parametrize('options,reason',[({'tf':False},'odometry_tf_stale'),
                                           ({'active':False},'collision_monitor_inactive')])
def test_invalid_tf_or_inactive_filter_blocks_motion(options,reason):
    output,ready,reasons = evaluate(lease(),**options)
    assert output==(0.,0.,0.) and not ready and reason in reasons


def test_clock_pause_and_rewind_cannot_replay_a_command():
    value = lease()
    evaluate(value)
    value.velocity((.2,0.,0.),1.6)
    assert 'clock_stale' in evaluate(value,10.1,1.6)[2]
    assert evaluate(value,2.,1.7)[0]==(0.,0.,0.)
    for name in value.sources:
        value.source(name,2.1,1.8)
    assert evaluate(value,2.1,1.8)[0]==(0.,0.,0.)
    value.velocity((.1,0.,0.),1.8)
    assert evaluate(value,2.2,1.9)[0]==(.1,0.,0.)


def test_invalid_or_expired_commands_never_leave_previous_motion_active():
    value = lease()
    value.velocity((math.nan,0.,0.),1.1)
    assert evaluate(value)[0]==(0.,0.,0.)
    value.velocity((.2,0.,3.),1.1)
    assert evaluate(value)[0]==(0.,0.,0.)
    value.velocity((.2,0.,0.),.5)
    assert evaluate(value)[0]==(0.,0.,0.)
