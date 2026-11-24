"""Real saved trials expose goal-stop false positives and early-stop omissions."""
from copy import deepcopy
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/validation'))
from audit_moving_yield import audit_moving_yield

FIXTURE = json.loads((Path(__file__).parent/'fixtures/moving_yield_20261124.json').read_text())


def test_real_early_stop_does_not_require_approaching_within_old_cutoff():
    case = FIXTURE['early_stop']
    result = audit_moving_yield(case['report'], case['trajectory'])
    assert result['yield_evidence_passed']
    assert all(s['minimum_odom_distance_to_nominal_crossing_m'] > 1.5 for s in result['stops'])
    assert result['thresholds']['near_stop_mps'] == .05
    assert result['thresholds']['stop_duration_seconds'] == .2


def test_real_observation_goal_stop_does_not_prove_yielding():
    case = FIXTURE['goal_stop']
    result = audit_moving_yield(case['report'], case['trajectory'])
    assert not result['yield_evidence_passed']
    assert not result['checks']['measured_near_stop_away_from_any_goal']


def test_other_long_stop_without_fresh_cost_snapshot_is_separate():
    case = FIXTURE['early_stop']
    result = audit_moving_yield(case['report'], case['trajectory'])
    assert result['yield_evidence_passed']
    assert all(s['duration_sim_seconds'] < .5 for s in result['stops'])
    assert any(s['duration_sim_seconds'] > .5 for s in result['uncorroborated_stops'])


def test_stop_without_native_obstacle_transition_is_unproven():
    case = deepcopy(FIXTURE['early_stop'])
    for observation in case['report']['obstacle_observations']:
        observation['centre_cost'] = 0
    result = audit_moving_yield(case['report'], case['trajectory'])
    assert not result['yield_evidence_passed']
    assert not result['checks']['native_cost_changed_to_occupied']


def test_missing_actual_published_path_cannot_prove_obstacle_encounter():
    case = deepcopy(FIXTURE['early_stop'])
    case['report'].pop('pre_trigger_plan')
    assert not audit_moving_yield(case['report'], case['trajectory'])['yield_evidence_passed']
