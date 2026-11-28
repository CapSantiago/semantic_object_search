import pytest
from semantic_search.task_sequence import split_sequence


def test_explicit_order_and_single_task():
    assert split_sequence('请先去厨房统计杯子，然后去卧室找书，最后回充。') == [
        '去厨房统计杯子','去卧室找书','回充']
    assert split_sequence('先去厨房找杯子，然后再去卧室找书') == ['去厨房找杯子','去卧室找书']
    assert split_sequence('去厨房找杯子再去卧室统计杯子') == ['去厨房找杯子','去卧室统计杯子']
    assert split_sequence('再去厨房找杯子') is None
    assert split_sequence('去厨房找杯子') is None
    assert split_sequence('先去厨房找杯子') is None


@pytest.mark.parametrize('command', ['先找杯子再','先再找杯子', '先A再B再C再D'])
def test_reject_empty_or_excessive_clauses(command):
    with pytest.raises(ValueError):split_sequence(command)
