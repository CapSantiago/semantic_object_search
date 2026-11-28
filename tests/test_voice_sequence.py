from types import SimpleNamespace
from semantic_search.voice import transcribe


class Model:
    def __init__(self,*responses):self.responses=iter(responses);self.calls=[]
    def transcribe(self,samples,**kwargs):
        self.calls.append(kwargs)
        return iter([SimpleNamespace(text=next(self.responses))]),None


def test_single_and_silence_keep_existing_decode():
    for original in ('去厨房统计椅子。',''):
        model=Model(original)
        assert transcribe(model,[])==original
        assert len(model.calls)==1 and 'temperature' not in model.calls[0]


def test_compound_decodes_again_without_posthoc_word_replacement():
    model=Model('请线区客厅寻找相较然后去客厅统计相较。',
                '请先去 客厅 寻找 香蕉 然后 去 客厅 统计 香蕉。')
    assert transcribe(model,[])=='请先去 客厅 寻找 香蕉 然后 去 客厅 统计 香蕉。'
    assert len(model.calls)==2


def test_second_decode_cannot_silently_drop_the_other_task():
    original='去厨房找杯子然后去卧室统计杯子。'
    assert transcribe(Model(original,'去厨房找杯子。'),[])==original


def test_mapping_keeps_its_original_decode():
    model=Model('前进')
    assert transcribe(model,[],context='mapping')=='前进'
    assert len(model.calls)==1 and 'temperature' not in model.calls[0]
