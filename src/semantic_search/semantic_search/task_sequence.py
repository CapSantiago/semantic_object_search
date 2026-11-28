"""Bounded explicit-order language; each clause is validated by existing parsers."""
import re


def split_sequence(text):
    if not isinstance(text, str) or len(text) > 500:
        raise ValueError('任务文字长度应为1–500字符')
    text = text.strip()
    prefixed = bool(re.match(r'^(?:请)?先', text))
    text = re.sub(r'^(?:请)?先', '', text, count=1)
    parts = re.split(r'[，,；;。\s]*(?:然后再?|接着再?|最后再?|再)[，,；;\s]*', text)
    if len(parts) == 1 or (not prefixed and len(parts)==2 and not parts[0].strip()):
        return None
    if not 2 <= len(parts) <= 3 or any(not p.strip('，,；;。 \n') for p in parts):
        raise ValueError('组合任务需要2–3条完整指令，请使用“先……再……”。')
    return [p.strip('，,；;。 \n') for p in parts]
