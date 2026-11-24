"""Shared object vocabulary and admission state for commands, memory and UI."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class ObjectType:
    category: str
    name: str
    coco_id: int
    aliases: tuple
    portable: bool = True
    admitted: bool = False


OBJECTS = {item.category: item for item in (
    ObjectType('cup', '杯子', 41, ('杯子', '水杯', '马克杯', 'cup', 'cups'), admitted=True),
    ObjectType('bottle', '瓶子', 39, ('瓶子', '水瓶', '可乐', 'bottle', 'bottles'), admitted=True),
    ObjectType('chair', '椅子', 56, ('椅子', 'chair', 'chairs'), portable=False, admitted=True),
    ObjectType('bowl', '碗', 45, ('碗', 'bowl', 'bowls')),
    ObjectType('book', '书', 73, ('书', '本子', 'book', 'books')),
    ObjectType('cell phone', '手机', 67, ('手机', 'cell phone', 'cell phones', 'phone')),
    ObjectType('vase', '花瓶', 75, ('花瓶', 'vase', 'vases')),
    ObjectType('tv', '电视', 62, ('电视', 'tv', 'television'), portable=False),
    ObjectType('refrigerator', '冰箱', 72, ('冰箱', 'refrigerator', 'fridge'), portable=False),
    ObjectType('couch', '沙发', 57, ('沙发', 'couch', 'sofa'), portable=False),
    ObjectType('bed', '床', 59, ('床', 'bed', 'beds'), portable=False),
    ObjectType('dining table', '餐桌', 60, ('餐桌', 'dining table', 'dining tables'), portable=False),
    ObjectType('apple', '苹果', 47, ('苹果', 'apple', 'apples')),
    ObjectType('banana', '香蕉', 46, ('香蕉', 'banana', 'bananas')),
    ObjectType('orange', '橙子', 49, ('橙子', 'orange', 'oranges')),
    ObjectType('sports ball', '球', 32, ('球', 'sports ball', 'ball', 'balls')),
)}


def admitted_aliases():
    return {key: item.aliases for key, item in OBJECTS.items() if item.admitted}


def validate_categories(categories):
    if not isinstance(categories, (list, tuple)) or not categories:
        raise ValueError('物品类别必须为非空列表。')
    if any(not isinstance(key, str) or key not in OBJECTS for key in categories):
        raise ValueError('物品类别不在当前 16 类目录中。')
    return tuple(dict.fromkeys(categories))


def aliases_for(categories=None):
    selected = validate_categories(categories) if categories is not None else admitted_aliases()
    return {key: OBJECTS[key].aliases for key in selected}


def object_catalog(validation_categories=()):
    return [dict(asdict(item), validation_enabled=item.category in validation_categories and not item.admitted)
            for item in OBJECTS.values()]
