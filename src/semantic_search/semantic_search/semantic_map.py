from dataclasses import dataclass
import json
from math import atan2, cos, hypot, sin
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment


OBJECT_ALIASES = {
    'chair': ('椅子', 'chair', 'chairs'),
    'cup': ('杯子', '水杯', 'cup', 'cups'),
    'bottle': ('瓶子', '水瓶', '可乐', 'bottle', 'bottles'),
}


def parse_command(text, rooms):
    """Accept constrained text or a validated JSON task from an input adapter."""
    import re

    text = text.strip()
    if text.startswith('{'):
        try:
            task = json.loads(text)
        except json.JSONDecodeError as error:
            raise ValueError('任务 JSON 格式错误。') from error
        if not isinstance(task, dict) or set(task) - {'mode', 'room', 'category', 'request_id'}:
            raise ValueError('任务 JSON 只接受 mode、room、category 和可选的 request_id 字段。')
        if 'request_id' in task and (not isinstance(task['request_id'], str)
                or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}', task['request_id'])):
            raise ValueError('任务 request_id 格式无效。')
        mode = task.get('mode')
        if mode == 'cancel':
            if set(task) - {'mode', 'request_id'}:
                raise ValueError('取消任务 JSON 只接受 mode 和可选的 request_id 字段。')
            return 'cancel', None, None
        if mode not in ('find', 'count'):
            raise ValueError('任务 mode 必须为 find、count 或 cancel。')
        room, category = task.get('room'), task.get('category')
        if not isinstance(room, str) or room not in rooms:
            raise ValueError('任务 room 必须是当前场景配置中的房间 ID。')
        if not isinstance(category, str) or category not in OBJECT_ALIASES:
            raise ValueError('任务 category 必须为 chair、cup 或 bottle。')
        return mode, room, category

    text = text.lower()
    if text in ('取消', '停止', 'cancel', 'stop'):
        return 'cancel', None, None

    def includes(alias):
        if alias.isascii():
            return re.search(r'(?<!\w)' + re.escape(alias) + r'(?!\w)', text) is not None
        return alias in text

    room_ids = [key for key, room in rooms.items() if any(map(includes, room['aliases']))]
    categories = [key for key, aliases in OBJECT_ALIASES.items() if any(map(includes, aliases))]
    if len(room_ids) != 1 or len(categories) != 1:
        raise ValueError('请指定且仅指定一个支持的房间和物品，例如：去厨房统计椅子。')
    if any(map(includes, ('统计', '清点', '有几个', '多少', 'count'))):
        mode = 'count'
    elif any(map(includes, ('找', '搜寻', '寻找', 'find', 'search'))):
        mode = 'find'
    else:
        raise ValueError('请使用“找”或“统计”指令。')
    return mode, room_ids[0], categories[0]


@dataclass
class Landmark:
    id: int
    x: float
    y: float
    z: float
    hits: int = 1

    def update(self, point):
        self.hits += 1
        self.x += (point[0] - self.x) / self.hits
        self.y += (point[1] - self.y) / self.hits
        self.z += (point[2] - self.z) / self.hits


class LandmarkMap:
    """One-to-one association in each frame, with map-plane gating across frames."""

    def __init__(self, radius, min_hits):
        self.radius = radius
        self.min_hits = min_hits
        self.items = []

    def observe(self, points):
        used_points = set()
        point_ids = [None] * len(points)
        if points and self.items:
            # Prefer the largest feasible one-to-one matching, then minimize
            # total distance. Dummy columns let any point start a new track.
            # A nearest-first greedy match can unnecessarily create duplicates.
            unmatched = (len(points) + 1) * self.radius + 1.0
            costs = np.full((len(points), len(self.items) + len(points)), unmatched)
            for index, point in enumerate(points):
                for item_index, item in enumerate(self.items):
                    distance = hypot(point[0] - item.x, point[1] - item.y)
                    costs[index, item_index] = distance if distance <= self.radius else 2*unmatched
            rows, columns = linear_sum_assignment(costs)
            for index, item_index in zip(rows, columns):
                if item_index < len(self.items) and costs[index, item_index] <= self.radius:
                    self.items[item_index].update(points[index])
                    used_points.add(index)
                    point_ids[index] = self.items[item_index].id
        for index, point in enumerate(points):
            if index not in used_points:
                self.items.append(Landmark(len(self.items) + 1, *point))
                point_ids[index] = self.items[-1].id
        return point_ids

    @property
    def confirmed(self):
        return [item for item in self.items if item.hits >= self.min_hits]


def write_results(folder, result, grid):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'result.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    if grid is None:
        return

    info = grid.info
    width, height = info.width, info.height
    origin = info.origin
    q = origin.orientation
    yaw = atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
           f'<rect width="{width}" height="{height}" fill="#b8c0c8"/>']
    for row in range(height):
        cells = grid.data[row*width:(row+1)*width]
        start = 0
        while start < width:
            value = cells[start]
            end = start + 1
            while end < width and cells[end] == value:
                end += 1
            if value >= 0:
                shade = 255 - round(220 * value / 100)
                svg.append(f'<rect x="{start}" y="{height-row-1}" width="{end-start}" '
                           f'height="1" fill="rgb({shade},{shade},{shade})"/>')
            start = end

    for item in result['objects']:
        dx, dy = item['x'] - origin.position.x, item['y'] - origin.position.y
        x = (cos(yaw)*dx + sin(yaw)*dy) / info.resolution
        y = height - (-sin(yaw)*dx + cos(yaw)*dy) / info.resolution
        svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2" fill="#e53935"/>')
        svg.append(f'<text x="{x+2.5:.2f}" y="{y:.2f}" font-size="4" '
                   f'fill="#b71c1c">{item["id"]}</text>')
    svg.append('</svg>')
    (folder / 'semantic_map.svg').write_text('\n'.join(svg), encoding='utf-8')
