"""Persistent, evidence-backed spatial records. No simulator state is consumed.

Spatial association preserves records near a previous observation; it cannot
establish physical identity after an object moves. Historical records are never
substituted for a current search count, or removed on an empty/partial search.
"""
from contextlib import closing
from datetime import datetime, timezone
import hashlib
import json
from math import dist, isfinite
from pathlib import Path
import sqlite3
from uuid import uuid4

import numpy as np
from scipy.optimize import linear_sum_assignment

from .objects import OBJECTS


def _json(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False, separators=(',', ':'))


def map_namespace(grid, rooms, scene='aws_house'):
    """Fingerprint observed occupancy geometry and room ownership, not fixtures.

    Timestamps, initial pose, view calibration and target placements are not map
    identity. Changing grid coordinates or room ownership isolates old memory.
    """
    if not grid.get('available') or grid.get('frame_id') != 'map':
        raise ValueError('需要 map 坐标系的有效地图。')
    width, height = grid.get('width'), grid.get('height')
    if (type(width) is not int or type(height) is not int or min(width, height) <= 0
            or len(grid.get('data', [])) != width * height):
        raise ValueError('地图尺寸或栅格数量无效。')
    geometry = {key: grid[key] for key in ('frame_id', 'width', 'height', 'resolution', 'origin', 'data')}
    partitions = {key: {field: room[field] for field in ('polygon', 'holes', 'bounds') if field in room}
                  for key, room in rooms.items()}
    return hashlib.sha256(_json(dict(scene=scene, map=geometry, rooms=partitions)).encode()).hexdigest()


class SemanticMemory:
    """SQLite transactions make task imports idempotent across threads/restarts."""

    def __init__(self, path, radius=.25, height_gate=.25, ambiguity_margin=.05):
        if not all(isfinite(v) and v > 0 for v in (radius, height_gate, ambiguity_margin)):
            raise ValueError('记忆关联阈值必须为有限正数。')
        self.path = Path(path)
        self.radius, self.height_gate, self.ambiguity_margin = radius, height_gate, ambiguity_margin

    def _connect(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        connection = sqlite3.connect(self.path, timeout=5)
        connection.row_factory = sqlite3.Row
        try:
            version = connection.execute('PRAGMA user_version').fetchone()[0]
            if version not in (0, 1):
                raise ValueError('语义记忆版本不兼容，保留原文件供迁移。')
            connection.executescript('''
                CREATE TABLE IF NOT EXISTS tasks (
                    namespace TEXT NOT NULL, task_id TEXT NOT NULL, digest TEXT NOT NULL,
                    category TEXT NOT NULL, room TEXT NOT NULL,
                    observed_at TEXT NOT NULL, evidence TEXT NOT NULL, result TEXT NOT NULL,
                    PRIMARY KEY (namespace, task_id));
                CREATE TABLE IF NOT EXISTS objects (
                    id TEXT PRIMARY KEY, namespace TEXT NOT NULL, category TEXT NOT NULL,
                    room TEXT NOT NULL, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
                    first_seen TEXT NOT NULL, last_seen TEXT NOT NULL, last_checked TEXT NOT NULL,
                    observation_tasks INTEGER NOT NULL, hits INTEGER NOT NULL, task_id TEXT NOT NULL,
                    evidence TEXT NOT NULL, state TEXT NOT NULL, association TEXT NOT NULL);
                CREATE INDEX IF NOT EXISTS objects_lookup ON objects (namespace, category, room);
                CREATE TABLE IF NOT EXISTS observations (
                    namespace TEXT NOT NULL, task_id TEXT NOT NULL, local_id INTEGER NOT NULL,
                    memory_id TEXT NOT NULL, PRIMARY KEY(namespace, task_id, local_id));
                PRAGMA user_version=1;
            ''')
        except BaseException:
            connection.close()
            raise
        return connection

    def ingest(self, namespace, result, *, observed_at, evidence):
        """Import a robot result only, retaining its original time and evidence."""
        task_id, category, room = (result.get(key) for key in ('task_id', 'category', 'room'))
        if not all(isinstance(v, str) and v for v in (namespace, task_id, category, room, evidence)):
            raise ValueError('记忆需要地图版本、任务、类别、房间和证据路径。')
        if category not in OBJECTS or result.get('frame_id') != 'map' or result.get('coordinate_unit') != 'm':
            raise ValueError('仅接受已定义类别的 map 米制感知结果。')
        if result.get('status') not in ('found', 'completed', 'not_found', 'partial', 'canceled'):
            raise ValueError('该任务状态不能写入物品记忆。')
        if result.get('mode') not in ('find', 'count'):
            raise ValueError('记忆只接受寻找或盘点结果。')
        stamp = datetime.fromisoformat(observed_at)
        if stamp.tzinfo is None:
            raise ValueError('观测时间必须含时区。')
        observed_at = stamp.astimezone(timezone.utc).isoformat(timespec='microseconds')
        points = result.get('objects')
        if (not isinstance(points, list) or type(result.get('count')) is not int
                or result['count'] != len(points)):
            raise ValueError('数量与物品记录不一致。')
        seen = set()
        for point in points:
            if (not isinstance(point, dict) or type(point.get('id')) is not int
                    or point['id'] <= 0 or point['id'] in seen
                    or type(point.get('hits')) is not int or point['hits'] < 1
                    or any(type(point.get(k)) not in (int, float) or not isfinite(point[k]) for k in ('x', 'y', 'z'))):
                raise ValueError('物品编号、命中次数或坐标无效。')
            seen.add(point['id'])
        digest = hashlib.sha256(_json(result).encode()).hexdigest()
        with closing(self._connect()) as db, db:
            db.execute('BEGIN IMMEDIATE')
            old = db.execute('SELECT digest FROM tasks WHERE namespace=? AND task_id=?',
                             (namespace, task_id)).fetchone()
            if old:
                if old['digest'] != digest:
                    raise ValueError('同一任务 ID 的内容变化，拒绝覆盖原证据。')
                return dict(status='already_saved', task_id=task_id, objects=len(points))
            latest = db.execute('SELECT MAX(observed_at) FROM tasks WHERE namespace=? AND category=? AND room=?',
                                (namespace, category, room)).fetchone()[0]
            if latest and observed_at < latest:
                raise ValueError('旧观测不能覆盖较新的记忆；请按原始时间顺序导入。')
            rows = db.execute('SELECT * FROM objects WHERE namespace=? AND category=? AND room=? ORDER BY id',
                              (namespace, category, room)).fetchall()
            assignments, ambiguous = {}, set()
            if points and rows:
                unreachable = (len(points)+1)*self.radius+1
                costs = np.full((len(points), len(rows)+len(points)), unreachable)
                for i, point in enumerate(points):
                    for j, row in enumerate(rows):
                        distance = dist((point['x'], point['y']), (row['x'], row['y']))
                        if distance <= self.radius and abs(point['z']-row['z']) <= self.height_gate:
                            costs[i, j] = distance
                    candidates = sorted(costs[i, :len(rows)][costs[i, :len(rows)] <= self.radius])
                    if len(candidates) > 1 and candidates[1]-candidates[0] < self.ambiguity_margin:
                        ambiguous.add(i)
                for j in range(len(rows)):
                    candidates = sorted((costs[i, j], i) for i in range(len(points)) if costs[i, j] <= self.radius)
                    if len(candidates) > 1 and candidates[1][0]-candidates[0][0] < self.ambiguity_margin:
                        ambiguous.update(i for _, i in candidates)
                for i in ambiguous:
                    costs[i, :len(rows)] = unreachable
                assigned_rows, assigned_columns = linear_sum_assignment(costs)
                assignments = {int(i): rows[j]['id'] for i, j in zip(assigned_rows, assigned_columns)
                               if j < len(rows) and costs[i, j] <= self.radius}
            db.execute('INSERT INTO tasks VALUES (?, ?, ?, ?, ?, ?, ?, ?)',
                       (namespace, task_id, digest, category, room, observed_at, evidence, _json(result)))
            coverage = result.get('coverage') or {}
            if (result['mode'] == 'count' and result['status'] in ('completed', 'not_found')
                    and coverage.get('observation_plan_completed') is True
                    and coverage.get('observation_data_complete') is True):
                db.execute("UPDATE objects SET state='needs_verification', last_checked=? "
                           'WHERE namespace=? AND category=? AND room=?',
                           (observed_at, namespace, category, room))
            for i, point in enumerate(points):
                memory_id = assignments.get(i)
                if memory_id:
                    db.execute("UPDATE objects SET x=?, y=?, z=?, last_seen=?, last_checked=?, "
                               "observation_tasks=observation_tasks+1, hits=?, task_id=?, evidence=?, "
                               "state='last_observed', association='spatial' WHERE id=?",
                               (point['x'], point['y'], point['z'], observed_at, observed_at,
                                point['hits'], task_id, evidence, memory_id))
                else:
                    memory_id = uuid4().hex
                    db.execute('INSERT INTO objects VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)',
                               (memory_id, namespace, category, room, point['x'], point['y'], point['z'],
                                observed_at, observed_at, observed_at, 1, point['hits'], task_id, evidence,
                                'last_observed', 'ambiguous' if i in ambiguous else 'new_spatial_record'))
                db.execute('INSERT INTO observations VALUES (?, ?, ?, ?)',
                           (namespace, task_id, point['id'], memory_id))
        return dict(status='saved', task_id=task_id, objects=len(points))

    def successful_viewpoints(self, namespace, records, *, root):
        """Read historical instance/view associations; never return a live count."""
        if not self.path.exists():return []
        root=Path(root).resolve();allowed=(root/'results').resolve();views=[];cache={}
        with closing(self._connect()) as db:
            for record in records:
                if record.get('classification_conflict') or record.get('namespace')!=namespace:continue
                association=db.execute('SELECT local_id FROM observations WHERE namespace=? '
                    'AND task_id=? AND memory_id=?',(namespace,record['task_id'],record['id'])).fetchone()
                task=db.execute('SELECT evidence,result FROM tasks WHERE namespace=? AND task_id=?',
                    (namespace,record['task_id'])).fetchone()
                if not association or not task:continue
                try:
                    result=json.loads(task['result']);local_id=association['local_id']
                    if local_id not in {o['id'] for o in result.get('objects',[])}:continue
                    path=(root/task['evidence']).resolve()
                    if not path.is_relative_to(allowed):continue
                    if path not in cache:cache[path]=json.loads((path.parent/'observations.json').read_text())
                    counts={}
                    for frame in cache[path]:
                        if local_id in frame.get('instance_ids',[]):
                            index=frame['viewpoint'];counts[index]=counts.get(index,0)+1
                    for view in result.get('viewpoint_diagnostics',[]):
                        pose=view['pose'];frames=counts.get(view['index'],0)
                        if (frames>=2 and view.get('reached') and len(pose)==3 and
                                all(type(v) in (int,float) and isfinite(v) for v in pose)):
                            views.append(dict(memory_id=record['id'],task_id=record['task_id'],
                                              pose=pose,frames=frames))
                except (OSError,ValueError,KeyError,TypeError):
                    continue
        return views

    def query(self, namespace, *, room=None, category=None, limit=200):
        if type(limit) is not int or not 1 <= limit <= 500:
            raise ValueError('查询数量上限为 1–500。')
        if not self.path.exists():
            return dict(objects=[], total=0, truncated=False, revision='0:')
        conditions, values = ['a.namespace=?'], [namespace]
        for key, value in (('room', room), ('category', category)):
            if value is not None:
                conditions.append('a.'+key+'=?')
                values.append(value)
        where = ' AND '.join(conditions)
        with closing(self._connect()) as db, db:
            db.execute('BEGIN')
            count, stamp = db.execute('SELECT COUNT(*), MAX(observed_at) FROM tasks WHERE namespace=?',
                                     (namespace,)).fetchone()
            total = db.execute('SELECT COUNT(*) FROM objects a WHERE '+where, values).fetchone()[0]
            portable = tuple(k for k,v in OBJECTS.items() if v.portable)
            placeholders = ','.join('?' for _ in portable)
            # Surface centroids of furniture and supported objects may overlap.
            # Flag only two portable-class hypotheses at nearly the same place.
            # This can mean misclassification OR replacement; never choose the
            # newer label automatically and never change the saved task count.
            rows = db.execute('SELECT a.*, (a.category IN ('+placeholders+') AND EXISTS ('
                'SELECT 1 FROM objects b WHERE b.namespace=a.namespace AND b.room=a.room '
                'AND b.category<>a.category AND b.category IN ('+placeholders+') '
                'AND (b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y)<=? AND ABS(b.z-a.z)<=?)) '
                'AS classification_conflict FROM objects a WHERE '+where+
                ' ORDER BY a.last_seen DESC,a.id LIMIT ?',
                (*portable,*portable,.08**2,.10,*values,limit)).fetchall()
        objects = [dict(row) for row in rows]
        for item in objects:
            item['category_admitted'] = OBJECTS[item['category']].admitted
            item['classification_conflict'] = bool(item['classification_conflict'])
        return dict(objects=objects, total=total, truncated=total > len(objects), revision=f'{count}:{stamp or ""}')
