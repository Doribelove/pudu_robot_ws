"""Strict input contract. Convex boxes describe feasible *center* positions."""
from dataclasses import dataclass, field
import math
import numpy as np


@dataclass
class Region:
    id: str
    bounds: np.ndarray  # xmin, ymin, xmax, ymax
    labels: frozenset = frozenset()
    halfspaces: list = field(default_factory=list)  # ax + by <= c, center predicates
    direction: object = None  # [heading_rad, half_angle_rad], <= pi/2

    def contains(self, point, tol=1e-8):
        p = np.asarray(point)[:2]
        return bool(np.all(p >= self.bounds[:2]-tol) and
                    np.all(p <= self.bounds[2:]+tol) and
                    all(np.dot(h[:2], p) <= h[2]+tol for h in self.halfspaces))


@dataclass
class Task:
    horizon_s: float
    visits: list
    avoid_labels: frozenset


@dataclass
class Scene:
    name: str
    start: np.ndarray
    goal: np.ndarray
    regions: list
    task: Task
    resolution_m: float = 0.05
    half_length_m: float = 0.265
    half_width_m: float = 0.225
    radius_min_m: float = 0.40
    speed_max_mps: float = 0.8
    metadata: dict = field(default_factory=dict)


def _keys(value, allowed, context):
    extra = set(value)-set(allowed)
    if extra:
        raise ValueError(f"UNSUPPORTED_FIELDS:{context}:{sorted(extra)}")


def load_scene(data):
    _keys(data, ['name','start','goal','regions','task','robot','metadata','resolution_m'], 'scene')
    robot = data.get('robot', {})
    _keys(robot, ['half_length_m','half_width_m','radius_min_m','speed_max_mps'], 'robot')
    task = data['task']
    _keys(task, ['horizon_s','always','ordered_eventually'], 'task')
    horizon = float(task['horizon_s'])
    if not math.isfinite(horizon) or horizon <= 0:
        raise ValueError('INVALID_HORIZON')
    avoid = set()
    for rule in task.get('always', []):
        _keys(rule, ['predicate','labels'], 'always')
        if rule['predicate'] == 'avoid_labels':
            avoid.update(rule['labels'])
        elif rule != {'predicate':'safe'}:
            raise ValueError('UNSUPPORTED_STL_PREDICATE')
    visits=[]
    for rule in task.get('ordered_eventually', []):
        _keys(rule, ['label','window_s'], 'eventually')
        lo,hi=map(float,rule['window_s'])
        if not all(map(math.isfinite,[lo,hi])) or not 0 <= lo <= hi <= horizon:
            raise ValueError('INVALID_STL_WINDOW')
        visits.append({'label':str(rule['label']), 'window_s':[lo,hi]})
    regions=[]
    for r in data['regions']:
        _keys(r, ['id','bounds','labels','halfspaces','direction'], 'region')
        b=np.asarray(r['bounds'],float)
        if b.shape != (4,) or not np.all(np.isfinite(b)) or np.any(b[:2]>=b[2:]):
            raise ValueError('INVALID_CONVEX_REGION')
        hs=np.asarray(r.get('halfspaces',[]),float).reshape(-1,3).tolist()
        if not np.all(np.isfinite(hs)):
            raise ValueError('NONFINITE_HALFSPACE')
        direction=r.get('direction')
        if direction is not None and (len(direction)!=2 or not all(map(math.isfinite,direction)) or not 0<float(direction[1])<=math.pi/2):
            raise ValueError('INVALID_DIRECTION_CONE')
        regions.append(Region(str(r['id']), b, frozenset(r.get('labels',[])),hs,direction))
    if len({r.id for r in regions})!=len(regions) or not regions:
        raise ValueError('DUPLICATE_OR_EMPTY_REGIONS')
    start,goal=np.asarray(data['start'],float),np.asarray(data['goal'],float)
    if start.shape!=(3,) or goal.shape!=(3,) or not np.all(np.isfinite([start,goal])):
        raise ValueError('INVALID_ENDPOINTS')
    scene=Scene(data.get('name','unnamed'),start,goal,regions,Task(horizon,visits,frozenset(avoid)),metadata=data.get('metadata',{}),**robot)
    if any(not math.isfinite(v) or v<=0 for v in [scene.half_length_m,scene.half_width_m,scene.radius_min_m,scene.speed_max_mps]):
        raise ValueError('INVALID_ROBOT')
    if float(data.get('resolution_m',0.05))!=0.05:
        raise ValueError('RESOLUTION_MUST_BE_0_05')
    return scene
