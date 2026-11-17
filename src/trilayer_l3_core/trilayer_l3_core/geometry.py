# Extracted frozen geometric kernels; see GEOMETRY_PROVENANCE.json.
from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable, Tuple
from pathlib import Path
import math, hashlib, ctypes
import numpy as np
from ament_index_python.packages import get_package_prefix
SPACING = 0.025
HALF_LENGTH = .265
HALF_WIDTH = .225
def wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _mod2pi(value: float) -> float:
    return float(value) % (2.0 * math.pi)


def _dubins_words(alpha: float, beta: float, distance: float) -> Iterable[Tuple[str, Tuple[float, float, float]]]:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)

    p2 = 2.0 + distance * distance - 2.0 * cab + 2.0 * distance * (sa - sb)
    if p2 >= -1.0e-12:
        tmp = math.atan2(cb - ca, distance + sa - sb)
        yield "LSL", (_mod2pi(-alpha + tmp), math.sqrt(max(0.0, p2)), _mod2pi(beta - tmp))

    p2 = 2.0 + distance * distance - 2.0 * cab + 2.0 * distance * (-sa + sb)
    if p2 >= -1.0e-12:
        tmp = math.atan2(ca - cb, distance - sa + sb)
        yield "RSR", (_mod2pi(alpha - tmp), math.sqrt(max(0.0, p2)), _mod2pi(-beta + tmp))

    p2 = -2.0 + distance * distance + 2.0 * cab + 2.0 * distance * (sa + sb)
    if p2 >= -1.0e-12:
        p = math.sqrt(max(0.0, p2))
        tmp = math.atan2(-ca - cb, distance + sa + sb) - math.atan2(-2.0, p)
        yield "LSR", (_mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp))

    p2 = distance * distance - 2.0 + 2.0 * cab - 2.0 * distance * (sa + sb)
    if p2 >= -1.0e-12:
        p = math.sqrt(max(0.0, p2))
        tmp = math.atan2(ca + cb, distance - sa - sb) - math.atan2(2.0, p)
        yield "RSL", (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp))

    tmp = (6.0 - distance * distance + 2.0 * cab + 2.0 * distance * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0 + 1.0e-12:
        p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp))))
        t = _mod2pi(alpha - math.atan2(ca - cb, distance - sa + sb) + p / 2.0)
        yield "RLR", (t, p, _mod2pi(alpha - beta - t + p))

    tmp = (6.0 - distance * distance + 2.0 * cab + 2.0 * distance * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0 + 1.0e-12:
        p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp))))
        t = _mod2pi(-alpha - math.atan2(ca - cb, distance + sa - sb) + p / 2.0)
        yield "LRL", (t, p, _mod2pi(beta - alpha - t + p))


@dataclass
class Edge:
    word: str
    params: tuple
    radius: float
    start: tuple
    goal: tuple
    length: float
    samples: np.ndarray
    n: int = 0
    correct: int = 0
    target: int = 0

    @property
    def side_resource(self):
        return 5 * self.correct - 4 * self.n

    @property
    def target_resource(self):
        return 2 * self.target - self.n

    def certificate(self):
        return {"word": self.word, "params": self.params, "radius_m": self.radius,
                "start": self.start, "goal": self.goal, "arc_length_m": self.length,
                "sample_count": len(self.samples), "lane_samples": self.n,
                "correct_samples": self.correct, "target_samples": self.target,
                "sample_hash": hashlib.sha256(self.samples.tobytes()).hexdigest()}


def dubins_choices(start, goal, radius=.401):
    start, goal = tuple(map(float, start)), tuple(map(float, goal))
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    direction = math.atan2(dy, dx)
    return sorted(_dubins_words((start[2]-direction) % (2*math.pi),
                               (goal[2]-direction) % (2*math.pi), math.hypot(dx, dy)/radius),
                  key=lambda item: (sum(item[1]), item[0]))


def dubins_edge_from_parameters(start, goal, radius, word, params):
    """Build an exact edge from one already-solved Dubins word.

    Enumeration callers can reject choices from analytic length before
    allocating samples.  The resulting accepted edge is byte-identical to
    :func:`dubins_edge`.
    """
    start, goal = tuple(map(float, start)), tuple(map(float, goal))
    word = str(word)
    params = tuple(float(value) for value in params)
    pieces, pose = [], start
    for kind, parameter in zip(word, params):
        length = parameter * radius
        if length < 1e-10:
            continue
        # Strictly less than the auditor spacing avoids ceil(1+roundoff) adding
        # an unplanned semantic sample on exact .025-m straight intervals.
        count = max(1, math.ceil(length / (SPACING * (1-1e-10))))
        t = np.arange(1, count+1, dtype=np.float64) * (parameter / count)
        x, y, yaw = pose
        if kind == "S":
            samples = np.column_stack((x+t*radius*math.cos(yaw), y+t*radius*math.sin(yaw), np.full(count, yaw)))
        else:
            sign = 1 if kind == "L" else -1
            angle = yaw + sign*t
            samples = np.column_stack((x+sign*radius*(np.sin(angle)-math.sin(yaw)),
                                       y+sign*radius*(-np.cos(angle)+math.cos(yaw)),
                                       (angle+math.pi) % (2*math.pi)-math.pi))
        pieces.append(samples)
        pose = tuple(samples[-1])
    if not pieces:
        return None
    samples = np.concatenate(pieces)
    error = np.array((samples[-1, 0]-goal[0], samples[-1, 1]-goal[1], wrap_angle(samples[-1, 2]-goal[2])))
    if np.max(np.abs(error)) > 1e-8:
        raise ValueError(f"analytic endpoint construction error: {error}")
    # Only floating-point reconstruction residue is replaced. All goal values
    # are the requested exact pose; this is not an endpoint attachment change.
    samples[-1] = goal
    return Edge(word, params, radius, start, goal, radius*sum(params), samples)


def wrap(a): return np.arctan2(np.sin(a),np.cos(a))


def densify(poses,ds=.025,dyaw=math.pi/180):
 p=np.asarray(poses,dtype=float);out=[p[0]]
 for a,b in zip(p,p[1:]):
  d=b-a;d[2]=wrap(d[2]);n=max(1,int(math.ceil(np.linalg.norm(d[:2])/ds)),int(math.ceil(abs(d[2])/dyaw)))
  out.extend(a+d*np.arange(1,n+1)[:,None]/n)
 return np.asarray(out)


class SweptChecker:
 def __init__(self,hard,resolution,origin,half_length=HALF_LENGTH,half_width=HALF_WIDTH):
  self.hard=np.ascontiguousarray(hard,dtype=np.uint8);self.res=float(resolution);self.origin=origin
  self.half_length=float(half_length);self.half_width=float(half_width)
  self.lib=ctypes.CDLL(str(Path(get_package_prefix('trilayer_l3_core'))/'lib/trilayer_l3_core/libsemantic_sweep.so'))
  self.fn=self.lib.swept_rect
  self.fn.argtypes=[np.ctypeslib.ndpointer(dtype=np.uint8,flags='C_CONTIGUOUS'),ctypes.c_int,ctypes.c_int,
    np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS'),ctypes.c_int,*([ctypes.c_double]*5)]
  self.fn.restype=ctypes.c_long
 def check(self,poses):
  p=np.ascontiguousarray(poses,dtype=np.float64)
  if p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.all(np.isfinite(p)): return False
  return self.fn(self.hard,*self.hard.shape,p,len(p),self.res,*self.origin[:2],self.half_length,self.half_width)==0


def collision_checker(hard,world):
    # Temporary request-local bool is not mutated while the checker uses it.
    a=np.asarray(hard)
    if a.dtype==np.dtype(bool) and a.flags.c_contiguous: a=a.view(np.uint8)
    return SweptChecker(a,world.map.resolution,world.map.origin,*world.vehicle[:2])

