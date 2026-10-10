from __future__ import annotations
from dataclasses import dataclass
import hashlib
import math
import numpy as np
from .se2_semantic_guide import _dubins_words, wrap_angle
SPACING = 0.025

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
        return {'word': self.word, 'params': self.params, 'radius_m': self.radius, 'start': self.start, 'goal': self.goal, 'arc_length_m': self.length, 'sample_count': len(self.samples), 'lane_samples': self.n, 'correct_samples': self.correct, 'target_samples': self.target, 'sample_hash': hashlib.sha256(self.samples.tobytes()).hexdigest()}

def dubins_choices(start, goal, radius=0.401):
    (start, goal) = (tuple(map(float, start)), tuple(map(float, goal)))
    (dx, dy) = (goal[0] - start[0], goal[1] - start[1])
    direction = math.atan2(dy, dx)
    return sorted(_dubins_words((start[2] - direction) % (2 * math.pi), (goal[2] - direction) % (2 * math.pi), math.hypot(dx, dy) / radius), key=lambda item: (sum(item[1]), item[0]))

def dubins_edge_from_parameters(start, goal, radius, word, params):
    """Build an exact edge from one already-solved Dubins word.

    Enumeration callers can reject choices from analytic length before
    allocating samples.  The resulting accepted edge is byte-identical to
    :func:`dubins_edge`.
    """
    (start, goal) = (tuple(map(float, start)), tuple(map(float, goal)))
    word = str(word)
    params = tuple((float(value) for value in params))
    (pieces, pose) = ([], start)
    for (kind, parameter) in zip(word, params):
        length = parameter * radius
        if length < 1e-10:
            continue
        count = max(1, math.ceil(length / (SPACING * (1 - 1e-10))))
        t = np.arange(1, count + 1, dtype=np.float64) * (parameter / count)
        (x, y, yaw) = pose
        if kind == 'S':
            samples = np.column_stack((x + t * radius * math.cos(yaw), y + t * radius * math.sin(yaw), np.full(count, yaw)))
        else:
            sign = 1 if kind == 'L' else -1
            angle = yaw + sign * t
            samples = np.column_stack((x + sign * radius * (np.sin(angle) - math.sin(yaw)), y + sign * radius * (-np.cos(angle) + math.cos(yaw)), (angle + math.pi) % (2 * math.pi) - math.pi))
        pieces.append(samples)
        pose = tuple(samples[-1])
    if not pieces:
        return None
    samples = np.concatenate(pieces)
    error = np.array((samples[-1, 0] - goal[0], samples[-1, 1] - goal[1], wrap_angle(samples[-1, 2] - goal[2])))
    if np.max(np.abs(error)) > 1e-08:
        raise ValueError(f'analytic endpoint construction error: {error}')
    samples[-1] = goal
    return Edge(word, params, radius, start, goal, radius * sum(params), samples)
