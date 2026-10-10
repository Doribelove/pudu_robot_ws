from __future__ import annotations
import math
from typing import Iterable, Tuple
Pose = Tuple[float, float, float]

def wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi

def _advance(pose: Pose, kind: str, distance_or_angle: float, radius: float) -> Pose:
    (x, y, yaw) = pose
    if kind == 'S':
        return (x + distance_or_angle * math.cos(yaw), y + distance_or_angle * math.sin(yaw), yaw)
    angle = float(distance_or_angle)
    if kind == 'L':
        return (x + radius * (math.sin(yaw + angle) - math.sin(yaw)), y + radius * (-math.cos(yaw + angle) + math.cos(yaw)), wrap_angle(yaw + angle))
    if kind == 'R':
        return (x + radius * (math.sin(yaw) - math.sin(yaw - angle)), y + radius * (math.cos(yaw - angle) - math.cos(yaw)), wrap_angle(yaw - angle))
    raise ValueError(f'unknown Dubins segment {kind}')

def _mod2pi(value: float) -> float:
    return float(value) % (2.0 * math.pi)

def _dubins_words(alpha: float, beta: float, distance: float) -> Iterable[Tuple[str, Tuple[float, float, float]]]:
    (sa, sb) = (math.sin(alpha), math.sin(beta))
    (ca, cb) = (math.cos(alpha), math.cos(beta))
    cab = math.cos(alpha - beta)
    p2 = 2.0 + distance * distance - 2.0 * cab + 2.0 * distance * (sa - sb)
    if p2 >= -1e-12:
        tmp = math.atan2(cb - ca, distance + sa - sb)
        yield ('LSL', (_mod2pi(-alpha + tmp), math.sqrt(max(0.0, p2)), _mod2pi(beta - tmp)))
    p2 = 2.0 + distance * distance - 2.0 * cab + 2.0 * distance * (-sa + sb)
    if p2 >= -1e-12:
        tmp = math.atan2(ca - cb, distance - sa + sb)
        yield ('RSR', (_mod2pi(alpha - tmp), math.sqrt(max(0.0, p2)), _mod2pi(-beta + tmp)))
    p2 = -2.0 + distance * distance + 2.0 * cab + 2.0 * distance * (sa + sb)
    if p2 >= -1e-12:
        p = math.sqrt(max(0.0, p2))
        tmp = math.atan2(-ca - cb, distance + sa + sb) - math.atan2(-2.0, p)
        yield ('LSR', (_mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp)))
    p2 = distance * distance - 2.0 + 2.0 * cab - 2.0 * distance * (sa + sb)
    if p2 >= -1e-12:
        p = math.sqrt(max(0.0, p2))
        tmp = math.atan2(ca + cb, distance - sa - sb) - math.atan2(2.0, p)
        yield ('RSL', (_mod2pi(alpha - tmp), p, _mod2pi(beta - tmp)))
    tmp = (6.0 - distance * distance + 2.0 * cab + 2.0 * distance * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0 + 1e-12:
        p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp))))
        t = _mod2pi(alpha - math.atan2(ca - cb, distance - sa + sb) + p / 2.0)
        yield ('RLR', (t, p, _mod2pi(alpha - beta - t + p)))
    tmp = (6.0 - distance * distance + 2.0 * cab + 2.0 * distance * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0 + 1e-12:
        p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp))))
        t = _mod2pi(-alpha - math.atan2(ca - cb, distance + sa - sb) + p / 2.0)
        yield ('LRL', (t, p, _mod2pi(beta - alpha - t + p)))
