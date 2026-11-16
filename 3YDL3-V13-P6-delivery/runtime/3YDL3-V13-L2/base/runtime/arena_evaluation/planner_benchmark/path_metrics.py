from __future__ import annotations

import gzip
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence

import numpy as np

from .map_utils import HospitalMap
from .models import PathMetric, Query


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def path_from_message(path_message: Any) -> List[Dict[str, float]]:
    points: List[Dict[str, float]] = []
    path_frame_id = str(getattr(getattr(path_message, "header", None), "frame_id", "map"))
    for pose_stamped in getattr(path_message, "poses", []):
        pose = pose_stamped.pose
        stamp = getattr(pose_stamped.header, "stamp", None)
        timestamp = 0.0
        if stamp is not None:
            timestamp = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9
        points.append({
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_to_yaw(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
            "timestamp": timestamp,
            "frame_id": str(getattr(pose_stamped.header, "frame_id", "") or path_frame_id),
        })
    return points


def interpolate_path(points: Sequence[Dict[str, float]], max_spacing: float) -> List[Dict[str, float]]:
    if not points:
        return []
    if max_spacing <= 0:
        raise ValueError("max_spacing must be positive")
    output = [dict(points[0])]
    for first, second in zip(points, points[1:]):
        dx = float(second["x"]) - float(first["x"])
        dy = float(second["y"]) - float(first["y"])
        distance = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(distance / max_spacing)))
        yaw_delta = _angle_delta(float(second.get("yaw", 0.0)), float(first.get("yaw", 0.0)))
        for step in range(1, steps + 1):
            fraction = step / steps
            output.append({
                "x": float(first["x"]) + dx * fraction,
                "y": float(first["y"]) + dy * fraction,
                "yaw": _wrap_angle(float(first.get("yaw", 0.0)) + yaw_delta * fraction),
                "timestamp": float(first.get("timestamp", 0.0)) + (float(second.get("timestamp", 0.0)) - float(first.get("timestamp", 0.0))) * fraction,
            })
    return output


def path_length(points: Sequence[Dict[str, float]]) -> float:
    return float(sum(
        math.hypot(float(second["x"]) - float(first["x"]), float(second["y"]) - float(first["y"]))
        for first, second in zip(points, points[1:])
    ))


def analyze_path(
    *,
    run_id: str,
    query: Query,
    planner_id: str,
    config_variant: str,
    points: Sequence[Dict[str, float]],
    hospital_map: HospitalMap,
    footprint: Sequence[Sequence[float]],
    preferred_minimum_turning_radius: float,
    allow_unknown: bool,
    max_spacing: Optional[float] = None,
) -> PathMetric:
    if not points:
        return PathMetric(run_id=run_id, query_id=query.query_id, planner_id=planner_id, config_variant=config_variant)
    sampled = interpolate_path(points, max_spacing or hospital_map.resolution / 2.0)
    lengths = [
        math.hypot(float(second["x"]) - float(first["x"]), float(second["y"]) - float(first["y"]))
        for first, second in zip(sampled, sampled[1:])
    ]
    total_length = float(sum(lengths))
    euclidean = math.hypot(query.goal[0] - query.start[0], query.goal[1] - query.start[1])
    clearances = [hospital_map.clearance(point["x"], point["y"]) for point in sampled]
    finite_clearances = np.asarray([value for value in clearances if value is not None], dtype=float)
    collisions = [
        hospital_map.footprint_collision(
            (point["x"], point["y"], point.get("yaw", 0.0)),
            footprint,
            unknown_is_collision=not allow_unknown,
        )
        for point in sampled
    ]
    collision_length = float(sum(length for length, collision in zip(lengths, collisions[1:]) if collision))
    yaw_changes = np.asarray([
        abs(_angle_delta(float(second.get("yaw", 0.0)), float(first.get("yaw", 0.0))))
        for first, second in zip(sampled, sampled[1:])
    ], dtype=float)
    curvatures = _curvature(sampled)
    moving_segments = [(first, second, length) for first, second, length in zip(sampled, sampled[1:], lengths) if length > 1e-9]
    reverse_lengths: List[float] = []
    directions: List[int] = []
    for first, second, length in moving_segments:
        dx = second["x"] - first["x"]
        dy = second["y"] - first["y"]
        heading_projection = dx * math.cos(first.get("yaw", 0.0)) + dy * math.sin(first.get("yaw", 0.0))
        direction = -1 if heading_projection < 0 else 1
        directions.append(direction)
        if direction < 0:
            reverse_lengths.append(length)
    switches = sum(1 for first, second in zip(directions, directions[1:]) if first != second)
    violations = int(sum(1 for value in curvatures if value > 1.0 / preferred_minimum_turning_radius))
    rotations = int(sum(
        1 for first, second in zip(sampled, sampled[1:])
        if math.hypot(second["x"] - first["x"], second["y"] - first["y"]) <= 1e-9
        and abs(_angle_delta(second.get("yaw", 0.0), first.get("yaw", 0.0))) > 1e-6
    ))
    final = points[-1]
    goal_position_error = math.hypot(final["x"] - query.goal[0], final["y"] - query.goal[1])
    goal_yaw_error = abs(_angle_delta(final.get("yaw", 0.0), query.goal[2]))
    reverse_distance = float(sum(reverse_lengths))
    return PathMetric(
        run_id=run_id,
        query_id=query.query_id,
        planner_id=planner_id,
        config_variant=config_variant,
        path_length_m=total_length,
        euclidean_distance_m=euclidean,
        length_over_euclidean=(total_length / euclidean if euclidean > 0 else None),
        minimum_clearance_m=(float(np.min(finite_clearances)) if finite_clearances.size else None),
        clearance_p05_m=(float(np.percentile(finite_clearances, 5)) if finite_clearances.size else None),
        clearance_p50_m=(float(np.percentile(finite_clearances, 50)) if finite_clearances.size else None),
        footprint_collision_count=int(sum(collisions)),
        footprint_collision_length_m=collision_length,
        heading_change_p95_rad=(float(np.percentile(yaw_changes, 95)) if yaw_changes.size else 0.0),
        heading_change_max_rad=(float(np.max(yaw_changes)) if yaw_changes.size else 0.0),
        curvature_p95_per_m=(float(np.percentile(curvatures, 95)) if curvatures.size else 0.0),
        curvature_max_per_m=(float(np.max(curvatures)) if curvatures.size else 0.0),
        preferred_radius_violation_count=violations,
        in_place_rotation_count=rotations,
        reverse_distance_m=reverse_distance,
        reverse_ratio=(reverse_distance / total_length if total_length > 0 else 0.0),
        direction_switch_count=switches,
        goal_position_error_m=goal_position_error,
        goal_yaw_error_rad=goal_yaw_error,
    )


def save_path(path: Path, points: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        json.dump(list(points), stream, separators=(",", ":"))


def load_path(path: Path) -> List[Dict[str, float]]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        return list(json.load(stream))


def _curvature(points: Sequence[Dict[str, float]]) -> np.ndarray:
    values: List[float] = []
    for first, middle, last in zip(points, points[1:], points[2:]):
        a = math.hypot(middle["x"] - first["x"], middle["y"] - first["y"])
        b = math.hypot(last["x"] - middle["x"], last["y"] - middle["y"])
        c = math.hypot(last["x"] - first["x"], last["y"] - first["y"])
        if min(a, b, c) <= 1e-9:
            values.append(0.0)
            continue
        area_twice = abs(
            (middle["x"] - first["x"]) * (last["y"] - first["y"])
            - (middle["y"] - first["y"]) * (last["x"] - first["x"])
        )
        values.append(2.0 * area_twice / (a * b * c))
    return np.asarray(values, dtype=float)


def _angle_delta(target: float, source: float) -> float:
    return _wrap_angle(target - source)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
