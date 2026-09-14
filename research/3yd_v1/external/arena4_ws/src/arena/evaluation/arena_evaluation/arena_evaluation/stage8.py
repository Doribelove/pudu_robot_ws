"""Stage 8 hard-radius L3 validation and static preference primitives.

The module is deliberately independent from the Stage 7 validator.  It treats
the project-level 0.40 m radius as a hard constraint, rejects in-place
rotations, and keeps action success separate from static and kinematic validity.
No simulator or dynamic obstacle input is used here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .kinematic import angle_delta, unwrap_angles, wrap_angle
from .planner_benchmark.map_utils import HospitalMap
from .planner_benchmark.path_metrics import interpolate_path

Point = Dict[str, float]
Footprint = Sequence[Sequence[float]]


@dataclass(frozen=True)
class HardRadiusConfig:
    allow_in_place_rotation: bool = False
    minimum_turning_radius: float = 0.40
    maximum_curvature: float = 2.50
    allow_reverse: bool = True
    reverse_penalty: float = 2.0
    motion_model: str = "REEDS_SHEPP"
    heading_jump_trigger_deg: float = 30.0
    curvature_sample_spacing_m: float = 0.05
    curvature_evaluation_window_m: float = 0.80
    collision_sample_spacing_m: float = 0.025
    numerical_tolerance: float = 1.0e-3
    stitch_position_tolerance_m: float = 0.05
    stitch_yaw_tolerance_deg: float = 10.0
    initial_repair_window_m: float = 1.0
    expanded_repair_window_m: float = 2.0
    rotation_collision_sample_deg: float = 5.0


@dataclass
class HardDiagnostics:
    points: List[Point]
    resampled_points: List[Point]
    failure_codes: List[str]
    zero_displacement_yaw_changes: int
    static_collision_count: int
    heading_jump_count: int
    heading_jump_max_rad: float
    hard_radius_violation_count: int
    minimum_radius_m: Optional[float]
    maximum_curvature: Optional[float]
    direction_distance: Dict[str, float]
    direction_switch_count: int
    position_continuous: bool
    endpoint_yaw_valid: bool
    hard_kinematic_valid: bool
    turning_radius_preference_satisfied: bool

    def as_dict(self) -> Dict[str, object]:
        total = self.direction_distance["forward"] + self.direction_distance["reverse"]
        return {
            "zero_displacement_yaw_changes": self.zero_displacement_yaw_changes,
            "static_footprint_collision_count": self.static_collision_count,
            "heading_jump_count": self.heading_jump_count,
            "heading_jump_max_rad": self.heading_jump_max_rad,
            "hard_radius_violation_count": self.hard_radius_violation_count,
            "minimum_radius_observed_m": self.minimum_radius_m,
            "maximum_curvature_observed": self.maximum_curvature,
            "forward_distance_m": self.direction_distance["forward"],
            "reverse_distance_m": self.direction_distance["reverse"],
            "reverse_ratio": self.direction_distance["reverse"] / total if total else 0.0,
            "direction_switch_count": self.direction_switch_count,
            "position_continuous": self.position_continuous,
            "endpoint_yaw_valid": self.endpoint_yaw_valid,
            "hard_kinematic_valid": self.hard_kinematic_valid,
            "turning_radius_preference_satisfied": self.turning_radius_preference_satisfied,
            "failure_codes": list(self.failure_codes),
        }


@dataclass(frozen=True)
class RepairWindow:
    start_index: int
    end_index: int
    center_index: int
    reason: str


def _copy(point: Point, yaw: Optional[float] = None) -> Point:
    return {"x": float(point["x"]), "y": float(point["y"]), "yaw": float(point["yaw"] if yaw is None else yaw)}


def distance(a: Point, b: Point) -> float:
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def resample_by_arc(points: Sequence[Point], spacing_m: float = 0.05) -> List[Point]:
    """Resample XY and unwrapped yaw at a fixed arc-length spacing."""
    if not points:
        return []
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be positive")
    source = [_copy(point) for point in points]
    yaws = unwrap_angles([point["yaw"] for point in source])
    cumulative = [0.0]
    for a, b in zip(source, source[1:]):
        cumulative.append(cumulative[-1] + distance(a, b))
    total = cumulative[-1]
    if total <= 1.0e-12:
        return [_copy(source[0], yaws[0])]
    targets = [0.0]
    while targets[-1] + spacing_m < total:
        targets.append(targets[-1] + spacing_m)
    if targets[-1] < total:
        targets.append(total)
    output: List[Point] = []
    segment = 0
    for target in targets:
        while segment + 1 < len(cumulative) and cumulative[segment + 1] < target - 1.0e-12:
            segment += 1
        if segment + 1 >= len(source):
            output.append(_copy(source[-1], yaws[-1])); continue
        span = cumulative[segment + 1] - cumulative[segment]
        alpha = 0.0 if span <= 1.0e-12 else (target - cumulative[segment]) / span
        a, b = source[segment], source[segment + 1]
        output.append({
            "x": a["x"] + alpha * (b["x"] - a["x"]),
            "y": a["y"] + alpha * (b["y"] - a["y"]),
            "yaw": yaws[segment] + alpha * (yaws[segment + 1] - yaws[segment]),
        })
    return output


def menger_curvature(a: Point, b: Point, c: Point, tolerance: float = 1.0e-9) -> float:
    ab = distance(a, b); bc = distance(b, c); ac = distance(a, c)
    denominator = ab * bc * ac
    if denominator <= tolerance:
        return 0.0
    area_twice = abs((b["x"] - a["x"]) * (c["y"] - a["y"]) - (b["y"] - a["y"]) * (c["x"] - a["x"]))
    return 2.0 * area_twice / denominator


def _motion_direction(a: Point, b: Point) -> str:
    tangent = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
    return "forward" if abs(angle_delta(a["yaw"], tangent)) <= math.pi / 2.0 else "reverse"


def _direction_stats(points: Sequence[Point]) -> Tuple[Dict[str, float], int]:
    values = {"forward": 0.0, "reverse": 0.0}
    directions: List[str] = []
    for a, b in zip(points, points[1:]):
        length = distance(a, b)
        if length <= 1.0e-9:
            continue
        direction = _motion_direction(a, b)
        values[direction] += length
        directions.append(direction)
    return values, sum(left != right for left, right in zip(directions, directions[1:]))


def _directional_yaw_error(yaw: float, tangent: float) -> float:
    error = abs(angle_delta(yaw, tangent))
    return min(error, abs(math.pi - error))


def _endpoint_yaw_valid(points: Sequence[Point], tolerance_deg: float) -> bool:
    if len(points) < 2:
        return False
    first = next((p for p in points[1:] if distance(points[0], p) > 1.0e-9), None)
    last = next((p for p in reversed(points[:-1]) if distance(points[-1], p) > 1.0e-9), None)
    if first is None or last is None:
        return False
    start_tangent = math.atan2(first["y"] - points[0]["y"], first["x"] - points[0]["x"])
    goal_tangent = math.atan2(points[-1]["y"] - last["y"], points[-1]["x"] - last["x"])
    return math.degrees(_directional_yaw_error(points[0]["yaw"], start_tangent)) <= tolerance_deg and math.degrees(_directional_yaw_error(points[-1]["yaw"], goal_tangent)) <= tolerance_deg


def sampled_curvatures(points: Sequence[Point], config: HardRadiusConfig) -> List[Tuple[int, float]]:
    """Menger curvature over a fixed metric window, excluding direction cusps."""
    if len(points) < 3:
        return []
    target_half = max(1, int(round(config.curvature_evaluation_window_m / (2.0 * config.curvature_sample_spacing_m))))
    half = min(target_half, max(1, (len(points) - 1) // 2))
    segment_directions = [_motion_direction(a, b) for a, b in zip(points, points[1:]) if distance(a, b) > 1.0e-9]
    if len(segment_directions) != len(points) - 1:
        return []
    values: List[Tuple[int, float]] = []
    for index in range(half, len(points) - half):
        directions = segment_directions[index - half:index + half]
        if not directions or any(item != directions[0] for item in directions[1:]):
            continue
        values.append((index, menger_curvature(points[index - half], points[index], points[index + half])))
    return values


def diagnose_hard_path(
    points: Sequence[Point], hospital_map: HospitalMap, footprint: Footprint, config: HardRadiusConfig,
) -> HardDiagnostics:
    normalized = [_copy(point) for point in points]
    failures: List[str] = []
    if not normalized:
        failures.append("EMPTY_PATH")
    zero_changes = 0
    for a, b in zip(normalized, normalized[1:]):
        if distance(a, b) <= config.numerical_tolerance and abs(angle_delta(a["yaw"], b["yaw"])) > config.numerical_tolerance:
            zero_changes += 1
    if zero_changes:
        failures.append("IN_PLACE_ROTATION_FORBIDDEN")
    resampled = resample_by_arc(normalized, config.curvature_sample_spacing_m)
    collision_points = interpolate_path(normalized, min(config.collision_sample_spacing_m, hospital_map.resolution / 2.0))
    collisions = sum(hospital_map.footprint_collision((p["x"], p["y"], p["yaw"]), footprint, unknown_is_collision=True) for p in collision_points)
    if collisions:
        failures.append("STATIC_FOOTPRINT_COLLISION")
    gaps = [distance(a, b) for a, b in zip(normalized, normalized[1:])]
    position_continuous = max(gaps, default=0.0) <= max(0.20, hospital_map.resolution * 4.0) + 1.0e-6
    if not position_continuous:
        failures.append("POSITION_DISCONTINUITY")
    unwrapped = unwrap_angles([p["yaw"] for p in resampled]) if resampled else []
    heading_jumps = [abs(unwrapped[i + 1] - unwrapped[i]) for i in range(len(unwrapped) - 1)]
    jump_threshold = math.radians(config.heading_jump_trigger_deg)
    jump_count = sum(value > jump_threshold for value in heading_jumps)
    if jump_count:
        failures.append("HEADING_DISCONTINUITY")
    curvatures = [value for _, value in sampled_curvatures(resampled, config)]
    max_curvature = max(curvatures, default=None)
    radii = [1.0 / value for value in curvatures if value > config.numerical_tolerance]
    min_radius = min(radii, default=None)
    radius_violations = sum(
        curvature > config.maximum_curvature + config.numerical_tolerance
        or (1.0 / curvature) < config.minimum_turning_radius - config.numerical_tolerance
        for curvature in curvatures if curvature > config.numerical_tolerance
    )
    if radius_violations:
        failures.append("MINIMUM_TURNING_RADIUS_VIOLATION")
    endpoint_valid = _endpoint_yaw_valid(normalized, config.stitch_yaw_tolerance_deg) if normalized else False
    if normalized and not endpoint_valid:
        failures.append("ENDPOINT_YAW_DISCONTINUITY")
    directions, switches = _direction_stats(resampled)
    hard_valid = bool(normalized and not failures)
    return HardDiagnostics(normalized, resampled, list(dict.fromkeys(failures)), zero_changes, collisions, jump_count, max(heading_jumps, default=0.0), radius_violations, min_radius, max_curvature, directions, switches, position_continuous, endpoint_valid, hard_valid, radius_violations == 0)


def merge_repair_windows(indices: Iterable[int], count: int, merge_gap: int = 2) -> List[RepairWindow]:
    values = sorted(set(int(item) for item in indices if 0 <= int(item) < count))
    groups: List[List[int]] = []
    for value in values:
        if not groups or value - groups[-1][-1] > merge_gap:
            groups.append([value])
        else:
            groups[-1].append(value)
    return [RepairWindow(max(0, group[0] - 1), min(count - 1, group[-1] + 1), group[len(group) // 2], "hard_radius_or_heading") for group in groups]


def trigger_indices(points: Sequence[Point], diagnostics: HardDiagnostics, config: HardRadiusConfig) -> List[int]:
    values: List[int] = []
    for index, (a, b) in enumerate(zip(points, points[1:])):
        if distance(a, b) <= config.numerical_tolerance and abs(angle_delta(a["yaw"], b["yaw"])) > config.numerical_tolerance:
            values.extend((index, index + 1))
    yaws = unwrap_angles([p["yaw"] for p in points])
    threshold = math.radians(config.heading_jump_trigger_deg)
    values.extend(index for index, (a, b) in enumerate(zip(yaws, yaws[1:])) if abs(b - a) > threshold for _ in (0,))
    # Curvature is evaluated on fixed-arc resampling, then mapped back to the
    # nearest original path point solely to locate repair windows.
    for index, curvature in sampled_curvatures(diagnostics.resampled_points, config):
        if curvature <= config.maximum_curvature + config.numerical_tolerance:
            continue
        sample = diagnostics.resampled_points[index]
        nearest = min(range(len(points)), key=lambda item: distance(sample, points[item]))
        values.append(nearest)
    if normalized_endpoint_trigger(points, config):
        values.extend((0, max(0, len(points) - 1)))
    return sorted(set(values))


def arc_window(points: Sequence[Point], center_index: int, radius_m: float, reason: str = "hard_radius_or_heading") -> RepairWindow:
    """Return anchors radius_m before and after a trigger along path arc."""
    if not points:
        raise ValueError("cannot make a repair window for an empty path")
    center = min(max(0, int(center_index)), len(points) - 1)
    start = center; travelled = 0.0
    while start > 0 and travelled < radius_m:
        travelled += distance(points[start - 1], points[start]); start -= 1
    end = center; travelled = 0.0
    while end + 1 < len(points) and travelled < radius_m:
        travelled += distance(points[end], points[end + 1]); end += 1
    return RepairWindow(start, end, center, reason)


def arc_repair_windows(points: Sequence[Point], indices: Iterable[int], radius_m: float) -> List[RepairWindow]:
    """Merge only overlapping arc windows, preserving separate local repairs."""
    windows = [arc_window(points, index, radius_m) for index in sorted(set(indices))]
    merged: List[RepairWindow] = []
    for window in windows:
        if not merged or window.start_index > merged[-1].end_index:
            merged.append(window)
        else:
            previous = merged[-1]
            merged[-1] = RepairWindow(previous.start_index, max(previous.end_index, window.end_index), previous.center_index, previous.reason)
    return merged


def normalized_endpoint_trigger(points: Sequence[Point], config: HardRadiusConfig) -> bool:
    return bool(points) and not _endpoint_yaw_valid(points, config.stitch_yaw_tolerance_deg)


def repair_window_schedule(config: HardRadiusConfig) -> Tuple[float, float]:
    if config.initial_repair_window_m <= 0.0 or config.expanded_repair_window_m <= config.initial_repair_window_m:
        raise ValueError("repair windows must be positive and strictly increasing")
    return config.initial_repair_window_m, config.expanded_repair_window_m


def stitch_errors(before: Point, after: Point, config: HardRadiusConfig) -> Tuple[float, float, bool]:
    position = distance(before, after)
    yaw = abs(math.degrees(angle_delta(before["yaw"], after["yaw"])))
    return position, yaw, position <= config.stitch_position_tolerance_m + 1.0e-9 and yaw <= config.stitch_yaw_tolerance_deg + 1.0e-9


def classify_segments(points: Sequence[Point], *, grid_mode: str, topology_edge_ids: Sequence[int], source: str = "grid", planner: str = "grid_astar", repair_reason: str = "") -> List[Dict[str, object]]:
    segments: List[Dict[str, object]] = []
    for a, b in zip(points, points[1:]):
        length = distance(a, b)
        if length <= 1.0e-9:
            continue
        segments.append({"source": source, "planner": planner, "direction": _motion_direction(a, b), "length_m": length, "grid_mode": grid_mode, "repair_reason": repair_reason, "topology_edge_ids": list(topology_edge_ids)})
    return segments
