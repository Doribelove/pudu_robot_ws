"""Static, dependency-light L3 checks and on-demand repair primitives.

This module intentionally does not start ROS or a planner server.  It validates
grid paths and can insert explicit in-place rotations.  A local Smac callback
may be supplied by a future static action adapter; if it is absent, an unsafe
rotation is reported as a structured repair failure rather than fabricated as
success.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from .planner_benchmark.map_utils import HospitalMap


Point = Dict[str, float]
Footprint = Sequence[Sequence[float]]


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_angles(angles: Sequence[float]) -> List[float]:
    if not angles:
        return []
    result = [float(angles[0])]
    for angle in angles[1:]:
        result.append(result[-1] + wrap_angle(float(angle) - result[-1]))
    return result


def angle_delta(a: float, b: float) -> float:
    return wrap_angle(float(b) - float(a))


def _distance(a: Point, b: Point) -> float:
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def _copy_point(point: Point, *, yaw: Optional[float] = None) -> Point:
    return {
        "x": float(point["x"]),
        "y": float(point["y"]),
        "yaw": float(point["yaw"] if yaw is None else yaw),
    }


@dataclass(frozen=True)
class KinematicConfig:
    allow_in_place_rotation: bool = True
    allow_reverse: bool = True
    reverse_penalty: float = 2.0
    preferred_minimum_turning_radius: float = 0.4
    heading_jump_trigger_deg: float = 30.0
    rotation_collision_sample_deg: float = 5.0
    stitch_position_tolerance_m: float = 0.05
    stitch_yaw_tolerance_deg: float = 10.0
    initial_repair_window_m: float = 1.0
    expanded_repair_window_m: float = 2.0
    motion_model: str = "REEDS_SHEPP"


@dataclass
class RepairWindow:
    indices: List[int]
    start_index: int
    end_index: int
    center_index: int
    reason: str


@dataclass
class KinematicDiagnostics:
    points: List[Point]
    unwrapped_yaws: List[float]
    trigger_indices: List[int]
    repair_windows: List[RepairWindow]
    position_continuous: bool
    max_position_gap_m: float
    static_collision_count: int
    rotation_collision_count: int
    rotation_sample_count: int
    heading_jump_count: int
    heading_jump_max_rad: float
    curvature_preference_violation_count: int
    turning_radius_preference_satisfied: bool
    direction_distance: Dict[str, float]
    direction_switch_count: int
    hard_kinematic_valid: bool

    def as_dict(self) -> Dict[str, object]:
        return {
            "heading_jump_count": self.heading_jump_count,
            "heading_jump_max_rad": self.heading_jump_max_rad,
            "trigger_count": len(self.trigger_indices),
            "repair_window_count": len(self.repair_windows),
            "trigger_indices": list(self.trigger_indices),
            "position_continuous": self.position_continuous,
            "max_position_gap_m": self.max_position_gap_m,
            "static_footprint_collision_count": self.static_collision_count,
            "rotation_sweep_collision_count": self.rotation_collision_count,
            "rotation_sample_count": self.rotation_sample_count,
            "curvature_preference_violation_count": self.curvature_preference_violation_count,
            "turning_radius_preference_satisfied": self.turning_radius_preference_satisfied,
            "reverse_distance_m": self.direction_distance["reverse"],
            "forward_distance_m": self.direction_distance["forward"],
            "reverse_ratio": self.direction_distance["reverse"] / max(
                1e-12, self.direction_distance["forward"] + self.direction_distance["reverse"]
            ),
            "direction_switch_count": self.direction_switch_count,
            "hard_kinematic_valid": self.hard_kinematic_valid,
        }


def repair_window_schedule(config: KinematicConfig) -> Tuple[float, float]:
    """The only allowed local-repair radii: initial window, then one expansion."""
    if config.initial_repair_window_m <= 0.0 or config.expanded_repair_window_m <= config.initial_repair_window_m:
        raise ValueError("expanded repair window must be greater than the initial window")
    return (float(config.initial_repair_window_m), float(config.expanded_repair_window_m))


def _motion_yaw(points: Sequence[Point], index: int) -> float:
    if len(points) < 2:
        return float(points[index]["yaw"])
    # Rotation samples intentionally repeat XY.  Endpoint tangent detection
    # must skip those zero-length samples and find the nearest actual motion
    # point instead of treating a yaw-only sample as a zero vector.
    prev = None
    for candidate in range(index - 1, -1, -1):
        if _distance(points[candidate], points[index]) > 1e-9:
            prev = points[candidate]
            break
    nxt = None
    for candidate in range(index + 1, len(points)):
        if _distance(points[candidate], points[index]) > 1e-9:
            nxt = points[candidate]
            break
    if prev is None and nxt is None:
        return float(points[index]["yaw"])
    if prev is None:
        return math.atan2(nxt["y"] - points[index]["y"], nxt["x"] - points[index]["x"])
    if nxt is None:
        return math.atan2(points[index]["y"] - prev["y"], points[index]["x"] - prev["x"])
    if _distance(prev, nxt) <= 1e-9:
        return float(points[index]["yaw"])
    return math.atan2(nxt["y"] - prev["y"], nxt["x"] - prev["x"])


def rotation_sweep_collision(
    hospital_map: HospitalMap,
    point: Point,
    yaw_start: float,
    yaw_goal: float,
    footprint: Footprint,
    sample_deg: float = 5.0,
) -> Tuple[bool, int]:
    """Check a shortest-angle in-place rotation at a fixed XY position."""
    delta = angle_delta(yaw_start, yaw_goal)
    steps = max(1, int(math.ceil(abs(math.degrees(delta)) / max(0.1, sample_deg))))
    collisions = 0
    for index in range(steps + 1):
        yaw = float(yaw_start) + delta * index / steps
        if hospital_map.footprint_collision(
            (float(point["x"]), float(point["y"]), yaw), footprint, unknown_is_collision=True
        ):
            collisions += 1
    return collisions > 0, steps + 1


def merge_trigger_indices(indices: Sequence[int], *, max_index_gap: int = 2) -> List[List[int]]:
    groups: List[List[int]] = []
    for index in sorted(set(int(value) for value in indices)):
        if not groups or index - groups[-1][-1] > max_index_gap:
            groups.append([index])
        else:
            groups[-1].append(index)
    return groups


def detect_trigger_indices(points: Sequence[Point], config: KinematicConfig) -> Tuple[List[int], List[float]]:
    yaws = unwrap_angles([float(point["yaw"]) for point in points])
    threshold = math.radians(config.heading_jump_trigger_deg)
    triggers: List[int] = []
    if len(points) < 2:
        return triggers, yaws
    # Endpoint yaw is part of the A2B contract and is checked against the first
    # and last motion tangent explicitly.
    start_tangent = _motion_yaw(points, 0)
    start_has_rotation_samples = len(points) > 1 and _distance(points[0], points[1]) <= 1e-9
    if not start_has_rotation_samples and abs(angle_delta(yaws[0], start_tangent)) > threshold:
        triggers.append(0)
    for index in range(len(points) - 1):
        if abs(angle_delta(yaws[index], yaws[index + 1])) > threshold:
            triggers.append(index if index == 0 else index + 1)
    goal_tangent = _motion_yaw(points, len(points) - 1)
    goal_has_rotation_samples = len(points) > 1 and _distance(points[-1], points[-2]) <= 1e-9
    if not goal_has_rotation_samples and abs(angle_delta(goal_tangent, yaws[-1])) > threshold:
        triggers.append(len(points) - 1)
    return sorted(set(triggers)), yaws


def _direction_stats(points: Sequence[Point], yaws: Sequence[float]) -> Tuple[Dict[str, float], int]:
    distances = {"forward": 0.0, "reverse": 0.0}
    directions: List[str] = []
    for index, (a, b) in enumerate(zip(points, points[1:])):
        length = _distance(a, b)
        if length <= 1e-9:
            continue
        tangent = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
        direction = "forward" if abs(angle_delta(yaws[index], tangent)) <= math.pi / 2.0 else "reverse"
        distances[direction] += length
        directions.append(direction)
    switches = sum(a != b for a, b in zip(directions, directions[1:]))
    return distances, switches


def _curvature_violations(points: Sequence[Point], yaws: Sequence[float], preferred_radius: float) -> int:
    violations = 0
    for index in range(1, len(points) - 1):
        before = _distance(points[index - 1], points[index])
        after = _distance(points[index], points[index + 1])
        if before <= 1e-9 or after <= 1e-9:
            continue
        delta = abs(angle_delta(yaws[index - 1], yaws[index + 1]))
        if delta <= 1e-9:
            continue
        radius = (before + after) / (2.0 * delta)
        if radius < preferred_radius:
            violations += 1
    return violations


def diagnose_path(
    points: Sequence[Point],
    hospital_map: HospitalMap,
    footprint: Footprint,
    config: KinematicConfig,
    *,
    merge_gap_points: int = 2,
) -> KinematicDiagnostics:
    normalized = [_copy_point(point) for point in points]
    triggers, yaws = detect_trigger_indices(normalized, config)
    windows = [
        RepairWindow(
            indices=group,
            start_index=max(0, group[0] - 1),
            end_index=min(len(normalized) - 1, group[-1] + 1),
            center_index=group[len(group) // 2],
            reason="heading_jump_or_endpoint_heading",
        )
        for group in merge_trigger_indices(triggers, max_index_gap=merge_gap_points)
    ]
    yaws = unwrap_angles([point["yaw"] for point in normalized])
    gaps = [_distance(a, b) for a, b in zip(normalized, normalized[1:])]
    max_gap = max(gaps, default=0.0)
    # A grid path is sampled at 0.05 m; allow diagonal cell spacing plus a
    # small numerical margin, while still detecting malformed path files.
    position_continuous = max_gap <= hospital_map.resolution * math.sqrt(2.0) + 1e-6
    static_collisions = sum(
        hospital_map.footprint_collision(
            (point["x"], point["y"], point["yaw"]), footprint, unknown_is_collision=True
        )
        for point in normalized
    )
    rotation_collisions = 0
    rotation_samples = 0
    for window in windows:
        index = window.center_index
        if index == 0:
            start_yaw, goal_yaw = yaws[0], yaws[min(1, len(yaws) - 1)]
        elif index == len(normalized) - 1:
            start_yaw, goal_yaw = yaws[index - 1], yaws[index]
        else:
            start_yaw, goal_yaw = yaws[index - 1], yaws[index + 1]
        collision, samples = rotation_sweep_collision(
            hospital_map, normalized[index], start_yaw, goal_yaw, footprint, config.rotation_collision_sample_deg
        )
        rotation_collisions += int(collision)
        rotation_samples += samples
    distances, switches = _direction_stats(normalized, yaws)
    preference_violations = _curvature_violations(normalized, yaws, config.preferred_minimum_turning_radius)
    heading_jumps = [abs(angle_delta(a, b)) for a, b in zip(yaws, yaws[1:])]
    max_jump = max(heading_jumps, default=0.0)
    hard_valid = bool(
        normalized
        and position_continuous
        and static_collisions == 0
        and rotation_collisions == 0
        and not triggers
    )
    return KinematicDiagnostics(
        points=normalized,
        unwrapped_yaws=yaws,
        trigger_indices=triggers,
        repair_windows=windows,
        position_continuous=position_continuous,
        max_position_gap_m=max_gap,
        static_collision_count=static_collisions,
        rotation_collision_count=rotation_collisions,
        rotation_sample_count=rotation_samples,
        heading_jump_count=sum(value > math.radians(config.heading_jump_trigger_deg) for value in heading_jumps),
        heading_jump_max_rad=max_jump,
        curvature_preference_violation_count=preference_violations,
        turning_radius_preference_satisfied=preference_violations == 0,
        direction_distance=distances,
        direction_switch_count=switches,
        hard_kinematic_valid=hard_valid,
    )


def insert_safe_rotations(
    diagnostics: KinematicDiagnostics,
    hospital_map: HospitalMap,
    footprint: Footprint,
    config: KinematicConfig,
) -> Tuple[List[Point], List[Dict[str, object]], Dict[str, object]]:
    """Insert explicit fixed-XY rotations for all safe repair windows."""
    points = diagnostics.points
    if not points:
        return [], [], {"success": False, "failure_code": "EMPTY_PATH"}
    output: List[Point] = []
    rotation_segments: List[Dict[str, object]] = []
    windows_by_index = {window.center_index: window for window in diagnostics.repair_windows}
    for index, point in enumerate(points):
        window = windows_by_index.get(index)
        if window is None:
            output.append(_copy_point(point, yaw=diagnostics.unwrapped_yaws[index]))
            continue
        if index == 0:
            start_yaw, goal_yaw = diagnostics.unwrapped_yaws[0], diagnostics.unwrapped_yaws[min(1, len(points) - 1)]
        elif index == len(points) - 1:
            start_yaw, goal_yaw = diagnostics.unwrapped_yaws[index - 1], diagnostics.unwrapped_yaws[index]
        else:
            start_yaw, goal_yaw = diagnostics.unwrapped_yaws[index - 1], diagnostics.unwrapped_yaws[index + 1]
        collision, sample_count = rotation_sweep_collision(
            hospital_map, point, start_yaw, goal_yaw, footprint, config.rotation_collision_sample_deg
        )
        if collision or not config.allow_in_place_rotation:
            return [], [], {
                "success": False,
                "failure_code": "KINEMATIC_REPAIR_FAILED",
                "hybrid_failure_reason": "ROTATE_IN_PLACE_COLLISION_OR_DISABLED",
                "repair_window_index": index,
            }
        delta = angle_delta(start_yaw, goal_yaw)
        steps = max(1, int(math.ceil(abs(math.degrees(delta)) / max(0.1, config.rotation_collision_sample_deg))))
        output.append(_copy_point(point, yaw=start_yaw))
        rotation_points = []
        for step in range(1, steps + 1):
            rotated = _copy_point(point, yaw=start_yaw + delta * step / steps)
            output.append(rotated)
            rotation_points.append(rotated)
        rotation_segments.append({
            "source": "kinematic",
            "direction": "rotate_in_place",
            "planner": "explicit_rotate_in_place",
            "rotation_angle_rad": delta,
            "rotation_sample_count": sample_count,
            "repair_reason": window.reason,
            "center_index": index,
            "points": rotation_points,
        })
    return output, rotation_segments, {"success": True, "failure_code": ""}


def build_segments(
    points: Sequence[Point],
    *,
    grid_mode: str,
    topology_edge_ids: Sequence[int],
    rotation_centers: Optional[Sequence[int]] = None,
) -> List[Dict[str, object]]:
    """Classify consecutive output points into grid or explicit rotation segments."""
    rotations = set(int(value) for value in (rotation_centers or []))
    segments: List[Dict[str, object]] = []
    for index, (a, b) in enumerate(zip(points, points[1:])):
        distance = _distance(a, b)
        if distance <= 1e-9 and abs(angle_delta(a["yaw"], b["yaw"])) > 1e-9:
            segments.append({
                "source": "kinematic", "direction": "rotate_in_place",
                "planner": "explicit_rotate_in_place", "repair_reason": "heading_jump",
                "rotation_angle_rad": angle_delta(a["yaw"], b["yaw"]),
                "length_m": 0.0, "grid_mode": grid_mode,
            })
            continue
        if distance <= 1e-9:
            continue
        tangent = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
        direction = "forward" if abs(angle_delta(a["yaw"], tangent)) <= math.pi / 2.0 else "reverse"
        segments.append({
            "source": "grid", "direction": direction, "planner": "grid_astar",
            "repair_reason": "", "rotation_angle_rad": 0.0,
            "length_m": distance, "grid_mode": grid_mode,
            "topology_edge_ids": list(topology_edge_ids),
        })
    return segments


def stitch_error(
    before: Point, after: Point, config: KinematicConfig
) -> Tuple[float, float, bool]:
    position_error = _distance(before, after)
    yaw_error = abs(math.degrees(angle_delta(before["yaw"], after["yaw"])))
    return position_error, yaw_error, bool(
        position_error <= config.stitch_position_tolerance_m + 1e-9
        and yaw_error <= config.stitch_yaw_tolerance_deg + 1e-9
    )


def repair_path(
    points: Sequence[Point],
    hospital_map: HospitalMap,
    footprint: Footprint,
    config: KinematicConfig,
    *,
    smac_repair: Optional[Callable[[RepairWindow, float], Optional[Sequence[Point]]]] = None,
) -> Dict[str, object]:
    diagnostics = diagnose_path(points, hospital_map, footprint, config)
    if not points:
        return {"diagnostics": diagnostics, "success": False, "failure_code": "EMPTY_PATH", "points": []}
    if not diagnostics.trigger_indices:
        return {
            "diagnostics": diagnostics, "success": diagnostics.static_collision_count == 0,
            "failure_code": "" if diagnostics.static_collision_count == 0 else "STATIC_FOOTPRINT_COLLISION",
            "points": diagnostics.points, "rotation_segments": [], "hybrid_calls": 0,
        }
    if diagnostics.rotation_collision_count == 0 and config.allow_in_place_rotation:
        repaired, rotations, result = insert_safe_rotations(diagnostics, hospital_map, footprint, config)
        if result.get("success"):
            return {
                "diagnostics": diagnostics, "success": True, "failure_code": "",
                "points": repaired, "rotation_segments": rotations, "hybrid_calls": 0,
                "repair_window_count": len(diagnostics.repair_windows),
                "repair_window_padding_m": config.initial_repair_window_m,
            }
    if smac_repair is None:
        return {
            "diagnostics": diagnostics, "success": False,
            "failure_code": "KINEMATIC_REPAIR_FAILED",
            "hybrid_failure_reason": "SMAC_LOCAL_BACKEND_NOT_STARTED_STATIC_ONLY",
            "points": [], "rotation_segments": [], "hybrid_calls": 0,
        }
    attempts = 0
    for window in diagnostics.repair_windows:
        for radius in repair_window_schedule(config):
            attempts += 1
            candidate = smac_repair(window, radius)
            if not candidate:
                continue
            candidate = [_copy_point(point) for point in candidate]
            candidate_diag = diagnose_path(candidate, hospital_map, footprint, config)
            if candidate_diag.static_collision_count == 0 and candidate_diag.position_continuous:
                return {
                    "diagnostics": diagnostics, "success": True, "failure_code": "",
                    "points": candidate, "rotation_segments": [], "hybrid_calls": attempts,
                    "hybrid_success": True, "repair_window_count": len(diagnostics.repair_windows),
                    "repair_window_padding_m": radius,
                }
    return {
        "diagnostics": diagnostics, "success": False,
        "failure_code": "KINEMATIC_REPAIR_FAILED",
        "hybrid_failure_reason": "LOCAL_SMAC_NO_VALID_STITCH",
        "points": [], "rotation_segments": [], "hybrid_calls": attempts,
    }
