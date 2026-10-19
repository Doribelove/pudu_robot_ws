"""Single-pass canonical path validation for 2D-V1-r2.

The result is intentionally a superset of the legacy validator and the formal
benchmark collision diagnostic.  Callers serialize this result directly; a
second process-local validation scan is neither required nor permitted.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np

from . import unified_four_backends_smoke as legacy


CANONICAL_VALIDATION_VERSION = "2d-v1-r2-canonical-v1"


def canonical_validate_path(
    ctx: Any,
    query: Any,
    points: Optional[Sequence[Mapping[str, Any]]],
) -> Dict[str, Any]:
    """Return all safety, kinematic, quality and collision diagnostics once."""
    started_ns = time.monotonic_ns()
    values: Dict[str, Any] = {
        "static_footprint_valid": False,
        "kinematic_valid": False,
        "final_valid_success": False,
        "path_length_m": None,
        "minimum_clearance_m": None,
        "curvature_p95": None,
        "maximum_curvature": None,
        "heading_discontinuity_count": 0,
        "reverse_distance_m": 0.0,
        "in_place_rotation_count": 0,
        "position_discontinuity_count": 0,
        "steering_jump_count": 0,
        "start_position_error_m": None,
        "start_yaw_error_rad": None,
        "goal_position_error_m": None,
        "goal_yaw_error_rad": None,
        "collision_count": 0,
        "collision_segment_indices": [],
        "collision_positions": [],
        "path_point_count": 0,
        "total_heading_change_rad": 0.0,
        "large_turn_count": 0,
        "euclidean_ratio": None,
        "failure_code": "EMPTY_PATH",
        "failure_detail": "path is empty",
        "canonical_validation_version": CANONICAL_VALIDATION_VERSION,
        "canonical_validation_reused": False,
    }
    if not points:
        values["canonical_validation_time_ms"] = (
            time.monotonic_ns() - started_ns
        ) / 1.0e6
        return values

    required = (
        "x", "y", "yaw", "source", "motion_direction", "steering",
        "planner_backend", "backend_version", "source_commit", "path_hash",
    )
    if any(any(field not in point for field in required) for point in points):
        values.update(
            failure_code="PATH_SCHEMA_INVALID",
            failure_detail="required path field missing",
            canonical_validation_time_ms=(time.monotonic_ns() - started_ns) / 1.0e6,
        )
        return values

    lengths: List[float] = []
    curvatures: List[float] = []
    heading_jumps = 0
    steering_jumps = 0
    rotations = 0
    reverse = 0.0
    discontinuities = 0
    total_heading_change = 0.0
    large_turn_count = 0
    collision_count = 0
    collision_segments: set[int] = set()
    collision_positions: List[List[float]] = []
    any_static_collision = False
    first_clearance = ctx.hospital_map.clearance(
        float(points[0]["x"]), float(points[0]["y"]),
    )
    minimum_clearance = float(first_clearance or 0.0)

    # Clearance belongs to the returned path points, exactly as in the legacy
    # validator. Collision checks additionally sample every segment with the
    # exact benchmark spacing and include both segment endpoints so the legacy
    # collision_count/positions diagnostics remain byte-for-byte comparable.
    if len(points) == 1:
        only = points[0]
        any_static_collision = bool(ctx.hospital_map.footprint_collision(
            (float(only["x"]), float(only["y"]), float(only["yaw"])),
            legacy.FOOTPRINT,
            unknown_is_collision=True,
        ))

    for segment_index, (first, second) in enumerate(zip(points, points[1:])):
        clearance = ctx.hospital_map.clearance(float(second["x"]), float(second["y"]))
        minimum_clearance = min(minimum_clearance, float(clearance or 0.0))
        dx = float(second["x"]) - float(first["x"])
        dy = float(second["y"]) - float(first["y"])
        distance = math.hypot(dx, dy)
        dyaw = legacy._delta(float(second["yaw"]), float(first["yaw"]))
        lengths.append(distance)
        total_heading_change += abs(dyaw)
        large_turn_count += int(abs(dyaw) > math.radians(45.0))
        heading_jumps += int(abs(dyaw) > math.radians(25.0))
        steering_jumps += int(
            abs(float(second["steering"]) - float(first["steering"]))
            > math.radians(15.0) + 1.0e-6
        )
        rotations += int(distance <= 1.0e-9 and abs(dyaw) > 1.0e-6)
        if (
            str(first["motion_direction"]) != "forward"
            or str(second["motion_direction"]) != "forward"
            or (
                distance > 1.0e-9
                and dx * math.cos(float(first["yaw"]))
                + dy * math.sin(float(first["yaw"])) < -1.0e-6
            )
        ):
            reverse += distance
        discontinuities += int(distance > legacy.MAX_PATH_SAMPLE_SPACING_M + 1.0e-9)
        if segment_index + 2 < len(points):
            curvatures.append(
                legacy._curvature(first, second, points[segment_index + 2])
            )

        steps = max(
            1,
            int(math.ceil(distance / legacy.COLLISION_SAMPLE_SPACING_M)),
            int(math.ceil(abs(dyaw) / legacy.COLLISION_YAW_SAMPLE_STEP_RAD)),
        )
        for step in range(steps + 1):
            fraction = step / steps
            pose = (
                float(first["x"]) + fraction * dx,
                float(first["y"]) + fraction * dy,
                legacy._wrap(float(first["yaw"]) + fraction * dyaw),
            )
            if ctx.hospital_map.footprint_collision(
                pose, legacy.FOOTPRINT, unknown_is_collision=True,
            ):
                any_static_collision = True
                collision_count += 1
                collision_segments.add(int(segment_index))
                if len(collision_positions) < 64:
                    collision_positions.append([
                        float(pose[0]), float(pose[1]), float(pose[2]),
                    ])

    start_error = math.hypot(
        float(points[0]["x"]) - query.start[0],
        float(points[0]["y"]) - query.start[1],
    )
    start_yaw_error = abs(legacy._delta(float(points[0]["yaw"]), query.start[2]))
    goal_error = math.hypot(
        float(points[-1]["x"]) - query.goal[0],
        float(points[-1]["y"]) - query.goal[1],
    )
    goal_yaw_error = abs(legacy._delta(float(points[-1]["yaw"]), query.goal[2]))
    max_curvature = max(curvatures, default=0.0)
    failures: List[str] = []
    kinematic_failures: List[str] = []
    if any_static_collision:
        failures.append("STATIC_FOOTPRINT_COLLISION")
    if reverse > 1.0e-6:
        kinematic_failures.append("REVERSE_MOTION")
    if rotations:
        kinematic_failures.append("IN_PLACE_ROTATION_FORBIDDEN")
    if max_curvature > 2.5 + legacy.KINEMATIC_NUMERICAL_TOLERANCE:
        kinematic_failures.append("MAXIMUM_CURVATURE_VIOLATION")
    if heading_jumps:
        kinematic_failures.append("HEADING_DISCONTINUITY")
    if steering_jumps:
        kinematic_failures.append("STEERING_DISCONTINUITY")
    if discontinuities:
        kinematic_failures.append("POSITION_DISCONTINUITY")
    if start_error > 0.25:
        kinematic_failures.append("START_POSITION_ERROR")
    if start_yaw_error > math.radians(10.0):
        kinematic_failures.append("START_YAW_ERROR")
    if goal_error > 0.25:
        kinematic_failures.append("ENDPOINT_POSITION_ERROR")
    if goal_yaw_error > math.radians(10.0):
        kinematic_failures.append("ENDPOINT_YAW_ERROR")
    failures.extend(kinematic_failures)
    values.update(
        static_footprint_valid=not any_static_collision,
        kinematic_valid=not kinematic_failures,
        final_valid_success=not any_static_collision and not kinematic_failures,
        path_length_m=sum(lengths),
        minimum_clearance_m=(minimum_clearance if math.isfinite(minimum_clearance) else 0.0),
        curvature_p95=float(np.percentile(curvatures, 95)) if curvatures else 0.0,
        maximum_curvature=max_curvature,
        heading_discontinuity_count=int(heading_jumps),
        reverse_distance_m=reverse,
        in_place_rotation_count=int(rotations),
        position_discontinuity_count=int(discontinuities),
        steering_jump_count=int(steering_jumps),
        start_position_error_m=start_error,
        start_yaw_error_rad=start_yaw_error,
        goal_position_error_m=goal_error,
        goal_yaw_error_rad=goal_yaw_error,
        collision_count=collision_count,
        collision_segment_indices=sorted(collision_segments),
        collision_positions=collision_positions,
        path_point_count=len(points),
        total_heading_change_rad=total_heading_change,
        large_turn_count=large_turn_count,
        euclidean_ratio=(
            sum(lengths) / math.hypot(
                float(query.goal[0]) - float(query.start[0]),
                float(query.goal[1]) - float(query.start[1]),
            )
            if math.hypot(
                float(query.goal[0]) - float(query.start[0]),
                float(query.goal[1]) - float(query.start[1]),
            ) > 1.0e-9 else None
        ),
        failure_code=failures[0] if failures else "",
        failure_detail=", ".join(failures),
        canonical_validation_time_ms=(time.monotonic_ns() - started_ns) / 1.0e6,
    )
    return values


__all__ = ["CANONICAL_VALIDATION_VERSION", "canonical_validate_path"]
