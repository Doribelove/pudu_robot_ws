"""Single-pass path audit for the static layered-planner experiments.

The audit deliberately keeps the frozen validation thresholds and sampling
rules from :mod:`unified_four_backends_smoke`.  It only removes duplicate
work: interpolation, map-cell conversion, corridor containment, footprint
collision checks, kinematic checks, and provenance hashing are computed once
for one returned path and reused by the attempt and final-result layers.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, MutableMapping, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from . import unified_four_backends_smoke as legacy


@dataclass
class PathAuditResult:
    """Canonical, reusable validation result for one exact returned path."""

    path_hash: str = ""
    pose_hash: str = ""
    mask_hash: str = ""
    within_mask: bool = False
    metrics: Dict[str, Any] = field(default_factory=dict)
    timings: Dict[str, float] = field(default_factory=dict)
    sampled_pose_count: int = 0
    exact_footprint_check_count: int = 0

    @property
    def final_valid_success(self) -> bool:
        return bool(
            self.metrics.get("static_footprint_valid")
            and self.metrics.get("kinematic_valid")
            and not self.metrics.get("failure_code")
            and self.within_mask
        )

    def diagnostics(self) -> Dict[str, Any]:
        return {
            "canonical_path_hash": self.path_hash,
            "canonical_pose_hash": self.pose_hash,
            "canonical_mask_hash": self.mask_hash,
            "canonical_path_within_mask": self.within_mask,
            "canonical_sampled_pose_count": self.sampled_pose_count,
            "canonical_exact_footprint_check_count": self.exact_footprint_check_count,
            "canonical_path_audit_reused": True,
            **self.timings,
        }


def _mask_hash(mask: Optional[np.ndarray]) -> str:
    if mask is None:
        return ""
    return hashlib.sha256(
        np.ascontiguousarray(np.asarray(mask, dtype=np.uint8)).tobytes()
    ).hexdigest()


def _pose_hash(points: Sequence[Mapping[str, Any]]) -> str:
    payload = [
        {
            "x": float(point["x"]),
            "y": float(point["y"]),
            "yaw": float(point["yaw"]),
            "steering": float(point.get("steering", 0.0)),
            "motion_direction": str(point.get("motion_direction", "")),
        }
        for point in points
    ]
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    ).hexdigest()


def _empty_metrics(failure_code: str = "EMPTY_PATH") -> Dict[str, Any]:
    return {
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
        "failure_code": failure_code,
        "failure_detail": "path is empty" if failure_code == "EMPTY_PATH" else failure_code,
    }


class PathAuditor:
    """Perform the frozen hard validation once for a returned path."""

    def __init__(self, ctx: Any, *, source_commit: str, footprint: Sequence[Sequence[float]] = legacy.FOOTPRINT):
        self.ctx = ctx
        self.source_commit = str(source_commit or "unknown")
        self.footprint = [[float(value) for value in vertex] for vertex in footprint]
        # Build this once outside every online request.  Unlike the historical
        # center-clearance field, this treats unknown cells as obstacles too,
        # matching ``unknown_is_collision=True`` in the frozen validator.
        traversable = np.asarray(ctx.hospital_map.occupancy == 0, dtype=bool)
        self._distance_to_unsafe_m = ndimage.distance_transform_edt(
            traversable, sampling=float(ctx.hospital_map.resolution),
        )

    def audit(
        self,
        query: Any,
        points: Sequence[MutableMapping[str, Any]],
        allowed_mask: Optional[np.ndarray],
    ) -> PathAuditResult:
        audit_started_ns = time.monotonic_ns()
        timings = {
            "path_interpolation_ms": 0.0,
            "world_to_cell_ms": 0.0,
            "path_within_mask_ms": 0.0,
            "path_hash_provenance_ms": 0.0,
            "footprint_validation_ms": 0.0,
            "kinematic_validation_ms": 0.0,
        }
        if not points:
            timings["canonical_path_audit_ms"] = (time.monotonic_ns() - audit_started_ns) / 1.0e6
            return PathAuditResult(metrics=_empty_metrics(), timings=timings)

        hash_started_ns = time.monotonic_ns()
        for point in points:
            point.setdefault("source_commit", self.source_commit)
        pose_digest = _pose_hash(points)
        path_digest = legacy._path_hash(points)
        for point in points:
            point["path_hash"] = path_digest
        timings["path_hash_provenance_ms"] = (time.monotonic_ns() - hash_started_ns) / 1.0e6

        required = (
            "x", "y", "yaw", "source", "motion_direction", "steering",
            "planner_backend", "backend_version", "source_commit", "path_hash",
        )
        if any(any(field not in point for field in required) for point in points):
            metrics = _empty_metrics("PATH_SCHEMA_INVALID")
            metrics["failure_detail"] = "required path field missing"
            timings["canonical_path_audit_ms"] = (time.monotonic_ns() - audit_started_ns) / 1.0e6
            return PathAuditResult(
                path_hash=path_digest, pose_hash=pose_digest, mask_hash=_mask_hash(allowed_mask),
                metrics=metrics, timings=timings,
            )

        interpolation_started_ns = time.monotonic_ns()
        # Corridor containment historically sampled at resolution/2 (2.5 cm
        # on the frozen map), while footprint validation sampled at 5 cm plus
        # 5-degree yaw increments.  Use the denser of those rules once so the
        # canonical audit weakens neither acceptance check.
        sampled_values: List[Tuple[float, float, float]] = [
            (float(points[0]["x"]), float(points[0]["y"]), float(points[0]["yaw"])),
        ]
        translation_spacing = min(
            legacy.COLLISION_SAMPLE_SPACING_M,
            max(0.01, float(self.ctx.hospital_map.resolution) * 0.5),
        )
        for first, second in zip(points, points[1:]):
            dx = float(second["x"]) - float(first["x"])
            dy = float(second["y"]) - float(first["y"])
            dyaw = legacy._delta(float(second["yaw"]), float(first["yaw"]))
            steps = max(
                1,
                int(math.ceil(math.hypot(dx, dy) / translation_spacing)),
                int(math.ceil(abs(dyaw) / legacy.COLLISION_YAW_SAMPLE_STEP_RAD)),
            )
            for step in range(1, steps + 1):
                fraction = step / steps
                sampled_values.append((
                    float(first["x"]) + fraction * dx,
                    float(first["y"]) + fraction * dy,
                    legacy._wrap(float(first["yaw"]) + fraction * dyaw),
                ))
        sampled = np.asarray(sampled_values, dtype=float)
        timings["path_interpolation_ms"] = (time.monotonic_ns() - interpolation_started_ns) / 1.0e6

        cells_started_ns = time.monotonic_ns()
        hospital_map = self.ctx.hospital_map
        columns = np.floor((sampled[:, 0] - float(hospital_map.origin[0])) / float(hospital_map.resolution)).astype(np.int64)
        rows_from_bottom = np.floor((sampled[:, 1] - float(hospital_map.origin[1])) / float(hospital_map.resolution)).astype(np.int64)
        rows = int(hospital_map.height) - 1 - rows_from_bottom
        in_bounds = (
            (rows >= 0) & (rows < int(hospital_map.height))
            & (columns >= 0) & (columns < int(hospital_map.width))
        )
        timings["world_to_cell_ms"] = (time.monotonic_ns() - cells_started_ns) / 1.0e6

        mask_started_ns = time.monotonic_ns()
        if allowed_mask is None:
            within_mask = bool(np.all(in_bounds))
            mask_digest = ""
        else:
            mask = np.asarray(allowed_mask, dtype=bool)
            if mask.shape != (int(hospital_map.height), int(hospital_map.width)):
                raise ValueError("path audit mask shape does not match map")
            within = np.zeros((len(sampled),), dtype=bool)
            valid_indices = np.flatnonzero(in_bounds)
            within[valid_indices] = mask[rows[valid_indices], columns[valid_indices]]
            within_mask = bool(np.all(within))
            mask_digest = _mask_hash(mask)
        timings["path_within_mask_ms"] = (time.monotonic_ns() - mask_started_ns) / 1.0e6

        footprint_started_ns = time.monotonic_ns()
        collisions = int(np.count_nonzero(~in_bounds))
        exact_checks = 0
        if np.any(in_bounds):
            valid_indices = np.flatnonzero(in_bounds)
            # A center farther than the footprint bounding radius plus one
            # half-cell diagonal cannot intersect an occupied cell.  Only the
            # remaining near-obstacle poses need the frozen exact polygon test.
            footprint_radius = max(math.hypot(vertex[0], vertex[1]) for vertex in self.footprint)
            safe_threshold = footprint_radius + math.sqrt(2.0) * float(hospital_map.resolution) / 2.0
            clearances = np.asarray(
                self._distance_to_unsafe_m[rows[valid_indices], columns[valid_indices]],
                dtype=float,
            )
            near_indices = valid_indices[~np.isfinite(clearances) | (clearances <= safe_threshold + 1.0e-12)]
            for index in near_indices:
                exact_checks += 1
                collisions += int(hospital_map.footprint_collision(
                    sampled[int(index), :3], self.footprint, unknown_is_collision=True,
                ))
        timings["footprint_validation_ms"] = (time.monotonic_ns() - footprint_started_ns) / 1.0e6

        kinematic_started_ns = time.monotonic_ns()
        lengths = [
            math.hypot(float(second["x"]) - float(first["x"]), float(second["y"]) - float(first["y"]))
            for first, second in zip(points, points[1:])
        ]
        curvatures = [legacy._curvature(first, second, third) for first, second, third in zip(points, points[1:], points[2:])]
        heading_jumps = sum(
            abs(legacy._delta(float(second["yaw"]), float(first["yaw"]))) > math.radians(25.0)
            for first, second in zip(points, points[1:])
        )
        steering_jumps = sum(
            abs(float(second["steering"]) - float(first["steering"])) > math.radians(15.0) + 1.0e-6
            for first, second in zip(points, points[1:])
        )
        rotations = sum(
            distance <= 1.0e-9 and abs(legacy._delta(float(second["yaw"]), float(first["yaw"]))) > 1.0e-6
            for first, second, distance in zip(points, points[1:], lengths)
        )
        reverse = sum(
            distance
            for first, second, distance in zip(points, points[1:], lengths)
            if str(first["motion_direction"]) != "forward"
            or str(second["motion_direction"]) != "forward"
            or (
                distance > 1.0e-9
                and (float(second["x"]) - float(first["x"])) * math.cos(float(first["yaw"]))
                + (float(second["y"]) - float(first["y"])) * math.sin(float(first["yaw"])) < -1.0e-6
            )
        )
        discontinuities = sum(distance > legacy.MAX_PATH_SAMPLE_SPACING_M + 1.0e-9 for distance in lengths)
        start_error = math.hypot(float(points[0]["x"]) - query.start[0], float(points[0]["y"]) - query.start[1])
        start_yaw_error = abs(legacy._delta(float(points[0]["yaw"]), query.start[2]))
        goal_error = math.hypot(float(points[-1]["x"]) - query.goal[0], float(points[-1]["y"]) - query.goal[1])
        goal_yaw_error = abs(legacy._delta(float(points[-1]["yaw"]), query.goal[2]))
        max_curvature = max(curvatures, default=0.0)
        kinematic_failures: List[str] = []
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
        failures = (["STATIC_FOOTPRINT_COLLISION"] if collisions else []) + kinematic_failures
        timings["kinematic_validation_ms"] = (time.monotonic_ns() - kinematic_started_ns) / 1.0e6

        metrics = {
            "static_footprint_valid": collisions == 0,
            "kinematic_valid": not kinematic_failures,
            "final_valid_success": collisions == 0 and not kinematic_failures and within_mask,
            "path_length_m": sum(lengths),
            "minimum_clearance_m": min((hospital_map.clearance(float(point["x"]), float(point["y"])) or 0.0) for point in points),
            "curvature_p95": float(np.percentile(curvatures, 95)) if curvatures else 0.0,
            "maximum_curvature": max_curvature,
            "heading_discontinuity_count": int(heading_jumps),
            "reverse_distance_m": reverse,
            "in_place_rotation_count": int(rotations),
            "position_discontinuity_count": int(discontinuities),
            "steering_jump_count": int(steering_jumps),
            "start_position_error_m": start_error,
            "start_yaw_error_rad": start_yaw_error,
            "goal_position_error_m": goal_error,
            "goal_yaw_error_rad": goal_yaw_error,
            "failure_code": failures[0] if failures else ("L3_PRIME_PATH_OUTSIDE_CORRIDOR" if not within_mask else ""),
            "failure_detail": ", ".join(failures) if failures else ("path left allowed corridor" if not within_mask else ""),
        }
        timings["canonical_path_audit_ms"] = (time.monotonic_ns() - audit_started_ns) / 1.0e6
        return PathAuditResult(
            path_hash=path_digest,
            pose_hash=pose_digest,
            mask_hash=mask_digest,
            within_mask=within_mask,
            metrics=metrics,
            timings=timings,
            sampled_pose_count=len(sampled),
            exact_footprint_check_count=exact_checks,
        )


__all__ = ["PathAuditResult", "PathAuditor"]
