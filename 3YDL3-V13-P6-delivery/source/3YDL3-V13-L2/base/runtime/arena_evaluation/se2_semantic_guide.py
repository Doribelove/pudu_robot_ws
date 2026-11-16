"""Explicit 48-bin SE(2) feasibility oracle for PLN-02 2A-V2 r4.

The pinned Humble Smac plugin has no per-query SE(2) guide input.  This module
therefore reproduces its forward-only DUBIN projection formulas in an isolated
oracle.  It never changes Nav2 and it never turns semantic preference into a
lethal mask.  The oracle is intentionally conservative: every primitive is
dense-sampled with the padded Jackal footprint against the exact expected
master costmap.
"""

from __future__ import annotations

import hashlib
import heapq
import math
import time
from collections import Counter
from dataclasses import dataclass, field, replace
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np

from .semantic_map import canonical_hash


Pose = Tuple[float, float, float]
StateKey = Tuple[int, int, int]


FAILURE_CODES = {
    "START_ATTACH_FAILED",
    "GOAL_ATTACH_FAILED",
    "YAW_DISCONTINUITY",
    "PRIMITIVE_COLLISION",
    "CURVATURE_INFEASIBLE",
    "LANE_INSTANCE_DISCONNECTED",
    "NO_SE2_ROUTE",
    "NO_SE2_METRIC_WITNESS",
    "SEARCH_TIMEOUT",
    "MAX_ITERATIONS",
}


@dataclass(frozen=True)
class SE2GuidePolicy:
    resolution_m: float = 0.05
    yaw_bins: int = 48
    minimum_turning_radius_m: float = 0.40
    maximum_curvature_1pm: float = 2.50
    primitive_sample_spacing_m: float = 0.025
    analytic_expansion_max_length_m: float = 3.0
    max_iterations: int = 1_000_000
    timeout_s: float = 120.0
    cost_penalty: float = 2.0
    non_straight_penalty: float = 1.2
    retrospective_penalty: float = 0.015
    reference_deviation_weight: float = 4.0
    reference_position_scale_m: float = 1.50
    reference_yaw_weight: float = 1.50
    heuristic_weight: float = 2.0
    wrong_side_weight: float = 3.0
    target_error_m: float = 0.50
    correct_side_ratio_min: float = 0.80
    target_band_ratio_min_exclusive: float = 0.50
    lane_target_right_boundary_m: float = 0.40
    nav2_footprint_padding_m: float = 0.01
    footprint: Tuple[Tuple[float, float], ...] = (
        (0.255, 0.215), (0.255, -0.215),
        (-0.255, -0.215), (-0.255, 0.215),
    )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "SE2GuidePolicy":
        names = {item.name for item in cls.__dataclass_fields__.values()}
        payload = {key: item for key, item in value.items() if key in names}
        if "footprint" in payload:
            payload["footprint"] = tuple(tuple(float(v) for v in p) for p in payload["footprint"])
        return cls(**payload)

    def validate(self) -> None:
        if self.yaw_bins != 48:
            raise ValueError("r4 requires exactly 48 yaw bins")
        if not math.isclose(self.resolution_m, 0.05, abs_tol=1.0e-12):
            raise ValueError("r4 requires exactly 0.05 m/cell")
        if not math.isclose(
            self.maximum_curvature_1pm,
            1.0 / self.minimum_turning_radius_m,
            rel_tol=0.0, abs_tol=1.0e-12,
        ):
            raise ValueError("turning radius and maximum curvature are inconsistent")
        if self.max_iterations > 1_000_000:
            raise ValueError("r4 may not exceed the frozen Smac iteration budget")

    @property
    def padded_footprint(self) -> Tuple[Tuple[float, float], ...]:
        padding = float(self.nav2_footprint_padding_m)
        return tuple(
            (
                x + math.copysign(padding, x) if x else x,
                y + math.copysign(padding, y) if y else y,
            )
            for x, y in self.footprint
        )

    @property
    def policy_hash(self) -> str:
        return canonical_hash({
            key: getattr(self, key)
            for key in self.__dataclass_fields__
        })


@dataclass(frozen=True)
class SmacDubinPrimitive:
    primitive_id: int
    name: str
    delta_yaw_bins: int
    delta_yaw_rad: float
    chord_length_m: float
    arc_length_m: float
    curvature_1pm: float


@dataclass
class _Record:
    pose: Pose
    g_cost: float
    motion_cost: float
    master_cost: float
    reference_cost: float
    wrong_side_cost: float
    parent: Optional[StateKey]
    primitive_id: Optional[int]
    primitive_samples: Tuple[Pose, ...]
    lane_samples: int
    correct_samples: int
    target_samples: int
    minimum_collision_margin_m: float


@dataclass(frozen=True)
class SE2OracleResult:
    query_id: str
    witness_exists: bool
    failure_code: str
    failure_detail: str
    path: Tuple[Pose, ...]
    primitive_trace: Tuple[Dict[str, Any], ...]
    diagnostics: Dict[str, Any]


def wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_bin(yaw: float, bins: int = 48) -> int:
    value = int(round(float(yaw) / (2.0 * math.pi / bins)))
    return value % bins


def bin_to_yaw(index: int, bins: int = 48) -> float:
    return wrap_angle((int(index) % bins) * (2.0 * math.pi / bins))


def build_smac_dubin_primitives(policy: SE2GuidePolicy) -> Tuple[SmacDubinPrimitive, ...]:
    """Reproduce pinned NodeHybrid::HybridMotionTable::initDubin."""
    policy.validate()
    radius_cells = policy.minimum_turning_radius_m / policy.resolution_m
    angle = 2.0 * math.asin(math.sqrt(2.0) / (2.0 * radius_cells))
    bin_size = 2.0 * math.pi / policy.yaw_bins
    increments = 1 if angle < bin_size else int(math.ceil(angle / bin_size))
    angle = increments * bin_size
    delta_x_cells = radius_cells * math.sin(angle)
    delta_y_cells = radius_cells - radius_cells * math.cos(angle)
    chord = math.hypot(delta_x_cells, delta_y_cells) * policy.resolution_m
    radius = policy.minimum_turning_radius_m
    return (
        SmacDubinPrimitive(0, "FORWARD", 0, 0.0, chord, chord, 0.0),
        SmacDubinPrimitive(1, "FORWARD_LEFT", increments, angle, chord, radius * angle, 1.0 / radius),
        SmacDubinPrimitive(2, "FORWARD_RIGHT", -increments, -angle, chord, radius * angle, -1.0 / radius),
    )


def _advance(pose: Pose, kind: str, distance_or_angle: float, radius: float) -> Pose:
    x, y, yaw = pose
    if kind == "S":
        return x + distance_or_angle * math.cos(yaw), y + distance_or_angle * math.sin(yaw), yaw
    angle = float(distance_or_angle)
    if kind == "L":
        return (
            x + radius * (math.sin(yaw + angle) - math.sin(yaw)),
            y + radius * (-math.cos(yaw + angle) + math.cos(yaw)),
            wrap_angle(yaw + angle),
        )
    if kind == "R":
        return (
            x + radius * (math.sin(yaw) - math.sin(yaw - angle)),
            y + radius * (math.cos(yaw - angle) - math.cos(yaw)),
            wrap_angle(yaw - angle),
        )
    raise ValueError(f"unknown Dubins segment {kind}")


def sample_primitive(
    pose: Pose, primitive: SmacDubinPrimitive, policy: SE2GuidePolicy,
) -> Tuple[Pose, ...]:
    count = max(1, int(math.ceil(primitive.arc_length_m / policy.primitive_sample_spacing_m)))
    samples: List[Pose] = []
    if primitive.name == "FORWARD":
        for index in range(1, count + 1):
            samples.append(_advance(
                pose, "S", primitive.arc_length_m * index / count,
                policy.minimum_turning_radius_m,
            ))
    else:
        kind = "L" if primitive.delta_yaw_rad > 0.0 else "R"
        total_angle = abs(primitive.delta_yaw_rad)
        for index in range(1, count + 1):
            samples.append(_advance(
                pose, kind, total_angle * index / count,
                policy.minimum_turning_radius_m,
            ))
    x, y, _ = samples[-1]
    samples[-1] = (
        x, y,
        bin_to_yaw(yaw_to_bin(pose[2], policy.yaw_bins) + primitive.delta_yaw_bins, policy.yaw_bins),
    )
    return tuple(samples)


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


def shortest_dubins_path(start: Pose, goal: Pose, radius: float) -> Optional[Tuple[str, Tuple[float, float, float]]]:
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    direct = math.atan2(dy, dx)
    distance = math.hypot(dx, dy) / radius
    alpha = _mod2pi(start[2] - direct)
    beta = _mod2pi(goal[2] - direct)
    choices = list(_dubins_words(alpha, beta, distance))
    if not choices:
        return None
    return min(choices, key=lambda item: (sum(item[1]), item[0]))


def sample_dubins_path(
    start: Pose, goal: Pose, radius: float, spacing_m: float,
) -> Optional[Tuple[str, Tuple[Pose, ...], float]]:
    solution = shortest_dubins_path(start, goal, radius)
    if solution is None:
        return None
    word, params = solution
    pose = start
    samples: List[Pose] = []
    total = 0.0
    for kind, parameter in zip(word, params):
        segment_length = parameter * radius
        count = max(1, int(math.ceil(segment_length / spacing_m)))
        initial = pose
        for index in range(1, count + 1):
            value = parameter * index / count
            candidate = _advance(
                initial, kind,
                value * radius if kind == "S" else value,
                radius,
            )
            samples.append(candidate)
        pose = samples[-1]
        total += segment_length
    if samples:
        samples[-1] = (float(goal[0]), float(goal[1]), wrap_angle(float(goal[2])))
    return word, tuple(samples), total


class EffectiveMasterCollisionChecker:
    """Complete padded-footprint sweep against expected effective master."""

    def __init__(self, hospital_map: Any, expected_master: np.ndarray, policy: SE2GuidePolicy):
        self.map = hospital_map
        self.master = np.asarray(expected_master, dtype=np.uint8)
        if self.master.shape != (hospital_map.height, hospital_map.width):
            raise ValueError("expected master shape mismatch")
        self.policy = policy
        self.obstacle = (self.master == 255) | (self.master >= 254)
        free = (~self.obstacle).astype(np.uint8)
        self.distance_m = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * float(
            hospital_map.resolution
        )
        occupancy = np.zeros(self.master.shape, dtype=np.int8)
        occupancy[self.obstacle] = 100
        self._collision_map = replace(
            hospital_map, occupancy=occupancy,
            distance_m=np.asarray(self.distance_m, dtype=np.float32),
        )
        self.footprint = policy.padded_footprint
        self.circumscribed_radius = max(math.hypot(x, y) for x, y in self.footprint)
        self.pose_checks = 0
        self.full_polygon_checks = 0

    def pose_status(self, pose: Pose) -> Tuple[bool, float]:
        self.pose_checks += 1
        cell = self.map.world_to_cell(pose[0], pose[1])
        if cell is None:
            return False, float("-inf")
        value = int(self.master[cell])
        center_clearance = float(self.distance_m[cell])
        margin = center_clearance - self.circumscribed_radius
        if value == 255 or value >= 253:
            return False, margin
        half_diagonal = math.sqrt(2.0) * float(self.map.resolution) / 2.0
        if center_clearance > self.circumscribed_radius + half_diagonal:
            return True, margin
        self.full_polygon_checks += 1
        collision = self._collision_map.footprint_collision(
            pose, self.footprint, unknown_is_collision=True,
        )
        return not collision, margin

    def sweep_status(self, poses: Sequence[Pose]) -> Tuple[bool, float, int]:
        minimum = float("inf")
        for index, pose in enumerate(poses):
            valid, margin = self.pose_status(pose)
            minimum = min(minimum, margin)
            if not valid:
                return False, minimum, index
        return True, minimum, -1


class ExplicitSE2GuideOracle:
    """Deterministic guide-aware A* over Smac-compatible DUBIN states."""

    def __init__(
        self, hospital_map: Any, expected_master: np.ndarray,
        lane_instance_labels: np.ndarray, lane_error_m: np.ndarray,
        lane_correct_side: np.ndarray, selected_lane_labels: Sequence[int],
        *, policy: Optional[SE2GuidePolicy] = None,
        binding: Optional[Mapping[str, Any]] = None,
        reference_polylines: Optional[Sequence[Sequence[Sequence[float]]]] = None,
    ) -> None:
        self.map = hospital_map
        self.master = np.asarray(expected_master, dtype=np.uint8)
        self.labels = np.asarray(lane_instance_labels, dtype=np.int32)
        self.error = np.asarray(lane_error_m, dtype=np.float32)
        self.correct = np.asarray(lane_correct_side, dtype=bool)
        self.selected = frozenset(int(value) for value in selected_lane_labels if int(value) > 0)
        self.policy = policy or SE2GuidePolicy()
        self.policy.validate()
        if not self.selected:
            raise ValueError("selected lane instance set is empty")
        if any(value.shape != self.master.shape for value in (self.labels, self.error, self.correct)):
            raise ValueError("SE2 guide grids must share one shape")
        self.primitives = build_smac_dubin_primitives(self.policy)
        self.checker = EffectiveMasterCollisionChecker(hospital_map, self.master, self.policy)
        reference_points: List[Tuple[float, float]] = []
        reference_yaws: List[float] = []
        for raw_segment in reference_polylines or ():
            segment = [tuple(float(value) for value in point[:2]) for point in raw_segment]
            if len(segment) < 2:
                continue
            segment_yaws = [
                math.atan2(right[1] - left[1], right[0] - left[0])
                for left, right in zip(segment, segment[1:])
            ]
            segment_yaws.append(segment_yaws[-1])
            reference_points.extend(segment)
            reference_yaws.extend(segment_yaws)
        self.reference_points = np.asarray(reference_points, dtype=np.float64).reshape((-1, 2))
        self.reference_yaws = np.asarray(reference_yaws, dtype=np.float64)
        self.reference_yaw_bins = np.asarray(
            [yaw_to_bin(value, self.policy.yaw_bins) for value in reference_yaws], dtype=np.int16,
        )
        self.reference_hash = canonical_hash({
            "points": self.reference_points.tolist(),
            "yaw_bins": self.reference_yaw_bins.tolist(),
        })
        self.binding = dict(binding or {})
        self.binding.update({
            "policy_hash": self.policy.policy_hash,
            "expected_master_hash": hashlib.sha256(
                np.ascontiguousarray(self.master).tobytes()
            ).hexdigest(),
            "lane_labels_hash": hashlib.sha256(
                np.ascontiguousarray(self.labels).tobytes()
            ).hexdigest(),
            "selected_lane_labels": sorted(self.selected),
            "se2_reference_hash": self.reference_hash,
        })
        self.binding_hash = canonical_hash(self.binding)

    def _key(self, pose: Pose) -> Optional[StateKey]:
        cell = self.map.world_to_cell(pose[0], pose[1])
        if cell is None:
            return None
        return int(cell[0]), int(cell[1]), yaw_to_bin(pose[2], self.policy.yaw_bins)

    def _snapped_pose(self, pose: Sequence[float]) -> Optional[Pose]:
        cell = self.map.world_to_cell(float(pose[0]), float(pose[1]))
        if cell is None:
            return None
        # NodeHybrid discretizes the heading but retains the continuous map
        # coordinates supplied by the request.  Moving the endpoint to a cell
        # centre would silently change the frozen query.
        return (
            float(pose[0]), float(pose[1]),
            bin_to_yaw(yaw_to_bin(float(pose[2]), self.policy.yaw_bins), self.policy.yaw_bins),
        )

    def _lane_sample(self, pose: Pose) -> Tuple[bool, bool, bool, float, int]:
        cell = self.map.world_to_cell(pose[0], pose[1])
        if cell is None:
            return False, False, False, float("inf"), 0
        label = int(self.labels[cell])
        in_lane = label in self.selected
        error = float(self.error[cell]) if np.isfinite(self.error[cell]) else float("inf")
        correct = bool(self.correct[cell]) if in_lane else False
        target = bool(in_lane and correct and error <= self.policy.target_error_m + 1.0e-12)
        return in_lane, correct, target, error, int(self.master[cell])

    def _reference_sample(self, pose: Pose) -> Tuple[float, float, int]:
        if not self.reference_points.size:
            return float("nan"), float("nan"), -1
        delta = self.reference_points - np.asarray(pose[:2], dtype=np.float64)
        index = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
        position_error = float(math.hypot(float(delta[index, 0]), float(delta[index, 1])))
        yaw_error = abs(wrap_angle(float(pose[2]) - float(self.reference_yaws[index])))
        return position_error, yaw_error, int(self.reference_yaw_bins[index])

    def _samples_status(self, samples: Sequence[Pose]) -> Tuple[bool, Dict[str, Any]]:
        collision_free, margin, failed_index = self.checker.sweep_status(samples)
        if not collision_free:
            return False, {
                "reason": "PRIMITIVE_COLLISION", "failed_sample_index": failed_index,
                "minimum_collision_margin_m": margin,
            }
        errors: List[float] = []
        reference_position_errors: List[float] = []
        reference_yaw_errors: List[float] = []
        reference_yaw_bins: List[int] = []
        correct = target = 0
        master_sum = 0.0
        for index, pose in enumerate(samples):
            in_lane, right, in_target, error, master = self._lane_sample(pose)
            if not in_lane:
                return False, {
                    "reason": "LANE_INSTANCE_DISCONNECTED", "failed_sample_index": index,
                    "minimum_collision_margin_m": margin,
                }
            errors.append(error)
            correct += int(right)
            target += int(in_target)
            master_sum += min(float(master), 252.0) / 252.0
            reference_position, reference_yaw, reference_bin = self._reference_sample(pose)
            if math.isfinite(reference_position):
                reference_position_errors.append(reference_position)
                reference_yaw_errors.append(reference_yaw)
                reference_yaw_bins.append(reference_bin)
        return True, {
            "reason": "", "minimum_collision_margin_m": margin,
            "lane_samples": len(samples), "correct_samples": correct,
            "target_samples": target, "errors": errors,
            "reference_position_errors": reference_position_errors,
            "reference_yaw_errors": reference_yaw_errors,
            "reference_yaw_bins": reference_yaw_bins,
            "mean_normalized_master_cost": master_sum / max(1, len(samples)),
        }

    def _reference_increment(self, metrics: Mapping[str, Any], length_m: float) -> Tuple[float, float, float]:
        errors = [value for value in metrics.get("errors", []) if math.isfinite(float(value))]
        if not errors:
            return float("inf"), float("inf"), float("inf")
        reference_positions = [
            float(value) for value in metrics.get("reference_position_errors", [])
            if math.isfinite(float(value))
        ]
        reference_yaws = [
            float(value) for value in metrics.get("reference_yaw_errors", [])
            if math.isfinite(float(value))
        ]
        deviation = (
            sum(min(value / self.policy.reference_position_scale_m, 1.0) for value in reference_positions)
            / len(reference_positions)
            if reference_positions
            else sum(min(float(value), 5.0) / 5.0 for value in errors) / len(errors)
        )
        yaw_deviation = (
            sum(min(value / math.pi, 1.0) for value in reference_yaws) / len(reference_yaws)
            if reference_yaws else 0.0
        )
        wrong_ratio = 1.0 - float(metrics["correct_samples"]) / max(1, int(metrics["lane_samples"]))
        master = float(metrics["mean_normalized_master_cost"])
        return (
            (
                self.policy.reference_deviation_weight * deviation
                + self.policy.reference_yaw_weight * yaw_deviation
            ) * length_m,
            self.policy.wrong_side_weight * wrong_ratio * length_m,
            self.policy.cost_penalty * master * length_m,
        )

    def _heuristic(self, pose: Pose, goal: Pose) -> float:
        distance = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
        yaw_distance = abs(wrap_angle(goal[2] - pose[2])) * self.policy.minimum_turning_radius_m
        return max(distance, yaw_distance)

    def _reconstruct(
        self, records: Mapping[StateKey, _Record], key: StateKey,
        connector: Optional[Tuple[str, Tuple[Pose, ...], float, Mapping[str, Any]]] = None,
    ) -> Tuple[Tuple[Pose, ...], Tuple[Dict[str, Any], ...]]:
        chain: List[Tuple[StateKey, _Record]] = []
        cursor: Optional[StateKey] = key
        while cursor is not None:
            record = records[cursor]
            chain.append((cursor, record))
            cursor = record.parent
        chain.reverse()
        path: List[Pose] = [chain[0][1].pose]
        trace: List[Dict[str, Any]] = []
        parent_pose = chain[0][1].pose
        for _, record in chain[1:]:
            primitive = self.primitives[int(record.primitive_id)]
            # Successor samples are deterministic from the parent pose and
            # primitive.  Recreate them only for the final path instead of
            # retaining 4-6 Python tuples in every explored state.
            primitive_samples = sample_primitive(parent_pose, primitive, self.policy)
            start_index = len(path) - 1
            path.extend(primitive_samples)
            trace.append({
                "trace_index": len(trace), "primitive_id": primitive.primitive_id,
                "primitive_type": primitive.name, "start_path_index": start_index,
                "end_path_index": len(path) - 1, "sample_count": len(primitive_samples),
                "curvature_1pm": primitive.curvature_1pm,
                "minimum_collision_margin_m": record.minimum_collision_margin_m,
            })
            parent_pose = record.pose
        if connector is not None:
            word, samples, length, metrics = connector
            start_index = len(path) - 1
            path.extend(samples)
            trace.append({
                "trace_index": len(trace), "primitive_id": -1,
                "primitive_type": f"ANALYTIC_{word}", "start_path_index": start_index,
                "end_path_index": len(path) - 1, "sample_count": len(samples),
                "length_m": length, "curvature_1pm": self.policy.maximum_curvature_1pm,
                "minimum_collision_margin_m": metrics["minimum_collision_margin_m"],
            })
        return tuple(path), tuple(trace)

    def _path_metrics(self, path: Sequence[Pose], trace: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
        lane_errors: List[float] = []
        correct_count = target_count = 0
        minimum_margin = float("inf")
        for pose in path:
            valid, margin = self.checker.pose_status(pose)
            if not valid:
                return {"path_collision_free": False, "minimum_collision_margin_m": margin}
            minimum_margin = min(minimum_margin, margin)
            in_lane, correct, target, error, _ = self._lane_sample(pose)
            if in_lane:
                lane_errors.append(error)
                correct_count += int(correct)
                target_count += int(target)
        lengths = [
            math.hypot(right[0] - left[0], right[1] - left[1])
            for left, right in zip(path, path[1:])
        ]
        count = len(lane_errors)
        maximum_curvature = max(
            (abs(float(item.get("curvature_1pm", 0.0))) for item in trace), default=0.0,
        )
        reverse_distance = 0.0
        in_place = 0
        for left, right, length in zip(path, path[1:], lengths):
            if length <= 1.0e-9:
                if abs(wrap_angle(right[2] - left[2])) > 1.0e-6:
                    in_place += 1
                continue
            projection = (
                (right[0] - left[0]) * math.cos(left[2])
                + (right[1] - left[1]) * math.sin(left[2])
            )
            if projection < -1.0e-6:
                reverse_distance += length
        error_p50 = float(np.median(np.asarray(lane_errors))) if lane_errors else None
        correct_ratio = float(correct_count / count) if count else None
        target_ratio = float(target_count / count) if count else None
        gate = bool(
            count and correct_ratio is not None and correct_ratio >= self.policy.correct_side_ratio_min
            and error_p50 is not None and error_p50 <= self.policy.target_error_m
            and target_ratio is not None and target_ratio > self.policy.target_band_ratio_min_exclusive
            and maximum_curvature <= self.policy.maximum_curvature_1pm + 1.0e-9
            and reverse_distance <= 1.0e-9 and in_place == 0
        )
        return {
            "path_collision_free": True,
            "path_length_m": float(sum(lengths)),
            "lane_sample_count": count,
            "lane_correct_side_ratio": correct_ratio,
            "lane_target_error_p50_m": error_p50,
            "lane_target_band_ratio": target_ratio,
            "maximum_curvature_1pm": maximum_curvature,
            "reverse_distance_m": reverse_distance,
            "in_place_rotation_count": in_place,
            "minimum_collision_margin_m": minimum_margin,
            "semantic_metric_gate_passed": gate,
        }

    def search(self, query_id: str, start: Sequence[float], goal: Sequence[float]) -> SE2OracleResult:
        started = time.monotonic()
        start_pose = self._snapped_pose(start)
        goal_pose = self._snapped_pose(goal)
        if start_pose is None:
            return SE2OracleResult(query_id, False, "START_ATTACH_FAILED", "start outside map", (), (), {})
        if goal_pose is None:
            return SE2OracleResult(query_id, False, "GOAL_ATTACH_FAILED", "goal outside map", (), (), {})
        start_lane = self._lane_sample(start_pose)[0]
        goal_lane = self._lane_sample(goal_pose)[0]
        start_free, start_margin = self.checker.pose_status(start_pose)
        goal_free, goal_margin = self.checker.pose_status(goal_pose)
        if not start_lane or not start_free:
            return SE2OracleResult(
                query_id, False, "START_ATTACH_FAILED",
                f"start lane={start_lane} footprint_free={start_free}", (), (),
                {"start_collision_margin_m": start_margin},
            )
        if not goal_lane or not goal_free:
            return SE2OracleResult(
                query_id, False, "GOAL_ATTACH_FAILED",
                f"goal lane={goal_lane} footprint_free={goal_free}", (), (),
                {"goal_collision_margin_m": goal_margin},
            )
        start_key = self._key(start_pose)
        goal_key = self._key(goal_pose)
        assert start_key is not None and goal_key is not None
        start_in_lane, start_correct, start_target, _, _ = self._lane_sample(start_pose)
        records: Dict[StateKey, _Record] = {
            start_key: _Record(
                pose=start_pose, g_cost=0.0, motion_cost=0.0, master_cost=0.0,
                reference_cost=0.0, wrong_side_cost=0.0, parent=None,
                primitive_id=None, primitive_samples=(), lane_samples=int(start_in_lane),
                correct_samples=int(start_correct), target_samples=int(start_target),
                minimum_collision_margin_m=start_margin,
            )
        }
        queue: List[Tuple[float, float, float, int, int, int, int]] = []
        counter = 0
        heapq.heappush(queue, (
            self._heuristic(start_pose, goal_pose), 0.0, 0.0,
            start_key[0], start_key[1], start_key[2], counter,
        ))
        closed: set[StateKey] = set()
        expanded_by_yaw: Counter[int] = Counter()
        rejected = Counter()
        analytic_attempts = 0
        regular_goal_metric_failures = 0
        best_rejected_metric: Optional[Dict[str, Any]] = None
        best_rejected_rank: Optional[Tuple[float, float, float, float]] = None
        first_neighbor_valid = False

        while queue:
            if len(closed) >= self.policy.max_iterations:
                failure = "MAX_ITERATIONS"
                detail = "frozen one-million-state budget exhausted"
                break
            if time.monotonic() - started >= self.policy.timeout_s:
                failure = "SEARCH_TIMEOUT"
                detail = f"oracle exceeded {self.policy.timeout_s:.3f}s"
                break
            _, queued_g, _, row, col, yaw_bin, serial = heapq.heappop(queue)
            del serial
            key = (row, col, yaw_bin)
            if key in closed:
                continue
            record = records.get(key)
            if record is None or queued_g > record.g_cost + 1.0e-12:
                continue
            closed.add(key)
            expanded_by_yaw[key[2]] += 1

            if math.hypot(goal_pose[0] - record.pose[0], goal_pose[1] - record.pose[1]) <= self.policy.analytic_expansion_max_length_m:
                analytic_attempts += 1
                connector_raw = sample_dubins_path(
                    record.pose, goal_pose, self.policy.minimum_turning_radius_m,
                    self.policy.primitive_sample_spacing_m,
                )
                if connector_raw is not None:
                    word, connector_samples, connector_length = connector_raw
                    valid, connector_metrics = self._samples_status(connector_samples)
                    if valid:
                        path, trace = self._reconstruct(
                            records, key,
                            (word, connector_samples, connector_length, connector_metrics),
                        )
                        metrics = self._path_metrics(path, trace)
                        if metrics.get("semantic_metric_gate_passed"):
                            return self._success(
                                query_id, path, trace, metrics, record, expanded_by_yaw,
                                rejected, analytic_attempts, started,
                                connector=(word, connector_samples, connector_length, connector_metrics),
                            )
                        correct_ratio = float(metrics.get("lane_correct_side_ratio") or 0.0)
                        target_ratio = float(metrics.get("lane_target_band_ratio") or 0.0)
                        error_p50 = float(metrics.get("lane_target_error_p50_m") or float("inf"))
                        rank = (
                            min(correct_ratio / self.policy.correct_side_ratio_min, 1.0),
                            min(target_ratio / self.policy.target_band_ratio_min_exclusive, 1.0),
                            min(self.policy.target_error_m / max(error_p50, 1.0e-12), 1.0),
                            -float(metrics.get("path_length_m") or float("inf")),
                        )
                        if best_rejected_rank is None or rank > best_rejected_rank:
                            best_rejected_rank = rank
                            best_rejected_metric = {
                                key: metrics.get(key) for key in (
                                    "lane_correct_side_ratio", "lane_target_error_p50_m",
                                    "lane_target_band_ratio", "path_length_m",
                                    "maximum_curvature_1pm", "minimum_collision_margin_m",
                                    "reverse_distance_m", "in_place_rotation_count",
                                )
                            }
                            best_rejected_metric.update({
                                "analytic_word": word,
                                "expanded_state_count_at_candidate": len(closed),
                            })
                        rejected["ANALYTIC_METRIC_GATE"] += 1
                        if key == goal_key:
                            regular_goal_metric_failures += 1
                    else:
                        rejected[str(connector_metrics["reason"])] += 1

            for primitive in self.primitives:
                samples = sample_primitive(record.pose, primitive, self.policy)
                child_pose = samples[-1]
                child_key = self._key(child_pose)
                if child_key is None or child_key in closed:
                    rejected["OUTSIDE_OR_CLOSED"] += 1
                    continue
                valid, sample_metrics = self._samples_status(samples)
                if not valid:
                    rejected[str(sample_metrics["reason"])] += 1
                    continue
                first_neighbor_valid = True
                reference_cost, wrong_cost, master_cost = self._reference_increment(
                    sample_metrics, primitive.arc_length_m,
                )
                motion_cost = primitive.chord_length_m * (1.0 - self.policy.retrospective_penalty)
                if primitive.primitive_id != 0:
                    motion_cost *= self.policy.non_straight_penalty
                increment = motion_cost + master_cost + reference_cost + wrong_cost
                new_g = record.g_cost + increment
                prior = records.get(child_key)
                if prior is not None and new_g >= prior.g_cost - 1.0e-12:
                    rejected["NOT_BETTER"] += 1
                    continue
                child = _Record(
                    pose=child_pose, g_cost=new_g,
                    motion_cost=record.motion_cost + motion_cost,
                    master_cost=record.master_cost + master_cost,
                    reference_cost=record.reference_cost + reference_cost,
                    wrong_side_cost=record.wrong_side_cost + wrong_cost,
                    parent=key, primitive_id=primitive.primitive_id,
                    primitive_samples=(),
                    lane_samples=record.lane_samples + int(sample_metrics["lane_samples"]),
                    correct_samples=record.correct_samples + int(sample_metrics["correct_samples"]),
                    target_samples=record.target_samples + int(sample_metrics["target_samples"]),
                    minimum_collision_margin_m=min(
                        record.minimum_collision_margin_m,
                        float(sample_metrics["minimum_collision_margin_m"]),
                    ),
                )
                records[child_key] = child
                counter += 1
                priority = new_g + self.policy.heuristic_weight * self._heuristic(child_pose, goal_pose)
                heapq.heappush(queue, (
                    priority, new_g, child.reference_cost + child.wrong_side_cost,
                    child_key[0], child_key[1], child_key[2], counter,
                ))
        else:
            failure = "NO_SE2_ROUTE" if regular_goal_metric_failures == 0 else "NO_SE2_METRIC_WITNESS"
            detail = "open set exhausted"

        if not first_neighbor_valid and rejected.get("PRIMITIVE_COLLISION", 0):
            failure = "PRIMITIVE_COLLISION"
            detail = "every start primitive collided or left the lane instance"
        diagnostics = self._diagnostics(
            expanded_by_yaw, rejected, analytic_attempts, started,
            start_pose=start_pose, goal_pose=goal_pose,
        )
        diagnostics.update({
            "regular_goal_metric_failures": regular_goal_metric_failures,
            "start_collision_margin_m": start_margin,
            "goal_collision_margin_m": goal_margin,
            "best_rejected_metric_candidate": best_rejected_metric,
        })
        return SE2OracleResult(query_id, False, failure, detail, (), (), diagnostics)

    def _diagnostics(
        self, expanded_by_yaw: Mapping[int, int], rejected: Mapping[str, int],
        analytic_attempts: int, started: float, **extra: Any,
    ) -> Dict[str, Any]:
        return {
            "schema_version": "PLN-02-2A-V2-R4-SE2-ORACLE-RESULT-V1",
            "binding_hash": self.binding_hash,
            "binding": self.binding,
            "yaw_bins": self.policy.yaw_bins,
            "motion_model": "DUBIN",
            "primitive_increment_bins": abs(self.primitives[1].delta_yaw_bins),
            "primitive_chord_length_m": self.primitives[0].chord_length_m,
            "primitive_arc_length_m": self.primitives[1].arc_length_m,
            "explicit_se2_reference_consumed": bool(self.reference_points.size),
            "se2_reference_hash": self.reference_hash,
            "guide_yaw_bin_coverage": sorted(set(int(value) for value in self.reference_yaw_bins)),
            "start_yaw_bin": yaw_to_bin(extra.get("start_pose", (0.0, 0.0, 0.0))[2], self.policy.yaw_bins),
            "goal_yaw_bin": yaw_to_bin(extra.get("goal_pose", (0.0, 0.0, 0.0))[2], self.policy.yaw_bins),
            "expanded_state_count": int(sum(expanded_by_yaw.values())),
            "expanded_states_by_yaw_bin": {
                str(index): int(expanded_by_yaw.get(index, 0)) for index in range(self.policy.yaw_bins)
            },
            "rejected_successors": dict(sorted(rejected.items())),
            "analytic_attempt_count": int(analytic_attempts),
            "collision_pose_check_count": int(self.checker.pose_checks),
            "collision_full_polygon_check_count": int(self.checker.full_polygon_checks),
            "wall_ms": (time.monotonic() - started) * 1000.0,
            "deterministic_tie_break": "f,g,reference,row,col,yaw_bin,insertion_serial",
            **extra,
        }

    def _success(
        self, query_id: str, path: Tuple[Pose, ...], trace: Tuple[Dict[str, Any], ...],
        metrics: Mapping[str, Any], record: _Record, expanded_by_yaw: Mapping[int, int],
        rejected: Mapping[str, int], analytic_attempts: int, started: float,
        connector: Optional[Tuple[str, Tuple[Pose, ...], float, Mapping[str, Any]]] = None,
    ) -> SE2OracleResult:
        diagnostics = self._diagnostics(
            expanded_by_yaw, rejected, analytic_attempts, started,
            start_pose=path[0], goal_pose=path[-1],
        )
        diagnostics.update(dict(metrics))
        connector_motion = connector_master = connector_reference = connector_wrong = 0.0
        if connector is not None:
            _word, _samples, connector_motion, connector_metrics = connector
            connector_reference, connector_wrong, connector_master = self._reference_increment(
                connector_metrics, connector_motion,
            )
        diagnostics["cost_breakdown"] = {
            "g_total": record.g_cost + connector_motion + connector_master
            + connector_reference + connector_wrong,
            "motion_cost": record.motion_cost + connector_motion,
            "master_cost": record.master_cost + connector_master,
            "reference_deviation_cost": record.reference_cost + connector_reference,
            "wrong_side_cost": record.wrong_side_cost + connector_wrong,
        }
        diagnostics["path_hash"] = canonical_hash([
            [round(pose[0], 9), round(pose[1], 9), round(pose[2], 9)] for pose in path
        ])
        diagnostics["primitive_trace_hash"] = canonical_hash(trace)
        return SE2OracleResult(query_id, True, "", "", path, trace, diagnostics)


__all__ = [
    "FAILURE_CODES", "EffectiveMasterCollisionChecker", "ExplicitSE2GuideOracle",
    "SE2GuidePolicy", "SE2OracleResult", "SmacDubinPrimitive", "bin_to_yaw",
    "build_smac_dubin_primitives", "sample_dubins_path", "sample_primitive",
    "shortest_dubins_path", "wrap_angle", "yaw_to_bin",
]
