"""Independent semantic audit layered on the frozen canonical PathAudit."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .semantic_map import SemanticMapV1, point_in_polygon
from .semantic_rasterizer import RasterizedSemantics


def _percentile(values: Sequence[float], percentile: float) -> Optional[float]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return float(np.percentile(finite, percentile)) if finite else None


def _xy(point: Any) -> Tuple[float, float]:
    if isinstance(point, Mapping):
        return float(point["x"]), float(point["y"])
    return float(point[0]), float(point[1])


def _yaw(point: Any, fallback: float = 0.0) -> float:
    if isinstance(point, Mapping):
        return float(point.get("yaw", fallback))
    return float(point[2]) if len(point) > 2 else fallback


@dataclass
class SemanticPathAuditResult:
    metrics: Dict[str, Any]
    hard_constraints_held: bool
    failure_code: str
    sampled_pose_count: int
    diagnostics: Dict[str, Any] = field(default_factory=dict)

    @property
    def final_valid_success(self) -> bool:
        return self.hard_constraints_held and not self.failure_code

    def to_dict(self) -> Dict[str, Any]:
        return {
            **self.metrics,
            "hard_constraints_held": self.hard_constraints_held,
            "failure_code": self.failure_code,
            "final_valid_success": self.final_valid_success,
            "semantic_sampled_pose_count": self.sampled_pose_count,
            "semantic_audit_diagnostics": self.diagnostics,
        }


class SemanticPathAuditor:
    def __init__(
        self, hospital_map: Any, semantic_map: SemanticMapV1,
        raster: RasterizedSemantics,
    ) -> None:
        semantic_map.validate_against_map(hospital_map)
        self.hospital_map = hospital_map
        self.semantic_map = semantic_map
        self.raster = raster

    def _samples(self, points: Sequence[Any]) -> List[Tuple[float, float, float]]:
        if not points:
            return []
        result = [(*_xy(points[0]), _yaw(points[0]))]
        spacing = max(0.01, float(self.hospital_map.resolution) * 0.5)
        for first, second in zip(points, points[1:]):
            x0, y0 = _xy(first)
            x1, y1 = _xy(second)
            distance = math.hypot(x1 - x0, y1 - y0)
            steps = max(1, int(math.ceil(distance / spacing)))
            yaw0, yaw1 = _yaw(first), _yaw(second)
            delta_yaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
            for index in range(1, steps + 1):
                fraction = index / steps
                result.append((
                    x0 + fraction * (x1 - x0), y0 + fraction * (y1 - y0),
                    yaw0 + fraction * delta_yaw,
                ))
        return result

    @staticmethod
    def _explicit_direction(value: Any) -> Optional[Tuple[float, float]]:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            direction = (float(value[0]), float(value[1]))
        elif isinstance(value, Mapping) and "x" in value and "y" in value:
            direction = (float(value["x"]), float(value["y"]))
        elif isinstance(value, (int, float)):
            direction = (math.cos(float(value)), math.sin(float(value)))
        else:
            return None
        norm = math.hypot(*direction)
        return (direction[0] / norm, direction[1] / norm) if norm > 1e-12 else None

    def audit(
        self, points: Sequence[Any], preference_field: Any, *, relaxation_level: str,
        canonical_metrics: Optional[Mapping[str, Any]] = None,
        baseline_path_length_m: Optional[float] = None,
    ) -> SemanticPathAuditResult:
        canonical = dict(canonical_metrics or {})
        samples = self._samples(points)
        cells = [self.hospital_map.world_to_cell(x, y) for x, y, _ in samples]
        valid_cells = [cell for cell in cells if cell is not None]
        hard_overlap = sum(
            cell is None or bool(self.raster.hard_footprint_mask[cell]) for cell in cells
        )
        goal_cell = cells[-1] if cells else None
        no_stopping_goal = bool(
            goal_cell is not None and self.raster.no_stopping_mask[goal_cell]
        )
        lane_distances: List[float] = []
        lane_left_distances: List[float] = []
        lane_errors: List[float] = []
        lane_sides: List[bool] = []
        tangent_agreements: List[float] = []
        lane_direction_stabilities: List[float] = []
        lane_instance_samples: Dict[int, Dict[str, List[Any]]] = {}
        parking_deviations: List[float] = []
        transitions: List[float] = []
        left_grid = getattr(preference_field, "lane_distance_to_left_m", None)
        tangent_x_grid = getattr(preference_field, "route_tangent_x", None)
        tangent_y_grid = getattr(preference_field, "route_tangent_y", None)
        lane_instance_grid = getattr(preference_field, "lane_instance_id", None)
        for sample, cell in zip(samples, cells):
            if cell is None:
                continue
            lane_distance = float(preference_field.lane_distance_to_right_m[cell])
            lane_error = float(preference_field.lane_error_m[cell])
            parking = float(preference_field.parking_normalized_deviation[cell])
            if math.isfinite(lane_distance):
                lane_distances.append(lane_distance)
                lane_errors.append(lane_error)
                lane_sides.append(bool(preference_field.lane_correct_side[cell]))
                if left_grid is not None:
                    left_distance = float(left_grid[cell])
                    if math.isfinite(left_distance):
                        lane_left_distances.append(left_distance)
                stability_value = float(preference_field.direction_stability[cell])
                if math.isfinite(stability_value):
                    lane_direction_stabilities.append(stability_value)
                if tangent_x_grid is not None and tangent_y_grid is not None:
                    tx, ty = float(tangent_x_grid[cell]), float(tangent_y_grid[cell])
                    norm = math.hypot(tx, ty)
                    if norm > 1.0e-6:
                        tangent_agreements.append(
                            (math.cos(float(sample[2])) * tx + math.sin(float(sample[2])) * ty) / norm
                        )
                if lane_instance_grid is not None:
                    instance_id = int(lane_instance_grid[cell])
                    if instance_id:
                        values = lane_instance_samples.setdefault(
                            instance_id, {"errors": [], "sides": [], "stability": []},
                        )
                        values["errors"].append(lane_error)
                        values["sides"].append(bool(preference_field.lane_correct_side[cell]))
                        values["stability"].append(stability_value)
            if math.isfinite(parking):
                parking_deviations.append(parking)
            transitions.append(float(preference_field.junction_transition_factor[cell]))
        transition_jumps = [abs(second - first) for first, second in zip(transitions, transitions[1:])]

        region_lengths: Dict[str, float] = {}
        source_lengths: Dict[str, float] = {}
        wrong_way_distance = 0.0
        explicit_direction_observed = False
        relaxed_distance = 0.0
        directional_features = [
            feature for feature in self.semantic_map.features
            if feature.direction_rule == "explicit" and feature.geometry_type == "polygon"
        ]
        for first, second in zip(points, points[1:]):
            x0, y0 = _xy(first)
            x1, y1 = _xy(second)
            length = math.hypot(x1 - x0, y1 - y0)
            if length <= 1e-12:
                continue
            midpoint = ((x0 + x1) * 0.5, (y0 + y1) * 0.5)
            cell = self.hospital_map.world_to_cell(*midpoint)
            classes = [] if cell is None else [
                semantic_class for semantic_class, mask in self.raster.masks.items()
                if bool(mask[cell])
            ]
            if not classes:
                classes = ["unlabelled"]
            for semantic_class in classes:
                region_lengths[semantic_class] = region_lengths.get(semantic_class, 0.0) + length
            if relaxation_level != "R0" and any(value in {"lane", "parking_area"} for value in classes):
                relaxed_distance += length
            source = str(first.get("source", "unclassified")) if isinstance(first, Mapping) else "unclassified"
            source_lengths[source] = source_lengths.get(source, 0.0) + length
            tangent = ((x1 - x0) / length, (y1 - y0) / length)
            for feature in directional_features:
                if not point_in_polygon(midpoint, feature.coordinates):
                    continue
                direction = self._explicit_direction(feature.properties.get("explicit_direction"))
                if direction is None:
                    continue
                explicit_direction_observed = True
                if tangent[0] * direction[0] + tangent[1] * direction[1] < 0.0:
                    wrong_way_distance += length
        path_length = sum(
            math.hypot(_xy(second)[0] - _xy(first)[0], _xy(second)[1] - _xy(first)[1])
            for first, second in zip(points, points[1:])
        )
        increment = (
            path_length - float(baseline_path_length_m)
            if baseline_path_length_m is not None else None
        )
        collision_count = int(canonical.get("collision_count", 0))
        if canonical.get("static_footprint_valid") is False:
            collision_count = max(1, collision_count)
        kinematic_violation_count = 0
        if canonical.get("kinematic_valid") is False:
            kinematic_violation_count = 1
        kinematic_violation_count += int(canonical.get("in_place_rotation_count") or 0)
        kinematic_violation_count += int(float(canonical.get("reverse_distance_m") or 0.0) > 1e-6)
        kinematic_violation_count += int(
            float(canonical.get("maximum_curvature") or 0.0) > 2.5 + 1e-6
        )
        hard_semantic_violation_count = int(hard_overlap > 0) + int(wrong_way_distance > 1e-9)
        hard_constraints_held = (
            collision_count == 0 and kinematic_violation_count == 0
            and hard_semantic_violation_count == 0 and not no_stopping_goal
        )
        failure_code = ""
        if collision_count:
            failure_code = "STATIC_FOOTPRINT_COLLISION"
        elif kinematic_violation_count:
            failure_code = "KINEMATIC_CONSTRAINT_VIOLATION"
        elif hard_overlap:
            failure_code = "HARD_SEMANTIC_VIOLATION"
        elif wrong_way_distance > 1e-9:
            failure_code = "EXPLICIT_DIRECTION_VIOLATION"
        elif no_stopping_goal:
            failure_code = "NO_STOPPING_GOAL_VIOLATION"
        metrics = {
            "hard_semantic_violation_count": hard_semantic_violation_count,
            "forbidden_footprint_overlap_count": int(hard_overlap),
            "no_stopping_goal_violation": no_stopping_goal,
            "lane_distance_to_right_boundary_p50_m": _percentile(lane_distances, 50),
            "lane_distance_to_right_boundary_p95_m": _percentile(lane_distances, 95),
            "lane_distance_to_left_boundary_p50_m": _percentile(lane_left_distances, 50),
            "lane_distance_to_left_boundary_p95_m": _percentile(lane_left_distances, 95),
            "lane_correct_side_ratio": float(np.mean(lane_sides)) if lane_sides else None,
            "base_center_to_right_boundary_error_p50_m": _percentile(lane_errors, 50),
            "base_center_to_right_boundary_error_p95_m": _percentile(lane_errors, 95),
            "parking_center_normalized_deviation_p50": _percentile(parking_deviations, 50),
            "parking_center_normalized_deviation_p95": _percentile(parking_deviations, 95),
            "path_vs_route_tangent_agreement_p50": _percentile(tangent_agreements, 50),
            "path_vs_route_tangent_agreement_p05": _percentile(tangent_agreements, 5),
            "lane_direction_stability_p50": _percentile(lane_direction_stabilities, 50),
            "lane_direction_stability_p05": _percentile(lane_direction_stabilities, 5),
            "junction_transition_discontinuity_count": int(sum(value > 0.35 for value in transition_jumps)),
            "junction_transition_max_jump": max(transition_jumps, default=0.0),
            "wrong_way_distance_m": float(wrong_way_distance),
            "explicit_direction_observed": explicit_direction_observed,
            "relaxation_level": relaxation_level,
            "relaxed_preference_distance_m": float(relaxed_distance),
            "semantic_region_path_length_m": dict(sorted(region_lengths.items())),
            "segment_source_path_length_m": dict(sorted(source_lengths.items())),
            "path_length_m": float(path_length),
            "path_length_increment_vs_no_semantics_m": increment,
            "collision_violation_count": collision_count,
            "kinematic_violation_count": kinematic_violation_count,
            "reverse_distance_m": float(canonical.get("reverse_distance_m") or 0.0),
            "in_place_rotation_count": int(canonical.get("in_place_rotation_count") or 0),
            "maximum_curvature": canonical.get("maximum_curvature"),
            "lane_instance_path_metrics": {
                str(instance_id): {
                    "sample_count": len(values["errors"]),
                    "correct_side_ratio": float(np.mean(values["sides"])) if values["sides"] else None,
                    "target_error_p50_m": _percentile(values["errors"], 50),
                    "target_error_p95_m": _percentile(values["errors"], 95),
                    "direction_stability_p50": _percentile(values["stability"], 50),
                }
                for instance_id, values in sorted(lane_instance_samples.items())
            },
        }
        return SemanticPathAuditResult(
            metrics=metrics, hard_constraints_held=hard_constraints_held,
            failure_code=failure_code, sampled_pose_count=len(samples),
            diagnostics={
                "semantic_map_hash": self.semantic_map.semantic_map_hash,
                "raster_hash": self.raster.raster_hash,
                "no_stopping_scope": "goal_and_task_endpoints_only",
                "no_stopping_runtime_boundary": (
                    "global planning cannot guarantee that a downstream controller never pauses transiently"
                ),
                "lane_direction_source": "selected_l1_route_tangent_unless_explicit",
                "lane_correct_side_definition": (
                    "distance_right_le_distance_left" if left_grid is not None
                    else "legacy_preference_field_definition"
                ),
            },
        )


__all__ = ["SemanticPathAuditResult", "SemanticPathAuditor"]
