"""Direction-safe, crop-native regional preference geometry for 2A-V2 r2."""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from .regional_preference import PreferenceField
from .regional_preference_r1 import (
    CroppedGrid,
    DEFAULT_R1_POLICY,
    RegionalPreferenceBuilderR1,
    _huber_plateau,
    expand_roi_to_route_lane_instances,
    orient_route_for_query,
)
from .semantic_map import canonical_hash


DEFAULT_R2_POLICY: Dict[str, Any] = {
    **DEFAULT_R1_POLICY,
    # Keep the r1 comfort magnitude.  Root-cause probes with 1 m and 2 m
    # scales, and 24/40 caps, all exhausted the frozen one-million-state Smac
    # budget on the 155 m reverse query.  r2 changes geometry propagation and
    # allocation behavior, not the evidence-backed comfort magnitude.
    "lane_error_scale_m": 5.0,
}


def _route_raster_crop(
    hospital_map: Any, route: Sequence[Sequence[float]],
    relevant: np.ndarray, margin_cells: int,
) -> Tuple[Tuple[int, int, int, int], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Rasterize a start->goal route directly into its active crop."""
    shape = relevant.shape
    route_segments = []
    min_row, min_col = shape[0], shape[1]
    max_row = max_col = -1
    relevant_cells = np.argwhere(relevant)
    if relevant_cells.size:
        min_row, min_col = relevant_cells.min(axis=0)
        max_row, max_col = relevant_cells.max(axis=0)
    for first, second in zip(route, route[1:]):
        dx = float(second[0]) - float(first[0])
        dy = float(second[1]) - float(first[1])
        length = math.hypot(dx, dy)
        if length <= 1.0e-9:
            continue
        first_cell = hospital_map.world_to_cell(float(first[0]), float(first[1]))
        second_cell = hospital_map.world_to_cell(float(second[0]), float(second[1]))
        if first_cell is None or second_cell is None:
            continue
        sample_count = max(
            abs(int(second_cell[0]) - int(first_cell[0])),
            abs(int(second_cell[1]) - int(first_cell[1])), 1,
        ) + 1
        rows = np.rint(np.linspace(first_cell[0], second_cell[0], sample_count)).astype(np.int32)
        cols = np.rint(np.linspace(first_cell[1], second_cell[1], sample_count)).astype(np.int32)
        cells = np.unique(np.column_stack((rows, cols)), axis=0)
        rows, cols = cells[:, 0], cells[:, 1]
        min_row, max_row = min(min_row, int(rows.min())), max(max_row, int(rows.max()))
        min_col, max_col = min(min_col, int(cols.min())), max(max_col, int(cols.max()))
        route_segments.append((rows, cols, dx / length, dy / length))
    if max_row < min_row or max_col < min_col:
        bounds = (0, shape[0], 0, shape[1])
    else:
        bounds = (
            max(0, int(min_row) - int(margin_cells)),
            min(shape[0], int(max_row) + int(margin_cells) + 1),
            max(0, int(min_col) - int(margin_cells)),
            min(shape[1], int(max_col) + int(margin_cells) + 1),
        )
    row0, row1, col0, col1 = bounds
    crop_shape = (row1 - row0, col1 - col0)
    marker = np.zeros(crop_shape, dtype=bool)
    tangent_x = np.zeros(crop_shape, dtype=np.float32)
    tangent_y = np.zeros(crop_shape, dtype=np.float32)
    counts = np.zeros(crop_shape, dtype=np.uint16)
    for rows, cols, tx, ty in route_segments:
        inside = (rows >= row0) & (rows < row1) & (cols >= col0) & (cols < col1)
        rows, cols = rows[inside] - row0, cols[inside] - col0
        marker[rows, cols] = True
        tangent_x[rows, cols] += float(tx)
        tangent_y[rows, cols] += float(ty)
        counts[rows, cols] += 1
    valid = counts > 0
    tangent_x[valid] /= counts[valid]
    tangent_y[valid] /= counts[valid]
    norm = np.hypot(tangent_x, tangent_y)
    normalized = norm > 1.0e-6
    tangent_x[normalized] /= norm[normalized]
    tangent_y[normalized] /= norm[normalized]
    stability = np.zeros(crop_shape, dtype=np.float32)
    stability[valid] = np.clip(norm[valid], 0.0, 1.0)
    return bounds, marker, tangent_x, tangent_y, stability


class RegionalPreferenceBuilderR2(RegionalPreferenceBuilderR1):
    """Build the same boundary-pair metric without full-map float temporaries."""

    def __init__(
        self, hospital_map: Any, raster: Any, *, policy: Optional[Mapping[str, Any]] = None,
        semantic_map: Any = None,
    ) -> None:
        merged = {**DEFAULT_R2_POLICY, **dict(policy or {})}
        super().__init__(hospital_map, raster, policy=merged, semantic_map=semantic_map)
        self.policy = merged
        self.policy_hash = canonical_hash(self.policy)

    def _probe_seed_boundaries(
        self, seed_mask: np.ndarray, label_grid: np.ndarray,
        tangent_x: np.ndarray, tangent_y: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Vectorized per-step probes, always constrained to one lane ID."""
        right = np.full(seed_mask.shape, np.nan, dtype=np.float32)
        left = np.full(seed_mask.shape, np.nan, dtype=np.float32)
        seeds = np.argwhere(seed_mask)
        if not seeds.size:
            return right, left
        rows = seeds[:, 0].astype(np.float64)
        cols = seeds[:, 1].astype(np.float64)
        tx = tangent_x[seeds[:, 0], seeds[:, 1]].astype(np.float64)
        ty = tangent_y[seeds[:, 0], seeds[:, 1]].astype(np.float64)
        labels = label_grid[seeds[:, 0], seeds[:, 1]]
        stable = np.hypot(tx, ty) >= 0.5
        resolution = float(self.hospital_map.resolution)
        max_steps = int(math.ceil(float(self.policy["max_boundary_probe_m"]) / resolution))
        outputs = []
        for sign in (1.0, -1.0):
            distances = np.zeros(len(seeds), dtype=np.float32)
            alive = stable.copy()
            previous_row = np.full(len(seeds), -1, dtype=np.int32)
            previous_col = np.full(len(seeds), -1, dtype=np.int32)
            for step in range(1, max_steps + 1):
                probe_row = np.rint(rows + sign * step * tx).astype(np.int32)
                probe_col = np.rint(cols + sign * step * ty).astype(np.int32)
                duplicate = (probe_row == previous_row) & (probe_col == previous_col)
                previous_row, previous_col = probe_row, probe_col
                inside = (
                    (probe_row >= 0) & (probe_row < label_grid.shape[0])
                    & (probe_col >= 0) & (probe_col < label_grid.shape[1])
                )
                candidates = alive & ~duplicate & inside
                same = np.zeros(len(seeds), dtype=bool)
                same[candidates] = (
                    label_grid[probe_row[candidates], probe_col[candidates]] == labels[candidates]
                )
                # A duplicate rounded cell is not an exit; all other failures
                # terminate propagation for this seed.
                alive &= duplicate | same
                distances[same] = float(step) * resolution
                if not np.any(alive):
                    break
            outputs.append(distances)
        right[seeds[:, 0], seeds[:, 1]] = outputs[0]
        left[seeds[:, 0], seeds[:, 1]] = outputs[1]
        return right, left

    def build(
        self, route: Sequence[Sequence[float]], *, goal: Optional[Sequence[float]] = None,
        allowed_mask: Optional[np.ndarray] = None, relaxation_level: str = "R0",
        planning_preference_enabled: bool = True,
        route_diagnostics: Optional[Mapping[str, Any]] = None,
    ) -> PreferenceField:
        started = time.monotonic_ns()
        if relaxation_level not in {"R0", "R1", "R2", "R3", "R4"}:
            raise ValueError(f"invalid relaxation level: {relaxation_level}")
        shape = (int(self.raster.height), int(self.raster.width))
        permitted = np.ones(shape, dtype=bool) if allowed_mask is None else np.asarray(allowed_mask, dtype=bool)
        if permitted.shape != shape:
            raise ValueError("allowed_mask shape mismatch")
        route_hash = canonical_hash([[float(point[0]), float(point[1])] for point in route])
        cost = np.zeros(shape, dtype=np.uint8)
        active = np.zeros(shape, dtype=bool)
        if len(route) < 2:
            empty_float = CroppedGrid(shape, (0, 0, 0, 0), np.empty((0, 0), np.float32), np.nan)
            empty_bool = CroppedGrid(shape, (0, 0, 0, 0), np.empty((0, 0), bool), False)
            empty_int = CroppedGrid(shape, (0, 0, 0, 0), np.empty((0, 0), np.int16), 0)
            empty_zero = CroppedGrid(shape, (0, 0, 0, 0), np.empty((0, 0), np.float32), 0.0)
            return PreferenceField(
                cost, empty_float, empty_float, empty_bool, empty_float,
                CroppedGrid(shape, (0, 0, 0, 0), np.empty((0, 0), np.float32), 1.0),
                empty_zero, active, relaxation_level, self.policy_hash, route_hash,
                {"empty_route": True}, empty_float, empty_zero, empty_zero, empty_int,
            )

        raster_started = time.monotonic_ns()
        parking_full = np.asarray(
            self.raster.masks.get("parking_area", np.zeros(shape, dtype=bool)), dtype=bool,
        )
        relevant = permitted & ((self._lane_labels > 0) | parking_full)
        margin = int(math.ceil(float(self.policy["route_crop_margin_m"]) / self.hospital_map.resolution))
        bounds, route_crop, route_tx, route_ty, route_stability = _route_raster_crop(
            self.hospital_map, route, relevant, margin,
        )
        route_raster_ms = (time.monotonic_ns() - raster_started) / 1.0e6
        row0, row1, col0, col1 = bounds
        target = np.s_[row0:row1, col0:col1]
        permitted_crop = permitted[target]
        label_crop = self._lane_labels[target]
        crop_shape = route_crop.shape
        lane_distance = np.full(crop_shape, np.nan, dtype=np.float32)
        lane_left = np.full(crop_shape, np.nan, dtype=np.float32)
        lane_error = np.full(crop_shape, np.nan, dtype=np.float32)
        lane_correct = np.zeros(crop_shape, dtype=bool)
        parking_deviation = np.full(crop_shape, np.nan, dtype=np.float32)
        transition_factor = np.ones(crop_shape, dtype=np.float32)
        stability_grid = np.zeros(crop_shape, dtype=np.float32)
        tangent_x_grid = np.zeros(crop_shape, dtype=np.float32)
        tangent_y_grid = np.zeros(crop_shape, dtype=np.float32)
        lane_instance_grid = np.zeros(crop_shape, dtype=np.int16)

        probe_started = time.monotonic_ns()
        right_seed, left_seed = self._probe_seed_boundaries(
            route_crop & (label_crop > 0), label_crop, route_tx, route_ty,
        )
        boundary_probe_ms = (time.monotonic_ns() - probe_started) / 1.0e6
        propagate_started = time.monotonic_ns()
        lane_segment_stats = []
        desired = float(self.policy["lane_base_center_to_right_boundary_m"])
        labels = [int(value) for value in np.unique(label_crop[permitted_crop]) if int(value) > 0]
        for label in labels:
            component = (label_crop == label) & permitted_crop
            seeds = route_crop & (label_crop == label)
            cells = np.argwhere(component)
            seed_cells = np.argwhere(seeds)
            if not cells.size or not seed_cells.size:
                continue
            local_row0, local_col0 = cells.min(axis=0)
            local_row1, local_col1 = cells.max(axis=0) + 1
            local = np.s_[local_row0:local_row1, local_col0:local_col1]
            component_local = component[local]
            seeds_local = seeds[local]
            nearest = ndimage.distance_transform_edt(
                ~seeds_local, return_distances=False, return_indices=True,
            )
            nr = nearest[0] + int(local_row0)
            nc = nearest[1] + int(local_col0)
            tx = route_tx[nr, nc]
            ty = route_ty[nr, nc]
            stability = route_stability[nr, nc]
            rows, cols = np.indices(component_local.shape)
            global_rows = rows + int(local_row0)
            global_cols = cols + int(local_col0)
            delta_col = (global_cols - nc) * float(self.hospital_map.resolution)
            delta_world_y = -(global_rows - nr) * float(self.hospital_map.resolution)
            signed_right = delta_col * ty + delta_world_y * (-tx)
            d_right = right_seed[nr, nc] - signed_right
            d_left = left_seed[nr, nc] + signed_right
            valid = (
                component_local & np.isfinite(d_right) & np.isfinite(d_left)
                & (d_right >= 0.0) & (d_left >= 0.0)
            )
            if not np.any(valid):
                continue
            distance_view = lane_distance[local]
            left_view = lane_left[local]
            error_view = lane_error[local]
            correct_view = lane_correct[local]
            stability_view = stability_grid[local]
            tangent_x_view = tangent_x_grid[local]
            tangent_y_view = tangent_y_grid[local]
            instance_view = lane_instance_grid[local]
            distance_view[valid] = d_right[valid]
            left_view[valid] = d_left[valid]
            error = np.abs(d_right - desired)
            error_view[valid] = error[valid]
            correct_view[valid] = d_right[valid] <= d_left[valid]
            stability_view[valid] = stability[valid]
            tangent_x_view[valid] = tx[valid]
            tangent_y_view[valid] = ty[valid]
            instance_view[valid] = label
            target_band = valid & (error <= 0.50)
            nearest_flat = (
                nr.astype(np.int64) * crop_shape[1] + nc.astype(np.int64)
            )[target_band]
            route_seed_flat = np.flatnonzero(seeds)
            lane_segment_stats.append({
                "lane_segment_id": label,
                "lane_semantic_id": self._lane_instance_ids.get(label, ""),
                "cell_count": int(np.count_nonzero(valid)),
                "direction_stability_p50": float(np.percentile(stability[valid], 50)),
                "direction_stability_min": float(np.min(stability[valid])),
                "target_band_cell_count": int(np.count_nonzero(target_band)),
                "target_band_route_seed_coverage_ratio": (
                    float(len(np.unique(nearest_flat)) / len(route_seed_flat))
                    if len(route_seed_flat) else 0.0
                ),
            })
        instance_propagation_ms = (time.monotonic_ns() - propagate_started) / 1.0e6

        junction = np.asarray(
            self.raster.masks.get("junction_area", np.zeros(shape, dtype=bool)), dtype=bool,
        )
        junction_crop = junction[target]
        if np.any(junction_crop):
            transition = np.clip(
                ndimage.distance_transform_edt(
                    ~junction_crop, sampling=float(self.hospital_map.resolution),
                ) / float(self.policy["junction_transition_distance_m"]),
                0.0, 1.0,
            ).astype(np.float32)
            transition[junction_crop] = 0.0
        else:
            transition = np.ones(crop_shape, dtype=np.float32)
        transition_factor[:] = transition

        valid_lane = np.isfinite(lane_error) & permitted_crop
        normalized = _huber_plateau(
            lane_error, tolerance=float(self.policy["lane_target_tolerance_m"]),
            delta=float(self.policy["lane_huber_delta_m"]),
            scale=float(self.policy["lane_error_scale_m"]),
        )
        lane_weight = transition * np.clip(
            stability_grid, float(self.policy["direction_instability_floor"]), 1.0,
        )
        lateral_scale = 1.0 if relaxation_level == "R0" else float(self.policy["r1_lateral_weight_scale"])
        if relaxation_level in {"R2", "R3", "R4"}:
            clearance = ndimage.distance_transform_edt(
                label_crop > 0, sampling=float(self.hospital_map.resolution),
            )
            lane_weight[
                (2.0 * clearance < float(self.policy["narrow_channel_width_m"]))
                | (transition < 1.0)
            ] = 0.0
            if goal is not None:
                rows, cols = np.indices(crop_shape)
                wx = self.hospital_map.origin[0] + (cols + col0 + 0.5) * self.hospital_map.resolution
                wy = self.hospital_map.origin[1] + (shape[0] - (rows + row0) - 0.5) * self.hospital_map.resolution
                lane_weight[
                    np.hypot(wx - float(goal[0]), wy - float(goal[1]))
                    <= float(self.policy["endpoint_relax_radius_m"])
                ] = 0.0
        lane_values = np.clip(
            float(self.policy["lane_cost_cap"]) * lateral_scale
            * np.nan_to_num(normalized, nan=0.0) * lane_weight,
            0.0, float(self.policy["lane_cost_cap"]),
        ).astype(np.uint8)
        if planning_preference_enabled:
            cost_view = cost[target]
            cost_view[valid_lane] = lane_values[valid_lane]
            active_view = active[target]
            active_view[valid_lane] = lane_weight[valid_lane] > 0.0

        parking_crop = parking_full[target] & permitted_crop
        if np.any(parking_crop):
            base_deviation = self._parking_base[target]
            taper = np.ones(crop_shape, dtype=np.float32)
            if goal is not None:
                rows, cols = np.indices(crop_shape)
                wx = self.hospital_map.origin[0] + (cols + col0 + 0.5) * self.hospital_map.resolution
                wy = self.hospital_map.origin[1] + (shape[0] - (rows + row0) - 0.5) * self.hospital_map.resolution
                taper = np.clip(
                    np.hypot(wx - float(goal[0]), wy - float(goal[1]))
                    / float(self.policy["parking_endpoint_taper_distance_m"]),
                    0.0, 1.0,
                ).astype(np.float32)
            parking_deviation[parking_crop] = base_deviation[parking_crop]
            parking_values = np.clip(
                float(self.policy["parking_cost_cap"]) * lateral_scale
                * np.nan_to_num(base_deviation, nan=0.0) * taper,
                0.0, float(self.policy["parking_cost_cap"]),
            ).astype(np.uint8)
            if planning_preference_enabled:
                cost_view = cost[target]
                cost_view[parking_crop] = parking_values[parking_crop]
                active_view = active[target]
                active_view[parking_crop] = taper[parking_crop] > 0.0
        cost[junction & permitted] = 0
        active[junction] = False
        nonzero = cost[cost > 0]
        edges = [1, 9, 17, 33, 49, 65, 81, 129, 201, 254]
        diagnostics = {
            **dict(route_diagnostics or {}),
            "geometry_revision": "r2_crop_native_lane_instance_boundary_pair",
            "correct_side_definition": "distance_right_le_distance_left",
            "lane_target_tolerance_m": float(self.policy["lane_target_tolerance_m"]),
            "lane_error_scale_m": float(self.policy["lane_error_scale_m"]),
            "lane_cost_cap": int(self.policy["lane_cost_cap"]),
            "parking_cost_cap": int(self.policy["parking_cost_cap"]),
            "planning_preference_enabled": bool(planning_preference_enabled),
            "route_direction_source": "query_start_to_goal_selected_l1_route_tangent",
            "lane_instance_propagation": "same_semantic_feature_only",
            "lane_segment_direction_stability": lane_segment_stats,
            "lane_preference_cell_count": int(np.count_nonzero(np.isfinite(lane_distance))),
            "parking_preference_cell_count": int(np.count_nonzero(np.isfinite(parking_deviation))),
            "active_lateral_cell_count": int(np.count_nonzero(active)),
            "junction_neutral_cell_count": int(np.count_nonzero(junction & permitted)),
            "soft_cost_histogram_edges": edges,
            "soft_cost_histogram_counts": (
                np.histogram(nonzero, bins=edges)[0].tolist()
                if nonzero.size else [0] * (len(edges) - 1)
            ),
            "soft_cost_saturation_ratio": float(
                np.mean(nonzero >= max(
                    int(self.policy["lane_cost_cap"]), int(self.policy["parking_cost_cap"]),
                ))
            ) if nonzero.size else 0.0,
            "soft_cost_effective_area_m2": float(
                np.count_nonzero(nonzero) * self.hospital_map.resolution ** 2
            ),
            "field_crop_bbox": [int(row0), int(row1), int(col0), int(col1)],
            "field_crop_cells": int((row1 - row0) * (col1 - col0)),
            "route_raster_ms": route_raster_ms,
            "boundary_probe_ms": boundary_probe_ms,
            "instance_propagation_ms": instance_propagation_ms,
            "field_build_ms": (time.monotonic_ns() - started) / 1.0e6,
            "parking_static_precompute_ms": float(self._static_precompute_ms),
            "relaxation_level": relaxation_level,
            "lateral_weight_scale": lateral_scale,
            "polygon_vertex_order_used_for_direction": False,
        }
        return PreferenceField(
            cost=cost,
            lane_distance_to_right_m=CroppedGrid(shape, bounds, lane_distance, np.nan),
            lane_error_m=CroppedGrid(shape, bounds, lane_error, np.nan),
            lane_correct_side=CroppedGrid(shape, bounds, lane_correct, False),
            parking_normalized_deviation=CroppedGrid(shape, bounds, parking_deviation, np.nan),
            junction_transition_factor=CroppedGrid(shape, bounds, transition_factor, 1.0),
            direction_stability=CroppedGrid(shape, bounds, stability_grid, 0.0),
            active_lateral_mask=active, relaxation_level=relaxation_level,
            policy_hash=self.policy_hash, route_hash=route_hash, diagnostics=diagnostics,
            lane_distance_to_left_m=CroppedGrid(shape, bounds, lane_left, np.nan),
            route_tangent_x=CroppedGrid(shape, bounds, tangent_x_grid, 0.0),
            route_tangent_y=CroppedGrid(shape, bounds, tangent_y_grid, 0.0),
            lane_instance_id=CroppedGrid(shape, bounds, lane_instance_grid, 0),
        )


def classify_semantic_query_feasibility(
    hospital_map: Any, raster: Any, field: PreferenceField,
    allowed_mask: np.ndarray, footprint_safe_mask: np.ndarray,
    start: Sequence[float], goal: Sequence[float], *, target_error_m: float = 0.50,
) -> Dict[str, Any]:
    """Algorithm-independent endpoint and target-band connectivity audit."""
    allowed = np.asarray(allowed_mask, dtype=bool)
    footprint_safe = np.asarray(footprint_safe_mask, dtype=bool)
    if allowed.shape != footprint_safe.shape:
        raise ValueError("allowed and footprint-safe masks must have equal shape")
    start_cell = hospital_map.world_to_cell(float(start[0]), float(start[1]))
    goal_cell = hospital_map.world_to_cell(float(goal[0]), float(goal[1]))
    result: Dict[str, Any] = {
        "validity_rule": "r2_endpoint_footprint_and_4_connected_target_band_v1",
        "target_error_threshold_m": float(target_error_m),
        "start_cell": list(start_cell) if start_cell is not None else None,
        "goal_cell": list(goal_cell) if goal_cell is not None else None,
    }
    if start_cell is None or goal_cell is None:
        return {**result, "classification": "INVALID_ENDPOINT", "query_valid": False}
    hard = np.asarray(raster.hard_footprint_mask, dtype=bool)
    no_stopping = np.asarray(raster.no_stopping_mask, dtype=bool)
    endpoint_checks = {
        "start_footprint_safe": bool(footprint_safe[start_cell]),
        "goal_footprint_safe": bool(footprint_safe[goal_cell]),
        "start_hard_semantic_free": not bool(hard[start_cell]),
        "goal_hard_semantic_free": not bool(hard[goal_cell]),
        "goal_no_stopping_free": not bool(no_stopping[goal_cell]),
    }
    result["endpoint_checks"] = endpoint_checks
    if not all(endpoint_checks.values()):
        return {**result, "classification": "SEMANTIC_QUERY_INFEASIBLE", "query_valid": False}
    traversable = allowed & footprint_safe & ~hard
    if not traversable[start_cell] or not traversable[goal_cell]:
        return {**result, "classification": "SEMANTIC_QUERY_INFEASIBLE", "query_valid": False}
    labels, _ = ndimage.label(
        traversable, structure=np.asarray([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8),
    )
    start_component = int(labels[start_cell])
    goal_component = int(labels[goal_cell])
    lane_error = np.asarray(field.lane_error_m)
    lane_correct = np.asarray(field.lane_correct_side, dtype=bool)
    target_band = (
        traversable & np.isfinite(lane_error) & lane_correct
        & (lane_error <= float(target_error_m))
    )
    reachable_target = target_band & (labels == start_component)
    result.update({
        "start_component": start_component,
        "goal_component": goal_component,
        "target_band_cells": int(np.count_nonzero(target_band)),
        "reachable_target_band_cells": int(np.count_nonzero(reachable_target)),
        "target_band_area_m2": float(
            np.count_nonzero(target_band) * hospital_map.resolution ** 2
        ),
    })
    valid = bool(
        start_component > 0 and start_component == goal_component
        and np.any(reachable_target)
    )
    return {
        **result,
        "classification": "VALID" if valid else "SEMANTIC_QUERY_INFEASIBLE",
        "query_valid": valid,
    }


__all__ = [
    "DEFAULT_R2_POLICY", "RegionalPreferenceBuilderR2",
    "classify_semantic_query_feasibility", "expand_roi_to_route_lane_instances",
    "orient_route_for_query",
]
