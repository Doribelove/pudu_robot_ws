"""r1 query-oriented, lane-instance-safe regional preference fields.

The r0 builder intentionally remains available for reproducing run12.  This
module fixes the evaluation geometry and lowers comfort-cost aggressiveness
without changing any lethal/static/kinematic invariant.
"""

from __future__ import annotations

import copy
import math
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy import ndimage

from .regional_preference import PreferenceField, _bounds, _route_raster
from .semantic_map import canonical_hash
from .semantic_rasterizer import rasterize_feature


DEFAULT_R1_POLICY: Dict[str, Any] = {
    "lane_base_center_to_right_boundary_m": 0.40,
    "lane_target_tolerance_m": 0.20,
    "lane_huber_delta_m": 1.0,
    "lane_error_scale_m": 5.0,
    "lane_cost_cap": 64,
    "parking_cost_cap": 48,
    "junction_transition_distance_m": 0.75,
    "parking_endpoint_taper_distance_m": 1.00,
    "endpoint_relax_radius_m": 0.75,
    "narrow_channel_width_m": 1.10,
    "direction_instability_floor": 0.20,
    "r1_lateral_weight_scale": 0.50,
    "max_boundary_probe_m": 12.0,
    "route_crop_margin_m": 1.0,
}


class CroppedGrid:
    """Array-compatible immutable grid backed only by one ROI crop.

    Semantic auditing uses scalar lookups, while the costmap composer only
    materializes the one transition grid it needs.  Keeping eight float32
    full-map diagnostic arrays per relaxation level caused multi-GB RSS.
    """

    __array_priority__ = 1000

    def __init__(
        self, shape: Tuple[int, int], bounds: Tuple[int, int, int, int],
        values: np.ndarray, fill_value: float,
    ) -> None:
        self.shape = tuple(int(v) for v in shape)
        self.bounds = tuple(int(v) for v in bounds)
        self.values = np.asarray(values).copy()
        self.fill_value = fill_value
        self.dtype = self.values.dtype
        self.ndim = 2
        self.size = int(self.shape[0] * self.shape[1])

    def __array__(self, dtype: Any = None) -> np.ndarray:
        result = np.full(self.shape, self.fill_value, dtype=self.dtype if dtype is None else dtype)
        row0, row1, col0, col1 = self.bounds
        result[row0:row1, col0:col1] = self.values.astype(result.dtype, copy=False)
        return result

    def __getitem__(self, key: Any) -> Any:
        if isinstance(key, tuple) and len(key) == 2 and all(isinstance(value, (int, np.integer)) for value in key):
            row, col = int(key[0]), int(key[1])
            row0, row1, col0, col1 = self.bounds
            if row0 <= row < row1 and col0 <= col < col1:
                return self.values[row - row0, col - col0]
            return self.fill_value
        if isinstance(key, tuple) and len(key) == 2 and all(isinstance(value, slice) for value in key):
            row_slice, col_slice = key
            if (row_slice.step in (None, 1) and col_slice.step in (None, 1)):
                start_row, stop_row, _ = row_slice.indices(self.shape[0])
                start_col, stop_col, _ = col_slice.indices(self.shape[1])
                result = np.full((stop_row - start_row, stop_col - start_col), self.fill_value, dtype=self.dtype)
                row0, row1, col0, col1 = self.bounds
                overlap_r0, overlap_r1 = max(start_row, row0), min(stop_row, row1)
                overlap_c0, overlap_c1 = max(start_col, col0), min(stop_col, col1)
                if overlap_r0 < overlap_r1 and overlap_c0 < overlap_c1:
                    result[
                        overlap_r0 - start_row:overlap_r1 - start_row,
                        overlap_c0 - start_col:overlap_c1 - start_col,
                    ] = self.values[
                        overlap_r0 - row0:overlap_r1 - row0,
                        overlap_c0 - col0:overlap_c1 - col0,
                    ]
                return result
        return np.asarray(self)[key]

    def __le__(self, other: Any) -> np.ndarray:
        return np.asarray(self) <= other

    def __lt__(self, other: Any) -> np.ndarray:
        return np.asarray(self) < other

    def __ge__(self, other: Any) -> np.ndarray:
        return np.asarray(self) >= other

    def __gt__(self, other: Any) -> np.ndarray:
        return np.asarray(self) > other


def _crop_grid(
    value: np.ndarray, bounds: Tuple[int, int, int, int], fill_value: float,
) -> CroppedGrid:
    row0, row1, col0, col1 = bounds
    return CroppedGrid(value.shape, bounds, value[row0:row1, col0:col1], fill_value)


def expand_roi_to_route_lane_instances(
    hospital_map: Any,
    raster: Any,
    semantic_map: Any,
    route_polyline: Sequence[Sequence[float]],
    base_allowed: np.ndarray,
    *,
    free_mask: Optional[np.ndarray] = None,
    route_probe_radius_m: float = 0.50,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Include full route-crossed lane instances without admitting neighbours.

    Junction pixels are removed before connected-component labelling so a
    junction cannot merge several lanes into one expansion.  Only components
    touched by the route probe are admitted and added cells must remain in the
    footprint-safe ``free_mask`` when it is supplied.
    """
    allowed = np.asarray(base_allowed, dtype=bool)
    masks = getattr(raster, "masks", {})
    junction = np.asarray(
        masks.get("junction_area", np.zeros_like(allowed)), dtype=bool,
    )
    route_mask, _, _, _ = _route_raster(hospital_map, route_polyline, allowed.shape)
    radius_cells = max(
        1, int(math.ceil(float(route_probe_radius_m) / hospital_map.resolution)),
    )
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * radius_cells + 1, 2 * radius_cells + 1),
    )
    route_probe = cv2.dilate(route_mask.astype(np.uint8), kernel).astype(bool)
    cached = getattr(raster, "_r1_lane_instance_crops", None)
    if cached is None:
        cached = []
        for feature in sorted(
            (item for item in semantic_map.features if item.semantic_class == "lane"),
            key=lambda item: item.semantic_id,
        ):
            feature_mask = rasterize_feature(semantic_map, feature) & ~junction
            cells = np.argwhere(feature_mask)
            if not cells.size:
                continue
            row0, col0 = cells.min(axis=0)
            row1, col1 = cells.max(axis=0) + 1
            bounds = (int(row0), int(row1), int(col0), int(col1))
            cached.append((str(feature.semantic_id), bounds, feature_mask[row0:row1, col0:col1].copy()))
        setattr(raster, "_r1_lane_instance_crops", cached)
    selected = np.zeros_like(allowed)
    selected_ids = []
    for semantic_id, bounds, crop in cached:
        row0, row1, col0, col1 = bounds
        if np.any(crop & route_probe[row0:row1, col0:col1]):
            selected_ids.append(semantic_id)
            view = selected[row0:row1, col0:col1]
            view |= crop
    if free_mask is not None:
        selected &= np.asarray(free_mask, dtype=bool)
    expanded = allowed | selected
    added = selected & ~allowed
    added_cells = int(np.count_nonzero(added))
    return expanded, {
        "lane_instance_roi_expansion": True,
        "lane_instance_count": len(cached),
        "selected_lane_instance_ids": selected_ids,
        "selected_lane_instance_count": len(selected_ids),
        "lane_instance_added_cells": added_cells,
        "lane_instance_added_area_m2": float(added_cells * hospital_map.resolution ** 2),
        "lane_instance_adjacent_instances_excluded": max(0, len(cached) - len(selected_ids)),
        "lane_route_probe_radius_m": float(route_probe_radius_m),
    }


def _xy(pose: Sequence[float]) -> Tuple[float, float]:
    return float(pose[0]), float(pose[1])


def _endpoint_orientation(route: Any, query: Any) -> Dict[str, Any]:
    points = list(getattr(route, "polyline", []) or [])
    if not points:
        return {
            "route_reversed_for_query": False,
            "route_start_distance_m": None,
            "route_end_distance_m": None,
            "route_normal_endpoint_sum_m": None,
            "route_reversed_endpoint_sum_m": None,
        }
    start = _xy(query.start)
    goal = _xy(query.goal)
    normal_start = math.dist(start, _xy(points[0]))
    normal_goal = math.dist(goal, _xy(points[-1]))
    reversed_start = math.dist(start, _xy(points[-1]))
    reversed_goal = math.dist(goal, _xy(points[0]))
    normal = normal_start + normal_goal
    reversed_value = reversed_start + reversed_goal
    return {
        "route_reversed_for_query": bool(reversed_value + 1.0e-9 < normal),
        "route_start_distance_m": float(min(normal_start, reversed_start)),
        "route_end_distance_m": float(min(normal_goal, reversed_goal)),
        "route_normal_endpoint_sum_m": float(normal),
        "route_reversed_endpoint_sum_m": float(reversed_value),
    }


def orient_route_for_query(route: Any, query: Any, *, annotator: Any = None) -> Tuple[Any, Dict[str, Any]]:
    """Return an isolated route whose polyline always runs query start->goal.

    Cached topology routes must never be mutated: forward and reverse queries
    can otherwise poison each other's direction fields.  If reversal is ever
    needed, edge annotations are regenerated in the reversed traversal order
    when an annotator is available.
    """

    diagnostics = _endpoint_orientation(route, query)
    oriented = copy.deepcopy(route)
    if not diagnostics["route_reversed_for_query"]:
        return oriented, diagnostics
    oriented.polyline = [list(point) for point in reversed(oriented.polyline)]
    oriented.node_ids = list(reversed(oriented.node_ids))
    oriented.edge_ids = list(reversed(oriented.edge_ids))
    old_annotations = list(getattr(oriented, "semantic_edge_annotations", []) or [])
    if annotator is not None:
        edge_by_id = {int(edge.edge_id): edge for edge in annotator.topology_edges} if hasattr(annotator, "topology_edges") else {}
        rebuilt = []
        for item in reversed(old_annotations):
            edge = edge_by_id.get(int(item.get("edge_id", -1)))
            if edge is None:
                changed = dict(item)
                changed["traversal_reversed"] = not bool(changed.get("traversal_reversed", False))
                rebuilt.append(changed)
            else:
                rebuilt.append(annotator.annotate(
                    edge,
                    reversed_traversal=not bool(item.get("traversal_reversed", False)),
                ).to_dict())
        setattr(oriented, "semantic_edge_annotations", rebuilt)
    elif old_annotations:
        rebuilt = []
        for item in reversed(old_annotations):
            changed = dict(item)
            changed["traversal_reversed"] = not bool(changed.get("traversal_reversed", False))
            rebuilt.append(changed)
        setattr(oriented, "semantic_edge_annotations", rebuilt)
    return oriented, diagnostics


def _huber_plateau(error: np.ndarray, *, tolerance: float, delta: float, scale: float) -> np.ndarray:
    excess = np.maximum(np.asarray(error, dtype=np.float32) - float(tolerance), 0.0)
    safe_delta = max(float(delta), 1.0e-6)
    huber = np.where(
        excess <= safe_delta,
        0.5 * excess * excess / safe_delta,
        excess - 0.5 * safe_delta,
    )
    return np.clip(huber / max(float(scale), 1.0e-6), 0.0, 1.0).astype(np.float32)


class RegionalPreferenceBuilderR1:
    """Build planning and audit geometry only inside the active ROI crop."""

    def __init__(
        self, hospital_map: Any, raster: Any, *, policy: Optional[Mapping[str, Any]] = None,
        semantic_map: Any = None,
    ):
        self.hospital_map = hospital_map
        self.raster = raster
        self.policy = {**DEFAULT_R1_POLICY, **dict(policy or {})}
        self.policy_hash = canonical_hash(self.policy)
        lane = np.asarray(raster.masks.get("lane", np.zeros((raster.height, raster.width), bool)), bool)
        junction = np.asarray(raster.masks.get("junction_area", np.zeros_like(lane)), bool)
        # Prefer source feature instances.  Connected components of the union
        # can merge adjacent lane polygons before a route is even considered.
        labels = np.zeros(lane.shape, dtype=np.int32)
        self._lane_instance_ids: Dict[int, str] = {}
        if semantic_map is not None:
            instance_index = 0
            for feature in sorted(
                (item for item in semantic_map.features if item.semantic_class == "lane"),
                key=lambda item: item.semantic_id,
            ):
                feature_mask = rasterize_feature(semantic_map, feature) & ~junction
                assign = feature_mask & (labels == 0)
                if not np.any(assign):
                    continue
                instance_index += 1
                labels[assign] = instance_index
                self._lane_instance_ids[instance_index] = str(feature.semantic_id)
        else:
            # Synthetic/legacy fixtures without the source map retain a safe
            # fallback, still split at junction pixels.
            _, labels = cv2.connectedComponents((lane & ~junction).astype(np.uint8), connectivity=8)
            labels = labels.astype(np.int32)
            self._lane_instance_ids = {
                int(value): f"connected-component-{int(value)}"
                for value in np.unique(labels) if int(value) > 0
            }
        self._lane_labels = labels
        self._static_precompute_ms = 0.0
        self._parking_base = self._precompute_parking_deviation()

    def _precompute_parking_deviation(self) -> np.ndarray:
        started = time.monotonic_ns()
        parking = np.asarray(
            self.raster.masks.get("parking_area", np.zeros((self.raster.height, self.raster.width), bool)),
            bool,
        )
        result = np.full(parking.shape, np.nan, dtype=np.float32)
        count, labels = cv2.connectedComponents(parking.astype(np.uint8), connectivity=8)
        map_clearance = np.asarray(getattr(self.hospital_map, "distance_m", np.full(parking.shape, np.inf)), np.float32)
        for label, component_slice in enumerate(ndimage.find_objects(labels, max_label=count - 1), start=1):
            if component_slice is None:
                continue
            component = labels[component_slice] == label
            region_clearance = ndimage.distance_transform_edt(
                component, sampling=float(self.hospital_map.resolution),
            ).astype(np.float32)
            combined = np.minimum(region_clearance, map_clearance[component_slice])
            maximum = float(np.max(combined[component])) if np.any(component) else 0.0
            target = result[component_slice]
            if maximum > 1.0e-9:
                target[component] = 1.0 - combined[component] / maximum
            else:
                target[component] = 1.0
        self._static_precompute_ms = (time.monotonic_ns() - started) / 1.0e6
        return result

    def _probe_seed_boundaries(
        self, seed_mask: np.ndarray, label_grid: np.ndarray,
        tangent_x: np.ndarray, tangent_y: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        right = np.full(seed_mask.shape, np.nan, dtype=np.float32)
        left = np.full(seed_mask.shape, np.nan, dtype=np.float32)
        resolution = float(self.hospital_map.resolution)
        max_steps = int(math.ceil(float(self.policy["max_boundary_probe_m"]) / resolution))
        for row, col in np.argwhere(seed_mask):
            tx, ty = float(tangent_x[row, col]), float(tangent_y[row, col])
            if math.hypot(tx, ty) < 0.5:
                continue
            label = int(label_grid[row, col])
            distances = []
            for sign in (1.0, -1.0):
                last = 0.0
                previous_cell = None
                for step in range(1, max_steps + 1):
                    probe = (int(round(row + sign * step * tx)), int(round(col + sign * step * ty)))
                    if probe == previous_cell:
                        continue
                    previous_cell = probe
                    if not (0 <= probe[0] < label_grid.shape[0] and 0 <= probe[1] < label_grid.shape[1]):
                        break
                    if int(label_grid[probe]) != label:
                        break
                    last = step * resolution
                distances.append(last)
            right[row, col], left[row, col] = distances
        return right, left

    def derive_relaxation(
        self, base: PreferenceField, *, allowed_mask: np.ndarray,
        relaxation_level: str, goal: Optional[Sequence[float]] = None,
        planning_preference_enabled: bool = True,
    ) -> PreferenceField:
        """Reuse R0 geometry for R1/R2 and only recompose soft costs."""
        if relaxation_level not in {"R1", "R2"}:
            raise ValueError("geometry reuse is limited to R1/R2")
        started = time.monotonic_ns()
        shape = (int(self.raster.height), int(self.raster.width))
        permitted = np.asarray(allowed_mask, dtype=bool)
        error_grid = base.lane_error_m
        if not isinstance(error_grid, CroppedGrid):
            raise ValueError("R1/R2 geometry reuse requires an r1 cropped field")
        bounds = error_grid.bounds
        row0, row1, col0, col1 = bounds
        target = np.s_[row0:row1, col0:col1]
        permitted_crop = permitted[target]
        lane_error = error_grid.values
        stability = base.direction_stability.values
        transition = base.junction_transition_factor.values
        label_crop = self._lane_labels[target]
        valid_lane = np.isfinite(lane_error) & permitted_crop
        normalized = _huber_plateau(
            lane_error, tolerance=float(self.policy["lane_target_tolerance_m"]),
            delta=float(self.policy["lane_huber_delta_m"]),
            scale=float(self.policy["lane_error_scale_m"]),
        )
        lane_weight = transition * np.clip(
            stability, float(self.policy["direction_instability_floor"]), 1.0,
        )
        lateral_scale = float(self.policy["r1_lateral_weight_scale"])
        if relaxation_level == "R2":
            clearance = ndimage.distance_transform_edt(
                label_crop > 0, sampling=float(self.hospital_map.resolution),
            )
            lane_weight[
                (2.0 * clearance < float(self.policy["narrow_channel_width_m"]))
                | (transition < 1.0)
            ] = 0.0
            if goal is not None:
                rows, cols = np.indices(lane_error.shape)
                wx = self.hospital_map.origin[0] + (cols + col0 + 0.5) * self.hospital_map.resolution
                wy = self.hospital_map.origin[1] + (shape[0] - (rows + row0) - 0.5) * self.hospital_map.resolution
                lane_weight[
                    np.hypot(wx - float(goal[0]), wy - float(goal[1]))
                    <= float(self.policy["endpoint_relax_radius_m"])
                ] = 0.0
        cost = np.zeros(shape, dtype=np.uint8)
        active = np.zeros(shape, dtype=bool)
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
        parking = np.asarray(
            self.raster.masks.get("parking_area", np.zeros(shape, bool)), bool,
        )[target] & permitted_crop
        if np.any(parking):
            parking_deviation = base.parking_normalized_deviation.values
            taper = np.ones(lane_error.shape, np.float32)
            if goal is not None:
                rows, cols = np.indices(lane_error.shape)
                wx = self.hospital_map.origin[0] + (cols + col0 + 0.5) * self.hospital_map.resolution
                wy = self.hospital_map.origin[1] + (shape[0] - (rows + row0) - 0.5) * self.hospital_map.resolution
                taper = np.clip(
                    np.hypot(wx - float(goal[0]), wy - float(goal[1]))
                    / float(self.policy["parking_endpoint_taper_distance_m"]), 0.0, 1.0,
                ).astype(np.float32)
            parking_values = np.clip(
                float(self.policy["parking_cost_cap"]) * lateral_scale
                * np.nan_to_num(parking_deviation, nan=0.0) * taper,
                0.0, float(self.policy["parking_cost_cap"]),
            ).astype(np.uint8)
            if planning_preference_enabled:
                cost_view = cost[target]
                cost_view[parking] = parking_values[parking]
                active_view = active[target]
                active_view[parking] = taper[parking] > 0.0
        junction = np.asarray(
            self.raster.masks.get("junction_area", np.zeros(shape, bool)), bool,
        )
        cost[junction & permitted] = 0
        active[junction] = False
        nonzero = cost[cost > 0]
        histogram_edges = [1, 9, 17, 33, 49, 65, 81, 129, 201, 254]
        diagnostics = {
            **dict(base.diagnostics),
            "geometry_cache_hit": True,
            "relaxation_level": relaxation_level,
            "lateral_weight_scale": lateral_scale,
            "planning_preference_enabled": bool(planning_preference_enabled),
            "active_lateral_cell_count": int(np.count_nonzero(active)),
            "soft_cost_histogram_edges": histogram_edges,
            "soft_cost_histogram_counts": (
                np.histogram(nonzero, bins=histogram_edges)[0].tolist()
                if nonzero.size else [0] * (len(histogram_edges) - 1)
            ),
            "soft_cost_saturation_ratio": float(
                np.mean(nonzero >= max(
                    int(self.policy["lane_cost_cap"]), int(self.policy["parking_cost_cap"]),
                ))
            ) if nonzero.size else 0.0,
            "soft_cost_effective_area_m2": float(
                np.count_nonzero(nonzero) * self.hospital_map.resolution ** 2
            ),
            "field_build_ms": (time.monotonic_ns() - started) / 1.0e6,
        }
        return PreferenceField(
            cost=cost,
            lane_distance_to_right_m=base.lane_distance_to_right_m,
            lane_error_m=base.lane_error_m,
            lane_correct_side=base.lane_correct_side,
            parking_normalized_deviation=base.parking_normalized_deviation,
            junction_transition_factor=base.junction_transition_factor,
            direction_stability=base.direction_stability,
            active_lateral_mask=active,
            relaxation_level=relaxation_level,
            policy_hash=self.policy_hash,
            route_hash=base.route_hash,
            diagnostics=diagnostics,
            lane_distance_to_left_m=base.lane_distance_to_left_m,
            route_tangent_x=base.route_tangent_x,
            route_tangent_y=base.route_tangent_y,
            lane_instance_id=base.lane_instance_id,
        )

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
        permitted = np.ones(shape, bool) if allowed_mask is None else np.asarray(allowed_mask, bool)
        if permitted.shape != shape:
            raise ValueError("allowed_mask shape mismatch")
        route_hash = canonical_hash([[float(point[0]), float(point[1])] for point in route])
        cost = np.zeros(shape, np.uint8)
        active = np.zeros(shape, bool)
        if len(route) < 2:
            lane_distance = np.full(shape, np.nan, np.float32)
            lane_left = np.full(shape, np.nan, np.float32)
            lane_error = np.full(shape, np.nan, np.float32)
            lane_correct = np.zeros(shape, bool)
            parking_deviation = np.full(shape, np.nan, np.float32)
            transition_factor = np.ones(shape, np.float32)
            stability_grid = np.zeros(shape, np.float32)
            tangent_x_grid = np.zeros(shape, np.float32)
            tangent_y_grid = np.zeros(shape, np.float32)
            lane_instance_grid = np.zeros(shape, np.int16)
            return PreferenceField(
                cost, lane_distance, lane_error, lane_correct, parking_deviation,
                transition_factor, stability_grid, active, relaxation_level,
                self.policy_hash, route_hash, {"empty_route": True}, lane_left,
                tangent_x_grid, tangent_y_grid, lane_instance_grid,
            )

        route_mask, route_tx, route_ty, route_stability = _route_raster(self.hospital_map, route, shape)
        margin = int(math.ceil(float(self.policy["route_crop_margin_m"]) / self.hospital_map.resolution))
        row0, row1, col0, col1 = _bounds(permitted & (route_mask | (self._lane_labels > 0)), margin)
        target = np.s_[row0:row1, col0:col1]
        route_crop = route_mask[target]
        permitted_crop = permitted[target]
        label_crop = self._lane_labels[target]
        if not np.any(route_crop):
            lane_distance = np.full(shape, np.nan, np.float32)
            lane_left = np.full(shape, np.nan, np.float32)
            lane_error = np.full(shape, np.nan, np.float32)
            lane_correct = np.zeros(shape, bool)
            parking_deviation = np.full(shape, np.nan, np.float32)
            transition_factor = np.ones(shape, np.float32)
            stability_grid = np.zeros(shape, np.float32)
            tangent_x_grid = np.zeros(shape, np.float32)
            tangent_y_grid = np.zeros(shape, np.float32)
            lane_instance_grid = np.zeros(shape, np.int16)
            return PreferenceField(
                cost, lane_distance, lane_error, lane_correct, parking_deviation,
                transition_factor, stability_grid, active, relaxation_level,
                self.policy_hash, route_hash, {"route_not_rasterized": True}, lane_left,
                tangent_x_grid, tangent_y_grid, lane_instance_grid,
            )

        crop_shape = route_crop.shape
        lane_distance = np.full(crop_shape, np.nan, np.float32)
        lane_left = np.full(crop_shape, np.nan, np.float32)
        lane_error = np.full(crop_shape, np.nan, np.float32)
        lane_correct = np.zeros(crop_shape, bool)
        parking_deviation = np.full(crop_shape, np.nan, np.float32)
        transition_factor = np.ones(crop_shape, np.float32)
        stability_grid = np.zeros(crop_shape, np.float32)
        tangent_x_grid = np.zeros(crop_shape, np.float32)
        tangent_y_grid = np.zeros(crop_shape, np.float32)
        lane_instance_grid = np.zeros(crop_shape, np.int16)
        route_tx_crop = route_tx[target]
        route_ty_crop = route_ty[target]
        route_stability_crop = route_stability[target]
        right_seed, left_seed = self._probe_seed_boundaries(
            route_crop & (label_crop > 0), label_crop, route_tx_crop, route_ty_crop,
        )
        lane_segment_stats = []
        desired = float(self.policy["lane_base_center_to_right_boundary_m"])
        labels = [int(value) for value in np.unique(label_crop[permitted_crop]) if int(value) > 0]
        for label in labels:
            component = (label_crop == label) & permitted_crop
            seeds = route_crop & (label_crop == label)
            if not np.any(component) or not np.any(seeds):
                continue
            _, nearest = ndimage.distance_transform_edt(~seeds, return_indices=True)
            nr, nc = nearest[0], nearest[1]
            tx = route_tx_crop[nr, nc]
            ty = route_ty_crop[nr, nc]
            stability = route_stability_crop[nr, nc]
            rows, cols = np.indices(component.shape)
            delta_col = (cols - nc) * float(self.hospital_map.resolution)
            delta_world_y = -(rows - nr) * float(self.hospital_map.resolution)
            signed_right = delta_col * ty + delta_world_y * (-tx)
            d_right = right_seed[nr, nc] - signed_right
            d_left = left_seed[nr, nc] + signed_right
            valid = component & np.isfinite(d_right) & np.isfinite(d_left) & (d_right >= 0.0) & (d_left >= 0.0)
            if not np.any(valid):
                continue
            lane_distance[valid] = d_right[valid]
            lane_left[valid] = d_left[valid]
            error = np.abs(d_right - desired)
            lane_error[valid] = error[valid]
            lane_correct[valid] = d_right[valid] <= d_left[valid]
            stability_grid[valid] = stability[valid]
            tangent_x_grid[valid] = tx[valid]
            tangent_y_grid[valid] = ty[valid]
            lane_instance_grid[valid] = label
            values = stability[valid]
            target_band = valid & (error <= 0.50)
            route_seed_flat = np.flatnonzero(seeds)
            covered_seed_flat = np.unique(
                (nr * component.shape[1] + nc)[target_band]
            )
            lane_segment_stats.append({
                "lane_segment_id": label,
                "lane_semantic_id": self._lane_instance_ids.get(label, ""),
                "cell_count": int(np.count_nonzero(valid)),
                "direction_stability_p50": float(np.percentile(values, 50)),
                "direction_stability_min": float(np.min(values)),
                "target_band_cell_count": int(np.count_nonzero(target_band)),
                "target_band_route_seed_coverage_ratio": (
                    float(len(covered_seed_flat) / len(route_seed_flat)) if len(route_seed_flat) else 0.0
                ),
            })

        junction = np.asarray(self.raster.masks.get("junction_area", np.zeros(shape, bool)), bool)
        junction_crop = junction[target]
        if np.any(junction_crop):
            transition = np.clip(
                ndimage.distance_transform_edt(~junction_crop, sampling=float(self.hospital_map.resolution))
                / float(self.policy["junction_transition_distance_m"]), 0.0, 1.0,
            ).astype(np.float32)
            transition[junction_crop] = 0.0
        else:
            transition = np.ones(route_crop.shape, np.float32)
        transition_factor[:] = transition

        valid_lane = np.isfinite(lane_error) & permitted_crop
        normalized = _huber_plateau(
            lane_error, tolerance=float(self.policy["lane_target_tolerance_m"]),
            delta=float(self.policy["lane_huber_delta_m"]), scale=float(self.policy["lane_error_scale_m"]),
        )
        stability_weight = np.clip(stability_grid, float(self.policy["direction_instability_floor"]), 1.0)
        lane_weight = transition * stability_weight
        lateral_scale = 1.0 if relaxation_level == "R0" else float(self.policy["r1_lateral_weight_scale"])
        if relaxation_level in {"R2", "R3", "R4"}:
            lane_mask_crop = label_crop > 0
            clearance = ndimage.distance_transform_edt(lane_mask_crop, sampling=float(self.hospital_map.resolution))
            narrow = 2.0 * clearance < float(self.policy["narrow_channel_width_m"])
            lane_weight[narrow | (transition < 1.0)] = 0.0
            if goal is not None:
                rows, cols = np.indices(route_crop.shape)
                wx = self.hospital_map.origin[0] + (cols + col0 + 0.5) * self.hospital_map.resolution
                wy = self.hospital_map.origin[1] + (shape[0] - (rows + row0) - 0.5) * self.hospital_map.resolution
                lane_weight[np.hypot(wx - float(goal[0]), wy - float(goal[1])) <= float(self.policy["endpoint_relax_radius_m"])] = 0.0
        lane_values = np.clip(
            float(self.policy["lane_cost_cap"]) * lateral_scale
            * np.nan_to_num(normalized, nan=0.0) * lane_weight,
            0.0, float(self.policy["lane_cost_cap"]),
        ).astype(np.uint8)
        if planning_preference_enabled:
            view = cost[target]
            view[valid_lane] = lane_values[valid_lane]
            view = active[target]
            view[valid_lane] = lane_weight[valid_lane] > 0.0

        parking = np.asarray(self.raster.masks.get("parking_area", np.zeros(shape, bool)), bool)
        parking_crop = parking[target] & permitted_crop
        if np.any(parking_crop):
            base_deviation = self._parking_base[target]
            taper = np.ones(route_crop.shape, np.float32)
            if goal is not None:
                rows, cols = np.indices(route_crop.shape)
                wx = self.hospital_map.origin[0] + (cols + col0 + 0.5) * self.hospital_map.resolution
                wy = self.hospital_map.origin[1] + (shape[0] - (rows + row0) - 0.5) * self.hospital_map.resolution
                taper = np.clip(
                    np.hypot(wx - float(goal[0]), wy - float(goal[1]))
                    / float(self.policy["parking_endpoint_taper_distance_m"]), 0.0, 1.0,
                ).astype(np.float32)
            parking_deviation[parking_crop] = base_deviation[parking_crop]
            parking_values = np.clip(
                float(self.policy["parking_cost_cap"]) * lateral_scale
                * np.nan_to_num(base_deviation, nan=0.0) * taper,
                0.0, float(self.policy["parking_cost_cap"]),
            ).astype(np.uint8)
            if planning_preference_enabled:
                view = cost[target]
                view[parking_crop] = parking_values[parking_crop]
                view = active[target]
                view[parking_crop] = taper[parking_crop] > 0.0

        cost[junction & permitted] = 0
        active[junction] = False
        nonzero = cost[cost > 0]
        histogram_edges = [1, 9, 17, 33, 49, 65, 81, 129, 201, 254]
        histogram_counts = np.histogram(nonzero, bins=histogram_edges)[0].tolist() if nonzero.size else [0] * (len(histogram_edges) - 1)
        diagnostics = {
            **dict(route_diagnostics or {}),
            "geometry_revision": "r1_boundary_pair",
            "correct_side_definition": "distance_right_le_distance_left",
            "lane_target_tolerance_m": float(self.policy["lane_target_tolerance_m"]),
            "lane_cost_cap": int(self.policy["lane_cost_cap"]),
            "parking_cost_cap": int(self.policy["parking_cost_cap"]),
            "planning_preference_enabled": bool(planning_preference_enabled),
            "route_direction_source": "query_oriented_selected_l1_route_tangent",
            "lane_segment_direction_stability": lane_segment_stats,
            "lane_preference_cell_count": int(np.count_nonzero(np.isfinite(lane_distance))),
            "parking_preference_cell_count": int(np.count_nonzero(np.isfinite(parking_deviation))),
            "active_lateral_cell_count": int(np.count_nonzero(active)),
            "junction_neutral_cell_count": int(np.count_nonzero(junction & permitted)),
            "soft_cost_histogram_edges": histogram_edges,
            "soft_cost_histogram_counts": histogram_counts,
            "soft_cost_saturation_ratio": float(np.mean(nonzero >= max(int(self.policy["lane_cost_cap"]), int(self.policy["parking_cost_cap"])))) if nonzero.size else 0.0,
            "soft_cost_effective_area_m2": float(np.count_nonzero(nonzero) * self.hospital_map.resolution ** 2),
            "field_crop_bbox": [int(row0), int(row1), int(col0), int(col1)],
            "field_crop_cells": int((row1 - row0) * (col1 - col0)),
            "field_build_ms": (time.monotonic_ns() - started) / 1.0e6,
            "parking_static_precompute_ms": float(self._static_precompute_ms),
            "relaxation_level": relaxation_level,
            "lateral_weight_scale": lateral_scale,
            "polygon_vertex_order_used_for_direction": False,
        }
        bounds = (row0, row1, col0, col1)
        return PreferenceField(
            cost=cost,
            lane_distance_to_right_m=CroppedGrid(shape, bounds, lane_distance, np.nan),
            lane_error_m=CroppedGrid(shape, bounds, lane_error, np.nan),
            lane_correct_side=CroppedGrid(shape, bounds, lane_correct, False),
            parking_normalized_deviation=CroppedGrid(shape, bounds, parking_deviation, np.nan),
            junction_transition_factor=CroppedGrid(shape, bounds, transition_factor, 1.0),
            direction_stability=CroppedGrid(shape, bounds, stability_grid, 0.0),
            active_lateral_mask=active,
            relaxation_level=relaxation_level,
            policy_hash=self.policy_hash,
            route_hash=route_hash,
            diagnostics=diagnostics,
            lane_distance_to_left_m=CroppedGrid(shape, bounds, lane_left, np.nan),
            route_tangent_x=CroppedGrid(shape, bounds, tangent_x_grid, 0.0),
            route_tangent_y=CroppedGrid(shape, bounds, tangent_y_grid, 0.0),
            lane_instance_id=CroppedGrid(shape, bounds, lane_instance_grid, 0),
        )


__all__ = [
    "CroppedGrid", "DEFAULT_R1_POLICY", "RegionalPreferenceBuilderR1", "orient_route_for_query",
]
