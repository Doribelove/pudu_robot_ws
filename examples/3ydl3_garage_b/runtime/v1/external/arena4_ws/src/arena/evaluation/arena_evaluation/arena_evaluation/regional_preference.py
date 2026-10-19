"""Query-conditioned keep-right and keep-center fields for 2A-V2 L3."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy import ndimage

from .semantic_map import canonical_hash
from .semantic_rasterizer import RasterizedSemantics, grid_hash


DEFAULT_PREFERENCE_POLICY: Dict[str, Any] = {
    # Distance is from robot base center to the right lane boundary.  It is
    # not a body-edge clearance; the complete footprint remains a hard check.
    "lane_base_center_to_right_boundary_m": 0.40,
    "lane_error_scale_m": 1.00,
    "junction_transition_distance_m": 0.75,
    "parking_endpoint_taper_distance_m": 1.00,
    "endpoint_relax_radius_m": 0.75,
    "narrow_channel_width_m": 1.10,
    "direction_instability_floor": 0.20,
    "r1_lateral_weight_scale": 0.35,
    "max_right_boundary_probe_m": 12.0,
}


@dataclass
class PreferenceField:
    cost: np.ndarray
    lane_distance_to_right_m: np.ndarray
    lane_error_m: np.ndarray
    lane_correct_side: np.ndarray
    parking_normalized_deviation: np.ndarray
    junction_transition_factor: np.ndarray
    direction_stability: np.ndarray
    active_lateral_mask: np.ndarray
    relaxation_level: str
    policy_hash: str
    route_hash: str
    diagnostics: Dict[str, Any] = field(default_factory=dict)
    # r1 audit-only geometry.  Optional fields keep the r0 constructor and
    # persisted run12 behavior intact while allowing control arms to measure
    # both physical lane boundaries without enabling planning preference.
    lane_distance_to_left_m: Optional[np.ndarray] = None
    route_tangent_x: Optional[np.ndarray] = None
    route_tangent_y: Optional[np.ndarray] = None
    lane_instance_id: Optional[np.ndarray] = None

    @property
    def field_hash(self) -> str:
        return canonical_hash({
            "cost": grid_hash(self.cost),
            "active": grid_hash(self.active_lateral_mask),
            "relaxation_level": self.relaxation_level,
            "policy_hash": self.policy_hash,
            "route_hash": self.route_hash,
        })


def _route_raster(
    hospital_map: Any, route: Sequence[Sequence[float]], shape: Tuple[int, int],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    marker = np.zeros(shape, dtype=bool)
    tangent_x = np.zeros(shape, dtype=np.float32)
    tangent_y = np.zeros(shape, dtype=np.float32)
    counts = np.zeros(shape, dtype=np.uint16)
    for first, second in zip(route, route[1:]):
        dx, dy = float(second[0]) - float(first[0]), float(second[1]) - float(first[1])
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        tx, ty = dx / length, dy / length
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
        marker[rows, cols] = True
        tangent_x[rows, cols] += tx
        tangent_y[rows, cols] += ty
        counts[rows, cols] += 1
    valid = counts > 0
    tangent_x[valid] /= counts[valid]
    tangent_y[valid] /= counts[valid]
    norm = np.hypot(tangent_x, tangent_y)
    normalized = norm > 1e-6
    tangent_x[normalized] /= norm[normalized]
    tangent_y[normalized] /= norm[normalized]
    stability = np.zeros(shape, dtype=np.float32)
    stability[valid] = np.clip(norm[valid], 0.0, 1.0)
    return marker, tangent_x, tangent_y, stability


def _bounds(mask: np.ndarray, margin_cells: int) -> Tuple[int, int, int, int]:
    cells = np.argwhere(mask)
    if cells.size == 0:
        return 0, mask.shape[0], 0, mask.shape[1]
    return (
        max(0, int(cells[:, 0].min()) - margin_cells),
        min(mask.shape[0], int(cells[:, 0].max()) + margin_cells + 1),
        max(0, int(cells[:, 1].min()) - margin_cells),
        min(mask.shape[1], int(cells[:, 1].max()) + margin_cells + 1),
    )


class RegionalPreferenceBuilder:
    def __init__(self, hospital_map: Any, raster: RasterizedSemantics, *, policy: Optional[Mapping[str, Any]] = None):
        self.hospital_map = hospital_map
        self.raster = raster
        self.policy = {**DEFAULT_PREFERENCE_POLICY, **dict(policy or {})}
        self.policy_hash = canonical_hash(self.policy)

    def _right_widths(
        self, route_mask: np.ndarray, tangent_x: np.ndarray, tangent_y: np.ndarray,
        lane_mask: np.ndarray,
    ) -> np.ndarray:
        widths = np.full(route_mask.shape, np.nan, dtype=np.float32)
        resolution = float(self.hospital_map.resolution)
        max_steps = int(math.ceil(float(self.policy["max_right_boundary_probe_m"]) / resolution))
        for row, col in np.argwhere(route_mask & lane_mask):
            tx, ty = float(tangent_x[row, col]), float(tangent_y[row, col])
            if math.hypot(tx, ty) < 0.5:
                continue
            # World right normal=(ty,-tx); image dr=-dy/res=tx/res, dc=dx/res=ty/res.
            last_distance = 0.0
            for step in range(1, max_steps + 1):
                probe_row = int(round(float(row) + step * tx))
                probe_col = int(round(float(col) + step * ty))
                if not (0 <= probe_row < lane_mask.shape[0] and 0 <= probe_col < lane_mask.shape[1]):
                    break
                if not lane_mask[probe_row, probe_col]:
                    break
                last_distance = step * resolution
            widths[row, col] = last_distance
        return widths

    def build(
        self, route: Sequence[Sequence[float]], *, goal: Optional[Sequence[float]] = None,
        allowed_mask: Optional[np.ndarray] = None, relaxation_level: str = "R0",
    ) -> PreferenceField:
        if relaxation_level not in {"R0", "R1", "R2", "R3", "R4"}:
            raise ValueError(f"invalid relaxation level: {relaxation_level}")
        shape = (int(self.raster.height), int(self.raster.width))
        route_hash = canonical_hash([[float(point[0]), float(point[1])] for point in route])
        cost = np.zeros(shape, dtype=np.uint8)
        nan_grid = lambda: np.full(shape, np.nan, dtype=np.float32)
        lane_distance = nan_grid()
        lane_error = nan_grid()
        lane_correct = np.zeros(shape, dtype=bool)
        parking_deviation = nan_grid()
        transition_factor = np.ones(shape, dtype=np.float32)
        direction_stability = np.zeros(shape, dtype=np.float32)
        active = np.zeros(shape, dtype=bool)
        if len(route) < 2:
            return PreferenceField(
                cost, lane_distance, lane_error, lane_correct, parking_deviation,
                transition_factor, direction_stability, active, relaxation_level,
                self.policy_hash, route_hash, {"empty_route": True},
            )
        route_mask, route_tx, route_ty, route_stability = _route_raster(
            self.hospital_map, route, shape,
        )
        permitted = np.ones(shape, dtype=bool) if allowed_mask is None else np.asarray(allowed_mask, dtype=bool)
        if permitted.shape != shape:
            raise ValueError("allowed_mask shape mismatch")
        relevant = permitted & (
            self.raster.masks.get("lane", np.zeros(shape, bool))
            | self.raster.masks.get("parking_area", np.zeros(shape, bool))
            | route_mask
        )
        margin = int(math.ceil(float(self.policy["max_right_boundary_probe_m"]) / self.hospital_map.resolution))
        row0, row1, col0, col1 = _bounds(relevant, margin)
        route_crop = route_mask[row0:row1, col0:col1]
        if not np.any(route_crop):
            return PreferenceField(
                cost, lane_distance, lane_error, lane_correct, parking_deviation,
                transition_factor, direction_stability, active, relaxation_level,
                self.policy_hash, route_hash, {"route_not_rasterized": True},
            )
        _, nearest = ndimage.distance_transform_edt(~route_crop, return_indices=True)
        nearest_rows = nearest[0] + row0
        nearest_cols = nearest[1] + col0
        target_slice = np.s_[row0:row1, col0:col1]
        tx = route_tx[nearest_rows, nearest_cols]
        ty = route_ty[nearest_rows, nearest_cols]
        stability = route_stability[nearest_rows, nearest_cols]
        direction_stability[target_slice] = stability
        rows, cols = np.indices(route_crop.shape)
        delta_col_m = (cols + col0 - nearest_cols) * float(self.hospital_map.resolution)
        delta_world_y_m = -(rows + row0 - nearest_rows) * float(self.hospital_map.resolution)
        signed_right_m = delta_col_m * ty + delta_world_y_m * (-tx)
        lane_mask = np.asarray(self.raster.masks.get("lane", np.zeros(shape, bool)), dtype=bool)
        parking = np.asarray(self.raster.masks.get("parking_area", np.zeros(shape, bool)), dtype=bool)
        widths = self._right_widths(route_mask, route_tx, route_ty, lane_mask)
        right_width = widths[nearest_rows, nearest_cols]
        distance_right = right_width - signed_right_m
        lane_crop = (
            lane_mask[target_slice] & ~parking[target_slice]
            & permitted[target_slice] & np.isfinite(right_width)
        )
        desired = float(self.policy["lane_base_center_to_right_boundary_m"])
        error = np.abs(distance_right - desired)
        lane_distance[target_slice][lane_crop] = distance_right[lane_crop].astype(np.float32)
        lane_error[target_slice][lane_crop] = error[lane_crop].astype(np.float32)
        lane_correct[target_slice][lane_crop] = signed_right_m[lane_crop] >= 0.0
        junction = np.asarray(self.raster.masks.get("junction_area", np.zeros(shape, bool)), dtype=bool)
        junction_crop = junction[target_slice]
        if np.any(junction_crop):
            distance_from_junction = ndimage.distance_transform_edt(
                ~junction_crop, sampling=float(self.hospital_map.resolution),
            )
            transition = np.clip(
                distance_from_junction / float(self.policy["junction_transition_distance_m"]),
                0.0, 1.0,
            )
        else:
            transition = np.ones(route_crop.shape, dtype=np.float32)
        transition[junction_crop] = 0.0
        transition_factor[target_slice] = transition.astype(np.float32)
        stability_weight = np.clip(
            stability, float(self.policy["direction_instability_floor"]), 1.0,
        )
        lane_normalized = np.clip(error / float(self.policy["lane_error_scale_m"]), 0.0, 1.0)
        lane_weight = transition * stability_weight
        lateral_scale = 1.0 if relaxation_level == "R0" else float(self.policy["r1_lateral_weight_scale"])
        if relaxation_level in {"R2", "R3", "R4"}:
            lane_clearance = ndimage.distance_transform_edt(
                lane_mask[target_slice], sampling=float(self.hospital_map.resolution),
            )
            narrow = (2.0 * lane_clearance) < float(self.policy["narrow_channel_width_m"])
            lane_weight[narrow | (transition < 1.0)] = 0.0
            if goal is not None:
                gx, gy = float(goal[0]), float(goal[1])
                world_x = self.hospital_map.origin[0] + (cols + col0 + 0.5) * self.hospital_map.resolution
                world_y = self.hospital_map.origin[1] + (shape[0] - (rows + row0) - 0.5) * self.hospital_map.resolution
                endpoint = np.hypot(world_x - gx, world_y - gy) <= float(self.policy["endpoint_relax_radius_m"])
                lane_weight[endpoint] = 0.0
        lane_values = np.clip(252.0 * lateral_scale * lane_normalized * lane_weight, 0.0, 252.0)
        cost[target_slice][lane_crop] = lane_values[lane_crop].astype(np.uint8)
        active[target_slice][lane_crop] = lane_weight[lane_crop] > 0.0

        parking_crop = parking[target_slice] & permitted[target_slice]
        if np.any(parking_crop):
            labels_count, labels = cv2.connectedComponents(parking_crop.astype(np.uint8), connectivity=8)
            deviation = np.zeros(route_crop.shape, dtype=np.float32)
            # An ROI may split a parking polygon into many components.  A
            # full-crop EDT for every label is O(labels * full-map-area) and
            # becomes pathological on the 2138x4020 real map.  Each label's
            # medial distance is local, so compute only inside its bbox.
            component_slices = ndimage.find_objects(labels, max_label=labels_count - 1)
            for label, component_slice in enumerate(component_slices, start=1):
                if component_slice is None:
                    continue
                component = labels[component_slice] == label
                clearance = ndimage.distance_transform_edt(
                    component, sampling=float(self.hospital_map.resolution),
                )
                maximum = float(np.max(clearance[component])) if np.any(component) else 0.0
                if maximum > 1e-9:
                    target = deviation[component_slice]
                    target[component] = 1.0 - clearance[component] / maximum
            taper = np.ones(route_crop.shape, dtype=np.float32)
            if goal is not None:
                gx, gy = float(goal[0]), float(goal[1])
                world_x = self.hospital_map.origin[0] + (cols + col0 + 0.5) * self.hospital_map.resolution
                world_y = self.hospital_map.origin[1] + (shape[0] - (rows + row0) - 0.5) * self.hospital_map.resolution
                endpoint_distance = np.maximum(
                    0.0,
                    np.hypot(world_x - gx, world_y - gy)
                    - math.sqrt(2.0) * float(self.hospital_map.resolution) / 2.0,
                )
                taper = np.clip(
                    endpoint_distance / float(self.policy["parking_endpoint_taper_distance_m"]),
                    0.0, 1.0,
                ).astype(np.float32)
            parking_deviation[target_slice][parking_crop] = deviation[parking_crop]
            parking_values = np.clip(252.0 * lateral_scale * deviation * taper, 0.0, 252.0).astype(np.uint8)
            cost[target_slice][parking_crop] = parking_values[parking_crop]
            active[target_slice][parking_crop] = taper[parking_crop] > 0.0
        # Junction neutralization wins over both lane and parking preferences.
        cost[junction] = 0
        active[junction] = False
        diagnostics = {
            "lane_preference_cell_count": int(np.count_nonzero(lane_crop)),
            "parking_preference_cell_count": int(np.count_nonzero(parking_crop)),
            "active_lateral_cell_count": int(np.count_nonzero(active)),
            "junction_neutral_cell_count": int(np.count_nonzero(junction & permitted)),
            "relaxation_level": relaxation_level,
            "lateral_weight_scale": lateral_scale,
            "route_direction_source": "selected_l1_route_tangent",
            "polygon_vertex_order_used_for_direction": False,
            "right_side_flips_with_route_reversal": True,
            "base_center_distance_definition": True,
        }
        return PreferenceField(
            cost=cost,
            lane_distance_to_right_m=lane_distance,
            lane_error_m=lane_error,
            lane_correct_side=lane_correct,
            parking_normalized_deviation=parking_deviation,
            junction_transition_factor=transition_factor,
            direction_stability=direction_stability,
            active_lateral_mask=active,
            relaxation_level=relaxation_level,
            policy_hash=self.policy_hash,
            route_hash=route_hash,
            diagnostics=diagnostics,
        )


__all__ = [
    "DEFAULT_PREFERENCE_POLICY", "PreferenceField", "RegionalPreferenceBuilder",
]
