"""Compact request-bound route-phase input for 2A-V3 r14.

The frozen r13 implementation materializes every diagnostic grid at full map
size and then copies all of them into a cropped RoutePhaseWorld.  r14 keeps the
same values and hashes but materializes query-dependent search grids only in
the active crop.  Full grids remain only where the online exact-ACK and the
canonical audit require them.
"""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any, Mapping
import math
import time

import cv2
import numpy as np

from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v3_semantic_r13_benchmark as r13
from .regional_preference_r1 import CroppedGrid, expand_roi_to_route_lane_instances
from .semantic_constraint_core import CropMap
from .semantic_map import canonical_hash
from .semantic_rasterizer import grid_hash
from .semantic_route_phase_v3 import RoutePhaseWorld


def _crop(value: Any, bounds: tuple[int, int, int, int], dtype: Any) -> np.ndarray:
    row0, row1, col0, col1 = bounds
    if isinstance(value, CroppedGrid):
        result = value[row0:row1, col0:col1]
    else:
        result = np.asarray(value)[row0:row1, col0:col1]
    return np.ascontiguousarray(result, dtype=dtype)


def _bounds(mask: np.ndarray, margin: int = 20) -> tuple[int, int, int, int]:
    rows, cols = np.where(np.asarray(mask, dtype=bool))
    if not len(rows):
        raise ValueError("empty route-phase corridor")
    return (
        max(0, int(rows.min())-margin), min(mask.shape[0], int(rows.max())+margin+1),
        max(0, int(cols.min())-margin), min(mask.shape[1], int(cols.max())+margin+1),
    )


def prepare_query_input_compact(
    *, query: Any, route: Any, orientation: Mapping[str, Any], ctx: Any,
    semantic_map: Any, raster: Any, topology: Any, builder: Any,
    composer: Any, parent: Mapping[str, Any], query_set_hash: str,
    maximum_lateral_probe_m: float,
) -> tuple[dict[str, Any], dict[str, np.ndarray], Any, np.ndarray, np.ndarray]:
    """Return hash-equivalent r13 metadata plus only cropped search arrays."""
    roi_started = time.monotonic_ns()
    base_allowed = r1.r2_runtime._raw_corridor_mask(
        ctx, topology, route, query, float(parent["roi"]["r0_padding_m"]),
    )
    free = r1.r2_runtime._raw_free_mask(ctx)
    expanded, roi_diagnostics = expand_roi_to_route_lane_instances(
        ctx.hospital_map, raster, semantic_map, route.polyline, base_allowed,
        free_mask=free,
        route_probe_radius_m=float(parent["roi"].get("lane_route_probe_radius_m", .50)),
    )
    # The explicit route-phase state generator cannot sample farther from the
    # selected route than maximum_lateral_probe_m.  Restrict query-directional
    # fields to that reachable support (plus endpoint/primitive margin) while
    # retaining the complete route-lane publication ROI below.
    search_tube_half_width_m = float(maximum_lateral_probe_m) + 0.75
    search_tube = r1.r2_runtime._raw_corridor_mask(
        ctx, topology, route, query, search_tube_half_width_m,
    )
    field_allowed = np.asarray(expanded, dtype=bool) & search_tube
    roi_ms = (time.monotonic_ns()-roi_started)/1.0e6
    field_started = time.monotonic_ns()
    preference = builder.build(
        route.polyline, goal=query.goal, allowed_mask=field_allowed,
        relaxation_level="R0", planning_preference_enabled=False,
        route_diagnostics=orientation,
    )
    field_ms = (time.monotonic_ns()-field_started)/1.0e6

    parking_mask = np.asarray(
        raster.masks.get("parking_area", np.zeros_like(ctx.hospital_map.occupancy, bool)),
        dtype=bool,
    )
    parking_components = getattr(builder, "_r14_parking_components", None)
    if parking_components is None:
        count, values = cv2.connectedComponents(parking_mask.astype(np.uint8), connectivity=8)
        if count > np.iinfo(np.int16).max:
            raise ValueError("parking component count exceeds compact int16 schema")
        parking_components = values.astype(np.int16)
        setattr(builder, "_r14_parking_components", parking_components)
    lane_mask = np.asarray(raster.masks.get("lane", np.zeros_like(parking_mask)), dtype=bool)
    junction = np.asarray(
        raster.masks.get("junction_area", np.zeros_like(parking_mask)), dtype=bool,
    )
    inverse = {semantic_id: int(label) for label, semantic_id in builder._lane_instance_ids.items()}
    selected_lane_ids = list(roi_diagnostics.get("selected_lane_instance_ids", []))
    selected_lanes = sorted(inverse[value] for value in selected_lane_ids if value in inverse)
    selected_parking = r13._route_parking_components(
        ctx.hospital_map, parking_components, route.polyline,
    )
    labels = np.asarray(builder._lane_labels, dtype=np.int32)
    lane_allowed = np.isin(labels, selected_lanes)
    parking_allowed = np.isin(parking_components, selected_parking)
    neutral = base_allowed & (junction | ~(lane_mask | parking_mask))
    phase_support = (lane_allowed | parking_allowed | neutral) & free & search_tube
    publication_allowed = np.asarray(expanded, dtype=bool) & free
    allowed = np.asarray(phase_support, dtype=bool)
    for pose in (query.start, query.goal):
        cell = ctx.hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
        if cell is not None and free[cell]:
            allowed[cell] = True
    if not selected_lanes and not selected_parking:
        raise RuntimeError(f"{query.query_id}: route has no bound lane or parking semantic instance")

    compose_started = time.monotonic_ns()
    composition = composer.compose(
        ctx.hospital_map.occupancy, raster, preference,
        allowed_mask=publication_allowed,
        hard_semantics_enabled=True, soft_class_costs_enabled=True,
        regional_preference_enabled=False, hard_semantics_use_footprint=True,
    )
    compose_ms = (time.monotonic_ns()-compose_started)/1.0e6
    bounds = _bounds(allowed)
    arrays = {
        "master": _crop(composition.expected_master_cost, bounds, np.uint8),
        "occupancy": _crop(ctx.hospital_map.occupancy, bounds, np.int8),
        "allowed": _crop(allowed, bounds, bool),
        "base_allowed": _crop(base_allowed, bounds, bool),
        "labels": _crop(labels, bounds, np.int32),
        "lane_mask": _crop(lane_mask, bounds, bool),
        "error": _crop(preference.lane_error_m, bounds, np.float32),
        "correct": _crop(preference.lane_correct_side, bounds, bool),
        "right": _crop(preference.lane_distance_to_right_m, bounds, np.float32),
        "left": _crop(preference.lane_distance_to_left_m, bounds, np.float32),
        "parking_components": _crop(parking_components, bounds, np.int16),
        "parking_deviation": _crop(preference.parking_normalized_deviation, bounds, np.float32),
        "junction": _crop(junction, bounds, bool),
        "hard": _crop(raster.hard_footprint_mask, bounds, bool),
        "no_stopping": _crop(raster.no_stopping_mask, bounds, bool),
    }
    metadata = {
        "architecture_id": "2A-V3",
        "implementation_revision": "r14-parking-dispatch-ack-cache",
        "query": query.as_dict(),
        "query_set_content_hash": str(query_set_hash),
        "map_hash": ctx.map_sha256,
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "expected_master_hash": composition.expected_master_hash,
        "route_hash": r1._path_hash(route),
        "route_polyline_hash": canonical_hash(route.polyline),
        "route_polyline": route.polyline,
        "route_orientation": dict(orientation),
        "roi_hash": grid_hash(publication_allowed),
        "search_roi_hash": grid_hash(allowed),
        "phase_support_hash": grid_hash(phase_support),
        "base_roi_hash": grid_hash(base_allowed),
        "selected_lane_labels": selected_lanes,
        "selected_lane_semantic_ids": selected_lane_ids,
        "selected_parking_components": selected_parking,
        "roi_diagnostics": roi_diagnostics,
        "preference_diagnostics": preference.diagnostics,
        "compact_crop_bounds": list(bounds),
        "compact_crop_bytes": int(sum(value.nbytes for value in arrays.values())),
        "search_route_tube_half_width_m": search_tube_half_width_m,
        "full_search_array_materialization": False,
        "map": {
            "resolution": ctx.hospital_map.resolution,
            "origin": ctx.hospital_map.origin,
            "width": ctx.hospital_map.width,
            "height": ctx.hospital_map.height,
            "image_path": str(ctx.hospital_map.image_path),
        },
        "timing": {"roi_build_ms": roi_ms, "field_build_ms": field_ms, "compose_ms": compose_ms},
        "npz_sha256": "",
    }
    return metadata, arrays, composition, publication_allowed, allowed


class CompactRoutePhaseWorldR14(RoutePhaseWorld):
    """RoutePhaseWorld with pre-cropped, request-owned grids."""

    def __init__(
        self, input_dir: Path, query_id: str, *, arrays_override: Mapping[str, np.ndarray],
        meta_override: Mapping[str, Any], full_occupancy: np.ndarray,
        full_allowed: np.ndarray, master_override: np.ndarray | None = None,
        expected_master_hash: str | None = None,
    ) -> None:
        del input_dir, query_id
        self.meta = dict(meta_override)
        desc = self.meta["map"]
        full_shape = (int(desc["height"]), int(desc["width"]))
        bounds = tuple(int(value) for value in self.meta["compact_crop_bounds"])
        row0, row1, col0, col1 = bounds
        crop_shape = (row1-row0, col1-col0)
        self.grids = {}
        for name in self.ARRAY_NAMES:
            value = np.asarray(arrays_override[name])
            if value.shape != crop_shape:
                raise ValueError(f"compact route-phase array shape mismatch: {name}")
            self.grids[name] = value
        if master_override is not None:
            effective = np.asarray(master_override, dtype=np.uint8)
            actual = grid_hash(effective)
            expected = str(expected_master_hash or self.meta["expected_master_hash"])
            if effective.shape != full_shape or actual != expected or actual != self.meta["expected_master_hash"]:
                raise ValueError("verified effective-master override hash mismatch")
            self.grids["master"] = np.ascontiguousarray(effective[row0:row1, col0:col1])
        self._full_occupancy = np.asarray(full_occupancy)
        self._full_allowed = np.asarray(full_allowed, dtype=bool)
        self.master = self.grids["master"]
        self.obstacle = self.master >= 254
        distance = cv2.distanceTransform(
            (~self.obstacle).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE,
        ) * float(desc["resolution"])
        image = Path(desc["image_path"])
        self.map = CropMap(
            image.with_suffix(".yaml"), image, float(desc["resolution"]),
            tuple(desc["origin"]), crop_shape[1], crop_shape[0],
            self.grids["occupancy"], distance,
        )
        self.map.full_origin, self.map.full_height = tuple(desc["origin"]), full_shape[0]
        self.map.row0, self.map.col0 = row0, col0
        self.query = SimpleNamespace(**self.meta["query"])
        self.start, self.goal = tuple(self.query.start), tuple(self.query.goal)
        self.selected_lanes = tuple(sorted(map(int, self.meta["selected_lane_labels"])))
        self.selected_parking = tuple(sorted(map(int, self.meta["selected_parking_components"])))
        self.safe_threshold = math.hypot(.265, .225) + math.sqrt(2) * self.map.resolution
        self.pose_checks = self.exact_checks = 0
        self._canonical_auditor = None


__all__ = ["CompactRoutePhaseWorldR14", "prepare_query_input_compact"]
