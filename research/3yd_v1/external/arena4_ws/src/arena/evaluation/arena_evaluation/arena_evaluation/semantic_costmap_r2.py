"""Pinned-Nav2 effective cost composition and bounded caches for 2A-V2 r2."""

from __future__ import annotations

import hashlib
import time
from collections import OrderedDict
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np

from . import _nav2_effective_costmap
from .semantic_costmap_composer import (
    LETHAL_OBSTACLE,
    NO_INFORMATION,
    SemanticCostmap,
    SemanticCostmapComposer,
    internal_soft_to_occupancy,
    occupancy_to_static_layer,
)
from .semantic_map import canonical_hash


def pinned_nav2_effective_master(
    static_cost: np.ndarray, *, resolution: float, inflation_radius_m: float,
    cost_scaling_factor: float, inscribed_radius_m: float,
) -> np.ndarray:
    """Run the pinned Humble InflationLayer algorithm in server row order.

    The old SciPy EDT model is mathematically nearest-obstacle exact, while the
    pinned implementation's source propagation and ``seen_`` ordering leaves a
    small, deterministic set of different cells.  Nav2 traverses its costmap
    bottom row first; HospitalMap arrays are top row first, hence both flips.
    """
    source = np.asarray(static_cost, dtype=np.uint8)
    if source.ndim != 2:
        raise ValueError("static_cost must be a 2-D grid")
    server_order = np.ascontiguousarray(np.flipud(source), dtype=np.uint8)
    payload = _nav2_effective_costmap.inflate(
        server_order,
        int(server_order.shape[1]), int(server_order.shape[0]),
        float(resolution), float(inflation_radius_m),
        float(cost_scaling_factor), float(inscribed_radius_m),
    )
    effective_server = np.frombuffer(payload, dtype=np.uint8).reshape(server_order.shape)
    return np.ascontiguousarray(np.flipud(effective_server), dtype=np.uint8)


class SemanticCostmapComposerR2(SemanticCostmapComposer):
    """ROI arithmetic plus a two-entry immutable inflation-template LRU."""

    def __init__(
        self, *, policy: Optional[Mapping[str, Any]] = None,
        inflation_cache_capacity: int = 2,
    ) -> None:
        super().__init__(policy=policy)
        self.inflation_cache_capacity = max(1, min(2, int(inflation_cache_capacity)))
        self._r2_base_key = ""
        self._r2_base: Optional[np.ndarray] = None
        self._r2_class_key = ""
        self._r2_class_cost: Optional[np.ndarray] = None
        self._r2_inflation_cache: "OrderedDict[str, np.ndarray]" = OrderedDict()

    @staticmethod
    def _array_hash(value: np.ndarray) -> str:
        return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()

    @staticmethod
    def _active_bounds(mask: np.ndarray) -> Tuple[int, int, int, int]:
        cells = np.argwhere(np.asarray(mask, dtype=bool))
        if not cells.size:
            return (0, 0, 0, 0)
        row0, col0 = cells.min(axis=0)
        row1, col1 = cells.max(axis=0) + 1
        return int(row0), int(row1), int(col0), int(col1)

    def _cached_base(self, occupancy: np.ndarray) -> Tuple[np.ndarray, bool]:
        key = self._array_hash(np.asarray(occupancy, dtype=np.int8))
        hit = key == self._r2_base_key and self._r2_base is not None
        if not hit:
            self._r2_base = self._base_internal(occupancy)
            self._r2_base_key = key
        return self._r2_base, hit

    def _cached_class_cost(self, raster: Any) -> Tuple[np.ndarray, bool]:
        key = canonical_hash({
            "raster": raster.raster_hash,
            "class_soft_cost": self.policy["class_soft_cost"],
            "format": "r2-uint16-v1",
        })
        hit = key == self._r2_class_key and self._r2_class_cost is not None
        if not hit:
            result = np.zeros((raster.height, raster.width), dtype=np.uint16)
            for semantic_class, value in self.policy["class_soft_cost"].items():
                feature_mask = raster.masks.get(semantic_class)
                if feature_mask is not None and int(value) > 0:
                    result[np.asarray(feature_mask, dtype=bool)] += np.uint16(int(value))
            self._r2_class_cost = result
            self._r2_class_key = key
        return self._r2_class_cost, hit

    def _inflation_template(
        self, lethal: np.ndarray, *, resolution: float,
    ) -> Tuple[np.ndarray, bool, float, int]:
        inflation = self.policy["inflation"]
        packed = np.packbits(np.asarray(lethal, dtype=np.uint8), axis=None)
        key = canonical_hash({
            "lethal_hash": hashlib.sha256(packed.tobytes()).hexdigest(),
            "shape": list(lethal.shape), "resolution": float(resolution),
            "inflation": inflation, "algorithm": "pinned-humble-propagation-v1",
        })
        cached = self._r2_inflation_cache.pop(key, None)
        if cached is not None:
            self._r2_inflation_cache[key] = cached
            return cached, True, 0.0, 0
        started = time.monotonic_ns()
        source = np.zeros(lethal.shape, dtype=np.uint8)
        source[np.asarray(lethal, dtype=bool)] = LETHAL_OBSTACLE
        template = pinned_nav2_effective_master(
            source, resolution=float(resolution),
            inflation_radius_m=float(inflation["inflation_radius_m"]),
            cost_scaling_factor=float(inflation["cost_scaling_factor"]),
            inscribed_radius_m=float(inflation["inscribed_radius_m"]),
        )
        build_ms = (time.monotonic_ns() - started) / 1.0e6
        self._r2_inflation_cache[key] = template
        evicted = 0
        while len(self._r2_inflation_cache) > self.inflation_cache_capacity:
            self._r2_inflation_cache.popitem(last=False)
            evicted += 1
        return template, False, build_ms, evicted

    @property
    def resident_cache_bytes(self) -> int:
        values = list(self._r2_inflation_cache.values())
        return int(
            (self._r2_base.nbytes if self._r2_base is not None else 0)
            + (self._r2_class_cost.nbytes if self._r2_class_cost is not None else 0)
            + sum(value.nbytes for value in values)
        )

    def compose(
        self, base_occupancy: np.ndarray, raster: Any,
        preference_field: Any = None, *, allowed_mask: Optional[np.ndarray] = None,
        semantics_enabled: Optional[bool] = None,
        hard_semantics_enabled: Optional[bool] = None,
        soft_class_costs_enabled: Optional[bool] = None,
        regional_preference_enabled: Optional[bool] = None,
        hard_semantics_use_footprint: bool = False,
    ) -> SemanticCostmap:
        started = time.monotonic_ns()
        legacy_enabled = True if semantics_enabled is None else bool(semantics_enabled)
        hard_enabled = legacy_enabled if hard_semantics_enabled is None else bool(hard_semantics_enabled)
        class_enabled = legacy_enabled if soft_class_costs_enabled is None else bool(soft_class_costs_enabled)
        regional_enabled = legacy_enabled if regional_preference_enabled is None else bool(regional_preference_enabled)
        base, base_hit = self._cached_base(np.asarray(base_occupancy))
        shape = base.shape
        if shape != (int(raster.height), int(raster.width)):
            raise ValueError("semantic/base costmap shape mismatch")
        allowed = np.ones(shape, dtype=bool) if allowed_mask is None else np.asarray(allowed_mask, dtype=bool)
        if allowed.shape != shape:
            raise ValueError("allowed mask shape mismatch")
        bounds = self._active_bounds(allowed)
        row0, row1, col0, col1 = bounds
        target = np.s_[row0:row1, col0:col1]
        soft = np.zeros(shape, dtype=np.uint8)
        class_cost, class_hit = self._cached_class_cost(raster)
        if row0 < row1 and col0 < col1:
            work = np.zeros((row1 - row0, col1 - col0), dtype=np.float32)
            if class_enabled:
                work += float(self.policy["w_class"]) * class_cost[target]
            if regional_enabled and preference_field is not None:
                lateral = np.asarray(preference_field.cost[target], dtype=np.float32)
                work += float(self.policy["w_lateral"]) * lateral
                if hasattr(preference_field, "junction_transition_factor"):
                    transition = np.asarray(
                        preference_field.junction_transition_factor[target], dtype=np.float32,
                    )
                    work += (
                        float(self.policy["w_transition"]) * lateral
                        * np.clip(1.0 - transition, 0.0, 1.0)
                    )
            crop_allowed = allowed[target]
            target_soft = soft[target]
            target_soft[crop_allowed] = np.clip(
                work[crop_allowed], 0.0, float(self.policy["soft_cost_max"]),
            ).astype(np.uint8)

        internal = base.copy()
        known = internal != NO_INFORMATION
        internal[known] = np.maximum(internal[known], soft[known])
        hard_source = raster.hard_footprint_mask if hard_semantics_use_footprint else raster.hard_mask
        hard = np.asarray(hard_source, dtype=bool) & allowed if hard_enabled else np.zeros(shape, dtype=bool)
        internal[hard] = LETHAL_OBSTACLE
        internal[~allowed] = LETHAL_OBSTACLE
        occupancy = np.zeros(shape, dtype=np.int8)
        occupancy[internal == NO_INFORMATION] = -1
        occupancy[internal == LETHAL_OBSTACLE] = 100
        soft_cells = (internal > 0) & (internal < LETHAL_OBSTACLE)
        occupancy[soft_cells] = internal_soft_to_occupancy(internal[soft_cells]).astype(np.int8)
        static = occupancy_to_static_layer(
            occupancy, lethal_threshold=int(self.policy["static_layer"]["lethal_cost_threshold"]),
        )
        template, inflation_hit, inflation_ms, inflation_evictions = self._inflation_template(
            static == LETHAL_OBSTACLE, resolution=float(raster.resolution),
        )
        expected_master = static.copy()
        known = expected_master != NO_INFORMATION
        expected_master[known] = np.maximum(expected_master[known], template[known])
        unknown_inscribed = (~known) & (template >= 253)
        expected_master[unknown_inscribed] = template[unknown_inscribed]
        affected = hard | (soft > 0)
        nonzero = soft[soft > 0]
        return SemanticCostmap(
            internal_cost=internal, occupancy_grid=occupancy,
            expected_master_cost=expected_master, soft_cost=soft,
            hard_semantic_mask=hard, affected_mask=affected,
            policy_hash=self.policy_hash, semantic_map_hash=raster.semantic_map_hash,
            preference_field_hash=str(getattr(preference_field, "field_hash", "")),
            semantics_enabled=bool(hard_enabled or class_enabled or regional_enabled),
            diagnostics={
                "composition": "r2_roi_max_preserves_static_obstacles",
                "effective_master_mapping": "pinned_humble_propagation_exact_v1",
                "static_obstacle_cells_before": int(np.count_nonzero(base == LETHAL_OBSTACLE)),
                "static_obstacle_cells_after": int(np.count_nonzero(internal == LETHAL_OBSTACLE)),
                "final_lethal_cells_including_roi_boundary": int(np.count_nonzero(internal == LETHAL_OBSTACLE)),
                "hard_semantic_cells": int(np.count_nonzero(hard)),
                "soft_semantic_cells": int(np.count_nonzero(soft)),
                "soft_internal_max": int(np.max(nonzero)) if nonzero.size else 0,
                "soft_occupancy_max": int(np.max(occupancy[(occupancy > 0) & (occupancy < 100)]))
                if np.any((occupancy > 0) & (occupancy < 100)) else 0,
                "trinary_costmap": False, "occupancy_100_reserved_for_lethal": True,
                "hard_semantics_enabled": hard_enabled,
                "soft_class_costs_enabled": class_enabled,
                "regional_preference_enabled": regional_enabled,
                "hard_semantics_use_footprint": bool(hard_semantics_use_footprint),
                "base_geometry_cache_hit": base_hit,
                "class_cost_cache_hit": class_hit,
                "inflation_template_cache_hit": inflation_hit,
                "inflation_template_build_ms": float(inflation_ms),
                "inflation_cache_entries": len(self._r2_inflation_cache),
                "inflation_cache_capacity": self.inflation_cache_capacity,
                "inflation_cache_evictions": inflation_evictions,
                "composer_cache_resident_bytes": self.resident_cache_bytes,
                "compose_active_bbox": list(bounds),
                "compose_active_cells": int(np.count_nonzero(allowed)),
                "compose_wall_ms": (time.monotonic_ns() - started) / 1.0e6,
                "affected_roi_only": True,
            },
        )


__all__ = ["SemanticCostmapComposerR2", "pinned_nav2_effective_master"]
