"""Deterministic semantic/static cost composition and soft-cost ACK checks."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional

import numpy as np
from scipy import ndimage

from .semantic_map import canonical_hash
from .semantic_rasterizer import RasterizedSemantics, grid_hash


FREE_SPACE = np.uint8(0)
INSCRIBED_INFLATED_OBSTACLE = np.uint8(253)
LETHAL_OBSTACLE = np.uint8(254)
NO_INFORMATION = np.uint8(255)

DEFAULT_COMPOSER_POLICY: Dict[str, Any] = {
    "w_class": 1.0,
    "w_lateral": 0.55,
    "w_transition": 0.10,
    "soft_cost_max": 252,
    "class_soft_cost": {
        "speed_bumps": 120,
        "parking_area": 8,
        "lane": 0,
        "junction_area": 0,
        "fence_area": 0,
        "no_stopping": 0,
    },
    "static_layer": {
        "trinary_costmap": False,
        "lethal_cost_threshold": 100,
        "unknown_cost_value": 255,
    },
    "inflation": {
        "inflation_radius_m": 0.55,
        "cost_scaling_factor": 3.0,
        # Nav2 Costmap2D applies its default 0.01 m footprint padding to the
        # Jackal rectangle, so the effective inscribed radius is 0.225 m.
        "nav2_footprint_padding_m": 0.01,
        "inscribed_radius_m": 0.225,
    },
}


def internal_soft_to_occupancy(value: np.ndarray | int) -> np.ndarray:
    """Map internal 0..252 soft values to OccupancyGrid 0..99."""
    array = np.asarray(value, dtype=np.float64)
    clipped = np.clip(array, 0.0, 252.0)
    # Ceil keeps every positive internal preference observable after the
    # StaticLayer integer conversion; 99 remains non-lethal.
    return np.where(clipped <= 0.0, 0, np.ceil(clipped * 99.0 / 252.0)).astype(np.int16)


def occupancy_to_static_layer(value: np.ndarray, *, lethal_threshold: int = 100) -> np.ndarray:
    """Mirror Nav2 Humble StaticLayer::interpretValue with trinary=false."""
    occupancy = np.asarray(value, dtype=np.int16)
    result = np.zeros(occupancy.shape, dtype=np.uint8)
    result[occupancy < 0] = NO_INFORMATION
    lethal = occupancy >= int(lethal_threshold)
    result[lethal] = LETHAL_OBSTACLE
    soft = (occupancy > 0) & ~lethal
    result[soft] = np.floor(
        occupancy[soft].astype(np.float64) / float(lethal_threshold) * float(LETHAL_OBSTACLE)
    ).astype(np.uint8)
    return result


def nav2_inflated_master(
    static_cost: np.ndarray, *, resolution: float, inflation_radius_m: float,
    cost_scaling_factor: float, inscribed_radius_m: float,
) -> np.ndarray:
    """Reproduce InflationLayer's max-combination for a full static grid."""
    static = np.asarray(static_cost, dtype=np.uint8)
    obstacles = static == LETHAL_OBSTACLE
    result = static.copy()
    if not np.any(obstacles):
        return result
    distance_cells = ndimage.distance_transform_edt(~obstacles)
    distance_m = distance_cells * float(resolution)
    inflation = np.zeros(static.shape, dtype=np.uint8)
    inflation[obstacles] = LETHAL_OBSTACLE
    inscribed = (~obstacles) & (distance_m <= float(inscribed_radius_m))
    inflation[inscribed] = INSCRIBED_INFLATED_OBSTACLE
    decayed = (
        (~obstacles) & ~inscribed & (distance_m <= float(inflation_radius_m))
    )
    factors = np.exp(
        -float(cost_scaling_factor) * (distance_m[decayed] - float(inscribed_radius_m))
    )
    inflation[decayed] = np.floor(252.0 * factors).astype(np.uint8)
    known = result != NO_INFORMATION
    result[known] = np.maximum(result[known], inflation[known])
    return result


@dataclass
class SemanticCostmap:
    internal_cost: np.ndarray
    occupancy_grid: np.ndarray
    expected_master_cost: np.ndarray
    soft_cost: np.ndarray
    hard_semantic_mask: np.ndarray
    affected_mask: np.ndarray
    policy_hash: str
    semantic_map_hash: str
    preference_field_hash: str
    semantics_enabled: bool
    diagnostics: Dict[str, Any] = field(default_factory=dict)

    @property
    def expected_grid_hash(self) -> str:
        return grid_hash(self.occupancy_grid)

    @property
    def expected_master_hash(self) -> str:
        return grid_hash(self.expected_master_cost)


class SemanticCostmapComposer:
    def __init__(self, *, policy: Optional[Mapping[str, Any]] = None):
        supplied = dict(policy or {})
        self.policy = {
            **DEFAULT_COMPOSER_POLICY,
            **supplied,
            "class_soft_cost": {
                **DEFAULT_COMPOSER_POLICY["class_soft_cost"],
                **dict(supplied.get("class_soft_cost") or {}),
            },
            "static_layer": {
                **DEFAULT_COMPOSER_POLICY["static_layer"],
                **dict(supplied.get("static_layer") or {}),
            },
            "inflation": {
                **DEFAULT_COMPOSER_POLICY["inflation"],
                **dict(supplied.get("inflation") or {}),
            },
        }
        if self.policy["static_layer"].get("trinary_costmap") is not False:
            raise ValueError("2A-V2 soft semantics require trinary_costmap=false")
        if int(self.policy["soft_cost_max"]) >= int(LETHAL_OBSTACLE):
            raise ValueError("soft_cost_max must remain below lethal")
        self.policy_hash = canonical_hash(self.policy)
        self._class_cost_cache: Dict[str, np.ndarray] = {}

    @staticmethod
    def _base_internal(occupancy: np.ndarray) -> np.ndarray:
        # The base image is already expressed as OccupancyGrid values.  Use
        # the same non-trinary conversion as Nav2 StaticLayer; converting it
        # through ``internal_soft_to_occupancy`` would incorrectly compress
        # an input 1..99 a second time.
        return occupancy_to_static_layer(
            np.asarray(occupancy, dtype=np.int16), lethal_threshold=100,
        )

    def compose(
        self, base_occupancy: np.ndarray, raster: RasterizedSemantics,
        preference_field: Any = None, *, allowed_mask: Optional[np.ndarray] = None,
        semantics_enabled: Optional[bool] = None,
        hard_semantics_enabled: Optional[bool] = None,
        soft_class_costs_enabled: Optional[bool] = None,
        regional_preference_enabled: Optional[bool] = None,
        hard_semantics_use_footprint: bool = False,
    ) -> SemanticCostmap:
        # ``semantics_enabled`` is the r0 compatibility switch.  r1 callers use
        # the three orthogonal switches and persist them per arm.
        legacy_enabled = True if semantics_enabled is None else bool(semantics_enabled)
        hard_enabled = legacy_enabled if hard_semantics_enabled is None else bool(hard_semantics_enabled)
        class_enabled = legacy_enabled if soft_class_costs_enabled is None else bool(soft_class_costs_enabled)
        regional_enabled = legacy_enabled if regional_preference_enabled is None else bool(regional_preference_enabled)
        base = self._base_internal(base_occupancy)
        shape = base.shape
        if shape != (raster.height, raster.width):
            raise ValueError("semantic/base costmap shape mismatch")
        lateral = (
            np.zeros(shape, dtype=np.float32) if preference_field is None
            else np.asarray(preference_field.cost, dtype=np.float32)
        )
        if lateral.shape != shape:
            raise ValueError("preference field shape mismatch")
        class_cache_key = canonical_hash({
            "raster": raster.raster_hash,
            "class_soft_cost": self.policy["class_soft_cost"],
        })
        class_cost = self._class_cost_cache.get(class_cache_key)
        class_cache_hit = class_cost is not None
        if class_cost is None:
            class_cost = np.zeros(shape, dtype=np.float32)
            for semantic_class, value in self.policy["class_soft_cost"].items():
                mask = raster.masks.get(semantic_class)
                if mask is not None and float(value) > 0.0:
                    class_cost[np.asarray(mask, dtype=bool)] += float(value)
            self._class_cost_cache[class_cache_key] = class_cost
        transition_penalty = np.zeros(shape, dtype=np.float32)
        if preference_field is not None and hasattr(preference_field, "junction_transition_factor"):
            factor = np.asarray(preference_field.junction_transition_factor, dtype=np.float32)
            transition_penalty = lateral * np.clip(1.0 - factor, 0.0, 1.0)
        soft = np.clip(
            float(self.policy["w_class"]) * class_cost * float(class_enabled)
            + float(self.policy["w_lateral"]) * lateral * float(regional_enabled)
            + float(self.policy["w_transition"]) * transition_penalty * float(regional_enabled),
            0.0, float(self.policy["soft_cost_max"]),
        ).astype(np.uint8)
        internal = base.copy()
        known = internal != NO_INFORMATION
        internal[known] = np.maximum(internal[known], soft[known])
        hard_source = raster.hard_footprint_mask if hard_semantics_use_footprint else raster.hard_mask
        hard = np.asarray(hard_source, dtype=bool) if hard_enabled else np.zeros(shape, bool)
        internal[hard] = LETHAL_OBSTACLE
        lethal_after_semantic_composition = int(np.count_nonzero(internal == LETHAL_OBSTACLE))
        allowed = np.ones(shape, dtype=bool)
        if allowed_mask is not None:
            allowed = np.asarray(allowed_mask, dtype=bool)
            if allowed.shape != shape:
                raise ValueError("allowed mask shape mismatch")
            # Soft semantics outside the published/search ROI are neither
            # observable nor relevant.  Removing them keeps ACK verification
            # proportional to the affected crop instead of the whole map.
            soft[~allowed] = 0
            internal[~allowed] = LETHAL_OBSTACLE
        occupancy = np.zeros(shape, dtype=np.int8)
        occupancy[internal == NO_INFORMATION] = -1
        occupancy[internal == LETHAL_OBSTACLE] = 100
        soft_cells = (internal > 0) & (internal < LETHAL_OBSTACLE)
        occupancy[soft_cells] = internal_soft_to_occupancy(internal[soft_cells]).astype(np.int8)
        static = occupancy_to_static_layer(
            occupancy, lethal_threshold=int(self.policy["static_layer"]["lethal_cost_threshold"]),
        )
        inflation = self.policy["inflation"]
        expected_master = nav2_inflated_master(
            static, resolution=float(raster.resolution),
            inflation_radius_m=float(inflation["inflation_radius_m"]),
            cost_scaling_factor=float(inflation["cost_scaling_factor"]),
            inscribed_radius_m=float(inflation["inscribed_radius_m"]),
        )
        effective_hard = hard & allowed
        affected = effective_hard | (soft > 0)
        return SemanticCostmap(
            internal_cost=internal,
            occupancy_grid=occupancy,
            expected_master_cost=expected_master,
            soft_cost=soft,
            hard_semantic_mask=effective_hard,
            affected_mask=affected,
            policy_hash=self.policy_hash,
            semantic_map_hash=raster.semantic_map_hash,
            preference_field_hash=str(getattr(preference_field, "field_hash", "")),
            semantics_enabled=bool(hard_enabled or class_enabled or regional_enabled),
            diagnostics={
                "composition": "max_preserves_static_obstacles",
                "static_obstacle_cells_before": int(np.count_nonzero(base == LETHAL_OBSTACLE)),
                "static_obstacle_cells_after": lethal_after_semantic_composition,
                "final_lethal_cells_including_roi_boundary": int(
                    np.count_nonzero(internal == LETHAL_OBSTACLE)
                ),
                "hard_semantic_cells": int(np.count_nonzero(effective_hard)),
                "soft_semantic_cells": int(np.count_nonzero(soft)),
                "soft_internal_max": int(np.max(soft)) if soft.size else 0,
                "soft_occupancy_max": int(np.max(occupancy[(occupancy > 0) & (occupancy < 100)]))
                if np.any((occupancy > 0) & (occupancy < 100)) else 0,
                "trinary_costmap": False,
                "occupancy_100_reserved_for_lethal": True,
                "hard_semantics_enabled": hard_enabled,
                "soft_class_costs_enabled": class_enabled,
                "regional_preference_enabled": regional_enabled,
                "hard_semantics_use_footprint": bool(hard_semantics_use_footprint),
                "class_cost_cache_hit": class_cache_hit,
                "affected_roi_only": True,
            },
        )


@dataclass
class SemanticAck:
    publication_version: str
    roi_sequence: int
    semantic_policy_hash: str
    expected_grid_hash: str
    expected_master_hash: str
    received_costmap_timestamp_ns: int
    affected_cells: int
    hard_checked_cells: int
    hard_mismatch_cells: int
    soft_checked_cells: int
    soft_mismatch_cells: int
    acknowledged: bool
    status: str

    def to_dict(self) -> Dict[str, Any]:
        return dict(self.__dict__)


class SemanticCostmapAckVerifier:
    PUBLICATION_VERSION = "2A-V2-semantic-costmap-v1"

    def verify(
        self, costmap: SemanticCostmap, received_master_cost: np.ndarray,
        *, roi_sequence: int, received_costmap_timestamp_ns: int,
        affected_mask: Optional[np.ndarray] = None,
    ) -> SemanticAck:
        received = np.asarray(received_master_cost, dtype=np.uint8)
        expected = np.asarray(costmap.expected_master_cost, dtype=np.uint8)
        if received.shape != expected.shape:
            raise ValueError("received costmap shape mismatch")
        affected = np.asarray(
            costmap.affected_mask if affected_mask is None else affected_mask, dtype=bool,
        )
        hard = affected & np.asarray(costmap.hard_semantic_mask, dtype=bool)
        soft = affected & (costmap.soft_cost > 0) & ~hard
        hard_mismatch = int(np.count_nonzero(received[hard] != LETHAL_OBSTACLE))
        # Inflation and StaticLayer quantization are part of expected_master;
        # soft ACK therefore compares the exact deterministic master value.
        soft_mismatch = int(np.count_nonzero(received[soft] != expected[soft]))
        acknowledged = hard_mismatch == 0 and soft_mismatch == 0
        return SemanticAck(
            publication_version=self.PUBLICATION_VERSION,
            roi_sequence=int(roi_sequence),
            semantic_policy_hash=costmap.policy_hash,
            expected_grid_hash=costmap.expected_grid_hash,
            expected_master_hash=costmap.expected_master_hash,
            received_costmap_timestamp_ns=int(received_costmap_timestamp_ns),
            affected_cells=int(np.count_nonzero(affected)),
            hard_checked_cells=int(np.count_nonzero(hard)),
            hard_mismatch_cells=hard_mismatch,
            soft_checked_cells=int(np.count_nonzero(soft)),
            soft_mismatch_cells=soft_mismatch,
            acknowledged=acknowledged,
            status="verified" if acknowledged else "semantic_cost_mismatch",
        )


__all__ = [
    "FREE_SPACE", "INSCRIBED_INFLATED_OBSTACLE", "LETHAL_OBSTACLE", "NO_INFORMATION",
    "DEFAULT_COMPOSER_POLICY", "SemanticCostmap", "SemanticCostmapComposer",
    "SemanticAck", "SemanticCostmapAckVerifier", "internal_soft_to_occupancy",
    "occupancy_to_static_layer", "nav2_inflated_master",
]
