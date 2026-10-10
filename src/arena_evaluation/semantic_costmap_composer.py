from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict
import numpy as np
from .semantic_rasterizer import grid_hash
LETHAL_OBSTACLE = np.uint8(254)
NO_INFORMATION = np.uint8(255)

def internal_soft_to_occupancy(value: np.ndarray | int) -> np.ndarray:
    """Map internal 0..252 soft values to OccupancyGrid 0..99."""
    array = np.asarray(value, dtype=np.float64)
    clipped = np.clip(array, 0.0, 252.0)
    return np.where(clipped <= 0.0, 0, np.ceil(clipped * 99.0 / 252.0)).astype(np.int16)

def occupancy_to_static_layer(value: np.ndarray, *, lethal_threshold: int=100) -> np.ndarray:
    """Mirror Nav2 Humble StaticLayer::interpretValue with trinary=false."""
    occupancy = np.asarray(value, dtype=np.int16)
    result = np.zeros(occupancy.shape, dtype=np.uint8)
    result[occupancy < 0] = NO_INFORMATION
    lethal = occupancy >= int(lethal_threshold)
    result[lethal] = LETHAL_OBSTACLE
    soft = (occupancy > 0) & ~lethal
    result[soft] = np.floor(occupancy[soft].astype(np.float64) / float(lethal_threshold) * float(LETHAL_OBSTACLE)).astype(np.uint8)
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
