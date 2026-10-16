"""Immutable dynamic-obstacle snapshots for the 3D-V0 planner.

Dynamic occupancy is kept separate from the static map.  A snapshot is a
small, hashable description of the currently observed obstacle cells; the
planner derives a temporary traversability overlay from it and never mutates
the static occupancy array.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np


Cell = Tuple[int, int]


def _normalise_cells(cells: Iterable[Sequence[int]]) -> Tuple[Cell, ...]:
    result = {(int(cell[0]), int(cell[1])) for cell in cells}
    return tuple(sorted(result))


@dataclass(frozen=True)
class DynamicSnapshot:
    """One versioned observation of dynamic occupancy."""

    snapshot_id: str
    timestamp: float
    occupied_cells: Tuple[Cell, ...] = field(default_factory=tuple)
    obstacle_confidence: Mapping[str, float] = field(default_factory=dict)
    ttl: Optional[float] = None
    map_version: str = ""
    map_shape: Optional[Tuple[int, int]] = None
    snapshot_hash: str = ""

    def __post_init__(self) -> None:
        cells = _normalise_cells(self.occupied_cells)
        object.__setattr__(self, "occupied_cells", cells)
        confidence = {str(key): float(value) for key, value in dict(self.obstacle_confidence).items()}
        object.__setattr__(self, "obstacle_confidence", confidence)
        if not self.snapshot_hash:
            payload = {
                "snapshot_id": str(self.snapshot_id),
                "timestamp": float(self.timestamp),
                "occupied_cells": [list(cell) for cell in cells],
                "obstacle_confidence": confidence,
                "ttl": None if self.ttl is None else float(self.ttl),
                "map_version": str(self.map_version),
                "map_shape": None if self.map_shape is None else list(self.map_shape),
            }
            digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
            object.__setattr__(self, "snapshot_hash", digest)

    @classmethod
    def empty(
        cls,
        snapshot_id: str = "static",
        *,
        timestamp: Optional[float] = None,
        map_version: str = "",
        map_shape: Optional[Tuple[int, int]] = None,
    ) -> "DynamicSnapshot":
        return cls(snapshot_id, time.time() if timestamp is None else float(timestamp), map_version=map_version, map_shape=map_shape)

    @classmethod
    def from_cells(
        cls,
        snapshot_id: str,
        cells: Iterable[Sequence[int]],
        *,
        timestamp: Optional[float] = None,
        confidence: Optional[Mapping[str, float]] = None,
        ttl: Optional[float] = None,
        map_version: str = "",
        map_shape: Optional[Tuple[int, int]] = None,
    ) -> "DynamicSnapshot":
        return cls(
            str(snapshot_id), time.time() if timestamp is None else float(timestamp),
            _normalise_cells(cells), confidence or {}, ttl, map_version, map_shape,
        )

    def is_expired(self, now: Optional[float] = None) -> bool:
        return self.ttl is not None and float(now if now is not None else time.time()) > self.timestamp + float(self.ttl)

    def as_dict(self) -> Dict[str, object]:
        return {
            "snapshot_id": self.snapshot_id,
            "timestamp": self.timestamp,
            "occupied_cells": [list(cell) for cell in self.occupied_cells],
            "obstacle_confidence": dict(self.obstacle_confidence),
            "ttl": self.ttl,
            "map_version": self.map_version,
            "map_shape": None if self.map_shape is None else list(self.map_shape),
            "snapshot_hash": self.snapshot_hash,
        }

    def inflated_cells(self, radius_cells: int = 0) -> Tuple[Cell, ...]:
        radius = max(0, int(radius_cells))
        if radius == 0:
            return self.occupied_cells
        result = set()
        for row, col in self.occupied_cells:
            for drow in range(-radius, radius + 1):
                for dcol in range(-radius, radius + 1):
                    if drow * drow + dcol * dcol <= radius * radius:
                        result.add((row + drow, col + dcol))
        return tuple(sorted(result))


def apply_dynamic_snapshot(
    static_free: np.ndarray,
    snapshot: DynamicSnapshot,
    *,
    inflation_radius_cells: int = 0,
    extra_cost: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray, Tuple[Cell, ...]]:
    """Return ``(free, costs, changed_cells)`` without mutating ``static_free``."""
    base = np.asarray(static_free, dtype=bool)
    if base.ndim != 2:
        raise ValueError("static_free must be a 2-D array")
    if snapshot.map_shape is not None and tuple(snapshot.map_shape) != tuple(base.shape):
        raise ValueError("snapshot map_shape does not match static_free")
    free = base.copy()
    costs = np.ones(base.shape, dtype=np.float64)
    changed = []
    for row, col in snapshot.inflated_cells(inflation_radius_cells):
        if 0 <= row < base.shape[0] and 0 <= col < base.shape[1]:
            changed.append((row, col))
            free[row, col] = False
            if extra_cost > 0.0:
                costs[row, col] = float(extra_cost)
    return free, costs, tuple(sorted(set(changed)))


def path_intersects_snapshot(
    path: Sequence[Cell], snapshot: DynamicSnapshot, *, inflation_radius_cells: int = 0,
    ahead_from_index: int = 0,
) -> bool:
    occupied = set(snapshot.inflated_cells(inflation_radius_cells))
    start = max(0, int(ahead_from_index))
    return any(tuple(cell) in occupied for cell in path[start:])


__all__ = ["Cell", "DynamicSnapshot", "apply_dynamic_snapshot", "path_intersects_snapshot"]
