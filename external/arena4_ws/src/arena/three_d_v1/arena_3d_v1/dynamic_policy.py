"""Dynamic confirmation and scheduler policy for 3D-V1.

Snapshots are complete occupancy observations.  Two observations are required
to block and two to recover.  Pending cells retain their last committed
traversability, so scheduler decisions are based only on effective cost
changes rather than sensor chatter.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Set, Tuple

import numpy as np

from arena_evaluation.dynamic_snapshot import DynamicSnapshot


Cell = Tuple[int, int]
AVAILABLE = "AVAILABLE"
BLOCKED_PENDING = "BLOCKED_PENDING"
BLOCKED = "BLOCKED"
RECOVERING = "RECOVERING"


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


def _confidence(snapshot: DynamicSnapshot, cell: Cell) -> float:
    values = snapshot.obstacle_confidence
    for key in (f"{cell[0]},{cell[1]}", str(cell), f"{cell[0]}:{cell[1]}"):
        if key in values:
            return float(values[key])
    return 1.0


def _inflate(cells: Iterable[Cell], shape: Sequence[int], radius: int) -> Set[Cell]:
    height, width = int(shape[0]), int(shape[1])
    radius = max(0, int(radius))
    offsets = [
        (drow, dcolumn)
        for drow in range(-radius, radius + 1)
        for dcolumn in range(-radius, radius + 1)
        if drow * drow + dcolumn * dcolumn <= radius * radius
    ]
    return {
        (row + drow, column + dcolumn)
        for row, column in cells
        for drow, dcolumn in offsets
        if 0 <= row + drow < height and 0 <= column + dcolumn < width
    }


@dataclass(frozen=True)
class ConfirmedGridUpdate:
    accepted: bool
    rejection_reason: str
    snapshot_id: str
    snapshot_hash: str
    occupied_observations: Tuple[Cell, ...]
    source_status_changes: Mapping[Cell, str]
    newly_blocked_sources: Tuple[Cell, ...]
    newly_freed_sources: Tuple[Cell, ...]
    blocked_cells: Tuple[Cell, ...]
    newly_blocked_cells: Tuple[Cell, ...]
    newly_freed_cells: Tuple[Cell, ...]
    effective_changed_cells: Tuple[Cell, ...]
    confirmation_ms: float
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


class DynamicGridConfirmation:
    """Protocol guard plus two-hit block/recovery hysteresis."""

    def __init__(
        self,
        *,
        map_version: str,
        map_shape: Sequence[int],
        inflation_radius_cells: int = 7,
        confidence_threshold: float = 0.60,
    ) -> None:
        self.map_version = str(map_version)
        self.map_shape = (int(map_shape[0]), int(map_shape[1]))
        self.inflation_radius_cells = max(0, int(inflation_radius_cells))
        self.confidence_threshold = float(confidence_threshold)
        self.source_states: Dict[Cell, str] = {}
        self.committed_blocked_sources: Set[Cell] = set()
        self.blocked_cells: Set[Cell] = set()
        self.last_timestamp: Optional[float] = None
        self.last_snapshot_hash = ""

    def _rejected(
        self, snapshot: DynamicSnapshot, started_ns: int, reason: str,
    ) -> ConfirmedGridUpdate:
        return ConfirmedGridUpdate(
            False, reason, snapshot.snapshot_id, snapshot.snapshot_hash, (), {}, (), (),
            tuple(sorted(self.blocked_cells)), (), (), (), _elapsed_ms(started_ns),
            {"state_mutated": False},
        )

    def consume(
        self, snapshot: DynamicSnapshot, *, now: Optional[float] = None,
    ) -> ConfirmedGridUpdate:
        started_ns = time.monotonic_ns()
        if str(snapshot.map_version) != self.map_version:
            return self._rejected(snapshot, started_ns, "MAP_VERSION_MISMATCH")
        if snapshot.map_shape is not None and tuple(snapshot.map_shape) != self.map_shape:
            return self._rejected(snapshot, started_ns, "MAP_SHAPE_MISMATCH")
        if snapshot.is_expired(now=now):
            return self._rejected(snapshot, started_ns, "EXPIRED_SNAPSHOT")
        if self.last_timestamp is not None and float(snapshot.timestamp) <= self.last_timestamp:
            return self._rejected(snapshot, started_ns, "OUT_OF_ORDER_SNAPSHOT")

        height, width = self.map_shape
        occupied = {
            (int(cell[0]), int(cell[1]))
            for cell in snapshot.occupied_cells
            if 0 <= int(cell[0]) < height and 0 <= int(cell[1]) < width
            and _confidence(snapshot, (int(cell[0]), int(cell[1]))) >= self.confidence_threshold
        }
        candidates = occupied | set(self.source_states)
        changes: Dict[Cell, str] = {}
        newly_blocked_sources: Set[Cell] = set()
        newly_freed_sources: Set[Cell] = set()
        for cell in sorted(candidates):
            state = self.source_states.get(cell, AVAILABLE)
            seen = cell in occupied
            if state == AVAILABLE:
                next_state = BLOCKED_PENDING if seen else AVAILABLE
            elif state == BLOCKED_PENDING:
                next_state = BLOCKED if seen else AVAILABLE
            elif state == BLOCKED:
                next_state = BLOCKED if seen else RECOVERING
            elif state == RECOVERING:
                next_state = BLOCKED if seen else AVAILABLE
            else:
                raise ValueError(f"unknown grid dynamic state: {state}")
            if next_state != state:
                changes[cell] = next_state
                if next_state == BLOCKED and state == BLOCKED_PENDING:
                    newly_blocked_sources.add(cell)
                elif next_state == AVAILABLE and state == RECOVERING:
                    newly_freed_sources.add(cell)
            if next_state == AVAILABLE:
                self.source_states.pop(cell, None)
                self.committed_blocked_sources.discard(cell)
            else:
                self.source_states[cell] = next_state
                if next_state in {BLOCKED, RECOVERING}:
                    self.committed_blocked_sources.add(cell)
                else:
                    self.committed_blocked_sources.discard(cell)

        previous = set(self.blocked_cells)
        current = _inflate(
            self.committed_blocked_sources, self.map_shape,
            self.inflation_radius_cells,
        )
        newly_blocked = current - previous
        newly_freed = previous - current
        self.blocked_cells = current
        self.last_timestamp = float(snapshot.timestamp)
        self.last_snapshot_hash = str(snapshot.snapshot_hash)
        return ConfirmedGridUpdate(
            True, "", snapshot.snapshot_id, snapshot.snapshot_hash,
            tuple(sorted(occupied)), dict(changes),
            tuple(sorted(newly_blocked_sources)), tuple(sorted(newly_freed_sources)),
            tuple(sorted(current)),
            tuple(sorted(newly_blocked)), tuple(sorted(newly_freed)),
            tuple(sorted(newly_blocked | newly_freed)), _elapsed_ms(started_ns),
            {
                "confidence_threshold": self.confidence_threshold,
                "inflation_radius_cells": self.inflation_radius_cells,
                "source_state_count": len(self.source_states),
                "committed_blocked_source_count": len(self.committed_blocked_sources),
                "blocked_pending_count": sum(
                    state == BLOCKED_PENDING for state in self.source_states.values()
                ),
                "recovering_count": sum(
                    state == RECOVERING for state in self.source_states.values()
                ),
            },
        )


@dataclass(frozen=True)
class SchedulerDecision:
    invoke_l2: bool
    reason: str
    relevant_changed_cells: Tuple[Cell, ...] = ()
    path_intersections: Tuple[Cell, ...] = ()


class RelevanceScheduler:
    """Skip only changes that cannot invalidate the current shortest path."""

    def __init__(self, corridor_mask: np.ndarray) -> None:
        self.corridor_mask = np.asarray(corridor_mask, dtype=bool)

    @staticmethod
    def _path_support(path: Optional[Sequence[Cell]], shape: Sequence[int]) -> Set[Cell]:
        if not path:
            return set()
        height, width = int(shape[0]), int(shape[1])
        return {
            (int(cell[0]) + drow, int(cell[1]) + dcolumn)
            for cell in path
            for drow in (-1, 0, 1)
            for dcolumn in (-1, 0, 1)
            if 0 <= int(cell[0]) + drow < height
            and 0 <= int(cell[1]) + dcolumn < width
        }

    def decide(
        self,
        update: ConfirmedGridUpdate,
        current_path: Optional[Sequence[Cell]],
    ) -> SchedulerDecision:
        if not update.accepted:
            return SchedulerDecision(False, update.rejection_reason)
        if not update.effective_changed_cells:
            return SchedulerDecision(False, "DUPLICATE_OR_UNCONFIRMED_OBSERVATION")
        relevant = tuple(sorted(
            cell for cell in update.effective_changed_cells
            if self.corridor_mask[cell]
        ))
        if not relevant:
            return SchedulerDecision(False, "CHANGE_OUTSIDE_ACTIVE_CORRIDOR")
        freed = set(update.newly_freed_cells).intersection(relevant)
        if freed:
            # A cost decrease can create a new optimum even off the old path.
            return SchedulerDecision(True, "RECOVERY_REQUIRES_OPTIMALITY_REPAIR", relevant)
        support = self._path_support(current_path, self.corridor_mask.shape)
        intersections = tuple(sorted(set(relevant).intersection(support)))
        if not intersections:
            # Increasing an off-path cost cannot invalidate or improve the
            # currently shortest feasible path.
            return SchedulerDecision(
                False, "OFF_PATH_COST_INCREASE", relevant, intersections,
            )
        return SchedulerDecision(
            True, "CURRENT_L2_PATH_AFFECTED", relevant, intersections,
        )


__all__ = [
    "AVAILABLE", "BLOCKED_PENDING", "BLOCKED", "RECOVERING",
    "ConfirmedGridUpdate", "DynamicGridConfirmation", "RelevanceScheduler",
    "SchedulerDecision",
]
