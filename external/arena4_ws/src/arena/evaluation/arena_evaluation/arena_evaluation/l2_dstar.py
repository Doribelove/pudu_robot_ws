"""L2 corridor planner for the independent ``3D-V0`` architecture."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .dstar_lite import Cell, DStarLite, DStarSearchStats
from .dynamic_snapshot import DynamicSnapshot, apply_dynamic_snapshot, path_intersects_snapshot


@dataclass(frozen=True)
class L2PlanResult:
    path: Optional[List[Cell]]
    success: bool
    failure_code: str = ""
    stats: DStarSearchStats = field(default_factory=DStarSearchStats)
    dynamic_update: bool = False
    path_changed: bool = False
    repair_start_index: Optional[int] = None
    repair_end_index: Optional[int] = None
    expanded_cells: int = 0
    changed_cells: int = 0
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


class L2DStarCorridor:
    """Persistent D* Lite state for one static corridor and one goal."""

    def __init__(
        self,
        static_free: np.ndarray,
        corridor_mask: np.ndarray,
        start: Cell,
        goal: Cell,
        *,
        corridor_id: str = "corridor-0",
        allow_diagonal: bool = True,
        dynamic_inflation_cells: int = 0,
        cost_map: Optional[np.ndarray] = None,
    ) -> None:
        static = np.asarray(static_free, dtype=bool)
        corridor = np.asarray(corridor_mask, dtype=bool)
        if static.ndim != 2 or corridor.shape != static.shape:
            raise ValueError("static_free and corridor_mask must be matching 2-D arrays")
        self.static_free = static.copy()
        self.corridor_mask = corridor.copy()
        self.corridor_id = str(corridor_id)
        self.dynamic_inflation_cells = max(0, int(dynamic_inflation_cells))
        self.base_free = self.static_free & self.corridor_mask
        if not self.base_free[tuple(start)] or not self.base_free[tuple(goal)]:
            raise ValueError("start and goal must be free inside corridor")
        self.start = tuple(map(int, start))
        self.goal = tuple(map(int, goal))
        self.current_snapshot = DynamicSnapshot.empty(map_shape=self.base_free.shape)
        self._dynamic_cells: Tuple[Cell, ...] = tuple()
        self._current_free = self.base_free.copy()
        self._current_costs = np.ones(self.base_free.shape, dtype=np.float64) if cost_map is None else np.asarray(cost_map, dtype=np.float64).copy()
        self.dstar = DStarLite(self._current_free, self.start, self.goal, cost_map=self._current_costs, allow_diagonal=allow_diagonal)
        self.last_path: Optional[List[Cell]] = None
        self.reset_count = 0
        self.update_count = 0

    def _apply_snapshot(self, snapshot: DynamicSnapshot) -> int:
        if snapshot.is_expired():
            snapshot = DynamicSnapshot.empty(
                snapshot_id=f"{snapshot.snapshot_id}:expired", map_shape=self.base_free.shape,
            )
        free, costs, new_cells = apply_dynamic_snapshot(
            self.base_free, snapshot, inflation_radius_cells=self.dynamic_inflation_cells,
        )
        old_cells = set(self._dynamic_cells)
        changed = old_cells.symmetric_difference(new_cells)
        self.current_snapshot = snapshot
        self._dynamic_cells = tuple(new_cells)
        self._current_free = free
        self._current_costs = costs
        if changed:
            self.dstar.update_cells(changed, traversable=free, cost_map=costs)
        self.update_count += 1
        return len(changed)

    def _run(self, *, timeout_s: Optional[float], max_expansions: Optional[int]) -> Tuple[Optional[List[Cell]], DStarSearchStats]:
        stats = self.dstar.compute_shortest_path(timeout_s=timeout_s, max_expansions=max_expansions)
        extract_started = time.monotonic_ns()
        path = self.dstar.extract_path()
        self._last_extract_time_ms = (time.monotonic_ns() - extract_started) / 1.0e6
        return path, stats

    def plan(
        self,
        *,
        snapshot: Optional[DynamicSnapshot] = None,
        start: Optional[Cell] = None,
        goal: Optional[Cell] = None,
        timeout_s: Optional[float] = None,
        max_expansions: Optional[int] = None,
    ) -> L2PlanResult:
        """Plan from the current start to goal, preserving D* Lite state."""
        if start is not None:
            new_start = tuple(map(int, start))
            self.dstar.set_start(new_start)
            self.start = new_start
        if goal is not None and tuple(goal) != self.goal:
            self.goal = tuple(map(int, goal))
            self.dstar.set_goal(self.goal)
        changed = self._apply_snapshot(snapshot or self.current_snapshot)
        path, stats = self._run(timeout_s=timeout_s, max_expansions=max_expansions)
        self.last_path = list(path) if path else None
        failure = "" if path else ("DSTAR_TIMEOUT" if stats.timeout_triggered else "NO_PATH_IN_CORRIDOR")
        return L2PlanResult(
            path=path, success=path is not None, failure_code=failure, stats=stats,
            dynamic_update=changed > 0, path_changed=False, changed_cells=changed,
            expanded_cells=stats.expanded_nodes,
            diagnostics={
                "corridor_id": self.corridor_id,
                "snapshot_id": self.current_snapshot.snapshot_id,
                "dstar_extract_path_ms": float(getattr(self, "_last_extract_time_ms", 0.0)),
                "dstar_initial_queue_size": int(stats.initial_queue_size),
                "dstar_final_queue_size": int(stats.final_queue_size),
                "dstar_queue_push_count": int(stats.queue_pushes),
                "dstar_queue_pop_count": int(stats.queue_pops),
                "dstar_update_vertex_count": int(stats.update_vertex_count),
            },
        )

    @staticmethod
    def _first_impacted_index(path: Sequence[Cell], snapshot: DynamicSnapshot, inflation: int, start_index: int) -> Optional[int]:
        occupied = set(snapshot.inflated_cells(inflation))
        for index in range(max(0, int(start_index)), len(path)):
            if tuple(path[index]) in occupied:
                return index
        return None

    def _select_ab(
        self, old_path: Sequence[Cell], impacted_index: int, *, current_index: int = 0,
    ) -> Tuple[int, int]:
        # Start before the first blocked cell.  Starting exactly on the
        # obstacle makes the local search invalid even though a detour exists.
        a_index = max(int(current_index), int(impacted_index) - 1)
        while a_index > int(current_index) and not self._current_free[tuple(old_path[a_index])]:
            a_index -= 1
        b_index = len(old_path) - 1
        for index in range(a_index + 1, len(old_path)):
            cell = tuple(old_path[index])
            if self._current_free[cell] and self.corridor_mask[cell]:
                # Require a short clear suffix to avoid reconnecting directly
                # at the obstacle boundary.
                suffix = old_path[index:min(len(old_path), index + 3)]
                if all(self._current_free[tuple(item)] for item in suffix):
                    b_index = index
                    break
        return a_index, b_index

    def repair_path(
        self,
        old_path: Sequence[Cell],
        *,
        current_index: int = 0,
        snapshot: Optional[DynamicSnapshot] = None,
        forward_buffer_cells: int = 0,
        timeout_s: Optional[float] = None,
        max_expansions: Optional[int] = None,
    ) -> L2PlanResult:
        """Update dynamic costs and replace only the affected A-B segment."""
        if not old_path:
            return self.plan(snapshot=snapshot, timeout_s=timeout_s, max_expansions=max_expansions)
        snap = snapshot or self.current_snapshot
        changed = self._apply_snapshot(snap)
        hit = self._first_impacted_index(old_path, snap, self.dynamic_inflation_cells, current_index)
        if hit is None:
            self.last_path = list(old_path)
            return L2PlanResult(
                path=list(old_path), success=True, dynamic_update=changed > 0,
                path_changed=False, changed_cells=changed,
                diagnostics={"triggered": False, "reason": "path_not_affected", "snapshot_id": snap.snapshot_id},
            )
        a_index, b_index = self._select_ab(old_path, hit + max(0, int(forward_buffer_cells)), current_index=current_index)
        a = tuple(old_path[a_index])
        b = tuple(old_path[b_index])
        # Keep the primary full-goal D* state alive.  A repair goal can change
        # from one snapshot to the next, so a bounded temporary reverse search
        # is safer than corrupting the persistent full-goal state.
        repair = DStarLite(self._current_free, a, b, cost_map=self._current_costs, allow_diagonal=self.dstar.allow_diagonal)
        started = time.monotonic_ns()
        stats = repair.compute_shortest_path(timeout_s=timeout_s, max_expansions=max_expansions)
        segment = repair.extract_path()
        if segment is None:
            failure = "DSTAR_TIMEOUT" if stats.timeout_triggered else "NO_PATH_IN_CORRIDOR"
            return L2PlanResult(
                path=None, success=False, failure_code=failure, stats=stats,
                dynamic_update=True, repair_start_index=a_index, repair_end_index=b_index,
                changed_cells=changed, expanded_cells=stats.expanded_nodes,
                diagnostics={"triggered": True, "snapshot_id": snap.snapshot_id, "repair_time_ms": (time.monotonic_ns() - started) / 1.0e6},
            )
        merged = list(old_path[:a_index]) + list(segment)
        if b_index + 1 < len(old_path):
            merged.extend(old_path[b_index + 1:])
        path_changed = merged != list(old_path)
        self.last_path = merged
        return L2PlanResult(
            path=merged, success=True, stats=stats, dynamic_update=True,
            path_changed=path_changed, repair_start_index=a_index, repair_end_index=b_index,
            changed_cells=changed, expanded_cells=stats.expanded_nodes,
            diagnostics={
                "triggered": True, "snapshot_id": snap.snapshot_id,
                "repair_a": list(a), "repair_b": list(b),
                "repair_time_ms": (time.monotonic_ns() - started) / 1.0e6,
            },
        )

    def reset(self, *, start: Optional[Cell] = None, goal: Optional[Cell] = None) -> None:
        self.start = self.start if start is None else tuple(map(int, start))
        self.goal = self.goal if goal is None else tuple(map(int, goal))
        self.dstar = DStarLite(self._current_free, self.start, self.goal, cost_map=self._current_costs, allow_diagonal=self.dstar.allow_diagonal)
        self.reset_count += 1


__all__ = ["L2DStarCorridor", "L2PlanResult"]
