"""Persistent, cropped-ROI grid D* Lite for 3D-V1.

The implementation intentionally does not import the historical 3D-V0 L2
wrapper.  It uses the independently tested grid D* primitive from
``arena_evaluation`` and adds the production requirements that were missing
there: complete state binding, in-place cell patches, diagonal corner safety,
bounded fallback, and an oracle-compatible deterministic grid A*.
"""

from __future__ import annotations

import hashlib
import heapq
import json
import math
import sys
import time
from dataclasses import dataclass, field
from itertools import count
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np

from arena_evaluation.dstar_lite import DStarLite, DStarSearchStats, INF


Cell = Tuple[int, int]


def _stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str,
    ).encode("utf-8")).hexdigest()


def _grid_hash(mask: np.ndarray) -> str:
    return hashlib.sha256(
        np.ascontiguousarray(np.asarray(mask, dtype=np.uint8)).tobytes()
    ).hexdigest()


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


@dataclass(frozen=True)
class CorridorBinding:
    """Identity of the exact L2 state; every field affects safe reuse."""

    map_hash: str
    map_shape: Tuple[int, int]
    map_origin: Tuple[float, float, float]
    resolution: float
    topology_hash: str
    route_edge_ids: Tuple[str, ...]
    corridor_mask_hash: str
    start_cell: Cell
    goal_cell: Cell
    footprint_hash: str

    @property
    def digest(self) -> str:
        return _stable_hash({
            "map_hash": self.map_hash,
            "map_shape": list(self.map_shape),
            "map_origin": list(self.map_origin),
            "resolution": self.resolution,
            "topology_hash": self.topology_hash,
            "route_edge_ids": list(self.route_edge_ids),
            "corridor_mask_hash": self.corridor_mask_hash,
            "start_cell": list(self.start_cell),
            "goal_cell": list(self.goal_cell),
            "footprint_hash": self.footprint_hash,
        })


@dataclass(frozen=True)
class CorridorROI:
    """A full-resolution crop around one adaptive L1 corridor."""

    bbox: Tuple[int, int, int, int]
    base_free: np.ndarray
    start_local: Cell
    goal_local: Cell
    binding: CorridorBinding
    global_corridor_cells: int

    @classmethod
    def from_global(
        cls,
        static_safe_free: np.ndarray,
        corridor_mask: np.ndarray,
        start: Cell,
        goal: Cell,
        *,
        binding_fields: Mapping[str, Any],
        border_cells: int = 1,
    ) -> "CorridorROI":
        static = np.asarray(static_safe_free, dtype=bool)
        corridor = np.asarray(corridor_mask, dtype=bool)
        if static.ndim != 2 or corridor.shape != static.shape:
            raise ValueError("static_safe_free and corridor_mask must be matching 2-D grids")
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        for name, cell in (("start", start), ("goal", goal)):
            if not (0 <= cell[0] < static.shape[0] and 0 <= cell[1] < static.shape[1]):
                raise ValueError(f"{name} lies outside the map")
            if not static[cell] or not corridor[cell]:
                raise ValueError(f"{name} must be footprint-safe and inside the corridor")
        rows, columns = np.nonzero(corridor)
        if not len(rows):
            raise ValueError("corridor_mask is empty")
        border = max(0, int(border_cells))
        r0 = max(0, int(rows.min()) - border)
        r1 = min(static.shape[0], int(rows.max()) + 1 + border)
        c0 = max(0, int(columns.min()) - border)
        c1 = min(static.shape[1], int(columns.max()) + 1 + border)
        base = np.ascontiguousarray(
            static[r0:r1, c0:c1] & corridor[r0:r1, c0:c1], dtype=bool,
        )
        corridor_hash = str(binding_fields.get("corridor_mask_hash") or _grid_hash(corridor))
        binding = CorridorBinding(
            map_hash=str(binding_fields.get("map_hash", "")),
            map_shape=(int(static.shape[0]), int(static.shape[1])),
            map_origin=tuple(float(value) for value in binding_fields.get(
                "map_origin", (0.0, 0.0, 0.0),
            )),
            resolution=float(binding_fields.get("resolution", 0.05)),
            topology_hash=str(binding_fields.get("topology_hash", "")),
            route_edge_ids=tuple(str(value) for value in binding_fields.get("route_edge_ids", ())),
            corridor_mask_hash=corridor_hash,
            start_cell=start,
            goal_cell=goal,
            footprint_hash=str(binding_fields.get("footprint_hash", "")),
        )
        if abs(binding.resolution - 0.05) > 1.0e-12:
            raise ValueError("3D-V1 L2 requires the project 0.05 m/cell grid")
        return cls(
            (r0, r1, c0, c1), base,
            (start[0] - r0, start[1] - c0),
            (goal[0] - r0, goal[1] - c0), binding,
            int(np.count_nonzero(corridor)),
        )

    @property
    def shape(self) -> Tuple[int, int]:
        return int(self.base_free.shape[0]), int(self.base_free.shape[1])

    def contains_global(self, cell: Cell) -> bool:
        r0, r1, c0, c1 = self.bbox
        return r0 <= int(cell[0]) < r1 and c0 <= int(cell[1]) < c1

    def to_local(self, cell: Cell) -> Cell:
        if not self.contains_global(cell):
            raise ValueError(f"global cell outside corridor ROI: {cell}")
        return int(cell[0]) - self.bbox[0], int(cell[1]) - self.bbox[2]

    def to_global(self, cell: Cell) -> Cell:
        return int(cell[0]) + self.bbox[0], int(cell[1]) + self.bbox[2]


class CornerSafeDStarLite(DStarLite):
    """D* Lite whose diagonal transitions cannot cut blocked corners."""

    def _edge_cost(self, first: Cell, second: Cell) -> float:
        if first[0] != second[0] and first[1] != second[1]:
            side_a = (first[0], second[1])
            side_b = (second[0], first[1])
            if not self.traversable[side_a] or not self.traversable[side_b]:
                return INF
        return super()._edge_cost(first, second)


def _neighbors(mask: np.ndarray, cell: Cell) -> Iterable[Cell]:
    row, column = cell
    height, width = mask.shape
    for drow, dcolumn in (
        (-1, -1), (-1, 0), (-1, 1), (0, -1),
        (0, 1), (1, -1), (1, 0), (1, 1),
    ):
        target = (row + drow, column + dcolumn)
        if not (0 <= target[0] < height and 0 <= target[1] < width):
            continue
        if not mask[target]:
            continue
        if drow and dcolumn:
            if not mask[row, target[1]] or not mask[target[0], column]:
                continue
        yield target


def _step_cost(first: Cell, second: Cell) -> float:
    return math.sqrt(2.0) if first[0] != second[0] and first[1] != second[1] else 1.0


@dataclass(frozen=True)
class GridAStarResult:
    path: Optional[List[Cell]]
    cost: float
    expanded_nodes: int
    generated_nodes: int
    search_time_ms: float
    timeout_triggered: bool = False


def deterministic_grid_astar(
    traversable: np.ndarray,
    start: Cell,
    goal: Cell,
    *,
    timeout_s: Optional[float] = None,
    max_expansions: Optional[int] = None,
) -> GridAStarResult:
    """Cold deterministic A* with exactly the same L2 motion semantics."""
    mask = np.asarray(traversable, dtype=bool)
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    started_ns = time.monotonic_ns()
    if not mask[start] or not mask[goal]:
        return GridAStarResult(None, INF, 0, 0, _elapsed_ms(started_ns))
    deadline = None if timeout_s is None else time.monotonic() + max(0.0, float(timeout_s))
    serial = count()
    queue: List[Tuple[float, float, int, int, int]] = [
        (math.hypot(start[0] - goal[0], start[1] - goal[1]), 0.0,
         next(serial), start[0], start[1]),
    ]
    distance: Dict[Cell, float] = {start: 0.0}
    previous: Dict[Cell, Cell] = {}
    expanded = 0
    generated = 1
    timed_out = False
    while queue:
        if deadline is not None and time.monotonic() >= deadline:
            timed_out = True
            break
        _estimate, cost, _serial, row, column = heapq.heappop(queue)
        cell = (row, column)
        if cost != distance.get(cell):
            continue
        if cell == goal:
            break
        if max_expansions is not None and expanded >= max(0, int(max_expansions)):
            timed_out = True
            break
        expanded += 1
        for target in _neighbors(mask, cell):
            candidate = cost + _step_cost(cell, target)
            old = distance.get(target, INF)
            if candidate < old:
                distance[target] = candidate
                previous[target] = cell
                estimate = candidate + math.hypot(
                    target[0] - goal[0], target[1] - goal[1],
                )
                heapq.heappush(queue, (
                    estimate, candidate, next(serial), target[0], target[1],
                ))
                generated += 1
    if goal not in distance or timed_out:
        return GridAStarResult(
            None, INF, expanded, generated, _elapsed_ms(started_ns), timed_out,
        )
    path = [goal]
    cursor = goal
    while cursor != start:
        cursor = previous[cursor]
        path.append(cursor)
    path.reverse()
    return GridAStarResult(
        path, float(distance[goal]), expanded, generated, _elapsed_ms(started_ns),
    )


@dataclass(frozen=True)
class L2PlanResult:
    success: bool
    path: Optional[List[Cell]]
    failure_code: str
    selected_backend: str
    response_ms: float
    dstar_stats: DStarSearchStats = field(default_factory=DStarSearchStats)
    fallback_stats: Optional[GridAStarResult] = None
    changed_cells: int = 0
    state_reused: bool = False
    partial_dstar_result_returned: bool = False
    oracle_cost_error: Optional[float] = None
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


class PersistentCorridorDStar:
    """Persistent D* state for one exact, full-resolution corridor binding."""

    def __init__(
        self,
        roi: CorridorROI,
        *,
        dstar_wall_budget_ms: float = 20.0,
        dstar_max_expansions: int = 20_000,
    ) -> None:
        self.roi = roi
        self.dstar_wall_budget_ms = max(0.01, float(dstar_wall_budget_ms))
        self.dstar_max_expansions = max(1, int(dstar_max_expansions))
        self.current_free = roi.base_free.copy()
        self.dynamic_blocked_local: Set[Cell] = set()
        self.planner = CornerSafeDStarLite(
            self.current_free, roi.start_local, roi.goal_local, allow_diagonal=True,
        )
        self.current_path_local: Optional[List[Cell]] = None
        self.dstar_ready = True
        self.initialized = False
        self.reinitialize_count = 0
        self.update_count = 0
        self.fallback_count = 0
        self.resync_count = 0

    def prime_blocked(self, blocked_global: Iterable[Cell]) -> int:
        """Apply the current overlay before the offline initial search."""
        if self.initialized:
            raise RuntimeError("prime_blocked() is only valid before initialize()")
        blocked = self._translate_blocked(blocked_global)
        changed = self.dynamic_blocked_local.symmetric_difference(blocked)
        self.dynamic_blocked_local = blocked
        for cell in changed:
            value = bool(self.roi.base_free[cell] and cell not in blocked)
            self.current_free[cell] = value
            self.planner.traversable[cell] = value
        if changed:
            # No search has run yet, so rebuilding the empty reverse-search
            # state is cheaper and clearer than queuing thousands of updates.
            self.planner = CornerSafeDStarLite(
                self.current_free, self.roi.start_local, self.roi.goal_local,
                allow_diagonal=True,
            )
        return len(changed)

    @property
    def binding_hash(self) -> str:
        return self.roi.binding.digest

    @property
    def path_global(self) -> Optional[List[Cell]]:
        if self.current_path_local is None:
            return None
        return [self.roi.to_global(cell) for cell in self.current_path_local]

    def _path_cost(self, path: Optional[Sequence[Cell]]) -> float:
        if not path:
            return INF
        return float(sum(_step_cost(first, second) for first, second in zip(path, path[1:])))

    def _path_is_valid(self, path: Optional[Sequence[Cell]]) -> bool:
        if not path:
            return False
        if tuple(path[0]) != self.roi.start_local or tuple(path[-1]) != self.roi.goal_local:
            return False
        for cell in path:
            if not self.current_free[tuple(cell)]:
                return False
        for first, second in zip(path, path[1:]):
            drow = abs(int(first[0]) - int(second[0]))
            dcolumn = abs(int(first[1]) - int(second[1]))
            if max(drow, dcolumn) != 1:
                return False
            if drow and dcolumn:
                if not self.current_free[int(first[0]), int(second[1])]:
                    return False
                if not self.current_free[int(second[0]), int(first[1])]:
                    return False
        return True

    def _result(
        self,
        *,
        started_ns: int,
        path: Optional[List[Cell]],
        failure: str,
        backend: str,
        stats: DStarSearchStats,
        fallback: Optional[GridAStarResult] = None,
        changed: int = 0,
        reused: bool = False,
        oracle_cost_error: Optional[float] = None,
    ) -> L2PlanResult:
        self.current_path_local = None if path is None else list(path)
        return L2PlanResult(
            success=path is not None,
            path=None if path is None else [self.roi.to_global(cell) for cell in path],
            failure_code=failure,
            selected_backend=backend,
            response_ms=_elapsed_ms(started_ns),
            dstar_stats=stats,
            fallback_stats=fallback,
            changed_cells=int(changed),
            state_reused=bool(reused),
            partial_dstar_result_returned=False,
            oracle_cost_error=oracle_cost_error,
            diagnostics={
                "binding_hash": self.binding_hash,
                "roi_bbox": list(self.roi.bbox),
                "roi_shape": list(self.roi.shape),
                "roi_array_cells": int(self.current_free.size),
                "corridor_cells": int(np.count_nonzero(self.roi.base_free)),
                "global_corridor_cells": int(self.roi.global_corridor_cells),
                "dynamic_blocked_cells": len(self.dynamic_blocked_local),
                "dstar_ready": bool(self.dstar_ready),
                "reinitialize_count": int(self.reinitialize_count),
                "fallback_count": int(self.fallback_count),
                "resync_count": int(self.resync_count),
                "state_memory_bytes": self.state_memory_bytes(),
            },
        )

    def initialize(self, *, verify_oracle: bool = False) -> L2PlanResult:
        """Build the reverse tree. Production prepares this with corridor caches."""
        started_ns = time.monotonic_ns()
        stats = self.planner.compute_shortest_path()
        path = self.planner.extract_path()
        self.initialized = True
        self.dstar_ready = not stats.timeout_triggered
        oracle_error: Optional[float] = None
        if verify_oracle:
            oracle = deterministic_grid_astar(
                self.current_free, self.roi.start_local, self.roi.goal_local,
            )
            dstar_cost = self._path_cost(path)
            if (path is None) != (oracle.path is None):
                raise AssertionError("initial D* reachability differs from grid A* oracle")
            oracle_error = 0.0 if path is None else abs(dstar_cost - oracle.cost)
            if oracle_error > 1.0e-9:
                raise AssertionError("initial D* cost differs from grid A* oracle")
        return self._result(
            started_ns=started_ns, path=path,
            failure="" if path is not None else "L2_NO_PATH_IN_CORRIDOR",
            backend="persistent_dstar", stats=stats,
            oracle_cost_error=oracle_error,
        )

    def _translate_blocked(self, blocked_global: Iterable[Cell]) -> Set[Cell]:
        local: Set[Cell] = set()
        for raw in blocked_global:
            cell = (int(raw[0]), int(raw[1]))
            if not self.roi.contains_global(cell):
                continue
            converted = self.roi.to_local(cell)
            if self.roi.base_free[converted]:
                local.add(converted)
        return local

    def update(
        self,
        blocked_global: Iterable[Cell],
        *,
        verify_oracle: bool = False,
        force_cold_astar: bool = False,
    ) -> L2PlanResult:
        """Patch changed cells and return a converged D* or cold-A* result.

        A bounded D* timeout is never exposed.  The cold deterministic grid A*
        fallback supplies the response and the D* instance becomes not-ready
        until :meth:`service_resync` finishes.
        """
        if not self.initialized:
            raise RuntimeError("initialize() must complete before dynamic updates")
        started_ns = time.monotonic_ns()
        new_blocked = self._translate_blocked(blocked_global)
        changed = self.dynamic_blocked_local.symmetric_difference(new_blocked)
        self.dynamic_blocked_local = new_blocked
        for cell in changed:
            self.current_free[cell] = bool(self.roi.base_free[cell] and cell not in new_blocked)
        self.update_count += 1
        if not changed:
            return self._result(
                started_ns=started_ns, path=self.current_path_local,
                failure="" if self.current_path_local else "L2_NO_PATH_IN_CORRIDOR",
                backend="scheduler_reuse", stats=DStarSearchStats(),
                changed=0, reused=True, oracle_cost_error=0.0 if verify_oracle else None,
            )
        # Keep the partial D* state synchronized even when the response policy
        # selects cold A*.  A later quiet-period resync can then continue the
        # same repair instead of reconstructing the graph.
        for cell in changed:
            self.planner.traversable[cell] = self.current_free[cell]
        self.planner.update_cells(changed)
        if force_cold_astar:
            self.dstar_ready = False
            self.fallback_count += 1
            fallback = deterministic_grid_astar(
                self.current_free, self.roi.start_local, self.roi.goal_local,
            )
            return self._result(
                started_ns=started_ns, path=fallback.path,
                failure="" if fallback.path is not None else "L2_NO_PATH_IN_CORRIDOR",
                backend="deterministic_grid_astar_direct", stats=DStarSearchStats(),
                fallback=fallback, changed=len(changed), reused=False,
                oracle_cost_error=0.0 if verify_oracle else None,
            )
        if self.dstar_ready:
            stats = self.planner.compute_shortest_path(
                timeout_s=self.dstar_wall_budget_ms / 1000.0,
                max_expansions=self.dstar_max_expansions,
            )
            if not stats.timeout_triggered:
                path = self.planner.extract_path()
                # A finite g(start) with no extractable safe path is an
                # inconsistent partial state, not a valid no-route answer.
                if path is None and not stats.no_path:
                    self.dstar_ready = False
                elif path is not None and not self._path_is_valid(path):
                    self.dstar_ready = False
                else:
                    oracle_error: Optional[float] = None
                    if verify_oracle:
                        oracle = deterministic_grid_astar(
                            self.current_free, self.roi.start_local, self.roi.goal_local,
                        )
                        if (path is None) != (oracle.path is None):
                            raise AssertionError("incremental D* reachability differs from grid A*")
                        oracle_error = 0.0 if path is None else abs(
                            self._path_cost(path) - oracle.cost
                        )
                        if oracle_error > 1.0e-9:
                            raise AssertionError("incremental D* cost differs from grid A*")
                    return self._result(
                        started_ns=started_ns, path=path,
                        failure="" if path is not None else "L2_NO_PATH_IN_CORRIDOR",
                        backend="persistent_dstar", stats=stats, changed=len(changed),
                        reused=True, oracle_cost_error=oracle_error,
                    )
        else:
            stats = DStarSearchStats(timeout_triggered=True)

        self.dstar_ready = False
        self.fallback_count += 1
        fallback = deterministic_grid_astar(
            self.current_free, self.roi.start_local, self.roi.goal_local,
        )
        return self._result(
            started_ns=started_ns, path=fallback.path,
            failure="" if fallback.path is not None else "L2_NO_PATH_IN_CORRIDOR",
            backend="deterministic_grid_astar_fallback", stats=stats,
            fallback=fallback, changed=len(changed), reused=False,
            oracle_cost_error=0.0 if verify_oracle else None,
        )

    def service_resync(self) -> L2PlanResult:
        """Rebuild persistent state after a fallback, outside response latency."""
        started_ns = time.monotonic_ns()
        if self.dstar_ready:
            return self._result(
                started_ns=started_ns, path=self.current_path_local, failure="",
                backend="resync_not_required", stats=DStarSearchStats(), reused=True,
            )
        # A bounded stop can leave a finite g(start) whose successor chain is
        # not yet extractable.  Rebuilding during the explicitly-accounted
        # quiet period is the safe resync; this cost is never charged as an
        # incremental response win.
        self.planner = CornerSafeDStarLite(
            self.current_free, self.roi.start_local, self.roi.goal_local,
            allow_diagonal=True,
        )
        self.reinitialize_count += 1
        self.resync_count += 1
        stats = self.planner.compute_shortest_path()
        path = self.planner.extract_path()
        self.dstar_ready = not stats.timeout_triggered
        return self._result(
            started_ns=started_ns, path=path,
            failure="" if path is not None else "L2_NO_PATH_IN_CORRIDOR",
            backend="persistent_dstar_resync", stats=stats,
        )

    def state_memory_bytes(self) -> int:
        planner = self.planner
        return int(
            self.roi.base_free.nbytes + self.current_free.nbytes
            + sys.getsizeof(planner.g) + sys.getsizeof(planner.rhs)
            + sys.getsizeof(planner._open) + sys.getsizeof(planner._queued_keys)
            + len(planner.g) * 96 + len(planner.rhs) * 96
            + len(planner._open) * 88 + len(planner._queued_keys) * 96
            + len(self.dynamic_blocked_local) * 72
        )


__all__ = [
    "Cell", "CorridorBinding", "CorridorROI", "CornerSafeDStarLite",
    "GridAStarResult", "L2PlanResult", "PersistentCorridorDStar",
    "deterministic_grid_astar",
]
