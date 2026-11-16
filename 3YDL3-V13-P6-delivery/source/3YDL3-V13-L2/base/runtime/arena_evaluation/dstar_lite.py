"""Deterministic, incremental D* Lite search on a 2-D occupancy grid.

The implementation is deliberately independent of ROS and Nav2.  A planner
instance owns the search state for one corridor and can update only changed
cells when a dynamic snapshot arrives.  L1 and L3 callers therefore do not
need to know about ``g``/``rhs`` bookkeeping.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from itertools import count
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


Cell = Tuple[int, int]
INF = float("inf")


@dataclass(frozen=True)
class DStarSearchStats:
    """Counters for the most recent ``compute_shortest_path`` call."""

    expanded_nodes: int = 0
    generated_nodes: int = 0
    queue_pops: int = 0
    queue_pushes: int = 0
    initial_queue_size: int = 0
    final_queue_size: int = 0
    update_vertex_count: int = 0
    search_time_ms: float = 0.0
    timeout_triggered: bool = False
    no_path: bool = False


class DStarLite:
    """A grid D* Lite planner with persistent ``g`` and ``rhs`` values.

    Cells use image coordinates ``(row, column)``.  Traversability and costs
    are supplied as arrays so callers can atomically replace a dynamic
    overlay without touching the immutable static map.
    """

    def __init__(
        self,
        traversable: np.ndarray,
        start: Cell,
        goal: Cell,
        *,
        cost_map: Optional[np.ndarray] = None,
        allow_diagonal: bool = True,
    ) -> None:
        self.allow_diagonal = bool(allow_diagonal)
        self._set_arrays(traversable, cost_map)
        self.start: Cell = self._validate_cell(start)
        self.goal: Cell = self._validate_cell(goal)
        self.last_start: Cell = self.start
        self.km = 0.0
        self.g: Dict[Cell, float] = {}
        self.rhs: Dict[Cell, float] = {self.goal: 0.0}
        self._open: List[Tuple[float, float, int, Cell]] = []
        self._queued_keys: Dict[Cell, Tuple[float, float]] = {}
        self._counter = count()
        self.queue_push_count = 0
        self.queue_pop_count = 0
        self.update_vertex_count = 0
        self._push(self.goal)
        self.last_stats = DStarSearchStats()
        self.total_expanded_nodes = 0
        self.total_generated_nodes = 1
        self.update_count = 0

    @property
    def shape(self) -> Tuple[int, int]:
        return self.traversable.shape

    def _set_arrays(self, traversable: np.ndarray, cost_map: Optional[np.ndarray]) -> None:
        mask = np.asarray(traversable, dtype=bool)
        if mask.ndim != 2 or min(mask.shape) <= 0:
            raise ValueError("traversable must be a non-empty 2-D array")
        self.traversable = mask.copy()
        if cost_map is None:
            self.cost_map = np.ones(mask.shape, dtype=np.float64)
        else:
            costs = np.asarray(cost_map, dtype=np.float64)
            if costs.shape != mask.shape:
                raise ValueError("cost_map shape must match traversable")
            if np.any(~np.isfinite(costs)) or np.any(costs <= 0.0):
                raise ValueError("cost_map must contain finite positive values")
            self.cost_map = costs.copy()

    def _validate_cell(self, cell: Cell) -> Cell:
        row, col = int(cell[0]), int(cell[1])
        if not (0 <= row < self.shape[0] and 0 <= col < self.shape[1]):
            raise ValueError(f"cell outside grid: {(row, col)}")
        return row, col

    def _heuristic(self, first: Cell, second: Cell) -> float:
        return math.hypot(float(first[0] - second[0]), float(first[1] - second[1]))

    def _neighbors(self, cell: Cell) -> Iterable[Cell]:
        row, col = cell
        offsets = (
            (-1, -1), (-1, 0), (-1, 1), (0, -1),
            (0, 1), (1, -1), (1, 0), (1, 1),
        ) if self.allow_diagonal else ((-1, 0), (0, -1), (0, 1), (1, 0))
        for drow, dcol in offsets:
            candidate = (row + drow, col + dcol)
            if 0 <= candidate[0] < self.shape[0] and 0 <= candidate[1] < self.shape[1]:
                yield candidate

    def _edge_cost(self, first: Cell, second: Cell) -> float:
        if not self.traversable[first] or not self.traversable[second]:
            return INF
        step = math.sqrt(2.0) if first[0] != second[0] and first[1] != second[1] else 1.0
        return step * 0.5 * (float(self.cost_map[first]) + float(self.cost_map[second]))

    def _value(self, values: Mapping[Cell, float], cell: Cell) -> float:
        return float(values.get(cell, INF))

    def _calculate_key(self, cell: Cell) -> Tuple[float, float]:
        best = min(self._value(self.g, cell), self._value(self.rhs, cell))
        return best + self._heuristic(self.start, cell) + self.km, best

    def _push(self, cell: Cell) -> None:
        if self._value(self.g, cell) == self._value(self.rhs, cell):
            self._queued_keys.pop(cell, None)
            return
        key = self._calculate_key(cell)
        current = self._queued_keys.get(cell)
        if current == key:
            return
        self._queued_keys[cell] = key
        heapq.heappush(self._open, (key[0], key[1], next(self._counter), cell))
        self.queue_push_count += 1

    def _predecessors(self, cell: Cell) -> Iterable[Cell]:
        # The grid is undirected, so predecessors are the same as successors.
        return self._neighbors(cell)

    def update_vertex(self, cell: Cell) -> None:
        self.update_vertex_count += 1
        cell = self._validate_cell(cell)
        if cell != self.goal:
            best = INF
            for successor in self._neighbors(cell):
                best = min(best, self._edge_cost(cell, successor) + self._value(self.g, successor))
            self.rhs[cell] = best
        self._push(cell)

    def set_start(self, start: Cell) -> None:
        new_start = self._validate_cell(start)
        self.km += self._heuristic(self.last_start, new_start)
        self.last_start = new_start
        self.start = new_start

    def set_goal(self, goal: Cell) -> None:
        new_goal = self._validate_cell(goal)
        if new_goal == self.goal:
            return
        # A changed goal invalidates the reverse search tree.  This is a
        # deliberate bounded reset used for a new A-B repair segment.
        self.goal = new_goal
        self.g.clear()
        self.rhs = {new_goal: 0.0}
        self._open.clear()
        self._queued_keys.clear()
        self._push(new_goal)
        self.update_count += 1

    def update_cells(
        self,
        changed_cells: Iterable[Cell],
        *,
        traversable: Optional[np.ndarray] = None,
        cost_map: Optional[np.ndarray] = None,
    ) -> int:
        """Apply a dynamic update and repair only affected graph vertices."""
        if traversable is not None or cost_map is not None:
            new_mask = self.traversable if traversable is None else np.asarray(traversable, dtype=bool)
            new_cost = self.cost_map if cost_map is None else np.asarray(cost_map, dtype=np.float64)
            if new_mask.shape != self.shape or new_cost.shape != self.shape:
                raise ValueError("dynamic arrays must match planner grid")
            self.traversable = new_mask.copy()
            self.cost_map = new_cost.copy()
        affected = {self._validate_cell(cell) for cell in changed_cells}
        expanded = set(affected)
        for cell in affected:
            expanded.update(self._neighbors(cell))
        for cell in sorted(expanded):
            self.update_vertex(cell)
        self.update_count += 1
        return len(expanded)

    def compute_shortest_path(
        self,
        *,
        max_expansions: Optional[int] = None,
        timeout_s: Optional[float] = None,
    ) -> DStarSearchStats:
        started = time.monotonic_ns()
        initial_queue_size = len(self._open)
        pushes_before = self.queue_push_count
        pops_before = self.queue_pop_count
        updates_before = self.update_vertex_count
        expanded = 0
        generated = 0
        pops = 0
        timeout = False
        deadline = None if timeout_s is None else time.monotonic() + max(0.0, float(timeout_s))
        while self._open:
            top_key = (self._open[0][0], self._open[0][1])
            start_key = self._calculate_key(self.start)
            if not (top_key < start_key or self._value(self.rhs, self.start) != self._value(self.g, self.start)):
                break
            if deadline is not None and time.monotonic() >= deadline:
                timeout = True
                break
            if max_expansions is not None and expanded >= max(0, int(max_expansions)):
                timeout = True
                break
            key_old_1, key_old_2, _counter, cell = heapq.heappop(self._open)
            pops += 1
            self.queue_pop_count += 1
            key_old = (key_old_1, key_old_2)
            if self._queued_keys.get(cell) != key_old:
                continue
            self._queued_keys.pop(cell, None)
            key_new = self._calculate_key(cell)
            if key_old < key_new:
                self._push(cell)
                continue
            if self._value(self.g, cell) > self._value(self.rhs, cell):
                self.g[cell] = self._value(self.rhs, cell)
                expanded += 1
                for predecessor in self._predecessors(cell):
                    self.update_vertex(predecessor)
            else:
                self.g[cell] = INF
                expanded += 1
                self.update_vertex(cell)
                for predecessor in self._predecessors(cell):
                    self.update_vertex(predecessor)
            generated += 1
        elapsed = (time.monotonic_ns() - started) / 1.0e6
        no_path = self._value(self.g, self.start) == INF
        self.last_stats = DStarSearchStats(
            expanded_nodes=expanded, generated_nodes=generated, queue_pops=pops,
            queue_pushes=self.queue_push_count - pushes_before,
            initial_queue_size=initial_queue_size,
            final_queue_size=len(self._open),
            update_vertex_count=self.update_vertex_count - updates_before,
            search_time_ms=float(elapsed), timeout_triggered=timeout, no_path=no_path,
        )
        self.total_expanded_nodes += expanded
        self.total_generated_nodes += generated
        return self.last_stats

    def extract_path(self, *, max_length: Optional[int] = None) -> Optional[List[Cell]]:
        """Return a deterministic shortest path, or ``None`` when unavailable."""
        if self._value(self.g, self.start) == INF:
            return None
        limit = max_length or (self.shape[0] * self.shape[1] * 2)
        path = [self.start]
        current = self.start
        visited = {current}
        while current != self.goal and len(path) < limit:
            choices = []
            for successor in self._neighbors(current):
                cost = self._edge_cost(current, successor)
                value = cost + self._value(self.g, successor)
                if math.isfinite(value):
                    choices.append((value, successor))
            if not choices:
                return None
            _, next_cell = min(choices, key=lambda item: (item[0], item[1][0], item[1][1]))
            if next_cell in visited:
                return None
            path.append(next_cell)
            visited.add(next_cell)
            current = next_cell
        return path if current == self.goal else None

    def reinitialize(self, start: Optional[Cell] = None, goal: Optional[Cell] = None) -> None:
        """Clear incremental state while preserving the current grid."""
        if start is not None:
            self.start = self._validate_cell(start)
            self.last_start = self.start
        if goal is not None:
            self.goal = self._validate_cell(goal)
        self.km = 0.0
        self.g.clear()
        self.rhs = {self.goal: 0.0}
        self._open.clear()
        self._queued_keys.clear()
        self._push(self.goal)
        self.update_count += 1

    def state_snapshot(self) -> Dict[str, object]:
        """Return the incremental state for audit or a corridor cache."""
        return {
            "start": list(self.start),
            "last_start": list(self.last_start),
            "goal": list(self.goal),
            "km": float(self.km),
            "g": {f"{cell[0]},{cell[1]}": value for cell, value in self.g.items() if math.isfinite(value)},
            "rhs": {f"{cell[0]},{cell[1]}": value for cell, value in self.rhs.items() if math.isfinite(value)},
            "priority_queue_size": len(self._open),
        }


__all__ = ["Cell", "DStarLite", "DStarSearchStats", "INF"]
