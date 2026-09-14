"""Deterministic D* Lite search over a refined topology graph.

Unlike :mod:`dstar_lite`, this module never treats occupancy-grid cells as
search states.  Its states are stable refined-topology node ids and its
dynamic updates touch only changed edge endpoints and their predecessors.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from itertools import count
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


INF = float("inf")


@dataclass(frozen=True)
class GraphEdge:
    edge_id: str
    source: int
    target: int
    length_m: float
    static_cost: float = 0.0
    min_clearance_m: float = 0.0
    turn_penalty: float = 0.0
    # Topology edges are normally bidirectional. Endpoint attachment edges
    # can opt into explicit one-way semantics.
    bidirectional: bool = True


@dataclass(frozen=True)
class GraphDStarSearchStats:
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
    budget_triggered: bool = False
    budget_reason: str = ""
    converged: bool = True
    stale_queue_entries: int = 0
    peak_open_size: int = 0
    g_changed_nodes: int = 0
    rhs_changed_nodes: int = 0
    predecessor_propagations: int = 0


class GraphDStarLite:
    """A D* Lite implementation whose states are graph node ids.

    Edges are undirected by default: callers provide one edge and the class
    creates both directional adjacency entries.  Dynamic state is kept in
    ``edge_status`` and ``edge_cost_override`` so the static graph remains
    immutable and reusable across snapshots and queries.
    """

    AVAILABLE = "AVAILABLE"
    PENALIZED = "PENALIZED"
    BLOCKED_PENDING = "BLOCKED_PENDING"
    BLOCKED = "BLOCKED"
    RECOVERING = "RECOVERING"

    def __init__(
        self,
        nodes: Iterable[int],
        edges: Iterable[GraphEdge],
        start: int,
        goal: int,
        *,
        edge_status: Optional[Mapping[str, str]] = None,
        edge_cost_override: Optional[Mapping[str, float]] = None,
        directed: bool = False,
    ) -> None:
        self.nodes = tuple(sorted({int(node) for node in nodes}))
        self.node_set = set(self.nodes)
        self.edges = {str(edge.edge_id): edge for edge in edges}
        if not self.edges:
            raise ValueError("GraphDStarLite requires at least one edge")
        self.directed = bool(directed)
        self.adjacency: Dict[int, List[Tuple[int, GraphEdge, bool]]] = {node: [] for node in self.nodes}
        self._predecessors: Dict[int, List[Tuple[int, GraphEdge, bool]]] = {node: [] for node in self.nodes}
        for edge in self.edges.values():
            if edge.source not in self.node_set or edge.target not in self.node_set:
                raise ValueError(f"edge {edge.edge_id} references an unknown node")
            self.adjacency[edge.source].append((edge.target, edge, False))
            self._predecessors[edge.target].append((edge.source, edge, False))
            if not self.directed and bool(edge.bidirectional):
                self.adjacency[edge.target].append((edge.source, edge, True))
                self._predecessors[edge.source].append((edge.target, edge, True))
        for values in self.adjacency.values():
            values.sort(key=lambda item: (int(item[0]), str(item[1].edge_id), bool(item[2])))
        for values in self._predecessors.values():
            values.sort(key=lambda item: (int(item[0]), str(item[1].edge_id), bool(item[2])))
        self.edge_status: Dict[str, str] = {
            str(edge_id): self.AVAILABLE for edge_id in self.edges
        }
        if edge_status:
            self.edge_status.update({str(key): str(value) for key, value in edge_status.items()})
        self.edge_cost_override: Dict[str, float] = {}
        if edge_cost_override:
            self.edge_cost_override.update({str(key): float(value) for key, value in edge_cost_override.items()})
        self.start = self._validate_node(start)
        self.goal = self._validate_node(goal)
        self.last_start = self.start
        self.km = 0.0
        self.g: Dict[int, float] = {}
        self.rhs: Dict[int, float] = {self.goal: 0.0}
        self._open: List[Tuple[float, float, int, int]] = []
        self._queued_keys: Dict[int, Tuple[float, float]] = {}
        self._counter = count()
        self.queue_push_count = 0
        self.queue_pop_count = 0
        self.update_vertex_count = 0
        self.rhs_change_count = 0
        self.g_change_count = 0
        self.predecessor_propagation_count = 0
        self.update_count = 0
        self._push(self.goal)
        self.last_stats = GraphDStarSearchStats()

    def _validate_node(self, node: int) -> int:
        value = int(node)
        if value not in self.node_set:
            raise ValueError(f"unknown topology node: {value}")
        return value

    @staticmethod
    def _value(values: Mapping[int, float], node: int) -> float:
        return float(values.get(int(node), INF))

    def _heuristic(self, first: int, second: int) -> float:
        # Topology callers may attach coordinates through ``node_positions``.
        # Integer-id distance is a conservative deterministic fallback.
        # A virtual endpoint/root has a negative id and may intentionally have
        # no coordinate.  The former 1.0 fallback could exceed a sub-metre
        # ranking edge, breaking heuristic consistency and stopping an
        # incremental cost-increase repair before the start state was updated.
        if int(first) < 0 or int(second) < 0:
            return 0.0
        if hasattr(self, "node_positions"):
            try:
                ax, ay = self.node_positions[int(first)]
                bx, by = self.node_positions[int(second)]
                return math.hypot(float(ax) - float(bx), float(ay) - float(by))
            except (KeyError, TypeError, ValueError):
                pass
        return 0.0 if int(first) == int(second) else 1.0

    def _calculate_key(self, node: int) -> Tuple[float, float]:
        best = min(self._value(self.g, node), self._value(self.rhs, node))
        return best + self._heuristic(self.start, node) + self.km, best

    def _push(self, node: int) -> None:
        if self._value(self.g, node) == self._value(self.rhs, node):
            self._queued_keys.pop(int(node), None)
            return
        key = self._calculate_key(int(node))
        current = self._queued_keys.get(int(node))
        if current == key:
            return
        self._queued_keys[int(node)] = key
        heapq.heappush(self._open, (key[0], key[1], next(self._counter), int(node)))
        self.queue_push_count += 1

    def _edge_cost(self, edge: GraphEdge) -> float:
        status = self.edge_status.get(str(edge.edge_id), self.AVAILABLE)
        # RECOVERING remains unavailable until the second clear observation
        # promotes it to AVAILABLE.  Treating it as a static-cost edge would
        # defeat recovery hysteresis and could expose a route too early.
        if status in {self.BLOCKED, self.RECOVERING}:
            return INF
        value = self.edge_cost_override.get(str(edge.edge_id), edge.length_m + edge.static_cost + edge.turn_penalty)
        if not math.isfinite(value) or value <= 0.0:
            return INF
        if status == self.PENALIZED or status == self.BLOCKED_PENDING:
            value *= 10.0 if status == self.PENALIZED else 3.0
        return float(value)

    def _successors(self, node: int) -> Iterable[Tuple[int, GraphEdge, bool]]:
        return self.adjacency.get(int(node), ())

    def _predecessor_nodes(self, node: int) -> Iterable[Tuple[int, GraphEdge, bool]]:
        return self._predecessors.get(int(node), ())

    def update_vertex(self, node: int) -> None:
        node = self._validate_node(node)
        self.update_vertex_count += 1
        if node != self.goal:
            best = INF
            for successor, edge, _reverse in self._successors(node):
                best = min(best, self._edge_cost(edge) + self._value(self.g, successor))
            if best != self._value(self.rhs, node):
                self.rhs_change_count += 1
            self.rhs[node] = best
        self._push(node)

    def set_start(self, start: int) -> None:
        new_start = self._validate_node(start)
        self.km += self._heuristic(self.last_start, new_start)
        self.last_start = new_start
        self.start = new_start

    def set_goal(self, goal: int) -> None:
        new_goal = self._validate_node(goal)
        if new_goal == self.goal:
            return
        self.goal = new_goal
        self.g.clear()
        self.rhs = {new_goal: 0.0}
        self._open.clear()
        self._queued_keys.clear()
        self._push(new_goal)
        self.update_count += 1

    def update_edges(
        self,
        changed_edge_ids: Iterable[str],
        *,
        statuses: Optional[Mapping[str, str]] = None,
        costs: Optional[Mapping[str, float]] = None,
    ) -> int:
        """Update only endpoints and predecessors of changed graph edges."""
        changed = {str(edge_id) for edge_id in changed_edge_ids}
        if statuses:
            self.edge_status.update({str(key): str(value) for key, value in statuses.items() if str(key) in self.edges})
        if costs:
            self.edge_cost_override.update({str(key): float(value) for key, value in costs.items() if str(key) in self.edges})
        affected = set()
        for edge_id in sorted(changed):
            edge = self.edges.get(edge_id)
            if edge is None:
                continue
            affected.update((int(edge.source), int(edge.target)))
            for node, _candidate, _reverse in self._predecessor_nodes(edge.source):
                affected.add(int(node))
            for node, _candidate, _reverse in self._predecessor_nodes(edge.target):
                affected.add(int(node))
        for node in sorted(affected):
            self.update_vertex(node)
        if changed:
            self.update_count += 1
        return len(affected)

    def compute_shortest_path(
        self, *, timeout_s: Optional[float] = None,
        max_expansions: Optional[int] = None,
        max_queue_pops: Optional[int] = None,
        max_update_vertex: Optional[int] = None,
        max_open_size: Optional[int] = None,
        max_inconsistent_states: Optional[int] = None,
    ) -> GraphDStarSearchStats:
        """Repair the shortest path, optionally stopping at auditable limits.

        A caller must check ``converged`` before extracting or returning a
        route.  Budget termination deliberately leaves the incremental state
        in-place so it can either be resumed from the same snapshot or
        discarded and rebuilt by the caller; it never pretends the partial
        state is a valid solution.
        """
        started = time.monotonic_ns()
        initial_queue_size = len(self._open)
        pushes_before = self.queue_push_count
        pops_before = self.queue_pop_count
        updates_before = self.update_vertex_count
        rhs_changes_before = self.rhs_change_count
        g_changes_before = self.g_change_count
        predecessor_before = self.predecessor_propagation_count
        expanded = generated = pops = 0
        timeout = False
        budget_reason = ""
        stale_entries = 0
        peak_open_size = len(self._open)
        deadline = None if timeout_s is None else time.monotonic() + max(0.0, float(timeout_s))
        while self._open:
            peak_open_size = max(peak_open_size, len(self._open))
            top_key = (self._open[0][0], self._open[0][1])
            start_key = self._calculate_key(self.start)
            if not (top_key < start_key or self._value(self.rhs, self.start) != self._value(self.g, self.start)):
                break
            if deadline is not None and time.monotonic() >= deadline:
                timeout = True
                budget_reason = "WALL_TIME_BUDGET"
                break
            if max_expansions is not None and expanded >= max(0, int(max_expansions)):
                timeout = True
                budget_reason = "EXPANSION_BUDGET"
                break
            if max_queue_pops is not None and pops >= max(0, int(max_queue_pops)):
                timeout = True
                budget_reason = "OPEN_POP_BUDGET"
                break
            if (max_update_vertex is not None
                    and self.update_vertex_count - updates_before >= max(0, int(max_update_vertex))):
                timeout = True
                budget_reason = "UPDATE_VERTEX_BUDGET"
                break
            if max_open_size is not None and len(self._open) > max(0, int(max_open_size)):
                timeout = True
                budget_reason = "OPEN_SIZE_BUDGET"
                break
            if (max_inconsistent_states is not None
                    and len(self._queued_keys) > max(0, int(max_inconsistent_states))):
                timeout = True
                budget_reason = "INCONSISTENT_STATE_BUDGET"
                break
            old_1, old_2, _counter, node = heapq.heappop(self._open)
            pops += 1
            self.queue_pop_count += 1
            old_key = (old_1, old_2)
            if self._queued_keys.get(node) != old_key:
                stale_entries += 1
                continue
            self._queued_keys.pop(node, None)
            new_key = self._calculate_key(node)
            if old_key < new_key:
                self._push(node)
                continue
            if self._value(self.g, node) > self._value(self.rhs, node):
                self.g[node] = self._value(self.rhs, node)
                self.g_change_count += 1
                expanded += 1
                for predecessor, _edge, _reverse in self._predecessor_nodes(node):
                    self.predecessor_propagation_count += 1
                    self.update_vertex(predecessor)
            else:
                self.g[node] = INF
                self.g_change_count += 1
                expanded += 1
                self.update_vertex(node)
                for predecessor, _edge, _reverse in self._predecessor_nodes(node):
                    self.predecessor_propagation_count += 1
                    self.update_vertex(predecessor)
            generated += 1
        elapsed = (time.monotonic_ns() - started) / 1.0e6
        no_path = self._value(self.g, self.start) == INF
        self.last_stats = GraphDStarSearchStats(
            expanded_nodes=expanded, generated_nodes=generated, queue_pops=pops,
            queue_pushes=self.queue_push_count - pushes_before,
            initial_queue_size=initial_queue_size, final_queue_size=len(self._open),
            update_vertex_count=self.update_vertex_count - updates_before,
            search_time_ms=float(elapsed), timeout_triggered=timeout, no_path=no_path,
            budget_triggered=timeout, budget_reason=budget_reason,
            converged=not timeout, stale_queue_entries=stale_entries,
            peak_open_size=peak_open_size,
            g_changed_nodes=self.g_change_count - g_changes_before,
            rhs_changed_nodes=self.rhs_change_count - rhs_changes_before,
            predecessor_propagations=(
                self.predecessor_propagation_count - predecessor_before
            ),
        )
        return self.last_stats

    def extract_path(self, *, max_length: Optional[int] = None) -> Optional[List[int]]:
        if self._value(self.g, self.start) == INF:
            return None
        limit = max_length or max(2, len(self.nodes) * 2)
        path = [int(self.start)]
        current = int(self.start)
        visited = {current}
        while current != int(self.goal) and len(path) < limit:
            choices = []
            for successor, edge, _reverse in self._successors(current):
                value = self._edge_cost(edge) + self._value(self.g, successor)
                if math.isfinite(value):
                    choices.append((value, int(successor), str(edge.edge_id)))
            if not choices:
                return None
            _, next_node, _edge_id = min(choices, key=lambda item: (item[0], item[1], item[2]))
            if next_node in visited:
                return None
            path.append(next_node)
            visited.add(next_node)
            current = next_node
        return path if current == int(self.goal) else None

    def reinitialize(self, start: Optional[int] = None, goal: Optional[int] = None) -> None:
        if start is not None:
            self.start = self._validate_node(start)
            self.last_start = self.start
        if goal is not None:
            self.goal = self._validate_node(goal)
        self.km = 0.0
        self.g.clear()
        self.rhs = {self.goal: 0.0}
        self._open.clear()
        self._queued_keys.clear()
        self._push(self.goal)
        self.update_count += 1

    def state_snapshot(self) -> Dict[str, object]:
        return {
            "g": {str(node): value for node, value in self.g.items() if math.isfinite(value)},
            "rhs": {str(node): value for node, value in self.rhs.items() if math.isfinite(value)},
            "OPEN": [[key[0], key[1], node] for key, node in ((item[:2], item[3]) for item in self._open)],
            "start_node": int(self.start), "goal_node": int(self.goal),
            "last_start_node": int(self.last_start), "km": float(self.km),
            "edge_cost_version": int(self.update_count),
            "priority_queue_size": len(self._open),
        }


__all__ = ["INF", "GraphEdge", "GraphDStarLite", "GraphDStarSearchStats"]
