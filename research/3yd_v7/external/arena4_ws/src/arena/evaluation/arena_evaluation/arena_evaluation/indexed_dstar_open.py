"""Auditable D* Lite OPEN implementations for the 2D-V2 r1 tail study.

The production :class:`GraphDStarLite` remains unchanged.  This module keeps
the exact graph, heuristic, cost and termination semantics while exposing a
unique-entry indexed heap and the counters needed to separate queue overhead
from mandatory consistency propagation.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .graph_dstar_lite import GraphDStarLite, GraphDStarSearchStats, GraphEdge, INF


LexicographicKey = Tuple[float, float]


@dataclass(frozen=True)
class IndexedOpenStats:
    insertions: int = 0
    updates: int = 0
    removals: int = 0
    pops: int = 0
    sift_operations: int = 0
    peak_size: int = 0


class IndexedLexicographicPriorityQueue:
    """Binary min-heap with at most one live entry per integer state."""

    def __init__(self) -> None:
        self._heap: List[Tuple[float, float, int, int]] = []
        self._positions: Dict[int, int] = {}
        self._serial = 0
        self.insertions = 0
        self.updates = 0
        self.removals = 0
        self.pops = 0
        self.sift_operations = 0
        self.peak_size = 0

    def __len__(self) -> int:
        return len(self._heap)

    def __contains__(self, node: object) -> bool:
        return isinstance(node, int) and node in self._positions

    @staticmethod
    def _ordered(entry: Tuple[float, float, int, int]) -> Tuple[float, float, int]:
        return entry[0], entry[1], entry[2]

    def _swap(self, first: int, second: int) -> None:
        self._heap[first], self._heap[second] = self._heap[second], self._heap[first]
        self._positions[self._heap[first][3]] = first
        self._positions[self._heap[second][3]] = second
        self.sift_operations += 1

    def _sift_up(self, index: int) -> int:
        while index > 0:
            parent = (index - 1) // 2
            if self._ordered(self._heap[parent]) <= self._ordered(self._heap[index]):
                break
            self._swap(parent, index)
            index = parent
        return index

    def _sift_down(self, index: int) -> int:
        size = len(self._heap)
        while True:
            left = 2 * index + 1
            right = left + 1
            best = index
            if left < size and self._ordered(self._heap[left]) < self._ordered(self._heap[best]):
                best = left
            if right < size and self._ordered(self._heap[right]) < self._ordered(self._heap[best]):
                best = right
            if best == index:
                return index
            self._swap(index, best)
            index = best

    def key_for(self, node: int) -> Optional[LexicographicKey]:
        index = self._positions.get(int(node))
        if index is None:
            return None
        entry = self._heap[index]
        return float(entry[0]), float(entry[1])

    def insert_or_update(self, node: int, key: LexicographicKey) -> bool:
        node = int(node)
        key = (float(key[0]), float(key[1]))
        index = self._positions.get(node)
        if index is not None:
            old = self._heap[index]
            if (old[0], old[1]) == key:
                return False
            self._serial += 1
            self._heap[index] = (key[0], key[1], self._serial, node)
            self.updates += 1
            moved = self._sift_up(index)
            self._sift_down(moved)
            return True
        self._serial += 1
        self._heap.append((key[0], key[1], self._serial, node))
        index = len(self._heap) - 1
        self._positions[node] = index
        self.insertions += 1
        self._sift_up(index)
        self.peak_size = max(self.peak_size, len(self._heap))
        return True

    def remove(self, node: int) -> bool:
        index = self._positions.pop(int(node), None)
        if index is None:
            return False
        last = self._heap.pop()
        self.removals += 1
        if index < len(self._heap):
            self._heap[index] = last
            self._positions[last[3]] = index
            moved = self._sift_up(index)
            self._sift_down(moved)
        return True

    def peek_key(self) -> LexicographicKey:
        if not self._heap:
            return INF, INF
        return float(self._heap[0][0]), float(self._heap[0][1])

    def pop_min(self) -> Tuple[LexicographicKey, int]:
        if not self._heap:
            raise IndexError("pop from an empty indexed OPEN")
        first = self._heap[0]
        self.remove(first[3])
        self.pops += 1
        return (float(first[0]), float(first[1])), int(first[3])

    def clear(self) -> None:
        self._heap.clear()
        self._positions.clear()

    def entries(self) -> List[Tuple[LexicographicKey, int]]:
        return [((float(a), float(b)), int(node)) for a, b, _serial, node in self._heap]

    def stats(self) -> IndexedOpenStats:
        return IndexedOpenStats(
            self.insertions, self.updates, self.removals, self.pops,
            self.sift_operations, self.peak_size,
        )

    def memory_bytes(self) -> int:
        return int(
            sys.getsizeof(self._heap)
            + sum(sys.getsizeof(item) for item in self._heap)
            + sys.getsizeof(self._positions)
            + sum(sys.getsizeof(k) + sys.getsizeof(v) for k, v in self._positions.items())
        )


class InstrumentedGraphDStarLite(GraphDStarLite):
    """Frozen lazy-heap baseline plus allocation/key observability."""

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.key_calculation_count = 0
        self.tuple_allocation_proxy_count = 0
        self.update_batch_candidate_count = 0
        self.update_batch_unique_count = 0
        super().__init__(*args, **kwargs)

    def _calculate_key(self, node: int) -> LexicographicKey:
        self.key_calculation_count += 1
        self.tuple_allocation_proxy_count += 1
        return super()._calculate_key(node)

    def _push(self, node: int) -> None:
        before = self.queue_push_count if hasattr(self, "queue_push_count") else 0
        super()._push(node)
        if hasattr(self, "queue_push_count") and self.queue_push_count > before:
            self.tuple_allocation_proxy_count += 1

    def update_edges(
        self, changed_edge_ids: Iterable[str], *,
        statuses: Optional[Mapping[str, str]] = None,
        costs: Optional[Mapping[str, float]] = None,
    ) -> int:
        changed = {str(edge_id) for edge_id in changed_edge_ids}
        candidates: List[int] = []
        for edge_id in sorted(changed):
            edge = self.edges.get(edge_id)
            if edge is None:
                continue
            candidates.extend((int(edge.source), int(edge.target)))
            candidates.extend(int(node) for node, _edge, _reverse in self._predecessor_nodes(edge.source))
            candidates.extend(int(node) for node, _edge, _reverse in self._predecessor_nodes(edge.target))
        self.update_batch_candidate_count += len(candidates)
        self.update_batch_unique_count += len(set(candidates))
        return super().update_edges(changed, statuses=statuses, costs=costs)


class IndexedGraphDStarLite(GraphDStarLite):
    """D* Lite with a unique-entry indexed OPEN and unchanged search rules."""

    def __init__(
        self, nodes: Iterable[int], edges: Iterable[GraphEdge], start: int,
        goal: int, *, edge_status: Optional[Mapping[str, str]] = None,
        edge_cost_override: Optional[Mapping[str, float]] = None,
        directed: bool = False,
    ) -> None:
        # The base constructor establishes graph/state semantics.  Its one
        # initial lazy entry is then replaced before any search is run.
        self.key_calculation_count = 0
        self.tuple_allocation_proxy_count = 0
        super().__init__(
            nodes, edges, start, goal, edge_status=edge_status,
            edge_cost_override=edge_cost_override, directed=directed,
        )
        self._indexed_open = IndexedLexicographicPriorityQueue()
        self._open.clear()
        self._queued_keys.clear()
        self.queue_push_count = 0
        self.queue_pop_count = 0
        self.key_calculation_count = 0
        self.tuple_allocation_proxy_count = 0
        self.update_batch_candidate_count = 0
        self.update_batch_unique_count = 0
        self._push(self.goal)

    def _calculate_key(self, node: int) -> LexicographicKey:
        self.key_calculation_count += 1
        self.tuple_allocation_proxy_count += 1
        return super()._calculate_key(node)

    def _push(self, node: int) -> None:
        # During GraphDStarLite.__init__, the indexed queue is not installed.
        if not hasattr(self, "_indexed_open"):
            return super()._push(node)
        node = int(node)
        if self._value(self.g, node) == self._value(self.rhs, node):
            self._indexed_open.remove(node)
            self._queued_keys.pop(node, None)
            return
        key = self._calculate_key(node)
        if self._indexed_open.insert_or_update(node, key):
            self.queue_push_count += 1
            self.tuple_allocation_proxy_count += 1
        self._queued_keys[node] = key

    def set_goal(self, goal: int) -> None:
        new_goal = self._validate_node(goal)
        if new_goal == self.goal:
            return
        self.goal = new_goal
        self.g.clear()
        self.rhs = {new_goal: 0.0}
        self._indexed_open.clear()
        self._queued_keys.clear()
        self._push(new_goal)
        self.update_count += 1

    def reinitialize(self, start: Optional[int] = None, goal: Optional[int] = None) -> None:
        if start is not None:
            self.start = self._validate_node(start)
            self.last_start = self.start
        if goal is not None:
            self.goal = self._validate_node(goal)
        self.km = 0.0
        self.g.clear()
        self.rhs = {self.goal: 0.0}
        self._indexed_open.clear()
        self._queued_keys.clear()
        self._push(self.goal)
        self.update_count += 1

    def update_edges(
        self, changed_edge_ids: Iterable[str], *,
        statuses: Optional[Mapping[str, str]] = None,
        costs: Optional[Mapping[str, float]] = None,
    ) -> int:
        changed = {str(edge_id) for edge_id in changed_edge_ids}
        if statuses:
            self.edge_status.update({str(k): str(v) for k, v in statuses.items() if str(k) in self.edges})
        if costs:
            self.edge_cost_override.update({str(k): float(v) for k, v in costs.items() if str(k) in self.edges})
        candidates: List[int] = []
        for edge_id in sorted(changed):
            edge = self.edges.get(edge_id)
            if edge is None:
                continue
            candidates.extend((int(edge.source), int(edge.target)))
            candidates.extend(int(node) for node, _edge, _reverse in self._predecessor_nodes(edge.source))
            candidates.extend(int(node) for node, _edge, _reverse in self._predecessor_nodes(edge.target))
        affected = sorted(set(candidates))
        self.update_batch_candidate_count += len(candidates)
        self.update_batch_unique_count += len(affected)
        for node in affected:
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
        started = time.monotonic_ns()
        initial_queue_size = len(self._indexed_open)
        pushes_before = self.queue_push_count
        updates_before = self.update_vertex_count
        rhs_changes_before = self.rhs_change_count
        g_changes_before = self.g_change_count
        predecessor_before = self.predecessor_propagation_count
        expanded = generated = pops = 0
        timeout = False
        budget_reason = ""
        peak_open_size = len(self._indexed_open)
        deadline = None if timeout_s is None else time.monotonic() + max(0.0, float(timeout_s))
        while len(self._indexed_open):
            peak_open_size = max(peak_open_size, len(self._indexed_open))
            top_key = self._indexed_open.peek_key()
            start_key = self._calculate_key(self.start)
            if not (top_key < start_key or self._value(self.rhs, self.start) != self._value(self.g, self.start)):
                break
            if deadline is not None and time.monotonic() >= deadline:
                timeout = True; budget_reason = "WALL_TIME_BUDGET"; break
            if max_expansions is not None and expanded >= max(0, int(max_expansions)):
                timeout = True; budget_reason = "EXPANSION_BUDGET"; break
            if max_queue_pops is not None and pops >= max(0, int(max_queue_pops)):
                timeout = True; budget_reason = "OPEN_POP_BUDGET"; break
            if max_update_vertex is not None and self.update_vertex_count - updates_before >= max(0, int(max_update_vertex)):
                timeout = True; budget_reason = "UPDATE_VERTEX_BUDGET"; break
            if max_open_size is not None and len(self._indexed_open) > max(0, int(max_open_size)):
                timeout = True; budget_reason = "OPEN_SIZE_BUDGET"; break
            if max_inconsistent_states is not None and len(self._queued_keys) > max(0, int(max_inconsistent_states)):
                timeout = True; budget_reason = "INCONSISTENT_STATE_BUDGET"; break
            old_key, node = self._indexed_open.pop_min()
            self._queued_keys.pop(node, None)
            pops += 1
            self.queue_pop_count += 1
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
            initial_queue_size=initial_queue_size, final_queue_size=len(self._indexed_open),
            update_vertex_count=self.update_vertex_count - updates_before,
            search_time_ms=float(elapsed), timeout_triggered=timeout,
            no_path=no_path, budget_triggered=timeout, budget_reason=budget_reason,
            converged=not timeout, stale_queue_entries=0,
            peak_open_size=peak_open_size,
            g_changed_nodes=self.g_change_count - g_changes_before,
            rhs_changed_nodes=self.rhs_change_count - rhs_changes_before,
            predecessor_propagations=self.predecessor_propagation_count - predecessor_before,
        )
        return self.last_stats

    def state_snapshot(self) -> Dict[str, object]:
        return {
            "g": {str(node): value for node, value in self.g.items() if math.isfinite(value)},
            "rhs": {str(node): value for node, value in self.rhs.items() if math.isfinite(value)},
            "OPEN": [[key[0], key[1], node] for key, node in self._indexed_open.entries()],
            "start_node": int(self.start), "goal_node": int(self.goal),
            "last_start_node": int(self.last_start), "km": float(self.km),
            "edge_cost_version": int(self.update_count),
            "priority_queue_size": len(self._indexed_open),
        }

    def indexed_open_memory_bytes(self) -> int:
        return self._indexed_open.memory_bytes()


class BatchIndexedGraphDStarLite(IndexedGraphDStarLite):
    """Indexed D* with immutable per-edge affected-node batches.

    The frozen baseline already deduplicates the initial ``update_vertex``
    set.  This variant therefore cannot honestly claim an algorithmic call
    reduction; it removes repeated predecessor discovery and exposes the
    naive-versus-unique candidate counts for the ablation.
    """

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._edge_update_nodes: Dict[str, Tuple[int, ...]] = {}
        for edge_id, edge in self.edges.items():
            values = {int(edge.source), int(edge.target)}
            values.update(int(node) for node, _candidate, _reverse in self._predecessor_nodes(edge.source))
            values.update(int(node) for node, _candidate, _reverse in self._predecessor_nodes(edge.target))
            self._edge_update_nodes[str(edge_id)] = tuple(sorted(values))
        self.batch_static_memory_bytes = int(
            sys.getsizeof(self._edge_update_nodes)
            + sum(
                sys.getsizeof(key) + sys.getsizeof(value)
                + sum(sys.getsizeof(node) for node in value)
                for key, value in self._edge_update_nodes.items()
            )
        )

    def update_edges(
        self, changed_edge_ids: Iterable[str], *,
        statuses: Optional[Mapping[str, str]] = None,
        costs: Optional[Mapping[str, float]] = None,
    ) -> int:
        changed = {str(edge_id) for edge_id in changed_edge_ids}
        if statuses:
            self.edge_status.update({str(k): str(v) for k, v in statuses.items() if str(k) in self.edges})
        if costs:
            self.edge_cost_override.update({str(k): float(v) for k, v in costs.items() if str(k) in self.edges})
        affected = set()
        candidate_count = 0
        for edge_id in sorted(changed):
            nodes = self._edge_update_nodes.get(edge_id, ())
            candidate_count += len(nodes)
            affected.update(nodes)
        self.update_batch_candidate_count += candidate_count
        self.update_batch_unique_count += len(affected)
        for node in sorted(affected):
            self.update_vertex(node)
        if changed:
            self.update_count += 1
        return len(affected)


def exact_start_goal_connected(
    nodes: Sequence[int], adjacency: Mapping[int, Sequence[Tuple[int, GraphEdge, bool]]],
    start: int, goal: int, statuses: Mapping[str, str],
) -> Tuple[bool, int, int]:
    """Exact deterministic reachability under the current dynamic overlay."""
    del nodes  # adjacency already defines the immutable topology universe.
    start, goal = int(start), int(goal)
    pending = [start]
    seen = {start}
    edge_checks = 0
    while pending:
        node = pending.pop()
        if node == goal:
            return True, len(seen), edge_checks
        for successor, edge, _reverse in adjacency.get(node, ()):
            edge_checks += 1
            status = str(statuses.get(str(edge.edge_id), GraphDStarLite.AVAILABLE))
            if status in {GraphDStarLite.BLOCKED, GraphDStarLite.RECOVERING}:
                continue
            successor = int(successor)
            if successor not in seen:
                seen.add(successor)
                pending.append(successor)
    return False, len(seen), edge_checks


__all__ = [
    "BatchIndexedGraphDStarLite", "IndexedGraphDStarLite",
    "IndexedLexicographicPriorityQueue",
    "IndexedOpenStats", "InstrumentedGraphDStarLite",
    "exact_start_goal_connected",
]
