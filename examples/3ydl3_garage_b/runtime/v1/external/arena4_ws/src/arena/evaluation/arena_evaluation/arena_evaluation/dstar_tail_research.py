"""Strictly paired L1 arms for the 2D-V2 r1 D* tail-latency study."""

from __future__ import annotations

import math
import hashlib
import resource
import struct
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple, Type

from . import dynamic_incremental_value as dynamic
from .graph_dstar_lite import GraphDStarLite, GraphDStarSearchStats, INF
from .indexed_dstar_open import (
    BatchIndexedGraphDStarLite,
    IndexedGraphDStarLite,
    InstrumentedGraphDStarLite,
    exact_start_goal_connected,
)


RESEARCH_ID = "2D-V2-r1-dstar-tail-research"
PARENT_ARCHITECTURE = "2D-V2-r0"
REFERENCE_ARCHITECTURE = "2D-V3-r0"
EXPERIMENT_KIND = "dstar_core_tail_latency"
STATUS = "research_only"
PROTOCOL_VERSION = "PLN-02-EXP-V1"

COLD_GRAPH_ASTAR = "cold_graph_astar"
BASELINE_DSTAR = "baseline_lazy_dstar"
INDEXED_DSTAR = "indexed_open_dstar"
INDEXED_BATCH_DSTAR = "indexed_batch_dstar"
INDEXED_BATCH_CONNECTIVITY = "indexed_batch_connectivity"
COMBO_DSTAR = "optimized_combo_dstar"
ARMS = (
    COLD_GRAPH_ASTAR, BASELINE_DSTAR, INDEXED_DSTAR,
    INDEXED_BATCH_DSTAR, INDEXED_BATCH_CONNECTIVITY, COMBO_DSTAR,
)


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


def _planner(template: dynamic.GraphTemplate, cls: Type[GraphDStarLite],
             statuses: Mapping[str, str]) -> GraphDStarLite:
    planner = cls(
        template.nodes, template.edges, template.start, template.goal,
        edge_status=statuses,
    )
    planner.node_positions = dict(template.node_positions)
    planner.state_representation = "original_topology_node_id"
    return planner


def _planner_memory_bytes(planner: GraphDStarLite) -> int:
    mappings = (
        planner.g, planner.rhs, planner._queued_keys, planner.edge_status,
        planner.edge_cost_override,
    )
    total = sys.getsizeof(planner._open)
    total += sum(sys.getsizeof(item) for item in planner._open)
    for mapping in mappings:
        total += sys.getsizeof(mapping)
        total += sum(sys.getsizeof(key) + sys.getsizeof(value) for key, value in mapping.items())
    if isinstance(planner, IndexedGraphDStarLite):
        total += planner.indexed_open_memory_bytes()
    if isinstance(planner, BatchIndexedGraphDStarLite):
        total += planner.batch_static_memory_bytes
    return int(total)


def _state_hash(planner: GraphDStarLite) -> str:
    # Avoid constructing a complete OPEN snapshot merely to verify g/rhs.
    # IEEE-754 packing preserves exact values and makes +inf explicit.
    digest = hashlib.sha256()
    for label, values in ((b"g", planner.g), (b"r", planner.rhs)):
        digest.update(label)
        for node in sorted(values):
            digest.update(struct.pack("!q", int(node)))
            digest.update(struct.pack("!d", float(values[node])))
    return digest.hexdigest()


def _invariant_holds(planner: GraphDStarLite) -> bool:
    start = int(planner.start)
    if planner._value(planner.g, start) != planner._value(planner.rhs, start):
        return False
    if isinstance(planner, IndexedGraphDStarLite):
        top = planner._indexed_open.peek_key()
    else:
        # Ignore stale physical entries and use the authoritative node->key map.
        top = min(planner._queued_keys.values(), default=(INF, INF))
    return not (top < planner._calculate_key(start))


@dataclass
class TailArmState:
    arm: str
    template: dynamic.GraphTemplate
    combo_backend: str = INDEXED_BATCH_DSTAR
    planner: Optional[GraphDStarLite] = None
    planner_identity: Optional[int] = None
    initialization_count: int = 0
    reinitialize_count: int = 0

    def _backend(self) -> str:
        return self.combo_backend if self.arm == COMBO_DSTAR else self.arm

    def _planner_class(self) -> Type[GraphDStarLite]:
        backend = self._backend()
        if backend == BASELINE_DSTAR:
            return InstrumentedGraphDStarLite
        if backend == INDEXED_DSTAR:
            return IndexedGraphDStarLite
        if backend in {INDEXED_BATCH_DSTAR, INDEXED_BATCH_CONNECTIVITY}:
            return BatchIndexedGraphDStarLite
        raise ValueError(f"arm has no D* planner: {backend}")

    def run(self, prepared: dynamic.PreparedSnapshot) -> Dict[str, Any]:
        if not prepared.accepted:
            return {
                "reachable": False, "failure_code": prepared.rejection_reason,
                "all_correct": False, "algorithm_input_hash": prepared.input_hash,
            }
        backend = self._backend()
        initial = self.initialization_count == 0
        algorithm_started = time.monotonic_ns()
        cpu_started = time.process_time_ns()
        cold_init_ms = update_ms = precheck_ms = search_ms = extraction_ms = 0.0
        maintenance_ms = diagnostics_ms = 0.0
        stats = GraphDStarSearchStats()
        node_path: Optional[Tuple[int, ...]] = None
        edge_path: Tuple[str, ...] = ()
        path_cost = INF
        connected: Optional[bool] = None
        connectivity_nodes = connectivity_edges = 0
        key_before = tuple_before = batch_before = unique_before = 0
        open_insert_before = open_update_before = open_remove_before = sift_before = 0

        if backend == COLD_GRAPH_ASTAR:
            result = dynamic.deterministic_graph_astar(self.template, prepared.statuses)
            node_path, edge_path, path_cost = result.node_path, result.edge_path, result.cost
            search_ms, extraction_ms = result.search_time_ms, result.extraction_time_ms
            expanded, generated = result.expanded_nodes, result.generated_nodes
            queue_pushes, queue_pops = result.queue_pushes, result.queue_pops
            state_memory_bytes = result.memory_bytes
            response_algorithm_ms = _elapsed_ms(algorithm_started)
            full_algorithm_ms = response_algorithm_ms
            key_calculations = tuple_allocations = update_vertex = stale = open_peak = 0
            g_changed = rhs_changed = predecessor = 0
            batch_candidates = batch_unique = 0
            indexed_insertions = indexed_updates = indexed_removals = indexed_sifts = 0
            g_rhs_hash = ""
            state_invariant = True
            identity_stable = False
            self.initialization_count += 1
        else:
            if self.planner is None:
                init_started = time.monotonic_ns()
                self.planner = _planner(self.template, self._planner_class(), prepared.statuses)
                cold_init_ms = _elapsed_ms(init_started)
                self.planner_identity = id(self.planner)
                self.initialization_count += 1
            planner = self.planner
            key_before = int(getattr(planner, "key_calculation_count", 0))
            tuple_before = int(getattr(planner, "tuple_allocation_proxy_count", 0))
            batch_before = int(getattr(planner, "update_batch_candidate_count", 0))
            unique_before = int(getattr(planner, "update_batch_unique_count", 0))
            if isinstance(planner, IndexedGraphDStarLite):
                qstats = planner._indexed_open.stats()
                open_insert_before, open_update_before = qstats.insertions, qstats.updates
                open_remove_before, sift_before = qstats.removals, qstats.sift_operations
            if not initial:
                update_started = time.monotonic_ns()
                for edge_id in prepared.changed_edges:
                    planner.edge_cost_override.pop(str(edge_id), None)
                planner.update_edges(prepared.changed_edges, statuses=prepared.changed_statuses)
                update_ms = _elapsed_ms(update_started)
            if backend == INDEXED_BATCH_CONNECTIVITY:
                precheck_started = time.monotonic_ns()
                connected, connectivity_nodes, connectivity_edges = exact_start_goal_connected(
                    self.template.nodes, self.template.adjacency,
                    self.template.start, self.template.goal, prepared.statuses,
                )
                precheck_ms = _elapsed_ms(precheck_started)
                if not connected:
                    # This is the earliest exact response point, but persistent
                    # state convergence is still performed and charged below.
                    response_algorithm_ms = _elapsed_ms(algorithm_started)
            stats = planner.compute_shortest_path()
            if not stats.converged:
                raise RuntimeError("unbounded research arm returned an incomplete D* state")
            search_ms = float(stats.search_time_ms)
            extract_started = time.monotonic_ns()
            raw_path = planner.extract_path()
            node_path = None if raw_path is None else tuple(raw_path)
            edge_path, path_cost = dynamic._path_edges_and_cost(
                self.template.adjacency, node_path, prepared.statuses,
            )
            extraction_ms = _elapsed_ms(extract_started)
            if backend != INDEXED_BATCH_CONNECTIVITY or connected:
                response_algorithm_ms = _elapsed_ms(algorithm_started)
            full_algorithm_ms = _elapsed_ms(algorithm_started)
            maintenance_ms = max(0.0, full_algorithm_ms - response_algorithm_ms)
            expanded, generated = stats.expanded_nodes, stats.generated_nodes
            queue_pushes, queue_pops = stats.queue_pushes, stats.queue_pops
            update_vertex = stats.update_vertex_count
            stale, open_peak = stats.stale_queue_entries, stats.peak_open_size
            g_changed, rhs_changed = stats.g_changed_nodes, stats.rhs_changed_nodes
            predecessor = stats.predecessor_propagations
            key_calculations = int(getattr(planner, "key_calculation_count", 0)) - key_before
            tuple_allocations = int(getattr(planner, "tuple_allocation_proxy_count", 0)) - tuple_before
            batch_candidates = int(getattr(planner, "update_batch_candidate_count", 0)) - batch_before
            batch_unique = int(getattr(planner, "update_batch_unique_count", 0)) - unique_before
            indexed_insertions = indexed_updates = indexed_removals = indexed_sifts = 0
            if isinstance(planner, IndexedGraphDStarLite):
                after = planner._indexed_open.stats()
                indexed_insertions = after.insertions - open_insert_before
                indexed_updates = after.updates - open_update_before
                indexed_removals = after.removals - open_remove_before
                indexed_sifts = after.sift_operations - sift_before
            diagnostics_started = time.monotonic_ns()
            state_memory_bytes = _planner_memory_bytes(planner)
            g_rhs_hash = _state_hash(planner)
            state_invariant = _invariant_holds(planner)
            identity_stable = id(planner) == self.planner_identity
            diagnostics_ms = _elapsed_ms(diagnostics_started)

        response_l1_ms = (
            prepared.parse_time_ms + prepared.mapping_time_ms
            + prepared.state_transition_time_ms + response_algorithm_ms
        )
        full_l1_ms = (
            prepared.parse_time_ms + prepared.mapping_time_ms
            + prepared.state_transition_time_ms + full_algorithm_ms
        )
        blocked = {
            edge_id for edge_id, status in prepared.statuses.items()
            if status in {GraphDStarLite.BLOCKED, GraphDStarLite.RECOVERING}
        }
        process_cpu_ms = (time.process_time_ns() - cpu_started) / 1.0e6
        return {
            "arm": self.arm, "effective_backend": backend,
            "initial_plan": bool(initial), "dynamic_update": not bool(initial),
            "reachable": node_path is not None,
            "failure_code": "" if node_path is not None else "L1_NO_ROUTE",
            "path_node_ids": list(node_path or ()), "path_edge_ids": list(edge_path),
            "path_cost": float(path_cost),
            "path_hash": dynamic.stable_hash(list(edge_path)) if node_path is not None else "",
            "blocked_edges_in_path": sorted(blocked.intersection(edge_path)),
            "cold_init_ms": cold_init_ms, "update_edges_ms": update_ms,
            "connectivity_precheck_ms": precheck_ms,
            "connectivity_reachable": connected,
            "connectivity_visited_nodes": connectivity_nodes,
            "connectivity_edge_checks": connectivity_edges,
            "compute_shortest_path_ms": search_ms,
            "route_extraction_ms": extraction_ms,
            "response_algorithm_ms": response_algorithm_ms,
            "full_algorithm_ms": full_algorithm_ms,
            "maintenance_after_exact_response_ms": maintenance_ms,
            "response_l1_ms": response_l1_ms, "full_l1_ms": full_l1_ms,
            "process_cpu_ms": process_cpu_ms, "diagnostics_ms_excluded": diagnostics_ms,
            "expanded_nodes": int(expanded), "generated_nodes": int(generated),
            "queue_pushes": int(queue_pushes), "queue_pops": int(queue_pops),
            "stale_queue_entries": int(stale), "open_peak": int(open_peak),
            "update_vertex_count": int(update_vertex),
            "predecessor_propagations": int(predecessor),
            "g_changed_nodes": int(g_changed), "rhs_changed_nodes": int(rhs_changed),
            "key_calculations": int(key_calculations),
            "tuple_allocation_proxy": int(tuple_allocations),
            "batch_candidate_nodes": int(batch_candidates),
            "batch_unique_nodes": int(batch_unique),
            "batch_dedup_saved": int(batch_candidates - batch_unique),
            "indexed_insertions": int(indexed_insertions),
            "indexed_updates": int(indexed_updates),
            "indexed_removals": int(indexed_removals),
            "indexed_sift_operations": int(indexed_sifts),
            "state_memory_bytes": int(state_memory_bytes),
            "g_rhs_state_hash": g_rhs_hash,
            "dstar_state_invariant": bool(state_invariant),
            "planner_identity_stable": bool(identity_stable),
            "initialization_count": int(self.initialization_count),
            "reinitialize_count": int(self.reinitialize_count),
            "implicit_reinitialize": False,
            "converged": bool(stats.converged) if backend != COLD_GRAPH_ASTAR else True,
            "partial_dstar_result_returned": False,
            "algorithm_input_hash": prepared.input_hash,
            "process_peak_rss_bytes": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024,
        }


@dataclass
class ResyncState:
    """Offline measured immediate/lazy/batched catch-up policy."""

    strategy: str
    template: dynamic.GraphTemplate
    quiet_window_snapshots: int = 2
    planner: Optional[BatchIndexedGraphDStarLite] = None
    ready: bool = False
    pending_statuses: Optional[Mapping[str, str]] = None
    pending_snapshot_id: str = ""
    pending_count: int = 0
    quiet_count: int = 0
    fallback_count: int = 0
    resync_count: int = 0

    def initialize(self, prepared: dynamic.PreparedSnapshot) -> float:
        started = time.process_time_ns()
        self.planner = _planner(
            self.template, BatchIndexedGraphDStarLite, prepared.statuses,
        )  # type: ignore[assignment]
        self.planner.compute_shortest_path()
        self.ready = True
        return (time.process_time_ns() - started) / 1.0e6

    def on_fallback(self, prepared: dynamic.PreparedSnapshot) -> Dict[str, Any]:
        self.fallback_count += 1
        self.ready = False
        self.pending_statuses = dict(prepared.statuses)
        self.pending_snapshot_id = str(prepared.snapshot.snapshot_id)
        self.pending_count += 1
        self.quiet_count = 0
        if self.strategy == "immediate":
            return self._catch_up()
        return self._empty()

    def observe(self, prepared: dynamic.PreparedSnapshot) -> Dict[str, Any]:
        if self.ready:
            return self._empty()
        self.pending_statuses = dict(prepared.statuses)
        self.pending_snapshot_id = str(prepared.snapshot.snapshot_id)
        self.pending_count += 1
        self.quiet_count = self.quiet_count + 1 if not prepared.changed_edges else 0
        threshold = 1 if self.strategy == "batched_background" else self.quiet_window_snapshots
        if self.quiet_count >= threshold:
            return self._catch_up()
        return self._empty()

    def _empty(self) -> Dict[str, Any]:
        return {
            "resync_ran": False, "resync_cpu_ms": 0.0,
            "resync_wall_ms": 0.0, "ready": self.ready,
            "coalesced_snapshots": 0,
        }

    def _catch_up(self) -> Dict[str, Any]:
        if self.pending_statuses is None:
            return self._empty()
        wall = time.monotonic_ns(); cpu = time.process_time_ns()
        planner = _planner(self.template, BatchIndexedGraphDStarLite, self.pending_statuses)
        stats = planner.compute_shortest_path()
        cpu_ms = (time.process_time_ns() - cpu) / 1.0e6
        wall_ms = _elapsed_ms(wall)
        if not stats.converged:
            raise RuntimeError("resync catch-up did not converge")
        count = self.pending_count
        snapshot_id = self.pending_snapshot_id
        status_hash = dynamic.stable_hash(self.pending_statuses)
        self.planner = planner
        self.ready = True
        self.pending_statuses = None
        self.pending_count = 0
        self.quiet_count = 0
        self.resync_count += 1
        return {
            "resync_ran": True, "resync_cpu_ms": cpu_ms,
            "resync_wall_ms": wall_ms, "ready": True,
            "coalesced_snapshots": count,
            "resync_snapshot_id": snapshot_id,
            "resync_status_hash": status_hash,
            "resync_expanded_nodes": stats.expanded_nodes,
            "resync_queue_pops": stats.queue_pops,
        }

    def flush(self) -> Dict[str, Any]:
        """Finish pending work so an experiment cannot hide maintenance."""
        return self._catch_up() if self.pending_statuses is not None else self._empty()


__all__ = [
    "ARMS", "BASELINE_DSTAR", "COLD_GRAPH_ASTAR", "COMBO_DSTAR",
    "EXPERIMENT_KIND", "INDEXED_BATCH_CONNECTIVITY", "INDEXED_BATCH_DSTAR",
    "INDEXED_DSTAR", "PARENT_ARCHITECTURE", "PROTOCOL_VERSION",
    "REFERENCE_ARCHITECTURE", "RESEARCH_ID", "ResyncState", "STATUS",
    "TailArmState",
]
