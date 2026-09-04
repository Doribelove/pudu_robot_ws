"""Tail-bounded L1 router for the independent 2D-V3 candidate.

The module intentionally contains no Nav2, costmap, or corridor publication
code.  All three experiment arms consume the same prepared dynamic overlay;
only the graph-search policy differs.  A bounded D* result is never exposed
unless :class:`GraphDStarLite` reports convergence.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Set, Tuple

from . import dynamic_incremental_value as dynamic
from .graph_dstar_lite import GraphDStarLite, GraphDStarSearchStats, INF


ARCHITECTURE_ID = "2D-V3"
IMPLEMENTATION_REVISION = "r0-hybrid-tail-bounded-v1"
PARENT_ARCHITECTURE = "2D-V2-r0"
PROTOCOL_VERSION = "PLN-02-EXP-V1"

COLD_GRAPH_ASTAR = "cold_graph_astar"
PURE_PERSISTENT_DSTAR = "pure_persistent_dstar"
HYBRID = "hybrid_bounded_dstar"
ARMS = (COLD_GRAPH_ASTAR, PURE_PERSISTENT_DSTAR, HYBRID)
_BRIDGE_CACHE: Dict[str, frozenset[str]] = {}


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


@dataclass(frozen=True)
class BudgetConfig:
    wall_ms: float = 2.0
    max_queue_pops: int = 256
    max_update_vertex: int = 4096
    max_open_size: int = 8192
    max_inconsistent_states: int = 4096
    max_changed_edges: int = 5
    max_changed_ratio: float = 0.00230
    max_route_intersections: int = 1
    recent_fallback_cooldown: int = 1

    def as_dict(self) -> Dict[str, Any]:
        return {
            "wall_ms": float(self.wall_ms),
            "max_queue_pops": int(self.max_queue_pops),
            "max_update_vertex": int(self.max_update_vertex),
            "max_open_size": int(self.max_open_size),
            "max_inconsistent_states": int(self.max_inconsistent_states),
            "max_changed_edges": int(self.max_changed_edges),
            "max_changed_ratio": float(self.max_changed_ratio),
            "max_route_intersections": int(self.max_route_intersections),
            "recent_fallback_cooldown": int(self.recent_fallback_cooldown),
        }


def exact_undirected_bridges(template: dynamic.GraphTemplate) -> Set[str]:
    """Return exact bridges, correctly accounting for parallel graph edges.

    Virtual endpoint ranking edges are excluded because they are query
    attachment mechanics, not static topology risk features.
    """
    edges = [
        edge for edge in template.edges
        if str(edge.edge_id).startswith("topology_") and bool(edge.bidirectional)
    ]
    adjacency: Dict[int, list[Tuple[int, str]]] = {}
    for edge in edges:
        edge_id = str(edge.edge_id)
        adjacency.setdefault(int(edge.source), []).append((int(edge.target), edge_id))
        adjacency.setdefault(int(edge.target), []).append((int(edge.source), edge_id))
    discovery: Dict[int, int] = {}
    low: Dict[int, int] = {}
    bridges: Set[str] = set()
    clock = 0

    def visit(node: int, parent_edge: str = "") -> None:
        nonlocal clock
        clock += 1
        discovery[node] = low[node] = clock
        for target, edge_id in adjacency.get(node, ()):
            if edge_id == parent_edge:
                continue
            if target not in discovery:
                visit(target, edge_id)
                low[node] = min(low[node], low[target])
                if low[target] > discovery[node]:
                    bridges.add(edge_id)
            else:
                low[node] = min(low[node], discovery[target])

    old_limit = sys.getrecursionlimit()
    # Skeleton chains can exceed Python's conservative default recursion
    # depth.  The bound is local to an exact offline topology feature and is
    # restored immediately after the traversal.
    sys.setrecursionlimit(max(old_limit, len(adjacency) * 2 + 256))
    try:
        for node in sorted(adjacency):
            if node not in discovery:
                visit(node)
    finally:
        sys.setrecursionlimit(old_limit)
    return bridges


def route_support_edges(
    template: dynamic.GraphTemplate, route_edges: Iterable[str],
) -> Set[str]:
    """Conservative one-hop topology support for corridor relevance.

    Stage-A has no materialized raster corridor.  Every topology edge sharing
    an endpoint with a route edge is therefore treated as corridor-relevant.
    This may trigger extra replans but can never suppress a nearby change.
    Dynamic ROS runs replace this conservative certificate with the V2 mask
    cell intersection.
    """
    by_id = {str(edge.edge_id): edge for edge in template.edges}
    route = {str(edge_id) for edge_id in route_edges}
    nodes: Set[int] = set()
    for edge_id in route:
        edge = by_id.get(edge_id)
        if edge is not None:
            nodes.update((int(edge.source), int(edge.target)))
    support = set(route)
    for edge in template.edges:
        if int(edge.source) in nodes or int(edge.target) in nodes:
            support.add(str(edge.edge_id))
    return support


def _path_cost(
    template: dynamic.GraphTemplate, edge_path: Sequence[str],
    statuses: Mapping[str, str],
) -> float:
    by_id = {str(edge.edge_id): edge for edge in template.edges}
    total = 0.0
    for edge_id in edge_path:
        edge = by_id.get(str(edge_id))
        if edge is None:
            return INF
        cost = dynamic.effective_edge_cost(edge, statuses)
        if not math.isfinite(cost):
            return INF
        total += cost
    return float(total)


def _dstar_result(
    template: dynamic.GraphTemplate, planner: GraphDStarLite,
    statuses: Mapping[str, str], stats: GraphDStarSearchStats,
) -> Dict[str, Any]:
    if not stats.converged:
        raise RuntimeError("attempted to extract an unconverged D* state")
    extraction_started = time.monotonic_ns()
    raw = planner.extract_path()
    nodes = None if raw is None else tuple(raw)
    edges, cost = dynamic._path_edges_and_cost(template.adjacency, nodes, statuses)
    extraction_ms = _elapsed_ms(extraction_started)
    return {
        "reachable": nodes is not None,
        "failure_code": "" if nodes is not None else "L1_NO_ROUTE",
        "path_node_ids": list(nodes or ()),
        "path_edge_ids": list(edges),
        "path_cost": float(cost),
        "path_hash": dynamic.stable_hash(list(edges)) if nodes is not None else "",
        "route_extraction_ms": extraction_ms,
        "expanded_nodes": int(stats.expanded_nodes),
        "generated_nodes": int(stats.generated_nodes),
        "queue_pushes": int(stats.queue_pushes),
        "queue_pops": int(stats.queue_pops),
        "update_vertex_count": int(stats.update_vertex_count),
        "stale_queue_entries": int(stats.stale_queue_entries),
        "open_peak": int(stats.peak_open_size),
        "g_changed_nodes": int(stats.g_changed_nodes),
        "rhs_changed_nodes": int(stats.rhs_changed_nodes),
        "predecessor_propagations": int(stats.predecessor_propagations),
        "compute_shortest_path_ms": float(stats.search_time_ms),
    }


class HybridL1Router:
    """One isolated episode arm with shared scheduling semantics."""

    def __init__(
        self, arm: str, template: dynamic.GraphTemplate, *,
        topology_edge_count: int, budget: Optional[BudgetConfig] = None,
    ) -> None:
        if arm not in ARMS:
            raise ValueError(f"unknown 2D-V3 arm: {arm}")
        self.arm = str(arm)
        self.template = template
        self.topology_edge_count = max(1, int(topology_edge_count))
        self.budget = budget or BudgetConfig()
        self.planner: Optional[GraphDStarLite] = None
        self.current_path_nodes: Tuple[int, ...] = ()
        self.current_path_edges: Tuple[str, ...] = ()
        self.current_cost = INF
        self.has_result = False
        cached_bridges = _BRIDGE_CACHE.get(template.static_hash)
        if cached_bridges is None:
            cached_bridges = frozenset(exact_undirected_bridges(template))
            _BRIDGE_CACHE[template.static_hash] = cached_bridges
        self.critical_bridges = set(cached_bridges)
        self.dstar_ready = False
        self.dstar_snapshot_id = ""
        self.fallback_cooldown = 0
        self.initialization_count = 0
        self.implicit_reinitialize_count = 0
        self.resync_count = 0
        self.snapshot_index = -1
        self._pending_resync: Optional[dynamic.PreparedSnapshot] = None

    def _features(self, prepared: dynamic.PreparedSnapshot) -> Dict[str, Any]:
        changed = set(prepared.changed_edges)
        route = set(self.current_path_edges)
        support = route_support_edges(self.template, route)
        intersections = changed & route
        corridor_intersections = changed & support
        critical = changed & self.critical_bridges
        predecessor_estimate = len({
            int(node)
            for edge_id in changed
            for edge in (self.template.edges)
            if str(edge.edge_id) == edge_id
            for node in (edge.source, edge.target)
        })
        return {
            "changed_edge_count": len(changed),
            "changed_edge_ratio": len(changed) / self.topology_edge_count,
            "route_intersection": bool(intersections),
            "route_intersection_count": len(intersections),
            "corridor_intersection": bool(corridor_intersections),
            "corridor_intersection_count": len(corridor_intersections),
            "critical_bridge_intersection": bool(critical),
            "critical_bridge_ids": sorted(critical),
            "affected_predecessor_estimate": predecessor_estimate,
            "open_size_before": 0 if self.planner is None else len(self.planner._open),
            "inconsistent_states_before": (
                0 if self.planner is None else len(self.planner._queued_keys)
            ),
            "dstar_ready_before": bool(self.dstar_ready),
            "recent_fallback": self.fallback_cooldown > 0,
        }

    @staticmethod
    def _safe_route_reuse(
        prepared: dynamic.PreparedSnapshot, current_edges: Sequence[str],
        *, has_result: bool,
    ) -> Tuple[bool, str]:
        if has_result and not prepared.changed_edges:
            return True, "DUPLICATE_OR_UNCONFIRMED_OBSERVATION"
        if not current_edges:
            return False, "NO_CURRENT_ROUTE"
        changed = set(prepared.changed_edges)
        if changed.intersection(current_edges):
            return False, "CURRENT_ROUTE_AFFECTED"
        # Off-route cost increases cannot invalidate or improve the current
        # shortest route.  A recovery is a cost decrease and must re-enter L1.
        statuses = {prepared.changed_statuses.get(edge_id, "") for edge_id in changed}
        if statuses.issubset({GraphDStarLite.BLOCKED_PENDING, GraphDStarLite.BLOCKED}):
            return True, "OFF_ROUTE_COST_INCREASE"
        return False, "OFF_ROUTE_COST_DECREASE_REQUIRES_OPTIMALITY_CHECK"

    def _apply_incremental_update(self, prepared: dynamic.PreparedSnapshot) -> float:
        if self.planner is None:
            return 0.0
        started = time.monotonic_ns()
        for edge_id in prepared.changed_edges:
            self.planner.edge_cost_override.pop(str(edge_id), None)
        self.planner.update_edges(
            prepared.changed_edges, statuses=prepared.changed_statuses,
        )
        return _elapsed_ms(started)

    def _run_astar(self, prepared: dynamic.PreparedSnapshot) -> Dict[str, Any]:
        result = dynamic.deterministic_graph_astar(self.template, prepared.statuses)
        return {
            "reachable": result.node_path is not None,
            "failure_code": "" if result.node_path is not None else "L1_NO_ROUTE",
            "path_node_ids": list(result.node_path or ()),
            "path_edge_ids": list(result.edge_path),
            "path_cost": float(result.cost),
            "path_hash": dynamic.stable_hash(list(result.edge_path)) if result.node_path is not None else "",
            "route_extraction_ms": float(result.extraction_time_ms),
            "expanded_nodes": int(result.expanded_nodes),
            "generated_nodes": int(result.generated_nodes),
            "queue_pushes": int(result.queue_pushes),
            "queue_pops": int(result.queue_pops),
            "update_vertex_count": 0,
            "stale_queue_entries": 0,
            "open_peak": 0,
            "g_changed_nodes": 0,
            "rhs_changed_nodes": 0,
            "predecessor_propagations": 0,
            "compute_shortest_path_ms": float(result.search_time_ms),
            "state_memory_bytes": int(result.memory_bytes),
        }

    def _resync(self, prepared: dynamic.PreparedSnapshot) -> Tuple[float, GraphDStarSearchStats]:
        """Rebuild a converged shadow state after an A* response.

        ``service_resync`` calls this after the response boundary.  Every
        microsecond is still charged as background CPU/accounted compute.
        """
        started = time.monotonic_ns()
        planner = self.template.new_dstar(prepared.statuses)
        stats = planner.compute_shortest_path()
        self.planner = planner
        self.dstar_ready = bool(stats.converged)
        self.dstar_snapshot_id = str(prepared.snapshot.snapshot_id)
        self.resync_count += 1
        return _elapsed_ms(started), stats

    def service_resync(self) -> Dict[str, Any]:
        """Run queued low-priority resync work between snapshot responses.

        The runner invokes this after it has captured response latency.  If a
        newer snapshot arrived first, ``step`` replaces the queued item and
        this method rebuilds only the newest state.  That models coalescing and
        makes the CPU charge explicit without adding it to route-availability
        latency.
        """
        prepared = self._pending_resync
        if prepared is None:
            return {"resync_ran": False, "resync_ms": 0.0,
                    "resync_cpu_ms": 0.0, "resync_ready": self.dstar_ready}
        self._pending_resync = None
        elapsed, stats = self._resync(prepared)
        return {
            "resync_ran": True, "resync_ms": elapsed,
            "resync_cpu_ms": elapsed, "resync_ready": bool(stats.converged),
            "resync_snapshot_id": str(prepared.snapshot.snapshot_id),
            "resync_expanded_nodes": int(stats.expanded_nodes),
            "resync_queue_pops": int(stats.queue_pops),
        }

    def _select_hybrid(self, features: Mapping[str, Any]) -> Tuple[str, str]:
        if not features["dstar_ready_before"]:
            return COLD_GRAPH_ASTAR, "DSTAR_NOT_READY"
        if features["recent_fallback"]:
            return COLD_GRAPH_ASTAR, "RECENT_FALLBACK_COOLDOWN"
        if features["critical_bridge_intersection"]:
            return COLD_GRAPH_ASTAR, "STATIC_BRIDGE_RISK"
        if int(features["changed_edge_count"]) > self.budget.max_changed_edges:
            return COLD_GRAPH_ASTAR, "CHANGED_EDGE_COUNT_RISK"
        if float(features["changed_edge_ratio"]) > self.budget.max_changed_ratio:
            return COLD_GRAPH_ASTAR, "CHANGED_EDGE_RATIO_RISK"
        if int(features["route_intersection_count"]) > self.budget.max_route_intersections:
            return COLD_GRAPH_ASTAR, "MULTI_ROUTE_EDGE_RISK"
        if int(features["open_size_before"]) > self.budget.max_open_size:
            return COLD_GRAPH_ASTAR, "OPEN_SIZE_PRECHECK"
        if int(features["inconsistent_states_before"]) > self.budget.max_inconsistent_states:
            return COLD_GRAPH_ASTAR, "INCONSISTENT_STATE_PRECHECK"
        return PURE_PERSISTENT_DSTAR, "BOUNDED_DSTAR_FAST_PATH"

    def step(self, prepared: dynamic.PreparedSnapshot) -> Dict[str, Any]:
        self.snapshot_index += 1
        if not prepared.accepted:
            return {
                "snapshot_accepted": False,
                "failure_code": prepared.rejection_reason,
                "algorithm_input_hash": prepared.input_hash,
            }
        wall_started = time.monotonic_ns()
        response_started = time.monotonic_ns()
        features = self._features(prepared)
        reusable, reuse_reason = self._safe_route_reuse(
            prepared, self.current_path_edges, has_result=self.has_result,
        )
        initial = not self.has_result
        update_ms = dstar_attempt_ms = fallback_ms = resync_ms = 0.0
        resync_cpu_ms = 0.0
        budget_triggered = False
        budget_reason = ""
        selector_reason = ""
        selected = self.arm
        actual_algorithm = self.arm
        scheduler_skip = False
        used_partial_result = False
        result: Dict[str, Any]

        if reusable and not initial:
            update_ms = self._apply_incremental_update(prepared)
            scheduler_skip = True
            selector_reason = reuse_reason
            actual_algorithm = "route_reuse"
            cost = _path_cost(self.template, self.current_path_edges, prepared.statuses)
            reachable = bool(self.current_path_nodes)
            result = {
                "reachable": reachable,
                "failure_code": "" if reachable else "L1_NO_ROUTE",
                "path_node_ids": list(self.current_path_nodes),
                "path_edge_ids": list(self.current_path_edges),
                "path_cost": cost,
                "path_hash": dynamic.stable_hash(list(self.current_path_edges)) if reachable else "",
                "route_extraction_ms": 0.0, "expanded_nodes": 0,
                "generated_nodes": 0, "queue_pushes": 0, "queue_pops": 0,
                "update_vertex_count": 0, "stale_queue_entries": 0,
                "open_peak": 0, "g_changed_nodes": 0, "rhs_changed_nodes": 0,
                "predecessor_propagations": 0, "compute_shortest_path_ms": 0.0,
                "state_memory_bytes": (
                    0 if self.planner is None else dynamic._dstar_memory_bytes(self.planner)
                ),
            }
            response_ms = _elapsed_ms(response_started)
        elif self.arm == COLD_GRAPH_ASTAR:
            selector_reason = "BASELINE_CONFIRMED_RELEVANT_UPDATE" if not initial else "S0_INITIAL"
            result = self._run_astar(prepared)
            response_ms = _elapsed_ms(response_started)
        elif self.arm == PURE_PERSISTENT_DSTAR:
            selector_reason = "PURE_DSTAR_DIAGNOSTIC"
            if self.planner is None:
                self.planner = self.template.new_dstar(prepared.statuses)
                self.initialization_count += 1
            else:
                update_ms = self._apply_incremental_update(prepared)
            stats = self.planner.compute_shortest_path()
            result = _dstar_result(self.template, self.planner, prepared.statuses, stats)
            result["state_memory_bytes"] = dynamic._dstar_memory_bytes(self.planner)
            self.dstar_ready = True
            self.dstar_snapshot_id = str(prepared.snapshot.snapshot_id)
            response_ms = _elapsed_ms(response_started)
        else:
            if self.planner is None:
                self.planner = self.template.new_dstar(prepared.statuses)
                self.initialization_count += 1
                stats = self.planner.compute_shortest_path()
                result = _dstar_result(self.template, self.planner, prepared.statuses, stats)
                result["state_memory_bytes"] = dynamic._dstar_memory_bytes(self.planner)
                self.dstar_ready = True
                self.dstar_snapshot_id = str(prepared.snapshot.snapshot_id)
                selector_reason = "S0_INITIAL_DSTAR"
                actual_algorithm = PURE_PERSISTENT_DSTAR
                response_ms = _elapsed_ms(response_started)
            else:
                selected, selector_reason = self._select_hybrid(features)
                update_ms = self._apply_incremental_update(prepared)
                if selected == PURE_PERSISTENT_DSTAR:
                    attempt_started = time.monotonic_ns()
                    stats = self.planner.compute_shortest_path(
                        timeout_s=max(0.0, self.budget.wall_ms / 1000.0),
                        max_queue_pops=self.budget.max_queue_pops,
                        max_update_vertex=self.budget.max_update_vertex,
                        max_open_size=self.budget.max_open_size,
                        max_inconsistent_states=self.budget.max_inconsistent_states,
                    )
                    dstar_attempt_ms = _elapsed_ms(attempt_started)
                    if stats.converged:
                        result = _dstar_result(
                            self.template, self.planner, prepared.statuses, stats,
                        )
                        result["state_memory_bytes"] = dynamic._dstar_memory_bytes(self.planner)
                        self.dstar_ready = True
                        self.dstar_snapshot_id = str(prepared.snapshot.snapshot_id)
                        actual_algorithm = PURE_PERSISTENT_DSTAR
                        response_ms = _elapsed_ms(response_started)
                    else:
                        budget_triggered = True
                        budget_reason = stats.budget_reason or "DSTAR_NOT_CONVERGED"
                        fallback_started = time.monotonic_ns()
                        result = self._run_astar(prepared)
                        fallback_ms = _elapsed_ms(fallback_started)
                        response_ms = _elapsed_ms(response_started)
                        actual_algorithm = COLD_GRAPH_ASTAR
                        self.dstar_ready = False
                        self.fallback_cooldown = self.budget.recent_fallback_cooldown
                        self._pending_resync = prepared
                else:
                    fallback_started = time.monotonic_ns()
                    result = self._run_astar(prepared)
                    fallback_ms = _elapsed_ms(fallback_started)
                    response_ms = _elapsed_ms(response_started)
                    actual_algorithm = COLD_GRAPH_ASTAR
                    self.dstar_ready = False
                    self._pending_resync = prepared

        if self.fallback_cooldown > 0 and not budget_triggered:
            self.fallback_cooldown -= 1
        if bool(result.get("reachable")):
            self.current_path_nodes = tuple(int(node) for node in result["path_node_ids"])
            self.current_path_edges = tuple(str(edge) for edge in result["path_edge_ids"])
            self.current_cost = float(result["path_cost"])
        else:
            self.current_path_nodes = ()
            self.current_path_edges = ()
            self.current_cost = INF
        self.has_result = True
        blocked = {
            edge_id for edge_id, status in prepared.statuses.items()
            if status in {GraphDStarLite.BLOCKED, GraphDStarLite.RECOVERING}
        }
        blocked_in_path = sorted(blocked.intersection(result.get("path_edge_ids", ())))
        if blocked_in_path:
            raise AssertionError(f"returned route contains unavailable edges: {blocked_in_path}")
        wall_ms = _elapsed_ms(wall_started)
        accounted_ms = (
            prepared.parse_time_ms + prepared.mapping_time_ms
            + prepared.state_transition_time_ms + wall_ms
        )
        response_l1_ms = (
            prepared.parse_time_ms + prepared.mapping_time_ms
            + prepared.state_transition_time_ms + response_ms
        )
        return {
            **result,
            **features,
            "snapshot_accepted": True,
            "initial_plan": initial,
            "dynamic_update": not initial,
            "selected_policy": selected,
            "actual_algorithm": actual_algorithm,
            "selector_reason": selector_reason,
            "scheduler_skip": scheduler_skip,
            "scheduler_skip_reason": reuse_reason if scheduler_skip else "",
            "l1_invoked": not scheduler_skip,
            "update_edges_ms": update_ms,
            "dstar_attempt_ms": dstar_attempt_ms,
            "fallback_astar_ms": fallback_ms,
            "resync_ms": resync_ms,
            "resync_cpu_ms": resync_cpu_ms,
            "response_l1_ms": response_l1_ms,
            "full_l1_accounted_ms": accounted_ms,
            "budget_triggered": budget_triggered,
            "budget_reason": budget_reason,
            "partial_dstar_result_returned": used_partial_result,
            "dstar_ready_after": self.dstar_ready,
            "dstar_snapshot_id": self.dstar_snapshot_id,
            "fallback_count": self.resync_count,
            "resync_count": self.resync_count,
            "implicit_reinitialize_count": self.implicit_reinitialize_count,
            "blocked_edges_in_path": blocked_in_path,
            "algorithm_input_hash": prepared.input_hash,
        }


__all__ = [
    "ARCHITECTURE_ID", "IMPLEMENTATION_REVISION", "PARENT_ARCHITECTURE",
    "PROTOCOL_VERSION", "COLD_GRAPH_ASTAR", "PURE_PERSISTENT_DSTAR", "HYBRID",
    "ARMS", "BudgetConfig", "HybridL1Router", "exact_undirected_bridges",
    "route_support_edges",
]
