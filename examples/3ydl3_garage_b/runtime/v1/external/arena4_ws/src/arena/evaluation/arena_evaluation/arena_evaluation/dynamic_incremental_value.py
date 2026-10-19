"""Paired graph-search primitives for the 2D-V1 dynamic value experiment.

This module is an independent ``r3`` implementation layer.  It leaves the
frozen r2 static planner untouched and supplies only the dynamic edge overlay,
snapshot protocol, deterministic Graph A* oracle, and the three paired arms
used by the formal experiment.
"""

from __future__ import annotations

import hashlib
import heapq
import json
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

from .dynamic_snapshot import DynamicSnapshot
from .graph_dstar_lite import GraphDStarLite, GraphDStarSearchStats, GraphEdge, INF


ARCHITECTURE_ID = "2D-V1"
IMPLEMENTATION_REVISION = "r3"
PARENT_ARCHITECTURE = "2D-V1-r2"
EXPERIMENT_KIND = "dynamic_incremental"
PROTOCOL_VERSION = "PLN-02-EXP-V1"

INCREMENTAL_DSTAR = "incremental_dstar"
COLD_DSTAR = "cold_dstar"
COLD_GRAPH_ASTAR = "cold_graph_astar"
ARMS = (INCREMENTAL_DSTAR, COLD_DSTAR, COLD_GRAPH_ASTAR)


Cell = Tuple[int, int]


def stable_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(
            value, sort_keys=True, separators=(",", ":"), allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


@dataclass(frozen=True)
class SnapshotMapping:
    changed_cells: Tuple[Cell, ...]
    observation_changed_edges: Tuple[str, ...]
    occupied_edges: Tuple[str, ...]
    mapping_time_ms: float


class CellToEdgeIndex:
    """Immutable reverse index plus per-episode occupancy bookkeeping."""

    def __init__(self, edge_cells: Mapping[str, Iterable[Sequence[int]]]) -> None:
        started_ns = time.monotonic_ns()
        mutable: Dict[Cell, Set[str]] = {}
        self.edge_cells: Dict[str, Tuple[Cell, ...]] = {}
        for edge_id, cells in sorted(edge_cells.items()):
            normalized = tuple(sorted({(int(cell[0]), int(cell[1])) for cell in cells}))
            self.edge_cells[str(edge_id)] = normalized
            for cell in normalized:
                mutable.setdefault(cell, set()).add(str(edge_id))
        self.cell_to_edges: Dict[Cell, Tuple[str, ...]] = {
            cell: tuple(sorted(edge_ids)) for cell, edge_ids in mutable.items()
        }
        self.build_time_ms = _elapsed_ms(started_ns)
        self.memory_bytes = int(
            sum(32 + len(edges) * 16 for edges in self.cell_to_edges.values())
            + sum(len(edge_id.encode("utf-8")) + len(cells) * 16
                  for edge_id, cells in self.edge_cells.items())
        )
        self.reset()

    def reset(self) -> None:
        self._occupied_cells: Set[Cell] = set()
        self._edge_hit_count: Dict[str, int] = {}

    def apply(self, occupied_cells: Iterable[Sequence[int]]) -> SnapshotMapping:
        started_ns = time.monotonic_ns()
        current = {(int(cell[0]), int(cell[1])) for cell in occupied_cells}
        changed_cells = self._occupied_cells.symmetric_difference(current)
        previously_occupied_edges = {
            edge_id for edge_id, count in self._edge_hit_count.items() if count > 0
        }
        for cell in sorted(changed_cells):
            delta = 1 if cell in current else -1
            for edge_id in self.cell_to_edges.get(cell, ()):
                next_count = self._edge_hit_count.get(edge_id, 0) + delta
                if next_count > 0:
                    self._edge_hit_count[edge_id] = next_count
                else:
                    self._edge_hit_count.pop(edge_id, None)
        occupied_edges = {
            edge_id for edge_id, count in self._edge_hit_count.items() if count > 0
        }
        observation_changed = previously_occupied_edges.symmetric_difference(occupied_edges)
        self._occupied_cells = current
        return SnapshotMapping(
            tuple(sorted(changed_cells)), tuple(sorted(observation_changed)),
            tuple(sorted(occupied_edges)), _elapsed_ms(started_ns),
        )


@dataclass(frozen=True)
class SnapshotDecision:
    accepted: bool
    reason: str = ""


class SnapshotProtocolGuard:
    """Reject stale, expired, wrong-map, and out-of-order snapshots atomically."""

    def __init__(self, *, map_version: str, map_shape: Sequence[int]) -> None:
        self.map_version = str(map_version)
        self.map_shape = (int(map_shape[0]), int(map_shape[1]))
        self.last_timestamp: Optional[float] = None
        self.last_snapshot_id = ""
        self.last_snapshot_hash = ""

    def validate(self, snapshot: DynamicSnapshot, *, now: Optional[float] = None) -> SnapshotDecision:
        if str(snapshot.map_version) != self.map_version:
            return SnapshotDecision(False, "MAP_VERSION_MISMATCH")
        if snapshot.map_shape is not None and tuple(snapshot.map_shape) != self.map_shape:
            return SnapshotDecision(False, "MAP_SHAPE_MISMATCH")
        if snapshot.is_expired(now=now):
            return SnapshotDecision(False, "EXPIRED_SNAPSHOT")
        if self.last_timestamp is not None and float(snapshot.timestamp) <= self.last_timestamp:
            return SnapshotDecision(False, "OUT_OF_ORDER_SNAPSHOT")
        return SnapshotDecision(True, "")

    def commit(self, snapshot: DynamicSnapshot) -> None:
        self.last_timestamp = float(snapshot.timestamp)
        self.last_snapshot_id = str(snapshot.snapshot_id)
        self.last_snapshot_hash = str(snapshot.snapshot_hash)


def next_edge_status(current: str, occupied: bool) -> str:
    """Apply the confirmed block/recovery state machine for one observation."""
    state = str(current or GraphDStarLite.AVAILABLE)
    if state == GraphDStarLite.AVAILABLE:
        return GraphDStarLite.BLOCKED_PENDING if occupied else GraphDStarLite.AVAILABLE
    if state == GraphDStarLite.BLOCKED_PENDING:
        return GraphDStarLite.BLOCKED if occupied else GraphDStarLite.AVAILABLE
    if state == GraphDStarLite.BLOCKED:
        return GraphDStarLite.BLOCKED if occupied else GraphDStarLite.RECOVERING
    if state == GraphDStarLite.RECOVERING:
        return GraphDStarLite.BLOCKED if occupied else GraphDStarLite.AVAILABLE
    if state == GraphDStarLite.PENALIZED:
        return GraphDStarLite.BLOCKED_PENDING if occupied else GraphDStarLite.AVAILABLE
    raise ValueError(f"unknown dynamic edge status: {state}")


@dataclass(frozen=True)
class PreparedSnapshot:
    snapshot: DynamicSnapshot
    accepted: bool
    rejection_reason: str
    parse_time_ms: float
    mapping_time_ms: float
    state_transition_time_ms: float
    changed_cells: Tuple[Cell, ...]
    observation_changed_edges: Tuple[str, ...]
    changed_edges: Tuple[str, ...]
    occupied_edges: Tuple[str, ...]
    statuses: Mapping[str, str]
    changed_statuses: Mapping[str, str]
    changed_costs: Mapping[str, float]
    input_hash: str


class DynamicEdgeOverlay:
    """Per-episode dynamic overlay; the static graph is never mutated."""

    def __init__(
        self,
        edge_cells: Mapping[str, Iterable[Sequence[int]]],
        *,
        map_version: str,
        map_shape: Sequence[int],
    ) -> None:
        self.index = CellToEdgeIndex(edge_cells)
        self.guard = SnapshotProtocolGuard(map_version=map_version, map_shape=map_shape)
        self.statuses: Dict[str, str] = {
            str(edge_id): GraphDStarLite.AVAILABLE for edge_id in edge_cells
        }

    def consume_json(self, payload: str, *, now: Optional[float] = None) -> PreparedSnapshot:
        parse_started_ns = time.monotonic_ns()
        raw = json.loads(payload)
        snapshot = DynamicSnapshot(
            snapshot_id=str(raw["snapshot_id"]),
            timestamp=float(raw["timestamp"]),
            occupied_cells=tuple(tuple(cell) for cell in raw.get("occupied_cells", ())),
            obstacle_confidence=dict(raw.get("obstacle_confidence") or {}),
            ttl=raw.get("ttl"),
            map_version=str(raw.get("map_version", "")),
            map_shape=None if raw.get("map_shape") is None else tuple(raw["map_shape"]),
            snapshot_hash=str(raw.get("snapshot_hash", "")),
        )
        parse_ms = _elapsed_ms(parse_started_ns)
        decision = self.guard.validate(snapshot, now=now)
        if not decision.accepted:
            rejected_hash = stable_hash({
                "snapshot_hash": snapshot.snapshot_hash,
                "accepted": False,
                "reason": decision.reason,
            })
            return PreparedSnapshot(
                snapshot, False, decision.reason, parse_ms, 0.0, 0.0,
                (), (), (), (), dict(self.statuses), {}, {}, rejected_hash,
            )

        mapping = self.index.apply(snapshot.occupied_cells)
        transition_started_ns = time.monotonic_ns()
        occupied = set(mapping.occupied_edges)
        advancing = {
            edge_id for edge_id, status in self.statuses.items()
            if status in {GraphDStarLite.BLOCKED_PENDING, GraphDStarLite.RECOVERING}
        }
        candidates = set(mapping.observation_changed_edges) | advancing
        changed_statuses: Dict[str, str] = {}
        changed_costs: Dict[str, float] = {}
        for edge_id in sorted(candidates):
            old_status = self.statuses.get(edge_id, GraphDStarLite.AVAILABLE)
            new_status = next_edge_status(old_status, edge_id in occupied)
            if new_status != old_status:
                self.statuses[edge_id] = new_status
                changed_statuses[edge_id] = new_status
                changed_costs[edge_id] = (
                    INF if new_status == GraphDStarLite.BLOCKED else float("nan")
                )
        transition_ms = _elapsed_ms(transition_started_ns)
        self.guard.commit(snapshot)
        input_payload = {
            "snapshot_hash": snapshot.snapshot_hash,
            "changed_cells": [list(cell) for cell in mapping.changed_cells],
            "observation_changed_edges": list(mapping.observation_changed_edges),
            "changed_edges": sorted(changed_statuses),
            "occupied_edges": list(mapping.occupied_edges),
            "statuses": self.statuses,
            "costs": {
                edge_id: "INF" if status == GraphDStarLite.BLOCKED else "STATIC_OR_STATE_MULTIPLIER"
                for edge_id, status in sorted(self.statuses.items())
            },
        }
        return PreparedSnapshot(
            snapshot, True, "", parse_ms, mapping.mapping_time_ms, transition_ms,
            mapping.changed_cells, mapping.observation_changed_edges,
            tuple(sorted(changed_statuses)), mapping.occupied_edges,
            dict(self.statuses), changed_statuses, changed_costs,
            stable_hash(input_payload),
        )


@dataclass(frozen=True)
class GraphTemplate:
    nodes: Tuple[int, ...]
    edges: Tuple[GraphEdge, ...]
    start: int
    goal: int
    node_positions: Mapping[int, Tuple[float, float]]
    adjacency: Mapping[int, Tuple[Tuple[int, GraphEdge, bool], ...]]
    predecessors: Mapping[int, Tuple[Tuple[int, GraphEdge, bool], ...]]
    static_hash: str

    @classmethod
    def from_dstar(cls, planner: GraphDStarLite) -> "GraphTemplate":
        edges = tuple(sorted(planner.edges.values(), key=lambda edge: str(edge.edge_id)))
        positions = {
            int(node): (float(value[0]), float(value[1]))
            for node, value in dict(getattr(planner, "node_positions", {})).items()
        }
        # Root-to-ranked-candidate edges carry a rank penalty rather than the
        # physical displacement between the two stored poses.  Using the raw
        # candidate pose in the Euclidean heuristic can therefore overestimate
        # that edge by a few centimetres, violating D* Lite's consistency
        # requirement and allowing premature termination after a cost increase.
        # Collapse only those virtual ranking edges for heuristic purposes;
        # topology and connection edge costs/geometry remain unchanged.
        for edge in edges:
            edge_id = str(edge.edge_id)
            if edge_id.startswith("root_start_") and int(edge.source) in positions:
                positions[int(edge.target)] = positions[int(edge.source)]
            elif edge_id.startswith("root_goal_") and int(edge.target) in positions:
                positions[int(edge.source)] = positions[int(edge.target)]
        adjacency = {
            int(node): tuple(values) for node, values in planner.adjacency.items()
        }
        predecessors = {
            int(node): tuple(values) for node, values in planner._predecessors.items()
        }
        payload = {
            "nodes": list(planner.nodes),
            "edges": [
                [edge.edge_id, edge.source, edge.target, edge.length_m,
                 edge.static_cost, edge.min_clearance_m, edge.turn_penalty,
                 edge.bidirectional]
                for edge in edges
            ],
            "start": planner.start,
            "goal": planner.goal,
            "positions": positions,
        }
        return cls(
            tuple(planner.nodes), edges, int(planner.start), int(planner.goal),
            positions, adjacency, predecessors, stable_hash(payload),
        )

    def new_dstar(self, statuses: Optional[Mapping[str, str]] = None) -> GraphDStarLite:
        planner = GraphDStarLite(
            self.nodes, self.edges, self.start, self.goal,
            edge_status=statuses or {},
        )
        planner.node_positions = dict(self.node_positions)
        planner.state_representation = "original_topology_node_id"
        return planner


def effective_edge_cost(edge: GraphEdge, statuses: Mapping[str, str]) -> float:
    status = str(statuses.get(str(edge.edge_id), GraphDStarLite.AVAILABLE))
    if status in {GraphDStarLite.BLOCKED, GraphDStarLite.RECOVERING}:
        return INF
    value = float(edge.length_m + edge.static_cost + edge.turn_penalty)
    if not math.isfinite(value) or value <= 0.0:
        return INF
    if status == GraphDStarLite.PENALIZED:
        value *= 10.0
    elif status == GraphDStarLite.BLOCKED_PENDING:
        value *= 3.0
    return value


def _path_edges_and_cost(
    adjacency: Mapping[int, Sequence[Tuple[int, GraphEdge, bool]]],
    node_path: Optional[Sequence[int]],
    statuses: Mapping[str, str],
) -> Tuple[Tuple[str, ...], float]:
    if not node_path:
        return (), INF
    edge_ids: List[str] = []
    total = 0.0
    for first, second in zip(node_path, node_path[1:]):
        choices = [
            (effective_edge_cost(edge, statuses), str(edge.edge_id))
            for target, edge, _reverse in adjacency.get(int(first), ())
            if int(target) == int(second)
        ]
        if not choices:
            return (), INF
        cost, edge_id = min(choices, key=lambda item: (item[0], item[1]))
        if not math.isfinite(cost):
            return (), INF
        edge_ids.append(edge_id)
        total += cost
    return tuple(edge_ids), float(total)


@dataclass(frozen=True)
class GraphAStarResult:
    node_path: Optional[Tuple[int, ...]]
    edge_path: Tuple[str, ...]
    cost: float
    expanded_nodes: int
    generated_nodes: int
    queue_pushes: int
    queue_pops: int
    search_time_ms: float
    extraction_time_ms: float
    memory_bytes: int


def deterministic_graph_astar(
    template: GraphTemplate, statuses: Mapping[str, str],
) -> GraphAStarResult:
    """Run fresh reverse Graph A* and extract with D* Lite's exact tie-break."""
    adjacency = template.adjacency
    predecessors = template.predecessors

    def heuristic(node: int) -> float:
        first = template.node_positions.get(int(node))
        second = template.node_positions.get(int(template.start))
        if first is None or second is None:
            return 0.0
        return math.hypot(first[0] - second[0], first[1] - second[1])

    started_ns = time.monotonic_ns()
    distance: Dict[int, float] = {int(template.goal): 0.0}
    queue: List[Tuple[float, float, int]] = [
        (heuristic(template.goal), 0.0, int(template.goal))
    ]
    expanded = generated = pushes = pops = 0
    while queue:
        _priority, cost, node = heapq.heappop(queue)
        pops += 1
        if cost != distance.get(node):
            continue
        expanded += 1
        if node == int(template.start):
            break
        for predecessor, edge, _reverse in predecessors.get(node, ()):
            edge_cost = effective_edge_cost(edge, statuses)
            if not math.isfinite(edge_cost):
                continue
            candidate = cost + edge_cost
            if candidate < distance.get(int(predecessor), INF):
                distance[int(predecessor)] = candidate
                heapq.heappush(
                    queue,
                    (candidate + heuristic(int(predecessor)), candidate, int(predecessor)),
                )
                generated += 1
                pushes += 1
    search_ms = _elapsed_ms(started_ns)

    extraction_started_ns = time.monotonic_ns()
    node_path: Optional[List[int]] = None
    if math.isfinite(distance.get(int(template.start), INF)):
        node_path = [int(template.start)]
        current = int(template.start)
        visited = {current}
        limit = max(2, len(template.nodes) * 2)
        while current != int(template.goal) and len(node_path) < limit:
            choices = []
            for successor, edge, _reverse in adjacency.get(current, ()):
                value = effective_edge_cost(edge, statuses) + distance.get(successor, INF)
                if math.isfinite(value):
                    choices.append((value, int(successor), str(edge.edge_id)))
            if not choices:
                node_path = None
                break
            _value, successor, _edge_id = min(
                choices, key=lambda item: (item[0], item[1], item[2]),
            )
            if successor in visited:
                node_path = None
                break
            node_path.append(successor)
            visited.add(successor)
            current = successor
        if node_path is not None and current != int(template.goal):
            node_path = None
    edge_path, path_cost = _path_edges_and_cost(adjacency, node_path, statuses)
    extraction_ms = _elapsed_ms(extraction_started_ns)
    memory_bytes = int(
        sys.getsizeof(distance) + sum(sys.getsizeof(k) + sys.getsizeof(v) for k, v in distance.items())
        + sys.getsizeof(queue) + sum(sys.getsizeof(item) for item in queue)
    )
    return GraphAStarResult(
        None if node_path is None else tuple(node_path), edge_path, path_cost,
        expanded, generated, pushes, pops, search_ms, extraction_ms, memory_bytes,
    )


def _dstar_memory_bytes(planner: GraphDStarLite) -> int:
    mappings = (planner.g, planner.rhs, planner._queued_keys, planner.edge_status,
                planner.edge_cost_override)
    total = sys.getsizeof(planner._open) + sum(sys.getsizeof(item) for item in planner._open)
    for mapping in mappings:
        total += sys.getsizeof(mapping)
        total += sum(sys.getsizeof(key) + sys.getsizeof(value) for key, value in mapping.items())
    return int(total)


@dataclass
class ArmState:
    arm: str
    template: GraphTemplate
    planner: Optional[GraphDStarLite] = None
    initialization_count: int = 0
    reinitialize_call_count: int = 0
    original_planner_identity: Optional[int] = None

    def run(self, prepared: PreparedSnapshot) -> Dict[str, Any]:
        if not prepared.accepted:
            return {
                "reachable": "not_run", "failure_code": prepared.rejection_reason,
                "algorithm_input_hash": prepared.input_hash,
            }
        wall_started_ns = time.monotonic_ns()
        initialization_ms = update_ms = search_ms = extraction_ms = 0.0
        expanded = generated = pushes = pops = update_vertex = 0
        reused_g = reused_rhs = reused_open = reused_km = False
        state_before: Dict[str, Any] = {}
        node_path: Optional[Tuple[int, ...]] = None
        edge_path: Tuple[str, ...] = ()
        path_cost = INF
        memory_bytes = 0

        if self.arm == INCREMENTAL_DSTAR:
            if self.planner is None:
                init_started_ns = time.monotonic_ns()
                self.planner = self.template.new_dstar(prepared.statuses)
                initialization_ms = _elapsed_ms(init_started_ns)
                self.initialization_count += 1
                self.original_planner_identity = id(self.planner)
            else:
                state_before = self.planner.state_snapshot()
                reused_g = bool(state_before.get("g"))
                reused_rhs = bool(state_before.get("rhs"))
                reused_open = "OPEN" in state_before
                reused_km = "km" in state_before
                update_started_ns = time.monotonic_ns()
                for edge_id in prepared.changed_edges:
                    self.planner.edge_cost_override.pop(str(edge_id), None)
                self.planner.update_edges(
                    prepared.changed_edges, statuses=prepared.changed_statuses,
                )
                update_ms = _elapsed_ms(update_started_ns)
            stats = self.planner.compute_shortest_path()
            search_ms = float(stats.search_time_ms)
            extraction_started_ns = time.monotonic_ns()
            raw_path = self.planner.extract_path()
            node_path = None if raw_path is None else tuple(raw_path)
            edge_path, path_cost = _path_edges_and_cost(
                self.planner.adjacency, node_path, prepared.statuses,
            )
            extraction_ms = _elapsed_ms(extraction_started_ns)
            expanded, generated = stats.expanded_nodes, stats.generated_nodes
            pushes, pops = stats.queue_pushes, stats.queue_pops
            update_vertex = stats.update_vertex_count
            memory_bytes = _dstar_memory_bytes(self.planner)
            planner_identity_stable = id(self.planner) == self.original_planner_identity
            state_after = self.planner.state_snapshot()
        elif self.arm == COLD_DSTAR:
            init_started_ns = time.monotonic_ns()
            if self.planner is None:
                self.planner = self.template.new_dstar(prepared.statuses)
                self.original_planner_identity = id(self.planner)
            else:
                self.planner.edge_status = dict(prepared.statuses)
                self.planner.edge_cost_override.clear()
                self.planner.reinitialize(self.template.start, self.template.goal)
                self.reinitialize_call_count += 1
            planner = self.planner
            initialization_ms = _elapsed_ms(init_started_ns)
            self.initialization_count += 1
            stats = planner.compute_shortest_path()
            search_ms = float(stats.search_time_ms)
            extraction_started_ns = time.monotonic_ns()
            raw_path = planner.extract_path()
            node_path = None if raw_path is None else tuple(raw_path)
            edge_path, path_cost = _path_edges_and_cost(
                planner.adjacency, node_path, prepared.statuses,
            )
            extraction_ms = _elapsed_ms(extraction_started_ns)
            expanded, generated = stats.expanded_nodes, stats.generated_nodes
            pushes, pops = stats.queue_pushes, stats.queue_pops
            update_vertex = stats.update_vertex_count
            memory_bytes = _dstar_memory_bytes(planner)
            planner_identity_stable = id(planner) == self.original_planner_identity
            state_after = planner.state_snapshot()
        elif self.arm == COLD_GRAPH_ASTAR:
            result = deterministic_graph_astar(self.template, prepared.statuses)
            search_ms, extraction_ms = result.search_time_ms, result.extraction_time_ms
            node_path, edge_path, path_cost = result.node_path, result.edge_path, result.cost
            expanded, generated = result.expanded_nodes, result.generated_nodes
            pushes, pops = result.queue_pushes, result.queue_pops
            memory_bytes = result.memory_bytes
            self.initialization_count += 1
            planner_identity_stable = False
            state_after = {}
        else:
            raise ValueError(f"unknown experiment arm: {self.arm}")

        algorithm_wall_ms = _elapsed_ms(wall_started_ns)
        full_l1_ms = (
            prepared.parse_time_ms + prepared.mapping_time_ms
            + prepared.state_transition_time_ms + algorithm_wall_ms
        )
        blocked = {
            edge_id for edge_id, status in prepared.statuses.items()
            if status == GraphDStarLite.BLOCKED
        }
        blocked_in_path = sorted(blocked.intersection(edge_path))
        return {
            "reachable": node_path is not None,
            "failure_code": "" if node_path is not None else "L1_NO_ROUTE",
            "path_node_ids": list(node_path or ()),
            "path_edge_ids": list(edge_path),
            "path_hash": stable_hash(list(edge_path)) if node_path is not None else "",
            "path_cost": path_cost,
            "blocked_edges_in_path": blocked_in_path,
            "expanded_nodes": int(expanded), "generated_nodes": int(generated),
            "queue_pushes": int(pushes), "queue_pops": int(pops),
            "update_vertex_count": int(update_vertex),
            "graph_initialization_ms": initialization_ms,
            "update_edges_ms": update_ms,
            "compute_shortest_path_ms": search_ms,
            "route_extraction_ms": extraction_ms,
            "algorithm_wall_ms": algorithm_wall_ms,
            "full_incremental_l1_ms": full_l1_ms,
            "state_memory_bytes": memory_bytes,
            "initialization_count": self.initialization_count,
            "reinitialize_call_count": self.reinitialize_call_count,
            "implicit_reinitialize": False,
            "planner_identity_stable": planner_identity_stable,
            "g_reused": reused_g, "rhs_reused": reused_rhs,
            "open_reused": reused_open, "km_reused": reused_km,
            "state_before_hash": stable_hash(state_before),
            "state_after_hash": stable_hash(state_after),
            "g_state_count": len(state_after.get("g", {})),
            "rhs_state_count": len(state_after.get("rhs", {})),
            "open_state_count": len(state_after.get("OPEN", [])),
            "km": state_after.get("km", 0.0),
            "edge_cost_version": state_after.get("edge_cost_version", 0),
            "algorithm_input_hash": prepared.input_hash,
        }


def run_paired_episode(
    template: GraphTemplate,
    event_payloads: Sequence[str],
    edge_cells: Mapping[str, Iterable[Sequence[int]]],
    *,
    map_version: str,
    map_shape: Sequence[int],
    arm_order: Sequence[str] = ARMS,
) -> List[Dict[str, Any]]:
    """Execute one episode with isolated state and verify paired inputs."""
    overlays = {
        arm: DynamicEdgeOverlay(
            edge_cells, map_version=map_version, map_shape=map_shape,
        )
        for arm in ARMS
    }
    states = {arm: ArmState(arm, template) for arm in ARMS}
    rows: List[Dict[str, Any]] = []
    for snapshot_index, payload in enumerate(event_payloads):
        per_snapshot: List[Dict[str, Any]] = []
        rotated = list(arm_order)
        if rotated:
            offset = snapshot_index % len(rotated)
            rotated = rotated[offset:] + rotated[:offset]
        for arm in rotated:
            prepared = overlays[arm].consume_json(payload)
            result = states[arm].run(prepared)
            row = {
                "snapshot_index": snapshot_index,
                "snapshot_id": prepared.snapshot.snapshot_id,
                "snapshot_hash": prepared.snapshot.snapshot_hash,
                "arm": arm,
                "snapshot_accepted": prepared.accepted,
                "snapshot_rejection_reason": prepared.rejection_reason,
                "snapshot_parse_ms": prepared.parse_time_ms,
                "changed_cell_to_edge_mapping_ms": prepared.mapping_time_ms,
                "edge_state_transition_ms": prepared.state_transition_time_ms,
                "changed_cells_count": len(prepared.changed_cells),
                "observation_changed_edge_count": len(prepared.observation_changed_edges),
                "changed_edge_count": len(prepared.changed_edges),
                "changed_edge_ids": list(prepared.changed_edges),
                "changed_edge_statuses": dict(prepared.changed_statuses),
                "changed_edge_costs": {
                    edge_id: (
                        "INF" if status == GraphDStarLite.BLOCKED
                        else "STATIC_OR_STATE_MULTIPLIER"
                    )
                    for edge_id, status in sorted(prepared.changed_statuses.items())
                },
                "occupied_edge_count": len(prepared.occupied_edges),
                "occupied_edge_ids": list(prepared.occupied_edges),
                "blocked_edge_count": sum(
                    status == GraphDStarLite.BLOCKED
                    for status in prepared.statuses.values()
                ),
                "blocked_edge_ids": sorted(
                    edge_id for edge_id, status in prepared.statuses.items()
                    if status == GraphDStarLite.BLOCKED
                ),
                "edge_status_hash": stable_hash(prepared.statuses),
                "edge_cost_hash": stable_hash({
                    edge_id: "INF" if status == GraphDStarLite.BLOCKED else status
                    for edge_id, status in prepared.statuses.items()
                }),
                **result,
            }
            per_snapshot.append(row)
            rows.append(row)
        hashes = {row["algorithm_input_hash"] for row in per_snapshot}
        statuses = {row["edge_status_hash"] for row in per_snapshot}
        costs = {row["edge_cost_hash"] for row in per_snapshot}
        if len(hashes) != 1 or len(statuses) != 1 or len(costs) != 1:
            raise AssertionError(f"paired-arm input mismatch at snapshot {snapshot_index}")
    return rows


__all__ = [
    "ARCHITECTURE_ID", "IMPLEMENTATION_REVISION", "PARENT_ARCHITECTURE",
    "EXPERIMENT_KIND", "PROTOCOL_VERSION", "INCREMENTAL_DSTAR", "COLD_DSTAR",
    "COLD_GRAPH_ASTAR", "ARMS", "ArmState", "CellToEdgeIndex",
    "DynamicEdgeOverlay", "GraphAStarResult", "GraphTemplate", "PreparedSnapshot",
    "SnapshotProtocolGuard", "deterministic_graph_astar", "effective_edge_cost",
    "next_edge_status", "run_paired_episode", "stable_hash",
]
