"""Adapter from the latest 2A production topology to the 3D-V1 L1 contract."""

from __future__ import annotations

import hashlib
import heapq
import json
import math
import time
from dataclasses import dataclass
from itertools import count
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

from arena_evaluation import l1_l3_corridor_hybrid_smoke as production
from arena_evaluation import two_layer_v1_formal_benchmark as adaptive
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.topology import TopologyArtifact, TopologyEdge, TopologyRoute

from .l2_incremental import Cell
from .pipeline import L1Plan


ATTACHMENT_RANK_PENALTY_M = 0.05


def _stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str,
    ).encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class GraphAStarDiagnostics:
    search_ms: float
    expanded_nodes: int
    generated_nodes: int
    blocked_edge_count: int
    start_candidate_count: int
    goal_candidate_count: int


class TopologyEdgeCellIndex:
    """Offline centerline index used only after L2 proves corridor no-route."""

    def __init__(self, artifact: TopologyArtifact) -> None:
        started_ns = time.monotonic_ns()
        self.cell_to_edges: Dict[Cell, Tuple[int, ...]] = {}
        mutable: Dict[Cell, Set[int]] = {}
        resolution = float(artifact.hospital_map.resolution)
        for edge in artifact.graph.edges:
            for first, second in zip(edge.polyline, edge.polyline[1:]):
                length = math.hypot(
                    float(second[0]) - float(first[0]),
                    float(second[1]) - float(first[1]),
                )
                samples = max(1, int(math.ceil(length / max(1.0e-9, resolution * 0.5))))
                for index in range(samples + 1):
                    ratio = index / samples
                    x = float(first[0]) + (float(second[0]) - float(first[0])) * ratio
                    y = float(first[1]) + (float(second[1]) - float(first[1])) * ratio
                    cell = artifact.hospital_map.world_to_cell(x, y)
                    if cell is not None:
                        mutable.setdefault((int(cell[0]), int(cell[1])), set()).add(int(edge.edge_id))
        self.cell_to_edges = {
            cell: tuple(sorted(values)) for cell, values in mutable.items()
        }
        self.build_ms = (time.monotonic_ns() - started_ns) / 1.0e6

    def query(self, cells: Iterable[Cell]) -> Set[int]:
        result: Set[int] = set()
        for cell in cells:
            result.update(self.cell_to_edges.get((int(cell[0]), int(cell[1])), ()))
        return result


class DeterministicGraphAStarL1:
    """Dynamic edge exclusion with current production endpoint/corridor code."""

    def __init__(
        self,
        ctx: Any,
        artifact: TopologyArtifact,
        *,
        map_hash: str,
        topology_hash: str,
        footprint: Sequence[Sequence[float]] = runtime.FOOTPRINT,
    ) -> None:
        self.ctx = ctx
        self.artifact = artifact
        self.map_hash = str(map_hash)
        self.topology_hash = str(topology_hash)
        self.footprint = tuple(tuple(float(value) for value in point) for point in footprint)
        self.footprint_hash = _stable_hash(self.footprint)
        self.nodes = {int(node.node_id): node for node in artifact.graph.nodes}
        self.edge_index = TopologyEdgeCellIndex(artifact)
        self.call_count = 0
        self.last_diagnostics: Optional[GraphAStarDiagnostics] = None

    def blocked_edges(self, blocked_cells: Iterable[Cell]) -> Set[int]:
        return self.edge_index.query(blocked_cells)

    def _heuristic(self, node_id: int, goal_ids: Sequence[int]) -> float:
        node = self.nodes[int(node_id)]
        return min(
            math.hypot(node.x - self.nodes[goal].x, node.y - self.nodes[goal].y)
            for goal in goal_ids
        )

    def _search(
        self, starts: Sequence[Any], goals: Sequence[Any], blocked: Set[int],
    ) -> Tuple[Optional[TopologyRoute], Optional[Any], Optional[Any], GraphAStarDiagnostics]:
        started_ns = time.monotonic_ns()
        adjacency = self.artifact.graph.adjacency()
        goal_by_node = {int(item.node_id): (index, item) for index, item in enumerate(goals)}
        goal_ids = tuple(sorted(goal_by_node))
        serial = count()
        queue: List[Tuple[float, float, int, int, int]] = []
        distance: Dict[int, float] = {}
        roots: Dict[int, int] = {}
        root_candidate: Dict[int, Any] = {}
        previous: Dict[int, Tuple[int, TopologyEdge, bool]] = {}
        generated = 0
        for index, candidate in enumerate(starts):
            node_id = int(candidate.node_id)
            # Production endpoint candidates are already sorted by exact-pose
            # distance.  Preserve that deterministic preference explicitly.
            initial = ATTACHMENT_RANK_PENALTY_M * index
            if initial < distance.get(node_id, math.inf):
                distance[node_id] = initial
                roots[node_id] = node_id
                root_candidate[node_id] = candidate
                heapq.heappush(queue, (
                    initial + self._heuristic(node_id, goal_ids), initial,
                    next(serial), node_id, node_id,
                ))
                generated += 1
        best_total = math.inf
        selected_goal: Optional[int] = None
        selected_root: Optional[int] = None
        expanded = 0
        while queue:
            estimate, cost, _serial, node_id, root_id = heapq.heappop(queue)
            if cost != distance.get(node_id) or root_id != roots.get(node_id):
                continue
            if estimate >= best_total:
                break
            expanded += 1
            if node_id in goal_by_node:
                goal_rank, candidate = goal_by_node[node_id]
                total = cost + ATTACHMENT_RANK_PENALTY_M * goal_rank
                key = (total, int(node_id), int(root_id))
                old_key = (
                    best_total,
                    2**63 - 1 if selected_goal is None else selected_goal,
                    2**63 - 1 if selected_root is None else selected_root,
                )
                if key < old_key:
                    best_total, selected_goal, selected_root = key
            for target, edge, reverse in adjacency.get(node_id, ()): 
                if int(edge.edge_id) in blocked:
                    continue
                candidate_cost = cost + float(edge.length_m)
                old = distance.get(int(target), math.inf)
                old_root = roots.get(int(target), 2**63 - 1)
                if candidate_cost < old or (candidate_cost == old and root_id < old_root):
                    distance[int(target)] = candidate_cost
                    roots[int(target)] = root_id
                    previous[int(target)] = (int(node_id), edge, bool(reverse))
                    heapq.heappush(queue, (
                        candidate_cost + self._heuristic(int(target), goal_ids),
                        candidate_cost, next(serial), int(target), root_id,
                    ))
                    generated += 1

        diagnostics = GraphAStarDiagnostics(
            search_ms=(time.monotonic_ns() - started_ns) / 1.0e6,
            expanded_nodes=expanded,
            generated_nodes=generated,
            blocked_edge_count=len(blocked),
            start_candidate_count=len(starts),
            goal_candidate_count=len(goals),
        )
        if selected_goal is None or selected_root is None:
            return None, starts[0] if starts else None, goals[0] if goals else None, diagnostics
        node_ids = [selected_goal]
        edges: List[TopologyEdge] = []
        reversals: List[bool] = []
        cursor = selected_goal
        while cursor != selected_root:
            parent, edge, reverse = previous[cursor]
            node_ids.append(parent)
            edges.append(edge)
            reversals.append(reverse)
            cursor = parent
        node_ids.reverse()
        edges.reverse()
        reversals.reverse()
        polyline: List[List[float]] = []
        min_width = math.inf
        length = 0.0
        for edge, reverse in zip(edges, reversals):
            points = list(reversed(edge.polyline)) if reverse else list(edge.polyline)
            if polyline and points:
                points = points[1:]
            polyline.extend([[float(point[0]), float(point[1])] for point in points])
            min_width = min(min_width, float(edge.min_width_m))
            length += float(edge.length_m)
        route = TopologyRoute(
            node_ids=node_ids,
            edge_ids=[int(edge.edge_id) for edge in edges],
            length_m=length,
            min_width_m=0.0 if math.isinf(min_width) else min_width,
            polyline=polyline,
        )
        start = root_candidate[selected_root]
        goal = goal_by_node[selected_goal][1]
        return route, start, goal, diagnostics

    def plan(self, query: Any, blocked_cells: Sequence[Cell] = ()) -> Optional[L1Plan]:
        starts = production._attachment_candidates(
            self.artifact, query.start,
            cache_mode=production.CACHE_MODE_OPTIMIZED,
        )
        goals = production._attachment_candidates(
            self.artifact, query.goal,
            cache_mode=production.CACHE_MODE_OPTIMIZED,
        )
        if not starts or not goals:
            return None
        blocked = self.blocked_edges(blocked_cells)
        if not blocked:
            # Preserve the exact validated 2A-r2 initial-route semantics and
            # caches. Dynamic exclusion enters the independent A* only after
            # L2 has proven that the active corridor is disconnected.
            timing: Dict[str, Any] = {}
            _start_attachment, _goal_attachment, route, _reason = (
                production._select_route_with_endpoint_attach(
                    self.artifact, query,
                    cache_mode=production.CACHE_MODE_OPTIMIZED,
                    timing=timing,
                )
            )
            diagnostics = GraphAStarDiagnostics(
                search_ms=float(timing.get("route_search_ms", 0.0)),
                expanded_nodes=int(timing.get("expanded_nodes", 0)),
                generated_nodes=int(timing.get("generated_nodes", 0)),
                blocked_edge_count=0,
                start_candidate_count=len(starts),
                goal_candidate_count=len(goals),
            )
        else:
            route, _start_attachment, _goal_attachment, diagnostics = self._search(
                starts, goals, blocked,
            )
        self.call_count += 1
        self.last_diagnostics = diagnostics
        if route is None:
            return None
        start_cell, goal_cell = production._endpoint_cells(self.ctx, query)
        if start_cell is None or goal_cell is None:
            return None
        corridor, corridor_diagnostics = adaptive.build_adaptive_corridor_mask(
            self.ctx, self.artifact, route, query, start_cell, goal_cell,
            2.0, "raw_map_smac_aligned",
        )
        route_signature = _stable_hash({
            "query": str(query.query_id),
            "route_edge_ids": list(route.edge_ids),
            "blocked_edge_ids": sorted(blocked),
            "corridor_profile": "topology_turn_adaptive_2m_4m",
        })
        return L1Plan(
            static_safe_free=self.artifact.free_mask,
            corridor_mask=corridor,
            start_cell=(int(start_cell[0]), int(start_cell[1])),
            goal_cell=(int(goal_cell[0]), int(goal_cell[1])),
            map_hash=self.map_hash,
            map_origin=tuple(float(value) for value in self.ctx.hospital_map.origin),
            resolution=float(self.ctx.hospital_map.resolution),
            topology_hash=self.topology_hash,
            route_edge_ids=tuple(str(edge_id) for edge_id in route.edge_ids),
            footprint_hash=self.footprint_hash,
            route_signature=route_signature,
            diagnostics={
                "l1_algorithm": "deterministic_graph_astar",
                "l1_call_count": self.call_count,
                "l1_search_ms": diagnostics.search_ms,
                "l1_expanded_nodes": diagnostics.expanded_nodes,
                "l1_generated_nodes": diagnostics.generated_nodes,
                "blocked_edge_ids": sorted(blocked),
                "edge_cell_index_build_ms": self.edge_index.build_ms,
                **dict(corridor_diagnostics),
            },
        )


__all__ = [
    "DeterministicGraphAStarL1", "GraphAStarDiagnostics",
    "TopologyEdgeCellIndex",
]
