"""Independent ``3D-V0`` pipeline: L1 Graph A* + L2 D* Lite + L3 Smac.

This module owns the dynamic orchestration only.  The existing ``3A-V0``
static runner remains untouched.  L3 is an adapter boundary: production code
can pass the existing map-level Smac session, while tests inject a deterministic
fake and exercise all L1/L2 state transitions without ROS.
"""

from __future__ import annotations

import hashlib
import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Protocol, Sequence, Set, Tuple

import numpy as np
import cv2

from . import topology
from .dstar_lite import Cell
from .dynamic_snapshot import DynamicSnapshot, apply_dynamic_snapshot, path_intersects_snapshot
from .l2_dstar import L2DStarCorridor, L2PlanResult


ARCHITECTURE_ID = "3D-V0"
IMPLEMENTATION_REVISION = "r1"
RMIN_M = 0.40
MAX_CURVATURE = 2.50


def _raw_corridor_mask(artifact: Any, route: Any, start: Cell, goal: Cell, padding_m: float) -> np.ndarray:
    """Build a raw-occupancy corridor for Smac's single inflation pass.

    L2 still intersects this mask with the footprint-safe static free mask,
    while L3 receives the raw mask so Nav2's costmap is the sole owner of
    footprint inflation.  This avoids double-inflating the corridor.
    """
    hospital_map = artifact.hospital_map
    occupancy = getattr(hospital_map, "occupancy", None)
    raw_free = (
        np.asarray(occupancy == 0, dtype=bool)
        if occupancy is not None else np.asarray(artifact.free_mask, dtype=bool).copy()
    )
    centerline = np.zeros(raw_free.shape, dtype=np.uint8)
    polyline = list(getattr(route, "polyline", []) or [])
    for point in polyline:
        cell = hospital_map.world_to_cell(float(point[0]), float(point[1]))
        if cell is not None:
            centerline[cell] = 1
    for first, second in zip(polyline, polyline[1:]):
        first_cell = hospital_map.world_to_cell(float(first[0]), float(first[1]))
        second_cell = hospital_map.world_to_cell(float(second[0]), float(second[1]))
        if first_cell is not None and second_cell is not None:
            cv2.line(centerline, (int(first_cell[1]), int(first_cell[0])), (int(second_cell[1]), int(second_cell[0])), 1, 1)
    centerline[tuple(start)] = 1
    centerline[tuple(goal)] = 1
    radius = max(1, int(math.ceil(float(padding_m) / float(hospital_map.resolution))))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    return (cv2.dilate(centerline, kernel, iterations=1) > 0) & raw_free


@dataclass(frozen=True)
class L3Result:
    success: bool
    points: Optional[List[Dict[str, float]]] = None
    failure_code: str = ""
    planner_time_ms: float = 0.0
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


class L3Planner(Protocol):
    def plan(
        self,
        query: Any,
        grid_path: Sequence[Cell],
        snapshot: DynamicSnapshot,
        *,
        corridor_mask: np.ndarray,
        topology_artifact: Any,
    ) -> L3Result:
        ...


class SmacHybridAdapter:
    """Adapter around the existing map-level Smac session.

    The adapter intentionally does not synthesize a path when no session is
    available.  A missing backend is reported as ``L3_BACKEND_UNAVAILABLE``.
    """

    def __init__(self, session: Any, backend_spec: Any, *, dynamic_inflation_cells: int = 0) -> None:
        self.session = session
        self.backend_spec = backend_spec
        self.dynamic_inflation_cells = max(0, int(dynamic_inflation_cells))

    def plan(
        self,
        query: Any,
        grid_path: Sequence[Cell],
        snapshot: DynamicSnapshot,
        *,
        corridor_mask: np.ndarray,
        topology_artifact: Any,
    ) -> L3Result:
        if self.session is None or not hasattr(self.session, "plan"):
            return L3Result(False, failure_code="L3_BACKEND_UNAVAILABLE")
        static_free = np.asarray(
            getattr(topology_artifact, "raw_free_mask", topology_artifact.hospital_map.occupancy == 0),
            dtype=bool,
        )
        dynamic_free, _costs, _changed = apply_dynamic_snapshot(
            static_free, snapshot, inflation_radius_cells=self.dynamic_inflation_cells,
        )
        allowed = np.asarray(corridor_mask, dtype=bool) & dynamic_free
        started = time.monotonic_ns()
        try:
            result = self.session.plan(query, self.backend_spec, source="3d_v0_l3_smac", allowed_mask=allowed)
        except Exception as exc:  # pragma: no cover - ROS-specific path
            return L3Result(
                False,
                failure_code="L3_EXCEPTION",
                planner_time_ms=(time.monotonic_ns() - started) / 1.0e6,
                diagnostics={"detail": str(exc), "backend_called": False, "backend_call_count": 0},
            )
        points = [dict(point) for point in (getattr(result, "points", None) or [])]
        success = bool(getattr(result, "planner_success", False) and points)
        session_diagnostics = dict(getattr(result, "diagnostics", {}) or {})
        return L3Result(
            success, points if success else None,
            "" if success else str(getattr(result, "failure_code", "L3_PLANNER_FAILED") or "L3_PLANNER_FAILED"),
            (time.monotonic_ns() - started) / 1.0e6,
            {
                **session_diagnostics,
                "backend": getattr(self.backend_spec, "backend", "SmacPlannerHybrid"),
                "snapshot_id": snapshot.snapshot_id,
                "backend_called": bool(session_diagnostics.get("backend_called", True)),
                "backend_call_count": int(session_diagnostics.get("backend_call_count") or 1),
            },
        )


@dataclass(frozen=True)
class LayeredDynamicResult:
    success: bool
    grid_path: Optional[List[Cell]]
    points: Optional[List[Dict[str, float]]]
    failure_code: str = ""
    snapshot_id: str = ""
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


def _cell_path_to_points(artifact: Any, path: Sequence[Cell], start_yaw: float, goal_yaw: float) -> List[Dict[str, float]]:
    points = topology.cells_to_poses(artifact, path, float(start_yaw), float(goal_yaw))
    for point in points:
        point["source"] = "l2_dstar"  # type: ignore[assignment]
    return points


class LayeredDynamicPipeline:
    """One static topology and one persistent D* Lite state per corridor."""

    def __init__(
        self,
        artifact: Any,
        *,
        footprint: Sequence[Sequence[float]],
        l3_planner: Optional[L3Planner] = None,
        corridor_padding_m: float = 2.0,
        dynamic_inflation_cells: int = 0,
        endpoint_radius_m: float = 5.0,
        endpoint_candidate_limit: int = 16,
        corridor_padding_schedule_m: Optional[Sequence[float]] = None,
    ) -> None:
        self.artifact = artifact
        self.footprint = footprint
        self.l3_planner = l3_planner
        self.corridor_padding_m = float(corridor_padding_m)
        schedule = corridor_padding_schedule_m if corridor_padding_schedule_m is not None else (self.corridor_padding_m,)
        self.corridor_padding_schedule_m = tuple(
            sorted({float(value) for value in schedule if float(value) > 0.0})
        ) or (self.corridor_padding_m,)
        self.dynamic_inflation_cells = max(0, int(dynamic_inflation_cells))
        self.endpoint_radius_m = float(endpoint_radius_m)
        self.endpoint_candidate_limit = max(1, int(endpoint_candidate_limit))
        self.node_index = topology.NodeSpatialIndex.build(artifact.graph.nodes)
        self.adjacency = artifact.graph.adjacency()
        # Endpoint lookup and static route selection are pure functions of
        # the map, footprint and poses.  Keep them in memory for repeated
        # A2B requests; dynamic occupancy only affects L2/L3 below.
        self._endpoint_candidate_cache: Dict[Tuple[float, float, float], List[Any]] = {}
        self._route_cache: Dict[Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[int, ...]], Any] = {}
        self.current_route: Optional[Any] = None
        self.current_l2: Optional[L2DStarCorridor] = None
        self.current_path: Optional[List[Cell]] = None
        self.current_result: Optional[LayeredDynamicResult] = None
        self.current_snapshot = DynamicSnapshot.empty(map_shape=artifact.free_mask.shape)
        self.edge_states: Dict[int, str] = {}
        self.l1_search_count = 0
        self.l1_reroute_count = 0
        self.l2_reset_count = 0

    def _candidates(self, pose: Sequence[float]) -> List[Any]:
        key = tuple(round(float(value), 6) for value in pose[:3])
        cached = self._endpoint_candidate_cache.get(key)
        if cached is not None:
            return list(cached)
        candidates = []
        for node in self.node_index.query(float(pose[0]), float(pose[1]), self.endpoint_radius_m):
            cell = (int(node.pixel_y), int(node.pixel_x))
            if not self.artifact.free_mask[cell]:
                continue
            if self.artifact.hospital_map.footprint_collision(
                (node.x, node.y, float(pose[2])), self.footprint, unknown_is_collision=True,
            ):
                continue
            candidates.append(node)
        candidates.sort(key=lambda node: (math.hypot(node.x - float(pose[0]), node.y - float(pose[1])), int(node.node_id)))
        selected = candidates[: self.endpoint_candidate_limit]
        self._endpoint_candidate_cache[key] = list(selected)
        return selected

    def _search_route_blocked(self, starts: Sequence[int], goals: Sequence[int], blocked: Set[int]) -> Tuple[Optional[Any], Optional[int], Optional[int]]:
        starts = list(dict.fromkeys(int(value) for value in starts))
        goals_list = list(dict.fromkeys(int(value) for value in goals))
        goals_set = set(goals_list)
        node_by_id = {int(node.node_id): node for node in self.artifact.graph.nodes}

        def heuristic(node_id: int) -> float:
            node = node_by_id.get(int(node_id))
            goal_nodes = [node_by_id[goal] for goal in goals_list if goal in node_by_id]
            if node is None or not goal_nodes:
                return 0.0
            return min(
                math.hypot(float(node.x) - float(goal.x), float(node.y) - float(goal.y))
                for goal in goal_nodes
            )

        # The first key is f=g+h, the second is g.  Euclidean distance is
        # admissible for positive geometric edge lengths and keeps the L1
        # search an actual deterministic Graph A* rather than a full Dijkstra.
        queue: List[Tuple[float, float, int, int]] = []
        distance: Dict[int, float] = {}
        previous: Dict[int, Tuple[int, Any, bool]] = {}
        roots: Dict[int, int] = {}
        for node_id in sorted(starts):
            distance[node_id] = 0.0
            roots[node_id] = node_id
            heapq.heappush(queue, (heuristic(node_id), 0.0, node_id, node_id))
        while queue:
            _priority, cost, node_id, root = heapq.heappop(queue)
            if cost != distance.get(node_id) or roots.get(node_id) != root:
                continue
            if node_id in goals_set:
                route = topology._route_from_search(self.artifact, root, node_id, distance, previous)
                return route, root, node_id
            for target, edge, reverse in self.adjacency.get(node_id, []):
                if int(edge.edge_id) in blocked:
                    continue
                candidate = cost + float(edge.length_m)
                if candidate < distance.get(target, float("inf")):
                    distance[target] = candidate
                    roots[target] = root
                    previous[target] = (node_id, edge, reverse)
                    heapq.heappush(queue, (candidate + heuristic(int(target)), candidate, int(target), root))
        return None, None, None

    def _select_route(self, query: Any, *, blocked_edges: Optional[Set[int]] = None) -> Tuple[Optional[Any], Dict[str, Any]]:
        started = time.monotonic_ns()
        blocked = tuple(sorted(int(value) for value in (blocked_edges or set())))
        route_key = (
            tuple(round(float(value), 6) for value in query.start[:3]),
            tuple(round(float(value), 6) for value in query.goal[:3]),
            blocked,
        )
        cached_route = self._route_cache.get(route_key)
        if cached_route is not None:
            cached_nodes = list(getattr(cached_route, "node_ids", []) or [])
            return cached_route, {
                "l1_start_candidate_count": 0,
                "l1_goal_candidate_count": 0,
                "endpoint_spatial_index_cache_hit": True,
                "topology_adjacency_cache_hit": True,
                "route_cache_hit": True,
                "l1_success": bool(cached_nodes),
                "selected_start_node": cached_nodes[0] if cached_nodes else None,
                "selected_goal_node": cached_nodes[-1] if cached_nodes else None,
                "l1_route_search_ms": 0.0,
                "l1_candidate_pair_attempts": 0,
            }
        endpoint_started = time.monotonic_ns()
        starts = self._candidates(query.start)
        goals = self._candidates(query.goal)
        endpoint_lookup_ms = (time.monotonic_ns() - endpoint_started) / 1.0e6
        # A short endpoint radius can make the same skeleton node appear in
        # both sets for a long query.  That would make the multi-source search
        # return a zero-length route even though the poses are far apart.
        endpoint_distance = math.hypot(
            float(query.goal[0]) - float(query.start[0]),
            float(query.goal[1]) - float(query.start[1]),
        )
        if endpoint_distance > max(1.0, 2.0 * self.endpoint_radius_m):
            start_ids = {int(item.node_id) for item in starts}
            filtered_goals = [item for item in goals if int(item.node_id) not in start_ids]
            if filtered_goals:
                goals = filtered_goals
        diagnostics: Dict[str, Any] = {
            "l1_start_candidate_count": len(starts),
            "l1_goal_candidate_count": len(goals),
            "endpoint_spatial_index_cache_hit": True,
            "topology_adjacency_cache_hit": bool(self.artifact.graph.adjacency_cache_hit),
        }
        if not starts or not goals:
            diagnostics.update({"l1_time_ms": (time.monotonic_ns() - started) / 1.0e6, "failure_code": "L1_ENDPOINT_NOT_ATTACHABLE"})
            return None, diagnostics
        blocked_set = set(blocked)
        route, selected_start, selected_goal = self._search_route_blocked(
            [item.node_id for item in starts], [item.node_id for item in goals], blocked_set,
        )
        self.l1_search_count += 1
        diagnostics.update({
            "l1_route_search_ms": (time.monotonic_ns() - started) / 1.0e6,
            "l1_attachment_lookup_ms": endpoint_lookup_ms,
            "l1_candidate_pair_attempts": 1,
            "selected_start_node": selected_start,
            "selected_goal_node": selected_goal,
            "l1_success": route is not None,
            "route_cache_hit": False,
        })
        if route is None:
            diagnostics["failure_code"] = "L1_NO_ROUTE"
        else:
            self._route_cache[route_key] = route
        return route, diagnostics

    def _new_l2(self, route: Any, query: Any, padding_m: Optional[float] = None) -> L2DStarCorridor:
        start_cell = self.artifact.hospital_map.world_to_cell(query.start[0], query.start[1])
        goal_cell = self.artifact.hospital_map.world_to_cell(query.goal[0], query.goal[1])
        if start_cell is None or goal_cell is None:
            raise ValueError("query endpoint outside map")
        padding = self.corridor_padding_m if padding_m is None else float(padding_m)
        mask = _raw_corridor_mask(self.artifact, route, start_cell, goal_cell, padding).astype(bool)
        return L2DStarCorridor(
            np.asarray(self.artifact.free_mask, dtype=bool), mask, start_cell, goal_cell,
            corridor_id=hashlib.sha256(json_bytes(route.edge_ids)).hexdigest()[:16],
            dynamic_inflation_cells=self.dynamic_inflation_cells,
        )

    def plan_initial(self, query: Any, snapshot: Optional[DynamicSnapshot] = None, *, timeout_s: Optional[float] = None) -> LayeredDynamicResult:
        snap = snapshot or DynamicSnapshot.empty(map_shape=self.artifact.free_mask.shape)
        route, l1_diag = self._select_route(query)
        if route is None:
            result = LayeredDynamicResult(False, None, None, "L1_NO_ROUTE", snap.snapshot_id, l1_diag)
            self.current_result = result
            return result
        l2_attempts: List[Dict[str, Any]] = []
        l2 = None
        l2_init_ms = 0.0
        l2_result = None
        for padding_m in self.corridor_padding_schedule_m:
            try:
                l2_init_started = time.monotonic_ns()
                candidate_l2 = self._new_l2(route, query, padding_m)
                candidate_init_ms = (time.monotonic_ns() - l2_init_started) / 1.0e6
                candidate_result = candidate_l2.plan(snapshot=snap, timeout_s=timeout_s)
            except (ValueError, IndexError) as exc:
                l2_attempts.append({"padding_m": padding_m, "success": False, "failure_code": "L2_CORRIDOR_INVALID", "detail": str(exc)})
                continue
            l2_attempts.append({
                "padding_m": padding_m, "success": bool(candidate_result.success),
                "failure_code": candidate_result.failure_code, "expanded_nodes": candidate_result.stats.expanded_nodes,
                "search_time_ms": candidate_result.stats.search_time_ms,
            })
            l2, l2_result, l2_init_ms = candidate_l2, candidate_result, candidate_init_ms
            if candidate_result.success:
                break
        if l2 is None or l2_result is None:
            result = LayeredDynamicResult(False, None, None, "L2_CORRIDOR_INVALID", snap.snapshot_id, {**l1_diag, "l2_attempts": l2_attempts})
            self.current_route, self.current_l2, self.current_path, self.current_snapshot = route, None, None, snap
            self.current_result = result
            return result
        diagnostics = {
            **l1_diag,
            "l2": dict(l2_result.diagnostics),
            "l2_initialization_ms": float(l2_init_ms),
            "l2_corridor_padding_m": float(l2_attempts[-1]["padding_m"]),
            "l2_attempts": l2_attempts,
            "l2_fallback_attempt_count": max(0, len(l2_attempts) - 1),
            "l2_expanded_nodes": l2_result.stats.expanded_nodes,
            "l2_generated_nodes": l2_result.stats.generated_nodes,
            "l2_search_time_ms": l2_result.stats.search_time_ms,
            "l2_extract_path_ms": float((l2_result.diagnostics or {}).get("dstar_extract_path_ms") or 0.0),
            "dstar_initial_queue_size": l2_result.stats.initial_queue_size,
            "dstar_final_queue_size": l2_result.stats.final_queue_size,
            "dstar_queue_push_count": l2_result.stats.queue_pushes,
            "dstar_queue_pop_count": l2_result.stats.queue_pops,
            "dstar_update_vertex_count": l2_result.stats.update_vertex_count,
            "l2_reset_count": 0,
        }
        if not l2_result.success or l2_result.path is None:
            result = LayeredDynamicResult(False, None, None, l2_result.failure_code, snap.snapshot_id, diagnostics)
            self.current_route, self.current_l2, self.current_path, self.current_snapshot = route, l2, None, snap
            self.current_result = result
            return result
        points = _cell_path_to_points(self.artifact, l2_result.path, query.start[2], query.goal[2])
        l3_result = self._run_l3(query, l2_result.path, snap, l2.corridor_mask)
        if l3_result.success and not self._dynamic_points_valid(l3_result.points, snap):
            l3_result = L3Result(False, failure_code="DYNAMIC_FOOTPRINT_COLLISION", planner_time_ms=l3_result.planner_time_ms, diagnostics=l3_result.diagnostics)
        diagnostics.update({"l3": dict(l3_result.diagnostics), "l3_planner_time_ms": l3_result.planner_time_ms, "l3_called": self.l3_planner is not None})
        if not l3_result.success:
            result = LayeredDynamicResult(False, l2_result.path, None, l3_result.failure_code or "L3_PLANNER_FAILED", snap.snapshot_id, diagnostics)
        else:
            result = LayeredDynamicResult(True, l2_result.path, l3_result.points or points, "", snap.snapshot_id, diagnostics)
        self.current_route, self.current_l2, self.current_path, self.current_snapshot = route, l2, l2_result.path, snap
        self.current_result = result
        return result

    def _run_l3(self, query: Any, path: Sequence[Cell], snapshot: DynamicSnapshot, corridor_mask: np.ndarray) -> L3Result:
        if self.l3_planner is None:
            return L3Result(False, failure_code="L3_BACKEND_UNAVAILABLE")
        return self.l3_planner.plan(query, path, snapshot, corridor_mask=corridor_mask, topology_artifact=self.artifact)

    def _dynamic_points_valid(self, points: Optional[Sequence[Mapping[str, Any]]], snapshot: DynamicSnapshot) -> bool:
        if not points:
            return False
        occupied = set(snapshot.inflated_cells(self.dynamic_inflation_cells))
        for point in points:
            cell = self.artifact.hospital_map.world_to_cell(float(point["x"]), float(point["y"]))
            if cell is None or tuple(cell) in occupied:
                return False
        return True

    def update_dynamic(
        self,
        query: Any,
        snapshot: DynamicSnapshot,
        *,
        current_index: int = 0,
        timeout_s: Optional[float] = None,
    ) -> LayeredDynamicResult:
        """Process one snapshot; no global update is done when path is clear."""
        if self.current_l2 is None or self.current_path is None or self.current_route is None:
            return self.plan_initial(query, snapshot, timeout_s=timeout_s)
        ahead = max(0, int(current_index))
        triggered = path_intersects_snapshot(
            self.current_path, snapshot, inflation_radius_cells=self.dynamic_inflation_cells, ahead_from_index=ahead,
        )
        base_diag = {
            "snapshot_id": snapshot.snapshot_id,
            "dynamic_replan_triggered": bool(triggered),
            "path_intersection": bool(path_intersects_snapshot(self.current_path, snapshot, inflation_radius_cells=self.dynamic_inflation_cells)),
            "ahead_region_intersection": bool(triggered),
            "local_control_can_handle": not triggered,
            "l1_reroute_count": self.l1_reroute_count,
        }
        if not triggered:
            if any(value == "TEMPORARILY_BLOCKED" for value in self.edge_states.values()):
                self.edge_states = {
                    edge_id: ("RECOVERED" if value == "TEMPORARILY_BLOCKED" else value)
                    for edge_id, value in self.edge_states.items()
                }
            result = LayeredDynamicResult(True, list(self.current_path), self.current_result.points if self.current_result else None, "", snapshot.snapshot_id, base_diag)
            self.current_snapshot = snapshot
            self.current_result = result
            return result
        l2_result = self.current_l2.repair_path(
            self.current_path, current_index=ahead, snapshot=snapshot, timeout_s=timeout_s,
        )
        if l2_result.success and l2_result.path is not None:
            for edge_id in self.current_route.edge_ids or []:
                if self.edge_states.get(int(edge_id)) == "TEMPORARILY_BLOCKED":
                    self.edge_states[int(edge_id)] = "RECOVERED"
            l3_result = self._run_l3(query, l2_result.path, snapshot, self.current_l2.corridor_mask)
            if l3_result.success and not self._dynamic_points_valid(l3_result.points, snapshot):
                l3_result = L3Result(False, failure_code="DYNAMIC_FOOTPRINT_COLLISION", planner_time_ms=l3_result.planner_time_ms, diagnostics=l3_result.diagnostics)
            diagnostics = {**base_diag, "l2_dynamic_update": True, "l2_changed_cells": l2_result.changed_cells, "l2_expanded_nodes": l2_result.expanded_cells, "l2_path_changed": l2_result.path_changed, "l3": dict(l3_result.diagnostics), "l3_planner_time_ms": l3_result.planner_time_ms}
            result = LayeredDynamicResult(bool(l3_result.success), l2_result.path, l3_result.points, "" if l3_result.success else (l3_result.failure_code or "L3_PLANNER_FAILED"), snapshot.snapshot_id, diagnostics)
            self.current_path, self.current_snapshot, self.current_result = l2_result.path, snapshot, result
            return result
        # A corridor-level failure is the only condition that may trigger L1.
        blocked = {int(edge_id) for edge_id in (self.current_route.edge_ids or [])}
        self.edge_states.update({edge_id: "TEMPORARILY_BLOCKED" for edge_id in blocked})
        self.l1_reroute_count += 1
        route, l1_diag = self._select_route(query, blocked_edges=blocked)
        if route is None:
            result = LayeredDynamicResult(False, None, None, l2_result.failure_code or "L1_NO_ROUTE", snapshot.snapshot_id, {**base_diag, **l1_diag, "l1_reroute": True})
            self.current_snapshot, self.current_result = snapshot, result
            return result
        new_l2 = self._new_l2(route, query)
        self.l2_reset_count += 1
        new_l2_result = new_l2.plan(snapshot=snapshot, timeout_s=timeout_s)
        if not new_l2_result.success or new_l2_result.path is None:
            result = LayeredDynamicResult(False, None, None, new_l2_result.failure_code, snapshot.snapshot_id, {**base_diag, **l1_diag, "l1_reroute": True, "l2_reset": True})
            self.current_route, self.current_l2, self.current_snapshot, self.current_result = route, new_l2, snapshot, result
            return result
        l3_result = self._run_l3(query, new_l2_result.path, snapshot, new_l2.corridor_mask)
        if l3_result.success and not self._dynamic_points_valid(l3_result.points, snapshot):
            l3_result = L3Result(False, failure_code="DYNAMIC_FOOTPRINT_COLLISION", planner_time_ms=l3_result.planner_time_ms, diagnostics=l3_result.diagnostics)
        result = LayeredDynamicResult(bool(l3_result.success), new_l2_result.path, l3_result.points, "" if l3_result.success else (l3_result.failure_code or "L3_PLANNER_FAILED"), snapshot.snapshot_id, {**base_diag, **l1_diag, "l1_reroute": True, "l2_reset": True, "l2_expanded_nodes": new_l2_result.expanded_cells, "l3": dict(l3_result.diagnostics)})
        self.current_route, self.current_l2, self.current_path, self.current_snapshot, self.current_result = route, new_l2, new_l2_result.path, snapshot, result
        return result


def json_bytes(value: Any) -> bytes:
    import json
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


__all__ = [
    "ARCHITECTURE_ID", "IMPLEMENTATION_REVISION", "L3Result", "SmacHybridAdapter",
    "LayeredDynamicPipeline", "LayeredDynamicResult",
]


def main(argv=None) -> int:
    """Delegate the console contract without importing CLI dependencies here."""
    from .layered_dynamic_pipeline_cli import main as cli_main
    return cli_main(argv)
