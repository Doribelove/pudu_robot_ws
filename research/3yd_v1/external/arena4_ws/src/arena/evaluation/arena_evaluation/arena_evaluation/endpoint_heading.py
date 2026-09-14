"""Bounded yaw-aware endpoint attachment for the 2A-V1 r2 experiment."""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from . import l1_l3_corridor_hybrid_smoke as candidate
from . import unified_four_backends_smoke as legacy


RMIN_M = 0.40
TOP_K = 8
DUBINS_SAMPLE_SPACING_M = 0.05


def _mod2pi(value: float) -> float:
    return float(value) % (2.0 * math.pi)


def _dubins_words(alpha: float, beta: float, distance: float) -> List[Tuple[str, Tuple[float, float, float]]]:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    d2 = distance * distance
    words: List[Tuple[str, Tuple[float, float, float]]] = []

    def add(name: str, values: Tuple[float, float, float], valid: bool = True) -> None:
        if valid and all(math.isfinite(value) and value >= -1.0e-12 for value in values):
            words.append((name, tuple(max(0.0, float(value)) for value in values)))

    tmp = distance + sa - sb
    p2 = 2.0 + d2 - 2.0 * cab + 2.0 * distance * (sa - sb)
    if p2 >= 0.0:
        angle = math.atan2(cb - ca, tmp)
        add("LSL", (_mod2pi(-alpha + angle), math.sqrt(p2), _mod2pi(beta - angle)))

    tmp = distance - sa + sb
    p2 = 2.0 + d2 - 2.0 * cab + 2.0 * distance * (-sa + sb)
    if p2 >= 0.0:
        angle = math.atan2(ca - cb, tmp)
        add("RSR", (_mod2pi(alpha - angle), math.sqrt(p2), _mod2pi(-beta + angle)))

    p2 = -2.0 + d2 + 2.0 * cab + 2.0 * distance * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        angle = math.atan2(-ca - cb, distance + sa + sb) - math.atan2(-2.0, p)
        add("LSR", (_mod2pi(-alpha + angle), p, _mod2pi(-beta + angle)))

    p2 = d2 - 2.0 + 2.0 * cab - 2.0 * distance * (sa + sb)
    if p2 >= 0.0:
        p = math.sqrt(p2)
        angle = math.atan2(ca + cb, distance - sa - sb) - math.atan2(2.0, p)
        add("RSL", (_mod2pi(alpha - angle), p, _mod2pi(beta - angle)))

    value = (6.0 - d2 + 2.0 * cab + 2.0 * distance * (sa - sb)) / 8.0
    if abs(value) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(value))
        t = _mod2pi(alpha - math.atan2(ca - cb, distance - sa + sb) + p / 2.0)
        add("RLR", (t, p, _mod2pi(alpha - beta - t + p)))

    value = (6.0 - d2 + 2.0 * cab + 2.0 * distance * (-sa + sb)) / 8.0
    if abs(value) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(value))
        t = _mod2pi(-alpha - math.atan2(ca - cb, distance + sa - sb) + p / 2.0)
        add("LRL", (t, p, _mod2pi(beta - alpha - t + p)))
    return words


def shortest_dubins(start: Sequence[float], goal: Sequence[float], radius_m: float = RMIN_M) -> Optional[Tuple[str, Tuple[float, float, float]]]:
    dx = float(goal[0]) - float(start[0])
    dy = float(goal[1]) - float(start[1])
    distance = math.hypot(dx, dy) / float(radius_m)
    theta = math.atan2(dy, dx)
    alpha = _mod2pi(float(start[2]) - theta)
    beta = _mod2pi(float(goal[2]) - theta)
    words = _dubins_words(alpha, beta, distance)
    if not words:
        return None
    return min(words, key=lambda item: sum(item[1]))


def sample_dubins(
    start: Sequence[float], goal: Sequence[float], *, radius_m: float = RMIN_M,
    spacing_m: float = DUBINS_SAMPLE_SPACING_M,
) -> Optional[Tuple[List[Tuple[float, float, float]], float, str]]:
    solution = shortest_dubins(start, goal, radius_m)
    if solution is None:
        return None
    word, parameters = solution
    x, y, yaw = float(start[0]), float(start[1]), float(start[2])
    poses: List[Tuple[float, float, float]] = [(x, y, legacy._wrap(yaw))]
    normalized_step = max(1.0e-6, float(spacing_m) / float(radius_m))
    for segment_type, segment_length in zip(word, parameters):
        remaining = float(segment_length)
        while remaining > 1.0e-12:
            step = min(normalized_step, remaining)
            if segment_type == "S":
                x += float(radius_m) * step * math.cos(yaw)
                y += float(radius_m) * step * math.sin(yaw)
            elif segment_type == "L":
                x += float(radius_m) * (math.sin(yaw + step) - math.sin(yaw))
                y += float(radius_m) * (-math.cos(yaw + step) + math.cos(yaw))
                yaw += step
            else:
                x += float(radius_m) * (-math.sin(yaw - step) + math.sin(yaw))
                y += float(radius_m) * (math.cos(yaw - step) - math.cos(yaw))
                yaw -= step
            remaining -= step
            poses.append((x, y, legacy._wrap(yaw)))
    endpoint_error = math.hypot(x - float(goal[0]), y - float(goal[1]))
    yaw_error = abs(legacy._delta(yaw, float(goal[2])))
    if endpoint_error > 1.0e-5 or yaw_error > 1.0e-5:
        return None
    poses[-1] = (float(goal[0]), float(goal[1]), legacy._wrap(float(goal[2])))
    return poses, float(sum(parameters) * radius_m), word


def _route_tangents(route: Any) -> Optional[Tuple[float, float]]:
    points = list(getattr(route, "polyline", []) or [])
    if len(points) < 2:
        return None
    first_index = next((index for index in range(1, len(points)) if math.hypot(points[index][0] - points[0][0], points[index][1] - points[0][1]) > 1.0e-9), None)
    last_index = next((index for index in range(len(points) - 2, -1, -1) if math.hypot(points[-1][0] - points[index][0], points[-1][1] - points[index][1]) > 1.0e-9), None)
    if first_index is None or last_index is None:
        return None
    start_yaw = math.atan2(points[first_index][1] - points[0][1], points[first_index][0] - points[0][0])
    goal_yaw = math.atan2(points[-1][1] - points[last_index][1], points[-1][0] - points[last_index][0])
    return start_yaw, goal_yaw


def _collision_free(topology: Any, poses: Sequence[Sequence[float]]) -> bool:
    hospital_map = topology.hospital_map
    return not any(
        hospital_map.footprint_collision(pose, candidate.FOOTPRINT, unknown_is_collision=True)
        for pose in poses
    )


@dataclass
class EndpointSelection:
    start: Any
    goal: Any
    route: Any
    start_envelope: List[Tuple[float, float, float]]
    goal_envelope: List[Tuple[float, float, float]]
    score: float
    start_yaw_error_rad: float
    goal_yaw_error_rad: float
    start_dubins_word: str
    goal_dubins_word: str


class YawAwareEndpointSelector:
    """Try a bounded Top-K set and require collision-free Dubins attachments."""

    def __init__(self, map_hash: str, *, top_k: int = TOP_K, rmin_m: float = RMIN_M):
        self.map_hash = str(map_hash)
        self.top_k = max(1, int(top_k))
        self.rmin_m = float(rmin_m)
        self.cache: Dict[str, Optional[EndpointSelection]] = {}

    def key(self, query: Any) -> str:
        payload = {
            "map_hash": self.map_hash,
            "footprint": candidate.FOOTPRINT,
            "rmin_m": self.rmin_m,
            "start": [float(value) for value in query.start],
            "goal": [float(value) for value in query.goal],
            "top_k": self.top_k,
        }
        return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()

    def selection(self, query: Any) -> Optional[EndpointSelection]:
        return self.cache.get(self.key(query))

    def __call__(
        self, topology: Any, query: Any, *, cache_mode: str = candidate.CACHE_MODE_OPTIMIZED,
        timing: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Optional[Any], Optional[Any], Optional[Any], str]:
        if cache_mode == candidate.CACHE_MODE_BASELINE:
            return candidate._select_route_with_endpoint_attach(
                topology, query, cache_mode=cache_mode, timing=timing,
            )
        started_ns = time.monotonic_ns()
        cache_key = self.key(query)
        if cache_key in self.cache:
            selected = self.cache[cache_key]
            if timing is not None:
                timing.update({
                    "route_cache_hit": True,
                    "endpoint_candidate_cache_hit": True,
                    "endpoint_yaw_cache_key": cache_key,
                    "endpoint_yaw_cache_hit": True,
                    "endpoint_yaw_selection_ms": 0.0,
                    "candidate_pair_attempts": 0,
                    "start_candidate_count": self.top_k,
                    "goal_candidate_count": self.top_k,
                })
            if selected is None:
                return candidate._select_route_with_endpoint_attach(
                    topology, query, cache_mode=cache_mode, timing=timing,
                )
            return selected.start, selected.goal, selected.route, "yaw_aware_route_cache_hit"

        starts = candidate._attachment_candidates(topology, query.start, limit=self.top_k, cache_mode=cache_mode)
        goals = candidate._attachment_candidates(topology, query.goal, limit=self.top_k, cache_mode=cache_mode)
        pair_attempts = 0
        feasible: List[EndpointSelection] = []
        for start_node in starts:
            for goal_node in goals:
                if int(start_node.component_id) != int(goal_node.component_id):
                    continue
                pair_attempts += 1
                route = legacy.search_topology(topology, int(start_node.node_id), int(goal_node.node_id))
                if route is None:
                    continue
                tangents = _route_tangents(route)
                if tangents is None:
                    continue
                start_pose = (float(start_node.x), float(start_node.y), float(tangents[0]))
                goal_pose = (float(goal_node.x), float(goal_node.y), float(tangents[1]))
                start_connection = sample_dubins(query.start, start_pose, radius_m=self.rmin_m)
                goal_connection = sample_dubins(goal_pose, query.goal, radius_m=self.rmin_m)
                if start_connection is None or goal_connection is None:
                    continue
                if not _collision_free(topology, start_connection[0]) or not _collision_free(topology, goal_connection[0]):
                    continue
                start_yaw_error = abs(legacy._delta(float(query.start[2]), tangents[0]))
                goal_yaw_error = abs(legacy._delta(float(query.goal[2]), tangents[1]))
                connector_length = start_connection[1] + goal_connection[1]
                score = connector_length + self.rmin_m * (start_yaw_error + goal_yaw_error) + 0.01 * float(route.length_m)
                feasible.append(EndpointSelection(
                    start_node, goal_node, route, start_connection[0], goal_connection[0],
                    score, start_yaw_error, goal_yaw_error, start_connection[2], goal_connection[2],
                ))
        selected = min(feasible, key=lambda item: (item.score, int(item.start.node_id), int(item.goal.node_id))) if feasible else None
        if selected is not None:
            # RouteMaskCache keys need to distinguish an endpoint-envelope
            # variant from an otherwise identical baseline topology route.
            # TopologyRoute is an ordinary dataclass, so the experiment-local
            # tag does not modify graph content or persisted topology.
            setattr(selected.route, "r2_endpoint_cache_key", cache_key)
        self.cache[cache_key] = selected
        if timing is not None:
            timing.update({
                "route_search_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                "route_construction_ms": 0.0,
                "adjacency_build_ms": 0.0,
                "topology_adjacency_cache_hit": bool(getattr(getattr(topology, "graph", None), "adjacency_cache_hit", False)),
                "endpoint_spatial_index_cache_hit": True,
                "endpoint_candidate_cache_hit": False,
                "route_cache_hit": False,
                "endpoint_yaw_cache_key": cache_key,
                "endpoint_yaw_cache_hit": False,
                "endpoint_yaw_selection_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                "candidate_pair_attempts": pair_attempts,
                "start_candidate_count": len(starts),
                "goal_candidate_count": len(goals),
                "endpoint_dubins_feasible_pair_count": len(feasible),
                "endpoint_selected_start_node_id": int(selected.start.node_id) if selected else -1,
                "endpoint_selected_goal_node_id": int(selected.goal.node_id) if selected else -1,
                "endpoint_start_yaw_error_rad": selected.start_yaw_error_rad if selected else None,
                "endpoint_goal_yaw_error_rad": selected.goal_yaw_error_rad if selected else None,
                "endpoint_start_dubins_word": selected.start_dubins_word if selected else "",
                "endpoint_goal_dubins_word": selected.goal_dubins_word if selected else "",
            })
        if selected is None:
            # Preserve validity: the r2 feature is not permitted to turn a
            # baseline-attachable request into an L1 failure.
            return candidate._select_route_with_endpoint_attach(
                topology, query, cache_mode=cache_mode, timing=timing,
            )
        return selected.start, selected.goal, selected.route, "yaw_aware_dubins_route"


__all__ = [
    "DUBINS_SAMPLE_SPACING_M", "EndpointSelection", "RMIN_M", "TOP_K",
    "YawAwareEndpointSelector", "sample_dubins", "shortest_dubins",
]
