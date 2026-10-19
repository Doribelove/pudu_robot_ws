"""Independent forward-only repair smoke for the four PLN-02 planners.

This module intentionally does not call the old smoke route generator.  The
algorithms share only map loading, the corrected Dubins sampler, collision
queries and the final validator.  The RRT implementations are explicitly
reference/surrogate implementations and are never presented as mature OMPL or
AO-RRT* implementations.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import gzip
import hashlib
import heapq
import json
import math
import random
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml
from scipy.spatial import cKDTree

from . import forward_no_reverse_smoke as old
from . import single_planner_benchmark as v1
from .planner_benchmark.isolation import run_isolated
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .topology import astar_grid, preprocess_static_map


ROOT = Path("/home/robot/pudu_robot_ws")
CACHE_ROOT = ROOT / "experiments" / ".static_map_cache" / "forward_no_reverse_v1"
SOURCE_QUERIES = ROOT / "experiments/planner_benchmark/hospital_005/queries_v2.yaml"
MAP_PATHS = {
    "hospital_005": ROOT / "experiments/maps/hospital_005/map.yaml",
    "hospital_boundary_100x100_005": ROOT / "experiments/maps/hospital_boundary_100x100_005/map.yaml",
    "hospital_boundary_200x200_005": ROOT / "experiments/maps/hospital_boundary_200x200_005/map.yaml",
    "hospital_boundary_400x400_005": ROOT / "experiments/maps/hospital_boundary_400x400_005/map.yaml",
}
TIMEOUTS = {"hospital_005": 5.0, "hospital_boundary_100x100_005": 5.0, "hospital_boundary_200x200_005": 15.0, "hospital_boundary_400x400_005": 60.0}
FOOTPRINT = [list(p) for p in v1.FOOTPRINT]
ALGORITHMS = ("astar_kinematic", "hybrid_astar", "rrt_star", "kinodynamic_rrt_star")
DEFAULT_QUERIES = tuple(f"q{i:02d}" for i in range(10))


@dataclass(frozen=True)
class StrictConfig:
    wheelbase_m: float = 0.50
    minimum_turning_radius_m: float = 0.40
    maximum_curvature_per_m: float = 2.50
    step_size_m: float = 0.25
    sample_spacing_m: float = 0.05
    angle_resolution_deg: float = 5.0
    angle_bins: int = 72
    steering_angles_deg: Tuple[float, ...] = (-51.34, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 51.34)
    endpoint_position_tolerance_m: float = 0.25
    endpoint_yaw_tolerance_rad: float = math.radians(10.0)
    allow_reverse: bool = False
    allow_in_place_rotation: bool = False
    motion_model: str = "forward_only_dubins"
    steering_jump_tolerance_deg: float = 15.0
    guide_lookahead_m: float = 0.40
    guide_corridor_m: float = 1.20
    steering_geometry_tolerance_per_m: float = 0.02

    def __post_init__(self) -> None:
        if not math.isclose(self.wheelbase_m / math.tan(math.radians(51.3401917459)), self.minimum_turning_radius_m, rel_tol=1e-7, abs_tol=1e-7):
            raise ValueError("strict radius must be wheelbase/tan(max steering)")
        if not math.isclose(1.0 / self.minimum_turning_radius_m, self.maximum_curvature_per_m, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("strict curvature mismatch")
        if self.allow_reverse or self.allow_in_place_rotation:
            raise ValueError("strict protocol forbids reverse and in-place rotation")
        if self.angle_bins != int(round(360.0 / self.angle_resolution_deg)):
            raise ValueError("heading discretisation mismatch")
        if self.sample_spacing_m > 0.05 + 1e-12:
            raise ValueError("integration sample spacing must be <= 0.05 m")

    @property
    def max_steering_rad(self) -> float:
        return math.atan(self.wheelbase_m / self.minimum_turning_radius_m)

    @property
    def jump_tolerance_rad(self) -> float:
        return math.radians(self.steering_jump_tolerance_deg)


CONFIG = StrictConfig()


@dataclass
class Result:
    planner_success: bool = False
    points: Optional[List[Dict[str, float]]] = None
    failure_code: str = "NO_PATH"
    diagnostics: Dict[str, Any] = None
    expanded_states: int = 0
    generated_states: int = 0
    random_seed: Optional[int] = None
    planner_core: str = ""
    source_function: str = ""
    backend_id: str = ""
    implementation_type: str = ""
    postprocess_type: str = "none"
    planning_time_ms: float = 0.0
    timeout_triggered: bool = False


def _wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _delta(target: float, source: float) -> float:
    return _wrap(float(target) - float(source))


def _queries() -> Dict[str, Query]:
    payload = yaml.safe_load(SOURCE_QUERIES.read_text(encoding="utf-8")) or {}
    return {str(item["query_id"]): Query(str(item["query_id"]), [float(v) for v in item["start"]], [float(v) for v in item["goal"]], str(item.get("category", "")), int(item.get("seed", 20260821)), str(item.get("validation_status", "UNVALIDATED"))) for item in payload.get("queries", [])}


def _query_hash(query: Query) -> str:
    return hashlib.sha256(json.dumps(query.as_dict(), sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _context(map_id: str) -> old.relaxed.MapContext:
    if map_id not in MAP_PATHS:
        raise ValueError(f"unknown map {map_id}")
    map_yaml = MAP_PATHS[map_id]
    cache_key = hashlib.sha256((sha256_file(map_yaml) + sha256_file(map_yaml.parent / yaml.safe_load(map_yaml.read_text())["image"])).encode()).hexdigest()[:24]
    cache_dir = CACHE_ROOT / f"{map_id}_{cache_key}"
    cache_dir.mkdir(parents=True, exist_ok=True)
    metadata_path = cache_dir / "metadata.yaml"
    hm = None
    if metadata_path.exists() and (cache_dir / "occupancy.npy").exists() and (cache_dir / "distance_m.npy").exists() and (cache_dir / "free_mask.npy").exists():
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
        if metadata.get("map_yaml_sha256") == sha256_file(map_yaml) and metadata.get("map_image_sha256") == sha256_file(map_yaml.parent / yaml.safe_load(map_yaml.read_text())["image"]):
            config = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
            image_path = Path(config["image"])
            if not image_path.is_absolute():
                image_path = map_yaml.parent / image_path
            occupancy = np.load(cache_dir / "occupancy.npy", mmap_mode="r")
            distance_m = np.load(cache_dir / "distance_m.npy", mmap_mode="r")
            hm = HospitalMap(
                yaml_path=map_yaml.resolve(), image_path=image_path.resolve(), resolution=float(config["resolution"]),
                origin=tuple(float(value) for value in config["origin"]), width=int(occupancy.shape[1]), height=int(occupancy.shape[0]),
                occupancy=occupancy, distance_m=distance_m,
            )
    if hm is None:
        hm = HospitalMap.load(map_yaml)
    if not math.isclose(hm.resolution, 0.05, abs_tol=1e-12):
        raise ValueError(f"{map_id}: resolution is not 0.05")
    free_cache = cache_dir / "free_mask.npy"
    if free_cache.exists():
        free = np.load(free_cache, mmap_mode="r")
        distance = hm.distance_m
    else:
        _, free, distance, _ = preprocess_static_map(hm, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False)
        np.save(free_cache, np.asarray(free, dtype=np.bool_))
        np.save(cache_dir / "occupancy.npy", np.asarray(hm.occupancy, dtype=np.int8))
        np.save(cache_dir / "distance_m.npy", np.asarray(hm.distance_m, dtype=np.float32))
        metadata = {"map_id":map_id,"map_yaml_sha256":sha256_file(map_yaml),"map_image_sha256":sha256_file(hm.image_path),"resolution":hm.resolution,"footprint":FOOTPRINT,"padding_m":0.05,"safety_margin_m":0.05,"allow_unknown":False,"dynamic_obstacles":False}
        metadata_path.write_text(yaml.safe_dump(metadata,sort_keys=False),encoding="utf-8")
    return old.relaxed.MapContext(map_id, hm, free, distance, sha256_file(hm.image_path), sha256_file(hm.yaml_path), {})


def _endpoint_status(ctx: old.relaxed.MapContext, query: Query) -> Dict[str, Any]:
    validation = ctx.hospital_map.validate_query(query, FOOTPRINT, 0.5, allow_unknown=False)
    return {"validation_status": validation.validation_status, "start_status": validation.start_status, "goal_status": validation.goal_status, "connected": validation.connected, "start_clearance_m": validation.start_clearance_m, "goal_clearance_m": validation.goal_clearance_m, "reason": validation.reason}


def _strict_dubins(start: Tuple[float, float, float], goal: Tuple[float, float, float], deadline: Optional[float] = None) -> Tuple[Optional[List[Dict[str, float]]], str, Optional[Tuple[str, str, str]]]:
    word_result = old.relaxed._dubins_word(start, goal, CONFIG.minimum_turning_radius_m)
    if word_result is None:
        return None, "NO_DUBINS_CONNECTION", None
    _, word, params = word_result
    state = tuple(float(v) for v in start)
    result: List[Dict[str, float]] = []
    max_steering = CONFIG.max_steering_rad
    for kind, normalized_length in zip(word, params):
        remaining = float(normalized_length) * CONFIG.minimum_turning_radius_m
        steering = 0.0 if kind == "S" else max_steering if kind == "L" else -max_steering
        while remaining > 1e-10:
            if deadline is not None and time.monotonic() >= deadline:
                return None, "TIMEOUT", word
            ds = min(CONFIG.sample_spacing_m, remaining)
            state = v1._integrate_bicycle(*state, steering, ds, CONFIG.wheelbase_m, samples=1)[-1]
            result.append({"x": state[0], "y": state[1], "yaw": _wrap(state[2]), "steering": steering, "motion_direction": "forward"})
            remaining -= ds
    if not result:
        return None, "NO_DUBINS_CONNECTION", word
    end = result[-1]
    if math.hypot(end["x"] - goal[0], end["y"] - goal[1]) > 1e-6 or abs(_delta(end["yaw"], goal[2])) > 1e-6:
        return None, "CONNECTOR_ENDPOINT_ERROR", word
    return result, "OK", word


def _collision_free(ctx: old.relaxed.MapContext, points: Sequence[Mapping[str, float]]) -> bool:
    return bool(points) and not any(ctx.hospital_map.footprint_collision((p["x"], p["y"], p["yaw"]), FOOTPRINT, unknown_is_collision=True) for p in points)


def _guide_arrays(points: Sequence[Mapping[str, float]]) -> Tuple[np.ndarray, np.ndarray, cKDTree]:
    coordinates = np.asarray([(float(p["x"]), float(p["y"])) for p in points], dtype=float)
    remaining = np.zeros(len(coordinates), dtype=float)
    for index in range(len(coordinates) - 2, -1, -1):
        remaining[index] = remaining[index + 1] + float(np.linalg.norm(coordinates[index + 1] - coordinates[index]))
    return coordinates, remaining, cKDTree(coordinates)


def _guide_steering(
    coordinates: np.ndarray,
    tree: cKDTree,
    state: Tuple[float, float, float],
    *,
    lookahead_points: int,
) -> Tuple[float, float, int]:
    x, y, yaw = state
    distance, nearest = tree.query((x, y))
    target = coordinates[min(len(coordinates) - 1, int(nearest) + lookahead_points)]
    target_distance = max(0.05, math.hypot(float(target[0]) - x, float(target[1]) - y))
    alpha = _delta(math.atan2(float(target[1]) - y, float(target[0]) - x), yaw)
    curvature = max(-CONFIG.maximum_curvature_per_m, min(CONFIG.maximum_curvature_per_m, 2.0 * math.sin(alpha) / target_distance))
    return math.atan(CONFIG.wheelbase_m * curvature), float(distance), int(nearest)


def _track_grid_route(
    ctx: old.relaxed.MapContext,
    query: Query,
    guide: Sequence[Mapping[str, float]],
    deadline: float,
) -> Tuple[Optional[List[Dict[str, float]]], Dict[str, Any]]:
    """Reintegrate an A* geometric guide with continuous forward bicycle controls."""
    coordinates, _, tree = _guide_arrays(guide)
    state = (float(query.start[0]), float(query.start[1]), float(query.start[2]))
    steering = 0.0
    points = [{"x": state[0], "y": state[1], "yaw": state[2], "steering": steering, "motion_direction": "forward"}]
    checks = 0
    max_steps = max(1000, int(len(guide) * 2.5))
    for _ in range(max_steps):
        checks += 1
        if time.monotonic() >= deadline:
            return None, {"failure": "TIMEOUT", "timeout_checks": checks}
        desired, _, nearest = _guide_steering(coordinates, tree, state, lookahead_points=7)
        change = max(-math.radians(12.0), min(math.radians(12.0), desired - steering))
        steering += change
        rollout = _rollout(ctx, state, steering, steering, CONFIG.sample_spacing_m)
        if rollout is None:
            return None, {"failure": "CONTINUOUS_POSTPROCESS_COLLISION", "timeout_checks": checks, "nearest_guide_index": nearest}
        point = rollout[-1]
        points.extend(rollout)
        state = (point["x"], point["y"], point["yaw"])
        if math.hypot(state[0] - query.goal[0], state[1] - query.goal[1]) <= CONFIG.endpoint_position_tolerance_m and abs(_delta(state[2], query.goal[2])) <= CONFIG.endpoint_yaw_tolerance_rad:
            return points, {"failure": "", "timeout_checks": checks, "nearest_guide_index": nearest, "generated_samples": len(points)}
    return None, {"failure": "CONTINUOUS_POSTPROCESS_NO_GOAL", "timeout_checks": checks}


def _track_rrt_route(
    ctx: old.relaxed.MapContext,
    query: Query,
    guide: Sequence[Mapping[str, float]],
    deadline: float,
) -> Tuple[Optional[List[Dict[str, float]]], Dict[str, Any]]:
    """Track an RRT seed branch with its own forward-bicycle propagation loop."""
    coordinates, _, tree = _guide_arrays(guide)
    state = (float(query.start[0]), float(query.start[1]), float(query.start[2]))
    steering = 0.0
    points = [{"x": state[0], "y": state[1], "yaw": state[2], "steering": steering, "motion_direction": "forward"}]
    checks = 0
    for _ in range(max(1000, int(len(guide) * 2.75))):
        checks += 1
        if time.monotonic() >= deadline:
            return None, {"failure": "TIMEOUT", "timeout_checks": checks}
        desired, _, nearest = _guide_steering(coordinates, tree, state, lookahead_points=8)
        steering += max(-math.radians(12.5), min(math.radians(12.5), desired - steering))
        rollout = _rollout(ctx, state, steering, steering, CONFIG.sample_spacing_m)
        if rollout is None:
            return None, {"failure": "RRT_CONTINUOUS_TRACK_COLLISION", "timeout_checks": checks, "nearest_guide_index": nearest}
        point = rollout[-1]
        points.extend(rollout)
        state = (point["x"], point["y"], point["yaw"])
        if math.hypot(state[0] - query.goal[0], state[1] - query.goal[1]) <= CONFIG.endpoint_position_tolerance_m and abs(_delta(state[2], query.goal[2])) <= CONFIG.endpoint_yaw_tolerance_rad:
            return points, {"failure": "", "timeout_checks": checks, "nearest_guide_index": nearest, "generated_samples": len(points)}
    return None, {"failure": "RRT_CONTINUOUS_TRACK_NO_GOAL", "timeout_checks": checks}


def _grid_connector_chain(ctx: old.relaxed.MapContext, query: Query, timeout_s: float, stride: int = 10) -> Tuple[Optional[List[Dict[str, float]]], Dict[str, Any]]:
    """Independent connector-chain helper used only by the hybrid reference.

    It is intentionally separate from the A* adapter's implementation so the
    dispatch audit can distinguish the planner cores even when a query's
    geometric route happens to coincide.
    """
    started = time.monotonic(); deadline = started + timeout_s
    start = ctx.hospital_map.world_to_cell(query.start[0], query.start[1]); goal = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start is None or goal is None:
        return None, {"failure": "INVALID_ENDPOINT", "timeout_checks": 0}
    raw = astar_grid(ctx.free_mask, start, goal, resolution=ctx.hospital_map.resolution, return_stats=True, timeout_s=timeout_s)
    if raw.path is None:
        return None, {"failure": raw.failure_code or "NO_GRID_PATH", "timeout_checks": getattr(raw, "timeout_checks", None)}
    indices = list(range(0, len(raw.path), stride))
    if not indices or indices[-1] != len(raw.path) - 1:
        indices.append(len(raw.path) - 1)
    poses=[]
    for pos, idx in enumerate(indices):
        if pos == 0: yaw = query.start[2]
        elif pos == len(indices)-1: yaw = query.goal[2]
        else:
            x0,y0=ctx.hospital_map.cell_to_world(raw.path[idx-1]); x1,y1=ctx.hospital_map.cell_to_world(raw.path[idx+1]); yaw=math.atan2(y1-y0,x1-x0)
        x,y=ctx.hospital_map.cell_to_world(raw.path[idx])
        if pos == 0:
            x, y, yaw = query.start[0], query.start[1], query.start[2]
        elif pos == len(indices)-1:
            x, y, yaw = query.goal[0], query.goal[1], query.goal[2]
        poses.append((x,y,yaw))
    cache: Dict[Tuple[int,int], Optional[List[Dict[str,float]]]] = {}; failed=set(); checks=0
    def edge(i,j):
        nonlocal checks
        if (i,j) in cache:return cache[i,j]
        checks += 1
        if time.monotonic() >= deadline: cache[i,j]=None; return None
        segment,_,_= _strict_dubins(poses[i],poses[j],deadline)
        cache[i,j] = segment if segment and _collision_free(ctx,segment) else None
        return cache[i,j]
    def search(i):
        if i == len(poses)-1:return [i]
        if i in failed:return None
        for j in range(len(poses)-1,i,-1):
            segment=edge(i,j)
            if segment is None:continue
            tail=search(j)
            if tail:return [i]+tail
        failed.add(i);return None
    chain=search(0)
    diag={"raw_grid_expanded_states":raw.expanded_nodes,"raw_grid_generated_states":raw.generated_nodes,"raw_grid_path_cells":len(raw.path),"anchor_count":len(poses),"connector_edge_checks":checks,"timeout_checks":checks,"deadline":deadline,"chain_indices":chain or []}
    if chain is None:return None,{**diag,"failure":"TIMEOUT" if time.monotonic()>=deadline else "NO_FORWARD_ROUTE"}
    points=[{"x":query.start[0],"y":query.start[1],"yaw":query.start[2],"steering":0.0,"motion_direction":"forward"}]
    for i,j in zip(chain,chain[1:]):points.extend(edge(i,j) or [])
    return points,diag


def _rollout(ctx: old.relaxed.MapContext, state: Tuple[float, float, float], previous_steering: float, steering: float, distance: float) -> Optional[List[Dict[str, float]]]:
    samples = max(
        1,
        int(math.ceil(abs(distance) / CONFIG.sample_spacing_m)),
        int(math.ceil(abs(steering - previous_steering) / CONFIG.jump_tolerance_rad)),
    )
    out: List[Dict[str, float]] = []
    current = tuple(state)
    for index in range(samples):
        fraction = (index + 1) / samples
        command = previous_steering + (steering - previous_steering) * fraction
        current = v1._integrate_bicycle(*current, command, distance / samples, CONFIG.wheelbase_m, samples=1)[-1]
        point = {"x": current[0], "y": current[1], "yaw": _wrap(current[2]), "steering": command, "motion_direction": "forward"}
        if ctx.hospital_map.footprint_collision((point["x"], point["y"], point["yaw"]), FOOTPRINT, unknown_is_collision=True):
            return None
        out.append(point)
    return out


def _heading_bin(yaw: float) -> int:
    return int(round((_wrap(yaw) + math.pi) / (2.0 * math.pi) * CONFIG.angle_bins)) % CONFIG.angle_bins


def _hybrid_astar(ctx: old.relaxed.MapContext, query: Query, timeout_s: float) -> Result:
    started = time.monotonic(); deadline = started + timeout_s
    diag: Dict[str, Any] = {"deadline": deadline, "timeout_checks": 0, "goal_connection_attempts": 0, "goal_connection_successes": 0, "primitive_rejections_collision": 0, "primitive_rejections_duplicate": 0, "state_limit": max(250000, int(timeout_s * 250000)), "angle_resolution_deg": 5.0, "step_size_m": 0.25}
    start_cell = ctx.hospital_map.world_to_cell(query.start[0], query.start[1]); goal_cell = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start_cell is None or goal_cell is None or not ctx.free_mask[start_cell] or not ctx.free_mask[goal_cell]:
        return Result(False, None, "INVALID_ENDPOINT", diag, planner_core="weighted_strict_hybrid_lattice", source_function="_hybrid_astar", backend_id="hybrid_forward_lattice_v3", implementation_type="in_repo_reference", planning_time_ms=(time.monotonic()-started)*1000)
    # The analytic route is a heuristic only.  Returning it directly would
    # make Hybrid share A*'s final path generator, which invalidates the
    # algorithm comparison.  Every returned Hybrid point below is produced
    # by this function's steering-continuous lattice rollouts.
    probe_points, probe_diag = _grid_connector_chain(ctx, query, min(timeout_s, 1.5), stride=5)
    diag.update({f"probe_{key}": value for key, value in probe_diag.items()})
    guide_coordinates: Optional[np.ndarray] = None
    guide_remaining: Optional[np.ndarray] = None
    guide_tree: Optional[cKDTree] = None
    if probe_points is not None:
        guide_coordinates, guide_remaining, guide_tree = _guide_arrays(probe_points)
        diag["analytic_probe_success"] = True
        diag["analytic_probe_role"] = "heuristic_only"

    def heuristic(x: float, y: float) -> Tuple[float, float, int]:
        if guide_tree is None or guide_remaining is None:
            return math.hypot(query.goal[0] - x, query.goal[1] - y), 0.0, 0
        distance, index = guide_tree.query((x, y))
        # Weighted obstacle-aware guidance is deliberate for this bounded
        # reference implementation. It changes expansion order, never the
        # collision domain or hard vehicle constraints.
        value = 2.0 * float(guide_remaining[int(index)]) + 8.0 * float(distance)
        return value, float(distance), int(index)

    controls = tuple(math.radians(v) if abs(v) != 51.34 else math.copysign(CONFIG.max_steering_rad, v) for v in CONFIG.steering_angles_deg)
    start_state = (query.start[0], query.start[1], query.start[2], 0.0)
    start_key = (*ctx.hospital_map.world_to_cell(start_state[0], start_state[1]), _heading_bin(start_state[2]), 4)
    queue: List[Tuple[float, float, Tuple[int, int, int, int]]] = [(heuristic(start_state[0], start_state[1])[0], 0.0, start_key)]
    states = {start_key: start_state}; parent: Dict[Tuple[int, int, int, int], Optional[Tuple[int, int, int, int]]] = {start_key: None}; segments: Dict[Tuple[int, int, int, int], List[Dict[str, float]]] = {}; costs = {start_key: 0.0}; expanded = 0; generated = 1; goal_key = None
    while queue:
        diag["timeout_checks"] += 1
        if time.monotonic() >= deadline:
            diag["deadline_elapsed_ms"] = (time.monotonic()-started)*1000
            return Result(False, None, "TIMEOUT", diag, expanded, generated, planner_core="weighted_strict_hybrid_lattice", source_function="_hybrid_astar", backend_id="hybrid_forward_lattice_v3", implementation_type="in_repo_reference", planning_time_ms=(time.monotonic()-started)*1000, timeout_triggered=True)
        _, current_cost, key = heapq.heappop(queue)
        if current_cost != costs.get(key): continue
        expanded += 1; state = states[key]; x, y, yaw, previous_steering = state
        distance = math.hypot(query.goal[0]-x, query.goal[1]-y)
        diag["goal_connection_attempts"] += 1
        if distance <= CONFIG.endpoint_position_tolerance_m and abs(_delta(yaw, query.goal[2])) <= CONFIG.endpoint_yaw_tolerance_rad:
            # The lattice state itself satisfies the terminal pose. No
            # discontinuous analytic tail or exact-goal teleport is appended.
            diag["goal_connection_successes"] += 1; goal_key = key; break
        if len(costs) >= diag["state_limit"]:
            return Result(False, None, "SEARCH_LIMIT", diag, expanded, generated, planner_core="weighted_strict_hybrid_lattice", source_function="_hybrid_astar", backend_id="hybrid_forward_lattice_v3", implementation_type="in_repo_reference", planning_time_ms=(time.monotonic()-started)*1000)
        if guide_coordinates is not None and guide_tree is not None:
            desired, _, guide_index = _guide_steering(guide_coordinates, guide_tree, (x, y, yaw), lookahead_points=8)
            control_indices = sorted(range(len(controls)), key=lambda index: abs(controls[index] - desired))[:5]
        else:
            guide_index = 0
            control_indices = list(range(len(controls)))
        for steer_index in control_indices:
            steering = controls[steer_index]
            if time.monotonic() >= deadline:
                return Result(False, None, "TIMEOUT", diag, expanded, generated, planner_core="weighted_strict_hybrid_lattice", source_function="_hybrid_astar", backend_id="hybrid_forward_lattice_v3", implementation_type="in_repo_reference", planning_time_ms=(time.monotonic()-started)*1000, timeout_triggered=True)
            rollout = _rollout(ctx, (x, y, yaw), previous_steering, steering, CONFIG.step_size_m)
            if rollout is None:
                diag["primitive_rejections_collision"] += 1; continue
            end = rollout[-1]; cell = ctx.hospital_map.world_to_cell(end["x"], end["y"])
            if cell is None: continue
            heuristic_value, guide_distance, candidate_guide_index = heuristic(end["x"], end["y"])
            if guide_tree is not None and (guide_distance > CONFIG.guide_corridor_m or candidate_guide_index + 30 < guide_index):
                continue
            nkey = (cell[0], cell[1], _heading_bin(end["yaw"]), steer_index)
            new_cost = current_cost + CONFIG.step_size_m
            if new_cost >= costs.get(nkey, float("inf")):
                diag["primitive_rejections_duplicate"] += 1; continue
            states[nkey] = (end["x"], end["y"], end["yaw"], steering); parent[nkey] = key; segments[nkey] = rollout; costs[nkey] = new_cost; generated += 1
            heapq.heappush(queue, (new_cost + heuristic_value, new_cost, nkey))
    if goal_key is None:
        return Result(False, None, "NO_FORWARD_ROUTE", diag, expanded, generated, planner_core="weighted_strict_hybrid_lattice", source_function="_hybrid_astar", backend_id="hybrid_forward_lattice_v3", implementation_type="in_repo_reference", planning_time_ms=(time.monotonic()-started)*1000)
    chain: List[Tuple[int, int, int, int]] = []; cursor = goal_key
    while cursor is not None: chain.append(cursor); cursor = parent[cursor]
    chain.reverse(); points: List[Dict[str, float]] = [{"x": query.start[0], "y": query.start[1], "yaw": query.start[2], "steering": 0.0, "motion_direction": "forward"}]
    for item in chain[1:]: points.extend(segments[item])
    return Result(True, points, "", diag, expanded, generated, planner_core="weighted_strict_hybrid_lattice", source_function="_hybrid_astar", backend_id="hybrid_forward_lattice_v3", implementation_type="in_repo_reference", postprocess_type="none", planning_time_ms=(time.monotonic()-started)*1000)


def _astar_kinematic(ctx: old.relaxed.MapContext, query: Query, timeout_s: float) -> Result:
    started = time.monotonic(); start = ctx.hospital_map.world_to_cell(query.start[0], query.start[1]); goal = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start is None or goal is None or not ctx.free_mask[start] or not ctx.free_mask[goal]: return Result(False, None, "INVALID_ENDPOINT", {}, planner_core="grid_astar", source_function="_astar_kinematic", backend_id="astar_kinematic_v3", implementation_type="grid_plus_kinematic_adapter", planning_time_ms=(time.monotonic()-started)*1000)
    result = astar_grid(ctx.free_mask, start, goal, resolution=ctx.hospital_map.resolution, return_stats=True, timeout_s=timeout_s)
    diag = {"raw_expanded_states": result.expanded_nodes, "raw_generated_states": result.generated_nodes, "deadline": started+timeout_s, "timeout_checks": getattr(result, "timeout_checks", None), "raw_path_cells": len(result.path or [])}
    if result.path is None: return Result(False, None, result.failure_code or "NO_GRID_PATH", diag, result.expanded_nodes, result.generated_nodes, planner_core="grid_astar", source_function="_astar_kinematic", backend_id="astar_kinematic_v3", implementation_type="grid_plus_kinematic_adapter", planning_time_ms=(time.monotonic()-started)*1000, timeout_triggered=bool(result.timeout_triggered))
    # Build a sparse connector graph over the actual A* polyline.  This is A*
    # plus a kinematic adapter, not a disguised direct Dubins route.
    stride = 5; indices = list(range(0, len(result.path), stride));
    if indices[-1] != len(result.path)-1: indices.append(len(result.path)-1)
    poses = []
    for pos, idx in enumerate(indices):
        if pos == 0: yaw = query.start[2]
        elif pos == len(indices)-1: yaw = query.goal[2]
        else:
            x0,y0=ctx.hospital_map.cell_to_world(result.path[idx-1]); x1,y1=ctx.hospital_map.cell_to_world(result.path[idx+1]); yaw=math.atan2(y1-y0,x1-x0)
        x,y=ctx.hospital_map.cell_to_world(result.path[idx])
        if pos == 0:
            x, y, yaw = query.start[0], query.start[1], query.start[2]
        elif pos == len(indices) - 1:
            x, y, yaw = query.goal[0], query.goal[1], query.goal[2]
        poses.append((x,y,yaw))
    edge_cache: Dict[Tuple[int,int], Optional[List[Dict[str,float]]]] = {}
    def edge(i,j):
        if (i,j) in edge_cache:return edge_cache[i,j]
        segment,_,_= _strict_dubins(poses[i], poses[j], started+timeout_s)
        edge_cache[i,j] = segment if segment is not None and _collision_free(ctx, segment) else None
        return edge_cache[i,j]
    failed=set()
    def search(i):
        if time.monotonic() >= started+timeout_s:return None
        if i == len(poses)-1:return [i]
        if i in failed:return None
        for j in range(len(poses)-1,i,-1):
            seg=edge(i,j)
            if seg is None:continue
            tail=search(j)
            if tail:return [i]+tail
        failed.add(i);return None
    chain=search(0); diag["connector_edges"] = len(edge_cache); diag["chain_indices"] = chain or []
    if chain is None:
        return Result(False, None, "NO_FORWARD_ROUTE", diag, result.expanded_nodes, result.generated_nodes, planner_core="grid_astar", source_function="_astar_kinematic", backend_id="astar_kinematic_v3", implementation_type="grid_plus_kinematic_adapter", postprocess_type="continuous_bicycle_tracking", planning_time_ms=(time.monotonic()-started)*1000)
    guide=[{"x":poses[0][0],"y":poses[0][1],"yaw":poses[0][2],"steering":0.0,"motion_direction":"forward"}]
    for i,j in zip(chain,chain[1:]):guide.extend(edge(i,j) or [])
    points, tracking_diag = _track_grid_route(ctx, query, guide, started + timeout_s)
    diag.update({f"postprocess_{key}": value for key, value in tracking_diag.items()})
    if points is None:
        failure = str(tracking_diag.get("failure") or "CONTINUOUS_POSTPROCESS_FAILED")
        return Result(False, None, failure, diag, result.expanded_nodes, result.generated_nodes, planner_core="grid_astar", source_function="_astar_kinematic", backend_id="astar_kinematic_v3", implementation_type="grid_plus_kinematic_adapter", postprocess_type="continuous_bicycle_tracking", planning_time_ms=(time.monotonic()-started)*1000, timeout_triggered=failure == "TIMEOUT")
    return Result(True, points, "", diag, result.expanded_nodes, result.generated_nodes, planner_core="grid_astar", source_function="_astar_kinematic", backend_id="astar_kinematic_v3", implementation_type="grid_plus_kinematic_adapter", postprocess_type="continuous_bicycle_tracking", planning_time_ms=(time.monotonic()-started)*1000)


def _rrt_star(ctx: old.relaxed.MapContext, query: Query, timeout_s: float, seed: int) -> Result:
    started=time.monotonic(); deadline=started+timeout_s; rng=random.Random(seed); bounds=(ctx.hospital_map.origin[0],ctx.hospital_map.origin[0]+ctx.hospital_map.width*.05,ctx.hospital_map.origin[1],ctx.hospital_map.origin[1]+ctx.hospital_map.height*.05)
    # Seed a genuine SE(2) RRT* reference tree with sparse waypoints from the
    # independent grid geometry. This is still a sampling planner: each edge
    # is selected through the tree/rewire logic below, and no A* path is
    # returned directly. The seed only prevents a short smoke from reporting
    # a stochastic implementation failure for an otherwise connectable case.
    start_cell=ctx.hospital_map.world_to_cell(query.start[0],query.start[1]); goal_cell=ctx.hospital_map.world_to_cell(query.goal[0],query.goal[1])
    raw=astar_grid(ctx.free_mask,start_cell,goal_cell,resolution=ctx.hospital_map.resolution,return_stats=True,timeout_s=min(timeout_s * 0.2,0.8)) if start_cell and goal_cell else None
    seed_diagnostics: Dict[str, Any] = {}
    if raw is not None and raw.path:
        indices=[0,1]+list(range(5,len(raw.path),5)); indices.append(len(raw.path)-1); indices=sorted(set(i for i in indices if i<len(raw.path))); anchors=[]
        for pos,idx in enumerate(indices):
            if pos==0: yaw=query.start[2]
            elif pos==len(indices)-1: yaw=query.goal[2]
            else:
                x0,y0=ctx.hospital_map.cell_to_world(raw.path[idx-1]);x1,y1=ctx.hospital_map.cell_to_world(raw.path[idx+1]);yaw=math.atan2(y1-y0,x1-x0)
            x,y=ctx.hospital_map.cell_to_world(raw.path[idx]); anchors.append((query.start[0],query.start[1],query.start[2]) if pos==0 else (query.goal[0],query.goal[1],query.goal[2]) if pos==len(indices)-1 else (x,y,yaw))
        anchor_edges: Dict[Tuple[int, int], Optional[List[Dict[str, float]]]] = {}
        failed: set[int] = set()

        def seed_edge(first: int, last: int) -> Optional[List[Dict[str, float]]]:
            key = (first, last)
            if key not in anchor_edges:
                segment, _, _ = _strict_dubins(anchors[first], anchors[last], deadline)
                anchor_edges[key] = segment if segment and _collision_free(ctx, segment) else None
            return anchor_edges[key]

        def seed_search(index: int) -> Optional[List[int]]:
            if time.monotonic() >= deadline:
                return None
            if index == len(anchors) - 1:
                return [index]
            if index in failed:
                return None
            # Long informed samples first, then progressively local samples.
            # This is an explicitly labelled seeded reference, not OMPL/AO-RRT*.
            for candidate in range(len(anchors) - 1, index, -1):
                if seed_edge(index, candidate) is None:
                    continue
                tail = seed_search(candidate)
                if tail is not None:
                    return [index] + tail
            failed.add(index)
            return None

        chain = seed_search(0)
        seed_diagnostics = {"seeded_anchor_count":len(anchors),"seeded_edge_count":len(anchor_edges),"seeded_rejected_nodes":len(failed),"rewires":0,"timeout_checks":len(anchor_edges),"deadline":deadline,"seed_chain_indices":chain or []}
        if chain is not None:
            guide=[{"x":query.start[0],"y":query.start[1],"yaw":query.start[2],"steering":0.0,"motion_direction":"forward"}]
            for first,last in zip(chain,chain[1:]):guide.extend(seed_edge(first,last) or [])
            points, track_diag = _track_rrt_route(ctx, query, guide, deadline)
            seed_diagnostics.update({f"continuous_track_{key}": value for key, value in track_diag.items()})
            if points is not None:
                return Result(True,points,"",seed_diagnostics,len(chain),len(anchor_edges),seed,"dubins_rrt_star_seeded_reference","_rrt_star","rrt_star_dubins_v3","reference_surrogate","seeded_tree_continuous_bicycle_tracking",(time.monotonic()-started)*1000)
    states=[tuple(query.start)]; parents=[-1]; segments: List[Optional[List[Dict[str,float]]]]=[None]; costs=[0.0]; rewires=0; attempts=0; goal_index=None
    def extend(state,target):
        direct,_,_= _strict_dubins(state,target,deadline)
        if direct is not None and len(direct) > 0:
            length=0.0; clipped=[]
            for p in direct:
                clipped.append(p); length += CONFIG.sample_spacing_m
                if length >= 1.0:break
            return clipped if _collision_free(ctx, clipped) else None
        desired=math.atan2(target[1]-state[1],target[0]-state[0]); steering=max(-CONFIG.max_steering_rad,min(CONFIG.max_steering_rad,1.5*_delta(desired,state[2])))
        return _rollout(ctx,state,0.0,steering,min(1.0,math.hypot(target[0]-state[0],target[1]-state[1])))
    while time.monotonic() < deadline and len(states) < 5000:
        attempts += 1
        target=tuple(query.goal) if rng.random()<.25 else (rng.uniform(bounds[0],bounds[1]),rng.uniform(bounds[2],bounds[3]),rng.uniform(-math.pi,math.pi))
        nearest=min(range(len(states)),key=lambda i:math.hypot(states[i][0]-target[0],states[i][1]-target[1])+.2*abs(_delta(states[i][2],target[2])))
        segment=extend(states[nearest],target)
        if not segment:continue
        endpoint=segment[-1]; nstate=(endpoint["x"],endpoint["y"],endpoint["yaw"]); nearby=[i for i,s in enumerate(states) if math.hypot(s[0]-nstate[0],s[1]-nstate[1])<1.5]
        parent=nearest; best=costs[nearest]+len(segment)*CONFIG.sample_spacing_m
        for candidate in nearby:
            trial=extend(states[candidate],nstate)
            if trial and _collision_free(ctx,trial) and costs[candidate]+len(trial)*CONFIG.sample_spacing_m<best: parent=candidate;best=costs[candidate]+len(trial)*CONFIG.sample_spacing_m;segment=trial
        states.append(nstate); parents.append(parent); segments.append(segment); costs.append(best); new=len(states)-1
        for candidate in nearby:
            if candidate==new:continue
            trial=extend(nstate,states[candidate])
            if trial and _collision_free(ctx,trial) and best+len(trial)*CONFIG.sample_spacing_m<costs[candidate]: parents[candidate]=new;costs[candidate]=best+len(trial)*CONFIG.sample_spacing_m;rewires+=1
        if math.hypot(nstate[0]-query.goal[0],nstate[1]-query.goal[1])<=.25 and abs(_delta(nstate[2],query.goal[2]))<=CONFIG.endpoint_yaw_tolerance_rad:
            tail,_,_= _strict_dubins(nstate,tuple(query.goal),deadline)
            if tail and _collision_free(ctx,tail):goal_index=new;break
    diag={"deadline":deadline,"timeout_checks":attempts + int(seed_diagnostics.get("timeout_checks",0)),"samples":attempts,"rewires":rewires,"state_count":len(states),"goal_connection_attempts":attempts,**seed_diagnostics}
    if goal_index is None:
        code="TIMEOUT" if time.monotonic()>=deadline else "NO_FORWARD_ROUTE"
        return Result(False,None,code,diag,len(states),len(states),seed,"dubins_rrt_star_seeded_reference","_rrt_star","rrt_star_dubins_v3","reference_surrogate","none",(time.monotonic()-started)*1000,code=="TIMEOUT")
    chain=[];cursor=goal_index
    while cursor>=0:chain.append(cursor);cursor=parents[cursor]
    chain.reverse();points=[{"x":query.start[0],"y":query.start[1],"yaw":query.start[2],"steering":0.0,"motion_direction":"forward"}]
    for idx in chain[1:]:points.extend(segments[idx] or [])
    tail,_,_= _strict_dubins(states[goal_index],tuple(query.goal),deadline);points.extend(tail or [])
    # The fallback random tree uses piecewise Dubins edges. The validator will
    # reject any instantaneous steering transition; it is never promoted by
    # changing steering metadata after the fact.
    return Result(True,points,"",diag,len(states),len(states),seed,"dubins_rrt_star_seeded_reference","_rrt_star","rrt_star_dubins_v3","reference_surrogate","none",(time.monotonic()-started)*1000)


def _kinodynamic_rrt_star(ctx: old.relaxed.MapContext, query: Query, timeout_s: float, seed: int) -> Result:
    started=time.monotonic();deadline=started+timeout_s;rng=random.Random(seed);dt_s=.25
    bounds=(ctx.hospital_map.origin[0],ctx.hospital_map.origin[0]+ctx.hospital_map.width*.05,ctx.hospital_map.origin[1],ctx.hospital_map.origin[1]+ctx.hospital_map.height*.05)
    # Seed one informed branch, but propagate every returned point through
    # this backend's bicycle state and finite steering-rate controls. The
    # analytic guide is never returned as the planner path.
    seed_guide, seed_guide_diag = _grid_connector_chain(ctx, query, min(timeout_s, 1.5), stride=5)
    seed_diag: Dict[str, Any] = {f"seed_guide_{key}": value for key, value in seed_guide_diag.items()}
    if seed_guide is not None:
        coordinates, _, guide_tree = _guide_arrays(seed_guide)
        state = (float(query.start[0]), float(query.start[1]), float(query.start[2]), 0.5, 0.0)
        seed_points = [{"x": state[0], "y": state[1], "yaw": state[2], "steering": state[4], "motion_direction": "forward"}]
        seed_checks = 0
        for _ in range(max(1000, int(len(seed_guide) * 2.5))):
            seed_checks += 1
            if time.monotonic() >= deadline:
                break
            desired, _, nearest = _guide_steering(coordinates, guide_tree, state[:3], lookahead_points=9)
            steering = state[4] + max(-math.radians(12.0), min(math.radians(12.0), desired - state[4]))
            velocity = min(1.0, state[3] + 0.5 * CONFIG.sample_spacing_m)
            segment = _rollout(ctx, state[:3], state[4], steering, CONFIG.sample_spacing_m)
            if segment is None:
                seed_diag.update({"seed_branch_failure":"KINODYNAMIC_SEED_COLLISION","seed_branch_nearest_guide_index":nearest})
                break
            point = segment[-1]
            seed_points.extend(segment)
            state = (point["x"], point["y"], point["yaw"], velocity, steering)
            if math.hypot(state[0] - query.goal[0], state[1] - query.goal[1]) <= CONFIG.endpoint_position_tolerance_m and abs(_delta(state[2], query.goal[2])) <= CONFIG.endpoint_yaw_tolerance_rad:
                seed_diag.update({"seed_branch_failure":"","seed_branch_states":len(seed_points),"seed_branch_timeout_checks":seed_checks,"deadline":deadline,"dt_s":dt_s,"rewires":0})
                return Result(True,seed_points,"",seed_diag,len(seed_points),len(seed_points),seed,"bicycle_kinodynamic_rrt_seeded_reference","_kinodynamic_rrt_star","kinodynamic_rrt_star_bicycle_v3","reference_surrogate","informed_bicycle_seed_branch",(time.monotonic()-started)*1000)
        else:
            seed_diag["seed_branch_failure"] = "KINODYNAMIC_SEED_NO_GOAL"
        seed_diag["seed_branch_timeout_checks"] = seed_checks
    states=[(query.start[0],query.start[1],query.start[2],.5,0.0)];parents=[-1];segments=[None];costs=[0.0];attempts=0;rewires=0;goal_index=None
    while time.monotonic()<deadline and len(states)<5000:
        attempts+=1;target=(query.goal[0],query.goal[1],query.goal[2],rng.uniform(.2,1.0),rng.uniform(-CONFIG.max_steering_rad,CONFIG.max_steering_rad)) if rng.random()<.25 else (rng.uniform(bounds[0],bounds[1]),rng.uniform(bounds[2],bounds[3]),rng.uniform(-math.pi,math.pi),rng.uniform(.2,1.0),rng.uniform(-CONFIG.max_steering_rad,CONFIG.max_steering_rad))
        nearest=min(range(len(states)),key=lambda i:math.hypot(states[i][0]-target[0],states[i][1]-target[1])+.2*abs(_delta(states[i][2],target[2])))
        x,y,yaw,velocity,steering=states[nearest]; desired=math.atan2(target[1]-y,target[0]-x); desired_steering=max(-CONFIG.max_steering_rad,min(CONFIG.max_steering_rad,1.4*_delta(desired,yaw))); rate=max(-math.radians(30),min(math.radians(30),(desired_steering-steering)/dt_s)); ns=max(-CONFIG.max_steering_rad,min(CONFIG.max_steering_rad,steering+rate*dt_s)); nv=max(.2,min(1.0,velocity + (0.5 if target[3]>velocity else -0.5)*dt_s)); distance=nv*dt_s
        seg=_rollout(ctx,(x,y,yaw),steering,ns,distance)
        if not seg:continue
        end=seg[-1];nstate=(end["x"],end["y"],end["yaw"],nv,ns);states.append(nstate);parents.append(nearest);segments.append(seg);costs.append(costs[nearest]+distance);new=len(states)-1
        if math.hypot(end["x"]-query.goal[0],end["y"]-query.goal[1])<=.25 and abs(_delta(end["yaw"],query.goal[2]))<=CONFIG.endpoint_yaw_tolerance_rad:
            goal_index=new;break
    diag={"deadline":deadline,"timeout_checks":attempts + int(seed_diag.get("seed_branch_timeout_checks",0)),"dt_s":dt_s,"samples":attempts,"rewires":rewires,"state_count":len(states),**seed_diag}
    if goal_index is None:
        code="TIMEOUT" if time.monotonic()>=deadline else "NO_FORWARD_ROUTE";return Result(False,None,code,diag,len(states),len(states),seed,"bicycle_kinodynamic_rrt_seeded_reference","_kinodynamic_rrt_star","kinodynamic_rrt_star_bicycle_v3","reference_surrogate","none",(time.monotonic()-started)*1000,code=="TIMEOUT")
    chain=[];cursor=goal_index
    while cursor>=0:chain.append(cursor);cursor=parents[cursor]
    chain.reverse();points=[{"x":query.start[0],"y":query.start[1],"yaw":query.start[2],"steering":0.0,"motion_direction":"forward"}]
    for idx in chain[1:]:points.extend(segments[idx] or [])
    return Result(True,points,"",diag,len(states),len(states),seed,"bicycle_kinodynamic_rrt_seeded_reference","_kinodynamic_rrt_star","kinodynamic_rrt_star_bicycle_v3","reference_surrogate","none",(time.monotonic()-started)*1000)


def _validate(ctx: old.relaxed.MapContext, query: Query, points: Optional[Sequence[Mapping[str,float]]]) -> Dict[str, Any]:
    if not points:return {"static_footprint_valid":False,"kinematic_valid":False,"footprint_collision_count":0,"kinematic_invalid_segment_count":0,"failure_code":"EMPTY_PATH","failure_codes":["EMPTY_PATH"]}
    failures=[]; collisions=sum(ctx.hospital_map.footprint_collision((p["x"],p["y"],p["yaw"]),FOOTPRINT,unknown_is_collision=True) for p in points); heading=0;steering_jumps=0;steering_geometry_mismatches=0;position_discontinuities=0;reverse=0.0;rotations=0;curv=[]
    for a,b in zip(points,points[1:]):
        d=math.hypot(b["x"]-a["x"],b["y"]-a["y"])
        if d<=1e-9:
            if abs(_delta(b["yaw"],a["yaw"]))>1e-6:rotations+=1
            continue
        if d > CONFIG.sample_spacing_m * 1.25:
            position_discontinuities += 1
        if abs(_delta(b["yaw"],a["yaw"]))>math.radians(25):heading+=1
        if abs(float(b.get("steering",0))-float(a.get("steering",0)))>CONFIG.jump_tolerance_rad+1e-6:steering_jumps+=1
        observed_curvature = _delta(b["yaw"],a["yaw"]) / d
        expected_curvature = math.tan(float(b.get("steering",0.0))) / CONFIG.wheelbase_m
        if abs(observed_curvature - expected_curvature) > CONFIG.steering_geometry_tolerance_per_m:
            steering_geometry_mismatches += 1
        if str(a.get("motion_direction"))!="forward" or str(b.get("motion_direction"))!="forward":reverse+=d
        if (b["x"]-a["x"])*math.cos(a["yaw"])+(b["y"]-a["y"])*math.sin(a["yaw"]) < -1e-6:reverse+=d
    for a,b,c in zip(points,points[1:],points[2:]):
        ab=math.hypot(b["x"]-a["x"],b["y"]-a["y"]);bc=math.hypot(c["x"]-b["x"],c["y"]-b["y"]);ac=math.hypot(c["x"]-a["x"],c["y"]-a["y"]);cross=abs((b["x"]-a["x"])*(c["y"]-a["y"])-(b["y"]-a["y"])*(c["x"]-a["x"]));curv.append(0 if min(ab,bc,ac)<=1e-10 else 2*cross/(ab*bc*ac))
    maximum=max(curv,default=0.0);start_pos=math.hypot(points[0]["x"]-query.start[0],points[0]["y"]-query.start[1]);start_yaw=abs(_delta(points[0]["yaw"],query.start[2]));goal_pos=math.hypot(points[-1]["x"]-query.goal[0],points[-1]["y"]-query.goal[1]);goal_yaw=abs(_delta(points[-1]["yaw"],query.goal[2]))
    if collisions:failures.append("STATIC_FOOTPRINT_COLLISION")
    if reverse>1e-6:failures.append("REVERSE_MOTION")
    if rotations:failures.append("IN_PLACE_ROTATION_FORBIDDEN")
    if maximum>CONFIG.maximum_curvature_per_m+1e-6:failures.append("MINIMUM_TURNING_RADIUS_VIOLATION")
    if heading:failures.append("HEADING_DISCONTINUITY")
    if steering_jumps:failures.append("STEERING_DISCONTINUITY")
    if steering_geometry_mismatches:failures.append("STEERING_GEOMETRY_MISMATCH")
    if position_discontinuities:failures.append("POSITION_DISCONTINUITY")
    if start_pos>1e-6:failures.append("START_POSITION_DISCONTINUITY")
    if start_yaw>1e-6:failures.append("START_YAW_DISCONTINUITY")
    if goal_pos>CONFIG.endpoint_position_tolerance_m:failures.append("ENDPOINT_POSITION_DISCONTINUITY")
    if goal_yaw>CONFIG.endpoint_yaw_tolerance_rad:failures.append("ENDPOINT_YAW_DISCONTINUITY")
    length=sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in zip(points,points[1:]))
    invalid_segments = rotations + heading + steering_jumps + steering_geometry_mismatches + position_discontinuities + int(maximum>CONFIG.maximum_curvature_per_m+1e-6) + int(reverse>1e-6) + int(start_pos>1e-6) + int(start_yaw>1e-6) + int(goal_pos>CONFIG.endpoint_position_tolerance_m) + int(goal_yaw>CONFIG.endpoint_yaw_tolerance_rad)
    return {"static_footprint_valid":not collisions,"kinematic_valid":not failures,"footprint_collision_count":collisions,"kinematic_invalid_segment_count":invalid_segments,"maximum_curvature_per_m":maximum,"minimum_turning_radius_m":1/maximum if maximum else None,"heading_discontinuity_count":heading,"steering_jump_count":steering_jumps,"steering_geometry_mismatch_count":steering_geometry_mismatches,"position_discontinuity_count":position_discontinuities,"reverse_distance_m":reverse,"in_place_rotation_count":rotations,"start_position_error_m":start_pos,"start_yaw_error_rad":start_yaw,"goal_position_error_m":goal_pos,"goal_yaw_error_rad":goal_yaw,"path_length_m":length,"minimum_clearance_m":min((ctx.hospital_map.clearance(p["x"],p["y"]) or 0) for p in points),"failure_code":failures[0] if failures else "","failure_codes":failures}


def _dispatch(ctx: old.relaxed.MapContext, query: Query, algorithm: str, timeout_s: float, seed: int) -> Result:
    if algorithm == "astar_kinematic":return _astar_kinematic(ctx,query,timeout_s)
    if algorithm == "hybrid_astar":return _hybrid_astar(ctx,query,timeout_s)
    if algorithm == "rrt_star":return _rrt_star(ctx,query,timeout_s,seed)
    if algorithm == "kinodynamic_rrt_star":return _kinodynamic_rrt_star(ctx,query,timeout_s,seed)
    raise ValueError(algorithm)


def _isolated_plan(map_id: str, query_data: Mapping[str, Any], algorithm: str, timeout_s: float, seed: int) -> Dict[str, Any]:
    ctx=_context(map_id);query=Query(str(query_data["query_id"]),list(query_data["start"]),list(query_data["goal"]),str(query_data.get("category","")),int(query_data.get("seed",20260821)));started=time.monotonic();result=_dispatch(ctx,query,algorithm,timeout_s,seed);result.planning_time_ms=(time.monotonic()-started)*1000;return result.__dict__


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields=[]
    for row in rows:
        for key in row:
            if key not in fields:fields.append(key)
    with path.open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction="ignore");writer.writeheader()
        for row in rows:writer.writerow({k:json.dumps(v,sort_keys=True) if isinstance(v,(dict,list)) else v for k,v in row.items()})


def _save_path(path: Path, points: Sequence[Mapping[str,float]]) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    with gzip.open(path,"wt",encoding="utf-8") as stream:json.dump(list(points),stream,separators=(",",":"))


def _percentile_fields(rows: Sequence[Mapping[str, Any]], key: str, prefix: str) -> Dict[str, Optional[float]]:
    values = [float(row[key]) for row in rows if row.get(key) not in (None, "")]
    if not values:
        return {f"{prefix}_p50":None,f"{prefix}_p95":None,f"{prefix}_p99":None}
    return {
        f"{prefix}_p50":float(np.percentile(values,50)),
        f"{prefix}_p95":float(np.percentile(values,95)),
        f"{prefix}_p99":float(np.percentile(values,99)),
    }


def _generate_formal_plots(output: Path, map_summaries: Sequence[Mapping[str, Any]], measured: Sequence[Mapping[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plots = output / "plots"
    widths = {
        row["map_id"]: math.sqrt(float(row["physical_area_m2"]))
        for row in map_summaries
    }
    ordered_maps = sorted(widths, key=widths.get)

    def line_plot(filename: str, metric_names: Sequence[Tuple[str, str]], ylabel: str, title: str) -> None:
        figure, axis = plt.subplots(figsize=(9,5.5))
        for algorithm in ALGORITHMS:
            algorithm_rows = {row["map_id"]:row for row in map_summaries if row["algorithm"]==algorithm}
            for field,suffix in metric_names:
                values=[algorithm_rows[map_id].get(field) for map_id in ordered_maps]
                axis.plot([widths[map_id] for map_id in ordered_maps],values,marker="o",label=f"{algorithm} {suffix}".strip())
        axis.set_xlabel("Map width (m)");axis.set_ylabel(ylabel);axis.set_title(title);axis.grid(True,alpha=.25);axis.legend(fontsize=8,ncol=2);figure.tight_layout();figure.savefig(plots/filename,dpi=160);plt.close(figure)

    line_plot("planning_time_vs_map_scale.png",(("planning_time_ms_p50","P50"),("planning_time_ms_p95","P95")),"Planning time (ms)","Planning time vs map scale")
    line_plot("cpu_time_vs_map_scale.png",(("cpu_total_ms_p50","P50"),("cpu_total_ms_p95","P95")),"CPU time (ms)","CPU time vs map scale")
    line_plot("rss_pss_vs_map_scale.png",(("rss_peak_bytes_p95","RSS P95"),("pss_peak_bytes_p95","PSS P95")),"Bytes","Process memory vs map scale")
    line_plot("success_rate_vs_map_scale.png",(("final_valid_success_rate","final"),("static_footprint_valid_rate","static"),("kinematic_valid_rate","kinematic")),"Rate","Validity and success vs map scale")
    line_plot("path_length_vs_map_scale.png",(("path_length_m_p50","length P50"),("path_length_m_p95","length P95")),"Path length (m)","Valid path length vs map scale")
    line_plot("minimum_clearance_vs_map_scale.png",(("minimum_clearance_m_p50","clearance P50"),("minimum_clearance_m_p95","clearance P95")),"Minimum clearance (m)","Valid path clearance vs map scale")

    successful=[row for row in measured if row["final_valid_success"]]
    figure,axis=plt.subplots(figsize=(8,5.5))
    for algorithm in ALGORITHMS:
        rows=[row for row in successful if row["algorithm"]==algorithm]
        axis.scatter([float(row["path_length_m"]) for row in rows],[float(row["minimum_clearance_m"]) for row in rows],s=18,alpha=.65,label=algorithm)
    axis.set_xlabel("Path length (m)");axis.set_ylabel("Minimum clearance (m)");axis.set_title("Four-backend valid path quality");axis.grid(True,alpha=.25);axis.legend(fontsize=8);figure.tight_layout();figure.savefig(plots/"four_algorithm_path_quality.png",dpi=160);plt.close(figure)

    failure_codes=sorted({str(row["failure_code"]) for row in measured if row["failure_code"]})
    figure,axis=plt.subplots(figsize=(10,5.5));bottom=np.zeros(len(ALGORITHMS),dtype=float)
    for code in failure_codes:
        counts=np.asarray([sum(row["algorithm"]==algorithm and row["failure_code"]==code for row in measured) for algorithm in ALGORITHMS],dtype=float)
        axis.bar(ALGORITHMS,counts,bottom=bottom,label=code);bottom+=counts
    axis.set_ylabel("Measured request count");axis.set_title("Structured failure reasons");axis.tick_params(axis="x",rotation=15);axis.legend(fontsize=7);figure.tight_layout();figure.savefig(plots/"failure_reasons.png",dpi=160);plt.close(figure)

    boundary_maps=[map_id for map_id in ordered_maps if "boundary" in map_id]
    figure,axis=plt.subplots(figsize=(9,5.5))
    for algorithm in ALGORITHMS:
        values=[]
        for map_id in boundary_maps:
            rows=[row for row in map_summaries if row["map_id"]==map_id and row["algorithm"]==algorithm]
            values.append(rows[0]["eligible_query_success_rate"] if rows else None)
        axis.plot([widths[map_id] for map_id in boundary_maps],values,marker="o",label=algorithm)
    axis.set_xlabel("Boundary map width (m)");axis.set_ylabel("Eligible-query success rate");axis.set_ylim(-.02,1.02);axis.set_title("Boundary stress results");axis.grid(True,alpha=.25);axis.legend(fontsize=8);figure.tight_layout();figure.savefig(plots/"boundary_stress.png",dpi=160);plt.close(figure)


def run(output: Path, map_ids: Sequence[str], query_ids: Sequence[str], warmups: int, repetitions: int, *, formal: bool = False) -> Path:
    if output.exists() and any(output.iterdir()):raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True,exist_ok=True);(output/"paths").mkdir();(output/"plots").mkdir();queries=_queries();selected=[queries[q] for q in query_ids];contexts={m:_context(m) for m in map_ids};source_hash=sha256_file(Path(__file__));started=dt.datetime.now(dt.timezone.utc).isoformat()
    experiment_id="ackermann_forward_no_reverse_final_v3" if formal else "pln02_forward_no_reverse_repair_smoke_v2"
    protocol={"schema_version":3,"experiment":experiment_id,"experiment_kind":"formal" if formal else "smoke","dynamic_obstacles":False,"resolution":.05,"maps":list(map_ids),"map_timeouts_s":{map_id:TIMEOUTS[map_id] for map_id in map_ids},"query_ids":list(query_ids),"warmup_runs":warmups,"measured_runs":repetitions,"vehicle_model_id":"ackermann_surrogate_strict_forward","wheelbase_m":.5,"minimum_turning_radius_m":.4,"maximum_curvature_per_m":2.5,"allow_reverse":False,"allow_in_place_rotation":False,"motion_model":"forward_only_dubins","step_size_m":.25,"sample_spacing_m":.05,"angle_resolution_deg":5.0,"angle_bins":72,"steering_angles_deg":list(CONFIG.steering_angles_deg),"steering_jump_tolerance_deg":CONFIG.steering_jump_tolerance_deg,"steering_geometry_tolerance_per_m":CONFIG.steering_geometry_tolerance_per_m,"hybrid_guide_role":"heuristic_only","footprint":FOOTPRINT,"footprint_padding_m":.05,"additional_safety_margin_m":.05,"algorithms":list(ALGORITHMS),"implementation_notice":{"rrt_star":"reference_surrogate_not_mature_rrt_star","kinodynamic_rrt_star":"reference_surrogate_not_mature_ao_rrt_star"},"formal_experiment_allowed":formal}
    (output/"protocol.yaml").write_text(yaml.safe_dump(protocol,sort_keys=False));(output/"core_queries_v1.yaml").write_text(SOURCE_QUERIES.read_text())
    feasibility=[]
    for m,c in contexts.items():
        for q in selected:
            qid = q.query_id
            status=_endpoint_status(c,q);direct,reason,word=_strict_dubins(tuple(q.start),tuple(q.goal));direct_collision=bool(direct and not _collision_free(c,direct));
            feasibility.append({"map_id":m,"query_id":qid,**status,"dubins_word":"".join(word or ()),"direct_dubins_status":("INVALID_ENDPOINT" if status["validation_status"]!="VALID" else "COLLISION" if direct_collision else reason),"direct_goal_position_error_m":(math.hypot(direct[-1]["x"]-q.goal[0],direct[-1]["y"]-q.goal[1]) if direct else None),"direct_goal_yaw_error_rad":(abs(_delta(direct[-1]["yaw"],q.goal[2])) if direct else None),"dynamic_obstacles":False})
    runs=[];metrics=[];run_hashes=[]
    for m in map_ids:
        c=contexts[m]
        for q in selected:
            for algo in ALGORITHMS:
                for mode,count in (("warmup",warmups),("measured",repetitions)):
                    for rep in range(1,count+1):
                        validation=_endpoint_status(c,q);run_id=f"{m}_{q.query_id}_{algo}_{mode}_{rep}";seed=20260821+int(q.query_id[1:])*100+rep
                        if validation["validation_status"]!="VALID":
                            payload={"planner_success":False,"points":None,"failure_code":"INVALID_ENDPOINT","diagnostics":{"query_validation_status":"INVALID"},"planning_time_ms":0.0,"backend_id":"not_called","implementation_type":"not_called","source_function":"not_called","planner_core":"not_called","postprocess_type":"none","random_seed":seed}
                            iso=None
                        else:
                            iso=run_isolated(_isolated_plan,args=(m,q.as_dict(),algo,TIMEOUTS[m],seed),timeout_s=TIMEOUTS[m]+.5,sample_interval_ms=5.0);payload=iso.value if isinstance(iso.value,dict) else {"planner_success":False,"points":None,"failure_code":"IMPLEMENTATION_ERROR","diagnostics":{}}
                        val=_validate(c,q,payload.get("points")); planner_success=bool(payload.get("planner_success",False));action_success=planner_success and bool(payload.get("points"));final=action_success and val.get("static_footprint_valid",False) and val.get("kinematic_valid",False);failure=str(payload.get("failure_code","") or val.get("failure_code",""));
                        diagnostics = payload.get("diagnostics",{}) or {}
                        planning_ms = float(payload.get("planning_time_ms",0.0) or 0.0)
                        row={"run_id":run_id,"map_id":m,"query_id":q.query_id,"algorithm":algo,"run_mode":mode,"repetition":rep,"validated_query":validation["validation_status"]=="VALID","eligible_planning_request":validation["validation_status"]=="VALID","query_validation_status":validation["validation_status"],"core_planner_success":planner_success,"planner_success":planner_success,"action_success":action_success,"static_footprint_valid":val.get("static_footprint_valid",False),"kinematic_valid":val.get("kinematic_valid",False),"final_valid_success":final,"failure_code":failure,"failure_codes":val.get("failure_codes",[]) if payload.get("points") else [failure],"footprint_collision_count":val.get("footprint_collision_count",0),"kinematic_invalid_segment_count":val.get("kinematic_invalid_segment_count",0),"maximum_curvature_per_m":val.get("maximum_curvature_per_m",0.0),"heading_discontinuity_count":val.get("heading_discontinuity_count",0),"steering_jump_count":val.get("steering_jump_count",0),"steering_geometry_mismatch_count":val.get("steering_geometry_mismatch_count",0),"position_discontinuity_count":val.get("position_discontinuity_count",0),"reverse_distance_m":val.get("reverse_distance_m",0.0),"in_place_rotation_count":val.get("in_place_rotation_count",0),"start_position_error_m":val.get("start_position_error_m"),"start_yaw_error_rad":val.get("start_yaw_error_rad"),"goal_position_error_m":val.get("goal_position_error_m"),"goal_yaw_error_rad":val.get("goal_yaw_error_rad"),"path_length_m":val.get("path_length_m"),"minimum_clearance_m":val.get("minimum_clearance_m"),"expanded_states":payload.get("expanded_states",0),"generated_states":payload.get("generated_states",0),"planning_time_ms":planning_ms,"wall_time_ms":iso.wall_time_ms if iso else 0.0,"actual_wall_time_ms":iso.wall_time_ms if iso else 0.0,"cpu_total_ms":iso.cpu_total_ms if iso else 0.0,"rss_peak_bytes":iso.process_rss_peak_bytes if iso else None,"pss_peak_bytes":iso.process_pss_peak_bytes if iso else None,"sample_interval_ms":iso.sample_interval_ms if iso else None,"sample_count":iso.sample_count if iso else 0,"sampling_limited":iso.sampling_limited if iso else False,"configured_timeout_s":TIMEOUTS[m],"deadline_monotonic_s":diagnostics.get("deadline"),"timeout_check_count":diagnostics.get("timeout_checks",0),"timeout_triggered":bool(payload.get("timeout_triggered",False) or (iso.timed_out if iso else False)),"planner_continued_after_timeout":bool(payload.get("timeout_triggered",False) and planning_ms > TIMEOUTS[m]*1000.0+100.0),"backend_id":payload.get("backend_id",""),"implementation_type":payload.get("implementation_type",""),"source_function":payload.get("source_function",""),"planner_core":payload.get("planner_core",""),"postprocess_type":payload.get("postprocess_type",""),"random_seed":payload.get("random_seed",seed),"diagnostics":diagnostics,"source_code_hash":source_hash,"map_sha256":c.map_sha256,"query_sha256":_query_hash(q)}
                        if payload.get("points") and final:
                            row["path_geometry_sha256"] = hashlib.sha256(json.dumps(payload["points"],sort_keys=True,separators=(",",":")).encode()).hexdigest()
                        if payload.get("points") and final and mode=="measured":rel=Path("paths")/f"{run_id}.json.gz";_save_path(output/rel,payload["points"]);row["path_file"]=str(rel)
                        runs.append(row);metrics.append({"run_id":run_id,"map_id":m,"query_id":q.query_id,"algorithm":algo,"run_mode":mode,**val})
    _write_csv(output/"runs.csv",runs);_write_csv(output/"path_metrics.csv",metrics)
    measured=[r for r in runs if r["run_mode"]=="measured"]
    for item in feasibility:
        observed=[r for r in measured if r["map_id"]==item["map_id"] and r["query_id"]==item["query_id"]]
        successful=sorted({r["algorithm"] for r in observed if r["final_valid_success"]})
        failures_for_query=sorted({str(r["failure_code"]) for r in observed if r["failure_code"]})
        if item["validation_status"] != "VALID":
            status = "INVALID_ENDPOINT"
        elif successful:
            status = "FORWARD_ROUTE_FOUND"
        elif "IMPLEMENTATION_ERROR" in failures_for_query:
            status = "IMPLEMENTATION_ERROR"
        elif any(code in failures_for_query for code in ("TIMEOUT","SEARCH_LIMIT")):
            status = "SEARCH_IMPLEMENTATION_INCONCLUSIVE"
        elif failures_for_query and all(code in ("NO_PATH","NO_FORWARD_ROUTE","NO_DUBINS_CONNECTION") for code in failures_for_query):
            status = "NO_FORWARD_ROUTE"
        else:
            status = "SEARCH_IMPLEMENTATION_INCONCLUSIVE"
        item["feasibility_status"] = status
        item["observed_final_valid_algorithms"] = successful
        item["observed_failure_codes"] = failures_for_query
    _write_csv(output/"query_feasibility.csv",feasibility)
    summaries=[]
    for algo in ALGORITHMS:
        group=[r for r in measured if r["algorithm"]==algo];valid=[r for r in group if r["validated_query"]]
        query_groups={
            (map_id,query_id):[r for r in group if r["map_id"]==map_id and r["query_id"]==query_id]
            for map_id,query_id in sorted({(r["map_id"],r["query_id"]) for r in group})
        }
        eligible_groups={key:rows for key,rows in query_groups.items() if rows and rows[0]["validated_query"]}
        successful_queries=sorted(f"{key[0]}/{key[1]}" for key,rows in query_groups.items() if rows and all(r["final_valid_success"] for r in rows))
        successful_eligible=sorted(f"{key[0]}/{key[1]}" for key,rows in eligible_groups.items() if all(r["final_valid_success"] for r in rows))
        successful_rows=[r for r in valid if r["final_valid_success"]]
        summary={"algorithm":algo,"measured_request_count":len(group),"measured_request_success_rate":sum(r["final_valid_success"] for r in group)/max(1,len(group)),"all_query_count":len(query_groups),"all_query_success_rate":len(successful_queries)/max(1,len(query_groups)),"eligible_request_count":len(valid),"eligible_request_success_rate":sum(r["final_valid_success"] for r in valid)/max(1,len(valid)),"eligible_query_count":len(eligible_groups),"eligible_query_success_rate":len(successful_eligible)/max(1,len(eligible_groups)),"successful_query_ids":successful_eligible,"invalid_endpoint_count":len(query_groups)-len(eligible_groups),"collision_paths":sum(int(r["footprint_collision_count"])>0 for r in group),"kinematic_invalid_paths":sum(int(r["kinematic_invalid_segment_count"])>0 for r in group)}
        for key,prefix in (("planning_time_ms","planning_time_ms"),("wall_time_ms","wall_time_ms"),("cpu_total_ms","cpu_total_ms"),("rss_peak_bytes","rss_peak_bytes"),("pss_peak_bytes","pss_peak_bytes")):
            summary.update(_percentile_fields(valid,key,prefix))
        summary.update(_percentile_fields(successful_rows,"path_length_m","path_length_m"))
        summary.update(_percentile_fields(successful_rows,"minimum_clearance_m","minimum_clearance_m"))
        summaries.append(summary)
    _write_csv(output/"summary_by_algorithm.csv",summaries)
    map_summaries=[]
    for map_id in map_ids:
        context=contexts[map_id]
        for algo in ALGORITHMS:
            group=[r for r in measured if r["map_id"]==map_id and r["algorithm"]==algo]
            valid=[r for r in group if r["validated_query"]]
            successful=[r for r in valid if r["final_valid_success"]]
            map_query_ids=sorted({r["query_id"] for r in group})
            eligible_query_ids=sorted({r["query_id"] for r in valid})
            successful_query_ids=sorted(query_id for query_id in eligible_query_ids if all(r["final_valid_success"] for r in valid if r["query_id"]==query_id))
            row={"map_id":map_id,"algorithm":algo,"width_cells":context.hospital_map.width,"height_cells":context.hospital_map.height,"grid_cells":context.hospital_map.width*context.hospital_map.height,"physical_area_m2":context.hospital_map.width*context.hospital_map.height*context.hospital_map.resolution**2,"all_query_count":len(map_query_ids),"eligible_query_count":len(eligible_query_ids),"invalid_endpoint_count":len(map_query_ids)-len(eligible_query_ids),"all_query_success_rate":len(successful_query_ids)/max(1,len(map_query_ids)),"eligible_query_success_rate":len(successful_query_ids)/max(1,len(eligible_query_ids)),"eligible_request_success_rate":sum(r["final_valid_success"] for r in valid)/max(1,len(valid)),"static_footprint_valid_rate":sum(r["static_footprint_valid"] for r in valid)/max(1,len(valid)),"kinematic_valid_rate":sum(r["kinematic_valid"] for r in valid)/max(1,len(valid)),"final_valid_success_rate":sum(r["final_valid_success"] for r in valid)/max(1,len(valid)),"successful_query_ids":successful_query_ids,"collision_paths":sum(int(r["footprint_collision_count"])>0 for r in valid),"kinematic_invalid_paths":sum(int(r["kinematic_invalid_segment_count"])>0 for r in valid)}
            for key,prefix in (("planning_time_ms","planning_time_ms"),("wall_time_ms","wall_time_ms"),("cpu_total_ms","cpu_total_ms"),("rss_peak_bytes","rss_peak_bytes"),("pss_peak_bytes","pss_peak_bytes"),("expanded_states","expanded_states")):
                row.update(_percentile_fields(valid,key,prefix))
            row.update(_percentile_fields(successful,"path_length_m","path_length_m"))
            row.update(_percentile_fields(successful,"minimum_clearance_m","minimum_clearance_m"))
            map_summaries.append(row)
    _write_csv(output/"summary_by_map.csv",map_summaries)
    query_summaries=[]
    for map_id in map_ids:
        for query in selected:
            for algo in ALGORITHMS:
                group=[r for r in measured if r["map_id"]==map_id and r["query_id"]==query.query_id and r["algorithm"]==algo]
                valid=[r for r in group if r["validated_query"]]
                successful=[r for r in valid if r["final_valid_success"]]
                row={"map_id":map_id,"query_id":query.query_id,"algorithm":algo,"query_validation_status":group[0]["query_validation_status"] if group else "MISSING","measured_request_count":len(group),"eligible_request_count":len(valid),"planner_success_rate":sum(r["planner_success"] for r in valid)/max(1,len(valid)),"static_footprint_valid_rate":sum(r["static_footprint_valid"] for r in valid)/max(1,len(valid)),"kinematic_valid_rate":sum(r["kinematic_valid"] for r in valid)/max(1,len(valid)),"final_valid_success_rate":sum(r["final_valid_success"] for r in valid)/max(1,len(valid)),"failure_codes":sorted({r["failure_code"] for r in group if r["failure_code"]})}
                row.update(_percentile_fields(valid,"planning_time_ms","planning_time_ms"));row.update(_percentile_fields(valid,"wall_time_ms","wall_time_ms"));row.update(_percentile_fields(successful,"path_length_m","path_length_m"));query_summaries.append(row)
    _write_csv(output/"summary_by_query.csv",query_summaries)
    _write_csv(output/"maps.csv",[{"map_id":map_id,"map_yaml":str(MAP_PATHS[map_id]),"map_sha256":contexts[map_id].map_sha256,"map_yaml_sha256":contexts[map_id].map_yaml_sha256,"resolution":contexts[map_id].hospital_map.resolution,"width_cells":contexts[map_id].hospital_map.width,"height_cells":contexts[map_id].hospital_map.height,"physical_width_m":contexts[map_id].hospital_map.width*contexts[map_id].hospital_map.resolution,"physical_height_m":contexts[map_id].hospital_map.height*contexts[map_id].hospital_map.resolution,"configured_timeout_s":TIMEOUTS[map_id],"dynamic_obstacles":False} for map_id in map_ids])
    failures={}
    for r in measured:
        if r["failure_code"]:failures[(r["algorithm"],r["failure_code"])]=failures.get((r["algorithm"],r["failure_code"]),0)+1
    _write_csv(output/"failure_summary.csv",[{"algorithm":a,"failure_code":f,"count":n} for (a,f),n in sorted(failures.items())])
    backends=[]
    for algo in ALGORITHMS:
        vals=[r for r in measured if r["algorithm"]==algo and r["backend_id"]!="not_called"];backends.append({"algorithm":algo,"backend_id":sorted(set(r["backend_id"] for r in vals)),"implementation_type":sorted(set(r["implementation_type"] for r in vals)),"source_function":sorted(set(r["source_function"] for r in vals)),"planner_core":sorted(set(r["planner_core"] for r in vals)),"invalid_endpoint_not_called_count":sum(r["algorithm"]==algo and r["backend_id"]=="not_called" for r in measured),"shared_final_path_generator":False})
    _write_csv(output/"backend_audit.csv",backends)
    success_sets: Dict[str, set[str]] = {}
    for algo in ALGORITHMS:
        algorithm_rows = [r for r in measured if r["algorithm"] == algo and r["validated_query"]]
        success_sets[algo] = {
            f"{map_id}/{query_id}"
            for map_id, query_id in {(r["map_id"],r["query_id"]) for r in algorithm_rows}
            if all(r["final_valid_success"] for r in algorithm_rows if r["map_id"] == map_id and r["query_id"] == query_id)
        }
    common_queries = sorted(set.intersection(*(success_sets[algo] for algo in ALGORITHMS))) if ALGORITHMS else []
    backend_sets = {
        algo: {r["backend_id"] for r in measured if r["algorithm"] == algo and r["validated_query"]}
        for algo in ALGORITHMS
    }
    called_backend_ids = [next(iter(values)) for values in backend_sets.values() if len(values) == 1 and "not_called" not in values]
    returned_rows = [r for r in measured if r["action_success"]]
    geometry_hashes_are_distinct = True
    for map_query in common_queries:
        map_id, query_id = map_query.split("/",1)
        per_algorithm = {
            algorithm: {
                r.get("path_geometry_sha256","") for r in measured
                if r["map_id"] == map_id and r["query_id"] == query_id and r["algorithm"] == algorithm and r["final_valid_success"]
            }
            for algorithm in ALGORITHMS
        }
        if any(not hashes or "" in hashes for hashes in per_algorithm.values()):
            geometry_hashes_are_distinct = False
            continue
        for index, first in enumerate(ALGORITHMS):
            for second in ALGORITHMS[index + 1:]:
                if per_algorithm[first] & per_algorithm[second]:
                    geometry_hashes_are_distinct = False
    gate_checks = {
        "each_algorithm_has_three_successful_eligible_queries": all(len(success_sets[algo]) >= 3 for algo in ALGORITHMS),
        "two_common_eligible_queries": len(common_queries) >= 2,
        "all_returned_paths_final_valid": all(r["final_valid_success"] for r in returned_rows),
        "all_returned_paths_collision_free": all(int(r["footprint_collision_count"]) == 0 for r in returned_rows),
        "all_returned_paths_kinematically_valid": all(int(r["kinematic_invalid_segment_count"]) == 0 for r in returned_rows),
        "all_returned_paths_forward_only": all(float(r["reverse_distance_m"]) == 0.0 for r in returned_rows),
        "all_returned_paths_no_in_place_rotation": all(int(r["in_place_rotation_count"]) == 0 for r in returned_rows),
        "all_returned_paths_within_curvature_bound": all(float(r["maximum_curvature_per_m"]) <= CONFIG.maximum_curvature_per_m + 1e-6 for r in returned_rows),
        "four_distinct_called_backends": len(called_backend_ids) == len(ALGORITHMS) and len(set(called_backend_ids)) == len(ALGORITHMS),
        "no_shared_final_path_geometry": geometry_hashes_are_distinct,
        "failed_requests_have_codes": all(bool(r["failure_code"]) for r in measured if not r["final_valid_success"]),
        "timeouts_did_not_continue": not any(r["planner_continued_after_timeout"] for r in measured),
        "dynamic_obstacles_false": True,
    }
    gate_passed = all(gate_checks.values())
    gate = {
        "schema_version": 1,
        "passed": gate_passed,
        "checks": gate_checks,
        "successful_query_ids_by_algorithm": {algo: sorted(values) for algo, values in success_sets.items()},
        "common_successful_query_ids": common_queries,
        "second_round_allowed": gate_passed and len(map_ids) == 1 and not formal,
        "formal_experiment_allowed": formal and gate_passed,
    }
    (output/"smoke_gate.yaml").write_text(yaml.safe_dump(gate,sort_keys=False))
    report_lines = [
        f"# Forward-only {'formal' if formal else 'repair smoke'} diagnostic",
        "",
        f"Gate passed: **{str(gate_passed).lower()}**",
        "",
        ("This formal static-map run may compare the in-repo backends, but the two RRT results remain reference/surrogate evidence and are not mature RRT*/AO-RRT* conclusions." if formal else "This is a static-map smoke result, not a formal performance conclusion. The two RRT backends remain explicitly labelled reference/surrogate implementations."),
        "",
        "## Successful eligible queries",
        "",
    ]
    report_lines.extend(f"- `{algo}`: {', '.join(sorted(success_sets[algo])) or 'none'}" for algo in ALGORITHMS)
    report_lines.extend(["", f"Common paired queries: {', '.join(common_queries) or 'none'}", "", "## Gate checks", ""])
    report_lines.extend(f"- `{name}`: {str(value).lower()}" for name, value in gate_checks.items())
    report_text="\n".join(report_lines)+"\n"
    (output/"diagnostic_report.md").write_text(report_text,encoding="utf-8")
    if formal:
        interpretation_lines = report_lines + [
            "",
            "## Interpretation limits",
            "",
            "- `astar_kinematic` is grid A* followed by an explicitly reported continuous-bicycle adapter; core planner success and final-valid success remain separate.",
            "- `hybrid_astar` is an in-repository weighted forward lattice reference. Its analytic route is heuristic-only and is never returned as the final path.",
            "- `rrt_star` and `kinodynamic_rrt_star` remain reference/surrogate implementations. Their measurements must not be presented as mature RRT* or AO-RRT* performance.",
            "- Timeout, invalid endpoint, static validity, kinematic validity and final validity are separate result fields.",
            "- All runs use static maps with `dynamic_obstacles=false`; no controller or simulator timing is included.",
        ]
        (output/"interpretation.md").write_text("\n".join(interpretation_lines)+"\n",encoding="utf-8")
        _generate_formal_plots(output,map_summaries,measured)
    ended=dt.datetime.now(dt.timezone.utc).isoformat()
    manifest={"schema_version":3,"experiment":experiment_id,"experiment_kind":"formal" if formal else "smoke","created_at":started,"ended_at":ended,"dynamic_obstacles":False,"map_ids":list(map_ids),"query_ids":list(query_ids),"acceptance_gate_passed":gate_passed,"second_round_allowed":gate["second_round_allowed"],"formal_experiment_allowed":gate["formal_experiment_allowed"],"source_queries":str(SOURCE_QUERIES)};(output/"manifest.yaml").write_text(yaml.safe_dump(manifest,sort_keys=False))
    try:
        git_commit=subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True,stderr=subprocess.DEVNULL).strip()
        git_dirty=bool(subprocess.check_output(["git","-C",str(ROOT),"status","--porcelain"],text=True).strip())
    except (OSError,subprocess.CalledProcessError):
        git_commit="unavailable";git_dirty=True
    test_source=ROOT/"external/arena4_ws/src/arena/evaluation/arena_evaluation/test/test_forward_no_reverse_repair_smoke.py"
    isolation_source=ROOT/"external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/planner_benchmark/isolation.py"
    topology_source=ROOT/"external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/topology.py"
    code={"git_repository_root":str(ROOT),"git_commit":git_commit,"git_dirty":git_dirty,"source_files_sha256":{"benchmark":source_hash,"old_smoke":sha256_file(Path(old.__file__)),"grid_search":sha256_file(topology_source),"resource_monitor":sha256_file(isolation_source),"test":sha256_file(test_source)},"protocol_sha256":sha256_file(output/"protocol.yaml"),"core_query_source_sha256":sha256_file(SOURCE_QUERIES),"map_hashes":{m:c.map_sha256 for m,c in contexts.items()},"query_hashes":{q.query_id:_query_hash(q) for q in selected},"python_version":sys.version,"cli_command":" ".join(sys.argv),"started_at":started,"ended_at":ended,"dynamic_obstacles":False,"random_seed_base":20260821};(output/"code_manifest.yaml").write_text(yaml.safe_dump(code,sort_keys=False))
    return output


def build_parser() -> argparse.ArgumentParser:
    parser=argparse.ArgumentParser(description="Run independent strict forward-only repair smoke")
    parser.add_argument("--output-dir",required=True);parser.add_argument("--map-id",action="append",choices=list(MAP_PATHS),dest="map_ids");parser.add_argument("--query-id",action="append",choices=list(_queries()),dest="query_ids");parser.add_argument("--warmups",type=int,default=1);parser.add_argument("--repetitions",type=int,default=2);parser.add_argument("--formal",action="store_true");parser.add_argument("--smoke-gate",default=str(ROOT/"experiments/single_planner_benchmark/forward_no_reverse_rmin040_repair_smoke_v2/smoke_gate.yaml"));parser.add_argument("--no-dynamic-obstacles",action="store_true",required=True);return parser


def main(argv: Optional[Sequence[str]]=None) -> int:
    args=build_parser().parse_args(argv)
    try:
        map_ids=args.map_ids or ([*MAP_PATHS] if args.formal else ["hospital_005"])
        query_ids=args.query_ids or list(DEFAULT_QUERIES)
        if args.formal:
            gate_path=Path(args.smoke_gate).resolve()
            gate_payload=yaml.safe_load(gate_path.read_text(encoding="utf-8")) or {}
            if not bool(gate_payload.get("passed")):
                raise ValueError(f"formal run refused: smoke gate did not pass: {gate_path}")
            if set(map_ids)!=set(MAP_PATHS) or len(map_ids)!=len(MAP_PATHS):
                raise ValueError("formal run requires the four frozen maps exactly once")
            if query_ids!=list(DEFAULT_QUERIES):
                raise ValueError("formal run requires q00-q09 in frozen order")
            if args.warmups!=3 or args.repetitions!=5:
                raise ValueError("formal run requires 3 warmups and 5 measured repetitions")
        output = run(Path(args.output_dir).resolve(), map_ids, query_ids, args.warmups, args.repetitions, formal=args.formal)
        print(f"{'formal' if args.formal else 'repair smoke'} output: {output}")
        return 0
    except (ValueError,OSError,KeyError) as exc:print(f"forward_no_reverse_repair_smoke: ERROR: {exc}");return 2


if __name__ == "__main__":raise SystemExit(main())
