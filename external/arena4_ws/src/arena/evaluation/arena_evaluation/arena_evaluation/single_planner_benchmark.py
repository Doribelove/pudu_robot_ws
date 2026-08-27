"""Static four-algorithm Ackermann-surrogate A2B benchmark.

The benchmark is deliberately independent of ROS, Nav2 and Gazebo.  It runs
on the fixed 0.05 m maps and records planner output separately from the static
footprint and Ackermann acceptance checks.  The sampling planners are small,
deterministic reference implementations: ``rrt_star_dubins_surrogate`` is
named explicitly because OMPL is not available in the Arena runtime, while
``kinodynamic_rrt_star_bicycle`` exposes the requested x/y/yaw/v/steering
state and bicycle controls.
"""

from __future__ import annotations

import argparse
import datetime as dt
import gzip
import hashlib
import heapq
import json
import math
import os
import random
import resource
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
import yaml

from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .planner_benchmark.resources import read_snapshot
from .topology import astar_grid, preprocess_static_map


ROOT = Path("/home/robot/pudu_robot_ws")
SOURCE_QUERIES = ROOT / "experiments/planner_benchmark/hospital_005/queries_v2.yaml"
MAP_IDS = (
    "hospital_005",
    "hospital_boundary_100x100_005",
    "hospital_boundary_200x200_005",
    "hospital_boundary_400x400_005",
)
MAP_PATHS = {
    "hospital_005": ROOT / "experiments/maps/hospital_005/map.yaml",
    "hospital_boundary_100x100_005": ROOT / "experiments/maps/hospital_boundary_100x100_005/map.yaml",
    "hospital_boundary_200x200_005": ROOT / "experiments/maps/hospital_boundary_200x200_005/map.yaml",
    "hospital_boundary_400x400_005": ROOT / "experiments/maps/hospital_boundary_400x400_005/map.yaml",
}
TIMEOUTS = {
    "hospital_005": 5.0,
    "hospital_boundary_100x100_005": 5.0,
    "hospital_boundary_200x200_005": 15.0,
    "hospital_boundary_400x400_005": 60.0,
}
FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]
ALGORITHMS = ("astar", "hybrid_astar", "rrt_star_dubins_surrogate", "kinodynamic_rrt_star_bicycle")
MAP_SCALE_LABEL = {
    "hospital_005": "80x80m",
    "hospital_boundary_100x100_005": "100x100m",
    "hospital_boundary_200x200_005": "200x200m",
    "hospital_boundary_400x400_005": "400x400m",
}
MAP_SCALE_M = {
    "hospital_005": 80.0,
    "hospital_boundary_100x100_005": 100.0,
    "hospital_boundary_200x200_005": 200.0,
    "hospital_boundary_400x400_005": 400.0,
}
ALGORITHM_LABEL = {
    "astar": "8-neighbor A*",
    "hybrid_astar": "SE(2) Hybrid A*",
    "rrt_star_dubins_surrogate": "RRT* Dubins surrogate",
    "kinodynamic_rrt_star_bicycle": "Kinodynamic RRT* bicycle",
}


@dataclass(frozen=True)
class AckermannConfig:
    wheelbase_m: float = 0.50
    max_steering_angle_rad: float = math.radians(30.0)
    minimum_turning_radius_m: float = 0.866
    maximum_curvature_per_m: float = 1.155
    allow_reverse: bool = False
    allow_in_place_rotation: bool = False
    footprint_padding_m: float = 0.05
    safety_margin_m: float = 0.05
    endpoint_position_tolerance_m: float = 0.25
    endpoint_yaw_tolerance_rad: float = math.radians(10.0)
    sample_spacing_m: float = 0.05
    curvature_window_m: float = 0.15
    steering_rate_rad_s: float = 0.6
    acceleration_m_s2: float = 0.5
    velocity_max_m_s: float = 1.0
    # Search discretisation is explicit so protocol metadata can be checked
    # against the implementation.  Legacy v1 defaults are retained; v2 uses
    # the relaxed factory in relaxed_single_planner_benchmark.py.
    hybrid_step_size_m: float = 0.75
    angle_resolution_deg: float = 10.0
    angle_bins: int = 36
    steering_angles_deg: Tuple[float, ...] = ()
    integration_sample_spacing_m: float = 0.05
    state_limit: Optional[int] = 120000
    goal_connection_enabled: bool = False


def relaxed_ackermann_config() -> AckermannConfig:
    """Return the frozen v2 60-degree forward-only surrogate configuration.

    The radius and curvature are derived from the wheelbase and steering angle
    rather than copied as independent tolerances.  The legacy default config
    remains unchanged for the v1 pressure-control data.
    """
    wheelbase = 0.50
    steering_deg = 60.0
    steering_rad = math.radians(steering_deg)
    radius = wheelbase / math.tan(steering_rad)
    curvature = 1.0 / radius
    steering_set = tuple(math.radians(value) for value in (-60, -45, -30, -15, 0, 15, 30, 45, 60))
    return AckermannConfig(
        wheelbase_m=wheelbase,
        max_steering_angle_rad=steering_rad,
        minimum_turning_radius_m=radius,
        maximum_curvature_per_m=curvature,
        allow_reverse=False,
        allow_in_place_rotation=False,
        hybrid_step_size_m=0.25,
        angle_resolution_deg=5.0,
        angle_bins=72,
        steering_angles_deg=tuple((-60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0)),
        integration_sample_spacing_m=0.05,
        state_limit=None,
        goal_connection_enabled=True,
    )


@dataclass
class MapContext:
    map_id: str
    hospital_map: HospitalMap
    free_mask: np.ndarray
    distance_m: np.ndarray
    map_sha256: str
    map_yaml_sha256: str
    metadata: Dict[str, Any]


@dataclass
class PlannerResult:
    planner_success: bool = False
    points: Optional[List[Dict[str, float]]] = None
    failure_code: str = "NO_PATH"
    detail: str = ""
    search_states: int = 0
    expanded_states: int = 0
    samples: int = 0
    rewires: int = 0
    first_solution_time_ms: Optional[float] = None
    seed: Optional[int] = None
    angle_resolution_deg: Optional[float] = None
    step_size_m: Optional[float] = None
    dt_s: Optional[float] = None
    configured_timeout_s: Optional[float] = None
    timeout_triggered: bool = False
    state_limit: Optional[int] = None
    goal_connection_attempts: int = 0
    goal_connection_successes: int = 0
    goal_connection_failure: str = ""
    diagnostics: Optional[Dict[str, Any]] = None


def _wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _delta(a: float, b: float) -> float:
    return _wrap(float(a) - float(b))


def _read_yaml(path: Path) -> Dict[str, Any]:
    return yaml.safe_load(path.read_text()) or {}


def _write_yaml(path: Path, value: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False))


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _query_objects(path: Path) -> List[Query]:
    payload = _read_yaml(path)
    return [Query(
        query_id=str(item["query_id"]),
        start=[float(v) for v in item["start"]],
        goal=[float(v) for v in item["goal"]],
        category=str(item.get("category", item.get("legacy_category", "unspecified"))),
        seed=int(item.get("seed", payload.get("seed", 20260821))),
        validation_status=str(item.get("validation_status", "UNVALIDATED")),
    ) for item in payload.get("queries", [])]


def _context(map_id: str, protocol: Dict[str, Any]) -> MapContext:
    map_yaml = MAP_PATHS[map_id]
    hospital_map = HospitalMap.load(map_yaml)
    if not math.isclose(hospital_map.resolution, 0.05, abs_tol=1e-12):
        raise ValueError(f"{map_id}: resolution must be 0.05 m")
    map_meta_path = map_yaml.parent / "metadata.yaml"
    metadata = _read_yaml(map_meta_path) if map_meta_path.exists() else {}
    if map_id != "hospital_005":
        if metadata.get("gate_plan_version") != "hospital_boundary_gates_v1":
            raise ValueError(f"{map_id}: missing hospital_boundary_gates_v1 metadata")
        if int(metadata.get("gate_count", 0)) != 10 or not math.isclose(float(metadata.get("gate_width_m", 0.0)), 1.0, abs_tol=1e-9):
            raise ValueError(f"{map_id}: boundary gate metadata is not fixed 1.0 m / 10 gates")
        if not metadata.get("outer_free_region_connected_to_source_query_space", False):
            raise ValueError(f"{map_id}: outer free region is not connected through gates")
        if not metadata.get("source_unchanged_outside_gates", False):
            raise ValueError(f"{map_id}: source core outside gates was not preserved")
    _, free, distance_m, _ = preprocess_static_map(
        hospital_map, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    return MapContext(
        map_id=map_id, hospital_map=hospital_map, free_mask=free, distance_m=distance_m,
        map_sha256=sha256_file(hospital_map.image_path),
        map_yaml_sha256=sha256_file(hospital_map.yaml_path), metadata=metadata,
    )


def _pose_from_cell(hospital_map: HospitalMap, cell: Tuple[int, int], yaw: float) -> Dict[str, float]:
    x, y = hospital_map.cell_to_world(cell)
    return {"x": float(x), "y": float(y), "yaw": float(_wrap(yaw))}


def _astar(ctx: MapContext, query: Query, timeout_s: float) -> PlannerResult:
    start = ctx.hospital_map.world_to_cell(query.start[0], query.start[1])
    goal = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start is None or goal is None or not ctx.free_mask[start] or not ctx.free_mask[goal]:
        return PlannerResult(failure_code="INVALID_ENDPOINT")
    result = astar_grid(ctx.free_mask, start, goal, resolution=ctx.hospital_map.resolution, return_stats=True, timeout_s=timeout_s)
    if result.path is None:
        return PlannerResult(
            failure_code=result.failure_code or "NO_PATH",
            expanded_states=result.expanded_nodes,
            search_states=result.generated_nodes,
            configured_timeout_s=result.configured_timeout_s or timeout_s,
            timeout_triggered=result.timeout_triggered,
        )
    points: List[Dict[str, float]] = []
    for i, cell in enumerate(result.path):
        if i == 0:
            yaw = query.start[2]
        elif i == len(result.path) - 1:
            yaw = query.goal[2]
        else:
            x0, y0 = ctx.hospital_map.cell_to_world(result.path[i - 1])
            x1, y1 = ctx.hospital_map.cell_to_world(result.path[i + 1])
            yaw = math.atan2(y1 - y0, x1 - x0)
        points.append(_pose_from_cell(ctx.hospital_map, cell, yaw))
    return PlannerResult(
        planner_success=True,
        points=points,
        search_states=result.generated_nodes,
        expanded_states=result.expanded_nodes,
        step_size_m=ctx.hospital_map.resolution,
        configured_timeout_s=result.configured_timeout_s or timeout_s,
    )


def _free_pose(ctx: MapContext, x: float, y: float) -> bool:
    cell = ctx.hospital_map.world_to_cell(x, y)
    return cell is not None and bool(ctx.free_mask[cell])


def _integrate_bicycle(x: float, y: float, yaw: float, steering: float, distance: float, wheelbase: float, samples: int = 5) -> List[Tuple[float, float, float]]:
    result = []
    # Keep every integration/collision sample at or below 5 cm.  This upper
    # bound applies even when a legacy caller supplies a coarser sample count.
    samples = max(int(samples), int(math.ceil(abs(float(distance)) / 0.05)))
    ds = float(distance) / max(1, samples)
    curvature = math.tan(float(steering)) / float(wheelbase)
    for _ in range(max(1, samples)):
        if abs(curvature) < 1e-9:
            x += ds * math.cos(yaw); y += ds * math.sin(yaw)
        else:
            yaw_new = yaw + ds * curvature
            radius = 1.0 / curvature
            x += radius * (math.sin(yaw_new) - math.sin(yaw))
            y += -radius * (math.cos(yaw_new) - math.cos(yaw))
            yaw = yaw_new
        result.append((x, y, yaw))
    return result


def _hybrid_astar(ctx: MapContext, query: Query, config: AckermannConfig, timeout_s: float) -> PlannerResult:
    """Deterministic forward-only SE(2) lattice search with Dubins bicycle actions."""
    started = time.monotonic()
    angle_bins = 36
    step = 0.75
    steering_set = (-config.max_steering_angle_rad, -config.max_steering_angle_rad / 2.0, 0.0, config.max_steering_angle_rad / 2.0, config.max_steering_angle_rad)
    start_cell = ctx.hospital_map.world_to_cell(query.start[0], query.start[1])
    goal_cell = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start_cell is None or goal_cell is None or not ctx.free_mask[start_cell] or not ctx.free_mask[goal_cell]:
        return PlannerResult(failure_code="INVALID_ENDPOINT", angle_resolution_deg=5.0, step_size_m=step)
    def key(x: float, y: float, yaw: float) -> Tuple[int, int, int]:
        cell = ctx.hospital_map.world_to_cell(x, y)
        if cell is None:
            return (-1, -1, 0)
        heading = int(round((_wrap(yaw) + math.pi) / (2.0 * math.pi) * angle_bins)) % angle_bins
        return cell[0], cell[1], heading
    start_state = (float(query.start[0]), float(query.start[1]), float(query.start[2]))
    start_key = key(*start_state)
    queue = [(math.hypot(query.goal[0] - start_state[0], query.goal[1] - start_state[1]), 0.0, start_key)]
    state_for = {start_key: start_state}; parent: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {start_key: None}
    cost = {start_key: 0.0}; expanded = 0
    goal_key = None
    while queue:
        if time.monotonic() - started > timeout_s:
            return PlannerResult(False, None, "TIMEOUT", expanded_states=expanded, search_states=len(cost), angle_resolution_deg=10.0, step_size_m=step)
        _, current_cost, current_key = heapq.heappop(queue)
        if current_cost != cost.get(current_key):
            continue
        expanded += 1
        x, y, yaw = state_for[current_key]
        if math.hypot(query.goal[0] - x, query.goal[1] - y) <= config.endpoint_position_tolerance_m and abs(_delta(yaw, query.goal[2])) <= config.endpoint_yaw_tolerance_rad:
            goal_key = current_key; break
        if expanded > 120000:
            return PlannerResult(False, None, "SEARCH_LIMIT", expanded_states=expanded, search_states=len(cost), angle_resolution_deg=10.0, step_size_m=step)
        for steering in steering_set:
            samples = _integrate_bicycle(x, y, yaw, steering, step, config.wheelbase_m, samples=5)
            if not all(_free_pose(ctx, p[0], p[1]) for p in samples):
                continue
            nx, ny, nyaw = samples[-1]
            nkey = key(nx, ny, nyaw)
            if nkey[0] < 0:
                continue
            new_cost = current_cost + step
            if new_cost >= cost.get(nkey, float("inf")):
                continue
            state_for[nkey] = (nx, ny, _wrap(nyaw)); cost[nkey] = new_cost; parent[nkey] = current_key
            heuristic = math.hypot(query.goal[0] - nx, query.goal[1] - ny)
            heapq.heappush(queue, (new_cost + heuristic, new_cost, nkey))
    if goal_key is None:
        return PlannerResult(False, None, "NO_PATH", expanded_states=expanded, search_states=len(cost), angle_resolution_deg=10.0, step_size_m=step)
    keys = []
    cursor = goal_key
    while cursor is not None:
        keys.append(cursor); cursor = parent[cursor]
    states = [state_for[item] for item in reversed(keys)]
    points = [{"x": s[0], "y": s[1], "yaw": _wrap(s[2])} for s in states]
    if math.hypot(points[-1]["x"] - query.goal[0], points[-1]["y"] - query.goal[1]) > 1e-6:
        points.append({"x": query.goal[0], "y": query.goal[1], "yaw": query.goal[2]})
    return PlannerResult(True, points, "", len(cost), expanded, angle_resolution_deg=10.0, step_size_m=step)


def _rrt_extend(ctx: MapContext, state: Tuple[float, float, float], target: Tuple[float, float, float], config: AckermannConfig, step: float) -> Optional[List[Tuple[float, float, float]]]:
    x, y, yaw = state
    desired = math.atan2(target[1] - y, target[0] - x)
    steering = max(-config.max_steering_angle_rad, min(config.max_steering_angle_rad, 1.8 * _delta(desired, yaw)))
    distance = min(float(step), math.hypot(target[0] - x, target[1] - y))
    samples = _integrate_bicycle(x, y, yaw, steering, distance, config.wheelbase_m, samples=max(4, int(math.ceil(distance / 0.1))))
    if not all(_free_pose(ctx, p[0], p[1]) for p in samples):
        return None
    return samples


def _rrt_extend_best(ctx: MapContext, state: Tuple[float, float, float], target: Tuple[float, float, float], config: AckermannConfig, step: float) -> Optional[List[Tuple[float, float, float]]]:
    """Try a small deterministic control fan for the Dubins surrogate."""
    desired = math.atan2(target[1] - state[1], target[0] - state[0])
    preferred = max(-config.max_steering_angle_rad, min(config.max_steering_angle_rad, 1.8 * _delta(desired, state[2])))
    controls = (preferred, -config.max_steering_angle_rad, 0.0, config.max_steering_angle_rad)
    candidates = []
    for steering in controls:
        distance = min(float(step), math.hypot(target[0] - state[0], target[1] - state[1]))
        samples = _integrate_bicycle(state[0], state[1], state[2], steering, distance, config.wheelbase_m, samples=max(4, int(math.ceil(distance / 0.1))))
        if all(_free_pose(ctx, p[0], p[1]) for p in samples):
            candidates.append((math.hypot(samples[-1][0] - target[0], samples[-1][1] - target[1]), samples))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def _rrt_star(ctx: MapContext, query: Query, config: AckermannConfig, timeout_s: float, seed: int) -> PlannerResult:
    started = time.monotonic(); rng = random.Random(seed); step = 1.0
    bounds = (ctx.hospital_map.origin[0], ctx.hospital_map.origin[0] + ctx.hospital_map.width * ctx.hospital_map.resolution,
              ctx.hospital_map.origin[1], ctx.hospital_map.origin[1] + ctx.hospital_map.height * ctx.hospital_map.resolution)
    states = [(float(query.start[0]), float(query.start[1]), float(query.start[2]))]; parents = [-1]; costs = [0.0]
    samples = 0; rewires = 0; first = None; goal_index = None
    max_samples = 2500
    while samples < max_samples and time.monotonic() - started <= timeout_s:
        samples += 1
        if rng.random() < 0.15:
            target = (query.goal[0], query.goal[1], query.goal[2])
        else:
            target = (rng.uniform(bounds[0], bounds[1]), rng.uniform(bounds[2], bounds[3]), rng.uniform(-math.pi, math.pi))
        nearest = min(range(len(states)), key=lambda i: math.hypot(states[i][0] - target[0], states[i][1] - target[1]) + 0.15 * abs(_delta(states[i][2], target[2])))
        segment = _rrt_extend_best(ctx, states[nearest], target, config, step)
        if not segment:
            continue
        endpoint = segment[-1]; nearby = [i for i, s in enumerate(states) if math.hypot(s[0] - endpoint[0], s[1] - endpoint[1]) < max(1.0, 2.0 * math.sqrt(math.log(len(states) + 2) / (len(states) + 1)))]
        parent_index = nearest; parent_cost = costs[nearest] + step
        for candidate in nearby:
            if costs[candidate] < parent_cost:
                parent_index = candidate; parent_cost = costs[candidate] + math.hypot(states[candidate][0] - endpoint[0], states[candidate][1] - endpoint[1])
        states.append(endpoint); parents.append(parent_index); costs.append(parent_cost); new_index = len(states) - 1
        for candidate in nearby:
            candidate_cost = parent_cost + math.hypot(states[candidate][0] - endpoint[0], states[candidate][1] - endpoint[1])
            if candidate_cost + 1e-9 < costs[candidate]:
                parents[candidate] = new_index; costs[candidate] = candidate_cost; rewires += 1
        if math.hypot(endpoint[0] - query.goal[0], endpoint[1] - query.goal[1]) <= config.endpoint_position_tolerance_m and abs(_delta(endpoint[2], query.goal[2])) <= config.endpoint_yaw_tolerance_rad:
            goal_index = new_index; first = first or (time.monotonic() - started) * 1000.0; break
    if goal_index is None:
        code = "TIMEOUT" if time.monotonic() - started > timeout_s else "NO_PATH"
        return PlannerResult(
            planner_success=False,
            points=None,
            failure_code=code,
            search_states=len(states),
            samples=samples,
            rewires=rewires,
            first_solution_time_ms=first,
            seed=seed,
            step_size_m=step,
        )
    chain = []; cursor = goal_index
    while cursor >= 0:
        chain.append(states[cursor]); cursor = parents[cursor]
    chain.reverse(); chain.append((query.goal[0], query.goal[1], query.goal[2]))
    return PlannerResult(
        planner_success=True,
        points=[{"x": float(x), "y": float(y), "yaw": _wrap(yaw)} for x, y, yaw in chain],
        failure_code="",
        search_states=len(states),
        expanded_states=len(states),
        samples=samples,
        rewires=rewires,
        first_solution_time_ms=first,
        seed=seed,
        step_size_m=step,
    )


def _kinodynamic_rrt_star(ctx: MapContext, query: Query, config: AckermannConfig, timeout_s: float, seed: int) -> PlannerResult:
    started = time.monotonic(); rng = random.Random(seed); dt_s = 0.5
    bounds = (ctx.hospital_map.origin[0], ctx.hospital_map.origin[0] + ctx.hospital_map.width * ctx.hospital_map.resolution,
              ctx.hospital_map.origin[1], ctx.hospital_map.origin[1] + ctx.hospital_map.height * ctx.hospital_map.resolution)
    states = [(float(query.start[0]), float(query.start[1]), float(query.start[2]), 0.5, 0.0)]; parents = [-1]; costs = [0.0]
    samples = 0; rewires = 0; first = None; goal_index = None; controls = (-config.acceleration_m_s2, 0.0, config.acceleration_m_s2)
    while samples < 2500 and time.monotonic() - started <= timeout_s:
        samples += 1
        target = (query.goal[0], query.goal[1], query.goal[2], rng.uniform(0.0, 1.0), rng.uniform(-config.max_steering_angle_rad, config.max_steering_angle_rad)) if rng.random() < 0.2 else (rng.uniform(bounds[0], bounds[1]), rng.uniform(bounds[2], bounds[3]), rng.uniform(-math.pi, math.pi), rng.uniform(0.0, 1.0), rng.uniform(-config.max_steering_angle_rad, config.max_steering_angle_rad))
        nearest = min(range(len(states)), key=lambda i: math.hypot(states[i][0] - target[0], states[i][1] - target[1]) + 0.15 * abs(_delta(states[i][2], target[2])))
        x, y, yaw, velocity, steering = states[nearest]
        accel = min(controls, key=lambda a: abs((velocity + a * dt_s) - target[3]))
        steer_rate = max(-config.steering_rate_rad_s, min(config.steering_rate_rad_s, _delta(target[4], steering) / dt_s))
        nv = max(0.0, min(config.velocity_max_m_s, velocity + accel * dt_s)); ns = max(-config.max_steering_angle_rad, min(config.max_steering_angle_rad, steering + steer_rate * dt_s))
        distance = max(0.05, nv * dt_s)
        segment = _integrate_bicycle(x, y, yaw, ns, distance, config.wheelbase_m, samples=6)
        if not all(_free_pose(ctx, p[0], p[1]) for p in segment):
            continue
        endpoint = (*segment[-1], nv, ns); states.append(endpoint); parents.append(nearest); costs.append(costs[nearest] + distance); new_index = len(states) - 1
        nearby = [i for i, s in enumerate(states[:-1]) if math.hypot(s[0] - endpoint[0], s[1] - endpoint[1]) < max(1.0, 2.0 * math.sqrt(math.log(len(states) + 2) / (len(states) + 1)))]
        for candidate in nearby:
            candidate_cost = costs[new_index] + math.hypot(states[candidate][0] - endpoint[0], states[candidate][1] - endpoint[1])
            if candidate_cost < costs[candidate]:
                parents[candidate] = new_index; costs[candidate] = candidate_cost; rewires += 1
        if math.hypot(endpoint[0] - query.goal[0], endpoint[1] - query.goal[1]) <= config.endpoint_position_tolerance_m and abs(_delta(endpoint[2], query.goal[2])) <= config.endpoint_yaw_tolerance_rad:
            goal_index = new_index; first = first or (time.monotonic() - started) * 1000.0; break
    if goal_index is None:
        code = "TIMEOUT" if time.monotonic() - started > timeout_s else "NO_PATH"
        return PlannerResult(
            planner_success=False,
            points=None,
            failure_code=code,
            search_states=len(states),
            samples=samples,
            rewires=rewires,
            first_solution_time_ms=first,
            seed=seed,
            dt_s=dt_s,
        )
    chain = []; cursor = goal_index
    while cursor >= 0:
        chain.append(states[cursor]); cursor = parents[cursor]
    chain.reverse(); chain.append((query.goal[0], query.goal[1], query.goal[2], 0.0, states[goal_index][4]))
    return PlannerResult(
        planner_success=True,
        points=[{"x": float(s[0]), "y": float(s[1]), "yaw": _wrap(float(s[2])), "velocity": float(s[3]), "steering_angle": float(s[4])} for s in chain],
        failure_code="",
        search_states=len(states),
        expanded_states=len(states),
        samples=samples,
        rewires=rewires,
        first_solution_time_ms=first,
        seed=seed,
        dt_s=dt_s,
    )


def _resample(points: Sequence[Dict[str, float]], spacing: float) -> List[Dict[str, float]]:
    if not points:
        return []
    out = [dict(points[0])]
    for first, second in zip(points, points[1:]):
        dx = float(second["x"]) - float(first["x"]); dy = float(second["y"]) - float(first["y"]); dist = math.hypot(dx, dy)
        n = max(1, int(math.ceil(dist / spacing)))
        dyaw = _delta(float(second.get("yaw", 0.0)), float(first.get("yaw", 0.0)))
        for i in range(1, n + 1):
            f = i / n; out.append({"x": float(first["x"]) + f * dx, "y": float(first["y"]) + f * dy, "yaw": _wrap(float(first.get("yaw", 0.0)) + f * dyaw)})
    return out


def _curvature(a: Dict[str, float], b: Dict[str, float], c: Dict[str, float]) -> float:
    ab = math.hypot(b["x"] - a["x"], b["y"] - a["y"]); bc = math.hypot(c["x"] - b["x"], c["y"] - b["y"]); ac = math.hypot(c["x"] - a["x"], c["y"] - a["y"])
    if min(ab, bc, ac) <= 1e-9: return 0.0
    cross = abs((b["x"] - a["x"]) * (c["y"] - a["y"]) - (b["y"] - a["y"]) * (c["x"] - a["x"]))
    return 2.0 * cross / (ab * bc * ac)


def validate_path(ctx: MapContext, query: Query, points: Optional[Sequence[Dict[str, float]]], config: AckermannConfig) -> Dict[str, Any]:
    if not points:
        return {"static_footprint_valid": False, "kinematic_valid": False, "footprint_collision_count": 0, "kinematic_invalid_segment_count": 0, "failure_code": "EMPTY_PATH"}
    sampled = _resample(points, config.sample_spacing_m)
    collisions = 0; reverse_distance = 0.0; rotations = 0; heading_jumps = 0; curvatures = []
    previous_direction = None; switches = 0; position_discontinuities = 0
    for index, (a, b) in enumerate(zip(sampled, sampled[1:])):
        distance = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        if distance <= 1e-9:
            if abs(_delta(b["yaw"], a["yaw"])) > 1e-6: rotations += 1
            continue
        if distance > config.sample_spacing_m * 2.5: position_discontinuities += 1
        if abs(_delta(b["yaw"], a["yaw"])) > math.radians(25.0): heading_jumps += 1
        projection = (b["x"] - a["x"]) * math.cos(a["yaw"]) + (b["y"] - a["y"]) * math.sin(a["yaw"])
        direction = -1 if projection < -1e-6 else 1
        if previous_direction is not None and direction != previous_direction: switches += 1
        previous_direction = direction
        if direction < 0: reverse_distance += distance
        collisions += int(ctx.hospital_map.footprint_collision((a["x"], a["y"], a["yaw"]), FOOTPRINT, unknown_is_collision=True))
    if sampled: collisions += int(ctx.hospital_map.footprint_collision((sampled[-1]["x"], sampled[-1]["y"], sampled[-1]["yaw"]), FOOTPRINT, unknown_is_collision=True))
    for a, b, c in zip(sampled, sampled[1:], sampled[2:]): curvatures.append(_curvature(a, b, c))
    max_curvature = max(curvatures, default=0.0); radius = (1.0 / max_curvature if max_curvature > 1e-9 else None)
    goal_pos_error = math.hypot(points[-1]["x"] - query.goal[0], points[-1]["y"] - query.goal[1]); goal_yaw_error = abs(_delta(points[-1].get("yaw", 0.0), query.goal[2]))
    failures = []
    if collisions: failures.append("STATIC_FOOTPRINT_COLLISION")
    if reverse_distance > 1e-6: failures.append("REVERSE_MOTION")
    if rotations: failures.append("IN_PLACE_ROTATION")
    if max_curvature > config.maximum_curvature_per_m + 0.03: failures.append("MINIMUM_TURNING_RADIUS_VIOLATION")
    if position_discontinuities: failures.append("POSITION_DISCONTINUITY")
    if heading_jumps: failures.append("HEADING_DISCONTINUITY")
    if goal_pos_error > config.endpoint_position_tolerance_m: failures.append("ENDPOINT_POSITION_DISCONTINUITY")
    if goal_yaw_error > config.endpoint_yaw_tolerance_rad: failures.append("ENDPOINT_YAW_DISCONTINUITY")
    return {
        "static_footprint_valid": collisions == 0, "kinematic_valid": not failures,
        "footprint_collision_count": collisions, "kinematic_invalid_segment_count": int(sum([rotations, heading_jumps, position_discontinuities]) + (1 if max_curvature > config.maximum_curvature_per_m + 0.03 else 0)),
        "reverse_distance_m": reverse_distance, "reverse_ratio": reverse_distance / max(1e-9, sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(sampled, sampled[1:]))),
        "in_place_rotation_count": rotations, "direction_switch_count": switches, "heading_jump_count": heading_jumps,
        "minimum_turning_radius_m": radius, "curvature_p95_per_m": float(np.percentile(curvatures, 95)) if curvatures else 0.0, "maximum_curvature_per_m": max_curvature,
        "path_length_m": float(sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(sampled, sampled[1:]))),
        "euclidean_distance_m": math.hypot(query.goal[0] - query.start[0], query.goal[1] - query.start[1]),
        "minimum_clearance_m": float(min((ctx.hospital_map.clearance(p["x"], p["y"]) or 0.0) for p in sampled) if sampled else 0.0),
        "heading_change_p95_rad": float(np.percentile([abs(_delta(b["yaw"], a["yaw"])) for a, b in zip(sampled, sampled[1:])], 95)) if len(sampled) > 1 else 0.0,
        "goal_position_error_m": goal_pos_error, "goal_yaw_error_rad": goal_yaw_error,
        "failure_code": failures[0] if failures else "",
        "failure_codes": json.dumps(failures),
    }


def _planner_call(ctx: MapContext, query: Query, algorithm: str, config: AckermannConfig, timeout_s: float, seed: int) -> PlannerResult:
    if algorithm == "astar": return _astar(ctx, query, timeout_s)
    if algorithm == "hybrid_astar": return _hybrid_astar(ctx, query, config, timeout_s)
    if algorithm == "rrt_star_dubins_surrogate": return _rrt_star(ctx, query, config, timeout_s, seed)
    if algorithm == "kinodynamic_rrt_star_bicycle": return _kinodynamic_rrt_star(ctx, query, config, timeout_s, seed)
    raise ValueError(f"unknown algorithm {algorithm}")


def _protocol(map_ids: Sequence[str], maps: Dict[str, MapContext], output: Path) -> Dict[str, Any]:
    return {
        "schema_version": 1, "experiment": "pln02_single_planner_ackermann_no_reverse_v1", "dynamic_obstacles": False,
        "resolution": 0.05, "random_seed": 20260821, "warmup_runs": 3, "measured_runs": 5,
        "external_timeout_seconds": {m: TIMEOUTS[m] for m in map_ids}, "query_policy": "core_queries_v1_exact_world_coordinates",
        "vehicle_model": "Jackal footprint + Ackermann kinematic abstraction (not Jackal mechanical structure)",
        "physical_robot": "jackal", "vehicle_model_id": "ackermann_surrogate", "wheelbase_m": 0.50,
        "max_steering_angle_deg": 30.0, "minimum_turning_radius_m": 0.866, "maximum_curvature_per_m": 1.155,
        "allow_reverse": False, "allow_in_place_rotation": False, "velocity_range_m_s": [0.0, 1.0],
        "acceleration_limit_m_s2": 0.5, "steering_rate_limit_rad_s": 0.6, "footprint": FOOTPRINT,
        "footprint_padding_m": 0.05, "additional_safety_margin_m": 0.05, "allow_unknown": False,
        "endpoint_position_tolerance_m": 0.25, "endpoint_yaw_tolerance_deg": 10.0,
        "curvature_sample_spacing_m": 0.05, "curvature_window_m": 0.15,
        "algorithms": list(ALGORITHMS), "maps": [{"map_id": m, "map_yaml": str(MAP_PATHS[m]), "map_sha256": maps[m].map_sha256, "map_yaml_sha256": maps[m].map_yaml_sha256, "origin": list(maps[m].hospital_map.origin), "width_cells": maps[m].hospital_map.width, "height_cells": maps[m].hospital_map.height, "physical_area_m2": maps[m].hospital_map.width * maps[m].hospital_map.height * 0.05 * 0.05, "gate_plan_version": maps[m].metadata.get("gate_plan_version"), "gate_count": maps[m].metadata.get("gate_count", 0)} for m in map_ids],
        "algorithm_details": {
            "astar": {"name": "8-connected Euclidean grid A*", "topology": False},
            "hybrid_astar": {"name": "SE(2) forward Dubins Hybrid A*", "angle_resolution_deg": 10.0, "step_size_m": 0.75, "reverse": False},
            "rrt_star_dubins_surrogate": {"name": "RRT* with forward bicycle/Dubins surrogate local connector", "implementation": "in_repo_reference", "ompl_available": False, "step_size_m": 1.0, "goal_bias": 0.15},
            "kinodynamic_rrt_star_bicycle": {"name": "kinodynamic RRT* bicycle reference", "state": ["x", "y", "yaw", "v", "steering_angle"], "dt_s": 0.5, "ompl_available": False},
        },
    }


def _make_boundary_queries(root: Path, map_ids: Sequence[str]) -> Dict[str, Any]:
    all_queries = []
    for map_id in map_ids:
        if map_id == "hospital_005": continue
        meta = _read_yaml(MAP_PATHS[map_id].parent / "metadata.yaml")
        origin = _read_yaml(MAP_PATHS[map_id]).get("origin", [-50.0, -50.0, 0.0]); gates = meta.get("gates", [])
        entry = []
        for idx, gate in enumerate(gates):
            cx, cy = gate["center_world_xy"]
            side = gate["side"]
            direction = {"top": (0.0, 1.0), "bottom": (0.0, -1.0), "left": (-1.0, 0.0), "right": (1.0, 0.0)}[side]
            inner = [cx - direction[0] * 0.55, cy - direction[1] * 0.55, math.atan2(direction[1], direction[0])]
            outer = [cx + direction[0] * 3.0, cy + direction[1] * 3.0, math.atan2(direction[1], direction[0])]
            entry.append({"query_id": f"{map_id}_gate_{idx:02d}_out", "map_id": map_id, "category": "gate_to_outer_free_boundary", "seed": 20260821, "start": inner, "goal": outer})
            entry.append({"query_id": f"{map_id}_gate_{idx:02d}_in", "map_id": map_id, "category": "outer_boundary_to_gate", "seed": 20260821, "start": outer, "goal": inner})
        all_queries.extend(entry)
    return {"schema_version": 1, "query_set_id": "boundary_stress_queries_v1", "dynamic_obstacles": False, "gate_plan_version": "hospital_boundary_gates_v1", "queries": all_queries}


def prepare_inputs(root: Path, map_ids: Sequence[str] = MAP_IDS) -> Path:
    _refuse_nonempty(root); root.mkdir(parents=True, exist_ok=True)
    if not SOURCE_QUERIES.exists(): raise ValueError(f"missing fixed source queries: {SOURCE_QUERIES}")
    source = _read_yaml(SOURCE_QUERIES)
    if len(source.get("queries", [])) != 10: raise ValueError("queries_v2 must contain q00-q09")
    shutil.copy2(SOURCE_QUERIES, root / "core_queries_v1.yaml")
    maps = {map_id: _context(map_id, {}) for map_id in map_ids}
    protocol = _protocol(map_ids, maps, root)
    _write_yaml(root / "protocol.yaml", protocol)
    _write_yaml(root / "boundary_stress_queries_v1.yaml", _make_boundary_queries(root, map_ids))
    rows = []
    for map_id in map_ids:
        ctx = maps[map_id]
        rows.append({"map_id": map_id, "map_yaml": str(MAP_PATHS[map_id]), "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "resolution": ctx.hospital_map.resolution, "width_cells": ctx.hospital_map.width, "height_cells": ctx.hospital_map.height, "physical_width_m": ctx.hospital_map.width * ctx.hospital_map.resolution, "physical_height_m": ctx.hospital_map.height * ctx.hospital_map.resolution, "physical_area_m2": ctx.hospital_map.width * ctx.hospital_map.height * ctx.hospital_map.resolution ** 2, "origin": json.dumps(ctx.hospital_map.origin), "gate_plan_version": ctx.metadata.get("gate_plan_version", ""), "gate_count": ctx.metadata.get("gate_count", 0), "dynamic_obstacles": False})
    pd.DataFrame(rows).to_csv(root / "maps.csv", index=False)
    _write_yaml(root / "manifest.yaml", {"schema_version": 1, "experiment": "pln02_single_planner_ackermann_no_reverse_v1", "created_at": dt.datetime.now(dt.timezone.utc).isoformat(), "dynamic_obstacles": False, "map_ids": list(map_ids), "core_queries": "core_queries_v1.yaml", "boundary_stress_queries": "boundary_stress_queries_v1.yaml", "protocol": "protocol.yaml", "source_queries": str(SOURCE_QUERIES), "gate_plan_version": "hospital_boundary_gates_v1"})
    (root / "README.md").write_text(
        "# PLN-02 single-planner static A2B benchmark\n\n"
        "This directory uses the fixed 0.05 m Hospital core and boundary maps, "
        "the exact world-coordinate `core_queries_v1.yaml`, and a static "
        "Ackermann-surrogate acceptance model. No ROS, Gazebo, dynamic obstacle, "
        "or local-control component is used.\n\n"
        "`astar` is the fixed 8-connected Euclidean grid reference. `hybrid_astar` "
        "is a forward SE(2) Dubins lattice search. The sampling planners are "
        "explicitly named in `protocol.yaml`; they are in-repository reference "
        "implementations because OMPL is unavailable in the runtime.\n\n"
        "Only `run_mode=measured` is used for formal summaries. Planner success, "
        "static footprint validity, kinematic validity, and final validity are "
        "stored as separate fields.\n"
    )
    return root


def validate_inputs(root: Path, map_ids: Sequence[str] = MAP_IDS) -> pd.DataFrame:
    protocol = _read_yaml(root / "protocol.yaml"); queries = _query_objects(root / "core_queries_v1.yaml")
    if protocol.get("dynamic_obstacles", True) or protocol.get("allow_reverse", True) or protocol.get("allow_in_place_rotation", True): raise ValueError("static Ackermann protocol flags are inconsistent")
    rows = []
    for map_id in map_ids:
        ctx = _context(map_id, protocol)
        for query in queries:
            checked = ctx.hospital_map.validate_query(query, FOOTPRINT, 0.5, allow_unknown=False)
            rows.append({"map_id": map_id, "query_id": query.query_id, "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2], "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2], "validation_status": checked.validation_status, "start_status": checked.start_status, "goal_status": checked.goal_status, "connected": checked.connected, "start_clearance_m": checked.start_clearance_m, "goal_clearance_m": checked.goal_clearance_m, "reason": checked.reason, "dynamic_obstacles": False})
    frame = pd.DataFrame(rows); frame.to_csv(root / "query_validation.csv", index=False)
    return frame


def _resource_before() -> Tuple[Optional[Any], float, int]:
    snap = read_snapshot(os.getpid()); cpu = resource.getrusage(resource.RUSAGE_SELF); return snap, float(cpu.ru_utime + cpu.ru_stime), int(cpu.ru_maxrss) * 1024


def _run_records(root: Path, map_ids: Sequence[str], algorithms: Sequence[str], *, repetitions: int, warmups: int, query_ids: Optional[Sequence[str]], boundary: bool = False) -> Tuple[pd.DataFrame, pd.DataFrame]:
    protocol = _read_yaml(root / "protocol.yaml"); query_file = root / ("boundary_stress_queries_v1.yaml" if boundary else "core_queries_v1.yaml")
    payload = _read_yaml(query_file); source_queries = _query_objects(root / "core_queries_v1.yaml")
    queries = []
    for item in payload.get("queries", []):
        if boundary and item.get("map_id") not in map_ids: continue
        if query_ids and str(item["query_id"]) not in set(query_ids): continue
        queries.append(Query(str(item["query_id"]), [float(v) for v in item["start"]], [float(v) for v in item["goal"]], str(item.get("category", "")), int(item.get("seed", 20260821))))
    contexts = {m: _context(m, protocol) for m in map_ids}
    prefix = root / ("boundary_stress" if boundary else Path(".")); prefix.mkdir(parents=True, exist_ok=True)
    runs_path = prefix / "runs.csv"
    metrics_path = prefix / "path_metrics.csv"
    existing_runs: List[Dict[str, Any]] = []
    existing_metrics: List[Dict[str, Any]] = []
    if runs_path.exists():
        old_runs = pd.read_csv(runs_path)
        existing_map_ids = set(old_runs.get("map_id", pd.Series(dtype=str)).astype(str))
        duplicate_maps = existing_map_ids.intersection(set(map_ids))
        if duplicate_maps:
            raise ValueError(f"refusing to overwrite completed map outputs: {sorted(duplicate_maps)}")
        existing_runs = old_runs.to_dict("records")
    if metrics_path.exists():
        existing_metrics = pd.read_csv(metrics_path).to_dict("records")
    validation = pd.read_csv(root / "query_validation.csv") if not boundary else None
    runs: List[Dict[str, Any]] = existing_runs; metrics: List[Dict[str, Any]] = existing_metrics; config = AckermannConfig()
    path_dir = root / ("boundary_stress/paths" if boundary else "paths"); path_dir.mkdir(parents=True, exist_ok=True)
    for map_id in map_ids:
        ctx = contexts[map_id]
        map_queries = [q for q in queries if not boundary or next((i for i in payload["queries"] if i["query_id"] == q.query_id), {}).get("map_id") == map_id]
        for query_index, query in enumerate(map_queries):
            check = None if boundary else validation[(validation.map_id == map_id) & (validation.query_id == query.query_id)].iloc[0]
            for algorithm in algorithms:
                for repetition in range(1, warmups + repetitions + 1):
                    run_mode = "warmup" if repetition <= warmups else "measured"
                    actual_seed = 20260821 + query_index * 100 + repetition if algorithm in ALGORITHMS[2:] else None
                    run_id = f"{map_id}_{query.query_id}_{algorithm}_{run_mode}_{repetition}_{time.time_ns()}"
                    row: Dict[str, Any] = {"run_id": run_id, "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(), "map_id": map_id, "map_sha256": ctx.map_sha256, "query_id": query.query_id, "query_category": query.category, "algorithm": algorithm, "repetition": repetition, "run_mode": run_mode, "seed": actual_seed, "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2], "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2], "planner_success": False, "action_success": False, "static_footprint_valid": False, "kinematic_valid": False, "final_valid_success": False, "failure_code": "", "failure_detail": "", "planning_time_ms": None, "wall_time_ms": None, "cpu_total_ms": None, "cpu_percent": None, "planner_rss_peak_bytes": None, "planner_pss_peak_bytes": None, "stack_rss_peak_bytes": None, "stack_pss_peak_bytes": None, "search_states": 0, "expanded_states": 0, "samples": 0, "rewires": 0, "first_solution_time_ms": None, "path_file": ""}
                    if check is not None and str(check.validation_status) != "VALID":
                        row["failure_code"] = "INVALID_ENDPOINT"; row["failure_detail"] = str(check.reason)
                        runs.append(row); metrics.append({"run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm}); continue
                    before, cpu_before, rss_before = _resource_before(); started = time.monotonic()
                    result = _planner_call(ctx, query, algorithm, config, TIMEOUTS[map_id], actual_seed or 0)
                    wall_ms = (time.monotonic() - started) * 1000.0; _, cpu_after, rss_after = _resource_before(); cpu_ms = max(0.0, (cpu_after - cpu_before) * 1000.0)
                    row.update({"planner_success": result.planner_success, "action_success": result.planner_success, "failure_code": result.failure_code, "failure_detail": result.detail, "planning_time_ms": wall_ms, "wall_time_ms": wall_ms, "cpu_total_ms": cpu_ms, "cpu_percent": cpu_ms / wall_ms * 100.0 if wall_ms else None, "planner_rss_peak_bytes": max(rss_before, rss_after), "planner_pss_peak_bytes": (int(read_snapshot(os.getpid()).pss_bytes) if read_snapshot(os.getpid()) and read_snapshot(os.getpid()).pss_bytes else None), "search_states": result.search_states, "expanded_states": result.expanded_states, "samples": result.samples, "rewires": result.rewires, "first_solution_time_ms": result.first_solution_time_ms, "angle_resolution_deg": result.angle_resolution_deg, "step_size_m": result.step_size_m, "dt_s": result.dt_s})
                    # All algorithms run in this single static benchmark
                    # process.  Stack scope is therefore the same process and
                    # is recorded explicitly rather than fabricated as a
                    # separate Nav2 stack measurement.
                    row["stack_rss_peak_bytes"] = row["planner_rss_peak_bytes"]
                    row["stack_pss_peak_bytes"] = row["planner_pss_peak_bytes"]
                    if result.points:
                        metric = validate_path(ctx, query, result.points, config); row.update({"static_footprint_valid": metric["static_footprint_valid"], "kinematic_valid": metric["kinematic_valid"], "final_valid_success": bool(result.planner_success and metric["static_footprint_valid"] and metric["kinematic_valid"]), "failure_code": row["failure_code"] or metric["failure_code"] or ""})
                        relative = Path("boundary_stress/paths" if boundary else "paths") / f"{run_id}.json.gz"; _save_points(root / relative, result.points); row["path_file"] = str(relative)
                        metric.update({"run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode})
                    else:
                        metric = {"run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode}
                    runs.append(row); metrics.append(metric)
        # Persist after each map so a long large-map run remains auditable if
        # it is interrupted. The final write below is intentionally repeated
        # to include all maps and keep the schema deterministic.
        pd.DataFrame(runs).to_csv(runs_path, index=False)
        pd.DataFrame(metrics).to_csv(metrics_path, index=False)
    run_frame = pd.DataFrame(runs); metric_frame = pd.DataFrame(metrics)
    run_frame.to_csv(runs_path, index=False); metric_frame.to_csv(metrics_path, index=False)
    return run_frame, metric_frame


def _save_points(path: Path, points: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", encoding="utf-8") as stream: json.dump(list(points), stream, separators=(",", ":"))


def _summary(root: Path, runs: pd.DataFrame, metrics: pd.DataFrame, filename_prefix: str = "") -> None:
    if runs.empty: return
    for field in ("stack_rss_peak_bytes", "stack_pss_peak_bytes"):
        planner_field = field.replace("stack_", "planner_")
        if field not in runs and planner_field in runs:
            runs[field] = runs[planner_field]
    merged = runs.merge(metrics, on=["run_id", "map_id", "query_id", "algorithm", "run_mode"], how="left", suffixes=("", "_metric"))
    measured = merged[merged.run_mode == "measured"].copy()
    # A* is the per-query reference, and shortest observed valid excludes static or kinematic failures.
    refs = measured[(measured.algorithm == "astar") & measured.final_valid_success.astype(bool)][["map_id", "query_id", "path_length_m"]].rename(columns={"path_length_m": "a_star_length"}) if "path_length_m" in measured else pd.DataFrame()
    if not refs.empty: measured = measured.merge(refs, on=["map_id", "query_id"], how="left")
    valid_lengths = measured[measured.final_valid_success.astype(bool)].groupby(["map_id", "query_id"])["path_length_m"].min().rename("shortest_observed_valid") if "path_length_m" in measured else pd.Series(dtype=float)
    if len(valid_lengths): measured = measured.join(valid_lengths, on=["map_id", "query_id"])
    if "path_length_m" in measured:
        if "euclidean_distance_m" in measured:
            euclidean = pd.to_numeric(measured["euclidean_distance_m"], errors="coerce")
            measured["length_over_euclidean"] = np.where(
                euclidean > 1e-9,
                pd.to_numeric(measured["path_length_m"], errors="coerce") / euclidean,
                np.nan,
            )
        measured["length_over_a_star"] = measured["path_length_m"] / measured["a_star_length"] if "a_star_length" in measured else np.nan
        measured["length_over_shortest_observed_valid"] = measured["path_length_m"] / measured["shortest_observed_valid"] if "shortest_observed_valid" in measured else np.nan
    measured.to_csv(root / (f"{filename_prefix}measured_enriched.csv" if filename_prefix else "measured_enriched.csv"), index=False)
    rows = []
    for keys, group in measured.groupby(["map_id", "algorithm"]):
        row = {"map_id": keys[0], "algorithm": keys[1], "count": len(group), "planner_success_count": int(group.planner_success.astype(bool).sum()), "static_footprint_valid_count": int(group.static_footprint_valid.astype(bool).sum()), "kinematic_valid_count": int(group.kinematic_valid.astype(bool).sum()), "final_valid_success_count": int(group.final_valid_success.astype(bool).sum()), "planner_success_rate": float(group.planner_success.astype(bool).mean()), "static_footprint_valid_rate": float(group.static_footprint_valid.astype(bool).mean()), "kinematic_valid_rate": float(group.kinematic_valid.astype(bool).mean()), "final_valid_success_rate": float(group.final_valid_success.astype(bool).mean()), "collision_path_count": int((pd.to_numeric(group.get("footprint_collision_count", 0), errors="coerce") > 0).sum())}
        for field in ("planning_time_ms", "wall_time_ms", "cpu_total_ms", "planner_rss_peak_bytes", "planner_pss_peak_bytes", "path_length_m", "length_over_euclidean", "length_over_a_star", "length_over_shortest_observed_valid", "minimum_clearance_m", "curvature_p95_per_m", "maximum_curvature_per_m", "heading_change_p95_rad", "kinematic_invalid_segment_count", "samples", "rewires", "first_solution_time_ms"):
            if field not in group: continue
            eligible = group.planner_success.astype(bool) if field in {"planning_time_ms", "wall_time_ms", "cpu_total_ms", "planner_rss_peak_bytes", "planner_pss_peak_bytes", "path_length_m", "length_over_euclidean", "length_over_a_star", "length_over_shortest_observed_valid", "minimum_clearance_m", "curvature_p95_per_m", "maximum_curvature_per_m", "heading_change_p95_rad", "samples", "rewires", "first_solution_time_ms"} else group.final_valid_success.astype(bool)
            values = pd.to_numeric(group.loc[eligible, field], errors="coerce").dropna()
            row[f"{field}_P50"] = float(values.quantile(.50)) if len(values) else None; row[f"{field}_P95"] = float(values.quantile(.95)) if len(values) else None; row[f"{field}_P99"] = float(values.quantile(.99)) if len(values) else None; row[f"{field}_mean"] = float(values.mean()) if len(values) else None
        rows.append(row)
    pd.DataFrame(rows).to_csv(root / (f"{filename_prefix}summary_by_map.csv" if filename_prefix else "summary_by_map.csv"), index=False)
    algorithm_rows = []
    for algorithm, group in measured.groupby("algorithm"):
        row = {"algorithm": algorithm, "count": len(group), "planner_success_rate": float(group.planner_success.astype(bool).mean()), "static_footprint_valid_rate": float(group.static_footprint_valid.astype(bool).mean()), "kinematic_valid_rate": float(group.kinematic_valid.astype(bool).mean()), "final_valid_success_rate": float(group.final_valid_success.astype(bool).mean())}
        for field in ("planning_time_ms", "wall_time_ms", "cpu_total_ms", "planner_rss_peak_bytes", "planner_pss_peak_bytes", "stack_rss_peak_bytes", "stack_pss_peak_bytes", "path_length_m", "length_over_euclidean", "curvature_p95_per_m", "maximum_curvature_per_m"):
            if field not in group: continue
            values = pd.to_numeric(group.loc[group.planner_success.astype(bool), field], errors="coerce").dropna()
            row[f"{field}_P50"] = float(values.quantile(.5)) if len(values) else None
            row[f"{field}_P95"] = float(values.quantile(.95)) if len(values) else None
            row[f"{field}_P99"] = float(values.quantile(.99)) if len(values) else None
        algorithm_rows.append(row)
    pd.DataFrame(algorithm_rows).to_csv(root / (f"{filename_prefix}summary_by_algorithm.csv" if filename_prefix else "summary_by_algorithm.csv"), index=False)
    byq = measured.groupby(["map_id", "query_id", "algorithm"], dropna=False).agg(count=("run_id", "count"), final_valid_success_rate=("final_valid_success", "mean"), planner_success_rate=("planner_success", "mean"), static_footprint_valid_rate=("static_footprint_valid", "mean"), kinematic_valid_rate=("kinematic_valid", "mean")).reset_index()
    byq.to_csv(root / (f"{filename_prefix}summary_by_query.csv" if filename_prefix else "summary_by_query.csv"), index=False)
    failures = measured[~measured.final_valid_success.astype(bool)].groupby(["map_id", "algorithm", "failure_code"], dropna=False).size().rename("count").reset_index(); failures.to_csv(root / (f"{filename_prefix}failure_summary.csv" if filename_prefix else "failure_summary.csv"), index=False)
    is_boundary = root.name == "boundary_stress"
    _plots(root / ("plots" if not filename_prefix else f"{filename_prefix}plots"), measured, boundary=is_boundary)


def _plots(directory: Path, frame: pd.DataFrame, *, boundary: bool = False) -> None:
    """Write scale and path-quality figures from measured records.

    The raw run tables remain the source of truth.  Plot aggregates use
    measured rows only; timing/resource curves retain planner attempts while
    path-quality distributions use rows for which a planner actually returned
    a path.  This keeps failed searches visible in success/failure summaries
    without treating missing paths as zero-quality paths.
    """
    directory.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    except Exception: return
    frame = frame.copy()
    frame["scale_m"] = frame["map_id"].map(MAP_SCALE_M)
    x_maps = [m for m in MAP_IDS if m in set(frame["map_id"].dropna())]
    x = [MAP_SCALE_M[m] for m in x_maps]
    algorithms = list(dict.fromkeys(frame["algorithm"].dropna().tolist()))
    percentile_styles = [(0.50, "P50", "-"), (0.95, "P95", "--"), (0.99, "P99", ":")]

    def scale_percentiles(field: str, title: str, ylabel: str, filename: str, *, successes_only: bool = False) -> None:
        if field not in frame: return
        source = frame[frame["planner_success"].astype(bool)] if successes_only and "planner_success" in frame else frame
        fig, ax = plt.subplots(figsize=(9, 5))
        plotted = False
        for algorithm in algorithms:
            group = source[source["algorithm"] == algorithm]
            for quantile, label, linestyle in percentile_styles:
                values = pd.to_numeric(group[field], errors="coerce").groupby(group["map_id"]).quantile(quantile).reindex(x_maps)
                if values.notna().any():
                    ax.plot(x, values.to_numpy(dtype=float), marker="o", linestyle=linestyle,
                            label=f"{ALGORITHM_LABEL.get(algorithm, algorithm)} {label}")
                    plotted = True
        ax.set_xlabel("Map side length (m)"); ax.set_ylabel(ylabel); ax.set_title(title)
        ax.set_xticks(x, [MAP_SCALE_LABEL[m] for m in x_maps], rotation=20)
        ax.grid(alpha=.25)
        if plotted: ax.legend(fontsize=8, ncol=2)
        else: ax.text(.5, .5, "No numeric records", ha="center", va="center", transform=ax.transAxes)
        fig.tight_layout(); fig.savefig(directory / filename, dpi=140); plt.close(fig)

    scale_percentiles("wall_time_ms", "Wall time vs map scale", "Wall time (ms)", "wall_time_vs_scale.png")
    scale_percentiles("planner_rss_peak_bytes", "Planner RSS vs map scale", "Peak RSS (bytes)", "memory_vs_scale.png")
    scale_percentiles("planner_pss_peak_bytes", "Planner PSS vs map scale", "Peak PSS (bytes)", "pss_memory_vs_scale.png")
    scale_percentiles("cpu_total_ms", "CPU time vs map scale", "CPU time (ms)", "cpu_vs_scale.png")

    # Keep planner/action/static/final success distinct.  In particular, a
    # planner-returned path that fails the Ackermann/static checks is not a
    # final success.
    fig, ax = plt.subplots(figsize=(9, 5)); plotted = False
    for algorithm in algorithms:
        group = frame[frame["algorithm"] == algorithm]
        for field, label, linestyle in (("planner_success", "planner", "-"),
                                         ("static_footprint_valid", "static", "--"),
                                         ("final_valid_success", "final", ":")):
            if field not in group: continue
            values = group.groupby("map_id")[field].mean().reindex(x_maps)
            if values.notna().any():
                ax.plot(x, values.to_numpy(dtype=float), marker="o", linestyle=linestyle,
                        label=f"{ALGORITHM_LABEL.get(algorithm, algorithm)} {label}"); plotted = True
    ax.set_xlabel("Map side length (m)"); ax.set_ylabel("Rate")
    ax.set_ylim(-0.05, 1.05); ax.set_title("Planner, static-valid and final success vs map scale")
    ax.set_xticks(x, [MAP_SCALE_LABEL[m] for m in x_maps], rotation=20); ax.grid(alpha=.25)
    if plotted: ax.legend(fontsize=8, ncol=2)
    fig.tight_layout(); fig.savefig(directory / "success_vs_scale.png", dpi=140); plt.close(fig)

    scale_percentiles("path_length_m", "Returned path length vs map scale", "Path length (m)",
                      "path_length_vs_scale.png", successes_only=True)
    scale_percentiles("length_over_euclidean", "Path length ratio vs map scale", "Length / Euclidean distance",
                      "path_length_ratio_vs_scale.png", successes_only=True)

    # A compact multi-panel quality view.  Empty algorithms remain visible as
    # labelled panels rather than silently disappearing from the comparison.
    quality_fields = (("path_length_m", "Path length (m)"),
                      ("length_over_euclidean", "Length / Euclidean"),
                      ("minimum_clearance_m", "Minimum clearance (m)"),
                      ("curvature_p95_per_m", "Curvature P95 (1/m)"),
                      ("heading_change_p95_rad", "Heading change P95 (rad)"))
    quality = frame[frame["planner_success"].astype(bool)] if "planner_success" in frame else frame
    fig, axes = plt.subplots(2, 3, figsize=(14, 8)); axes = axes.ravel()
    for idx, (field, title) in enumerate(quality_fields):
        ax = axes[idx]; data = []; labels = []; valid_counts = []
        for algorithm in algorithms:
            values = pd.to_numeric(quality.loc[quality["algorithm"] == algorithm, field], errors="coerce").dropna() if field in quality else pd.Series(dtype=float)
            if len(values):
                data.append(values.to_numpy()); labels.append(ALGORITHM_LABEL.get(algorithm, algorithm)); valid_counts.append(len(values))
        if data:
            ax.boxplot(data, tick_labels=labels, showmeans=True)
            ax.tick_params(axis="x", rotation=25, labelsize=8)
            ax.set_title(f"{title}\nplanner-success paths (n={sum(valid_counts)})")
        else:
            ax.text(.5, .5, "No planner-success paths", ha="center", va="center", transform=ax.transAxes)
            ax.set_title(title)
        ax.grid(alpha=.2)
    axes[-1].axis("off")
    fig.suptitle("Four-algorithm path quality comparison", y=.995)
    fig.tight_layout(); fig.savefig(directory / "algorithm_path_quality.png", dpi=140); plt.close(fig)

    if boundary:
        # Boundary stress is intentionally a separate experiment.  Use bars so
        # the single stress-map scale is not mistaken for a four-scale curve.
        rates = []
        timing = []
        for algorithm in algorithms:
            group = frame[frame["algorithm"] == algorithm]
            rates.append({"algorithm": ALGORITHM_LABEL.get(algorithm, algorithm),
                          "planner": group["planner_success"].astype(bool).mean() if "planner_success" in group else np.nan,
                          "static": group["static_footprint_valid"].astype(bool).mean() if "static_footprint_valid" in group else np.nan,
                          "final": group["final_valid_success"].astype(bool).mean() if "final_valid_success" in group else np.nan})
            vals = pd.to_numeric(group.get("wall_time_ms", pd.Series(dtype=float)), errors="coerce").dropna()
            timing.append({"algorithm": ALGORITHM_LABEL.get(algorithm, algorithm),
                           "P50": vals.quantile(.50) if len(vals) else np.nan,
                           "P95": vals.quantile(.95) if len(vals) else np.nan,
                           "P99": vals.quantile(.99) if len(vals) else np.nan})
        rates_df = pd.DataFrame(rates).set_index("algorithm")
        ax = rates_df.plot(kind="bar", figsize=(10, 5), ylim=(0, 1.05), rot=25)
        ax.set_title("Boundary stress success rates (separate experiment)"); ax.set_ylabel("Rate"); ax.grid(axis="y", alpha=.25)
        fig = ax.get_figure(); fig.tight_layout(); fig.savefig(directory / "boundary_stress_success.png", dpi=140); plt.close(fig)
        timing_df = pd.DataFrame(timing).set_index("algorithm")
        ax = timing_df.plot(kind="bar", figsize=(10, 5), rot=25)
        ax.set_title("Boundary stress wall time"); ax.set_ylabel("Wall time (ms)"); ax.grid(axis="y", alpha=.25)
        fig = ax.get_figure(); fig.tight_layout(); fig.savefig(directory / "boundary_stress_timing.png", dpi=140); plt.close(fig)


def run(root: Path, map_ids: Sequence[str], algorithms: Sequence[str], *, repetitions: int, warmups: int, query_ids: Optional[Sequence[str]] = None, boundary: bool = False) -> None:
    runs, metrics = _run_records(root, map_ids, algorithms, repetitions=repetitions, warmups=warmups, query_ids=query_ids, boundary=boundary)
    _summary(root / ("boundary_stress" if boundary else Path(".")), runs, metrics)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Static PLN-02 Ackermann/no-reverse four-algorithm benchmark")
    parser.add_argument("--root", default=str(ROOT / "experiments/single_planner_benchmark/ackermann_no_reverse_v1"))
    parser.add_argument("--stage", choices=["prepare", "validate", "run", "report", "boundary", "all"], default="prepare")
    parser.add_argument("--map-id", action="append", dest="map_ids", choices=list(MAP_IDS))
    parser.add_argument("--algorithm", action="append", dest="algorithms", choices=list(ALGORITHMS))
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--repetitions", type=int, default=5); parser.add_argument("--warmups", type=int, default=3)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv); root = Path(args.root).resolve(); maps = args.map_ids or list(MAP_IDS); algorithms = args.algorithms or list(ALGORITHMS)
    try:
        if args.stage in {"prepare", "all"}: prepare_inputs(root, maps)
        if args.stage in {"validate", "all"}: validate_inputs(root, maps)
        if args.stage in {"run", "all"}: run(root, maps, algorithms, repetitions=args.repetitions, warmups=args.warmups, query_ids=args.query_ids)
        if args.stage == "boundary": run(root, [m for m in maps if m != "hospital_005"], algorithms, repetitions=args.repetitions, warmups=args.warmups, query_ids=args.query_ids, boundary=True)
        if args.stage == "report":
            _summary(root, pd.read_csv(root / "runs.csv"), pd.read_csv(root / "path_metrics.csv"))
        print(f"single planner benchmark {args.stage} output: {root}"); return 0
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        print(f"single_planner_benchmark: ERROR: {exc}", file=os.sys.stderr); return 2


if __name__ == "__main__": raise SystemExit(main())
