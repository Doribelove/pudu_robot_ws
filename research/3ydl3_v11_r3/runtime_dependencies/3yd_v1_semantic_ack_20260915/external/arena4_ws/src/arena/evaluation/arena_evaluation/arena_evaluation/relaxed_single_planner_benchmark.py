"""PLN-02 relaxed Ackermann-surrogate four-planner benchmark.

This module is intentionally a new versioned entry point.  It reuses the
static map, footprint, path validation, A* and reference sampling utilities
from :mod:`single_planner_benchmark`, but never writes to the frozen v1
directory.  The relaxed vehicle model is an experiment abstraction (60 degree
steering, forward-only), not a claim about the mechanical Jackal platform.

The Hybrid A* implementation here is a small deterministic forward lattice
used for an offline benchmark.  It records its discretisation and goal
connection diagnostics so a run cannot silently use the old v1 settings.
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
import shutil
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
import yaml

from . import single_planner_benchmark as v1
from .planner_benchmark.isolation import run_isolated
from .planner_benchmark.provenance import write_code_manifest
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .topology import astar_grid, preprocess_static_map


ROOT = Path("/home/robot/pudu_robot_ws")
OUTPUT_NAME = "ackermann_no_reverse_relaxed_60deg_v2"
SOURCE_QUERIES = ROOT / "experiments/planner_benchmark/hospital_005/queries_v2.yaml"
MAP_IDS = tuple(v1.MAP_IDS)
MAP_PATHS = dict(v1.MAP_PATHS)
TIMEOUTS = dict(v1.TIMEOUTS)
FOOTPRINT = [list(point) for point in v1.FOOTPRINT]
ALGORITHMS = tuple(v1.ALGORITHMS)
SAMPLE_INTERVAL_MS = 5.0
MIN_ENDPOINT_CLEARANCE_M = 0.5
# Keep enough room for fork scheduling and the result/resource handshake on
# large maps.  The external protocol timeout remains the hard request budget;
# this internal guard returns a structured planner TIMEOUT before the parent
# watchdog has to terminate the child.
PLANNER_DEADLINE_MARGIN_MS = 300.0


def _as_bool(value: Any) -> bool:
    """Parse CSV/YAML booleans without treating ``"False"`` as true."""
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "y", "t"}
# Captured once at import and copied into every run row so timeout audits can
# identify the exact benchmark implementation even when ``external/`` is
# ignored by Git.
SOURCE_CODE_HASH = sha256_file(Path(__file__))


def _derived_radius(wheelbase_m: float, steering_deg: float) -> Tuple[float, float]:
    radius = float(wheelbase_m) / math.tan(math.radians(float(steering_deg)))
    return radius, 1.0 / radius


@dataclass(frozen=True)
class RelaxedAckermannConfig:
    """Fixed v2 vehicle/search protocol.

    ``minimum_turning_radius_m`` and ``maximum_curvature_per_m`` are derived
    values.  Keeping both in the dataclass makes protocol/runtime consistency
    checks straightforward and prevents a hand-edited mismatch.
    """

    wheelbase_m: float = 0.50
    max_steering_angle_deg: float = 60.0
    minimum_turning_radius_m: float = 0.2886751345948129
    maximum_curvature_per_m: float = 3.4641016151377544
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
    hybrid_step_size_m: float = 0.25
    angle_resolution_deg: float = 5.0
    angle_bins: int = 72
    steering_angles_deg: Tuple[float, ...] = (-60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0)
    integration_sample_spacing_m: float = 0.05
    state_limit: Optional[int] = None
    goal_connection_enabled: bool = True
    goal_connection_radius_m: float = 2.0

    def __post_init__(self) -> None:
        radius, curvature = _derived_radius(self.wheelbase_m, self.max_steering_angle_deg)
        if not math.isclose(self.minimum_turning_radius_m, radius, rel_tol=1e-8, abs_tol=1e-8):
            raise ValueError("minimum_turning_radius_m is inconsistent with wheelbase/steering")
        if not math.isclose(self.maximum_curvature_per_m, curvature, rel_tol=1e-8, abs_tol=1e-8):
            raise ValueError("maximum_curvature_per_m is inconsistent with wheelbase/steering")
        if self.allow_reverse or self.allow_in_place_rotation:
            raise ValueError("v2 is forward-only and forbids in-place rotation")
        if self.angle_bins != round(360.0 / self.angle_resolution_deg):
            raise ValueError("angle_bins must match angle_resolution_deg")
        if self.integration_sample_spacing_m > 0.05 + 1e-12:
            raise ValueError("integration sampling must be no coarser than 0.05 m")

    @property
    def max_steering_angle_rad(self) -> float:
        return math.radians(self.max_steering_angle_deg)

    def state_limit_for(self, ctx: "MapContext", timeout_s: float) -> int:
        """Return a timeout-scaled limit, never the legacy fixed 120000 cap."""
        if self.state_limit is not None:
            return int(self.state_limit)
        # The deadline remains authoritative.  This guard prevents accidental
        # unbounded memory use while scaling with the configured map timeout.
        return max(250_000, int(max(1.0, float(timeout_s)) * 250_000))


# Re-export the pure utility types expected by existing benchmark tooling.
MapContext = v1.MapContext
PlannerResult = v1.PlannerResult
_wrap = v1._wrap
_delta = v1._delta
_integrate_bicycle = v1._integrate_bicycle
_resample = v1._resample


def validate_path(ctx: MapContext, query: Query, points: Optional[Sequence[Dict[str, float]]], config: RelaxedAckermannConfig) -> Dict[str, Any]:
    """Validate a returned path with the v2 hard curvature threshold.

    The shared v1 checker remains the source for footprint, endpoint,
    direction and sampling metrics.  Its historical ``+0.03`` curvature
    allowance is intentionally removed here: v2's derived maximum curvature
    is a hard acceptance boundary, not a tuning knob.
    """
    metrics = v1.validate_path(ctx, query, points, config)
    if points and float(metrics.get("maximum_curvature_per_m") or 0.0) > float(config.maximum_curvature_per_m) + 1e-9:
        codes = []
        try:
            codes = list(json.loads(metrics.get("failure_codes", "[]")))
        except (TypeError, ValueError, json.JSONDecodeError):
            codes = [metrics.get("failure_code", "")] if metrics.get("failure_code") else []
        if "MINIMUM_TURNING_RADIUS_VIOLATION" not in codes:
            codes.append("MINIMUM_TURNING_RADIUS_VIOLATION")
        metrics["kinematic_valid"] = False
        metrics["kinematic_invalid_segment_count"] = max(1, int(metrics.get("kinematic_invalid_segment_count") or 0))
        # Keep the original primary diagnostic (for example
        # ``REVERSE_MOTION`` or ``STATIC_FOOTPRINT_COLLISION``) and expose the
        # hard-curvature violation in the structured list.  Replacing the
        # primary code would hide simultaneous failure causes.
        if not metrics.get("failure_code"):
            metrics["failure_code"] = "MINIMUM_TURNING_RADIUS_VIOLATION"
        metrics["failure_codes"] = json.dumps(codes)
    return metrics


def relaxed_config() -> RelaxedAckermannConfig:
    return RelaxedAckermannConfig()


def _read_yaml(path: Path) -> Dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def _write_yaml(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(dict(value), sort_keys=False), encoding="utf-8")


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _queries(path: Path) -> List[Query]:
    payload = _read_yaml(path)
    result: List[Query] = []
    for item in payload.get("queries", []):
        result.append(Query(
            query_id=str(item["query_id"]),
            start=[float(value) for value in item["start"]],
            goal=[float(value) for value in item["goal"]],
            category=str(item.get("category", "unspecified")),
            seed=int(item.get("seed", payload.get("seed", 20260821))),
            validation_status=str(item.get("validation_status", "UNVALIDATED")),
        ))
    return result


def _context(map_id: str) -> MapContext:
    if map_id not in MAP_PATHS:
        raise ValueError(f"unknown map id: {map_id}")
    hospital_map = HospitalMap.load(MAP_PATHS[map_id])
    if not math.isclose(hospital_map.resolution, 0.05, abs_tol=1e-12):
        raise ValueError(f"{map_id}: map resolution must be 0.05 m")
    metadata_path = MAP_PATHS[map_id].parent / "metadata.yaml"
    metadata = _read_yaml(metadata_path) if metadata_path.exists() else {}
    if map_id != "hospital_005":
        if metadata.get("gate_plan_version") != "hospital_boundary_gates_v1":
            raise ValueError(f"{map_id}: boundary gate metadata is missing")
        if int(metadata.get("gate_count", 0)) != 10 or not math.isclose(float(metadata.get("gate_width_m", 0.0)), 1.0, abs_tol=1e-9):
            raise ValueError(f"{map_id}: expected ten fixed 1.0 m gates")
        if not metadata.get("outer_free_region_connected_to_source_query_space", False):
            raise ValueError(f"{map_id}: outer free region is not connected")
        if not metadata.get("source_unchanged_outside_gates", False):
            raise ValueError(f"{map_id}: source core was not preserved outside gates")
    preprocess_started = time.monotonic_ns()
    _, free, distance, _ = preprocess_static_map(
        hospital_map, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    context = MapContext(
        map_id=map_id,
        hospital_map=hospital_map,
        free_mask=free,
        distance_m=distance,
        map_sha256=sha256_file(hospital_map.image_path),
        map_yaml_sha256=sha256_file(hospital_map.yaml_path),
        metadata=metadata,
    )
    # MapContext is intentionally reused from the v1 pure-data type.  This
    # non-invasive attribute records map preparation separately from request
    # planning without changing the frozen v1 schema.
    setattr(context, "preprocess_wall_time_ms", (time.monotonic_ns() - preprocess_started) / 1e6)
    return context


def _pose_from_cell(hospital_map: HospitalMap, cell: Tuple[int, int], yaw: float) -> Dict[str, float]:
    x, y = hospital_map.cell_to_world(cell)
    return {"x": float(x), "y": float(y), "yaw": float(_wrap(yaw))}


def _astar_relaxed(ctx: MapContext, query: Query, timeout_s: float) -> PlannerResult:
    """A* wrapper preserving explicit deadline/timeout diagnostics."""
    start = ctx.hospital_map.world_to_cell(query.start[0], query.start[1])
    goal = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start is None or goal is None or not ctx.free_mask[start] or not ctx.free_mask[goal]:
        return PlannerResult(False, None, "INVALID_ENDPOINT", configured_timeout_s=timeout_s)
    result = astar_grid(
        ctx.free_mask, start, goal, resolution=ctx.hospital_map.resolution,
        return_stats=True, timeout_s=float(timeout_s),
    )
    base = dict(
        configured_timeout_s=float(timeout_s), timeout_triggered=bool(result.timeout_triggered),
        expanded_states=int(result.expanded_nodes), search_states=int(result.generated_nodes),
        diagnostics={
            "max_open_set_size": int(result.max_open_set_size),
            "allowed_grid_cells": int(result.allowed_grid_cells),
            "total_free_grid_cells": int(result.total_free_grid_cells),
            "search_space_ratio": float(result.search_space_ratio),
            "path_cost": result.path_cost,
            "search_time_ms": float(result.search_time_ms),
        },
    )
    if result.path is None:
        return PlannerResult(False, None, result.failure_code or "NO_PATH", **base)
    points: List[Dict[str, float]] = []
    for index, cell in enumerate(result.path):
        if index == 0:
            yaw = query.start[2]
        elif index == len(result.path) - 1:
            yaw = query.goal[2]
        else:
            x0, y0 = ctx.hospital_map.cell_to_world(result.path[index - 1])
            x1, y1 = ctx.hospital_map.cell_to_world(result.path[index + 1])
            yaw = math.atan2(y1 - y0, x1 - x0)
        points.append(_pose_from_cell(ctx.hospital_map, cell, yaw))
    return PlannerResult(True, points, "", step_size_m=ctx.hospital_map.resolution, **base)


def _free_pose(ctx: MapContext, pose: Tuple[float, float, float]) -> bool:
    cell = ctx.hospital_map.world_to_cell(pose[0], pose[1])
    # ``free_mask`` is produced by orientation-conservative dilation of the
    # complete footprint plus padding and safety margin.  Reusing it in the
    # inner lattice loop is equivalent to a static footprint collision check
    # but avoids repeatedly rasterizing a polygon for every 5 cm sample.  The
    # returned path still receives the oriented full-footprint validator.
    return cell is not None and bool(ctx.free_mask[cell])


def _heading_bin(yaw: float, bins: int) -> int:
    return int(round((_wrap(yaw) + math.pi) / (2.0 * math.pi) * bins)) % bins


def _mod2pi(angle: float) -> float:
    return float(angle) % (2.0 * math.pi)


def _dubins_word(start: Tuple[float, float, float], goal: Tuple[float, float, float], radius: float) -> Optional[Tuple[float, Tuple[str, str, str], Tuple[float, float, float]]]:
    """Return the shortest normalized Dubins word and segment parameters.

    This is the standard six-word Dubins construction.  Parameters are in
    normalized arc lengths; multiplying by ``radius`` gives metres.  It is
    kept local to the benchmark because the Arena runtime does not ship the
    optional ``dubins`` Python package.
    """
    x0, y0, yaw0 = start
    x1, y1, yaw1 = goal
    dx, dy = (x1 - x0) / radius, (y1 - y0) / radius
    distance = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    alpha = _mod2pi(yaw0 - theta)
    beta = _mod2pi(yaw1 - theta)
    d = distance
    candidates: List[Tuple[float, Tuple[str, str, str], Tuple[float, float, float]]] = []

    def add(word: Tuple[str, str, str], values: Optional[Tuple[float, float, float]]) -> None:
        if values is None:
            return
        if all(math.isfinite(value) and value >= -1e-9 for value in values):
            candidates.append((sum(values), word, values))

    # LSL
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p2 = 2.0 + d * d - 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))
    if p2 >= 0.0:
        p = math.sqrt(p2); tmp1 = math.atan2(math.cos(beta) - math.cos(alpha), tmp0)
        add(("L", "S", "L"), (_mod2pi(-alpha + tmp1), p, _mod2pi(beta - tmp1)))
    # RSR
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p2 = 2.0 + d * d - 2.0 * math.cos(alpha - beta) + 2.0 * d * (-math.sin(alpha) + math.sin(beta))
    if p2 >= 0.0:
        p = math.sqrt(p2); tmp1 = math.atan2(math.cos(alpha) - math.cos(beta), tmp0)
        add(("R", "S", "R"), (_mod2pi(alpha - tmp1), p, _mod2pi(-beta + tmp1)))
    # LSR
    p2 = -2.0 + d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) + math.sin(beta))
    if p2 >= 0.0:
        p = math.sqrt(p2); tmp2 = math.atan2(-math.cos(alpha) - math.cos(beta), d + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
        add(("L", "S", "R"), (_mod2pi(-alpha + tmp2), p, _mod2pi(-_mod2pi(beta) + tmp2)))
    # RSL
    p2 = -2.0 + d * d + 2.0 * math.cos(alpha - beta) - 2.0 * d * (math.sin(alpha) + math.sin(beta))
    if p2 >= 0.0:
        p = math.sqrt(p2); tmp2 = math.atan2(math.cos(alpha) + math.cos(beta), d - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
        add(("R", "S", "L"), (_mod2pi(alpha - tmp2), p, _mod2pi(beta - tmp2)))
    # RLR
    tmp0 = (6.0 - d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(alpha) - math.sin(beta))) / 8.0
    if abs(tmp0) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp0))))
        tmp1 = math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta))
        add(("R", "L", "R"), (_mod2pi(alpha - tmp1 + p / 2.0), p, _mod2pi(alpha - beta - _mod2pi(alpha - tmp1 + p / 2.0) + p)))
    # LRL
    tmp0 = (6.0 - d * d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (-math.sin(alpha) + math.sin(beta))) / 8.0
    if abs(tmp0) <= 1.0:
        p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp0))))
        tmp1 = math.atan2(math.cos(alpha) - math.cos(beta), d + math.sin(alpha) - math.sin(beta))
        add(("L", "R", "L"), (_mod2pi(-alpha - tmp1 + p / 2.0), p, _mod2pi(beta - alpha - _mod2pi(-alpha - tmp1 + p / 2.0) + p)))
    return min(candidates, key=lambda item: item[0]) if candidates else None


def _sample_dubins(start: Tuple[float, float, float], goal: Tuple[float, float, float], radius: float, word: Tuple[str, str, str], params: Tuple[float, float, float], spacing_m: float, deadline: Optional[float] = None) -> Optional[List[Tuple[float, float, float]]]:
    state = tuple(float(value) for value in start)
    points: List[Tuple[float, float, float]] = []
    for kind, normalized_length in zip(word, params):
        length_m = float(normalized_length) * radius
        if length_m <= 1e-9:
            continue
        steering = 0.0 if kind == "S" else (math.radians(60.0) if kind == "L" else -math.radians(60.0))
        remaining = length_m
        while remaining > 1e-9:
            if deadline is not None and time.monotonic() >= deadline:
                return None
            distance = min(spacing_m, remaining)
            segment = v1._integrate_bicycle(state[0], state[1], state[2], steering, distance, 0.50, samples=1)
            state = segment[-1]
            points.append(state)
            remaining -= distance
    if not points:
        return None
    # Numerical integration should land at the requested pose within a few
    # millimetres; retain the exact target only after the caller's tolerance
    # and collision checks have passed.
    return points


def _goal_connection(
    ctx: MapContext,
    state: Tuple[float, float, float],
    query: Query,
    config: RelaxedAckermannConfig,
    deadline: Optional[float] = None,
) -> Tuple[Optional[List[Dict[str, float]]], str]:
    """Try short forward Dubins-style connections with 5 cm collision samples.

    A full Dubins library is not available in the Arena runtime.  The
    deterministic straight/constant-curvature primitives below are the same
    forward bicycle controls used by the lattice and are deliberately reported
    as a surrogate connector in diagnostics.
    """
    if deadline is not None and time.monotonic() >= deadline:
        return None, "DEADLINE"
    x, y, yaw = state
    dx, dy = query.goal[0] - x, query.goal[1] - y
    distance = math.hypot(dx, dy)
    if distance > config.goal_connection_radius_m + config.endpoint_position_tolerance_m:
        return None, "OUTSIDE_CONNECTOR_RADIUS"
    # Try the analytic shortest forward Dubins word first.  Its sampled path
    # is still checked point-by-point below, so the connector cannot bypass
    # static collision or endpoint validation.
    dubins = _dubins_word((x, y, yaw), tuple(query.goal), config.minimum_turning_radius_m)
    if dubins is not None:
        _, word, params = dubins
        sampled = _sample_dubins((x, y, yaw), tuple(query.goal), config.minimum_turning_radius_m, word, params, config.integration_sample_spacing_m, deadline)
        if sampled and all(_free_pose(ctx, pose) for pose in sampled):
            end_x, end_y, end_yaw = sampled[-1]
            if math.hypot(end_x - query.goal[0], end_y - query.goal[1]) <= config.endpoint_position_tolerance_m and abs(_delta(end_yaw, query.goal[2])) <= config.endpoint_yaw_tolerance_rad:
                return [{"x": float(px), "y": float(py), "yaw": _wrap(float(pyaw))} for px, py, pyaw in sampled], "DUBINS"
    line_yaw = math.atan2(dy, dx) if distance > 1e-9 else yaw
    if distance > 1e-9 and abs(_delta(line_yaw, yaw)) <= math.radians(7.5) and abs(_delta(query.goal[2], line_yaw)) <= config.endpoint_yaw_tolerance_rad:
        count = max(1, int(math.ceil(distance / config.integration_sample_spacing_m)))
        points = []
        for index in range(1, count + 1):
            if deadline is not None and time.monotonic() >= deadline:
                return None, "DEADLINE"
            fraction = index / count
            pose = (x + fraction * dx, y + fraction * dy, line_yaw if index < count else query.goal[2])
            if not _free_pose(ctx, pose):
                return None, "STATIC_COLLISION"
            points.append({"x": pose[0], "y": pose[1], "yaw": _wrap(pose[2])})
        return points, "STRAIGHT"
    controls = [math.radians(value) for value in config.steering_angles_deg]
    max_length = max(config.goal_connection_radius_m, distance * 2.5) + config.hybrid_step_size_m
    lengths = np.arange(config.integration_sample_spacing_m, max_length + 1e-9, config.integration_sample_spacing_m)
    for steering in controls:
        for length in lengths:
            if deadline is not None and time.monotonic() >= deadline:
                return None, "DEADLINE"
            segment = v1._integrate_bicycle(x, y, yaw, steering, float(length), config.wheelbase_m, samples=max(1, int(math.ceil(float(length) / config.integration_sample_spacing_m))))
            if not segment:
                continue
            collision = False
            for pose in segment:
                if deadline is not None and time.monotonic() >= deadline:
                    return None, "DEADLINE"
                if not _free_pose(ctx, pose):
                    collision = True
                    break
            if collision:
                continue
            end_x, end_y, end_yaw = segment[-1]
            if math.hypot(end_x - query.goal[0], end_y - query.goal[1]) <= config.endpoint_position_tolerance_m and abs(_delta(end_yaw, query.goal[2])) <= config.endpoint_yaw_tolerance_rad:
                return [{"x": float(px), "y": float(py), "yaw": _wrap(float(pyaw))} for px, py, pyaw in segment], "CONSTANT_CURVATURE"
    return None, "NO_FORWARD_DUBINS_CONNECTOR"


def _hybrid_astar(ctx: MapContext, query: Query, config: RelaxedAckermannConfig, timeout_s: float) -> PlannerResult:
    """Forward-only 72-heading, nine-control Hybrid A* with deadline checks."""
    started = time.monotonic()
    deadline = started + max(0.0, float(timeout_s))
    step = float(config.hybrid_step_size_m)
    bins = int(config.angle_bins)
    controls = tuple(math.radians(value) for value in config.steering_angles_deg)
    limit = config.state_limit_for(ctx, timeout_s)
    diagnostics: Dict[str, Any] = {
        "angle_resolution_deg": config.angle_resolution_deg,
        "angle_bins": bins,
        "step_size_m": step,
        "integration_sample_spacing_m": config.integration_sample_spacing_m,
        "steering_angles_deg": list(config.steering_angles_deg),
        "state_limit": limit,
        "goal_connection_attempts": 0,
        "goal_connection_successes": 0,
        "goal_connection_failure_reasons": {},
        "rejected_collision": 0,
        "rejected_out_of_bounds": 0,
        "rejected_discretization": 0,
        "rejected_duplicate": 0,
        "first_layer_successors": 0,
    }
    start_cell = ctx.hospital_map.world_to_cell(query.start[0], query.start[1])
    goal_cell = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start_cell is None or goal_cell is None or not ctx.free_mask[start_cell] or not ctx.free_mask[goal_cell] or not _free_pose(ctx, tuple(query.start)) or not _free_pose(ctx, tuple(query.goal)):
        return PlannerResult(False, None, "INVALID_ENDPOINT", configured_timeout_s=timeout_s, state_limit=limit, diagnostics=diagnostics, angle_resolution_deg=config.angle_resolution_deg, step_size_m=step)

    def key(x: float, y: float, yaw: float) -> Tuple[int, int, int]:
        cell = ctx.hospital_map.world_to_cell(x, y)
        if cell is None:
            return (-1, -1, 0)
        return cell[0], cell[1], _heading_bin(yaw, bins)

    start_state = (float(query.start[0]), float(query.start[1]), float(query.start[2]))
    start_key = key(*start_state)
    queue: List[Tuple[float, float, Tuple[int, int, int]]] = [(math.hypot(query.goal[0] - start_state[0], query.goal[1] - start_state[1]), 0.0, start_key)]
    state_for: Dict[Tuple[int, int, int], Tuple[float, float, float]] = {start_key: start_state}
    parent: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {start_key: None}
    cost: Dict[Tuple[int, int, int], float] = {start_key: 0.0}
    expanded = 0
    generated = 1
    goal_key: Optional[Tuple[int, int, int]] = None
    goal_tail: Optional[List[Dict[str, float]]] = None
    while queue:
        if time.monotonic() >= deadline:
            diagnostics["deadline_elapsed_ms"] = (time.monotonic() - started) * 1000.0
            return PlannerResult(False, None, "TIMEOUT", expanded_states=expanded, search_states=generated, configured_timeout_s=timeout_s, timeout_triggered=True, state_limit=limit, diagnostics=diagnostics, angle_resolution_deg=config.angle_resolution_deg, step_size_m=step)
        _, current_cost, current_key = heapq.heappop(queue)
        if current_cost != cost.get(current_key):
            continue
        expanded += 1
        x, y, yaw = state_for[current_key]
        distance_to_goal = math.hypot(query.goal[0] - x, query.goal[1] - y)
        if config.goal_connection_enabled and distance_to_goal <= config.goal_connection_radius_m:
            diagnostics["goal_connection_attempts"] += 1
            tail, reason = _goal_connection(ctx, (x, y, yaw), query, config, deadline)
            if reason == "DEADLINE":
                diagnostics["goal_connection_failure_reasons"][reason] = int(
                    diagnostics["goal_connection_failure_reasons"].get(reason, 0)
                ) + 1
                return PlannerResult(False, None, "TIMEOUT", expanded_states=expanded, search_states=generated,
                                     configured_timeout_s=timeout_s, timeout_triggered=True, state_limit=limit,
                                     diagnostics=diagnostics, angle_resolution_deg=config.angle_resolution_deg,
                                     step_size_m=step)
            if tail is not None:
                diagnostics["goal_connection_successes"] += 1
                goal_key = current_key
                goal_tail = tail
                break
            failures = diagnostics["goal_connection_failure_reasons"]
            failures[reason] = int(failures.get(reason, 0)) + 1
        # A connector or collision check can consume the remaining deadline;
        # never accept an endpoint that was reached after the configured limit.
        if time.monotonic() >= deadline:
            diagnostics["deadline_elapsed_ms"] = (time.monotonic() - started) * 1000.0
            return PlannerResult(False, None, "TIMEOUT", expanded_states=expanded, search_states=generated,
                                 configured_timeout_s=timeout_s, timeout_triggered=True, state_limit=limit,
                                 diagnostics=diagnostics, angle_resolution_deg=config.angle_resolution_deg,
                                 step_size_m=step)
        if distance_to_goal <= config.endpoint_position_tolerance_m and abs(_delta(yaw, query.goal[2])) <= config.endpoint_yaw_tolerance_rad:
            goal_key = current_key
            break
        if len(cost) >= limit:
            diagnostics["state_limit_reached"] = True
            return PlannerResult(False, None, "SEARCH_LIMIT", expanded_states=expanded, search_states=generated, configured_timeout_s=timeout_s, state_limit=limit, diagnostics=diagnostics, angle_resolution_deg=config.angle_resolution_deg, step_size_m=step)
        for steering in controls:
            if time.monotonic() >= deadline:
                return PlannerResult(False, None, "TIMEOUT", expanded_states=expanded, search_states=generated, configured_timeout_s=timeout_s, timeout_triggered=True, state_limit=limit, diagnostics=diagnostics, angle_resolution_deg=config.angle_resolution_deg, step_size_m=step)
            segment = v1._integrate_bicycle(x, y, yaw, steering, step, config.wheelbase_m, samples=max(1, int(math.ceil(step / config.integration_sample_spacing_m))))
            if not segment:
                diagnostics["rejected_discretization"] += 1
                continue
            collision = False
            for pose in segment:
                if time.monotonic() >= deadline:
                    diagnostics["deadline_elapsed_ms"] = (time.monotonic() - started) * 1000.0
                    return PlannerResult(False, None, "TIMEOUT", expanded_states=expanded, search_states=generated,
                                         configured_timeout_s=timeout_s, timeout_triggered=True, state_limit=limit,
                                         diagnostics=diagnostics, angle_resolution_deg=config.angle_resolution_deg,
                                         step_size_m=step)
                if not _free_pose(ctx, pose):
                    collision = True
                    break
            if collision:
                diagnostics["rejected_collision"] += 1
                continue
            nx, ny, nyaw = segment[-1]
            nkey = key(nx, ny, nyaw)
            if nkey[0] < 0:
                diagnostics["rejected_out_of_bounds"] += 1
                continue
            new_cost = current_cost + step
            if new_cost >= cost.get(nkey, float("inf")):
                diagnostics["rejected_duplicate"] += 1
                continue
            if expanded == 1:
                diagnostics["first_layer_successors"] += 1
            state_for[nkey] = (nx, ny, _wrap(nyaw))
            cost[nkey] = new_cost
            parent[nkey] = current_key
            generated += 1
            heapq.heappush(queue, (new_cost + math.hypot(query.goal[0] - nx, query.goal[1] - ny), new_cost, nkey))
    if goal_key is None:
        code = "TIMEOUT" if time.monotonic() >= deadline else "NO_PATH"
        return PlannerResult(False, None, code, expanded_states=expanded, search_states=generated, configured_timeout_s=timeout_s, timeout_triggered=code == "TIMEOUT", state_limit=limit, diagnostics=diagnostics, angle_resolution_deg=config.angle_resolution_deg, step_size_m=step)
    chain: List[Tuple[int, int, int]] = []
    cursor: Optional[Tuple[int, int, int]] = goal_key
    while cursor is not None:
        chain.append(cursor)
        cursor = parent[cursor]
    states = [state_for[item] for item in reversed(chain)]
    points = [{"x": float(px), "y": float(py), "yaw": _wrap(float(pyaw))} for px, py, pyaw in states]
    if goal_tail:
        points.extend(goal_tail)
    diagnostics["generated_states"] = generated
    diagnostics["expanded_states"] = expanded
    return PlannerResult(True, points, "", search_states=generated, expanded_states=expanded, configured_timeout_s=timeout_s, state_limit=limit, diagnostics=diagnostics, goal_connection_attempts=int(diagnostics["goal_connection_attempts"]), goal_connection_successes=int(diagnostics["goal_connection_successes"]), angle_resolution_deg=config.angle_resolution_deg, step_size_m=step)


def _planner_call(ctx: MapContext, query: Query, algorithm: str, config: RelaxedAckermannConfig, timeout_s: float, seed: int) -> PlannerResult:
    if algorithm == "astar":
        return _astar_relaxed(ctx, query, timeout_s)
    if algorithm == "hybrid_astar":
        return _hybrid_astar(ctx, query, config, timeout_s)
    if algorithm == "rrt_star_dubins_surrogate":
        return v1._rrt_star(ctx, query, config, timeout_s, seed)
    if algorithm == "kinodynamic_rrt_star_bicycle":
        return v1._kinodynamic_rrt_star(ctx, query, config, timeout_s, seed)
    raise ValueError(f"unknown algorithm: {algorithm}")


def _isolated_plan(ctx: MapContext, query: Query, algorithm: str, config: RelaxedAckermannConfig, timeout_s: float, seed: int) -> PlannerResult:
    started = time.monotonic()
    # Reserve a handoff margin so the child returns a structured TIMEOUT
    # result (including search diagnostics) before the parent watchdog fires.
    planner_timeout_s = max(0.001, float(timeout_s) - PLANNER_DEADLINE_MARGIN_MS / 1000.0)
    result = _planner_call(ctx, query, algorithm, config, planner_timeout_s, seed)
    result.configured_timeout_s = float(timeout_s)
    result.timeout_triggered = bool(result.timeout_triggered or result.failure_code == "TIMEOUT")
    if result.diagnostics is None:
        result.diagnostics = {}
    result.diagnostics["planner_timeout_s"] = planner_timeout_s
    result.diagnostics["planner_deadline_margin_ms"] = PLANNER_DEADLINE_MARGIN_MS
    setattr(result, "planning_time_ms", (time.monotonic() - started) * 1000.0)
    return result


def _protocol(map_ids: Sequence[str], contexts: Mapping[str, Any]) -> Dict[str, Any]:
    config = relaxed_config()
    return {
        "schema_version": 2,
        "experiment": "pln02_single_planner_ackermann_no_reverse_relaxed_60deg_v2",
        "version_label": "放宽约束对照实验",
        "dynamic_obstacles": False,
        "resolution": 0.05,
        "random_seed": 20260821,
        "warmup_runs": 3,
        "measured_runs": 5,
        "sample_interval_ms": SAMPLE_INTERVAL_MS,
        "process_start_method": "fork",
        "planner_deadline_margin_ms": PLANNER_DEADLINE_MARGIN_MS,
        "external_timeout_seconds": {map_id: TIMEOUTS[map_id] for map_id in map_ids},
        "query_policy": "core_queries_v1_exact_world_coordinates",
        "vehicle_model": "Jackal footprint + Ackermann surrogate (experimental abstraction; not Jackal mechanical structure)",
        "vehicle_model_id": "ackermann_surrogate_relaxed",
        "wheelbase_m": config.wheelbase_m,
        "max_steering_angle_deg": config.max_steering_angle_deg,
        "minimum_turning_radius_m": config.minimum_turning_radius_m,
        "maximum_curvature_per_m": config.maximum_curvature_per_m,
        "allow_reverse": config.allow_reverse,
        "allow_in_place_rotation": config.allow_in_place_rotation,
        "footprint": FOOTPRINT,
        "footprint_padding_m": config.footprint_padding_m,
        "additional_safety_margin_m": config.safety_margin_m,
        "allow_unknown": False,
        "endpoint_position_tolerance_m": config.endpoint_position_tolerance_m,
        "endpoint_yaw_tolerance_deg": math.degrees(config.endpoint_yaw_tolerance_rad),
        "curvature_sample_spacing_m": config.sample_spacing_m,
        "curvature_window_m": config.curvature_window_m,
        "hybrid": {
            "motion_model": "forward_bicycle_dubins_surrogate",
            "step_size_m": config.hybrid_step_size_m,
            "angle_resolution_deg": config.angle_resolution_deg,
            "angle_bins": config.angle_bins,
            "steering_angles_deg": list(config.steering_angles_deg),
            "integration_sample_spacing_m": config.integration_sample_spacing_m,
            "goal_connection_enabled": config.goal_connection_enabled,
            "goal_connection_radius_m": config.goal_connection_radius_m,
            "state_limit_policy": "timeout_scaled_not_fixed_120000",
        },
        "algorithms": list(ALGORITHMS),
        "maps": [
            {
                "map_id": map_id,
                "map_yaml": str(MAP_PATHS[map_id]),
                "map_sha256": (contexts[map_id].map_sha256 if isinstance(contexts[map_id], MapContext) else contexts[map_id]["map_sha256"]),
                "map_yaml_sha256": (contexts[map_id].map_yaml_sha256 if isinstance(contexts[map_id], MapContext) else contexts[map_id]["map_yaml_sha256"]),
                "origin": (list(contexts[map_id].hospital_map.origin) if isinstance(contexts[map_id], MapContext) else list(contexts[map_id]["origin"])),
                "width_cells": (contexts[map_id].hospital_map.width if isinstance(contexts[map_id], MapContext) else contexts[map_id]["width_cells"]),
                "height_cells": (contexts[map_id].hospital_map.height if isinstance(contexts[map_id], MapContext) else contexts[map_id]["height_cells"]),
            }
            for map_id in map_ids
        ],
    }


def _assert_protocol_map(context: MapContext, protocol: Mapping[str, Any]) -> None:
    """Reject runs against a map different from the prepared protocol."""
    entries = protocol.get("maps", [])
    entry = next((item for item in entries if str(item.get("map_id")) == context.map_id), None)
    if entry is None:
        raise ValueError(f"map {context.map_id} is absent from protocol")
    expected = {
        "map_sha256": context.map_sha256,
        "map_yaml_sha256": context.map_yaml_sha256,
        "width_cells": context.hospital_map.width,
        "height_cells": context.hospital_map.height,
        "origin": list(context.hospital_map.origin),
    }
    for field, actual in expected.items():
        recorded = entry.get(field)
        if field == "origin":
            if recorded is None or len(recorded) != len(actual) or any(not math.isclose(float(a), float(b), abs_tol=1e-12) for a, b in zip(recorded, actual)):
                raise ValueError(f"{context.map_id}: protocol {field} does not match current map")
        elif str(recorded) != str(actual):
            raise ValueError(f"{context.map_id}: protocol {field} does not match current map")


def _code_manifest(root: Path, command: Sequence[str] | str, started: Optional[str] = None, ended: Optional[str] = None) -> Dict[str, Any]:
    source_root = Path(__file__).resolve().parent
    tests = source_root.parent / "test"
    protocol = root / "protocol.yaml"
    queries = root / "core_queries_v1.yaml"
    manifest_path = root / "code_manifest.yaml"
    existing: Dict[str, Any] = {}
    if manifest_path.exists():
        try:
            existing = _read_yaml(manifest_path)
        except (OSError, ValueError, yaml.YAMLError):
            existing = {}
    # ``main`` refreshes provenance after every CLI stage.  Once a formal run
    # has recorded its exact invocation and time range, later refreshes (most
    # notably ``--stage report``) must not replace that evidence with a report
    # command and null timestamps.
    if (
        started is None
        and ended is None
        and existing.get("started_at")
        and existing.get("ended_at")
        and existing.get("command")
    ):
        # The run manifest is the immutable record of the code actually used
        # by the child processes.  In particular, do not recompute hashes
        # after a post-run report-only source change.
        return existing
    if started is None and ended is None and existing:
        existing_started = existing.get("started_at")
        existing_ended = existing.get("ended_at")
        if existing_started is not None or existing_ended is not None:
            started = existing_started
            ended = existing_ended
            if existing.get("command"):
                command = existing["command"]
    return write_code_manifest(
        manifest_path,
        repo_root=ROOT,
        benchmark_sources=[Path(__file__), source_root / "single_planner_benchmark.py"],
        hybrid_sources=[Path(__file__)],
        validator_sources=[source_root / "single_planner_benchmark.py", source_root / "planner_benchmark" / "path_metrics.py"],
        resource_sources=[source_root / "planner_benchmark" / "isolation.py", source_root / "planner_benchmark" / "resources.py"],
        # Hash every benchmark test source, including the v2-specific tests and
        # the shared isolation/provenance tests.  Ignored ``external/`` files
        # are still covered by these content digests even when Git reports a
        # clean worktree.
        test_sources=sorted(tests.glob("test_*.py")) if tests.exists() else [],
        protocol=protocol,
        core_queries=queries,
        command=command,
        started_at=started,
        ended_at=ended,
        extra={"version_label": "放宽约束对照实验", "dynamic_obstacles": False},
    )


def prepare_inputs(root: Path, map_ids: Sequence[str] = MAP_IDS) -> Path:
    _refuse_nonempty(root)
    root.mkdir(parents=True, exist_ok=True)
    if not SOURCE_QUERIES.exists():
        raise ValueError(f"missing fixed query set: {SOURCE_QUERIES}")
    source = _read_yaml(SOURCE_QUERIES)
    if len(source.get("queries", [])) != 10:
        raise ValueError("core query set must contain exactly q00-q09")
    shutil.copy2(SOURCE_QUERIES, root / "core_queries_v1.yaml")
    # Build maps one at a time.  Keep only lightweight metadata for protocol
    # emission; the inflated masks for the 400 m map are intentionally not
    # retained alongside the other maps.
    protocol_contexts: Dict[str, Dict[str, Any]] = {}
    map_rows = []
    for map_id in map_ids:
        context = _context(map_id)
        m = context.hospital_map
        protocol_contexts[map_id] = {
            "map_sha256": context.map_sha256,
            "map_yaml_sha256": context.map_yaml_sha256,
            "origin": list(m.origin),
            "width_cells": m.width,
            "height_cells": m.height,
        }
        map_rows.append({
            "map_id": map_id, "map_yaml": str(MAP_PATHS[map_id]), "map_sha256": context.map_sha256,
            "map_yaml_sha256": context.map_yaml_sha256, "resolution": m.resolution,
            "width_cells": m.width, "height_cells": m.height,
            "physical_width_m": m.width * m.resolution, "physical_height_m": m.height * m.resolution,
            "physical_area_m2": m.width * m.height * m.resolution ** 2,
            "origin": json.dumps(m.origin), "preprocess_wall_time_ms": getattr(context, "preprocess_wall_time_ms", None),
            "dynamic_obstacles": False,
        })
        del context
    protocol = _protocol(map_ids, protocol_contexts)
    protocol["core_queries_sha256"] = sha256_file(root / "core_queries_v1.yaml")
    _write_yaml(root / "protocol.yaml", protocol)
    pd.DataFrame(map_rows).to_csv(root / "maps.csv", index=False)
    _write_yaml(root / "manifest.yaml", {
        "schema_version": 2, "experiment": "pln02_single_planner_ackermann_no_reverse_relaxed_60deg_v2",
        "version_label": "放宽约束对照实验", "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "dynamic_obstacles": False, "map_ids": list(map_ids), "core_queries": "core_queries_v1.yaml", "protocol": "protocol.yaml",
        "source_queries": str(SOURCE_QUERIES), "resource_scope": "isolated_benchmark_process_only",
    })
    _code_manifest(root, ["prepare_inputs"])
    return root


def validate_inputs(root: Path, map_ids: Sequence[str] = MAP_IDS) -> pd.DataFrame:
    protocol = _read_yaml(root / "protocol.yaml")
    config = relaxed_config()
    if protocol.get("dynamic_obstacles", True) or protocol.get("allow_reverse", True) or protocol.get("allow_in_place_rotation", True):
        raise ValueError("v2 protocol must be static, forward-only and non-rotating")
    if not math.isclose(float(protocol.get("minimum_turning_radius_m", 0.0)), config.minimum_turning_radius_m, rel_tol=1e-8):
        raise ValueError("protocol radius does not match 60 degree steering model")
    queries = _queries(root / "core_queries_v1.yaml")
    recorded_query_hash = protocol.get("core_queries_sha256")
    if recorded_query_hash and str(recorded_query_hash) != sha256_file(root / "core_queries_v1.yaml"):
        raise ValueError("core query file hash does not match prepared protocol")
    rows: List[Dict[str, Any]] = []
    for map_id in map_ids:
        context = _context(map_id)
        _assert_protocol_map(context, protocol)
        for query in queries:
            checked = context.hospital_map.validate_query(query, FOOTPRINT, MIN_ENDPOINT_CLEARANCE_M, allow_unknown=False)
            invalid = checked.validation_status != "VALID"
            # q08's conservative 0.45 m endpoint is intentionally preserved;
            # validation is a record, never an automatic query replacement.
            rows.append({
                "map_id": map_id, "query_id": query.query_id,
                "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2],
                "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2],
                "validation_status": checked.validation_status, "validated_query": not invalid,
                "query_validation_status": checked.validation_status, "failure_code": "INVALID_ENDPOINT" if invalid else "",
                "start_status": checked.start_status, "goal_status": checked.goal_status,
                "connected": checked.connected, "start_clearance_m": checked.start_clearance_m,
                "goal_clearance_m": checked.goal_clearance_m, "reason": checked.reason,
                "dynamic_obstacles": False,
            })
    frame = pd.DataFrame(rows)
    frame.to_csv(root / "query_validation.csv", index=False)
    return frame


def _save_path(path: Path, points: Sequence[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        json.dump(list(points), stream, separators=(",", ":"))


def _base_row(run_id: str, map_id: str, query: Query, algorithm: str, repetition: int, run_mode: str, context: MapContext, seed: Optional[int]) -> Dict[str, Any]:
    return {
        "run_id": run_id, "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(), "map_id": map_id,
        "map_sha256": context.map_sha256, "query_id": query.query_id, "query_category": query.category,
        "source_code_hash": SOURCE_CODE_HASH,
        "algorithm": algorithm, "repetition": repetition, "run_mode": run_mode, "seed": seed,
        "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2],
        "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2],
        "planner_success": False, "action_success": False, "static_footprint_valid": False,
        "kinematic_valid": False, "final_valid_success": False, "failure_code": "", "failure_codes": "[]", "failure_detail": "",
        "query_validation_status": "VALID", "validated_query": True, "planner_attempted": False,
        "planning_time_ms": None, "wall_time_ms": None, "child_elapsed_ms": None, "monitor_overhead_ms": None, "cpu_user_ms": None, "cpu_system_ms": None,
        "cpu_total_ms": None, "cpu_percent": None,
        "process_rss_before_bytes": None, "process_rss_peak_bytes": None, "process_rss_after_bytes": None,
        "process_pss_before_bytes": None, "process_pss_peak_bytes": None, "process_pss_after_bytes": None,
        "rss_delta_bytes": None, "pss_delta_bytes": None, "sample_interval_ms": SAMPLE_INTERVAL_MS,
        "sample_count": 0, "sampling_limited": False, "resource_scope": "benchmark_process_only",
        "planner_rss_peak_bytes": None, "planner_pss_peak_bytes": None, "stack_rss_peak_bytes": None, "stack_pss_peak_bytes": None,
        "search_states": 0, "expanded_states": 0, "samples": 0, "rewires": 0, "first_solution_time_ms": None,
        "configured_timeout_s": TIMEOUTS[map_id], "timeout_triggered": False, "state_limit": None,
        "map_preprocess_wall_time_ms": getattr(context, "preprocess_wall_time_ms", None),
        "angle_resolution_deg": None, "angle_bins": None, "step_size_m": None, "integration_sample_spacing_m": None,
        "steering_angles_deg": "", "goal_connection_attempts": 0, "goal_connection_successes": 0,
        "goal_connection_failure": "", "rejected_collision": 0,
        "rejected_out_of_bounds": 0, "rejected_discretization": 0,
        "rejected_duplicate": 0, "first_layer_successors": 0,
        "path_file": "",
    }


def _run_records(
    root: Path,
    map_ids: Sequence[str],
    algorithms: Sequence[str],
    *,
    repetitions: int,
    warmups: int,
    query_ids: Optional[Sequence[str]] = None,
    command: Optional[Sequence[str] | str] = None,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    if not (root / "protocol.yaml").exists() or not (root / "query_validation.csv").exists():
        raise ValueError("prepare and validate must complete before run")
    # A formal output directory is immutable once a run table exists.  This
    # prevents accidental v2 reruns from silently mixing repetitions or
    # changing the frozen comparison denominator.
    if (root / "runs.csv").exists() or (root / "path_metrics.csv").exists():
        raise ValueError(f"refusing to overwrite existing run output: {root}")
    protocol = _read_yaml(root / "protocol.yaml")
    if protocol.get("dynamic_obstacles", True):
        raise ValueError("dynamic_obstacles must remain false")
    recorded_query_hash = protocol.get("core_queries_sha256")
    if recorded_query_hash and str(recorded_query_hash) != sha256_file(root / "core_queries_v1.yaml"):
        raise ValueError("core query file hash does not match prepared protocol")
    all_queries = _queries(root / "core_queries_v1.yaml")
    requested = set(query_ids or [query.query_id for query in all_queries])
    queries = [query for query in all_queries if query.query_id in requested]
    validation = pd.read_csv(root / "query_validation.csv")
    config = relaxed_config()
    runs: List[Dict[str, Any]] = []
    metrics: List[Dict[str, Any]] = []
    path_dir = root / "paths"
    path_dir.mkdir(parents=True, exist_ok=True)
    started_at = dt.datetime.now(dt.timezone.utc).isoformat()
    # Keep the exact invocation in provenance.  Callers such as ``main`` pass
    # the parsed command line; the fallback is useful for direct API users and
    # is still more truthful than a synthetic stage-only command.
    provenance_command: Sequence[str] | str = command or [str(sys.argv[0]), *sys.argv[1:]]
    for map_id in map_ids:
        # Keep only one inflated map in memory.  This is important for the
        # 400 m (8,000 x 8,000 cell) boundary map.
        context = _context(map_id)
        _assert_protocol_map(context, protocol)
        for query_index, query in enumerate(queries):
            check = validation[(validation.map_id == map_id) & (validation.query_id == query.query_id)]
            if check.empty:
                raise ValueError(f"missing validation row for {map_id}/{query.query_id}")
            check_row = check.iloc[0]
            is_valid = _as_bool(check_row.get("validated_query", str(check_row.get("validation_status", "")) == "VALID"))
            for algorithm in algorithms:
                for repetition in range(1, int(warmups) + int(repetitions) + 1):
                    run_mode = "warmup" if repetition <= warmups else "measured"
                    seed = 20260821 + query_index * 100 + repetition
                    run_id = f"{map_id}_{query.query_id}_{algorithm}_{run_mode}_{repetition}_{time.time_ns()}"
                    row = _base_row(run_id, map_id, query, algorithm, repetition, run_mode, context, seed)
                    if algorithm == "hybrid_astar":
                        # Preserve the effective lattice metadata even when a
                        # child is terminated exactly at its external
                        # deadline and cannot return a serialized result.
                        limit = config.state_limit_for(context, TIMEOUTS[map_id])
                        row.update({
                            "angle_resolution_deg": config.angle_resolution_deg,
                            "angle_bins": config.angle_bins,
                            "step_size_m": config.hybrid_step_size_m,
                            "integration_sample_spacing_m": config.integration_sample_spacing_m,
                            "steering_angles_deg": json.dumps(list(config.steering_angles_deg)),
                            "state_limit": limit,
                        })
                    if not is_valid:
                        row.update({"query_validation_status": "INVALID", "validated_query": False, "failure_code": "INVALID_ENDPOINT", "failure_codes": json.dumps(["INVALID_ENDPOINT"]), "failure_detail": str(check_row.get("reason", ""))})
                        runs.append(row)
                        metrics.append({"run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode})
                        continue
                    isolated = run_isolated(
                        _isolated_plan,
                        args=(context, query, algorithm, config, TIMEOUTS[map_id], seed),
                        timeout_s=TIMEOUTS[map_id], sample_interval_ms=SAMPLE_INTERVAL_MS,
                        # Fork preserves the prepared NumPy map mask without
                        # serializing multi-gigabyte 400 m arrays.  The child
                        # is still fresh for every request and is never reused.
                        start_method="fork",
                    )
                    result = isolated.value if isinstance(isolated.value, PlannerResult) else None
                    if isolated.timed_out:
                        failure = "TIMEOUT"
                    elif isolated.exception_type:
                        failure = "EXCEPTION"
                    elif result is None:
                        failure = "EXCEPTION"
                    else:
                        failure = result.failure_code or ""
                    row.update({
                        "planner_attempted": True,
                        "planner_success": bool(result and result.planner_success), "action_success": bool(result and result.planner_success),
                        "failure_code": failure, "failure_detail": (isolated.exception_message or (result.detail if result else "")),
                        "planning_time_ms": getattr(result, "planning_time_ms", None) if result else None,
                        "wall_time_ms": (isolated.child_elapsed_ms if isolated.child_elapsed_ms is not None else isolated.wall_time_ms), "child_elapsed_ms": isolated.child_elapsed_ms, "monitor_overhead_ms": isolated.monitor_overhead_ms, "cpu_user_ms": isolated.cpu_user_ms,
                        "cpu_system_ms": isolated.cpu_system_ms, "cpu_total_ms": isolated.cpu_total_ms,
                        "cpu_percent": (isolated.cpu_total_ms / (isolated.child_elapsed_ms if isolated.child_elapsed_ms is not None else isolated.wall_time_ms) * 100.0) if isolated.cpu_total_ms is not None and (isolated.child_elapsed_ms if isolated.child_elapsed_ms is not None else isolated.wall_time_ms) > 0 else None,
                        "process_rss_before_bytes": isolated.process_rss_before_bytes, "process_rss_peak_bytes": isolated.process_rss_peak_bytes,
                        "process_rss_after_bytes": isolated.process_rss_after_bytes, "process_pss_before_bytes": isolated.process_pss_before_bytes,
                        "process_pss_peak_bytes": isolated.process_pss_peak_bytes, "process_pss_after_bytes": isolated.process_pss_after_bytes,
                        "rss_delta_bytes": isolated.rss_delta_bytes, "pss_delta_bytes": isolated.pss_delta_bytes,
                        "sample_interval_ms": isolated.sample_interval_ms, "sample_count": isolated.sample_count,
                        "sampling_limited": isolated.sampling_limited, "planner_rss_peak_bytes": isolated.process_rss_peak_bytes,
                        "planner_pss_peak_bytes": isolated.process_pss_peak_bytes,
                        "search_states": result.search_states if result else 0, "expanded_states": result.expanded_states if result else 0,
                        "samples": result.samples if result else 0, "rewires": result.rewires if result else 0,
                        "first_solution_time_ms": result.first_solution_time_ms if result else None,
                        "configured_timeout_s": TIMEOUTS[map_id], "timeout_triggered": bool(isolated.timed_out or (result and result.timeout_triggered)),
                        "state_limit": result.state_limit if result else row.get("state_limit"), "angle_resolution_deg": result.angle_resolution_deg if result else row.get("angle_resolution_deg"),
                        "step_size_m": result.step_size_m if result else row.get("step_size_m"),
                    })
                    if result and result.diagnostics:
                        diagnostics = result.diagnostics
                        row.update({
                            "angle_bins": diagnostics.get("angle_bins"), "integration_sample_spacing_m": diagnostics.get("integration_sample_spacing_m"),
                            "steering_angles_deg": json.dumps(diagnostics.get("steering_angles_deg", [])),
                            "goal_connection_attempts": diagnostics.get("goal_connection_attempts", 0),
                            "goal_connection_successes": diagnostics.get("goal_connection_successes", 0),
                            "goal_connection_failure": json.dumps(diagnostics.get("goal_connection_failure_reasons", {}), sort_keys=True),
                            "rejected_collision": diagnostics.get("rejected_collision", 0),
                            "rejected_out_of_bounds": diagnostics.get("rejected_out_of_bounds", 0),
                            "rejected_discretization": diagnostics.get("rejected_discretization", 0),
                            "rejected_duplicate": diagnostics.get("rejected_duplicate", 0),
                            "first_layer_successors": diagnostics.get("first_layer_successors", 0),
                        })
                    metric: Dict[str, Any] = {"run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode}
                    if result and result.points:
                        # Use the v2 wrapper so the derived Ackermann curvature
                        # limit is a hard acceptance boundary.  The shared
                        # checker supplies geometry/resource metrics; this
                        # wrapper adds the strict v2 failure semantics.
                        metric.update(validate_path(context, query, result.points, config))
                        row.update({"static_footprint_valid": bool(metric.get("static_footprint_valid", False)), "kinematic_valid": bool(metric.get("kinematic_valid", False)), "final_valid_success": bool(result.planner_success and metric.get("static_footprint_valid", False) and metric.get("kinematic_valid", False))})
                        if not row["final_valid_success"]:
                            if not row["failure_code"]:
                                row["failure_code"] = metric.get("failure_code", "KINEMATIC_INVALID")
                            row["failure_codes"] = metric.get("failure_codes", json.dumps([row["failure_code"]]))
                        rel = Path("paths") / f"{run_id}.json.gz"
                        _save_path(root / rel, result.points)
                        row["path_file"] = str(rel)
                    runs.append(row)
                    metrics.append(metric)
        pd.DataFrame(runs).to_csv(root / "runs.csv", index=False)
        pd.DataFrame(metrics).to_csv(root / "path_metrics.csv", index=False)
        del context
    run_frame = pd.DataFrame(runs)
    metric_frame = pd.DataFrame(metrics)
    run_frame.to_csv(root / "runs.csv", index=False)
    metric_frame.to_csv(root / "path_metrics.csv", index=False)
    _code_manifest(root, provenance_command, started_at, dt.datetime.now(dt.timezone.utc).isoformat())
    return run_frame, metric_frame


def _quantiles(values: pd.Series) -> Dict[str, Optional[float]]:
    numeric = pd.to_numeric(values, errors="coerce").dropna()
    return {name: (float(numeric.quantile(q)) if len(numeric) else None) for name, q in (("P50", .5), ("P95", .95), ("P99", .99))}


def _plots(root: Path, measured: pd.DataFrame) -> None:
    """Write lightweight v2 plots from measured rows only.

    Plot generation is deliberately read-only with respect to run tables.  If
    Matplotlib is unavailable (for example in a minimal smoke environment), a
    README in ``plots/`` makes that limitation explicit instead of silently
    claiming that figures were produced.
    """
    plots = root / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - dependency-specific
        (plots / "README.txt").write_text(f"plots unavailable: {exc}\n", encoding="utf-8")
        return
    maps = pd.read_csv(root / "maps.csv")
    maps["scale_m"] = np.sqrt(pd.to_numeric(maps["physical_area_m2"], errors="coerce"))
    maps = maps.set_index("map_id").reindex(list(MAP_IDS))
    algorithms = [value for value in ALGORITHMS if value in set(measured["algorithm"].astype(str))]

    def scale_plot(field: str, title: str, ylabel: str, filename: str) -> None:
        fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharex=True)
        for axis, quantile, label in zip(axes, (.5, .95, .99), ("P50", "P95", "P99")):
            for algorithm in algorithms:
                group = measured[measured["algorithm"] == algorithm]
                if field not in group.columns:
                    continue
                values = pd.to_numeric(group[field], errors="coerce").groupby(group["map_id"]).quantile(quantile)
                values = values.reindex(maps.index)
                axis.plot(maps["scale_m"], values, marker="o", label=algorithm)
            axis.set_title(label)
            axis.set_xlabel("map side length (m)")
            axis.grid(alpha=.25)
        axes[0].set_ylabel(ylabel)
        if algorithms:
            axes[-1].legend(fontsize=7)
        fig.suptitle(title)
        fig.tight_layout()
        fig.savefig(plots / filename, dpi=150)
        plt.close(fig)

    scale_plot("wall_time_ms", "Relaxed v2 wall time vs map scale", "wall time (ms)", "planning_time_vs_scale.png")
    scale_plot("cpu_total_ms", "Relaxed v2 CPU time vs map scale", "CPU time (ms)", "cpu_vs_scale.png")
    scale_plot("process_rss_peak_bytes", "Relaxed v2 benchmark RSS vs map scale", "RSS (bytes)", "memory_vs_scale.png")
    scale_plot("process_pss_peak_bytes", "Relaxed v2 benchmark PSS vs map scale", "PSS (bytes)", "pss_memory_vs_scale.png")

    status = measured.copy()
    for field in ("planner_success", "static_footprint_valid", "kinematic_valid", "final_valid_success", "planner_attempted", "validated_query"):
        if field in status:
            status[field] = status[field].map(_as_bool)
    fig, ax = plt.subplots(figsize=(10, 5))
    # Query-level success is derived from one row per map/query, while timing
    # and resource curves above intentionally retain every measured attempt.
    query_groups = []
    for (_, _), query_group in status.groupby(["map_id", "algorithm"], sort=False):
        query_summary = _query_level_summary(query_group)
        if not query_summary.empty:
            query_groups.append(query_summary)
    query_status = pd.concat(query_groups, ignore_index=True) if query_groups else pd.DataFrame()
    for algorithm in algorithms:
        group = query_status[query_status["algorithm"] == algorithm] if not query_status.empty else pd.DataFrame()
        valid = group[group["validated_query"]] if len(group) and "validated_query" in group else group
        rates = valid.groupby("map_id")["query_final_valid_success"].mean().reindex(maps.index) if len(valid) else pd.Series(index=maps.index, dtype=float)
        ax.plot(maps["scale_m"], rates, marker="o", label=f"{algorithm} final(validated)")
    ax.set_xlabel("map side length (m)"); ax.set_ylabel("rate"); ax.set_ylim(-.02, 1.02); ax.grid(alpha=.25); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(plots / "success_rate_vs_scale.png", dpi=150); plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for algorithm in algorithms:
        values = pd.to_numeric(measured.loc[measured["algorithm"] == algorithm, "path_length_m"], errors="coerce").dropna() if "path_length_m" in measured else pd.Series(dtype=float)
        if len(values):
            axes[0].boxplot(values.to_numpy(), positions=[algorithms.index(algorithm) + 1], widths=.55)
    axes[0].set_xticks(range(1, len(algorithms) + 1), algorithms, rotation=25, ha="right"); axes[0].set_ylabel("path length (m)"); axes[0].grid(alpha=.2)
    quality_fields = ["minimum_clearance_m", "curvature_p95_per_m"]
    quality_labels = ["minimum clearance (m)", "curvature P95 (1/m)"]
    for algorithm in algorithms:
        values = pd.to_numeric(measured.loc[measured["algorithm"] == algorithm, quality_fields[0]], errors="coerce").dropna() if quality_fields[0] in measured else pd.Series(dtype=float)
        if len(values):
            axes[1].boxplot(values.to_numpy(), positions=[algorithms.index(algorithm) + 1], widths=.55)
    axes[1].set_xticks(range(1, len(algorithms) + 1), algorithms, rotation=25, ha="right"); axes[1].set_ylabel(quality_labels[0]); axes[1].grid(alpha=.2)
    for algorithm in algorithms:
        values = pd.to_numeric(measured.loc[measured["algorithm"] == algorithm, quality_fields[1]], errors="coerce").dropna() if quality_fields[1] in measured else pd.Series(dtype=float)
        if len(values):
            axes[2].boxplot(values.to_numpy(), positions=[algorithms.index(algorithm) + 1], widths=.55)
    axes[2].set_xticks(range(1, len(algorithms) + 1), algorithms, rotation=25, ha="right"); axes[2].set_ylabel(quality_labels[1]); axes[2].grid(alpha=.2)
    fig.suptitle("Relaxed v2 four-algorithm path quality (returned paths)"); fig.tight_layout(); fig.savefig(plots / "path_quality_comparison.png", dpi=150); plt.close(fig)

    # Failure counts are attempt-level by design: a repeated timeout or
    # invalid path must remain visible rather than being collapsed away.
    invalid = status[~status["final_valid_success"]].copy()
    if len(invalid) and "failure_code" in invalid.columns:
        failure_counts = invalid.groupby(["map_id", "algorithm", "failure_code"], dropna=False).size().rename("count").reset_index()
        failure_counts.to_csv(root / "failure_summary.csv", index=False)
        pivot = failure_counts.pivot_table(index="map_id", columns="failure_code", values="count", aggfunc="sum", fill_value=0).reindex(maps.index).fillna(0)
        ax = pivot.plot(kind="bar", stacked=True, figsize=(12, 5))
        ax.set_xlabel("map side length (m)"); ax.set_ylabel("invalid measured attempts"); ax.grid(axis="y", alpha=.25)
        ax.figure.tight_layout(); ax.figure.savefig(plots / "failure_codes_vs_scale.png", dpi=150); plt.close(ax.figure)


def _bool_series(values: pd.Series) -> pd.Series:
    """Parse boolean CSV fields without treating the string ``False`` as true."""
    if pd.api.types.is_bool_dtype(values):
        return values.fillna(False).astype(bool)
    true_values = {"1", "true", "t", "yes", "y"}
    return values.map(lambda value: str(value).strip().lower() in true_values if pd.notna(value) else False).astype(bool)


def _query_level_summary(group: pd.DataFrame) -> pd.DataFrame:
    """Collapse measured repetitions into one row per logical query.

    The benchmark intentionally repeats each query five times.  Query-level
    rates therefore must not use the number of measured rows as their
    denominator.  A query is considered successful when at least one measured
    attempt produces a final valid path; attempt-level rates remain available
    alongside it for timing/resource statistics.
    """
    if group.empty:
        return pd.DataFrame()
    rows: List[Dict[str, Any]] = []
    # A query id is only unique within a map.  The algorithm-level report
    # combines four maps, so collapsing by query_id alone would silently turn
    # 40 map/query pairs into 10 rows and corrupt its denominator.
    grouping = ["map_id", "query_id"] if "map_id" in group.columns else ["query_id"]
    for key, query_group in group.groupby(grouping, dropna=False, sort=False):
        if isinstance(key, tuple):
            map_id, query_id = key
        else:
            map_id, query_id = (None, key)
        validated = _bool_series(query_group.get("validated_query", pd.Series(True, index=query_group.index)))
        planner = _bool_series(query_group.get("planner_success", pd.Series(False, index=query_group.index)))
        action = _bool_series(query_group.get("action_success", pd.Series(False, index=query_group.index)))
        static = _bool_series(query_group.get("static_footprint_valid", pd.Series(False, index=query_group.index)))
        kinematic = _bool_series(query_group.get("kinematic_valid", pd.Series(False, index=query_group.index)))
        final = _bool_series(query_group.get("final_valid_success", pd.Series(False, index=query_group.index)))
        # Validation is deterministic per map/query.  ``all`` is conservative
        # if a malformed run table contains mixed validation flags.
        is_valid = bool(validated.all())
        rows.append({
            "map_id": map_id if map_id is not None else (query_group["map_id"].iloc[0] if "map_id" in query_group else None),
            "query_id": query_id,
            "algorithm": query_group["algorithm"].iloc[0] if "algorithm" in query_group else None,
            "count": int(len(query_group)),
            "validated_query": is_valid,
            "validated_query_count": int(is_valid),
            "invalid_query_count": int(not is_valid),
            "validated_attempt_count": int(validated.sum()),
            "invalid_attempt_count": int((~validated).sum()),
            "planner_attempt_count": int(_bool_series(query_group.get("planner_attempted", pd.Series(False, index=query_group.index))).sum()),
            "planner_success_count": int(planner.sum()),
            "action_success_count": int(action.sum()),
            "static_footprint_valid_count": int(static.sum()),
            "kinematic_valid_count": int(kinematic.sum()),
            "final_valid_success_count": int(final.sum()),
            "planner_success_rate_all_attempts": float(planner.mean()),
            "final_valid_success_rate_all_attempts": float(final.mean()),
            "validated_attempt_success_rate": float(planner[validated].mean()) if validated.any() else None,
            "validated_attempt_final_valid_success_rate": float(final[validated].mean()) if validated.any() else None,
            "query_planner_success": bool(planner.any()),
            "query_final_valid_success": bool(final.any()),
            "validated_query_success_rate": float(bool(final.any())) if is_valid else None,
            "all_query_success_rate": float(bool(final.any())),
            "failure_codes": ";".join(sorted({str(code) for code in query_group.get("failure_code", pd.Series(dtype=object)).dropna() if str(code)})),
        })
    return pd.DataFrame(rows)


def _backfill_path_metrics(runs: pd.DataFrame, metrics: pd.DataFrame) -> pd.DataFrame:
    """Keep structured diagnostics for attempts that returned no path.

    A planner failure is still a measured result.  Older runner revisions
    emitted a mostly empty path_metrics row in that case, which made a direct
    path_metrics consumer lose the failure code present in runs.csv.  Backfill
    only status/diagnostic fields; geometric metrics remain NaN because no path
    was available to evaluate.
    """
    if metrics.empty or "run_id" not in runs.columns:
        return metrics
    result = metrics.copy()
    run_index = runs.set_index("run_id", drop=False)
    if "path_available" not in result.columns:
        result["path_available"] = False
    if "path_validation_status" not in result.columns:
        result["path_validation_status"] = "NOT_EVALUATED"
    if "planner_success" not in result.columns:
        result["planner_success"] = False
    if "validated_query" not in result.columns:
        result["validated_query"] = False
    for column in ("static_footprint_valid", "kinematic_valid"):
        if column not in result.columns:
            result[column] = False
    if "failure_code" not in result.columns:
        result["failure_code"] = ""
    if "failure_codes" not in result.columns:
        result["failure_codes"] = "[]"
    for index, metric_row in result.iterrows():
        run_id = metric_row.get("run_id")
        if run_id not in run_index.index:
            continue
        run = run_index.loc[run_id]
        path_file = str(run.get("path_file", "") or "")
        has_path = bool(path_file and path_file.lower() != "nan")
        result.at[index, "path_available"] = has_path
        result.at[index, "path_validation_status"] = "EVALUATED" if has_path else "NOT_EVALUATED"
        result.at[index, "planner_success"] = _as_bool(run.get("planner_success", False))
        result.at[index, "validated_query"] = _as_bool(run.get("validated_query", False))
        if not has_path:
            result.at[index, "static_footprint_valid"] = False
            result.at[index, "kinematic_valid"] = False
            code = str(run.get("failure_code", "") or "UNSPECIFIED_FAILURE")
            result.at[index, "failure_code"] = code
            result.at[index, "failure_codes"] = json.dumps([code])
    return result


def report(root: Path) -> Path:
    runs = pd.read_csv(root / "runs.csv")
    metrics_path = root / "path_metrics.csv"
    metrics = pd.read_csv(metrics_path) if metrics_path.exists() else pd.DataFrame()
    if not metrics.empty:
        metrics = _backfill_path_metrics(runs, metrics)
        metrics.to_csv(metrics_path, index=False)
    measured = runs[runs.run_mode.astype(str).eq("measured")].copy()
    if not metrics.empty:
        join = ["run_id", "map_id", "query_id", "algorithm", "run_mode"]
        fields = [field for field in metrics.columns if field not in join]
        measured = measured.merge(metrics[join + fields], on=join, how="left", suffixes=("", "_metric"))
    # Pandas commonly reads literal True/False CSV values as object strings;
    # normalize once so every rate and denominator uses the intended semantics.
    for field in ("planner_success", "action_success", "static_footprint_valid", "kinematic_valid", "final_valid_success", "validated_query", "timeout_triggered", "sampling_limited"):
        if field in measured.columns:
            measured[field] = _bool_series(measured[field])
    rows = []
    query_rows: List[pd.DataFrame] = []
    for (map_id, algorithm), group in measured.groupby(["map_id", "algorithm"], sort=False):
        query_summary = _query_level_summary(group)
        if not query_summary.empty:
            query_rows.append(query_summary)
        valid_mask = group["validated_query"].map(_as_bool)
        valid_group = group[valid_mask]
        planner_ok = group["planner_success"].map(_as_bool)
        valid_planner_ok = valid_group["planner_success"].map(_as_bool)
        q = query_summary
        valid_queries = q[q["validated_query"]]
        all_queries = q
        returned_path_count = int(pd.to_numeric(group.get("path_length_m", pd.Series(dtype=float)), errors="coerce").notna().sum())
        row: Dict[str, Any] = {
            "map_id": map_id,
            "algorithm": algorithm,
            # ``count`` is deliberately an attempt count.  Query counts below
            # are unique map/query pairs and never repetitions.
            "count": int(len(group)),
            "query_count": int(len(all_queries)),
            "planner_success_count": int(planner_ok.sum()),
            "validated_query_count": int(len(valid_queries)),
            "invalid_query_count": int(len(all_queries) - len(valid_queries)),
            "validated_attempt_count": int(valid_mask.sum()),
            "invalid_attempt_count": int((~valid_mask).sum()),
            "all_query_attempt_rate": float(group["planner_attempted"].map(_as_bool).mean()) if "planner_attempted" in group else None,
            "all_query_success_rate": float(all_queries["query_final_valid_success"].mean()) if len(all_queries) else None,
            "validated_query_success_rate": float(valid_queries["query_final_valid_success"].mean()) if len(valid_queries) else None,
            "validated_attempt_success_rate": float(valid_planner_ok.mean()) if len(valid_group) else None,
            "static_footprint_valid_rate": float(valid_group["static_footprint_valid"].map(_as_bool).mean()) if len(valid_group) else None,
            "kinematic_valid_rate": float(valid_group["kinematic_valid"].map(_as_bool).mean()) if len(valid_group) else None,
            "final_valid_success_rate": float(valid_group["final_valid_success"].map(_as_bool).mean()) if len(valid_group) else None,
            "timeout_count": int(valid_group["timeout_triggered"].map(_as_bool).sum()) if "timeout_triggered" in valid_group else 0,
            "timing_resource_population": "all_valid_query_attempts",
            "path_quality_population": "planner_returned_paths",
            "path_quality_count": returned_path_count,
            "final_valid_path_count": int(valid_group["final_valid_success"].map(_as_bool).sum()) if len(valid_group) else 0,
        }
        timing_fields = ("planning_time_ms", "wall_time_ms", "cpu_total_ms", "process_rss_peak_bytes", "process_pss_peak_bytes")
        quality_fields = ("path_length_m", "length_over_euclidean", "minimum_clearance_m", "curvature_p95_per_m", "maximum_curvature_per_m")
        for field in timing_fields + quality_fields:
            if field in group:
                all_values = _quantiles(valid_group[field])
                success_values = _quantiles(valid_group.loc[valid_planner_ok, field])
                # Timing/resource quantiles use every valid-query attempt so
                # timeout and no-path costs remain visible.  Path-quality
                # values exist only for returned paths, so their ordinary
                # aliases remain success-conditioned.  Explicit aliases make
                # both populations auditable for every field.
                for key, value in all_values.items():
                    row[f"{field}_all_attempts_{key}"] = value
                for key, value in success_values.items():
                    row[f"{field}_success_{key}"] = value
                    if field in quality_fields:
                        row[f"{field}_{key}"] = value
                    else:
                        row[f"{field}_{key}"] = value if value is not None else all_values.get(key)
        rows.append(row)
    summary_frame = pd.DataFrame(rows)
    summary_frame.to_csv(root / "summary_by_map.csv", index=False)
    algorithm_rows = []
    for algorithm, group in measured.groupby("algorithm", sort=False):
        query_summary = _query_level_summary(group)
        valid_mask = group["validated_query"].map(_as_bool)
        valid_group = group[valid_mask]
        valid_queries = query_summary[query_summary["validated_query"]]
        returned_path_count = int(pd.to_numeric(group.get("path_length_m", pd.Series(dtype=float)), errors="coerce").notna().sum())
        row = {
            "algorithm": algorithm,
            "count": int(len(group)),
            "query_count": int(len(query_summary)),
            "map_count": int(group["map_id"].nunique()) if "map_id" in group else None,
            "map_query_count": int(len(query_summary)),
            "validated_query_count": int(len(valid_queries)),
            "invalid_query_count": int(len(query_summary) - len(valid_queries)),
            "validated_attempt_count": int(valid_mask.sum()),
            "invalid_attempt_count": int((~valid_mask).sum()),
            "all_query_attempt_rate": float(group["planner_attempted"].map(_as_bool).mean()) if "planner_attempted" in group else None,
            "all_query_success_rate": float(query_summary["query_final_valid_success"].mean()) if len(query_summary) else None,
            # Retain the old attempt-level aliases for compatibility, but make
            # the query-level fields above explicit and unambiguous.
            "planner_success_rate_all_queries": float(group["planner_success"].map(_as_bool).mean()),
            "planner_success_rate": float(valid_group["planner_success"].map(_as_bool).mean()) if len(valid_group) else None,
            "validated_query_success_rate": float(valid_queries["query_final_valid_success"].mean()) if len(valid_queries) else None,
            "validated_attempt_success_rate": float(valid_group["planner_success"].map(_as_bool).mean()) if len(valid_group) else None,
            "static_footprint_valid_rate": float(valid_group["static_footprint_valid"].map(_as_bool).mean()) if len(valid_group) else None,
            "kinematic_valid_rate": float(valid_group["kinematic_valid"].map(_as_bool).mean()) if len(valid_group) else None,
            "final_valid_success_rate": float(valid_group["final_valid_success"].map(_as_bool).mean()) if len(valid_group) else None,
            "timeout_count": int(valid_group["timeout_triggered"].map(_as_bool).sum()) if "timeout_triggered" in valid_group else 0,
            "timing_resource_population": "all_valid_query_attempts",
            "path_quality_population": "planner_returned_paths",
            "path_quality_count": returned_path_count,
            "final_valid_path_count": int(valid_group["final_valid_success"].map(_as_bool).sum()) if len(valid_group) else 0,
        }
        timing_fields = ("planning_time_ms", "wall_time_ms", "cpu_total_ms", "process_rss_peak_bytes", "process_pss_peak_bytes")
        quality_fields = ("path_length_m", "length_over_euclidean", "minimum_clearance_m", "curvature_p95_per_m", "maximum_curvature_per_m")
        for field in timing_fields + quality_fields:
            if field not in group.columns:
                continue
            all_values = _quantiles(valid_group[field])
            success_values = _quantiles(valid_group.loc[valid_group["planner_success"].map(_as_bool), field])
            for key, value in all_values.items():
                row[f"{field}_all_attempts_{key}"] = value
            for key, value in success_values.items():
                row[f"{field}_success_{key}"] = value
                if field in quality_fields:
                    row[f"{field}_{key}"] = value
                else:
                    row[f"{field}_{key}"] = value if value is not None else all_values.get(key)
        algorithm_rows.append(row)
    algorithm_frame = pd.DataFrame(algorithm_rows)
    algorithm_frame.to_csv(root / "summary_by_algorithm.csv", index=False)
    measured_query = pd.concat(query_rows, ignore_index=True) if query_rows else pd.DataFrame()
    measured_query.to_csv(root / "summary_by_query.csv", index=False)
    # Failure codes are attempt-level diagnostics.  Keep them sourced from the
    # measured run table rather than the collapsed query summary, which has
    # aggregate ``*_count`` fields instead of a scalar final_valid_success.
    failed_attempts = measured[~measured["final_valid_success"].map(_as_bool)].copy()
    if failed_attempts.empty:
        pd.DataFrame(columns=["map_id", "algorithm", "failure_code", "count"]).to_csv(root / "failure_summary.csv", index=False)
    else:
        failed_attempts["failure_code"] = failed_attempts["failure_code"].fillna("").replace("", "UNSPECIFIED_FAILURE")
        failed_attempts.groupby(["map_id", "algorithm", "failure_code"], dropna=False).size().rename("count").reset_index().to_csv(root / "failure_summary.csv", index=False)
    _plots(root, measured)
    # Audits keep timeout, isolated-process sampling, and Hybrid diagnostics
    # queryable without requiring a plotting dependency.  Missing diagnostics
    # are represented as empty JSON rather than inferred values.
    timeout_columns = [
        "run_id", "map_id", "query_id", "algorithm", "run_mode",
        "configured_timeout_s", "wall_time_ms", "timeout_triggered",
        "failure_code", "expanded_states", "search_states", "state_limit", "source_code_hash",
    ]
    measured.reindex(columns=[column for column in timeout_columns if column in measured.columns]).to_csv(root / "timeout_audit.csv", index=False)
    resource_columns = [
        "run_id", "map_id", "query_id", "algorithm", "run_mode", "wall_time_ms",
        "child_elapsed_ms", "monitor_overhead_ms",
        "cpu_user_ms", "cpu_system_ms", "cpu_total_ms", "cpu_percent",
        "process_rss_before_bytes", "process_rss_peak_bytes", "process_rss_after_bytes",
        "process_pss_before_bytes", "process_pss_peak_bytes", "process_pss_after_bytes",
        "rss_delta_bytes", "pss_delta_bytes", "sample_interval_ms", "sample_count",
        "sampling_limited", "resource_scope",
    ]
    measured.reindex(columns=[column for column in resource_columns if column in measured.columns]).to_csv(root / "resource_summary.csv", index=False)
    diagnostic_columns = [
        "run_id", "map_id", "query_id", "algorithm", "run_mode",
        "angle_resolution_deg", "angle_bins", "step_size_m", "integration_sample_spacing_m",
        "steering_angles_deg", "goal_connection_attempts", "goal_connection_successes",
        "goal_connection_failure", "expanded_states", "search_states", "state_limit",
        "rejected_collision", "rejected_out_of_bounds", "rejected_discretization",
        "rejected_duplicate", "first_layer_successors",
    ]
    measured.reindex(columns=[column for column in diagnostic_columns if column in measured.columns]).to_csv(root / "hybrid_diagnostics.csv", index=False)
    # A read-only v1 comparison table; it never mutates the frozen v1 output.
    old = ROOT / "experiments/single_planner_benchmark/ackermann_no_reverse_v1/summary_by_algorithm.csv"
    if old.exists():
        def failure_distribution(frame: pd.DataFrame) -> Dict[str, Dict[str, int]]:
            measured_frame = frame[frame["run_mode"].astype(str).eq("measured")] if "run_mode" in frame.columns else frame
            if measured_frame.empty or "algorithm" not in measured_frame.columns:
                return {}
            codes = measured_frame.get("failure_code", pd.Series("", index=measured_frame.index)).fillna("").replace("", "UNSPECIFIED_FAILURE")
            grouped = measured_frame.assign(_failure_code=codes).groupby(["algorithm", "_failure_code"], dropna=False).size()
            result: Dict[str, Dict[str, int]] = {}
            for (algorithm, code), count in grouped.items():
                result.setdefault(str(algorithm), {})[str(code)] = int(count)
            return result

        def comparison_frame(
            frame: pd.DataFrame,
            version: str,
            resource_scope: str,
            timing_population: str,
            query_denominator: str,
            failure_map: Mapping[str, Mapping[str, int]],
            protocol_fields: Mapping[str, Any],
        ) -> pd.DataFrame:
            """Add common, explicitly named fields before concatenating v1/v2.

            v1 called its isolated benchmark process ``planner_*`` while v2
            deliberately calls it ``process_*`` and does not expose a Nav2
            stack.  Keeping the source-specific columns is useful for
            provenance, but a canonical process column prevents a silent
            cross-version all-NaN comparison.
            """
            result = frame.copy()
            result.insert(0, "version", version)
            result["comparison_resource_scope"] = resource_scope
            result["comparison_timing_population"] = timing_population
            result["comparison_query_denominator"] = query_denominator
            result["comparison_path_quality_population"] = "planner_returned_paths_only"
            result["comparison_stack_metrics"] = "not_applicable"
            for field, value in protocol_fields.items():
                result[field] = value
            result["failure_code_distribution_json"] = result["algorithm"].map(
                lambda algorithm: json.dumps(dict(failure_map.get(str(algorithm), {})), sort_keys=True)
            )
            if "final_valid_success_rate" in result.columns:
                count_source = result.get("validated_attempt_count", result.get("count", pd.Series(np.nan, index=result.index)))
                result["final_valid_path_count"] = (
                    pd.to_numeric(result["final_valid_success_rate"], errors="coerce")
                    * pd.to_numeric(count_source, errors="coerce")
                ).round()
            else:
                result["final_valid_path_count"] = np.nan
            for metric in ("rss_peak_bytes", "pss_peak_bytes"):
                for quantile in ("P50", "P95", "P99"):
                    canonical = f"benchmark_process_{metric}_{quantile}"
                    if canonical in result.columns:
                        continue
                    source = next(
                        (
                            column
                            for column in (
                                f"process_{metric}_{quantile}",
                                f"planner_{metric}_{quantile}",
                            )
                            if column in result.columns
                        ),
                        None,
                    )
                    result[canonical] = result[source] if source is not None else np.nan
            return result

        def protocol_comparison_fields(protocol: Mapping[str, Any], *, legacy: bool = False) -> Dict[str, Any]:
            details = protocol.get("algorithm_details", {}).get("hybrid_astar", {}) if legacy else protocol.get("hybrid", {})
            return {
                "comparison_max_steering_angle_deg": protocol.get("max_steering_angle_deg"),
                "comparison_minimum_turning_radius_m": protocol.get("minimum_turning_radius_m"),
                "comparison_maximum_curvature_per_m": protocol.get("maximum_curvature_per_m"),
                "comparison_hybrid_step_size_m": details.get("step_size_m"),
                "comparison_hybrid_angle_resolution_deg": details.get("angle_resolution_deg"),
                "comparison_hybrid_integration_sample_spacing_m": details.get("integration_sample_spacing_m"),
                "comparison_hybrid_state_limit_policy": details.get(
                    "state_limit_policy",
                    "legacy_protocol_unspecified" if legacy else "unspecified",
                ),
            }

        old_protocol = _read_yaml(ROOT / "experiments/single_planner_benchmark/ackermann_no_reverse_v1/protocol.yaml")
        new_protocol = _read_yaml(root / "protocol.yaml")
        old_frame = comparison_frame(
            pd.read_csv(old),
            "ackermann_no_reverse_v1",
            "legacy_v1_planner_process_fields",
            "planner_success_conditioned",
            "legacy_all_measured_attempts",
            failure_distribution(pd.read_csv(ROOT / "experiments/single_planner_benchmark/ackermann_no_reverse_v1/runs.csv")),
            protocol_comparison_fields(old_protocol, legacy=True),
        )
        current_measured = measured.copy()
        new_frame = comparison_frame(
            pd.DataFrame(algorithm_rows),
            OUTPUT_NAME,
            "benchmark_process_only",
            "planner_success_conditioned;all_valid_query_attempts_available",
            "validated_query_attempts_for_validated_rate;query_level_for_all_query_rate",
            failure_distribution(current_measured),
            protocol_comparison_fields(new_protocol),
        )
        pd.concat([old_frame, new_frame], ignore_index=True, sort=False).to_csv(root / "v1_v2_comparison.csv", index=False)
    return root


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PLN-02 relaxed 60 degree Ackermann-surrogate static benchmark")
    parser.add_argument("--root", default=str(ROOT / "experiments/single_planner_benchmark" / OUTPUT_NAME))
    parser.add_argument("--stage", choices=("prepare", "validate", "smoke", "run", "report", "all"), default="prepare")
    parser.add_argument("--map-id", action="append", dest="map_ids", choices=list(MAP_IDS))
    parser.add_argument("--algorithm", action="append", dest="algorithms", choices=list(ALGORITHMS))
    parser.add_argument("--query-id", action="append", dest="query_ids")
    parser.add_argument("--warmups", type=int, default=3)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True, help="required static-map safety assertion")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    invocation: List[str] = [sys.executable, *(list(argv) if argv is not None else list(sys.argv[1:]))]
    root = Path(args.root).resolve()
    map_ids = args.map_ids or list(MAP_IDS)
    algorithms = args.algorithms or list(ALGORITHMS)
    try:
        if args.stage in {"prepare", "all"}:
            prepare_inputs(root, map_ids)
        if args.stage in {"validate", "all"}:
            validate_inputs(root, map_ids)
        if args.stage == "smoke":
            if not (root / "protocol.yaml").exists():
                prepare_inputs(root, map_ids)
            validate_inputs(root, map_ids)
            _run_records(root, map_ids, algorithms, repetitions=args.repetitions, warmups=args.warmups, query_ids=args.query_ids, command=invocation)
            report(root)
        elif args.stage == "run":
            _run_records(root, map_ids, algorithms, repetitions=args.repetitions, warmups=args.warmups, query_ids=args.query_ids, command=invocation)
        elif args.stage == "report":
            report(root)
        elif args.stage == "all":
            validate_inputs(root, map_ids)
            _run_records(root, map_ids, algorithms, repetitions=args.repetitions, warmups=args.warmups, query_ids=args.query_ids, command=invocation)
            report(root)
        _code_manifest(root, invocation)
        print(f"relaxed single planner benchmark {args.stage} output: {root}")
        return 0
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        print(f"relaxed_single_planner_benchmark: ERROR: {exc}", file=os.sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
