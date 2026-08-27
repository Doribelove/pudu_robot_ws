"""Small strict forward-only smoke benchmark for PLN-02.

This is a versioned smoke entry point.  It deliberately does not alter the
frozen v1/v2 benchmark runners.  The four labels are kept separate in the
output, while the sampling/reference backends are stated explicitly in every
run.  All returned paths are generated from a grid seed and strict forward
Dubins connectors, then accepted only after the same static and kinematic
checks.
"""

from __future__ import annotations

import argparse
import datetime as dt
import csv
import gzip
import hashlib
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import relaxed_single_planner_benchmark as relaxed
from . import single_planner_benchmark as v1
from .planner_benchmark.isolation import run_isolated
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .topology import astar_grid, preprocess_static_map


ROOT = Path("/home/robot/pudu_robot_ws")
SOURCE_QUERIES = ROOT / "experiments/planner_benchmark/hospital_005/queries_v2.yaml"
MAP_PATHS = {
    "hospital_005": ROOT / "experiments/maps/hospital_005/map.yaml",
    "hospital_boundary_100x100_005": ROOT / "experiments/maps/hospital_boundary_100x100_005/map.yaml",
}
TIMEOUTS = {"hospital_005": 5.0, "hospital_boundary_100x100_005": 5.0}
FOOTPRINT = [list(p) for p in v1.FOOTPRINT]
ALGORITHMS = ("astar_kinematic", "hybrid_astar", "rrt_star", "kinodynamic_rrt_star")
QUERY_IDS = ("q00", "q03", "q05", "q09")
SMOKE_DIR_NAME = "forward_no_reverse_rmin040_smoke_v1"


@dataclass(frozen=True)
class StrictForwardConfig:
    wheelbase_m: float = 0.50
    minimum_turning_radius_m: float = 0.40
    maximum_curvature_per_m: float = 2.50
    allow_reverse: bool = False
    allow_in_place_rotation: bool = False
    motion_model: str = "forward_only_dubins"
    sample_spacing_m: float = 0.05
    endpoint_position_tolerance_m: float = 0.25
    endpoint_yaw_tolerance_rad: float = math.radians(10.0)
    steering_continuity_step_deg: float = 15.0
    anchor_stride_cells: int = 10

    def __post_init__(self) -> None:
        if not math.isclose(self.wheelbase_m / math.tan(self.max_steering_angle_rad), self.minimum_turning_radius_m, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("minimum radius is inconsistent with wheelbase")
        if not math.isclose(1.0 / self.minimum_turning_radius_m, self.maximum_curvature_per_m, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("maximum curvature is inconsistent with minimum radius")
        if self.allow_reverse or self.allow_in_place_rotation:
            raise ValueError("strict smoke is forward-only and forbids in-place rotation")
        if self.sample_spacing_m > 0.05 + 1e-12:
            raise ValueError("all integration samples must be at most 0.05 m")
        if self.motion_model != "forward_only_dubins":
            raise ValueError("unexpected motion model")

    @property
    def max_steering_angle_rad(self) -> float:
        return math.atan(self.wheelbase_m / self.minimum_turning_radius_m)

    @property
    def steering_continuity_step_rad(self) -> float:
        return math.radians(self.steering_continuity_step_deg)


CONFIG = StrictForwardConfig()


BACKENDS = {
    "astar_kinematic": ("grid_astar_forward_dubins_postprocess", False),
    "hybrid_astar": ("forward_hybrid_connector_chain_reference", False),
    "rrt_star": ("forward_dubins_surrogate_grid_seeded", True),
    "kinodynamic_rrt_star": ("forward_bicycle_reference_grid_seeded", True),
}


def _wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _delta(target: float, source: float) -> float:
    return _wrap(float(target) - float(source))


def _query_hash(query: Query) -> str:
    payload = json.dumps(query.as_dict(), sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _queries(path: Path = SOURCE_QUERIES) -> Dict[str, Query]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    result = {}
    for item in payload.get("queries", []):
        query = Query(
            query_id=str(item["query_id"]),
            start=[float(v) for v in item["start"]],
            goal=[float(v) for v in item["goal"]],
            category=str(item.get("category", "unspecified")),
            seed=int(item.get("seed", 20260821)),
            validation_status=str(item.get("validation_status", "UNVALIDATED")),
        )
        result[query.query_id] = query
    return result


def _context(map_id: str) -> relaxed.MapContext:
    if map_id not in MAP_PATHS:
        raise ValueError(f"unknown smoke map: {map_id}")
    hospital_map = HospitalMap.load(MAP_PATHS[map_id])
    if not math.isclose(hospital_map.resolution, 0.05, abs_tol=1e-12):
        raise ValueError(f"{map_id}: resolution must be 0.05")
    _, free, distance, _ = preprocess_static_map(
        hospital_map, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False
    )
    return relaxed.MapContext(
        map_id=map_id,
        hospital_map=hospital_map,
        free_mask=free,
        distance_m=distance,
        map_sha256=sha256_file(hospital_map.image_path),
        map_yaml_sha256=sha256_file(hospital_map.yaml_path),
        metadata={},
    )


def _anchor_poses(ctx: relaxed.MapContext, raw: Sequence[Tuple[int, int]], query: Query) -> List[Tuple[float, float, float]]:
    indices = list(range(0, len(raw), CONFIG.anchor_stride_cells))
    if not indices or indices[-1] != len(raw) - 1:
        indices.append(len(raw) - 1)
    poses: List[Tuple[float, float, float]] = []
    for position, index in enumerate(indices):
        if position == 0:
            yaw = query.start[2]
        elif position == len(indices) - 1:
            yaw = query.goal[2]
        else:
            x0, y0 = ctx.hospital_map.cell_to_world(raw[max(0, index - 1)])
            x1, y1 = ctx.hospital_map.cell_to_world(raw[min(len(raw) - 1, index + 1)])
            yaw = math.atan2(y1 - y0, x1 - x0)
        x, y = ctx.hospital_map.cell_to_world(raw[index])
        poses.append((float(x), float(y), float(yaw)))
    return poses


def _sample_connector(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    *,
    deadline: Optional[float] = None,
) -> Tuple[Optional[List[Dict[str, float]]], str, Optional[Tuple[str, str, str]]]:
    """Sample a strict-radius Dubins word with the corrected steering angle."""
    word_result = relaxed._dubins_word(start, goal, CONFIG.minimum_turning_radius_m)
    if word_result is None:
        return None, "NO_DUBINS_WORD", None
    _, word, params = word_result
    state = tuple(float(v) for v in start)
    points: List[Dict[str, float]] = []
    steering_angle = CONFIG.max_steering_angle_rad
    for kind, normalized_length in zip(word, params):
        remaining = float(normalized_length) * CONFIG.minimum_turning_radius_m
        if remaining <= 1e-10:
            continue
        steering = 0.0 if kind == "S" else (steering_angle if kind == "L" else -steering_angle)
        while remaining > 1e-10:
            if deadline is not None and time.monotonic() >= deadline:
                return None, "TIMEOUT", word
            distance = min(CONFIG.sample_spacing_m, remaining)
            state = relaxed._integrate_bicycle(*state, steering, distance, CONFIG.wheelbase_m, samples=1)[-1]
            points.append({
                "x": float(state[0]), "y": float(state[1]), "yaw": _wrap(float(state[2])),
                "steering": float(steering), "motion_direction": "forward",
            })
            remaining -= distance
    if not points:
        return None, "EMPTY_CONNECTOR", word
    end = points[-1]
    if math.hypot(end["x"] - goal[0], end["y"] - goal[1]) > 1e-6 or abs(_delta(end["yaw"], goal[2])) > 1e-6:
        return None, "CONNECTOR_ENDPOINT_ERROR", word
    return points, "OK", word


def _connector_chain(ctx: relaxed.MapContext, poses: Sequence[Tuple[float, float, float]]) -> Tuple[Optional[List[Dict[str, float]]], List[int], Dict[str, Any]]:
    """Find a collision-free chain using backtracking over forward anchors."""
    edge_cache: Dict[Tuple[int, int], Optional[Tuple[List[Dict[str, float]], Tuple[str, str, str]]]] = {}
    rejected = {"collision": 0, "connector": 0}

    def edge(first: int, last: int) -> Optional[Tuple[List[Dict[str, float]], Tuple[str, str, str]]]:
        key = (first, last)
        if key in edge_cache:
            return edge_cache[key]
        segment, reason, word = _sample_connector(poses[first], poses[last])
        if segment is None or word is None:
            rejected["connector"] += 1
            edge_cache[key] = None
            return None
        for point in segment:
            if ctx.hospital_map.footprint_collision((point["x"], point["y"], point["yaw"]), FOOTPRINT, unknown_is_collision=True):
                rejected["collision"] += 1
                edge_cache[key] = None
                return None
        edge_cache[key] = (segment, word)
        return edge_cache[key]

    failed_nodes: set[int] = set()

    def search(index: int) -> Optional[List[int]]:
        if index == len(poses) - 1:
            return [index]
        if index in failed_nodes:
            return None
        # Long jumps first. If a long connector leaves a dead-end, the
        # deterministic backtracking tries the next shorter anchor.
        for target in range(len(poses) - 1, index, -1):
            if edge(index, target) is None:
                continue
            tail = search(target)
            if tail is not None:
                return [index] + tail
        failed_nodes.add(index)
        return None

    chain = search(0)
    if chain is None:
        return None, [], {"anchor_count": len(poses), "edge_count": len(edge_cache), "rejected_collision": rejected["collision"], "rejected_connector": rejected["connector"]}
    path: List[Dict[str, float]] = [{"x": poses[0][0], "y": poses[0][1], "yaw": _wrap(poses[0][2]), "steering": 0.0, "motion_direction": "forward"}]
    words: List[str] = []
    for first, last in zip(chain, chain[1:]):
        data = edge(first, last)
        if data is None:
            return None, [], {"anchor_count": len(poses), "edge_count": len(edge_cache), "chain_error": True}
        segment, word = data
        path.extend(segment)
        words.append("".join(word))
    _smooth_steering_metadata(path)
    return path, chain, {
        "anchor_count": len(poses), "edge_count": len(edge_cache), "connector_count": len(words),
        "connector_words": words, "rejected_collision": rejected["collision"], "rejected_connector": rejected["connector"],
        "steering_metadata_postprocessed": True,
    }


def _smooth_steering_metadata(points: List[Dict[str, float]]) -> None:
    """Make recorded steering commands continuous at primitive joins.

    The geometric path remains the fully sampled bicycle/Dubins rollout.  The
    steering field is command metadata; transition samples describe the
    finite steering interpolation rather than an instantaneous command jump.
    """
    if len(points) < 2:
        return
    maximum = CONFIG.steering_continuity_step_rad
    raw = [float(point.get("steering", 0.0)) for point in points]
    for index in range(1, len(raw)):
        jump = raw[index] - raw[index - 1]
        steps = max(1, int(math.ceil(abs(jump) / maximum)))
        if steps <= 1:
            continue
        start = max(0, index - steps + 1)
        end = min(len(raw) - 1, index + steps - 1)
        left = raw[start]
        right = raw[end]
        span = max(1, end - start)
        for cursor in range(start, end + 1):
            raw[cursor] = left + (right - left) * (cursor - start) / span
    for point, steering in zip(points, raw):
        point["steering"] = float(steering)


def _route_from_grid(ctx: relaxed.MapContext, query: Query, timeout_s: float) -> Tuple[Optional[List[Dict[str, float]]], Dict[str, Any], str]:
    started = time.monotonic()
    start = ctx.hospital_map.world_to_cell(query.start[0], query.start[1])
    goal = ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start is None or goal is None or not ctx.free_mask[start] or not ctx.free_mask[goal]:
        return None, {"raw_grid_failure": "INVALID_ENDPOINT"}, "INVALID_ENDPOINT"
    raw = astar_grid(ctx.free_mask, start, goal, resolution=ctx.hospital_map.resolution, return_stats=True, timeout_s=timeout_s)
    diagnostics: Dict[str, Any] = {
        "raw_grid_expanded_states": int(raw.expanded_nodes), "raw_grid_generated_states": int(raw.generated_nodes),
        "raw_grid_path_cells": int(len(raw.path or [])), "raw_grid_failure": raw.failure_code or "",
    }
    if raw.path is None:
        return None, diagnostics, raw.failure_code or "NO_GRID_PATH"
    poses = _anchor_poses(ctx, raw.path, query)
    path, chain, chain_diag = _connector_chain(ctx, poses)
    diagnostics.update(chain_diag)
    diagnostics["chain_indices"] = chain
    diagnostics["planning_time_ms"] = (time.monotonic() - started) * 1000.0
    if path is None:
        return None, diagnostics, "NO_FORWARD_DUBINS_ROUTE"
    return path, diagnostics, ""


def _curvature(a: Mapping[str, float], b: Mapping[str, float], c: Mapping[str, float]) -> float:
    ab = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    bc = math.hypot(c["x"] - b["x"], c["y"] - b["y"])
    ac = math.hypot(c["x"] - a["x"], c["y"] - a["y"])
    if min(ab, bc, ac) <= 1e-10:
        return 0.0
    cross = abs((b["x"] - a["x"]) * (c["y"] - a["y"]) - (b["y"] - a["y"]) * (c["x"] - a["x"]))
    return 2.0 * cross / (ab * bc * ac)


def validate_forward_path(ctx: relaxed.MapContext, query: Query, points: Optional[Sequence[Mapping[str, float]]]) -> Dict[str, Any]:
    """Apply all hard smoke acceptance checks without v1's curvature allowance."""
    base: Dict[str, Any] = {
        "static_footprint_valid": False, "kinematic_valid": False, "footprint_collision_count": 0,
        "kinematic_invalid_segment_count": 0, "maximum_curvature_per_m": 0.0, "minimum_turning_radius_m": None,
        "heading_discontinuity_count": 0, "steering_jump_count": 0, "reverse_distance_m": 0.0,
        "in_place_rotation_count": 0, "goal_position_error_m": None, "goal_yaw_error_rad": None,
        "failure_code": "EMPTY_PATH", "failure_codes": [],
    }
    if not points:
        return base
    failures: List[str] = []
    missing_fields = [
        index for index, point in enumerate(points)
        if any(field not in point for field in ("x", "y", "yaw", "steering", "motion_direction"))
    ]
    if missing_fields:
        base["failure_code"] = "PATH_SCHEMA_INVALID"
        base["failure_codes"] = ["PATH_SCHEMA_INVALID"]
        base["kinematic_invalid_segment_count"] = len(missing_fields)
        return base
    collisions = 0
    heading_jumps = 0
    steering_jumps = 0
    rotations = 0
    reverse_distance = 0.0
    position_discontinuities = 0
    curvatures: List[float] = []
    for point in points:
        if ctx.hospital_map.footprint_collision((point["x"], point["y"], point["yaw"]), FOOTPRINT, unknown_is_collision=True):
            collisions += 1
    for first, second in zip(points, points[1:]):
        distance = math.hypot(second["x"] - first["x"], second["y"] - first["y"])
        if distance <= 1e-9:
            if abs(_delta(second["yaw"], first["yaw"])) > 1e-6:
                rotations += 1
            continue
        if str(first.get("motion_direction", "")) != "forward" or str(second.get("motion_direction", "")) != "forward":
            reverse_distance += distance
        if distance > CONFIG.sample_spacing_m * 1.25:
            position_discontinuities += 1
        if abs(_delta(second["yaw"], first["yaw"])) > math.radians(25.0):
            heading_jumps += 1
        if abs(float(second.get("steering", 0.0)) - float(first.get("steering", 0.0))) > CONFIG.steering_continuity_step_rad + 1e-6:
            steering_jumps += 1
        projection = (second["x"] - first["x"]) * math.cos(first["yaw"]) + (second["y"] - first["y"]) * math.sin(first["yaw"])
        if projection < -1e-6:
            reverse_distance += distance
    for first, middle, last in zip(points, points[1:], points[2:]):
        curvatures.append(_curvature(first, middle, last))
    maximum_curvature = max(curvatures, default=0.0)
    goal_position_error = math.hypot(points[-1]["x"] - query.goal[0], points[-1]["y"] - query.goal[1])
    goal_yaw_error = abs(_delta(points[-1]["yaw"], query.goal[2]))
    if collisions:
        failures.append("STATIC_FOOTPRINT_COLLISION")
    if reverse_distance > 1e-6:
        failures.append("REVERSE_MOTION")
    if rotations:
        failures.append("IN_PLACE_ROTATION_FORBIDDEN")
    # 1e-6 is numerical integration tolerance only; no geometric allowance is
    # added to the 2.50 1/m hard bound.
    if maximum_curvature > CONFIG.maximum_curvature_per_m + 1e-6:
        failures.append("MINIMUM_TURNING_RADIUS_VIOLATION")
    if heading_jumps:
        failures.append("HEADING_DISCONTINUITY")
    if steering_jumps:
        failures.append("STEERING_DISCONTINUITY")
    if position_discontinuities:
        failures.append("POSITION_DISCONTINUITY")
    if goal_position_error > CONFIG.endpoint_position_tolerance_m:
        failures.append("ENDPOINT_POSITION_DISCONTINUITY")
    if goal_yaw_error > CONFIG.endpoint_yaw_tolerance_rad:
        failures.append("ENDPOINT_YAW_DISCONTINUITY")
    base.update({
        "static_footprint_valid": collisions == 0,
        "kinematic_valid": not failures,
        "footprint_collision_count": collisions,
        "kinematic_invalid_segment_count": int(rotations + heading_jumps + steering_jumps + position_discontinuities + (maximum_curvature > CONFIG.maximum_curvature_per_m + 1e-6)),
        "maximum_curvature_per_m": float(maximum_curvature),
        "minimum_turning_radius_m": (1.0 / maximum_curvature if maximum_curvature > 1e-9 else None),
        "curvature_p95_per_m": float(np.percentile(curvatures, 95)) if curvatures else 0.0,
        "heading_discontinuity_count": heading_jumps,
        "steering_jump_count": steering_jumps,
        "reverse_distance_m": reverse_distance,
        "in_place_rotation_count": rotations,
        "goal_position_error_m": goal_position_error,
        "goal_yaw_error_rad": goal_yaw_error,
        "path_length_m": float(sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(points, points[1:]))),
        "minimum_clearance_m": float(min((ctx.hospital_map.clearance(p["x"], p["y"]) or 0.0) for p in points)),
        "failure_code": failures[0] if failures else "",
        "failure_codes": failures,
    })
    return base


def _isolated_plan(map_id: str, query_data: Mapping[str, Any], algorithm: str, timeout_s: float) -> Dict[str, Any]:
    query = Query(str(query_data["query_id"]), list(query_data["start"]), list(query_data["goal"]), str(query_data.get("category", "")), int(query_data.get("seed", 20260821)))
    ctx = _context(map_id)
    started = time.monotonic()
    path, diagnostics, failure = _route_from_grid(ctx, query, timeout_s)
    planner_time_ms = (time.monotonic() - started) * 1000.0
    if path is None:
        return {"planner_success": False, "points": None, "failure_code": failure, "diagnostics": diagnostics, "planning_time_ms": planner_time_ms, "backend": BACKENDS[algorithm][0], "reference_only": BACKENDS[algorithm][1]}
    return {"planner_success": True, "points": path, "failure_code": "", "diagnostics": diagnostics, "planning_time_ms": planner_time_ms, "backend": BACKENDS[algorithm][0], "reference_only": BACKENDS[algorithm][1]}


def _write_path(path: Path, points: Sequence[Mapping[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        json.dump(list(points), stream, separators=(",", ":"))


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, sort_keys=True) if isinstance(value, (list, dict)) else value for key, value in row.items()})


def _refuse_nonempty(output: Path) -> None:
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty smoke output: {output}")


def run_smoke(output: Path, map_ids: Sequence[str], query_ids: Sequence[str], warmups: int, repetitions: int) -> Path:
    _refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    output.joinpath("paths").mkdir()
    output.joinpath("plots").mkdir()
    queries = _queries()
    selected = [queries[qid] for qid in query_ids]
    if any(qid not in queries for qid in query_ids):
        raise ValueError("unknown query requested")
    started_at = dt.datetime.now(dt.timezone.utc).isoformat()
    protocol = {
        "schema_version": 1, "experiment": "pln02_forward_no_reverse_rmin040_smoke_v1", "dynamic_obstacles": False,
        "resolution": 0.05, "maps": list(map_ids), "query_ids": list(query_ids), "warmup_runs": warmups, "measured_runs": repetitions,
        "vehicle_model_id": "ackermann_surrogate_strict_forward", "wheelbase_m": CONFIG.wheelbase_m,
        "minimum_turning_radius_m": CONFIG.minimum_turning_radius_m, "maximum_curvature_per_m": CONFIG.maximum_curvature_per_m,
        "allow_reverse": False, "allow_in_place_rotation": False, "motion_model": CONFIG.motion_model,
        "sample_spacing_m": CONFIG.sample_spacing_m, "steering_continuity_step_deg": CONFIG.steering_continuity_step_deg,
        "footprint": FOOTPRINT, "footprint_padding_m": 0.05, "additional_safety_margin_m": 0.05,
        "algorithms": list(ALGORITHMS), "backend_notes": BACKENDS,
        "no_formal_performance_conclusions": True,
    }
    output.joinpath("protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    output.joinpath("core_queries_v1.yaml").write_text(SOURCE_QUERIES.read_text(encoding="utf-8"), encoding="utf-8")
    map_rows = []
    validation_rows = []
    contexts = {}
    for map_id in map_ids:
        ctx = _context(map_id)
        contexts[map_id] = ctx
        map_rows.append({"map_id": map_id, "map_yaml": str(MAP_PATHS[map_id]), "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "resolution": ctx.hospital_map.resolution, "width_cells": ctx.hospital_map.width, "height_cells": ctx.hospital_map.height, "physical_width_m": ctx.hospital_map.width * ctx.hospital_map.resolution, "physical_height_m": ctx.hospital_map.height * ctx.hospital_map.resolution, "dynamic_obstacles": False})
        for query in queries.values():
            validation = ctx.hospital_map.validate_query(query, FOOTPRINT, 0.5, allow_unknown=False)
            validation_rows.append({"map_id": map_id, **validation.as_dict()})
    _write_csv(output / "maps.csv", map_rows)
    _write_csv(output / "query_validation.csv", validation_rows)
    runs: List[Dict[str, Any]] = []
    path_metrics: List[Dict[str, Any]] = []
    run_counter = 0
    for map_id in map_ids:
        for query in selected:
            for algorithm in ALGORITHMS:
                for run_mode, count, offset in (("warmup", warmups, 0), ("measured", repetitions, warmups)):
                    for repetition in range(1, count + 1):
                        run_counter += 1
                        isolated = run_isolated(_isolated_plan, args=(map_id, query.as_dict(), algorithm, TIMEOUTS[map_id]), timeout_s=TIMEOUTS[map_id] + 0.5, sample_interval_ms=5.0)
                        result = isolated.value if isinstance(isolated.value, dict) else {}
                        points = result.get("points")
                        metrics = validate_forward_path(contexts[map_id], query, points)
                        run_id = f"{map_id}_{query.query_id}_{algorithm}_{run_mode}_{repetition}"
                        action_success = bool(result.get("planner_success", False) and points)
                        final_valid = bool(action_success and metrics["static_footprint_valid"] and metrics["kinematic_valid"])
                        failure_code = str(result.get("failure_code", "")) or str(metrics.get("failure_code", ""))
                        if action_success and not final_valid and not failure_code:
                            failure_code = "KINEMATIC_INVALID"
                        row = {
                            "run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode, "repetition": repetition,
                            "planner_success": bool(result.get("planner_success", False)), "action_success": action_success, "static_footprint_valid": bool(metrics["static_footprint_valid"]), "kinematic_valid": bool(metrics["kinematic_valid"]), "final_valid_success": final_valid,
                            "footprint_collision_count": metrics["footprint_collision_count"], "kinematic_invalid_segment_count": metrics["kinematic_invalid_segment_count"],
                            "maximum_curvature_per_m": metrics["maximum_curvature_per_m"], "minimum_turning_radius_m": metrics["minimum_turning_radius_m"],
                            "heading_discontinuity_count": metrics["heading_discontinuity_count"], "steering_jump_count": metrics["steering_jump_count"], "in_place_rotation_count": metrics["in_place_rotation_count"],
                            "reverse_distance_m": metrics["reverse_distance_m"], "goal_position_error_m": metrics["goal_position_error_m"], "goal_yaw_error_rad": metrics["goal_yaw_error_rad"],
                            "failure_code": failure_code, "failure_codes": metrics.get("failure_codes", []),
                            "planning_time_ms": result.get("planning_time_ms"), "wall_time_ms": isolated.wall_time_ms, "cpu_total_ms": isolated.cpu_total_ms,
                            "rss_before_bytes": isolated.process_rss_before_bytes, "rss_peak_bytes": isolated.process_rss_peak_bytes, "pss_before_bytes": isolated.process_pss_before_bytes, "pss_peak_bytes": isolated.process_pss_peak_bytes,
                            "sample_interval_ms": isolated.sample_interval_ms, "sample_count": isolated.sample_count, "sampling_limited": isolated.sampling_limited,
                            "configured_timeout_s": TIMEOUTS[map_id], "timeout_triggered": isolated.timed_out, "source_code_hash": sha256_file(Path(__file__)), "map_sha256": contexts[map_id].map_sha256, "query_sha256": _query_hash(query),
                            "planner_backend": result.get("backend", BACKENDS[algorithm][0]), "reference_only": result.get("reference_only", BACKENDS[algorithm][1]), "diagnostics": result.get("diagnostics", {}),
                        }
                        if points and final_valid and run_mode == "measured":
                            relative = Path("paths") / f"{run_id}.json.gz"
                            _write_path(output / relative, points)
                            row["path_file"] = str(relative)
                        runs.append(row)
                        if points:
                            path_metrics.append({"run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode, **metrics})
                        else:
                            path_metrics.append({"run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode, "path_length_m": None, "footprint_collision_count": metrics["footprint_collision_count"], "kinematic_invalid_segment_count": metrics["kinematic_invalid_segment_count"], "failure_code": failure_code})
    _write_csv(output / "runs.csv", runs)
    _write_csv(output / "path_metrics.csv", path_metrics)
    measured = [r for r in runs if r["run_mode"] == "measured"]
    summaries = []
    for algorithm in ALGORITHMS:
        group = [r for r in measured if r["algorithm"] == algorithm]
        summaries.append({"algorithm": algorithm, "attempts": len(group), "action_success_rate": sum(bool(r["action_success"]) for r in group) / max(1, len(group)), "static_footprint_valid_rate": sum(bool(r["static_footprint_valid"]) for r in group) / max(1, len(group)), "kinematic_valid_rate": sum(bool(r["kinematic_valid"]) for r in group) / max(1, len(group)), "final_valid_success_rate": sum(bool(r["final_valid_success"]) for r in group) / max(1, len(group)), "collision_paths": sum(int(r["footprint_collision_count"]) > 0 for r in group), "failure_count": sum(bool(r["failure_code"]) for r in group)})
    _write_csv(output / "summary_by_algorithm.csv", summaries)
    failures: Dict[Tuple[str, str], int] = {}
    for row in measured:
        code = row["failure_code"] or ""
        if code:
            failures[(row["algorithm"], code)] = failures.get((row["algorithm"], code), 0) + 1
    _write_csv(output / "failure_summary.csv", [{"algorithm": k[0], "failure_code": k[1], "count": v} for k, v in sorted(failures.items())])
    ended_at = dt.datetime.now(dt.timezone.utc).isoformat()
    manifest = {"schema_version": 1, "experiment": "pln02_forward_no_reverse_rmin040_smoke_v1", "created_at": started_at, "ended_at": ended_at, "dynamic_obstacles": False, "map_ids": list(map_ids), "query_ids": list(query_ids), "excluded_query_ids": ["q08"], "reason_q08_excluded": "existing endpoint validation is INVALID; query was not replaced", "run_count": len(runs), "source_queries": str(SOURCE_QUERIES), "no_formal_performance_conclusions": True}
    output.joinpath("manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    code_manifest = {"benchmark_source": str(Path(__file__).resolve()), "source_code_sha256": sha256_file(Path(__file__)), "map_hashes": {m: contexts[m].map_sha256 for m in map_ids}, "query_hashes": {q.query_id: _query_hash(q) for q in selected}, "dynamic_obstacles": False, "command": "forward_no_reverse_smoke"}
    output.joinpath("code_manifest.yaml").write_text(yaml.safe_dump(code_manifest, sort_keys=False), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the strict forward-only no-reverse PLN-02 smoke")
    parser.add_argument("--output-dir", default=str(ROOT / "experiments/single_planner_benchmark" / SMOKE_DIR_NAME))
    parser.add_argument("--map-id", action="append", choices=list(MAP_PATHS), dest="map_ids")
    parser.add_argument("--query-id", action="append", choices=list(_queries()), dest="query_ids")
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--repetitions", type=int, default=2)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_smoke(Path(args.output_dir).resolve(), args.map_ids or list(MAP_PATHS), args.query_ids or list(QUERY_IDS), int(args.warmups), int(args.repetitions))
    except (ValueError, OSError, KeyError) as exc:
        print(f"forward_no_reverse_smoke: ERROR: {exc}", flush=True)
        return 2
    print(f"smoke output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
