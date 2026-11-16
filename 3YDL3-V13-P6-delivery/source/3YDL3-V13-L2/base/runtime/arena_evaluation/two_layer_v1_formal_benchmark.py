"""Formal 2A-V1-r0 benchmark with topology-turn-adaptive corridor width.

This module is an independent successor to the 2A-V0 runner.  It reuses the
same L1 topology/cache, Smac session, validation, and measurement contracts;
the only planning change is the corridor mask builder: ordinary route cells
use a 2 m corridor and topology-route regions with high arc-length curvature
use a 4 m corridor.  No 6 m retry or Grid A* stage is present.
"""

from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import hashlib
import json
import math
import os
import resource
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from . import l1_l3_corridor_hybrid_smoke as candidate
from . import l1_l3_corridor_hybrid_validity as validity
from . import layered_architecture_paired_benchmark as paired
from . import unified_four_backends_smoke as legacy
from .planner_benchmark.map_utils import sha256_file


ROOT = Path("/home/robot/pudu_robot_ws")
MAP_ID = "mentor_map_20260825_005"
ARCHITECTURE_ID = "2A-V1"
IMPLEMENTATION_REVISION = "r0"
PARENT_ARCHITECTURE = "2A-V0"
PROTOCOL_VERSION = "PLN-02-EXP-V1"
EXPERIMENT_KIND = "static_formal"
QUERY_SET_ID = "arena_a2b_benchmark_20"
DEFAULT_OUTPUT = ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r0_v1"
DEFAULT_CACHE_ROOT = ROOT / "experiments/layered_planner_benchmark/2a_v1_mentor_map_20260825_005_20_r0_v1_cache"
V0_REFERENCE = ROOT / "experiments/layered_planner_benchmark/l1_l3_corridor_hybrid_mentor_map_20_validity_v3"
WARMUPS = 3
REPETITIONS = 5
SEED = 0
ROS_DOMAIN_ID = 229
CORRIDOR_SEMANTICS = "raw_map_smac_aligned"
CORRIDOR_PROFILE = "topology_turn_adaptive_2m_4m"
BASE_CORRIDOR_PADDING_M = 2.0
CORNER_CORRIDOR_PADDING_M = 4.0
NO_6M_PADDING = True
CURVATURE_WINDOW_M = 0.5
CORNER_CURVATURE_THRESHOLD = 1.0
CORNER_SUPPORT_EXTENSION_M = 1.0
SMAC_PARAMETER_PROFILE = "lighter_smoother"
OPTIMIZATION_PROFILE = "v7_candidate"
OPTIMIZATION_STAGE = "step3_delta_map"


def _json_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")).hexdigest()


def _grid_hash(mask: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(np.asarray(mask, dtype=np.uint8)).tobytes()).hexdigest()


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _numeric(value: Any, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _percentile(rows: Sequence[Mapping[str, Any]], field: str, p: float) -> Optional[float]:
    values = [_numeric(row.get(field)) for row in rows]
    values = [value for value in values if value is not None]
    return float(np.percentile(values, p)) if values else None


def _reference_summary(directory: Path) -> Dict[str, Any]:
    """Read the retained 2A-V0-r3 result without modifying that experiment."""
    path = directory / "runs.csv"
    if not path.exists():
        return {"available": False, "directory": str(directory)}
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    valid = sum(_truth(row.get("final_valid_success")) for row in measured)
    query_ids = sorted({row.get("query_id") for row in measured if row.get("query_id")})
    return {
        "available": bool(measured),
        "directory": str(directory),
        "measured_count": len(measured),
        "valid_count": valid,
        "valid_rate": valid / len(measured) if measured else None,
        "query_any_valid": sum(any(_truth(row.get("final_valid_success")) for row in measured if row.get("query_id") == query_id) for query_id in query_ids),
        "query_all_repeat_valid": sum(all(_truth(row.get("final_valid_success")) for row in measured if row.get("query_id") == query_id) for query_id in query_ids),
        "wall_p50": _percentile(measured, "pipeline_wall_time_ms", 50),
        "wall_p95": _percentile(measured, "pipeline_wall_time_ms", 95),
        "wall_p99": _percentile(measured, "pipeline_wall_time_ms", 99),
        "cpu_p50": _percentile(measured, "pipeline_cpu_total_ms", 50),
        "cpu_p95": _percentile(measured, "pipeline_cpu_total_ms", 95),
        "cpu_p99": _percentile(measured, "pipeline_cpu_total_ms", 99),
        "rss_p50": _percentile(measured, "peak_rss", 50),
        "pss_p50": _percentile(measured, "peak_pss", 50),
        "l1_p50": _percentile(measured, "l1_graph_search_ms", 50),
        "l3_p50": _percentile(measured, "hybrid_planning_time_ms", 50),
        "path_length_mean": float(np.mean([value for value in (_numeric(row.get("path_length_m")) for row in measured) if value is not None])) if any(_numeric(row.get("path_length_m")) is not None for row in measured) else None,
        "clearance_mean": float(np.mean([value for value in (_numeric(row.get("minimum_clearance_m")) for row in measured) if value is not None])) if any(_numeric(row.get("minimum_clearance_m")) is not None for row in measured) else None,
        "max_curvature": max((value for value in (_numeric(row.get("maximum_curvature")) for row in measured) if value is not None), default=None),
        "failure_counts": dict(collections.Counter(str(row.get("failure_code") or "") for row in measured if row.get("failure_code"))),
    }


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    materialized = list(rows)
    fields: List[str] = []
    for row in materialized:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        for row in materialized:
            encoded = {}
            for key, value in row.items():
                encoded[key] = json.dumps(value, sort_keys=True, default=str) if isinstance(value, (dict, list, tuple)) else value
            writer.writerow(encoded)


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _set_map() -> None:
    world = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds" / MAP_ID
    validity.MAP_ID = MAP_ID
    validity.WORLD = world
    validity.MAP_YAML = world / "map/map.yaml"
    validity.SCENARIO_JSON = world / "scenarios/a2b_benchmark_20.json"
    paired._configure_map(MAP_ID)


def _load_tasks() -> Tuple[List[Any], Dict[str, Any]]:
    _set_map()
    queries, metadata = paired._load_tasks()
    metadata = dict(metadata)
    metadata.update({"query_set_id": QUERY_SET_ID, "query_order_seed": SEED, "protocol_version": PROTOCOL_VERSION})
    return queries, metadata


def _context() -> Any:
    _set_map()
    return validity._context()


def _directory_bytes(path: Path) -> int:
    total = 0
    if not path.exists():
        return total
    for item in path.rglob("*"):
        if item.is_file():
            try:
                total += item.stat().st_size
            except OSError:
                pass
    return total


def _load_or_build_topology(ctx: Any, output: Path, cache_root: Path) -> Tuple[Any, Dict[str, Any]]:
    topology, info = validity._load_topology(ctx, output, cache_root)
    info = dict(info)
    info["topology_cache_bytes"] = _directory_bytes(Path(info.get("cache_directory", cache_root)))
    info["topology_build_cpu_ms"] = info.get("topology_build_cpu_time_ms", 0.0)
    return topology, info


def _source_manifest() -> Tuple[Dict[str, str], str]:
    files, _ = validity._source_manifest()
    files.update({
        str(ROOT / "docs/PLN-02_LAYERED_ARCHITECTURE_MASTER_PLAN.md"): sha256_file(ROOT / "docs/PLN-02_LAYERED_ARCHITECTURE_MASTER_PLAN.md"),
        str(ROOT / "docs/PLN-02_ARCHITECTURE_STABLE_V7.md"): sha256_file(ROOT / "docs/PLN-02_ARCHITECTURE_STABLE_V7.md"),
        str(Path(__file__).resolve()): sha256_file(Path(__file__).resolve()),
    })
    return files, _json_hash(files)


def _clean_polyline(route: Any) -> Tuple[np.ndarray, np.ndarray]:
    raw = np.asarray(list(getattr(route, "polyline", []) or []), dtype=float)
    if raw.ndim != 2 or raw.shape[0] == 0 or raw.shape[1] < 2:
        return np.zeros((0, 2), dtype=float), np.zeros((0,), dtype=float)
    points: List[np.ndarray] = [raw[0, :2]]
    for point in raw[1:, :2]:
        if float(np.linalg.norm(point - points[-1])) > 1.0e-9:
            points.append(point)
    cleaned = np.asarray(points, dtype=float)
    cumulative = np.zeros((len(cleaned),), dtype=float)
    if len(cleaned) > 1:
        cumulative[1:] = np.cumsum(np.linalg.norm(np.diff(cleaned, axis=0), axis=1))
    return cleaned, cumulative


def _point_at(points: np.ndarray, cumulative: np.ndarray, distance_m: float) -> np.ndarray:
    if len(points) == 0:
        return np.zeros((2,), dtype=float)
    if len(points) == 1:
        return points[0].copy()
    distance = min(max(float(distance_m), 0.0), float(cumulative[-1]))
    index = int(np.searchsorted(cumulative, distance, side="right") - 1)
    index = min(max(index, 0), len(points) - 2)
    span = float(cumulative[index + 1] - cumulative[index])
    fraction = 0.0 if span <= 1.0e-12 else (distance - float(cumulative[index])) / span
    return points[index] + fraction * (points[index + 1] - points[index])


def _heading_at(points: np.ndarray, cumulative: np.ndarray, distance_m: float) -> float:
    if len(points) < 2:
        return 0.0
    delta = min(0.25, CURVATURE_WINDOW_M * 0.5)
    left = _point_at(points, cumulative, distance_m - delta)
    right = _point_at(points, cumulative, distance_m + delta)
    vector = right - left
    if float(np.linalg.norm(vector)) <= 1.0e-12:
        return 0.0
    return math.atan2(float(vector[1]), float(vector[0]))


def _wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def analyze_topology_route(route: Any, topology: Any) -> Dict[str, Any]:
    """Analyze route curvature over 0.5 m arc-length windows.

    Returns deterministic support intervals and the topology node/edge IDs
    intersecting those intervals.  This function has no map or Smac side
    effects and is intentionally exposed for focused unit tests.
    """
    points, cumulative = _clean_polyline(route)
    total = float(cumulative[-1]) if len(cumulative) else 0.0
    if total <= 2.0 * CURVATURE_WINDOW_M:
        return {
            "corner_count": 0, "corner_intervals_m": [], "corner_node_ids": [], "corner_edge_ids": [],
            "corner_max_curvature_1pm": 0.0, "corner_support_length_m": 0.0,
            "curvature_samples": [], "route_points": points, "route_cumulative_m": cumulative,
        }
    step = min(0.10, max(0.05, float(getattr(getattr(topology, "hospital_map", None), "resolution", 0.05))))
    samples = np.arange(CURVATURE_WINDOW_M, total - CURVATURE_WINDOW_M + 1.0e-9, step)
    curvatures: List[Tuple[float, float]] = []
    for distance in samples:
        theta = abs(_wrap_angle(_heading_at(points, cumulative, float(distance + CURVATURE_WINDOW_M)) - _heading_at(points, cumulative, float(distance - CURVATURE_WINDOW_M))))
        curvatures.append((float(distance), float(theta / (2.0 * CURVATURE_WINDOW_M))))
    high = [distance for distance, curvature in curvatures if curvature >= CORNER_CURVATURE_THRESHOLD]
    intervals: List[List[float]] = []
    for distance in high:
        candidate_interval = [max(0.0, distance - CORNER_SUPPORT_EXTENSION_M), min(total, distance + CORNER_SUPPORT_EXTENSION_M)]
        if intervals and candidate_interval[0] <= intervals[-1][1] + step + 1.0e-9:
            intervals[-1][1] = max(intervals[-1][1], candidate_interval[1])
        else:
            intervals.append(candidate_interval)
    support_length = sum(max(0.0, end - start) for start, end in intervals)
    max_curvature = max((curvature for _, curvature in curvatures), default=0.0)

    graph = getattr(topology, "graph", None)
    nodes_by_id = {int(node.node_id): node for node in getattr(graph, "nodes", [])}
    corner_nodes: List[int] = []
    for node_id in getattr(route, "node_ids", []) or []:
        node = nodes_by_id.get(int(node_id))
        if node is None or len(points) == 0:
            continue
        node_distance = float(cumulative[int(np.argmin(np.sum((points - np.asarray([node.x, node.y])) ** 2, axis=1)))])
        if any(start - 1.0e-9 <= node_distance <= end + 1.0e-9 for start, end in intervals):
            corner_nodes.append(int(node_id))

    corner_edges: List[int] = []
    offset = 0.0
    edge_by_id = {int(edge.edge_id): edge for edge in getattr(graph, "edges", [])}
    for edge_id in getattr(route, "edge_ids", []) or []:
        edge = edge_by_id.get(int(edge_id))
        edge_length = float(getattr(edge, "length_m", 0.0)) if edge is not None else 0.0
        edge_end = offset + edge_length
        if any(max(offset, start) <= min(edge_end, end) + 1.0e-9 for start, end in intervals):
            corner_edges.append(int(edge_id))
        offset = edge_end

    return {
        "corner_count": len(intervals),
        "corner_intervals_m": [[float(start), float(end)] for start, end in intervals],
        "corner_node_ids": sorted(set(corner_nodes)),
        "corner_edge_ids": sorted(set(corner_edges)),
        "corner_max_curvature_1pm": float(max_curvature),
        "corner_support_length_m": float(support_length),
        "curvature_samples": [[float(distance), float(curvature)] for distance, curvature in curvatures],
        "route_points": points, "route_cumulative_m": cumulative,
    }


def _dilate_raw(ctx: Any, centerline: np.ndarray, padding_m: float) -> np.ndarray:
    raw_free = candidate._raw_free_mask(ctx)
    effective_radius_m = max(0.0, float(padding_m)) + candidate.FOOTPRINT_SAFETY_MARGIN_M + candidate.BEND_MARGIN_M
    radius_cells = max(1, int(math.ceil(effective_radius_m / float(ctx.hospital_map.resolution))))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius_cells + 1, 2 * radius_cells + 1))
    cells = np.argwhere(centerline)
    if cells.size == 0:
        return np.zeros_like(raw_free, dtype=bool)
    min_row = max(0, int(cells[:, 0].min()) - radius_cells)
    max_row = min(raw_free.shape[0], int(cells[:, 0].max()) + radius_cells + 1)
    min_col = max(0, int(cells[:, 1].min()) - radius_cells)
    max_col = min(raw_free.shape[1], int(cells[:, 1].max()) + radius_cells + 1)
    expanded = cv2.dilate(centerline[min_row:max_row, min_col:max_col], kernel, iterations=1).astype(bool)
    result = np.zeros_like(raw_free, dtype=bool)
    result[min_row:max_row, min_col:max_col] = expanded & raw_free[min_row:max_row, min_col:max_col]
    return result


def _corner_centerline(ctx: Any, analysis: Mapping[str, Any]) -> np.ndarray:
    points = np.asarray(analysis.get("route_points", []), dtype=float)
    cumulative = np.asarray(analysis.get("route_cumulative_m", []), dtype=float)
    centerline = np.zeros_like(candidate._raw_free_mask(ctx), dtype=np.uint8)
    for start, end in analysis.get("corner_intervals_m", []) or []:
        start = float(start); end = float(end)
        spacing = max(0.05, float(ctx.hospital_map.resolution))
        distances = np.arange(start, end + spacing * 0.5, spacing)
        if len(distances) == 0 or distances[-1] < end:
            distances = np.append(distances, end)
        previous = _point_at(points, cumulative, float(distances[0]))
        cell = ctx.hospital_map.world_to_cell(float(previous[0]), float(previous[1]))
        if cell is not None:
            centerline[cell] = 1
        for distance in distances[1:]:
            current = _point_at(points, cumulative, float(distance))
            candidate._draw_world_segment(ctx, centerline, previous, current)
            previous = current
    return centerline


def build_adaptive_corridor_mask(
    ctx: Any, topology: Any, route: Any, query: Any, start_cell: Any, goal_cell: Any,
    padding_m: float, semantics: str,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Build `raw_free & (dilate(route,2m) | dilate(corners,4m))`."""
    if semantics != CORRIDOR_SEMANTICS:
        raise ValueError(f"2A-V1 requires {CORRIDOR_SEMANTICS} semantics")
    if abs(float(padding_m) - BASE_CORRIDOR_PADDING_M) > 1.0e-9:
        raise ValueError("2A-V1 only accepts the fixed 2 m base padding")
    base = candidate._build_corridor_mask(ctx, topology, route, query, start_cell, goal_cell, BASE_CORRIDOR_PADDING_M, semantics)
    analysis = analyze_topology_route(route, topology)
    # Avoid even constructing a 4 m dilation for routes with no detected
    # high-curvature support interval.  This keeps ordinary corridors on the
    # 2 m path and makes the no-corner diagnostic unambiguous.
    if analysis["corner_count"]:
        corner_line = _corner_centerline(ctx, analysis)
        corner_mask = _dilate_raw(ctx, corner_line, CORNER_CORRIDOR_PADDING_M)
    else:
        corner_mask = np.zeros_like(candidate._raw_free_mask(ctx), dtype=bool)
    mixed = np.asarray(base, dtype=bool) | np.asarray(corner_mask, dtype=bool)
    raw_free = candidate._raw_free_mask(ctx)
    mixed &= raw_free
    widened = np.asarray(corner_mask, dtype=bool) & ~np.asarray(base, dtype=bool)
    total_free = max(1, int(np.count_nonzero(raw_free)))
    diagnostics = {
        "corner_count": int(analysis["corner_count"]),
        "corner_node_ids": list(analysis["corner_node_ids"]),
        "corner_edge_ids": list(analysis["corner_edge_ids"]),
        "corner_max_curvature_1pm": float(analysis["corner_max_curvature_1pm"]),
        "corner_support_length_m": float(analysis["corner_support_length_m"]),
        "corner_support_intervals_m": analysis["corner_intervals_m"],
        "base_corridor_padding_m": BASE_CORRIDOR_PADDING_M,
        "corner_corridor_padding_m": CORNER_CORRIDOR_PADDING_M,
        "corner_widened_area_ratio": float(np.count_nonzero(widened) / total_free),
        "corner_corridor_mask_hash": _grid_hash(corner_mask),
        "corner_allowed_grid_cells": int(np.count_nonzero(corner_mask)),
        "corridor_mask_strategy": CORRIDOR_PROFILE,
        "no_6m_padding": True,
    }
    return mixed, diagnostics


def _annotate_row(output: Path, row: Mapping[str, Any], query: Any, metadata: Mapping[str, Any], topology_info: Mapping[str, Any], cache_mode: str) -> Dict[str, Any]:
    result = dict(row)
    result.update({
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE,
        "protocol_version": PROTOCOL_VERSION, "query_set_id": QUERY_SET_ID,
        "query_role": "raw", "case_id": query.query_id, "seed": SEED,
        "cache_mode": cache_mode, "corridor_profile": CORRIDOR_PROFILE,
        "corridor_semantics": CORRIDOR_SEMANTICS,
        "l2_called": False, "l2_call_count": 0,
        "query_sha256": result.get("query_hash", ""),
        "start": json.dumps([float(query.start[0]), float(query.start[1]), float(query.start[2])], separators=(",", ":")),
        "goal": json.dumps([float(query.goal[0]), float(query.goal[1]), float(query.goal[2])], separators=(",", ":")),
    })
    result["action_success"] = bool(result.get("planner_success"))
    result["result_code"] = "SUCCEEDED" if result["action_success"] else str(result.get("action_status") or result.get("failure_code") or "ACTION_ABORTED")
    result["reason_code"] = str(result.get("failure_code") or ("" if result["action_success"] else result["result_code"]))
    result["last_layer"] = "L1" if str(result.get("failure_code") or "").startswith("L1_") else "L3_PRIME"
    result["online_wall_ms"] = _numeric(result.get("pipeline_wall_time_ms"), 0.0)
    result["planner_wall_ms"] = _numeric(result.get("l3_action_wall_ms"), _numeric(result.get("hybrid_planning_time_ms"), 0.0))
    result["cpu_ms"] = _numeric(result.get("pipeline_cpu_total_ms"), 0.0)
    wall = float(result["online_wall_ms"] or 0.0)
    result["avg_cpu_percent"] = 100.0 * float(result["cpu_ms"] or 0.0) / wall if wall > 1.0e-9 else None
    result["RSS"] = result.get("peak_rss") if result.get("peak_rss") is not None else "not_available"
    result["PSS"] = result.get("peak_pss") if result.get("peak_pss") is not None else "not_available"
    result["peak_memory_mib"] = (float(result["RSS"]) / (1024.0 * 1024.0)) if _numeric(result.get("RSS")) is not None else "not_available"
    result["ready_memory_mib"] = "not_available"
    # The route-selection flag is emitted in the planner diagnostics on the
    # candidate path.  Accept the top-level form as well for mocked callers,
    # while keeping the count tied to an actual selected L1 route.
    l1_selected = result.get("l1_route_selected")
    if l1_selected is None and isinstance(result.get("diagnostics"), Mapping):
        l1_selected = result["diagnostics"].get("l1_route_selected")
    result["l1_call_count"] = 1 if _truth(l1_selected) else 0
    result["l1_time_ms"] = _numeric(result.get("l1_graph_search_ms"), 0.0)
    result["l1_route_search_nodes"] = "not_available"
    result["l2_time_ms"] = 0.0; result["l2_search_nodes_expanded"] = "not_available"; result["l2_search_nodes_generated"] = "not_available"
    result["l3_call_count"] = int(_numeric(result.get("l3_prime_call_count"), 0.0) or 0)
    result["l3_time_ms"] = _numeric(result.get("hybrid_planning_time_ms"), 0.0)
    result["l3_retry_count"] = max(0, result["l3_call_count"] - 1)
    result["fallback_count"] = 1 if _truth(result.get("fallback_used")) else 0
    result["fallback_trace"] = result.get("fallback_reason", "")
    result["topology_cache_hit"] = bool(topology_info.get("topology_cache_hit"))
    result["topology_load_wall_ms"] = _numeric(topology_info.get("topology_load_time_ms"), 0.0)
    result["heading_jump_count"] = result.get("heading_discontinuity_count", 0)
    result["reverse_length_m"] = result.get("reverse_distance_m", 0.0)
    result["goal_yaw_error_deg"] = math.degrees(_numeric(result.get("goal_yaw_error_rad"), 0.0) or 0.0)
    points = []
    if result.get("path_file"):
        try:
            points = json.loads((output / str(result["path_file"])).read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            points = []
    result["path_point_count"] = len(points)
    length = _numeric(result.get("path_length_m"))
    distance = math.hypot(float(query.goal[0]) - float(query.start[0]), float(query.goal[1]) - float(query.start[1]))
    result["euclidean_ratio"] = length / distance if length is not None and distance > 1.0e-9 else None
    result["reference_ratio"] = "not_available"; result["mean_clearance_m"] = "not_available"; result["heading_change_rate_p95"] = "not_available"
    turns = [abs(legacy._delta(float(second.get("yaw", 0.0)), float(first.get("yaw", 0.0)))) for first, second in zip(points, points[1:])]
    result["total_heading_change_rad"] = sum(turns); result["large_turn_count"] = sum(value > math.radians(45.0) for value in turns)
    return result


def _annotate_call(row: Mapping[str, Any], call: Mapping[str, Any], query: Any) -> Dict[str, Any]:
    result = dict(call)
    result.update({
        "experiment_id": row["experiment_id"], "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE,
        "protocol_version": PROTOCOL_VERSION, "query_set_id": QUERY_SET_ID,
        "case_id": query.query_id, "l2_called": False, "l2_call_count": 0,
        "l3_call_count": row.get("l3_call_count", 0), "query_sha256": row.get("query_sha256", row.get("query_hash", "")),
        "action_success": row.get("action_success"), "final_valid_success": row.get("final_valid_success"),
        "result_code": row.get("result_code"), "reason_code": row.get("reason_code"),
    })
    return result


def _path_metric(output: Path, row: Mapping[str, Any], query: Any) -> Dict[str, Any]:
    metric = {key: value for key, value in row.items() if key in {
        "run_id", "query_id", "query_hash", "static_footprint_valid", "kinematic_valid", "path_length_m",
        "minimum_clearance_m", "curvature_p95", "maximum_curvature", "heading_discontinuity_count", "reverse_distance_m",
        "in_place_rotation_count", "position_discontinuity_count", "steering_jump_count", "start_position_error_m",
        "start_yaw_error_rad", "goal_position_error_m", "goal_yaw_error_rad", "failure_code", "failure_detail",
        "final_validation_time_ms", "final_valid_success", "path_hash", "path_file",
    }}
    metric.update({"experiment_id": row["experiment_id"], "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "protocol_version": PROTOCOL_VERSION, "case_id": query.query_id, "action_success": row.get("action_success"), "result_code": row.get("result_code"), "reason_code": row.get("reason_code"), "query_sha256": row.get("query_sha256", ""), "heading_jump_count": row.get("heading_jump_count", 0), "reverse_length_m": row.get("reverse_length_m", 0.0)})
    return metric


def _report(output: Path, rows: Sequence[Mapping[str, Any]], metadata: Mapping[str, Any], topology_info: Mapping[str, Any], session_info: Mapping[str, Any], cache_mode: str, source_hash: str) -> Dict[str, Any]:
    measured = [row for row in rows if row.get("run_mode") == "measured"]
    valid = sum(_truth(row.get("final_valid_success")) for row in measured)
    failures = collections.Counter(str(row.get("reason_code") or row.get("failure_code") or "") for row in measured if row.get("reason_code") or row.get("failure_code"))
    static_collision_cases = sum(
        str(row.get("reason_code") or row.get("failure_code") or "") == "STATIC_FOOTPRINT_COLLISION"
        or (_truth(row.get("action_success")) and not _truth(row.get("static_footprint_valid")))
        for row in measured
    )
    kinematic_invalid_cases = sum(
        str(row.get("reason_code") or row.get("failure_code") or "") == "KINEMATIC_INVALID"
        or (_truth(row.get("action_success")) and not _truth(row.get("kinematic_valid")))
        for row in measured
    )
    v0 = _reference_summary(V0_REFERENCE)
    corner_by_query = {}
    for row in measured:
        query_id = str(row.get("query_id") or "")
        if query_id not in corner_by_query:
            corner_by_query[query_id] = {
                "count": int(_numeric(row.get("corner_count"), 0) or 0),
                "nodes": row.get("corner_node_ids", []),
                "edges": row.get("corner_edge_ids", []),
                "intervals": row.get("corner_support_intervals_m", []),
            }
    fmt = lambda value, digits=2: "not_available" if value is None else f"{float(value):.{digits}f}"
    report = [
        f"# {ARCHITECTURE_ID}-{IMPLEMENTATION_REVISION} formal experiment",
        "",
        "Independent static 20-query experiment on `mentor_map_20260825_005`; this is not a multi-map conclusion.",
        "",
        f"- Architecture: `{ARCHITECTURE_ID}`; parent=`{PARENT_ARCHITECTURE}`; revision=`{IMPLEMENTATION_REVISION}`; protocol=`{PROTOCOL_VERSION}`.",
        f"- Layers: L1 skeleton topology + Graph A*; L2 disabled; L3' full-corridor Smac Hybrid DUBIN.",
        f"- Map/query validation: JSON/CSV/scenario poses matched ({metadata.get('json_task_count')}/{metadata.get('csv_task_count')}/20); resolution=0.05 m/cell; dynamic_obstacles=false.",
        f"- Corridor: semantics=`{CORRIDOR_SEMANTICS}`; base=2.0 m; corner=4.0 m; curvature window=0.5 m; threshold=1.0 1/m; support extension=1.0 m; 6 m disabled.",
        f"- Cache mode=`{cache_mode}`; cache hit={bool(topology_info.get('topology_cache_hit'))}; build/load={topology_info.get('topology_build_count', 0)}/{topology_info.get('topology_load_count', 0)}; cache bytes={topology_info.get('topology_cache_bytes', 'not_available')}; load wall={fmt(topology_info.get('topology_load_time_ms'))} ms.",
        f"- Smac session start/close/restart={session_info.get('session_start_count', 0)}/{session_info.get('session_close_count', 0)}/{session_info.get('session_restart_count', 0)}; startup/shutdown={fmt(session_info.get('session_startup_time_ms'))}/{fmt(session_info.get('session_shutdown_time_ms'))} ms.",
        "",
        "## Results",
        "",
        f"- Measured final-valid: **{valid}/{len(measured)} ({100.0 * valid / len(measured) if measured else 0.0:.1f}%)**; query-any-valid={sum(any(_truth(row.get('final_valid_success')) for row in measured if row.get('query_id') == query_id) for query_id in sorted({row.get('query_id') for row in measured}))}/20; query-all-repeat-valid={sum(all(_truth(row.get('final_valid_success')) for row in measured if row.get('query_id') == query_id) for query_id in sorted({row.get('query_id') for row in measured}))}/20.",
        f"- Online wall P50/P95/P99={fmt(_percentile(measured, 'online_wall_ms', 50))}/{fmt(_percentile(measured, 'online_wall_ms', 95))}/{fmt(_percentile(measured, 'online_wall_ms', 99))} ms; CPU P50/P95/P99={fmt(_percentile(measured, 'cpu_ms', 50))}/{fmt(_percentile(measured, 'cpu_ms', 95))}/{fmt(_percentile(measured, 'cpu_ms', 99))} ms.",
        f"- RSS P50/P95/P99={fmt(_percentile(measured, 'RSS', 50), 0)}/{fmt(_percentile(measured, 'RSS', 95), 0)}/{fmt(_percentile(measured, 'RSS', 99), 0)} bytes; PSS P50/P95/P99={fmt(_percentile(measured, 'PSS', 50), 0)}/{fmt(_percentile(measured, 'PSS', 95), 0)}/{fmt(_percentile(measured, 'PSS', 99), 0)} bytes.",
        f"- Calls: L1={sum(int(_numeric(row.get('l1_call_count'), 0) or 0) for row in measured)}, L2={sum(int(_numeric(row.get('l2_call_count'), 0) or 0) for row in measured)}, L3'={sum(int(_numeric(row.get('l3_call_count'), 0) or 0) for row in measured)}; Smac retries={sum(int(_numeric(row.get('l3_retry_count'), 0) or 0) for row in measured)}; fallbacks={sum(int(_numeric(row.get('fallback_count'), 0) or 0) for row in measured)}.",
        f"- Mean layer time: L1={fmt(np.mean([_numeric(row.get('l1_time_ms'), 0.0) or 0.0 for row in measured]) if measured else None)} ms; L2=0.00 ms (disabled); L3'={fmt(np.mean([_numeric(row.get('l3_time_ms'), 0.0) or 0.0 for row in measured]) if measured else None)} ms; corridor mask={fmt(np.mean([_numeric(row.get('corridor_mask_total_time_ms'), 0.0) or 0.0 for row in measured]) if measured else None)} ms.",
        f"- Corner diagnostics: mean count={fmt(np.mean([_numeric(row.get('corner_count'), 0.0) or 0.0 for row in measured]) if measured else None)}; max curvature={fmt(max((_numeric(row.get('corner_max_curvature_1pm')) for row in measured if _numeric(row.get('corner_max_curvature_1pm')) is not None), default=None), 4)} 1/m; mean widened area ratio={fmt(np.mean([_numeric(row.get('corner_widened_area_ratio'), 0.0) or 0.0 for row in measured]) if measured else None, 6)}.",
        f"- High-curvature support locations are recorded per query in `corner_node_ids` and `corner_edge_ids` (with support intervals when exposed); representative first measured records: `{json.dumps(corner_by_query, sort_keys=True, default=str)}`.",
        f"- Path quality: mean length={fmt(np.mean([_numeric(row.get('path_length_m')) for row in measured if _numeric(row.get('path_length_m')) is not None]) if measured else None)} m; mean minimum clearance={fmt(np.mean([_numeric(row.get('minimum_clearance_m')) for row in measured if _numeric(row.get('minimum_clearance_m')) is not None]) if measured else None)} m; maximum curvature={fmt(max((_numeric(row.get('maximum_curvature')) for row in measured if _numeric(row.get('maximum_curvature')) is not None), default=None), 4)} 1/m; heading/position/steering discontinuities={sum(int(_numeric(row.get('heading_discontinuity_count'), 0) or 0) for row in measured)}/{sum(int(_numeric(row.get('position_discontinuity_count'), 0) or 0) for row in measured)}/{sum(int(_numeric(row.get('steering_jump_count'), 0) or 0) for row in measured)}.",
        f"- Hard validation totals: static collision cases={static_collision_cases}, kinematic-invalid cases={kinematic_invalid_cases}, reverse distance={fmt(sum(_numeric(row.get('reverse_length_m'), 0.0) or 0.0 for row in measured))} m, in-place rotations={sum(int(_numeric(row.get('in_place_rotation_count'), 0) or 0) for row in measured)}.",
        f"- Failure distribution: `{dict(failures)}`.",
        "",
        "## Comparison and decision",
        "",
        "The 2A-V1 change is limited to topology-turn-adaptive corridor width. It does not call `plan_grid_astar`, use 6 m, use fallback backends, or alter Smac poses/yaw/curvature.",
        f"Retained 2A-V0-r3 reference: `{v0['directory']}`; available={v0.get('available', False)}; measured={v0.get('valid_count', 'not_available')}/{v0.get('measured_count', 'not_available')} final-valid; query-any={v0.get('query_any_valid', 'not_available')}/20; query-all-repeat={v0.get('query_all_repeat_valid', 'not_available')}/20.",
        f"2A-V0-r3 wall P50/P95/P99={fmt(v0.get('wall_p50'))}/{fmt(v0.get('wall_p95'))}/{fmt(v0.get('wall_p99'))} ms; CPU P50/P95/P99={fmt(v0.get('cpu_p50'))}/{fmt(v0.get('cpu_p95'))}/{fmt(v0.get('cpu_p99'))} ms; L1 P50={fmt(v0.get('l1_p50'))} ms; L3' P50={fmt(v0.get('l3_p50'))} ms.",
        f"2A-V0-r3 path reference mean length/clearance/max curvature={fmt(v0.get('path_length_mean'))} m/{fmt(v0.get('clearance_mean'))} m/{fmt(v0.get('max_curvature'), 4)} 1/m; failures={v0.get('failure_counts', {})}.",
        "This is a retained historical reference rather than a same-process paired rerun; differences are reported as observations, not proof of superiority. If validity or latency does not improve under equivalent conditions, the decision remains: `2A-V1 局部转角走廊策略暂未证明优于 2A-V0-r3`.",
        f"- Source hash: `{source_hash}`; all returned paths remain in `paths/` and are validated without field rewriting.",
    ]
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    summary = {
        "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "final_valid_count": valid, "measured_count": len(measured), "final_valid_rate": valid / len(measured) if measured else 0.0,
        "online_p50_ms": _percentile(measured, "online_wall_ms", 50), "online_p95_ms": _percentile(measured, "online_wall_ms", 95), "online_p99_ms": _percentile(measured, "online_wall_ms", 99),
        "failure_counts": dict(failures), "l1_call_count": sum(int(_numeric(row.get("l1_call_count"), 0) or 0) for row in measured), "l2_call_count": 0,
        "l3_prime_call_count": sum(int(_numeric(row.get("l3_call_count"), 0) or 0) for row in measured), "fallback_count": sum(int(_numeric(row.get("fallback_count"), 0) or 0) for row in measured),
        "gate_passed": valid == len(measured), "corner_count_total": sum(int(_numeric(row.get("corner_count"), 0) or 0) for row in measured),
    }
    _write_csv(output / "summary.csv", [summary])
    failure_rows = [{"failure_code": code, "count": count} for code, count in sorted(failures.items())]
    if failure_rows:
        _write_csv(output / "failure_summary.csv", failure_rows)
    else:
        (output / "failure_summary.csv").write_text("failure_code,count\n", encoding="utf-8")
    return summary


def run_formal(
    output: Path, *, cache_mode: str = "optimized", warmups: int = WARMUPS,
    repetitions: int = REPETITIONS, query_ids: Optional[Sequence[str]] = None,
    ros_domain_id: int = ROS_DOMAIN_ID, topology_cache_dir: Optional[Path] = None,
) -> Path:
    if cache_mode not in {candidate.CACHE_MODE_BASELINE, candidate.CACHE_MODE_OPTIMIZED}:
        raise ValueError("cache_mode must be baseline or optimized")
    if warmups < 0 or repetitions <= 0:
        raise ValueError("warmups must be >= 0 and repetitions > 0")
    _refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries, metadata = _load_tasks()
    selected = list(query_ids or [query.query_id for query in queries])
    query_map = {query.query_id: query for query in queries}
    if any(item not in query_map for item in selected):
        raise ValueError("query_ids must be A2B-01..A2B-20")
    queries = [query_map[item] for item in selected]
    ctx = _context()
    cache_root = (topology_cache_dir or DEFAULT_CACHE_ROOT).resolve()
    topology, topology_info = _load_or_build_topology(ctx, output, cache_root)
    source_files, source_hash = _source_manifest()
    spec = legacy.backend_availability()["hybrid_astar"]
    if not spec.available:
        raise RuntimeError(f"BACKEND_UNAVAILABLE: {spec.reason}")
    os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
    session = candidate.SmacSession(ctx, output, map_yaml=validity.MAP_YAML, log_tag=f"formal_2a_v1_r0_{MAP_ID}", local_mask_updates=True, optimization_profile=OPTIMIZATION_PROFILE, smac_parameter_profile=SMAC_PARAMETER_PROFILE, optimization_stage=OPTIMIZATION_STAGE)
    session.start()
    rows: List[Dict[str, Any]] = []; calls: List[Dict[str, Any]] = []; metrics: List[Dict[str, Any]] = []
    try:
        for run_mode, count in (("warmup", warmups), ("measured", repetitions)):
            for repetition in range(1, int(count) + 1):
                for query in queries:
                    row, call, metric = candidate._run_one(
                        ctx, topology, topology_info, query, run_mode, repetition, session, spec,
                        output, validity._source_commit(), corridor_padding_m=BASE_CORRIDOR_PADDING_M,
                        corridor_semantics=CORRIDOR_SEMANTICS, profile_name=CORRIDOR_PROFILE,
                        padding_schedule_m=(BASE_CORRIDOR_PADDING_M,), force_full_update=True,
                        validate_each_attempt=True, cache_mode=cache_mode,
                        corridor_mask_builder=build_adaptive_corridor_mask,
                    )
                    annotated = _annotate_row(output, row, query, metadata, topology_info, cache_mode)
                    annotated["source_hash"] = source_hash
                    annotated["query_sha256"] = annotated.get("query_hash", "")
                    annotated_call = _annotate_call(annotated, call, query)
                    metric_row = _path_metric(output, annotated, query)
                    rows.append(annotated); calls.append(annotated_call); metrics.append(metric_row)
    finally:
        session.close()
    session_info = {
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION, "map_id": MAP_ID, "ros_domain_id": ros_domain_id,
        "session_start_count": session.session_start_count, "session_close_count": session.session_close_count, "session_restart_count": session.session_restart_count,
        "session_startup_time_ms": session.stack_startup_time_ms, "session_shutdown_time_ms": session.stack_shutdown_time_ms,
        "topology_build_count": topology_info.get("topology_build_count", 0), "topology_load_count": topology_info.get("topology_load_count", 0),
        "topology_build_wall_ms": topology_info.get("topology_build_time_ms", 0.0), "topology_build_cpu_ms": topology_info.get("topology_build_cpu_ms", 0.0),
        "topology_load_wall_ms": topology_info.get("topology_load_time_ms", 0.0), "topology_cache_hit": topology_info.get("topology_cache_hit", False),
    }
    _write_csv(output / "runs.csv", rows); _write_csv(output / "path_metrics.csv", metrics); _write_csv(output / "backend_call_log.csv", calls); _write_csv(output / "session_timing.csv", [session_info])
    (output / "topology_cache_manifest.yaml").write_text(yaml.safe_dump({**topology_info, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "cache_mode": cache_mode}, sort_keys=False), encoding="utf-8")
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "protocol_version": PROTOCOL_VERSION, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE,
        "map_id": MAP_ID, "query_set_id": QUERY_SET_ID, "query_ids": [query.query_id for query in queries], "seed": SEED, "warmups": warmups, "repetitions": repetitions,
        "resolution_m": 0.05, "dynamic_obstacles": False, "minimum_turning_radius_m": 0.40, "maximum_curvature_1pm": 2.50, "allow_reverse": False, "allow_in_place_rotation": False,
        "layers": {"L1": "skeleton topology + Graph A*", "L2": "disabled", "L3_prime": "corridor-wide Smac Hybrid DUBIN"},
        "corridor_semantics": CORRIDOR_SEMANTICS, "corridor_profile": CORRIDOR_PROFILE, "base_corridor_padding_m": BASE_CORRIDOR_PADDING_M, "corner_corridor_padding_m": CORNER_CORRIDOR_PADDING_M,
        "corner_curvature_window_m": CURVATURE_WINDOW_M, "corner_curvature_threshold_1pm": CORNER_CURVATURE_THRESHOLD, "corner_support_extension_m": CORNER_SUPPORT_EXTENSION_M, "six_meter_padding_used": False,
        "cache_mode": cache_mode, "smac_parameter_profile": SMAC_PARAMETER_PROFILE, "optimization_profile": OPTIMIZATION_PROFILE, "optimization_stage": OPTIMIZATION_STAGE,
        "l2_called": False, "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0,
        "metric_availability": {"expanded_generated_states": "not_available: Smac client does not expose state counters", "mean_clearance_m": "not_available: validator exposes minimum only", "heading_change_rate_p95": "not_available: no temporal sampling"},
    }, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({"experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "protocol_version": PROTOCOL_VERSION, "source_commit": validity._source_commit(), "source_hash": source_hash, "source_files": source_files, "map_id": MAP_ID, "map_sha256": ctx.map_sha256, "map_yaml_sha256": ctx.map_yaml_sha256, "query_sha256": {query.query_id: paired._query_hash(query) for query in queries}, "footprint_hash": _json_hash(legacy.FOOTPRINT)}, sort_keys=False), encoding="utf-8")
    summary = _report(output, rows, metadata, topology_info, session_info, cache_mode, source_hash)
    (output / "manifest.yaml").write_text(yaml.safe_dump({
        "experiment_id": output.name, "architecture_id": ARCHITECTURE_ID, "implementation_revision": IMPLEMENTATION_REVISION, "parent_architecture": PARENT_ARCHITECTURE, "protocol_version": PROTOCOL_VERSION,
        "map_id": MAP_ID, "query_set_id": QUERY_SET_ID, "query_ids": [query.query_id for query in queries], "warmup_count": warmups, "measured_repetitions": repetitions, "run_count": len(rows),
        "cache_mode": cache_mode, "topology_build_count": topology_info.get("topology_build_count", 0), "topology_load_count": topology_info.get("topology_load_count", 0), "topology_cache_hit": topology_info.get("topology_cache_hit", False), "topology_cache_key": topology_info.get("topology_cache_key", ""), "topology_cache_bytes": topology_info.get("topology_cache_bytes", "not_available"),
        "session_start_count": session_info["session_start_count"], "session_close_count": session_info["session_close_count"], "session_restart_count": session_info["session_restart_count"], "l2_called": False, "l2_call_count": 0, "rrtstar_call_count": 0, "sst_call_count": 0,
        "corridor_profile": CORRIDOR_PROFILE, "corridor_semantics": CORRIDOR_SEMANTICS, "base_corridor_padding_m": BASE_CORRIDOR_PADDING_M, "corner_corridor_padding_m": CORNER_CORRIDOR_PADDING_M, "six_meter_padding_used": False, "source_hash": source_hash, "metric_availability": "see protocol.yaml", **summary,
    }, sort_keys=False), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run independent 2A-V1-r0 topology-turn-adaptive corridor Hybrid benchmark")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--topology-cache-dir", default=str(DEFAULT_CACHE_ROOT))
    parser.add_argument("--cache-mode", choices=(candidate.CACHE_MODE_BASELINE, candidate.CACHE_MODE_OPTIMIZED), default=candidate.CACHE_MODE_OPTIMIZED)
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--query-id", action="append", dest="query_ids", help="bounded preflight subset")
    parser.add_argument("--ros-domain-id", type=int, default=ROS_DOMAIN_ID)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_formal(Path(args.output_dir).resolve(), cache_mode=args.cache_mode, warmups=args.warmups, repetitions=args.repetitions, query_ids=args.query_ids, ros_domain_id=args.ros_domain_id, topology_cache_dir=Path(args.topology_cache_dir).resolve())
    except Exception as exc:
        print(f"2a_v1_formal_benchmark: ERROR: {exc}")
        return 2
    print(f"2A-V1-r0 output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
