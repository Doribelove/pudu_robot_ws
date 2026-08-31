"""Independent two-layer candidate smoke.

The candidate is deliberately narrower than the formal three-layer planner:
L1 selects a persisted skeleton route and its corridor mask, then one real
Nav2 Smac Hybrid DUBIN request searches the complete corridor from A to B.
There is no intermediate geometric path generator and no fallback backend.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
import resource
import subprocess
import time
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from . import fixed_layered_pipeline_smoke as fixed
from . import unified_four_backends_smoke as legacy
from .planner_benchmark.map_utils import sha256_file
from .planner_benchmark.models import Query
from .topology import (
    NodeSpatialIndex,
    TopologyArtifact,
    corridor_mask,
    load_topology,
    search_topology_multi_goal_timed,
)


ROOT = legacy.ROOT
OUTPUT_NAME = "l1_l3_corridor_hybrid_smoke_v1"
SOURCE_QUERIES = legacy.SOURCE_QUERIES
MAP_PATHS = legacy.MAP_PATHS
DEFAULT_MAP_IDS = ("hospital_005",)
RAW_QUERY_IDS = ("q02", "q06", "q07", "q09")
DIAGNOSTIC_QUERY_IDS = ("q00",)
FOOTPRINT = legacy.FOOTPRINT
ARCHITECTURE = "l1_l3_corridor_hybrid"
L1_BACKEND = legacy.TOPOLOGY_ALGORITHM_VERSION
L3_PRIME_SOURCE = "l3_prime_corridor_hybrid"
CORRIDOR_PADDING_M = 1.0
CORRIDOR_PROFILES_M = (1.0, 2.0, 4.0)
VALIDITY_PADDING_SCHEDULE_M = (2.0, 4.0, 6.0)
VALIDITY_PROFILE_BASELINE = "v2_repair"
VALIDITY_PROFILE_EXPAND = "bounded_corridor_expansion"
CORRIDOR_SEMANTICS = "raw_map_smac_aligned"
FOOTPRINT_SAFETY_MARGIN_M = 0.05
BEND_MARGIN_M = 0.15
ENDPOINT_CONNECT_BACK_M = 0.25
ENDPOINT_CONNECT_FORWARD_M = 0.75
WARMUPS = 1
REPETITIONS = 3
TIMEOUT_S = 5.0
CACHE_MODE_BASELINE = "baseline"
CACHE_MODE_OPTIMIZED = "optimized"
ENDPOINT_SPATIAL_INDEX_CELL_M = 5.0
OPTIMIZED_MAX_CANDIDATES = 32
V7_TOPOLOGY_CACHE = ROOT / (
    "experiments/layered_planner_benchmark/"
    "fixed_layered_pipeline_v7_online_efficiency_postfix5_final/topology_cache"
)


# Exposed aliases make the boundary easy to replace in unit tests without
# changing the formal three-layer entry point.
SmacSession = legacy.SmacSession
BackendSpec = legacy.BackendSpec
PlanResult = legacy.PlanResult


def _truth(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _query_hash(query: Query) -> str:
    payload = json.dumps(
        {"query_id": query.query_id, "start": list(query.start),
         "goal": list(query.goal), "seed": query.seed},
        sort_keys=True, separators=(",", ":"), allow_nan=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _grid_hash(mask: np.ndarray) -> str:
    return hashlib.sha256(
        np.ascontiguousarray(np.asarray(mask, dtype=np.uint8)).tobytes()
    ).hexdigest()


def _raw_free_mask(ctx: legacy.MapContext) -> np.ndarray:
    """Return raw-map traversable cells; footprint inflation belongs to Smac."""
    cached = getattr(ctx, "raw_free_mask_cache", None)
    if cached is not None:
        return np.asarray(cached, dtype=bool)
    occupancy = np.asarray(ctx.hospital_map.occupancy)
    mask = (occupancy >= 0) & (occupancy < 100)
    try:
        setattr(ctx, "raw_free_mask_cache", mask)
    except (AttributeError, TypeError):
        pass
    return mask


def _draw_world_segment(
    ctx: legacy.MapContext, image: np.ndarray,
    first: Sequence[float], second: Sequence[float],
) -> None:
    first_cell = ctx.hospital_map.world_to_cell(float(first[0]), float(first[1]))
    second_cell = ctx.hospital_map.world_to_cell(float(second[0]), float(second[1]))
    if first_cell is None or second_cell is None:
        return
    # OpenCV uses (column, row), while HospitalMap cells are (row, column).
    cv2.line(
        image,
        (int(first_cell[1]), int(first_cell[0])),
        (int(second_cell[1]), int(second_cell[0])),
        1,
        thickness=1,
        lineType=cv2.LINE_8,
    )


def _raw_corridor_mask(
    ctx: legacy.MapContext,
    topology: TopologyArtifact,
    route: Any,
    query: Query,
    padding_m: float,
) -> np.ndarray:
    """Rasterize the L1 route and dilate only raw free cells.

    The route is still selected from the footprint-aware L1 topology.  The
    mask passed to Smac is deliberately based on the raw occupancy map so the
    static layer/inflation layer is the single owner of footprint inflation.
    Endpoint connection strips provide fixed heading-aligned room without
    changing either endpoint pose.
    """
    raw_free = _raw_free_mask(ctx)
    centerline = getattr(route, "_corridor_centerline_cache", None)
    if centerline is None or np.asarray(centerline).shape != raw_free.shape:
        centerline = np.zeros_like(raw_free, dtype=np.uint8)
        polyline = list(getattr(route, "polyline", []) or [])
        for first, second in zip(polyline, polyline[1:]):
            _draw_world_segment(ctx, centerline, first, second)
        for point in polyline:
            cell = ctx.hospital_map.world_to_cell(float(point[0]), float(point[1]))
            if cell is not None:
                centerline[cell] = 1
        try:
            setattr(route, "_corridor_centerline_cache", centerline)
        except (AttributeError, TypeError):
            pass
    start = query.start
    goal = query.goal
    start_dir = (math.cos(float(start[2])), math.sin(float(start[2])))
    goal_dir = (math.cos(float(goal[2])), math.sin(float(goal[2])))
    _draw_world_segment(
        ctx, centerline,
        (start[0] - ENDPOINT_CONNECT_BACK_M * start_dir[0],
         start[1] - ENDPOINT_CONNECT_BACK_M * start_dir[1]),
        (start[0] + ENDPOINT_CONNECT_FORWARD_M * start_dir[0],
         start[1] + ENDPOINT_CONNECT_FORWARD_M * start_dir[1]),
    )
    _draw_world_segment(
        ctx, centerline,
        (goal[0] - ENDPOINT_CONNECT_FORWARD_M * goal_dir[0],
         goal[1] - ENDPOINT_CONNECT_FORWARD_M * goal_dir[1]),
        (goal[0] + ENDPOINT_CONNECT_BACK_M * goal_dir[0],
         goal[1] + ENDPOINT_CONNECT_BACK_M * goal_dir[1]),
    )
    start_cell, goal_cell = _endpoint_cells(ctx, query)
    if start_cell is not None:
        centerline[start_cell] = 1
    if goal_cell is not None:
        centerline[goal_cell] = 1
    effective_radius_m = max(0.0, float(padding_m)) + FOOTPRINT_SAFETY_MARGIN_M + BEND_MARGIN_M
    radius_cells = max(1, int(math.ceil(effective_radius_m / float(ctx.hospital_map.resolution))))
    kernel_cache = getattr(ctx, "corridor_kernel_cache", None)
    if kernel_cache is None:
        kernel_cache = {}
        try:
            setattr(ctx, "corridor_kernel_cache", kernel_cache)
        except (AttributeError, TypeError):
            pass
    kernel_key = (int(radius_cells), float(ctx.hospital_map.resolution))
    kernel = kernel_cache.get(kernel_key)
    if kernel is None:
        diameter = 2 * radius_cells + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter))
        kernel_cache[kernel_key] = kernel

    # Dilation is mathematically local.  Crop to the route envelope plus the
    # kernel radius, then place the result back into the full-size mask needed
    # by the existing Smac costmap interface.  This avoids scanning the empty
    # map with OpenCV for every retry while preserving exact border behavior.
    cells = np.argwhere(centerline)
    if cells.size == 0:
        return np.zeros_like(raw_free, dtype=bool)
    min_row = max(0, int(cells[:, 0].min()) - radius_cells)
    max_row = min(raw_free.shape[0], int(cells[:, 0].max()) + radius_cells + 1)
    min_col = max(0, int(cells[:, 1].min()) - radius_cells)
    max_col = min(raw_free.shape[1], int(cells[:, 1].max()) + radius_cells + 1)
    roi = centerline[min_row:max_row, min_col:max_col]
    expanded_roi = cv2.dilate(roi, kernel, iterations=1).astype(bool)
    result = np.zeros_like(raw_free, dtype=bool)
    result[min_row:max_row, min_col:max_col] = (
        expanded_roi & raw_free[min_row:max_row, min_col:max_col]
    )
    return result


def _build_corridor_mask(
    ctx: legacy.MapContext,
    topology: TopologyArtifact,
    route: Any,
    query: Query,
    start_cell: Any,
    goal_cell: Any,
    padding_m: float,
    semantics: str,
) -> np.ndarray:
    if semantics == "raw_map_smac_aligned":
        return _raw_corridor_mask(ctx, topology, route, query, padding_m)
    if semantics == "inflated_l1_legacy":
        return corridor_mask(topology, route, start_cell, goal_cell, float(padding_m))
    if semantics == "raw_full_map":
        return _raw_free_mask(ctx)
    raise ValueError(f"unsupported corridor semantics: {semantics}")


def _profile_name(semantics: str, padding_m: Optional[float]) -> str:
    if semantics == "raw_full_map":
        return "raw_full_map_smac"
    if padding_m is None:
        return semantics
    return f"{semantics}_{float(padding_m):g}m"


def _source_commit() -> Optional[str]:
    try:
        value = subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"],
            text=True, stderr=subprocess.DEVNULL,
        ).strip()
        return value or None
    except (OSError, subprocess.SubprocessError):
        return None


def _queries() -> Dict[str, Query]:
    payload = yaml.safe_load(SOURCE_QUERIES.read_text(encoding="utf-8")) or {}
    return {
        str(item["query_id"]): Query(
            query_id=str(item["query_id"]),
            start=[float(value) for value in item["start"]],
            goal=[float(value) for value in item["goal"]],
            category=str(item.get("category", "unspecified")),
            seed=int(item.get("seed", payload.get("seed", 0))),
            validation_status=str(item.get("validation_status", "UNVALIDATED")),
        )
        for item in payload.get("queries", [])
    }


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _write_csv(
    path: Path, rows: Iterable[Mapping[str, Any]], fields: Optional[Sequence[str]] = None,
) -> None:
    materialized = list(rows)
    field_names: List[str] = list(fields or [])
    for row in materialized:
        for key in row:
            if key not in field_names:
                field_names.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=field_names or ["empty"])
        writer.writeheader()
        for row in materialized:
            writer.writerow({
                key: json.dumps(value, sort_keys=True, default=str)
                if isinstance(value, (dict, list)) else value
                for key, value in row.items()
            })


def _source_manifest(output: Path, source_commit: Optional[str]) -> Tuple[Dict[str, str], str]:
    files = [
        Path(__file__).resolve(),
        Path(fixed.__file__).resolve(),
        Path(legacy.__file__).resolve(),
        Path(__file__).resolve().parent / "topology.py",
        Path(__file__).resolve().parents[1] / "setup.py",
        SOURCE_QUERIES,
        legacy._strict_smac_config_path(),
    ]
    file_hashes = {str(path): sha256_file(path) for path in files if path.exists()}
    code_hash = hashlib.sha256(
        "\n".join(f"{key}\0{value}" for key, value in sorted(file_hashes.items())).encode()
    ).hexdigest()
    return file_hashes, code_hash


def _cache_metadata_matches(
    metadata: Mapping[str, Any], expected: Mapping[str, Any],
) -> bool:
    # Existing V7 cache metadata predates this candidate.  Source commit may
    # differ because the worktree contains later, unrelated edits; all
    # algorithm, map, footprint and source-content keys remain exact.
    for key, value in expected.items():
        if key == "source_commit":
            continue
        if metadata.get(key) != value:
            return False
    return True


def _load_authoritative_topology(
    map_id: str,
    ctx: legacy.MapContext,
    cache_root: Path,
    source_commit: Optional[str],
    source_hash: str,
    fallback_root: Path,
) -> Tuple[TopologyArtifact, Dict[str, Any]]:
    """Load a V7-compatible cache without mutating its historical directory."""
    expected = fixed._topology_cache_expected(map_id, ctx, source_commit, source_hash)
    started_ns = time.monotonic_ns()
    candidates = sorted((cache_root / map_id).glob("*/cache_manifest.yaml"))
    for manifest_path in candidates:
        try:
            payload = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
            metadata = payload.get("metadata") or {}
            cache_key = payload.get("cache_key", manifest_path.parent.name)
            if payload.get("cache_key") != cache_key or not _cache_metadata_matches(metadata, expected):
                continue
            artifact = load_topology(
                manifest_path.parent, ctx.hospital_map, FOOTPRINT,
                padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
            )
            return artifact, {
                **expected,
                "topology_cache_key": str(cache_key),
                "topology_cache_hit": True,
                "topology_build_count": 0,
                "topology_load_count": 1,
                "topology_build_time_ms": 0.0,
                "topology_load_time_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                "cache_directory": str(manifest_path.parent),
                "cache_source_commit": metadata.get("source_commit", "unknown"),
                "skeleton_backend": artifact.metadata.get("skeleton_backend", "unknown"),
            }
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
            continue

    # A missing or stale historical cache is rebuilt only under the new
    # experiment directory.  This keeps all existing experiment evidence
    # immutable while preserving exact metadata binding.
    fallback_root.mkdir(parents=True, exist_ok=True)
    artifact, info = fixed._load_or_build_topology_cache(
        map_id, ctx, fallback_root, source_commit, source_hash,
    )
    info["skeleton_backend"] = artifact.metadata.get("skeleton_backend", "unknown")
    return artifact, info


def _path_within_mask(
    ctx: legacy.MapContext,
    points: Sequence[Mapping[str, Any]],
    allowed_mask: np.ndarray,
) -> bool:
    if not points:
        return False
    mask = np.asarray(allowed_mask, dtype=bool)
    if mask.shape != ctx.hospital_map.occupancy.shape:
        return False
    spacing = max(0.01, ctx.hospital_map.resolution * 0.5)
    for first, second in zip(points, points[1:]):
        dx = float(second["x"]) - float(first["x"])
        dy = float(second["y"]) - float(first["y"])
        steps = max(1, int(math.ceil(math.hypot(dx, dy) / spacing)))
        for step in range(steps + 1):
            fraction = step / steps
            cell = ctx.hospital_map.world_to_cell(
                float(first["x"]) + fraction * dx,
                float(first["y"]) + fraction * dy,
            )
            if cell is None or not bool(mask[cell]):
                return False
    if len(points) == 1:
        cell = ctx.hospital_map.world_to_cell(float(points[0]["x"]), float(points[0]["y"]))
        return cell is not None and bool(mask[cell])
    return True


def _endpoint_cells(ctx: legacy.MapContext, query: Query) -> Tuple[Any, Any]:
    return (
        ctx.hospital_map.world_to_cell(query.start[0], query.start[1]),
        ctx.hospital_map.world_to_cell(query.goal[0], query.goal[1]),
    )


def _get_endpoint_spatial_index(
    topology: TopologyArtifact,
) -> Tuple[Optional[NodeSpatialIndex], bool, Dict[int, bool]]:
    graph = getattr(topology, "graph", None)
    nodes = getattr(graph, "nodes", None)
    if nodes is None:
        return None, False, {}
    index = getattr(topology, "_endpoint_spatial_index", None)
    certificate = getattr(topology, "_endpoint_safety_certificate", None)
    hit = index is not None and certificate is not None
    if index is None:
        index = NodeSpatialIndex.build(nodes, ENDPOINT_SPATIAL_INDEX_CELL_M)
        try:
            setattr(topology, "_endpoint_spatial_index", index)
        except (AttributeError, TypeError):
            pass
    if certificate is None:
        free_mask = np.asarray(getattr(topology, "free_mask", np.zeros((0, 0), dtype=bool)), dtype=bool)
        certificate = {}
        for node in nodes:
            row, column = int(node.pixel_y), int(node.pixel_x)
            certificate[int(node.node_id)] = bool(
                0 <= row < free_mask.shape[0]
                and 0 <= column < free_mask.shape[1]
                and free_mask[row, column]
            )
        try:
            setattr(topology, "_endpoint_safety_certificate", certificate)
        except (AttributeError, TypeError):
            pass
    return index, bool(hit), certificate


def _attachment_candidates(
    topology: TopologyArtifact,
    pose: Sequence[float],
    *,
    max_radius_m: float = 5.0,
    limit: int = 32,
    cache_mode: str = CACHE_MODE_BASELINE,
    timing: Optional[Dict[str, Any]] = None,
) -> List[Any]:
    """Return nearby footprint-safe skeleton nodes in deterministic order."""
    lookup_started = time.monotonic_ns()
    candidates: List[Tuple[float, int, Any]] = []
    graph = getattr(topology, "graph", None)
    nodes = getattr(graph, "nodes", None)
    # Lightweight test doubles and older persisted artifacts may expose only
    # the legacy attach API.  Keep that compatibility path bounded and leave
    # the real topology candidate search above it for production artifacts.
    if nodes is None:
        try:
            attached = legacy.attach_pose(topology, pose, FOOTPRINT)
        except (AttributeError, TypeError, ValueError):
            attached = None
        if timing is not None:
            timing.update({
                "lookup_ms": (time.monotonic_ns() - lookup_started) / 1.0e6,
                "collision_check_ms": 0.0,
                "spatial_index_cache_hit": False,
                "candidate_count": 1 if attached is not None else 0,
            })
        return [attached] if attached is not None else []
    spatial_hit = False
    certificate: Dict[int, bool] = {}
    if cache_mode == CACHE_MODE_OPTIMIZED:
        index, spatial_hit, certificate = _get_endpoint_spatial_index(topology)
        candidate_cache = getattr(topology, "_endpoint_candidate_cache", None)
        if candidate_cache is None:
            candidate_cache = {}
            try:
                setattr(topology, "_endpoint_candidate_cache", candidate_cache)
            except (AttributeError, TypeError):
                pass
        candidate_key = (
            tuple(float(value) for value in pose), float(max_radius_m), int(limit),
        )
        cached_candidates = candidate_cache.get(candidate_key)
        if cached_candidates is not None:
            cached_nodes = [index.nodes_by_id[node_id] for node_id in cached_candidates if index and node_id in index.nodes_by_id]
            if cached_nodes:
                if timing is not None:
                    timing.update({
                        "lookup_ms": 0.0, "collision_check_ms": 0.0,
                        "spatial_index_cache_hit": True, "endpoint_candidate_cache_hit": True,
                        "candidate_count": len(cached_nodes), "scanned_count": 0,
                    })
                return cached_nodes[:max(1, int(limit))]
        search_nodes = index.query(float(pose[0]), float(pose[1]), float(max_radius_m)) if index else list(nodes)
    else:
        search_nodes = list(nodes)
    lookup_ms = (time.monotonic_ns() - lookup_started) / 1.0e6
    collision_started = time.monotonic_ns()
    for node in search_nodes:
        distance = math.hypot(node.x - float(pose[0]), node.y - float(pose[1]))
        if distance > float(max_radius_m):
            continue
        cell = (node.pixel_y, node.pixel_x)
        if cache_mode == CACHE_MODE_OPTIMIZED:
            if not certificate.get(int(node.node_id), False):
                continue
        elif not topology.free_mask[cell]:
            continue
        if topology.hospital_map.footprint_collision(
            pose=(node.x, node.y, float(pose[2])),
            footprint=FOOTPRINT,
            unknown_is_collision=True,
        ):
            continue
        candidates.append((distance, int(node.node_id), node))
    collision_ms = (time.monotonic_ns() - collision_started) / 1.0e6
    candidates.sort(key=lambda item: (item[0], item[1]))
    if cache_mode == CACHE_MODE_OPTIMIZED:
        candidate_cache = getattr(topology, "_endpoint_candidate_cache", None)
        if candidate_cache is not None:
            candidate_cache[candidate_key] = [int(item[1]) for item in candidates[:max(1, int(limit))]]
    if timing is not None:
        timing.update({
            "lookup_ms": float(lookup_ms),
            "collision_check_ms": float(collision_ms),
            "spatial_index_cache_hit": bool(spatial_hit),
            "endpoint_candidate_cache_hit": False,
            "candidate_count": len(candidates),
            "scanned_count": len(search_nodes),
        })
    return [item[2] for item in candidates[:max(1, int(limit))]]


def _select_route_with_endpoint_attach(
    topology: TopologyArtifact,
    query: Query,
    *,
    cache_mode: str = CACHE_MODE_BASELINE,
    timing: Optional[Dict[str, Any]] = None,
) -> Tuple[Optional[Any], Optional[Any], Optional[Any], str]:
    """Choose endpoint attachments that actually admit a graph route.

    A nearest skeleton node can belong to a small disconnected component even
    when another nearby node reaches the goal.  Trying a bounded set of
    footprint-safe candidates is the endpoint attach-edge equivalent of a
    local graph connection and avoids query-specific exceptions.
    """
    start_timing: Dict[str, Any] = {}
    goal_timing: Dict[str, Any] = {}
    starts = _attachment_candidates(topology, query.start, cache_mode=cache_mode, timing=start_timing)
    goals = _attachment_candidates(topology, query.goal, cache_mode=cache_mode, timing=goal_timing)
    if timing is not None:
        timing.update({
            "start_lookup_ms": float(start_timing.get("lookup_ms", 0.0)),
            "goal_lookup_ms": float(goal_timing.get("lookup_ms", 0.0)),
            "start_collision_check_ms": float(start_timing.get("collision_check_ms", 0.0)),
            "goal_collision_check_ms": float(goal_timing.get("collision_check_ms", 0.0)),
            "start_candidate_count": len(starts),
            "goal_candidate_count": len(goals),
            "endpoint_spatial_index_cache_hit": bool(
                start_timing.get("spatial_index_cache_hit", False)
                and goal_timing.get("spatial_index_cache_hit", False)
            ),
            "endpoint_candidate_cache_hit": bool(
                start_timing.get("endpoint_candidate_cache_hit", False)
                and goal_timing.get("endpoint_candidate_cache_hit", False)
            ),
        })
    if not starts or not goals:
        return None, None, None, "endpoint_candidates_empty"
    if cache_mode == CACHE_MODE_OPTIMIZED:
        route_cache = getattr(topology, "_endpoint_route_cache", None)
        if route_cache is None:
            route_cache = {}
            try:
                setattr(topology, "_endpoint_route_cache", route_cache)
            except (AttributeError, TypeError):
                pass
        route_key = (tuple(int(item.node_id) for item in starts), tuple(int(item.node_id) for item in goals))
        cached_route = route_cache.get(route_key)
        if cached_route is not None:
            cached_start_id, cached_goal_id, route = cached_route
            start_by_id = {int(item.node_id): item for item in starts}
            goal_by_id = {int(item.node_id): item for item in goals}
            if cached_start_id in start_by_id and cached_goal_id in goal_by_id:
                if timing is not None:
                    timing.update({
                        "route_search_ms": 0.0, "route_construction_ms": 0.0,
                        "adjacency_build_ms": 0.0, "topology_adjacency_cache_hit": True,
                        "candidate_pair_attempts": 0, "route_cache_hit": True,
                    })
                return start_by_id[cached_start_id], goal_by_id[cached_goal_id], route, "route_cache_hit"
        started = time.monotonic_ns()
        component_goals = {
            int(goal.node_id) for goal in goals
            if any(int(start.component_id) == int(goal.component_id) for start in starts)
        }
        starts_in_components = [
            int(start.node_id) for start in starts
            if any(int(start.component_id) == int(goal.component_id) for goal in goals)
        ]
        if not starts_in_components or not component_goals:
            if timing is not None:
                timing.update({"route_search_ms": 0.0, "route_construction_ms": 0.0, "candidate_pair_attempts": 0})
            return starts[0], goals[0], None, "no_candidate_pair_route"
        ordered_goals = [int(goal.node_id) for goal in goals if int(goal.node_id) in component_goals]
        route, start_id, goal_id, search_info = search_topology_multi_goal_timed(
            topology, starts_in_components, ordered_goals,
        )
        if timing is not None:
            timing.update({
                "route_search_ms": float(search_info.get("route_search_ms", 0.0)),
                "route_construction_ms": float(search_info.get("route_construction_ms", 0.0)),
                "adjacency_build_ms": float(search_info.get("adjacency_build_ms", 0.0)),
                "topology_adjacency_cache_hit": bool(search_info.get("adjacency_cache_hit", False)),
                "candidate_pair_attempts": int(search_info.get("candidate_pair_attempts", 1)),
                "route_cache_hit": False,
            })
        start_by_id = {int(item.node_id): item for item in starts}
        goal_by_id = {int(item.node_id): item for item in goals}
        if route is not None and start_id in start_by_id and goal_id in goal_by_id:
            route_cache[route_key] = (int(start_id), int(goal_id), route)
            return start_by_id[start_id], goal_by_id[goal_id], route, "multi_source_route"
        return starts[0], goals[0], None, "no_candidate_pair_route"
    route_search_started = time.monotonic_ns()
    pair_attempts = 0
    for start in starts:
        for goal in goals:
            if int(start.component_id) != int(goal.component_id):
                continue
            pair_attempts += 1
            route = legacy.search_topology(topology, start.node_id, goal.node_id)
            if route is not None:
                if timing is not None:
                    timing.update({
                        "route_search_ms": (time.monotonic_ns() - route_search_started) / 1.0e6,
                        "route_construction_ms": 0.0,
                        "candidate_pair_attempts": pair_attempts,
                        "topology_adjacency_cache_hit": bool(getattr(getattr(topology, "graph", None), "adjacency_cache_hit", False)),
                    })
                return start, goal, route, "candidate_pair_route"
    if timing is not None:
        timing.update({
            "route_search_ms": (time.monotonic_ns() - route_search_started) / 1.0e6,
            "route_construction_ms": 0.0,
            "candidate_pair_attempts": pair_attempts,
            "topology_adjacency_cache_hit": bool(getattr(getattr(topology, "graph", None), "adjacency_cache_hit", False)),
        })
    return starts[0], goals[0], None, "no_candidate_pair_route"


def _session_log_cursor(session: Any) -> int:
    path = getattr(getattr(session, "stack", None), "log_file", None)
    if path is None:
        return 0
    try:
        return int(Path(path).stat().st_size)
    except OSError:
        return 0


def _session_log_delta(session: Any, cursor: int) -> str:
    path = getattr(getattr(session, "stack", None), "log_file", None)
    if path is None:
        return ""
    try:
        with Path(path).open("rb") as stream:
            stream.seek(max(0, int(cursor)))
            return stream.read().decode("utf-8", errors="replace")
    except OSError:
        return ""


def _classify_smac_failure(
    result_code: str, result_diagnostics: Mapping[str, Any], log_delta: str,
) -> Tuple[str, Any, str]:
    """Map a generic Nav2 abort to an evidence-backed failure category."""
    details = " ".join(
        str(result_diagnostics.get(key, ""))
        for key in ("failure_detail", "error_message", "planner_error", "smac_failure_detail")
    )
    text = f"{log_delta or ''} {details}".lower()
    if result_code in {"SERVER_UNAVAILABLE", "ACTION_REJECTED", "BACKEND_UNAVAILABLE"}:
        return "BACKEND_UNAVAILABLE", False, details or f"Smac action unavailable: {result_code}"
    if "starting point in lethal space" in text or ("starting point" in text and "lethal" in text):
        return "START_IN_LETHAL_SPACE", False, "Smac reported starting point in lethal space"
    if "goal point in lethal space" in text or ("goal point" in text and "lethal" in text):
        return "GOAL_IN_LETHAL_SPACE", False, "Smac reported goal point in lethal space"
    if any(token in text for token in ("exceeded maximum iterations", "maximum iterations", "max iterations", "iteration limit")):
        return "SMAC_MAX_ITERATIONS", True, "Smac exceeded maximum iterations"
    if any(token in text for token in ("no valid path", "cannot create feasible plan", "failed to generate a valid path")):
        return "NO_PATH_IN_CORRIDOR", True, "Smac reported no valid path"
    if result_code in {"CLIENT_TIMEOUT", "PLANNER_TIMEOUT"} or "timed out" in text or "timeout" in text:
        return "PLANNER_TIMEOUT", "not_available", "Smac or action timeout"
    if result_code == "ACTION_ABORTED":
        if not text.strip():
            # Preserve the historical v2/v3 contract when no planner log was
            # captured; the independent r4 pipeline uses the stricter
            # ACTION_ABORTED_UNKNOWN classification in that case.
            planning_ms = result_diagnostics.get("l3_planning_time_ms") or result_diagnostics.get("planning_time_ms")
            started = bool(planning_ms and float(planning_ms) > 0.0)
            return "ACTION_ABORTED", started, "Nav2 action returned ABORTED"
        return "ACTION_ABORTED_UNKNOWN", "not_available", "Nav2 action returned ABORTED without a recognized planner reason"
    if result_code:
        return "ACTION_ABORTED_UNKNOWN", "not_available", f"Unrecognized Smac result code={result_code}"
    return "ACTION_ABORTED_UNKNOWN", "not_available", "Nav2 returned no success result"


def plan_l1_l3_corridor_hybrid(
    ctx: legacy.MapContext,
    query: Query,
    topology: TopologyArtifact,
    session: Any,
    smac_spec: BackendSpec,
    *,
    corridor_padding_m: float = CORRIDOR_PADDING_M,
    corridor_semantics: str = CORRIDOR_SEMANTICS,
    timeout_s: float = TIMEOUT_S,
    padding_schedule_m: Optional[Sequence[float]] = None,
    force_full_update: bool = False,
    validate_each_attempt: bool = False,
    cache_mode: str = CACHE_MODE_BASELINE,
    corridor_mask_builder: Optional[Callable[..., Tuple[np.ndarray, Mapping[str, Any]]]] = None,
) -> Tuple[PlanResult, Dict[str, Any]]:
    """Execute L1 followed by one or a bounded set of corridor L3' requests.

    The default remains the v2 one-shot behavior.  The validity repair profile
    opts into a deterministic 2 -> 4 -> 6 m schedule; each larger mask is
    tried only after the preceding action or local validation failed.
    """
    del timeout_s  # The Smac client owns its configured action deadline.
    if cache_mode not in {CACHE_MODE_BASELINE, CACHE_MODE_OPTIMIZED}:
        raise ValueError(f"unsupported cache mode: {cache_mode}")
    started_ns = time.monotonic_ns()
    endpoint_started_ns = time.monotonic_ns()
    diagnostics: Dict[str, Any] = {
        "architecture": ARCHITECTURE,
        "l1_backend": L1_BACKEND,
        "l3_prime_backend": smac_spec.backend,
        "corridor_semantics": corridor_semantics,
        "l2_called": False,
        "l2_call_count": 0,
        "l3_prime_call_count": 0,
        "l1_route_selected": False,
        "failure_code": "",
        "planner_search_started": "not_available",
        "smac_start_cost": "not_available",
        "smac_goal_cost": "not_available",
        "cache_mode": cache_mode,
        "fallback_used": False,
        "fallback_reason": "",
        "l1_attachment_lookup_ms": 0.0,
        "l1_candidate_collision_check_ms": 0.0,
        "l1_adjacency_build_ms": 0.0,
        "l1_route_search_ms": 0.0,
        "l1_route_construction_ms": 0.0,
        "l1_graph_search_ms": 0.0,
        "l1_start_candidate_count": 0,
        "l1_goal_candidate_count": 0,
        "l1_candidate_pair_attempts": 0,
        "topology_adjacency_cache_hit": bool(getattr(getattr(topology, "graph", None), "adjacency_cache_hit", False)),
        "endpoint_spatial_index_cache_hit": False,
        "endpoint_candidate_cache_hit": False,
        "route_cache_hit": False,
    }
    start_cell, goal_cell = _endpoint_cells(ctx, query)
    diagnostics["l1_endpoint_cell_lookup_ms"] = (time.monotonic_ns() - endpoint_started_ns) / 1.0e6
    raw_mask_cached = getattr(ctx, "raw_free_mask_cache", None) is not None
    raw_mask_started_ns = time.monotonic_ns()
    raw_free = _raw_free_mask(ctx)
    diagnostics["raw_free_mask_cache_hit"] = bool(raw_mask_cached)
    diagnostics["l1_raw_free_mask_ms"] = (time.monotonic_ns() - raw_mask_started_ns) / 1.0e6
    l1_free_mask = np.asarray(getattr(topology, "free_mask", getattr(ctx, "free_mask", raw_free)), dtype=bool)
    diagnostics.update({
        "raw_start_occupancy": int(ctx.hospital_map.occupancy[start_cell]) if start_cell is not None else "not_available",
        "raw_goal_occupancy": int(ctx.hospital_map.occupancy[goal_cell]) if goal_cell is not None else "not_available",
        "l1_free_start": bool(l1_free_mask[start_cell]) if start_cell is not None else False,
        "l1_free_goal": bool(l1_free_mask[goal_cell]) if goal_cell is not None else False,
    })
    if start_cell is None or goal_cell is None:
        diagnostics["failure_code"] = "L1_ENDPOINT_OUT_OF_BOUNDS"
        return PlanResult(
            planner_backend=smac_spec.backend, backend_version=smac_spec.version,
            source=L3_PRIME_SOURCE, failure_code=diagnostics["failure_code"],
            failure_detail="endpoint is outside the map", diagnostics=diagnostics,
        ), diagnostics

    attach_timing: Dict[str, Any] = {}
    start_attachment, goal_attachment, route, attach_reason = _select_route_with_endpoint_attach(
        topology, query, cache_mode=cache_mode, timing=attach_timing,
    )
    diagnostics.update({
        "l1_attachment_lookup_ms": float(attach_timing.get("start_lookup_ms", 0.0))
        + float(attach_timing.get("goal_lookup_ms", 0.0)),
        "l1_candidate_collision_check_ms": float(attach_timing.get("start_collision_check_ms", 0.0))
        + float(attach_timing.get("goal_collision_check_ms", 0.0)),
        "l1_adjacency_build_ms": float(attach_timing.get("adjacency_build_ms", 0.0)),
        "l1_route_search_ms": float(attach_timing.get("route_search_ms", 0.0)),
        "l1_route_construction_ms": float(attach_timing.get("route_construction_ms", 0.0)),
        "l1_start_candidate_count": int(attach_timing.get("start_candidate_count", 0)),
        "l1_goal_candidate_count": int(attach_timing.get("goal_candidate_count", 0)),
        "l1_candidate_pair_attempts": int(attach_timing.get("candidate_pair_attempts", 0)),
        "topology_adjacency_cache_hit": bool(attach_timing.get("topology_adjacency_cache_hit", diagnostics.get("topology_adjacency_cache_hit", False))),
        "endpoint_spatial_index_cache_hit": bool(attach_timing.get("endpoint_spatial_index_cache_hit", False)),
        "endpoint_candidate_cache_hit": bool(attach_timing.get("endpoint_candidate_cache_hit", False)),
        "route_cache_hit": bool(attach_timing.get("route_cache_hit", False)),
    })
    diagnostics["l1_graph_search_ms"] = float(
        diagnostics["l1_attachment_lookup_ms"]
        + diagnostics["l1_candidate_collision_check_ms"]
        + diagnostics["l1_adjacency_build_ms"]
        + diagnostics["l1_route_search_ms"]
        + diagnostics["l1_route_construction_ms"]
    )
    if start_attachment is None or goal_attachment is None:
        diagnostics["failure_code"] = "L1_ENDPOINT_NOT_ATTACHABLE"
        diagnostics["endpoint_attach_reason"] = attach_reason
        return PlanResult(
            planner_backend=smac_spec.backend, backend_version=smac_spec.version,
            source=L3_PRIME_SOURCE, failure_code=diagnostics["failure_code"],
            failure_detail="L1 could not attach one or both endpoint poses", diagnostics=diagnostics,
        ), diagnostics
    if route is None:
        diagnostics["failure_code"] = "L1_NO_ROUTE"
        diagnostics["endpoint_attach_reason"] = attach_reason
        return PlanResult(
            planner_backend=smac_spec.backend, backend_version=smac_spec.version,
            source=L3_PRIME_SOURCE, failure_code=diagnostics["failure_code"],
            failure_detail="topology graph has no route between attached nodes", diagnostics=diagnostics,
        ), diagnostics

    total_free_cells = int(np.count_nonzero(raw_free))
    diagnostics.update({
        "l1_route_selected": True,
        "topology_node_ids": list(route.node_ids),
        "topology_edge_ids": list(route.edge_ids),
        "total_free_grid_cells": total_free_cells,
        "corridor_min_width_m": float(route.min_width_m),
        "corridor_route_length_m": float(route.length_m),
        "corridor_footprint_safety_margin_m": FOOTPRINT_SAFETY_MARGIN_M,
        "corridor_bend_margin_m": BEND_MARGIN_M,
        "endpoint_connection_back_m": ENDPOINT_CONNECT_BACK_M,
        "endpoint_connection_forward_m": ENDPOINT_CONNECT_FORWARD_M,
    })
    if session is None:
        unavailable = legacy.unavailable_plan(smac_spec, source=L3_PRIME_SOURCE)
        diagnostics.update({
            "failure_code": unavailable.failure_code,
            "l3_prime_call_count": 0,
            "l3_prime_called": False,
        })
        unavailable.diagnostics = {**(unavailable.diagnostics or {}), **diagnostics}
        return unavailable, diagnostics

    schedule = [float(corridor_padding_m)] if padding_schedule_m is None else [float(value) for value in padding_schedule_m]
    if not schedule or any(value <= 0.0 for value in schedule):
        raise ValueError("corridor padding schedule must contain positive values")
    if any(right < left for left, right in zip(schedule, schedule[1:])):
        raise ValueError("corridor padding schedule must be non-decreasing")
    diagnostics["padding_schedule_m"] = schedule
    attempts: List[Dict[str, Any]] = []
    result: Optional[PlanResult] = None
    total_calls = 0
    total_planning_ms = 0.0
    total_action_wall_ms = 0.0
    total_mask_time_ms = 0.0
    last_failure = "L3_PRIME_FAILED"

    def _promote_attempt_state(attempt: Mapping[str, Any]) -> None:
        """Expose the authoritative last-attempt state at query level.

        Smac and the costmap updater report their state per action.  Keeping
        only the nested attempt list made ``runs.csv`` look as if no update
        had happened on failures.  Promote the auditable fields without
        inferring a ROS acknowledgement from a local hash comparison.
        """
        for key in (
            "costmap_update_before_hash", "costmap_update_after_hash",
            "costmap_update_expected_hash", "costmap_update_time_ms",
            "costmap_update_messages", "costmap_update_mode",
            "costmap_update_fallback", "costmap_update_fallback_reason",
            "action_status", "action_result_code", "smac_log_excerpt",
            "planner_search_started", "corridor_mask_hash",
            "allowed_grid_cells", "corridor_area_ratio", "corridor_mask_start",
        "corridor_mask_goal", "corridor_padding_m",
            "corner_count", "corner_node_ids", "corner_edge_ids",
            "corner_max_curvature_1pm", "corner_support_length_m",
            "corner_support_intervals_m",
            "base_corridor_padding_m", "corner_corridor_padding_m",
            "corner_widened_area_ratio", "corner_corridor_mask_hash",
            "corridor_mask_strategy",
            "route_signature", "mask_cache_key", "mask_cache_hit",
            "route_analysis_cache_hit", "edge_mask_cache_hit", "endpoint_strip_cache_hit",
            "centerline_rasterization_ms", "corner_analysis_ms", "dilation_ms",
            "mask_union_ms", "mask_copy_ms", "mask_hash_ms", "allowed_cell_count_ms",
            "costmap_update_ms", "costmap_update_cells", "costmap_update_bytes",
            "costmap_update_skipped", "total_corridor_mask_online_ms",
            "edge_mask_cache_verified",
        ):
            if key in attempt:
                diagnostics[key] = attempt[key]
        diagnostics["costmap_update_acknowledged"] = attempt.get(
            "costmap_update_acknowledged", "not_available",
        )
        diagnostics["last_attempt_index"] = attempt.get("attempt_index", "not_available")
    for attempt_index, padding in enumerate(schedule, start=1):
        mask_started_ns = time.monotonic_ns()
        custom_mask_diagnostics: Mapping[str, Any] = {}
        if corridor_mask_builder is None:
            allowed = _build_corridor_mask(ctx, topology, route, query, start_cell, goal_cell, padding, corridor_semantics)
        else:
            built = corridor_mask_builder(
                ctx, topology, route, query, start_cell, goal_cell, padding,
                corridor_semantics,
            )
            if isinstance(built, tuple) and len(built) == 2:
                allowed, custom_mask_diagnostics = built
            else:
                allowed = built
            allowed = np.asarray(allowed, dtype=bool)
        mask_time_ms = (time.monotonic_ns() - mask_started_ns) / 1.0e6
        total_mask_time_ms += mask_time_ms
        allowed_cell_started_ns = time.monotonic_ns()
        cached_allowed_cells = custom_mask_diagnostics.get("precomputed_allowed_cells")
        allowed_cells = int(cached_allowed_cells) if cached_allowed_cells is not None else int(np.count_nonzero(allowed))
        allowed_cell_count_ms = (time.monotonic_ns() - allowed_cell_started_ns) / 1.0e6
        mask_hash_started_ns = time.monotonic_ns()
        mask_hash = str(custom_mask_diagnostics.get("precomputed_mask_hash") or _grid_hash(allowed))
        mask_hash_ms = (time.monotonic_ns() - mask_hash_started_ns) / 1.0e6
        attempt_diag: Dict[str, Any] = {
            "attempt_index": attempt_index, "corridor_padding_m": padding,
            "corridor_mask_hash": mask_hash, "allowed_grid_cells": allowed_cells,
            "corridor_area_ratio": float(allowed_cells / total_free_cells) if total_free_cells else 0.0,
            "corridor_mask_build_time_ms": mask_time_ms,
            "allowed_cell_count_ms": allowed_cell_count_ms,
            "mask_hash_ms": mask_hash_ms,
            "corridor_mask_start": bool(allowed[start_cell]), "corridor_mask_goal": bool(allowed[goal_cell]),
        }
        attempt_diag.update(dict(custom_mask_diagnostics))
        if not bool(allowed[start_cell]) or not bool(allowed[goal_cell]):
            last_failure = "L1_CORRIDOR_ENDPOINT_OUTSIDE"
            attempt_diag.update({"failure_code": last_failure, "planner_search_started": False})
            attempts.append(attempt_diag)
            continue
        log_cursor = _session_log_cursor(session)
        call_started_ns = time.monotonic_ns()
        try:
            plan_kwargs = {
                "source": L3_PRIME_SOURCE, "allowed_mask": allowed,
                "window_start_index": 0, "window_end_index": -1,
                "window_path_length_m": float(route.length_m),
            }
            if force_full_update:
                plan_kwargs["force_full_update"] = True
            result = session.plan(query, smac_spec, **plan_kwargs)
        except Exception as exc:
            message = str(exc)
            lower = message.lower()
            last_failure = "COSTMAP_UPDATE_TIMEOUT" if "costmap" in lower and "timeout" in lower else "L3_PRIME_EXCEPTION"
            attempt_diag.update({"failure_code": last_failure, "failure_detail": message, "planner_search_started": False})
            attempts.append(attempt_diag)
            total_calls += 1
            if attempt_index == len(schedule):
                result = PlanResult(planner_backend=smac_spec.backend, backend_version=smac_spec.version, source=L3_PRIME_SOURCE, failure_code=last_failure, failure_detail=message, diagnostics=diagnostics)
            continue
        result_diagnostics = dict(result.diagnostics or {})
        log_delta = _session_log_delta(session, log_cursor)
        called = bool(result_diagnostics.get("backend_called", True))
        physical_calls = int(result_diagnostics.get("backend_call_count") or result_diagnostics.get("physical_backend_call_count") or (1 if called else 0))
        total_calls += physical_calls if called else 0
        planning_ms = float(result_diagnostics.get("l3_planning_time_ms") or result_diagnostics.get("planning_time_ms") or 0.0)
        action_wall_ms = float(result_diagnostics.get("l3_action_wall_ms") or result_diagnostics.get("wall_time_ms") or ((time.monotonic_ns() - call_started_ns) / 1.0e6))
        total_planning_ms += planning_ms
        total_action_wall_ms += action_wall_ms
        classified_code, search_started, failure_detail = _classify_smac_failure(
            str(result.failure_code or result_diagnostics.get("action_result_code") or ""),
            {**result_diagnostics, **diagnostics}, log_delta,
        )
        points = result.points or []
        path_mask_started_ns = time.monotonic_ns()
        within = not points or _path_within_mask(ctx, points, allowed)
        path_mask_check_ms = (time.monotonic_ns() - path_mask_started_ns) / 1.0e6
        local_valid = False
        local_validation_code = ""
        local_validation_ms = 0.0
        if result.planner_success and points and not within:
            local_validation_code = "L3_PRIME_PATH_OUTSIDE_CORRIDOR"
        elif result.planner_success and points and validate_each_attempt:
            # The shared validator requires immutable provenance fields.  Smac
            # itself does not emit those audit fields, so attach them to this
            # exact returned path before local validation; pose, yaw,
            # steering, curvature, and motion direction remain untouched.
            # Resolve the immutable provenance once per attempt.  Calling
            # ``git rev-parse`` for every sampled path point turns a long
            # route into thousands of subprocess launches.
            provenance_commit = _source_commit() or "unknown"
            for point in points:
                point.setdefault("source_commit", provenance_commit)
            attempt_path_hash = legacy._path_hash(points)
            for point in points:
                point.setdefault("path_hash", attempt_path_hash)
            validation_started_ns = time.monotonic_ns()
            local_metrics = legacy.validate_path(ctx, query, points)
            local_validation_ms = (time.monotonic_ns() - validation_started_ns) / 1.0e6
            # ``validate_path`` is the canonical source for the two hard
            # validity dimensions.  Older versions do not populate its
            # convenience ``final_valid_success`` key, so derive acceptance
            # from the explicit fields and reject any reported failure.
            local_valid = bool(
                local_metrics.get("static_footprint_valid")
                and local_metrics.get("kinematic_valid")
                and not local_metrics.get("failure_code")
            )
            if not local_valid:
                local_validation_code = str(local_metrics.get("failure_code") or "FINAL_VALIDATION_FAILED")
        elif result.planner_success and points:
            local_valid = True
        success = bool(result.planner_success and points and within and (local_valid or not validate_each_attempt))
        if not success:
            last_failure = local_validation_code or ("" if result.planner_success else classified_code) or "FINAL_VALIDATION_FAILED"
            result.planner_success = False
            result.failure_code = last_failure
            result.failure_detail = failure_detail if last_failure == classified_code else local_validation_code
        attempt_diag.update({
            "failure_code": "" if success else last_failure,
            "planner_search_started": True if result.planner_success or success else search_started,
            "planner_success": bool(result.planner_success), "validation_passed": local_valid if validate_each_attempt else success,
            "within_corridor": within, "planning_time_ms": planning_ms, "action_wall_time_ms": action_wall_ms,
            "path_mask_check_ms": path_mask_check_ms,
            "local_validation_ms": local_validation_ms,
            "l3_process_overhead_ms": float(result_diagnostics.get("l3_process_overhead_ms") or max(0.0, action_wall_ms - planning_ms)),
            "smac_log_excerpt": log_delta[-1000:] if log_delta else "",
            "costmap_update_before_hash": result_diagnostics.get("previous_mask_hash", ""),
            "costmap_update_after_hash": result_diagnostics.get("applied_mask_hash", ""),
            "costmap_update_expected_hash": result_diagnostics.get("expected_mask_hash", ""),
            "costmap_update_time_ms": result_diagnostics.get("local_map_update_ms", 0.0),
            "costmap_update_messages": result_diagnostics.get("local_map_update_messages", 0),
            "costmap_update_mode": result_diagnostics.get("local_map_update_mode", "not_available"),
            "costmap_update_fallback": bool(result_diagnostics.get("local_map_update_fallback", False)),
            "costmap_update_fallback_reason": result_diagnostics.get("local_map_update_fallback_reason", ""),
            "costmap_update_acknowledged": result_diagnostics.get("costmap_update_acknowledged", "not_available"),
            "action_status": result_diagnostics.get("action_status", ""),
            "action_result_code": result_diagnostics.get("action_result_code", ""),
        })
        attempts.append(attempt_diag)
        _promote_attempt_state(attempt_diag)
        if success:
            diagnostics.update({
                "l3_prime_called": called, "l3_prime_call_count": total_calls,
                "corridor_padding_m": padding, "allowed_grid_cells": allowed_cells,
                "corridor_area_ratio": attempt_diag["corridor_area_ratio"], "corridor_mask_hash": mask_hash,
                "corridor_mask_shape": [int(allowed.shape[1]), int(allowed.shape[0])],
                "corridor_mask_build_time_ms": mask_time_ms,
                "corridor_mask_total_time_ms": total_mask_time_ms,
                "l1_time_ms": diagnostics.get("l1_graph_search_ms", 0.0),
                "plan_function_wall_time_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                "corridor_mask_start": True, "corridor_mask_goal": True,
                "corridor_effective_radius_m": padding + FOOTPRINT_SAFETY_MARGIN_M + BEND_MARGIN_M,
                "hybrid_planning_time_ms": total_planning_ms, "l3_action_wall_ms": total_action_wall_ms,
                "l3_process_overhead_ms": max(0.0, total_action_wall_ms - total_planning_ms),
                "peak_rss": result_diagnostics.get("stack_rss_peak_bytes") or result_diagnostics.get("planner_rss_peak_bytes"),
                "peak_pss": result_diagnostics.get("stack_pss_peak_bytes") or result_diagnostics.get("planner_pss_peak_bytes"),
                "planner_search_started": True, "returned_path_within_corridor": True,
            })
            result.diagnostics = {**result_diagnostics, **diagnostics, "attempts": attempts}
            return result, diagnostics
    if result is None:
        result = PlanResult(planner_backend=smac_spec.backend, backend_version=smac_spec.version, source=L3_PRIME_SOURCE, failure_code=last_failure, failure_detail=last_failure)
    diagnostics.update({
        "l3_prime_called": total_calls > 0, "l3_prime_call_count": total_calls,
        "corridor_padding_m": float(schedule[-1]), "padding_schedule_m": schedule,
        "hybrid_planning_time_ms": total_planning_ms, "l3_action_wall_ms": total_action_wall_ms,
        "corridor_mask_total_time_ms": total_mask_time_ms,
        "l1_time_ms": diagnostics.get("l1_graph_search_ms", 0.0),
        "plan_function_wall_time_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
        "l3_process_overhead_ms": max(0.0, total_action_wall_ms - total_planning_ms),
        "planner_search_started": any(bool(item.get("planner_search_started")) for item in attempts),
        "failure_code": last_failure, "attempts": attempts,
        "attempt_count": len(attempts),
    })
    if attempts:
        _promote_attempt_state(attempts[-1])
    result.failure_code = last_failure
    result.failure_detail = str(result.failure_detail or last_failure)
    result.diagnostics = {**(result.diagnostics or {}), **diagnostics}
    return result, diagnostics


def _empty_metrics(failure_code: str = "EMPTY_PATH") -> Dict[str, Any]:
    return {
        "static_footprint_valid": False,
        "kinematic_valid": False,
        "final_valid_success": False,
        "path_length_m": None,
        "minimum_clearance_m": None,
        "maximum_curvature": None,
        "heading_discontinuity_count": 0,
        "position_discontinuity_count": 0,
        "steering_jump_count": 0,
        "reverse_distance_m": 0.0,
        "in_place_rotation_count": 0,
        "start_position_error_m": None,
        "start_yaw_error_rad": None,
        "goal_position_error_m": None,
        "goal_yaw_error_rad": None,
        "failure_code": failure_code,
        "failure_detail": failure_code,
    }


def _run_one(
    ctx: legacy.MapContext,
    topology: TopologyArtifact,
    topology_info: Mapping[str, Any],
    query: Query,
    run_mode: str,
    repetition: int,
    session: Any,
    smac_spec: BackendSpec,
    output: Path,
    source_commit: Optional[str],
    *,
    corridor_padding_m: float = CORRIDOR_PADDING_M,
    corridor_semantics: str = CORRIDOR_SEMANTICS,
    profile_name: str = "formal_default",
    padding_schedule_m: Optional[Sequence[float]] = None,
    force_full_update: bool = False,
    validate_each_attempt: bool = False,
    cache_mode: str = CACHE_MODE_BASELINE,
    corridor_mask_builder: Optional[Callable[..., Tuple[np.ndarray, Mapping[str, Any]]]] = None,
) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    run_id = f"{ctx.map_id}_{query.query_id}_l1_l3_{run_mode}_{repetition}"
    query_hash = _query_hash(query)
    cpu_before = resource.getrusage(resource.RUSAGE_SELF)
    wall_started_ns = time.monotonic_ns()
    reset_info: Dict[str, Any] = {}
    result: Optional[PlanResult] = None
    diagnostics: Dict[str, Any] = {
        "architecture": ARCHITECTURE,
        "l1_backend": L1_BACKEND,
        "l3_prime_backend": smac_spec.backend,
        "l2_called": False,
        "l2_call_count": 0,
        "l3_prime_call_count": 0,
    }
    metrics = _empty_metrics("NOT_RUN")
    fallback_used = False
    fallback_reason = ""
    try:
        if session is not None:
            reset_info = session.reset_query_state(query.query_id, restore_base_map=False)

        def execute(
            selected_mode: str,
            *,
            force_full_override: Optional[bool] = None,
        ) -> Tuple[PlanResult, Dict[str, Any], Dict[str, Any]]:
            planned, plan_diagnostics = plan_l1_l3_corridor_hybrid(
                ctx, query, topology, session, smac_spec,
                corridor_padding_m=corridor_padding_m,
                corridor_semantics=corridor_semantics,
                padding_schedule_m=padding_schedule_m,
                force_full_update=(
                    force_full_update if force_full_override is None else force_full_override
                ),
                validate_each_attempt=validate_each_attempt,
                cache_mode=selected_mode,
                corridor_mask_builder=corridor_mask_builder,
            )
            if planned.points:
                # Provenance is the only metadata added here.  The planner's
                # poses, yaw, steering, curvature and direction are untouched.
                for point in planned.points:
                    point.setdefault("source_commit", source_commit or "unknown")
                result_path_hash = legacy._path_hash(planned.points)
                for point in planned.points:
                    point["path_hash"] = result_path_hash
                validation_started_ns = time.monotonic_ns()
                measured = legacy.validate_path(ctx, query, planned.points)
                measured["final_validation_time_ms"] = (time.monotonic_ns() - validation_started_ns) / 1.0e6
            else:
                measured = _empty_metrics(planned.failure_code or "EMPTY_PATH")
                measured["final_validation_time_ms"] = 0.0
            measured["final_valid_success"] = bool(
                planned.planner_success and measured.get("static_footprint_valid")
                and measured.get("kinematic_valid")
            )
            if not measured["final_valid_success"] and not measured.get("failure_code"):
                measured["failure_code"] = planned.failure_code or "FINAL_VALIDATION_FAILED"
            return planned, plan_diagnostics, measured

        result, diagnostics, metrics = execute(cache_mode)
        if cache_mode == CACHE_MODE_OPTIMIZED and not bool(metrics.get("final_valid_success")):
            # Optimized endpoint/cached route selection is never allowed to
            # reduce validity.  Re-run the unchanged baseline path and retain
            # both diagnostics so the fallback is auditable.
            fallback_used = True
            fallback_reason = str(metrics.get("failure_code") or result.failure_code or "optimized_validation_failed")
            # A failed optimized delta update is not evidence that the ROS
            # costmap applied the requested state.  Re-establish the baseline
            # semantics with a complete map update before retrying, so a
            # stale/partial patch cannot turn into a validity regression.
            if session is not None and hasattr(session, "reset_query_state"):
                reset_info = session.reset_query_state(query.query_id, restore_base_map=True)
            baseline_result, baseline_diagnostics, baseline_metrics = execute(
                CACHE_MODE_BASELINE, force_full_override=True,
            )
            optimized_calls = int(diagnostics.get("l3_prime_call_count") or 0)
            baseline_calls = int(baseline_diagnostics.get("l3_prime_call_count") or 0)
            baseline_diagnostics = dict(baseline_diagnostics)
            baseline_diagnostics.update({
                "fallback_used": True,
                "fallback_reason": fallback_reason,
                "optimized_diagnostics": diagnostics,
                "optimized_l3_prime_call_count": optimized_calls,
                "l3_prime_call_count": optimized_calls + baseline_calls,
                "cache_mode": CACHE_MODE_BASELINE,
            })
            result, diagnostics, metrics = baseline_result, baseline_diagnostics, baseline_metrics
        diagnostics["fallback_used"] = bool(fallback_used)
        diagnostics["fallback_reason"] = fallback_reason
        diagnostics["final_validation_time_ms"] = float(metrics.get("final_validation_time_ms") or 0.0)
    except Exception as exc:
        diagnostics["failure_code"] = "PIPELINE_EXCEPTION"
        diagnostics["failure_detail"] = str(exc)
        metrics = _empty_metrics("PIPELINE_EXCEPTION")
        result = PlanResult(
            planner_backend=smac_spec.backend, backend_version=smac_spec.version,
            source=L3_PRIME_SOURCE, failure_code="PIPELINE_EXCEPTION",
            failure_detail=str(exc), diagnostics=diagnostics,
        )
    wall_ms = (time.monotonic_ns() - wall_started_ns) / 1.0e6
    cpu_after = resource.getrusage(resource.RUSAGE_SELF)
    cpu_ms = max(
        0.0,
        (cpu_after.ru_utime - cpu_before.ru_utime + cpu_after.ru_stime - cpu_before.ru_stime) * 1000.0,
    )
    path_hash = ""
    path_file = ""
    if result is not None and result.points:
        points = [dict(point) for point in result.points]
        for point in points:
            point.setdefault("source_commit", source_commit or "unknown")
        path_hash = legacy._path_hash(points)
        for point in points:
            point["path_hash"] = path_hash
        path_file = f"paths/{run_id}.json"
        (output / path_file).write_text(
            json.dumps(points, indent=2, sort_keys=True), encoding="utf-8",
        )

    l3_calls = int(diagnostics.get("l3_prime_call_count") or 0)
    l1_timing_fields = {
        "cache_mode": diagnostics.get("cache_mode", cache_mode),
        "fallback_used": bool(diagnostics.get("fallback_used", False)),
        "fallback_reason": diagnostics.get("fallback_reason", ""),
        "l1_attachment_lookup_ms": diagnostics.get("l1_attachment_lookup_ms", 0.0),
        "l1_candidate_collision_check_ms": diagnostics.get("l1_candidate_collision_check_ms", 0.0),
        "l1_adjacency_build_ms": diagnostics.get("l1_adjacency_build_ms", 0.0),
        "l1_route_search_ms": diagnostics.get("l1_route_search_ms", 0.0),
        "l1_route_construction_ms": diagnostics.get("l1_route_construction_ms", 0.0),
        "l1_graph_search_ms": diagnostics.get("l1_graph_search_ms", 0.0),
        "l1_start_candidate_count": diagnostics.get("l1_start_candidate_count", 0),
        "l1_goal_candidate_count": diagnostics.get("l1_goal_candidate_count", 0),
        "l1_candidate_pair_attempts": diagnostics.get("l1_candidate_pair_attempts", 0),
        "topology_adjacency_cache_hit": diagnostics.get("topology_adjacency_cache_hit", False),
        "endpoint_spatial_index_cache_hit": diagnostics.get("endpoint_spatial_index_cache_hit", False),
        "endpoint_candidate_cache_hit": diagnostics.get("endpoint_candidate_cache_hit", False),
        "route_cache_hit": diagnostics.get("route_cache_hit", False),
        "raw_free_mask_cache_hit": diagnostics.get("raw_free_mask_cache_hit", False),
        "corridor_mask_total_time_ms": diagnostics.get("corridor_mask_total_time_ms", 0.0),
        "l1_raw_free_mask_ms": diagnostics.get("l1_raw_free_mask_ms", 0.0),
        "l1_endpoint_cell_lookup_ms": diagnostics.get("l1_endpoint_cell_lookup_ms", 0.0),
        "path_mask_check_ms": diagnostics.get("path_mask_check_ms", 0.0),
        "local_validation_ms": sum(float(item.get("local_validation_ms") or 0.0) for item in diagnostics.get("attempts", []) if isinstance(item, Mapping)),
        "final_validation_time_ms": diagnostics.get("final_validation_time_ms", 0.0),
        "plan_function_wall_time_ms": diagnostics.get("plan_function_wall_time_ms", 0.0),
        "hybrid_planning_time_ms": diagnostics.get("hybrid_planning_time_ms", 0.0),
        "l3_action_wall_ms": diagnostics.get("l3_action_wall_ms", 0.0),
        "l3_process_overhead_ms": diagnostics.get("l3_process_overhead_ms", 0.0),
        "costmap_update_time_ms": diagnostics.get("costmap_update_time_ms", 0.0),
        "costmap_update_ms": diagnostics.get("costmap_update_ms", diagnostics.get("costmap_update_time_ms", 0.0)),
        "costmap_update_messages": diagnostics.get("costmap_update_messages", 0),
        "costmap_update_cells": diagnostics.get("costmap_update_cells", 0),
        "costmap_update_bytes": diagnostics.get("costmap_update_bytes", 0),
        "costmap_update_skipped": diagnostics.get("costmap_update_skipped", False),
        "costmap_update_mode": diagnostics.get("costmap_update_mode", "not_available"),
        "costmap_update_acknowledged": diagnostics.get("costmap_update_acknowledged", "not_available"),
        "corner_count": diagnostics.get("corner_count", 0),
        "corner_node_ids": diagnostics.get("corner_node_ids", []),
        "corner_edge_ids": diagnostics.get("corner_edge_ids", []),
        "corner_max_curvature_1pm": diagnostics.get("corner_max_curvature_1pm", 0.0),
        "corner_support_length_m": diagnostics.get("corner_support_length_m", 0.0),
        "corner_support_intervals_m": diagnostics.get("corner_support_intervals_m", []),
        "base_corridor_padding_m": diagnostics.get("base_corridor_padding_m", corridor_padding_m),
        "corner_corridor_padding_m": diagnostics.get("corner_corridor_padding_m", 4.0),
        "corner_widened_area_ratio": diagnostics.get("corner_widened_area_ratio", 0.0),
        "corner_corridor_mask_hash": diagnostics.get("corner_corridor_mask_hash", ""),
        "corridor_mask_strategy": diagnostics.get("corridor_mask_strategy", "fixed_padding"),
        "route_signature": diagnostics.get("route_signature", ""),
        "mask_cache_key": diagnostics.get("mask_cache_key", ""),
        "mask_cache_hit": diagnostics.get("mask_cache_hit", False),
        "route_analysis_cache_hit": diagnostics.get("route_analysis_cache_hit", False),
        "edge_mask_cache_hit": diagnostics.get("edge_mask_cache_hit", False),
        "endpoint_strip_cache_hit": diagnostics.get("endpoint_strip_cache_hit", False),
        "centerline_rasterization_ms": diagnostics.get("centerline_rasterization_ms", 0.0),
        "corner_analysis_ms": diagnostics.get("corner_analysis_ms", 0.0),
        "dilation_ms": diagnostics.get("dilation_ms", 0.0),
        "mask_union_ms": diagnostics.get("mask_union_ms", 0.0),
        "mask_copy_ms": diagnostics.get("mask_copy_ms", 0.0),
        "mask_hash_ms": diagnostics.get("mask_hash_ms", 0.0),
        "allowed_cell_count_ms": diagnostics.get("allowed_cell_count_ms", 0.0),
        "total_corridor_mask_online_ms": diagnostics.get("total_corridor_mask_online_ms", diagnostics.get("corridor_mask_total_time_ms", 0.0)),
        "edge_mask_cache_verified": diagnostics.get("edge_mask_cache_verified", False),
    }
    call_row = {
        "run_id": run_id,
        "map_id": ctx.map_id,
        "query_id": query.query_id,
        "query_hash": query_hash,
        "run_mode": run_mode,
        "repetition": repetition,
        "architecture": ARCHITECTURE,
        "l1_backend": L1_BACKEND,
        "l3_prime_backend": smac_spec.backend,
        "corridor_semantics": corridor_semantics,
        "corridor_profile": profile_name,
        "stage": "L3_PRIME",
        "role": "l3_prime_full_corridor_hybrid",
        "planner_backend": smac_spec.backend,
        "backend_version": smac_spec.version,
        "called": bool(diagnostics.get("l3_prime_called", False)),
        "physical_backend_call_count": l3_calls,
        "l3_prime_call_count": l3_calls,
        "l2_called": False,
        "l2_call_count": 0,
        "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
        "allowed_grid_cells": diagnostics.get("allowed_grid_cells", 0),
        "corridor_padding_m": diagnostics.get("corridor_padding_m", corridor_padding_m),
        "corridor_min_width_m": diagnostics.get("corridor_min_width_m", 0.0),
        "corridor_area_ratio": diagnostics.get("corridor_area_ratio", 0.0),
        "costmap_update_before_hash": diagnostics.get("costmap_update_before_hash", ""),
        "costmap_update_after_hash": diagnostics.get("costmap_update_after_hash", ""),
        "costmap_update_acknowledged": diagnostics.get("costmap_update_acknowledged", "not_available"),
        "attempt_count": diagnostics.get("attempt_count", 0),
        "attempts": diagnostics.get("attempts", []),
        "planner_search_started": diagnostics.get("planner_search_started", "not_available"),
        "expanded_states": "not_available",
        "generated_states": "not_available",
        "final_valid_success": bool(metrics.get("final_valid_success")),
        "failure_code": result.failure_code if result is not None else "PIPELINE_EXCEPTION",
        **l1_timing_fields,
    }
    run_row = {
        "run_id": run_id,
        "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "map_id": ctx.map_id,
        "map_sha256": ctx.map_sha256,
        "map_yaml_sha256": ctx.map_yaml_sha256,
        "query_id": query.query_id,
        "query_hash": query_hash,
        "query_role": (
            "raw" if query.query_id in RAW_QUERY_IDS
            else ("diagnostic" if query.query_id == "q00" else "derived_diagnostic")
        ),
        "architecture": ARCHITECTURE,
        "run_mode": run_mode,
        "repetition": repetition,
        "start_x": query.start[0], "start_y": query.start[1], "start_yaw": query.start[2],
        "goal_x": query.goal[0], "goal_y": query.goal[1], "goal_yaw": query.goal[2],
        "l1_backend": L1_BACKEND,
        "l3_prime_backend": smac_spec.backend,
        "l2_called": False,
        "l2_call_count": 0,
        "l3_prime_call_count": l3_calls,
        "l3_backend_call_count": l3_calls,
        "expanded_states": "not_available",
        "generated_states": "not_available",
        "l3_attempted": l3_calls > 0,
        "corridor_semantics": corridor_semantics,
        "corridor_profile": profile_name,
        "corridor_padding_m": diagnostics.get("corridor_padding_m", CORRIDOR_PADDING_M),
        "allowed_grid_cells": diagnostics.get("allowed_grid_cells", 0),
        "total_free_grid_cells": diagnostics.get("total_free_grid_cells", 0),
        "corridor_area_ratio": diagnostics.get("corridor_area_ratio", 0.0),
        "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
        "corridor_mask_shape": diagnostics.get("corridor_mask_shape", []),
        "corridor_min_width_m": diagnostics.get("corridor_min_width_m", 0.0),
        "corridor_effective_radius_m": diagnostics.get("corridor_effective_radius_m", corridor_padding_m),
        "raw_start_occupancy": diagnostics.get("raw_start_occupancy", "not_available"),
        "raw_goal_occupancy": diagnostics.get("raw_goal_occupancy", "not_available"),
        "l1_free_start": diagnostics.get("l1_free_start", False),
        "l1_free_goal": diagnostics.get("l1_free_goal", False),
        "corridor_mask_start": diagnostics.get("corridor_mask_start", False),
        "corridor_mask_goal": diagnostics.get("corridor_mask_goal", False),
        "smac_start_cost": diagnostics.get("smac_start_cost", "not_available"),
        "smac_goal_cost": diagnostics.get("smac_goal_cost", "not_available"),
        "costmap_update_before_hash": diagnostics.get("costmap_update_before_hash", ""),
        "costmap_update_after_hash": diagnostics.get("costmap_update_after_hash", ""),
        "costmap_update_expected_hash": diagnostics.get("costmap_update_expected_hash", ""),
        "costmap_update_time_ms": diagnostics.get("costmap_update_time_ms", 0.0),
        "costmap_update_messages": diagnostics.get("costmap_update_messages", 0),
        "costmap_update_mode": diagnostics.get("costmap_update_mode", "not_available"),
        "costmap_update_acknowledged": diagnostics.get("costmap_update_acknowledged", "not_available"),
        "planner_search_started": diagnostics.get("planner_search_started", "not_available"),
        "smac_log_excerpt": diagnostics.get("smac_log_excerpt", ""),
        "attempts": diagnostics.get("attempts", []),
        "topology_node_ids": diagnostics.get("topology_node_ids", []),
        "topology_edge_ids": diagnostics.get("topology_edge_ids", []),
        "topology_cache_key": topology_info.get("topology_cache_key", ""),
        "topology_cache_hit": bool(topology_info.get("topology_cache_hit", False)),
        "topology_build_time_ms": topology_info.get("topology_build_time_ms", 0.0),
        "topology_load_time_ms": topology_info.get("topology_load_time_ms", 0.0),
        "query_topology_reused": True,
        "hybrid_planning_time_ms": diagnostics.get("hybrid_planning_time_ms", 0.0),
        "l3_action_wall_ms": diagnostics.get("l3_action_wall_ms", 0.0),
        "l3_process_overhead_ms": diagnostics.get("l3_process_overhead_ms", 0.0),
        "pipeline_wall_time_ms": wall_ms,
        "pipeline_cpu_total_ms": cpu_ms,
        "peak_rss": diagnostics.get("peak_rss"),
        "peak_pss": diagnostics.get("peak_pss"),
        "final_valid_success": bool(metrics.get("final_valid_success")),
        "failure_code": (metrics.get("failure_code") or (result.failure_code if result else "PIPELINE_EXCEPTION")),
        "failure_detail": metrics.get("failure_detail", ""),
        "planner_success": bool(result.planner_success) if result else False,
        "action_status": (diagnostics.get("action_status") or ""),
        "path_hash": path_hash,
        "path_file": path_file,
        "source_commit": source_commit or "unknown",
        "source_hash": "",
        "session_start_count": getattr(session, "session_start_count", 0) if session is not None else 0,
        "session_close_count": getattr(session, "session_close_count", 0) if session is not None else 0,
        "session_restart_count": getattr(session, "session_restart_count", 0) if session is not None else 0,
        "query_session_reused": bool(session is not None),
        "query_session_reset_ms": reset_info.get("query_session_reset_ms", 0.0),
        **l1_timing_fields,
        **{key: value for key, value in metrics.items() if key not in {"final_valid_success"}},
    }
    run_row["diagnostics"] = diagnostics
    metric_row = {"run_id": run_id, "query_id": query.query_id, "query_hash": query_hash, **metrics}
    return run_row, call_row, metric_row


def _percentile(values: Sequence[float], percentile: float) -> float:
    return float(np.percentile(values, percentile)) if values else 0.0


def _configure_ros_domain(ros_domain_id: Optional[int]) -> int:
    """Isolate this candidate from unrelated visualization Nav2 stacks."""
    if ros_domain_id is not None:
        value = int(ros_domain_id)
    elif os.environ.get("ROS_DOMAIN_ID", "").strip():
        value = int(os.environ["ROS_DOMAIN_ID"])
    else:
        value = 100 + (os.getpid() % 100)
    if value < 0 or value > 232:
        raise ValueError("ROS domain id must be in [0, 232]")
    os.environ["ROS_DOMAIN_ID"] = str(value)
    return value


def _profile_specs() -> List[Tuple[str, str, Optional[float]]]:
    return [
        ("raw_full_map_smac", "raw_full_map", None),
        ("inflated_l1_legacy_1m", "inflated_l1_legacy", 1.0),
        ("raw_map_smac_aligned_1m", CORRIDOR_SEMANTICS, 1.0),
        ("raw_map_smac_aligned_2m", CORRIDOR_SEMANTICS, 2.0),
        ("raw_map_smac_aligned_4m", CORRIDOR_SEMANTICS, 4.0),
    ]


def _profile_summary_rows(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    for profile, group in __import__("itertools").groupby(
        sorted(rows, key=lambda item: (str(item.get("corridor_profile", "")), str(item.get("query_id", "")))),
        key=lambda item: str(item.get("corridor_profile", "")),
    ):
        items = list(group)
        valid = sum(_truth(item.get("final_valid_success")) for item in items)
        walls = [float(item.get("pipeline_wall_time_ms") or 0.0) for item in items]
        plans = [float(item.get("hybrid_planning_time_ms") or 0.0) for item in items]
        result.append({
            "profile": profile,
            "corridor_semantics": items[0].get("corridor_semantics", ""),
            "corridor_padding_m": items[0].get("corridor_padding_m", ""),
            "query_count": len(items),
            "valid_count": valid,
            "l3_prime_call_count": sum(int(item.get("l3_prime_call_count") or 0) for item in items),
            "mean_pipeline_wall_time_ms": float(np.mean(walls)) if walls else 0.0,
            "p95_pipeline_wall_time_ms": _percentile(walls, 95),
            "mean_hybrid_planning_time_ms": float(np.mean(plans)) if plans else 0.0,
            "corridor_min_width_m": min(float(item.get("corridor_min_width_m") or 0.0) for item in items),
            "corridor_area_ratio_min": min(float(item.get("corridor_area_ratio") or 0.0) for item in items),
            "corridor_mask_hashes": sorted({str(item.get("corridor_mask_hash") or "") for item in items}),
            "allowed_grid_cells": sorted({int(float(item.get("allowed_grid_cells") or 0)) for item in items}),
            "failure_codes": sorted({str(item.get("failure_code") or "") for item in items if item.get("failure_code")}),
        })
    return result


def _endpoint_diagnostic_rows(rows: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    fields = (
        "run_id", "query_id", "run_mode", "corridor_profile", "corridor_semantics",
        "corridor_padding_m", "corridor_mask_hash", "allowed_grid_cells", "corridor_area_ratio",
        "raw_start_occupancy", "raw_goal_occupancy",
        "l1_free_start", "l1_free_goal", "corridor_mask_start", "corridor_mask_goal",
        "smac_start_cost", "smac_goal_cost", "costmap_update_before_hash",
        "costmap_update_expected_hash", "costmap_update_after_hash",
        "costmap_update_acknowledged", "costmap_update_time_ms",
        "costmap_update_messages", "costmap_update_mode", "planner_search_started",
        "action_status", "failure_code", "smac_log_excerpt",
    )
    return [{key: row.get(key, "") for key in fields} for row in rows]


def _write_report(
    output: Path,
    run_rows: Sequence[Mapping[str, Any]],
    topology_info: Mapping[str, Any],
    session_info: Mapping[str, Any],
    *,
    warmups: int,
    repetitions: int,
    v7_runs: Optional[Path],
    diagnostic_rows: Sequence[Mapping[str, Any]] = (),
    default_profile: str = "formal_default",
) -> Dict[str, Any]:
    measured = [row for row in run_rows if row.get("run_mode") == "measured" and row.get("query_role") == "raw"]
    online = [float(row.get("pipeline_wall_time_ms") or 0.0) for row in measured]
    valid_count = sum(_truth(row.get("final_valid_success")) for row in measured)
    l3_calls = sum(int(row.get("l3_prime_call_count") or 0) for row in measured)
    l2_calls = sum(int(row.get("l2_call_count") or 0) for row in measured)
    per_query = {
        query_id: {
            "valid": sum(_truth(row.get("final_valid_success")) for row in measured if row.get("query_id") == query_id),
            "total": sum(1 for row in measured if row.get("query_id") == query_id),
            "failures": sorted({str(row.get("failure_code") or "") for row in measured if row.get("query_id") == query_id and row.get("failure_code")}),
        }
        for query_id in RAW_QUERY_IDS
    }
    v7_summary: Dict[str, Any] = {}
    if v7_runs and v7_runs.exists():
        with v7_runs.open(newline="", encoding="utf-8") as stream:
            baseline = [
                row for row in csv.DictReader(stream)
                if row.get("run_mode") == "measured" and row.get("query_role") == "raw"
            ]
        baseline_online = [float(row.get("online_pipeline_wall_time_ms") or row.get("pipeline_wall_time_ms") or 0.0) for row in baseline]
        v7_summary = {
            "rows": len(baseline),
            "valid": sum(_truth(row.get("final_valid_success")) for row in baseline),
            "calls": sum(int(float(row.get("l3_backend_call_count") or 0)) for row in baseline),
            "p50": _percentile(baseline_online, 50),
            "p95": _percentile(baseline_online, 95),
        }
    gate = (
        len(measured) == repetitions * len(RAW_QUERY_IDS)
        and valid_count == len(measured)
        and l2_calls == 0
        and all(int(row.get("l3_prime_call_count") or 0) == 1 for row in measured)
        and int(session_info.get("session_start_count") or 0) == 1
        and int(session_info.get("session_close_count") or 0) == 1
        and int(session_info.get("session_restart_count") or 0) == 0
    )
    report = [
        "# L1 + Corridor Smac Hybrid Candidate Smoke Report",
        "",
        "This is an independent two-layer candidate smoke, not a formal paired experiment.",
        "",
        "## Architecture and scope",
        "",
        "- Three-layer reference: L1 skeleton topology + L2 corridor Grid A* + local L3 Smac.",
        "- Candidate: L1 skeleton topology + one complete corridor-constrained L3' Smac Hybrid DUBIN search.",
        f"- Raw measured queries: `{', '.join(RAW_QUERY_IDS)}`; warmups={warmups}; measured repetitions={repetitions}.",
        "- Original q00 is not modified and is excluded from the formal raw-query rate; any optional run is diagnostic-only.",
        "- RRTstar and SST are disabled and are not a fallback path.",
        "",
        "## Results",
        "",
        f"- Final valid measured paths: **{valid_count}/{len(measured)}**.",
        f"- L2 calls: **{l2_calls}**; L3' Smac calls: **{l3_calls}**.",
        f"- Online pipeline P50/P95/P99: **{_percentile(online, 50):.2f}/{_percentile(online, 95):.2f}/{_percentile(online, 99):.2f} ms**.",
        f"- L1 topology cache: hit={bool(topology_info.get('topology_cache_hit'))}, key=`{topology_info.get('topology_cache_key', '')}`, skeleton backend=`{topology_info.get('skeleton_backend', 'unknown')}`.",
        f"- Smac session start/close/restart: **{session_info.get('session_start_count', 0)}/{session_info.get('session_close_count', 0)}/{session_info.get('session_restart_count', 0)}**.",
        f"- Formal corridor profile: **{default_profile}**; semantics=`{next((row.get('corridor_semantics') for row in measured if row.get('corridor_semantics')), CORRIDOR_SEMANTICS)}`.",
        "- Per-query measured validity and structured failures: "
        + "; ".join(
            f"{query_id}={item['valid']}/{item['total']} ({','.join(item['failures']) or 'none'})"
            for query_id, item in per_query.items()
        ) + ".",
        "",
        "## Corridor and resource diagnostics",
        "",
        "Each measured row records corridor mask hash, dimensions, allowed cells, free-cell count, corridor area ratio, topology route IDs, Hybrid wall/planner time, peak RSS/PSS, and a structured failure code.",
        "The planner result is accepted only when the returned path remains inside the corridor mask and passes the unchanged full static footprint and kinematic validator.",
        "Smac start/goal cost is recorded as `not_available` because this client does not subscribe to the internal costmap cell-cost service; no cost is inferred.",
        "",
        "## Profile diagnostics",
        "",
    ]
    for item in _profile_summary_rows(diagnostic_rows):
        report.append(
            f"- `{item['profile']}` ({item['corridor_semantics']}, padding={item['corridor_padding_m']} m): "
            f"valid={item['valid_count']}/{item['query_count']}, calls={item['l3_prime_call_count']}, "
            f"mean wall={item['mean_pipeline_wall_time_ms']:.2f} ms, failures={','.join(item['failure_codes']) or 'none'}."
        )
    report.extend([
        "",
        "## V7 comparison boundary",
        "",
    ])
    if v7_summary:
        report.extend([
            f"- V7 reference measured rows: {v7_summary['valid']}/{v7_summary['rows']} valid; L3 calls={v7_summary['calls']}; online P50/P95={v7_summary['p50']:.2f}/{v7_summary['p95']:.2f} ms.",
            f"- Candidate measured rows: {valid_count}/{len(measured)} valid; L3' calls={l3_calls}; online P50/P95={_percentile(online, 50):.2f}/{_percentile(online, 95):.2f} ms.",
            "These observations use the same map/query/footprint protocol but are not a paired multi-map conclusion.",
        ])
    else:
        report.append("- No V7 reference CSV was available at run time; comparison remains pending.")
    report.extend([
        "",
        "## Gate decision",
        "",
        f"- Two-layer smoke functional gate: **{'PASS' if gate else 'FAIL'}**.",
        "- Entry to a formal three-layer versus two-layer paired experiment: **LOCKED** until this candidate is reviewed and a separate approved paired protocol is run.",
        "",
        "The smoke does not claim that the two-layer candidate is superior. It only establishes whether the independent implementation produced auditable paths under the frozen constraints.",
    ])
    (output / "final_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return {
        "gate_passed": gate,
        "measured_count": len(measured),
        "valid_count": valid_count,
        "online_p50_ms": _percentile(online, 50),
        "online_p95_ms": _percentile(online, 95),
        "online_p99_ms": _percentile(online, 99),
        "l2_call_count": l2_calls,
        "l3_prime_call_count": l3_calls,
        "v7_summary": v7_summary,
    }


def run_smoke(
    output: Path,
    *,
    map_ids: Sequence[str] = DEFAULT_MAP_IDS,
    query_ids: Sequence[str] = RAW_QUERY_IDS,
    warmups: int = WARMUPS,
    repetitions: int = REPETITIONS,
    include_q00: bool = False,
    topology_cache_dir: Optional[Path] = None,
    v7_runs: Optional[Path] = None,
    ros_domain_id: Optional[int] = None,
    cache_mode: str = CACHE_MODE_BASELINE,
) -> Path:
    if warmups < 0 or repetitions <= 0:
        raise ValueError("warmups must be >= 0 and repetitions must be > 0")
    if cache_mode not in {CACHE_MODE_BASELINE, CACHE_MODE_OPTIMIZED}:
        raise ValueError(f"unsupported cache mode: {cache_mode}")
    requested = list(dict.fromkeys(query_ids))
    invalid = [item for item in requested if item not in RAW_QUERY_IDS]
    if invalid:
        raise ValueError(f"only raw q02/q06/q07/q09 are allowed for formal smoke: {invalid}")
    if list(map_ids) != ["hospital_005"]:
        raise ValueError("the candidate smoke is bounded to hospital_005")
    _refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    (output / "topology").mkdir()
    queries = _queries()
    selected_ids = list(requested) + (["q00"] if include_q00 else [])
    selected_queries = [queries[item] for item in selected_ids]
    source_commit = _source_commit()
    source_files, code_hash = _source_manifest(output, source_commit)
    topology_source_hash = sha256_file(Path(__file__).resolve().parent / "topology.py")
    contexts = {map_id: legacy._context(map_id) for map_id in map_ids}
    topology_cache_root = (topology_cache_dir or V7_TOPOLOGY_CACHE).resolve()
    run_rows: List[Dict[str, Any]] = []
    call_rows: List[Dict[str, Any]] = []
    metric_rows: List[Dict[str, Any]] = []
    diagnostic_rows: List[Dict[str, Any]] = []
    topology_infos: Dict[str, Dict[str, Any]] = {}
    topologies: Dict[str, TopologyArtifact] = {}
    for map_id, ctx in contexts.items():
        artifact, info = _load_authoritative_topology(
            map_id, ctx, topology_cache_root, source_commit, topology_source_hash,
            output / "topology_cache",
        )
        topologies[map_id] = artifact
        topology_infos[map_id] = info
        (output / "topology" / f"{map_id}_metadata.yaml").write_text(
            yaml.safe_dump({"cache_info": info, "artifact_metadata": artifact.metadata}, sort_keys=False),
            encoding="utf-8",
        )

    smac_spec = legacy.backend_availability()["hybrid_astar"]
    session: Any = None
    session_start_error = ""
    selected_ros_domain = _configure_ros_domain(ros_domain_id)
    session_started_ns = time.monotonic_ns()
    if smac_spec.available:
        try:
            ctx = contexts["hospital_005"]
            session = SmacSession(
                ctx, output, map_yaml=ctx.map_yaml,
                log_tag="l1_l3_corridor_hospital_005",
                local_mask_updates=True,
                optimization_profile="v7_candidate",
                optimization_stage="step3_delta_map",
            )
            session.start()
        except Exception as exc:
            session_start_error = str(exc)
            if session is not None:
                try:
                    session.close()
                except Exception:
                    pass
            session = None
    session_start_ms = (time.monotonic_ns() - session_started_ns) / 1.0e6
    default_profile = "raw_map_smac_aligned_1m"
    default_semantics = CORRIDOR_SEMANTICS
    default_padding = CORRIDOR_PADDING_M
    try:
        ctx = contexts["hospital_005"]

        # Bounded diagnostic sweep. These rows are auditable but excluded from
        # the formal measured success-rate denominator.
        for profile_name, semantics, padding in _profile_specs():
            for query in selected_queries:
                diagnostic_mode = f"diagnostic_{profile_name}"
                row, call, metric = _run_one(
                    ctx, topologies["hospital_005"], topology_infos["hospital_005"],
                    query, diagnostic_mode, 1, session, smac_spec, output, source_commit,
                    corridor_padding_m=float(padding or 0.0),
                    corridor_semantics=semantics,
                    profile_name=profile_name,
                    cache_mode=cache_mode,
                )
                row["source_hash"] = code_hash
                run_rows.append(row)
                call_rows.append(call)
                metric_rows.append(metric)
                diagnostic_rows.append(row)

        # Select a single raw-map profile for all formal queries. Ranking is
        # global and deterministic: maximize valid profiles, then minimize
        # padding. No query-specific profile selection is permitted.
        aligned_specs = {
            name: (semantics, padding)
            for name, semantics, padding in _profile_specs()
            if semantics == CORRIDOR_SEMANTICS
        }
        profile_scores: List[Tuple[int, float, str]] = []
        for profile_name, (_semantics, padding) in aligned_specs.items():
            rows = [row for row in diagnostic_rows if row.get("corridor_profile") == profile_name]
            valid = sum(_truth(row.get("final_valid_success")) for row in rows)
            profile_scores.append((valid, -float(padding or 0.0), profile_name))
        if profile_scores:
            _valid, _negative_padding, default_profile = max(profile_scores)
            default_semantics, default_padding = aligned_specs[default_profile]

        selected_runs = (
            [(query, "warmup", index + 1) for query in selected_queries for index in range(warmups)]
            + [(query, "measured", index + 1) for query in selected_queries for index in range(repetitions)]
        )
        for query, run_mode, repetition in selected_runs:
            row, call, metric = _run_one(
                ctx, topologies["hospital_005"], topology_infos["hospital_005"],
                query, run_mode, repetition, session, smac_spec, output, source_commit,
                corridor_padding_m=default_padding,
                corridor_semantics=default_semantics,
                profile_name=default_profile,
                cache_mode=cache_mode,
            )
            row["source_hash"] = code_hash
            run_rows.append(row)
            call_rows.append(call)
            metric_rows.append(metric)
    finally:
        session_info = {
            "map_id": "hospital_005",
            "session_start_count": int(getattr(session, "session_start_count", 0)) if session is not None else 0,
            "session_close_count": 0,
            "session_restart_count": int(getattr(session, "session_restart_count", 0)) if session is not None else 0,
            "session_startup_time_ms": float(getattr(session, "stack_startup_time_ms", session_start_ms)) if session is not None else session_start_ms,
            "session_shutdown_time_ms": 0.0,
            "session_start_error": session_start_error,
            "ros_domain_id": selected_ros_domain,
            "default_corridor_profile": default_profile,
            "diagnostic_profile_count": len(_profile_specs()),
        }
        if session is not None:
            try:
                session.close()
            finally:
                session_info.update({
                    "session_close_count": int(getattr(session, "session_close_count", 0)),
                    "session_shutdown_time_ms": float(getattr(session, "stack_shutdown_time_ms", 0.0)),
                })

    # The session is map-owned and closes only after the final query.  Patch
    # the already-computed rows with the authoritative lifecycle counters so
    # runs.csv and session_timing.csv express the same state.
    for row in run_rows:
        row["session_start_count"] = session_info["session_start_count"]
        row["session_close_count"] = session_info["session_close_count"]
        row["session_restart_count"] = session_info["session_restart_count"]
        row["session_startup_time_ms"] = session_info["session_startup_time_ms"]
        row["session_shutdown_time_ms"] = session_info["session_shutdown_time_ms"]

    _write_csv(output / "runs.csv", run_rows)
    _write_csv(output / "path_metrics.csv", metric_rows)
    _write_csv(output / "backend_call_log.csv", call_rows)
    _write_csv(output / "corridor_profile_comparison.csv", _profile_summary_rows(diagnostic_rows))
    _write_csv(output / "endpoint_costmap_diagnostic.csv", _endpoint_diagnostic_rows(diagnostic_rows))
    _write_csv(output / "repair_window_summary.csv", [], fields=(
        "run_id", "query_id", "run_mode", "repetition", "window_index",
        "window_path_length_m", "attempt_radius_m", "planner_success",
        "validation_passed", "failure_code",
    ))
    _write_csv(output / "session_timing.csv", [session_info])
    topology_info = topology_infos["hospital_005"]
    (output / "topology_cache_manifest.yaml").write_text(
        yaml.safe_dump({"schema_version": 1, "maps": [topology_info]}, sort_keys=False),
        encoding="utf-8",
    )
    (output / "protocol.yaml").write_text(yaml.safe_dump({
        "schema_version": 1,
        "experiment": output.name,
        "architecture": ARCHITECTURE,
        "layers": {
            "L1": "skeleton distance-transform topology + graph A*",
            "L2": "disabled; no two-dimensional grid path generation",
            "L3_prime": "corridor-constrained Nav2 Smac Hybrid DUBIN, one complete request",
        },
        "map_ids": list(map_ids), "query_ids": requested,
        "diagnostic_query_ids": ["q00"] if include_q00 else [],
        "resolution": 0.05, "footprint": FOOTPRINT,
        "minimum_turning_radius_m": 0.40, "maximum_curvature": 2.50,
        "allow_reverse": False, "allow_in_place_rotation": False,
        "dynamic_obstacles": False, "corridor_padding_m": default_padding,
        "corridor_semantics": default_semantics,
        "corridor_profiles": [
            {"profile": name, "semantics": semantics, "padding_m": padding}
            for name, semantics, padding in _profile_specs()
        ],
        "default_corridor_profile": default_profile,
        "cache_mode": cache_mode,
        "footprint_safety_margin_m": FOOTPRINT_SAFETY_MARGIN_M,
        "bend_margin_m": BEND_MARGIN_M,
        "endpoint_connection_back_m": ENDPOINT_CONNECT_BACK_M,
        "endpoint_connection_forward_m": ENDPOINT_CONNECT_FORWARD_M,
        "ros_domain_id": selected_ros_domain,
        "warmups": warmups, "repetitions": repetitions,
        "topology_cache_directory": str(topology_cache_root),
        "topology_cache_metadata_exact": True,
        "l2_called": False, "l2_call_count": 0,
        "disabled_optional_backends": ["OMPL geometric::RRTstar", "OMPL control::SST"],
        "timing_semantics": {
            "pipeline_wall_time_ms": "complete A2B request excluding topology preparation and session lifecycle",
            "hybrid_planning_time_ms": "Smac planner-reported internal duration",
        },
    }, sort_keys=False), encoding="utf-8")
    query_manifest = {
        "schema_version": 1,
        "source_query_file": str(SOURCE_QUERIES),
        "queries": [
            {**queries[item].as_dict(), "derived": False, "diagnostic_only": False}
            for item in requested
        ] + ([
            {**queries["q00"].as_dict(), "derived": False, "diagnostic_only": True}
        ] if include_q00 else []),
    }
    (output / "queries.yaml").write_text(yaml.safe_dump(query_manifest, sort_keys=False), encoding="utf-8")
    (output / "source_manifest.yaml").write_text(yaml.safe_dump({
        "source_commit": source_commit,
        "source_hash": code_hash,
        "code_hash": code_hash,
        "source_files": source_files,
        "map_hashes": {map_id: contexts[map_id].map_sha256 for map_id in map_ids},
        "map_yaml_hashes": {map_id: contexts[map_id].map_yaml_sha256 for map_id in map_ids},
        "query_hashes": {item: _query_hash(queries[item]) for item in selected_ids},
    }, sort_keys=False), encoding="utf-8")
    result_summary = _write_report(
        output, run_rows, topology_info, session_info,
        warmups=warmups, repetitions=repetitions,
        v7_runs=v7_runs or (ROOT / "experiments/layered_planner_benchmark/"
                            "fixed_layered_pipeline_v7_online_efficiency_postfix5_final/"
                            "selected_v7_candidate/runs.csv"),
        diagnostic_rows=diagnostic_rows,
        default_profile=default_profile,
    )
    (output / "manifest.yaml").write_text(yaml.safe_dump({
        "schema_version": 1,
        "experiment": output.name,
        "architecture": ARCHITECTURE,
        "map_ids": list(map_ids), "query_ids": requested,
        "diagnostic_query_ids": ["q00"] if include_q00 else [],
        "formal_query_count": len(requested),
        "warmup_count": warmups, "measured_repetitions": repetitions,
        "run_count": len(run_rows),
        "diagnostic_run_count": len(diagnostic_rows),
        "gate_passed": result_summary["gate_passed"],
        "formal_scale_benchmark_unlocked": False,
        "topology_build_count": sum(int(topology_infos[m].get("topology_build_count") or 0) for m in map_ids),
        "topology_load_count": sum(int(topology_infos[m].get("topology_load_count") or 0) for m in map_ids),
        "topology_cache_hit": all(bool(topology_infos[m].get("topology_cache_hit")) for m in map_ids),
        "session_start_count": session_info["session_start_count"],
        "session_close_count": session_info["session_close_count"],
        "session_restart_count": session_info["session_restart_count"],
        "session_start_error": session_info.get("session_start_error", ""),
        "ros_domain_id": selected_ros_domain,
        "corridor_semantics": default_semantics,
        "default_corridor_profile": default_profile,
        "cache_mode": cache_mode,
        "l2_called": False, "l2_call_count": 0,
        "l3_prime_call_count": result_summary["l3_prime_call_count"],
        "rrtstar_call_count": 0, "sst_call_count": 0,
        **result_summary,
    }, sort_keys=False), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the independent L1 topology + corridor-wide Smac Hybrid candidate smoke",
    )
    parser.add_argument("--output-dir", default=str(ROOT / "experiments/layered_planner_benchmark" / OUTPUT_NAME))
    parser.add_argument("--map-id", action="append", choices=list(DEFAULT_MAP_IDS), dest="map_ids")
    parser.add_argument("--query-id", action="append", choices=list(RAW_QUERY_IDS), dest="query_ids")
    parser.add_argument("--warmups", type=int, default=WARMUPS)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--topology-cache-dir", default=str(V7_TOPOLOGY_CACHE))
    parser.add_argument("--ros-domain-id", type=int, default=None,
                        help="isolated ROS domain (default: deterministic per-process domain)")
    parser.add_argument("--cache-mode", choices=(CACHE_MODE_BASELINE, CACHE_MODE_OPTIMIZED),
                        default=CACHE_MODE_BASELINE,
                        help="L1 cache strategy; optimized falls back to baseline on validation failure")
    parser.add_argument("--include-q00", action="store_true", help="record original q00 as diagnostic-only")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_smoke(
            Path(args.output_dir).resolve(),
            map_ids=args.map_ids or DEFAULT_MAP_IDS,
            query_ids=args.query_ids or RAW_QUERY_IDS,
            warmups=args.warmups, repetitions=args.repetitions,
            include_q00=args.include_q00,
            topology_cache_dir=Path(args.topology_cache_dir).resolve(),
            ros_domain_id=args.ros_domain_id,
            cache_mode=args.cache_mode,
        )
    except (OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f"l1_l3_corridor_hybrid_smoke: ERROR: {exc}")
        return 2
    gate = bool((yaml.safe_load((output / "manifest.yaml").read_text(encoding="utf-8")) or {}).get("gate_passed"))
    print(f"smoke output: {output}")
    return 0 if gate else 1


if __name__ == "__main__":
    raise SystemExit(main())
