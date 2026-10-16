"""CLI for the static Hospital topology benchmark."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import math
import os
import resource
import sys
import time
import threading
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import pandas as pd
import numpy as np
import yaml

from .planner_benchmark.config import CONFIG_ROOT, load_protocol, load_queries, resolve_path
from .planner_benchmark.map_utils import HospitalMap
from .planner_benchmark.path_metrics import interpolate_path
from .planner_benchmark.resources import read_snapshot
from .topology import (
    AStarResult,
    TOPOLOGY_ALGORITHM_VERSION,
    TopologyArtifact,
    TopologyRoute,
    astar_grid,
    attach_pose,
    build_topology,
    cells_to_poses,
    corridor_mask,
    load_topology,
    map_input_hash,
    path_length_cells,
    save_path,
    save_topology,
    search_topology,
    static_collision_count,
)


DEFAULT_FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]
MODES = ["full_grid", "topology_only", "topology_guided_grid", "topology_guided_grid_fallback"]
REPORT_ONLY_MODES: List[str] = []
CORRIDOR_PADDING_SEQUENCE_M = (1.0, 2.0, 4.0)
FORBIDDEN_DYNAMIC_TOKENS = (
    "tm_obstacles:=random",
    "tm_obstacles:=scenario",
    "default.json",
    "pedsim",
    "hunav",
    "actor",
)


def _assert_static_cli(argv: Sequence[str]) -> None:
    joined = " ".join(str(value).lower() for value in argv)
    for token in FORBIDDEN_DYNAMIC_TOKENS:
        if token in joined:
            raise ValueError(f"dynamic obstacle input is forbidden in topology benchmark: {token}")


def _now_ns() -> int:
    return time.monotonic_ns()


def _cpu_seconds() -> float:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return float(usage.ru_utime + usage.ru_stime)


def _peak_rss_bytes() -> int:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    # Linux reports KiB for ru_maxrss.
    return int(usage.ru_maxrss) * 1024


class _QueryResourceMonitor:
    """Small in-process sampler reusing the benchmark /proc snapshot reader."""

    def __init__(self, interval_ms: float = 10.0):
        self.interval_s = max(float(interval_ms), 1.0) / 1000.0
        self.pid = os.getpid()
        self.before = None
        self.samples = []
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        self.before = read_snapshot(self.pid)
        self._thread = threading.Thread(target=self._sample, name="topology-query-resource-monitor", daemon=True)
        self._thread.start()

    def finish(self, wall_time_ms: float) -> Dict[str, object]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.1, self.interval_s * 5.0))
        after = read_snapshot(self.pid)
        self.samples.append(after)
        valid = [sample for sample in self.samples if sample is not None]
        if self.before is None or after is None:
            return {
                "cpu_user_ms": None, "cpu_system_ms": None, "cpu_total_ms": None,
                "cpu_percent": None, "query_rss_before_bytes": None,
                "query_rss_peak_bytes": None, "query_pss_before_bytes": None,
                "query_pss_peak_bytes": None, "resource_error": "process snapshot unavailable",
                "resource_sample_interval_ms": self.interval_s * 1000.0,
            }
        user = max(0.0, after.cpu_user_ms - self.before.cpu_user_ms)
        system = max(0.0, after.cpu_system_ms - self.before.cpu_system_ms)
        rss = [int(item.rss_bytes) for item in valid if item.rss_bytes is not None]
        pss = [int(item.pss_bytes) for item in valid if item.pss_bytes is not None]
        return {
            "cpu_user_ms": user, "cpu_system_ms": system,
            "cpu_total_ms": user + system,
            "cpu_percent": (user + system) / wall_time_ms * 100.0 if wall_time_ms > 0 else None,
            "query_rss_before_bytes": self.before.rss_bytes,
            "query_rss_peak_bytes": max(rss) if rss else None,
            "query_pss_before_bytes": self.before.pss_bytes,
            "query_pss_peak_bytes": max(pss) if pss else None,
            "resource_error": "" if rss and pss else "RSS/PSS snapshot unavailable",
            "resource_sample_interval_ms": self.interval_s * 1000.0,
        }

    def _sample(self) -> None:
        while not self._stop.is_set():
            self.samples.append(read_snapshot(self.pid))
            self._stop.wait(self.interval_s)


def _resolve_hospital_map(protocol_path: Path, protocol: Dict[str, object], map_name: str) -> Path:
    if map_name not in {"hospital", "hospital_005"}:
        raise ValueError(f"only the static Hospital-derived maps are supported: {map_name}")
    value = protocol.get("map_yaml", "external/arena4_ws/src/arena/simulation-setup/worlds/hospital/map/map.yaml")
    return resolve_path(str(value), base=protocol_path.parent)


def _write_frame(path: Path, rows: Iterable[Dict[str, object]]) -> None:
    frame = pd.DataFrame(list(rows))
    path.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(path, index=False)


def _distance(points: Sequence[Sequence[float]]) -> float:
    return sum(math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])) for a, b in zip(points, points[1:]))


def corridor_padding_schedule(base_padding_m: float) -> Tuple[float, ...]:
    """Return the frozen metre-defined corridor expansion schedule."""
    base = float(base_padding_m)
    if not math.isclose(base, CORRIDOR_PADDING_SEQUENCE_M[0], rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("Phase 6 requires corridor padding sequence 1.0, 2.0, 4.0 m")
    return CORRIDOR_PADDING_SEQUENCE_M


def _path_clearance(artifact: TopologyArtifact, path: Sequence[Tuple[int, int]]) -> float:
    values = [float(artifact.distance_m[cell]) for cell in path]
    return min(values, default=0.0)


def _merge_astar_results(results: Sequence[AStarResult], path, resolution: float) -> AStarResult:
    """Aggregate work from sequential corridor attempts and optional fallback."""
    valid = [item for item in results if item is not None]
    if not valid:
        return AStarResult(None, 0, 0, 0, 0, 0, 0.0, None, 0.0, "NO_PATH")
    final = valid[-1]
    total_free = max(item.total_free_grid_cells for item in valid)
    # The final allowed set is the most useful search-space value; all
    # attempts are still represented by the aggregate work counters.
    return AStarResult(
        path=path,
        expanded_nodes=sum(item.expanded_nodes for item in valid),
        generated_nodes=sum(item.generated_nodes for item in valid),
        max_open_set_size=max(item.max_open_set_size for item in valid),
        allowed_grid_cells=final.allowed_grid_cells,
        total_free_grid_cells=total_free,
        search_space_ratio=(float(final.allowed_grid_cells / total_free) if total_free else 0.0),
        path_cost=(path_length_cells(path, resolution) if path is not None else None),
        search_time_ms=sum(item.search_time_ms for item in valid),
        failure_code="" if path is not None else final.failure_code,
    )


def _result_row(
    *,
    run_id: str,
    query,
    mode: str,
    repetition: int,
    status: str,
    final_planner: str,
    fallback_used: bool,
    fallback_reason: str,
    fallback_attempts: int,
    attach_success: bool,
    graph_route_success: bool,
    corridor_search_success: bool,
    final_success: bool,
    topology_false_failure: bool,
    topology_collision_count: int,
    topology_route: Optional[TopologyRoute],
    final_path: Optional[Sequence[Tuple[int, int]]],
    artifact: TopologyArtifact,
    attach_time_ms: float,
    graph_time_ms: float,
    corridor_time_ms: float,
    grid_time_ms: float,
    fallback_time_ms: float,
    footprint: Sequence[Sequence[float]],
    astar_stats: Optional[AStarResult] = None,
    source: str = "grid",
    grid_mode: str = "",
    topology_edge_ids: Optional[Sequence[int]] = None,
    corridor_padding_used_m: Optional[float] = None,
    corridor_attempts: Optional[int] = None,
    resource: Optional[Dict[str, object]] = None,
) -> Tuple[Dict[str, object], Dict[str, object]]:
    path_file = ""
    path_length = None
    clearance = None
    collision_count = 0
    if final_path:
        path_length = path_length_cells(final_path, artifact.hospital_map.resolution)
        clearance = _path_clearance(artifact, final_path)
        poses = interpolate_path(
            cells_to_poses(artifact, final_path, query.start[2], query.goal[2]),
            artifact.hospital_map.resolution / 2.0,
        )
        collision_count = static_collision_count(artifact, poses, footprint, allow_unknown=False)
    route_length = topology_route.length_m if topology_route else None
    route_width = topology_route.min_width_m if topology_route else None
    result_code = "SUCCEEDED" if final_success else status
    stats = astar_stats
    resource = resource or {}
    action_success = bool(final_success)
    static_valid = bool(final_path is not None and collision_count == 0)
    run = {
        "run_id": run_id,
        "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "map_id": artifact.hospital_map.map_id,
        "map_sha256": artifact.metadata["map_sha256"],
        "query_id": query.query_id,
        "query_category": query.category,
        "mode": mode,
        "repetition": repetition,
        "topology_status": status,
        "fallback_used": bool(fallback_used),
        "fallback_reason": fallback_reason,
        "fallback_attempts": fallback_attempts,
        "final_planner": final_planner,
        "attach_success": bool(attach_success),
        "graph_route_success": bool(graph_route_success),
        "corridor_search_success": bool(corridor_search_success),
        "final_success": bool(final_success),
        "action_success": action_success,
        "static_footprint_valid": static_valid,
        "final_valid_success": bool(action_success and static_valid),
        "result_code": result_code,
        "topology_false_failure": bool(topology_false_failure),
        "action_success": action_success,
        "static_footprint_valid": static_valid,
        "final_valid_success": bool(action_success and static_valid),
        "topology_collision_count": int(topology_collision_count),
        "static_footprint_collision_count": int(collision_count),
        "topology_attach_time_ms": attach_time_ms,
        "topology_graph_search_time_ms": graph_time_ms,
        "corridor_build_time_ms": corridor_time_ms,
        "corridor_grid_search_time_ms": grid_time_ms,
        "full_grid_fallback_time_ms": fallback_time_ms,
        "total_topology_query_time_ms": attach_time_ms + graph_time_ms + corridor_time_ms + grid_time_ms + fallback_time_ms,
        "topology_route_length_m": route_length,
        "topology_route_width_min_m": route_width,
        "final_path_length_m": path_length,
        "minimum_clearance_m": clearance,
        "path_file": path_file,
        "path_point_count": len(final_path or []),
        "source": source,
        "grid_mode": grid_mode,
        "topology_edge_ids": list(topology_edge_ids or []),
        "corridor_padding_used_m": corridor_padding_used_m,
        "corridor_attempts": corridor_attempts if corridor_attempts is not None else fallback_attempts,
        "expanded_nodes": stats.expanded_nodes if stats else None,
        "generated_nodes": stats.generated_nodes if stats else None,
        "max_open_set_size": stats.max_open_set_size if stats else None,
        "allowed_grid_cells": stats.allowed_grid_cells if stats else None,
        "total_free_grid_cells": stats.total_free_grid_cells if stats else int(np.count_nonzero(artifact.free_mask)),
        "search_space_ratio": stats.search_space_ratio if stats else None,
        "path_cost": stats.path_cost if stats else None,
        "search_time_ms": stats.search_time_ms if stats else None,
        "failure_code": stats.failure_code if stats else ("" if final_success else status),
        "cpu_user_ms": resource.get("cpu_user_ms"),
        "cpu_system_ms": resource.get("cpu_system_ms"),
        "cpu_total_ms": resource.get("cpu_total_ms"),
        "cpu_percent": resource.get("cpu_percent"),
        "query_rss_before_bytes": resource.get("query_rss_before_bytes"),
        "query_rss_peak_bytes": resource.get("query_rss_peak_bytes"),
        "query_pss_before_bytes": resource.get("query_pss_before_bytes"),
        "query_pss_peak_bytes": resource.get("query_pss_peak_bytes"),
        "resource_sample_interval_ms": resource.get("resource_sample_interval_ms"),
        "resource_error": resource.get("resource_error", ""),
    }
    metrics = {
        "run_id": run_id,
        "query_id": query.query_id,
        "mode": mode,
        "repetition": repetition,
        "topology_path_length_m": route_length,
        "full_grid_path_length_m": path_length if final_planner == "full_grid_astar" else None,
        "length_over_full_grid": None,
        "final_path_length_m": path_length,
        "length_over_euclidean": path_length / math.hypot(query.goal[0] - query.start[0], query.goal[1] - query.start[1]) if path_length else None,
        "minimum_clearance_m": clearance,
        "footprint_collision_count": collision_count,
        "topology_route_length_m": route_length,
        "topology_route_width_min_m": route_width,
        "topology_false_failure": bool(topology_false_failure),
        "source": source,
        "grid_mode": grid_mode,
        "topology_edge_ids": list(topology_edge_ids or []),
        "corridor_padding_used_m": corridor_padding_used_m,
        "expanded_nodes": stats.expanded_nodes if stats else None,
        "generated_nodes": stats.generated_nodes if stats else None,
        "max_open_set_size": stats.max_open_set_size if stats else None,
        "allowed_grid_cells": stats.allowed_grid_cells if stats else None,
        "total_free_grid_cells": stats.total_free_grid_cells if stats else int(np.count_nonzero(artifact.free_mask)),
        "search_space_ratio": stats.search_space_ratio if stats else None,
        "path_cost": stats.path_cost if stats else None,
        "search_time_ms": stats.search_time_ms if stats else None,
        "failure_code": stats.failure_code if stats else ("" if final_success else status),
    }
    return run, metrics


def _full_grid(artifact: TopologyArtifact, query) -> AStarResult:
    start = artifact.hospital_map.world_to_cell(query.start[0], query.start[1])
    goal = artifact.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    if start is None or goal is None:
        return AStarResult(None, 0, 0, 0, int(np.count_nonzero(artifact.free_mask)), int(np.count_nonzero(artifact.free_mask)), 1.0, None, 0.0, "INVALID_ENDPOINT")
    return astar_grid(artifact.free_mask, start, goal, resolution=artifact.hospital_map.resolution, return_stats=True)


def _full_grid_fallback_result(
    artifact: TopologyArtifact,
    query,
    footprint,
    mode: str,
    reason: str,
    *,
    attach_ms: float,
    graph_ms: float,
    attach_success: bool,
    route: Optional[TopologyRoute] = None,
) -> Tuple[Dict[str, object], Dict[str, object], Optional[List[Tuple[int, int]]]]:
    started = _now_ns()
    astar_result = _full_grid(artifact, query)
    path = astar_result.path
    fallback_ms = (_now_ns() - started) / 1e6
    final_success = path is not None
    status = "FULL_GRID_FALLBACK" if final_success else "FULL_GRID_FAILED"
    run, metric = _result_row(
        run_id="", query=query, mode=mode, repetition=0, status=status,
        final_planner="full_grid_astar" if final_success else "none", fallback_used=True,
        fallback_reason=reason, fallback_attempts=0, attach_success=attach_success,
        graph_route_success=route is not None, corridor_search_success=False,
        final_success=final_success, topology_false_failure=False, topology_collision_count=0,
        topology_route=route, final_path=path, artifact=artifact,
        attach_time_ms=attach_ms, graph_time_ms=graph_ms, corridor_time_ms=0.0,
        grid_time_ms=0.0, fallback_time_ms=fallback_ms, footprint=footprint,
        astar_stats=astar_result, source="grid", grid_mode="full_grid_fallback",
        topology_edge_ids=route.edge_ids if route else [], corridor_padding_used_m=None,
        corridor_attempts=0,
    )
    return run, metric, path


def _route_cells(artifact: TopologyArtifact, route: TopologyRoute, start_cell, goal_cell) -> List[Tuple[int, int]]:
    cells = []
    for x, y in route.polyline:
        cell = artifact.hospital_map.world_to_cell(x, y)
        if cell is not None and (not cells or cells[-1] != cell):
            cells.append(cell)
    if not cells or cells[0] != start_cell:
        cells.insert(0, start_cell)
    if cells[-1] != goal_cell:
        cells.append(goal_cell)
    return cells


def _topology_attempt(
    artifact: TopologyArtifact,
    query,
    footprint,
    mode: str,
    corridor_padding_m: float,
    attach_radius_m: float,
) -> Tuple[Dict[str, object], Dict[str, object], Optional[List[Tuple[int, int]]]]:
    started = _now_ns()
    start_attachment = attach_pose(artifact, query.start, footprint, max_radius_m=attach_radius_m, allow_unknown=False)
    goal_attachment = attach_pose(artifact, query.goal, footprint, max_radius_m=attach_radius_m, allow_unknown=False)
    attach_ms = (_now_ns() - started) / 1e6
    attach_success = start_attachment is not None and goal_attachment is not None
    if mode == "full_grid":
        grid_started = _now_ns()
        astar_result = _full_grid(artifact, query)
        path = astar_result.path
        grid_ms = (_now_ns() - grid_started) / 1e6
        status = "TOPOLOGY_NOT_USED"
        final = path is not None
        run, metric = _result_row(
            run_id="", query=query, mode=mode, repetition=0, status=status,
            final_planner="full_grid_astar", fallback_used=False, fallback_reason="", fallback_attempts=0,
            attach_success=True, graph_route_success=True, corridor_search_success=True, final_success=final,
            topology_false_failure=False, topology_collision_count=0, topology_route=None, final_path=path,
            artifact=artifact, attach_time_ms=0.0, graph_time_ms=0.0, corridor_time_ms=0.0,
            grid_time_ms=grid_ms, fallback_time_ms=0.0, footprint=footprint,
            astar_stats=astar_result, source="grid", grid_mode="full_grid",
            corridor_attempts=0,
        )
        if not final:
            run["topology_status"] = "FULL_GRID_FAILED"
            run["result_code"] = "FULL_GRID_FAILED"
        return run, metric, path
    if not start_attachment:
        status = "TOPOLOGY_START_NOT_ATTACHABLE"
        if mode == "topology_guided_grid_fallback":
            return _full_grid_fallback_result(
                artifact, query, footprint, mode, status,
                attach_ms=attach_ms, graph_ms=0.0, attach_success=False,
            )
        run, metric = _result_row(
            run_id="", query=query, mode=mode, repetition=0, status=status, final_planner="none",
            fallback_used=False, fallback_reason=status, fallback_attempts=0, attach_success=False,
            graph_route_success=False, corridor_search_success=False, final_success=False,
            topology_false_failure=False, topology_collision_count=0, topology_route=None, final_path=None,
            artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=0.0, corridor_time_ms=0.0,
            grid_time_ms=0.0, fallback_time_ms=0.0, footprint=footprint,
        )
        return run, metric, None
    if not goal_attachment:
        status = "TOPOLOGY_GOAL_NOT_ATTACHABLE"
        if mode == "topology_guided_grid_fallback":
            return _full_grid_fallback_result(
                artifact, query, footprint, mode, status,
                attach_ms=attach_ms, graph_ms=0.0, attach_success=False,
            )
        run, metric = _result_row(
            run_id="", query=query, mode=mode, repetition=0, status=status, final_planner="none",
            fallback_used=False, fallback_reason=status, fallback_attempts=0, attach_success=False,
            graph_route_success=False, corridor_search_success=False, final_success=False,
            topology_false_failure=False, topology_collision_count=0, topology_route=None, final_path=None,
            artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=0.0, corridor_time_ms=0.0,
            grid_time_ms=0.0, fallback_time_ms=0.0, footprint=footprint,
        )
        return run, metric, None
    if start_attachment.component_id != goal_attachment.component_id:
        status = "TOPOLOGY_COMPONENT_MISMATCH"
        if mode == "topology_guided_grid_fallback":
            return _full_grid_fallback_result(
                artifact, query, footprint, mode, status,
                attach_ms=attach_ms, graph_ms=0.0, attach_success=True,
            )
        run, metric = _result_row(
            run_id="", query=query, mode=mode, repetition=0, status=status, final_planner="none",
            fallback_used=False, fallback_reason=status, fallback_attempts=0, attach_success=True,
            graph_route_success=False, corridor_search_success=False, final_success=False,
            topology_false_failure=False, topology_collision_count=0, topology_route=None, final_path=None,
            artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=0.0, corridor_time_ms=0.0,
            grid_time_ms=0.0, fallback_time_ms=0.0, footprint=footprint,
        )
        return run, metric, None
    graph_started = _now_ns()
    route = search_topology(artifact, start_attachment.node_id, goal_attachment.node_id)
    graph_ms = (_now_ns() - graph_started) / 1e6
    if route is None:
        status = "TOPOLOGY_NO_ROUTE"
        if mode == "topology_guided_grid_fallback":
            return _full_grid_fallback_result(
                artifact, query, footprint, mode, status,
                attach_ms=attach_ms, graph_ms=graph_ms, attach_success=True,
            )
        run, metric = _result_row(
            run_id="", query=query, mode=mode, repetition=0, status=status, final_planner="none",
            fallback_used=False, fallback_reason=status, fallback_attempts=0, attach_success=True,
            graph_route_success=False, corridor_search_success=False, final_success=False,
            topology_false_failure=False, topology_collision_count=0, topology_route=None, final_path=None,
            artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=graph_ms, corridor_time_ms=0.0,
            grid_time_ms=0.0, fallback_time_ms=0.0, footprint=footprint,
        )
        return run, metric, None
    if mode == "topology_only":
        run, metric = _result_row(
            run_id="", query=query, mode=mode, repetition=0, status="TOPOLOGY_OK", final_planner="topology_graph",
            fallback_used=False, fallback_reason="", fallback_attempts=0, attach_success=True,
            graph_route_success=True, corridor_search_success=False, final_success=True,
            topology_false_failure=False, topology_collision_count=0, topology_route=route, final_path=None,
            artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=graph_ms, corridor_time_ms=0.0,
            grid_time_ms=0.0, fallback_time_ms=0.0, footprint=footprint,
        )
        return run, metric, None
    start_cell = artifact.hospital_map.world_to_cell(query.start[0], query.start[1])
    goal_cell = artifact.hospital_map.world_to_cell(query.goal[0], query.goal[1])
    corridor_ms = 0.0
    grid_ms = 0.0
    if start_cell is None or goal_cell is None:
        corridor_path = None
    else:
        corridor_started = _now_ns()
        corridor = corridor_mask(artifact, route, start_cell, goal_cell, corridor_padding_m)
        corridor_ms = (_now_ns() - corridor_started) / 1e6
        grid_started = _now_ns()
        corridor_result = astar_grid(
            artifact.free_mask, start_cell, goal_cell, corridor,
            resolution=artifact.hospital_map.resolution, return_stats=True,
        )
        corridor_path = corridor_result.path
        grid_ms = (_now_ns() - grid_started) / 1e6
    if start_cell is None or goal_cell is None:
        corridor_result = AStarResult(None, 0, 0, 0, 0, int(np.count_nonzero(artifact.free_mask)), 0.0, None, 0.0, "INVALID_ENDPOINT")
    elif 'corridor_result' not in locals():
        corridor_result = AStarResult(None, 0, 0, 0, 0, int(np.count_nonzero(artifact.free_mask)), 0.0, None, 0.0, "INVALID_ENDPOINT")
    if corridor_path is not None:
        run, metric = _result_row(
            run_id="", query=query, mode=mode, repetition=0, status="TOPOLOGY_OK", final_planner="corridor_grid_astar",
            fallback_used=False, fallback_reason="", fallback_attempts=1, attach_success=True,
            graph_route_success=True, corridor_search_success=True, final_success=True,
            topology_false_failure=False, topology_collision_count=0, topology_route=route, final_path=corridor_path,
            artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=graph_ms, corridor_time_ms=corridor_ms,
            grid_time_ms=grid_ms, fallback_time_ms=0.0, footprint=footprint,
            astar_stats=corridor_result, source="grid", grid_mode="corridor",
            topology_edge_ids=route.edge_ids, corridor_padding_used_m=corridor_padding_m,
            corridor_attempts=1,
        )
        return run, metric, corridor_path
    if mode == "topology_guided_grid":
        run, metric = _result_row(
            run_id="", query=query, mode=mode, repetition=0, status="CORRIDOR_NO_PATH", final_planner="none",
            fallback_used=False, fallback_reason="corridor_no_path", fallback_attempts=1, attach_success=True,
            graph_route_success=True, corridor_search_success=False, final_success=False,
            topology_false_failure=False, topology_collision_count=0, topology_route=route, final_path=None,
            artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=graph_ms, corridor_time_ms=corridor_ms,
            grid_time_ms=grid_ms, fallback_time_ms=0.0, footprint=footprint,
            astar_stats=corridor_result, source="grid", grid_mode="corridor",
            topology_edge_ids=route.edge_ids, corridor_padding_used_m=corridor_padding_m,
            corridor_attempts=1,
        )
        return run, metric, None
    # Guided-grid with fallback: enlarge the corridor, then use full-grid A*.
    attempts = 1
    expanded_corridor_ms = corridor_ms
    expanded_grid_ms = grid_ms
    corridor_results = [corridor_result]
    used_padding = corridor_padding_m
    for padding in corridor_padding_schedule(corridor_padding_m)[1:]:
        attempts += 1
        started = _now_ns()
        expanded = corridor_mask(artifact, route, start_cell, goal_cell, padding)
        expanded_corridor_ms += (_now_ns() - started) / 1e6
        started = _now_ns()
        expanded_result = astar_grid(
            artifact.free_mask, start_cell, goal_cell, expanded,
            resolution=artifact.hospital_map.resolution, return_stats=True,
        )
        expanded_path = expanded_result.path
        corridor_results.append(expanded_result)
        expanded_grid_ms += (_now_ns() - started) / 1e6
        if expanded_path is not None:
            used_padding = padding
            combined = _merge_astar_results(corridor_results, expanded_result.path, artifact.hospital_map.resolution)
            run, metric = _result_row(
                run_id="", query=query, mode=mode, repetition=0, status="CORRIDOR_EXPANDED", final_planner="corridor_grid_astar",
                fallback_used=False, fallback_reason="corridor_no_path", fallback_attempts=attempts, attach_success=True,
                graph_route_success=True, corridor_search_success=True, final_success=True,
                topology_false_failure=False, topology_collision_count=0, topology_route=route, final_path=expanded_path,
                artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=graph_ms, corridor_time_ms=expanded_corridor_ms,
                grid_time_ms=expanded_grid_ms, fallback_time_ms=0.0, footprint=footprint,
                astar_stats=combined, source="grid", grid_mode="corridor",
                topology_edge_ids=route.edge_ids, corridor_padding_used_m=used_padding,
                corridor_attempts=attempts,
            )
            return run, metric, expanded_path
    started = _now_ns()
    fallback_result = _full_grid(artifact, query)
    fallback_path = fallback_result.path
    fallback_ms = (_now_ns() - started) / 1e6
    run, metric = _result_row(
        run_id="", query=query, mode=mode, repetition=0, status="FULL_GRID_FALLBACK" if fallback_path else "FULL_GRID_FAILED",
        final_planner="full_grid_astar" if fallback_path else "none", fallback_used=True,
        fallback_reason="corridor_no_path", fallback_attempts=attempts, attach_success=True,
        graph_route_success=True, corridor_search_success=False, final_success=fallback_path is not None,
        topology_false_failure=False, topology_collision_count=0, topology_route=route, final_path=fallback_path,
        artifact=artifact, attach_time_ms=attach_ms, graph_time_ms=graph_ms, corridor_time_ms=expanded_corridor_ms,
        grid_time_ms=expanded_grid_ms, fallback_time_ms=fallback_ms, footprint=footprint,
        astar_stats=_merge_astar_results(corridor_results + [fallback_result], fallback_path, artifact.hospital_map.resolution),
        source="grid", grid_mode="full_grid_fallback", topology_edge_ids=route.edge_ids,
        corridor_padding_used_m=used_padding, corridor_attempts=attempts,
    )
    return run, metric, fallback_path


def _update_false_failures(runs: List[Dict[str, object]]) -> None:
    baseline = {(str(row["query_id"]), int(row["repetition"])): bool(row["final_success"])
                for row in runs if row["mode"] == "full_grid"}
    statically_unreachable = {
        str(row["query_id"])
        for row in runs
        if row["mode"] == "full_grid" and not bool(row["final_success"])
    }
    for row in runs:
        key = (str(row["query_id"]), int(row["repetition"]))
        if row["mode"] == "topology_only" and not row["final_success"] and baseline.get(key, False):
            row["topology_false_failure"] = True
            row["topology_status"] = "TOPOLOGY_FALSE_FAILURE"
            row["result_code"] = "TOPOLOGY_FALSE_FAILURE"
        # Preserve the frozen conservative static diagnosis for a query that
        # has no path even in the common full-grid model.  This is derived
        # from reachability, never from a query-specific special case.
        if str(row["query_id"]) in statically_unreachable and not bool(row["final_success"]):
            row["topology_status"] = "STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH"
            row["result_code"] = "STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH"
            row["failure_code"] = "STATIC_SEMANTICS_CONSERVATIVE_INFLATION_MISMATCH"


def _fill_cross_mode_path_ratios(metrics: List[Dict[str, object]]) -> None:
    baselines = {
        (str(row["query_id"]), int(row["repetition"])): row.get("final_path_length_m")
        for row in metrics
        if row["mode"] == "full_grid" and row.get("final_path_length_m") is not None
    }
    for row in metrics:
        baseline = baselines.get((str(row["query_id"]), int(row["repetition"])))
        row["full_grid_path_length_m"] = baseline
        path_length = row.get("final_path_length_m")
        if baseline and path_length is not None:
            row["length_over_full_grid"] = float(path_length) / float(baseline)


def _write_summaries(output: Path, runs: List[Dict[str, object]], metrics: List[Dict[str, object]]) -> None:
    run_frame = pd.DataFrame(runs)
    metric_frame = pd.DataFrame(metrics)
    run_frame.to_csv(output / "query_runs.csv", index=False)
    metric_frame.to_csv(output / "path_metrics.csv", index=False)
    rows = []
    numeric = [
        "topology_attach_time_ms", "topology_graph_search_time_ms", "corridor_build_time_ms",
        "corridor_grid_search_time_ms", "full_grid_fallback_time_ms", "total_topology_query_time_ms",
        "total_online_time_ms", "cpu_total_ms", "cpu_percent", "query_rss_peak_bytes", "query_pss_peak_bytes",
        "expanded_nodes", "generated_nodes", "max_open_set_size", "allowed_grid_cells",
        "total_free_grid_cells", "search_space_ratio", "path_cost", "search_time_ms",
        "topology_route_length_m", "topology_route_width_min_m", "final_path_length_m",
        "full_grid_path_length_m", "length_over_full_grid", "minimum_clearance_m",
        "static_footprint_collision_count", "topology_build_wall_time_ms",
        "amortized_topology_cost_ms", "total_with_amortized_topology_cost_ms",
    ]
    reachable_ids = set(
        run_frame.loc[(run_frame["mode"] == "full_grid") & run_frame["final_valid_success"].astype(bool), "query_id"].astype(str)
    ) if not run_frame.empty else set()
    all_query_count = int(run_frame["query_id"].astype(str).nunique()) if not run_frame.empty else 0
    for mode, group in run_frame.groupby("mode"):
        success = group["final_success"].astype(bool)
        valid_success = group["final_valid_success"].astype(bool)
        reachable = group["query_id"].astype(str).isin(reachable_ids)
        for field in numeric:
            values = pd.to_numeric(group[field], errors="coerce").dropna()
            rows.append({
                "mode": mode, "metric": field, "count": int(values.count()),
                "success_count": int(success.sum()), "success_rate": float(success.mean()),
                "mean": float(values.mean()) if len(values) else None,
                "std": float(values.std(ddof=1)) if len(values) > 1 else (0.0 if len(values) else None),
                "P50": float(values.quantile(0.5)) if len(values) else None,
                "P95": float(values.quantile(0.95)) if len(values) else None,
                "P99": float(values.quantile(0.99)) if len(values) else None,
                "min": float(values.min()) if len(values) else None,
                "max": float(values.max()) if len(values) else None,
                "reachable_query_count": len(reachable_ids),
                "reachable_success_count": int(group.loc[reachable & valid_success, "query_id"].astype(str).nunique()),
                "reachable_success_rate": (float(group.loc[reachable & valid_success, "query_id"].astype(str).nunique() / len(reachable_ids)) if reachable_ids else None),
            })
    pd.DataFrame(rows).to_csv(output / "summary_by_mode.csv", index=False)
    query_rows = []
    for keys, group in run_frame.groupby(["query_id", "mode"]):
        query_id, mode = keys
        success = group["final_success"].astype(bool)
        query_rows.append({
            "query_id": query_id, "mode": mode, "count": len(group),
            "success_count": int(success.sum()), "success_rate": float(success.mean()),
            "mean_total_topology_query_time_ms": float(pd.to_numeric(group["total_topology_query_time_ms"], errors="coerce").mean()),
            "mean_total_online_time_ms": float(pd.to_numeric(group["total_online_time_ms"], errors="coerce").mean()),
            "mean_final_path_length_m": float(pd.to_numeric(group["final_path_length_m"], errors="coerce").mean()),
            "fallback_count": int(group["fallback_used"].astype(bool).sum()),
            "false_failure_count": int(group["topology_false_failure"].astype(bool).sum()),
            "all_query_count": all_query_count,
            "all_query_success_count": int(group.loc[group["final_valid_success"].astype(bool), "query_id"].astype(str).nunique()),
            "all_query_success_rate": float(group.loc[group["final_valid_success"].astype(bool), "query_id"].astype(str).nunique() / all_query_count) if all_query_count else None,
            "reachable_query_count": len(reachable_ids),
            "reachable_success_rate": float(group.loc[reachable & group["final_valid_success"].astype(bool), "query_id"].astype(str).nunique() / len(reachable_ids)) if reachable_ids else None,
        })
    pd.DataFrame(query_rows).to_csv(output / "summary_by_query.csv", index=False)
    fallback = run_frame[
        run_frame["fallback_used"].astype(bool)
        | run_frame["topology_status"].eq("CORRIDOR_EXPANDED")
    ].copy()
    if fallback.empty:
        pd.DataFrame(columns=["mode", "fallback_reason", "count"]).to_csv(output / "fallback_summary.csv", index=False)
    else:
        fallback.groupby(["mode", "fallback_reason"], dropna=False).size().rename("count").reset_index().to_csv(output / "fallback_summary.csv", index=False)
    _write_plots(output / "plots", run_frame)
    # A compact machine-readable table for the Phase 6 acceptance criteria.
    acceptance = []
    for mode, group in run_frame.groupby("mode"):
        valid = group["final_valid_success"].astype(bool)
        reachable = group["query_id"].astype(str).isin(reachable_ids)
        acceptance.append({
            "mode": mode,
            "query_count": int(group["query_id"].nunique()),
            "all_query_success_count": int(group.loc[valid, "query_id"].astype(str).nunique()),
            "all_query_success_rate": float(group.loc[valid, "query_id"].astype(str).nunique() / all_query_count) if all_query_count else None,
            "reachable_query_count": len(reachable_ids),
            "reachable_query_success_count": int(group.loc[reachable & valid, "query_id"].astype(str).nunique()),
            "reachable_query_success_rate": float(group.loc[reachable & valid, "query_id"].astype(str).nunique() / len(reachable_ids)) if reachable_ids else None,
            "static_footprint_valid_rate": float(group["static_footprint_valid"].astype(bool).mean()) if len(group) else None,
            "direct_corridor_success_count": int(((group["grid_mode"] == "corridor") & valid & ~group["fallback_used"].astype(bool)).sum()),
            "expanded_success_count": int(((group["grid_mode"] == "corridor") & valid & group["corridor_padding_used_m"].fillna(0).gt(1.0)).sum()),
            "full_grid_fallback_count": int(group["fallback_used"].astype(bool).sum()),
        })
    pd.DataFrame(acceptance).to_csv(output / "stage6_acceptance_summary.csv", index=False)
    pd.DataFrame([
        {"query_count": count, "topology_build_wall_time_ms": float(run_frame["topology_build_wall_time_ms"].iloc[0]) if not run_frame.empty else 0.0,
         "amortized_topology_cost_ms": (float(run_frame["topology_build_wall_time_ms"].iloc[0]) / count) if not run_frame.empty else 0.0}
        for count in (10, 100, 1000)
    ]).to_csv(output / "topology_amortization.csv", index=False)
    # Query-level view (one row per query/mode) keeps repetition counts out of
    # the acceptance denominator while retaining repetition-level CSVs above.
    query_level = []
    for (query_id, mode), group in run_frame.groupby(["query_id", "mode"]):
        valid = group["final_valid_success"].astype(bool)
        query_level.append({
            "query_id": query_id, "mode": mode, "repetitions": len(group),
            "success_count": int(valid.sum()), "query_success": bool(valid.all()),
            "mean_online_time_ms": float(pd.to_numeric(group["total_online_time_ms"], errors="coerce").mean()),
            "P50_online_time_ms": float(pd.to_numeric(group["total_online_time_ms"], errors="coerce").quantile(0.5)),
            "mean_expanded_nodes": float(pd.to_numeric(group["expanded_nodes"], errors="coerce").mean()),
            "mean_search_space_ratio": float(pd.to_numeric(group["search_space_ratio"], errors="coerce").mean()),
            "fallback_count": int(group["fallback_used"].astype(bool).sum()),
        })
    pd.DataFrame(query_level).to_csv(output / "query_level_summary.csv", index=False)
    # Direct comparison against full-grid on matching query/repetition keys.
    comparison = []
    baseline = run_frame[run_frame["mode"] == "full_grid"].set_index(["query_id", "repetition"])
    for mode in ["topology_guided_grid", "topology_guided_grid_fallback"]:
        current = run_frame[run_frame["mode"] == mode].set_index(["query_id", "repetition"])
        joined = current.join(baseline, lsuffix="", rsuffix="_full", how="inner")
        valid = joined["final_valid_success"] & joined["final_valid_success_full"]
        if valid.any():
            time_ratio = (joined.loc[valid, "total_online_time_ms"] / joined.loc[valid, "total_online_time_ms_full"])
            expansion_ratio = (joined.loc[valid, "expanded_nodes"] / joined.loc[valid, "expanded_nodes_full"])
            length_ratio = (joined.loc[valid, "final_path_length_m"] / joined.loc[valid, "final_path_length_m_full"])
            comparison.append({
                "mode": mode, "paired_valid_count": int(valid.sum()),
                "online_speedup_vs_full_grid": float(1.0 / time_ratio.mean()),
                "online_time_ratio_vs_full_grid": float(time_ratio.mean()),
                "expanded_node_reduction_ratio": float(1.0 - expansion_ratio.mean()),
                "expanded_node_reduction_P95": float(1.0 - expansion_ratio.quantile(0.95)),
                "path_length_ratio_mean": float(length_ratio.mean()),
                "path_length_ratio_P95": float(length_ratio.quantile(0.95)),
                "fallback_count": int(joined["fallback_used"].astype(bool).sum()),
                "non_fallback_count": int((~joined["fallback_used"].astype(bool)).sum()),
            })
    pd.DataFrame(comparison).to_csv(output / "stage6_comparison.csv", index=False)
    fallback_rows = run_frame[run_frame["fallback_used"].astype(bool)].copy()
    if not fallback_rows.empty:
        fallback_rows["extra_online_time_vs_full_grid_ms"] = fallback_rows["total_online_time_ms"].to_numpy() - baseline.loc[fallback_rows.set_index(["query_id", "repetition"]).index, "total_online_time_ms"].to_numpy()
    fallback_rows.to_csv(output / "stage6_fallback_cost.csv", index=False)
    split_rows = []
    for mode, group in run_frame.groupby("mode"):
        for bucket, subset in (("non_fallback", group[~group["fallback_used"].astype(bool)]), ("fallback", group[group["fallback_used"].astype(bool)])):
            values = pd.to_numeric(subset.loc[subset["final_valid_success"].astype(bool), "total_online_time_ms"], errors="coerce").dropna()
            nodes = pd.to_numeric(subset.loc[subset["final_valid_success"].astype(bool), "expanded_nodes"], errors="coerce").dropna()
            split_rows.append({
                "mode": mode, "bucket": bucket, "count": len(subset),
                "online_time_P50_ms": float(values.quantile(0.5)) if len(values) else None,
                "online_time_P95_ms": float(values.quantile(0.95)) if len(values) else None,
                "expanded_nodes_P50": float(nodes.quantile(0.5)) if len(nodes) else None,
                "expanded_nodes_P95": float(nodes.quantile(0.95)) if len(nodes) else None,
            })
    pd.DataFrame(split_rows).to_csv(output / "stage6_performance_split.csv", index=False)


def _write_plots(directory: Path, frame: pd.DataFrame) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    for field, name, title, ylabel in [
        ("total_online_time_ms", "query_time_by_mode.png", "Online query time", "ms"),
        ("final_path_length_m", "path_length_by_mode.png", "Final path length", "m"),
        ("expanded_nodes", "expanded_nodes_by_mode.png", "A* expanded nodes", "nodes"),
        ("search_space_ratio", "search_space_ratio_by_mode.png", "Allowed search-space ratio", "ratio"),
        ("cpu_total_ms", "cpu_time_by_mode.png", "Query CPU time", "ms"),
        ("query_rss_peak_bytes", "query_peak_rss_by_mode.png", "Query peak RSS", "bytes"),
        ("length_over_full_grid", "path_length_ratio_by_mode.png", "Path length / full-grid", "ratio"),
    ]:
        fig, axis = plt.subplots(figsize=(9, 5))
        groups, labels = [], []
        for mode, group in frame.groupby("mode"):
            values = pd.to_numeric(group[field], errors="coerce").dropna()
            if len(values):
                groups.append(values.to_numpy()); labels.append(mode)
        if groups:
            # Matplotlib renamed this keyword from ``labels`` to
            # ``tick_labels`` in newer releases. Keep the benchmark usable
            # with the Ubuntu 22.04 system Matplotlib as well as newer envs.
            try:
                axis.boxplot(groups, tick_labels=labels)
            except TypeError:
                axis.boxplot(groups, labels=labels)
            axis.tick_params(axis="x", rotation=25)
        axis.set_title(title); axis.set_ylabel(ylabel); axis.grid(True, alpha=0.25)
        fig.tight_layout(); fig.savefig(directory / name, dpi=140); plt.close(fig)
    fig, axis = plt.subplots(figsize=(9, 5))
    rates = frame.groupby("mode")["final_success"].mean()
    rates.plot(kind="bar", ax=axis); axis.set_ylim(0, 1); axis.set_ylabel("success rate")
    axis.set_title("Static topology benchmark success rate")
    fig.tight_layout(); fig.savefig(directory / "success_rate_by_mode.png", dpi=140); plt.close(fig)


def run_topology_benchmark(
    *,
    map_name: str,
    protocol_path: str | Path | None = None,
    queries_path: str | Path,
    output_dir: str | Path,
    topology_dir: str | Path | None,
    modes: Sequence[str],
    query_ids: Optional[Sequence[str]],
    repetitions: int,
    warmups: int = 0,
    build_only: bool,
    corridor_padding_m: float,
    attach_radius_m: float,
) -> Path:
    protocol_file, protocol = load_protocol(protocol_path)
    if bool(protocol.get("dynamic_obstacles", False)):
        raise ValueError("dynamic_obstacles must be false for the static topology benchmark")
    map_yaml = _resolve_hospital_map(protocol_file, protocol, map_name)
    hospital_map = HospitalMap.load(map_yaml)
    queries_file, queries = load_queries(queries_path)
    if repetitions <= 0 or warmups < 0 or warmups >= repetitions:
        raise ValueError("repetitions must be positive and warmups must be in [0, repetitions)")
    if query_ids:
        selected = set(query_ids)
        queries = [query for query in queries if query.query_id in selected]
        if not queries:
            raise ValueError("none of the requested query IDs exist")
    footprint = protocol.get("footprint", DEFAULT_FOOTPRINT)
    padding_m = float(protocol.get("footprint_padding_m", 0.05))
    safety_margin_m = float(protocol.get("additional_safety_margin_m", 0.05))
    allow_unknown = bool(protocol.get("allow_unknown", False))
    output = Path(output_dir).resolve()
    if (output / "query_runs.csv").exists() or (output / "topology").exists() and build_only:
        raise ValueError(f"refusing to overwrite existing topology benchmark output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    topology_path = Path(topology_dir).resolve() if topology_dir else output / "topology"
    build_started = _now_ns(); cpu_started = _cpu_seconds()
    if topology_dir:
        artifact = load_topology(topology_path, hospital_map, footprint, padding_m=padding_m, safety_margin_m=safety_margin_m, allow_unknown=allow_unknown)
        build_wall_ms = 0.0; build_cpu_ms = 0.0; build_rss = 0
        precompute_source = topology_path.parent / "precompute_metrics.csv"
        if precompute_source.exists():
            source_row = pd.read_csv(precompute_source).iloc[0]
            build_wall_ms = float(source_row.get("topology_build_wall_time_ms", 0.0))
            build_cpu_ms = float(source_row.get("topology_build_cpu_time_ms", 0.0))
            build_rss = int(float(source_row.get("topology_build_peak_rss_bytes", 0.0)))
    else:
        artifact = build_topology(hospital_map, footprint, padding_m=padding_m, safety_margin_m=safety_margin_m, allow_unknown=allow_unknown)
        build_wall_ms = (_now_ns() - build_started) / 1e6
        build_cpu_ms = (_cpu_seconds() - cpu_started) * 1000.0
        build_rss = _peak_rss_bytes()
        save_topology(artifact, topology_path)
    precompute = {
        "map_id": str(protocol.get("map", hospital_map.map_id)), "map_sha256": artifact.metadata["map_sha256"],
        "topology_build_wall_time_ms": build_wall_ms, "topology_build_cpu_time_ms": build_cpu_ms,
        "topology_build_peak_rss_bytes": build_rss, "topology_graph_nodes": len(artifact.graph.nodes),
        "topology_graph_edges": len(artifact.graph.edges), "topology_graph_components": artifact.graph.components,
        "topology_file_size_bytes": sum(path.stat().st_size for path in topology_path.glob("*")) if topology_path.exists() else 0,
        "topology_min_clearance_m": min((edge.min_clearance_m for edge in artifact.graph.edges), default=0.0),
        "topology_mean_clearance_m": float(np.mean(artifact.distance_m[artifact.skeleton])) if artifact.skeleton.any() else 0.0,
        "footprint_padding_m": padding_m, "additional_safety_margin_m": safety_margin_m,
        "algorithm": TOPOLOGY_ALGORITHM_VERSION, "dynamic_obstacles": False,
    }
    _write_frame(output / "precompute_metrics.csv", [precompute])
    (output / "topology").mkdir(exist_ok=True)
    validation = []
    for query in queries:
        checked = hospital_map.validate_query(query, footprint, 0.0, allow_unknown=False)
        row = checked.as_dict(); row["config_variant"] = "normalized_static_topology"; validation.append(row)
    _write_frame(output / "query_validation.csv", validation)
    _write_frame(output / "queries.csv", [query.as_dict() for query in queries])
    manifest = {
        "schema_version": 1, "experiment": "hospital_static_topology_benchmark", "map": str(protocol.get("map", hospital_map.map_id)),
        "map_yaml": str(map_yaml), "map_sha256": artifact.metadata["map_sha256"],
        "queries_file": str(queries_file), "modes": list(modes), "repetitions": repetitions,
        "warmup_runs": warmups, "measured_runs": repetitions - warmups,
        "dynamic_obstacles": False, "dynamic_input_policy": "static_map_only",
        "topology_directory": str(topology_path), "topology_metadata": artifact.metadata,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    if build_only:
        return output
    runs: List[Dict[str, object]] = []
    metrics: List[Dict[str, object]] = []
    for query in queries:
        for repetition in range(1, repetitions + 1):
            for mode in modes:
                monitor = _QueryResourceMonitor(interval_ms=10.0)
                query_started = _now_ns()
                monitor.start()
                run, metric, path = _topology_attempt(
                    artifact, query, footprint, mode, corridor_padding_m, attach_radius_m,
                )
                online_ms = (_now_ns() - query_started) / 1e6
                resource_row = monitor.finish(online_ms)
                run.update(resource_row)
                run["total_online_time_ms"] = online_ms
                run["wall_time_ms"] = online_ms
                metric.update({
                    "total_online_time_ms": online_ms,
                    "wall_time_ms": online_ms,
                    "cpu_total_ms": resource_row.get("cpu_total_ms"),
                    "cpu_percent": resource_row.get("cpu_percent"),
                    "query_rss_peak_bytes": resource_row.get("query_rss_peak_bytes"),
                    "query_pss_peak_bytes": resource_row.get("query_pss_peak_bytes"),
                })
                run_id = f"{query.query_id}_{mode}_measured_{repetition}_{time.time_ns()}"
                run["run_id"] = run_id; metric["run_id"] = run_id
                run["repetition"] = repetition; metric["repetition"] = repetition
                run_mode = "warmup" if repetition <= warmups else "measured"
                run["run_mode"] = run_mode; metric["run_mode"] = run_mode
                if path:
                    path_file = Path("paths") / f"{run_id}.json.gz"
                    points = cells_to_poses(artifact, path, query.start[2], query.goal[2])
                    save_path(output / path_file, points)
                    run["path_file"] = str(path_file); run["path_point_count"] = len(points)
                runs.append(run); metrics.append(metric)
    _update_false_failures(runs)
    _fill_cross_mode_path_ratios(metrics)
    metric_by_run = {str(metric["run_id"]): metric for metric in metrics}
    for run in runs:
        metric = metric_by_run[str(run["run_id"])]
        for field in (
            "full_grid_path_length_m", "length_over_full_grid", "topology_path_length_m",
            "topology_route_width_min_m",
        ):
            run[field] = metric.get(field)
    amortized_ms = build_wall_ms / max(1, len(queries))
    for run in runs:
        run["topology_build_wall_time_ms"] = build_wall_ms
        run["amortized_topology_cost_ms"] = amortized_ms
        run["total_with_amortized_topology_cost_ms"] = float(run["total_topology_query_time_ms"]) + amortized_ms
    for metric in metrics:
        metric["topology_build_wall_time_ms"] = build_wall_ms
        metric["amortized_topology_cost_ms"] = amortized_ms
    _write_summaries(output, runs, metrics)
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build and benchmark a static Hospital topology planner")
    parser.add_argument("--map", default="hospital", choices=["hospital", "hospital_005"])
    parser.add_argument("--protocol", default=str(CONFIG_ROOT / "planner_benchmark_protocol.yaml"), help="static-map topology protocol YAML")
    parser.add_argument("--queries", default=str(CONFIG_ROOT / "planner_benchmark_queries_hospital.yaml"))
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--topology", default=None)
    parser.add_argument("--query-id", action="append", dest="query_ids", help="Query ID or comma-separated IDs")
    parser.add_argument("--mode", choices=["all", *MODES, *REPORT_ONLY_MODES], default="all")
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--warmups", type=int, default=0, help="Initial repetitions per query/mode marked warmup")
    parser.add_argument("--corridor-padding", type=float, default=1.0)
    parser.add_argument("--attach-radius", type=float, default=5.0)
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", help="Required static-only policy marker")
    return parser


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        _assert_static_cli(argv)
        args = build_parser().parse_args(argv)
        if not args.no_dynamic_obstacles:
            raise ValueError("--no-dynamic-obstacles is required for the static topology benchmark")
        query_ids = []
        for value in args.query_ids or []:
            query_ids.extend(part for part in value.split(",") if part)
        # ``all`` is the reproducible topology ablation: include the graph-only
        # measurement as well as both guided-grid variants and the full-grid
        # reference. Keep the explicit mode flag for single-mode smoke runs.
        modes = MODES if args.mode == "all" else [args.mode]
        output = run_topology_benchmark(
            map_name=args.map, protocol_path=args.protocol, queries_path=args.queries, output_dir=args.output_dir,
            topology_dir=args.topology, modes=modes, query_ids=query_ids or None,
            repetitions=args.repetitions, warmups=args.warmups, build_only=args.build_only,
            corridor_padding_m=args.corridor_padding, attach_radius_m=args.attach_radius,
        )
        print(f"topology benchmark output: {output}")
        return 0
    except (ValueError, OSError, KeyError) as exc:
        print(f"topology_benchmark: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
