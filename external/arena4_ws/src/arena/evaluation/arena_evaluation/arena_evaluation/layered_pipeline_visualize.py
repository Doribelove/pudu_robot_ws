"""Run one frozen A2B layered planning request and render its artifacts.

This command is intentionally offline-first: L1 and L2 use the project's
static topology implementation directly.  L3 is an optional Nav2 Smac
Hybrid DUBIN adapter; it is only started when a repair window is detected.
When ROS/Smac is unavailable the run is retained with an explicit failure
code instead of reusing the geometric path under a kinematic label.
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
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml

from . import fixed_layered_pipeline_smoke as fixed
from . import unified_four_backends_smoke as layered_runtime
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .topology import (
    AStarResult,
    TopologyArtifact,
    TopologyRoute,
    astar_grid,
    attach_pose,
    build_topology,
    cells_to_poses,
    corridor_mask,
    footprint_hash,
    save_topology,
    search_topology,
)


ROOT = Path("/home/robot/pudu_robot_ws")
WORLD_ROOT = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds"
BENCHMARK_ROOT = ROOT / "benchmarks/arena_a2b_20"
BENCHMARK_JSON = BENCHMARK_ROOT / "arena_a2b_benchmark_20.json"
BENCHMARK_CSV = BENCHMARK_ROOT / "arena_a2b_benchmark_20.csv"
DEFAULT_OUTPUT_ROOT = ROOT / "experiments/layered_planner_visualization"
FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]
PROTOCOL_VERSION = "layered_pipeline_visualization.v2_v3_latency"
TOPOLOGY_PADDING_M = 0.05
TOPOLOGY_SAFETY_MARGIN_M = 0.05
CORRIDOR_PADDING_SCHEDULE_M = (1.0, 2.0, 4.0, 8.0)
MAX_CURVATURE = 2.5
MIN_TURNING_RADIUS_M = 0.40
L3_TIMEOUT_S = 5.0
L3_RETRY_RADII_M = (2.0, 4.0, 6.0)
L3_WINDOW_MERGE_RADIUS_M = 8.0


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _hash_json(value: Any) -> str:
    return _sha256_bytes(json.dumps(value, sort_keys=True, separators=(",", ":")).encode())


def _source_hash() -> str:
    files = [
        Path(__file__).resolve(),
        Path(fixed.__file__).resolve(),
        Path(layered_runtime.__file__).resolve(),
        Path(__file__).resolve().parent / "topology.py",
        layered_runtime._strict_smac_config_path(),
    ]
    payload = "\n".join(
        f"{path}\0{sha256_file(path)}" for path in sorted(files) if path.is_file()
    )
    return _sha256_bytes(payload.encode("utf-8"))


@dataclass(frozen=True)
class BenchmarkTask:
    map_id: str
    query_id: str
    label: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    preference: str
    preference_side: str
    feature_tags: tuple[str, ...]
    expected_reachability: str = "expected_reachable"

    @property
    def query_hash(self) -> str:
        return _hash_json({
            "map_id": self.map_id,
            "query_id": self.query_id,
            "label": self.label,
            "start": list(self.start),
            "goal": list(self.goal),
            "preference": self.preference,
            "preference_side": self.preference_side,
            "feature_tags": list(self.feature_tags),
        })


def load_benchmark_tasks(
    json_path: Path = BENCHMARK_JSON, csv_path: Path = BENCHMARK_CSV,
) -> tuple[dict[str, Mapping[str, Any]], list[BenchmarkTask]]:
    """Read the frozen JSON and CSV and verify their map/query identity."""
    manifest = json.loads(json_path.read_text(encoding="utf-8"))
    tasks: list[BenchmarkTask] = []
    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    json_rows: dict[tuple[str, str], Mapping[str, Any]] = {}
    for map_id, map_info in manifest.get("maps", {}).items():
        for item in map_info.get("tasks", []):
            json_rows[(map_id, str(item["id"]))] = item
            tasks.append(BenchmarkTask(
                map_id=map_id,
                query_id=str(item["id"]),
                label=str(item.get("label", item["id"])),
                start=tuple(float(v) for v in item["start"]),
                goal=tuple(float(v) for v in item["goal"]),
                preference=str(item.get("preference", "none")),
                preference_side=str(item.get("preference_side", "none")),
                feature_tags=tuple(str(v) for v in item.get("feature_tags", [])),
            ))
    csv_keys = {(str(row.get("world", "")), str(row.get("task_id", ""))) for row in rows}
    if set(json_rows) != csv_keys:
        raise ValueError("benchmark JSON and CSV contain different map/query records")
    for row in rows:
        key = (str(row["world"]), str(row["task_id"]))
        task = json_rows[key]
        expected = [float(row[name]) for name in ("start_x_m", "start_y_m", "start_yaw_rad")]
        expected_goal = [float(row[name]) for name in ("goal_x_m", "goal_y_m", "goal_yaw_rad")]
        if not np.allclose(expected, task["start"], atol=1e-9) or not np.allclose(expected_goal, task["goal"], atol=1e-9):
            raise ValueError(f"benchmark JSON/CSV pose mismatch for {key}")
    return manifest.get("maps", {}), tasks


def map_yaml_for(map_id: str, world_root: Path = WORLD_ROOT) -> Path:
    path = world_root / map_id / "map" / "map.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"map_id {map_id!r} has no map YAML: {path}")
    return path


def discover_maps(
    manifest_maps: Mapping[str, Any], world_root: Path = WORLD_ROOT,
) -> list[dict[str, Any]]:
    # Read dimensions from image headers only.  Loading every xlarge raster
    # just to print a selector would allocate hundreds of megabytes.
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None
    result = []
    for map_id, info in manifest_maps.items():
        yaml_path = map_yaml_for(map_id, world_root)
        config = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        image_path = Path(str(config["image"]))
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        with Image.open(image_path) as image:
            width, height = image.size
        resolution = float(config["resolution"])
        origin = [float(value) for value in config.get("origin", [0.0, 0.0, 0.0])]
        result.append({
            "map_id": map_id,
            "band": info.get("band", ""),
            "width_m": width * resolution,
            "height_m": height * resolution,
            "area_m2": width * height * resolution ** 2,
            "resolution_m": resolution,
            "origin": origin,
            "width": width,
            "height": height,
            "map_yaml": str(yaml_path),
        })
    return result


def _task_rows(tasks: Iterable[BenchmarkTask], map_id: str | None = None) -> list[dict[str, Any]]:
    return [{
        "map_id": t.map_id, "query_id": t.query_id, "label": t.label,
        "start": list(t.start), "goal": list(t.goal),
        "preference": t.preference, "preference_side": t.preference_side,
        "scene": ";".join(t.feature_tags), "expected_reachability": t.expected_reachability,
    } for t in tasks if map_id is None or t.map_id == map_id]


def format_maps(rows: Sequence[Mapping[str, Any]]) -> str:
    lines = ["地图列表:", "编号  map_id                                      尺寸(m)       面积(m²)    分辨率"]
    for index, row in enumerate(rows, 1):
        lines.append(f"{index:>2}    {row['map_id']:<42} {row['width_m']:.2f}x{row['height_m']:.2f}  {row['area_m2']:.2f}  {row['resolution_m']:.3f}")
    return "\n".join(lines)


def format_queries(rows: Sequence[Mapping[str, Any]]) -> str:
    lines = ["起终点任务列表:", "编号  map_id / query_id                 起点(x,y,yaw)                 终点(x,y,yaw)                 偏好      可达性              场景"]
    for index, row in enumerate(rows, 1):
        lines.append(
            f"{index:>2}    {row['map_id']} / {row['query_id']:<7} "
            f"({row['start'][0]:.3f},{row['start'][1]:.3f},{row['start'][2]:.4f})  "
            f"({row['goal'][0]:.3f},{row['goal'][1]:.3f},{row['goal'][2]:.4f})  "
            f"{row['preference']}/{row['preference_side']:<8} {row.get('expected_reachability', 'expected_reachable'):<18} {row['scene']}"
        )
    return "\n".join(lines)


def _select_index(prompt: str, count: int) -> int:
    while True:
        answer = input(prompt).strip()
        try:
            index = int(answer) - 1
        except ValueError:
            print("请输入列表中的编号。")
            continue
        if 0 <= index < count:
            return index
        print("编号超出范围，请重试。")


def _write_yaml(path: Path, payload: Mapping[str, Any]) -> None:
    path.write_text(yaml.safe_dump(dict(payload), sort_keys=False, allow_unicode=False), encoding="utf-8")


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key: json.dumps(value, sort_keys=True) if isinstance(value, (dict, list)) else value
                for key, value in row.items()
            })


def _world_path(
    artifact: TopologyArtifact,
    cells: Sequence[tuple[int, int]],
    start_yaw: float,
    goal_yaw: float,
) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = [dict(point) for point in cells_to_poses(
        artifact, cells, start_yaw, goal_yaw,
    )]
    for point in points:
        point.update({
            "source": "grid",
            "motion_direction": "forward",
            "steering": 0.0,
            "velocity": 0.0,
            "planner_backend": "arena_evaluation.topology.astar_grid",
            "backend_version": "skeleton_distance_transform_v1",
        })
    return layered_runtime._annotate_geometric_metadata(points)


def _path_hash(points: Sequence[Mapping[str, Any]]) -> str:
    return fixed._path_hash(points) if points else ""


def _path_length(points: Sequence[Mapping[str, Any]]) -> float:
    return sum(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(points, points[1:]))


def _curvature(points: Sequence[Mapping[str, Any]]) -> tuple[float, int]:
    maximum = 0.0
    violations = 0
    for before, current, after in zip(points, points[1:], points[2:]):
        a = math.hypot(float(current["x"]) - float(before["x"]), float(current["y"]) - float(before["y"]))
        b = math.hypot(float(after["x"]) - float(current["x"]), float(after["y"]) - float(current["y"]))
        if a <= 1e-9 or b <= 1e-9:
            continue
        yaw_a = math.atan2(float(current["y"]) - float(before["y"]), float(current["x"]) - float(before["x"]))
        yaw_b = math.atan2(float(after["y"]) - float(current["y"]), float(after["x"]) - float(current["x"]))
        delta = abs((yaw_b - yaw_a + math.pi) % (2 * math.pi) - math.pi)
        kappa = delta / max((a + b) * 0.5, 1e-9)
        maximum = max(maximum, kappa)
        violations += int(kappa > MAX_CURVATURE)
    return maximum, violations


def _run_l1_l2(artifact: TopologyArtifact, task: BenchmarkTask) -> tuple[dict[str, Any], TopologyRoute | None, AStarResult]:
    started = time.perf_counter()
    l1_started = time.perf_counter()
    start_attachment = attach_pose(artifact, task.start, FOOTPRINT, max_radius_m=max(20.0, artifact.hospital_map.width * artifact.hospital_map.resolution), allow_unknown=False)
    goal_attachment = attach_pose(artifact, task.goal, FOOTPRINT, max_radius_m=max(20.0, artifact.hospital_map.height * artifact.hospital_map.resolution), allow_unknown=False)
    base: dict[str, Any] = {"l1_backend": "skeleton_distance_transform_v1 + graph_astar", "l2_backend": "arena_evaluation.topology.astar_grid", "start_attachment_node": None, "goal_attachment_node": None, "topology_edge_ids": [], "topology_node_ids": [], "corridor_padding_m": None, "fallback_used": False, "fallback_reason": "", "l1_success": False, "l2_success": False, "failure_code": "", "l1_time_ms": 0.0, "l2_time_ms": 0.0, "l1_l2_time_ms": 0.0}
    invalid = AStarResult(None, 0, 0, 0, 0, int(np.count_nonzero(artifact.free_mask)), 0.0, None, 0.0, "INVALID_ENDPOINT")
    if start_attachment is None or goal_attachment is None:
        base["failure_code"] = "TOPOLOGY_START_NOT_ATTACHABLE" if start_attachment is None else "TOPOLOGY_GOAL_NOT_ATTACHABLE"
        base["l1_time_ms"] = (time.perf_counter() - l1_started) * 1000
        base["l1_l2_time_ms"] = (time.perf_counter() - started) * 1000
        return base, None, invalid
    base.update({"start_attachment_node": start_attachment.node_id, "goal_attachment_node": goal_attachment.node_id})
    route = search_topology(artifact, start_attachment.node_id, goal_attachment.node_id)
    if route is None:
        base["failure_code"] = "TOPOLOGY_NO_ROUTE"
        base["l1_time_ms"] = (time.perf_counter() - l1_started) * 1000
        base["l1_l2_time_ms"] = (time.perf_counter() - started) * 1000
        return base, None, invalid
    base.update({"l1_success": True, "topology_edge_ids": route.edge_ids, "topology_node_ids": route.node_ids, "topology_route_length_m": route.length_m, "topology_min_width_m": route.min_width_m})
    base["l1_time_ms"] = (time.perf_counter() - l1_started) * 1000
    l2_started = time.perf_counter()
    start_cell = artifact.hospital_map.world_to_cell(*task.start[:2])
    goal_cell = artifact.hospital_map.world_to_cell(*task.goal[:2])
    if start_cell is None or goal_cell is None:
        base["failure_code"] = "INVALID_ENDPOINT"
        base["l2_time_ms"] = (time.perf_counter() - l2_started) * 1000
        base["l1_l2_time_ms"] = (time.perf_counter() - started) * 1000
        return base, route, invalid
    result = invalid
    attempts: list[float] = []
    for padding in CORRIDOR_PADDING_SCHEDULE_M:
        attempts.append(padding)
        mask = corridor_mask(artifact, route, start_cell, goal_cell, padding)
        result = astar_grid(artifact.free_mask, start_cell, goal_cell, mask, resolution=artifact.hospital_map.resolution, return_stats=True)
        if result.path is not None:
            base.update({"l2_success": True, "corridor_padding_m": padding, "corridor_attempts": len(attempts), "grid_mode": "corridor" if len(attempts) == 1 else "corridor_expanded"})
            break
    if result.path is None:
        base["fallback_used"] = True
        base["fallback_reason"] = "corridor_no_path"
        result = astar_grid(artifact.free_mask, start_cell, goal_cell, resolution=artifact.hospital_map.resolution, return_stats=True)
        base.update({"l2_success": result.path is not None, "grid_mode": "full_grid_fallback", "corridor_attempts": len(attempts), "failure_code": "" if result.path is not None else "FULL_GRID_FAILED"})
    base["l1_l2_time_ms"] = (time.perf_counter() - started) * 1000
    base["l2_time_ms"] = (time.perf_counter() - l2_started) * 1000
    if result.path is not None:
        base["failure_code"] = ""
    return base, route, result


def _window_geometry(rows: Sequence[dict[str, Any]], points: Sequence[Mapping[str, Any]]) -> None:
    """Attach world-coordinate bounds before path indices shift after a splice."""
    for row in rows:
        first = max(0, min(len(points) - 1, int(row.get("window_start_index", 0))))
        last = max(first, min(len(points) - 1, int(row.get("window_end_index", first))))
        window_points = points[first:last + 1]
        if not window_points:
            continue
        xs = [float(point["x"]) for point in window_points]
        ys = [float(point["y"]) for point in window_points]
        row.update({
            "start_index": first,
            "end_index": last,
            "window_start_x": float(window_points[0]["x"]),
            "window_start_y": float(window_points[0]["y"]),
            "window_end_x": float(window_points[-1]["x"]),
            "window_end_y": float(window_points[-1]["y"]),
            "window_min_x": min(xs),
            "window_max_x": max(xs),
            "window_min_y": min(ys),
            "window_max_y": max(ys),
        })


def _merge_post_pass_diagnostics(
    first: Mapping[str, Any], second: Mapping[str, Any], windows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    merged = {
        **second,
        "l3_attempted": bool(first.get("l3_attempted") or second.get("l3_attempted")),
        "l3_backend_call_count": int(first.get("l3_backend_call_count") or 0) + int(second.get("l3_backend_call_count") or 0),
        "repair_window_count": len({row.get("window_index") for row in windows}),
        "l3_planning_time_ms": float(first.get("l3_planning_time_ms") or 0.0) + float(second.get("l3_planning_time_ms") or 0.0),
        "l3_process_overhead_ms": float(first.get("l3_process_overhead_ms") or 0.0) + float(second.get("l3_process_overhead_ms") or 0.0),
        "l3_action_wall_ms": float(first.get("l3_action_wall_ms") or 0.0) + float(second.get("l3_action_wall_ms") or 0.0),
        "stitch_validation_time_ms": float(first.get("stitch_validation_time_ms") or 0.0) + float(second.get("stitch_validation_time_ms") or 0.0),
        "repair_windows": list(windows),
    }
    for key in (
        "planner_rss_peak_bytes", "planner_pss_peak_bytes",
        "stack_rss_peak_bytes", "stack_pss_peak_bytes",
    ):
        values = [float(value) for value in (first.get(key), second.get(key)) if value is not None]
        if values:
            merged[key] = max(values)
    return merged


def _l3_plan(
    artifact: TopologyArtifact,
    task: BenchmarkTask,
    l2_points: list[dict[str, Any]],
    output: Path,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    """Run the v3 query-level Smac session and bounded repair workflow once."""
    started_ns = time.monotonic_ns()
    source_commit = fixed._source_commit()
    fixed._enrich(l2_points, source_commit)
    query = Query(task.query_id, list(task.start), list(task.goal), task.label, 0, "VALID")
    context = layered_runtime.MapContext(
        task.map_id,
        artifact.hospital_map,
        artifact.free_mask,
        artifact.distance_m,
        sha256_file(artifact.hospital_map.image_path),
        sha256_file(artifact.hospital_map.yaml_path),
        artifact.hospital_map.yaml_path,
    )
    l2_result = layered_runtime.PlanResult(
        planner_success=bool(l2_points),
        points=l2_points or None,
        failure_code="" if l2_points else "L2_PATH_UNAVAILABLE",
        planner_backend="arena_evaluation.topology.astar_grid",
        backend_version="skeleton_distance_transform_v1",
        source="grid",
    )
    smac_spec = layered_runtime.backend_availability()["hybrid_astar"]
    pending = fixed._merged_window_ranges(
        l2_points,
        fixed._violation_groups(l2_points),
        radius_m=L3_WINDOW_MERGE_RADIUS_M,
    ) if l2_points else []
    session: Any = None
    session_timing: dict[str, float] = {}
    calls: list[dict[str, Any]] = []
    windows: list[dict[str, Any]] = []
    diagnostic_points: list[dict[str, Any]] = []
    result: layered_runtime.PlanResult
    if pending and not smac_spec.available:
        windows = [
            {"window_index": index, "attempt_index": -1, "radius_m": L3_WINDOW_MERGE_RADIUS_M, **dict(window)}
            for index, window in enumerate(pending)
        ]
        _window_geometry(windows, l2_points)
        result = layered_runtime.PlanResult(
            failure_code="L3_SMAC_UNAVAILABLE",
            failure_detail=smac_spec.reason,
            planner_backend=smac_spec.backend,
            backend_version=smac_spec.version,
            source="l3_local_smac",
            diagnostics={
                "backend_called": False,
                "l3_attempted": False,
                "l3_backend_call_count": 0,
                "repair_window_count": len(pending),
                "repair_windows": [],
            },
        )
        calls.append({
            "stage": "L3",
            "role": "l3_query_session_unavailable",
            "planner_backend": smac_spec.backend,
            "backend_version": smac_spec.version,
            "called": False,
            "planner_success": False,
            "failure_code": result.failure_code,
            "failure_detail": result.failure_detail,
        })
    else:
        try:
            if pending:
                os.environ.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
                session, _local_context, session_timing = fixed._build_query_smac_session(
                    context, query, l2_points, pending, smac_spec, output,
                )
            result, calls, windows = fixed.repair_all_windows(
                context,
                query,
                l2_result,
                smac_spec,
                output,
                source_commit,
                L3_TIMEOUT_S,
                smac_session=session,
            )
            if result.points:
                diagnostic_points = [dict(point) for point in result.points]
            _window_geometry(windows, l2_points)

            seen_signatures: set[tuple[tuple[int, int], ...]] = set()
            for _post_pass in range(2):
                if not result.points or result.failure_code != "L3_FINAL_VALIDATION_FAILED":
                    break
                seed_points = [dict(point) for point in result.points]
                signature = tuple(
                    (group[0], group[-1]) for group in fixed._violation_groups(seed_points)
                )
                if not signature or signature in seen_signatures:
                    break
                seen_signatures.add(signature)
                first_diagnostics = dict(result.diagnostics or {})
                seed = layered_runtime.PlanResult(
                    planner_success=True,
                    points=seed_points,
                    planner_backend=smac_spec.backend,
                    backend_version=smac_spec.version,
                    source="l3_post_validation_pass",
                )
                next_result, next_calls, next_windows = fixed.repair_all_windows(
                    context,
                    query,
                    seed,
                    smac_spec,
                    output,
                    source_commit,
                    L3_TIMEOUT_S,
                    smac_session=session,
                )
                window_offset = len({row.get("window_index") for row in windows})
                for row in next_calls:
                    row["window_index"] = int(row.get("window_index", 0)) + window_offset
                for row in next_windows:
                    row["window_index"] = int(row.get("window_index", 0)) + window_offset
                _window_geometry(next_windows, seed_points)
                calls.extend(next_calls)
                windows.extend(next_windows)
                result = next_result
                if next_result.points:
                    diagnostic_points = [dict(point) for point in next_result.points]
                result.diagnostics = _merge_post_pass_diagnostics(
                    first_diagnostics, dict(next_result.diagnostics or {}), windows,
                )
        except (OSError, RuntimeError, ValueError) as exc:
            if not windows:
                windows = [
                    {"window_index": index, "attempt_index": -1, "radius_m": L3_WINDOW_MERGE_RADIUS_M, **dict(window)}
                    for index, window in enumerate(pending)
                ]
                _window_geometry(windows, l2_points)
            result = layered_runtime.PlanResult(
                failure_code="L3_STACK_START_FAILED",
                failure_detail=str(exc),
                planner_backend=smac_spec.backend,
                backend_version=smac_spec.version,
                source="l3_local_smac",
                diagnostics={
                    "backend_called": False,
                    "l3_attempted": False,
                    "l3_backend_call_count": 0,
                    "repair_window_count": len(pending),
                    "repair_windows": windows,
                },
            )
            calls.append({
                "stage": "L3",
                "role": "l3_query_session_startup",
                "planner_backend": smac_spec.backend,
                "backend_version": smac_spec.version,
                "called": False,
                "planner_success": False,
                "failure_code": result.failure_code,
                "failure_detail": result.failure_detail,
            })
        finally:
            if session is not None:
                session.close()
                session_timing["l3_stack_shutdown_ms"] = float(session.stack_shutdown_time_ms)

    diagnostics = {**(result.diagnostics or {}), **session_timing}
    call_count = int(diagnostics.get(
        "l3_backend_call_count", sum(bool(row.get("called")) for row in calls),
    ))
    logical_window_count = len({row.get("window_index") for row in windows}) or len(pending)
    diagnostics.update({
        "l3_backend": smac_spec.backend,
        "l3_backend_version": smac_spec.version,
        "l3_triggered": bool(pending),
        "l3_called": call_count > 0,
        "l3_attempted": call_count > 0,
        "l3_backend_call_count": call_count,
        "repair_window_count": logical_window_count,
        "l3_success": bool(result.planner_success),
        "l3_failure_code": result.failure_code,
        "l3_failure_detail": result.failure_detail,
        "l3_time_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
        "query_level_smac_context_reuse": True,
        "query_level_local_map_build_once": True,
        "l3_diagnostic_points": diagnostic_points,
    })
    return list(result.points or []), windows, calls, diagnostics


def _plot_base(ax: Any, artifact: TopologyArtifact) -> None:
    import matplotlib.pyplot as plt  # noqa: F401
    image = np.where(artifact.hospital_map.occupancy == 100, 0.15, 0.92).astype(float)
    image = np.flipud(image)
    x0, y0, _ = artifact.hospital_map.origin
    extent = [x0, x0 + artifact.hospital_map.width * artifact.hospital_map.resolution, y0, y0 + artifact.hospital_map.height * artifact.hospital_map.resolution]
    ax.imshow(image, cmap="gray", vmin=0, vmax=1, extent=extent, origin="lower", interpolation="nearest", zorder=0)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")


def _plot_points(ax: Any, task: BenchmarkTask) -> None:
    ax.scatter([task.start[0]], [task.start[1]], c="#16a34a", s=42, label="start", zorder=8)
    ax.scatter([task.goal[0]], [task.goal[1]], c="#dc2626", s=42, label="goal", zorder=8)


def _plot_polyline(ax: Any, points: Sequence[Sequence[float]], **kwargs: Any) -> None:
    if points:
        arr = np.asarray(points, dtype=float)
        ax.plot(arr[:, 0], arr[:, 1], **kwargs)


def _is_smac_point(point: Mapping[str, Any]) -> bool:
    return str(point.get("source", "")) == "l3_hybrid_smac"


def _smac_yaw_points(points: Sequence[Mapping[str, Any]]) -> list[Mapping[str, Any]]:
    """Select only poses whose yaw came directly from Smac output."""
    return [point for point in points if _is_smac_point(point)]


def _logical_repair_windows(rows: Sequence[Mapping[str, Any]]) -> list[Mapping[str, Any]]:
    selected: dict[int, Mapping[str, Any]] = {}
    for row in rows:
        index = int(row.get("window_index", len(selected)))
        previous = selected.get(index)
        if previous is None or bool(row.get("selected_candidate")):
            selected[index] = row
        elif not bool(previous.get("selected_candidate")):
            if int(row.get("attempt_index", -1)) >= int(previous.get("attempt_index", -1)):
                selected[index] = row
    return [selected[index] for index in sorted(selected)]


def _plot_window_boxes(ax: Any, rows: Sequence[Mapping[str, Any]]) -> None:
    for window in _logical_repair_windows(rows):
        if all(key in window for key in ("window_min_x", "window_max_x", "window_min_y", "window_max_y")):
            xs = [float(window["window_min_x"]), float(window["window_max_x"])]
            ys = [float(window["window_min_y"]), float(window["window_max_y"])]
        else:
            continue
        padding = 0.15
        ax.fill(
            [xs[0] - padding, xs[1] + padding, xs[1] + padding, xs[0] - padding],
            [ys[0] - padding, ys[0] - padding, ys[1] + padding, ys[1] + padding],
            color="#facc15",
            alpha=0.22,
            zorder=2,
        )


def _plot_final_segments(ax: Any, points: Sequence[Mapping[str, Any]]) -> None:
    if not points:
        return
    segments: list[tuple[bool, list[Mapping[str, Any]]]] = []
    for point in points:
        is_smac = _is_smac_point(point)
        if not segments or segments[-1][0] != is_smac:
            segments.append((is_smac, [point]))
        else:
            segments[-1][1].append(point)
    used_labels: set[bool] = set()
    for is_smac, segment in segments:
        if len(segment) < 2:
            continue
        label = None
        if is_smac not in used_labels:
            label = "L3 Smac repair segment" if is_smac else "L2 prefix / suffix"
            used_labels.add(is_smac)
        _plot_polyline(
            ax,
            [(point["x"], point["y"]) for point in segment],
            color="#dc2626" if is_smac else "#16a34a",
            linewidth=2.0 if is_smac else 1.5,
            label=label,
            zorder=5,
        )


def _plot_pose(ax: Any, pose: Sequence[float], color: str, label: str) -> None:
    x, y, yaw = (float(value) for value in pose)
    ax.scatter([x], [y], c=color, s=42, label=label, zorder=8)
    ax.arrow(
        x, y, 0.8 * math.cos(yaw), 0.8 * math.sin(yaw),
        color=color, width=0.018, head_width=0.20,
        length_includes_head=True, zorder=8,
    )


def _plot_corridor(ax: Any, artifact: TopologyArtifact, task: BenchmarkTask, route: TopologyRoute | None, padding_m: float | None) -> None:
    if route is None or padding_m is None:
        return
    start_cell = artifact.hospital_map.world_to_cell(task.start[0], task.start[1])
    goal_cell = artifact.hospital_map.world_to_cell(task.goal[0], task.goal[1])
    if start_cell is None or goal_cell is None:
        return
    mask = corridor_mask(artifact, route, start_cell, goal_cell, float(padding_m)).astype(float)
    mask = np.flipud(mask)
    x0, y0, _ = artifact.hospital_map.origin
    extent = [x0, x0 + artifact.hospital_map.width * artifact.hospital_map.resolution, y0, y0 + artifact.hospital_map.height * artifact.hospital_map.resolution]
    ax.imshow(np.ma.masked_where(mask == 0, mask), cmap="Oranges", alpha=0.20, extent=extent, origin="lower", interpolation="nearest", zorder=1)


def render_visualizations(
    output: Path,
    artifact: TopologyArtifact,
    task: BenchmarkTask,
    route: TopologyRoute | None,
    l2_points: Sequence[Mapping[str, Any]],
    l3_points: Sequence[Mapping[str, Any]],
    repair_windows: Sequence[Mapping[str, Any]],
    *,
    l3_diagnostic_points: Sequence[Mapping[str, Any]] = (),
    show_node_ids: bool = False,
    metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    vis = output / "visualizations"
    vis.mkdir(parents=True, exist_ok=True)
    edges = artifact.graph.edges
    corridor_overlay = None
    if route is not None and (metadata or {}).get("corridor_padding_m") is not None:
        start_cell = artifact.hospital_map.world_to_cell(task.start[0], task.start[1])
        goal_cell = artifact.hospital_map.world_to_cell(task.goal[0], task.goal[1])
        if start_cell is not None and goal_cell is not None:
            corridor_overlay = corridor_mask(artifact, route, start_cell, goal_cell, float((metadata or {})["corridor_padding_m"]))

    def overlay(axis: Any) -> None:
        if corridor_overlay is None:
            return
        x0, y0, _ = artifact.hospital_map.origin
        extent = [x0, x0 + artifact.hospital_map.width * artifact.hospital_map.resolution, y0, y0 + artifact.hospital_map.height * artifact.hospital_map.resolution]
        image = np.flipud(corridor_overlay.astype(float))
        axis.imshow(np.ma.masked_where(image == 0, image), cmap="Oranges", alpha=0.20, extent=extent, origin="lower", interpolation="nearest", zorder=1)
    # L1 overview.
    fig, ax = plt.subplots(figsize=(10, 7), constrained_layout=True); _plot_base(ax, artifact)
    skeleton = np.flipud(artifact.skeleton.astype(float)); x0, y0, _ = artifact.hospital_map.origin
    extent = [x0, x0 + artifact.hospital_map.width * artifact.hospital_map.resolution, y0, y0 + artifact.hospital_map.height * artifact.hospital_map.resolution]
    ax.imshow(np.ma.masked_where(skeleton == 0, skeleton), cmap="Blues", alpha=0.65, extent=extent, origin="lower", interpolation="nearest", zorder=1)
    for edge in edges: _plot_polyline(ax, edge.polyline, color="#2563eb", linewidth=0.7, alpha=0.7)
    ax.scatter([node.x for node in artifact.graph.nodes], [node.y for node in artifact.graph.nodes], s=8, c="#2563eb", zorder=3)
    if show_node_ids:
        for node in artifact.graph.nodes: ax.text(node.x, node.y, str(node.node_id), fontsize=5, color="#1d4ed8")
    _plot_points(ax, task); ax.set_title("L1 topology overview"); ax.legend(loc="upper right")
    fig.savefig(vis / "l1_topology_overview.png", dpi=140); plt.close(fig)
    # L1 selected route.
    fig, ax = plt.subplots(figsize=(10, 7), constrained_layout=True); _plot_base(ax, artifact)
    overlay(ax)
    for edge in edges: _plot_polyline(ax, edge.polyline, color="#93c5fd", linewidth=0.6, alpha=0.5)
    if route: _plot_polyline(ax, route.polyline, color="#f97316", linewidth=2.4, label="selected topology route")
    width_text = f"; min channel width={route.min_width_m:.2f} m" if route is not None else ""
    _plot_points(ax, task); ax.set_title("L1 selected topology route and corridor centerline" + width_text); ax.legend(loc="upper right")
    fig.savefig(vis / "l1_selected_topology_route.png", dpi=140); plt.close(fig)
    # L2 path.
    fig, ax = plt.subplots(figsize=(10, 7), constrained_layout=True); _plot_base(ax, artifact)
    overlay(ax)
    if route: _plot_polyline(ax, route.polyline, color="#f97316", linewidth=5, alpha=0.18, label="L1 corridor centerline")
    _plot_polyline(ax, [(p["x"], p["y"]) for p in l2_points], color="#16a34a", linewidth=1.8, label="L2 grid path")
    l2_title = "L2 Grid A* route (L2 geometric tangent)"
    if metadata:
        l2_title += f"; mode={metadata.get('grid_mode', 'unknown')}; fallback={metadata.get('fallback_used', False)}; expanded={metadata.get('expanded_nodes', 0)}; search-space={float(metadata.get('search_space_ratio', 0.0)):.3f}"
    _plot_points(ax, task); ax.set_title(l2_title); ax.legend(loc="upper right")
    fig.savefig(vis / "l2_grid_route.png", dpi=140); plt.close(fig)
    # L3 path and windows.
    fig, ax = plt.subplots(figsize=(10, 7), constrained_layout=True); _plot_base(ax, artifact)
    _plot_polyline(ax, [(p["x"], p["y"]) for p in l2_points], color="#2563eb", linewidth=1.2, label="L2 grid path")
    l3_called = bool((metadata or {}).get("l3_called", False))
    display_points = l3_points or l3_diagnostic_points
    display_is_final = bool(l3_points)
    _plot_window_boxes(ax, repair_windows)
    _plot_final_segments(ax, display_points)
    smac_yaw_points = _smac_yaw_points(display_points)
    arrow_stride = max(1, len(smac_yaw_points) // 24)
    if l3_called:
        for point in smac_yaw_points[::arrow_stride]:
            ax.arrow(point["x"], point["y"], 0.6 * math.cos(point["yaw"]), 0.6 * math.sin(point["yaw"]), color="#7c3aed", width=0.015, head_width=0.18, length_includes_head=True, zorder=6)
    if display_points:
        clearances = [artifact.hospital_map.clearance(point["x"], point["y"]) or 0.0 for point in display_points]
        clearance_index = int(np.argmin(clearances))
        ax.scatter([display_points[clearance_index]["x"]], [display_points[clearance_index]["y"]], c="#facc15", edgecolors="#854d0e", s=38, marker="o", label="minimum clearance", zorder=9)
        if len(display_points) >= 3:
            curvature_values = []
            for before, current, after in zip(display_points, display_points[1:], display_points[2:]):
                first = math.hypot(current["x"] - before["x"], current["y"] - before["y"])
                second = math.hypot(after["x"] - current["x"], after["y"] - current["y"])
                if first <= 1e-9 or second <= 1e-9:
                    curvature_values.append(-1.0)
                    continue
                first_yaw = math.atan2(current["y"] - before["y"], current["x"] - before["x"])
                second_yaw = math.atan2(after["y"] - current["y"], after["x"] - current["x"])
                turn = abs((second_yaw - first_yaw + math.pi) % (2.0 * math.pi) - math.pi)
                curvature_values.append(turn / max((first + second) * 0.5, 1.0e-9))
            curvature_index = int(np.argmax(curvature_values)) + 1
            ax.scatter([display_points[curvature_index]["x"]], [display_points[curvature_index]["y"]], c="#f59e0b", edgecolors="#7c2d12", s=42, marker="^", label="maximum curvature", zorder=9)
    _plot_pose(ax, task.start, "#16a34a", "start pose")
    _plot_pose(ax, task.goal, "#7c3aed", "goal pose")
    if l3_called and display_is_final:
        l3_title = f"L3 kinematic route; {len(smac_yaw_points)} Smac yaw poses; {int((metadata or {}).get('l3_backend_call_count', 0))} call(s)"
    elif l3_called and display_points:
        l3_title = f"L3 diagnostic candidate rejected by final validation; {len(smac_yaw_points)} actual Smac yaw poses"
    elif (metadata or {}).get("l3_triggered"):
        l3_title = f"L3 triggered but unavailable/failed: {(metadata or {}).get('l3_failure_code', '')}"
    else:
        l3_title = "L3 not triggered; final path remains geometric Grid A*"
    ax.set_title(l3_title); ax.legend(loc="upper right")
    fig.savefig(vis / "l3_kinematic_route_heading.png", dpi=140); plt.close(fig)
    # Separate repair-window diagnostic.
    fig, ax = plt.subplots(figsize=(10, 7), constrained_layout=True); _plot_base(ax, artifact)
    _plot_polyline(ax, [(p["x"], p["y"]) for p in l2_points], color="#2563eb", linewidth=1.2, label="L2 grid path")
    _plot_window_boxes(ax, repair_windows)
    _plot_final_segments(ax, display_points)
    _plot_points(ax, task); ax.set_title("L3 repair windows"); ax.legend(loc="upper right")
    fig.savefig(vis / "l3_repair_windows.png", dpi=140); plt.close(fig)
    # Four-panel overview.
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    for axis in axes.flat: _plot_base(axis, artifact)
    for edge in edges: _plot_polyline(axes[0, 0], edge.polyline, color="#2563eb", linewidth=0.45, alpha=0.55)
    if route: _plot_polyline(axes[0, 1], route.polyline, color="#f97316", linewidth=2.2)
    _plot_polyline(axes[1, 0], [(p["x"], p["y"]) for p in l2_points], color="#16a34a", linewidth=1.6)
    _plot_window_boxes(axes[1, 1], repair_windows)
    _plot_final_segments(axes[1, 1], display_points)
    if l3_called:
        overview_stride = max(1, len(smac_yaw_points) // 16)
        for point in smac_yaw_points[::overview_stride]:
            axes[1, 1].arrow(
                point["x"], point["y"], 0.5 * math.cos(point["yaw"]), 0.5 * math.sin(point["yaw"]),
                color="#7c3aed", width=0.012, head_width=0.16, length_includes_head=True, zorder=6,
            )
    for axis in axes.flat: _plot_points(axis, task)
    axes[0, 0].set_title("L1 complete topology"); axes[0, 1].set_title("L1 selected route"); axes[1, 0].set_title("L2 Grid A*"); axes[1, 1].set_title("L3 DUBIN + yaw")
    fig.savefig(vis / "layered_pipeline_overview.png", dpi=140); plt.close(fig)
    from PIL import Image
    image_sizes = {}
    for path in sorted(vis.glob("*.png")):
        with Image.open(path) as image:
            image_sizes[path.name] = list(image.size)
    manifest = {
        "map_id": task.map_id,
        "query_id": task.query_id,
        "map_hash": (metadata or {}).get("map_hash", artifact.metadata.get("map_sha256")),
        "query_hash": task.query_hash,
        "footprint_hash": footprint_hash(FOOTPRINT),
        "topology_graph_hash": _sha256_bytes((output / "topology/topology_graph.json").read_bytes()),
        "path_hash": _path_hash(l3_points),
        "l3_diagnostic_candidate_hash": _path_hash(l3_diagnostic_points),
        "l3_diagnostic_candidate_used_for_rendering": bool(l3_diagnostic_points and not l3_points),
        "source_code_hash": _source_hash(),
        "map_resolution": artifact.hospital_map.resolution,
        "origin": list(artifact.hospital_map.origin),
        "l1_backend": "skeleton_distance_transform_v1 + graph_astar",
        "l2_backend": "grid_astar",
        "l2_grid_mode": (metadata or {}).get("grid_mode", ""),
        "l2_expanded_nodes": (metadata or {}).get("expanded_nodes", 0),
        "l2_search_space_ratio": (metadata or {}).get("search_space_ratio", 0.0),
        "l3_backend": (metadata or {}).get("l3_backend", "Nav2 SmacPlannerHybrid DUBIN"),
        "l3_backend_call_count": int((metadata or {}).get("l3_backend_call_count", 0)),
        "l3_actual_yaw_pose_count": len(smac_yaw_points),
        "query_level_smac_context_reuse": True,
        "query_level_local_map_build_once": True,
        "fallback_used": bool((metadata or {}).get("fallback_used", False)),
        "l3_repair_window_count": len(_logical_repair_windows(repair_windows)),
        "final_valid_success": bool((metadata or {}).get("final_valid_success", False)),
        "image_generation_command": (
            f"ros2 run arena_evaluation layered_pipeline_visualize --map-id {task.map_id} "
            f"--query-id {task.query_id} --no-dynamic-obstacles"
        ),
        "image_sizes": image_sizes,
    }
    for key in (
        "l3_local_map_build_ms", "l3_stack_startup_ms", "l3_planning_time_ms",
        "l3_process_overhead_ms", "l3_action_wall_ms", "l3_stack_shutdown_ms",
        "stitch_validation_time_ms", "unaccounted_time_ms",
        "pipeline_peak_rss_bytes", "l3_planner_rss_peak_bytes",
        "l3_planner_pss_peak_bytes", "l3_stack_rss_peak_bytes",
        "l3_stack_pss_peak_bytes",
    ):
        manifest[key] = float((metadata or {}).get(key) or 0.0)
    _write_yaml(output / "visualization_manifest.yaml", manifest)
    return manifest


def run_once(task: BenchmarkTask, output: Path, *, show_node_ids: bool = False) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=False)
    map_yaml = map_yaml_for(task.map_id)
    hospital_map = HospitalMap.load(map_yaml)
    if not math.isclose(hospital_map.resolution, 0.05, abs_tol=1e-9):
        raise ValueError(f"map resolution must be 0.05 m/cell, got {hospital_map.resolution}")
    expected_extent = (json.loads(BENCHMARK_JSON.read_text(encoding="utf-8")).get("maps", {}).get(task.map_id, {}).get("extent_m") or [])
    actual_extent = [hospital_map.width * hospital_map.resolution, hospital_map.height * hospital_map.resolution]
    if expected_extent and not np.allclose(actual_extent, expected_extent, atol=hospital_map.resolution):
        raise ValueError(f"map dimensions do not match frozen benchmark extent: {actual_extent} != {expected_extent}")
    map_hash = _sha256_bytes(map_yaml.read_bytes() + hospital_map.image_path.read_bytes())
    validation = hospital_map.validate_query(Query(task.query_id, list(task.start), list(task.goal), task.label, 0, "UNVALIDATED"), FOOTPRINT, 0.0, allow_unknown=False)
    if validation.validation_status != "VALID":
        raise ValueError(f"query validation failed: {validation.reason}")
    topology_dir = output / "topology"
    topology_started_ns = time.monotonic_ns()
    artifact = build_topology(hospital_map, FOOTPRINT, padding_m=TOPOLOGY_PADDING_M, safety_margin_m=TOPOLOGY_SAFETY_MARGIN_M, allow_unknown=False)
    topology_build_time_ms = (time.monotonic_ns() - topology_started_ns) / 1.0e6
    artifact.metadata["precompute_wall_time_ms"] = topology_build_time_ms
    save_topology(artifact, topology_dir)

    pipeline_started_ns = time.monotonic_ns()
    resources_before = resource.getrusage(resource.RUSAGE_SELF)
    l1_l2, route, grid_result = _run_l1_l2(artifact, task)
    l2_points = _world_path(artifact, grid_result.path or [], task.start[2], task.goal[2])
    l3_points, windows, backend_calls, l3 = _l3_plan(artifact, task, l2_points, output)
    l3_diagnostic_points = list(l3.pop("l3_diagnostic_points", []))
    query = Query(task.query_id, list(task.start), list(task.goal), task.label, 0, "VALID")
    context = layered_runtime.MapContext(
        task.map_id,
        hospital_map,
        artifact.free_mask,
        artifact.distance_m,
        sha256_file(hospital_map.image_path),
        sha256_file(map_yaml),
        map_yaml,
    )
    if l3_points:
        fixed._enrich(l3_points, fixed._source_commit())
    if l3_diagnostic_points:
        fixed._enrich(l3_diagnostic_points, fixed._source_commit())
    final_validation_started_ns = time.monotonic_ns()
    metrics = layered_runtime.validate_path(context, query, l3_points)
    final_output_validation_ms = (time.monotonic_ns() - final_validation_started_ns) / 1.0e6
    resources_after = resource.getrusage(resource.RUSAGE_SELF)
    # Linux reports ru_maxrss in KiB. This process is one isolated CLI run per
    # query, so the value is the peak RSS of the complete pipeline process.
    pipeline_peak_rss_bytes = int(resources_after.ru_maxrss * 1024)
    pipeline_wall_time_ms = (time.monotonic_ns() - pipeline_started_ns) / 1.0e6
    pipeline_cpu_total_ms = max(0.0, (
        resources_after.ru_utime - resources_before.ru_utime
        + resources_after.ru_stime - resources_before.ru_stime
    ) * 1000.0)

    stitch_validation_time_ms = float(l3.get("stitch_validation_time_ms") or 0.0) + final_output_validation_ms
    l3_planning_time_ms = float(l3.get("l3_planning_time_ms") or 0.0)
    l3_process_overhead_ms = float(l3.get("l3_process_overhead_ms") or 0.0)
    timing_sum_ms = (
        float(l1_l2.get("l1_time_ms") or 0.0)
        + float(l1_l2.get("l2_time_ms") or 0.0)
        + l3_planning_time_ms
        + l3_process_overhead_ms
        + stitch_validation_time_ms
    )
    unaccounted_time_ms = pipeline_wall_time_ms - timing_sum_ms
    l3.update({
        "stitch_validation_time_ms": stitch_validation_time_ms,
        "unaccounted_time_ms": unaccounted_time_ms,
        "map_hash": map_hash,
    })

    backend_calls = [
        {"stage": "L1", "role": "l1_graph_astar", "planner_backend": "skeleton_distance_transform_v1 + graph_astar", "called": True, "planner_success": l1_l2["l1_success"], "topology_node_ids": l1_l2.get("topology_node_ids", [])},
        {"stage": "L2", "role": "l2_grid_astar", "planner_backend": "arena_evaluation.topology.astar_grid", "called": True, "planner_success": l1_l2["l2_success"], "grid_mode": l1_l2.get("grid_mode", ""), "fallback_used": l1_l2.get("fallback_used", False)},
    ] + backend_calls
    final_points = l3_points or []
    static_valid = bool(final_points and metrics["static_footprint_valid"])
    kinematic_valid = bool(final_points and metrics["kinematic_valid"])
    final_valid = bool(static_valid and kinematic_valid and l1_l2["l2_success"] and l3.get("l3_success"))
    l3.update({"final_valid_success": final_valid, "fallback_used": l1_l2.get("fallback_used", False)})
    path_hash = _path_hash(final_points)
    protocol = {
        "schema_version": PROTOCOL_VERSION,
        "pipeline_version": "fixed_layered_pipeline_v3_latency",
        "layers": {
            "L1": "skeleton topology + graph A*",
            "L2": "topology corridor/full-grid Grid A*",
            "L3": "local Smac Hybrid DUBIN",
        },
        "enabled_backends": [
            "skeleton_distance_transform_v1",
            "arena_evaluation.topology.astar_grid",
            "Nav2 SmacPlannerHybrid DUBIN",
        ],
        "disabled_optional_backends": ["OMPL geometric::RRTstar", "OMPL control::SST"],
        "dynamic_obstacles": False,
        "resolution_m": 0.05,
        "footprint": FOOTPRINT,
        "footprint_hash": footprint_hash(FOOTPRINT),
        "allow_reverse": False,
        "allow_in_place_rotation": False,
        "minimum_turning_radius_m": MIN_TURNING_RADIUS_M,
        "maximum_curvature_per_m": MAX_CURVATURE,
        "motion_model": "forward_only_dubins",
        "window_radius_m": fixed.WINDOW_RADIUS_M,
        "window_margin_m": fixed.WINDOW_MARGIN_M,
        "window_retry_radii_m": list(L3_RETRY_RADII_M),
        "window_merge_radius_m": L3_WINDOW_MERGE_RADIUS_M,
        "query_level_smac_context_reuse": True,
        "query_level_local_map_build_once": True,
        "smac_path_smoothing": True,
        "post_validation_pass_limit": 2,
        "memory_metrics": {
            "pipeline_peak_rss_bytes": "main CLI process peak RSS; Linux ru_maxrss converted from KiB",
            "l3_planner_rss_peak_bytes": "Smac planner process peak RSS from monitor samples",
            "l3_planner_pss_peak_bytes": "Smac planner process peak PSS from monitor samples",
            "l3_stack_rss_peak_bytes": "Nav2 stack process-group peak RSS from monitor samples",
            "l3_stack_pss_peak_bytes": "Nav2 stack process-group peak PSS from monitor samples",
        },
        "formal_performance_conclusions": False,
    }
    _write_yaml(output / "protocol.yaml", protocol)
    source_files = [
        Path(__file__).resolve(),
        Path(fixed.__file__).resolve(),
        Path(layered_runtime.__file__).resolve(),
        Path(__file__).resolve().parent / "topology.py",
        layered_runtime._strict_smac_config_path(),
    ]
    _write_yaml(output / "source_manifest.yaml", {
        "benchmark_json": str(BENCHMARK_JSON),
        "benchmark_csv": str(BENCHMARK_CSV),
        "benchmark_json_sha256": sha256_file(BENCHMARK_JSON),
        "benchmark_csv_sha256": sha256_file(BENCHMARK_CSV),
        "map_yaml": str(map_yaml),
        "map_yaml_sha256": sha256_file(map_yaml),
        "map_image": str(hospital_map.image_path),
        "map_image_sha256": sha256_file(hospital_map.image_path),
        "map_hash": map_hash,
        "query_hash": task.query_hash,
        "source_commit": fixed._source_commit(),
        "source_code_hash": _source_hash(),
        "source_files": {str(path): sha256_file(path) for path in source_files if path.is_file()},
    })
    (output / "paths").mkdir(exist_ok=True)
    (output / "paths/l2_path.json").write_text(json.dumps({"map_id": task.map_id, "query_id": task.query_id, "path_hash": _path_hash(l2_points), "source": "grid", "points": l2_points}, indent=2), encoding="utf-8")
    path_source = "kinematic" if _smac_yaw_points(final_points) else "grid"
    (output / "paths/l3_path.json").write_text(json.dumps({"map_id": task.map_id, "query_id": task.query_id, "path_hash": path_hash, "source": path_source, "points": final_points}, indent=2), encoding="utf-8")
    diagnostic_path_hash = _path_hash(l3_diagnostic_points)
    if l3_diagnostic_points and not final_points:
        (output / "paths/l3_diagnostic_candidate.json").write_text(json.dumps({
            "map_id": task.map_id,
            "query_id": task.query_id,
            "path_hash": diagnostic_path_hash,
            "source": "diagnostic_candidate",
            "final_valid_success": False,
            "failure_code": l3.get("l3_failure_code", ""),
            "points": l3_diagnostic_points,
        }, indent=2), encoding="utf-8")
    repair_window_count = int(l3.get("repair_window_count") or len(_logical_repair_windows(windows)))
    run = {
        "run_id": output.name,
        "timestamp": dt.datetime.now(dt.timezone.utc).isoformat(),
        "map_id": task.map_id,
        "query_id": task.query_id,
        "query_hash": task.query_hash,
        "map_hash": map_hash,
        "start_x_m": task.start[0],
        "start_y_m": task.start[1],
        "start_yaw_rad": task.start[2],
        "goal_x_m": task.goal[0],
        "goal_y_m": task.goal[1],
        "goal_yaw_rad": task.goal[2],
        "preference": task.preference,
        "scene": ";".join(task.feature_tags),
        "dynamic_obstacles": False,
        "pipeline": "fixed_layered_v3_latency",
        "l1_success": l1_l2["l1_success"],
        "l2_success": l1_l2["l2_success"],
        "l3_triggered": bool(l3.get("l3_triggered")),
        "l3_called": bool(l3.get("l3_called")),
        "l3_attempted": bool(l3.get("l3_attempted")),
        "l3_backend_call_count": int(l3.get("l3_backend_call_count") or 0),
        "path_source": path_source,
        "fallback_used": l1_l2["fallback_used"],
        "fallback_reason": l1_l2["fallback_reason"],
        "repair_window_count": repair_window_count,
        "planner_success": bool(l3.get("l3_success")),
        "action_success": bool(final_points),
        "static_footprint_valid": static_valid,
        "kinematic_valid": kinematic_valid,
        "final_valid_success": final_valid,
        "failure_code": l3.get("l3_failure_code") or metrics["failure_code"] or l1_l2["failure_code"],
        "failure_detail": l3.get("l3_failure_detail") or metrics["failure_detail"],
        "topology_build_time_ms": topology_build_time_ms,
        "pipeline_wall_time_ms": pipeline_wall_time_ms,
        "pipeline_cpu_total_ms": pipeline_cpu_total_ms,
        "pipeline_peak_rss_bytes": pipeline_peak_rss_bytes,
        "l1_time_ms": l1_l2["l1_time_ms"],
        "l2_time_ms": l1_l2["l2_time_ms"],
        "l1_l2_time_ms": l1_l2["l1_l2_time_ms"],
        "l3_time_ms": l3["l3_time_ms"],
        "l3_local_map_build_ms": float(l3.get("l3_local_map_build_ms") or 0.0),
        "l3_stack_startup_ms": float(l3.get("l3_stack_startup_ms") or 0.0),
        "l3_planning_time_ms": l3_planning_time_ms,
        "l3_process_overhead_ms": l3_process_overhead_ms,
        "l3_action_wall_ms": float(l3.get("l3_action_wall_ms") or 0.0),
        "l3_stack_shutdown_ms": float(l3.get("l3_stack_shutdown_ms") or 0.0),
        "l3_planner_rss_peak_bytes": l3.get("planner_rss_peak_bytes"),
        "l3_planner_pss_peak_bytes": l3.get("planner_pss_peak_bytes"),
        "l3_stack_rss_peak_bytes": l3.get("stack_rss_peak_bytes"),
        "l3_stack_pss_peak_bytes": l3.get("stack_pss_peak_bytes"),
        "stitch_validation_time_ms": stitch_validation_time_ms,
        "unaccounted_time_ms": unaccounted_time_ms,
        "path_hash": path_hash,
        "l3_diagnostic_candidate_hash": diagnostic_path_hash,
        "l3_diagnostic_actual_yaw_pose_count": len(_smac_yaw_points(l3_diagnostic_points)),
        "path_length_m": metrics["path_length_m"],
        "minimum_clearance_m": metrics["minimum_clearance_m"],
        "maximum_curvature_per_m": metrics["maximum_curvature"],
        "curvature_p95": metrics["curvature_p95"],
        "heading_discontinuity_count": metrics["heading_discontinuity_count"],
        "steering_jump_count": metrics["steering_jump_count"],
        "reverse_distance_m": metrics["reverse_distance_m"],
        "in_place_rotation_count": metrics["in_place_rotation_count"],
        "position_discontinuity_count": metrics["position_discontinuity_count"],
        "expanded_nodes": grid_result.expanded_nodes,
        "generated_nodes": grid_result.generated_nodes,
        "search_space_ratio": grid_result.search_space_ratio,
        "grid_mode": l1_l2.get("grid_mode", ""),
    }
    common = {
        "run_id": run["run_id"],
        "map_id": task.map_id,
        "query_id": task.query_id,
        "query_hash": task.query_hash,
        "l3_triggered": run["l3_triggered"],
        "l3_attempted": run["l3_attempted"],
        "l3_backend_call_count": run["l3_backend_call_count"],
        "repair_window_count": repair_window_count,
    }
    for row in backend_calls:
        row.update(common)
    window_rows = []
    for row in windows:
        window_rows.append({
            **row,
            **common,
            "pipeline_wall_time_ms": pipeline_wall_time_ms,
        "pipeline_cpu_total_ms": pipeline_cpu_total_ms,
        "pipeline_peak_rss_bytes": pipeline_peak_rss_bytes,
            "l1_time_ms": l1_l2["l1_time_ms"],
            "l2_time_ms": l1_l2["l2_time_ms"],
            "l3_planning_time_ms": l3_planning_time_ms,
            "l3_process_overhead_ms": l3_process_overhead_ms,
            "l3_local_map_build_ms": run["l3_local_map_build_ms"],
            "l3_stack_startup_ms": run["l3_stack_startup_ms"],
            "l3_action_wall_ms": run["l3_action_wall_ms"],
        "l3_stack_shutdown_ms": run["l3_stack_shutdown_ms"],
        "l3_planner_rss_peak_bytes": run["l3_planner_rss_peak_bytes"],
        "l3_planner_pss_peak_bytes": run["l3_planner_pss_peak_bytes"],
        "l3_stack_rss_peak_bytes": run["l3_stack_rss_peak_bytes"],
        "l3_stack_pss_peak_bytes": run["l3_stack_pss_peak_bytes"],
            "stitch_validation_time_ms": stitch_validation_time_ms,
            "unaccounted_time_ms": unaccounted_time_ms,
        })
    _write_csv(output / "runs.csv", [run])
    _write_csv(output / "path_metrics.csv", [{
        "run_id": run["run_id"], "map_id": task.map_id, "query_id": task.query_id,
        "query_hash": task.query_hash, "path_hash": path_hash,
        **metrics, "final_valid_success": final_valid,
    }])
    _write_csv(output / "backend_call_log.csv", backend_calls)
    _write_csv(output / "repair_window_summary.csv", window_rows)
    viz_manifest = render_visualizations(
        output, artifact, task, route, l2_points, final_points, windows,
        l3_diagnostic_points=l3_diagnostic_points,
        show_node_ids=show_node_ids,
        metadata={
            **l3,
            "map_hash": map_hash,
            "pipeline_peak_rss_bytes": pipeline_peak_rss_bytes,
            "l3_planner_rss_peak_bytes": l3.get("planner_rss_peak_bytes"),
            "l3_planner_pss_peak_bytes": l3.get("planner_pss_peak_bytes"),
            "l3_stack_rss_peak_bytes": l3.get("stack_rss_peak_bytes"),
            "l3_stack_pss_peak_bytes": l3.get("stack_pss_peak_bytes"),
            "fallback_used": l1_l2["fallback_used"],
            "corridor_padding_m": l1_l2.get("corridor_padding_m"),
            "grid_mode": l1_l2.get("grid_mode", ""),
            "expanded_nodes": grid_result.expanded_nodes,
            "search_space_ratio": grid_result.search_space_ratio,
            "final_valid_success": final_valid,
        },
    )
    _write_yaml(output / "manifest.yaml", {"schema_version": PROTOCOL_VERSION, "map_id": task.map_id, "query_id": task.query_id, "map_hash": map_hash, "query_hash": task.query_hash, "footprint_hash": footprint_hash(FOOTPRINT), "map_yaml": str(map_yaml), "resolution": hospital_map.resolution, "origin": list(hospital_map.origin), "dynamic_obstacles": False, "final_valid_success": final_valid, "output_dir": str(output), "artifacts": viz_manifest})
    logical_windows = _logical_repair_windows(windows)
    window_text = ", ".join(
        f"w{int(row.get('window_index', 0))} indices={int(row.get('start_index', 0))}-{int(row.get('end_index', 0))} radius={float(row.get('radius_m', 0.0)):.1f}m status={'accepted' if row.get('selected_candidate') else row.get('failure_code', 'not-called')}"
        for row in logical_windows
    ) or "none"
    metric_number = lambda key: float(metrics[key]) if metrics.get(key) is not None else 0.0
    report = output / "final_report.md"
    report.write_text("\n".join([
        f"# 单次分层导航报告: {task.map_id} / {task.query_id}",
        "",
        f"- 起点: `{task.start}`",
        f"- 终点: `{task.goal}`",
        f"- 通行偏好: `{task.preference}/{task.preference_side}`; 场景: `{';'.join(task.feature_tags)}`",
        f"- L1 拓扑节点: `{l1_l2.get('topology_node_ids', [])}`; 通道边: `{l1_l2.get('topology_edge_ids', [])}`",
        f"- L2: `{l1_l2.get('grid_mode', 'failed')}`; 回退: `{l1_l2.get('fallback_used', False)}`",
        f"- L3: triggered=`{run['l3_triggered']}`, Smac calls=`{run['l3_backend_call_count']}`, repair windows=`{repair_window_count}`; backend=`{l3.get('l3_backend')}`",
        f"- L3 窗口: {window_text}",
        f"- L3 诊断候选: hash=`{diagnostic_path_hash}`; actual Smac yaw poses=`{run['l3_diagnostic_actual_yaw_pose_count']}`; 仅失败诊断使用，不计为最终路径",
        f"- 最终静态/运动学验收: **{final_valid}** (`{static_valid}/{kinematic_valid}`); 失败码: `{run['failure_code']}`",
        f"- 耗时(ms): topology build `{topology_build_time_ms:.2f}`, L1 `{run['l1_time_ms']:.2f}`, L2 `{run['l2_time_ms']:.2f}`, pipeline wall `{pipeline_wall_time_ms:.2f}`",
        f"- L3 耗时(ms): local map `{run['l3_local_map_build_ms']:.2f}`, stack startup `{run['l3_stack_startup_ms']:.2f}`, planner `{run['l3_planning_time_ms']:.2f}`, action `{run['l3_action_wall_ms']:.2f}`, process overhead `{run['l3_process_overhead_ms']:.2f}`, stack shutdown `{run['l3_stack_shutdown_ms']:.2f}`",
        f"- 内存峰值(MiB): pipeline RSS `{run['pipeline_peak_rss_bytes'] / 1048576.0:.2f}`, planner RSS `{float(run['l3_planner_rss_peak_bytes'] or 0.0) / 1048576.0:.2f}`, planner PSS `{float(run['l3_planner_pss_peak_bytes'] or 0.0) / 1048576.0:.2f}`, stack RSS `{float(run['l3_stack_rss_peak_bytes'] or 0.0) / 1048576.0:.2f}`, stack PSS `{float(run['l3_stack_pss_peak_bytes'] or 0.0) / 1048576.0:.2f}`",
        f"- 拼接/验收 `{stitch_validation_time_ms:.2f} ms`; 未归属端到端开销 `{unaccounted_time_ms:.2f} ms`",
        f"- 路径长度: `{metric_number('path_length_m'):.3f} m`; 最小净空: `{metric_number('minimum_clearance_m'):.3f} m`; 最大曲率: `{metric_number('maximum_curvature'):.3f} 1/m`",
        "",
        "本产物仅用于单次静态导航可视化，不作为正式性能评测结论。RRTstar/SST 未调用。",
        "",
        "## 可视化",
        "",
        *[f"![{name}](visualizations/{name})" for name in sorted(viz_manifest["image_sizes"])],
        "",
    ]), encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run one static layered L1/L2/L3 A2B navigation visualization")
    parser.add_argument("--map-id")
    parser.add_argument("--query-id")
    parser.add_argument("--output-dir")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", help="required: enforce static-map-only execution")
    parser.add_argument("--list-maps", action="store_true", help="list benchmark maps and exit")
    parser.add_argument("--list-queries", action="store_true", help="list frozen A2B queries and exit")
    parser.add_argument("--show-node-ids", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    manifest_maps, tasks = load_benchmark_tasks()
    maps = discover_maps(manifest_maps)
    if args.list_maps:
        print(format_maps(maps)); return 0
    if args.list_queries:
        print(format_queries(_task_rows(tasks, args.map_id))); return 0
    if not args.no_dynamic_obstacles:
        parser.error("--no-dynamic-obstacles must be explicitly specified")
    map_id = args.map_id
    if not map_id:
        print(format_maps(maps)); map_id = maps[_select_index("选择地图编号: ", len(maps))]["map_id"]
    if map_id not in manifest_maps:
        parser.error(f"unknown --map-id: {map_id}")
    selected = [task for task in tasks if task.map_id == map_id]
    query_id = args.query_id
    if not query_id:
        print(format_queries(_task_rows(selected, map_id))); query_id = selected[_select_index("选择 query 编号: ", len(selected))].query_id
    matches = [task for task in selected if task.query_id == query_id]
    if not matches:
        parser.error(f"unknown --query-id for {map_id}: {query_id}")
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = Path(args.output_dir).expanduser().resolve() if args.output_dir else DEFAULT_OUTPUT_ROOT / map_id / query_id / timestamp
    try:
        result = run_once(matches[0], output, show_node_ids=args.show_node_ids)
    except Exception as exc:
        print(f"layered_pipeline_visualize: ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"layered visualization output: {result}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
