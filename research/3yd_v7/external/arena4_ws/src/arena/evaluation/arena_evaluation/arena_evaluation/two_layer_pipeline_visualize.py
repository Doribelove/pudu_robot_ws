"""Replay and render one measured run of the frozen 2A-V0 architecture.

The 2A-V0 architecture is intentionally different from the three-layer
visualizer: L2 is disabled and the selected L1 corridor is passed directly to
the full-corridor Smac Hybrid DUBIN planner (L3').  This command never calls a
planner.  It loads the immutable topology cache, frozen query, measured path,
and recorded corridor metadata, then renders the result in map world
coordinates for audit.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from . import l1_l3_corridor_hybrid_smoke as candidate
from . import unified_four_backends_smoke as runtime
from .layered_pipeline_visualize import BenchmarkTask, load_benchmark_tasks
from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .topology import TopologyEdge, TopologyRoute, footprint_hash, load_topology


ROOT = Path("/home/robot/pudu_robot_ws")
WORLD_ROOT = ROOT / "external/arena4_ws/src/arena/simulation-setup/worlds"
DEFAULT_EXPERIMENT = ROOT / "experiments/layered_planner_benchmark/2a_v0_mentor_map_20260825_005_4x_area_20_r3_v1"
DEFAULT_OUTPUT_ROOT = ROOT / "experiments/layered_planner_visualization/2a_v0"
FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]
ARCHITECTURE_ID = "2A-V0"
IMPLEMENTATION_REVISION = "r3"
PROTOCOL_VERSION = "PLN-02-EXP-V1"
L2_STATUS = "disabled"
L3_BACKEND = "Nav2 SmacPlannerHybrid DUBIN"
CORRIDOR_PADDING_M = 2.0
TOPOLOGY_PADDING_M = 0.05
TOPOLOGY_SAFETY_MARGIN_M = 0.05
IMAGE_NAMES = (
    "l1_topology_overview.png",
    "l1_selected_route_corridor.png",
    "l3_prime_corridor_smac.png",
    "two_a_v0_overview.png",
)


def _parse_json(value: Any, fallback: Any) -> Any:
    if isinstance(value, (list, dict)):
        return value
    try:
        return json.loads(str(value))
    except (TypeError, ValueError, json.JSONDecodeError):
        return fallback


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _row_metric(row: Mapping[str, Any], *keys: str, default: float = 0.0) -> float:
    for key in keys:
        value = row.get(key)
        if value not in (None, "", "not_available"):
            return _as_float(value, default)
    return float(default)


def _formal_query_hash(task: BenchmarkTask, seed: int = 0) -> str:
    payload = json.dumps(
        {"query_id": task.query_id, "start": list(task.start), "goal": list(task.goal), "seed": int(seed)},
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return _sha256_bytes(payload)


def _source_hash() -> str:
    files = [
        Path(__file__).resolve(),
        Path(candidate.__file__).resolve(),
        Path(runtime.__file__).resolve(),
        Path(__file__).resolve().parent / "topology.py",
    ]
    payload = "\n".join(
        f"{path}\0{sha256_file(path)}" for path in sorted(files) if path.is_file()
    )
    return _sha256_bytes(payload.encode("utf-8"))


def _map_yaml(map_id: str) -> Path:
    path = WORLD_ROOT / map_id / "map" / "map.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"map_id {map_id!r} has no map YAML: {path}")
    return path


def _task_for(map_id: str, query_id: str) -> BenchmarkTask:
    _maps, tasks = load_benchmark_tasks()
    matches = [task for task in tasks if task.map_id == map_id and task.query_id == query_id]
    if not matches:
        raise ValueError(f"query {query_id!r} is not present for map {map_id!r}")
    return matches[0]


def _run_row(experiment: Path, query_id: str, run_mode: str, repetition: int) -> dict[str, str]:
    rows = _read_csv(experiment / "runs.csv")
    matches = [
        row for row in rows
        if row.get("query_id") == query_id
        and row.get("run_mode") == run_mode
        and int(row.get("repetition") or 0) == int(repetition)
    ]
    if not matches:
        raise ValueError(f"no {run_mode} repetition {repetition} found for {query_id}")
    row = matches[0]
    if row.get("architecture") != ARCHITECTURE_ID or row.get("architecture_id") != ARCHITECTURE_ID:
        raise ValueError(f"run is not {ARCHITECTURE_ID}: {row.get('architecture')!r}/{row.get('architecture_id')!r}")
    if row.get("implementation_revision") != IMPLEMENTATION_REVISION:
        raise ValueError(f"run implementation revision is not {IMPLEMENTATION_REVISION}: {row.get('implementation_revision')!r}")
    if str(row.get("l2_called", "")).lower() == "true" or int(row.get("l2_call_count") or 0) != 0:
        raise ValueError("2A-V0 audit failure: the selected run contains an L2 call")
    return row


def _route_from_row(artifact: Any, row: Mapping[str, Any]) -> TopologyRoute:
    node_ids = [int(value) for value in _parse_json(row.get("topology_node_ids"), [])]
    edge_ids = [int(value) for value in _parse_json(row.get("topology_edge_ids"), [])]
    if len(node_ids) != len(edge_ids) + 1 or not edge_ids:
        raise ValueError("selected 2A-V0 run has no reconstructable topology route")
    edges = {int(edge.edge_id): edge for edge in artifact.graph.edges}
    route_polyline: list[list[float]] = []
    for index, edge_id in enumerate(edge_ids):
        edge: TopologyEdge | None = edges.get(edge_id)
        if edge is None:
            raise ValueError(f"topology edge {edge_id} is missing from the cache")
        source = node_ids[index]
        target = node_ids[index + 1]
        if edge.source == source and edge.target == target:
            polyline = [list(point) for point in edge.polyline]
        elif edge.source == target and edge.target == source:
            polyline = [list(point) for point in reversed(edge.polyline)]
        else:
            raise ValueError(f"edge {edge_id} does not connect route nodes {source}->{target}")
        if not polyline:
            continue
        if not route_polyline:
            route_polyline.extend(polyline)
        else:
            route_polyline.extend(polyline[1:])
    if len(route_polyline) < 2:
        raise ValueError("selected topology route has fewer than two world points")
    length = sum(
        math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        for a, b in zip(route_polyline, route_polyline[1:])
    )
    min_width = min((float(edges[eid].min_width_m) for eid in edge_ids), default=0.0)
    return TopologyRoute(node_ids, edge_ids, length, min_width, route_polyline)


def _load_path(experiment: Path, row: Mapping[str, Any]) -> list[dict[str, Any]]:
    path_file = str(row.get("path_file") or "")
    if not path_file:
        return []
    path = (experiment / path_file).resolve()
    if experiment.resolve() not in path.parents:
        raise ValueError(f"path_file escapes experiment directory: {path_file}")
    points = _parse_json(path.read_text(encoding="utf-8"), [])
    if not isinstance(points, list):
        raise ValueError(f"path file is not a list: {path}")
    stored_hash = str(row.get("path_hash") or "")
    file_hash = str(points[0].get("path_hash") or "") if points else ""
    if stored_hash != file_hash:
        raise ValueError(f"path hash mismatch for {row.get('query_id')}: {stored_hash} != {file_hash}")
    return [dict(point) for point in points]


def _context(artifact: Any, map_yaml: Path) -> runtime.MapContext:
    return runtime.MapContext(
        artifact.hospital_map.map_id,
        artifact.hospital_map,
        artifact.free_mask,
        artifact.distance_m,
        sha256_file(artifact.hospital_map.image_path),
        sha256_file(map_yaml),
        map_yaml,
    )


def _world_extent(hospital_map: HospitalMap) -> list[float]:
    x0, y0, _ = hospital_map.origin
    return [
        float(x0),
        float(x0 + hospital_map.width * hospital_map.resolution),
        float(y0),
        float(y0 + hospital_map.height * hospital_map.resolution),
    ]


def _plot_base(axis: Any, artifact: Any) -> None:
    occupancy = np.asarray(artifact.hospital_map.occupancy)
    image = np.where(occupancy == 100, 0.16, np.where(occupancy < 0, 0.52, 0.94)).astype(float)
    # Downsample only the raster display.  All vector geometry remains in the
    # map's native world coordinates and uses the original full-map extent.
    stride = max(1, int(math.ceil(max(image.shape) / 2200.0)))
    image = np.flipud(image)[::stride, ::stride]
    axis.imshow(
        image,
        cmap="gray",
        vmin=0,
        vmax=1,
        extent=_world_extent(artifact.hospital_map),
        origin="lower",
        interpolation="nearest",
        zorder=0,
    )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")


def _plot_polyline(axis: Any, points: Sequence[Sequence[float]], **kwargs: Any) -> None:
    if points:
        values = np.asarray(points, dtype=float)
        axis.plot(values[:, 0], values[:, 1], **kwargs)


def _plot_points(axis: Any, task: BenchmarkTask, *, labels: bool = True) -> None:
    axis.scatter([task.start[0]], [task.start[1]], c="#16a34a", s=48, zorder=8, label="start")
    axis.scatter([task.goal[0]], [task.goal[1]], c="#dc2626", s=48, zorder=8, label="goal")
    if labels:
        axis.text(task.start[0], task.start[1], "  start", color="#166534", fontsize=8, va="bottom")
        axis.text(task.goal[0], task.goal[1], "  goal", color="#991b1b", fontsize=8, va="bottom")


def _plot_pose(axis: Any, pose: Sequence[float], color: str, label: str) -> None:
    x, y, yaw = (float(value) for value in pose)
    axis.scatter([x], [y], c=color, s=45, label=label, zorder=8)
    axis.arrow(
        x,
        y,
        1.2 * math.cos(yaw),
        1.2 * math.sin(yaw),
        color=color,
        width=0.035,
        head_width=0.32,
        length_includes_head=True,
        zorder=8,
    )


def _plot_skeleton_and_graph(axis: Any, artifact: Any, *, selected: TopologyRoute | None = None, show_node_ids: bool = False) -> None:
    skeleton = np.flipud(artifact.skeleton.astype(float))
    stride = max(1, int(math.ceil(max(skeleton.shape) / 2200.0)))
    skeleton = skeleton[::stride, ::stride]
    axis.imshow(
        np.ma.masked_where(skeleton == 0, skeleton),
        cmap="Blues",
        alpha=0.60,
        extent=_world_extent(artifact.hospital_map),
        origin="lower",
        interpolation="nearest",
        zorder=1,
    )
    for edge in artifact.graph.edges:
        _plot_polyline(axis, edge.polyline, color="#2563eb", linewidth=0.45, alpha=0.48, zorder=2)
    axis.scatter(
        [node.x for node in artifact.graph.nodes],
        [node.y for node in artifact.graph.nodes],
        s=4,
        c="#1d4ed8",
        alpha=0.70,
        zorder=3,
        label="L1 topology nodes",
    )
    if show_node_ids:
        for node in artifact.graph.nodes:
            axis.text(node.x, node.y, str(node.node_id), fontsize=4, color="#1d4ed8")
    if selected is not None:
        _plot_polyline(axis, selected.polyline, color="#f97316", linewidth=2.8, label="selected L1 route", zorder=5)


def _plot_corridor(axis: Any, artifact: Any, task: BenchmarkTask, route: TopologyRoute) -> np.ndarray:
    query = runtime.Query(task.query_id, list(task.start), list(task.goal), task.label, 0, "VALID")
    context = _context(artifact, artifact.hospital_map.yaml_path)
    mask = candidate._raw_corridor_mask(context, artifact, route, query, CORRIDOR_PADDING_M)
    display = np.flipud(mask.astype(float))
    axis.imshow(
        np.ma.masked_where(display == 0, display),
        cmap="Oranges",
        alpha=0.24,
        extent=_world_extent(artifact.hospital_map),
        origin="lower",
        interpolation="nearest",
        zorder=1,
    )
    return mask


def _sample_by_distance(points: Sequence[Mapping[str, Any]], spacing_m: float = 10.0) -> list[Mapping[str, Any]]:
    if not points:
        return []
    selected = [points[0]]
    next_distance = max(0.1, float(spacing_m))
    distance = 0.0
    for before, point in zip(points, points[1:]):
        distance += math.hypot(float(point["x"]) - float(before["x"]), float(point["y"]) - float(before["y"]))
        if distance + 1.0e-9 >= next_distance:
            selected.append(point)
            while next_distance <= distance:
                next_distance += max(0.1, float(spacing_m))
    if selected[-1] is not points[-1]:
        selected.append(points[-1])
    return selected


def _curvature_values(points: Sequence[Mapping[str, Any]]) -> list[float]:
    values: list[float] = []
    for before, current, after in zip(points, points[1:], points[2:]):
        first = math.hypot(float(current["x"]) - float(before["x"]), float(current["y"]) - float(before["y"]))
        second = math.hypot(float(after["x"]) - float(current["x"]), float(after["y"]) - float(current["y"]))
        if first <= 1.0e-9 or second <= 1.0e-9:
            values.append(0.0)
            continue
        first_yaw = math.atan2(float(current["y"]) - float(before["y"]), float(current["x"]) - float(before["x"]))
        second_yaw = math.atan2(float(after["y"]) - float(current["y"]), float(after["x"]) - float(current["x"]))
        turn = abs((second_yaw - first_yaw + math.pi) % (2.0 * math.pi) - math.pi)
        values.append(turn / max((first + second) * 0.5, 1.0e-9))
    return values


def _plot_l3(axis: Any, artifact: Any, task: BenchmarkTask, route: TopologyRoute, points: Sequence[Mapping[str, Any]], row: Mapping[str, Any], *, corridor: bool = True) -> None:
    if corridor:
        _plot_corridor(axis, artifact, task, route)
    _plot_polyline(axis, route.polyline, color="#f97316", linewidth=1.0, alpha=0.38, label="L1 corridor centerline", zorder=3)
    _plot_polyline(axis, [(float(p["x"]), float(p["y"])) for p in points], color="#16a34a", linewidth=2.0, label="L3' Smac full-corridor path", zorder=5)
    arrows = _sample_by_distance(points, spacing_m=10.0)
    for point in arrows:
        yaw = float(point.get("yaw", 0.0))
        axis.arrow(
            float(point["x"]),
            float(point["y"]),
            1.0 * math.cos(yaw),
            1.0 * math.sin(yaw),
            color="#7c3aed",
            width=0.025,
            head_width=0.24,
            length_includes_head=True,
            zorder=7,
        )
    if points:
        clearances = [artifact.hospital_map.clearance(float(p["x"]), float(p["y"])) or 0.0 for p in points]
        clearance_index = int(np.argmin(clearances))
        axis.scatter(
            [float(points[clearance_index]["x"])],
            [float(points[clearance_index]["y"])],
            c="#facc15",
            edgecolors="#854d0e",
            s=55,
            marker="o",
            label="minimum clearance",
            zorder=9,
        )
        curvatures = _curvature_values(points)
        if curvatures:
            curvature_index = int(np.argmax(curvatures)) + 1
            axis.scatter(
                [float(points[curvature_index]["x"])],
                [float(points[curvature_index]["y"])],
                c="#f59e0b",
                edgecolors="#7c2d12",
                s=58,
                marker="^",
                label="maximum curvature",
                zorder=9,
            )
    _plot_pose(axis, task.start, "#16a34a", "start pose")
    _plot_pose(axis, task.goal, "#7c3aed", "goal pose")
    valid = str(row.get("final_valid_success", "")).lower() == "true"
    axis.set_title(
        f"2A-V0 L3' Smac Hybrid DUBIN | valid={valid} | L2={L2_STATUS} | "
        f"{len(points)} yaw poses"
    )
    axis.legend(loc="upper right", fontsize=7)


def render_visualizations(
    output: Path,
    artifact: Any,
    task: BenchmarkTask,
    route: TopologyRoute,
    points: Sequence[Mapping[str, Any]],
    row: Mapping[str, Any],
    *,
    source_experiment_dir: Path,
    topology_graph_path: Path,
    show_node_ids: bool = False,
) -> dict[str, Any]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    visualization_dir = output / "visualizations"
    visualization_dir.mkdir(parents=True, exist_ok=True)
    attachment_start = route.node_ids[0]
    attachment_goal = route.node_ids[-1]
    node_by_id = {int(node.node_id): node for node in artifact.graph.nodes}

    fig, axis = plt.subplots(figsize=(12, 7), constrained_layout=True)
    _plot_base(axis, artifact)
    _plot_skeleton_and_graph(axis, artifact, show_node_ids=show_node_ids)
    _plot_points(axis, task)
    axis.set_title(f"2A-V0 L1 topology overview | {task.query_id} | L2 disabled")
    axis.legend(loc="upper right", fontsize=7)
    fig.savefig(visualization_dir / "l1_topology_overview.png", dpi=140)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(12, 7), constrained_layout=True)
    _plot_base(axis, artifact)
    _plot_corridor(axis, artifact, task, route)
    for edge in artifact.graph.edges:
        _plot_polyline(axis, edge.polyline, color="#93c5fd", linewidth=0.35, alpha=0.28, zorder=2)
    _plot_polyline(axis, route.polyline, color="#f97316", linewidth=2.8, label="selected L1 route", zorder=5)
    if attachment_start in node_by_id:
        axis.scatter([node_by_id[attachment_start].x], [node_by_id[attachment_start].y], marker="s", s=54, c="#16a34a", label=f"start attach node {attachment_start}", zorder=8)
    if attachment_goal in node_by_id:
        axis.scatter([node_by_id[attachment_goal].x], [node_by_id[attachment_goal].y], marker="s", s=54, c="#7c3aed", label=f"goal attach node {attachment_goal}", zorder=8)
    _plot_points(axis, task, labels=False)
    axis.set_title(
        f"2A-V0 selected L1 route + Smac corridor | padding={CORRIDOR_PADDING_M:g} m | "
        f"min width={route.min_width_m:.2f} m | L2 disabled"
    )
    axis.legend(loc="upper right", fontsize=7)
    fig.savefig(visualization_dir / "l1_selected_route_corridor.png", dpi=140)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(12, 7), constrained_layout=True)
    _plot_base(axis, artifact)
    _plot_l3(axis, artifact, task, route, points, row)
    fig.savefig(visualization_dir / "l3_prime_corridor_smac.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)
    for axis in axes.flat:
        _plot_base(axis, artifact)
    _plot_skeleton_and_graph(axes[0, 0], artifact)
    _plot_points(axes[0, 0], task, labels=False)
    axes[0, 0].set_title("L1 topology")
    _plot_corridor(axes[0, 1], artifact, task, route)
    _plot_polyline(axes[0, 1], route.polyline, color="#f97316", linewidth=2.3, label="selected L1 route")
    _plot_points(axes[0, 1], task, labels=False)
    axes[0, 1].set_title("L1 route -> L3' corridor")
    _plot_l3(axes[1, 0], artifact, task, route, points, row)
    axes[1, 0].set_title("L3' Smac path + actual yaw")
    _plot_base(axes[1, 1], artifact)
    _plot_corridor(axes[1, 1], artifact, task, route)
    _plot_polyline(axes[1, 1], [(float(p["x"]), float(p["y"])) for p in points], color="#16a34a", linewidth=1.8, label="L3' path")
    _plot_pose(axes[1, 1], task.start, "#16a34a", "start pose")
    _plot_pose(axes[1, 1], task.goal, "#7c3aed", "goal pose")
    valid = str(row.get("final_valid_success", "")).lower() == "true"
    l1_time_ms = _row_metric(row, "l1_time_ms", "l1_graph_search_ms")
    l2_time_ms = _row_metric(row, "l2_time_ms")
    l3_time_ms = _row_metric(row, "l3_time_ms", "hybrid_planning_time_ms")
    pipeline_wall_ms = _row_metric(row, "pipeline_wall_time_ms", "online_wall_ms")
    cpu_ms = _row_metric(row, "cpu_ms", "pipeline_cpu_total_ms")
    avg_cpu_percent = _row_metric(row, "avg_cpu_percent")
    peak_rss_mib = _row_metric(row, "peak_memory_mib", default=0.0)
    if peak_rss_mib <= 0.0:
        peak_rss_mib = _row_metric(row, "peak_rss") / (1024.0 * 1024.0)
    peak_pss_mib = _row_metric(row, "peak_pss") / (1024.0 * 1024.0)
    axes[1, 1].text(
        0.02,
        0.03,
        "\n".join([
            f"architecture: {ARCHITECTURE_ID} / {IMPLEMENTATION_REVISION}",
            "L1: skeleton topology + Graph A*",
            "L2: disabled (0 calls)",
            "L3': full corridor Smac Hybrid DUBIN",
            f"final_valid_success: {valid}",
            f"path length: {row.get('path_length_m', 'not_available')} m",
            f"min clearance: {row.get('minimum_clearance_m', 'not_available')} m",
            f"max curvature: {row.get('maximum_curvature', 'not_available')} 1/m",
            f"layer time L1/L2/L3': {l1_time_ms:.1f}/{l2_time_ms:.1f}/{l3_time_ms:.1f} ms",
            f"pipeline wall / CPU: {pipeline_wall_ms:.1f}/{cpu_ms:.1f} ms",
            f"CPU avg: {avg_cpu_percent:.1f}% | peak RSS/PSS: {peak_rss_mib:.1f}/{peak_pss_mib:.1f} MiB",
        ]),
        transform=axes[1, 1].transAxes,
        fontsize=9,
        va="bottom",
        bbox={"facecolor": "white", "alpha": 0.82, "edgecolor": "#cbd5e1"},
        zorder=10,
    )
    axes[1, 1].set_title("2A-V0 execution audit")
    fig.savefig(visualization_dir / "two_a_v0_overview.png", dpi=140)
    plt.close(fig)

    image_sizes: dict[str, list[int]] = {}
    for path in sorted(visualization_dir.glob("*.png")):
        with Image.open(path) as image:
            if image.size[0] <= 0 or image.size[1] <= 0 or path.stat().st_size <= 0:
                raise ValueError(f"empty visualization image: {path}")
            image_sizes[path.name] = [int(image.size[0]), int(image.size[1])]

    path_hash = str(row.get("path_hash") or "")
    map_hash = sha256_file(artifact.hospital_map.image_path)
    manifest = {
        "schema_version": "two_layer_pipeline_visualization.v1",
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_version": PROTOCOL_VERSION,
        "experiment_id": str(row.get("experiment_id") or output.name),
        "source_experiment_dir": str(source_experiment_dir),
        "map_id": task.map_id,
        "query_id": task.query_id,
        "run_mode": row.get("run_mode"),
        "repetition": int(row.get("repetition") or 0),
        "map_hash": map_hash,
        "map_yaml_hash": sha256_file(artifact.hospital_map.yaml_path),
        "query_hash": str(row.get("query_hash") or ""),
        "path_hash": path_hash,
        "footprint_hash": footprint_hash(FOOTPRINT),
        "topology_graph_hash": _sha256_bytes(topology_graph_path.read_bytes()),
        "map_resolution": float(artifact.hospital_map.resolution),
        "origin": list(artifact.hospital_map.origin),
        "map_width_cells": int(artifact.hospital_map.width),
        "map_height_cells": int(artifact.hospital_map.height),
        "l1_backend": "skeleton_distance_transform_v1 + graph_astar",
        "l1_selected_topology_node_ids": list(route.node_ids),
        "l1_selected_topology_edge_ids": list(route.edge_ids),
        "l1_start_attachment_node": int(attachment_start),
        "l1_goal_attachment_node": int(attachment_goal),
        "l1_route_length_m": float(route.length_m),
        "l1_min_channel_width_m": float(route.min_width_m),
        "l2_enabled": False,
        "l2_call_count": 0,
        "l3_backend": L3_BACKEND,
        "l3_prime_corridor_semantics": str(row.get("corridor_semantics") or "raw_map_smac_aligned"),
        "l3_prime_corridor_padding_m": float(row.get("corridor_padding_m") or CORRIDOR_PADDING_M),
        "l3_prime_corridor_area_ratio": float(row.get("corridor_area_ratio") or 0.0),
        "l3_prime_call_count": int(row.get("l3_prime_call_count") or row.get("l3_call_count") or 0),
        "l3_actual_yaw_pose_count": len(points),
        "final_valid_success": str(row.get("final_valid_success", "")).lower() == "true",
        "timing_ms": {
            "l1": _row_metric(row, "l1_time_ms", "l1_graph_search_ms"),
            "l2": _row_metric(row, "l2_time_ms"),
            "l3_prime": _row_metric(row, "l3_time_ms", "hybrid_planning_time_ms"),
            "planner_wall": _row_metric(row, "planner_wall_ms", "l3_action_wall_ms"),
            "pipeline_wall": _row_metric(row, "pipeline_wall_time_ms", "online_wall_ms"),
            "pipeline_cpu": _row_metric(row, "cpu_ms", "pipeline_cpu_total_ms"),
        },
        "resource_usage": {
            "avg_cpu_percent": _row_metric(row, "avg_cpu_percent"),
            "peak_rss_mib": peak_rss_mib,
            "peak_pss_mib": peak_pss_mib,
            "ready_memory_mib": _row_metric(row, "ready_memory_mib"),
        },
        "dynamic_obstacles": False,
        "source_code_hash": _source_hash(),
        "image_generation_command": (
            "ros2 run arena_evaluation two_layer_pipeline_visualize "
            f"--experiment-dir {source_experiment_dir} --query-id {task.query_id} "
            f"--run-mode {row.get('run_mode')} --repetition {row.get('repetition')} --no-dynamic-obstacles"
        ),
        "image_sizes": image_sizes,
    }
    return manifest


def run_once(experiment: Path, query_id: str, output: Path, *, run_mode: str = "measured", repetition: int = 1, show_node_ids: bool = False) -> Path:
    experiment = experiment.resolve()
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    row = _run_row(experiment, query_id, run_mode, repetition)
    map_id = str(row.get("map_id") or "")
    if not map_id:
        raise ValueError("selected run has no map_id")
    task = _task_for(map_id, query_id)
    map_yaml = _map_yaml(map_id)
    hospital_map = HospitalMap.load(map_yaml)
    if not math.isclose(float(hospital_map.resolution), 0.05, abs_tol=1.0e-9):
        raise ValueError(f"2A-V0 requires 0.05 m/cell, got {hospital_map.resolution}")
    if sha256_file(hospital_map.image_path) != str(row.get("map_sha256") or ""):
        raise ValueError("map image hash does not match the selected measured run")
    if sha256_file(map_yaml) != str(row.get("map_yaml_sha256") or ""):
        raise ValueError("map YAML hash does not match the selected measured run")
    recorded_query_hash = str(row.get("query_hash") or "")
    seed = int(_as_float(row.get("seed"), 0.0))
    if recorded_query_hash != _formal_query_hash(task, seed):
        raise ValueError("query hash does not match the frozen benchmark record")
    recorded_pose = (
        ("start_x", "start_y", "start_yaw", task.start),
        ("goal_x", "goal_y", "goal_yaw", task.goal),
    )
    for x_key, y_key, yaw_key, expected in recorded_pose:
        actual = tuple(_as_float(row.get(key), float("nan")) for key in (x_key, y_key, yaw_key))
        if not np.allclose(actual, expected, atol=1.0e-6, rtol=0.0):
            raise ValueError(f"frozen {x_key.split('_')[0]} pose does not match the selected measured run")
    cache_manifest_path = experiment / "topology_cache_manifest.yaml"
    if not cache_manifest_path.is_file():
        raise FileNotFoundError(f"missing topology cache manifest: {cache_manifest_path}")
    cache_manifest = yaml.safe_load(cache_manifest_path.read_text(encoding="utf-8")) or {}
    cache_directory = Path(str(cache_manifest.get("cache_directory") or "")).resolve()
    if not cache_directory.is_dir():
        raise FileNotFoundError(f"topology cache directory does not exist: {cache_directory}")
    artifact = load_topology(
        cache_directory,
        hospital_map,
        FOOTPRINT,
        padding_m=TOPOLOGY_PADDING_M,
        safety_margin_m=TOPOLOGY_SAFETY_MARGIN_M,
        allow_unknown=False,
    )
    route = _route_from_row(artifact, row)
    points = _load_path(experiment, row)
    if not points:
        raise ValueError("selected 2A-V0 run has an empty path; choose a successful measured sample")
    manifest = render_visualizations(
        output,
        artifact,
        task,
        route,
        points,
        row,
        source_experiment_dir=experiment,
        topology_graph_path=cache_directory / "topology_graph.json",
        show_node_ids=show_node_ids,
    )
    (output / "visualization_manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=False), encoding="utf-8")
    report = [
        f"# {ARCHITECTURE_ID} 可视化检查: {map_id} / {query_id}",
        "",
        f"- 实验目录: `{experiment}`",
        f"- 运行样本: `{run_mode}` repetition `{repetition}`",
        f"- 起点: `{task.start}`",
        f"- 终点: `{task.goal}`",
        f"- 架构: `{ARCHITECTURE_ID}` / `{IMPLEMENTATION_REVISION}`；L1 Graph A*；L2 **关闭**；L3' 全走廊 Smac Hybrid DUBIN。",
        f"- L1 挂接节点: `{route.node_ids[0]}` -> `{route.node_ids[-1]}`；选中节点数 `{len(route.node_ids)}`；通道最小净宽 `{route.min_width_m:.3f} m`。",
        f"- L3' 实际 Smac yaw 点数: `{len(points)}`；调用数: `{row.get('l3_prime_call_count') or row.get('l3_call_count')}`。",
        f"- 最终验收: **{row.get('final_valid_success')}**；路径 hash: `{row.get('path_hash')}`。",
        f"- 分层耗时: L1 `{manifest['timing_ms']['l1']:.1f} ms`；L2 `{manifest['timing_ms']['l2']:.1f} ms`（禁用）；L3' `{manifest['timing_ms']['l3_prime']:.1f} ms`；端到端 `{manifest['timing_ms']['pipeline_wall']:.1f} ms`。",
        f"- 资源: CPU 平均 `{manifest['resource_usage']['avg_cpu_percent']:.1f}%`，峰值 RSS/PSS `{manifest['resource_usage']['peak_rss_mib']:.1f}/{manifest['resource_usage']['peak_pss_mib']:.1f} MiB`。",
        f"- 坐标检查: 分辨率 `{hospital_map.resolution} m/cell`，origin `{list(hospital_map.origin)}`，所有矢量使用世界坐标。",
        "",
        "## 图像",
        "",
        *[f"![{name}](visualizations/{name})" for name in IMAGE_NAMES],
    ]
    (output / "visualization_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay one measured 2A-V0 run and render L1/L3' world-coordinate visualizations")
    parser.add_argument("--experiment-dir", default=str(DEFAULT_EXPERIMENT), help="existing 2A-V0 formal experiment directory")
    parser.add_argument("--query-id", default="A2B-01")
    parser.add_argument("--run-mode", choices=("warmup", "measured"), default="measured")
    parser.add_argument("--repetition", type=int, default=1)
    parser.add_argument("--output-dir")
    parser.add_argument("--no-dynamic-obstacles", action="store_true", help="required: replay static-map-only artifacts")
    parser.add_argument("--show-node-ids", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.no_dynamic_obstacles:
        parser.error("--no-dynamic-obstacles must be explicitly specified")
    experiment = Path(args.experiment_dir).expanduser().resolve()
    timestamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = Path(args.output_dir).expanduser().resolve() if args.output_dir else DEFAULT_OUTPUT_ROOT / experiment.name / args.query_id / timestamp
    try:
        result = run_once(
            experiment,
            args.query_id,
            output,
            run_mode=args.run_mode,
            repetition=args.repetition,
            show_node_ids=args.show_node_ids,
        )
    except Exception as exc:
        print(f"two_layer_pipeline_visualize: ERROR: {exc}", file=sys.stderr)
        return 1
    print(f"2A-V0 visualization output: {result}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
