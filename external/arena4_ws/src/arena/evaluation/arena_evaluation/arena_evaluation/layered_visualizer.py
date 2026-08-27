"""Replay the recorded static L1/L2/L3 Hospital planning results in RViz.

This node is deliberately a replay visualizer. It does not run a planner and
does not represent a live vehicle or dynamic-obstacle experiment. The selected
topology route comes from Stage 8 metadata, the grid path comes from its Stage
6 source run, and the final path comes from the valid Stage 8 L3 run.
"""

from __future__ import annotations

import ast
import csv
import gzip
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .planner_benchmark.config import load_queries
from .planner_benchmark.models import Query


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise ValueError(f"required visualization CSV does not exist: {path}")
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def _parse_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return list(value)
    text = str(value).strip()
    if not text:
        return []
    try:
        result = json.loads(text)
    except json.JSONDecodeError:
        try:
            result = ast.literal_eval(text)
        except (SyntaxError, ValueError):
            return []
    return list(result) if isinstance(result, (list, tuple)) else []


def _load_points(path: Path) -> list[dict[str, float]]:
    if not path.is_file():
        return []
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt") as stream:
        payload = json.load(stream)
    points = payload if isinstance(payload, list) else payload.get("points", [])
    result: list[dict[str, float]] = []
    for point in points:
        if isinstance(point, dict) and {"x", "y", "yaw"}.issubset(point):
            result.append({"x": float(point["x"]), "y": float(point["y"]), "yaw": float(point["yaw"])})
        elif isinstance(point, (list, tuple)) and len(point) >= 3:
            result.append({"x": float(point[0]), "y": float(point[1]), "yaw": float(point[2])})
    return result


def _pose(values: Sequence[float], stamp, frame_id: str = "map") -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.pose.position.x = float(values[0])
    message.pose.position.y = float(values[1])
    yaw = float(values[2])
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def _path(points: Iterable[dict[str, float]], stamp) -> PathMessage:
    message = PathMessage()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    for point in points:
        message.poses.append(_pose((point["x"], point["y"], point["yaw"]), stamp))
    return message


def _path_from_polyline(polyline: Iterable[Sequence[float]], stamp) -> PathMessage:
    message = PathMessage()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    points = list(polyline)
    for index, point in enumerate(points):
        if index + 1 < len(points):
            yaw = math.atan2(points[index + 1][1] - point[1], points[index + 1][0] - point[0])
        elif index:
            yaw = math.atan2(point[1] - points[index - 1][1], point[0] - points[index - 1][0])
        else:
            yaw = 0.0
        message.poses.append(_pose((point[0], point[1], yaw), stamp))
    return message


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _orient_polyline(polyline: list[list[float]], previous: Sequence[float] | None) -> list[list[float]]:
    if not polyline:
        return []
    if previous is None:
        return polyline
    return polyline if _distance(previous, polyline[0]) <= _distance(previous, polyline[-1]) else list(reversed(polyline))


@dataclass
class LayeredRecord:
    query: Query
    stage8: dict[str, str]
    stage6: dict[str, str]
    topology_ids: list[int]
    topology_points: list[list[float]]
    grid_points: list[dict[str, float]]
    kinematic_points: list[dict[str, float]]

    @property
    def l3_valid(self) -> bool:
        return str(self.stage8.get("final_valid_success", "")).lower() in {"true", "1"}


def _select_row(rows: list[dict[str, str]], *, query_id: str, mode: str, repetition: int = 1) -> dict[str, str]:
    candidates = [row for row in rows if row.get("query_id") == query_id and row.get("mode") == mode]
    candidates.sort(key=lambda row: (str(row.get("final_valid_success", "")).lower() not in {"true", "1"}, int(row.get("repetition", "999") or 999)))
    for row in candidates:
        if int(row.get("repetition", "0") or 0) == repetition:
            return row
    return candidates[0] if candidates else {}


def _topology_points(graph_path: Path, edge_ids: Sequence[int], start: Sequence[float]) -> list[list[float]]:
    if not edge_ids or not graph_path.is_file():
        return []
    payload = json.loads(graph_path.read_text())
    edges = {int(edge["edge_id"]): edge for edge in payload.get("edges", [])}
    result: list[list[float]] = []
    previous: Sequence[float] | None = start
    for edge_id in edge_ids:
        edge = edges.get(int(edge_id))
        if edge is None:
            continue
        polyline = [[float(point[0]), float(point[1])] for point in edge.get("polyline", [])]
        polyline = _orient_polyline(polyline, previous)
        if not polyline:
            continue
        if result and _distance(result[-1], polyline[0]) < 1.0e-6:
            result.extend(polyline[1:])
        else:
            result.extend(polyline)
        previous = result[-1]
    return result


def _resolve_path(root: Path, value: str) -> Path:
    candidate = Path(value)
    return candidate if candidate.is_absolute() else root / candidate


class LayeredVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("layered_path_visualizer")
        self.declare_parameter("queries_file", "")
        self.declare_parameter("query_id", "all")
        self.declare_parameter("cycle_interval_seconds", 2.0)
        self.declare_parameter("stage8_directory", "")
        self.declare_parameter("stage6_directory", "")
        self.declare_parameter("topology_directory", "")
        self.declare_parameter("stage_label", "Stage 8A layered replay")

        queries_file = Path(str(self.get_parameter("queries_file").value)).expanduser()
        stage8_directory = Path(str(self.get_parameter("stage8_directory").value)).expanduser()
        stage6_directory = Path(str(self.get_parameter("stage6_directory").value)).expanduser()
        topology_directory = Path(str(self.get_parameter("topology_directory").value)).expanduser()
        query_id = str(self.get_parameter("query_id").value)
        if not queries_file.is_file():
            raise ValueError(f"queries_file does not exist: {queries_file}")
        for directory in (stage8_directory, stage6_directory, topology_directory):
            if not directory.is_dir():
                raise ValueError(f"visualization directory does not exist: {directory}")
        _, queries = load_queries(queries_file)
        if query_id == "all":
            selected_queries = list(queries)
        else:
            selected_queries = [query for query in queries if query.query_id == query_id]
        if not selected_queries:
            raise ValueError(f"no query selected for {query_id!r}")
        self.queries = selected_queries
        self.interval = float(self.get_parameter("cycle_interval_seconds").value)
        if self.interval <= 0.0:
            raise ValueError("cycle_interval_seconds must be positive")
        self.stage_label = str(self.get_parameter("stage_label").value)

        stage8_rows = _read_csv(stage8_directory / "kinematic_runs.csv")
        stage6_rows = _read_csv(stage6_directory / "query_runs.csv")
        stage6_by_id = {row.get("run_id"): row for row in stage6_rows}
        graph_path = topology_directory / "topology_graph.json"
        self.records: list[LayeredRecord] = []
        for query in self.queries:
            stage8 = _select_row(stage8_rows, query_id=query.query_id, mode="layered_hard_radius_l3")
            source_id = stage8.get("source_stage6_run_id", "")
            stage6 = stage6_by_id.get(source_id, {})
            if not stage6:
                stage6 = _select_row(stage6_rows, query_id=query.query_id, mode="topology_guided_grid_fallback")
            topology_ids = [int(value) for value in _parse_list(stage8.get("topology_edge_ids") or stage6.get("topology_edge_ids"))]
            grid_points = _load_points(_resolve_path(stage6_directory, stage6.get("path_file", "")))
            kinematic_points = _load_points(_resolve_path(stage8_directory, stage8.get("path_file", ""))) if stage8 else []
            topology_points = _topology_points(graph_path, topology_ids, query.start)
            self.records.append(LayeredRecord(query, stage8, stage6, topology_ids, topology_points, grid_points, kinematic_points))

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.topology_publisher = self.create_publisher(PathMessage, "/baseline_visualization/topology_route", qos)
        self.grid_publisher = self.create_publisher(PathMessage, "/baseline_visualization/grid_path", qos)
        self.kinematic_publisher = self.create_publisher(PathMessage, "/baseline_visualization/kinematic_path", qos)
        self.start_publisher = self.create_publisher(PoseStamped, "/baseline_visualization/start", qos)
        self.goal_publisher = self.create_publisher(PoseStamped, "/baseline_visualization/goal", qos)
        self.map_subscription = self.create_subscription(OccupancyGrid, "/map", self._map_callback, qos)
        self.map_ready = False
        self.record_index = 0
        self.next_publish_ns = 0
        self.started_ns = self.get_clock().now().nanoseconds
        self.startup_timeout_seconds = 30.0
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(f"replaying {len(self.records)} layered query(s) every {self.interval:.2f} s")

    def _map_callback(self, message: OccupancyGrid) -> None:
        if not self.map_ready and int(message.info.width) > 0 and int(message.info.height) > 0 and len(message.data) == int(message.info.width) * int(message.info.height):
            self.map_ready = True
            self.get_logger().info(f"static map ready: {message.info.width}x{message.info.height} at {message.info.resolution:.3f} m/cell")

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if not self.map_ready:
            if (now_ns - self.started_ns) / 1.0e9 > self.startup_timeout_seconds:
                self.get_logger().error("/map did not become available")
                self.started_ns = now_ns
            return
        if now_ns < self.next_publish_ns:
            return
        self._publish_record(self.records[self.record_index])
        self.record_index = (self.record_index + 1) % len(self.records)
        self.next_publish_ns = now_ns + int(self.interval * 1.0e9)

    def _publish_record(self, record: LayeredRecord) -> None:
        stamp = self.get_clock().now().to_msg()
        self.start_publisher.publish(_pose(record.query.start, stamp))
        self.goal_publisher.publish(_pose(record.query.goal, stamp))
        self.topology_publisher.publish(_path_from_polyline(record.topology_points, stamp))
        self.grid_publisher.publish(_path(record.grid_points, stamp))
        # Do not display an invalid L3 result as a successful final route.
        final_points = record.kinematic_points if record.l3_valid else []
        self.kinematic_publisher.publish(_path(final_points, stamp))

        stage8 = record.stage8
        result = "VALID" if record.l3_valid else str(stage8.get("failure_code") or "L3_NOT_VALID")
        grid_mode = stage8.get("grid_mode") or record.stage6.get("grid_mode") or "unknown"
        repairs = stage8.get("repair_window_count") or "0"
        radius = stage8.get("minimum_radius_observed_m") or "n/a"
        self.get_logger().info(
            f"showing {record.query.query_id}: L1 edges={len(record.topology_ids)}, "
            f"L2 poses={len(record.grid_points)}, L3={'valid' if record.l3_valid else result}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LayeredVisualizer()
        rclpy.spin(node)
    except (OSError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"layered_path_visualizer: ERROR: {exc}")
        raise SystemExit(2) from exc
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
