"""Publish one frozen Stage 5 query for RViz inspection."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMessage
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from .planner_benchmark.config import load_queries
from .planner_benchmark.models import Query


def select_query(queries: Sequence[Query], query_id: str) -> Query:
    for query in queries:
        if query.query_id == query_id:
            return query
    available = ", ".join(query.query_id for query in queries)
    raise ValueError(f"unknown query_id {query_id!r}; available: {available}")


def select_queries(queries: Sequence[Query], query_id: str) -> list[Query]:
    if query_id == "all":
        if not queries:
            raise ValueError("query file contains no queries")
        return list(queries)
    return [select_query(queries, query_id)]


def pose_stamped(values: Sequence[float], frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(values[0])
    pose.pose.position.y = float(values[1])
    yaw = float(values[2])
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def costmap_is_ready(message: OccupancyGrid) -> bool:
    expected_size = int(message.info.width) * int(message.info.height)
    if expected_size <= 0 or len(message.data) != expected_size:
        return False
    has_free = False
    has_lethal = False
    for value in message.data:
        has_free = has_free or value == 0
        has_lethal = has_lethal or value >= 100
        if has_free and has_lethal:
            return True
    return False


class BaselineVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("baseline_path_visualizer")
        self.declare_parameter("queries_file", "")
        self.declare_parameter("query_id", "all")
        self.declare_parameter("cycle_interval_seconds", 1.0)
        self.declare_parameter("planner_label", "baseline")
        self.declare_parameter("action_timeout_seconds", 30.0)

        queries_file = Path(str(self.get_parameter("queries_file").value)).expanduser()
        query_id = str(self.get_parameter("query_id").value)
        if not queries_file.is_file():
            raise ValueError(f"queries_file does not exist: {queries_file}")
        _, queries = load_queries(queries_file)
        self.queries = select_queries(queries, query_id)
        self.cycle_interval_seconds = float(
            self.get_parameter("cycle_interval_seconds").value
        )
        if self.cycle_interval_seconds <= 0.0:
            raise ValueError("cycle_interval_seconds must be positive")
        self.planner_label = str(self.get_parameter("planner_label").value)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(
            PathMessage, "/baseline_visualization/path", latched_qos
        )
        self.start_publisher = self.create_publisher(
            PoseStamped, "/baseline_visualization/start", latched_qos
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/baseline_visualization/goal", latched_qos
        )
        self.label_publisher = self.create_publisher(
            Marker, "/baseline_visualization/query_label", latched_qos
        )
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap,
            latched_qos,
        )
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.current_query = self.queries[0]
        self.start_pose = pose_stamped(self.current_query.start)
        self.goal_pose = pose_stamped(self.current_query.goal)
        self.path: PathMessage | None = None
        self.query_index = 0
        self.goal_in_flight = False
        self.startup_failed = False
        self.costmap_ready = False
        self.next_send_ns = 0
        self.wait_started_ns = self.get_clock().now().nanoseconds
        self.timeout_seconds = float(self.get_parameter("action_timeout_seconds").value)
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"waiting for the populated static costmap, then cycling "
            f"{len(self.queries)} query(s) every {self.cycle_interval_seconds:.2f} s"
        )

    def _costmap(self, message: OccupancyGrid) -> None:
        if self.costmap_ready or not costmap_is_ready(message):
            return
        self.costmap_ready = True
        self.get_logger().info(
            f"static costmap ready: {message.info.width}x{message.info.height} "
            f"at {message.info.resolution:.3f} m/cell"
        )

    def _tick(self) -> None:
        if self.goal_in_flight or self.startup_failed:
            return
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.wait_started_ns) / 1e9
        if not self.costmap_ready or not self.client.server_is_ready():
            if elapsed > self.timeout_seconds:
                self.startup_failed = True
                self.get_logger().error(
                    "populated /global_costmap/costmap or /compute_path_to_pose did not become available"
                )
            return
        if now_ns < self.next_send_ns:
            return

        query = self.queries[self.query_index]
        self.query_index = (self.query_index + 1) % len(self.queries)
        self._activate_query(query)
        request = ComputePathToPose.Goal()
        request.start = self.start_pose
        request.goal = self.goal_pose
        request.planner_id = "GridBased"
        request.use_start = True
        self.goal_in_flight = True
        self.next_send_ns = now_ns + int(self.cycle_interval_seconds * 1e9)
        future = self.client.send_goal_async(request)
        future.add_done_callback(
            lambda completed, active_query=query: self._goal_response(
                completed, active_query
            )
        )
        self.get_logger().info(f"sent {query.query_id} to /compute_path_to_pose")

    def _activate_query(self, query: Query) -> None:
        self.current_query = query
        self.start_pose = pose_stamped(query.start)
        self.goal_pose = pose_stamped(query.goal)
        now = self.get_clock().now().to_msg()
        self.start_pose.header.stamp = now
        self.goal_pose.header.stamp = now
        self.start_publisher.publish(self.start_pose)
        self.goal_publisher.publish(self.goal_pose)
        empty_path = PathMessage()
        empty_path.header.frame_id = "map"
        empty_path.header.stamp = now
        self.path = None
        self.path_publisher.publish(empty_path)
        self._publish_label(query, "planning")

    def _publish_label(self, query: Query, status: str) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "baseline_query"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(query.goal[0])
        marker.pose.position.y = float(query.goal[1])
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.8
        marker.color.r = 0.08
        marker.color.g = 0.08
        marker.color.b = 0.08
        marker.color.a = 1.0
        marker.text = f"{query.query_id} | {self.planner_label} | {status}"
        self.label_publisher.publish(marker)

    def _goal_response(self, future, query: Query) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.goal_in_flight = False
            self._publish_label(query, "REJECTED")
            self.get_logger().error(
                f"planner rejected visualization query {query.query_id}"
            )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, active_query=query: self._result(
                completed, active_query
            )
        )

    def _result(self, future, query: Query) -> None:
        wrapped = future.result()
        self.goal_in_flight = False
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = getattr(wrapped, "status", "unknown")
            self._publish_label(query, f"FAILED ({status})")
            self.get_logger().error(
                f"{query.query_id} planning failed with action status {status}"
            )
            return
        path = wrapped.result.path
        if not path.poses:
            self._publish_label(query, "EMPTY PATH")
            self.get_logger().error(f"{query.query_id} returned an empty path")
            return
        self.path = path
        self.path_publisher.publish(path)
        duration = wrapped.result.planning_time
        planning_ms = float(duration.sec) * 1000.0 + float(duration.nanosec) / 1e6
        self._publish_label(query, f"{planning_ms:.1f} ms")
        self.get_logger().info(
            f"showing {query.query_id}: {len(path.poses)} poses, "
            f"planning_time={planning_ms:.3f} ms"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BaselineVisualizer()
        rclpy.spin(node)
    except (OSError, ValueError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"baseline_path_visualizer: ERROR: {exc}")
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
