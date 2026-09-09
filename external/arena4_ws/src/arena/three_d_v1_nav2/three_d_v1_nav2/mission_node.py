"""Event-driven sequential execution of the frozen independent query set."""

from __future__ import annotations

import csv
import json
import math
import statistics
import time
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from arena_evaluation.semantic_query_defaults import load_query_set

from .contracts import (
    CONTROLLER_GOAL_POSITION_TOLERANCE_M, EXPECTED_MAP_SHA256, EXPECTED_QUERY_IDS,
    EXPECTED_SEMANTIC_MAP_HASH, GOAL_POSITION_TOLERANCE_M, GOAL_YAW_TOLERANCE_RAD,
    QUERY_SET, controller_goal_yaw_tolerance, sha256_file,
    verify_run_inputs,
)
from .mission_state import Acceptance, MissionMachine, MissionState
from .runtime_safety import MotionEvidence, SweptFootprintMonitor, telemetry_failure


def _yaw(orientation: Any) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _angle_error(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


class CsvSink:
    def __init__(self, path: Path, fields: Iterable[str]) -> None:
        self.path = path
        self.fields = tuple(fields)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("w", newline="", encoding="utf-8", buffering=1)
        self.writer = csv.DictWriter(self.stream, fieldnames=self.fields, extrasaction="ignore")
        self.writer.writeheader()

    def write(self, **row: Any) -> None:
        self.writer.writerow(row)

    def close(self) -> None:
        self.stream.close()


class SequentialMission(Node):
    LIFECYCLE_NODES = (
        "map_server", "planner_server", "controller_server", "smoother_server",
        "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother",
    )

    def __init__(self) -> None:
        super().__init__("three_d_v1_seq8_mission")
        for name, default in (
            ("output_dir", ""), ("path_bank_index", ""), ("query_id", ""),
            ("behavior_tree", ""), ("stack_timeout_s", 120.0),
            ("localization_timeout_s", 30.0), ("action_timeout_s", 900.0),
            ("stop_hold_s", 1.0),
            ("online", False),
        ):
            self.declare_parameter(name, default)
        output_value = str(self.get_parameter("output_dir").value)
        index_value = str(self.get_parameter("path_bank_index").value)
        if not output_value or not index_value:
            raise ValueError("output_dir and path_bank_index are mandatory")
        self.output = Path(output_value).resolve()
        self.index_path = Path(index_value).resolve()
        verify_run_inputs(self.index_path.parent.parent)
        from arena_evaluation.planner_benchmark.map_utils import HospitalMap
        self.footprint_monitor = SweptFootprintMonitor(HospitalMap.load(
            self.index_path.parent.parent / "derived_map/extracted/optemap.yaml"))
        self.behavior_tree = str(self.get_parameter("behavior_tree").value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.stack_timeout_s = float(self.get_parameter("stack_timeout_s").value)
        self.localization_timeout_s = float(self.get_parameter("localization_timeout_s").value)
        self.action_timeout_s = float(self.get_parameter("action_timeout_s").value)
        self.stop_hold_s = float(self.get_parameter("stop_hold_s").value)
        self.online = bool(self.get_parameter("online").value)
        self.online_ready = False
        self.online_audit = None

        queries, _intents, _metadata = load_query_set(
            QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
            actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
            require_default_contract=True,
        )
        requested = str(self.get_parameter("query_id").value)
        if requested:
            queries = [item for item in queries if item.query_id == requested]
            if len(queries) != 1:
                raise ValueError(f"unknown frozen query: {requested}")
        elif tuple(item.query_id for item in queries) != EXPECTED_QUERY_IDS:
            raise RuntimeError("frozen query order mismatch")
        self.queries = queries
        self.query_by_id = {item.query_id: item for item in queries}
        self.path_records = self._load_path_records()
        self.machine = MissionMachine(tuple(item.query_id for item in queries))

        self.force_stop_pub = self.create_publisher(Bool, "/three_d_v1/force_stop", 10)
        self.state_pub = self.create_publisher(String, "/three_d_v1/mission_state", 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.teleport = self.create_client(SetEntityPose, "/world/default/set_pose")
        self.navigation_manager = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes",
        )
        self.controller_parameters = self.create_client(
            SetParameters, "/controller_server/set_parameters",
        )
        self.clear_clients = {
            "global": self.create_client(
                ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"),
            "local": self.create_client(
                ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"),
        }
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self.LIFECYCLE_NODES
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, "/model/jackal/odometry", self._odom, 100)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._localization, 20)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._global_costmap, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._local_costmap, 10)
        self.create_subscription(LaserScan, "/scan", self._scan, 20)
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._teb_command, 100)
        self.create_subscription(String, "/three_d_v1/teb_trace", self._teb_trace, 100)
        self.create_subscription(String, "/three_d_v1/cmd_guard_trace", self._guard_trace, 100)
        self.create_subscription(Bool, "/three_d_v1/online_ready", self._online_ready,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/three_d_v1/online_audit", self._online_audit, 10)

        self.latest_odom: Optional[Odometry] = None
        self.odom_mono_ns = 0
        self.latest_localization: Optional[PoseWithCovarianceStamped] = None
        self.localization_mono_ns = 0
        self.scan_mono_ns = 0
        self.teleported_odom_ready_ns = 0
        self.latest_plan: Optional[NavPath] = None
        self.latest_plan_mono_ns = 0
        self.global_costmap_mono_ns = 0
        self.local_costmap_mono_ns = 0
        self.stopped_since_ns: Optional[int] = None
        self.active_metrics = False
        self.motion_evidence = MotionEvidence()
        self.runtime_failure = ""
        self.guard_samples = 0
        self.teb_samples = 0
        self.current_goal_handle = None
        self.lifecycle_checks = {}
        self.last_metric_position: Optional[tuple[float, float]] = None
        self.distance_traveled = 0.0
        self.tracking_errors: list[float] = []
        self.odom_sample_index = 0
        self.teb_command_times: list[int] = []
        self.teb_zero_commands = 0
        self.teb_feasibility_stops = 0
        self.teb_collision_stops = 0
        self.transition_start_ns = 0

        self.transitions = CsvSink(self.output / "mission_state_transitions.csv", (
            "wall_time_ns", "ros_time_ns", "query_index", "query_id", "from_state",
            "event", "to_state", "failure_code", "goal_uuid",
        ))
        self.health = CsvSink(self.output / "lifecycle_and_tf_health.csv", (
            "wall_time_ns", "query_id", "all_lifecycle_active", "inactive_nodes",
            "tf_map_to_base", "localization_fresh", "global_costmap_fresh",
            "local_costmap_fresh", "odom_fresh", "scan_fresh", "pose_ready", "stopped",
        ))
        self.goal_checker_budgets = CsvSink(self.output / "goal_checker_budgets.csv", (
            "wall_time_ns", "sequence", "query_id", "l3_endpoint_yaw_rad",
            "action_goal_yaw_rad", "endpoint_yaw_offset_rad",
            "controller_xy_tolerance_m", "controller_yaw_tolerance_rad",
            "parameter_update_success",
        ))
        self.goal_results = CsvSink(self.output / "goal_result_and_tolerance.csv", (
            "sequence", "query_id", "goal_uuid", "action_status", "nav2_succeeded",
            "position_error_m", "yaw_error_rad", "position_tolerance_m",
            "yaw_tolerance_rad", "position_pass", "yaw_pass", "stopped_pass",
            "plan_identity_pass", "canonical_final_audit_pass", "accepted",
        ))
        self.runs = CsvSink(self.output / "runs.csv", (
            "sequence", "query_id", "goal_uuid", "status", "duration_s",
            "distance_traveled_m", "final_position_error_m", "final_yaw_error_rad",
            "canonical_path_hash", "path_file_sha256", "failure_code",
        ))
        self.metrics = CsvSink(self.output / "per_query_metrics.csv", (
            "sequence", "query_id", "arrival_time_s", "distance_traveled_m",
            "tracking_error_mean_m", "tracking_error_p95_m", "tracking_error_max_m",
            "teb_command_count", "teb_control_frequency_hz", "teb_period_p95_ms",
            "teb_zero_command_count", "reverse_distance_m", "rotate_in_place_count",
            "measured_collision_samples", "measured_curvature_violations",
            "measured_accel_violations", "minimum_footprint_clearance_lower_bound_m",
        ))
        self.teb_metrics = CsvSink(self.output / "teb_control_metrics.csv", (
            "sequence", "query_id", "teb_plugin", "compute_command_count",
            "control_frequency_hz", "period_p50_ms", "period_p95_ms", "period_p99_ms",
            "zero_command_count", "feasibility_stop_events", "collision_stop_events",
            "metric_source",
        ))
        self.teb_trace = CsvSink(self.output / "teb_trace.csv", (
            "wall_time_ns", "ros_time_ns", "sequence", "query_id", "mission_state",
            "outcome", "planner_success", "diverged", "feasibility_stop",
            "collision_stop", "feasibility_index", "feasibility_stop_count",
            "teb_pose_count", "raw_cmd_v", "raw_cmd_w", "command_lookahead",
            "measured_start_v", "optimizer_v_max", "optimizer_time_weight",
            "command_forward_projection", "command_dt", "forward_only_fallback",
            "terminal_goal_distance", "terminal_speed_cap", "terminal_slowdown",
            "terminal_extension",
            "terminal_extension_curvature",
            "terminal_waypoint_active", "terminal_waypoint_weight",
            "cmd_v", "cmd_w",
            "compute_duration_us",
        ))
        self.odom_trace = CsvSink(self.output / "odom_trace.csv", (
            "wall_time_ns", "ros_time_ns", "sequence", "query_id", "mission_state",
            "x", "y", "yaw", "linear_x", "angular_z",
        ))
        self.failures = CsvSink(self.output / "failures.csv", (
            "wall_time_ns", "sequence", "query_id", "state", "failure_code", "failure_detail",
        ))
        self.create_timer(1.0, self._active_health)

    def _active_health(self):
        if not self.active_metrics:
            self.lifecycle_checks.clear()
            return
        now = time.monotonic_ns()
        inactive = []
        for name, client in self.lifecycle_clients.items():
            pending = self.lifecycle_checks.get(name)
            if pending is not None:
                future, sent_ns = pending
                if future.done():
                    response = future.result()
                    if response is None or response.current_state.label != "active":
                        inactive.append(name)
                    del self.lifecycle_checks[name]
                elif now - sent_ns > 2_000_000_000:
                    inactive.append(name)
            if name not in self.lifecycle_checks:
                if client.service_is_ready():
                    self.lifecycle_checks[name] = (client.call_async(GetState.Request()), now)
                else:
                    inactive.append(name)
        tf_ready = False
        try:
            transform = self.tf_buffer.lookup_transform("map", "jackal/base_link", Time())
            age = self.get_clock().now().nanoseconds - Time.from_msg(transform.header.stamp).nanoseconds
            tf_ready = 0 <= age < 500_000_000
        except Exception:
            pass
        fresh = {
            "localization_fresh": now - self.localization_mono_ns < 500_000_000,
            "global_costmap_fresh": now - self.global_costmap_mono_ns < 2_000_000_000,
            "local_costmap_fresh": now - self.local_costmap_mono_ns < 1_000_000_000,
            "odom_fresh": now - self.odom_mono_ns < 500_000_000,
            "scan_fresh": now - self.scan_mono_ns < 500_000_000,
        }
        self.health.write(wall_time_ns=time.time_ns(), query_id=self.machine.query_id,
                          all_lifecycle_active=not inactive, inactive_nodes=";".join(inactive),
                          tf_map_to_base=tf_ready, pose_ready=True, stopped=self._is_stopped(), **fresh)
        if inactive or not tf_ready or not all(fresh.values()):
            self.runtime_failure = self.runtime_failure or "ACTIVE_STACK_HEALTH_FAILED"

    def _load_path_records(self) -> Dict[str, Dict[str, Any]]:
        if not self.index_path.is_file():
            raise FileNotFoundError(self.index_path)
        records: Dict[str, Dict[str, Any]] = {}
        with self.index_path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                query_id = str(row["query_id"])
                if query_id in records:
                    raise RuntimeError(f"duplicate path-bank row: {query_id}")
                if row.get("final_audit_passed") != "true":
                    raise RuntimeError(f"path-bank final audit is not passed: {query_id}")
                path_file = Path(str(row["path_file"]))
                if not path_file.is_absolute():
                    path_file = self.index_path.parent / path_file
                if sha256_file(path_file) != row.get("path_sha256"):
                    raise RuntimeError(f"path-bank SHA mismatch: {query_id}")
                points = []
                with path_file.open(newline="", encoding="utf-8") as path_stream:
                    for point in csv.DictReader(path_stream):
                        points.append((float(point["x"]), float(point["y"]), float(point["yaw"])))
                row["path_file_resolved"] = str(path_file.resolve())
                row["points"] = points
                records[query_id] = row
        missing = sorted(set(self.query_by_id) - set(records))
        if missing:
            raise RuntimeError(f"path bank missing selected queries: {missing}")
        return records

    def destroy_node(self):  # type: ignore[no-untyped-def]
        self._force_stop(True)
        for sink in (
            self.transitions, self.health, self.goal_results, self.runs,
            self.goal_checker_budgets, self.metrics, self.teb_metrics, self.teb_trace,
            self.odom_trace, self.failures,
        ):
            sink.close()
        return super().destroy_node()

    def _odom(self, message: Odometry) -> None:
        now_ns = time.monotonic_ns()
        self.latest_odom = message
        self.odom_mono_ns = now_ns
        twist = message.twist.twist
        self.odom_trace.write(
            wall_time_ns=time.time_ns(), ros_time_ns=self.get_clock().now().nanoseconds,
            sequence=self.machine.query_index + 1, query_id=self.machine.query_id,
            mission_state=self.machine.state.value, x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y), yaw=_yaw(message.pose.pose.orientation),
            linear_x=float(twist.linear.x), angular_z=float(twist.angular.z),
        )
        if abs(twist.linear.x) <= 0.02 and abs(twist.angular.z) <= 0.03:
            if self.stopped_since_ns is None:
                self.stopped_since_ns = now_ns
        else:
            self.stopped_since_ns = None
        if not self.active_metrics:
            return
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        pose = message.pose.pose
        collision = self.footprint_monitor.observe((float(pose.position.x), float(pose.position.y),
                                                    _yaw(pose.orientation)))
        self.motion_evidence.observe(stamp_ns, float(twist.linear.x), float(twist.angular.z), collision=collision)
        self.runtime_failure = self.runtime_failure or self.motion_evidence.failure_code
        position = (float(message.pose.pose.position.x), float(message.pose.pose.position.y))
        if self.last_metric_position is not None:
            self.distance_traveled += math.hypot(
                position[0] - self.last_metric_position[0], position[1] - self.last_metric_position[1],
            )
        self.last_metric_position = position
        self.odom_sample_index += 1
        if self.odom_sample_index % 5 == 0:
            points = self.path_records[self.machine.query_id]["points"]
            self.tracking_errors.append(min(
                math.hypot(position[0] - point[0], position[1] - point[1]) for point in points
            ))

    def _localization(self, message: PoseWithCovarianceStamped) -> None:
        self.latest_localization = message
        self.localization_mono_ns = time.monotonic_ns()

    def _guard_trace(self, message: String) -> None:
        if not self.active_metrics:
            return
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            self.runtime_failure = "INVALID_GUARD_TELEMETRY"
            return
        self.guard_samples += 1
        self.runtime_failure = self.runtime_failure or telemetry_failure(payload)

    def _online_ready(self, message):
        self.online_ready = bool(message.data)

    def _online_audit(self, message):
        if not self.online:
            return
        payload = json.loads(message.data)
        if payload.get("query_id") == self.machine.query_id and payload.get("success"):
            self.online_audit = payload
            self.path_records[self.machine.query_id]["points"] = [
                (float(p["x"]), float(p["y"]), float(p["yaw"])) for p in payload["points"]]
            self.path_records[self.machine.query_id]["canonical_path_hash"] = payload["canonical_path_hash"]
            self.path_records[self.machine.query_id]["path_sha256"] = payload.get("path_sha256", "")

    def _scan(self, _message: LaserScan) -> None:
        self.scan_mono_ns = time.monotonic_ns()

    def _global_costmap(self, _message: OccupancyGrid) -> None:
        self.global_costmap_mono_ns = time.monotonic_ns()

    def _local_costmap(self, _message: OccupancyGrid) -> None:
        self.local_costmap_mono_ns = time.monotonic_ns()

    def _plan(self, message: NavPath) -> None:
        self.latest_plan = message
        self.latest_plan_mono_ns = time.monotonic_ns()

    def _teb_command(self, message: Twist) -> None:
        if not self.active_metrics:
            return
        self.teb_command_times.append(time.monotonic_ns())
        if abs(message.linear.x) <= 1.0e-6 and abs(message.angular.z) <= 1.0e-6:
            self.teb_zero_commands += 1

    def _teb_trace(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self.teb_trace.write(
            wall_time_ns=time.time_ns(), ros_time_ns=self.get_clock().now().nanoseconds,
            sequence=self.machine.query_index + 1, query_id=self.machine.query_id,
            mission_state=self.machine.state.value,
            outcome=payload.get("outcome", ""),
            planner_success=payload.get("planner_success", ""),
            diverged=payload.get("diverged", ""),
            feasibility_stop=payload.get("feasibility_stop", ""),
            collision_stop=payload.get("collision_stop", ""),
            feasibility_index=payload.get("feasibility_index", ""),
            feasibility_stop_count=payload.get("feasibility_stop_count", ""),
            teb_pose_count=payload.get("teb_pose_count", ""),
            raw_cmd_v=payload.get("raw_cmd_v", ""),
            raw_cmd_w=payload.get("raw_cmd_w", ""),
            measured_start_v=payload.get("measured_start_v", ""),
            optimizer_v_max=payload.get("optimizer_v_max", ""),
            optimizer_time_weight=payload.get("optimizer_time_weight", ""),
            command_lookahead=payload.get("command_lookahead", ""),
            command_forward_projection=payload.get("command_forward_projection", ""),
            command_dt=payload.get("command_dt", ""),
            forward_only_fallback=payload.get("forward_only_fallback", ""),
            terminal_goal_distance=payload.get("terminal_goal_distance", ""),
            terminal_speed_cap=payload.get("terminal_speed_cap", ""),
            terminal_slowdown=payload.get("terminal_slowdown", ""),
            terminal_extension=payload.get("terminal_extension", ""),
            terminal_extension_curvature=payload.get("terminal_extension_curvature", ""),
            terminal_waypoint_active=payload.get("terminal_waypoint_active", ""),
            terminal_waypoint_weight=payload.get("terminal_waypoint_weight", ""),
            cmd_v=payload.get("cmd_v", ""), cmd_w=payload.get("cmd_w", ""),
            compute_duration_us=payload.get("compute_duration_us", ""),
        )
        if not self.active_metrics:
            return
        self.teb_samples += 1
        self.runtime_failure = self.runtime_failure or telemetry_failure(payload, controller=True)
        self.teb_feasibility_stops += int(bool(payload.get("feasibility_stop", False)))
        self.teb_collision_stops += int(bool(payload.get("collision_stop", False)))

    def _force_stop(self, enabled: bool) -> None:
        message = Bool()
        message.data = enabled
        self.force_stop_pub.publish(message)

    def _publish_state(self) -> None:
        message = String()
        message.data = json.dumps({
            "query_index": self.machine.query_index,
            "query_id": self.machine.query_id,
            "state": self.machine.state.value,
            "goal_uuid": self.machine.goal_uuid or "",
            "failure_code": self.machine.failure_code,
        }, sort_keys=True, separators=(",", ":"))
        self.state_pub.publish(message)

    def _event(self, previous: MissionState, event: str) -> None:
        self.transitions.write(
            wall_time_ns=time.time_ns(), ros_time_ns=self.get_clock().now().nanoseconds,
            query_index=self.machine.query_index, query_id=self.machine.query_id,
            from_state=previous.value, event=event, to_state=self.machine.state.value,
            failure_code=self.machine.failure_code, goal_uuid=self.machine.goal_uuid or "",
        )
        self._publish_state()

    def _wait(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
            if predicate():
                return True
        return False

    def _service_call(self, client: Any, request: Any, timeout_s: float) -> Any:
        if not self._wait(client.service_is_ready, timeout_s):
            raise TimeoutError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        if not self._wait(future.done, timeout_s):
            raise TimeoutError(f"service call timeout: {client.srv_name}")
        return future.result()

    def _set_controller_goal_budget(self, query: Any) -> None:
        endpoint_yaw = float(self.path_records[query.query_id]["points"][-1][2])
        action_yaw = float(query.goal[2])
        yaw_tolerance = controller_goal_yaw_tolerance(action_yaw, endpoint_yaw)
        request = SetParameters.Request()
        for name, value in (
            ("stopped_goal_checker.xy_goal_tolerance", CONTROLLER_GOAL_POSITION_TOLERANCE_M),
            ("stopped_goal_checker.yaw_goal_tolerance", yaw_tolerance),
        ):
            request.parameters.append(Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value),
            ))
        response = self._service_call(self.controller_parameters, request, 5.0)
        successful = (
            response is not None and len(response.results) == len(request.parameters) and
            all(result.successful for result in response.results)
        )
        self.goal_checker_budgets.write(
            wall_time_ns=time.time_ns(), sequence=self.machine.query_index + 1,
            query_id=query.query_id, l3_endpoint_yaw_rad=endpoint_yaw,
            action_goal_yaw_rad=action_yaw,
            endpoint_yaw_offset_rad=_angle_error(endpoint_yaw, action_yaw),
            controller_xy_tolerance_m=CONTROLLER_GOAL_POSITION_TOLERANCE_M,
            controller_yaw_tolerance_rad=yaw_tolerance,
            parameter_update_success=successful,
        )
        if not successful:
            reasons = ";".join(
                result.reason for result in (response.results if response else [])
                if not result.successful
            )
            raise RuntimeError(f"controller goal budget update rejected: {reasons}")

    def _lifecycle_states(self) -> Dict[str, str]:
        result: Dict[str, str] = {}
        for name, client in self.lifecycle_clients.items():
            if not client.service_is_ready():
                result[name] = "SERVICE_UNAVAILABLE"
                continue
            future = client.call_async(GetState.Request())
            if not self._wait(future.done, 1.0):
                result[name] = "TIMEOUT"
                continue
            response = future.result()
            result[name] = str(response.current_state.label if response else "NO_RESPONSE")
        return result

    def _stack_active(self) -> bool:
        states = self._lifecycle_states()
        inactive = [name for name, state in states.items() if state != "active"]
        tf_ready = self.tf_buffer.can_transform(
            "map", "jackal/base_link", Time(), timeout=Duration(seconds=0.0),
        )
        self.health.write(
            wall_time_ns=time.time_ns(), query_id=self.machine.query_id,
            all_lifecycle_active=not inactive, inactive_nodes=";".join(inactive),
            tf_map_to_base=tf_ready, localization_fresh=False,
            global_costmap_fresh=False, local_costmap_fresh=False,
            odom_fresh=self.latest_odom is not None, scan_fresh=self.scan_mono_ns > 0,
            pose_ready=False, stopped=self._is_stopped(),
        )
        return (not inactive and tf_ready and self.latest_odom is not None and self.action.server_is_ready()
                and (not self.online or self.online_ready))

    def _is_stopped(self) -> bool:
        fresh = self.odom_mono_ns > 0 and (time.monotonic_ns() - self.odom_mono_ns) < 500_000_000
        return fresh and self.stopped_since_ns is not None and (
            time.monotonic_ns() - self.stopped_since_ns
        ) / 1.0e9 >= self.stop_hold_s

    def _set_initial_pose(self, query: Any) -> int:
        self._force_stop(True)
        if not self._wait(self._is_stopped, self.localization_timeout_s):
            raise TimeoutError("robot failed to stop before teleport")
        reset = ManageLifecycleNodes.Request()
        reset.command = ManageLifecycleNodes.Request.RESET
        reset_response = self._service_call(self.navigation_manager, reset, 30.0)
        if reset_response is None or not reset_response.success:
            raise RuntimeError("Nav2 lifecycle manager rejected reset before teleport")
        request = SetEntityPose.Request()
        request.entity.name = "jackal"
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(query.start[0])
        request.pose.position.y = float(query.start[1])
        request.pose.position.z = 0.20
        request.pose.orientation.z = math.sin(float(query.start[2]) / 2.0)
        request.pose.orientation.w = math.cos(float(query.start[2]) / 2.0)
        teleport_ns = time.monotonic_ns()
        self.teleported_odom_ready_ns = 0
        response = self._service_call(self.teleport, request, 10.0)
        if response is None or not response.success:
            raise RuntimeError("Gazebo SetEntityPose rejected frozen start")
        if not self._wait(
            lambda: self._teleported_sensors_ready(query, teleport_ns),
            self.localization_timeout_s,
        ):
            raise TimeoutError("fresh odometry/scan did not settle after teleport")
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.header.stamp = self.get_clock().now().to_msg()
        initial.pose.pose = request.pose
        initial.pose.pose.position.z = 0.0
        initial.pose.covariance[0] = 1.0e-4
        initial.pose.covariance[7] = 1.0e-4
        initial.pose.covariance[35] = 4.0e-4
        self.initial_pose_pub.publish(initial)
        startup = ManageLifecycleNodes.Request()
        startup.command = ManageLifecycleNodes.Request.STARTUP
        startup_response = self._service_call(self.navigation_manager, startup, 30.0)
        if startup_response is None or not startup_response.success:
            raise RuntimeError("Nav2 lifecycle manager rejected startup after teleport")
        if not self._wait(self._stack_active, self.stack_timeout_s):
            raise TimeoutError("Nav2 did not return active after teleport")
        self._set_controller_goal_budget(query)
        for client in self.clear_clients.values():
            self._service_call(client, ClearEntireCostmap.Request(), 5.0)
        # Every readiness source used below must be newer than the completed reset.
        return time.monotonic_ns()

    def _teleported_sensors_ready(self, query: Any, teleport_ns: int) -> bool:
        odom = self.latest_odom
        if odom is None or self.odom_mono_ns <= teleport_ns:
            return False
        pose = odom.pose.pose
        position_error = math.hypot(
            float(pose.position.x) - float(query.start[0]),
            float(pose.position.y) - float(query.start[1]),
        )
        yaw_error = _angle_error(_yaw(pose.orientation), float(query.start[2]))
        if position_error > 0.15 or yaw_error > 0.10 or not self._is_stopped():
            self.teleported_odom_ready_ns = 0
            return False
        if self.teleported_odom_ready_ns == 0:
            self.teleported_odom_ready_ns = time.monotonic_ns()
        return self.scan_mono_ns > self.teleported_odom_ready_ns

    def _localization_ready(self, query: Any, reset_ns: int) -> bool:
        now_ns = time.monotonic_ns()
        odom = self.latest_odom
        localization = self.latest_localization
        tf_ready = self.tf_buffer.can_transform(
            "map", "jackal/base_link", Time(), timeout=Duration(seconds=0.0),
        )
        odom_fresh = odom is not None and self.odom_mono_ns > reset_ns
        loc_fresh = localization is not None and self.localization_mono_ns > reset_ns
        scan_fresh = self.scan_mono_ns > reset_ns
        global_fresh = self.global_costmap_mono_ns > reset_ns
        local_fresh = self.local_costmap_mono_ns > reset_ns
        pose_ready = False
        if odom is not None and localization is not None:
            pose = odom.pose.pose
            position_error = math.hypot(
                float(pose.position.x) - float(query.start[0]),
                float(pose.position.y) - float(query.start[1]),
            )
            yaw_error = _angle_error(_yaw(pose.orientation), float(query.start[2]))
            covariance = localization.pose.covariance
            pose_ready = (
                position_error <= 0.15 and yaw_error <= 0.10 and
                0.0 <= covariance[0] <= 0.01 and 0.0 <= covariance[7] <= 0.01 and
                0.0 <= covariance[35] <= 0.01
            )
        stopped = self._is_stopped()
        ready = all((
            tf_ready, odom_fresh, loc_fresh, scan_fresh, global_fresh,
            local_fresh, pose_ready, stopped,
        ))
        self.health.write(
            wall_time_ns=time.time_ns(), query_id=query.query_id,
            all_lifecycle_active=True, inactive_nodes="", tf_map_to_base=tf_ready,
            localization_fresh=loc_fresh, global_costmap_fresh=global_fresh,
            local_costmap_fresh=local_fresh, odom_fresh=odom_fresh,
            scan_fresh=scan_fresh, pose_ready=pose_ready, stopped=stopped,
        )
        return ready

    def _plan_identity(self, query_id: str) -> bool:
        if self.latest_plan is None:
            return False
        expected = self.path_records[query_id]["points"]
        actual = self.latest_plan.poses
        if len(actual) != len(expected):
            return False
        for pose, point in zip(actual, expected):
            if max(
                abs(float(pose.pose.position.x) - point[0]),
                abs(float(pose.pose.position.y) - point[1]),
                _angle_error(_yaw(pose.pose.orientation), point[2]),
            ) > 1.0e-8:
                return False
        return True

    @staticmethod
    def _percentile(values: list[float], percentile: float) -> float:
        if not values:
            return 0.0
        ordered = sorted(values)
        rank = (len(ordered) - 1) * percentile / 100.0
        lower = math.floor(rank)
        upper = math.ceil(rank)
        if lower == upper:
            return ordered[lower]
        return ordered[lower] * (upper - rank) + ordered[upper] * (rank - lower)

    def _halt(self, code: str, detail: str) -> None:
        previous = self.machine.state
        self.machine.halt(code)
        self._event(previous, code)
        self.failures.write(
            wall_time_ns=time.time_ns(), sequence=self.machine.query_index + 1,
            query_id=self.machine.query_id, state=self.machine.state.value,
            failure_code=code, failure_detail=detail,
        )
        self._force_stop(True)

    def run(self) -> bool:
        self._force_stop(True)
        self._publish_state()
        if not self._wait(self._stack_active, self.stack_timeout_s):
            self._halt("STACK_NOT_ACTIVE", "lifecycle/TF/odom/action readiness timeout")
            return False
        previous = self.machine.state
        self.machine.stack_active()
        self._event(previous, "STACK_ACTIVE")

        while rclpy.ok() and self.machine.state not in (
            MissionState.COMPLETE, MissionState.HALT_CURRENT_QUERY,
        ):
            query = self.query_by_id[self.machine.query_id]
            sequence = self.machine.query_index + 1
            try:
                previous = self.machine.state
                reset_ns = self._set_initial_pose(query)
                self.machine.initial_pose_set()
                self._event(previous, "INITIAL_POSE_AND_TELEPORT_SET")
                if not self._wait(
                    lambda: self._localization_ready(query, reset_ns),
                    self.localization_timeout_s,
                ):
                    raise TimeoutError("localization/TF/costmap/stop recovery timeout")
                previous = self.machine.state
                self.machine.localization_ready()
                self._event(previous, "LOCALIZATION_READY")

                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = "map"
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x = float(query.goal[0])
                goal.pose.pose.position.y = float(query.goal[1])
                goal.pose.pose.orientation.z = math.sin(float(query.goal[2]) / 2.0)
                goal.pose.pose.orientation.w = math.cos(float(query.goal[2]) / 2.0)
                goal.behavior_tree = self.behavior_tree
                self._force_stop(False)
                sent = self.action.send_goal_async(goal)
                if not self._wait(sent.done, 10.0):
                    raise TimeoutError("NavigateToPose goal send timeout")
                handle = sent.result()
                if handle is None or not handle.accepted:
                    raise RuntimeError("NavigateToPose goal rejected")
                self.current_goal_handle = handle
                uuid = bytes(handle.goal_id.uuid).hex()
                previous = self.machine.state
                self.machine.goal_sent(uuid)
                self._event(previous, "GOAL_ACCEPTED")
                started_ns = time.monotonic_ns()
                self.active_metrics = True
                self.motion_evidence = MotionEvidence()
                self.footprint_monitor.previous = None
                self.footprint_monitor.minimum_clearance_lower_bound = math.inf
                self.runtime_failure = ""
                self.guard_samples = 0
                self.teb_samples = 0
                self.last_metric_position = None
                self.distance_traveled = 0.0
                self.tracking_errors = []
                self.odom_sample_index = 0
                self.teb_command_times = []
                self.teb_zero_commands = 0
                self.teb_feasibility_stops = 0
                self.teb_collision_stops = 0
                result_future = handle.get_result_async()
                if not self._wait(lambda: result_future.done() or bool(self.runtime_failure), self.action_timeout_s):
                    cancel = handle.cancel_goal_async()
                    self._wait(cancel.done, 5.0)
                    self.active_metrics = False
                    previous = self.machine.state
                    self.machine.action_result(False, "NAV2_TIMEOUT")
                    self._event(previous, "ACTION_TIMEOUT")
                    raise TimeoutError("NavigateToPose result timeout")
                if self.runtime_failure:
                    self._halt(self.runtime_failure, "runtime evidence rejected current query")
                    raise RuntimeError(self.runtime_failure)
                wrapped = result_future.result()
                duration_s = (time.monotonic_ns() - started_ns) / 1.0e9
                succeeded = wrapped is not None and wrapped.status == GoalStatus.STATUS_SUCCEEDED
                previous = self.machine.state
                self.machine.action_result(succeeded, f"NAV2_STATUS_{getattr(wrapped, 'status', -1)}")
                self._event(previous, "ACTION_RESULT")
                if not succeeded:
                    raise RuntimeError(self.machine.failure_code)

                self._force_stop(True)
                stop_pass = self._wait(self._is_stopped, 10.0)
                self.active_metrics = False
                odom = self.latest_odom
                if odom is None:
                    raise RuntimeError("missing final odometry")
                final_pose = odom.pose.pose
                position_error = math.hypot(
                    float(final_pose.position.x) - float(query.goal[0]),
                    float(final_pose.position.y) - float(query.goal[1]),
                )
                yaw_error = _angle_error(_yaw(final_pose.orientation), float(query.goal[2]))
                position_pass = position_error <= GOAL_POSITION_TOLERANCE_M
                yaw_pass = yaw_error <= GOAL_YAW_TOLERANCE_RAD
                identity_pass = self._plan_identity(query.query_id)
                canonical_pass = (
                    self.path_records[query.query_id]["final_audit_passed"] == "true" and identity_pass and
                    not self.runtime_failure and self.motion_evidence.samples > 1 and
                    self.guard_samples > 0 and self.teb_samples > 0 and
                    self.teb_feasibility_stops == 0 and self.teb_collision_stops == 0
                )
                if self.online:
                    canonical_pass = canonical_pass and bool(
                        self.online_audit and self.online_audit.get("query_id") == query.query_id and
                        self.online_audit.get("canonical_audit", {}).get("final_valid_success"))
                acceptance = Acceptance(True, position_pass, yaw_pass, stop_pass, canonical_pass)
                previous = self.machine.state
                self.machine.verify(acceptance)
                self._event(previous, "FINAL_ACCEPTANCE_AUDIT")
                self.goal_results.write(
                    sequence=sequence, query_id=query.query_id, goal_uuid=uuid,
                    action_status=wrapped.status, nav2_succeeded=True,
                    position_error_m=position_error, yaw_error_rad=yaw_error,
                    position_tolerance_m=GOAL_POSITION_TOLERANCE_M,
                    yaw_tolerance_rad=GOAL_YAW_TOLERANCE_RAD,
                    position_pass=position_pass, yaw_pass=yaw_pass, stopped_pass=stop_pass,
                    plan_identity_pass=identity_pass,
                    canonical_final_audit_pass=canonical_pass, accepted=acceptance.all_passed,
                )
                record = self.path_records[query.query_id]
                self.runs.write(
                    sequence=sequence, query_id=query.query_id, goal_uuid=uuid,
                    status="SUCCEEDED_ACCEPTED" if acceptance.all_passed else "AUDIT_FAILED",
                    duration_s=duration_s, distance_traveled_m=self.distance_traveled,
                    final_position_error_m=position_error, final_yaw_error_rad=yaw_error,
                    canonical_path_hash=record["canonical_path_hash"],
                    path_file_sha256=record["path_sha256"], failure_code=self.machine.failure_code,
                )
                periods_ms = [
                    (second - first) / 1.0e6
                    for first, second in zip(self.teb_command_times, self.teb_command_times[1:])
                ]
                control_frequency = (
                    (len(self.teb_command_times) - 1) /
                    ((self.teb_command_times[-1] - self.teb_command_times[0]) / 1.0e9)
                    if len(self.teb_command_times) >= 2 and self.teb_command_times[-1] > self.teb_command_times[0]
                    else 0.0
                )
                self.metrics.write(
                    sequence=sequence, query_id=query.query_id, arrival_time_s=duration_s,
                    distance_traveled_m=self.distance_traveled,
                    tracking_error_mean_m=statistics.fmean(self.tracking_errors) if self.tracking_errors else 0.0,
                    tracking_error_p95_m=self._percentile(self.tracking_errors, 95),
                    tracking_error_max_m=max(self.tracking_errors, default=0.0),
                    teb_command_count=len(self.teb_command_times),
                    teb_control_frequency_hz=control_frequency,
                    teb_period_p95_ms=self._percentile(periods_ms, 95),
                    teb_zero_command_count=self.teb_zero_commands,
                    reverse_distance_m=self.motion_evidence.reverse_distance,
                    rotate_in_place_count=self.motion_evidence.rotate_samples,
                    measured_collision_samples=self.motion_evidence.collision_samples,
                    measured_curvature_violations=self.motion_evidence.curvature_violations,
                    measured_accel_violations=self.motion_evidence.accel_violations,
                    minimum_footprint_clearance_lower_bound_m=self.footprint_monitor.minimum_clearance_lower_bound,
                )
                self.teb_metrics.write(
                    sequence=sequence, query_id=query.query_id,
                    teb_plugin="nav2_teb_controller::TEBController",
                    compute_command_count=len(self.teb_command_times),
                    control_frequency_hz=control_frequency,
                    period_p50_ms=self._percentile(periods_ms, 50),
                    period_p95_ms=self._percentile(periods_ms, 95),
                    period_p99_ms=self._percentile(periods_ms, 99),
                    zero_command_count=self.teb_zero_commands,
                    feasibility_stop_events=self.teb_feasibility_stops,
                    collision_stop_events=self.teb_collision_stops,
                    metric_source="controller_server_teb_trace_and_/cmd_vel_nav",
                )
                if not acceptance.all_passed:
                    raise RuntimeError(self.machine.failure_code)
                previous = self.machine.state
                self.machine.advance()
                self._event(previous, "ADVANCE_AFTER_ALL_GATES")
            except Exception as error:
                self.active_metrics = False
                self._force_stop(True)
                if self.current_goal_handle is not None:
                    cancel = self.current_goal_handle.cancel_goal_async()
                    self._wait(cancel.done, 5.0)
                if self.machine.state != MissionState.HALT_CURRENT_QUERY:
                    self._halt(type(error).__name__.upper(), str(error))
                else:
                    self.failures.write(
                        wall_time_ns=time.time_ns(), sequence=sequence,
                        query_id=query.query_id, state=self.machine.state.value,
                        failure_code=self.machine.failure_code, failure_detail=str(error),
                    )
                    self._force_stop(True)
                stopped = self._wait(self._is_stopped, 10.0)
                self._event(self.machine.state, "HALT_STOP_CONFIRMED" if stopped else "HALT_STOP_TIMEOUT")
                return False
        self._force_stop(True)
        return self.machine.state == MissionState.COMPLETE


def main() -> None:
    rclpy.init()
    node: Optional[SequentialMission] = None
    exit_code = 1
    try:
        node = SequentialMission()
        exit_code = 0 if node.run() else 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)
