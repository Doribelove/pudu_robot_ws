"""ROS node applying the hard command contract after controller_server."""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

from .guard_core import CommandGuard, GuardLimits


FIELDS = (
    "wall_time_ns", "ros_time_ns", "query_id", "mission_state", "raw_v", "raw_w",
    "out_v", "out_w", "dt_s", "output_linear_accel", "output_angular_accel",
    "curvature_1pm", "events", "output_reverse_distance_m", "raw_reverse_distance_m",
)


def stop_target_velocity(force_stop: bool, dynamic_stop: bool, hold_speed: float) -> float:
    """Return a forward hold only for a recoverable dynamic planning stop."""
    return hold_speed if dynamic_stop and not force_stop else 0.0


class CommandGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("three_d_v1_cmd_vel_guard")
        for name, default in (
            ("input_topic", "/cmd_vel_teb_raw"), ("output_topic", "/cmd_vel"),
            ("audit_csv", ""), ("max_linear_speed", GuardLimits().max_linear_speed), ("max_angular_speed", 1.50),
            ("max_linear_accel", GuardLimits().max_linear_accel), ("max_angular_accel", 2.00),
            ("maximum_curvature", 2.50), ("near_zero_linear_speed", 1.0e-3),
            ("dynamic_stop_hold_speed", 1.0e-3),
            ("watchdog_timeout_s", 0.25),
        ):
            self.declare_parameter(name, default)
        limits = GuardLimits(
            max_linear_speed=float(self.get_parameter("max_linear_speed").value),
            max_angular_speed=float(self.get_parameter("max_angular_speed").value),
            max_linear_accel=float(self.get_parameter("max_linear_accel").value),
            max_angular_accel=float(self.get_parameter("max_angular_accel").value),
            maximum_curvature=float(self.get_parameter("maximum_curvature").value),
            near_zero_linear_speed=float(self.get_parameter("near_zero_linear_speed").value),
        )
        self.guard = CommandGuard(limits)
        self.measured_angular_speed = 0.0
        self.measured_velocity_ns = 0
        self.create_subscription(Odometry, "/model/jackal/odometry", self._odometry, 20)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10,
        )
        self.diagnostic_publisher = self.create_publisher(String, "/three_d_v1/cmd_guard_trace", 10)
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._command, 20,
        )
        self.create_subscription(Bool, "/three_d_v1/force_stop", self._force_stop, 10)
        self.create_subscription(Bool, "/three_d_v1/dynamic_stop", self._dynamic_stop,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/three_d_v1/mission_state", self._state, 10)
        self.last_input_ns = time.monotonic_ns()
        self.last_apply_ns = self.last_input_ns
        self.last_watchdog_publish_ns = 0
        self.force_stop = True
        self.dynamic_stop = False
        self.query_id = ""
        self.mission_state = "BOOT"
        self.watchdog_timeout_s = float(self.get_parameter("watchdog_timeout_s").value)
        self.dynamic_stop_hold_speed = float(self.get_parameter("dynamic_stop_hold_speed").value)
        if not 0.0 < self.dynamic_stop_hold_speed <= 2.0e-3:
            raise ValueError("dynamic_stop_hold_speed must be in (0, 0.002] m/s")
        self.create_timer(0.05, self._watchdog)

        audit = str(self.get_parameter("audit_csv").value)
        self.audit_stream = None
        self.audit_writer = None
        if audit:
            path = Path(audit).resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.audit_stream = path.open("a", newline="", encoding="utf-8", buffering=1)
            self.audit_writer = csv.DictWriter(self.audit_stream, fieldnames=FIELDS)
            if path.stat().st_size == 0:
                self.audit_writer.writeheader()
        self.get_logger().info(
            "Command guard active: no reverse, no rotate-in-place, |curvature|<=2.50 1/m"
        )

    def destroy_node(self):  # type: ignore[no-untyped-def]
        self._emit(0.0, 0.0, "NODE_SHUTDOWN_HARD_STOP", immediate=True)
        if self.audit_stream is not None:
            self.audit_stream.close()
        return super().destroy_node()

    def _state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.query_id = str(payload.get("query_id", ""))
            self.mission_state = str(payload.get("state", ""))
        except (TypeError, ValueError):
            self.mission_state = "INVALID_STATE_MESSAGE"

    def _odometry(self, message: Odometry) -> None:
        self.measured_angular_speed = float(message.twist.twist.angular.z)
        self.measured_velocity_ns = time.monotonic_ns()

    def _force_stop(self, message: Bool) -> None:
        was_stopped = self.force_stop
        self.force_stop = bool(message.data)
        if was_stopped and not self.force_stop:
            self.last_input_ns = time.monotonic_ns()
        if self.force_stop:
            self._emit(0.0, 0.0, "MISSION_FORCE_STOP")

    def _dynamic_stop(self, message: Bool) -> None:
        self.dynamic_stop = bool(message.data)
        if self.dynamic_stop:
            self._emit(0.0, 0.0, "DYNAMIC_REPLAN_STOP")
        else:
            self.last_input_ns = time.monotonic_ns()

    def _command(self, message: Twist) -> None:
        now_ns = time.monotonic_ns()
        self.last_input_ns = now_ns
        raw_v = float(message.linear.x)
        raw_w = float(message.angular.z)
        event = "MISSION_FORCE_STOP" if self.force_stop else ("DYNAMIC_REPLAN_STOP" if self.dynamic_stop else "")
        self._emit(raw_v, raw_w, event)

    def _emit(self, raw_v: float, raw_w: float, extra_event: str, *, immediate=False) -> None:
        now_ns = time.monotonic_ns()
        dt = max(1.0e-9, (now_ns - self.last_apply_ns) / 1.0e9)
        self.last_apply_ns = now_ns
        previous_v, previous_w = self.guard.previous_v, self.guard.previous_w
        measured_w = self.measured_angular_speed if now_ns - self.measured_velocity_ns < 250_000_000 else 0.0
        if self.force_stop or self.dynamic_stop:
            # The Gazebo skid-steer body can rebound a few nanometres per
            # second after a moving stop. During a recoverable dynamic replan,
            # hold 1 mm/s forward (inside the frozen 2 mm/s stopped gate) so
            # the hard no-reverse contract remains physically true. Mission
            # and shutdown stops still target exact zero.
            stop_v = stop_target_velocity(
                self.force_stop, self.dynamic_stop, self.dynamic_stop_hold_speed)
            sample = self.guard.apply(stop_v, 0.0, dt, measured_w)
        else:
            sample = self.guard.apply(raw_v, raw_w, dt, measured_w)
        output = Twist()
        output.linear.x = sample.out_v
        output.angular.z = sample.out_w
        if immediate:
            output = Twist()
            self.guard.reset()
        self.publisher.publish(output)
        events = sorted(set(sample.events) | ({extra_event} if extra_event else set()))
        row = {
            "wall_time_ns": time.time_ns(), "ros_time_ns": self.get_clock().now().nanoseconds,
            "query_id": self.query_id, "mission_state": self.mission_state,
            "raw_v": raw_v, "raw_w": raw_w, "out_v": output.linear.x, "out_w": output.angular.z,
            "dt_s": sample.dt, "output_linear_accel": (output.linear.x - previous_v) / dt,
            "output_angular_accel": (output.angular.z - previous_w) / dt,
            "curvature_1pm": 0.0 if immediate else sample.curvature, "events": ";".join(events),
            "output_reverse_distance_m": self.guard.output_reverse_distance,
            "raw_reverse_distance_m": self.guard.raw_reverse_distance,
        }
        if self.audit_writer is not None:
            self.audit_writer.writerow(row)
        diagnostic = String()
        diagnostic.data = json.dumps(row, sort_keys=True, separators=(",", ":"))
        self.diagnostic_publisher.publish(diagnostic)

    def _watchdog(self) -> None:
        now_ns = time.monotonic_ns()
        if (now_ns - self.last_input_ns) / 1.0e9 < self.watchdog_timeout_s:
            return
        if now_ns - self.last_watchdog_publish_ns < 50_000_000:
            return
        self.last_watchdog_publish_ns = now_ns
        event = "MISSION_FORCE_STOP" if self.force_stop else ("DYNAMIC_REPLAN_STOP" if self.dynamic_stop else "COMMAND_WATCHDOG_STOP")
        self._emit(0.0, 0.0, event)


def main() -> None:
    rclpy.init()
    node = CommandGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
