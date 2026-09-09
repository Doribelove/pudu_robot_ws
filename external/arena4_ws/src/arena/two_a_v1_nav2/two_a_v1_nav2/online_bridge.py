"""Online service exposing frozen 2A-V1-r2 to the Nav2 planner plugin."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav_msgs.srv import GetPlan
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from arena_evaluation.semantic_query_defaults import load_query_set
from three_d_v1_nav2.contracts import (
    EXPECTED_MAP_SHA256, EXPECTED_SEMANTIC_MAP_HASH, QUERY_SET, sha256_file,
)
from .contracts import verify_2a_run_inputs
from .core import FrozenTwoAPlannerCore


def _yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class OnlineBridge(Node):
    def __init__(self) -> None:
        super().__init__("two_a_v1_r2_online_bridge")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("l3_domain_id", 221)
        self.run_dir = Path(str(self.get_parameter("run_dir").value)).resolve()
        self.output = Path(str(self.get_parameter("output_dir").value)).resolve() / "online_planner"
        verify_2a_run_inputs(self.run_dir)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Mission/guard topics intentionally remain unchanged so the tested
        # candidate7 control and acceptance chain is reused byte-for-byte.
        self.stop_pub = self.create_publisher(Bool, "/three_d_v1/dynamic_stop", qos)
        self.ready_pub = self.create_publisher(Bool, "/three_d_v1/online_ready", qos)
        self.audit_pub = self.create_publisher(String, "/three_d_v1/online_audit", 10)
        self.trace_pub = self.create_publisher(String, "/two_a_v1/global_layer_trace", 10)
        self.stop_pub.publish(Bool(data=True))
        self.motion = None
        self.stopped_since = None
        self.create_subscription(Odometry, "/model/jackal/odometry", self._odom, 20)
        queries, _, _ = load_query_set(
            QUERY_SET, actual_map_hash=EXPECTED_MAP_SHA256,
            actual_semantic_map_hash=EXPECTED_SEMANTIC_MAP_HASH,
            require_default_contract=True,
        )
        self.queries = {query.query_id: query for query in queries}
        self.current_query = ""
        self.cached = None
        self.cached_stamp = None
        self.sequence = 0
        self.core = FrozenTwoAPlannerCore(
            self.run_dir, self.output,
            l3_domain_id=int(self.get_parameter("l3_domain_id").value),
        )
        self.create_service(GetPlan, "/two_a_v1/compute_plan", self._plan)
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info("Frozen 2A-V1-r2 online bridge ready")

    def _odom(self, message: Odometry) -> None:
        now = time.monotonic()
        twist = message.twist.twist
        if abs(twist.linear.x) <= .002 and abs(twist.angular.z) <= .003:
            self.stopped_since = self.stopped_since or now
        else:
            self.stopped_since = None
        self.motion = (now, message.pose.pose)

    def _stopped_at_query_start(self, query) -> None:
        deadline = time.monotonic() + 5.0
        while self.context.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if self.motion and now - self.motion[0] < .5 and self.stopped_since is not None:
                pose = self.motion[1]
                if now - self.stopped_since >= .5:
                    if math.hypot(pose.position.x - query.start[0], pose.position.y - query.start[1]) > .5:
                        raise RuntimeError("START_NOT_FROZEN_QUERY")
                    return
            time.sleep(.02)
        raise RuntimeError("PLANNING_STOP_NOT_CONFIRMED")

    def _match_query(self, request):
        if request.start.header.frame_id != "map" or request.goal.header.frame_id != "map":
            raise RuntimeError("INVALID_FRAME")
        goal = request.goal.pose
        matches = [query for query in self.queries.values() if
                   math.hypot(goal.position.x - query.goal[0], goal.position.y - query.goal[1]) < 1e-6 and
                   abs(math.atan2(math.sin(_yaw(goal) - query.goal[2]),
                                  math.cos(_yaw(goal) - query.goal[2]))) < 1e-6]
        if len(matches) != 1:
            raise RuntimeError("GOAL_NOT_FROZEN_QUERY")
        return matches[0]

    def _plan(self, request, response):
        started = time.monotonic_ns()
        self.sequence += 1
        trace = {
            "request_sequence": self.sequence, "architecture_id": "2A-V1",
            "implementation_revision": "r2-roi-pathaudit-v1", "l2_called": False,
            "l2_backend": "not_applicable_2a",
        }
        try:
            query = self._match_query(request)
            trace["query_id"] = query.query_id
            need_plan = query.query_id != self.current_query or self.cached is None
            if need_plan:
                self.stop_pub.publish(Bool(data=True))
                self._stopped_at_query_start(query)
                result, diagnostics = self.core.plan(query)
                audit = result.path_audit
                path_file = self.output / f"{self.sequence:06d}_path.csv"
                with path_file.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(("x", "y", "yaw"))
                    writer.writerows([
                        [format(float(point[key]), ".17g") for key in ("x", "y", "yaw")]
                        for point in result.points
                    ])
                self.cached = {
                    "points": result.points, "canonical_path_hash": audit.path_hash,
                    "canonical_audit": audit.metrics, "canonical_audit_diagnostics": audit.diagnostics(),
                    "path_file": str(path_file), "path_sha256": sha256_file(path_file),
                    "l3_diagnostics": diagnostics,
                }
                self.cached_stamp = self.get_clock().now().to_msg()
                self.current_query = query.query_id
                trace.update({
                    "l1": "deterministic_graph_astar",
                    "l1_ms": diagnostics.get("l1_graph_search_ms", 0.0),
                    "route_edge_ids": diagnostics.get("topology_edge_ids", []),
                    "corridor_mask_hash": diagnostics.get("corridor_mask_hash", ""),
                    "corridor_padding_m": diagnostics.get("corridor_padding_m", 2.0),
                    "corner_corridor_padding_m": diagnostics.get("corner_corridor_padding_m", 4.0),
                    "l3": "smac_hybrid_astar", "l3_ms": diagnostics.get("l3_action_wall_ms", 0.0),
                    "l3_call_count": diagnostics.get("l3_prime_call_count", 0),
                    "angle_bins": 48, "motion_model": "DUBIN",
                    "roi_acknowledged": diagnostics.get("costmap_update_acknowledged"),
                    "roi_ack_mismatch_cells": diagnostics.get("costmap_ack_mismatch_cells", 0),
                    "roi_sequence": diagnostics.get("costmap_ack_sequence", 0),
                    "fixed_settle_cycles": 0,
                    "fallback_used": diagnostics.get("fallback_used", False),
                    "canonical_path_audit_reused": True,
                })
            else:
                trace.update({"cached_path_reused": True, "l1": "persistent_static_route_reused",
                              "l3": "canonical_path_reused", "l3_call_count": 0})
            trace.update(self.cached)
            message = NavPath()
            message.header.frame_id = "map"
            message.header.stamp = self.cached_stamp
            for point in self.cached["points"]:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = float(point["x"])
                pose.pose.position.y = float(point["y"])
                pose.pose.orientation.z = math.sin(float(point["yaw"]) / 2)
                pose.pose.orientation.w = math.cos(float(point["yaw"]) / 2)
                message.poses.append(pose)
            response.plan = message
            trace["success"] = True
            self.audit_pub.publish(String(data=json.dumps(trace, default=str)))
            self.stop_pub.publish(Bool(data=False))
        except Exception as error:
            self.stop_pub.publish(Bool(data=True))
            trace.update(success=False, failure_code=str(error))
            self.get_logger().error(str(error))
        trace["request_ms"] = (time.monotonic_ns() - started) / 1e6
        payload = json.dumps(trace, allow_nan=False, default=str)
        self.trace_pub.publish(String(data=payload))
        (self.output / f"{self.sequence:06d}.json").write_text(payload + "\n", encoding="utf-8")
        return response

    def close(self) -> None:
        self.stop_pub.publish(Bool(data=True))
        self.core.close()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = OnlineBridge()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
