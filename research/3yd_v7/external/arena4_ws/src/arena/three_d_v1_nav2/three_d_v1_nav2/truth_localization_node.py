"""Auditable Gazebo-truth localization adapter; map->odom is static identity."""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class TruthLocalization(Node):
    def __init__(self) -> None:
        super().__init__("three_d_v1_truth_localization")
        self.declare_parameter("odom_topic", "/model/jackal/odometry")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("position_variance", 1.0e-4)
        self.declare_parameter("yaw_variance", 4.0e-4)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
        self.audit_publisher = self.create_publisher(String, "/three_d_v1/localization_trace", 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom, 50,
        )
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initial_pose, 10)
        self.initial_pose_count = 0

    def _initial_pose(self, _message: PoseWithCovarianceStamped) -> None:
        self.initial_pose_count += 1

    def _odom(self, message: Odometry) -> None:
        output = PoseWithCovarianceStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = str(self.get_parameter("map_frame").value)
        output.pose.pose = message.pose.pose
        position_variance = float(self.get_parameter("position_variance").value)
        yaw_variance = float(self.get_parameter("yaw_variance").value)
        output.pose.covariance[0] = position_variance
        output.pose.covariance[7] = position_variance
        output.pose.covariance[35] = yaw_variance
        self.publisher.publish(output)
        audit = String()
        audit.data = json.dumps({
            "source": "gazebo_ground_truth_odometry",
            "map_to_odom": "static_identity",
            "initial_pose_count": self.initial_pose_count,
            "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec),
        }, sort_keys=True, separators=(",", ":"))
        self.audit_publisher.publish(audit)


def main() -> None:
    rclpy.init()
    node = TruthLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
