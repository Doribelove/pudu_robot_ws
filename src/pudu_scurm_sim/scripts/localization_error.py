#!/usr/bin/env python3
"""Compare the localized map->base pose with Gazebo's noise-free ground truth."""

import csv
import math
import os

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationError(Node):
    def __init__(self):
        super().__init__("scurm_localization_error")
        self.declare_parameter("truth_topic", "/ground_truth/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("csv_path", "auto")

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        truth_topic = self.get_parameter("truth_topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.truth = None
        self.samples = 0
        self.sum_position_squared = 0.0
        self.sum_yaw_squared = 0.0
        self.max_position = 0.0

        self.vector_pub = self.create_publisher(
            Vector3Stamped, "/scurm/localization_error/vector", 10)
        self.position_pub = self.create_publisher(
            Float64, "/scurm/localization_error/position", 10)
        self.yaw_pub = self.create_publisher(
            Float64, "/scurm/localization_error/yaw", 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(Odometry, truth_topic, self.truth_callback, 10)
        self.create_timer(1.0 / max(1.0, publish_rate), self.measure)

        csv_path = self.get_parameter("csv_path").value
        if csv_path == "auto":
            runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "/tmp")
            csv_path = os.path.join(
                runtime_dir, f"scurm-localization-error-{os.getuid()}.csv")
        self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "stamp", "error_x", "error_y", "error_z", "position_error",
            "yaw_error", "position_rmse", "yaw_rmse",
        ])
        self.get_logger().info(f"Localization error CSV: {csv_path}")

    def destroy_node(self):
        self.csv_file.close()
        return super().destroy_node()

    def truth_callback(self, message):
        self.truth = message

    def measure(self):
        if self.truth is None:
            return
        try:
            estimate = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as error:
            self.get_logger().debug(f"Waiting for localized TF: {error}")
            return

        truth_pose = self.truth.pose.pose
        translation = estimate.transform.translation
        error_x = translation.x - truth_pose.position.x
        error_y = translation.y - truth_pose.position.y
        error_z = translation.z - truth_pose.position.z
        position_error = math.sqrt(
            error_x * error_x + error_y * error_y + error_z * error_z)
        yaw_error = normalize_angle(
            yaw_from_quaternion(estimate.transform.rotation)
            - yaw_from_quaternion(truth_pose.orientation))

        self.samples += 1
        self.sum_position_squared += position_error * position_error
        self.sum_yaw_squared += yaw_error * yaw_error
        self.max_position = max(self.max_position, position_error)
        position_rmse = math.sqrt(self.sum_position_squared / self.samples)
        yaw_rmse = math.sqrt(self.sum_yaw_squared / self.samples)

        vector = Vector3Stamped()
        vector.header.stamp = self.get_clock().now().to_msg()
        vector.header.frame_id = self.map_frame
        vector.vector.x = error_x
        vector.vector.y = error_y
        vector.vector.z = error_z
        self.vector_pub.publish(vector)
        self.position_pub.publish(Float64(data=position_error))
        self.yaw_pub.publish(Float64(data=yaw_error))

        diagnostics = DiagnosticArray()
        diagnostics.header = vector.header
        status = DiagnosticStatus()
        status.name = "SCURM localization error"
        status.hardware_id = "gazebo"
        status.level = (
            DiagnosticStatus.OK if position_error < 0.20
            else DiagnosticStatus.WARN if position_error < 0.50
            else DiagnosticStatus.ERROR)
        status.message = f"position={position_error:.3f} m, yaw={yaw_error:.3f} rad"
        status.values = [
            KeyValue(key="samples", value=str(self.samples)),
            KeyValue(key="position_rmse_m", value=f"{position_rmse:.6f}"),
            KeyValue(key="yaw_rmse_rad", value=f"{yaw_rmse:.6f}"),
            KeyValue(key="max_position_error_m", value=f"{self.max_position:.6f}"),
        ]
        diagnostics.status = [status]
        self.diagnostic_pub.publish(diagnostics)

        self.csv_writer.writerow([
            f"{self.get_clock().now().nanoseconds / 1e9:.9f}",
            f"{error_x:.6f}", f"{error_y:.6f}", f"{error_z:.6f}",
            f"{position_error:.6f}", f"{yaw_error:.6f}",
            f"{position_rmse:.6f}", f"{yaw_rmse:.6f}",
        ])
        if self.samples % 10 == 0:
            self.csv_file.flush()
            self.get_logger().info(
                f"error={position_error:.3f} m/{math.degrees(abs(yaw_error)):.2f} deg, "
                f"RMSE={position_rmse:.3f} m/{math.degrees(yaw_rmse):.2f} deg")


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationError()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
