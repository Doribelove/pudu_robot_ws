#!/usr/bin/env python3
"""Build a simulation reference PCD by transforming lidar scans with Gazebo truth."""

from collections import deque
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def rotate(quaternion, point):
    # Quaternion-vector rotation without a matrix dependency.
    x, y, z = point
    qx, qy, qz, qw = (
        quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


class GroundTruthMapBuilder(Node):
    def __init__(self):
        super().__init__("scurm_ground_truth_map_builder")
        self.declare_parameter("cloud_topic", "/scurm/lidar_points")
        self.declare_parameter("truth_topic", "/ground_truth/odom")
        self.declare_parameter("output_path", "/tmp/scurm_ground_truth_map.pcd")
        self.declare_parameter("duration", 5.0)
        self.declare_parameter("voxel_size", 0.08)
        self.declare_parameter("sensor_x", 0.12)
        self.declare_parameter("sensor_y", 0.0)
        self.declare_parameter("sensor_z", 0.33)

        self.output_path = Path(self.get_parameter("output_path").value)
        self.duration = float(self.get_parameter("duration").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.sensor_offset = (
            float(self.get_parameter("sensor_x").value),
            float(self.get_parameter("sensor_y").value),
            float(self.get_parameter("sensor_z").value),
        )
        self.truth_history = deque(maxlen=100)
        self.voxels = {}
        self.start_time = None
        self.finished = False

        self.create_subscription(
            Odometry, self.get_parameter("truth_topic").value,
            self.truth_callback, 50)
        self.create_subscription(
            PointCloud2, self.get_parameter("cloud_topic").value,
            self.cloud_callback, 10)
        self.create_timer(0.2, self.check_finished)

    def truth_callback(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.truth_history.append((stamp, message.pose.pose))

    def cloud_callback(self, message):
        if self.finished or not self.truth_history:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = stamp
        truth_pose = min(self.truth_history, key=lambda item: abs(item[0] - stamp))[1]
        position = truth_pose.position
        orientation = truth_pose.orientation

        for point in point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True):
            local = (
                float(point[0]) + self.sensor_offset[0],
                float(point[1]) + self.sensor_offset[1],
                float(point[2]) + self.sensor_offset[2],
            )
            rotated = rotate(orientation, local)
            world = (
                rotated[0] + position.x,
                rotated[1] + position.y,
                rotated[2] + position.z,
            )
            if not all(math.isfinite(value) for value in world):
                continue
            key = tuple(round(value / self.voxel_size) for value in world)
            self.voxels[key] = world

    def check_finished(self):
        if self.finished or self.start_time is None:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        if elapsed < self.duration:
            return
        self.write_map()
        self.finished = True
        self.get_logger().info(
            f"Wrote {len(self.voxels)} points to {self.output_path}")

    def write_map(self):
        points = sorted(self.voxels.values())
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="ascii") as stream:
            stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
            stream.write("VERSION 0.7\n")
            stream.write("FIELDS x y z intensity normal_x normal_y normal_z curvature\n")
            stream.write("SIZE 4 4 4 4 4 4 4 4\n")
            stream.write("TYPE F F F F F F F F\n")
            stream.write("COUNT 1 1 1 1 1 1 1 1\n")
            stream.write(f"WIDTH {len(points)}\nHEIGHT 1\n")
            stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            stream.write(f"POINTS {len(points)}\nDATA ascii\n")
            for x, y, z in points:
                stream.write(f"{x:.4f} {y:.4f} {z:.4f} 100 0 0 0 0\n")


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthMapBuilder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
