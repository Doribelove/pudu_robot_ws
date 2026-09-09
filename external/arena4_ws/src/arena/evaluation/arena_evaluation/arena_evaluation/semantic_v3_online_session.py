"""ROS/Nav2 boundary for the independent 2A-V3 state-lattice planner.

The pinned Smac plugin exposes only a two-dimensional master costmap.  This
session therefore uses Nav2 as the authoritative effective-cost producer and
content-ACK server, then publishes the independently planned SE(2) result as a
real ``nav_msgs/Path``.  No historical path is accepted as planner input.
"""
from __future__ import annotations

import hashlib
import math
import struct
import time
from typing import Any, Mapping, Sequence

import numpy as np

from .semantic_rasterizer import grid_hash
from .semantic_smac_session_r2 import ExactSemanticSmacSessionR2


class VerifiedV3PlannerSession(ExactSemanticSmacSessionR2):
    """Exact master readback plus a fail-closed ``nav_msgs/Path`` interface."""

    PUBLICATION_VERSION = "2A-V3-r12-exact-effective-master-v1"
    PATH_TOPIC = "/pln02_2a_v3/global_path"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._path_publisher = None
        self._path_subscription = None
        self._path_type = None
        self._pose_type = None
        self._serialize_message = None
        self._last_path_echo: bytes | None = None
        self._path_echoes: list[tuple[str, str]] = []

    def start(self) -> None:
        super().start()
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        from rclpy.serialization import serialize_message

        if self.client is None:
            raise RuntimeError("V3 ROS path interface has no active client node")
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self._path_type = Path
        self._pose_type = PoseStamped
        self._serialize_message = serialize_message
        self._path_publisher = self.client.node.create_publisher(Path, self.PATH_TOPIC, qos)

        def receive(message: Any) -> None:
            serialized = bytes(serialize_message(message))
            self._last_path_echo = serialized
            self._path_echoes.append((
                self._path_content_hash(message),
                hashlib.sha256(serialized).hexdigest(),
            ))

        self._path_subscription = self.client.node.create_subscription(
            Path, self.PATH_TOPIC, receive, qos,
        )

    def close(self) -> None:
        if self.client is not None:
            if self._path_subscription is not None:
                self.client.node.destroy_subscription(self._path_subscription)
            if self._path_publisher is not None:
                self.client.node.destroy_publisher(self._path_publisher)
        self._path_subscription = None
        self._path_publisher = None
        super().close()

    @staticmethod
    def _path_content_hash(message: Any) -> str:
        """Hash only defined Path fields, excluding CDR padding bytes."""
        frame = str(message.header.frame_id).encode("utf-8")
        stamp = message.header.stamp
        payload = bytearray(struct.pack(
            "<I", len(frame),
        ))
        payload.extend(frame)
        payload.extend(struct.pack(
            "<qII", int(stamp.sec), int(stamp.nanosec), len(message.poses),
        ))
        for item in message.poses:
            pose = item.pose
            payload.extend(struct.pack(
                "<7d", float(pose.position.x), float(pose.position.y),
                float(pose.position.z), float(pose.orientation.x),
                float(pose.orientation.y), float(pose.orientation.z),
                float(pose.orientation.w),
            ))
        return hashlib.sha256(payload).hexdigest()

    def verified_master_snapshot(
        self, ack: Mapping[str, Any], *, timeout_s: float = 1.0,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Return top-row-first master bytes only after a complete exact ACK."""
        semantic = self._semantic_costmap
        if semantic is None:
            raise RuntimeError("semantic costmap is not bound")
        required = {
            "costmap_update_acknowledged": True,
            "costmap_ack_semantics": "exact_effective_master",
            "costmap_ack_hard_mismatch_cells": 0,
            "costmap_ack_soft_exact_mismatch_cells": 0,
            "costmap_ack_stale_roi_cells": 0,
            "costmap_ack_hash_mismatch": 0,
            "costmap_ack_sequence_mismatch": 0,
        }
        for key, expected in required.items():
            if ack.get(key) != expected:
                raise RuntimeError(f"V3 effective-content ACK failed for {key}")
        deadline = time.monotonic() + max(0.05, float(timeout_s))
        server_order, timestamp_ns = self._server_costmap_snapshot(deadline)
        expected_server = np.ascontiguousarray(
            np.flipud(np.asarray(semantic.expected_master_cost, dtype=np.uint8)),
        )
        mismatch = int(np.count_nonzero(server_order != expected_server))
        server_hash = grid_hash(server_order)
        expected_hash = grid_hash(expected_server)
        if (
            mismatch != 0
            or server_hash != expected_hash
            or server_hash != str(ack.get("server_costmap_content_hash", ""))
            or expected_hash != str(ack.get("semantic_expected_server_content_hash", ""))
            or int(ack.get("semantic_publication_sequence", -1))
            != int(self._semantic_publication_sequence)
        ):
            self._costmap_state_trusted = False
            self._force_full_next_update = True
            raise RuntimeError("V3 post-ACK master readback changed or is not exact")
        top_order = np.ascontiguousarray(np.flipud(server_order), dtype=np.uint8)
        if grid_hash(top_order) != str(semantic.expected_master_hash):
            raise RuntimeError("V3 top-row master binding hash mismatch")
        return top_order, {
            "verified_master_hash": grid_hash(top_order),
            "verified_server_content_hash": server_hash,
            "verified_master_mismatch_cells": mismatch,
            "verified_server_update_time_ns": int(timestamp_ns),
            "verified_publication_sequence": int(self._semantic_publication_sequence),
        }

    def publish_verified_path(
        self, points: Sequence[Mapping[str, Any]], *, query_id: str,
        timeout_s: float = 0.75,
    ) -> dict[str, Any]:
        """Publish and locally echo-verify an exact serialized path message."""
        if (
            self.client is None or self._path_publisher is None
            or self._path_type is None or self._pose_type is None
            or self._serialize_message is None
        ):
            raise RuntimeError("V3 ROS path interface is not active")
        if not points:
            raise RuntimeError("refusing to publish an empty V3 path")
        message = self._path_type()
        message.header.frame_id = "map"
        message.header.stamp = self.client.node.get_clock().now().to_msg()
        for point in points:
            pose = self._pose_type()
            pose.header = message.header
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            yaw = float(point["yaw"])
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)
        payload = bytes(self._serialize_message(message))
        expected_serialized_hash = hashlib.sha256(payload).hexdigest()
        expected_content_hash = self._path_content_hash(message)
        self._last_path_echo = None
        self._path_echoes.clear()
        discovery_deadline = time.monotonic() + min(0.5, max(0.05, timeout_s))
        while self._path_publisher.get_subscription_count() < 1 and time.monotonic() < discovery_deadline:
            self.client.executor.spin_once(timeout_sec=0.01)
        if self._path_publisher.get_subscription_count() < 1:
            raise RuntimeError("V3 nav_msgs/Path echo subscriber was not discovered")
        self._path_publisher.publish(message)
        deadline = time.monotonic() + max(0.05, float(timeout_s))
        while (
            not any(item[0] == expected_content_hash for item in self._path_echoes)
            and time.monotonic() < deadline
        ):
            self.client.executor.spin_once(timeout_sec=0.01)
        received = next(
            (item for item in self._path_echoes if item[0] == expected_content_hash),
            None,
        )
        if received is None:
            raise RuntimeError("V3 nav_msgs/Path canonical-content echo mismatch")
        received_content_hash, received_serialized_hash = received
        return {
            "query_id": str(query_id),
            "topic": self.PATH_TOPIC,
            "frame_id": "map",
            "pose_count": len(message.poses),
            "published_content_sha256": expected_content_hash,
            "echo_content_sha256": received_content_hash,
            "published_serialized_sha256": expected_serialized_hash,
            "echo_serialized_sha256": received_serialized_hash,
            "serialized_transport_bytes_equal": (
                received_serialized_hash == expected_serialized_hash
            ),
            "exact_content_echo_verified": True,
            "exact_echo_verified": True,
        }


__all__ = ["VerifiedV3PlannerSession"]
