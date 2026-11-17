# SPDX-License-Identifier: Apache-2.0
"""Validated snapshots with optional original P6 v1/v2 delta transport."""
import copy
import math
import re
import secrets
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import GetCostmap


def validate_map(msg):
    meta = msg.metadata
    if meta.size_x <= 0 or meta.size_y <= 0:
        raise ValueError('costmap dimensions must be positive')
    if len(msg.data) != meta.size_x * meta.size_y:
        raise ValueError('costmap dimensions do not match data length')
    if not math.isfinite(meta.resolution) or meta.resolution <= 0:
        raise ValueError('resolution must be finite and positive')
    if not msg.header.frame_id:
        raise ValueError('frame_id must be nonempty')
    pose = meta.origin
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('origin must be finite')
    if abs(sum(v * v for v in values[3:]) - 1.0) > 1e-3:
        raise ValueError('origin quaternion must be normalized')
    if meta.layer.startswith(('3yd_delta_v1', '3yd_delta_v2')):
        raise ValueError('delta payload is not a full costmap')


class CostmapBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('trilayer_costmap_bridge', **kwargs)
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('topic', 'costmap_raw', descriptor)
        self.declare_parameter('service', 'get_costmap', descriptor)
        self.declare_parameter('max_age', 1.0, descriptor)
        self.declare_parameter('enable_delta', False, descriptor)
        self._max_age = float(self.get_parameter('max_age').value)
        if not math.isfinite(self._max_age) or self._max_age <= 0:
            raise ValueError('max_age must be finite and positive')
        self._delta_enabled = bool(self.get_parameter('enable_delta').value)
        self._codec = None
        self._identity = 1
        self._geometry = None
        if self._delta_enabled:
            from trilayer_delta_codec import DeltaCodec
            self._codec_factory = DeltaCodec
            self._codec = DeltaCodec(secrets.randbits(64) or 1)
        self._lock = threading.Lock()
        self._snapshot = None
        self._received = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self._subscription = self.create_subscription(
            Costmap, self.get_parameter('topic').value, self._on_map, qos)
        self._service = self.create_service(
            GetCostmap, self.get_parameter('service').value, self._get_map)

    def _invalidate(self):
        self._snapshot = None
        self._received = None
        self._geometry = None
        if self._codec is not None:
            self._codec.close()
            self._codec = self._codec_factory(secrets.randbits(64) or 1)

    def destroy_node(self):
        with self._lock:
            if self._codec is not None:
                self._codec.close()
                self._codec = None
        return super().destroy_node()

    def _on_map(self, msg):
        # Receiving invalid data invalidates the previous snapshot immediately.
        with self._lock:
            if self._received is not None and time.monotonic() - self._received > self._max_age:
                self._invalidate()
            self._snapshot = None
            self._received = None
            try:
                validate_map(msg)
                m = msg.metadata
                o = m.origin
                geometry = (msg.header.frame_id, m.size_x, m.size_y, m.resolution,
                            o.position.x, o.position.y, o.position.z,
                            o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)
                if geometry != self._geometry:
                    self._identity += 1
                    self._geometry = geometry
                self._snapshot = copy.deepcopy(msg)
                self._received = time.monotonic()
            except ValueError as exc:
                self._invalidate()
                self.get_logger().warning('Rejected costmap: ' + str(exc))

    def _get_map(self, request, response):
        # The ROS service has no error field. Empty map explicitly means failure.
        response.map = Costmap()
        with self._lock:
            if self._snapshot is None:
                self.get_logger().warning('No valid costmap snapshot')
                return response
            if time.monotonic() - self._received > self._max_age:
                self._invalidate()
                self.get_logger().warning('Costmap snapshot expired')
                return response
            layer = request.specs.layer
            if layer.startswith(('3yd_delta_v1', '3yd_delta_v2')):
                if not self._delta_enabled:
                    self.get_logger().warning('Delta transport is disabled')
                    return response
                try:
                    match = re.fullmatch(r'3yd_delta_v([12])\|([0-9a-f]{32})\|([0-9]{1,20})\|([0-9]{1,20})', layer)
                    if match is None:
                        raise ValueError('invalid delta request')
                    version, token, epoch_text, base_text = match.groups()
                    epoch, base = int(epoch_text), int(base_text)
                    if max(epoch, base) >= 2**64:
                        raise ValueError('delta sequence overflow')
                    m = self._snapshot.metadata
                    packet = self._codec.encode(bytes(self._snapshot.data), m.size_x,
                                                m.size_y, self._identity, token, epoch, base,
                                                sparse_full=(version == '2'))
                    response.map = copy.deepcopy(self._snapshot)
                    response.map.metadata.layer = f'3yd_delta_v{version}|{token}'
                    response.map.data = packet
                except (ValueError, RuntimeError) as exc:
                    response.map.metadata.layer = '3yd_delta_error'
                    self.get_logger().warning('Rejected delta request: ' + str(exc))
                return response
            if layer and layer != self._snapshot.metadata.layer:
                self.get_logger().warning('Unsupported layer/protocol: ' + layer)
                return response
            response.map = copy.deepcopy(self._snapshot)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CostmapBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
