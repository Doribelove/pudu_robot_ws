#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Actual DDS topic/service integration test, intentionally no navigation nodes."""
import copy
import math
import os
import time
import uuid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import GetCostmap
from trilayer_costmap_bridge.node import CostmapBridge


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'require localhost isolation'
    assert os.environ.get('ROS_DOMAIN_ID'), 'require explicit isolated domain'
    rclpy.init()
    ns = '/trilayer_smoke_' + uuid.uuid4().hex[:10]
    bridge = CostmapBridge(namespace=ns, parameter_overrides=[Parameter('max_age', value=0.5)])
    client_node = Node('test_client', namespace=ns)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge); executor.add_node(client_node)
    pub = client_node.create_publisher(Costmap, 'costmap_raw', 1)
    client = client_node.create_client(GetCostmap, 'get_costmap')
    def spin_until(predicate, limit=5):
        end = time.monotonic() + limit
        while not predicate():
            assert time.monotonic() < end, 'timed out'
            executor.spin_once(timeout_sec=0.01)
    def call(layer=''):
        req = GetCostmap.Request(); req.specs.layer = layer
        f = client.call_async(req); spin_until(f.done)
        assert f.exception() is None, f.exception()
        return f.result().map
    def send(msg):
        old = bridge._received
        pub.publish(msg)
        # Observe the callback's state only to synchronize, not to retrieve data.
        spin_until(lambda: bridge._received != old or (old is None and bridge._snapshot is not None))
    def invalid(msg):
        pub.publish(msg); spin_until(lambda: bridge._snapshot is None)
        assert not call().data
    good = Costmap(); good.header.frame_id = 'map'
    good.metadata.size_x = 2; good.metadata.size_y = 2
    good.metadata.resolution = 0.1; good.metadata.origin.orientation.w = 1.0
    good.metadata.layer = 'master'; good.data = [0, 1, 253, 254]
    try:
        spin_until(lambda: client.service_is_ready() and pub.get_subscription_count() > 0)
        assert not call().data; print('PASS no map')
        send(good); result = call(); assert list(result.data) == list(good.data)
        result.data[0] = 99; assert call().data[0] == 0
        # Additionally check callback ownership, without bypassing DDS tests.
        source = copy.deepcopy(good); bridge._on_map(source); source.data[0] = 77
        assert call().data[0] == 0; print('PASS valid full snapshot and deep copies')
        for layer in ('3yd_delta_v1|0', '3yd_delta_v2|0', 'unknown'):
            assert not call(layer).data
        assert call('master').data; print('PASS delta and unknown layer rejection')
        for mode in ('length', 'dimensions', 'resolution', 'nan', 'pose'):
            send(good); bad = copy.deepcopy(good)
            if mode == 'length': bad.data = [0]
            if mode == 'dimensions': bad.metadata.size_x = 0
            if mode == 'resolution': bad.metadata.resolution = 0.0
            if mode == 'nan': bad.metadata.resolution = float('nan')
            if mode == 'pose': bad.metadata.origin.orientation.w = 0.0
            invalid(bad)
        print('PASS malformed input invalidates previous valid map')
        send(good); time.sleep(0.6); assert not call().data; assert bridge._snapshot is None
        print('PASS expiry discards old snapshot')
        send(good); assert call().data; print('PASS recovery after new valid map')
        print('ALL ROS SMOKE TESTS PASSED')
    finally:
        executor.shutdown(); bridge.destroy_node(); client_node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
