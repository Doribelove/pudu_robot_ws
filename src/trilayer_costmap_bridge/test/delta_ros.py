#!/usr/bin/env python3
"""DDS integration against the unchanged frozen P6 DeltaDecoder."""
import copy
import hashlib
import os
from pathlib import Path
import struct
import sys
import time
import uuid
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import GetCostmap
from trilayer_costmap_bridge.node import CostmapBridge
sys.path.insert(0, str(Path(__file__).parent / 'fixtures'))
from p6_r6_delta import DeltaDecoder


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID')
    rclpy.init()
    ns = '/trilayer_delta_' + uuid.uuid4().hex[:10]
    bridge = CostmapBridge(namespace=ns, parameter_overrides=[
        Parameter('enable_delta', value=True), Parameter('max_age', value=1.0)])
    client_node = Node('p6_transport_client', namespace=ns)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge); executor.add_node(client_node)
    pub = client_node.create_publisher(Costmap, 'costmap_raw', 1)
    client = client_node.create_client(GetCostmap, 'get_costmap')
    def spin_until(predicate, limit=5.):
        end = time.monotonic() + limit
        while not predicate():
            assert time.monotonic() < end, 'DDS timeout'
            executor.spin_once(timeout_sec=.01)
    def request(layer):
        req = GetCostmap.Request(); req.specs.layer = layer
        future = client.call_async(req); spin_until(future.done)
        assert future.exception() is None
        return future.result().map
    def send(msg):
        old = bridge._received
        pub.publish(msg)
        spin_until(lambda: bridge._received != old)
    msg = Costmap(); msg.header.frame_id='map'; m=msg.metadata
    m.size_x=m.size_y=256; m.resolution=.05; m.layer='master'
    m.origin.orientation.w=1.; msg.data=[254]*(256*256)
    decoder = DeltaDecoder((256,256),version=2)
    def decode(response, dec=decoder):
        grid, info = dec.decode(response.data,response.metadata.layer)
        assert grid.tobytes() == bytes(msg.data)
        proof=dec.take_current_digest(grid)
        assert proof == hashlib.sha256(bytes(msg.data)).hexdigest()
        assert dec.take_current_digest(grid) is None
        return info
    try:
        spin_until(lambda: client.service_is_ready() and pub.get_subscription_count()>0)
        send(msg)
        info=decode(request(decoder.request_layer()))
        assert info['r4_full_closed_tiles']; print('PASS P6 v2 closed full + one-use SHA proof')
        info=decode(request(decoder.request_layer()))
        assert not info['r6_readback_full'] and info['r6_readback_patch_count']==0
        msg.data[0]=0; send(msg)
        info=decode(request(decoder.request_layer()))
        assert not info['r6_readback_full'] and info['r6_readback_patch_count']==1
        print('PASS unchanged and one-tile delta')
        # Server commits even if the client loses the reply; stale base forces full.
        request(decoder.request_layer())
        info=decode(request(decoder.request_layer()))
        assert info['r6_readback_full']; print('PASS dropped response resynchronizes full')
        earlier=request(decoder.request_layer())
        later=request(decoder.request_layer()); decode(later)
        try: decode(earlier)
        except RuntimeError: pass
        else: raise AssertionError('out-of-order response accepted')
        assert decoder.grid is None and decoder.sequence==0
        assert decode(request(decoder.request_layer()))['r6_readback_full']
        print('PASS out-of-order rejection and recovery')
        bad=request(decoder.request_layer()); data=bytearray(bad.data);data[48]^=1
        try: decoder.decode(data,bad.metadata.layer)
        except RuntimeError: pass
        else: raise AssertionError('corrupt checksum accepted')
        assert decoder.grid is None
        assert decode(request(decoder.request_layer()))['r6_readback_full']
        print('PASS corruption invalidates P6 state and recovers')
        other=DeltaDecoder((256,256),version=1)
        assert decode(request(other.request_layer()),other)['r6_readback_full']
        assert decode(request(decoder.request_layer()))['r6_readback_full']
        print('PASS owner switch and v1 dense compatibility')
        for layer in ['3yd_delta_v2|bad|0|0',f'3yd_delta_v2|{decoder.token}|18446744073709551616|0']:
            response=request(layer)
            assert not response.data and response.metadata.layer=='3yd_delta_error'
        epoch=decoder.epoch
        time.sleep(1.05)
        assert not request(decoder.request_layer()).data
        decoder.invalidate(); send(msg)
        assert decode(request(decoder.request_layer()))['r6_readback_full']
        assert decoder.epoch!=epoch
        print('PASS stream expiry resets epoch and returns full on recovery')
        bad=copy.deepcopy(msg);bad.data=[0]; old=bridge._received;pub.publish(bad)
        spin_until(lambda: bridge._received is None)
        assert not request(decoder.request_layer()).data
        decoder.invalidate();send(msg)
        assert decode(request(decoder.request_layer()))['r6_readback_full']
        print('PASS malformed upstream resets transport')
        epoch=decoder.epoch
        time.sleep(1.05); send(msg)
        assert decode(request(decoder.request_layer()))['r6_readback_full']
        assert decoder.epoch!=epoch
        print('PASS gap detected on next upstream message without service poll')
        epoch=decoder.epoch
        executor.remove_node(bridge); bridge.destroy_node()
        bridge = CostmapBridge(namespace=ns, parameter_overrides=[
            Parameter('enable_delta', value=True), Parameter('max_age', value=1.0)])
        executor.add_node(bridge)
        deadline=time.monotonic()+5.
        while bridge._snapshot is None:
            assert time.monotonic()<deadline, 'restart discovery timeout'
            pub.publish(msg);executor.spin_once(timeout_sec=.1)
        assert decode(request(decoder.request_layer()))['r6_readback_full']
        assert decoder.epoch!=epoch
        print('PASS ROS service node restart resynchronizes P6 decoder')
        print('ALL P6 DELTA DDS TESTS PASSED')
    finally:
        executor.remove_node(bridge);executor.remove_node(client_node)
        bridge.destroy_node();client_node.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
