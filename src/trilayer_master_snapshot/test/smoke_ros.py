#!/usr/bin/env python3
"""Stock Costmap2DROS/StaticLayer/pluginlib integration; no robot nodes."""
import argparse
import os
from pathlib import Path
import subprocess
import time
import uuid
# Verify pluginlib's package index will resolve this fresh private fixture.
from ament_index_python.packages import get_package_prefix
if os.environ.get('TRILAYER_TEST_PREFIX'):
    expected=Path(os.environ['TRILAYER_TEST_PREFIX']).resolve()
    actual=Path(get_package_prefix('trilayer_master_snapshot')).resolve()
    assert actual==expected,(actual,expected)
    assert (actual/'lib/libtrilayer_master_snapshot.so').is_file()
    print('MASTER_PLUGIN_PREFIX',actual,flush=True)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.srv import GetCostmap
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import ChangeState


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--host',required=True);args=parser.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DOMAIN_ID')
    ns='/trilayer_master_'+uuid.uuid4().hex[:10]
    env=dict(os.environ,TRILAYER_TEST_NAMESPACE=ns,TRILAYER_TEST_SECONDS='12')
    rclpy.init();node=Node('master_smoke_client',namespace=ns)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    pub=node.create_publisher(OccupancyGrid,'input_map',qos)
    updates=node.create_publisher(OccupancyGridUpdate,'input_map_updates',10)
    client=node.create_client(GetCostmap,'trilayer_get_costmap')
    hold=node.create_client(Trigger,'hold_master')
    lifecycle=node.create_client(ChangeState,'costmap/change_state')
    observed=[]
    sub=node.create_subscription(Bool,'master_locked',lambda msg:observed.append(msg.data),10)
    def spin_until(predicate,seconds=8):
        end=time.monotonic()+seconds
        while not predicate():
            assert time.monotonic()<end,'timeout'
            assert process.poll() is None,'host exited early'
            rclpy.spin_once(node,timeout_sec=.01)
    def call(layer=''):
        req=GetCostmap.Request();req.specs.layer=layer
        future=client.call_async(req);spin_until(future.done)
        assert future.exception() is None
        return future.result().map
    def state(transition):
        req=ChangeState.Request();req.transition.id=transition
        future=lifecycle.call_async(req);spin_until(future.done)
        assert future.result().success
    log=Path('/tmp')/(ns[1:]+'.log')
    with log.open('w') as out:
        process=subprocess.Popen([args.host],env=env,stdout=out,stderr=subprocess.STDOUT)
        try:
            full=OccupancyGrid();full.header.frame_id='map';full.info.resolution=1.
            full.info.width=full.info.height=4;full.info.origin.position.x=-1.;full.info.origin.position.y=-1.
            full.info.origin.orientation.w=1.;full.data=[0]*16;full.data[0]=100
            pub.publish(full)
            spin_until(lambda:client.service_is_ready() and updates.get_subscription_count()>0)
            expected=[0]*16;expected[0]=254
            end=time.monotonic()+5
            while True:
                msg=call()
                if list(msg.data)==expected:break
                assert time.monotonic()<end,'StaticLayer master did not settle'
                rclpy.spin_once(node,timeout_sec=.03)
            assert msg.metadata.layer=='master' and msg.header.frame_id=='map'
            assert (msg.metadata.size_x,msg.metadata.size_y)==(4,4)
            assert msg.metadata.origin.position.x==-1.
            assert list(call().data)==expected
            print('PASS real plugin-loaded StaticLayer full map and non-mutating reads')
            update=OccupancyGridUpdate();update.header.frame_id='map';update.x=1;update.y=1
            update.width=update.height=1;update.data=[100];updates.publish(update);expected[5]=254
            end=time.monotonic()+3
            while list(call().data)!=expected:
                assert time.monotonic()<end,'StaticLayer update not reflected'
                rclpy.spin_once(node,timeout_sec=.02)
            print('PASS real StaticLayer OccupancyGridUpdate master content')
            # Block the actual master mutex in a separate server callback.
            spin_until(lambda:hold.service_is_ready())
            pending_hold=hold.call_async(Trigger.Request());spin_until(lambda:True in observed)
            started=time.monotonic();req=GetCostmap.Request();pending=client.call_async(req)
            stop=time.monotonic()+.1
            while time.monotonic()<stop:rclpy.spin_once(node,timeout_sec=.01)
            assert not pending.done(),'snapshot escaped real master mutex'
            spin_until(pending.done);elapsed=time.monotonic()-started
            assert elapsed>=.2 and list(pending.result().map.data)==expected
            spin_until(pending_hold.done);assert pending_hold.result().success
            print('PASS read waits for actual master mutex, elapsed',round(elapsed,3))
            token='a'*32;packet=call('3yd_delta_v2|'+token+'|0|0')
            assert packet.metadata.layer=='3yd_delta_v2|'+token
            assert bytes(packet.data[:8])==b'3YDACK07'
            assert not call('3yd_delta_v2|bad|0|0').data
            print('PASS original delta protocol through master service')
            spin_until(lambda:lifecycle.service_is_ready());state(4)
            assert not call().data
            state(3)
            end=time.monotonic()+3
            while list(call().data)!=expected:
                assert time.monotonic()<end
            print('PASS deactivate rejects reads and reactivation resumes')
            assert process.wait(timeout=15)==0,'host cleanup failed'
            print('ALL MASTER SNAPSHOT TESTS PASSED; host log',log)
        finally:
            if process.poll() is None:process.terminate();process.wait(timeout=5)
            node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
