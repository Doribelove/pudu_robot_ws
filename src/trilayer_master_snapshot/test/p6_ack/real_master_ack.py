#!/usr/bin/env python3
"""Original P6 ACK decision against a real stock Nav2 StaticLayer/master."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import numpy as np
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
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.srv import GetCostmap
from session_adapter import SnapshotSession


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--host',required=True)
    parser.add_argument('--log',default='/tmp/trilayer-p6-real-master-host.log')
    args=parser.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DOMAIN_ID')
    rclpy.init();ns='/p6_ack_'+uuid.uuid4().hex[:8]
    node=Node('p6_ack_test',namespace=ns);executor=SingleThreadedExecutor();executor.add_node(node)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    pub=node.create_publisher(OccupancyGrid,'input_map',qos)
    updates=node.create_publisher(OccupancyGridUpdate,'input_map_updates',10)
    service=node.create_client(GetCostmap,'trilayer_get_costmap')
    grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=4
    grid.info.resolution=1.;grid.info.origin.position.x=grid.info.origin.position.y=-1.
    grid.info.origin.orientation.w=1.;grid.data=[0]*16
    pub.publish(grid)
    output=open(args.log,'w');process=subprocess.Popen([args.host],stdout=output,stderr=subprocess.STDOUT,
        env={**os.environ,'TRILAYER_TEST_NAMESPACE':ns,'TRILAYER_TEST_SECONDS':'30'})
    def spin_until(predicate,timeout=10.):
        deadline=time.monotonic()+timeout
        while not predicate():
            assert process.poll() is None,'Nav2 test host exited; see '+args.log
            assert time.monotonic()<deadline,'DDS discovery deadline'
            executor.spin_once(timeout_sec=.01)
    def update(value):
        u=OccupancyGridUpdate();u.header.frame_id='map';u.header.stamp=node.get_clock().now().to_msg()
        u.x=u.y=1;u.width=u.height=1;u.data=[value];updates.publish(u)
    session=SnapshotSession(service,executor,(4,4),1.,(-1.,-1.))
    occupancy=np.zeros((4,4),np.int8);master=np.zeros((4,4),np.uint8)
    try:
        spin_until(lambda:service.service_is_ready() and pub.get_subscription_count()>0 and updates.get_subscription_count()>0)
        session.expect(occupancy,master)
        result,_=session.wait(1.)
        assert result and result['semantic_exact_stable_observations']==2
        print('PASS original P6 ACK on real master, two stable full-content observations')
        initial_count=len(session.ack_events)
        occupancy[1,1]=100;master[1,1]=254;session.expect(occupancy,master)
        result,_=session.wait(.12)
        assert result is None and len(session.ack_events)==initial_count
        print('PASS expected obstacle without actual StaticLayer update does not ACK')
        update(100);result,_=session.wait(1.)
        assert result and result['costmap_ack_hard_checked_cells']>=1
        assert result['server_costmap_content_hash']==__import__('hashlib').sha256(master).hexdigest()
        print('PASS obstacle publication -> StaticLayer update -> actual master -> P6 exact ACK')
        baseline=result['server_costmap_update_time_ns'];session._active_publication_baseline_timestamp_ns=baseline+10**12
        result,_=session.wait(.12)
        assert result is None
        session._active_publication_baseline_timestamp_ns=baseline
        print('PASS timestamp freshness gate rejects old-baseline observations')
        count=len(session.ack_events)
        def mutate(s):
            s._semantic_publication_sequence+=1;s.before_read=None
        session.before_read=mutate
        try:session.wait(.3)
        except RuntimeError as exc:assert 'publication changed' in str(exc)
        else:raise AssertionError('concurrent publication accepted')
        assert len(session.ack_events)==count
        print('PASS publication change during read prevents ACK')
        occupancy[1,1]=0;master[1,1]=0;session.expect(occupancy,master)
        result,_=session.wait(.12);assert result is None
        update(0);result,_=session.wait(1.)
        assert result and result['costmap_ack_stale_checked_cells']>=1
        print('PASS stale obstacle cannot ACK until real layer clears it')
        print('ALL REAL NAV2 MASTER / ORIGINAL P6 ACK TESTS PASSED')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2)
        output.close();executor.remove_node(node);node.destroy_node();executor.shutdown();rclpy.shutdown()
        assert process.returncode == 0, f"Nav2 test host exited with {process.returncode}; see {args.log}"


if __name__=='__main__':main()
