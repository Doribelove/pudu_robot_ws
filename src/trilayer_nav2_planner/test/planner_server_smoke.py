#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Real planner_server/ComputePathToPose acceptance; requires a real worker socket.
No fake worker is provided. Caller domain185; worker MUST use a different domain.
Usage: python3 planner_server_smoke.py ABS_WORKER_SOCKET
"""
import os,sys,time,tempfile,subprocess,signal,json
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='185'
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import yaml
socket=str(Path(sys.argv[1]).resolve());out=Path(tempfile.mkdtemp(prefix='trilayer-planner-server-'))
params={'planner_server':{'ros__parameters':{'planner_plugins':['TriLayer'],'expected_planner_frequency':.01,'TriLayer':{'plugin':'trilayer_nav2_planner/Planner','worker_socket':socket,'timeout_seconds':55.,'allow_unknown':False}}},'global_costmap':{'global_costmap':{'ros__parameters':{'global_frame':'map','robot_base_frame':'base_link','resolution':.05,'rolling_window':False,'track_unknown_space':True,'footprint':'[[0.265,0.225],[0.265,-0.225],[-0.265,-0.225],[-0.265,0.225]]','footprint_padding':0.,'update_frequency':10.,'publish_frequency':1.,'plugins':['static_layer'],'static_layer':{'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True,'map_topic':'/map'}}}}}
(out/'params.yaml').write_text(yaml.safe_dump(params))
rclpy.init();node=rclpy.create_node('trilayer_planner_acceptance')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
pub=node.create_publisher(OccupancyGrid,'map',qos)
msg=OccupancyGrid();msg.header.frame_id='map';msg.info.resolution=.05;msg.info.width=240;msg.info.height=240;msg.info.origin.orientation.w=1.
msg.data=[100 if x<4 or x>=236 or y<4 or y>=236 else 0 for y in range(240) for x in range(240)]
tf=StaticTransformBroadcaster(node);tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='base_link';tr.transform.translation.x=2.;tr.transform.translation.y=2.;tr.transform.rotation.w=1.;tf.sendTransform(tr)
log=open(out/'planner.log','w');proc=subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server','--ros-args','--params-file',str(out/'params.yaml')],stdout=log,stderr=log,start_new_session=True)
def spin_until(future,seconds):
 end=time.monotonic()+seconds
 while not future.done() and time.monotonic()<end:
  msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg);rclpy.spin_once(node,timeout_sec=.05)
 assert future.done(),'operation timeout'
 return future.result()
def lifecycle(transition):
 client=node.create_client(ChangeState,'/planner_server/change_state');assert client.wait_for_service(timeout_sec=15)
 req=ChangeState.Request();req.transition.id=transition;assert spin_until(client.call_async(req),20).success
try:
 pub.publish(msg);lifecycle(1);lifecycle(3)
 # Activation does not prove the first static master update has completed.
 from nav2_msgs.srv import GetCostmap
 master=node.create_client(GetCostmap,'/global_costmap/get_costmap');assert master.wait_for_service(timeout_sec=5)
 ready_deadline=time.monotonic()+10
 while True:
  initial=spin_until(master.call_async(GetCostmap.Request()),3).map
  if initial.metadata.size_x==240 and initial.metadata.size_y==240 and all((cost>=253)==(occupancy==100) for cost,occupancy in zip(initial.data,msg.data)):break
  assert time.monotonic()<ready_deadline,'host initial static map not ready'

 client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose');assert client.wait_for_server(timeout_sec=10)
 request=ComputePathToPose.Goal();request.use_start=True;request.planner_id='TriLayer'
 for pose,x in [(request.start,2.),(request.goal,5.)]:pose.header.frame_id='map';pose.pose.position.x=x;pose.pose.position.y=4.;pose.pose.orientation.w=1.
 handle=spin_until(client.send_goal_async(request),10);assert handle.accepted
 result=spin_until(handle.get_result_async(),65)
 assert result.status==4,(result.status,str(result.result))
 path=result.result.path;assert path.header.frame_id=='map' and len(path.poses)>=2
 assert abs(path.poses[0].pose.position.x-2.)<1e-4 and abs(path.poses[-1].pose.position.x-5.)<1e-4
 # Same caller context must support another real action on the persistent worker.
 handle2=spin_until(client.send_goal_async(request),10);assert handle2.accepted
 repeated=spin_until(handle2.get_result_async(),65);assert repeated.status==4
 # A real new host obstacle crosses the former route; no observed-free inference.
 for y in range(65,95):
  for x in range(65,75):msg.data[y*240+x]=100
 pub.publish(msg)
 # Query this planner_server's own master until the changed cell is present.
 end=time.monotonic()+10
 while True:
  snapshot=spin_until(master.call_async(GetCostmap.Request()),3).map
  if snapshot.metadata.size_x==240 and snapshot.data[80*240+70]>=253:break
  assert time.monotonic()<end,'host map did not update'
 handle3=spin_until(client.send_goal_async(request),10);assert handle3.accepted
 changed=spin_until(handle3.get_result_async(),65);assert changed.status==4,(changed.status,str(changed.result))
 assert len(changed.result.path.poses)>2
 assert not any(3.25<=p.pose.position.x<3.75 and 3.25<=p.pose.position.y<4.75 for p in changed.result.path.poses)
 # Start a cold request, observe generation3 snapshot persisted, then change host.
 msg.data[30*240+30]=100;pub.publish(msg)
 while True:
  snap=spin_until(master.call_async(GetCostmap.Request()),3).map
  if snap.data[30*240+30]>=253:break
 race=spin_until(client.send_goal_async(request),10);assert race.accepted
 generation=Path(socket).parent/'generation-0003/map.pgm'
 end=time.monotonic()+10
 while not generation.exists():
  assert time.monotonic()<end,'no captured cold snapshot'
  rclpy.spin_once(node,timeout_sec=.005)
 msg.data[31*240+31]=100;pub.publish(msg)
 while True:
  snap=spin_until(master.call_async(GetCostmap.Request()),3).map
  if snap.data[31*240+31]>=253:break
 stale=spin_until(race.get_result_async(),65)
 assert stale.status==6 and not stale.result.path.poses,'stale host snapshot returned executable path'
 assert 'caller costmap changed while planning' in (out/'planner.log').read_text()
 # The underlying P6 really succeeded; only the caller snapshot race rejected it.
 evidence=json.loads((Path(socket).parent/'request-000004.json').read_text())
 assert evidence['success'] and evidence['path']
 # Real boundary rejection: invalid planar quaternion, no worker invocation.
 request.goal.pose.orientation.w=0.
 invalid=spin_until(client.send_goal_async(request),10);assert invalid.accepted
 invalid_result=spin_until(invalid.get_result_async(),10)
 assert invalid_result.status==6 and not invalid_result.result.path.poses
 assert 'requires finite planar normalized map poses' in (out/'planner.log').read_text()
 request.goal.pose.orientation.w=1.
 # Make only our private test socket unavailable; do not substitute a worker.
 socket_path=Path(socket);hidden=socket_path.with_name('temporarily-hidden.sock');socket_path.rename(hidden)
 try:
  unavailable=spin_until(client.send_goal_async(request),10);assert unavailable.accepted
  unavailable_result=spin_until(unavailable.get_result_async(),10)
  assert unavailable_result.status==6 and not unavailable_result.result.path.poses
  assert 'worker unavailable' in (out/'planner.log').read_text()
 finally:hidden.rename(socket_path)
 print(json.dumps({'passed':True,'points':len(path.poses),'changed_points':len(changed.result.path.poses),'stale_snapshot_rejected':True,'invalid_pose_rejected':True,'unavailable_worker_rejected':True,'evidence':str(out),'scope':'real planner_server actions: initial, repeat, cold obstacle replan, midflight caller snapshot rejection'}),flush=True)
 lifecycle(4);lifecycle(2)
finally:
 if proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=5)
 log.close();node.destroy_node();rclpy.shutdown()
