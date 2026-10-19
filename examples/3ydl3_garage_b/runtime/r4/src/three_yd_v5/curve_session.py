"""Consolidate identical benchmark static transforms into the worker process."""
from pathlib import Path
import os,re,subprocess,time
from arena_evaluation.planner_benchmark.runner import BenchmarkStack,BenchmarkInputError
from .v9_r7_batch import BoundStorageSession
class CurveStack(BenchmarkStack):
 def __init__(self,*,composed=False,native=False,**kwargs):
  super().__init__(**kwargs);self.composed=composed;self.native=native
 def _started_processes(self):
  if self.native:
   if self.process is None or self.process.poll() is not None or not self._log_contains('CURVE_NAV2_NODES_CREATED'):return {}
   return {name:self.process.pid for name in ('planner_server','map_server','lifecycle_manager')}
  if not self.composed:return super()._started_processes()
  try:content=self.log_file.read_text(errors='replace')
  except OSError:return {}
  match=re.search(r'\[component_container_isolated-[^]]+\]: process started with pid \[(\d+)\]',content)
  if match is None:return {}
  pid=int(match.group(1))
  # A process alone is not readiness: verify each requested component loaded.
  return {name:pid for name in ('planner_server','map_server','lifecycle_manager')
          if "Loaded node '/"+('lifecycle_manager_planner' if name=='lifecycle_manager' else name)+"' in container" in content}
 def pids(self):
  main,children,error=super().pids()
  return main,list(dict.fromkeys(children)),error
 def start(self,timeout=45.):
  self.log_file.parent.mkdir(parents=True,exist_ok=True);self.log_stream=self.log_file.open('w')
  launch=Path(__file__).resolve().parents[2]/'tools'/('curve_composed.launch.py' if self.composed else 'curve_stack.launch.py')
  command=['ros2','launch',str(launch),f'map_yaml:={self.map_yaml}',f'params_file:={self.params_file}','use_sim_time:=false']
  if self.native:
   executable=Path(__file__).resolve().parents[2]/'native/curve_nav2_stack'
   command=[str(executable),str(self.map_yaml.resolve()),'--ros-args','--params-file',str(self.params_file.resolve())]
  self.process=subprocess.Popen(command,stdout=self.log_stream,stderr=subprocess.STDOUT,start_new_session=True,env=os.environ.copy())
  observed_ready=os.environ.get('THREEYD_V10_READY_START','1')=='1'
  deadline=time.monotonic()+timeout
  while time.monotonic()<deadline:
   if self.process.poll() is not None:raise BenchmarkInputError(f'curve stack exited; see {self.log_file}')
   started=self._started_processes()
   if all(n in started for n in ('planner_server','map_server','lifecycle_manager')) and self._log_contains('Managed nodes are active'):
    self.readiness_diagnostics={'nodes_created':True,'lifecycle_active_observed':True,
       'fixed_settle_sleep_s':0. if observed_ready else 1.,
       'bootstrap_exact_ACK_still_required':True,'poll_interval_s':.02 if observed_ready else .2}
    # Parent startup still waits for the actual subscriber and verifies the
    # bootstrap costmap. A fixed delay after observed activation adds no ACK.
    if not observed_ready:time.sleep(1.)
    return
   time.sleep(.02 if observed_ready else .2)
  raise BenchmarkInputError(f'curve stack startup timed out; see {self.log_file}')
class CurveSession(BoundStorageSession):
 def start(self):
  from .curve_backend import settings
  mode,cfg,_=settings();self._curve_tf_node=None
  if mode!='control' and cfg.get('pose_interpolation_cache',False):
   from .curve_interpolation import install
   self._curve_interpolation_installed=install()
  if mode!='control' and cfg.get('worker_static_tf',False):
   from rclpy.node import Node
   from geometry_msgs.msg import TransformStamped
   from tf2_ros import StaticTransformBroadcaster
   self._curve_tf_node=Node('curve_static_tf_'+str(os.getpid()),context=self.context)
   self._curve_tf=StaticTransformBroadcaster(self._curve_tf_node)
   transforms=[]
   for parent,child in [('map','odom'),('odom','base_link')]:
    t=TransformStamped();t.header.stamp=self._curve_tf_node.get_clock().now().to_msg()
    t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;transforms.append(t)
   self._curve_tf.sendTransform(transforms)
   old=self.stack;self.stack=CurveStack(map_yaml=old.map_yaml,params_file=old.params_file,log_file=old.log_file,
       composed=cfg.get('composed_nav2',False),native=cfg.get('native_nav2_stack',False))
  try:return super().start()
  except BaseException:
   if self._curve_tf_node is not None:self._curve_tf_node.destroy_node();self._curve_tf_node=None
   raise
 def close(self):
  # Node must remain alive while the native stack uses transient-local TF.
  node=getattr(self,'_curve_tf_node',None)
  try:return super().close()
  finally:
   if node is not None:node.destroy_node();self._curve_tf_node=None
