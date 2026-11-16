"""Explicit 3D-V1-r1 extension; no default production/r2 factory imports."""
from dataclasses import replace
import time,os
import numpy as np
from arena_evaluation.semantic_smac_session_r2 import ExactSemanticSmacSessionR2
from .r1_pipeline import Layered3DV1R1Controller
from .pipeline import PipelineStep,ProductionL3Adapter,corridor_dirty_transition
from .semantic_l2 import SemanticL2Lifecycle
from .semantic_reference import preference_field,reference_costmap
from .semantic_world import WORK

class ReferenceSmacSession(ExactSemanticSmacSessionR2):
 PUBLICATION_VERSION='3D-V1-r1-semantic-reference-adapter-v3'
 bound_vehicle=(.265,.225,.40)
 def __init__(self,*args,**kwargs):
  # Humble's localhost_only flag appends a default 512 KiB SHM transport
  # even when a custom profile is loaded. Our explicit UDP whitelist already
  # restricts discovery to loopback; suppress the extra undersized transport.
  profile=str(WORK/'config/fastdds_large_map.xml')
  os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE',profile)
  if os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']==profile:os.environ['ROS_LOCALHOST_ONLY']='0'
  # A semantic key is not a reason to resend the entire 8.6 MB map. The
  # inherited exact ACK still verifies all hard, soft and effective-dirty
  # cells after the bounded old/new source ROI has been published.
  kwargs.setdefault('force_full_on_semantic_signature_change',False)
  super().__init__(*args,**kwargs)
  from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
  # The map topic's depth-one profile is unsuitable for a multi-tile update:
  # a later tile can replace an earlier unsent sample. Keep the bounded
  # update stream in its own reliable queue and allow subscriber callbacks
  # to consume it while publishing. This is transport pacing before ACK,
  # never a fixed settle delay after successful content verification.
  self._map_update_qos=QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
  self.roi_publish_pacing_s=.015
 def update_local_mask(self,allowed_mask,**kwargs):
  semantic=self._semantic_costmap
  changed_key=semantic is not None and self._content_signature(semantic)!=self._last_exact_signature
  reuse=self.enable_mask_reuse_noop
  if changed_key:self.enable_mask_reuse_noop=False
  try:return super().update_local_mask(allowed_mask,**kwargs)
  finally:self.enable_mask_reuse_noop=reuse
 def _publish_dirty_roi(self,expected,changed):
  discovery=time.monotonic()+self.costmap_ack_timeout_s
  while self._local_update_publisher.get_subscription_count()==0:
   if time.monotonic()>=discovery:raise RuntimeError('costmap update subscriber discovery timed out')
   self.client.executor.spin_once(timeout_sec=.005)
  # Byte-identical content with a new policy still needs fresh sequence-bound
  # evidence. Reassert one cell, then apply the same full-content ACK gate.
  if self._semantic_costmap is not None and not np.any(changed):
   changed=np.asarray(changed,dtype=bool).copy();changed[0,0]=True
  return super()._publish_dirty_roi(expected,changed)
 def _publish_full_grid(self,values,*,clear_costmap=True):
  # BenchmarkStack.start has already loaded the real map's dimensions and
  # origin. Cold all-lethal initialization can therefore use bounded update
  # tiles too; no duplicate multi-megabyte map/update message is necessary.
  clear_ms=self._clear_global_costmap() if clear_costmap else 0.
  old=self._current_grid
  if old is None:self._current_grid=np.zeros_like(values)
  _,timing=self._publish_dirty_roi(values,np.ones(np.asarray(values).shape,dtype=bool))
  # On initial publication keep the untrusted bookkeeping buffer available
  # for the inherited content-mismatch repair. It becomes reusable only
  # after the parent has successfully ACKed and committed the source grid.
  if old is not None:self._current_grid=old
  self._last_publish_timing=timing
  return clear_ms

class SemanticR1Controller(Layered3DV1R1Controller):
 def __init__(self,world,route,*,cache_root,side='right',l2_weight=2.,verify_l2_oracle=False):
  self.world=world;self.route=route;self.side=side
  self.preference=preference_field(world,route,side=side)
  self.reference_key='';self.acknowledged_reference_key='';self.reference_changed=True
  manager=SemanticL2Lifecycle(cache_root,self.preference.potential,l2_weight)
  super().__init__(route.plan,cache_root=cache_root,lifecycle_manager=manager,verify_l2_oracle=verify_l2_oracle)
 @property
 def runtime_contract(self):
  return {**super().runtime_contract,'revision_id':'r1-semantic-dual-map-extension-v1',
    'parent_revision':'r1-l2-state-lifecycle-soak','l1':'semantic_interface_directed_graph_astar',
    'l2_cost':'shared_finite_nonnegative_weighted_edge_cost','l3_reference':'exact_effective_master_soft_reference'}
 def change_preference(self,side,weight):
  # Rebinding on a new direction/weight avoids reusing the old mutable g/rhs.
  self.side=side;self.preference=preference_field(self.world,self.route,side=side)
  self.lifecycle.potential=self.preference.potential;self.lifecycle.weight=weight
  self.initial_l2_result=self._bind_l1_plan(self.route.plan,self.confirmation.blocked_cells)
  self.reference_changed=True
  return self.initial_l2_result
 def replace_route(self,route):
  self.route=route;self.preference=preference_field(self.world,route,side=self.side)
  self.lifecycle.potential=self.preference.potential
  self.initial_l2_result=self._bind_l1_plan(route.plan,self.confirmation.blocked_cells)
  self.reference_changed=True
  return self.initial_l2_result
 def process_snapshot(self,snapshot,*,graph=None,query=None,l1_replan=None,now=None):
  if graph is not None:
   if graph.world.key!=self.world.key or query is None:raise ValueError('reroute world/query mismatch')
   def replan(blocked):
    replacement=graph.plan(query,blocked_cells=blocked)
    if replacement is None:return None
    self.route=replacement;self.preference=preference_field(self.world,replacement,side=self.side)
    self.lifecycle.potential=self.preference.potential;self.reference_changed=True
    return replacement.plan
   l1_replan=replan
  step=super().process_snapshot(snapshot,l1_replan=l1_replan,now=now)
  if step.l2_result is not None:self.initial_l2_result=step.l2_result
  # L3 can deviate from L2. Even an increase off the L2 path must invalidate
  # the old L3 candidate when it changes the allowed corridor.
  hard_changed=bool(np.any(self._target_mask()!=self.server_l3_mask))
  if not step.l3_required and self.l2.path_global and (hard_changed or self.reference_changed):
   target=self._target_mask();self.pending_l3_mask=target
   dirty=corridor_dirty_transition(self.server_l3_mask,target);self._pending_l3_hash=dirty.target_hash
   step=replace(step,l3_required=True,target_l3_mask=target,dirty_roi=dirty,
    diagnostics={**step.diagnostics,'semantic_reference_or_l3_corridor_changed':True})
  return step
 def prepare_l3(self,*,enabled=True,cap=140):
  if not self.l2.path_global:return None
  costmap=reference_costmap(self.world,self.route,self.l2.path_global,enabled=enabled,cap=cap,side=self.side,blocked=self.confirmation.blocked_cells)
  self.reference_key=costmap.policy_hash
  self.reference_changed=self.reference_key!=self.acknowledged_reference_key
  target=self._target_mask();self.pending_l3_mask=target
  dirty=corridor_dirty_transition(self.server_l3_mask,target);self._pending_l3_hash=dirty.target_hash
  step=PipelineStep(None,None,self.initial_l2_result,False,False,True,'',target,dirty,self.plan.route_signature,
   {'reference_changed':self.reference_changed,'reference_key':self.reference_key,'world_key':self.world.key})
  return step,costmap
 def plan_l3(self,query,session,auditor,spec,*,enabled=True,cap=140):
  if tuple(self.world.vehicle)!=tuple(session.bound_vehicle):
   raise ValueError('vehicle version mismatch: frozen Smac session cannot reuse another vehicle configuration')
  prepared=self.prepare_l3(enabled=enabled,cap=cap)
  if prepared is None:return {'success':False,'failure_code':'L2_NO_PATH'}
  step,costmap=prepared;session.set_semantic_costmap(costmap)
  outcome=ProductionL3Adapter(self,auditor).plan(step,query,session,spec)
  outcome['reference_diagnostics']=costmap.diagnostics
  outcome['reference_changed']=self.reference_changed
  if outcome.get('called') and outcome.get('diagnostics',{}).get('costmap_update_acknowledged'):
   self.acknowledged_reference_key=self.reference_key;self.reference_changed=False
  if outcome.get('result') and outcome['result'].points:
   p=np.asarray([[v['x'],v['y'],v['yaw']] for v in outcome['result'].points])
   independent=self.world.audit(query,p,step.target_l3_mask)
   outcome['independent_audit']=independent
   outcome['success']=bool(outcome.get('success') and independent['valid'])
   if not independent['valid']:outcome['failure_code']='INDEPENDENT_AUDIT_REJECTED'
  return outcome
