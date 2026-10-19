"""Native storage covers the complete logical task; global audits stay global.

Every ACK observes all cells of the actual native map. No unobserved global
cells are filled in or represented as server readback. Progress does not
shrink/recenter a task's rectangle; only an affected new route may expand it.
"""
from pathlib import Path
from array import array
import time,weakref
import numpy as np,yaml
from arena_evaluation.semantic_map import canonical_hash
from arena_evaluation.semantic_costmap_composer import SemanticCostmap
from arena_3d_v1.semantic_world import json_write
from .v9_maps import content_digest,seal,seal_grid,immutable
from .v9_r3_session import StartupDeltaPerformanceSession
from .v9_r5_options import enabled
from .v9_r5_storage import bounds,union,context,soft_context

class FullRouteStorageSession(StartupDeltaPerformanceSession):
 def __init__(self,ctx,output,**kwargs):
  self.r5_enabled=enabled();self.global_ctx=ctx;self.r5_task_key=None
  self.r5_scope_bounds=None;self.r5_native_bounds=None;self.r5_geometry_events=[]
  self._r5_global_grid=None;self._r5_controller=None;self._r5_switching=False
  self._r5_grid_cache=None;self._r5_mask_cache=None
  if not self.r5_enabled:super().__init__(ctx,output,**kwargs);return
  m=ctx.hospital_map;h=min(16,m.height);w=min(16,m.width)
  self._r5_bootstrap_shape=(h,w)
  directory=(Path(output)/'bootstrap').resolve();directory.mkdir(parents=True,exist_ok=True)
  pgm=directory/'map.pgm';pgm.write_bytes(f'P5\n{w} {h}\n255\n'.encode()+bytes(w*h))
  boot=context(ctx,(m.height-h,m.height,0,w))
  boot.hospital_map.occupancy=seal(np.full((h,w),100,np.int8))
  boot.free_mask=np.zeros((h,w),bool);boot.distance_m=np.zeros((h,w),np.float32)
  boot.hospital_map.distance_m=boot.distance_m
  path=directory/'map.yaml';path.write_text(yaml.safe_dump({'image':str(pgm),'resolution':m.resolution,
        'origin':list(boot.hospital_map.origin),'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
  kwargs['map_yaml']=path
  super().__init__(boot,output,**kwargs)

 def bind_complete_task(self,query,controller=None):
  if not self.r5_enabled:return
  key=(self.global_ctx.map_sha256,str(query.query_id),tuple(query.goal),tuple(self.bound_vehicle))
  if key!=self.r5_task_key:self.r5_task_key=key;self.r5_scope_bounds=None
  if controller is not None:self._r5_controller=weakref.ref(controller)

 def bind_complete_route_scope(self,query,mask):
  if not self.r5_enabled:return
  self.bind_complete_task(query)
  if np.shape(mask)!=(self.global_ctx.hospital_map.height,self.global_ctx.hospital_map.width):
   raise ValueError('R5 complete route must use original global mask')
  self.r5_scope_bounds=union(self.r5_scope_bounds,bounds(mask,self.global_ctx.hospital_map.resolution))

 def set_semantic_costmap(self,costmap):
  if not self.r5_enabled:return super().set_semantic_costmap(costmap)
  if self._action_in_progress:raise RuntimeError('R5 map binding changed during action')
  self._r5_global_grid=costmap

 def _prepare_native(self,mask):
  shape=(self.global_ctx.hospital_map.height,self.global_ctx.hospital_map.width)
  if np.shape(mask)!=shape:
   if self.r5_native_bounds is None or np.shape(mask)!=(self.ctx.hospital_map.height,self.ctx.hospital_map.width):
    raise ValueError('R5 mask geometry mismatch')
   return mask
  if self.r5_scope_bounds is None:
   c=None if self._r5_controller is None else self._r5_controller()
   if c is None or c._search_mask is None or c.r13_remainder_mask is not None:
    raise RuntimeError('R5 complete task scope required before native planning')
   self.r5_scope_bounds=bounds(c._target_mask(),self.global_ctx.hospital_map.resolution)
  box=self.r5_scope_bounds;a,b,x,y=box
  cached_mask=self._r5_mask_cache;mask_immutable=immutable(np.asarray(mask))
  same_proven_mask=(mask_immutable and cached_mask is not None
                    and cached_mask[0] is mask and cached_mask[1]==box)
  # Every allowed cell must fit the already bound complete task, including
  # any explicit reroute expansion. Never infer a moving box from progress.
  if not same_proven_mask and (np.any(mask[:a]) or np.any(mask[b:]) or np.any(mask[a:b,:x]) or np.any(mask[a:b,y:])):
   raise RuntimeError('R5 current allowed mask escaped complete task scope')
  global_grid=self._r5_global_grid
  if global_grid is None:raise RuntimeError('R5 global semantic source missing')
  if box!=self.r5_native_bounds:
   if self._action_in_progress:raise RuntimeError('R5 geometry switch during action')
   old=self.r5_native_bounds;self.ctx=context(self.global_ctx,box);self.r5_native_bounds=box
   self._current_grid=None;self._current_allowed_mask=None
   self._last_exact_expected_master=None;self._last_exact_signature=None;self._last_exact_ack=None
   self._exact_ack_cache.clear();self._semantic_ack_cache.clear();self._r6_decoder=None
   self._costmap_state_trusted=False;self._force_full_next_update=True;self._r5_switching=True
   self._r5_grid_cache=None;self._r5_mask_cache=None
   self.r5_geometry_events.append({'task':list(self.r5_task_key),'old_bounds':old,'bounds':box,
       'origin':self.ctx.hospital_map.origin,'shape':[b-a,y-x],'complete_task_scope':True})
  if same_proven_mask:
   local_mask=cached_mask[2]
  else:
   local_mask=mask[a:b,x:y]
   # Mutable recovery masks are always checked and rehashed. They must not
   # discard a still valid immutable full-task mask/view binding.
   if mask_immutable:self._r5_mask_cache=(mask,box,local_mask)
  signature=self._content_signature(global_grid);cached=self._r5_grid_cache
  if (cached is not None and cached[0].occupancy_grid is global_grid.occupancy_grid
      and cached[0].expected_master_cost is global_grid.expected_master_cost
      and cached[1]==box and cached[2]==signature
      and immutable(global_grid.occupancy_grid) and immutable(global_grid.expected_master_cost)):
   occupancy,master=cached[3:]
  else:
   occupancy=seal(global_grid.occupancy_grid[a:b,x:y]);master=seal(global_grid.expected_master_cost[a:b,x:y])
  # R6 may wrap the same immutable storage in a fresh SemanticCostmap. Keep
  # the publication views while releasing the old wrapper's mutable fields.
  self._r5_grid_cache=(global_grid,box,signature,occupancy,master)
  policy=canonical_hash(['v9-r5-complete-task-storage-v1',global_grid.policy_hash,self.r5_task_key,box,
                         self.ctx.hospital_map.origin,self.ctx.hospital_map.resolution])
  full_mask_hash=content_digest(mask)
  detail={**global_grid.diagnostics,
      'r5_complete_task_bounds':list(box),'r5_complete_scope_not_rolling':True,
      'r5_original_global_shape':list(shape),'r5_native_shape':[b-a,y-x],
      'r5_native_origin':list(self.ctx.hospital_map.origin),'r5_original_map_loaded_in_nav2':False,
      'r5_source_full_grid_hash':global_grid.expected_grid_hash,'r5_source_full_master_hash':global_grid.expected_master_hash,
      'r5_source_full_mask_hash':full_mask_hash,
      'r5_source_composed_bound_hash':global_grid.diagnostics.get('search_bound_hash')}
  # A crop cannot manufacture the parent's exact immutable mask certificate.
  # A differing recovery mask follows the original copy-and-close path.
  if global_grid.diagnostics.get('search_bound_hash')==full_mask_hash:
   detail['search_bound_hash']=content_digest(local_mask)
  else:detail.pop('search_bound_hash',None)
  # Hard/soft views are refreshed on every call, preserving source mutations
  # as the parent does. Only truly immutable publication arrays are cached.
  local=SemanticCostmap(global_grid.internal_cost[a:b,x:y],occupancy,master,soft_context(global_grid.soft_cost,box),
       global_grid.hard_semantic_mask[a:b,x:y],global_grid.affected_mask[a:b,x:y],policy,
       global_grid.semantic_map_hash,global_grid.preference_field_hash,global_grid.semantics_enabled,detail)
  super().set_semantic_costmap(seal_grid(local))
  return local_mask

 def update_local_mask(self,mask,**kwargs):
  if not self.r5_enabled:return super().update_local_mask(mask,**kwargs)
  if (kwargs.get('initialization') and self.r5_native_bounds is None and self.r5_scope_bounds is None
      and self._r5_global_grid is None and np.shape(mask)==self._r5_bootstrap_shape and not np.any(mask)):
   result=super().update_local_mask(mask,**kwargs)
   result.update(r5_bootstrap_all_closed=True,r5_native_shape=list(self._r5_bootstrap_shape))
   return result
  local=self._prepare_native(mask);resize_pending=self._r5_switching
  result=super().update_local_mask(local,**kwargs)
  # The parent excludes reuse after its emergency full-grid fallback. A
  # required task geometry resize uses that same publication branch, although
  # it is an ordinary storage operation. Only a complete, twice observed,
  # exact native frame permits the existing complete-key no-op on the next
  # query. Failed/partial repairs and later genuine fallbacks keep the guard.
  if (resize_pending and result.get('local_map_update_mode')=='full_fallback'
      and result.get('local_map_update_fallback_reason')=='forced_full_update'
      and self._costmap_state_trusted and not self._r5_switching
      and result.get('costmap_update_acknowledged') is True
      and result.get('costmap_ack_mismatch_cells')==0
      and result.get('costmap_ack_checked_cells')==local.size
      and result.get('semantic_exact_stable_observations',0)>=2
      and result.get('server_costmap_content_hash')==result.get('semantic_expected_server_content_hash')
      and result.get('server_costmap_content_hash') is not None):
   self._last_update_had_fallback=False
   result['r5_complete_resize_exactly_verified']=True
  self._reuse_verified_bookkeeping(local,result)
  result.update({k:v for k,v in self._semantic_costmap.diagnostics.items() if k.startswith('r5_')})
  return result

 def _reuse_verified_bookkeeping(self,mask,result):
  """Release equal bookkeeping copies only after the actual current ACK.

  A full geometry publication in the parent retains writable copies even
  when the exact immutable composed source is still available. Compare the
  existing bytes before replacing them with views; do not manufacture source
  mask certificates, skip a readback, or alter an emergency fallback guard.
  """
  grid=getattr(self,'_semantic_costmap',None)
  if (grid is None or not getattr(self,'_costmap_state_trusted',False)
      or result.get('costmap_update_acknowledged') is not True
      or result.get('costmap_ack_mismatch_cells')!=0
      or result.get('semantic_exact_stable_observations',0)<2
      or not result.get('server_costmap_content_hash')
      or result.get('server_costmap_content_hash')!=result.get('semantic_expected_server_content_hash')
      or result.get('semantic_policy_hash')!=str(grid.policy_hash)
      or result.get('semantic_expected_grid_hash')!=grid.expected_grid_hash
      or result.get('semantic_expected_master_hash')!=grid.expected_master_hash):return
  def equal(a,b):
   if not isinstance(a,np.ndarray) or a.shape!=b.shape or a.dtype!=b.dtype:return False
   if (a.strides==b.strides and a.ctypes.data==b.ctypes.data):return True
   return all(np.array_equal(a[y:y+64],b[y:y+64]) for y in range(0,len(a),64))
  values=grid.occupancy_grid
  if immutable(values):
   expected=np.flipud(values);current=getattr(self,'_current_grid',None)
   if equal(current,expected):
    result['r5_bookkeeping_grid_replaced_owned_copy_bytes']=int(current.nbytes) if current.flags.owndata else 0
    self._current_grid=expected
    result['r5_bookkeeping_grid_immutable_view']=True
  if immutable(mask):
   current=getattr(self,'_current_allowed_mask',None)
   if equal(current,mask):
    result['r5_bookkeeping_mask_replaced_owned_copy_bytes']=int(current.nbytes) if current.flags.owndata else 0
    self._current_allowed_mask=mask
    result['r5_bookkeeping_mask_immutable_view']=True

 def plan(self,query,*args,**kwargs):
  if self.r5_enabled and kwargs.get('allowed_mask') is not None:
   kwargs['allowed_mask']=self._prepare_native(kwargs['allowed_mask'])
  return super().plan(query,*args,**kwargs)

 def reset_query_state(self,query_id,*,restore_base_map=True):
  if self.r5_enabled and restore_base_map:raise RuntimeError('R5 unbounded base restore forbidden')
  return super().reset_query_state(query_id,restore_base_map=restore_base_map)

 def _publish_full_grid(self,values,*,clear_costmap=True):
  if not self.r5_enabled:return super()._publish_full_grid(values,clear_costmap=clear_costmap)
  self._begin_publication(np.ones(values.shape,bool),full=True)
  started=time.monotonic();m=self.ctx.hospital_map
  msg=self.OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=self.client.node.get_clock().now().to_msg()
  msg.info.resolution=m.resolution;msg.info.width=m.width;msg.info.height=m.height
  msg.info.origin.position.x=m.origin[0];msg.info.origin.position.y=m.origin[1];msg.info.origin.orientation.w=1.
  msg.data=array('b',np.ascontiguousarray(values,dtype=np.int8).tobytes())
  serialization_ms=(time.monotonic()-started)*1000;deadline=time.monotonic()+self.costmap_ack_timeout_s
  while self._local_map_publisher.get_subscription_count()==0:
   if time.monotonic()>=deadline:raise RuntimeError('R5 native map subscriber unavailable')
   self.client.executor.spin_once(timeout_sec=.005)
  started=time.monotonic();self._local_map_publisher.publish(msg)
  self._last_publish_timing={'local_map_serialization_ms':serialization_ms,
       'local_map_publication_ms':(time.monotonic()-started)*1000,'costmap_settle_ms':0.}
  return 0.

 def _server_costmap_snapshot(self,deadline):
  if not self.r5_enabled:return super()._server_costmap_snapshot(deadline)
  while True:
   try:
    result=super()._server_costmap_snapshot(deadline);self._r5_switching=False;return result
   except RuntimeError as error:
    geometry=any(v in str(error) for v in ('dimensions','geometry','size mismatch'))
    if not self._r5_switching or not geometry or time.monotonic()>=deadline:raise
    self.client.executor.spin_once(timeout_sec=min(.01,max(0.,deadline-time.monotonic())))

 def close(self):
  try:return super().close()
  finally:json_write(self.lifecycle_root/'r5_complete_geometry.json',self.r5_geometry_events)
