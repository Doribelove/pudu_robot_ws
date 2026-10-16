import time,math
from dataclasses import replace
from types import SimpleNamespace as S
import numpy as np,pytest
from arena_evaluation.semantic_costmap_composer import SemanticCostmap
from three_yd_v5.v9_maps import seal,seal_grid,content_digest
from three_yd_v5.v9_r3_session import StartupDeltaPerformanceSession
from three_yd_v5.v9_r5_session import FullRouteStorageSession
from three_yd_v5.v9_r5_storage import bounds,context,soft_context

class Map:
 def __init__(self):
  self.width=120;self.height=100;self.resolution=.05;self.origin=(-2.,3.,0.)
  self.occupancy=np.zeros((100,120),np.int8);self.distance_m=np.arange(12000,dtype=np.float32).reshape(100,120)
 def world_to_cell(self,x,y):
  col=math.floor((x-self.origin[0])/self.resolution);row=self.height-1-math.floor((y-self.origin[1])/self.resolution)
  return (row,col) if 0<=row<100 and 0<=col<120 else None
 def cell_to_world(self,q):return (self.origin[0]+(q[1]+.5)*.05,self.origin[1]+(100-q[0]-.5)*.05)

def make(monkeypatch):
 monkeypatch.setenv('THREEYD_V9_MODE','A');monkeypatch.setenv('THREEYD_V9_R1_MODE','S')
 m=Map();ctx=S(hospital_map=m,free_mask=np.ones((100,120),bool),distance_m=m.distance_m,map_sha256='original')
 s=FullRouteStorageSession.__new__(FullRouteStorageSession)
 s.r5_enabled=True;s.global_ctx=ctx;s.ctx=ctx;s.bound_vehicle=(.265,.225,.4)
 s.r5_task_key=None;s.r5_scope_bounds=None;s.r5_native_bounds=None;s.r5_geometry_events=[]
 s._r5_controller=None;s._r5_switching=False;s._r5_grid_cache=None;s._r5_mask_cache=None
 s._action_in_progress=False;s._exact_ack_cache={};s._semantic_ack_cache={}
 q=S(query_id='task_with_underscores',goal=(1.,5.,0.),start=(0.,4.,0.))
 mask=np.zeros((100,120),bool);mask[35:65,40:70]=True;mask=seal(mask)
 grid=seal_grid(SemanticCostmap(np.zeros((100,120),np.uint8),np.where(mask,0,100).astype(np.int8),
      np.where(mask,0,254).astype(np.uint8),np.zeros((100,120),np.float32),~mask,np.ones((100,120),bool),
      'policy','semantic','reference',True,{}))
 s._r5_global_grid=grid
 monkeypatch.setattr(StartupDeltaPerformanceSession,'set_semantic_costmap',lambda self,g:setattr(self,'_semantic_costmap',g))
 return s,q,mask,grid

def test_crop_keeps_original_cell_lattice_and_global_arrays(monkeypatch):
 s,q,mask,g=make(monkeypatch);before=s.global_ctx.hospital_map.distance_m.copy();s.bind_complete_route_scope(q,mask)
 local=s._prepare_native(mask);a,b,c,d=s.r5_native_bounds;m=s.ctx.hospital_map
 assert np.array_equal(local,mask[a:b,c:d]) and np.array_equal(s._semantic_costmap.expected_master_cost,g.expected_master_cost[a:b,c:d])
 for original in [(a,c),(b-1,d-1),(50,55)]:
  xy=s.global_ctx.hospital_map.cell_to_world(original);expected=(original[0]-a,original[1]-c)
  assert m.world_to_cell(*xy)==expected and m.cell_to_world(expected)==xy
 assert np.array_equal(before,s.global_ctx.hospital_map.distance_m)
 assert np.array_equal(s.ctx.distance_m,before[a:b,c:d])

def test_progress_never_moves_or_shrinks_complete_task(monkeypatch):
 s,q,mask,g=make(monkeypatch);s.bind_complete_route_scope(q,mask);initial=s.r5_scope_bounds
 small=np.zeros_like(mask);small[45:55,50:60]=True
 progressed=S(**{**vars(q),'start':(.03,4.,.03)});s.bind_complete_route_scope(progressed,small)
 assert s.r5_scope_bounds==initial
 reroute=np.zeros_like(mask);reroute[10:80,15:85]=True;s.bind_complete_route_scope(q,reroute)
 assert s.r5_scope_bounds[0]<=initial[0] and s.r5_scope_bounds[1]>=initial[1]
 other=S(**{**vars(q),'goal':(2.,6.,0.)});s.bind_complete_route_scope(other,small)
 assert s.r5_scope_bounds==bounds(small,.05)

def test_only_immutable_publication_data_cached_mutable_fields_refresh(monkeypatch):
 s,q,mask,g=make(monkeypatch);s.bind_complete_route_scope(q,mask);s._prepare_native(mask)
 first=s._semantic_costmap;g.soft_cost=np.ones_like(g.soft_cost)*7;g.hard_semantic_mask=np.zeros_like(mask)
 s._prepare_native(mask);second=s._semantic_costmap
 assert second.occupancy_grid is first.occupancy_grid and second.expected_master_cost is first.expected_master_cost
 assert np.all(second.soft_cost==7) and not second.hard_semantic_mask.any()
 with pytest.raises(ValueError):second.expected_master_cost.flags.writeable=True
 assert second.diagnostics['r5_source_full_master_hash']==g.expected_master_hash

def test_fresh_source_wrapper_reuses_only_identical_immutable_storage(monkeypatch):
 s,q,mask,g=make(monkeypatch);s.bind_complete_route_scope(q,mask);s._prepare_native(mask)
 first=s._semantic_costmap
 wrapped=replace(g,soft_cost=np.full_like(g.soft_cost,7),hard_semantic_mask=np.zeros_like(mask))
 s.set_semantic_costmap(wrapped);s._prepare_native(mask);second=s._semantic_costmap
 assert second.occupancy_grid is first.occupancy_grid and second.expected_master_cost is first.expected_master_cost
 assert np.all(second.soft_cost==7) and not second.hard_semantic_mask.any()
 assert s._r5_grid_cache[0] is wrapped
 changed=wrapped.expected_master_cost.copy();changed[50,55]=71
 rebound=replace(wrapped,expected_master_cost=seal(changed));s.set_semantic_costmap(rebound);s._prepare_native(mask)
 assert s._semantic_costmap.expected_master_cost is not first.expected_master_cost
 a,b,c,d=s.r5_native_bounds
 assert s._semantic_costmap.expected_master_cost[50-a,55-c]==71

def test_mutable_recovery_preserves_immutable_binding_but_is_never_reused(monkeypatch):
 s,q,mask,g=make(monkeypatch);s.bind_complete_route_scope(q,mask);first=s._prepare_native(mask)
 mutable=mask.copy();mutable[50,55]=False;s._prepare_native(mutable)
 assert s._r5_mask_cache[0] is mask and s._prepare_native(mask) is first
 mutable[0,0]=True
 with pytest.raises(RuntimeError,match='escaped complete'):s._prepare_native(mutable)

@pytest.mark.parametrize('cap',[0.,140.])
def test_sparse_soft_crop_exact_values_rows_and_live_mutations(cap):
 from three_yd_v5.r4_tiles import TiledSoft
 from three_yd_v5.firstcost import soft_positive_rows
 shape=(80,100);drive=np.ones(shape,bool);drive[18:45,42:53]=False
 data=np.arange(36*40,dtype=np.float32).reshape(36,40)%143
 source=TiledSoft(shape,cap,drive,[((12,48,24,64),data)])
 box=(17,57,31,82);cropped=soft_context(source,box)
 assert type(cropped) is TiledSoft and cropped.shape==(40,51)
 assert np.array_equal(np.asarray(cropped),np.asarray(source)[17:57,31:82])
 assert np.array_equal(soft_positive_rows(cropped,5,19),(np.asarray(source)[22:36,31:82]>0))
 assert np.shares_memory(cropped.pieces[0][1],data) and np.shares_memory(cropped.drivable,drive)
 data[:]=9;drive[17:22,31:39]=False
 assert np.array_equal(np.asarray(cropped),np.asarray(source)[17:57,31:82])
 assert np.array_equal(cropped>0,np.asarray(source)[17:57,31:82]>0)
 dense=np.asarray(source);assert np.shares_memory(soft_context(dense,box),dense)

def test_crop_cannot_invent_immutable_source_mask_certificate(monkeypatch):
 s,q,mask,g=make(monkeypatch);g.diagnostics['search_bound_hash']=content_digest(mask)
 s.bind_complete_route_scope(q,mask);local=s._prepare_native(mask)
 assert s._semantic_costmap.diagnostics['search_bound_hash']==content_digest(local)
 narrow=mask.copy();narrow[50,55]=False;s._prepare_native(narrow)
 assert 'search_bound_hash' not in s._semantic_costmap.diagnostics
 assert s._semantic_costmap.diagnostics['r5_source_composed_bound_hash']==content_digest(mask)

def test_escape_rejected_and_old_geometry_ack_cleared(monkeypatch):
 s,q,mask,g=make(monkeypatch);s.bind_complete_route_scope(q,mask);s._prepare_native(mask)
 bad=np.zeros_like(mask);bad[0,0]=True
 with pytest.raises(RuntimeError,match='escaped complete'):s._prepare_native(bad)
 s._exact_ack_cache={'old':'certificate'};s._semantic_ack_cache={'old':'certificate'};s._r6_decoder=object()
 expanded=mask.copy();expanded[5,5]=True;s.bind_complete_route_scope(q,expanded);s._prepare_native(expanded)
 assert not s._exact_ack_cache and not s._semantic_ack_cache and s._r6_decoder is None

def test_old_frame_retry_only_during_explicit_switch(monkeypatch):
 s,q,mask,g=make(monkeypatch);s._r5_switching=True;calls=[]
 s.client=S(executor=S(spin_once=lambda **kw:calls.append(kw)))
 replies=[RuntimeError('costmap geometry mismatch'),(np.zeros((2,3),np.uint8),72)]
 def read(self,deadline):
  reply=replies.pop(0)
  if isinstance(reply,Exception):raise reply
  return reply
 monkeypatch.setattr(StartupDeltaPerformanceSession,'_server_costmap_snapshot',read)
 result=s._server_costmap_snapshot(time.monotonic()+1);assert result[1]==72 and calls and not s._r5_switching
 replies.append(RuntimeError('costmap geometry mismatch'))
 with pytest.raises(RuntimeError,match='geometry mismatch'):s._server_costmap_snapshot(time.monotonic()+1)

def test_binding_change_during_action_and_unbounded_restore_rejected(monkeypatch):
 s,q,mask,g=make(monkeypatch);s._action_in_progress=True
 with pytest.raises(RuntimeError,match='during action'):s.set_semantic_costmap(g)
 with pytest.raises(RuntimeError,match='unbounded base'):s.reset_query_state('task')

def test_only_all_closed_bootstrap_can_precede_task_binding(monkeypatch):
 s,q,mask,g=make(monkeypatch);s._r5_global_grid=None;s._r5_bootstrap_shape=(16,16);s.ctx=S(hospital_map=S(height=16,width=16))
 seen=[]
 monkeypatch.setattr(StartupDeltaPerformanceSession,'update_local_mask',lambda self,m,**kw:(seen.append((m.copy(),kw)) or {'costmap_update_acknowledged':True}))
 result=s.update_local_mask(np.zeros((16,16),bool),force_full=True,initialization=True)
 assert result['r5_bootstrap_all_closed'] and len(seen)==1 and not seen[0][0].any()
 with pytest.raises(ValueError,match='geometry mismatch'):s.update_local_mask(np.ones((16,16),bool),initialization=True)
 with pytest.raises(ValueError,match='geometry mismatch'):s.update_local_mask(np.zeros((16,16),bool))

@pytest.mark.parametrize('fault',[None,'partial','unstable','content','ordinary_fallback','untrusted'])
def test_only_complete_exact_resize_can_drop_emergency_fallback_guard(monkeypatch,fault):
 s,q,mask,g=make(monkeypatch);s.bind_complete_route_scope(q,mask)
 def update(self,local,**kwargs):
  self._r5_switching=False;self._costmap_state_trusted=fault!='untrusted';self._last_update_had_fallback=True
  result={'local_map_update_mode':'full_fallback','local_map_update_fallback_reason':'forced_full_update',
   'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0,'costmap_ack_checked_cells':local.size,
   'semantic_exact_stable_observations':2,'server_costmap_content_hash':'actual','semantic_expected_server_content_hash':'actual'}
  if fault=='partial':result['costmap_ack_checked_cells']-=1
  if fault=='unstable':result['semantic_exact_stable_observations']=1
  if fault=='content':result['server_costmap_content_hash']='different'
  if fault=='ordinary_fallback':result['local_map_update_fallback_reason']='roi_failure'
  return result
 monkeypatch.setattr(StartupDeltaPerformanceSession,'update_local_mask',update)
 result=s.update_local_mask(mask)
 assert s._last_update_had_fallback==(fault is not None)
 assert bool(result.get('r5_complete_resize_exactly_verified'))==(fault is None)

@pytest.mark.parametrize('fault',[None,'untrusted','failed','mismatch','unstable','content','policy','source','master','values','mask','mutable_grid','mutable_mask'])
def test_bookkeeping_alias_requires_actual_current_ack_and_equal_immutable_source(monkeypatch,fault):
 s,q,mask,g=make(monkeypatch);s.bind_complete_route_scope(q,mask);local=s._prepare_native(mask)
 grid=s._semantic_costmap
 s._costmap_state_trusted=fault!='untrusted';s._last_update_had_fallback=True
 s._current_grid=np.flipud(grid.occupancy_grid).copy();s._current_allowed_mask=local.copy()
 old_grid=s._current_grid;old_mask=s._current_allowed_mask
 result={'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0,
  'semantic_exact_stable_observations':2,'server_costmap_content_hash':'actual',
  'semantic_expected_server_content_hash':'actual','semantic_policy_hash':str(grid.policy_hash),
  'semantic_expected_grid_hash':grid.expected_grid_hash,'semantic_expected_master_hash':grid.expected_master_hash}
 if fault=='failed':result['costmap_update_acknowledged']=False
 if fault=='mismatch':result['costmap_ack_mismatch_cells']=1
 if fault=='unstable':result['semantic_exact_stable_observations']=1
 if fault=='content':result['server_costmap_content_hash']='different'
 if fault=='policy':result['semantic_policy_hash']='different'
 if fault=='source':result['semantic_expected_grid_hash']='different'
 if fault=='master':result['semantic_expected_master_hash']='different'
 if fault=='values':s._current_grid[0,0]=0
 if fault=='mask':s._current_allowed_mask[0,0]=True
 if fault=='mutable_grid':grid.occupancy_grid=grid.occupancy_grid.copy()
 if fault=='mutable_mask':local=local.copy()
 s._reuse_verified_bookkeeping(local,result)
 rejected=fault in ('untrusted','failed','mismatch','unstable','content','policy','source','master')
 grid_aliased=not rejected and fault not in ('values','mutable_grid')
 mask_aliased=not rejected and fault not in ('mask','mutable_mask')
 assert bool(result.get('r5_bookkeeping_grid_immutable_view'))==grid_aliased
 assert bool(result.get('r5_bookkeeping_mask_immutable_view'))==mask_aliased
 assert s._last_update_had_fallback
 if grid_aliased:
  assert np.shares_memory(s._current_grid,grid.occupancy_grid) and not s._current_grid.flags.writeable
  assert np.array_equal(s._current_grid,old_grid)
  with pytest.raises(ValueError):s._current_grid.flags.writeable=True
 else:assert s._current_grid is old_grid
 if mask_aliased:
  assert s._current_allowed_mask is local and not local.flags.writeable and np.array_equal(local,old_mask)
 else:assert s._current_allowed_mask is old_mask
