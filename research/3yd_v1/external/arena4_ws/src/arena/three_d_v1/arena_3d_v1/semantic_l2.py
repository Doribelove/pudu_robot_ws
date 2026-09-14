"""Weighted extension of frozen r1 compact D* and its A* fallback."""
from __future__ import annotations
import math,time,heapq
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from arena_evaluation.semantic_map import canonical_hash
from .l2_state_lifecycle import (CompactDStarState,CompactPersistentCorridorDStar,
 CompactCorridorGeometry,CompactGeometryBinding,MutableStateBinding,VerifiedGeometryCache,
 VerifiedMutableStateCache,CacheTelemetry,DStarSearchStats)
from .l2_incremental import GridAStarResult
from .semantic_world import digest

class WeightedDStarState(CompactDStarState):
 def __init__(self,geometry,binding,potential,weight):
  self.potential=np.ascontiguousarray(potential,dtype=np.float64);self.weight=float(weight)
  if self.potential.shape!=(geometry.state_count,) or not np.all(np.isfinite(self.potential)) or np.any(self.potential<0) or not math.isfinite(self.weight) or self.weight<0:raise ValueError('invalid finite nonnegative costs')
  self.cost_hash=canonical_hash([digest(self.potential),self.weight]);super().__init__(geometry,binding)
 def _edge_cost(self,a,b):
  base=super()._edge_cost(a,b)
  return base*(1+self.weight*(self.potential[a]+self.potential[b])*.5)
 def update_vertex(self,s):
  self.update_vertex_count+=1
  if s!=self.goal_id:
   best=math.inf
   for t in self.geometry.neighbor_ids(s):
    self.neighbor_visit_count+=1;v=self._edge_cost(s,t)+self._g_view[t]
    if v<best:best=v
   self._rhs_view[s]=best
  self._push(s)
 def replace_potential(self,potential):
  p=np.asarray(potential,dtype=float)
  if p.shape!=self.potential.shape or not np.all(np.isfinite(p)) or np.any(p<0):raise ValueError('invalid potential update')
  changed=np.flatnonzero(p!=self.potential);self.potential=p.copy();affected=set(map(int,changed))
  for i in changed:affected.update(self.geometry.neighbor_ids(int(i)))
  self.cache_pristine=False
  for i in sorted(affected):self.update_vertex(i)
  self.cost_hash=canonical_hash([digest(self.potential),self.weight]);return len(changed)
 @property
 def resident_bytes(self):return super().resident_bytes+self.potential.nbytes

def weighted_astar(state,timeout_s=20.):
 """Cold forward A*, consuming the same state._edge_cost as D* Lite."""
 started=time.monotonic();g={state.start_id:0.};parent={};serial=0;expanded=0;generated=1
 q=[(state._heuristic(state.start_id,state.goal_id),0.,serial,state.start_id)]
 timed_out=False;found=False
 if state.blocked[state.start_id] or state.blocked[state.goal_id]:q=[]
 while q:
  if time.monotonic()-started>=timeout_s:timed_out=True;break
  _,cost,_,u=heapq.heappop(q)
  if cost!=g[u]:continue
  if u==state.goal_id:found=True;break
  expanded+=1
  for v in state.geometry.neighbor_ids(u):
   candidate=cost+state._edge_cost(u,v)
   if candidate<g.get(v,math.inf):
    g[v]=candidate;parent[v]=u;serial+=1;generated+=1
    heapq.heappush(q,(candidate+state._heuristic(v,state.goal_id),candidate,serial,v))
 ids=None
 if found:
  ids=[state.goal_id]
  while ids[-1]!=state.start_id:ids.append(parent[ids[-1]])
  ids.reverse()
 path=None if ids is None else [state.geometry.local_cell(i) for i in ids]
 return ids,GridAStarResult(path,g.get(state.goal_id,math.inf) if found else math.inf,expanded,generated,(time.monotonic()-started)*1000,timed_out)

class WeightedCorridorDStar(CompactPersistentCorridorDStar):
 def initialize(self,*,verify_oracle=False):
  if self.state.initialized:
   return self._result(started_ns=time.monotonic_ns(),path_ids=self.state.current_path_ids,failure='' if self.state.current_path_ids else 'L2_NO_PATH',backend='semantic_compact_cache_restore',stats=DStarSearchStats(),reused=True)
  return self._solve(initial=True,verify_oracle=verify_oracle)
 def _solve(self,*,initial=False,verify_oracle=False,force=False,changed=0):
  started=time.monotonic_ns();stats=DStarSearchStats();ids=None;fallback=None
  if (self.state.ready or initial) and not force:
   stats=self.state.compute_shortest_path(timeout_s=20. if initial else self.dstar_wall_budget_ms/1000,max_expansions=None if initial else self.dstar_max_expansions)
   if not stats.timeout_triggered:ids=self.state.extract_path_ids()
   if ids is not None and not self.state.path_is_valid(ids):ids=None;self.state.ready=False
   else:self.state.ready=not stats.timeout_triggered
  else:self.state.ready=False
  if not self.state.ready:
   self.fallback_count+=1;ids,fallback=weighted_astar(self.state)
  self.state.initialized=True
  error=None
  if verify_oracle:
   oi,oracle=weighted_astar(self.state)
   if (ids is None)!=(oi is None):raise AssertionError('weighted reachability mismatch')
   error=0. if ids is None else abs(self.state.path_cost(ids)-oracle.cost)
   if error>1e-9:raise AssertionError(f'weighted cost parity mismatch {error}')
  result=self._result(started_ns=started,path_ids=ids,failure='' if ids is not None else ('L2_TIMEOUT' if fallback and fallback.timeout_triggered else 'L2_UNVERIFIED_NO_PATH'),
   backend='semantic_compact_dstar' if self.state.ready else 'semantic_weighted_astar_fallback',stats=stats,fallback=fallback,changed=changed,reused=not initial,oracle_cost_error=error)
  result.diagnostics.update(actual_cost_hash=self.state.cost_hash,weight=self.state.weight,path_cost=self.state.path_cost(ids) if ids is not None else None)
  return result
 def update(self,blocked_global,*,verify_oracle=False,force_cold_astar=False):
  changed=self.state.set_blocked_ids(self._translate_blocked(blocked_global))
  if not changed:return self.initialize(verify_oracle=verify_oracle)
  return self._solve(verify_oracle=verify_oracle,force=force_cold_astar,changed=changed)
 def change_cost(self,potential):
  changed=self.state.replace_potential(potential)
  return self._solve(changed=changed,force=not self.state.ready)
 def service_resync(self):return self._solve(initial=True)

class SemanticL2Lifecycle:
 HARD_MAX_ACTIVE_STATES=2
 def __init__(self,root,potential,weight=2.):
  self.root=Path(root);self.potential=np.asarray(potential,dtype=float);self.weight=float(weight)
  self.geometry_cache=VerifiedGeometryCache(self.root);self.state_cache=VerifiedMutableStateCache(self.root)
  self.active={};self.max_active_states=1;self.peak_active_state_count=0;self.peak_resident_bytes=0
 @property
 def resident_bytes(self):return sum(x.state_memory_bytes() for x in self.active.values())
 def activate(self,roi,*,dynamic_baseline_version='empty',verify_oracle=False):
  start=time.monotonic();gb=CompactGeometryBinding.from_roi(roi);geometry,gt=self.geometry_cache.restore(gb)
  if geometry is None:geometry=CompactCorridorGeometry.build(roi);self.geometry_cache.save(geometry)
  rows=np.asarray([geometry.global_cell(i) for i in range(geometry.state_count)])
  costs=self.potential[rows[:,0],rows[:,1]]
  version=canonical_hash(['semantic-weighted-r1-v1',digest(costs),self.weight])
  binding=MutableStateBinding(geometry.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,dynamic_baseline_version,algorithm_version=version)
  expected_cost_hash=canonical_hash([digest(np.asarray(costs,dtype=np.float64)),self.weight])
  reusable=binding.digest in self.active and self.active[binding.digest].state.cache_pristine and self.active[binding.digest].state.cost_hash==expected_cost_hash
  if reusable:
   planner=self.active[binding.digest];result=planner.initialize();hit=True;st=CacheTelemetry(True,'',0.)
  else:
   hit=False;self.active.clear();restored,st=self.state_cache.restore(geometry,binding)
   state=WeightedDStarState(geometry,binding,costs,self.weight)
   if restored is not None:
    state.__dict__.update(restored.__dict__);state._refresh_views()
   planner=WeightedCorridorDStar(roi,geometry,state);result=planner.initialize(verify_oracle=verify_oracle)
   self.state_cache.save(state);self.active[binding.digest]=planner
  self.peak_active_state_count=max(self.peak_active_state_count,len(self.active));self.peak_resident_bytes=max(self.peak_resident_bytes,self.resident_bytes)
  telemetry={'active_hit':hit,'geometry_cache_hit':gt.hit,'state_cache_hit':st.hit,'geometry_reject':gt.reject_reason,'state_reject':st.reject_reason,'activate_ms':(time.monotonic()-start)*1000,'cost_hash':planner.state.cost_hash,'resident_bytes':self.resident_bytes}
  return planner,result,SimpleNamespace(as_dict=lambda:telemetry)
 def clear(self):self.active.clear()
