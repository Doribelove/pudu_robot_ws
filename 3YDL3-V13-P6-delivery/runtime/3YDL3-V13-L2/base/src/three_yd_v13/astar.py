"""Fresh A* baseline. Retains graph inputs, never retains g/parent/OPEN."""
import ctypes,math,os,time
from pathlib import Path
import numpy as np
from three_yd_v5.fast_state import PrecomputedEdgeState
from three_yd_v5.firstplan import state_coordinates
from three_yd_v5.request_edges import exact_heuristic
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.semantic_world import digest
from arena_3d_v1.l2_state_lifecycle import OFFSETS,OFFSET_INDEX,DStarSearchStats
from arena_3d_v1.semantic_l2 import GridAStarResult
from .search import EventDStar
_LIB=None

class AStarGraph(PrecomputedEdgeState):
    def __init__(self,geometry,binding,potential,weight,directions,progress):
        self.geometry=geometry;self.binding=binding
        self.start_id=geometry.state_id_from_global(binding.start_cell);self.goal_id=geometry.state_id_from_global(binding.goal_cell)
        self.potential=np.ascontiguousarray(potential,dtype=np.float64);self.weight=float(weight)
        if self.potential.shape!=(geometry.state_count,) or not np.all(np.isfinite(self.potential)) or np.any(self.potential<0) or not math.isfinite(self.weight) or self.weight<0:raise ValueError('invalid costs')
        self.cost_hash=canonical_hash([digest(self.potential),self.weight])
        self.xy_direction=np.asarray(directions,float);self.progress=np.asarray(progress,np.int16)
        self.cells=state_coordinates(geometry)
        self._rows=memoryview(self.cells[:,0].copy());self._cols=memoryview(self.cells[:,1].copy())
        self._dx=memoryview(self.xy_direction[:,0].copy());self._dy=memoryview(self.xy_direction[:,1].copy())
        self.blocked=np.zeros(geometry.state_count,bool);self._blocked_view=memoryview(self.blocked)
        self.current_path_ids=None;self.ready=True;self.initialized=False;self.reinitialize_count=0
        self.predecessor_visit_count=0;self.neighbor_visit_count=0;self.cache_pristine=True
        self._rebuild_costs()
    def set_blocked_ids(self,ids):
        new=np.zeros_like(self.blocked)
        if ids:new[list(ids)]=True
        changed=int(np.count_nonzero(new!=self.blocked));self.blocked[:]=new
        self._has_dynamic_blocks=bool(ids);self.cache_pristine=False
        return changed
    @property
    def resident_bytes(self):
        # Same component scope as legacy D*: inputs not counted there remain excluded here.
        return self.blocked.nbytes+self.potential.nbytes+self.edge_weights.nbytes+(0 if self.current_path_ids is None else 4*len(self.current_path_ids))

def fresh_search(state,seconds):
    global _LIB
    tick=time.monotonic();n=state.geometry.state_count
    if state.geometry.neighbor_deltas.dtype!=np.int16 or hasattr(state,'approach_strength'):raise ValueError('UNSUPPORTED_FRESH_ASTAR_GRAPH')
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libv13_astar.so'))
        _LIB.fresh_astar.argtypes=[ctypes.c_int64]*3+[ctypes.c_void_p]*5+[ctypes.c_double]+[ctypes.c_void_p]*4+[ctypes.c_int64]
        _LIB.fresh_astar.restype=ctypes.c_int
    h=exact_heuristic(state._rows,state._cols,state._rows[state.goal_id],state._cols[state.goal_id])
    sides=np.asarray([(OFFSET_INDEX[(0,dc)],OFFSET_INDEX[(dr,0)]) if dr and dc else (-1,-1) for dr,dc in OFFSETS],np.int64)
    path=np.empty(n,np.int64);stats=np.zeros(7,np.uint64);cost=ctypes.c_double()
    capture=os.environ.get('V13_CAPTURE_TRACE')=='1';trace=np.empty(n*4,np.int64) if capture else np.empty(0,np.int64)
    code=_LIB.fresh_astar(n,state.start_id,state.goal_id,state.geometry.neighbor_deltas.ctypes.data,state.edge_weights.ctypes.data,h.ctypes.data,state.blocked.ctypes.data,sides.ctypes.data,max(0,seconds-(time.monotonic()-tick)),path.ctypes.data,stats.ctypes.data,ctypes.byref(cost),trace.ctypes.data if capture else None,len(trace))
    if code:raise RuntimeError('FRESH_ASTAR_NATIVE:'+str(code))
    ids=path[:int(stats[3])].tolist() if stats[3] else None
    if capture:
        if stats[0]>len(trace):raise RuntimeError('ASTAR_TRACE_OVERFLOW')
        state.v13_expansion_trace=trace[:int(stats[0])].copy()
    state.v13_unique_expanded=int(stats[4]);state.v13_queue_peak=int(stats[5]);state.v13_queue_pops=int(stats[6])
    return ids,GridAStarResult(None if ids is None else [state.geometry.local_cell(i) for i in ids],cost.value,int(stats[0]),int(stats[1]),(time.monotonic()-tick)*1000,bool(stats[2]))

class FreshAStar(EventDStar):
    def initialize(self,*,verify_oracle=False):return self._solve(initial=True,verify_oracle=verify_oracle)
    def event_solve(self,changed):return self._solve(changed=changed)
    def _solve(self,*,initial=False,verify_oracle=False,force=False,changed=0):
        tick=time.monotonic_ns();phase=tick;phases={}
        ids,stats=fresh_search(self.state,8. if initial else 2.5)
        phases['search_call_ms']=(time.monotonic_ns()-phase)/1e6;phase=time.monotonic_ns()
        phases['path_extract_ms']=0.0
        self.state.initialized=True;self.state.ready=not stats.timeout_triggered
        if ids is not None and not self.state.path_is_valid(ids):raise AssertionError('INVALID_ASTAR_PATH')
        phases['path_validate_ms']=(time.monotonic_ns()-phase)/1e6;phase=time.monotonic_ns()
        result=self._result(started_ns=tick,path_ids=ids,failure='' if ids else ('L2_TIMEOUT' if stats.timeout_triggered else 'L2_UNVERIFIED_NO_PATH'),backend='v13_native_fresh_astar',stats=DStarSearchStats(),fallback=stats,changed=changed,reused=False,oracle_cost_error=None)
        phases['result_build_ms']=(time.monotonic_ns()-phase)/1e6;phase=time.monotonic_ns()
        result.diagnostics.update(path_cost=None if ids is None else self.state.path_cost(ids),dstar_expanded=0,astar_expanded=stats.expanded_nodes,search_state_reused=False,astar_is_primary=True,actual_cost_hash=self.state.cost_hash)
        phases['path_cost_ms']=(time.monotonic_ns()-phase)/1e6
        total=(time.monotonic_ns()-tick)/1e6
        phases['unaccounted_ms']=total-sum(phases.values())
        result.diagnostics.update(phase_ms=phases,solve_wall_ms=total,extraction_backend='astar_parent_chain_inside_search')
        return result
