"""Same directed weighted D* graph, resumable native queue for both arms."""
import ctypes,time,math,os
from pathlib import Path
import numpy as np
from three_yd_v5.fast_state import PrecomputedEdgeState
from three_yd_v5.native_cold_search import Entry
from arena_evaluation.dstar_lite import DStarSearchStats
from arena_3d_v1.l2_state_lifecycle import OFFSETS,OFFSET_INDEX
_LIB=None

def lib():
    global _LIB
    if _LIB is None:
        _LIB=ctypes.CDLL(__import__('trilayer_p6_runtime.resources',fromlist=['native_path']).native_path('libv13_dstar.so'))
        _LIB.update_affected_rhs.argtypes=[ctypes.c_int64,ctypes.c_void_p,ctypes.c_int64]+[ctypes.c_void_p]*6
        _LIB.update_affected_rhs.restype=ctypes.c_uint64
        _LIB.repair_dstar.argtypes=[ctypes.c_int64]*3+[ctypes.c_void_p]*5+[ctypes.c_double,ctypes.c_int64]+[ctypes.c_void_p]*3+[ctypes.c_int64,ctypes.c_void_p,ctypes.c_uint64,ctypes.c_double,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_void_p]
        _LIB.repair_dstar.restype=ctypes.c_void_p
        _LIB.repair_size.argtypes=[ctypes.c_void_p];_LIB.repair_size.restype=ctypes.c_int64
        _LIB.repair_export.argtypes=[ctypes.c_void_p]*4
        _LIB.repair_trace_size.argtypes=[ctypes.c_void_p];_LIB.repair_trace_size.restype=ctypes.c_int64
        _LIB.repair_trace_export.argtypes=[ctypes.c_void_p]*2
        _LIB.repair_work_export.argtypes=[ctypes.c_void_p]*2
        _LIB.repair_phase_export.argtypes=[ctypes.c_void_p]*2
        _LIB.repair_free.argtypes=[ctypes.c_void_p]
    return _LIB

class NativeIncrementalState(PrecomputedEdgeState):
    def set_blocked_ids(self,ids):
        if (os.environ.get('V13_NATIVE_EVENT_UPDATE','1')!='1' or
            self.geometry.neighbor_deltas.dtype!=np.int16 or
            not hasattr(self,'edge_weights') or hasattr(self,'approach_strength')):
            return super().set_blocked_ids(ids)
        changed=set(map(int,np.flatnonzero(self.blocked))).symmetric_difference(ids)
        if not changed:
            self._has_dynamic_blocks=bool(np.any(self.blocked))
            return 0
        self.cache_pristine=False
        for node in changed:self._blocked_view[node]=node in ids
        affected=set(changed)
        for node in changed:affected.update(self.geometry.neighbor_ids(node))
        ordered=sorted(affected);nodes=np.asarray(ordered,np.int64)
        sides=np.asarray([(OFFSET_INDEX[(0,dc)],OFFSET_INDEX[(dr,0)]) if dr and dc else (-1,-1) for dr,dc in OFFSETS],np.int64)
        self.neighbor_visit_count+=lib().update_affected_rhs(len(nodes),nodes.ctypes.data,self.goal_id,
            self.geometry.neighbor_deltas.ctypes.data,self.edge_weights.ctypes.data,
            self.g.ctypes.data,self.rhs.ctypes.data,self.blocked.ctypes.data,sides.ctypes.data)
        self.update_vertex_count+=len(ordered)
        # All rhs updates depend only on unchanged g; retain exact queue order.
        for node in ordered:self._push(node)
        self.update_count+=1
        self._has_dynamic_blocks=bool(np.any(self.blocked))
        return len(changed)

    def extract_path_ids(self, *, max_length=None):
        from .path_extract import supported, extract
        if os.environ.get('V13_NATIVE_PATH', '1') == '1' and supported(self):
            self.v13_extraction_backend = 'native_equivalent'
            return extract(self, max_length=max_length)
        self.v13_extraction_backend = 'python_original'
        return super().extract_path_ids(max_length=max_length)

    def compute_shortest_path(self,*,timeout_s=None,max_expansions=None):
        if self.geometry.neighbor_deltas.dtype!=np.int16 or hasattr(self,'approach_strength'):
            self.v13_native=False
            return super().compute_shortest_path(timeout_s=timeout_s,max_expansions=max_expansions)
        tick=time.monotonic();dll=lib();n=self.geometry.state_count
        entries=(Entry*len(self.open))(*(Entry(*x) for x in self.open))
        tokens=np.full(n,np.iinfo(np.uint64).max,np.uint64)
        for node,entry in self.queued.items():tokens[node]=entry[2]
        sides=np.asarray([(OFFSET_INDEX[(0,dc)],OFFSET_INDEX[(dr,0)]) if dr and dc else (-1,-1) for dr,dc in OFFSETS],np.int64)
        opposite=np.asarray([OFFSET_INDEX[(-dr,-dc)] for dr,dc in OFFSETS],np.int64)
        smart_rhs=os.environ.get('V13_SMART_RHS','1')=='1'
        skip_unchanged=os.environ.get('V13_SKIP_UNCHANGED','1')=='1'
        indexed=os.environ.get('V13_INDEXED_HEAP','1')=='1'
        initial=len(self.open)
        native_begin=time.monotonic();marshal_input_ms=(native_begin-tick)*1000
        ptr=dll.repair_dstar(n,self.start_id,self.goal_id,self.geometry.neighbor_deltas.ctypes.data,
            self.edge_weights.ctypes.data,self._start_heuristic.ctypes.data,self.g.ctypes.data,self.rhs.ctypes.data,
            -1. if timeout_s is None else max(0.,timeout_s),-1 if max_expansions is None else max(0,max_expansions),
            self.blocked.ctypes.data,sides.ctypes.data,ctypes.cast(entries,ctypes.c_void_p),len(entries),tokens.ctypes.data,self.serial,self.km,int(os.environ.get('V13_CAPTURE_TRACE')=='1'),int(smart_rhs),int(skip_unchanged),int(indexed),opposite.ctypes.data)
        export_begin=time.monotonic()
        if not ptr:raise RuntimeError('V13_NATIVE_DSTAR_INPUT')
        try:
            result=(Entry*dll.repair_size(ptr))();stats=np.zeros(10,np.uint64)
            work=np.empty(7,np.uint64);dll.repair_work_export(ptr,work.ctypes.data)
            native_phases=np.empty(2,np.float64);dll.repair_phase_export(ptr,native_phases.ctypes.data)
            dll.repair_export(ptr,ctypes.cast(result,ctypes.c_void_p),tokens.ctypes.data,stats.ctypes.data)
            self.open=[(e.a,e.b,int(e.token),int(e.id)) for e in result]
            self.queued={e.id:(e.a,e.b,int(e.token)) for e in result if tokens[e.id]==e.token}
            if os.environ.get('V13_CAPTURE_TRACE')=='1':
                self.v13_expansion_trace=np.empty(dll.repair_trace_size(ptr),np.int64)
                dll.repair_trace_export(ptr,self.v13_expansion_trace.ctypes.data)
        finally:dll.repair_free(ptr)
        self.v13_repair_work={'smart_rhs':smart_rhs,'rhs_full_scans':int(work[0]),'fast_relaxations':int(work[1]),'unaffected_predecessors':int(work[2]),'successor_edge_visits':int(stats[6]),'indexed_heap':indexed,'queue_replacements':int(work[4]),'queue_erases':int(work[5]),'initial_stale_pruned':int(work[6]),'skip_unchanged':skip_unchanged,'unchanged_queue_skips':int(work[3])}
        self.v13_search_internal_ms={'marshal_input_ms':marshal_input_ms,'native_setup_ms':float(native_phases[0]),'native_kernel_ms':float(native_phases[1]),'marshal_output_ms':(time.monotonic()-export_begin)*1000,
            'native_call_other_ms':(export_begin-native_begin)*1000-float(native_phases.sum())}
        expanded,generated,pops,pushes,updates,pred,visits,serial,timeout,unique=map(int,stats)
        self.serial=serial;self.queue_push_count+=pushes;self.queue_pop_count+=pops
        self.update_vertex_count+=updates;self.predecessor_visit_count+=pred;self.neighbor_visit_count+=visits
        self.total_expanded_nodes+=expanded;self.v13_native=True;self.v13_unique_expanded=unique
        return DStarSearchStats(expanded_nodes=expanded,generated_nodes=generated,queue_pops=pops,
            queue_pushes=pushes,initial_queue_size=initial,final_queue_size=len(self.open),update_vertex_count=updates,
            search_time_ms=(time.monotonic()-tick)*1000,timeout_triggered=bool(timeout),no_path=not math.isfinite(self.g[self.start_id]))
