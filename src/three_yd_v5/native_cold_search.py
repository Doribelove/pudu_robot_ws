"""Identical weighted D* first search, native loop and resumable Python state."""
import ctypes,time,math,heapq
from pathlib import Path
import numpy as np
from arena_evaluation.dstar_lite import DStarSearchStats
from .fast_state import PrecomputedEdgeState
class Entry(ctypes.Structure):
    _fields_=[('a',ctypes.c_double),('b',ctypes.c_double),('token',ctypes.c_uint64),('id',ctypes.c_int64)]
_LIB=None

def lib():
    global _LIB
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libcold_dstar.so'))
        a=np.ctypeslib.ndpointer
        _LIB.cold_dstar.argtypes=[ctypes.c_int64]*3+[a(dtype=np.int16,flags='C_CONTIGUOUS')]+[a(dtype=np.float64,flags='C_CONTIGUOUS')]*4+[ctypes.c_double,ctypes.c_int64]
        _LIB.cold_dstar.restype=ctypes.c_void_p
        _LIB.cold_size.argtypes=[ctypes.c_void_p];_LIB.cold_size.restype=ctypes.c_int64
        _LIB.cold_export.argtypes=[ctypes.c_void_p,ctypes.POINTER(Entry),a(dtype=np.uint64,flags='C_CONTIGUOUS'),a(dtype=np.uint64,flags='C_CONTIGUOUS')]
        _LIB.cold_free.argtypes=[ctypes.c_void_p]
    return _LIB

class NativeColdState(PrecomputedEdgeState):
    def compute_shortest_path(self,*,timeout_s=None,max_expansions=None):
        if (self.initialized or self.serial!=1 or len(self.open)!=1 or self.km!=0.
            or self._has_dynamic_blocks or hasattr(self,'approach_strength')
            or self.geometry.neighbor_deltas.dtype!=np.int16):
            return super().compute_shortest_path(timeout_s=timeout_s,max_expansions=max_expansions)
        started=time.monotonic();dll=lib();n=self.geometry.state_count
        ptr=dll.cold_dstar(n,self.start_id,self.goal_id,self.geometry.neighbor_deltas,self.edge_weights,
            self._start_heuristic,self.g,self.rhs,-1. if timeout_s is None else max(0.,timeout_s),-1 if max_expansions is None else max(0,max_expansions))
        if not ptr:raise RuntimeError('NATIVE_DSTAR_INVALID_INPUT')
        try:
            entries=(Entry*dll.cold_size(ptr))();tokens=np.empty(n,np.uint64);stats=np.zeros(9,np.uint64)
            dll.cold_export(ptr,entries,tokens,stats)
            self.open=[(e.a,e.b,int(e.token),int(e.id)) for e in entries]
            self.queued={e.id:(e.a,e.b,int(e.token)) for e in entries if tokens[e.id]==e.token}
        finally:dll.cold_free(ptr)
        expanded,generated,pops,pushes,updates,pred,visits,serial,timeout=map(int,stats)
        self.serial=serial;self.queue_push_count+=pushes;self.queue_pop_count+=pops
        self.update_vertex_count+=updates;self.predecessor_visit_count+=pred;self.neighbor_visit_count+=visits
        self.total_expanded_nodes+=expanded;self.native_cold_used=True
        return DStarSearchStats(expanded_nodes=expanded,generated_nodes=generated,queue_pops=pops,
            queue_pushes=pushes,initial_queue_size=1,final_queue_size=len(self.open),update_vertex_count=updates,
            search_time_ms=(time.monotonic()-started)*1000,timeout_triggered=bool(timeout),no_path=not math.isfinite(self.g[self.start_id]))
