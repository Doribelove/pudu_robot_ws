"""Exact static L2 A* fallback; unsupported or dynamic states stay original."""
import ctypes,math,time
from pathlib import Path
import numpy as np

_LIB=None


def weighted_astar(state,timeout_s=20.):
    from arena_3d_v1.semantic_l2 import weighted_astar as original,GridAStarResult
    from .fast_state import PrecomputedEdgeState
    from .native_cold_search import NativeColdState
    from .tail_options import enabled
    if (not enabled('N') or type(state) not in (PrecomputedEdgeState,NativeColdState)
        or state._has_dynamic_blocks or hasattr(state,'approach_strength')
        or '_edge_cost' in state.__dict__ or '_heuristic' in state.__dict__
        or type(state)._heuristic is not PrecomputedEdgeState._heuristic
        or state.geometry.neighbor_deltas.dtype!=np.int16):
        return original(state,timeout_s=timeout_s)
    started=time.monotonic();n=state.geometry.state_count
    if state.blocked[state.start_id] or state.blocked[state.goal_id]:
        return None,GridAStarResult(None,math.inf,0,1,(time.monotonic()-started)*1000,False)
    # Keep math.hypot's original rounding, including exact tie cases.
    gr,gc=state._rows[state.goal_id],state._cols[state.goal_id]
    heuristic=np.asarray([math.hypot(r-gr,c-gc) for r,c in zip(state._rows,state._cols)],np.float64)
    global _LIB
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libtail_astar.so'))
        array=np.ctypeslib.ndpointer
        _LIB.tail_astar.argtypes=[ctypes.c_int64]*3+[array(dtype=np.int16,flags='C_CONTIGUOUS')]+[array(dtype=np.float64,flags='C_CONTIGUOUS')]*2+[ctypes.c_double,array(dtype=np.int64,flags='C_CONTIGUOUS'),array(dtype=np.uint64,flags='C_CONTIGUOUS'),ctypes.POINTER(ctypes.c_double)]
        _LIB.tail_astar.restype=ctypes.c_int
    ids=np.empty(n,np.int64);stats=np.zeros(4,np.uint64);cost=ctypes.c_double()
    code=_LIB.tail_astar(n,state.start_id,state.goal_id,state.geometry.neighbor_deltas,state.edge_weights,heuristic,
                         max(0.,timeout_s-(time.monotonic()-started)),ids,stats,ctypes.byref(cost))
    if code:raise RuntimeError('R4_NATIVE_ASTAR_FAILED:'+str(code))
    count=int(stats[3]);path_ids=ids[:count].tolist() if count else None
    path=None if path_ids is None else [state.geometry.local_cell(i) for i in path_ids]
    state.r4_native_astar_used=True
    return path_ids,GridAStarResult(path,cost.value,int(stats[0]),int(stats[1]),
                                   (time.monotonic()-started)*1000,bool(stats[2]))
