"""Fuse static edge assembly; unsupported representations use the old path."""
import ctypes, math, itertools, operator
from pathlib import Path
import numpy as np
_LIB=None

def costs(state):
    from arena_3d_v1.l2_state_lifecycle import OFFSETS,OFFSET_INDEX,NO_NEIGHBOR_DELTA
    n=state.geometry.state_count;width=len(OFFSETS)
    delta=state.geometry.neighbor_deltas;direction=state.xy_direction;potential=state.potential
    def supported(a,dtype,shape):
        return type(a) is np.ndarray and a.dtype==np.dtype(dtype) and a.shape==shape and a.flags.c_contiguous
    if (not supported(delta,np.int16,(n,width)) and not supported(delta,np.int16,(n*width,))):return None
    if not supported(direction,np.float64,(n,2)) or not supported(potential,np.float64,(n,)):return None
    stage=getattr(state.geometry,'stage',None)
    if stage is not None:
        if not isinstance(stage,np.ndarray) or stage.dtype.kind not in 'iu' or stage.shape!=(n,):return None
        if stage.dtype.kind=='u' and stage.size and stage.max()>np.iinfo(np.int64).max:return None
        stage=np.ascontiguousarray(stage,dtype=np.int64)
    offsets=np.asarray(OFFSETS,np.int64)
    sides=np.asarray([(OFFSET_INDEX[(0,dc)],OFFSET_INDEX[(dr,0)]) if dr and dc else (0,0) for dr,dc in OFFSETS],np.int64)
    thresholds=np.asarray([-math.sin(math.radians(5))*math.hypot(dr,dc)-1e-6 for dr,dc in OFFSETS],np.float64)
    lengths=np.asarray([math.sqrt(2.) if dr and dc else 1. for dr,dc in OFFSETS],np.float64)
    global _LIB
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/librequest_edges.so'))
        _LIB.request_edges.argtypes=[ctypes.c_int64]*2+[ctypes.c_void_p]*3+[ctypes.c_double]+[ctypes.c_void_p]*5+[ctypes.c_int16,ctypes.c_void_p]
        _LIB.request_edges.restype=ctypes.c_int
    result=np.empty((n,width),np.float64)
    status=_LIB.request_edges(n,width,delta.ctypes.data,direction.ctypes.data,potential.ctypes.data,state.weight,
        0 if stage is None else stage.ctypes.data,offsets.ctypes.data,sides.ctypes.data,thresholds.ctypes.data,lengths.ctypes.data,NO_NEIGHBOR_DELTA,result.ctypes.data)
    if status:raise ValueError('REQUEST_EDGE_INVALID_GEOMETRY:'+str(status))
    return result

def exact_heuristic(rows,cols,row,col):
    # map keeps Python integer subtraction and the exact same math.hypot call;
    # it removes the Python comprehension and intermediate list only.
    return np.fromiter(map(math.hypot,map(operator.sub,itertools.repeat(row),rows),
        map(operator.sub,itertools.repeat(col),cols)),dtype=np.float64,count=len(rows))
