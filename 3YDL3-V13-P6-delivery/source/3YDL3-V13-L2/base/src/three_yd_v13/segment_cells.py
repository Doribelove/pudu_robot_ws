"""Only accelerate conservative line-cell enumeration; keep downstream checks."""
import ctypes
import os
from pathlib import Path
import numpy as np
from three_yd_v5.geometry import segment_cells as original_segment_cells

_LIB=None


def segment_cells(a,b):
    global _LIB
    if os.environ.get('V13_NATIVE_SEGMENTS','1')!='1':
        return original_segment_cells(a,b)
    a=np.asarray(a,dtype=np.float64);b=np.asarray(b,dtype=np.float64)
    if (a.shape!=(2,) or b.shape!=(2,) or not np.all(np.isfinite([a,b]))
            or np.max(np.abs([a,b]))>2**30):
        return original_segment_cells(a,b)
    a=np.ascontiguousarray(a);b=np.ascontiguousarray(b)
    n=max(2,int(np.max(abs(b-a))*4)+1)
    t=np.linspace(0,1,n)
    out=np.empty((3*n,2),dtype=np.int64)
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libv13_segment_cells.so'))
        _LIB.segment_cells.argtypes=[ctypes.c_void_p]*3+[ctypes.c_int64,ctypes.c_void_p]
        _LIB.segment_cells.restype=ctypes.c_int64
    count=_LIB.segment_cells(a.ctypes.data,b.ctypes.data,t.ctypes.data,n,out.ctypes.data)
    return out[:count]
