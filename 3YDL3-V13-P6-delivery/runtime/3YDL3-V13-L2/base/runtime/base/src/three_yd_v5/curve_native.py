"""Exact native acceleration of ordered convex rectangle construction."""
from pathlib import Path
import ctypes,time
import numpy as np
_root=Path(__file__).resolve().parents[2]
_lib=ctypes.CDLL(str(_root/'native/libcurve_rectangles.so'))
_ptr=ctypes.c_void_p
_lib.curve_rectangles.argtypes=[_ptr,ctypes.c_int,ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double,
 _ptr,_ptr,ctypes.c_int,_ptr,ctypes.c_int,ctypes.c_double,_ptr,_ptr]
_lib.curve_rectangles.restype=ctypes.c_int
_offsets=np.array([(a,b) for a in np.arange(-.45,.451,.075) for b in np.arange(-.45,.451,.075)],float)
_offsets=np.ascontiguousarray(_offsets[np.argsort(np.linalg.norm(_offsets,axis=1))])
def rectangles(hard,res,origin,guide,deadline,tangents=None):
 from .curve_l3 import CurveFailure
 hard=np.asarray(hard)
 if hard.dtype==np.dtype(bool) and hard.flags.c_contiguous:hard=hard.view(np.uint8)
 hard=np.ascontiguousarray(hard,dtype=np.uint8);p=np.ascontiguousarray(guide,dtype=np.float64)
 t=np.ascontiguousarray(np.tile([1.,0.],(len(p),1)) if tangents is None else tangents,dtype=np.float64)
 out=np.empty((len(p),4));axes=np.empty_like(out)
 code=_lib.curve_rectangles(hard.ctypes.data,*hard.shape,float(res),*map(float,origin[:2]),p.ctypes.data,t.ctypes.data,
  len(p),_offsets.ctypes.data,len(_offsets),max(0.,deadline-time.monotonic()),out.ctypes.data,axes.ctypes.data)
 if code:raise CurveFailure('CURVE_BUDGET' if code==1 else 'CURVE_GUIDE_OUTSIDE_SCOPE')
 return out,axes
