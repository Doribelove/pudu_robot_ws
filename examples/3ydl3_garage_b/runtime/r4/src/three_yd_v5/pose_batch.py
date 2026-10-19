"""Call the original footprint predicate from C for ordered pose alternatives."""
import ctypes
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
_LIB=None


def first_clear(checker, candidates):
    global _LIB
    if _LIB is None:
        _LIB=ctypes.CDLL(str(ROOT/'native/libpose_batch.so'))
        _LIB.first_clear_pose.argtypes=[ctypes.c_size_t,
            np.ctypeslib.ndpointer(dtype=np.uint8,flags='C_CONTIGUOUS'),ctypes.c_int,ctypes.c_int,
            np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS'),ctypes.c_int,
            *([ctypes.c_double]*5),ctypes.POINTER(ctypes.c_int)]
        _LIB.first_clear_pose.restype=ctypes.c_long
    p=np.ascontiguousarray(candidates,dtype=np.float64)
    if p.ndim!=2 or p.shape[1]!=3 or not np.all(np.isfinite(p)):
        raise ValueError('INVALID_BATCH_POSE_CANDIDATES')
    checked=ctypes.c_int(0)
    index=_LIB.first_clear_pose(ctypes.cast(checker.fn,ctypes.c_void_p).value,
        checker.hard,*checker.hard.shape,p,len(p),checker.res,*checker.origin[:2],
        checker.half_length,checker.half_width,ctypes.byref(checked))
    if index < -1:raise RuntimeError('INVALID_NATIVE_POSE_BATCH_DISPATCH')
    return int(index),checked.value
