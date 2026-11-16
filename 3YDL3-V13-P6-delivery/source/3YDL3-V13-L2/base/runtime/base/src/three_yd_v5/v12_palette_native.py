"""Exact packed integer decoding only; no planning or validation decisions."""
import ctypes
from pathlib import Path
import numpy as np
_lib=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libv12_palette.so'))
_p=ctypes.c_void_p
_lib.v12_palette_mask.argtypes=[_p,ctypes.c_size_t,ctypes.c_size_t,ctypes.c_int,_p,ctypes.c_size_t,_p]
_lib.v12_palette_mask.restype=ctypes.c_int
_lib.v12_palette_points.argtypes=[_p,ctypes.c_size_t,ctypes.c_size_t,_p,ctypes.c_size_t,ctypes.c_int,_p,ctypes.c_size_t,ctypes.c_int,_p]
_lib.v12_palette_points.restype=ctypes.c_int
_lib.v12_mask_needed.argtypes=[_p,ctypes.c_size_t,_p,_p]
_lib.v12_mask_needed.restype=ctypes.c_int
_lib.v12_full_mask.argtypes=[_p,ctypes.c_size_t,_p,ctypes.c_size_t,ctypes.c_size_t,_p]
_lib.v12_full_mask.restype=ctypes.c_int

def mask(packed,bits,values,shape):
    values=np.ascontiguousarray(values,dtype=np.uint8);out=np.empty(shape,np.bool_)
    code=_lib.v12_palette_mask(packed.ctypes.data,packed.nbytes,out.size,bits,values.ctypes.data,len(values),out.ctypes.data)
    if code:raise ValueError('V12_PALETTE_MASK_CODE_RANGE')
    return out

def points(packed,bits,palette,index,cells):
    index=np.ascontiguousarray(index,np.int64);out=np.empty(index.shape,palette.dtype)
    code=_lib.v12_palette_points(packed.ctypes.data,packed.nbytes,cells,index.ctypes.data,index.size,bits,palette.ctypes.data,len(palette),palette.dtype.itemsize,out.ctypes.data)
    if code:raise ValueError('V12_PALETTE_POINT_CODE_RANGE')
    return out

def needed(descriptors,table):
    flags=np.empty(len(descriptors),np.bool_)
    code=_lib.v12_mask_needed(descriptors.ctypes.data,len(descriptors),table.ctypes.data,flags.ctypes.data)
    if code:raise ValueError('V12_PALETTE_DESCRIPTOR')
    return flags

def full_mask(descriptors,table,shape,out=None):
    if out is None:out=np.empty(shape,np.bool_)
    if out.shape!=tuple(shape) or out.dtype!=np.bool_ or not out.flags.c_contiguous:raise ValueError("V12_MASK_OUTPUT")
    code=_lib.v12_full_mask(descriptors.ctypes.data,len(descriptors),table.ctypes.data,*shape,out.ctypes.data)
    if code:raise ValueError('V12_PALETTE_FULL_MASK_CODE_RANGE')
    return out
