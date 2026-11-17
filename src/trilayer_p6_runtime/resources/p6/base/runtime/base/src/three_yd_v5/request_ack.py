"""Fused exact ACK classification; every actual readback is checked afresh."""
import ctypes
import hashlib
from pathlib import Path
import numpy as np
from .request_cost import enabled
from .r6_ack import grid_digest
from .firstcost import enabled as firstcost_enabled, soft_positive_rows


class ByteView(ctypes.Structure):
    _fields_=[('data',ctypes.c_void_p),('row_stride',ctypes.c_ssize_t),('col_stride',ctypes.c_ssize_t)]


_LIB=None
_COUNTS={'classification_cells':0,'actual_readback_cells':0,'readback_calls':0}


def lib():
    global _LIB
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/librequest_ack.so'))
        _LIB.classify_ack.argtypes=[ctypes.POINTER(ByteView),ctypes.c_void_p,
            ctypes.c_ssize_t,ctypes.c_ssize_t,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p]
        _LIB.classify_ack.restype=None
        _LIB.equal_ack.argtypes=[ctypes.POINTER(ByteView)]*2+[ctypes.c_ssize_t]*2
        _LIB.equal_ack.restype=ctypes.c_int
        _LIB.vector_ack_available.argtypes=[]
        _LIB.vector_ack_available.restype=ctypes.c_int
    return _LIB


def view(a):return ByteView(a.ctypes.data,*a.strides)


def supported(a,dtype,shape):
    return type(a) is np.ndarray and a.dtype==np.dtype(dtype) and a.ndim==2 and a.shape==shape


def initialize(plan,semantic,expected,changed,previous,rows):
    if not enabled('K'):return False
    master=np.flipud(semantic.expected_master_cost)
    hard=np.flipud(semantic.hard_semantic_mask)
    shape=expected.shape
    if (len(shape)!=2 or not min(shape) or rows<1
        or not supported(expected,np.int8,shape) or not supported(changed,bool,shape)
        or not supported(master,np.uint8,shape) or not supported(hard,bool,shape)
        or (previous is not None and not supported(previous,np.uint8,shape))):return False
    dll=lib()
    if not dll.vector_ack_available():return False
    height,width=shape;flags=np.empty(shape,np.uint8)
    names=('affected','hard','soft','stale','ordinary');counts=dict.fromkeys(names,0)
    bbox=[width,height,0,0];affected_hash=hashlib.sha256()
    for y in range(0,height,rows):
        end=min(height,y+rows);sl=slice(y,end)
        soft=(soft_positive_rows(semantic.soft_cost,height-end,height-y)[::-1] if firstcost_enabled('B') else
              np.asarray(semantic.soft_cost[height-end:height-y,:])[::-1]>0)
        if not supported(soft,bool,(end-y,width)):return False
        arrays=[expected[sl],changed[sl],master[sl],hard[sl],soft,
                previous[sl] if previous is not None else changed[sl]]
        descriptors=(ByteView*6)(*(view(a) for a in arrays))
        local_counts=np.zeros(5,np.uint64);local_bbox=np.empty(4,np.int64)
        dll.classify_ack(descriptors,flags[sl].ctypes.data,end-y,width,previous is not None,
                         local_counts.ctypes.data,local_bbox.ctypes.data)
        for name,n in zip(names,local_counts):counts[name]+=int(n)
        affected=(flags[sl]&1)!=0
        affected_hash.update(np.ascontiguousarray(master[sl][affected]))
        if local_counts[3]:
            x0,y0,x1,y1=map(int,local_bbox)
            bbox=[min(bbox[0],x0),min(bbox[1],y+y0),max(bbox[2],x1),max(bbox[3],y+y1)]
    plan.master=master;plan.flags=flags;plan.counts=counts
    plan.dirty_bbox=[bbox[0],bbox[1],bbox[2]-bbox[0],bbox[3]-bbox[1]] if counts['stale'] else [0,0,0,0]
    plan.expected_affected_hash=affected_hash.hexdigest()
    plan.expected_full_hash=grid_digest(master)
    _COUNTS['classification_cells']+=master.size
    return True


def equal(server,master):
    if (not enabled('K') or not supported(master,np.uint8,getattr(server,'shape',None))
        or not supported(server,np.uint8,master.shape)):return None
    a,b=view(server),view(master)
    result=bool(lib().equal_ack(ctypes.byref(a),ctypes.byref(b),*master.shape))
    _COUNTS['readback_calls']+=1
    if result:_COUNTS['actual_readback_cells']+=master.size
    return result


def stats():return dict(_COUNTS)
