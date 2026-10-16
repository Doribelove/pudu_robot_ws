"""Bounded groups of exact L2 pose checks; candidate order and cache unchanged."""
import ctypes,math,os
from pathlib import Path
import numpy as np

_LIB=None
def enabled(): return 'B' in os.environ.get('THREEYD_V10_R8_MODE','control')

def dispatch(checker,centers,yaws,offsets):
    global _LIB
    if (checker.hard.ndim!=2 or checker.hard.dtype!=np.uint8 or
        not checker.hard.flags.c_contiguous):raise ValueError('INVALID_GROUPED_POSE_MAP')
    centers=np.ascontiguousarray(centers,dtype=np.float64)
    yaws=np.ascontiguousarray(yaws,dtype=np.float64)
    offsets=np.ascontiguousarray(offsets,dtype=np.float64)
    if (centers.ndim!=2 or centers.shape[1]!=2 or yaws.ndim!=2 or len(yaws)!=len(centers)
        or yaws.shape[1]<1 or offsets.ndim!=2 or offsets.shape[1]!=2 or not len(offsets)
        or not all(np.all(np.isfinite(x)) for x in (centers,yaws,offsets))):
        raise ValueError('INVALID_GROUPED_POSE_CANDIDATES')
    indices=np.full(len(centers),-1,np.int32);checked=np.zeros(len(centers),np.int32)
    if not len(centers):return indices,checked
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libpose_groups.so'))
        _LIB.first_clear_pose_groups.argtypes=[ctypes.c_size_t,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,
            ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_int,
            *([ctypes.c_double]*5),ctypes.c_void_p,ctypes.c_void_p]
        _LIB.first_clear_pose_groups.restype=ctypes.c_int
    code=_LIB.first_clear_pose_groups(ctypes.cast(checker.fn,ctypes.c_void_p).value,
        checker.hard.ctypes.data,*checker.hard.shape,centers.ctypes.data,yaws.ctypes.data,
        len(centers),yaws.shape[1],offsets.ctypes.data,len(offsets),checker.res,*checker.origin[:2],
        checker.half_length,checker.half_width,indices.ctypes.data,checked.ctypes.data)
    if code!=0:raise RuntimeError('INVALID_NATIVE_GROUPED_POSE_DISPATCH')
    return indices,checked

def certify_many(certifier,cells,preferred_yaws=None,chunk_size=64):
    if chunk_size<1:raise ValueError('INVALID_GROUPED_POSE_CHUNK_SIZE')
    cells=list(cells);yaws=[None]*len(cells) if preferred_yaws is None else list(preferred_yaws)
    if len(yaws)!=len(cells):raise ValueError('POSE_PREFERENCE_COUNT_MISMATCH')
    fine=certifier.world.map.resolution
    offsets=[(0.,0.),(-fine,0.),(fine,0.),(0.,-fine),(0.,fine),
             (-fine,-fine),(-fine,fine),(fine,-fine),(fine,fine)]
    offsets_array=np.asarray(offsets,float)
    result=[]
    for at in range(0,len(cells),chunk_size):
        keys=[tuple(map(int,x)) for x in cells[at:at+chunk_size]]
        pending=[];orders=[];seen=set();order_cache={}
        for cell,preferred in zip(keys,yaws[at:at+chunk_size]):
            if cell in certifier.cache or cell in seen:continue
            seen.add(cell);r,c=cell
            if r<0 or c<0 or r>=certifier.bits.shape[0] or c>=certifier.bits.shape[1]:
                certifier.cache[cell]=None;continue
            bits=int(certifier.bits[r,c]);order_key=(bits,preferred)
            order=order_cache.get(order_key)
            if order is None:
                primary=[k for k in range(certifier.yaw_bins) if bits&(1<<k)]
                order=primary+[k for k in range(certifier.yaw_bins) if k not in primary]
                if preferred is not None:
                    # Same Python scalar operations and stable tie order as certify.
                    order.sort(key=lambda k:abs((2*math.pi*k/certifier.yaw_bins-preferred+math.pi)%(2*math.pi)-math.pi))
                order_cache[order_key]=order
            pending.append(cell);orders.append(order)
        if pending:
            rc=np.asarray(pending,int);centers=certifier.view.world_points(rc[:,0],rc[:,1])
            ordered_yaws=certifier._yaw_values[np.asarray(orders)]
            indices,checked=dispatch(certifier.checker,centers,ordered_yaws,offsets_array)
            # Keep existing logical counters comparable; native group calls are separate.
            certifier.batch_calls+=len(pending);certifier.exact_pose_checks+=int(checked.sum())
            certifier.native_group_calls=getattr(certifier,'native_group_calls',0)+1
            for i,cell in enumerate(pending):
                index=int(indices[i])
                if index<0:certifier.cache[cell]=None;continue
                oi,yi=divmod(index,certifier.yaw_bins);k=orders[i][yi]
                certifier.cache[cell]={'cell':list(cell),'cell_center_xy':centers[i].tolist(),
                    'xy':(centers[i]+offsets_array[oi]).tolist(),'yaw_rad':float(ordered_yaws[i,yi]),
                    'yaw_bin':k,'subcell_offset_xy':list(offsets[oi]),'exact_rectangle_pose':True}
        result.extend(certifier.cache[key] for key in keys)
    return result
