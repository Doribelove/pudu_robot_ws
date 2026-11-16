"""Exact pose interpolation reuse; obstacle/vehicle/traffic checks stay live."""
from collections import OrderedDict
import hashlib,inspect,math,os,sys,threading
import numpy as np
from arena_3d_v1 import semantic_world as parent

ORIGINAL=parent.densify
SOURCE_SHA256='6bb2db359c3e27f42b461b4be751e069eec52477093ce45f4c4f47afaa9fcfc2'
_cache=OrderedDict();_bytes=0;_hits=0;_misses=0;_evictions=0;_installed=False
_aliases=[];_lock=threading.RLock()
MAX_BYTES=8*2**20;MAX_ENTRIES=4

def exact_interpolation(p,ds,dyaw):
    # Retain scalar wrap/norm/ceil for identical interval counts, including
    # floating-point boundary cases. Only the per-interval array assembly is
    # batched. Multiplication and division retain their original order.
    from .request_interpolation import interval_data
    prepared=interval_data(p,ds,dyaw)
    if prepared is None:
        d=np.diff(p,axis=0);counts=np.empty(len(d),np.int64)
        for i,row in enumerate(d):
            row[2]=parent.wrap(row[2])
            counts[i]=max(1,int(math.ceil(np.linalg.norm(row[:2])/ds)),int(math.ceil(abs(row[2])/dyaw)))
    else:d,counts=prepared
    starts=np.cumsum(counts)-counts
    index=np.repeat(np.arange(len(d)),counts)
    step=np.arange(int(counts.sum()))-np.repeat(starts,counts)+1
    out=np.empty((len(index)+1,3));out[0]=p[0]
    out[1:]=p[index]+d[index]*step[:,None]/counts[index,None]
    return out

def fast_densify(poses,ds=.025,dyaw=math.pi/180):
    with _lock:return _fast_densify(poses,ds,dyaw)

def _fast_densify(poses,ds,dyaw):
    global _bytes,_hits,_misses,_evictions
    p=np.asarray(poses,dtype=float)
    if (p.ndim!=2 or p.shape[1]!=3 or len(p)<2 or not np.isfinite(p).all()
        or not math.isfinite(ds) or not math.isfinite(dyaw) or ds<=0 or dyaw<=0):
        return ORIGINAL(poses,ds,dyaw)
    p=np.ascontiguousarray(p);raw=p.tobytes()
    key=(p.shape,hashlib.sha256(raw).digest(),float(ds),float(dyaw))
    # Keep the actual source bytes too: even a hash collision cannot reuse
    # the wrong interpolation. Caller mutation is rehashed each invocation.
    hit=_cache.get(key)
    if hit is not None and hit[0]==raw:
        _hits+=1;_cache.move_to_end(key);return hit[1].copy()
    _misses+=1
    snapshot=np.frombuffer(raw,np.float64).reshape(p.shape)
    dense=exact_interpolation(snapshot,ds,dyaw)
    stored=np.frombuffer(dense.tobytes(),np.float64).reshape(dense.shape)
    size=len(raw)+stored.nbytes
    if size<=MAX_BYTES:
        old=_cache.pop(key,None)
        if old is not None:_bytes-=old[2]
        while _cache and (len(_cache)>=MAX_ENTRIES or _bytes+size>MAX_BYTES):
            _,old=_cache.popitem(last=False);_bytes-=old[2];_evictions+=1
        _cache[key]=(raw,stored,size);_bytes+=size
    return dense

def dispatch(poses,ds=.025,dyaw=math.pi/180):
    if os.environ.get('THREEYD_L3_BACKEND','control')=='control':return ORIGINAL(poses,ds,dyaw)
    return fast_densify(poses,ds,dyaw)

def install():
    global _installed
    if parent.densify is not dispatch and (parent.densify is not ORIGINAL
        or hashlib.sha256(inspect.getsource(ORIGINAL).encode()).hexdigest()!=SOURCE_SHA256):
        return False
    parent.densify=dispatch;_installed=True
    # Original traffic modules captured the function at import time. Rebind
    # only aliases still equal to the guarded original; all decisions in
    # their audit methods remain untouched and execute on each request.
    for name in ('three_yd_v4.guidance','three_yd_v5.controller'):
        module=sys.modules.get(name)
        if module is not None and getattr(module,'densify',None) in (ORIGINAL,dispatch):
            module.densify=dispatch
            if name not in _aliases:_aliases.append(name)
    return True

def stats():
    return {'installed':_installed,'guarded_aliases':list(_aliases),'hits':_hits,'misses':_misses,'resident_bytes':_bytes,'entries':len(_cache),
        'evictions':_evictions,'max_bytes':MAX_BYTES,'binding':'exact float64 pose bytes + shape + interpolation step/yaw step',
        'audit_or_ACK_results_cached':False,'each_return_is_fresh_mutable_array':True}

def clear():
    global _bytes,_hits,_misses,_evictions
    with _lock:_cache.clear();_bytes=_hits=_misses=_evictions=0
