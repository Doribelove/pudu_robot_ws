"""Batch interval counts, retaining original norms near rounding boundaries."""
import math
import numpy as np
from .request_cost import enabled

_COUNTS={'calls':0,'intervals':0,'scalar_boundary_intervals':0}


def interval_data(p,ds,dyaw):
    if (not enabled('I') or type(p) is not np.ndarray or p.dtype!=np.float64
        or p.ndim!=2 or p.shape[1]!=3 or len(p)<2
        or not math.isfinite(ds) or not math.isfinite(dyaw) or ds<=0 or dyaw<=0):return None
    d=np.diff(p,axis=0)
    if not np.isfinite(d).all():return None
    # Preserve the original atan2(sin,cos) calls and their scalar rounding.
    # Even algebraically equivalent angle reductions can change output bits.
    from arena_3d_v1.semantic_world import wrap
    d[:,2]=np.fromiter((wrap(value) for value in d[:,2]),np.float64,count=len(d))
    with np.errstate(over='ignore',under='ignore',invalid='ignore'):
        square=d[:,0]*d[:,0]+d[:,1]*d[:,1]
        ratio=np.sqrt(square)/ds
        yaw_count=np.ceil(np.abs(d[:,2])/dyaw)
    if not np.isfinite(ratio).all() or not np.isfinite(yaw_count).all():return None
    # Huge output/overflow behaviour stays with the original implementation.
    if np.any(ratio>=2**31) or np.any(yaw_count>=2**31):return None
    distance_count=np.ceil(ratio)
    counts=np.maximum(np.maximum(distance_count,yaw_count),1).astype(np.int64)
    # A 2-term nonnegative dot, sqrt and division have a small relative
    # rounding envelope. Do NOT choose a ceil across an integer boundary.
    # Subnormal squares and exact boundaries always use the original norm.
    tolerance=32*np.finfo(np.float64).eps*np.maximum(1.,np.abs(ratio))
    near=(np.abs(ratio-np.rint(ratio))<=tolerance)|(square<4*np.finfo(np.float64).tiny)
    stationary=(d[:,0]==0)&(d[:,1]==0)
    near &= ~stationary
    for i in np.flatnonzero(near):
        counts[i]=max(1,int(math.ceil(np.linalg.norm(d[i,:2])/ds)),int(yaw_count[i]))
    _COUNTS['calls']+=1;_COUNTS['intervals']+=len(d)
    _COUNTS['scalar_boundary_intervals']+=int(near.sum())
    return d,counts


def stats():return dict(_COUNTS)
