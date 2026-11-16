"""Exact target-mask intersection with one owned temporary buffer."""
from functools import lru_cache
import hashlib,inspect
import numpy as np
from .v9_maps import immutable

ORIGINAL_SHA256='fdcf986f4545e6e74a7694cb6d0a26cc72133aab18a58abb821665c60817d81c'
@lru_cache(maxsize=8)
def original_supported(function):
    try:return hashlib.sha256(inspect.getsource(function).encode()).hexdigest()==ORIGINAL_SHA256
    except (TypeError,OSError):return False


def build(controller,original):
    corridor=controller.plan.corridor_mask;search=controller._search_mask
    dynamic=None if controller.dynamic is None else controller.dynamic.mask
    remainder=controller.r13_remainder_mask
    from .compact_auxiliary import PackedHardMask
    if (type(search) is PackedHardMask and original_supported(original)
        and all(type(v) is np.ndarray and v.dtype==np.dtype(bool) and v.shape==search.shape
                for v in (corridor,dynamic,remainder) if v is not None)):
        # Intersect original bits before decoding the one required target output.
        # This avoids repeatedly decoding the search field into boolean scratch.
        packed=np.packbits(corridor,axis=1,bitorder='big')
        np.bitwise_and(packed,search.packed,out=packed)
        zero=(dynamic is not None and dynamic.size>0 and all(v==0 for v in dynamic.strides)
              and immutable(dynamic) and not bool(dynamic[0,0]))
        if dynamic is not None and not zero:
            bits=np.packbits(dynamic,axis=1,bitorder='big')
            np.bitwise_not(bits,out=bits);np.bitwise_and(packed,bits,out=packed)
        if remainder is not None:
            np.bitwise_and(packed,np.packbits(remainder,axis=1,bitorder='big'),out=packed)
        return np.unpackbits(packed,axis=1,count=search.shape[1],bitorder='big').view(bool)
    arrays=[a for a in [corridor,search,dynamic,remainder] if a is not None]
    supported=(original_supported(original) and arrays and all(type(a) is np.ndarray
        and a.dtype==np.dtype(bool) and a.ndim==2 and a.shape==corridor.shape for a in arrays))
    if not supported:
        result=original(controller)&search
        return result if remainder is None else result&remainder
    zero=(dynamic is not None and dynamic.size>0 and all(s==0 for s in dynamic.strides)
          and immutable(dynamic) and not bool(dynamic[0,0]))
    if dynamic is None or zero:
        result=np.bitwise_and(corridor,search)
    else:
        result=np.bitwise_not(dynamic)
        np.bitwise_and(corridor,result,out=result)
        np.bitwise_and(result,search,out=result)
    if remainder is not None:np.bitwise_and(result,remainder,out=result)
    return result
