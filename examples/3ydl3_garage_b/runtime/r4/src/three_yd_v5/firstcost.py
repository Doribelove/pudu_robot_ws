"""V8-r11 exact data preparation optimizations; no routing/audit changes."""
import os
import numpy as np


def enabled(part):
    mode=os.environ.get('THREEYD_V8_R11_MODE','control')
    if mode not in ('control','A','B','AB'):
        raise ValueError('Invalid V8-r11 mode: '+mode)
    return mode!='control' and part in mode


def region_membership(labels, ids):
    """Exact membership for small integer ID arrays, including negative IDs.

    A table covers the complete dtype domain, so no range clipping or aliasing
    can admit unknown labels. Wider/custom dtypes use NumPy's original path.
    """
    a=np.asarray(labels)
    if a.dtype.kind not in 'iu' or a.dtype.itemsize>2:
        return np.isin(a,ids)
    limits=np.iinfo(a.dtype)
    table=np.zeros(1 << (8*a.dtype.itemsize),bool)
    values=np.asarray(list(ids))
    if values.size:
        exact=[int(v) for v in values if limits.min<=v<=limits.max and int(v)==v]
        if exact:table[np.asarray(exact,np.int64)]=True
    return table[a]


def neutral_adapter(controller):
    # This algebraic fast path is specific to the frozen occupancy adapter.
    # A custom free-space adapter must retain the general original path.
    from .free_space_topology import FreeSpaceTopology
    return controller.generic_mode and type(controller.topology) is FreeSpaceTopology


def soft_positive_rows(soft, start, end):
    """Read exact soft>0 classification without constructing float rows."""
    from .r4_tiles import TiledSoft
    if type(soft) is not TiledSoft:
        return np.asarray(soft[start:end,:])>0
    shape=(end-start,soft.shape[1])
    result=np.full(shape,soft.cap>0,bool)
    for (a,b,c,d),value in soft.pieces:
        y0,y1=max(start,a),min(end,b)
        if y0<y1:
            result[y0-start:y1-start,c:d]=value[y0-a:y1-a]>0
    result[~soft.drivable[start:end,:]]=True  # Original outside cost is 200.
    return result


def pack_flags(affected, hard, soft, dirty, ordinary, out):
    """Same five bits as original ACK; only bounded row scratch is allocated."""
    out[:]=affected
    scratch=np.empty_like(out)
    for bit,mask in ((2,hard),(4,soft),(8,dirty),(16,ordinary)):
        np.multiply(mask,np.uint8(bit),out=scratch,dtype=np.uint8,casting='unsafe')
        np.bitwise_or(out,scratch,out=out)
