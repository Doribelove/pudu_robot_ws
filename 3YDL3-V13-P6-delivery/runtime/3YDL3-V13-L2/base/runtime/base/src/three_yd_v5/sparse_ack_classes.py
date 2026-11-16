"""Exact full-dirty ACK classes: hard cells default, non-hard exceptions.

Classification is snapshotted at construction. Every real server byte is
still compared on every readback; mismatches expand only the requested rows.
"""
import numpy as np

class FullDirtyFlags:
    def __init__(self,shape,index,values):
        self.shape=shape;self.index=index;self.values=values;self.dtype=np.dtype('uint8');self.ndim=2
    @property
    def nbytes(self):return self.index.nbytes+self.values.nbytes
    def __array__(self,dtype=None):
        out=np.full(self.shape,11,np.uint8);out.ravel()[self.index]=self.values
        return out if dtype is None else out.astype(dtype,copy=False)
    def __getitem__(self,key):
        if isinstance(key,slice) and key.step in (None,1):
            a,b,_=key.indices(self.shape[0]);width=self.shape[1]
            lo,hi=np.searchsorted(self.index,[a*width,b*width])
            out=np.full((max(0,b-a),width),11,np.uint8)
            out.ravel()[self.index[lo:hi]-a*width]=self.values[lo:hi]
            return out
        return np.asarray(self)[key]

def initialize(plan,semantic,expected,rows):
    from .r4_tiles import TiledSoft
    from .firstcost import enabled
    from .v9_maps import flipped_digest
    from .tail_options import enabled as tail_enabled
    from .r6_ack import grid_digest
    soft=semantic.soft_cost;master=np.flipud(semantic.expected_master_cost);hard=np.flipud(semantic.hard_semantic_mask)
    if (expected.size<2**20 or not enabled('B') or type(soft) not in (np.ndarray,TiledSoft)
        or hard.dtype!=np.bool_ or expected.ndim!=2 or master.shape!=expected.shape
        or hard.shape!=expected.shape or soft.shape!=expected.shape):return False
    height,width=expected.shape;indices=[];values=[];soft_count=nonhard_count=0
    index_dtype=np.int32 if expected.size<=np.iinfo(np.int32).max else np.int64
    for y in range(0,height,rows):
        end=min(y+rows,height)
        yy,xx=np.nonzero(~(hard[y:end]|(expected[y:end]==100)))
        nonhard_count+=len(yy)
        if nonhard_count>expected.size*.35:return False
        rr=height-1-(yy+y)
        if type(soft) is np.ndarray:positive=soft[rr,xx]>0
        else:
            positive=np.full(len(rr),soft.cap>0,bool)
            for (a,b,c,d),v in soft.pieces:
                if b<=height-end or a>=height-y:continue
                selected=(rr>=a)&(rr<b)&(xx>=c)&(xx<d)
                positive[selected]=v[rr[selected]-a,xx[selected]-c]>0
            positive[~soft.drivable[rr,xx]]=True
        indices.append(((yy+y)*width+xx).astype(index_dtype))
        values.append(np.where(positive,13,25).astype(np.uint8));soft_count+=int(np.count_nonzero(positive))
    plan.master=master;plan.flags=FullDirtyFlags(expected.shape,np.concatenate(indices),np.concatenate(values))
    plan.counts={'affected':int(expected.size),'hard':int(expected.size)-nonhard_count,'soft':soft_count,'stale':int(expected.size),'ordinary':nonhard_count-soft_count}
    plan.dirty_bbox=[0,0,width,height]
    plan.expected_full_hash=flipped_digest(semantic.expected_master_cost) if tail_enabled('D') else grid_digest(master)
    plan.expected_affected_hash=plan.expected_full_hash
    return True
