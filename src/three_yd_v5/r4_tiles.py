"""Finite-support cost computation on route tiles, retaining global grid coordinates.

Occupancy and the ACK-facing master remain dense byte arrays for the unchanged
Nav2 boundary. The float reference field is stored only where it varies. No
obstacle, halo, publication or independent audit is omitted.
"""
import math
import cv2
import numpy as np
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master

def blocks(mask,tile=256,neighbors=0):
    h,w=mask.shape;ny=(h+tile-1)//tile;nx=(w+tile-1)//tile
    # Reduce without constructing world coordinates or padded full-size data.
    from .v9_r8_options import enabled as r8_enabled
    small=None
    if r8_enabled('R'):
        from .v9_r8_blocks import tile_support
        small=tile_support(mask,tile)
    if small is None:
        small=np.logical_or.reduceat(np.logical_or.reduceat(mask,np.arange(0,h,tile),axis=0),np.arange(0,w,tile),axis=1)
    if neighbors:
        small=cv2.dilate(small.astype(np.uint8),np.ones((2*neighbors+1,2*neighbors+1),np.uint8))>0
    return [(y*tile,min(h,(y+1)*tile),x*tile,min(w,(x+1)*tile)) for y,x in zip(*np.nonzero(small))]

class TiledSoft:
    def __init__(self,shape,cap,drivable,pieces):
        self.shape=tuple(shape);self.cap=float(cap);self.drivable=drivable;self.pieces=pieces
        self.dtype=np.dtype(np.float32);self.ndim=2
    @property
    def nbytes(self):return sum(x.nbytes for _,x in self.pieces)
    def __array__(self,dtype=None):
        result=np.full(self.shape,self.cap,np.float32)
        for (a,b,c,d),value in self.pieces:result[a:b,c:d]=value
        result[~self.drivable]=200.
        return result if dtype is None else result.astype(dtype,copy=False)
    def __getitem__(self,key):
        if not isinstance(key,tuple):key=(key,slice(None))
        if len(key)!=2:raise IndexError('Two dimensional soft field')
        if all(isinstance(k,slice) and (k.step is None or k.step==1) for k in key):
            a,b,_=key[0].indices(self.shape[0]);c,d,_=key[1].indices(self.shape[1])
            result=np.full((max(0,b-a),max(0,d-c)),self.cap,np.float32)
            for (p,q,r,s),value in self.pieces:
                y0,y1=max(a,p),min(b,q);x0,x1=max(c,r),min(d,s)
                if y0<y1 and x0<x1:result[y0-a:y1-a,x0-c:x1-c]=value[y0-p:y1-p,x0-r:x1-r]
            result[~self.drivable[a:b,c:d]]=200.
            return result
        if any(isinstance(k,slice) for k in key):return np.asarray(self)[key]
        rr,cc=np.broadcast_arrays(np.asarray(key[0]),np.asarray(key[1]))
        result=np.full(rr.shape,self.cap,np.float32)
        for (a,b,c,d),value in self.pieces:
            selected=(rr>=a)&(rr<b)&(cc>=c)&(cc<d)
            result[selected]=value[rr[selected]-a,cc[selected]-c]
        result[~self.drivable[rr,cc]]=200.
        return result
    def __gt__(self,value):
        result=np.full(self.shape,self.cap>value,bool)
        for (a,b,c,d),item in self.pieces:result[a:b,c:d]=item>value
        result[~self.drivable]=200.>value
        return result
    def max(self):
        # cap and 200 are upper bounds attained outside support; exact max is
        # used only for diagnostics. Include every piece for arbitrary caps.
        return max([self.cap]+([200.] if np.any(~self.drivable) else [])+[float(v.max()) for _,v in self.pieces])

def reference_soft(shape,reference,resolution,drivable,cap=140,tile=256):
    seed=np.ones(shape,np.uint8)
    cv2.polylines(seed,[reference[:,::-1].astype(np.int32)],False,0,1)
    halo=math.ceil(2./resolution)+2
    boxes=blocks(seed==0,tile,math.ceil(halo/tile))
    if not boxes:raise ValueError('Reference has no cells inside map')
    # OpenCV's precise EDT has coordinate-size-dependent rounding on long
    # maps. Smaller tiles can change quantized soft bytes. Keep the original
    # EDT domain; sparsify only retained values. Inflation remains tiled.
    from .r11_fields import bbox
    bounds=bbox(seed==0,halo)
    y0,y1,x0,x1=bounds
    distance=cv2.distanceTransform(seed[y0:y1,x0:x1],cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
    pieces=[];cells=distance.size
    for a,b,c,d in boxes:
        aa,bb=max(a,y0),min(b,y1);cc,dd=max(c,x0),min(d,x1)
        value=np.full((b-a,d-c),cap,np.float32)
        if aa<bb and cc<dd:
            value[aa-a:bb-a,cc-c:dd-c]=(cap*np.minimum(1.,distance[aa-y0:bb-y0,cc-x0:dd-x0]/2.)).astype(np.float32)
        if np.any(value!=cap):pieces.append(((a,b,c,d),value))
    soft=TiledSoft(shape,cap,drivable,pieces)
    return soft,{'r4_soft_tiles':len(pieces),'r4_soft_resident_bytes':soft.nbytes,
                 'soft_computed_cells':cells,'total_cells':int(np.prod(shape)),'soft_storage':'original EDT domain; sparse retained float tiles + scalar','r4_edt_domain_preserved':True}

def bounded_inflation(static,resolution,tile=256):
    result=np.full(static.shape,254,np.uint8)
    halo=math.ceil(.55/resolution)+2;cells=0;boxes=blocks(static!=254,tile)
    for a,b,c,d in boxes:
        y0,y1=max(0,a-halo),min(static.shape[0],b+halo);x0,x1=max(0,c-halo),min(static.shape[1],d+halo)
        expanded=pinned_nav2_effective_master(static[y0:y1,x0:x1],resolution=resolution,
            inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
        result[a:b,c:d]=expanded[a-y0:b-y0,c-x0:d-x0];cells+=expanded.size
    return result,{'r4_inflation_tiles':len(boxes),'inflation_computed_cells':cells}
