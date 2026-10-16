"""Exact bounded-support computations; output retains full global indexing."""
import math
import cv2
import numpy as np
from arena_evaluation.semantic_costmap_composer import internal_soft_to_occupancy,occupancy_to_static_layer
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master

def bbox(mask,halo=0):
    rows=np.flatnonzero(np.any(mask,axis=1));cols=np.flatnonzero(np.any(mask,axis=0))
    if not len(rows):return None
    return (max(0,int(rows[0])-halo),min(mask.shape[0],int(rows[-1])+1+halo),
            max(0,int(cols[0])-halo),min(mask.shape[1],int(cols[-1])+1+halo))

def reference_soft(shape,reference,resolution,drivable,cap=140):
    # Global rasterization is unchanged; only the distance computation is local.
    seed=np.ones(shape,np.uint8)
    cv2.polylines(seed,[reference[:,::-1].astype(np.int32)],False,0,1)
    bounds=bbox(seed==0,math.ceil(2./resolution)+2)
    if bounds is None:raise ValueError('Reference has no cells inside map')
    a,b,c,d=bounds
    distance=cv2.distanceTransform(seed[a:b,c:d],cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*resolution
    soft=np.full(shape,cap,np.float32)
    soft[a:b,c:d]=(cap*np.minimum(1.,distance/2.)).astype(np.float32)
    soft[~drivable]=200.
    return soft,{'soft_roi':[a,b,c,d],'soft_computed_cells':(b-a)*(d-c),'total_cells':seed.size}

def occupancy_chunked(source,soft,rows=96):
    result=np.empty(source.shape,np.int8)
    for a in range(0,len(source),rows):
        b=min(len(source),a+rows)
        result[a:b]=np.maximum(source[a:b].astype(np.int16),internal_soft_to_occupancy(soft[a:b])).astype(np.int8)
        result[a:b][source[a:b]<0]=-1
    return result

def static_chunked(occupancy,rows=96):
    result=np.empty(occupancy.shape,np.uint8)
    for a in range(0,len(occupancy),rows):result[a:a+rows]=occupancy_to_static_layer(occupancy[a:a+rows])
    return result

def bounded_inflation(static,resolution):
    # Everything beyond support is lethal, so outside costs cannot change.
    bounds=bbox(static!=254,math.ceil(.55/resolution)+2)
    if bounds is None:return np.full(static.shape,254,np.uint8),{'inflation_computed_cells':0}
    a,b,c,d=bounds
    result=np.full(static.shape,254,np.uint8)
    result[a:b,c:d]=pinned_nav2_effective_master(static[a:b,c:d],resolution=resolution,
        inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
    return result,{'inflation_roi':[a,b,c,d],'inflation_computed_cells':(b-a)*(d-c)}
