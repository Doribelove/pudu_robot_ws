"""Necessary center-space certificates for the frozen canonical cell geometry.

This is an offline diagnostic, not an SE(2) route or a replacement PathAudit.
The canonical footprint predicate treats each obstacle center as a disk of
radius h = resolution / sqrt(2). A centered footprint disk of radius r implies
every safe robot center has obstacle-center distance > r+h. Moving from any
point in its raster cell to the cell center changes distance by at most h.
Therefore every cell containing a safe pose must have center distance > r.
Disconnected 8-neighbor components of this *necessary* mask prove there is no
continuous collision-free center path under that canonical predicate.
Connected components do not prove SE(2) reachability.
"""
from __future__ import annotations

import hashlib
import json
import math

import cv2
import numpy as np


def inscribed_radius(footprint):
    # A nondegenerate convex polygon containing the origin is required.
    points=np.asarray(footprint,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or not np.isfinite(points).all():
        raise ValueError('convex centered footprint required')
    crosses=[];radii=[]
    for a,b in zip(points,np.roll(points,-1,axis=0)):
        cross=float(a[0]*b[1]-a[1]*b[0]);length=float(np.linalg.norm(b-a))
        if length==0 or cross==0:raise ValueError('origin must be strictly inside footprint')
        crosses.append(cross);radii.append(abs(cross)/length)
    if not(all(v>0 for v in crosses) or all(v<0 for v in crosses)):
        raise ValueError('origin must be strictly inside footprint')
    sign=1 if crosses[0]>0 else -1
    for a,b,c in zip(points,np.roll(points,-1,axis=0),np.roll(points,-2,axis=0)):
        ab=b-a;bc=c-b
        if sign*(ab[0]*bc[1]-ab[1]*bc[0])<0:raise ValueError('convex footprint required')
    return min(radii)


def necessary_center_mask(occupancy,resolution,footprint,*,max_cells=20000000):
    if resolution!=.05:raise ValueError('frozen 0.05 m resolution required')
    if occupancy.ndim!=2 or occupancy.size>max_cells:
        raise ValueError('OFFLINE_CERTIFICATE_RESOURCE_BOUND')
    radius=inscribed_radius(footprint);extent=math.ceil(radius/resolution)
    y,x=np.mgrid[-extent:extent+1,-extent:extent+1]
    # A tiny inward numerical margin only enlarges the necessary free set.
    kernel=((x*x+y*y)*resolution**2<=(radius-1e-12)**2).astype(np.uint8)
    raw=((occupancy!=100)&(occupancy>=0)).astype(np.uint8)
    return cv2.erode(raw,kernel,borderType=cv2.BORDER_CONSTANT,borderValue=1),radius


def component_certificate(hospital_map,footprint,start,goal,map_hash):
    mask,radius=necessary_center_mask(hospital_map.occupancy,hospital_map.resolution,footprint)
    count,labels=cv2.connectedComponents(mask,connectivity=8)
    cells=[hospital_map.world_to_cell(*p[:2]) for p in (start,goal)]
    components=[int(labels[c]) if c is not None else 0 for c in cells]
    disconnected=not all(components) or components[0]!=components[1]
    binding={'algorithm':'canonical-necessary-center-components-v1','map_hash':map_hash,
             'occupancy_sha256':hashlib.sha256(memoryview(np.ascontiguousarray(hospital_map.occupancy))).hexdigest(),
             'resolution':hospital_map.resolution,'origin':list(hospital_map.origin),
             'shape':list(mask.shape),'footprint':[list(v) for v in footprint],
             'inscribed_radius_m':radius,'canonical_obstacle_disk_radius_m':hospital_map.resolution/math.sqrt(2),
             'cell_position_offset_bound_m':hospital_map.resolution/math.sqrt(2),
             'start':list(start),'goal':list(goal),'cells':cells,'components':components,
             'component_count':count-1,'necessary_mask_sha256':hashlib.sha256(memoryview(mask)).hexdigest(),
             'verdict':'CANONICAL_STATIC_DISCONNECTED' if disconnected else 'INCONCLUSIVE_CONNECTED_OVERAPPROXIMATION',
             'scope':'Unrestricted continuous center path under frozen canonical obstacle-cell disk envelope; not physical-environment infeasibility.'}
    binding['hash']=hashlib.sha256(json.dumps(binding,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    return binding
