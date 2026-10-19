"""Position-only L2 recovery for places rejected by a circumscribed circle.

The morphology in this module is only a fast candidate generator.  A coarse
cell is opened to L2 only after the original 0.05 m obstacle map accepts at
least one pose of the real rectangular footprint at that cell centre.  The
accepted yaw is diagnostic metadata; it is deliberately not an L2 state.
Continuous motion between the poses remains an L3 responsibility.
"""
import math

import cv2
import numpy as np
from scipy import ndimage

from arena_3d_v1.semantic_world import SweptChecker
from .r3_options import enabled as r3_enabled


def _angle_distance(a, b):
    return abs((a-b+math.pi) % (2*math.pi)-math.pi)


def rectangle_candidate_bits(world, view, fine_hard, yaw_bins=24):
    """Return an optimistic per-cell bit set of collision-free rectangle yaws.

    Pixel morphology can disagree with the continuous checker at a boundary,
    so callers must use :class:`RectanglePoseCertifier` before opening cells.
    """
    if yaw_bins != 24:
        raise ValueError('V5_R1_REQUIRES_24_YAW_CANDIDATES')
    hard=np.asarray(fine_hard,np.uint8)
    bits=np.zeros(view.safe.shape,np.uint32)
    rr,cc=np.indices(view.safe.shape)
    xy=view.world_points(rr.ravel(),cc.ravel())
    fr,fc=world.cells(xy)
    valid=(fr>=0)&(fc>=0)&(fr<world.map.height)&(fc<world.map.width)
    half_length,half_width=map(float,world.vehicle[:2])
    reach=int(math.ceil(math.hypot(half_length,half_width)/world.map.resolution))+2
    size=2*reach+1
    center=np.array([reach,reach],np.float32)
    local=np.array([[-half_length,-half_width],[-half_length,half_width],
                    [half_length,half_width],[half_length,-half_width]],np.float32)/world.map.resolution
    # A rectangle is invariant under pi rotation, hence twelve unique kernels.
    for k in range(yaw_bins//2):
        yaw=2*math.pi*k/yaw_bins
        rotation=np.array([[math.cos(yaw),-math.sin(yaw)],
                           [math.sin(yaw), math.cos(yaw)]],np.float32)
        polygon=np.rint(local@rotation.T+center).astype(np.int32)
        kernel=np.zeros((size,size),np.uint8)
        cv2.fillConvexPoly(kernel,polygon,1)
        occupied=cv2.dilate(hard,kernel,anchor=(reach,reach),
                            borderType=cv2.BORDER_CONSTANT,borderValue=1)>0
        clear=np.zeros(len(fr),bool)
        clear[valid]=~occupied[fr[valid],fc[valid]]
        clear=clear.reshape(bits.shape)
        bits[clear]|=np.uint32((1<<k)|(1<<(k+yaw_bins//2)))
    return bits


class RectanglePoseCertifier:
    """Exact, memoized single-pose certificates on the immutable fine map."""
    def __init__(self, world, fine_hard, view, candidate_bits, yaw_bins=24, cache=None):
        self.world=world;self.view=view;self.bits=np.asarray(candidate_bits,np.uint32)
        self.yaw_bins=yaw_bins
        self.checker=SweptChecker(fine_hard,world.map.resolution,world.map.origin,
                                  *world.vehicle[:2])
        self.cache={} if cache is None else cache
        self.batch_calls=0;self.exact_pose_checks=0
        self._yaw_values=np.array([2*math.pi*k/self.yaw_bins for k in range(self.yaw_bins)])

    def certify(self, cell, preferred_yaw=None):
        cell=tuple(map(int,cell))
        if cell in self.cache:return self.cache[cell]
        r,c=cell
        if r<0 or c<0 or r>=self.bits.shape[0] or c>=self.bits.shape[1]:
            self.cache[cell]=None;return None
        bits=int(self.bits[r,c])
        primary=[k for k in range(self.yaw_bins) if bits&(1<<k)]
        # Morphology is only a candidate accelerator.  Try all remaining yaws
        # before rejecting a cell, avoiding false negatives at pixel borders.
        remaining=[k for k in range(self.yaw_bins) if k not in primary]
        order=primary+remaining
        if preferred_yaw is not None:
            order.sort(key=lambda k:_angle_distance(2*math.pi*k/self.yaw_bins,preferred_yaw))
        center=self.view.world_points(np.array([r]),np.array([c]))[0]
        # A 0.15 m state denotes a cell, not a mandatory centre point.  Check
        # the centre first, then the 0.05 m subcell centres it represents.
        # The returned continuous pose is retained in the certificate.
        fine=self.world.map.resolution
        offsets=[(0.,0.),(-fine,0.),(fine,0.),(0.,-fine),(0.,fine),
                 (-fine,-fine),(-fine,fine),(fine,-fine),(fine,fine)]
        if r3_enabled('A'):
            from .pose_batch import first_clear
            # Preserve the original offset-major / yaw-minor order exactly.
            # No pair of candidates is checked as a continuous trajectory.
            xy=center+np.asarray(offsets,float)
            poses=np.empty((len(offsets)*len(order),3),np.float64)
            poses[:,:2]=np.repeat(xy,len(order),axis=0)
            poses[:,2]=np.tile(self._yaw_values[order],len(offsets))
            index,checked=first_clear(self.checker,poses)
            self.batch_calls+=1;self.exact_pose_checks+=checked
            if index<0:self.cache[cell]=None;return None
            offset_index,yaw_index=divmod(index,len(order));k=order[yaw_index]
            result={'cell':[r,c],'cell_center_xy':center.tolist(),'xy':poses[index,:2].tolist(),
                    'yaw_rad':float(poses[index,2]),'yaw_bin':k,
                    'subcell_offset_xy':list(offsets[offset_index]),'exact_rectangle_pose':True}
            self.cache[cell]=result;return result
        for dx,dy in offsets:
            xy=center+np.array([dx,dy])
            for k in order:
                yaw=2*math.pi*k/self.yaw_bins
                if self.checker.check(np.array([[xy[0],xy[1],yaw]],np.float64)):
                    result={'cell':[r,c],'cell_center_xy':center.tolist(),'xy':xy.tolist(),'yaw_rad':yaw,
                            'yaw_bin':k,'subcell_offset_xy':[dx,dy],'exact_rectangle_pose':True}
                    self.cache[cell]=result;return result
        self.cache[cell]=None
        return None


def path_tangents(path, view):
    """Preferred world yaw at each path cell; used only to order pose checks."""
    path=np.asarray(path,int);xy=view.world_points(path[:,0],path[:,1])
    if len(xy)==1:return np.zeros(1)
    delta=np.empty_like(xy)
    delta[0]=xy[1]-xy[0];delta[-1]=xy[-1]-xy[-2]
    if len(xy)>2:delta[1:-1]=xy[2:]-xy[:-2]
    return np.arctan2(delta[:,1],delta[:,0])


def certify_relaxed_path(path, circle_safe, certifier):
    """Certify every path cell which was unavailable to circle-only L2."""
    path=np.asarray(path,int);tangent=path_tangents(path,certifier.view)
    certificates=[];invalid=[]
    seen=set()
    pending=[];preferred=[]
    for cell,yaw in zip(path,tangent):
        key=tuple(map(int,cell))
        if circle_safe[key] or key in seen:continue
        seen.add(key)
        pending.append(key);preferred.append(float(yaw))
    from .pose_groups import enabled,certify_many
    items=(certify_many(certifier,pending,preferred) if enabled() else
           [certifier.certify(key,yaw) for key,yaw in zip(pending,preferred)])
    for key,item in zip(pending,items):
        if item is None:invalid.append(key)
        else:certificates.append(item)
    return certificates,invalid


def first_component_break(path, safe, view, certificates=None):
    """Locate the first disconnected component transition along a point guide."""
    path=np.asarray(path,int)
    labels,count=ndimage.label(np.asarray(safe,bool),np.ones((3,3),int))
    values=np.asarray([labels[tuple(cell)] for cell in path],int)
    nonzero=np.flatnonzero(values)
    if not len(nonzero):
        return {'found':False,'reason':'NO_CERTIFIED_COMPONENT','component_count':int(count)}
    first=int(nonzero[0]);source=int(values[first]);target_index=None
    for i in nonzero:
        if int(values[i])!=source:
            target_index=int(i);break
    if target_index is None:
        return {'found':False,'reason':'NO_SECOND_COMPONENT_ON_GUIDE','component_count':int(count),'source_component':source}
    source_candidates=np.flatnonzero((np.arange(len(path))<target_index)&(values==source))
    a=int(source_candidates[-1]);b=target_index
    xy=view.world_points(path[:,0],path[:,1])
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    def yaw(i):
        lo=max(0,i-2);hi=min(len(xy)-1,i+2);d=xy[hi]-xy[lo]
        return float(math.atan2(d[1],d[0]))
    certified={tuple(x['cell']):x for x in (certificates or [])}
    def pose(i):
        item=certified.get(tuple(map(int,path[i])))
        return list(map(float,item['xy']))+[float(item['yaw_rad'])] if item is not None else [float(xy[i,0]),float(xy[i,1]),yaw(i)]
    return {'found':True,'component_count':int(count),'source_component':source,
            'target_component':int(values[b]),'source_index':a,'target_index':b,
            'source_cell':path[a].tolist(),'target_cell':path[b].tolist(),
            'source_pose':pose(a),'target_pose':pose(b),
            'source_pose_from_rectangle_certificate':tuple(map(int,path[a])) in certified,
            'target_pose_from_rectangle_certificate':tuple(map(int,path[b])) in certified,
            'guide_gap_m':float(arc[b]-arc[a]),'guide_gap_nodes':int(b-a+1),
            'guide_segment_xy':xy[a:b+1].tolist()}
