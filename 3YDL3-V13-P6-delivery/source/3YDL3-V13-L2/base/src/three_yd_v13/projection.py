"""Exact circle inflation of dynamic overlay in an aligned dirty bounding box."""
import math
import cv2
import numpy as np
from arena_3d_v1.multires_grid import aggregate

def blocked_state_ids(world,dynamic,rows,cols,factor=3):
    if not dynamic.obstacles:return set(),{'fine_window_cells':0,'full_map_cells':dynamic.mask.size}
    points=np.concatenate(list(dynamic.obstacles.values()),axis=0)
    rr,cc=world.cells(points);radius=math.hypot(*world.vehicle[:2]);res=world.map.resolution
    halo=int(math.ceil(radius/res+.5))+1
    h,w=dynamic.mask.shape
    # GridView groups rows from the physical bottom, not image row zero.
    top_padding=(-h)%factor
    r0=max(0,(int(rr.min())-halo+top_padding)//factor*factor-top_padding)
    r1=min(h,((int(rr.max())+halo+factor+top_padding)//factor)*factor-top_padding)
    c0=max(0,(int(cc.min())-halo)//factor*factor);c1=min(w,((int(cc.max())+halo+factor)//factor)*factor)
    n=int(math.ceil(radius/res+.5));y,x=np.mgrid[-n:n+1,-n:n+1]
    kernel=(np.hypot(np.maximum(abs(x)-.5,0),np.maximum(abs(y)-.5,0))*res<=radius+1e-9).astype(np.uint8)
    patch=np.asarray(dynamic.mask[r0:r1,c0:c1],np.uint8)
    inflated=cv2.dilate(patch,kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0)>0
    coarse=aggregate(inflated,factor,pad_value=False)
    rows=np.asarray(rows);cols=np.asarray(cols)
    sr=rows-(r0+top_padding)//factor;sc=cols-c0//factor
    valid=(sr>=0)&(sc>=0)&(sr<coarse.shape[0])&(sc<coarse.shape[1])
    indices=np.flatnonzero(valid)
    ids=set(map(int,indices[coarse[sr[valid],sc[valid]]]))
    return ids,{'fine_window_cells':patch.size,'full_map_cells':dynamic.mask.size,
                'bbox_rc':[r0,r1,c0,c1],'circle_radius_m':radius,'factor':factor}
