"""Cover every 4-connected safe ROI pixel and bridge overlap components.

All added convex boxes remain in the conservative safe mask. This guarantees
only finite raster connectivity, never SE(2) trajectory completeness.
"""
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components


def grow_core(mask, box, cap=400):
    x0,y0,x1,y1=map(int,box)
    if x0<0 or y0<0 or x1>mask.shape[1] or y1>mask.shape[0] or not mask[y0:y1,x0:x1].all():
        raise ValueError('UNSAFE_REPAIR_CORE')
    original=(x0,y0,x1,y1)
    while True:
        before=(x0,y0,x1,y1)
        if x0>max(0,original[0]-cap) and mask[y0:y1,x0-1].all():x0-=1
        if x1<min(mask.shape[1],original[2]+cap) and mask[y0:y1,x1].all():x1+=1
        if y0>max(0,original[1]-cap) and mask[y0-1,x0:x1].all():y0-=1
        if y1<min(mask.shape[0],original[3]+cap) and mask[y1,x0:x1].all():y1+=1
        if before==(x0,y0,x1,y1):return before


def components(rects):
    from .spatial import box_pairs
    b=np.asarray(rects,int);n=len(b);pairs,_=box_pairs(b,epsilon=0)
    rows=pairs[:,0];cols=pairs[:,1]
    return connected_components(csr_matrix((np.ones(len(rows)),(rows,cols)),shape=(n,n)),directed=False)


def repair_cover(mask,rects):
    rects=list(rects);original=len(rects);owner=np.full(mask.shape,-1,dtype=np.int32)
    for i,(a,b,c,d) in enumerate(rects):owner[b:d,a:c]=i
    missing_before=int(np.sum(mask & (owner<0)));fill=[]
    # Raster scan over all missing pixels; no route search or route input.
    for y,x in np.argwhere(mask & (owner<0)):
        if owner[y,x]>=0:continue
        box=grow_core(mask,(x,y,x+1,y+1));a,b,c,d=box
        owner[b:d,a:c]=len(rects);rects.append(box);fill.append(box)
    count,labels=components(rects);parent=list(range(count))
    def find(x):
        while parent[x]!=x:parent[x]=parent[parent[x]];x=parent[x]
        return x
    pixel_labels=np.full(mask.shape,-1,dtype=np.int32)
    pixel_labels[mask]=labels[owner[mask]];bridges=[]
    for dy,dx in [(0,1),(1,0)]:
        first=pixel_labels[:mask.shape[0]-dy,:mask.shape[1]-dx];second=pixel_labels[dy:,dx:]
        for y,x in np.argwhere((first>=0)&(second>=0)&(first!=second)):
            if find(int(first[y,x]))==find(int(second[y,x])):continue
            box=grow_core(mask,(x,y,x+dx+1,y+dy+1));a,b,c,d=box
            ids=np.unique(pixel_labels[b:d,a:c]);ids=ids[ids>=0];anchor=find(int(ids[0]))
            for ident in ids:parent[find(int(ident))]=anchor
            rects.append(box);bridges.append(box)
    # Independently recompute final graph instead of trusting union bookkeeping.
    final_count,final_labels=components(rects)
    for i,(a,b,c,d) in enumerate(rects):
        if not mask[b:d,a:c].all():raise ValueError('REPAIR_INCLUDED_UNSAFE_PIXEL')
        owner[b:d,a:c]=i
    final_pixels=np.full(mask.shape,-1,dtype=np.int32);final_pixels[mask]=final_labels[owner[mask]]
    missing_after=int(np.sum(mask&(owner<0)));bad_edges=0
    for dy,dx in [(0,1),(1,0)]:
        a=final_pixels[:mask.shape[0]-dy,:mask.shape[1]-dx];b=final_pixels[dy:,dx:]
        bad_edges+=int(np.sum((a>=0)&(b>=0)&(a!=b)))
    if missing_after or bad_edges:raise ValueError('COVER_REPAIR_CONNECTIVITY_CHECK_FAILED')
    return rects,{'algorithm':'safe_pixel_fill_and_overlap_component_bridge','original_regions':original,
                  'uncovered_safe_pixels_before':missing_before,'uncovered_safe_pixels_after':missing_after,
                  'components_after_fill':int(count),'components_after_bridge':int(final_count),
                  'unbridged_safe_adjacencies':bad_edges,'fill_boxes_cells':fill,'bridge_boxes_cells':bridges,
                  'added_fill_regions':len(fill),'added_bridge_regions':len(bridges),
                  'four_connected_safe_roi_covered':True,'se2_completeness_claim':False}
