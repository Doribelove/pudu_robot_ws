"""Deterministic sweep index for exact positive-area box intersections."""
import time
import numpy as np


def box_pairs(boxes,epsilon=1e-6,deadline=float('inf')):
    boxes=np.asarray(boxes);n=len(boxes)
    if n<2:return np.empty((0,2),int),{'possible_pairs':n*(n-1)//2,'axis_candidates':0,'axis':0}
    # Choose the axis with fewer simultaneously active intervals.
    counts=[]
    for axis in [0,1]:
        starts=np.sort(boxes[:,axis]);ends=np.sort(boxes[:,axis+2])
        counts.append(int(np.sum(np.maximum(0,np.arange(n)-np.searchsorted(ends,starts+epsilon,side='right')))))
    axis=int(np.argmin(counts));other=1-axis
    order=np.argsort(boxes[:,axis],kind='stable');active=np.empty(0,dtype=int);chunks=[]
    for i in order:
        if boxes[i,axis+2]-boxes[i,axis]<=epsilon:continue
        if time.monotonic()>deadline:raise TimeoutError('GEOMETRY_BUILD_BUDGET_EXHAUSTED')
        active=active[boxes[active,axis+2]-boxes[i,axis]>epsilon]
        keep=(np.minimum(boxes[active,other+2],boxes[i,other+2])-np.maximum(boxes[active,other],boxes[i,other])>epsilon)
        js=active[keep]
        if len(js):chunks.append(np.c_[np.minimum(i,js),np.maximum(i,js)])
        active=np.r_[active,i]
    pairs=np.concatenate(chunks) if chunks else np.empty((0,2),int)
    if len(pairs):pairs=pairs[np.lexsort((pairs[:,1],pairs[:,0]))]
    return pairs,{'possible_pairs':n*(n-1)//2,'axis_candidates':counts[axis],'axis':axis,'bbox_overlap_pairs':len(pairs)}
