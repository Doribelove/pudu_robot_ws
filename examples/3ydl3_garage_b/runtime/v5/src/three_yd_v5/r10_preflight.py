"""Necessary reachability on a superset of all L2 rectangle recovery cells."""
import math,time
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import breadth_first_order

def optimistic_reachable(safe,direction,start,goal):
    """False proves no XY path under the existing direction/corner rules.

    This is not a physical vehicle blockage certificate. The caller must pass
    a superset of every cell that the existing recovery could possibly open.
    Costs are deliberately ignored, and no budget failure is cached as false.
    """
    began=time.monotonic();safe=np.asarray(safe,bool);direction=np.asarray(direction,np.float64);h,w=safe.shape
    info={'kind':'optimistic_directed_xy','physical_blockage_proven':False}
    for r,c in (start,goal):
        if not (0<=r<h and 0<=c<w and safe[r,c]):
            return False,{**info,'reason':'optimistic_endpoint_absent','wall_ms':(time.monotonic()-began)*1000}
    index=np.full((h,w),-1,np.int32);rr,cc=np.nonzero(safe);index[rr,cc]=np.arange(len(rr),dtype=np.int32)
    rows=[];cols=[]
    for dr in (-1,0,1):
        for dc in (-1,0,1):
            if not(dr or dc):continue
            br=rr+dr;bc=cc+dc;valid=(br>=0)&(br<h)&(bc>=0)&(bc<w)
            ar=rr[valid];ac=cc[valid];br=br[valid];bc=bc[valid]
            valid=safe[br,bc]
            if dr and dc:valid&=safe[ar,bc]&safe[br,ac]
            a=direction[ar,ac];b=direction[br,bc]
            tol=math.sin(math.radians(5))*math.hypot(dr,dc)+1e-6
            valid&=(dc*a[:,0]-dr*a[:,1]>=-tol)&(dc*b[:,0]-dr*b[:,1]>=-tol)
            rows.append(index[ar[valid],ac[valid]]);cols.append(index[br[valid],bc[valid]])
    a=np.concatenate(rows);b=np.concatenate(cols)
    graph=csr_matrix((np.ones(len(a),np.uint8),(a,b)),shape=(len(rr),len(rr)))
    visited=breadth_first_order(graph,int(index[start]),directed=True,return_predecessors=False)
    found=bool(np.any(visited==index[goal]))
    return found,{**info,'reachable':found,'states':len(rr),'edges':len(a),
                  'visited':len(visited),'wall_ms':(time.monotonic()-began)*1000}
