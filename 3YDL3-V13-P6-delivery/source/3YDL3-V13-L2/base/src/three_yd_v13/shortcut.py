"""Frozen shortcut logic with equivalent cell enumeration shared by A* and D*."""
import math
import numpy as np
from .segment_cells import segment_cells

def shortcut_path(path, safe, potential, weight=2., directions=None):
    """Only accept collision-free, direction-legal and cost-preserving shortcuts."""
    path=np.asarray(path,int)
    def score(points):
        ds=np.linalg.norm(np.diff(points,axis=0),axis=1)
        values=potential[points[:,0],points[:,1]]
        return float(np.sum(ds*(1+weight*(values[:-1]+values[1:])*.5)))
    def valid(a,b,old):
        cells=segment_cells(a,b)
        r,c=cells.T
        if np.any(r<0) or np.any(c<0) or np.any(r>=safe.shape[0]) or np.any(c>=safe.shape[1]) or not np.all(safe[r,c]): return False
        delta=np.asarray([b[1]-a[1],a[0]-b[0]],float)
        if directions is not None and np.any(directions[r,c]@delta < -math.sin(math.radians(5))*np.linalg.norm(delta)-1e-6): return False
        n=max(2,int(np.linalg.norm(b-a)*3)+1)
        sample=np.rint(a+(b-a)*np.linspace(0,1,n)[:,None]).astype(int)
        # Integrate along the actual straight segment, not the length of the
        # quantized sample staircase (which artificially penalizes straightness).
        values=potential[sample[:,0],sample[:,1]]
        straight_cost=float(np.linalg.norm(b-a)*np.mean(1+weight*(values[:-1]+values[1:])*.5))
        return straight_cost<=score(old)*1.03+1e-6
    def simplify(i,j):
        if j<=i+1 or valid(path[i],path[j],path[i:j+1]): return [i,j]
        p=path[i:j+1].astype(float);v=p[-1]-p[0];norm=np.dot(v,v)
        d=np.linalg.norm(p-(p[0]+np.clip(((p-p[0])@v)/max(norm,1e-9),0,1)[:,None]*v),axis=1)
        split=i+int(np.argmax(d))
        if split<=i or split>=j: split=(i+j)//2
        return simplify(i,split)[:-1]+simplify(split,j)
    if len(path)<3:return path.copy()
    return path[simplify(0,len(path)-1)]
