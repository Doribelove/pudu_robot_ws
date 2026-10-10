"""XY-only shortcut checks, corner detection, and variable-radius bounds."""
import math
import cv2
import numpy as np
from scipy.spatial import cKDTree


def segment_cells(a,b):
    """Conservative supercover of a line between integer cell centers."""
    a=np.asarray(a,float); b=np.asarray(b,float)
    n=max(2,int(np.max(abs(b-a))*4)+1)
    rc=np.floor(a+(b-a)*np.linspace(0,1,n)[:,None]+.5).astype(int)
    keep=np.r_[True,np.any(rc[1:]!=rc[:-1],axis=1)]; rc=rc[keep]
    result=[tuple(x) for x in rc]
    for p,q in zip(rc,rc[1:]):
        if np.all(p!=q): result.extend([(int(p[0]),int(q[1])),(int(q[0]),int(p[1]))])
    return np.asarray(list(dict.fromkeys(result)),int)


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


def corners(points, threshold_deg=30.):
    points=np.asarray(points,float)
    if len(points)<3:return []
    d=np.diff(points,axis=0);length=np.linalg.norm(d,axis=1)
    angles=np.degrees(np.arccos(np.clip(np.sum(d[:-1]*d[1:],axis=1)/np.maximum(length[:-1]*length[1:],1e-12),-1,1)))
    s=np.r_[0,np.cumsum(length)]
    return [{'index':i+1,'position':points[i+1].tolist(),'turn_angle_deg':float(a),'arc_m':float(s[i+1])} for i,a in enumerate(angles) if a>=threshold_deg]


def variable_neighborhood(world, points, corner_records, normal=.8, turning=1.2, plateau=.6, transition=1.2,
                          forced_turning_points=None):
    """Union of metric disks along the reference; 1.2 m is the absolute cap.

    Radius is 1.2 within 0.6 m arc distance of a corner and tapers to .8
    over a further 1.2 m. Endpoints are included, never globally expanded.
    """
    if not 0<normal<=turning<=1.2:raise ValueError('V5_INVALID_RADIUS')
    p=np.asarray(points,float);distance=np.linalg.norm(np.diff(p,axis=0),axis=1)
    arc=np.r_[0,np.cumsum(distance)]
    samples=[];ss=[]
    for i,(a,b) in enumerate(zip(p,p[1:])):
        t=np.linspace(0,1,max(2,int(math.ceil(distance[i]/world.map.resolution))+1))
        samples.extend(a+(b-a)*t[:,None]);ss.extend(arc[i]+t*distance[i])
    xy=np.asarray(samples);ss=np.asarray(ss)
    radius=np.full(len(xy),normal)
    for record in corner_records:
        delta=abs(ss-record['arc_m'])
        radius=np.maximum(radius,normal+(turning-normal)*np.clip(1-(delta-plateau)/transition,0,1))
    forced=np.asarray(forced_turning_points if forced_turning_points is not None else [],float).reshape(-1,2)
    if len(forced):
        # A forced point marks a location where the XY reference is not a
        # vehicle-feasibility proof (for example a static obstacle softened in
        # L2). Give L3 the same plateau and taper used at geometric corners so
        # it has room to avoid locally without widening the complete route.
        tree=cKDTree(forced)
        delta=tree.query(xy)[0]
        radius=np.maximum(radius,normal+(turning-normal)*np.clip(1-(delta-plateau)/transition,0,1))
    mask=np.zeros(world.hard.shape,np.uint8)
    rows,cols=world.cells(xy)
    # Pixel radius is floored: rasterization never opens outside the requested
    # radius around its cell center. Every original physical obstacle survives.
    for r,c,rad in zip(rows,cols,radius):
        cv2.circle(mask,(int(c),int(r)),int(math.floor(rad/world.map.resolution+1e-8)),1,-1)
    # Enforce metric radius after rasterization; a rounded seed cannot enlarge
    # the authorized .8/1.2 m range by a fraction of a cell.
    if hasattr(world,'world_points'):
        rr,cc=np.nonzero(mask)
        distance,ix=cKDTree(xy).query(world.world_points(rr,cc))
        mask[rr,cc]=(distance<=radius[ix]+1e-9).astype(np.uint8)
    return mask.astype(bool),{'normal_radius_m':normal,'corner_radius_m':turning,'corner_threshold_deg':30.,'plateau_arc_m':plateau,'transition_arc_m':transition,'corner_count':len(corner_records),'forced_bridge_points':len(forced),'forced_repair_points':len(forced),'max_sample_radius_m':float(radius.max()),'global_fallback':False}


def view_resolution(world):
    """L2 bridge samples are fixed at three 0.05 m cells in V5-r1."""
    return world.map.resolution*3
