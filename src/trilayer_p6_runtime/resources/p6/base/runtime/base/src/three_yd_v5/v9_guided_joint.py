"""Reference-guided subdivision of analytic G2 proposals, bounded by time.

Only existing collision-free straight reference poses become internal seams.
They are not L1 nodes. No certificate is carried across the current audit.
"""
import math,time
import numpy as np
from arena_3d_v1.semantic_world import densify,wrap
from arena_3d_v1.curvature_repair import violating_segments
from .v9_joints import connect,evaluate


def compound_corner(world,p,a,b,checker,deadline):
    """Straight legs and a compact G2 bend at the forward-ray intersection.

    This is a geometric proposal inside the current mask, not a new semantic
    junction. Only zero-curvature original anchors permit straight legs.
    """
    started=time.monotonic();maximum=1/world.vehicle[2];trials=0
    lengths=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    for j,k in ((a-1,a),(b,b+1)):
        if 0<=j and k<len(p) and abs(wrap(p[k,2]-p[j,2]))>1e-6:return None,{}
    u=np.array([math.cos(p[a,2]),math.sin(p[a,2])]);v=np.array([math.cos(p[b,2]),math.sin(p[b,2])])
    rays=np.column_stack((u,v))
    if abs(np.linalg.det(rays))<.15 or abs(wrap(p[b,2]-p[a,2]))>math.pi*.8:return None,{}
    t,s=np.linalg.solve(rays,p[b,:2]-p[a,:2])
    if min(t,s)<.5:return None,{}
    vertex=p[a,:2]+t*u
    def line(start,goal):
        n=max(1,math.ceil(np.linalg.norm(goal[:2]-start[:2])/.02))
        return np.c_[np.linspace(start[:2],goal[:2],n+1),np.full(n+1,start[2])]
    for d in (.5,.6,.8,1.,1.2,1.5):
        if d>min(t,s):continue
        start=np.r_[vertex-d*u,p[a,2]];goal=np.r_[vertex+d*v,p[b,2]]
        for coeff in (1.5,1.75,2.,2.5):
            if time.monotonic()>=deadline:return None,{}
            trials+=1
            try:q,k=evaluate(start,goal,0.,0.,(coeff*d,coeff*d))
            except ValueError:continue
            if np.max(abs(k))>maximum-.01:continue
            joined=np.vstack((line(p[a],start)[:-1],q,line(goal,p[b])[1:]))
            distance=float(np.linalg.norm(np.diff(joined[:,:2],axis=0),axis=1).sum())
            if distance>float(lengths[a:b].sum())*1.25+.2:continue
            splice=np.vstack((p[max(0,a-1):a],joined,p[b+1:min(len(p),b+2)]))
            if len(violating_segments(splice,maximum)[0]):continue
            delta=np.diff(splice,axis=0);motion=np.arctan2(delta[:,1],delta[:,0]);heading=splice[:-1,2]+wrap(delta[:,2])/2
            if np.max(abs(wrap(motion-heading)),initial=0)>math.radians(5):continue
            if not checker.check(densify(splice)):continue
            return joined,{'method':'compound_G2_corner','success':True,'trials':trials,
                'wall_ms':(time.monotonic()-started)*1000,'corner_inset_m':d,'handle_factor':coeff,
                'length_m':distance,'max_analytic_curvature':float(np.max(abs(k))),
                'exact_original_anchor_poses':True,'straight_bend_curvature':[float(k[0]),float(k[-1])]}
    return None,{}


def guided_connect(world,p,a,b,checker,deadline,depth=0):
    q,detail=compound_corner(world,p,a,b,checker,min(deadline,time.monotonic()+.012))
    if q is not None:return q,detail
    lengths=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    nominal=float(lengths[a:b].sum())
    q,detail=connect(world,p,a,b,checker,min(deadline,time.monotonic()+.012),
        maximum_length=nominal*1.25+.2,sample_limit=8192)
    if q is not None:return q,detail
    if depth>=4 or time.monotonic()>=deadline or b-a<8:
        return None,{'method':'guided_G2','success':False,'direct':detail,'failure':'GUIDED_DEPTH_OR_TIME_LIMIT'}
    arc=np.r_[0.,np.cumsum(lengths)]
    candidates=[]
    for m in range(a+3,b-2):
        if min(arc[m]-arc[a],arc[b]-arc[m])<.4:continue
        if max(abs(wrap(p[m,2]-p[m-1,2])),abs(wrap(p[m+1,2]-p[m,2])))>1e-6:continue
        d=p[m+1,:2]-p[m-1,:2]
        if abs(wrap(math.atan2(d[1],d[0])-p[m,2]))>1e-5:continue
        candidates.append((abs(arc[m]-(arc[a]+arc[b])/2),m))
    for _,m in sorted(candidates)[:3]:
        if time.monotonic()>=deadline:break
        if not checker.check(densify(p[m-1:m+2])):continue
        left,ld=guided_connect(world,p,a,m,checker,deadline,depth+1)
        if left is None:continue
        right,rd=guided_connect(world,p,m,b,checker,deadline,depth+1)
        if right is None:continue
        joined=np.vstack((left,right[1:]))
        if len(violating_segments(joined,1/world.vehicle[2])[0]):continue
        if not checker.check(densify(joined)):continue
        return joined,{'method':'guided_G2','success':True,'reference_internal_seam':p[m].tolist(),
            'seam_index':m,'direct':detail,'pieces':[ld,rd],'exact_original_anchor_poses':True}
    return None,{'method':'guided_G2','success':False,'direct':detail,'failure':'NO_VALID_REFERENCE_GUIDED_JOINT'}
