"""Bounded G2 quintic proposals with analytic tangents, never yaw relabeling.

The polynomial matches the original positions, headings and adjacent signed
curvature. It is only a proposal: sampled kinematics, exact current sweeps and
the unchanged full-route audits remain required. No map or audit is cached.
"""
import math
import time
import numpy as np
from arena_3d_v1.semantic_world import densify, wrap
from arena_3d_v1.curvature_repair import violating_segments


def evaluate(start, goal, left_k, right_k, handles, step=.02,sample_limit=4096):
    start=np.asarray(start,float);goal=np.asarray(goal,float)
    l0,l1=handles
    u0=np.array([math.cos(start[2]),math.sin(start[2])])
    u1=np.array([math.cos(goal[2]),math.sin(goal[2])])
    d0=l0*u0;d1=l1*u1
    dd0=left_k*l0*l0*np.array([-u0[1],u0[0]])
    dd1=right_k*l1*l1*np.array([-u1[1],u1[0]])
    control=np.array([start[:2],start[:2]+d0/5,
        start[:2]+2*d0/5+dd0/20,goal[:2]-2*d1/5+dd1/20,
        goal[:2]-d1/5,goal[:2]])
    # Convex-hull derivative bound guarantees <=step travelled per sample.
    n=max(8,math.ceil(5*np.max(np.linalg.norm(np.diff(control,axis=0),axis=1))/step))
    if n>sample_limit:raise ValueError('JOINT_SAMPLE_LIMIT')
    t=np.linspace(0,1,n+1)
    def bernstein(c):
        degree=len(c)-1
        basis=np.array([math.comb(degree,i)*t**i*(1-t)**(degree-i) for i in range(degree+1)])
        return basis.T@c
    xy=bernstein(control);d=bernstein(5*np.diff(control,axis=0))
    dd=bernstein(20*np.diff(control,n=2,axis=0))
    speed=np.linalg.norm(d,axis=1)
    if not np.all(np.isfinite(xy)) or np.min(speed)<1e-5:raise ValueError('JOINT_CUSP')
    curvature=(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/speed**3
    yaw=np.arctan2(d[:,1],d[:,0])
    p=np.column_stack((xy,yaw))
    # Branch normalization only; these values equal the analytic tangent.
    assert np.max(abs(wrap(p[[0,-1],2]-[start[2],goal[2]])))<1e-12
    p[0]=start;p[-1]=goal
    return p,curvature


def connect(world,p,a,b,checker,deadline,maximum_length=6.,sample_limit=4096):
    started=time.monotonic();trials=0;reasons={}
    lengths=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    left=float(wrap(p[a,2]-p[a-1,2])/lengths[a-1]) if a and lengths[a-1]>1e-8 else 0.
    right=float(wrap(p[b+1,2]-p[b,2])/lengths[b]) if b<len(p)-1 and lengths[b]>1e-8 else 0.
    chord=float(np.linalg.norm(p[b,:2]-p[a,:2]));nominal=float(lengths[a:b].sum())
    maximum=1/world.vehicle[2]
    def result(success,**extra):
        return {'method':'analytic_G2_quintic','success':success,'trials':trials,
            'rejections':reasons,'wall_ms':(time.monotonic()-started)*1000,**extra}
    if chord<.05 or max(abs(left),abs(right))>maximum:
        return None,result(False,failure='JOINT_ANCHOR_GEOMETRY')
    # Finite grid, nearest to chord first. No costly continuous optimizer.
    pairs=sorted(((x,y) for x in (.5,.75,1.,1.5,2.) for y in (.5,.75,1.,1.5,2.)),
        key=lambda v:abs(v[0]-1)+abs(v[1]-1))
    for x,y in pairs:
        if time.monotonic()>=deadline:return None,result(False,failure='JOINT_BUDGET')
        trials+=1
        try:q,k=evaluate(p[a],p[b],left,right,(x*chord,y*chord),sample_limit=sample_limit)
        except ValueError as e:
            reasons[str(e)]=reasons.get(str(e),0)+1;continue
        distance=float(np.linalg.norm(np.diff(q[:,:2],axis=0),axis=1).sum())
        if distance>min(maximum_length,nominal*1.25+.2) or np.max(abs(k))>maximum-.01:
            reasons['length_or_curvature']=reasons.get('length_or_curvature',0)+1;continue
        splice=np.vstack((p[max(0,a-1):a],q,p[b+1:min(len(p),b+2)]))
        if len(violating_segments(splice,maximum)[0]):
            reasons['sampled_seam']=reasons.get('sampled_seam',0)+1;continue
        delta=np.diff(splice,axis=0)
        motion=np.arctan2(delta[:,1],delta[:,0])
        heading=splice[:-1,2]+wrap(delta[:,2])/2
        if np.max(abs(wrap(motion-heading)),initial=0)>math.radians(5):
            reasons['motion_heading']=reasons.get('motion_heading',0)+1;continue
        if not checker.check(densify(splice)):
            reasons['current_swept_body_or_scope']=reasons.get('current_swept_body_or_scope',0)+1;continue
        return q,result(True,exact_original_anchor_poses=True,
            handles=[x*chord,y*chord],length_m=distance,max_analytic_curvature=float(np.max(abs(k))),
            boundary_curvature=[left,right],analytic_boundary_curvature=[float(k[0]),float(k[-1])])
    return None,result(False,failure='NO_VALID_ANALYTIC_JOINT')
