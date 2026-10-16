# Rejected offline parameterization, never imported by the planner.
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


def evaluate(start, goal, left_k, right_k, handles, step=.02,sample_limit=4096,tangent_acceleration=(0.,0.)):
    start=np.asarray(start,float);goal=np.asarray(goal,float)
    l0,l1=handles
    u0=np.array([math.cos(start[2]),math.sin(start[2])])
    u1=np.array([math.cos(goal[2]),math.sin(goal[2])])
    d0=l0*u0;d1=l1*u1
    dd0=left_k*l0*l0*np.array([-u0[1],u0[0]])+tangent_acceleration[0]*d0
    dd1=right_k*l1*l1*np.array([-u1[1],u1[0]])+tangent_acceleration[1]*d1
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

