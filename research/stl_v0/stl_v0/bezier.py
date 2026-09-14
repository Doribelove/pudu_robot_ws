"""Cubic curves, subdivision, and conservative continuous curvature bounds."""
import math
import numpy as np


def evaluate(control, u):
    u=np.asarray(u,float);v=1-u
    b=np.stack([v**3,3*v*v*u,3*v*u*u,u**3],axis=-1)
    d=np.stack([v*v,2*v*u,u*u],axis=-1)
    a=np.stack([v,u],axis=-1)
    q=3*np.diff(control,axis=-2);r=2*np.diff(q,axis=-2)
    return b@control, d@q, a@r


def split(p):
    a=(p[:-1]+p[1:])/2;b=(a[:-1]+a[1:])/2;c=(b[0]+b[1])/2
    return np.array([p[0],a[0],b[0],c]),np.array([c,b[1],a[2],p[3]])


def curvature_bound(p):
    q=3*np.diff(p,axis=0);r=2*np.diff(q,axis=0)
    lo=q.min(axis=0);hi=q.max(axis=0)
    near=np.where(lo>0,lo,np.where(hi<0,hi,0.))
    lower=float(np.linalg.norm(near))
    numerator=np.zeros(4)
    for i in range(3):
        for j in range(2):
            numerator[i+j]+=math.comb(2,i)*math.comb(1,j)/math.comb(3,i+j)*float(q[i,0]*r[j,1]-q[i,1]*r[j,0])
    return (float(np.max(np.abs(numerator)))/lower**3 if lower>1e-12 else math.inf),lower


def certify_curvature(control, limit, depth_limit=13):
    stack=[(np.asarray(control,float),0)];leaves=0;max_bound=0.
    while stack:
        p,d=stack.pop();bound,lower=curvature_bound(p)
        if bound <= limit and lower > 1e-12:
            leaves+=1;max_bound=max(max_bound,bound)
            continue
        _,v,a=evaluate(p,np.array([0.,0.5,1.]))
        norm=np.linalg.norm(v,axis=1)
        if np.min(norm)<=1e-12:
            return {'valid':False,'reason':'ZERO_DERIVATIVE','leaves':leaves}
        k=np.abs(v[:,0]*a[:,1]-v[:,1]*a[:,0])/norm**3
        if np.max(k)>limit+1e-8:
            return {'valid':False,'reason':'CURVATURE_EXCEEDS_LIMIT','sample_max':float(np.max(k))}
        if d>=depth_limit:
            return {'valid':False,'reason':'CURVATURE_BOUND_UNRESOLVED','depth':d}
        left,right=split(p);stack.extend([(left,d+1),(right,d+1)])
    return {'valid':True,'upper_bound_per_m':max_bound,'leaves':leaves,'method':'Bernstein numerator / derivative-box lower bound with subdivision'}
