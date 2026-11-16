"""Reuse one exact densification when locating current swept-body failures.

Each recursive interval has the same poses as the original densify(interval),
including its exact first anchor. No collision certificate is retained.
"""
import math
import numpy as np
from arena_3d_v1.semantic_world import densify,wrap
from arena_3d_v1.curvature_repair import violating_segments


def kinematic_risks(world,p):
    bad,lengths=violating_segments(p,1/world.vehicle[2]);bad=set(map(int,bad))
    delta=np.diff(p,axis=0);heading=p[:-1,2]+wrap(delta[:,2])/2
    motion=np.arctan2(delta[:,1],delta[:,0])
    bad.update(map(int,np.flatnonzero(abs(wrap(motion-heading))>math.radians(5))))
    return np.asarray(sorted(bad),int),lengths


def risk_segments(world,p,mask,checker):
    bad,lengths=kinematic_risks(world,p);bad=set(map(int,bad))
    dense=densify(p);counts=[]
    for a,b in zip(p,p[1:]):
        d=b-a;d[2]=wrap(d[2])
        counts.append(max(1,math.ceil(np.linalg.norm(d[:2])/.025),math.ceil(abs(d[2])/(math.pi/180))))
    cuts=np.r_[0,np.cumsum(counts)]
    if cuts[-1]+1!=len(dense):raise ValueError('DENSIFICATION_CONTRACT_CHANGED')
    def locate(a,b):
        interval=dense[cuts[a]:cuts[b]+1]
        # Original densify(subpath) starts from the literal original anchor;
        # the preceding interpolation may differ by floating roundoff.
        if not np.array_equal(interval[0],p[a]):
            interval=interval.copy();interval[0]=p[a]
        if checker.check(interval):return
        if b-a<=1:bad.add(a);return
        middle=(a+b)//2;locate(a,middle);locate(middle,b)
    locate(0,len(p)-1)
    return np.asarray(sorted(bad),int),lengths
