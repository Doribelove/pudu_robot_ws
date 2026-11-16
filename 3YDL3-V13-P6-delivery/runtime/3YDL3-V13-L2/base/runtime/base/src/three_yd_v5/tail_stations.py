"""Coarser SQP collocation on straight guide interiors, dense around bends.

These are optimization stations, never collision or acceptance samples. Every
delivered spline still needs the unchanged continuous Bernstein/body certificate.
"""
import numpy as np


def stations(p,s,fine):
    v=np.diff(p,axis=0);v/=np.maximum(np.linalg.norm(v,axis=1)[:,None],1e-15)
    turns=s[1:-1][np.sum(v[:-1]*v[1:],axis=1)<np.cos(np.deg2rad(3.))]
    dense=np.arange(0,s[-1],fine)
    keep=(dense<=5.)|(dense>=s[-1]-5.)
    for t in turns:keep|=abs(dense-t)<=1.2
    u=np.unique(np.r_[np.arange(0,s[-1],max(.3,fine)),dense[keep],s,s[-1]])
    u=u[np.r_[True,np.diff(u)>1e-6]]
    if s[-1]-u[-1]<1e-6:u[-1]=s[-1]
    else:u=np.r_[u,s[-1]]
    return u
