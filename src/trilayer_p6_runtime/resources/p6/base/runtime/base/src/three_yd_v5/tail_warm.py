"""Select a lower-violation initialization; original certification still decides.

This function never accepts a trajectory. Both inputs keep the same exact
endpoint coefficients and all later SQP constraints, budgets and audits remain.
"""
import numpy as np


def select(original,candidate,B,D,E,boxes,vehicle,axes,ratio):
    from .curve_l3 import violation
    def score(cp):
        p=B@cp;v=D@cp;a=E@cp;speed=np.linalg.norm(v,axis=1)
        k=(v[:,0]*a[:,1]-v[:,1]*a[:,0])/np.maximum(speed,1e-6)**3
        return violation(p,v,speed,k,boxes,vehicle,axes,ratio)
    old=score(original);new=score(candidate)
    selected=bool(np.isfinite(candidate).all() and np.isfinite(new).all()
                  and new[0]<old[0]-1e-7 and new[1]<=old[1])
    return (candidate if selected else original),{
        'selected':selected,'original_violation':old,'quadratic_violation':new,
        'acceptance_unchanged':True}
