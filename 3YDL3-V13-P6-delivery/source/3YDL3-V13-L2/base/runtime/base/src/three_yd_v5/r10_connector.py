"""Exact pure-geometry connector reuse; every map/traffic audit still runs."""
import math,time,copy
from functools import lru_cache
from collections import OrderedDict
from types import SimpleNamespace
import numpy as np
from scipy.optimize import minimize,LinearConstraint
from arena_3d_v1.semantic_world import wrap
from arena_3d_v1.curvature_repair import fit_connector as original_fit
from .r10_options import enabled
_CACHE=OrderedDict()
_STATS={'calls':0,'hits':0,'failure_hits':0,'misses':0,'evictions':0}
@lru_cache(maxsize=32)
def _linear_constraint(n):
    return LinearConstraint(np.diff(np.eye(n),axis=0),-.4,.4)
def stats():
    return {**_STATS,'entries':len(_CACHE),'arrays_bytes':sum(0 if isinstance(v,str) else v[0].nbytes for v in _CACHE.values()),'constraint_entries':_linear_constraint.cache_info().currsize}
def _fit_uncached(start,goal,word,params,radius,left_k,right_k,deadline,maximum):
    """Integrate forward circular steps with bounded changes of curvature.

    Dubins words initialize a small constrained fit. The fit changes actual
    positions and headings; steering is later derived without clipping.
    """
    lengths=np.asarray(params)*radius;length=float(sum(lengths));n=max(8,math.ceil(length/.05));ds=length/n
    if n>128:raise ValueError('CONNECTOR_STATE_BUDGET')
    bins=np.arange(n)*ds;seed=np.zeros(n)
    for j,kind in enumerate(word):
        a=float(sum(lengths[:j]));b=a+lengths[j]
        seed+=np.maximum(0,np.minimum(bins+ds,b)-np.maximum(bins,a))/ds*(0 if kind=='S' else (1 if kind=='L' else -1)/radius)
    winding=sum((0 if kind=='S' else 1 if kind=='L' else -1)*par for kind,par in zip(word,params))
    def coords(k):
        delta=k*ds;mid=start[2]+np.cumsum(delta)-delta/2;q=delta/2
        amplitude=ds*np.sinc(q/np.pi)
        return amplitude*np.cos(mid),amplitude*np.sin(mid),mid,q
    def eq(k):
        dx,dy,_,_=coords(k)
        return np.array([start[0]+sum(dx)-goal[0],start[1]+sum(dy)-goal[1],sum(k)*ds-winding])
    def jac(k):
        dx,dy,mid,q=coords(k);deriv=-q/3;nz=abs(q)>1e-6
        deriv[nz]=(q[nz]*np.cos(q[nz])-np.sin(q[nz]))/q[nz]**2
        aprime=ds*.5*deriv
        return np.array([ds*(aprime*np.cos(mid)-dy/2-(np.cumsum(dy[::-1])[::-1]-dy)),
            ds*(aprime*np.sin(mid)+dx/2+(np.cumsum(dx[::-1])[::-1]-dx)),np.full(n,ds)])
    def objective(k):
        d=np.diff(np.r_[left_k,k,right_k]);return float(np.sum((k-seed)**2)+4*np.sum(d*d))
    def gradient(k):
        return 2*(k-seed)+8*(2*k-np.r_[left_k,k[:-1]]-np.r_[k[1:],right_k])
    def check_budget(_=None):
        if time.monotonic()>=deadline:raise TimeoutError('LOCAL_CURVATURE_BUDGET')
    limit=maximum-.01;bounds=[(-limit,limit)]*n
    bounds[0]=(max(-limit,left_k-.4),min(limit,left_k+.4))
    bounds[-1]=(max(-limit,right_k-.4),min(limit,right_k+.4))
    if any(lo>hi for lo,hi in bounds):raise ValueError('CONNECTOR_BOUNDARY_CURVATURE')
    def feasible(k):
        return (np.all(np.isfinite(k)) and np.max(abs(eq(k)))<=1e-7
            and np.max(abs(np.diff(k)))<=.4+1e-6
            and all(lo-1e-6<=v<=hi+1e-6 for v,(lo,hi) in zip(k,bounds)))
    class FeasibleIterate(Exception):
        def __init__(self,k):self.k=k.copy()
    iterations=0
    def on_iteration(k):
        nonlocal iterations
        iterations+=1;check_budget()
        if feasible(k):raise FeasibleIterate(k)
    check_budget()
    try:
        sol=minimize(objective,np.clip(seed,-limit,limit),jac=gradient,bounds=bounds,
            constraints=[{'type':'eq','fun':eq,'jac':jac},_linear_constraint(n)],
            method='SLSQP',callback=on_iteration,options={'maxiter':80,'ftol':1e-8})
    except FeasibleIterate as found:
        sol=SimpleNamespace(x=found.k,success=False,nit=iterations,message='Feasible iterate; objective optimization stopped')
    check_budget()
    # This branch needs a feasible connector, not a proof of objective
    # optimality. An iteration-limit exit is usable only when all constraints
    # independently hold; the final whole-path audits remain mandatory.
    if not feasible(sol.x):
        raise ValueError('CONNECTOR_FIT_FAILED')
    dx,dy,_,_=coords(sol.x)
    p=np.column_stack([start[0]+np.r_[0,np.cumsum(dx)],start[1]+np.r_[0,np.cumsum(dy)],
                      wrap(start[2]+np.r_[0,np.cumsum(sol.x*ds)])])
    # Numerical endpoint residue only; final whole-path audits remain binding.
    p[0]=start;p[-1]=goal
    return p,{'word':word,'initial_radius_m':radius,'states':n,'iterations':int(sol.nit),'optimizer_converged':bool(sol.success),
              'optimizer_status':str(sol.message),'acceptance':'independently_feasible_not_claimed_optimal',
              'endpoint_residual':float(np.max(abs(eq(sol.x)))),'max_curvature_step':float(max(abs(np.diff(sol.x))))}


def fit_connector(start,goal,word,params,radius,left_k,right_k,deadline,maximum):
    if not enabled('C'):
        return original_fit(start,goal,word,params,radius,left_k,right_k,deadline,maximum)
    if time.monotonic()>=deadline:raise TimeoutError('LOCAL_CURVATURE_BUDGET')
    _STATS['calls']+=1
    # No quantization: all endpoint poses, slack-adjusted parameters and
    # boundary curvatures are part of the exact numerical problem. Geometry
    # alone is cached: collision/semantic checks are never cached here.
    key=(tuple(map(float,start)),tuple(map(float,goal)),str(word),tuple(map(float,params)),
         float(radius),float(left_k),float(right_k),float(maximum))
    if key in _CACHE:
        value=_CACHE.pop(key);_CACHE[key]=value;_STATS['hits']+=1
        if isinstance(value,str):
            _STATS['failure_hits']+=1;raise ValueError(value)
        return value[0].copy(),copy.deepcopy(value[1])
    _STATS['misses']+=1
    try:
        value=_fit_uncached(start,goal,word,params,radius,left_k,right_k,deadline,maximum)
    except ValueError as error:
        value=str(error)
        # Numerical failures are repeatable for identical geometry, but
        # deadline exceptions never enter the cache.
        _CACHE[key]=value
        while len(_CACHE)>256:_CACHE.popitem(last=False);_STATS['evictions']+=1
        raise
    _CACHE[key]=(value[0].copy(),copy.deepcopy(value[1]))
    while len(_CACHE)>256:_CACHE.popitem(last=False);_STATS['evictions']+=1
    return value
