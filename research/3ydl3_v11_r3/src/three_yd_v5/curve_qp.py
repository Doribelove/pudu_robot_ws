"""Explicit interchangeable convex-QP solvers with identical two-sided bounds."""
import time
from types import SimpleNamespace
import numpy as np
from scipy import sparse
def clarabel_setup(P,q,A,lo,hi,deadline,eps):
 import clarabel
 equal=np.isfinite(lo)&np.isfinite(hi)&(np.abs(hi-lo)<1e-12)
 upper=np.isfinite(hi)&~equal;lower=np.isfinite(lo)&~equal
 aa=sparse.vstack((A[equal],A[upper],-A[lower]),format='csc')
 bb=np.r_[hi[equal],hi[upper],-lo[lower]]
 cones=[]
 if equal.any():cones.append(clarabel.ZeroConeT(int(equal.sum())))
 if upper.any() or lower.any():cones.append(clarabel.NonnegativeConeT(int(upper.sum()+lower.sum())))
 settings=clarabel.DefaultSettings();settings.verbose=False
 settings.max_threads=1;settings.max_iter=150
 settings.tol_feas=eps;settings.tol_gap_abs=eps;settings.tol_gap_rel=eps
 settings.time_limit=max(.001,deadline-time.monotonic())
 return clarabel.DefaultSolver(sparse.triu(P,format='csc'),q,aa,bb,cones,settings)
def clarabel_solve(solver):
 result=solver.solve();status=str(result.status)
 valid={'Solved':1,'AlmostSolved':2}.get(status,3)
 return SimpleNamespace(x=np.asarray(result.x),info=SimpleNamespace(status=status,status_val=valid,iter=result.iterations))
