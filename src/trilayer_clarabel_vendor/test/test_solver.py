import json,sys
import numpy as np
from scipy import sparse
import clarabel
cases={}
def solve(name,P,q,A,b,cones):
 s=clarabel.DefaultSettings();s.verbose=False;s.max_threads=1
 s.tol_feas=s.tol_gap_abs=s.tol_gap_rel=1e-8
 solver=clarabel.DefaultSolver(sparse.triu(sparse.csc_matrix(P),format='csc'),np.array(q,float),sparse.csc_matrix(A),np.array(b,float),cones,s)
 r=solver.solve();info=solver.get_info()
 cases[name]={'status':str(r.status),'x':list(r.x),'objective':r.obj_val,'iterations':r.iterations,'linear_solver':info.linsolver.name}
solve('box',[[4.,1.],[1.,2.]],[-1.,-1.],[[1.,0.],[0.,1.],[-1.,0.],[0.,-1.]],[1.,1.,0.,0.],[clarabel.NonnegativeConeT(4)])
solve('equality',[[1.,0.],[0.,1.]],[0.,0.],[[1.,1.]],[2.],[clarabel.ZeroConeT(1)])
solve('infeasible',[[1.]],[0.],[[1.],[-1.]],[0.,-1.],[clarabel.NonnegativeConeT(2)])
rng=np.random.default_rng(42);n=180;M=rng.standard_normal((n,n));P=M.T@M+np.eye(n);q=rng.standard_normal(n)
solve('dense_auto_faer',P,q,np.vstack([np.eye(n),-np.eye(n)]),np.ones(2*n),[clarabel.NonnegativeConeT(2*n)])
print(json.dumps({'clarabel':clarabel.__version__,'module':clarabel.__file__,'numpy':np.__version__,'cases':cases},allow_nan=True))
import os,pathlib
assert pathlib.Path(clarabel.__file__).resolve().is_relative_to(pathlib.Path(os.environ['TRILAYER_EXPECTED_VENDOR_PREFIX']).resolve())
assert clarabel.__version__=='0.11.1'
assert cases['box']['status']=='Solved' and np.allclose(cases['box']['x'],[1/7,3/7],atol=1e-7)
assert cases['equality']['status']=='Solved' and np.allclose(cases['equality']['x'],[1,1])
assert cases['infeasible']['status']=='PrimalInfeasible'
assert cases['dense_auto_faer']['status']=='Solved' and cases['dense_auto_faer']['linear_solver']=='faer'
