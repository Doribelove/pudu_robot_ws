import json,sys,os
from pathlib import Path
import numpy as np
from scipy import sparse
import osqp,qdldl
prefix=Path(os.environ['TRILAYER_EXPECTED_VENDOR_PREFIX']).resolve()
assert Path(osqp.__file__).resolve().is_relative_to(prefix),osqp.__file__
assert Path(qdldl.__file__).resolve().is_relative_to(prefix),qdldl.__file__
assert osqp.__version__=='0.6.7.post3'
reports=[]
def run(P,q,A,l,u):
 s=osqp.OSQP();s.setup(P=sparse.csc_matrix(P),q=np.array(q,float),A=sparse.csc_matrix(A),l=np.array(l,float),u=np.array(u,float),eps_abs=1e-9,eps_rel=1e-9,verbose=False,polish=True,max_iter=10000)
 r=s.solve();return r
r=run([[4,1],[1,2]],[1,1],[[1,1],[1,0],[0,1]],[1,0,0],[1,.7,.7]);assert r.info.status_val==1;np.testing.assert_allclose(r.x,[.3,.7],atol=1e-7)
reports.append({'case':'constrained_qp','x':r.x.tolist(),'status':r.info.status,'objective':r.info.obj_val})
r=run([[2,0],[0,2]],[0,0],[[1,0],[0,1]],[1,1],[np.inf,np.inf]);assert r.info.status_val==1;np.testing.assert_allclose(r.x,[1,1],atol=1e-7)
reports.append({'case':'lower_bounds','x':r.x.tolist(),'status':r.info.status,'objective':r.info.obj_val})
r=run([[1]],[0],[[1],[1]],[1,-np.inf],[np.inf,0]);assert 'infeasible' in r.info.status
reports.append({'case':'infeasible','status':r.info.status})
M=np.array([[4.,1.],[1.,2.]]);b=np.array([1.,2.]);solver=qdldl.Solver(sparse.csc_matrix(M));x=solver.solve(b);np.testing.assert_allclose(M@x,b,atol=1e-12)
reports.append({'case':'qdldl_linear_system','x':x.tolist(),'residual':float(np.linalg.norm(M@x-b))})
print(json.dumps({'osqp_version':osqp.__version__,'osqp_path':osqp.__file__,'qdldl_path':qdldl.__file__,'numpy_version':np.__version__,'cases':reports},indent=2))
