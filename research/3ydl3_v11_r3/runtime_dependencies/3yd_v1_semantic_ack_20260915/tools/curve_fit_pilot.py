import math,time,json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize,LinearConstraint
from arena_3d_v1.semantic_world import SemanticWorld,wrap,json_write
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.unified_four_backends_smoke import MapContext

def fit(start,goal,word,params,radius,left_k=0.,right_k=0.):
    lengths=np.asarray(params)*radius;L=sum(lengths);N=max(8,math.ceil(L/.05));ds=L/N
    bins=np.arange(N)*ds;seed=np.zeros(N)
    for j,k in enumerate(word):
        a=sum(lengths[:j]);b=a+lengths[j]
        seed+=np.maximum(0,np.minimum(bins+ds,b)-np.maximum(bins,a))/ds*(0 if k=='S' else (1 if k=='L' else -1)/radius)
    winding=sum((0 if k=='S' else 1 if k=='L' else -1)*par for k,par in zip(word,params))
    def coords(k):
        delta=k*ds;mid=start[2]+np.cumsum(delta)-delta/2;q=delta/2
        A=ds*np.sinc(q/np.pi);dx=A*np.cos(mid);dy=A*np.sin(mid)
        return dx,dy,mid,q
    def eq(k):
        dx,dy,_,_=coords(k)
        return [start[0]+sum(dx)-goal[0],start[1]+sum(dy)-goal[1],sum(k)*ds-winding]
    def jac(k):
        dx,dy,mid,q=coords(k)
        deriv=np.zeros_like(q);nz=abs(q)>1e-6;deriv[nz]=(q[nz]*np.cos(q[nz])-np.sin(q[nz]))/q[nz]**2;deriv[~nz]=-q[~nz]/3
        aprime=ds*.5*deriv
        return np.array([ds*(aprime*np.cos(mid)-dy/2-(np.cumsum(dy[::-1])[::-1]-dy)),ds*(aprime*np.sin(mid)+dx/2+(np.cumsum(dx[::-1])[::-1]-dx)),np.full(N,ds)])
    def obj(k):
        d=np.diff(np.r_[left_k,k,right_k]);return float(np.sum((k-seed)**2)+4*np.sum(d*d))
    def grad(k):
        return 2*(k-seed)+8*(2*k-np.r_[left_k,k[:-1]]-np.r_[k[1:],right_k])
    matrix=np.diff(np.eye(N),axis=0);bounds=[(-2.49,2.49)]*N;bounds[0]=(max(-2.49,left_k-.4),min(2.49,left_k+.4));bounds[-1]=(max(-2.49,right_k-.4),min(2.49,right_k+.4))
    t=time.monotonic();sol=minimize(obj,np.clip(seed,-2.49,2.49),jac=grad,bounds=bounds,constraints=[{'type':'eq','fun':eq,'jac':jac},LinearConstraint(matrix,-.4,.4)],method='SLSQP',options={'maxiter':80,'ftol':1e-8})
    dx,dy,_,_=coords(sol.x);p=np.column_stack([start[0]+np.r_[0,np.cumsum(dx)],start[1]+np.r_[0,np.cumsum(dy)],wrap(start[2]+np.r_[0,np.cumsum(sol.x*ds)])])
    return p,{'success':sol.success,'elapsed':time.monotonic()-t,'message':sol.message,'maxerr':float(max(abs(np.array(eq(sol.x))))),'max_dk':float(np.max(abs(np.diff(sol.x))))}

if __name__=='__main__':
 r=Path('results/bulb_diagnosis_02');w=SemanticWorld(r/'inputs/map.yaml',r/'inputs/semantic.json');c=r/'smac/stage_evidence/call_002';q=Query(**json.loads((c/'result.json').read_text())['query']);mask=np.load(c/'corridor.npy');p=np.load(c/'final.npy')
 ctx=MapContext('bulb',w.map,w.safe,w.map.distance_m,w.map.sha256,'',r/'inputs/map.yaml');auditor=PathAuditor(ctx,source_commit='pilot')
 for i,j in [(46,63),(44,65),(42,67)]:
  for rad in [.45,.6,.8]:
   for word,params in dubins_choices(p[i],p[j],rad):
    if sum(params)*rad>6:continue
    pp,stats=fit(p[i],p[j],word,params,rad);full=np.r_[p[:i],pp,p[j+1:]];a=w.audit(q,full,mask)
    if a['valid']:
     dd=np.diff(full,axis=0);ll=np.linalg.norm(dd[:,:2],axis=1);steer=np.arctan(.5*wrap(dd[:,2])/ll);points=[{'x':float(x),'y':float(y),'yaw':float(h),'steering':float(steer[min(z,len(steer)-1)]),'source':'l3_curvature_repair','motion_direction':'forward','planner_backend':'bounded_curvature_fit','backend_version':'pilot'} for z,(x,y,h) in enumerate(full)]
     can=auditor.audit(q,points,mask);print(i,j,rad,word,stats,a['valid'],can.final_valid_success,can.metrics.get('failure_code'),flush=True)
     if can.final_valid_success:
      np.save('report/continuous_curvature_witness.npy',full);json_write('report/continuous_curvature_witness.json',{'fit':stats,'independent':a,'canonical':can.metrics,'i':i,'j':j,'radius':rad,'word':word});raise SystemExit
    else:print(i,j,rad,word,stats,a['failures'],flush=True)
