"""Bounded local curvature repair; no change to Smac or acceptance thresholds."""
import math,time
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from scipy.optimize import minimize, LinearConstraint
from arena_evaluation.semantic_constraint_core import dubins_choices
from .semantic_world import wrap,digest,densify
from .recovery_pipeline import RecoveryController,CandidateRecovery


def violating_segments(p, maximum):
    delta=np.diff(p,axis=0);length=np.linalg.norm(delta[:,:2],axis=1)
    heading=np.divide(2*np.sin(abs(wrap(delta[:,2]))/2),length,out=np.zeros_like(length),where=length>1e-8)
    a,b=delta[:-1,:2],delta[1:,:2]
    den=np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)*np.linalg.norm(a+b,axis=1)
    geometry=np.divide(2*abs(a[:,0]*b[:,1]-a[:,1]*b[:,0]),den,out=np.zeros(len(a)),where=den>1e-12)
    bends=np.flatnonzero(geometry>maximum+1e-6)
    # Recomputing truthful steering after a splice can expose a neighboring
    # discontinuity hidden by the frozen Smac metadata's clipping. Include
    # both adjacent segments so that the geometric repair covers it too.
    steering=np.arctan(.5*np.divide(wrap(delta[:,2]),length,out=np.zeros_like(length),where=length>1e-8))
    jumps=np.flatnonzero(abs(np.diff(steering))>math.radians(15)+1e-6)
    return np.unique(np.r_[np.flatnonzero(heading>maximum+1e-6),bends,bends+1,jumps,jumps+1]),length


def fit_connector(start,goal,word,params,radius,left_k,right_k,deadline,maximum):
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
            constraints=[{'type':'eq','fun':eq,'jac':jac},LinearConstraint(np.diff(np.eye(n),axis=0),-.4,.4)],
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


def truthful_points(p,original,first,last):
    delta=np.diff(p,axis=0);length=np.linalg.norm(delta[:,:2],axis=1)
    steering=np.arctan(.5*np.divide(wrap(delta[:,2]),length,out=np.zeros_like(length),where=length>1e-8))
    points=[]
    for i,(x,y,yaw) in enumerate(p):
        inserted=first<=i<=last
        points.append({'x':float(x),'y':float(y),'yaw':float(yaw),'steering':float(steering[min(i,len(steering)-1)]),
            'velocity':0.,'motion_direction':'forward','source':'l3_local_curvature_repair' if inserted else original.source,
            'planner_backend':'bounded_forward_curvature_fit' if inserted else original.planner_backend,
            'backend_version':'local-curvature-v1' if inserted else original.backend_version})
    return points


def ordered_occurrences(topology,route,p):
    r,c=topology.world.cells(densify(p));inside=(r>=0)&(c>=0)&(r<topology.subregions.shape[0])&(c<topology.subregions.shape[1])
    if not np.all(inside):return False,[]
    values=topology.subregions[r,c];visits=list(map(int,values[np.r_[True,values[1:]!=values[:-1]]]))
    required=list(route.region_ids);index=0
    for region in visits:
        if index<len(required) and region==required[index]:index+=1
    return index==len(required),visits


class TurnaroundController(RecoveryController):
    def plan_l3(self,query,session,auditor,spec,**kwargs):
        outcome=super().plan_l3(query,session,auditor,spec,**kwargs)
        result=outcome.get('result');independent=outcome.get('independent_audit',{})
        # Only a collision-free, endpoint-valid curvature rejection with an
        # acknowledged current map can enter the local repair branch.
        if outcome.get('success') or not result or independent.get('failures')!=['CURVATURE']:
            return outcome
        if outcome.get('diagnostics',{}).get('costmap_update_acknowledged') is not True:return outcome
        started=time.monotonic();deadline=started+.25;mask=self._target_mask()
        p=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points]);maximum=1/self.world.vehicle[2]
        bad,lengths=violating_segments(p,maximum)
        log={'attempted':True,'success':False,'original_pose_hash':digest(p),'original_independent':independent,
             'original_metrics':outcome.get('metrics',{}),'trials':[],'budget_s':.25,'max_candidates':24,'max_window_m':6.}
        outcome['local_curvature_repair']=log
        if not len(bad):return outcome
        arc=np.r_[0,np.cumsum(lengths)];trials=0
        try:
            # Include one physical turning radius before/after the rejected
            # span so the fit can enter and leave its turn continuously.
            for margin in [self.world.vehicle[2],.15,.8,1.6]:
                i=max(0,int(np.searchsorted(arc,arc[bad.min()]-margin))-1)
                j=min(len(p)-1,int(np.searchsorted(arc,arc[bad.max()+1]+margin)))
                if arc[j]-arc[i]>6:continue
                left=float(wrap(p[i,2]-p[i-1,2])/lengths[i-1]) if i else 0.
                right=float(wrap(p[j+1,2]-p[j,2])/lengths[j]) if j<len(p)-1 else 0.
                for radius in [self.world.vehicle[2]+.05,self.world.vehicle[2]*1.5,self.world.vehicle[2]*2]:
                    for word,params in dubins_choices(p[i],p[j],radius):
                        if time.monotonic()>=deadline:raise TimeoutError('LOCAL_CURVATURE_BUDGET')
                        if sum(params)*radius>6 or trials>=24:continue
                        trials+=1;trial={'anchors':[i,j],'margin_m':margin,'radius_m':radius,'word':word};log['trials'].append(trial)
                        try:connector,fit=fit_connector(p[i],p[j],word,params,radius,left,right,deadline,maximum)
                        except ValueError as error:trial['rejection']=str(error);continue
                        full=np.r_[p[:i],connector,p[j+1:]]
                        valid,visits=ordered_occurrences(self.topology,self.topology_route,full)
                        if not valid:trial['rejection']='ORDERED_REGION_VISIT_MISSING';continue
                        check=self.world.audit(query,full,mask)
                        if not check['valid']:trial['rejection']=check['failures'];continue
                        points=truthful_points(full,result,i,i+len(connector)-1)
                        audit=auditor.audit(query,points,mask)
                        if not audit.final_valid_success:trial['rejection']=audit.metrics['failure_code'];continue
                        if time.monotonic()>=deadline:raise TimeoutError('LOCAL_CURVATURE_BUDGET')
                        log.update(success=True,fit=fit,anchors=[i,j],original_window_m=float(arc[j]-arc[i]),
                                   ordered_regions=list(self.topology_route.region_ids),visited_regions=visits,
                                   repaired_pose_hash=digest(full),repaired_points=len(connector))
                        repaired=replace(result,points=points,path_audit=audit,source='smac_with_local_curvature_repair')
                        outcome.update(success=True,failure_code='',result=repaired,metrics=audit.metrics,independent_audit=check,
                                       diagnostics={**outcome['diagnostics'],**audit.diagnostics()},local_curvature_repair=log)
                        return outcome
        except TimeoutError as error:log['failure']=str(error)
        finally:log['wall_ms']=(time.monotonic()-started)*1000
        log.setdefault('failure','NO_VALID_LOCAL_CONNECTOR')
        return outcome


class TurnaroundRecovery(CandidateRecovery):
    def __init__(self,*args,**kwargs):
        kwargs.setdefault('controller_class',TurnaroundController)
        super().__init__(*args,**kwargs)
