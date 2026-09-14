"""Bounded route enumeration coupled with nonlinear trajectory optimization.

This is NOT a GCS convex relaxation. Only the geometric regions are convex.
Cubic control hulls stay in regions; shared physical velocity enforces C1.
Internal knot positions and tangent vectors (therefore yaw) are variables.
"""
import time
import math
import numpy as np
from scipy.optimize import minimize,linprog
from .bezier import evaluate,certify_curvature
from .logic import candidate_routes,overlap,witness_segments


class BudgetExpired(RuntimeError):
    pass


def _angle_delta(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))


def solve_route(scene,route,deadline,maxiter=180,transition_mode="joint",segments_per_region=1,feasibility_first=False):
    n=len(route);regs=[scene.regions[i] for i in route]
    boxes=[np.r_[scene.start[:2],scene.start[:2]]]
    boxes += [overlap(a,b) for a,b in zip(regs[:-1],regs[1:])]
    boxes += [np.r_[scene.goal[:2],scene.goal[:2]]]
    boxes=np.array(boxes)
    q=(boxes[:,:2]+boxes[:,2:])/2
    # Project every initial interface into BOTH regions' semantic halfspaces.
    # All ablations share these identical seeds; none fixes an illegal midpoint.
    for i in range(1,n):
        hs=regs[i-1].halfspaces+regs[i].halfspaces
        if hs:
            ab=np.asarray(hs,float);middle=q[i].copy()
            mat=np.r_[np.c_[ab[:,:2],np.zeros(len(ab))],[[1,0,-1],[-1,0,-1],[0,1,-1],[0,-1,-1]]]
            rhs=np.r_[ab[:,2],middle[0],-middle[0],middle[1],-middle[1]]
            projected=linprog([0,0,1],A_ub=mat,b_ub=rhs,bounds=[(boxes[i,0],boxes[i,2]),(boxes[i,1],boxes[i,3]),(0,None)],method='highs')
            if not projected.success:return {'status':'EMPTY_SEMANTIC_INTERFACE','route':route},None
            q[i]=projected.x[:2]
    if segments_per_region not in [1,2,3]:raise ValueError('INVALID_SEGMENT_SUBDIVISION')
    if segments_per_region>1:
        new_q=[q[0]];new_boxes=[boxes[0]];expanded=[]
        for i in range(n):
            for j in range(1,segments_per_region+1):
                expanded.append(route[i]);new_q.append(q[i]+(q[i+1]-q[i])*j/segments_per_region)
                new_boxes.append(boxes[i+1] if j==segments_per_region else regs[i].bounds)
        q=np.array(new_q);boxes=np.array(new_boxes);route=expanded;n=len(route);regs=[scene.regions[i] for i in route]
    if transition_mode not in ["joint","fixed_position","fixed_pose"]:raise ValueError("INVALID_TRANSITION_MODE")
    if transition_mode!="joint":boxes=np.c_[q,q]
    # Internal centers seed only; optimization bounds are full intersections.
    delta=np.diff(q,axis=0);length=np.linalg.norm(delta,axis=1)
    if np.any(length<1e-5):
        return {'status':'DEGENERATE_SEED_SEGMENT','route':route},None
    direction=delta/length[:,None]
    v=np.empty_like(q);v[0]=[math.cos(scene.start[2]),math.sin(scene.start[2])]
    v[-1]=[math.cos(scene.goal[2]),math.sin(scene.goal[2])]
    if n>1:
        middle=direction[:-1]+direction[1:]
        norms=np.linalg.norm(middle,axis=1)
        middle[norms<1e-6]=direction[:-1][norms<1e-6]
        v[1:-1]=middle/np.maximum(np.linalg.norm(middle,axis=1)[:,None],1e-9)
    seed_yaw=np.arctan2(v[:,1],v[:,0])
    v*=scene.speed_max_mps*0.45
    durations=length/(scene.speed_max_mps*0.45)
    initial=np.r_[q.ravel(),v.ravel(),durations]
    nq=2*(n+1);nv=nq;vmax=scene.speed_max_mps
    bounds=list(zip(boxes[:,:2].ravel(),boxes[:,2:].ravel()))
    bounds += [(-vmax,vmax)]*nv+[(0.05,scene.task.horizon_s)]*n
    u=np.linspace(0,1,17)
    lower=np.array([r.bounds[:2] for r in regs])[:,None,:]
    upper=np.array([r.bounds[2:] for r in regs])[:,None,:]
    witnesses=witness_segments(scene,route)
    calls=0

    def unpack(z):
        pos=z[:nq].reshape(-1,2);vel=z[nq:nq+nv].reshape(-1,2);dt=z[nq+nv:]
        cp=np.stack([pos[:-1],pos[:-1]+dt[:,None]*vel[:-1]/3,
                     pos[1:]-dt[:,None]*vel[1:]/3,pos[1:]],axis=1)
        return pos,vel,dt,cp

    def check_budget():
        if time.monotonic()>deadline:
            raise BudgetExpired

    def objective(z):
        check_budget()
        pos,vel,dt,cp=unpack(z)
        p,_,_=evaluate(cp,u)
        lengths=np.linalg.norm(np.diff(p,axis=1),axis=2).sum()
        return float(dt.sum()+0.1*lengths+0.02*np.sum(np.diff(vel,axis=0)**2))

    def eq(z):
        _,vel,_,_=unpack(z)
        headings=np.array([scene.start[2],scene.goal[2]])
        if transition_mode=="fixed_pose":return vel[:,0]*np.sin(seed_yaw)-vel[:,1]*np.cos(seed_yaw)
        return vel[[0,-1],0]*np.sin(headings)-vel[[0,-1],1]*np.cos(headings)

    def inequalities(z):
        nonlocal calls
        calls+=1;check_budget()
        pos,vel,dt,cp=unpack(z)
        _,d,a=evaluate(cp,u)
        norm2=np.sum(d*d,axis=-1)
        cross=d[:,:,0]*a[:,:,1]-d[:,:,1]*a[:,:,0]
        # Squared curvature expression avoids division in the optimizer.
        kmax=0.97/scene.radius_min_m
        vals=[(cp-lower).ravel(),(upper-cp).ravel(),
              (kmax*kmax-cross**2/np.maximum(norm2**3,1e-24)).ravel(),
              (norm2/dt[:,None]**2-0.025**2).ravel(),
              np.array([scene.task.horizon_s-dt.sum()])]
        qv=3*np.diff(cp,axis=1)/dt[:,None,None]
        vals.append((vmax*vmax-np.sum(qv*qv,axis=-1)).ravel())
        for index,endpoint in [(0,scene.start),(-1,scene.goal)]:
            vals.append(np.array([vel[index]@np.array([math.cos(endpoint[2]),math.sin(endpoint[2])])-0.025]))
        if transition_mode=="fixed_pose":
            vals.append(vel[:,0]*np.cos(seed_yaw)+vel[:,1]*np.sin(seed_yaw)-.025)
        for i,r in enumerate(regs):
            for a0,b0,c0 in r.halfspaces:
                vals.append(c0-a0*cp[i,:,0]-b0*cp[i,:,1])
            if r.direction is not None:
                angle,half=r.direction
                axis=np.array([math.cos(angle),math.sin(angle)])
                normal=np.array([-axis[1],axis[0]])
                along=qv[i]@axis;across=qv[i]@normal
                vals.extend([along-0.001,math.tan(half)*along-across,math.tan(half)*along+across])
        times=np.r_[0.,np.cumsum(dt)]
        for seg,rule in witnesses:
            lo,hi=rule['window_s'];vals.append(np.array([times[seg]-lo,hi-times[seg]]))
        return np.concatenate(vals)

    started=time.monotonic()
    try:
        restoration_info=None
        if feasibility_first:
            restoration_started=time.monotonic()
            def violation(z):
                negative=np.minimum(inequalities(z),0.)
                return float(np.sum(negative*negative))
            restoration=minimize(violation,initial,method='SLSQP',bounds=bounds,
                constraints=[{'type':'eq','fun':eq}],options={'maxiter':min(100,maxiter),'ftol':1e-10,'disp':False})
            initial=restoration.x
            restoration_info={'wall_s':time.monotonic()-restoration_started,'iterations':int(restoration.nit),
                'minimum_constraint':float(np.min(inequalities(initial))),'success':bool(restoration.success)}
        result=minimize(objective,initial,method='SLSQP',bounds=bounds,
                        constraints=[{'type':'eq','fun':eq},{'type':'ineq','fun':inequalities}],
                        options={'maxiter':maxiter,'ftol':1e-8,'disp':False})
        z=result.x
        cmin=float(np.min(inequalities(z)));eres=float(np.max(np.abs(eq(z))))
        pos,vel,dt,cp=unpack(z)
        log={'status':'OPTIMIZER_CANDIDATE','route':route,'optimizer_success':bool(result.success),
             'message':str(result.message),'iterations':int(result.nit),'constraint_min':cmin,
             'equality_max':eres,'objective':float(result.fun),'wall_s':time.monotonic()-started,
             'restoration':restoration_info,'segments_per_region':segments_per_region,
             'constraint_calls':calls,'transition_mode':transition_mode,'initial_yaw':seed_yaw.tolist(),'initial_knots':q.tolist(),
             'optimized_knots':pos.tolist(),'optimized_yaw':np.arctan2(vel[:,1],vel[:,0]).tolist(),
             'knot_displacement_max_m':float(np.max(np.linalg.norm(pos-q,axis=1)))}
        _,derivative,acceleration=evaluate(cp,np.linspace(0,1,101))
        norm=np.linalg.norm(derivative,axis=-1)
        curvature=np.abs(derivative[:,:,0]*acceleration[:,:,1]-derivative[:,:,1]*acceleration[:,:,0])/np.maximum(norm**3,1e-24)
        worst=np.unravel_index(np.argmax(curvature),curvature.shape)
        log['residual_diagnostics']={'sampled_max_curvature_per_m':float(curvature[worst]),
            'worst_curvature_segment':int(worst[0]),'curvature_limit_per_m':1/scene.radius_min_m,
            'control_hull_box_violation_m':float(max(0,np.max(lower-cp),np.max(cp-upper))),
            'horizon_violation_s':float(max(0,dt.sum()-scene.task.horizon_s))}
        if not np.all(np.isfinite(z)) or cmin < -1e-7 or eres>1e-7:
            log['rejected_control_points']=cp.tolist();log['rejected_durations_s']=dt.tolist()
            log['status']='CONTINUOUS_CONSTRAINTS_NOT_SATISFIED';return log,None
        certs=[certify_curvature(p,1/scene.radius_min_m) for p in cp]
        if not all(c['valid'] for c in certs):
            log.update(status='CONTINUOUS_CURVATURE_NOT_CERTIFIED',curvature_certificates=certs)
            return log,None
        # Verify independent exact endpoint angle and physical C1 continuity.
        yaw=np.arctan2(vel[:,1],vel[:,0])
        if abs(_angle_delta(yaw[0],scene.start[2]))>1e-6 or abs(_angle_delta(yaw[-1],scene.goal[2]))>1e-6:
            log['status']='ENDPOINT_YAW_FAILED';return log,None
        log['status']='CERTIFIED_RESEARCH_CANDIDATE'
        return log,{'control_points':cp.tolist(),'durations_s':dt.tolist(),'knots':pos.tolist(),
                    'knot_velocity':vel.tolist(),'curvature_certificates':certs,'route':route,
                    'witnesses':[{'label':r['label'],'time_s':float(np.sum(dt[:i])),'segment':i,'window_s':r['window_s']} for i,r in witnesses]}
    except BudgetExpired:
        return {'status':'CONTINUOUS_SOLVER_BUDGET_EXHAUSTED','route':route,'wall_s':time.monotonic()-started},None


def plan(scene,budget_s=30.,candidates=8,maxiter=180,transition_mode="joint",refine_on_failure=True):
    if not math.isfinite(budget_s) or budget_s<=0 or candidates<1:
        raise ValueError('INVALID_BUDGET')
    started=time.monotonic();deadline=started+budget_s
    from .geometry_graph import prepare_geometry
    try:geometry,geometry_timing=prepare_geometry(scene,deadline)
    except TimeoutError:
        return {'status':'GRAPH_BUDGET_EXHAUSTED','research_valid':False,'production_accepted':False,
                'r2_semantic_acceptance':None,'search':{'status':'GRAPH_BUDGET_EXHAUSTED'},'candidate_results':[],
                'trajectory':None,'planning_wall_s':time.monotonic()-started,'selected_objective':None,
                'completeness_claim':False,'global_optimality_claim':False}
    search_started=time.monotonic()
    has_sparse=any('sparse_seed_cover' in region.labels for region in scene.regions)
    sparse=[];sparse_search=None
    if has_sparse:
        sparse,sparse_search=candidate_routes(scene,max(1,candidates//2),deadline=deadline,
            required_label='sparse_seed_cover',width_weight=0.,geometry=geometry)
    wide,wide_search=candidate_routes(scene,max(1,candidates//3),deadline=deadline,min_region_width=2*scene.radius_min_m,geometry=geometry)
    other,other_search=candidate_routes(scene,candidates,deadline=deadline,geometry=geometry)
    routes=[]
    for route in sparse+wide+other:
        if route not in routes and len(routes)<candidates:routes.append(route)
    search={'status':'CANDIDATES_FOUND' if routes else other_search['status'],
            'sparse_pass':sparse_search,'wide_pass':wide_search,'unrestricted_pass':other_search,
            'candidate_diversity':'fresh sparse cover, broad regions, unrestricted repaired cover',
            'wide_region_threshold_m':2*scene.radius_min_m,'completeness_claim':False}
    search_s=time.monotonic()-search_started;solve_started=time.monotonic()
    logs=[];best=None;best_cost=math.inf
    for route in routes:
        if time.monotonic()>=deadline:
            break
        log,trajectory=solve_route(scene,route,deadline,maxiter,transition_mode)
        logs.append(log)
        if trajectory is not None and log['objective']<best_cost:
            best=trajectory;best_cost=log['objective']
    refinement_attempts=0
    if best is None and refine_on_failure:
        # Retain the same discrete route; add freedom inside convex regions.
        # Only failed requests spend this budget, still under the same deadline.
        for route in routes[:2]:
            if time.monotonic()>=deadline:break
            log,trajectory=solve_route(scene,route,deadline,maxiter,transition_mode,segments_per_region=2)
            log['refinement_of_route']=route;logs.append(log);refinement_attempts+=1
            if trajectory is not None and log['objective']<best_cost:best=trajectory;best_cost=log['objective']
    return {'status':'RESEARCH_VALID' if best else ('NO_CERTIFIED_TRAJECTORY_WITHIN_BUDGET' if routes else search['status']),
            'transition_mode':transition_mode,'research_valid':best is not None,'production_accepted':False,'r2_semantic_acceptance':None,
            'r2_status':'NOT_EVALUATED_IN_STL_V0_R2','search':search,'candidate_results':logs,
            'trajectory':best,'planning_wall_s':time.monotonic()-started,
            'timing':{'geometry':geometry_timing,'candidate_search_s':search_s,'continuous_s':time.monotonic()-solve_started},
            'refinement_attempts':refinement_attempts,'refine_on_failure':refine_on_failure,
            'route_candidates_optimized':len(logs),'route_candidates_available':len(routes),
            'selected_objective':best_cost if best is not None else None,
            'solver':'bounded_product_graph_enumeration_plus_SLSQP_cubic_trajectory',
            'completeness_claim':False,'global_optimality_claim':False}
