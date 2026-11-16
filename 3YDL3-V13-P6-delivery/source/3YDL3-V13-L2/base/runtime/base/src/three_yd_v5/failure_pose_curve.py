"""Pose-seed spline recovery, followed by the unchanged physical certificate."""
import ctypes,time
from pathlib import Path
import numpy as np
from scipy import sparse
from scipy.interpolate import BSpline
from .failure_pose_cells import PoseCells

_LIB=None

def center_rectangles(hard,res,origin,guide,deadline,tangents=None):
    global _LIB
    from . import curve_native
    from .curve_l3 import CurveFailure
    if _LIB is None:
        _LIB=ctypes.CDLL(str(Path(__file__).resolve().parents[2]/'native/libcurve_rectangles_center.so'))
        _LIB.curve_rectangles_center.argtypes=curve_native._lib.curve_rectangles.argtypes
        _LIB.curve_rectangles_center.restype=ctypes.c_int
    hard=np.ascontiguousarray(hard,dtype=np.uint8);p=np.ascontiguousarray(guide,dtype=float);t=np.ascontiguousarray(tangents,dtype=float)
    out=np.empty((len(p),4));axes=np.empty_like(out)
    result=_LIB.curve_rectangles_center(hard.ctypes.data,*hard.shape,float(res),*map(float,origin[:2]),p.ctypes.data,t.ctypes.data,len(p),curve_native._offsets.ctypes.data,0,max(0.,deadline-time.monotonic()),out.ctypes.data,axes.ctypes.data)
    if result:raise CurveFailure('CURVE_BUDGET' if result==1 else 'CURVE_GUIDE_OUTSIDE_SCOPE')
    # The inherited mesh applies a .02 inset. The resulting center cell is
    # inside the exact obstacle-free rectangle by .0005; no body model changes.
    return out+np.array([-.0195,.0195,-.0195,.0195]),axes

def insert_knot(knots,cp,x):
    """Exact cubic Boehm refinement; no position/heading/curvature alteration."""
    if np.min(abs(knots-x))<1e-9:return knots,cp
    k=np.searchsorted(knots,x,side='right')-1;p=3
    if not knots[p]<x<knots[-p-1]:return knots,cp
    out=np.empty((len(cp)+1,2));out[:k-p+1]=cp[:k-p+1];out[k+1:]=cp[k:]
    for i in range(k-p+1,k+1):
        a=(x-knots[i])/(knots[i+p]-knots[i]);out[i]=a*cp[i]+(1-a)*cp[i-1]
    return np.insert(knots,k+1,x),out

def repair(world,mask,model,deadline,config):
    from . import curve_l3
    from .curve_qp import clarabel_setup,clarabel_solve
    knots=np.array(model['knots']);cp=np.array(model['coefficients']);hard=world.hard|~mask
    records=[];inserted_total=0
    for trial in range(6):
        curve_l3.check_deadline(deadline);sp=BSpline(knots,cp,3)
        try:
            path,certificate=curve_l3.certify(sp,world,mask,deadline,config,model['corridor'],hard=hard)
            parameters=certificate.pop('sample_parameters')
            return path,dict(model,knots=knots.tolist(),coefficients=cp.tolist(),parameters=parameters),{'certificate':certificate,'repairs':records,'inserted_knots':inserted_total}
        except curve_l3.CurveFailure as error:
            if str(error)!='CURVE_CONTINUOUS_BODY_OR_SCOPE' or not hasattr(error,'intervals'):raise
            left,right=error.intervals
        order=np.argsort(left);bands=[]
        for lo,hi in zip(left[order],right[order]):
            if bands and lo<=bands[-1][1]+.01:bands[-1][1]=max(bands[-1][1],float(hi))
            else:bands.append([float(lo),float(hi)])
        if len(bands)>64:raise curve_l3.CurveFailure('POSE_CURVE_REPAIR_SPATIAL_BUDGET')
        record={'bands':len(bands),'local_patches':0,'inserted_knots':0}
        for lo,hi in bands:
            curve_l3.check_deadline(deadline);sp=BSpline(knots,cp,3)
            u=np.r_[np.arange(lo,hi,.0005),hi];cells=PoseCells(world,hard,sp(u));_,gap,_,_=cells.gaps(sp(u),sp(u,1))
            if gap.min(initial=1.)>=.00025:
                for x in np.linspace(lo,hi,max(2,int(np.ceil((hi-lo)/.005)))+1)[1:-1]:
                    old=len(cp);knots,cp=insert_knot(knots,cp,float(x));record['inserted_knots']+=len(cp)-old
                inserted_total=record['inserted_knots']+sum(r['inserted_knots'] for r in records)
                if inserted_total>2048:raise curve_l3.CurveFailure('POSE_CURVE_REPAIR_KNOT_BUDGET')
                continue
            lo=max(knots[3],lo-.4);hi=min(knots[-4],hi+.4);u=np.r_[np.arange(lo,hi,.001),hi]
            n=len(cp);z=np.r_[cp[:,0],cp[:,1]]
            B=curve_l3.basis(knots,u);D=curve_l3.basis(knots,u,1);E=curve_l3.basis(knots,u,2);F=curve_l3.basis(knots,u,3)
            free=curve_l3.basis(knots,[lo,hi]).nonzero()[1];free=np.arange(max(2,free.min()-2),min(n-2,free.max()+3));variables=np.r_[free,n+free]
            cells=PoseCells(world,hard,B@cp)
            for iteration in range(3):
                curve_l3.check_deadline(deadline)
                pos,v,_,_,_,k,J,Y=curve_l3.differential(B,D,E,F,z);G,g=cells.linearize(B,Y,pos,v);g+=.0005-1e-6
                limit=.996/world.vehicle[2]
                A=sparse.vstack([G[:,variables],J[:,variables],-J[:,variables],sparse.eye(len(variables))],format='csc')
                lower=np.r_[np.full(len(g)+2*len(u),-np.inf),np.full(len(variables),-.02)]
                upper=np.r_[-g,limit-k,limit+k,np.full(len(variables),.02)]
                sol=clarabel_solve(clarabel_setup(sparse.eye(len(variables),format='csc')*2,np.zeros(len(variables)),A,lower,upper,deadline,1e-8))
                if sol.info.status_val not in (1,2):raise curve_l3.CurveFailure('POSE_CURVE_LOCAL_QP_'+sol.info.status)
                z[variables]+=sol.x;cp=np.c_[z[:n],z[n:]]
                if abs(sol.x).max()<1e-5:break
            record['local_patches']+=1
        records.append(record)
    raise curve_l3.CurveFailure('POSE_CURVE_REPAIR_LIMIT')

def optimize_pose(world,mask,poses,start,goal,deadline,config,original_guide):
    from . import curve_l3
    tick=time.monotonic()
    cfg=dict(config,r8_pose_curve=True,r4_rounded_guide=False,tracking_weight=1000.,bending_weight=.0001,jerk_weight=.000001,r4_guide_epsilon_cap=0.,constraint_step_m=.02,r4_adaptive_stations=False,adaptive_constraint_stations=True,certified_quadratic_start=False,r4_defer_recovery=False,r4_recovery_only=False,curvature_target_ratio=.99,r4_tighter_primary=False,max_refinements=0,max_iterations=12,r5_early_seed=False)
    path,model,log=curve_l3._optimize_mesh(world,mask,poses[:,:2],start,goal,deadline,cfg)
    log['r8_pose_curve']={'initialization_only_body_samples':len(poses),'original_physical_certificate':True}
    if path is None and model is not None and time.monotonic()<deadline:
        try:
            path,model,repaired=repair(world,mask,model,deadline,config)
            log.update(success=True,certificate=repaired['certificate'],r8_certificate_repair=repaired)
        except curve_l3.CurveFailure as error:log.update(success=False,failure=str(error))
    if path is not None:
        _,arc=curve_l3.clean_guide(original_guide,start,goal,.22)
        length=float(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum())
        if length>config['max_length_ratio_to_guide']*arc[-1]:path=None;log.update(success=False,failure='CURVE_LENGTH_GATE')
        else:log.update(success=True,length_m=length);log.pop('failure',None)
    log['wall_ms']=(time.monotonic()-tick)*1000
    return path,model,log
