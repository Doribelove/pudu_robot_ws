"""Offline whole-route block SQP experiment; never executable navigation output."""
import ctypes,json,os,re,time,types
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
from replay_curve import load
from three_yd_v5 import curve_l3,curve_native
from three_yd_v5.v10_storage import collision_checker

R=Path(__file__).resolve().parents[1]
O=R/'results/block_pose_probe';O.mkdir(exist_ok=False)
os.sched_setaffinity(0,{14})
code=Path(curve_l3.__file__).read_text()
code=code.replace('B.T@B+1e-4*(E.T@E)','B.T@B+1e-7*(E.T@E)')
needle="free_tangent=config.get('free_endpoint_tangent_scale',False)"
patch="""fitfixed=[0,1,n-2,n-1]
            if config.get('zero_start_acc'):
                cp[2]=cp[1]+(knots[5]-knots[3])/3*np.array([math.cos(start[2]),math.sin(start[2])]);fitfixed.append(2)
            if config.get('zero_goal_acc'):
                cp[-3]=cp[-2]-(knots[-4]-knots[-6])/3*np.array([math.cos(goal[2]),math.sin(goal[2])]);fitfixed.append(n-3)
            fitfixed=np.array(sorted(set(fitfixed)));fitfree=np.setdiff1d(np.arange(n),fitfixed)
            fitA=(B.T@B+1e-7*(E.T@E)+sparse.eye(n)*1e-8).tocsc()
            cp[fitfree]=spsolve(fitA[fitfree][:,fitfree],(B.T@targets)[fitfree]-fitA[fitfree][:,fitfixed]@cp[fitfixed])
            """
code=code.replace(needle,patch+needle)
needle="fixed_values=z[fixed_indices].copy();trust=config['trust_radius_m'];history=[]"
code=code.replace(needle,"fixed_indices=np.r_[fitfixed,fitfixed+n];"+needle)
old="""endpoint=sparse.vstack((B[0],B[-1],D[0],D[-1]),format='csc')
            Aeq=sparse.block_diag((endpoint,endpoint),format='csc')
            beq=np.array([start[0],goal[0],math.cos(start[2]),math.cos(goal[2]),start[1],goal[1],math.sin(start[2]),math.sin(goal[2])])"""
new="""endpoint=sparse.eye(n,format='csc')[fitfixed]
            Aeq=sparse.block_diag((endpoint,endpoint),format='csc')
            beq=np.r_[cp[fitfixed,0],cp[fitfixed,1]]"""
assert old in code;code=code.replace(old,new)
mod=types.ModuleType('three_yd_v5.curve_blocks_probe');mod.__file__=curve_l3.__file__
exec(compile(code,mod.__file__,'exec'),mod.__dict__)
foot=mod.footprint_constraints;viol=mod.violation
mod.footprint_constraints=lambda B,Y,v,hl,hw,axes:foot(B,Y,v,0.,0.,axes)
mod.violation=lambda p,v,speed,k,boxes,vehicle,axes,curvature_ratio=.98:viol(p,v,speed,k,boxes,(0.,0.,vehicle[2]),axes,curvature_ratio)
lib=ctypes.CDLL(str(R/'experiments/libcurve_rectangles_center.so'));fn=lib.curve_rectangles_center
fn.argtypes=curve_native._lib.curve_rectangles.argtypes;fn.restype=ctypes.c_int
def rectangles(hard,res,origin,guide,deadline,tangents=None):
    h=np.ascontiguousarray(hard,dtype=np.uint8);p=np.ascontiguousarray(guide,dtype=float);t=np.ascontiguousarray(tangents,dtype=float)
    out=np.empty((len(p),4));axes=np.empty_like(out)
    result=fn(h.ctypes.data,*h.shape,float(res),*map(float,origin[:2]),p.ctypes.data,t.ctypes.data,len(p),curve_native._offsets.ctypes.data,0,max(0.,deadline-time.monotonic()),out.ctypes.data,axes.ctypes.data)
    if result:raise curve_l3.CurveFailure('CURVE_BUDGET' if result==1 else 'CURVE_GUIDE_OUTSIDE_SCOPE')
    out+=np.array([-.0195,.0195,-.0195,.0195])
    return out,axes
curve_native.rectangles=rectangles
w,d,m=load(R/'results/scope_recovery_probe/pudu_wanda_3f/curve_inputs/Q09_004.npz')
p=np.load(R/'results/semantic_witness_probe/guide.npy');s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
checker=collision_checker(w.hard|~d['physical'],w)
cuts=[0.];poses=[d['start']]
for i,length in enumerate(np.diff(s)):
    at=(s[i]+s[i+1])/2
    if length<4 or at<15 or s[-1]-at<15:continue
    v=p[i+1]-p[i];pose=np.r_[(p[i]+p[i+1])/2,np.arctan2(v[1],v[0])]
    if checker.check(pose[None,:]):cuts.append(at);poses.append(pose)
cuts.append(s[-1]);poses.append(d['goal']);models=[];records=[]
for j,(lo,hi,a,b) in enumerate(zip(cuts[:-1],cuts[1:],poses[:-1],poses[1:])):
    guide=np.vstack((a[:2],p[(s>lo)&(s<hi)],b[:2]))
    cfg=dict(m['config'],r4_rounded_guide=True,tracking_weight=1000.,bending_weight=.0001,jerk_weight=.000001,r4_guide_epsilon_cap=0.,constraint_step_m=.025,r4_adaptive_stations=False,adaptive_constraint_stations=True,certified_quadratic_start=False,r4_defer_recovery=False,r4_recovery_only=False,curvature_target_ratio=.99,r4_tighter_primary=False,max_refinements=1,max_iterations=40,r5_early_seed=False,straight_knot_m=2.,turn_knot_m=.08,body_refinement_passes=6,r4_endpoint_threshold_deg=25,zero_start_acc=j>0,zero_goal_acc=j<len(cuts)-2)
    path,model,log=mod._optimize_mesh(w,d['physical'],guide,a,b,time.monotonic()+5.,cfg)
    print(j,hi-lo,path is not None,log.get('failure'),flush=True)
    rec={'block':j,'arc':[lo,hi],'start':a.tolist(),'goal':b.tolist(),'model':model,'log':log};records.append(rec)
    (O/f'block{j}.json').write_text(json.dumps(rec,indent=2))
    if path is not None:np.save(O/f'block{j}.npy',path)
    models.append(model if path is not None else None)
if all(x is not None for x in models):
    knots=np.array(models[0]['knots']);cp=np.array(models[0]['coefficients']);corridor=models[0]['corridor'];joins=[]
    for model in models[1:]:
        offset=knots[-1];sp=BSpline(knots,cp,3);next_sp=BSpline(model['knots'],model['coefficients'],3)
        joins.append({str(order):float(np.max(abs(sp(offset,nu=order)-next_sp(0.,nu=order)))) for order in range(3)})
        knots=np.r_[knots[:-1],np.array(model['knots'])[4:]+offset];cp=np.vstack((cp,np.array(model['coefficients'])[1:]))
        corridor['stations']+=list(np.array(model['corridor']['stations'])[1:]+offset)
        corridor['bounds']+=model['corridor']['bounds'][1:];corridor['axes']+=model['corridor']['axes'][1:]
    assert all(max(x.values())<1e-7 for x in joins),joins
    try:
        path,certificate=curve_l3.certify(BSpline(knots,cp,3),w,d['physical'],time.monotonic()+5.,dict(m['config'],body_refinement_passes=6),corridor)
        np.save(O/'whole_path.npy',path);(O/'whole_curve.json').write_text(json.dumps({'knots':knots.tolist(),'coefficients':cp.tolist(),'corridor':corridor,'certificate':certificate,'joins':joins}))
        print('WHOLE_GEOMETRY_CERTIFIED',flush=True)
    except Exception as e:print('WHOLE_REJECTED',str(e),flush=True)
