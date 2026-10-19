"""Validate straight reference spans; solve bounded local vehicle connections.

The proposal has no success status until the original complete-path audits
pass. Local batches share one current map publication/real ACK. A rejected
proposal returns only to the inherited bounded full-route solver.
"""
import math,time
from contextlib import nullcontext
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from arena_3d_v1.semantic_world import densify,wrap
from arena_3d_v1.curvature_repair import truthful_points,violating_segments
from arena_evaluation.unified_four_backends_smoke import PlanResult
from three_yd_v4.local_l3 import reference_poses,risk_segments,repair_windows
from .v9_options import enabled
from .v9_maps import content_digest

def anchored_connector(world,p,a,b,mask,deadline,checker=None):
    """Fit actual geometry to the ORIGINAL anchor poses and adjacent curvature.

    Frozen Smac may return endpoint poses within its allowed tolerance, which
    is insufficient for an exact splice. Never hide that offset by changing yaw
    metadata. Short collision-free connectors can repair the real geometry.
    """
    from .r10_connector import fit_connector
    from arena_evaluation.semantic_constraint_core import dubins_choices
    from arena_3d_v1.semantic_world import SweptChecker
    from .v9_review import enabled as review_enabled
    joint_detail=None
    if review_enabled('J'):
        from .v9_joints import connect
        if checker is None:
            checker=SweptChecker(world.hard|~mask,world.map.resolution,world.map.origin,*world.vehicle[:2])
        patch,joint_detail=connect(world,p,a,b,checker,min(deadline,time.monotonic()+.08))
        if patch is not None:return patch,joint_detail
    length=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    left=float(wrap(p[a,2]-p[a-1,2])/length[a-1]) if a else 0.
    right=float(wrap(p[b+1,2]-p[b,2])/length[b]) if b<len(p)-1 else 0.
    nominal=float(length[a:b].sum());candidates=[]
    for radius in (.45,.6,.8):
        for word,par in dubins_choices(p[a],p[b],radius):
            distance=sum(par)*radius
            if distance>min(6.,nominal*1.25+.2):continue
            candidates.append((distance,radius,word,par))
    if checker is None:checker=SweptChecker(world.hard|~mask,world.map.resolution,world.map.origin,*world.vehicle[:2])
    trials=0
    for _,radius,word,par in sorted(candidates,key=lambda item:item[0])[:4]:
        for slack in (.005,.02,.05):
            if time.monotonic()>=deadline:return None,{'success':False,'failure':'ANCHOR_FIT_BUDGET','trials':trials}
            if 'S' not in word:continue
            adjusted=list(par);adjusted[word.index('S')]+=slack/radius;trials+=1
            try:patch,fit=fit_connector(p[a],p[b],word,adjusted,radius,left,right,deadline,1/world.vehicle[2])
            except ValueError:continue
            except TimeoutError:return None,{'success':False,'failure':'ANCHOR_FIT_BUDGET','trials':trials}
            splice=np.vstack((p[max(0,a-1):a],patch,p[b+1:min(len(p),b+2)]))
            if len(violating_segments(splice,1/world.vehicle[2])[0]):continue
            if not checker.check(densify(splice)):continue
            return patch,{'success':True,'trials':trials,'fit':fit,'exact_original_anchor_poses':True,'analytic_joint':joint_detail}
    return None,{'success':False,'failure':'NO_VALID_ANCHORED_CONNECTOR','trials':trials,'analytic_joint':joint_detail}


def heading_error(p):
    d=np.diff(p,axis=0);ds=np.linalg.norm(d[:,:2],axis=1)
    error=abs(wrap(np.arctan2(d[:,1],d[:,0])-(p[:-1,2]+wrap(d[:,2])/2)))
    return float(np.max(error[ds>1e-8],initial=0.))

def fit_smac_anchors(world,q,start,goal,mask,deadline):
    """Smac's tolerated endpoints are a proposal, requiring real refitting."""
    q=q.copy();q[0]=start;q[-1]=goal
    bad,_=risk_segments(world,q,mask)
    windows,_,_=repair_windows(q,bad,margin=1.2)
    logs=[]
    if len(windows)>6:return None,{'failure':'SMAC_ANCHOR_WINDOWS_LIMIT'}
    for a,b in reversed(windows):
        patch,detail=anchored_connector(world,q,a,b,mask,deadline)
        logs.append(detail)
        if patch is None:return None,{'failure':'SMAC_ANCHOR_REFIT_REJECTED','fits':logs}
        q=np.vstack((q[:a],patch,q[b+1:]))
    if len(risk_segments(world,q,mask)[0]) or heading_error(q)>math.radians(5):
        return None,{'failure':'SMAC_ANCHOR_FINAL_GEOMETRY','fits':logs}
    return q,{'success':True,'fits':logs,'exact_original_anchors':True}

def plan(c,query,session,auditor,spec,*,cap=140):
    from .v9_r2_options import enabled as r2_enabled
    started=time.monotonic();version=0 if c.dynamic is None else c.dynamic.version
    mask=c._target_mask();mask_hash=content_digest(mask)
    if tuple(c.world.vehicle)!=tuple(session.bound_vehicle):raise ValueError('VEHICLE_VERSION_MISMATCH')
    if (c.reference_plan.world_key!=c.world.key or
        c.reference_plan.fine_corridor_hash!=content_digest(c.plan.corridor_mask)):
        raise ValueError('REFERENCE_BINDING_MISMATCH')
    log={'attempted':True,'success':False,'bounded_whole_fallback':False,'global_fallback':False,
         'shared_map_and_ack':True,'max_windows':20,'vehicle_budget_s':3.,'connections':[]}
    ds=[];ack={}
    def rejected(reason):
        c.trajectory=None;c.trajectory_version=None
        log.update(failure=reason,wall_ms=(time.monotonic()-started)*1000)
        from .early_sections import merge_diagnostics
        diagnostics=merge_diagnostics(ds) if ds else {}
        return {'success':False,'failure_code':reason,'v9_local':log,'diagnostics':diagnostics}
    if not c.initial_l2_result.success:return rejected('V9_L2_REFERENCE_INVALID')
    p=reference_poses(c.reference_plan.world_xy,query.start,query.goal)
    if enabled('B'):
        from .v9_segments import propose
        p,segment_log=propose(c,p,mask);log['segments']=segment_log
        if not enabled('C') and not r2_enabled('P') and not segment_log['matched_segments']:return None
    if r2_enabled('P'):
        from .v9_risk import kinematic_risks,risk_segments as indexed_risks
        cheap_bad,_=kinematic_risks(c.world,p)
        cheap_windows,cheap_coverage,cheap_length=repair_windows(p,cheap_bad,margin=1.2)
        if len(cheap_windows)>20 or cheap_coverage>max(12.,.75*cheap_length):
            log.update(kinematic_early_rejection=True,windows=cheap_windows,local_length_m=cheap_coverage,total_length_m=cheap_length)
            return rejected('V9_LOCAL_COVERAGE_LIMIT')
        from arena_3d_v1.semantic_world import SweptChecker
        checker=SweptChecker(c.world.hard|~mask,c.world.map.resolution,c.world.map.origin,*c.world.vehicle[:2])
        bad,_=indexed_risks(c.world,p,mask,checker)
        log['risk_densification_reused']=True
    else:bad,_=risk_segments(c.world,p,mask)
    windows,coverage,length=repair_windows(p,bad,margin=1.2)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
    # A reversed endpoint heading requires a real turn, not an arbitrarily
    # short endpoint window. Expand only along the same bounded reference.
    if windows and windows[0][0]==0 and abs(wrap(p[1,2]-p[0,2]))>math.pi/6:
        windows[0][1]=max(windows[0][1],min(len(p)-1,int(np.searchsorted(arc,3.))))
    if windows and windows[-1][1]==len(p)-1 and abs(wrap(p[-1,2]-p[-2,2]))>math.pi/6:
        windows[-1][0]=min(windows[-1][0],max(0,int(np.searchsorted(arc,arc[-1]-3.))-1))
    merged=[]
    for a,b in windows:
        if merged and a<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],b)
        else:merged.append([a,b])
    windows=merged;coverage=sum(float(arc[b]-arc[a]) for a,b in windows)
    log.update(risk_segments=len(bad),windows=windows,local_length_m=coverage,
        total_length_m=length,straight_validated_length_m=max(0.,length-coverage))
    if len(windows)>20 or coverage>max(12.,.75*length):return rejected('V9_LOCAL_COVERAGE_LIMIT')
    preflight=r2_enabled('P')
    if preflight:
        from .v9_guided_joint import guided_connect
        vehicle_started=time.monotonic()
        available=session.budget.remaining() if hasattr(session,'budget') else .35
        deadline=vehicle_started+min(.35,max(0.,available))
        log.update(analytic_preflight=True,pre_ack_geometry_only=True,vehicle_budget_s=.35,actual_map_ACK_performed=False)
        try:
            for a,b in reversed(windows):
                if time.monotonic()>=deadline:return rejected('V9_FAST_PREFLIGHT_BUDGET')
                patch,detail=guided_connect(c.world,p,a,b,checker,min(deadline,time.monotonic()+.07))
                call={'anchors':[a,b],'Smac_called':False,'anchor_fit':detail};log['connections'].append(call)
                if __import__('os').environ.get('THREEYD_V9_R2_CAPTURE')=='1':
                    call['reference_window_poses']=p[max(0,a-1):min(len(p),b+2)].tolist()
                    call['window_start_index']=int(a-max(0,a-1));call['window_end_index']=int(b-max(0,a-1))
                if patch is None:return rejected('V9_FAST_PREFLIGHT_NO_COMPLETE_GEOMETRY')
                call.update(seam_position_error_m=float(max(np.linalg.norm(patch[0,:2]-p[a,:2]),np.linalg.norm(patch[-1,:2]-p[b,:2]))),
                    seam_yaw_error_rad=float(max(abs(wrap(patch[0,2]-p[a,2])),abs(wrap(patch[-1,2]-p[b,2])))))
                p=np.vstack((p[:a],patch,p[b+1:]))
            splice_bad,_=violating_segments(p,1/c.world.vehicle[2])
            if len(splice_bad) or heading_error(p)>math.radians(5):return rejected('V9_FAST_PREFLIGHT_KINEMATICS')
            if not checker.check(densify(p)):return rejected('V9_FAST_PREFLIGHT_BODY_OR_SCOPE')
        finally:
            preflight_ms=(time.monotonic()-vehicle_started)*1000;log['preflight_ms']=preflight_ms
            if hasattr(session,'budget'):session.budget.vehicle_used+=preflight_ms/1000
    # Real publication, fresh complete server readback and content comparison.
    ack=c.sync_only(session);ds.append({**ack,'backend_call_count':0,'l3_action_wall_ms':0.})
    if preflight:log['actual_map_ACK_performed']=bool(ack.get('costmap_update_acknowledged'))
    if ack.get('costmap_update_acknowledged') is not True:return rejected('V9_LOCAL_MAP_ACK_FAILED')
    c.pending_l3_mask=mask;c.server_l3_mask=mask
    available=session.budget.remaining() if hasattr(session,'budget') else 3.
    deadline=time.monotonic()+min(3.,max(0.,available));vehicle_started=time.monotonic()
    template=SimpleNamespace(source='v9_validated_reference',planner_backend=spec.backend,backend_version=spec.version)
    context=session.v9_batch(mask,ack) if hasattr(session,'v9_batch') else nullcontext()
    checker=None
    from .v9_review import enabled as review_enabled
    if review_enabled('J'):
        from arena_3d_v1.semantic_world import SweptChecker
        checker=SweptChecker(c.world.hard|~mask,c.world.map.resolution,c.world.map.origin,*c.world.vehicle[:2])
    try:
        with context:
            for a,b in reversed([] if preflight else windows):
                if time.monotonic()>=deadline:return rejected('V9_LOCAL_BUDGET')
                call={'anchors':[a,b],'Smac_called':False}
                patch,detail=anchored_connector(c.world,p,a,b,mask,min(deadline,time.monotonic()+.45),checker)
                call['anchor_fit']=detail
                if patch is None and time.monotonic()<deadline:
                    # Exact anchor fit with obstacle residuals. Its sampled
                    # field is only an optimizer hint; exact sweeps decide.
                    from three_yd_v4.geometry_repair import repair_segment
                    patch,detail=repair_segment(c.world,p,a,b,mask,min(deadline,time.monotonic()+.45))
                    call['obstacle_fit']=detail
                if patch is None and time.monotonic()<deadline:
                    local=replace(query,query_id=f'{query.query_id}_v9_local_{a}_{b}',start=tuple(p[a]),goal=tuple(p[b]))
                    saved=session.client.timeout;session.client.timeout=min(saved,max(.01,deadline-time.monotonic()))
                    try:result=session.plan(local,spec,source='v9_local_frozen_smac',allowed_mask=mask,skip_path_mask_validation=True)
                    finally:session.client.timeout=saved
                    call['Smac_called']=True;call['Smac_success']=bool(result.planner_success);ds.append(result.diagnostics or {})
                    if (result.diagnostics or {}).get('costmap_update_acknowledged') is not True:
                        return rejected('V9_LOCAL_ACTION_ACK_FAILED')
                    # A tolerance-offset Smac endpoint cannot be spliced by
                    # changing pose metadata. Use only exact anchor geometry.
                    if result.planner_success and result.points:
                        q=np.array([[v['x'],v['y'],v['yaw']] for v in result.points])
                        patch,detail=fit_smac_anchors(c.world,q,p[a],p[b],mask,min(deadline,time.monotonic()+.45))
                        call['Smac_anchor_refit']=detail
                        if patch is not None:template=result
                log['connections'].append(call)
                if patch is None:return rejected('V9_LOCAL_CONNECTION_REJECTED')
                call.update(seam_position_error_m=float(max(np.linalg.norm(patch[0,:2]-p[a,:2]),np.linalg.norm(patch[-1,:2]-p[b,:2]))),
                    seam_yaw_error_rad=float(max(abs(wrap(patch[0,2]-p[a,2])),abs(wrap(patch[-1,2]-p[b,2])))))
                p=np.vstack((p[:a],patch,p[b+1:]))
    finally:
        if hasattr(session,'budget') and not preflight:session.budget.vehicle_used+=time.monotonic()-vehicle_started
    points=truthful_points(p,template,0,len(p)-1)
    ca,ind,traffic,jump,failures=c._audit(query,p,mask,auditor,points)
    motion=heading_error(p)
    if motion>math.radians(5)+1e-6:failures.append('V9_HEADING_MOTION_MISMATCH')
    if (c.dynamic is not None and c.dynamic.version!=version) or content_digest(c._target_mask())!=mask_hash:
        return rejected('V9_MAP_VERSION_OR_BINDING_CHANGED')
    if failures:return rejected('V9_POST_SPLICE_REJECTED:'+';'.join(failures))
    from .early_sections import merge_diagnostics
    diagnostics={**merge_diagnostics(ds),**ca.diagnostics(),'v9_shared_map_batch':True,
        'v9_local_vehicle_ms':(time.monotonic()-vehicle_started)*1000+(preflight_ms if preflight else 0.)}
    result=PlanResult(planner_success=True,points=points,failure_code='',planner_backend=spec.backend,
        backend_version=spec.version,source='v9_validated_straights_and_local_connections',diagnostics=diagnostics,path_audit=ca)
    c.trajectory=p;c.trajectory_version=version
    log.update(success=True,wall_ms=(time.monotonic()-started)*1000,max_heading_motion_error_deg=math.degrees(motion),
        final_steering_jump_deg=jump,final_curvature_bad_segments=len(violating_segments(p,1/c.world.vehicle[2])[0]),
        final_complete_audit=True)
    return {'success':True,'geometry_success':True,'failure_code':'','result':result,'called':bool(diagnostics.get('backend_call_count')),
        'metrics':ca.metrics,'independent_audit':ind,'traffic_audit':traffic,
        'constraint_audit':{'max_raw_steering_jump_deg':jump,'steering_jump_valid':True,'traffic':traffic},
        'diagnostics':diagnostics,'map_version':version,'v9_local':log,
        'reference_diagnostics':c._bounded_grid.diagnostics,'legacy_contract_success':True}
