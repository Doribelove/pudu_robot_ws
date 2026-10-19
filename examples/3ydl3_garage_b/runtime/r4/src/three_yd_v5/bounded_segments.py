"""Iteration-limit recovery inside the original L2 tube, with whole-path audit.

No new L1 route, no wider costmap, no relaxed vehicle acceptance. At most four
Smac calls share eight seconds. A returned section is never final success.
"""
from dataclasses import replace
import math,time
from .steering_rejection import repairable_steering_failure
import numpy as np
from arena_3d_v1.curvature_repair import truthful_points
from .endpoint_repair import repair_path_extended


def segment_anchors(xy,start,goal,max_sections=4):
    xy=np.asarray(xy,float)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    count=min(max_sections,max(2,int(math.ceil(arc[-1]/100.))))
    # Prefer a straight portion. Moving a split along the existing reference
    # does not create or change its geometry, scope, or soft cost.
    anchors=[tuple(start)]
    for target in np.linspace(0.,arc[-1],count+1)[1:-1]:
        eligible=[]
        for i in range(len(xy)-1):
            lo,hi=arc[i]+2.,arc[i+1]-2.
            if hi<lo:continue
            at=float(np.clip(target,lo,hi));eligible.append((abs(at-target),i,at))
        if not eligible:raise ValueError('NO_STRAIGHT_SEGMENT_ANCHOR')
        _,i,at=min(eligible)
        if abs(at-target)>10.:raise ValueError('SEGMENT_ANCHOR_SHIFT_LIMIT')
        delta=xy[i+1]-xy[i];point=xy[i]+delta*((at-arc[i])/(arc[i+1]-arc[i]))
        anchors.append((*map(float,point),math.atan2(delta[1],delta[0])))
    anchors.append(tuple(goal))
    return anchors


def strategy_cache(controller,query):
    from arena_3d_v1.semantic_world import digest
    from .r6_cache import get_cache
    key=('V8-r1-bounded-sections',controller.world.key,controller._dynamic_identity(),
         tuple(query.start),tuple(query.goal),controller.topology_route.signature,
         digest(controller.reference_plan.world_xy),controller.reference_plan.fine_corridor_hash,
         digest(np.asarray(controller.obstacle_repair_points)),
         digest(np.asarray(controller.rectangle_bridge_points)),.8,1.2)
    return get_cache(controller.views,'r1-section-strategy',max_entries=16),key


def recover(controller,query,session,auditor,spec,initial,*,cap=140,budget_s=8.):
    from three_yd_v4.r5_controller import SoftEdgeController
    started=time.monotonic();deadline=started+budget_s
    log={'attempted':True,'success':False,'trigger': 'EARLY_SECTION_POLICY' if initial.get('failure_code')=='EARLY_SECTION_POLICY' else 'SEARCH_ITERATION_LIMIT',
         'max_sections':4,'budget_s':budget_s,'cached_strategy':initial.get('failure_code')=='CACHED_SECTION_STRATEGY',
         'same_L2_reference':True,
         'same_allowed_mask':True,'global_fallback':False,'sections':[]}
    try:anchors=segment_anchors(controller.reference_plan.world_xy,query.start,query.goal)
    except ValueError as error:
        log['failure']=str(error);initial['r1_segmented_recovery']=log;return initial
    chunks=[];results=[];template=None;previous=None
    for i,(a,b) in enumerate(zip(anchors,anchors[1:])):
        remaining=deadline-time.monotonic()
        if remaining<=.1:log['failure']='SEGMENT_RECOVERY_BUDGET';break
        if previous is not None:a=tuple(map(float,previous))
        section=replace(query,query_id=f'{query.query_id}_section{i}',start=a,goal=b)
        # Native parameters are unchanged. Bound the action client by the
        # shared recovery deadline; do not reset time for another section.
        saved_timeout=session.client.timeout
        session.client.timeout=min(saved_timeout,max(.01,remaining-.1))
        previous_deadline=getattr(session,'r1_recovery_deadline',None)
        session.r1_recovery_deadline=deadline
        try:out=SoftEdgeController.plan_l3(controller,section,session,auditor,spec,enabled=True,cap=cap)
        finally:
            session.client.timeout=saved_timeout
            session.r1_recovery_deadline=previous_deadline
        results.append(out)
        log['sections'].append({'index':i,'start':a,'goal':b,'success':bool(out.get('success')),
            'failure':out.get('failure_code'),'diagnostics':out.get('diagnostics',{}),
            'curvature_repair':out.get('curvature_repair',{})})
        result=out.get('result')
        if not out.get('success') or result is None:
            log['failure']='SECTION_REJECTED:'+str(out.get('failure_code'));break
        p=np.array([[v['x'],v['y'],v['yaw']] for v in result.points])
        if previous is not None:
            position=float(np.linalg.norm(previous[:2]-p[0,:2]))
            yaw=abs(math.atan2(math.sin(previous[2]-p[0,2]),math.cos(previous[2]-p[0,2])))
            log['sections'][-1].update(seam_position_error_m=position,seam_yaw_error_rad=yaw)
            if position>1e-5 or yaw>1e-5:
                log['failure']='SECTION_SEAM_MISMATCH';break
        chunks.append(p if previous is None else p[1:]);previous=p[-1];template=result
    if len(chunks)==len(anchors)-1:
        p=np.vstack(chunks)
        points=truthful_points(p,template,0,len(p)-1)
        ca,ind,traffic,jump,failures=controller._audit(query,p,controller.pending_l3_mask,auditor,points)
        log['before_stitch_repair_failures']=failures
        if repairable_steering_failure(failures) and time.monotonic()<deadline:
            repaired,detail=repair_path_extended(controller.world,controller.traffic,query,p,
                controller.pending_l3_mask,budget_s=max(0.,deadline-time.monotonic()))
            log['stitch_repair']=detail
            if repaired is not None:
                p=repaired;points=truthful_points(p,template,0,len(p)-1)
                ca,ind,traffic,jump,failures=controller._audit(query,p,controller.pending_l3_mask,auditor,points)
        if time.monotonic()>deadline:failures=sorted(set(failures+['SEGMENT_RECOVERY_BUDGET']))
        log.update(final_failures=failures,final_steering_jump_deg=jump,success=not failures)
        if not failures:
            initial.update(called=True,canonical_path_audit_reused=False,result=replace(template,points=points,path_audit=ca,source='v8_r1_bounded_sections'),
                success=True,failure_code='',geometry_success=True,independent_audit=ind,
                traffic_audit=traffic,constraint_audit={'max_raw_steering_jump_deg':jump,
                'steering_jump_valid':True,'traffic':traffic},metrics=ca.metrics)
            controller.trajectory=p;controller.trajectory_version=0 if controller.dynamic is None else controller.dynamic.version
            cache,key=strategy_cache(controller,query);cache[key]=True
    # Include every actual call, including the rejected whole-route call, in
    # timing and native-memory summaries. Keep section ACK evidence separately.
    ds=([initial['diagnostics']] if initial.get('diagnostics') else [])+[r.get('diagnostics',{}) for r in results]
    if not ds:
        initial['r1_segmented_recovery']=log;return initial
    diagnostics=dict(ds[-1])
    for key in ('local_map_update_ms','l3_action_wall_ms','l3_planning_time_ms','backend_call_count',
                'wall_time_ms','planning_time_ms','planner_cpu_total_ms'):
        diagnostics[key]=sum(float(d.get(key) or 0.) for d in ds)
    for key in ('planner_rss_peak_bytes','planner_pss_peak_bytes','stack_rss_peak_bytes','stack_pss_peak_bytes'):
        values=[d[key] for d in ds if d.get(key) is not None]
        if values:diagnostics[key]=max(values)
    diagnostics['costmap_update_acknowledged']=all(d.get('costmap_update_acknowledged') is True for d in ds)
    diagnostics['costmap_ack_mismatch_cells']=max(d.get('costmap_ack_mismatch_cells',0) for d in ds)
    diagnostics['r1_whole_route_termination_reason']=ds[0].get('termination_reason')
    if log['success']:diagnostics.update(ca.diagnostics())
    initial['diagnostics']=diagnostics
    repair_ms=sum(r.get('curvature_repair',{}).get('wall_ms',0.) for r in results)
    repair_ms+=log.get('stitch_repair',{}).get('wall_ms',0.)
    initial['curvature_repair']={'attempted':repair_ms>0.,'success':bool(log['success']),
        'wall_ms':initial.get('curvature_repair',{}).get('wall_ms',0.)+repair_ms,
        'r1_segment_and_stitch_details_in':'r1_segmented_recovery'}
    log['wall_ms']=(time.monotonic()-started)*1000
    initial['r1_segmented_recovery']=log
    if not log['success']:
        controller.trajectory=None;controller.trajectory_version=None
    return initial
