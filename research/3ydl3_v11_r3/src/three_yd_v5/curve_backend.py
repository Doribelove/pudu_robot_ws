"""Backend identity, bounded dispatch and strict spline result integration."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from contextlib import contextmanager
from contextvars import ContextVar
from copy import deepcopy
import hashlib,json,os,time
import numpy as np
from arena_evaluation.semantic_map import canonical_hash
from .v9_maps import content_digest

CONFIG_PATH=Path(__file__).resolve().parents[2]/'config/l3.json'
_REQUEST_SETTINGS=ContextVar('v11_request_settings',default=None)

def settings():
    snapshot=_REQUEST_SETTINGS.get()
    if snapshot is not None:
        return snapshot[0],deepcopy(snapshot[1]),snapshot[2]
    return _read_settings()

def _read_settings():
    mode=os.environ.get('THREEYD_L3_BACKEND','control')
    if mode not in ('control','independent','compatibility'):raise ValueError('UNKNOWN_L3_BACKEND')
    raw=CONFIG_PATH.read_bytes();cfg=json.loads(raw)
    from .tail_options import enabled,mode as tail_mode
    if enabled('E'):cfg['r4_lateral_endpoint_seed']=True
    if enabled('G'):
        cfg.update(r4_rounded_guide=True,adaptive_constraint_stations=True)
    if enabled('S'):cfg['tracking_weight']*=100.
    if enabled('F'):cfg['r4_failure_recovery']=True
    if enabled('W'):cfg['r4_quadratic_warm_start']=True
    if enabled('T'):cfg['r4_tighter_primary']=True
    if enabled('V'):cfg['r4_adaptive_stations']=True
    cfg['r4_mode']=tail_mode()
    from .recovery_options import mode as recovery_mode,enabled as recovery_enabled
    cfg['r5_mode']=recovery_mode()
    from .localfix_options import mode as localfix_mode
    cfg['r6_mode']=localfix_mode()
    cfg['r7_mode']=os.environ.get('THREEYD_V10_R7_MODE','control')
    cfg['r8_mode']=os.environ.get('THREEYD_V10_R8_MODE','control')
    from .endpoint_tolerance import policy
    cfg['endpoint_tolerance']=policy()
    if recovery_enabled('S'):cfg['r5_early_seed']=True
    if recovery_enabled('K'):cfg['r5_interior_seed_only']=True
    if recovery_enabled('F'):cfg['free_endpoint_tangent_scale']=True
    raw=json.dumps(cfg,sort_keys=True).encode()
    return mode,cfg,hashlib.sha256(raw).hexdigest()

@contextmanager
def settings_scope():
    """Read/hash once per request; reject changed configuration at delivery.

    No query result, map or audit is cached here. Callers receive owned config
    dictionaries, so solver-local overrides cannot mutate the snapshot.
    """
    if _REQUEST_SETTINGS.get() is not None:
        yield
        return
    snapshot=_read_settings()
    environment={k:v for k,v in os.environ.items() if k.startswith('THREEYD_')}
    token=_REQUEST_SETTINGS.set(snapshot)
    try:
        yield
        current=_read_settings()
        if (current[0]!=snapshot[0] or current[2]!=snapshot[2] or
                environment!={k:v for k,v in os.environ.items() if k.startswith('THREEYD_')}):
            raise ValueError('CURVE_CONFIG_VERSION_CHANGED_DURING_REQUEST')
    finally:
        _REQUEST_SETTINGS.reset(token)

def identity():
    snapshot=_REQUEST_SETTINGS.get()
    mode,cfg,sha=_read_settings() if snapshot is None else snapshot
    return ('3YDL3-V11',mode,sha)

def service_binding(service,semantic_hash=None):
    if identity()[1]=='control':return canonical_hash(identity())
    w=service.world;c=service.controller
    if semantic_hash is None:
        from .curve_semantic_binding import fresh_hash
        semantic_hash=fresh_hash(w.semantic)
    return canonical_hash([identity(),w.key,content_digest(w.hard),content_digest(w.map.occupancy),
        content_digest(w.labels),semantic_hash,w.rules,w.vehicle,tuple(service.session.bound_vehicle),
        None if c is None else [c.topology_route.signature,content_digest(c.reference_plan.world_xy),
        content_digest(c.plan.corridor_mask),c.weight,c.max_search_cells,c.max_map_fraction,
        content_digest(np.asarray(getattr(c,'_curve_progress_guide',[]),float))]])

def retained_binding(service,semantic_hash=None):
    """Identity of an untrusted recovery candidate, excluding live obstacles.

    A candidate may survive an obstacle change, but is never executable until
    the new ACK, whole-path audits and exact curve certificate all pass.
    Backend/config/static-map/vehicle/route changes still invalidate it.
    """
    if identity()[1]=='control':return canonical_hash(identity())
    w=service.world;c=service.controller
    if semantic_hash is None:
        from .curve_semantic_binding import fresh_hash
        semantic_hash=fresh_hash(w.semantic)
    return canonical_hash([identity(),w.key,content_digest(service.dynamic.static_hard),
        content_digest(w.map.occupancy),content_digest(w.labels),semantic_hash,w.rules,
        w.vehicle,tuple(service.session.bound_vehicle),None if c is None else
        [c.topology_route.signature,content_digest(c.reference_plan.world_xy),content_digest(c.plan.corridor_mask),
         c.weight,c.max_search_cells,c.max_map_fraction]])

def request_start(service):
    if hasattr(service.session,'_r4_seed_phase'):del service.session._r4_seed_phase
    if hasattr(service.session,'_r5_speculative_seed'):del service.session._r5_speculative_seed
    if hasattr(service.session,'_r8_scope_recovery'):del service.session._r8_scope_recovery
    mode,cfg,sha=settings()
    semantic_hash=None
    if mode!='control':
        from .curve_semantic_binding import phase_hash
        semantic_hash=phase_hash(service.world.semantic)
    current=service_binding(service,semantic_hash)
    previous=getattr(service,'_curve_binding',None)
    if previous is not None and previous!=current:
        service.record=None;service.remaining=None
        if service.controller is not None:service.controller.trajectory=None;service.controller.trajectory_version=None
    if getattr(service,'_curve_retained_binding',None) not in (None,retained_binding(service,semantic_hash)):
        service.retained_record=None;service.retained_path=None
    service.session._curve_request={'started':time.monotonic(),'deadline':time.monotonic()+cfg['total_request_budget_s'],
        'new_used_s':0.,'mode':mode,'config':cfg,'config_sha256':sha,'attempts':[]}
    if mode!='control':
        from .curve_interpolation import stats
        service.session._curve_request['interpolation_before']=stats()

def request_finish(service,out):
    state=service.session._curve_request
    if out.get('success'):
        semantic_hash=None
        if state['mode']!='control':
            from .curve_semantic_binding import phase_hash
            semantic_hash=phase_hash(service.world.semantic)
        service._curve_binding=service_binding(service,semantic_hash)
        service._curve_retained_binding=retained_binding(service,semantic_hash)
    log=out.get('curve_l3') or {}
    if not log and service.record:log=service.record.get('curve_l3',{})
    out['l3_backend']={'identity':list(identity()),'mode':state['mode'],
        'source':log.get('result_source','old_l3' if state['mode']=='control' else 'no_result'),
        'new_used_s':state['new_used_s'],'attempts':state['attempts'],
        'strict_success':bool(out.get('success')),'reused':bool(out.get('task_progress',{}).get('reused'))}
    if 'endpoint_tolerance' in out:
        out['l3_backend']['strict_success_definition']='unchanged audits at explicitly resolved endpoints'
        out['l3_backend']['exact_endpoint_success']=out.get('exact_endpoint_success',False)
        out['l3_backend']['tolerant_endpoint_success']=out.get('tolerant_endpoint_success',False)
    if state['mode']!='control':
        from .curve_interpolation import stats
        current=stats();before=state.get('interpolation_before',{})
        out['l3_backend']['pose_interpolation']={**current,
            'hits_this_request':current['hits']-before.get('hits',0),
            'misses_this_request':current['misses']-before.get('misses',0)}
        from .curve_semantic_binding import stats as semantic_stats
        out['l3_backend']['semantic_binding']=semantic_stats()
    return out

def plan(c,query,session,auditor,spec,*,enabled=True,cap=140):
    from .curve_l3 import optimize
    from .geometry import variable_neighborhood
    from .v9_r8_fields import freeze_owned_scope
    from arena_evaluation.unified_four_backends_smoke import PlanResult
    from arena_3d_v1.curvature_repair import truthful_points
    mode,cfg,sha=settings();state=getattr(session,'_curve_request',None)
    recovery_phase=bool(getattr(session,'_r4_seed_phase',False))
    scope_recovery=bool(getattr(session,'_r8_scope_recovery',False))
    if hasattr(session,'_r11_primary_slice_s') and not recovery_phase:
        cfg=dict(cfg,r11_primary_slice_s=session._r11_primary_slice_s)
    if hasattr(session,'_r8_scope_recovery'):del session._r8_scope_recovery
    if ('A' in cfg.get('r7_mode','control') and getattr(c,'generic_mode',False)
        and any(x.get('r6_local_detour_selected') for x in c.timing.get('activation_attempts',[]))):
        cfg=dict(cfg,r7_detour_seed=True)
    if cfg.get('r4_failure_recovery') and hasattr(session,'_r4_seed_phase'):
        cfg=dict(cfg,r4_defer_recovery=not session._r4_seed_phase,
                 r4_recovery_only=session._r4_seed_phase)
        del session._r4_seed_phase
    if hasattr(session,'_r5_speculative_seed'):
        cfg=dict(cfg,r5_speculative_seed=session._r5_speculative_seed)
        del session._r5_speculative_seed
    if state is None:
        state={'deadline':time.monotonic()+cfg['total_request_budget_s'],'new_used_s':0.,'attempts':[]}
        session._curve_request=state
    tick=time.monotonic();version=0 if c.dynamic is None else c.dynamic.version
    log={'mode':mode,'config_sha256':sha,'result_source':'no_result','old_l3_calls':0,'success':False}
    ack={};curve=None;out={}
    def reject(reason):
        c.trajectory=None;c.trajectory_version=None;log['failure']=reason
        return {'success':False,'failure_code':reason,'curve_l3':log,
                'diagnostics':{**ack,'curve_l3':log,'backend_call_count':0},'bounded_l3':getattr(c,'variable_scope_info',{})}
    if not enabled:raise ValueError('L2_SOFT_COST_REQUIRED')
    if tuple(c.world.vehicle)!=tuple(session.bound_vehicle):raise ValueError('VEHICLE_VERSION_MISMATCH')
    if (c.reference_plan.world_key!=c.world.key or
        c.reference_plan.fine_corridor_hash!=content_digest(c.plan.corridor_mask)):raise ValueError('REFERENCE_BINDING_MISMATCH')
    forced=np.concatenate((np.asarray(getattr(c,'rectangle_bridge_points',[]),float).reshape(-1,2),
                           np.asarray(getattr(c,'obstacle_repair_points',[]),float).reshape(-1,2)))
    mask,info=variable_neighborhood(c.world,c.reference_plan.world_xy,c.corner_records,forced_turning_points=forced)
    prepared_guide=None;terminal_detail=None;original_cfg=cfg
    pose_seed=None;pose_detail=None
    if (scope_recovery and 'P' in cfg['r8_mode'] and not c.generic_mode
        and getattr(c,'r4_semantic_body',False) and not hasattr(c,'_curve_progress_model')):
        from .failure_pose_chain import pose_chain
        from .geometry import corners
        pose_started=time.monotonic()
        remaining=min(cfg['new_l3_budget_s']-state['new_used_s'],state['deadline']-pose_started)
        local_deadline=pose_started+min(6.3,max(0.,remaining-2.5))
        legal=c.plan.corridor_mask if c.dynamic is None else c.plan.corridor_mask&~c.dynamic.mask
        try:
            pose_seed,pose_detail=pose_chain(c.world,legal,c.topology.subregions,c.traffic.roads,
                c.reference_plan.world_xy,query.start,query.goal,local_deadline)
        except ValueError as error:
            pose_seed=None;pose_detail={'selected':False,'failure':str(error),'initialization_only':True}
        if 'E' in cfg['r8_mode']:del legal
        if pose_seed is not None:
            prepared_guide=pose_seed[:,:2]
            local_mask,local_info=variable_neighborhood(c.world,prepared_guide,corners(prepared_guide),
                forced_turning_points=np.asarray([query.start[:2],query.goal[:2]]))
            mask|=local_mask
            if 'E' in cfg['r8_mode']:del local_mask
            info['r8_pose_scope']={**local_info,'original_scope_retained':True}
        state['new_used_s']+=time.monotonic()-pose_started
    if recovery_phase and 'T' in cfg['r8_mode'] and c.generic_mode and not hasattr(c,'_curve_progress_model'):
        from .failure_terminal import terminal_guide
        from .geometry import corners
        terminal_started=time.monotonic()
        remaining=cfg['new_l3_budget_s']-state['new_used_s']
        local_deadline=min(state['deadline'],terminal_started+min(1.2,max(0.,remaining)))
        legal=c.plan.corridor_mask if c.dynamic is None else c.plan.corridor_mask&~c.dynamic.mask
        prepared_guide,terminal_detail=terminal_guide(c.world,legal,c.reference_plan.world_xy,query.start,query.goal,local_deadline)
        if 'E' in cfg['r8_mode']:del legal
        if prepared_guide is not None:
            local_mask,local_info=variable_neighborhood(c.world,prepared_guide,corners(prepared_guide),
                forced_turning_points=np.asarray([query.start[:2],query.goal[:2]]))
            mask|=local_mask
            if 'E' in cfg['r8_mode']:del local_mask
            info['r8_terminal_scope']={**local_info,'local_seed':terminal_detail,'original_scope_retained':True}
            cfg=dict(cfg,r4_rounded_guide=True,tracking_weight=cfg['tracking_weight']*100.,
                     r4_guide_epsilon_cap=.06,r4_endpoint_threshold_deg=25,
                     r4_recovery_only=False,r4_defer_recovery=False)
        state['new_used_s']+=time.monotonic()-terminal_started
    c._search_mask=freeze_owned_scope(mask)
    # The immutable scope owns its bytes. Its consumed construction buffer is
    # no longer needed by map synchronization, curve solving, or strict audit.
    if 'E' in cfg['r8_mode']:del mask
    target=c._target_mask();c.variable_scope_info=info
    count=int(target.sum());limit=min(c.max_search_cells,int(target.size*c.max_map_fraction))
    info.update(search_cells=count,limit_cells=limit,full_map_cells=target.size,map_fraction=count/target.size,global_fallback=False)
    if count>limit:return reject('L3_SPATIAL_BUDGET')
    if hasattr(session,'bind_complete_route_scope'):session.bind_complete_route_scope(query,target)
    sync=time.monotonic();ack=c.sync_only(session);log['sync_ms']=(time.monotonic()-sync)*1000
    if ack.get('costmap_update_acknowledged') is not True:return reject('CURVE_MAP_ACK_FAILED')
    c.pending_l3_mask=target;c.server_l3_mask=target;mask_hash=content_digest(target)
    available=min(cfg['new_l3_budget_s']-state['new_used_s'],state['deadline']-time.monotonic())
    curve_started=time.monotonic()
    if available<=0:out=reject('CURVE_BUDGET')
    else:
        guide=prepared_guide if prepared_guide is not None else getattr(c,'_curve_progress_guide',c.reference_plan.world_xy)
        deadline=time.monotonic()+available
        model=getattr(c,'_curve_progress_model',None)
        continuation_detail=None
        if model is not None:
            from .curve_continuation import continue_curve,repair_obstacles
            if getattr(c,'_curve_obstacle_repair',False):
                p,curve,detail=repair_obstacles(c.world,target,model,c.dynamic.mask,deadline,cfg)
            else:p,curve,detail=continue_curve(c.world,target,model,query.start,query.goal,deadline,cfg)
            continuation_detail=detail
        else:p=None
        if p is None:
            if pose_seed is not None:
                from .failure_pose_curve import optimize_pose
                p,curve,detail=optimize_pose(c.world,target,pose_seed,query.start,query.goal,deadline,cfg,c.reference_plan.world_xy)
            else:p,curve,detail=optimize(c.world,target,guide,query.start,query.goal,deadline,cfg)
            if p is None and prepared_guide is not None and time.monotonic()<deadline:
                failed_seed=detail
                p,curve,detail=optimize(c.world,target,c.reference_plan.world_xy,query.start,query.goal,deadline,original_cfg)
                detail['r8_terminal_solver_failure']=failed_seed
            if terminal_detail is not None:detail['r8_terminal_initialization']=terminal_detail
            if pose_detail is not None:detail['r8_pose_initialization']=pose_detail
            if continuation_detail is not None:detail['continuation_attempt']=continuation_detail
        log.update(detail);log['mode']=mode;log['config_sha256']=sha
        if p is None:out=reject(detail.get('failure','CURVE_NO_SOLUTION'))
        else:
            template=SimpleNamespace(source='3ydl3_v11_bspline',planner_backend='bspline_'+cfg.get('qp_solver','osqp'),backend_version='3YDL3-V11')
            points=truthful_points(p,template,0,len(p)-1)
            for point in points:
                point.update(source=template.source,planner_backend=template.planner_backend,backend_version=template.backend_version)
            audit_tick=time.monotonic()
            ca,ind,traffic,jump,failures=c._audit(query,p,target,auditor,points)
            log['strict_audit_ms']=(time.monotonic()-audit_tick)*1000
            # Retry only a geometrically certified curve rejected by unchanged
            # traffic audits. Keep the same scope, pose, deadline and ACK.
            if failures and (detail.get('accepted_quadratic_initialization') or detail.get('r4_solver_stations') or detail.get('r5_early_seed') or detail.get('r7_detour_seed')) and time.monotonic()<deadline:
                initial_rejection={'strict_failures':list(failures),'diagnostics':dict(log)}
                fallback_cfg=dict(cfg,certified_quadratic_start=False,r4_adaptive_stations=False,r4_tighter_primary=False,r5_early_seed=False,r7_detour_seed=False)
                p0,curve0,detail0=optimize(c.world,target,guide,query.start,query.goal,deadline,fallback_cfg)
                if p0 is not None:
                    points0=truthful_points(p0,template,0,len(p0)-1)
                    for point in points0:
                        point.update(source=template.source,planner_backend=template.planner_backend,backend_version=template.backend_version)
                    ca0,ind0,traffic0,jump0,failures0=c._audit(query,p0,target,auditor,points0)
                    p,curve,points,ca,ind,traffic,jump,failures=p0,curve0,points0,ca0,ind0,traffic0,jump0,failures0
                    log.update(detail0)
                else:initial_rejection['original_sqp_failure']=detail0.get('failure')
                log['quadratic_strict_fallback']=initial_rejection
                cfg=fallback_cfg
            traffic_codes=('ROAD_SEQUENCE','ROAD_OWNERSHIP','WRONG_TRAVEL_DIRECTION')
            if (failures and all(any(code in failure for code in traffic_codes) for failure in failures)
                and time.monotonic()<deadline):
                first={'failure':list(failures),'diagnostics':dict(log)}
                retry_cfg=dict(cfg,tracking_weight=cfg['tracking_weight']*2.,certified_quadratic_start=False)
                p2,curve2,detail2=optimize(c.world,target,guide,query.start,query.goal,deadline,retry_cfg)
                retry_log={'trigger':first['failure'],'tracking_weight':retry_cfg['tracking_weight'],
                           'same_scope':True,'same_deadline':True,'diagnostics':detail2}
                if p2 is not None:
                    points2=truthful_points(p2,template,0,len(p2)-1)
                    for point in points2:
                        point.update(source=template.source,planner_backend=template.planner_backend,backend_version=template.backend_version)
                    ca2,ind2,traffic2,jump2,failures2=c._audit(query,p2,target,auditor,points2)
                    retry_log['strict_failures']=list(failures2)
                    if not failures2:
                        p,curve,points,ca,ind,traffic,jump,failures=p2,curve2,points2,ca2,ind2,traffic2,jump2,failures2
                        log.update(detail2)
                log['traffic_retry']=retry_log;log['first_attempt']=first
            if (c.dynamic is not None and c.dynamic.version!=version) or content_digest(c._target_mask())!=mask_hash:
                failures.append('CURVE_MAP_VERSION_OR_BINDING_CHANGED')
            if time.monotonic()>state['deadline']:failures.append('CURVE_REQUEST_BUDGET')
            if failures:out=reject('CURVE_STRICT_REJECTED:'+';'.join(failures))
            else:
                log.update(success=True,result_source='new_l3_independent',old_l3_calls=0)
                diagnostics={**ack,**ca.diagnostics(),'backend_call_count':0,'curve_l3':log}
                result=PlanResult(planner_success=True,points=points,failure_code='',planner_backend='bspline_'+cfg.get('qp_solver','osqp'),
                    backend_version='3YDL3-V11',source='3ydl3_v11_certified_bspline',diagnostics=diagnostics,path_audit=ca)
                c.trajectory=p;c.trajectory_version=version
                out={'success':True,'geometry_success':True,'failure_code':'','result':result,'called':True,
                    'metrics':ca.metrics,'independent_audit':ind,'traffic_audit':traffic,
                    'constraint_audit':{'max_raw_steering_jump_deg':jump,'steering_jump_valid':True,'traffic':traffic},
                    'diagnostics':diagnostics,'map_version':version,'bounded_l3':info,'curve_l3':log,'curve_model':curve,
                    'reference_diagnostics':c._bounded_grid.diagnostics,'legacy_contract_success':True}
    state['new_used_s']+=time.monotonic()-curve_started
    # Sidecars preserve rejected curves for failure analysis; never deliver them.
    if curve is not None:
        folder=c.root/'curve_diagnostics';folder.mkdir(exist_ok=True)
        tag=f'{query.query_id}_{len(state["attempts"])}_{time.monotonic_ns()}'
        (folder/(tag+'.json')).write_text(json.dumps({'curve':curve,'diagnostics':log}))
    state['attempts'].append(dict(log))
    if (mode=='compatibility' and not out.get('success') and time.monotonic()<state['deadline']
        and not any(v in out.get('failure_code','') for v in ('ACK','VERSION','BINDING'))):
        failed=out;fallback=time.monotonic()
        out=c._plan_l3_legacy(query,session,auditor,spec,enabled=enabled,cap=cap)
        final={**log,'fallback_ms':(time.monotonic()-fallback)*1000,'new_failure':failed['failure_code'],
            'result_source':'old_l3_fallback' if out.get('success') else 'no_result','old_l3_calls':1,
            'success':bool(out.get('success'))}
        out['curve_l3']=final;out.setdefault('diagnostics',{})['curve_l3']=final
        state['attempts'][-1]=dict(final)
    out['l3_backend_identity']=list(identity())
    c.timing['curve_l3_total_ms']=(time.monotonic()-tick)*1000
    return out

def validate_record(c,query,record,session):
    """Re-certify the actual curve at each delivery, including identical hits."""
    expected=list(identity())
    if record.get('l3_backend_identity',expected)!=expected:return ['CURVE_BACKEND_BINDING_CHANGED'],{}
    model=record.get('curve_model')
    if model is None:
        if identity()[1]=='independent':return ['CURVE_MODEL_MISSING'],{}
        return [],{}
    from .curve_l3 import certify,CurveFailure
    from scipy.interpolate import BSpline
    old=record['result'].points;p=np.array([[v['x'],v['y'],v['yaw']] for v in old])
    if query is not None:
        # The inherited endpoint residual tolerance is not permission to
        # restore an old start after a failed displaced request. A certified
        # independent curve must begin/end at the actual requested poses.
        for actual,expected,name in ((p[0],query.start,'START'),(p[-1],query.goal,'GOAL')):
            yaw_error=abs((actual[2]-expected[2]+np.pi)%(2*np.pi)-np.pi)
            if np.linalg.norm(actual[:2]-np.asarray(expected)[:2])>1e-8 or yaw_error>1e-8:
                return ['CURVE_ACTUAL_'+name+'_MISMATCH'],{}
    # A suffix preserves its exact parameter domain and is recertified. An
    # off-curve actual pose requires full remaining-curve reoptimization.
    spline=BSpline(model['knots'],model['coefficients'],3)
    state=getattr(session,'_curve_request',{});deadline=state.get('deadline',time.monotonic()+8.)
    try:
        checked,detail=certify(spline,c.world,c._target_mask(),deadline,corridor=model.get('corridor'),parameters=model.get('parameters'))
        detail.pop('sample_parameters',None)
        if checked.shape!=p.shape or not np.allclose(checked,p,atol=1e-9,rtol=0):return ['CURVE_OUTPUT_BINDING_CHANGED'],detail
        return [],detail
    except CurveFailure as e:return [str(e)],{}

def displaced_continuation(service,query):
    """Reoptimize the FULL remaining spline from the real slightly moved pose.

    Matching selects task history, never snaps the returned pose. No inherited
    connector or old L3 is called in independent mode. Ambiguous route visits
    cannot supply task ownership. Original L1/L2 arrays remain untouched.
    """
    from .task_progress import ProgressTraffic,wrap
    from .fullroute import record_from
    path=service.remaining;pose=np.asarray(query.start,float)
    distance=np.linalg.norm(path[:,:2]-pose[:2],axis=1)
    angle=abs(wrap(path[:,2]-pose[2]))
    hits=np.flatnonzero((distance<=.08)&(angle<=np.deg2rad(5)))
    if not len(hits) or (len(hits)>1 and np.any(np.diff(hits)>1)):return None
    index=int(hits[np.argmin(distance[hits])]);c=service.controller
    prefix=np.concatenate((service.prefix,path[:index]))
    c.traffic=ProgressTraffic(service.base_traffic,prefix)
    c._curve_progress_guide=path[index:,:2].copy()
    from .curve_records import FrozenCurve
    model=service.record.get('curve_model')
    c._curve_progress_model=None if model is None else FrozenCurve(model).suffix(index)
    c.r13_prefix_connector=None;c.r13_remainder_mask=None
    c.timing={'adapter_calls':[]};tick=time.monotonic()
    service.session.active_controller=c;service.session.active_route=service.route
    try:out=c.plan_l3(query,service.session,service.auditor,service.spec)
    finally:c._curve_progress_model=None
    elapsed=(time.monotonic()-tick)*1000
    out['task_progress']={'attempted':True,'reused':False,'retained_task_context':True,
        'action':'curve_complete_remaining_from_actual_pose','matched_history_vertex':index,
        'L1_search_called':False,'L2_search_called':False,'full_remaining_route':True,
        'position_offset_m':float(distance[index]),'yaw_offset_deg':float(np.rad2deg(angle[index])),
        'pose_snapped':False,'committed_prefix_vertices':len(prefix)}
    out['candidate_recovery']={'l1_actual_ms':0.,'all_candidates_controller_ms':0.,
        'all_candidates_l3_pipeline_ms':elapsed,'all_candidates_adapter_ms':c.timing.get('adapter_actual_ms',0.),
        'global_fallback':False,'attempts':[{'success':out.get('success',False),'l3_pipeline_actual_ms':elapsed,
        'l3_diagnostics':out.get('diagnostics',{})}]}
    service.query=query;service.record=record_from(out)
    if out.get('success'):
        service.progress_m+=float(np.linalg.norm(np.diff(path[:index+1,:2],axis=0),axis=1).sum())
        service.prefix=prefix;service.remaining=service.current_trajectory()
    else:
        service.remaining=None;c.trajectory=None;c.trajectory_version=None
    return out
