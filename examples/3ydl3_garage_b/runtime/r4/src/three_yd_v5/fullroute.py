"""V8-r7 full-route reuse. No prefix/window planning and no stale ACK reuse.

Only successful trajectories are cached. The unchanged independent audits run
on every delivery. Static content, query, reference, constraints and vehicle
are part of the binding; the cache retains no controller or map objects.
"""
from collections import OrderedDict
from copy import deepcopy
from dataclasses import replace
import os
import time
import numpy as np
from arena_evaluation.semantic_map import canonical_hash
from .curve_semantic_binding import fresh_hash as fresh_semantic_hash
from .v9_maps import content_digest as digest
from .r6_cache import resident_bytes


class RouteCache:
    def __init__(self, max_entries=8, max_bytes=8*2**20):
        self.max_entries=max_entries; self.max_bytes=max_bytes
        self.items=OrderedDict(); self.bytes=0; self.hits=0; self.evictions=0

    def get(self, key):
        item=self.items.get(key)
        if item is None:return None
        self.items.move_to_end(key); self.hits+=1
        from .v9_records import decode
        return decode(item[0])

    def put(self, key, value):
        from .v9_review import enabled as review_enabled
        if review_enabled('K'):
            from .v9_records import encode
            value=encode(value)
        else:
            from .v9_r9_records import copy_for_cache
            value=copy_for_cache(value)
        from .curve_records import record_bytes
        size=record_bytes(value)
        if size>self.max_bytes:return False
        self.remove(key)
        while self.items and (len(self.items)>=self.max_entries or self.bytes+size>self.max_bytes):
            _,(_,n)=self.items.popitem(last=False);self.bytes-=n;self.evictions+=1
        self.items[key]=(value,size);self.bytes+=size
        return True

    def remove(self,key):
        old=self.items.pop(key,None)
        if old is not None:self.bytes-=old[1]

    def diagnostics(self):
        return {'entries':len(self.items),'bytes':self.bytes,'max_entries':self.max_entries,
                'max_bytes':self.max_bytes,'hits':self.hits,'evictions':self.evictions}


def cache_for(views):
    return views.setdefault(('v8-r7','full-route'),RouteCache())


def binding(c,query,session,spec,enabled,cap):
    w=c.world
    # Hash real arrays, not just file names/version numbers. This also rejects
    # an in-place map edit accidentally made without bumping its version.
    from .curve_backend import identity
    return canonical_hash(['V8-r7-fullroute-1',identity(),w.key,w.map.sha256,
        w.map.resolution,w.map.origin,w.hard.shape,digest(w.hard),digest(w.map.occupancy),
        fresh_semantic_hash(w.semantic),digest(w.labels),w.rules,w.vehicle,
        tuple(session.bound_vehicle),query.start,query.goal,c.topology_route.signature,
        digest(c.reference_plan.world_xy),digest(c.plan.corridor_mask),c.weight,
        digest(np.asarray(getattr(c,'_curve_progress_guide',[]),float)),
        getattr(c,'corner_records',[]),
        digest(np.asarray(getattr(c,'rectangle_bridge_points',[]),float)),
        digest(np.asarray(getattr(c,'obstacle_repair_points',[]),float)),
        c.max_search_cells,c.max_map_fraction,
        enabled,cap,str(spec),sorted((k,v) for k,v in os.environ.items() if k.startswith('THREEYD_'))])


def record_from(out,internal_points=None):
    result=out.get('result')
    if not out.get('success') or result is None or not result.points:return None
    # Do not retain diagnostics/auditor objects and their source-call clocks.
    result=replace(result,diagnostics={},path_audit=None,expanded_states=0,
                   generated_states=0,first_solution_time_ms=None)
    from .curve_records import repack_or_reuse
    result=replace(result,points=repack_or_reuse(result.points,internal_points))
    from .curve_backend import identity
    from .curve_records import freeze_model
    # The result crosses both the route cache and task-history boundary.
    # Freeze its owned geometry once; both boundaries can safely share it.
    model=freeze_model(out.get('curve_model'))
    if model is not None:out['curve_model']=model
    return {'result':result,'bounded_l3':out.get('bounded_l3',{}),'v5':out.get('v5',{}),
        'l3_backend_identity':out.get('l3_backend_identity',list(identity())),
        'curve_model':model,'curve_l3':out.get('curve_l3',{})}


def validated_reuse(c,query,session,auditor,record):
    """Return a current whole-path result, or rejection requiring normal search."""
    version=0 if c.dynamic is None else c.dynamic.version
    if tuple(c.world.vehicle)!=tuple(session.bound_vehicle):
        raise ValueError('VEHICLE_VERSION_MISMATCH')
    if c.reference_plan.world_key!=c.world.key or c.reference_plan.fine_corridor_hash!=digest(c.plan.corridor_mask):
        raise ValueError('REFERENCE_BINDING_MISMATCH')
    mask=c._target_mask()
    limit=min(c.max_search_cells,int(mask.size*c.max_map_fraction))
    count=int(mask.sum())
    if count>limit:return {'success':False,'failure_code':'L3_SPATIAL_BUDGET'}
    tick=time.monotonic()
    ack=c.sync_only(session)
    sync_ms=(time.monotonic()-tick)*1000
    diagnostics={**ack,'fullroute_sync_wall_ms':sync_ms,'backend_called':False,
        'backend_call_count':0,'l3_action_wall_ms':0.,'l3_planning_time_ms':0.,
        'planning_time_ms':0.,'fullroute_cache_hit':True,
        'memory_sampling':'no vehicle search; process-tree query peak measured by caller'}
    if ack.get('costmap_update_acknowledged') is not True:
        c.trajectory=None;c.trajectory_version=None
        return {'success':False,'failure_code':'FULLROUTE_MAP_ACK_FAILED','diagnostics':diagnostics}
    result=record['result'];points=result.points
    p=np.asarray([[v['x'],v['y'],v['yaw']] for v in points],float)
    ca,ind,traffic,jump,failures=c._audit(query,p,mask,auditor,points)
    from .curve_backend import validate_record
    extra,certificate=validate_record(c,query,record,session)
    failures.extend(extra);diagnostics['curve_revalidation']=certificate
    if c.dynamic is not None and c.dynamic.version!=version:
        failures.append('MAP_VERSION_CHANGED_DURING_REUSE')
    if failures:
        c.trajectory=None;c.trajectory_version=None
    else:
        c.trajectory=p.copy();c.trajectory_version=version
        c.pending_l3_mask=mask;c.server_l3_mask=mask
    result=replace(result,path_audit=ca,source='v8_r7_verified_whole_route_reuse',
                   diagnostics=diagnostics)
    return {'success':not failures,'failure_code':';'.join(failures),'result':result,
        'called':False,'geometry_success':bool(ca.final_valid_success and ind['valid']),
        'independent_audit':ind,'traffic_audit':traffic,'metrics':ca.metrics,
        'constraint_audit':{'max_raw_steering_jump_deg':jump,'steering_jump_valid':jump<=15+1e-6,'traffic':traffic},
        'legacy_contract_success':not failures,'map_version':version,
        'diagnostics':diagnostics,'bounded_l3':{**record.get('bounded_l3',{}),
            'search_cells':count,'limit_cells':limit,'map_fraction':count/mask.size,
            'full_map_cells':mask.size,'global_fallback':False},
        'curve_model':record.get('curve_model'),'curve_l3':record.get('curve_l3',{}),
        'l3_backend_identity':record.get('l3_backend_identity'),
        'v5':record.get('v5',{}),'reference_diagnostics':c._bounded_grid.diagnostics,
        'fullroute':{'reused':not failures,'whole_path':True,'ack_checked_this_call':True,
            'ack_evidence_reused':bool(ack.get('semantic_ack_reused')),
            'full_original_audits':True,'search_calls':0}}


def plan(c,query,session,auditor,spec,*,enabled=True,cap=140):
    if hasattr(session,'bind_complete_task'):session.bind_complete_task(query,c)
    # sync_only uses the registered 140 cost cap. Other explicit caps retain
    # the parent path so reuse cannot silently change the requested costmap.
    if os.environ.get('THREEYD_V8_FULLROUTE_REUSE','0')!='1' or cap!=140:
        return c._plan_l3_uncached(query,session,auditor,spec,enabled=enabled,cap=cap)
    if not enabled:raise ValueError('L2_SOFT_COST_REQUIRED')
    tick=time.monotonic();cache=cache_for(c.views)
    key=binding(c,query,session,spec,enabled,cap)
    record=cache.get(key)
    c.timing['fullroute_lookup_ms']=(time.monotonic()-tick)*1000
    c.timing['fullroute_cache_hit']=record is not None
    if record is not None:
        from .controller import variable_neighborhood
        forced=np.concatenate((np.asarray(getattr(c,'rectangle_bridge_points',[]),float).reshape(-1,2),
                               np.asarray(getattr(c,'obstacle_repair_points',[]),float).reshape(-1,2)))
        c._search_mask,c.variable_scope_info=variable_neighborhood(c.world,c.reference_plan.world_xy,
            c.corner_records,forced_turning_points=forced)
        from .v9_r8_fields import freeze_owned_scope
        from .v9_maps import immutable
        c._search_mask=freeze_owned_scope(c._search_mask)
        c.timing['r8_owned_search_mask_immutable']=immutable(c._search_mask)
        out=validated_reuse(c,query,session,auditor,record)
        if out['success'] or any(s in out['failure_code'] for s in ('ACK','VERSION','BINDING')):
            out['fullroute_cache']=cache.diagnostics();return out
        cache.remove(key)
        c.timing['fullroute_rejected']=out['failure_code']
    out=c._plan_l3_uncached(query,session,auditor,spec,enabled=enabled,cap=cap)
    from .v9_options import enabled as v9_enabled
    if v9_enabled('B') and out.get('success'):
        from .v9_segments import store
        out['v9_segment_cache']=store(c,out)
    record=record_from(out)
    if record is None:c.trajectory=None;c.trajectory_version=None
    stored=record is not None and cache.put(key,record)
    out['fullroute']={'reused':False,'whole_path':True,'stored':stored}
    out['fullroute_cache']=cache.diagnostics()
    return out
