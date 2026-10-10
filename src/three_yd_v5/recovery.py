"""Bounded semantic-route recovery for V5.

At most three distinct legal region routes share the existing per-query
deadline.  Infrastructure/version failures stop immediately; a geometric or
traffic rejection may advance to the next route.  There is no global-map L3
fallback.
"""
import os
import time
from arena_evaluation.semantic_map import canonical_hash
from .v9_maps import content_digest as digest
from .r6_cache import get_cache


class V5Runner:
    def __init__(self,world,topology,cache_root,views,dynamic=None,controller_class=None,**kwargs):
        self.world=world;self.topology=topology;self.root=cache_root;self.views=views;self.dynamic=dynamic;self.controller_class=controller_class
        self.weight=kwargs.get('weight',2.)
        self.max_region_routes=int(kwargs.get('max_region_routes',3))

    def _dynamic_identity(self):
        if self.dynamic is None:return (0,'static')
        return (int(self.dynamic.version),digest(self.dynamic.mask))

    def _query_key(self,query):
        return ('V5-r9-preflight-v1',self.world.map.sha256,self.world.semantic.semantic_map_hash,
                canonical_hash(self.world.vehicle),self._dynamic_identity(),
                tuple(map(float,query.start)),tuple(map(float,query.goal)),float(self.weight))

    @staticmethod
    def _deterministic_failure(failure):
        text=str(failure).upper()
        if any(x in text for x in ('ACK','CONTENT','VERSION','BINDING','CORRUPT','TIMEOUT','BUDGET')):
            return False
        codes=('V5_L2_RECTANGLE_BRIDGE_NOT_FOUND','V5_L2_NO_PATH','REFERENCE_GRID_DISCONNECTED',
               'STATIC_FOOTPRINT_COLLISION','SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT',
               'RAW_STEERING_JUMP','ROAD_SEQUENCE_OR_OWNERSHIP','UNAUTHORIZED_PARKING_TRANSIT',
               'UNAUTHORIZED_U_TURN','WRONG_TRAVEL_DIRECTION')
        return any(code in text for code in codes)

    def plan(self,query,session,auditor,spec):
        tick=time.monotonic();c=None;route=None;out=None;attempts=[]
        query_key=self._query_key(query)
        failures=get_cache(self.views,'candidate-failures',max_entries=64)
        successes=get_cache(self.views,'candidate-success',max_entries=64)
        preferred_record=successes.get(query_key)
        preferred=(preferred_record.get('signature') if isinstance(preferred_record,dict)
                   else preferred_record)
        preferred_rank=(preferred_record.get('route_rank') if isinstance(preferred_record,dict)
                        else None)
        lazy_alternates=(os.environ.get('THREEYD_V6_LAZY_RESUME','0')=='1'
                         and (preferred is None or preferred_rank==1)
                         and self.max_region_routes>1)
        l1_tick=time.monotonic()
        routes=list(self.topology.routes(query,limit=1 if lazy_alternates else self.max_region_routes))
        if lazy_alternates and preferred is not None and routes and routes[0].signature!=preferred:
            # A stale first-choice certificate cannot override a stored
            # successful route; reconstruct the normal ordered candidates.
            routes=list(self.topology.routes(query,limit=self.max_region_routes))
            lazy_alternates=False
        l1_actual_ms=(time.monotonic()-l1_tick)*1000
        initial_l1_diagnostics=dict(getattr(self.topology,'_probe_stats',{}))
        fallback_l1_diagnostics=None
        if not routes:raise RuntimeError('NO_LEGAL_SEMANTIC_ROUTE')
        original_rank={route.signature:index+1 for index,route in enumerate(routes)}
        if preferred is not None:
            routes=sorted(routes,key=lambda route:(route.signature!=preferred,original_rank[route.signature]))
        alternate_search_triggered=False
        first_phase_resumed=False
        def expand_after_rejection():
            nonlocal l1_actual_ms,original_rank,alternate_search_triggered,first_phase_resumed,fallback_l1_diagnostics
            if not lazy_alternates or alternate_search_triggered:return
            alternate_search_triggered=True
            started=time.monotonic()
            complete=list(self.topology.routes(query,limit=self.max_region_routes))
            fallback_l1_diagnostics=dict(getattr(self.topology,'_probe_stats',{}))
            first_phase_resumed=bool(getattr(self.topology,'_last_first_phase_resume',False))
            l1_actual_ms+=(time.monotonic()-started)*1000
            original_rank={item.signature:index+1 for index,item in enumerate(complete)}
            known={item.signature for item in routes}
            routes.extend(item for item in complete if item.signature not in known)
        for attempt_order,candidate in enumerate(routes,1):
            attempt_started=time.monotonic();route=candidate;c=None
            failure_key=(query_key,route.signature)
            cached=failures.get(failure_key)
            if cached is not None:
                attempts.append({'route_rank':original_rank[route.signature],'attempt_order':attempt_order,
                    'regions':list(route.region_ids),'success':False,'failure':cached['failure'],
                    'wall_ms':(time.monotonic()-attempt_started)*1000,'preflight':'cached deterministic rejection',
                    'failure_cache_hit':True})
                out={'success':False,'failure_code':cached['failure']}
                expand_after_rejection()
                continue
            init_started=time.monotonic();init_ms=None;planning_ms=0.
            try:
                c=self.controller_class(self.world,self.topology,route,cache_root=self.root,views=self.views,dynamic=self.dynamic,weight=self.weight)
                init_ms=(time.monotonic()-init_started)*1000
                plan_started=time.monotonic()
                session.active_controller=c;session.active_route=route
                out=c.plan_l3(query,session,auditor,spec)
                planning_ms=(time.monotonic()-plan_started)*1000
            except (ValueError,RuntimeError,TimeoutError) as error:
                if any(code in str(error) for code in ('CORRUPT','BINDING','VERSION')):raise
                out={'success':False,'failure_code':str(error)}
            if init_ms is None:init_ms=(time.monotonic()-init_started)*1000
            attempts.append({'route_rank':original_rank[route.signature],'attempt_order':attempt_order,
                'regions':list(route.region_ids),
                'success':bool(out.get('success')),'failure':out.get('failure_code'),
                'wall_ms':(time.monotonic()-attempt_started)*1000,'preflight':'executed','controller_actual_ms':init_ms,'l3_pipeline_actual_ms':planning_ms,
                'adapter_actual_ms':0. if c is None else getattr(c,'timing',{}).get('adapter_actual_ms',0.),
                'l2_timing':{} if c is None else dict(getattr(c,'timing',{})),
                'l3_diagnostics':out.get('diagnostics',{}),
                'curvature_repair_ms':out.get('curvature_repair',{}).get('wall_ms',0.),
                'local_geometry_repair_ms':out.get('local_geometry_repair',{}).get('wall_ms',0.),
                'failure_cache_hit':False})
            failure=str(out.get('failure_code',''))
            if (not out.get('success') and os.environ.get('THREEYD_R7_SAVE_REJECTED')=='1'
                    and out.get('result') is not None and out['result'].points):
                import numpy as np
                points=np.asarray([[p['x'],p['y'],p['yaw']] for p in out['result'].points])
                np.save(self.root/f'r7_rejected_{query.query_id}_rank{original_rank[route.signature]}.npy',points)
            if out.get('success'):
                successes[query_key]={'signature':route.signature,
                                      'route_rank':original_rank[route.signature]}
                break
            if any(code in failure for code in ('ACK','CONTENT','VERSION','BINDING','CORRUPT')):break
            if self._deterministic_failure(failure):
                failures[failure_key]={'failure':failure,'created_monotonic':time.monotonic()}
            if c is not None:c.lifecycle.clear()
            expand_after_rejection()
        out['candidate_recovery']={'max_region_routes':self.max_region_routes,'attempts':attempts,
            'fallback_trigger':'prior legal route rejected; shared query deadline','global_fallback':False,
            'wall_ms':(time.monotonic()-tick)*1000,'l1_actual_ms':l1_actual_ms,
            'all_candidates_controller_ms':sum(x.get('controller_actual_ms',0.) for x in attempts),
            'all_candidates_l3_pipeline_ms':sum(x.get('l3_pipeline_actual_ms',0.) for x in attempts),
            'all_candidates_adapter_ms':sum(x.get('adapter_actual_ms',0.) for x in attempts),
            'preflight_cache_version':'V5-r9-preflight-v1',
            'failure_cache':failures.diagnostics(),'success_route_cache':successes.diagnostics(),
            'preferred_route_hit':preferred is not None}
        out['candidate_recovery']['lazy_alternates']=lazy_alternates
        out['candidate_recovery']['alternate_search_triggered']=alternate_search_triggered
        out['candidate_recovery']['first_phase_resumed']=first_phase_resumed
        out['candidate_recovery']['l1_search_diagnostics']={
            'initial':initial_l1_diagnostics,
            'after_rejection':fallback_l1_diagnostics,
        }
        return c,route,out
