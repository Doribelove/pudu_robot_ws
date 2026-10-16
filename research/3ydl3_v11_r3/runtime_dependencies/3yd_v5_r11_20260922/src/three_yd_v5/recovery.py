"""Bounded semantic-route recovery for V5.

At most three distinct legal region routes share the existing per-query
deadline.  Infrastructure/version failures stop immediately; a geometric or
traffic rejection may advance to the next route.  There is no global-map L3
fallback.
"""
import time
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.semantic_world import digest
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
        l1_tick=time.monotonic()
        routes=self.topology.routes(query,limit=self.max_region_routes)
        l1_actual_ms=(time.monotonic()-l1_tick)*1000
        if not routes:raise RuntimeError('NO_LEGAL_SEMANTIC_ROUTE')
        query_key=self._query_key(query)
        failures=get_cache(self.views,'candidate-failures',max_entries=64)
        successes=get_cache(self.views,'candidate-success',max_entries=64)
        preferred=successes.get(query_key)
        original_rank={route.signature:index+1 for index,route in enumerate(routes)}
        if preferred is not None:
            routes=sorted(routes,key=lambda route:(route.signature!=preferred,original_rank[route.signature]))
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
            if out.get('success'):
                successes[query_key]=route.signature
                break
            if any(code in failure for code in ('ACK','CONTENT','VERSION','BINDING','CORRUPT')):break
            if self._deterministic_failure(failure):
                failures[failure_key]={'failure':failure,'created_monotonic':time.monotonic()}
            if c is not None:c.lifecycle.clear()
        out['candidate_recovery']={'max_region_routes':self.max_region_routes,'attempts':attempts,
            'fallback_trigger':'prior legal route rejected; shared query deadline','global_fallback':False,
            'wall_ms':(time.monotonic()-tick)*1000,'l1_actual_ms':l1_actual_ms,
            'all_candidates_controller_ms':sum(x.get('controller_actual_ms',0.) for x in attempts),
            'all_candidates_l3_pipeline_ms':sum(x.get('l3_pipeline_actual_ms',0.) for x in attempts),
            'all_candidates_adapter_ms':sum(x.get('adapter_actual_ms',0.) for x in attempts),
            'preflight_cache_version':'V5-r9-preflight-v1',
            'failure_cache':failures.diagnostics(),'success_route_cache':successes.diagnostics(),
            'preferred_route_hit':preferred is not None}
        return c,route,out
