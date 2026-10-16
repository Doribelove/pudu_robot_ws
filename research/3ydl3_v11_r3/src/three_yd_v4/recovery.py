import time
from .guidance import road_groups
from three_yd_v3.edge_routing import AttachmentError

class V4CandidateRecovery:
    """Bounded alternative ORIGINAL edge sequences, without modifying the graph."""
    def __init__(self, world, topology, cache_root, views, dynamic=None, max_attempts=3, controller_class=None, reference_feedback=False, approach_feedback=False):
        self.world=world;self.topology=topology;self.cache_root=cache_root;self.views=views
        self.dynamic=dynamic;self.max_attempts=max_attempts;self.controller_class=controller_class
        self.reference_feedback=reference_feedback
        self.approach_feedback=approach_feedback

    def plan(self,query,session,auditor,spec):
        tick=time.monotonic();queue=[(None,None,frozenset())];seen=set();attempts=[];c=None;r=None;out={};searches=0
        # Diversity by authorized endpoint road precedes variations of the
        # middle edges. No nearest-road substitution or permanent graph edit.
        roads={}
        for role in ('start','goal'):
            pose=getattr(query,role);rid=int(self.topology.subregions[self.world.map.world_to_cell(*pose[:2])])
            roads[role]=sorted({a['road_id'] for a in self.topology.router.attach(query.query_id,role,pose,rid)})
        if len(roads['start'])*len(roads['goal'])>1:
            queue.extend((a,b,frozenset()) for a in roads['start'] for b in roads['goal'])
        while queue and len(attempts)<self.max_attempts and searches<32:
            start_road,goal_road,exclusions=queue.pop(0);searches+=1
            try:r=self.topology.plan(query,excluded_edge_transitions=exclusions,start_road=start_road,goal_road=goal_road)
            except AttachmentError as error:
                out={'success':False,'failure_code':str(error),'attachment_error':error.details};break
            except (RuntimeError,ValueError) as error:
                out=out or {'success':False,'failure_code':str(error)};continue
            signature=r.interface_ids
            task=self.topology.task(r)
            semantic_signature=(tuple((group[0]['region_id'],group[0]['screen_side']) for group in road_groups(task)),
                tuple(task['endpoint_access'][role]['road_id'] for role in ('start','goal')))
            if semantic_signature in seen:continue
            seen.add(semantic_signature);c=None;started=time.monotonic()
            try:
                version=0 if self.dynamic is None else self.dynamic.version
                feedback_key=('validated_approach_connections',self.world.key,r.signature,version)
                connections=self.views.get(feedback_key,[]) if self.approach_feedback else []
                c=self.controller_class(self.world,self.topology,r,cache_root=self.cache_root,views=self.views,dynamic=self.dynamic,feedback_enabled=self.reference_feedback and not attempts,approach_connections=connections)
                out=c.plan_l3(query,session,auditor,spec)
            except (RuntimeError,ValueError,TimeoutError) as error:
                if any(v in str(error).upper() for v in ('CORRUPT','BINDING','VERSION')):raise
                out={'success':False,'failure_code':str(error)}
            attempts.append({'regions':list(r.region_ids),'edge_uses':list(signature),'success':out.get('success',False),
                'start_road':self.topology.task(r)['endpoint_access']['start']['road_id'],
                'goal_road':self.topology.task(r)['endpoint_access']['goal']['road_id'],
                'geometry_success':out.get('geometry_success',False),'failure':out.get('failure_code'),
                'feasibility_recovery':out.get('feasibility_recovery',{}),
                'native_termination_reason':out.get('diagnostics',{}).get('termination_reason'),
                'wall_ms':(time.monotonic()-started)*1000,'l1':r.diagnostics,'semantic_signature':semantic_signature})
            if out.get('success'):
                verified=out.get('approach_feedback',{}).get('connections',[])
                if verified:self.views[feedback_key]=verified
                break
            if any(v in str(out.get('failure_code','')).upper() for v in ('ACK','CONTENT','CORRUPT','BINDING','VERSION')):break
            # Exclude a query-local transition only; no claim of permanent blockage.
            for a,b in zip(signature,signature[1:]):
                ua,ub=self.topology.router.uses[a],self.topology.router.uses[b]
                if ua.region_id==ub.region_id:continue
                # Exclude equivalent region crossings together for this query,
                # so variants of one failed perimeter are not expensive retries.
                equivalent={(x.use_id,y.use_id) for x in self.topology.router.uses.values()
                    if x.region_id==ua.region_id for y in self.topology.router.uses.values()
                    if y.region_id==ub.region_id and x.end_key==y.start_key}
                queue.append((start_road,goal_road,exclusions|equivalent))
            if c:c.lifecycle.clear()
        if not out:out={'success':False,'failure_code':'EDGE_CANDIDATES_EXHAUSTED'}
        out['candidate_recovery']={'attempts':attempts,'searches':searches,'wall_ms':(time.monotonic()-tick)*1000,'persistent_topology_mutations':0,'deduplication':'road_order_and_travel_side','failure_is_blockage_evidence':False}
        return c,r,out
