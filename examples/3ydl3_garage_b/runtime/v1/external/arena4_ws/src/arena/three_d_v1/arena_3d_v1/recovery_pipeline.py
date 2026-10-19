"""3YD-V1 static recovery candidate; original Smac, vehicle and audits retained."""
import json,time,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .multires_pipeline import MultiresController,reference_route
from .multires_grid import GridView
from .semantic_reference import preference_field,RoutePreference
from .semantic_world import digest,json_write
from .local_fine import LocalFineGraph
from arena_evaluation.semantic_map import canonical_hash

RECOVERABLE={'COARSE_ENDPOINT_UNREPRESENTABLE','GUIDE_NO_PATH','GUIDE_SEARCH_TIMEOUT','WEIGHTED_L2_NO_PATH','L2_NO_PATH','L2_TIMEOUT','L2_UNVERIFIED_NO_PATH'}

class RecoveryController(MultiresController):
    def __init__(self,*args,**kwargs):
        kwargs["cache_root"]=Path(kwargs["cache_root"])/"recovery-junction-axis-v4"
        super().__init__(*args,**kwargs)
        self.timing.setdefault('completion_mode','full_fine_fallback' if self.fallbacks else 'coarse_only')
        self.timing['reference_resolutions_m']=[.15,.05] if self.timing['completion_mode']=='local_fine_assisted' else [self.view.map.resolution]

    def _reference_route(self,world,plan,xy):
        route=reference_route(world,plan,xy)
        # Long, approximately straight semantic lanes supply a stable lateral
        # axis. The sign still comes from this ordered route's direction.
        # Short/non-elongated lanes and cross-lane segments retain tangents.
        poses=route.poses.copy();labels=world.sample_labels(poses)
        junction_ids=[k for k,f in world.features.items() if f.semantic_class=='junction_area']
        junction=np.flatnonzero(np.isin(labels,junction_ids))
        if not len(junction):return route
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1))]
        # The soft influence extends 2 m; add the unchanged vehicle's turn
        # radius and half diagonal, rounded up to a whole metre (currently 3).
        influence=math.ceil(2.+world.vehicle[2]+math.hypot(*world.vehicle[:2]))
        near=np.min(np.abs(arc[:,None]-arc[junction][None,:]),axis=1)<=influence
        for label,feature in world.features.items():
            if feature.semantic_class!='lane':continue
            vertices=np.asarray(feature.coordinates,float)
            if len(vertices)>1 and np.allclose(vertices[0],vertices[-1]):vertices=vertices[:-1]
            vals,axes=np.linalg.eigh(np.cov(vertices[:,:2].T))
            if vals[-1]<9*max(vals[0],1e-12):continue
            axis=axes[:,-1];sel=np.flatnonzero((labels==label)&near)
            tangent=np.column_stack((np.cos(poses[sel,2]),np.sin(poses[sel,2])))
            dot=tangent@axis;sel=sel[np.abs(dot)>=math.cos(math.pi/4)];dot=dot[np.abs(dot)>=math.cos(math.pi/4)]
            poses[sel,2]=math.atan2(axis[1],axis[0])+np.where(dot>=0,0.,math.pi)
        route.poses=poses;route.edges[0].path=poses
        return route

    def _activate(self,factor,verify):
        if factor!=3:return super()._activate(factor,verify)
        try:return super()._activate(factor,verify)
        except (ValueError,RuntimeError,TimeoutError) as error:
            if str(error) not in RECOVERABLE:raise
            try:self._activate_local(verify)
            except (RuntimeError,TimeoutError) as local:
                if not str(local).startswith(('LOCAL_FINE_','GUIDE_')):raise
                self.timing['local_fine_failure']=str(local)
                raise error

    def _activate_local(self,verify):
        tick=time.monotonic();world=self.world
        for factor in (1,3):
            if factor not in self.views:self.views[factor]=GridView(world,factor)
        v=self.views[3]
        key=canonical_hash(['local-fine-junction-axis-v4',world.key,self.plan.route_signature,digest(self.plan.corridor_mask),self.side,self.weight])
        cache=self.root/'local_fine'/key;cache.mkdir(parents=True,exist_ok=True)
        hit=(cache/'path.npz').exists()
        if hit:
            with np.load(cache/'path.npz',allow_pickle=False) as data:rc=data['reference'];guide=data['guide']
            meta=json.loads((cache/'path.json').read_text())
            if meta['key']!=key or meta['hash']!=canonical_hash([digest(rc),digest(guide)]):raise ValueError('LOCAL_REFERENCE_CACHE_CORRUPT')
            stats=meta['stats'];prefkey=meta['preference_key'];resident=0
        else:
            graph=LocalFineGraph(world,v,self.plan.corridor_mask,self.plan.start_cell,self.plan.goal_cell,self.plan.route_signature)
            guide,gt=graph.search();xy=world.world_points(guide[:,0],guide[:,1])
            # Evaluate preference only on hybrid nodes; fine geometry supplies
            # physical lane widths even where conservative coarse cells vanish.
            rp=SimpleNamespace(corridor_mask=graph.active,route_signature=self.plan.route_signature)
            pref=preference_field(world,self._reference_route(world,rp,xy),side=self.side)
            rc,wt=graph.search(pref.potential,self.weight)
            if verify:
                _,oracle=graph.search(pref.potential,self.weight,heuristic=False)
                if abs(oracle['cost_cells']-wt['cost_cells'])>1e-8:raise AssertionError('LOCAL_FINE_ORACLE_MISMATCH')
            coarse_steps=int(np.sum(np.max(np.abs(np.diff(rc,axis=0)),axis=1)==3))
            if not coarse_steps:raise RuntimeError('LOCAL_FINE_NO_COARSE_SEGMENT')
            stats={**graph.stats,'guide':gt,'weighted':wt,'coarse_steps':coarse_steps};prefkey=pref.key
            resident=int(graph.active.nbytes+graph.patch.nbytes+graph.coarse_centers.nbytes+graph.safe.nbytes+pref.potential.nbytes)
            np.savez_compressed(cache/'path.npz',reference=rc,guide=guide)
            json_write(cache/'path.json',{'key':key,'hash':canonical_hash([digest(rc),digest(guide)]),'stats':stats,'preference_key':prefkey})
        if not np.all(self.plan.corridor_mask[rc[:,0],rc[:,1]]&world.safe[rc[:,0],rc[:,1]]):raise ValueError('LOCAL_REFERENCE_UNSAFE')
        self.reference_cells=rc;self.route=self._reference_route(world,self.plan,world.world_points(guide[:,0],guide[:,1]))
        self.view=self.views[1];self.preference=SimpleNamespace(key=prefkey)
        diagnostics={'backend':'bounded_hybrid_astar','activation':{'state_cache_hit':False,'reference_cache_hit':hit},'state_memory_bytes':resident,**stats}
        self.initial_l2_result=SimpleNamespace(success=True,diagnostics=diagnostics,oracle_cost_error=0. if verify else None)
        self.l2=SimpleNamespace(path_global=rc);self.lifecycle=SimpleNamespace(clear=lambda:None)
        self.timing.update(completion_mode='local_fine_assisted',local_fine=stats,reference_cache_hit=hit,local_fine_ms=(time.monotonic()-tick)*1000)

class CandidateRecovery:
    """Bounded query-local alternatives; a failed route never closes a channel.

Each branch excludes one physical portal transition *for that candidate
search only*. The union of branches contains alternatives to the failed
sequence. Exclusions are discarded at the end of the query, never written
as observed blockage. Samples on the same portal do not consume retries.
"""
    def __init__(self,world,topology,*,cache_root,views=None,max_attempts=4,max_candidate_searches=64,budget_s=60.,controller_class=RecoveryController):
        self.world=world;self.topology=topology;self.root=Path(cache_root);self.views=views
        self.max_attempts=max_attempts;self.max_candidate_searches=max_candidate_searches;self.budget_s=budget_s;self.controller_class=controller_class
    def plan(self,query,session,auditor,spec,*,side='right',initial_route=None):
        import heapq,itertools
        started=time.monotonic();queue=[];serial=itertools.count();seen_constraints=set();seen_routes=set();excluded_interfaces=set();attempts=[];searches=0;l1_ms=0.;l2_ms=0.;last=None;controller=None
        def ids(route):return (-1,*[self.topology.nodes[i].portal for i in route.interface_ids],-2)
        def submit(constraints,route=None):
            nonlocal searches,l1_ms
            if constraints in seen_constraints or searches>=self.max_candidate_searches:return
            if time.monotonic()-started>=self.budget_s:return
            seen_constraints.add(constraints);searches+=1;t=time.monotonic()
            try:route=route or self.topology.plan(query,excluded_interfaces=excluded_interfaces,excluded_portal_transitions=constraints,timeout=min(5.,max(.01,self.budget_s-(time.monotonic()-started))))
            except (RuntimeError,TimeoutError):return
            finally:l1_ms+=(time.monotonic()-t)*1000
            points=[route.start]+[self.topology.nodes[i].pose for i in route.interface_ids]+[route.goal]
            cost=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
            heapq.heappush(queue,(cost,next(serial),route,constraints))
        submit(frozenset(),initial_route)
        while queue and len(attempts)<self.max_attempts and time.monotonic()-started<self.budget_s:
            _,_,route,constraints=heapq.heappop(queue);sequence=ids(route)
            if sequence in seen_routes:continue
            seen_routes.add(sequence);controller=None;t=time.monotonic()
            try:
                controller=self.controller_class(self.world,self.topology,route,cache_root=self.root,factor=3,side=side,views=self.views)
            except (RuntimeError,TimeoutError) as error:
                if str(error) not in RECOVERABLE|{'FINE_ENDPOINT_UNSAFE','LOCAL_FINE_NO_PATH'}:raise
                last={'success':False,'failure_code':str(error)}
            else:
                l2_ms+=(time.monotonic()-t)*1000
                last=controller.plan_l3(query,session,auditor,spec,enabled=True,cap=140)
            if controller is None:l2_ms+=(time.monotonic()-t)*1000
            failure=str(last.get('failure_code',''));attempts.append({'candidate_signature':route.signature,'portal_sequence':sequence,'success':bool(last.get('success')),'failure_code':failure,'evidence':'validated_final_path' if last.get('success') else 'candidate_failure_only_no_channel_closure'})
            if last.get('local_curvature_repair'):
                # Retain rejected local repairs even when a later topology
                # candidate replaces the final outcome returned to the caller.
                attempts[-1]['local_curvature_repair']=last['local_curvature_repair']
            if last.get('success'):break
            if controller:controller.lifecycle.clear()
            if any(word in failure.upper() for word in ['ACK','CONTENT','BINDING','CORRUPT','VERSION']):break
            invalid=[]
            for node_id in route.interface_ids:
                node=self.topology.nodes[node_id];cell=self.world.map.world_to_cell(*node.pose[:2])
                if cell is None or self.world.hard[cell]:invalid.append(node_id)
            if invalid:
                # Exact static evidence rejects only these sampled interfaces
                # for this query, not a whole physical/dynamic channel.
                attempts[-1]['invalid_interface_samples']=invalid
                excluded_interfaces.update(invalid);queue.clear();seen_constraints.clear();submit(frozenset())
            else:
                for pair in zip(sequence,sequence[1:]):submit(constraints|{pair})
        if last is None:last={'success':False,'failure_code':'CANDIDATE_SEARCH_EXHAUSTED'}
        diagnostics={'attempts':attempts,'candidate_searches':searches,'l1_ms':l1_ms,'l2_ms':l2_ms,'wall_ms':(time.monotonic()-started)*1000,'max_attempts':self.max_attempts,'max_candidate_searches':self.max_candidate_searches,'new_attempt_admission_budget_s':self.budget_s,'persistent_channel_updates':0}
        last['candidate_recovery']=diagnostics
        return controller,route if attempts else initial_route,last,diagnostics
