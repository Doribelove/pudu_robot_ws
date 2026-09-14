"""Static multiresolution composition with unchanged fine Smac and audits."""
from pathlib import Path
from dataclasses import dataclass
from types import SimpleNamespace
import json, math, time
import numpy as np
from scipy.ndimage import gaussian_filter1d
from arena_evaluation.semantic_map import canonical_hash
from .multires_grid import GridView, make_roi, guide_astar
from .semantic_world import digest, json_write
from .semantic_graph import SemanticRoute, Connection
from .semantic_reference import preference_field, RoutePreference, reference_costmap
from .semantic_l2 import WeightedDStarState, WeightedCorridorDStar, SemanticL2Lifecycle
from .l2_state_lifecycle import CompactGeometryBinding, CompactCorridorGeometry, MutableStateBinding, CacheTelemetry
from .pipeline import L1Plan, PipelineStep, ProductionL3Adapter, corridor_dirty_transition

@dataclass(frozen=True)
class ReferencePlan:
    world_xy: np.ndarray
    l2_resolution_m: float
    topology_signature: str
    world_key: str
    preference_key: str
    fine_corridor_hash: str
    occupancy_version: str = 'static'

class DirectionalState(WeightedDStarState):
    def __init__(self,geometry,binding,potential,weight,view):
        self.cells=np.asarray([geometry.global_cell(i) for i in range(geometry.state_count)])
        self.directions=[(mask[self.cells[:,0],self.cells[:,1]],yaw) for mask,yaw in view.directions]
        super().__init__(geometry,binding,potential,weight)
    def _edge_cost(self,a,b):
        if self.directions:
            dr,dc=self.cells[b]-self.cells[a]
            for membership,yaw in self.directions:
                if (membership[a] or membership[b]) and dc*math.cos(yaw)-dr*math.sin(yaw)<-1e-6:return math.inf
        return super()._edge_cost(a,b)
    @property
    def resident_bytes(self):
        return super().resident_bytes+self.cells.nbytes+sum(x.nbytes for x,_ in self.directions)

class MultiresLifecycle(SemanticL2Lifecycle):
    def __init__(self,root,potential,weight,view):
        super().__init__(root,potential,weight);self.view=view
    def activate(self,roi,*,dynamic_baseline_version='static',verify_oracle=False):
        tick=time.monotonic();gb=CompactGeometryBinding.from_roi(roi,safety_policy_hash='conservative-aggregate-original-safe-static-v1')
        geometry,gt=self.geometry_cache.restore(gb)
        if geometry is None:
            geometry=CompactCorridorGeometry.build(roi,safety_policy_hash='conservative-aggregate-original-safe-static-v1');self.geometry_cache.save(geometry)
        rc=np.asarray([geometry.global_cell(i) for i in range(geometry.state_count)])
        costs=self.potential[rc[:,0],rc[:,1]]
        version=canonical_hash(['3yd-static-weighted-v1',digest(costs),self.weight,self.view.key])
        binding=MutableStateBinding(geometry.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,dynamic_baseline_version,algorithm_version=version)
        restored,st=self.state_cache.restore(geometry,binding)
        state=DirectionalState(geometry,binding,costs,self.weight,self.view)
        if restored is not None:state.__dict__.update(restored.__dict__);state._refresh_views()
        planner=WeightedCorridorDStar(roi,geometry,state);result=planner.initialize(verify_oracle=verify_oracle)
        if not st.hit:self.state_cache.save(state)
        self.active={binding.digest:planner};self.peak_active_state_count=1
        self.peak_resident_bytes=max(self.peak_resident_bytes,self.resident_bytes)
        telemetry={'active_hit':False,'geometry_cache_hit':gt.hit,'state_cache_hit':st.hit,
            'geometry_reject':gt.reject_reason,'state_reject':st.reject_reason,'activate_ms':(time.monotonic()-tick)*1000,
            'l2_resolution_m':self.view.map.resolution,'resident_bytes':self.resident_bytes}
        return planner,result,SimpleNamespace(as_dict=lambda:telemetry)

def reference_route(world,plan,xy):
    """Direction is a metre-scale tangent; the guide is not an executable path."""
    xy=np.asarray(xy,float);ds=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    if len(xy)<2 or np.any(ds<=0):raise ValueError('INVALID_REFERENCE_GEOMETRY')
    # Smooth only tangents, never move guide points through obstacles.
    delta=np.diff(xy,axis=0);sigma=.50/max(float(np.median(ds)),.01)
    tangent=gaussian_filter1d(delta,sigma=sigma,axis=0,mode='nearest')
    yaw=np.arctan2(tangent[:,1],tangent[:,0]);poses=np.column_stack([xy,np.r_[yaw,yaw[-1]]])
    edge=Connection(0,0,1,0,float(ds.sum()),poses,{'path_sha256':digest(poses),'kind':'2d_reference_not_vehicle_connection'})
    return SemanticRoute(plan,poses,[],[edge],[],{})

class MultiresController:
    def __init__(self,world,topology,route,*,cache_root,factor=3,side='right',weight=2.,verify_l2_oracle=False,views=None):
        tick=time.monotonic();cpu=time.process_time();self.world=world;self.topology=topology;self.topology_route=route
        self.root=Path(cache_root);self.side=side;self.weight=weight;self.requested_factor=factor
        self.views=views if views is not None else {};self.fallbacks=[]
        mask=topology.fine_mask(route)
        self.plan=L1Plan(world.safe,mask,world.map.world_to_cell(*route.start[:2]),world.map.world_to_cell(*route.goal[:2]),
            world.map.sha256,world.map.origin,world.map.resolution,topology.key,tuple(map(str,route.interface_ids)),
            canonical_hash(world.vehicle),route.signature,{'semantic_topology':True,'regions':route.region_ids})
        self.server_l3_mask=mask.copy();self.pending_l3_mask=mask;self._pending_l3_hash=''
        self.timing={'fine_corridor_ms':(time.monotonic()-tick)*1000}
        try:self._activate(factor,verify_l2_oracle)
        except (ValueError,RuntimeError,TimeoutError) as e:
            # Only coarse representability/search failures justify a fine retry.
            # Integrity failures must remain visible and cannot be hidden by it.
            recoverable={'COARSE_ENDPOINT_UNREPRESENTABLE','GUIDE_NO_PATH','GUIDE_SEARCH_TIMEOUT',
                'WEIGHTED_L2_NO_PATH','DSTAR_NO_PATH','L2_NO_PATH','L2_TIMEOUT','L2_UNVERIFIED_NO_PATH'}
            if factor==1 or str(e) not in recoverable:raise
            self.fallbacks.append({'from_resolution':.15,'to_resolution':.05,'reason':type(e).__name__+': '+str(e)})
            self._activate(1,verify_l2_oracle)
        self.timing.update(l2_total_ms=(time.monotonic()-tick)*1000,l2_cpu_ms=(time.process_time()-cpu)*1000,
            requested_resolution_m=.05*factor,actual_resolution_m=self.view.map.resolution,fine_fallback=bool(self.fallbacks))
        xy=world.world_points(self.reference_cells[:,0],self.reference_cells[:,1]);xy.setflags(write=False)
        self.reference_plan=ReferencePlan(xy,self.view.map.resolution,route.signature,world.key,
            self.preference.key,digest(self.plan.corridor_mask))

    def _activate(self,factor,verify):
        tick=time.monotonic()
        if factor not in self.views:self.views[factor]=GridView(self.world,factor)
        self.view=self.views[factor];v=self.view;mask=v.corridor(self.plan.corridor_mask)
        start,goal=v.cell(self.topology_route.start),v.cell(self.topology_route.goal)
        roi=make_roi(v,mask,start,goal,self.plan.route_signature)
        self.timing['grid_bind_ms']=(time.monotonic()-tick)*1000
        key=canonical_hash([v.key,self.plan.route_signature,self.side,'tangent-half-m-v1',digest(mask)])
        cache=self.root/'reference'/key;cache.mkdir(parents=True,exist_ok=True)
        tick=time.monotonic()
        if (cache/'field.npz').exists():
            with np.load(cache/'field.npz',allow_pickle=False) as d:
                guide=d['guide'];pot=d['potential'];q=d['normalized'];eligible=d['eligible'];amb=d['ambiguous']
            meta=json.loads((cache/'field.json').read_text())
            if meta['key']!=key or meta['arrays_hash']!=canonical_hash([digest(x) for x in [guide,pot,q,eligible,amb]]):raise ValueError('REFERENCE_CACHE_CORRUPT')
            self.preference=RoutePreference(pot,q,eligible,amb,meta['preference_key'],{'cache_hit':True,'build_ms':0.,'cpu_ms':0.})
            self.timing['guide_expanded']=0;self.timing['reference_cache_hit']=True
        else:
            guide,gt=guide_astar(v,roi);self.timing.update(gt);self.timing['reference_cache_hit']=False
            xy=v.world_points(guide[:,0],guide[:,1])
            coarse_plan=SimpleNamespace(corridor_mask=mask,route_signature=self.plan.route_signature)
            guide_route=reference_route(v,coarse_plan,xy)
            self.preference=preference_field(v,guide_route,side=self.side)
            p=self.preference
            np.savez_compressed(cache/'field.npz',guide=guide,potential=p.potential,normalized=p.normalized_right,eligible=p.eligible,ambiguous=p.ambiguous)
            json_write(cache/'field.json',{'key':key,'preference_key':p.key,'arrays_hash':canonical_hash([digest(x) for x in [guide,p.potential,p.normalized_right,p.eligible,p.ambiguous]])})
        self.timing['guide_preference_cache_ms']=(time.monotonic()-tick)*1000
        self.timing['preference_build_ms']=self.preference.diagnostics.get('build_ms',0.)
        self.lifecycle=MultiresLifecycle(self.root/f'grid_{factor}',self.preference.potential,self.weight,v)
        tick=time.monotonic();self.l2,self.initial_l2_result,activation=self.lifecycle.activate(roi,verify_oracle=verify)
        self.timing['weighted_l2_ms']=(time.monotonic()-tick)*1000
        self.initial_l2_result.diagnostics['activation']=activation.as_dict()
        if not self.initial_l2_result.success:raise RuntimeError(self.initial_l2_result.failure_code or 'WEIGHTED_L2_NO_PATH')
        rc=np.asarray(self.l2.path_global);xy=v.world_points(rc[:,0],rc[:,1])
        fr,fc=self.world.cells(xy);self.reference_cells=np.column_stack([fr,fc])
        if not np.all(self.plan.corridor_mask[fr,fc]&self.world.safe[fr,fc]):raise ValueError('REFERENCE_FINE_PROJECTION_UNSAFE')
        # Use the unbiased guide for route matching and the weighted path as soft reference.
        self.route=reference_route(self.world,self.plan,v.world_points(guide[:,0],guide[:,1]))

    def _target_mask(self):return self.plan.corridor_mask
    def acknowledge_l3_mask(self,content_hash):
        if content_hash!=self._pending_l3_hash:raise ValueError('L3_MASK_ACK_MISMATCH')
        self.server_l3_mask=self.pending_l3_mask.copy();self._pending_l3_hash=''
    def plan_l3(self,query,session,auditor,spec,*,enabled=True,cap=140):
        if tuple(self.world.vehicle)!=tuple(session.bound_vehicle):raise ValueError('VEHICLE_VERSION_MISMATCH')
        if self.reference_plan.world_key!=self.world.key or self.reference_plan.fine_corridor_hash!=digest(self.plan.corridor_mask):
            raise ValueError('REFERENCE_BINDING_MISMATCH')
        grid=reference_costmap(self.world,self.route,self.reference_cells,enabled=enabled,cap=cap,side=self.side)
        self.pending_l3_mask=self.plan.corridor_mask;dirty=corridor_dirty_transition(self.server_l3_mask,self.pending_l3_mask)
        self._pending_l3_hash=dirty.target_hash
        step=PipelineStep(None,None,self.initial_l2_result,False,False,True,'',self.pending_l3_mask,dirty,self.plan.route_signature,{})
        session.set_semantic_costmap(grid);out=ProductionL3Adapter(self,auditor).plan(step,query,session,spec)
        out['reference_diagnostics']=grid.diagnostics;out['multires_diagnostics']={**self.timing,'fallbacks':self.fallbacks}
        if out.get('result') and out['result'].points:
            p=np.asarray([[x['x'],x['y'],x['yaw']] for x in out['result'].points])
            audit=self.world.audit(query,p,self.plan.corridor_mask);out['independent_audit']=audit
            out['success']=bool(out.get('success') and audit['valid'])
            if not audit['valid']:out['failure_code']='INDEPENDENT_AUDIT_REJECTED'
        return out
