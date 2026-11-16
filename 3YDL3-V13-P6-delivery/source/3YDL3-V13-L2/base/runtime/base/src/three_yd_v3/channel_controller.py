"""Ordered-channel integration; fixed Smac and vehicle remain unchanged."""
from dataclasses import replace
from types import SimpleNamespace
import math,time,json
import numpy as np
from scipy import ndimage
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.multires_grid import GridView,make_roi
from arena_3d_v1.l2_state_lifecycle import MutableStateBinding,VerifiedMutableStateCache
from arena_3d_v1.semantic_l2 import WeightedCorridorDStar,weighted_astar
from arena_3d_v1.semantic_reference import preference_field,RoutePreference
from arena_3d_v1.semantic_world import digest,json_write
from arena_evaluation.semantic_costmap_composer import occupancy_to_static_layer,internal_soft_to_occupancy
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master
from .controller import V3Controller,V3CandidateRecovery
from .channel import Channel
from .staged_l2 import StageGeometry,StageState

class ChannelController(V3Controller):
    channel_class=Channel
    def __init__(self,world,topology,route,**kwargs):
        self.channel=self.channel_class(world,topology,route)
        self.center_mask=topology.center_mask(route)
        super().__init__(world,topology,route,**kwargs)
    def _activate(self,factor,verify):
        started=time.monotonic()
        if factor not in self.views:self.views[factor]=GridView(self.world,factor)
        v=self.views[factor];self.view=v
        # Semantic center restriction is distinct from the body footprint mask.
        mask=v.corridor(self.center_mask)
        roi=make_roi(v,mask,v.cell(self.topology_route.start),v.cell(self.topology_route.goal),self.plan.route_signature)
        labels,_=ndimage.label(roi.base_free)
        if labels[roi.start_local]==0 or labels[roi.start_local]!=labels[roi.goal_local]:raise RuntimeError('GUIDE_NO_PATH')
        geometry=StageGeometry(roi,v,self.channel)
        key=canonical_hash(['stage-pref-v1',geometry.binding.digest,self.side,self.weight])
        cache=self.root/'stage_reference'/key;cache.mkdir(parents=True,exist_ok=True)
        if (cache/'field.npz').exists():
            with np.load(cache/'field.npz',allow_pickle=False) as d:guide=d['guide'];pot=d['potential'];q=d['normalized'];eligible=d['eligible'];amb=d['ambiguous']
            meta=json.loads((cache/'field.json').read_text())
            if meta['hash']!=canonical_hash([digest(x) for x in [guide,pot,q,eligible,amb]]):raise ValueError('REFERENCE_CACHE_CORRUPT')
            self.preference=RoutePreference(pot,q,eligible,amb,meta['key'],{'cache_hit':True,'build_ms':0.})
            self.timing['reference_cache_hit']=True
        else:
            b=MutableStateBinding(geometry.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,'static','stage-guide-v1')
            st=StageState(geometry,b,np.zeros(geometry.state_count),0.)
            ids,stats=weighted_astar(st)
            if ids is None:raise RuntimeError('GUIDE_SEARCH_TIMEOUT' if stats.timeout_triggered else 'GUIDE_NO_PATH')
            guide=np.asarray([geometry.global_cell(i) for i in ids]);self.timing['guide_expanded']=stats.expanded_nodes
            coarse_plan=SimpleNamespace(corridor_mask=mask,route_signature=self.plan.route_signature)
            guide_route=self._reference_route(v,coarse_plan,v.world_points(guide[:,0],guide[:,1]))
            self.preference=preference_field(v,guide_route,side=self.side);p=self.preference
            np.savez_compressed(cache/'field.npz',guide=guide,potential=p.potential,normalized=p.normalized_right,eligible=p.eligible,ambiguous=p.ambiguous)
            json_write(cache/'field.json',{'key':p.key,'hash':canonical_hash([digest(x) for x in [guide,p.potential,p.normalized_right,p.eligible,p.ambiguous]])})
            self.timing['reference_cache_hit']=False
        potential=self.preference.potential[geometry.cells[:,0],geometry.cells[:,1]]
        binding=MutableStateBinding(geometry.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,'static',canonical_hash(['ordered-dstar-v1',digest(potential),self.weight]))
        disk=VerifiedMutableStateCache(self.root/'staged_dstar');restored,telemetry=disk.restore(geometry,binding)
        state=StageState(geometry,binding,potential,self.weight)
        if restored is not None:state.__dict__.update(restored.__dict__);state._refresh_views()
        self.l2=WeightedCorridorDStar(roi,geometry,state)
        self.initial_l2_result=self.l2.initialize(verify_oracle=verify)
        if restored is None:disk.save(state)
        self.lifecycle=SimpleNamespace(clear=lambda:None)
        if not self.initial_l2_result.success:raise RuntimeError(self.initial_l2_result.failure_code or 'WEIGHTED_L2_NO_PATH')
        rc=np.asarray(self.l2.path_global);xy=v.world_points(rc[:,0],rc[:,1]);r,c=self.world.cells(xy)
        self.reference_cells=np.column_stack((r,c))
        # Audit projected coarse reference, including exact semantic sequence.
        delta=np.diff(xy,axis=0);yaw=np.arctan2(delta[:,1],delta[:,0]);audit=self.channel.audit(np.column_stack((xy,np.r_[yaw,yaw[-1]])))
        if not audit['valid']:raise RuntimeError('GUIDE_NO_PATH')
        self.route=self._reference_route(self.world,self.plan,v.world_points(guide[:,0],guide[:,1]))
        self.timing.update(stage_geometry_states=geometry.state_count,stage_count=len(self.channel.regions),stage_activation_ms=(time.monotonic()-started)*1000,reference_channel_audit=audit)
        self.initial_l2_result.diagnostics['activation']={'state_cache_hit':telemetry.hit,'stage_state':True}
    def prepare_grid(self,enabled=True,cap=140):
        grid=super().prepare_grid(enabled,cap)
        if enabled:
            # r0 left unmatched semantic cells at zero cost. They could attract
            # shortcuts outside the ordered route. Fill that missing reference
            # penalty, while keeping all added costs finite and nonlethal.
            from scipy.spatial import cKDTree
            selected=self.plan.corridor_mask&~self.world.hard
            r,c=np.nonzero(selected);xy=self.world.world_points(r,c)
            ref=self.world.world_points(self.reference_cells[:,0],self.reference_cells[:,1])
            distance=cKDTree(ref).query(xy)[0]
            grid.soft_cost[r,c]=np.maximum(grid.soft_cost[r,c],cap*np.minimum(1.,distance/2.))
            grid.soft_cost[selected&~self.center_mask]=200.
            base=np.asarray(self.world.map.occupancy,dtype=np.int16).copy()
            occupancy=np.maximum(base,internal_soft_to_occupancy(grid.soft_cost)).astype(np.int8)
            occupancy[self.world.map.occupancy<0]=-1;occupancy[self.world.hard|~self.plan.corridor_mask]=100
            grid.occupancy_grid=occupancy
            grid.internal_cost=occupancy_to_static_layer(occupancy)
            grid.expected_master_cost=pinned_nav2_effective_master(grid.internal_cost,resolution=self.world.map.resolution,inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
            grid.policy_hash=canonical_hash([grid.policy_hash,'ordered-center-soft-v1',digest(grid.soft_cost),digest(occupancy)])
            grid.diagnostics.update(max_soft_cost=float(grid.soft_cost.max()),channel_center_soft=200.,channel_reference_cap=cap,channel_enforcement='stage L2 plus independent exact L3 audit; soft center cost is guidance only')
        return grid
    def plan_l3(self,query,session,auditor,spec,**kwargs):
        out=super().plan_l3(query,session,auditor,spec,**kwargs)
        if out.get('result') and out['result'].points:
            p=np.asarray([[v['x'],v['y'],v['yaw']] for v in out['result'].points]);audit=self.channel.audit(p)
            out['channel_audit']=audit;out.setdefault('constraint_audit',{})['channel']=audit
            out['success']=bool(out.get('success') and audit['valid'])
            if not audit['valid']:
                out['failure_code']=';'.join(filter(None,[out.get('failure_code'),audit['failure']]))
                self.trajectory=None;self.trajectory_version=None
            version=0 if self.dynamic is None else self.dynamic.version
            if (not out['success'] and out.get('geometry_success') and audit['valid']
                    and out.get('failure_code')=='RAW_STEERING_JUMP'
                    and out.get('diagnostics',{}).get('costmap_update_acknowledged') is True
                    and out.get('map_version')==version):
                from .channel_repair import repair_path
                from arena_3d_v1.curvature_repair import truthful_points
                from arena_3d_v1.semantic_world import wrap
                repaired,log=repair_path(self.world,self.channel,query,p,self._target_mask(),budget_s=5.)
                out['channel_curvature_repair']=log
                if repaired is not None:
                    points=truthful_points(repaired,out['result'],0,len(repaired)-1)
                    canonical=auditor.audit(query,points,self._target_mask())
                    independent=self.world.audit(query,repaired,self._target_mask())
                    ca=self.channel.audit(repaired)
                    d=np.diff(repaired,axis=0);ds=np.linalg.norm(d[:,:2],axis=1)
                    steering=np.arctan(.5*np.divide(wrap(d[:,2]),ds,out=np.zeros_like(ds),where=ds>1e-8))
                    jump=float(np.max(np.abs(np.diff(steering)),initial=0)*180/math.pi)
                    current=0 if self.dynamic is None else self.dynamic.version
                    accepted=bool(canonical.final_valid_success and independent['valid'] and ca['valid'] and jump<=15+1e-6 and current==version)
                    log.update(final_acceptance=accepted,final_raw_steering_jump_deg=jump,canonical_failure=canonical.metrics.get('failure_code'))
                    if accepted:
                        result=replace(out['result'],points=points,path_audit=canonical,source='smac_with_channel_local_curvature_repair')
                        out.update(success=True,failure_code='',result=result,metrics=canonical.metrics,independent_audit=independent,channel_audit=ca,
                                   diagnostics={**out['diagnostics'],**canonical.diagnostics()},constraint_audit={'max_raw_steering_jump_deg':jump,'steering_jump_valid':True,'ordered_regions_valid':True,'channel':ca})
                        self.trajectory=repaired;self.trajectory_version=version
            if self.dynamic is not None and self.dynamic.version!=out.get('map_version'):
                out.update(success=False,failure_code='MAP_VERSION_CHANGED_DURING_CHANNEL_REPAIR')
                self.trajectory=None;self.trajectory_version=None
        return out

    def update_dynamic(self,verify=False):
        update=super().update_dynamic(verify)
        if update['success']:
            xy=self.reference_plan.world_xy;delta=np.diff(xy,axis=0);yaw=np.arctan2(delta[:,1],delta[:,0])
            check=self.channel.audit(np.column_stack((xy,np.r_[yaw,yaw[-1]])))
            update['channel_audit']=check
            if not check['valid']:
                self.initial_l2_result.success=False;self.initial_l2_result.failure_code='DYNAMIC_CHANNEL_REFERENCE_REJECTED'
                update['success']=False
        return update
