"""V4 reference search and frozen Smac boundary, with independent acceptance."""
from contextlib import nullcontext
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import math
import time
import numpy as np
import cv2
from scipy import ndimage
from scipy.spatial import cKDTree
from arena_evaluation.semantic_map import canonical_hash
from arena_evaluation.semantic_costmap_composer import SemanticCostmap, occupancy_to_static_layer, internal_soft_to_occupancy
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master
from arena_3d_v1.semantic_world import digest, wrap
from arena_3d_v1.multires_grid import GridView, aggregate, make_roi
from arena_3d_v1.multires_pipeline import MultiresController, ReferencePlan, reference_route
from arena_3d_v1.l2_state_lifecycle import CompactCorridorGeometry, MutableStateBinding, VerifiedMutableStateCache
from arena_3d_v1.semantic_l2 import WeightedDStarState, WeightedCorridorDStar, weighted_astar
from arena_3d_v1.pipeline import L1Plan, PipelineStep, ProductionL3Adapter, corridor_dirty_transition
from .guidance import edge_field, TrafficAudit, sample_uses
from .progress import ProgressGeometry


class BoundedReferenceDStar(WeightedCorridorDStar):
    """One bounded attempt per backend; timeouts are never blockage evidence."""
    def _solve(self,*,initial=False,verify_oracle=False,force=False,changed=0):
        from arena_3d_v1.l2_state_lifecycle import DStarSearchStats
        started=time.monotonic_ns();stats=DStarSearchStats();ids=None;fallback=None
        if (self.state.ready or initial) and not force:
            stats=self.state.compute_shortest_path(timeout_s=4. if initial else .5,
                max_expansions=80000 if initial else 20000)
            if not stats.timeout_triggered:ids=self.state.extract_path_ids()
            self.state.ready=not stats.timeout_triggered
            if ids is not None and not self.state.path_is_valid(ids):ids=None;self.state.ready=False
        else:self.state.ready=False
        if not self.state.ready:
            self.fallback_count+=1;ids,fallback=weighted_astar(self.state,timeout_s=4. if initial else 2.)
        self.state.initialized=True
        oracle_error=None
        if verify_oracle:
            oracle,oracle_stats=weighted_astar(self.state)
            if (ids is None)!=(oracle is None):raise AssertionError('V4 oracle reachability mismatch')
            oracle_error=0. if ids is None else abs(self.state.path_cost(ids)-oracle_stats.cost)
            if oracle_error>1e-8:raise AssertionError('V4 oracle cost mismatch')
        result=self._result(started_ns=started,path_ids=ids,
            failure='' if ids is not None else ('L2_TIMEOUT' if fallback and fallback.timeout_triggered else 'L2_UNVERIFIED_NO_PATH'),
            backend='v4_bounded_dstar' if self.state.ready else 'v4_bounded_astar_fallback',
            stats=stats,fallback=fallback,changed=changed,reused=not initial,oracle_cost_error=oracle_error)
        result.diagnostics.update(actual_cost_hash=self.state.cost_hash,weight=self.state.weight,
            path_cost=self.state.path_cost(ids) if ids is not None else None,
            dstar_expanded=stats.expanded_nodes,astar_expanded=0 if fallback is None else fallback.expanded_nodes,
            initial_backend_budgets_s=[4.,4.],timeout_is_physical_blockage=False)
        return result


class EdgeState(WeightedDStarState):
    def __init__(self, geometry, binding, potential, weight, directions, progress):
        self.xy_direction=np.asarray(directions,float)
        self.progress=np.asarray(progress,np.int16)
        self.cells=np.asarray([geometry.global_cell(i) for i in range(geometry.state_count)],np.int32)
        self._rows=self.cells[:,0].tolist();self._cols=self.cells[:,1].tolist()
        self._dx=self.xy_direction[:,0].tolist();self._dy=self.xy_direction[:,1].tolist()
        super().__init__(geometry,binding,potential,weight)

    def _edge_cost(self,a,b):
        dr=self._rows[b]-self._rows[a];dc=self._cols[b]-self._cols[a]
        if hasattr(self.geometry,'stage') and self.geometry.stage[b]<self.geometry.stage[a]:return math.inf
        # A lateral merge is allowed; motion against the selected road is not.
        tolerance=math.sin(math.radians(5))*math.hypot(dr,dc)
        if ((dc*self._dx[a]-dr*self._dy[a]) < -tolerance-1e-6 or
            (dc*self._dx[b]-dr*self._dy[b]) < -tolerance-1e-6):return math.inf
        return super()._edge_cost(a,b)


class FeedbackReferenceController(MultiresController):
    l3_mode='full'

    def __init__(self,world,topology,route,*,cache_root,views=None,dynamic=None,weight=2.,**kwargs):
        started=time.monotonic();cpu=time.process_time()
        self.feedback_zones=kwargs.pop('feedback_zones',[])
        self.world=world;self.topology=topology;self.topology_route=route;self.task=topology.task(route)
        self.root=Path(cache_root);self.root.mkdir(parents=True,exist_ok=True)
        self.views=views if views is not None else {};self.dynamic=dynamic;self.weight=weight;self.side='right'
        self.trajectory=None;self.trajectory_version=None;self.fallbacks=[];self.timing={'activation_attempts':[]}
        self.traffic=TrafficAudit(world,topology,route)
        self.lifecycle=SimpleNamespace(clear=lambda:None)
        self._soft_grid_cache={}
        self.global_drivable=np.isin(topology.subregions,[rid for rid,r in topology.regions.items()
            if r['kind'] in ('lane','junction_area') or rid in self.traffic.endpoint_parking])
        # Global no-parking-transit rule, independent of which edges L1 chose.
        # Current adapter enforces whole-body containment, conservatively. This
        # is reported separately from immutable physical occupancy, not hidden
        # as a new wall in the source map or as the L2 computational search ROI.
        physical=~topology.static_hard&self.global_drivable
        self.plan=L1Plan(world.safe,physical,world.map.world_to_cell(*route.start[:2]),world.map.world_to_cell(*route.goal[:2]),
            world.map.sha256,world.map.origin,world.map.resolution,topology.key,tuple(route.interface_ids),
            canonical_hash(world.vehicle),route.signature,{'v4_runtime_physical_mask_only':True,'not_l1_corridor':True})
        self.server_l3_mask=physical.copy();self.pending_l3_mask=physical;self._pending_l3_hash=''
        self.priority=np.isin(topology.subregions,self.task['priority_region_ids'])
        # Expansion can enter nearby junctions/gaps but not invent a different
        # road or use unrelated parking as transit. Choosing roads stays in L1.
        selected_roads={u['region_id'] for u in self.task['ordered_uses'] if u['kind']=='lane'}
        endpoint_parking=self.traffic.endpoint_parking
        allowed_ids=[rid for rid,r in topology.regions.items() if r['kind']=='junction_area' or rid in selected_roads or rid in endpoint_parking]
        self.search_legal=np.isin(topology.subregions,allowed_ids)
        gap=cv2.dilate(self.search_legal.astype(np.uint8),np.ones((7,7),np.uint8))>0
        self.search_legal |= gap&(topology.subregions==0)
        self._activate_reference()
        rc=np.asarray(self.l2.path_global);xy=self.view.world_points(rc[:,0],rc[:,1])
        self._set_reference(xy)
        self.timing.update(l2_total_ms=(time.monotonic()-started)*1000,l2_cpu_ms=(time.process_time()-cpu)*1000,
                          actual_resolution_m=self.view.map.resolution,requested_resolution_m=.15,
                          completion_mode=self.reference_mode,search_states=self.l2.geometry.state_count,
                          original_physical_obstacles_unchanged=True,global_parking_transit_rule=True)
        if dynamic and np.any(dynamic.mask):self.update_dynamic()

    def _view(self,factor,clearance):
        key=('v4',factor,clearance)
        if key in self.views:return self.views[key]
        base=self.views.get(factor)
        if base is None:base=GridView(self.world,factor);self.views[factor]=base
        import copy
        v=copy.copy(base)
        if clearance!='circle':
            if 'v4_static_distance' not in self.views:
                self.views['v4_static_distance']=ndimage.distance_transform_edt(~self.topology.static_hard)*self.world.map.resolution
            threshold=self.world.vehicle[1] if clearance=='inscribed' else 0.
            fine=(self.views['v4_static_distance']>=threshold)&~self.topology.static_hard
            v.safe=fine if factor==1 else aggregate(fine,factor,all_free=True)
        v.key=canonical_hash([base.key,'v4-reference-only',clearance])
        self.views[key]=v
        return v

    def _scope(self,view,radius=None,guide=None):
        selected=view.corridor(self.priority)
        legal=aggregate(self.search_legal,view.factor,all_free=True) if view.factor>1 else self.search_legal
        if radius is None:return legal
        seed=np.zeros(view.safe.shape,np.uint8)
        if guide is None:
            lines=[u['geometry'] for u in self.task['ordered_uses']]
            for role in ('start','goal'):
                a=self.task['endpoint_access'][role]
                lines.append([a['original_pose'][:2],a['xy']])
            for line in lines:
                rc=np.column_stack(view.cells(np.asarray(line,float)))
                cv2.polylines(seed,[rc[:,::-1].astype(np.int32)],False,1,1)
        else:
            rc=np.column_stack(view.cells(guide));cv2.polylines(seed,[rc[:,::-1].astype(np.int32)],False,1,1)
        scope=ndimage.distance_transform_edt(seed==0)*view.map.resolution<=radius
        return scope&legal

    def _search(self,factor,clearance,radius=None,guide=None,only_guide=False):
        started=time.monotonic();v=self._view(factor,clearance)
        mask=self._scope(v,radius,guide)
        record={'resolution':v.map.resolution,'clearance_filter':clearance,'scope_radius_m':radius,
                'scope_cells':int(mask.sum()),'not_vehicle_feasibility_proof':True}
        self.timing['activation_attempts'].append(record)
        try:
            if factor==1 and int(mask.sum())>120000:
                raise RuntimeError('FINE_STATE_BUDGET_BEFORE_ALLOCATION')
            roi=make_roi(v,mask,v.cell(self.topology_route.start),v.cell(self.topology_route.goal),self.task['signature'])
            labels,_=ndimage.label(roi.base_free)
            if not labels[roi.start_local] or labels[roi.start_local]!=labels[roi.goal_local]:raise RuntimeError('REFERENCE_GRID_DISCONNECTED')
            field_key=('v4field',v.key,self.task['signature'])
            if field_key not in self.views:
                # Limit route-conditioned cache residency: map views survive,
                # route fields are evicted rather than accumulating every query.
                for k in list(self.views):
                    if isinstance(k,tuple) and k and k[0]=='v4field':self.views.pop(k)
                self.views[field_key]=edge_field(self.world,v,self.topology,self.task)
            pref,directions,progress,_=self.views[field_key]
            def sample(points):
                fr,fc=self.world.cells(points);ok=(fr>=0)&(fc>=0)&(fr<self.world.map.height)&(fc<self.world.map.width)
                out=np.zeros(len(fr),np.int16);out[ok]=self.topology.subregions[fr[ok],fc[ok]]
                junction_ids=[rid for rid,x in self.topology.regions.items() if x['kind']=='junction_area']
                out[np.isin(out,junction_ids)]=-1
                return out
            # This index is built in L2 from edge occurrences. It does not
            # recreate L1 raster masks or constrain crossing to an outline gate.
            phases=[]
            for road,_ in self.traffic.roads:
                if phases:phases.append(-1)
                phases.append(road)
            for role in ('start','goal'):
                rid=self.task['endpoint_access'][role]['semantic_region_id']
                if rid in self.traffic.endpoint_parking:
                    phases.insert(0,rid) if role=='start' else phases.append(rid)
            contract=SimpleNamespace(sample=sample,regions=phases,
                plan={'signature':self.task['signature'],'mandatory_gates':False,'junction_identity_is_soft':True})
            geometry=ProgressGeometry(roi,v,contract)
            if factor==1 and geometry.state_count>120000:
                record['fine_state_budget']=120000
                raise RuntimeError('FINE_STATE_BUDGET_REQUEST_VEHICLE_FALLBACK')
            rc=np.asarray([geometry.global_cell(i) for i in range(geometry.state_count)],np.int32)
            potential=pref.potential[rc[:,0],rc[:,1]]
            direction=np.zeros((len(rc),2));order=np.full(len(rc),-1,np.int16)
            road_index=0
            for stage,rid in enumerate(geometry.regions):
                if rid<0 or self.topology.regions[rid]['kind']!='lane':continue
                selected=geometry.stage==stage
                direction[selected]=self.traffic.roads[road_index][1]
                refs=sample_uses({'ordered_uses':self.traffic.groups[road_index]})
                xy=v.world_points(rc[selected,0],rc[selected,1]);ix=cKDTree(refs[:,:2]).query(xy)[1]
                nearest=refs[ix];left=np.column_stack((-nearest[:,3],nearest[:,2]))
                offset=self.world.vehicle[1]+.05+math.sqrt(2)*self.world.map.resolution/2
                potential[selected]=np.minimum(1.,np.linalg.norm(xy-(nearest[:,:2]+offset*left),axis=1)/2.)
                road_index+=1
            # Occupied AND unknown cells count in this reference clearance.
            # Map.distance_m excludes unknown, so it is unsuitable here.
            if 'r2_physical_clearance' not in self.views:
                self.views['r2_physical_clearance']=ndimage.distance_transform_edt(~self.world.hard)*self.world.map.resolution
            xy_all=v.world_points(rc[:,0],rc[:,1]);fr,fc=self.world.cells(xy_all)
            clearance_values=self.views['r2_physical_clearance'][fr,fc]
            penalty=12.*np.maximum(0.,1.-clearance_values/.43)**2
            from .diagnostics import feedback_cost
            potential=potential+penalty+feedback_cost(xy_all,self.feedback_zones)
            record.update(clearance_penalty_max=float(penalty.max(initial=0)),feedback_zones=self.feedback_zones)
            binding=MutableStateBinding(geometry.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,'static',
                canonical_hash(['r2-finite-clearance-and-local-feedback-v1',digest(potential),digest(direction),digest(order),self.weight]))
            state=EdgeState(geometry,binding,potential,self.weight,direction,order)
            if only_guide:
                ids,stats=weighted_astar(state)
                if ids is None:raise RuntimeError('POINT_GUIDE_TIMEOUT' if stats.timeout_triggered else 'POINT_GUIDE_NO_PATH')
                planner=BoundedReferenceDStar(roi,geometry,state)
                from arena_3d_v1.l2_state_lifecycle import DStarSearchStats
                result=planner._result(started_ns=time.monotonic_ns(),path_ids=ids,failure='',
                    backend='v4_point_guide_pending_vehicle_validation',stats=DStarSearchStats(),fallback=stats)
                state.initialized=True;state.ready=False
                self._unverified_reference=(v,pref,planner,result)
                record.update(success=True,expanded_nodes=stats.expanded_nodes)
                return v.world_points(rc[ids,0],rc[ids,1])
            disk=VerifiedMutableStateCache(self.root/'edge_dstar')
            restored,telemetry=disk.restore(geometry,binding)
            if restored is not None:state.__dict__.update(restored.__dict__);state._refresh_views()
            planner=BoundedReferenceDStar(roi,geometry,state)
            result=planner.initialize()
            if not result.success:raise RuntimeError(result.failure_code or 'REFERENCE_NO_PATH')
            rc_path=np.asarray(planner.path_global);xy=v.world_points(rc_path[:,0],rc_path[:,1])
            d=np.diff(xy,axis=0);yaw=np.arctan2(d[:,1],d[:,0])
            traffic=self.traffic.audit(np.column_stack((xy,np.r_[yaw,yaw[-1]])))
            if not traffic['valid']:raise RuntimeError('REFERENCE_TRAFFIC:'+traffic['failure'])
            if restored is None:disk.save(state)
            self.view=v;self.preference=pref;self.l2=planner;self.initial_l2_result=result
            result.diagnostics['activation']={'state_cache_hit':telemetry.hit,'scope_cells':int(mask.sum()),
                'state_count':geometry.state_count,'clearance_filter':clearance,'binding_hash':binding.digest}
            record.update(success=True,states=geometry.state_count,cache_hit=telemetry.hit)
            return xy
        except (RuntimeError,ValueError) as e:
            record.update(success=False,failure=str(e))
            if any(s in str(e) for s in ('CORRUPT','BINDING','VERSION')):raise
            return None
        finally:record['wall_ms']=(time.monotonic()-started)*1000

    def _activate_reference(self):
        for factor,clearance,radius in [(3,'circle',3.),(3,'circle',None),(3,'inscribed',None)]:
            xy=self._search(factor,clearance,radius)
            if xy is not None:
                self.reference_mode='coarse_'+clearance;return
        guide=self._search(3,'point',None,only_guide=True)
        if guide is not None:
            for radius in (1.5,4.):
                xy=self._search(1,'inscribed',radius,guide)
                if xy is not None:
                    self.reference_mode='local_fine_reference_pending_full_vehicle_check';return
        # Bounded reference fallback, honestly reported. This cannot dispatch a
        # path: only canonical and independent L3 checks permit acceptance.
        xy=self._search(1,'inscribed',None)
        if xy is not None:self.reference_mode='full_fine_reference_pending_full_vehicle_check';return
        if guide is not None:
            # A point guide is evidence of where to request vehicle search, NOT
            # a vehicle-feasible L2 result. It can only reach L3, never dispatch.
            self.view,self.preference,self.l2,self.initial_l2_result=self._unverified_reference
            self.reference_mode='point_guide_unverified_full_vehicle_fallback'
            self.initial_l2_result.diagnostics['vehicle_feasibility']='unverified; mandatory original Smac and all final checks'
            return
        raise RuntimeError('V4_L2_NO_REFERENCE')

    def _set_reference(self,xy):
        r,c=self.world.cells(xy);self.reference_cells=np.column_stack((r,c))
        version=0 if self.dynamic is None else self.dynamic.version
        self.reference_plan=ReferencePlan(np.asarray(xy),self.view.map.resolution,self.task['signature'],self.world.key,
            self.preference.key,digest(self.plan.corridor_mask),str(version))
        self.route=reference_route(self.world,self.plan,xy)
        self._soft_grid_cache.clear()

    def _target_mask(self):
        return self.plan.corridor_mask if self.dynamic is None else self.plan.corridor_mask&~self.dynamic.mask

    def current_trajectory(self):
        with self.dynamic.lock if self.dynamic else nullcontext():
            if self.dynamic and self.trajectory_version!=self.dynamic.version:return None
            return None if self.trajectory is None else self.trajectory.copy()

    def prepare_grid(self,enabled=True,cap=140):
        started=time.monotonic();version=0 if self.dynamic is None else self.dynamic.version
        key=(version,digest(self.reference_cells),enabled,cap)
        if key in self._soft_grid_cache:return self._soft_grid_cache[key]
        shape=self.world.hard.shape
        # Global finite background avoids a zero-cost shortcut outside the ROI.
        # Distance transform is bounded-memory and every obstacle is retained.
        if enabled:
            seed=np.ones(shape,np.uint8)
            rc=self.reference_cells
            cv2.polylines(seed,[rc[:,::-1].astype(np.int32)],False,0,1)
            distance=cv2.distanceTransform(seed,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*self.world.map.resolution
            soft=(cap*np.minimum(1.,distance/2.)).astype(np.float32)
        else:soft=np.zeros(shape,np.float32)
        soft[~self.global_drivable]=200.
        occupancy=np.maximum(self.world.map.occupancy.astype(np.int16),internal_soft_to_occupancy(soft)).astype(np.int8)
        occupancy[self.world.map.occupancy<0]=-1;occupancy[self.world.hard|~self.plan.corridor_mask]=100
        static=occupancy_to_static_layer(occupancy)
        master=pinned_nav2_effective_master(static,resolution=self.world.map.resolution,inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
        policy=canonical_hash(['v4-reference-soft-v1',self.task['signature'],key,digest(occupancy)])
        grid=SemanticCostmap(static,occupancy,master,soft,self.world.hard|~self.plan.corridor_mask,np.ones(shape,bool),policy,
            self.world.semantic.semantic_map_hash,digest(self.reference_cells),enabled,
            {'adapter_ms':(time.monotonic()-started)*1000,'dynamic_version':version,'finite_nonnegative':True,
             'max_soft_cost':float(soft.max()),'hard_mask_sha256':digest(self.world.hard),
             'search_roi_encoded_as_obstacle':False,'global_parking_transit_rule':True,
             'adapter':'finite L2 reference; global parking exclusion is independent of selected edges'})
        self._soft_grid_cache={key:grid}
        return grid

    def sync_only(self,session):
        with self.dynamic.lock if self.dynamic else nullcontext():
            version=0 if self.dynamic is None else self.dynamic.version
            grid=self.prepare_grid();mask=self._target_mask().copy()
        session.set_semantic_costmap(grid);ack=session.update_local_mask(mask)
        if self.dynamic and self.dynamic.version!=version:ack.update(costmap_update_acknowledged=False,costmap_ack_status='MAP_VERSION_CHANGED_DURING_SYNC')
        return ack

    def _audit(self,query,p,mask,auditor,points):
        canonical=auditor.audit(query,points,mask)
        independent=self.world.audit(query,p,mask)
        traffic=self.traffic.audit(p)
        d=np.diff(p,axis=0);ds=np.linalg.norm(d[:,:2],axis=1)
        steer=np.arctan(.5*np.divide(wrap(d[:,2]),ds,out=np.zeros_like(ds),where=ds>1e-8))
        jump=float(np.max(np.abs(np.diff(steer)),initial=0)*180/math.pi)
        failures=list(independent.get('failures',[]))+traffic.get('failures',[])
        if not canonical.final_valid_success:failures.append(str(canonical.metrics.get('failure_code') or 'CANONICAL_AUDIT'))
        if jump>15+1e-6:failures.append('RAW_STEERING_JUMP')
        return canonical,independent,traffic,jump,sorted(set(failures))

    def plan_l3(self,query,session,auditor,spec,*,enabled=True,cap=140):
        if tuple(self.world.vehicle)!=tuple(session.bound_vehicle):raise ValueError('VEHICLE_VERSION_MISMATCH')
        if self.reference_plan.world_key!=self.world.key or self.reference_plan.fine_corridor_hash!=digest(self.plan.corridor_mask):raise ValueError('REFERENCE_BINDING_MISMATCH')
        if not self.initial_l2_result.success:
            return {'success':False,'failure_code':'DYNAMIC_L2_NO_PATH','map_sync':self.sync_only(session),'trajectory_invalidated':True}
        with self.dynamic.lock if self.dynamic else nullcontext():
            version=0 if self.dynamic is None else self.dynamic.version
            grid=self.prepare_grid(enabled,cap);self.pending_l3_mask=self._target_mask().copy()
        dirty=corridor_dirty_transition(self.server_l3_mask,self.pending_l3_mask);self._pending_l3_hash=dirty.target_hash
        step=PipelineStep(None,None,self.initial_l2_result,False,False,True,'',self.pending_l3_mask,dirty,self.task['signature'],{})
        session.set_semantic_costmap(grid)
        out=ProductionL3Adapter(self,auditor).plan(step,query,session,spec)
        out.update(map_version=version,reference_diagnostics=grid.diagnostics,multires_diagnostics=self.timing,l3_mode=self.l3_mode)
        result=out.get('result')
        if result and result.points:
            p=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points])
            ca,ind,traffic,jump,failures=self._audit(query,p,self.pending_l3_mask,auditor,result.points)
            out.update(geometry_success=bool(ca.final_valid_success and ind['valid']),independent_audit=ind,
                traffic_audit=traffic,constraint_audit={'max_raw_steering_jump_deg':jump,'steering_jump_valid':jump<=15+1e-6,'traffic':traffic})
            # Existing bounded curvature repair remains available, with the new
            # traffic audit instead of the deleted gate/corridor contract.
            if failures==['RAW_STEERING_JUMP']:
                from three_yd_v3.channel_repair import repair_path
                from arena_3d_v1.curvature_repair import truthful_points
                repaired,log=repair_path(self.world,self.traffic,query,p,self.pending_l3_mask,budget_s=5.)
                out['curvature_repair']=log
                if repaired is not None:
                    points=truthful_points(repaired,result,0,len(repaired)-1)
                    ca,ind,traffic,jump,failures=self._audit(query,repaired,self.pending_l3_mask,auditor,points)
                    result=replace(result,points=points,path_audit=ca,source='v4_smac_curvature_repaired')
                    p=repaired;out.update(result=result,independent_audit=ind,traffic_audit=traffic,
                        constraint_audit={'max_raw_steering_jump_deg':jump,'steering_jump_valid':jump<=15+1e-6,'traffic':traffic})
            out.update(success=not failures,failure_code=';'.join(failures),metrics=ca.metrics)
            if out['success']:self.trajectory=p;self.trajectory_version=version
        if self.dynamic and self.dynamic.version!=version:
            out.update(success=False,failure_code='MAP_VERSION_CHANGED_DURING_PLANNING')
            self.trajectory=None;self.trajectory_version=None
        return out

    def update_dynamic(self,verify=False):
        if self.dynamic is None:raise ValueError('NO_DYNAMIC_MANAGER')
        with self.dynamic.lock:
            started=time.monotonic();old=id(self.l2.state)
            self.trajectory=None;self.trajectory_version=None;self._soft_grid_cache.clear()
            # Preserve the validated conservative obstacle delta envelope.
            self.initial_l2_result=self.l2.update(self.dynamic.blocked_cells(self.view),verify_oracle=verify)
            if self.initial_l2_result.success:
                rc=np.asarray(self.l2.path_global);xy=self.view.world_points(rc[:,0],rc[:,1]);self._set_reference(xy)
            return {'success':bool(self.initial_l2_result.success),'map_version':self.dynamic.version,
                'state_identity_reused':id(self.l2.state)==old,'trajectory_invalidated':True,
                'wall_ms':(time.monotonic()-started)*1000,'source_hash':self.dynamic.content_hash}
