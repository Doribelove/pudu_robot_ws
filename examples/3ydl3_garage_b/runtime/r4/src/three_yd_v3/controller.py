"""D* Lite reference search and frozen Smac integration with truthful audits."""
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import time,math
from contextlib import nullcontext
import numpy as np
from scipy import ndimage
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.multires_pipeline import MultiresController
from arena_3d_v1.multires_grid import GridView,make_roi
from arena_3d_v1.recovery_pipeline import RecoveryController
from arena_3d_v1.semantic_reference import reference_costmap,preference_field
from arena_3d_v1.semantic_world import digest,densify,wrap
from arena_3d_v1.pipeline import ProductionL3Adapter,PipelineStep,corridor_dirty_transition

class V3Controller(MultiresController):
    # Preserve r2's tangent stabilization; main and fine fallback both use D*.
    _reference_route=RecoveryController._reference_route
    def __init__(self,*args,dynamic=None,**kwargs):
        self.dynamic=dynamic;self.trajectory=None;self.trajectory_version=None
        super().__init__(*args,**kwargs)
        self.timing['completion_mode']='full_fine_dstar_fallback' if self.view.factor==1 else 'coarse_dstar'
        if dynamic and np.any(dynamic.mask):self.update_dynamic()
    def _activate(self,factor,verify):
        if factor not in self.views:self.views[factor]=GridView(self.world,factor)
        v=self.views[factor];mask=v.corridor(self.plan.corridor_mask)
        roi=make_roi(v,mask,v.cell(self.topology_route.start),v.cell(self.topology_route.goal),self.plan.route_signature)
        components,_=ndimage.label(roi.base_free)
        if components[roi.start_local]==0 or components[roi.start_local]!=components[roi.goal_local]:
            self.timing.setdefault('connectivity_rejections',[]).append({'resolution':v.map.resolution,'reason':'conservative_reference_grid_disconnected','not_vehicle_impossibility_proof':True})
            raise RuntimeError('GUIDE_NO_PATH')
        return super()._activate(factor,verify)
    def _target_mask(self):
        return self.plan.corridor_mask if self.dynamic is None else self.plan.corridor_mask&~self.dynamic.mask
    def update_dynamic(self,verify=False):
        if self.dynamic is None:raise ValueError('NO_DYNAMIC_MANAGER')
        with self.dynamic.lock:
            t=time.monotonic();old_state=id(self.l2.state);old=self.reference_cells.copy()
            blocked=self.dynamic.blocked_cells(self.view)
            self.initial_l2_result=self.l2.update(blocked,verify_oracle=verify)
            self.trajectory=None;self.trajectory_version=None
            if self.initial_l2_result.success:
                rc=np.asarray(self.l2.path_global);xy=self.view.world_points(rc[:,0],rc[:,1]);r,c=self.world.cells(xy)
                self.reference_cells=np.column_stack((r,c))
                self.reference_plan=replace(self.reference_plan,world_xy=xy,occupancy_version=str(self.dynamic.version))
            return {'success':bool(self.initial_l2_result.success),'map_version':self.dynamic.version,'blocked_l2_cells':len(blocked),'state_identity_reused':id(self.l2.state)==old_state,'reference_changed':not np.array_equal(old,self.reference_cells),'trajectory_invalidated':True,'wall_ms':(time.monotonic()-t)*1000,'l2':{**self.initial_l2_result.diagnostics,'selected_backend':self.initial_l2_result.selected_backend,'response_ms':self.initial_l2_result.response_ms,'changed_cells':self.initial_l2_result.changed_cells},'source_hash':self.dynamic.content_hash}
    def current_trajectory(self):
        with self.dynamic.lock if self.dynamic else nullcontext():
            if self.dynamic is not None and self.trajectory_version!=self.dynamic.version:return None
            return None if self.trajectory is None else self.trajectory.copy()
    def prepare_grid(self,enabled=True,cap=140):
        grid=reference_costmap(self.world,self.route,self.reference_cells,enabled=enabled,cap=cap,side=self.side)
        version=0 if self.dynamic is None else self.dynamic.version
        grid.policy_hash=canonical_hash([grid.policy_hash,version,digest(self.world.hard)])
        grid.diagnostics['dynamic_version']=version
        return grid
    def sync_only(self,session):
        with self.dynamic.lock if self.dynamic else nullcontext():
            version=0 if self.dynamic is None else self.dynamic.version
            grid=self.prepare_grid();target=self._target_mask().copy()
        session.set_semantic_costmap(grid)
        result=session.update_local_mask(target)
        if self.dynamic is not None and self.dynamic.version!=version:result.update(costmap_update_acknowledged=False,costmap_ack_status='MAP_VERSION_CHANGED_DURING_SYNC')
        return result
    def plan_l3(self,query,session,auditor,spec,*,enabled=True,cap=140):
        if tuple(self.world.vehicle)!=tuple(session.bound_vehicle):raise ValueError('VEHICLE_VERSION_MISMATCH')
        if self.reference_plan.world_key!=self.world.key or self.reference_plan.fine_corridor_hash!=digest(self.plan.corridor_mask):raise ValueError('REFERENCE_BINDING_MISMATCH')
        if not self.initial_l2_result.success:
            ack=self.sync_only(session)
            return {'success':False,'failure_code':'DYNAMIC_L2_NO_PATH','map_sync':ack,'trajectory_invalidated':True}
        with self.dynamic.lock if self.dynamic else nullcontext():
            version=0 if self.dynamic is None else self.dynamic.version
            grid=self.prepare_grid(enabled,cap);self.pending_l3_mask=self._target_mask().copy()
        dirty=corridor_dirty_transition(self.server_l3_mask,self.pending_l3_mask)
        self._pending_l3_hash=dirty.target_hash
        step=PipelineStep(None,None,self.initial_l2_result,False,False,True,'',self.pending_l3_mask,dirty,self.plan.route_signature,{})
        session.set_semantic_costmap(grid);out=ProductionL3Adapter(self,auditor).plan(step,query,session,spec)
        out['reference_diagnostics']=grid.diagnostics;out['multires_diagnostics']=self.timing
        if out.get('result') and out['result'].points:
            p=np.asarray([[v['x'],v['y'],v['yaw']] for v in out['result'].points])
            a=self.world.audit(query,p,self._target_mask());out['independent_audit']=a
            out['geometry_success']=bool(out.get('success') and a['valid'])
            d=np.diff(p,axis=0);ds=np.linalg.norm(d[:,:2],axis=1);dyaw=wrap(d[:,2]);steer=np.arctan(.5*np.divide(dyaw,ds,out=np.zeros_like(ds),where=ds>1e-8))
            maxjump=float(np.max(np.abs(np.diff(steer)),initial=0)*180/math.pi)
            dense=densify(p);rr,cc=self.world.cells(dense);seen=self.topology.subregions[rr,cc];visits=list(map(int,seen[np.r_[True,seen[1:]!=seen[:-1]]]))
            index=0
            for rid in visits:
                if index<len(self.topology_route.region_ids) and rid==self.topology_route.region_ids[index]:index+=1
            ordered=index==len(self.topology_route.region_ids)
            out['constraint_audit']={'max_raw_steering_jump_deg':maxjump,'steering_jump_valid':maxjump<=15+1e-6,'ordered_regions_valid':ordered,'expected_regions':list(self.topology_route.region_ids),'visited_regions':visits}
            out['success']=bool(out['geometry_success'] and maxjump<=15+1e-6 and ordered)
            failures=[]
            if not a['valid']:failures+=a['failures']
            if maxjump>15+1e-6:failures.append('RAW_STEERING_JUMP')
            if not ordered:failures.append('ORDERED_REGIONS_MISSING')
            if failures:out['failure_code']=';'.join(failures)
            if out['success']:self.trajectory=p;self.trajectory_version=version
        if self.dynamic is not None and self.dynamic.version!=version:
            out.update(success=False,failure_code='MAP_VERSION_CHANGED_DURING_PLAN');self.trajectory=None;self.trajectory_version=None
        out['map_version']=version
        return out

class V3CandidateRecovery:
    def __init__(self,world,topology,cache_root,views,dynamic=None,max_attempts=4,controller_class=V3Controller):
        self.world=world;self.topology=topology;self.cache_root=Path(cache_root);self.views=views;self.dynamic=dynamic;self.max_attempts=max_attempts;self.controller_class=controller_class
    def plan(self,query,session,auditor,spec):
        started=time.monotonic();queue=[frozenset()];seen=set();attempts=[];controller=None;route=None;outcome={};searches=0
        while queue and len(attempts)<self.max_attempts and searches<32:
            constraints=queue.pop(0);searches+=1
            try:route=self.topology.plan(query,excluded_portal_transitions=constraints)
            except (RuntimeError,TimeoutError):continue
            signature=tuple(self.topology.nodes[k].portal for k in route.interface_ids)
            if signature in seen:continue
            seen.add(signature);controller=None;tick=time.monotonic()
            try:
                controller=self.controller_class(self.world,self.topology,route,cache_root=self.cache_root,views=self.views,dynamic=self.dynamic)
                outcome=controller.plan_l3(query,session,auditor,spec)
            except (RuntimeError,ValueError,TimeoutError) as error:
                if any(w in str(error).upper() for w in ['CORRUPT','BINDING','VERSION']):raise
                outcome={'success':False,'failure_code':str(error)}
            attempts.append({'regions':list(route.region_ids),'gates':list(signature),'success':outcome.get('success',False),'geometry_success':outcome.get('geometry_success',False),'failure':outcome.get('failure_code'),'wall_ms':(time.monotonic()-tick)*1000,'l1':route.diagnostics})
            # Keep geometry-valid trajectories for honest constraint reporting;
            # topology rerouting cannot repair a steering metadata defect.
            if outcome.get('success') or outcome.get('geometry_success'):break
            if any(w in str(outcome.get('failure_code','')).upper() for w in ['ACK','CONTENT','BINDING','CORRUPT','VERSION']):break
            sequence=(-1,*signature,-2)
            for a,b in zip(sequence,sequence[1:]):queue.append(constraints|{(a,b)})
            if controller:controller.lifecycle.clear()
        if not outcome:outcome={'success':False,'failure_code':'V3_CANDIDATES_EXHAUSTED'}
        outcome['candidate_recovery']={'attempts':attempts,'searches':searches,'wall_ms':(time.monotonic()-started)*1000,'persistent_topology_mutations':0}
        return controller,route,outcome
