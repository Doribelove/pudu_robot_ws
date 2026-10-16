"""Search only within bounded L2 neighborhoods. Never fall back to global L3."""
import time
from dataclasses import replace
import cv2
import numpy as np
from arena_3d_v1.semantic_world import digest
from .r5_controller import SoftEdgeController as StrictController
from arena_evaluation.semantic_costmap_composer import occupancy_to_static_layer
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master
from arena_evaluation.semantic_map import canonical_hash


def reference_neighborhood(world,xy,start,goal,radius):
    if not 0 < radius <= 6.4:raise ValueError('L3_RADIUS_OUTSIDE_BUDGET')
    seed=np.ones(world.hard.shape,np.uint8)
    rc=np.column_stack(world.cells(np.asarray(xy)))
    cv2.polylines(seed,[rc[:,::-1].astype(np.int32)],False,0,1)
    for pose in (start,goal):
        r,c=world.cells(np.asarray(pose)[None,:]);seed[int(r[0]),int(c[0])]=0
    distance=cv2.distanceTransform(seed,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*world.map.resolution
    return distance<=radius


class R5BoundedController(StrictController):
    radii=(.8,1.6,3.2,6.4)
    max_search_cells=400000
    max_map_fraction=.1

    def __init__(self,*args,**kwargs):
        self._search_mask=None
        from .approach_feedback import validate_connections
        world,topology,route=args[:3];dynamic=kwargs.get('dynamic');version=0 if dynamic is None else dynamic.version
        records=kwargs.pop('approach_connections',[])
        self.approach_connections=validate_connections(world,records,route_signature=route.signature,map_version=version)
        super().__init__(*args,**kwargs)

    def _search(self,factor,clearance,radius=None,guide=None,only_guide=False):
        if factor==1:
            view=self._view(factor,clearance);scope=self._scope(view,radius,guide)
            if int(scope.sum())>120000:
                self.timing['activation_attempts'].append({'resolution':view.map.resolution,'scope_cells':int(scope.sum()),'failure':'FINE_STATE_BUDGET_BEFORE_ALLOCATION','success':False})
                return None
        return super()._search(factor,clearance,radius,guide,only_guide)

    def _target_mask(self):
        if self._search_mask is None:raise RuntimeError('GLOBAL_L3_SEARCH_FORBIDDEN')
        return super()._target_mask() & self._search_mask

    def prepare_grid(self,enabled=True,cap=140):
        if not enabled:raise ValueError('L2_SOFT_COST_REQUIRED')
        base=super().prepare_grid(True,cap)
        mask=self._target_mask()
        key=canonical_hash([base.policy_hash,digest(mask),'r5-bounded-l3-v1'])
        cached=getattr(self,'_bounded_grid',None)
        if cached is not None and cached.policy_hash==key:return cached
        occupancy=base.occupancy_grid.copy();occupancy[~mask]=100
        static=occupancy_to_static_layer(occupancy)
        master=pinned_nav2_effective_master(static,resolution=self.world.map.resolution,
            inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
        # ACK must match the effective map AFTER search bounds, including
        # inflation from the temporary search boundary. Never weaken ACK.
        result=replace(base,internal_cost=static,occupancy_grid=occupancy,
            expected_master_cost=master,hard_semantic_mask=base.hard_semantic_mask|~mask,
            policy_hash=key,diagnostics={**base.diagnostics,'search_bound_hash':digest(mask),
                'bounded_search_cells':int(mask.sum()),'search_roi_encoded_as_obstacle':True,
                'source_obstacles_unchanged':True,'soft_cost_preserved':True})
        self._bounded_grid=result
        return result

    def plan_l3(self,query,session,auditor,spec,*,enabled=True,cap=140):
        if not enabled:raise ValueError('L2_SOFT_COST_REQUIRED')
        self.trajectory=None;self.trajectory_version=None
        started=time.monotonic();attempts=[];out={}
        base=super()._target_mask()
        for radius in self.radii:
            if hasattr(session,'budget'):session.budget.require()
            mask=reference_neighborhood(self.world,self.reference_plan.world_xy,query.start,query.goal,radius)&base
            count=int(mask.sum());limit=min(self.max_search_cells,int(mask.size*self.max_map_fraction))
            record={'radius_m':radius,'search_cells':count,'full_map_cells':int(mask.size),
                    'map_fraction':count/mask.size,'limit_cells':limit,'mask_hash':digest(mask),
                    'L2_soft_cost_enabled':True,'global_fallback':False}
            attempts.append(record)
            if count==0 or count>limit:
                record.update(called=False,failure='L3_SPATIAL_BUDGET');continue
            self._search_mask=mask
            if hasattr(session,'budget'):
                key=(query.start,query.goal,record['mask_hash'],digest(self.reference_cells))
                if key in session.budget.seen:
                    record.update(called=False,failure='IDENTICAL_INPUT_SKIPPED');continue
                session.budget.seen.add(key)
            tick=time.monotonic()
            out=super().plan_l3(query,session,auditor,spec,enabled=True,cap=cap)
            record.update(called=True,success=bool(out.get('success')),failure=out.get('failure_code',''),
                          native_reason=out.get('diagnostics',{}).get('termination_reason'),wall_ms=(time.monotonic()-tick)*1000)
            if out.get('success'):break
            self.trajectory=None;self.trajectory_version=None
            if any(x in str(out.get('failure_code','')).upper() for x in ('ACK','CONTENT','VERSION','BINDING','CORRUPT')):break
        if not out:out={'success':False,'failure_code':'L3_SPATIAL_BUDGET'}
        out['bounded_l3']={'attempts':attempts,'global_fallback':False,'max_radius_m':max(self.radii),
                           'wall_ms':(time.monotonic()-started)*1000,'storage':'full-size persistent costmap; search outside neighborhood is lethal',
                           'map_version':0 if self.dynamic is None else self.dynamic.version}
        out['approach_feedback']={'consumed':len(getattr(self,'approach_connections',[])),'connections':[],'global_reachability_not_inferred':True}
        if out.get('success'):
            from .approach_feedback import export_connections
            out['legacy_contract_success']=True
            out['approach_feedback']['connections']=export_connections(self.world,self.trajectory,
                route_signature=self.topology_route.signature,map_version=0 if self.dynamic is None else self.dynamic.version)
        return out

    def update_dynamic(self,*args,**kwargs):
        self.trajectory=None;self.trajectory_version=None
        result=super().update_dynamic(*args,**kwargs)
        self._search_mask=None
        return result
