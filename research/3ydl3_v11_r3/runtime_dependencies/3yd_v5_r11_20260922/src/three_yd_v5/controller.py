"""Pure XY D* reference and original fine-grid Smac inside variable bounds."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
import copy
import math
import time
import os
import cv2
import numpy as np
from scipy import ndimage
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.semantic_world import digest,densify
from arena_3d_v1.multires_grid import GridView,aggregate,make_roi
from arena_3d_v1.l2_state_lifecycle import CompactCorridorGeometry,MutableStateBinding
from arena_3d_v1.semantic_reference import RoutePreference
from arena_3d_v1.pipeline import L1Plan
from three_yd_v4.guidance import TrafficAudit
from three_yd_v4.r5_guidance import cross_section
from three_yd_v4.r5_controller import SoftEdgeController,BoundedReferenceDStar,EdgeState
from three_yd_v4.r5_bounded import R5BoundedController
from .geometry import shortcut_path,corners,variable_neighborhood
from .rectangle_bridge import rectangle_candidate_bits,RectanglePoseCertifier,certify_relaxed_path,first_component_break
from .r6_cache import RoiGrid,get_cache


def inflate_circle(fine_hard, radius, resolution):
    """Circle versus complete obstacle-cell squares; no extra safety buffer."""
    n=int(math.ceil(radius/resolution+.5))
    y,x=np.mgrid[-n:n+1,-n:n+1]
    distance=np.hypot(np.maximum(abs(x)-.5,0),np.maximum(abs(y)-.5,0))*resolution
    kernel=(distance<=radius+1e-9).astype(np.uint8)
    return cv2.dilate(np.asarray(fine_hard,np.uint8),kernel,borderType=cv2.BORDER_CONSTANT,borderValue=1)>0


def centerward_offset_guide(points,topology,road_context,corner_region_ids=(),
                            offset_m=.30,transition_m=.60,spacing_m=.15):
    """Build one continuous, full-route inward soft target.

    The selected L1 polyline is ordered in travel direction and represents the
    right side of that route.  A short centred chord supplies a stable local
    tangent; its left normal moves the target 0.30 m into the drivable side.
    Chords spanning a vertex create a bounded round-like join instead of a
    miter spike.  Junction polygons receive the same offset as road segments.
    ``transition_m`` remains in the signature for compatibility and is ignored.
    """
    vertices=np.asarray(points,float).reshape(-1,2)
    roads=[]
    for rid,direction in road_context:
        d=np.asarray(direction,float);norm=float(np.linalg.norm(d))
        if norm<=1e-9:continue
        d=d/norm;poly=np.asarray(topology.router.polygons[int(rid)],np.float32)
        roads.append((int(rid),d,poly))
    if len(vertices)<2 or not roads:raise ValueError('V5_R5_OFFSET_GUIDE_UNAVAILABLE')
    corner_polygons=[]
    for rid in dict.fromkeys(map(int,corner_region_ids)):
        region=topology.regions.get(rid)
        if region is None or region['kind']!='junction_area':continue
        feature=topology.router.features[region['name']]
        corner_polygons.append((rid,np.asarray(feature['coordinates'],np.float32)))
    segment_length=np.linalg.norm(np.diff(vertices,axis=0),axis=1)
    vertex_arc=np.r_[0.,np.cumsum(segment_length)]
    dense=[];dense_arc=[]
    for index,(a,b) in enumerate(zip(vertices,vertices[1:])):
        length=float(segment_length[index]);count=max(2,int(math.ceil(length/spacing_m))+1)
        u=np.linspace(0.,1.,count)
        segment=a+(b-a)*u[:,None]
        take=slice(None,-1) if index<len(vertices)-2 else slice(None)
        dense.extend(segment[take]);dense_arc.extend((vertex_arc[index]+length*u)[take])
    points=np.asarray(dense,float);dense_arc=np.asarray(dense_arc,float)
    # A 0.45 m chord on each side is long enough to blend a 90-degree corner,
    # yet local enough to leave ordinary straight sections exactly parallel.
    join_window_m=.45
    shifted=[];records=[]
    for index,(point,at_arc) in enumerate(zip(points,dense_arc)):
        before=int(np.searchsorted(dense_arc,at_arc-join_window_m,side='left'))
        after=int(np.searchsorted(dense_arc,at_arc+join_window_m,side='right')-1)
        before=max(0,min(before,index));after=min(len(points)-1,max(after,index))
        d=points[after]-points[before]
        norm=float(np.linalg.norm(d))
        if norm<=1e-9:
            d=points[min(index+1,len(points)-1)]-points[max(index-1,0)];norm=float(np.linalg.norm(d))
        if norm<=1e-9:raise ValueError('V5_R8_OFFSET_TANGENT_UNDEFINED')
        d=d/norm;left=np.array([-d[1],d[0]])
        target=point+float(offset_m)*left
        candidates=[]
        for rid,road_direction,poly in roads:
            signed=float(cv2.pointPolygonTest(poly,tuple(map(float,point)),True))
            candidates.append((abs(signed),rid,poly,signed,road_direction))
        _,rid,poly,signed,_=min(candidates,key=lambda x:x[0])
        corner_region_id=None;corner_signed_distance=-float('inf')
        for candidate_rid,candidate_poly in corner_polygons:
            value=float(cv2.pointPolygonTest(candidate_poly,tuple(map(float,point)),True))
            if value>corner_signed_distance:
                corner_region_id=candidate_rid;corner_signed_distance=value
        applied=float(offset_m)
        target_signed=float(cv2.pointPolygonTest(poly,tuple(map(float,target)),True))
        shifted.append(target)
        records.append({'road_id':rid,'direction':d.tolist(),'source_xy':point.tolist(),
                        'target_xy':target.tolist(),'requested_offset_m':float(offset_m),
                        'applied_offset_m':applied,'arc_m':float(at_arc),
                        'corner_region_id':corner_region_id,
                        'corner_polygon_signed_distance_m':None if not np.isfinite(corner_signed_distance) else float(corner_signed_distance),
                        'corner_exempt':False,'full_offset_in_junction':bool(corner_signed_distance>=0.),
                        'source_signed_distance_m':signed,'target_signed_distance_m':target_signed})
    shifted=np.asarray(shifted,float)
    keep=np.r_[True,np.linalg.norm(np.diff(shifted,axis=0),axis=1)>1e-7]
    return shifted[keep],[record for record,chosen in zip(records,keep) if chosen],{
        'corner_region_count':len(corner_polygons),
        'corner_region_ids':[rid for rid,_ in corner_polygons],
        'corner_definition':'selected L1 junction_area semantic polygons',
        'corner_offset_policy':'full 0.30 m; no exemption or taper',
        'transition_outside_polygon_m':0.,'round_join_window_m':join_window_m,
        'straight_offset_m':float(offset_m),'spacing_m':float(spacing_m)}


class V5Traffic(TrafficAudit):
    def __init__(self,world,topology,route):
        self.world=world;self.topology=topology;self.route=route
        context=topology.contexts[route.signature]
        self.roads=context['roads']
        self.task={'endpoint_access':context['endpoint_access']}
        self.endpoint_parking={a['semantic_region_id'] for a in context['endpoint_access'].values() if topology.regions[a['semantic_region_id']]['kind']=='parking_area'}

    def _wrong_direction_indices(self,poses):
        """Reproduce the base road-stage direction check with sample indices."""
        p=densify(np.asarray(poses,float),ds=.025)
        r,c=self.world.cells(p);labels=self.topology.subregions[r,c]
        lane_ids={rid for rid,x in self.topology.regions.items() if x['kind']=='lane'}
        bad=[];stage=0;previous=None;between=False
        for i,rid0 in enumerate(labels):
            rid=int(rid0)
            if rid not in lane_ids:
                between=True;continue
            if rid!=previous or between:
                returning=previous is not None;previous=rid
                if returning and between and stage+1<len(self.roads) and rid==self.roads[stage+1][0]:stage+=1
                elif stage<len(self.roads) and rid!=self.roads[stage][0]:
                    if stage+1<len(self.roads) and rid==self.roads[stage+1][0]:stage+=1
                between=False
            if stage>=len(self.roads) or rid!=self.roads[stage][0] or not i or labels[i-1]!=rid:continue
            delta=p[i,:2]-p[i-1,:2]
            if delta@self.roads[stage][1] < -math.sin(math.radians(5))*np.linalg.norm(delta)-1e-6:
                bad.append(i)
        return p,labels,np.asarray(bad,int)

    def audit(self,poses):
        result=super().audit(poses)
        if 'WRONG_TRAVEL_DIRECTION' not in result.get('failures',[]):return result
        p,labels,bad=self._wrong_direction_indices(poses)
        if not len(bad):return result
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
        changes=np.flatnonzero(np.r_[True,labels[1:]!=labels[:-1]]|np.r_[labels[:-1]!=labels[1:],True])
        transition_ok=np.asarray([np.min(abs(arc[changes]-arc[i]))<=.30 for i in bad])
        parking_ok=np.zeros(len(bad),bool)
        start=self.task['endpoint_access']['start']
        if start['semantic_region_id'] in self.endpoint_parking and start.get('allow_local_cross_road'):
            first=np.flatnonzero(labels==start['road_id'])
            if len(first):parking_ok=(arc[bad]-arc[first[0]]>=-1e-6)&(arc[bad]-arc[first[0]]<=1.50)
        allowed=transition_ok|parking_ok
        result['direction_violation_samples']=int(len(bad))
        result['local_transition_exemptions']=int(transition_ok.sum())
        result['confirmed_parking_cross_road_exemptions']=int(parking_ok.sum())
        result['direction_exemption_max_arc_m']=.30
        result['parking_cross_road_max_arc_m']=1.50
        if np.all(allowed):
            failures=[x for x in result['failures'] if x!='WRONG_TRAVEL_DIRECTION']
            result.update(valid=not failures,failure=';'.join(failures),failures=failures,
                wrong_direction_reclassified_as_local_transition=True)
        return result


class V5Controller(R5BoundedController):
    radii=(.8,1.2)
    rectangle_check_radius_m=1.2
    max_search_cells=400000
    max_map_fraction=.1
    r6_contract='V5-r9-measured-cache-v1'
    optimize=os.environ.get('THREEYD_R9_OPTIMIZE','0')=='1'

    def _bind_endpoint_parking_access(self,xy):
        """Limit endpoint parking to its reference access tube.

        Parking remains free for endpoint connection, but it cannot become a
        long-range transit shortcut after the route has entered its owned road.
        """
        samples=[]
        for a,b in zip(xy,xy[1:]):
            length=float(np.linalg.norm(b-a))
            u=np.linspace(0.,1.,max(2,int(math.ceil(length/self.world.map.resolution))+1))
            samples.extend(a+(b-a)*u[:,None])
        samples=np.asarray(samples,float).reshape(-1,2)
        sr,sc=self.world.cells(samples)
        valid=(sr>=0)&(sc>=0)&(sr<self.world.map.height)&(sc<self.world.map.width)
        labels=np.zeros(len(samples),self.topology.subregions.dtype)
        labels[valid]=self.topology.subregions[sr[valid],sc[valid]]
        access=np.zeros_like(self.world.hard,np.uint8);details={}
        for role in ('start','goal'):
            item=self.topology.contexts[self.topology_route.signature]['endpoint_access'][role]
            rid=int(item['semantic_region_id']);road=int(item['road_id'])
            if self.topology.regions[rid]['kind']!='parking_area':continue
            road_hits=np.flatnonzero(labels==road)
            if not len(road_hits):continue
            if role=='start':portion=samples[:road_hits[0]+1]
            else:portion=samples[road_hits[-1]:]
            pr,pc=self.world.cells(portion)
            keep=(pr>=0)&(pc>=0)&(pr<self.world.map.height)&(pc<self.world.map.width)
            pr,pc=pr[keep],pc[keep]
            if len(pr):cv2.polylines(access,[np.column_stack((pc,pr)).astype(np.int32)],False,1,1)
            details[role]={'parking_region_id':rid,'owned_road_id':road,'reference_samples':int(len(portion))}
        radius_cells=int(math.ceil(1.2/self.world.map.resolution))
        y,x=np.mgrid[-radius_cells:radius_cells+1,-radius_cells:radius_cells+1]
        access=cv2.dilate(access,((x*x+y*y)<=radius_cells*radius_cells).astype(np.uint8))>0
        parking_ids=list(self.traffic.endpoint_parking)
        access &= np.isin(self.topology.subregions,parking_ids)
        lane_junction_ids=[rid for rid,r in self.topology.regions.items() if r['kind'] in ('lane','junction_area')]
        selected_ids=[rid for rid in self.topology_route.region_ids if self.topology.regions[rid]['kind'] in ('lane','junction_area')]
        selected=np.isin(self.topology.subregions,selected_ids)|access
        margin_cells=int(math.ceil(.45/self.world.map.resolution))
        margin=cv2.dilate(selected.astype(np.uint8),np.ones((2*margin_cells+1,2*margin_cells+1),np.uint8))>0
        self.global_drivable=(np.isin(self.topology.subregions,lane_junction_ids)|access)&margin
        physical=~self.world.hard&self.global_drivable
        diagnostics={**dict(self.plan.diagnostics),'endpoint_parking_access_tube_m':1.2,
                     'whole_endpoint_parking_transit':False,'endpoint_access':details}
        self.plan=replace(self.plan,corridor_mask=physical,diagnostics=diagnostics)
        self.server_l3_mask=physical.copy();self.pending_l3_mask=physical;self._pending_l3_hash=''
        return int(access.sum())

    def __init__(self,world,topology,route,*,cache_root,views=None,dynamic=None,weight=2.,**kwargs):
        tick=time.monotonic();cpu=time.process_time()
        self.world=world;self.topology=topology;self.topology_route=route;self.task=topology.task(route)
        self.root=Path(cache_root);self.root.mkdir(parents=True,exist_ok=True)
        self.views=views if views is not None else {};self.dynamic=dynamic;self.weight=weight;self.side='right'
        self.trajectory=None;self.trajectory_version=None;self.fallbacks=[];self._soft_grid_cache={};self._search_mask=None
        self.timing={'activation_attempts':[]};self.lifecycle=SimpleNamespace(clear=self._clear_transients)
        self.traffic=V5Traffic(world,topology,route)
        if self.optimize and self._restore_static_route():
            # Validated cache identity is available before constructing the
            # large semantic masks that the saved route would replace.
            self._static_route_cache_hit=True
            self.priority=None;self.search_legal=None
            self.timing.update(l2_total_ms=(time.monotonic()-tick)*1000,
                l2_cpu_ms=(time.process_time()-cpu)*1000,actual_resolution_m=.15,
                requested_resolution_m=.15,completion_mode=self.reference_mode,
                search_states=self.l2.geometry.state_count,adapter_calls=[],
                l2_cache_source_response_ms=self.initial_l2_result.response_ms,
                l2_current_search_response_ms=0.,early_route_restore=True)
            return
        # L3 remains free to choose geometry inside the L1-selected semantic
        # regions, but may not shortcut through a different nearby road.  This
        # is a union mask, not an entry/exit gate contract: no waypoint or
        # outline edge is imposed on the vehicle planner.
        selected_physical={rid for rid in route.region_ids
            if topology.regions[rid]['kind'] in ('lane','junction_area','parking_area')}
        selected_physical|=self.traffic.endpoint_parking
        selected_mask=np.isin(topology.subregions,sorted(selected_physical))
        all_physical=np.isin(topology.subregions,[rid for rid,r in topology.regions.items()
            if r['kind'] in ('lane','junction_area') or rid in self.traffic.endpoint_parking])
        # Give the rectangular body and Nav2 inflation room at a semantic
        # boundary while still preventing multi-metre shortcuts through a
        # neighbouring, unselected road.
        margin_cells=int(math.ceil(.45/world.map.resolution))
        semantic_margin=cv2.dilate(selected_mask.astype(np.uint8),
            np.ones((2*margin_cells+1,2*margin_cells+1),np.uint8))>0
        self.global_drivable=all_physical&semantic_margin
        physical=~world.hard&self.global_drivable
        self.plan=L1Plan(world.safe,physical,world.map.world_to_cell(*route.start[:2]),world.map.world_to_cell(*route.goal[:2]),world.map.sha256,world.map.origin,world.map.resolution,topology.key,(),canonical_hash(world.vehicle),route.signature,{'v5_runtime_mask':True})
        self.server_l3_mask=physical.copy();self.pending_l3_mask=physical;self._pending_l3_hash=''
        self.priority=np.isin(topology.subregions,route.region_ids)
        self.search_legal=self.priority.copy()
        gap=cv2.dilate(self.search_legal.astype(np.uint8),np.ones((7,7),np.uint8))>0
        self.search_legal|=gap&(topology.subregions==0)
        before_restore=time.monotonic()
        self._static_route_cache_hit=self._restore_static_route()
        self.timing['route_lookup_actual_ms']=(time.monotonic()-before_restore)*1000
        if not self._static_route_cache_hit:
            self._activate_reference();self._store_static_route()
        self.timing['l2_cache_source_response_ms']=self.initial_l2_result.response_ms
        self.timing['l2_current_search_response_ms']=0. if self._static_route_cache_hit else self.initial_l2_result.response_ms
        self.timing['adapter_calls']=[]
        self.timing.update(l2_total_ms=(time.monotonic()-tick)*1000,l2_cpu_ms=(time.process_time()-cpu)*1000,actual_resolution_m=.15,requested_resolution_m=.15,completion_mode=self.reference_mode,search_states=self.l2.geometry.state_count)

    def _dynamic_identity(self):
        if self.dynamic is None:return (0,'static')
        return (int(self.dynamic.version),digest(self.dynamic.mask))

    def _route_cache_key(self):
        return (self.r6_contract,self.world.map.sha256,self.world.semantic.semantic_map_hash,
                canonical_hash(self.world.vehicle),self._dynamic_identity(),self.topology_route.signature,
                float(self.weight),tuple(self.topology_route.start),tuple(self.topology_route.goal))

    def _restore_static_route(self):
        cache=get_cache(self.views,'route-fields',max_entries=1)
        saved=cache.get(self._route_cache_key())
        if saved is None:return False
        for name,value in saved.items():setattr(self,name,value)
        # Each controller owns its publication cache and L3 query state even
        # when the immutable L2 route objects are reused.
        self._soft_grid_cache={};self._bounded_grid=None;self._search_mask=None
        self.server_l3_mask=self.plan.corridor_mask.copy();self.pending_l3_mask=self.server_l3_mask.copy()
        self.timing={'activation_attempts':[{'success':True,'cache_hit':True,
            'cache':'r6 route-conditioned L2 field and D* result'}],
            'r6_route_cache_hit':True,'r6_route_cache':cache.diagnostics()}
        return True

    def _store_static_route(self):
        names=('view','preference','l2','initial_l2_result','raw_reference_xy','reference_mode',
               'rectangle_bridge_certificates','rectangle_bridge_points','rectangle_bridge_cells',
               'l2_soft_target_xy','l2_guide_offset_records','l2_guide_offset_info','corner_records',
               'obstacle_repair_points','reference_cells','reference_plan','route','plan','global_drivable')
        cache=get_cache(self.views,'route-fields',max_entries=1)
        cache[self._route_cache_key()]={name:getattr(self,name) for name in names}
        self.timing.update(r6_route_cache_hit=False,r6_route_cache=cache.diagnostics())

    def _clear_transients(self):
        """Release controller-owned query arrays; shared bounded caches survive."""
        self.trajectory=None;self._search_mask=None;self._bounded_grid=None
        self.pending_l3_mask=None;self.server_l3_mask=None
        self._soft_grid_cache.clear()

    def prepare_grid(self,enabled=True,cap=140):
        tick=time.monotonic()
        grid=self._prepare_grid_cached(enabled,cap)
        elapsed=(time.monotonic()-tick)*1000
        hit=bool(self.timing.get('r6_costmap_cache_hit'))
        source_ms=float(grid.diagnostics.get('adapter_source_build_ms',grid.diagnostics.get('adapter_ms',0.)))
        self.timing['adapter_calls'].append({'actual_ms':elapsed,'cache_hit':hit,'source_build_ms':source_ms})
        self.timing['adapter_actual_ms']=sum(x['actual_ms'] for x in self.timing['adapter_calls'])
        return replace(grid,diagnostics={**grid.diagnostics,'adapter_ms':elapsed,
            'adapter_actual_ms':elapsed,'adapter_source_build_ms':source_ms,'adapter_cache_hit':hit})

    def _prepare_grid_cached(self,enabled=True,cap=140):
        """Reuse byte-identical bounded maps; ACK still validates every use."""
        if not enabled:raise ValueError('L2_SOFT_COST_REQUIRED')
        mask=self._target_mask()
        version=self._dynamic_identity()
        key=(self.r6_contract,self.world.map.sha256,self.topology_route.signature,
             digest(self.reference_cells),digest(mask),version,int(cap))
        cache=get_cache(self.views,'bounded-costmap',max_entries=1)
        grid=cache.get(key)
        if grid is not None:
            self._bounded_grid=grid
            self.timing['r6_costmap_cache_hit']=True
            self.timing['r6_costmap_cache']=cache.diagnostics()
            return grid
        if self.optimize:
            from .fast_costmap import compose_bounded
            grid=compose_bounded(self,mask,cap)
        else:grid=R5BoundedController.prepare_grid(self,True,cap)
        cache[key]=grid
        self.timing['r6_costmap_cache_hit']=False
        self.timing['r6_costmap_cache']=cache.diagnostics()
        return grid

    def _activate_reference(self):
        tick=time.monotonic();world=self.world
        radius=math.hypot(*world.vehicle[:2])
        key=('v5_r4_conservative_circle',radius,0 if self.dynamic is None else self.dynamic.version)
        if key not in self.views:
            view=copy.copy(self.views.get(3) or GridView(world,3))
            fine_hard=world.hard if self.dynamic is None else world.hard|self.dynamic.mask
            expanded=inflate_circle(fine_hard,radius,world.map.resolution)
            hard=~aggregate(~expanded,3,all_free=True,pad_value=False)
            penalty=np.zeros(hard.shape,np.float32)
            view.safe=~hard;view.hard=hard;view.r4_obstacle_penalty=penalty
            view.key=canonical_hash([view.key,'v5-r4-conservative-circle',radius,digest(view.safe)])
            self.views[key]=view
        circle_view=self.views[key]
        mask=aggregate(self.search_legal,3)
        record={'resolution':.15,'former_circle_radius_m':radius,'scope_cells':int(mask.sum()),'state_dimensions':['x','y'],
                'hard_obstacle_model':'original circumscribed circle; every represented fine cell clear',
                'static_obstacle_model':'confirmed V5-r3 conservative primary',
                'l2_vehicle_feasibility_is_final':False,
                'success':False,'rectangle_recovery_attempted':False}
        self.timing['activation_attempts'].append(record)
        try:
            shape=circle_view.safe.shape
            scope_cells=np.column_stack(np.nonzero(mask))
            if not len(scope_cells):raise RuntimeError('V5_L2_EMPTY_ROUTE_ROI')
            padding=3
            r0=max(0,int(scope_cells[:,0].min())-padding);r1=min(shape[0],int(scope_cells[:,0].max())+padding+1)
            c0=max(0,int(scope_cells[:,1].min())-padding);c1=min(shape[1],int(scope_cells[:,1].max())+padding+1)
            bbox=(r0,r1,c0,c1);local_shape=(r1-r0,c1-c0)
            seed=np.ones(local_shape,np.uint8)
            line=np.asarray(self.task['right_reference_polyline'],float)
            line,self.l2_guide_offset_records,self.l2_guide_offset_info=centerward_offset_guide(
                line,self.topology,self.topology.contexts[self.topology_route.signature]['unique_roads'],
                [rid for rid in self.topology_route.region_ids
                 if self.topology.regions[rid]['kind']=='junction_area'],.30)
            self.l2_soft_target_xy=line
            r,c=circle_view.cells(line)
            local_line=np.column_stack((c-c0,r-r0)).astype(np.int32)
            cv2.polylines(seed,[local_line],False,0,1)
            potential_local=np.minimum(cv2.distanceTransform(seed,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*.15/3.,4.)
            potential=RoiGrid(potential_local,bbox,shape,4.)
            search_potential=potential
            normalized_local=np.full(local_shape,np.nan,np.float32)
            eligible_local=np.zeros(local_shape,bool)
            direction_local=np.zeros((*local_shape,2),np.float32)
            rr,cc=np.indices(local_shape);all_rc=np.column_stack(((rr+r0).ravel(),(cc+c0).ravel()))
            xy=circle_view.world_points(all_rc[:,0],all_rc[:,1]);fr,fc=world.cells(xy)
            region_labels=np.zeros(len(fr),self.topology.subregions.dtype)
            inside=(fr>=0)&(fc>=0)&(fr<world.map.height)&(fc<world.map.width)
            region_labels[inside]=self.topology.subregions[fr[inside],fc[inside]]
            for rid,d in self.topology.contexts[self.topology_route.signature]['unique_roads']:
                selected=region_labels==rid
                direction_local.reshape(-1,2)[selected]=d
                # Evaluate right-side quality on all road cells, independently
                # of inflated search safety, as in the r5/r6 comparison metric.
                if np.any(selected):
                    norm,_,_=cross_section(self.topology.router.polygons[rid],d,xy[selected])
                    normalized_local.reshape(-1)[selected]=norm
                    eligible_local.reshape(-1)[selected]=np.isfinite(norm)
            direction=RoiGrid(direction_local,bbox,(*shape,2),0.)
            normalized=RoiGrid(normalized_local,bbox,shape,np.nan)
            eligible=RoiGrid(eligible_local,bbox,shape,False)
            ambiguous=RoiGrid(np.zeros(local_shape,bool),bbox,shape,False)
            pref=RoutePreference(potential,normalized,eligible,ambiguous,canonical_hash([self.task['signature'],'r8-full-route-centerward-offset-0.30m',bbox,digest(potential_local)]),
                {'source':'selected L1 right polyline shifted 0.30 m inward across roads and junctions','soft_not_hard':True,
                 'l1_geometry_unchanged':False,'centerward_offset_m':.30,
                 'storage':'route-local ROI','bbox_rc':list(bbox),'resident_field_bytes':int(potential_local.nbytes+normalized_local.nbytes+eligible_local.nbytes+direction_local.nbytes)})

            def solve(safe,tag,use_direction=True):
                view=copy.copy(circle_view);view.safe=np.asarray(safe,bool);view.hard=~view.safe
                view.key=canonical_hash([circle_view.key,tag,digest(view.safe)])
                roi=make_roi(view,mask,view.cell(self.topology_route.start),view.cell(self.topology_route.goal),self.task['signature'])
                geometry=CompactCorridorGeometry.build(roi)
                rc=np.asarray([geometry.global_cell(i) for i in range(geometry.state_count)],np.int32)
                binding=MutableStateBinding(geometry.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,'static',canonical_hash([bbox,digest(potential_local),self.weight,'V5_R8_FULL_ROUTE_OFFSET_030',tag]))
                state_direction=direction[rc[:,0],rc[:,1]] if use_direction else np.zeros((len(rc),2),np.float32)
                from .fast_state import PrecomputedEdgeState
                state_class=PrecomputedEdgeState if self.optimize else EdgeState
                state=state_class(geometry,binding,search_potential[rc[:,0],rc[:,1]],self.weight,state_direction,np.zeros(len(rc)))
                planner=BoundedReferenceDStar(roi,geometry,state);result=planner.initialize()
                if not result.success:raise RuntimeError(result.failure_code or 'V5_L2_NO_PATH')
                return view,geometry,planner,result,np.asarray(planner.path_global,int)

            circle_error=None
            try:
                labels,_=ndimage.label((circle_view.safe&mask)[r0:r1,c0:c1],np.ones((3,3),int))
                start=circle_view.cell(self.topology_route.start);goal=circle_view.cell(self.topology_route.goal)
                sl=(start[0]-r0,start[1]-c0);gl=(goal[0]-r0,goal[1]-c0)
                if min(*sl,*gl)<0 or sl[0]>=labels.shape[0] or gl[0]>=labels.shape[0] or sl[1]>=labels.shape[1] or gl[1]>=labels.shape[1] or not labels[sl] or labels[sl]!=labels[gl]:raise RuntimeError('V5_L2_CIRCLE_DISCONNECTED')
                view,geometry,planner,result,raw=solve(circle_view.safe,'circle')
                self.rectangle_bridge_certificates=[];self.rectangle_bridge_points=np.empty((0,2),float)
                self.rectangle_bridge_cells=np.empty((0,2),int)
                self.reference_mode='v5_r4_conservative_circle'
            except (ValueError,RuntimeError) as error:
                circle_error=str(error);record['circle_failure']=circle_error;record['rectangle_recovery_attempted']=True
                fine_hard=world.hard if self.dynamic is None else world.hard|self.dynamic.mask
                from .r10_options import enabled as r10_enabled
                if not r10_enabled('B'):
                    bits_key=('v5_r1_rectangle_bits',0 if self.dynamic is None else self.dynamic.version)
                    if bits_key not in self.views:
                        self.views[bits_key]=rectangle_candidate_bits(world,circle_view,fine_hard)
                    morphology_bits=np.asarray(self.views[bits_key],np.uint32)
                # The initial path is position-only and exists solely to locate
                # the circle-disconnected band.  It never becomes a published
                # reference and never opens an obstacle cell.
                flat_xy=circle_view.world_points(all_rc[:,0],all_rc[:,1]);pr,pc=world.cells(flat_xy)
                point_safe_values=np.zeros(len(pr),bool)
                pvalid=(pr>=0)&(pc>=0)&(pr<world.map.height)&(pc<world.map.width)
                point_safe_values[pvalid]=~fine_hard[pr[pvalid],pc[pvalid]]
                point_safe=np.zeros(shape,bool)
                point_safe[r0:r1,c0:c1]=point_safe_values.reshape(local_shape)
                point_safe&=mask
                if r10_enabled('B'):
                    from .r10_preflight import optimistic_reachable
                    optimistic=((circle_view.safe|point_safe)&mask)[r0:r1,c0:c1]
                    pre_start=circle_view.cell(self.topology_route.start)
                    pre_goal=circle_view.cell(self.topology_route.goal)
                    possible,evidence=optimistic_reachable(optimistic,direction_local,
                        (pre_start[0]-r0,pre_start[1]-c0),(pre_goal[0]-r0,pre_goal[1]-c0))
                    record['r10_preflight']=evidence
                    if not possible:
                        raise RuntimeError('V5_L2_RECTANGLE_BRIDGE_NOT_FOUND:R10_OPTIMISTIC_DIRECTION_DISCONNECTED')
                    bits_key=('v5_r1_rectangle_bits',0 if self.dynamic is None else self.dynamic.version)
                    if bits_key not in self.views:
                        self.views[bits_key]=rectangle_candidate_bits(world,circle_view,fine_hard)
                    morphology_bits=np.asarray(self.views[bits_key],np.uint32)
                try:
                    point_trial=solve(circle_view.safe|point_safe,'point-locator',use_direction=False)
                except (ValueError,RuntimeError) as candidate_error:
                    record['rectangle_failure']=str(candidate_error);point_trial=None
                solution=None;certificates=[]
                if point_trial is not None:
                    point_raw=point_trial[-1]
                    seed=np.zeros(shape,np.uint8)
                    relaxed=np.asarray([x for x in point_raw if not circle_view.safe[tuple(x)]],int).reshape(-1,2)
                    if len(relaxed):seed[relaxed[:,0],relaxed[:,1]]=1
                    if self.rectangle_check_radius_m is None:
                        band=point_safe&mask&~circle_view.safe
                    else:
                        band_cells=int(math.ceil(self.rectangle_check_radius_m/circle_view.map.resolution))
                        y,x=np.mgrid[-band_cells:band_cells+1,-band_cells:band_cells+1]
                        kernel=((x*x+y*y)<=band_cells*band_cells).astype(np.uint8)
                        band=(cv2.dilate(seed,kernel)>0)&point_safe&mask&~circle_view.safe
                    # All yaw bits are allowed into the exact checker.  The
                    # morphology bits only order likely cells first; they do
                    # not reject a pose at a discretized boundary.
                    bits=np.zeros(shape,np.uint32);bits[point_safe]=np.uint32((1<<24)-1)
                    exact_key=('v5_r1_exact_rectangle_pose_cache',0 if self.dynamic is None else self.dynamic.version)
                    exact_cache=self.views.setdefault(exact_key,{})
                    certifier=RectanglePoseCertifier(world,fine_hard,circle_view,bits,cache=exact_cache)
                    path_tangent={tuple(x['cell']):x['yaw_rad'] for x in certify_relaxed_path(point_raw,circle_view.safe,certifier)[0]}
                    candidates=np.column_stack(np.nonzero(band))
                    # Likely morphology passes first; this does not change the
                    # set eventually checked or the authoritative decision.
                    likely=morphology_bits[candidates[:,0],candidates[:,1]]!=0 if len(candidates) else np.empty(0,bool)
                    candidates=np.concatenate((candidates[likely],candidates[~likely])) if len(candidates) else candidates
                    opened=np.zeros(shape,bool);rejected=0
                    for cell in candidates:
                        preferred=path_tangent.get(tuple(map(int,cell)))
                        item=certifier.certify(cell,preferred)
                        if item is None:rejected+=1
                        else:
                            opened[tuple(cell)]=True;certificates.append(item)
                    record['rectangle_band_radius_m']=self.rectangle_check_radius_m
                    record['rectangle_band_candidates']=len(candidates)
                    record['rectangle_band_certified']=len(certificates)
                    record['rectangle_rejected_cells']=rejected
                    record['first_component_break']=first_component_break(point_raw,circle_view.safe|opened,circle_view,certificates)
                    try:solution=solve(circle_view.safe|opened,'rect-certified-band')
                    except (ValueError,RuntimeError) as candidate_error:record['rectangle_failure']=str(candidate_error)
                record['rectangle_iterations']=[{'attempt':1,'locator_nodes':0 if point_trial is None else len(point_trial[-1]),'band_candidates':record.get('rectangle_band_candidates',0),'certified_nodes':record.get('rectangle_band_certified',0),'rejected_nodes':record.get('rectangle_rejected_cells',0)}]
                if solution is not None:
                    final_raw=solution[-1]
                    final_relaxed=[tuple(x) for x in final_raw if not circle_view.safe[tuple(x)]]
                    certified={tuple(x['cell']):x for x in certificates}
                    if all(cell in certified for cell in final_relaxed):
                        view,geometry,planner,result,raw=solution
                        self.rectangle_bridge_certificates=[certified[x] for x in dict.fromkeys(final_relaxed)]
                        self.rectangle_bridge_cells=np.asarray(final_relaxed,int).reshape(-1,2)
                        self.rectangle_bridge_points=np.asarray([x['xy'] for x in self.rectangle_bridge_certificates],float).reshape(-1,2)
                        self.reference_mode='v5_r4_conservative_circle_plus_certified_rectangle'
                    else:solution=None
                if solution is None:raise RuntimeError('V5_L2_RECTANGLE_BRIDGE_NOT_FOUND:'+record.get('rectangle_failure','NO_CERTIFIED_CHAIN'))
                record.update(rectangle_certified_cells=len(self.rectangle_bridge_certificates),rectangle_bridge_length_upper_m=len(self.rectangle_bridge_certificates)*.15,rectangle_pose_is_static_not_continuous=True)

            self.view=view;self.l2=planner;self.preference=pref;self.initial_l2_result=result
            self.raw_reference_xy=view.world_points(raw[:,0],raw[:,1])
            # The obstacle rings steer D* away from clutter, but they must not
            # preserve grid-scale zigzags in the published reference. Shortcut
            # against the L1 guide cost while still enforcing every hard
            # obstacle cell and the selected road direction.
            nodes=shortcut_path(raw,view.safe&mask,potential,self.weight,direction)
            xy=view.world_points(nodes[:,0],nodes[:,1])
            # Exact query endpoints replace only the terminal coarse-center
            # positions. The original pose is still independently checked by L3.
            xy[0]=self.topology_route.start[:2];xy[-1]=self.topology_route.goal[:2]
            xy=xy[np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-7]]
            self.corner_records=corners(xy)
            # L2 intentionally has no heading state. A large mismatch between
            # the fixed query pose and its adjacent XY segment is therefore an
            # implicit L3 turn, and receives the same 1.2 m local treatment as
            # an interior corner. This does not alter the vehicle model.
            if len(xy)>=2:
                arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
                endpoint_specs=((0,xy[1]-xy[0],self.topology_route.start[2],0.),
                                (len(xy)-1,xy[-1]-xy[-2],self.topology_route.goal[2],float(arc[-1])))
                for index,delta,yaw,at_arc in endpoint_specs:
                    segment_yaw=math.atan2(float(delta[1]),float(delta[0]))
                    mismatch=abs(math.degrees(math.atan2(math.sin(segment_yaw-yaw),math.cos(segment_yaw-yaw))))
                    if mismatch>=30.:
                        self.corner_records.append({'index':index,'position':xy[index].tolist(),
                            'turn_angle_deg':mismatch,'arc_m':at_arc,'kind':'endpoint_heading_mismatch'})
                self.corner_records.sort(key=lambda x:x['arc_m'])
            # Static occupancy inside a manually marked drivable region is a
            # soft L2 hint. Mark the affected reference neighborhood so L3 can
            # use its existing 1.2 m local radius there and decide against the
            # untouched fine map. Dynamic obstacles are handled separately.
            samples=[]
            for a,b in zip(xy,xy[1:]):
                length=float(np.linalg.norm(b-a))
                u=np.linspace(0.,1.,max(2,int(math.ceil(length/world.map.resolution))+1))
                samples.extend(a+(b-a)*u[:,None])
            samples=np.asarray(samples,float).reshape(-1,2)
            sr,sc=world.cells(samples)
            valid=(sr>=0)&(sc>=0)&(sr<world.map.height)&(sc<world.map.width)
            near=np.zeros(len(samples),bool)
            near[valid]=world.map.distance_m[sr[valid],sc[valid]]<=radius+1e-9
            self.obstacle_repair_points=samples[near]
            record['endpoint_parking_access_cells']=self._bind_endpoint_parking_access(xy)
            self._set_reference(xy)
            record.update(success=True,states=geometry.state_count,raw_nodes=len(raw),straightened_nodes=len(xy),corner_count=len(self.corner_records),obstacle_repair_points=len(self.obstacle_repair_points),completion_mode=self.reference_mode,
                          r6_roi_bbox_rc=list(bbox),r6_roi_cells=int(np.prod(local_shape)),r6_full_grid_cells=int(np.prod(shape)),r6_roi_fraction=float(np.prod(local_shape)/np.prod(shape)),
                          r6_roi_field_bytes=int(potential_local.nbytes+normalized_local.nbytes+eligible_local.nbytes+direction_local.nbytes),
                          shortcut_cost_field='r8 selected L1 guide inward offset 0.30m across roads and junctions; hard obstacles and direction enforced',
                          l2_soft_target_offset_m=.30,l2_soft_target_side='toward_route_interior',l1_geometry_unchanged=False,
                          l2_soft_target_points=len(self.l2_soft_target_xy),
                          l2_soft_target_corner_policy='full 0.30m inside junction; round join; no exemption or taper',
                          l2_soft_target_corner_count=self.l2_guide_offset_info['corner_region_count'])
            result.diagnostics.update(v5=record,heading_state=False,progress_state=False)
        except (ValueError,RuntimeError) as e:
            record['failure']=str(e)
            from arena_3d_v1.semantic_world import json_write
            json_write(self.root/('failed_reference_'+self.task['signature'][:12]+'.json'),self.timing)
            raise
        finally:record['wall_ms']=(time.monotonic()-tick)*1000

    def _target_mask(self):
        base=SoftEdgeController._target_mask(self)
        if self._search_mask is None:raise RuntimeError('GLOBAL_L3_SEARCH_FORBIDDEN')
        return base&self._search_mask

    def _audit(self,*args,**kwargs):
        started=time.monotonic()
        try:return SoftEdgeController._audit(self,*args,**kwargs)
        finally:
            self.timing['strict_audit_calls']=self.timing.get('strict_audit_calls',0)+1
            self.timing['strict_audit_total_ms']=self.timing.get('strict_audit_total_ms',0.)+(time.monotonic()-started)*1000

    def plan_l3(self,query,session,auditor,spec,*,enabled=True,cap=140):
        if not enabled:raise ValueError('L2_SOFT_COST_REQUIRED')
        forced=np.concatenate((np.asarray(getattr(self,'rectangle_bridge_points',[]),float).reshape(-1,2),
                               np.asarray(getattr(self,'obstacle_repair_points',[]),float).reshape(-1,2)))
        mask,info=variable_neighborhood(self.world,self.reference_plan.world_xy,self.corner_records,
                                        forced_turning_points=forced)
        self._search_mask=mask
        count=int(self._target_mask().sum());limit=min(self.max_search_cells,int(mask.size*self.max_map_fraction))
        info.update(search_cells=count,limit_cells=limit,full_map_cells=mask.size,map_fraction=count/mask.size,L2_soft_cost_enabled=True)
        self.variable_scope_info=info
        if count>limit:
            return {'success':False,'failure_code':'L3_SPATIAL_BUDGET','bounded_l3':info}
        out=SoftEdgeController.plan_l3(self,query,session,auditor,spec,enabled=True,cap=cap)
        out['bounded_l3']=info
        out['v5']={'heading_state':False,'progress_state':False,'corners':self.corner_records,'l1_points':len(self.task['right_reference_polyline']),'l2_points':len(self.reference_plan.world_xy),'rectangle_bridge_cells':len(getattr(self,'rectangle_bridge_cells',[])),'rectangle_bridge_static_pose_only':True,
                   'l2_soft_target_offset_m':.30,'l2_soft_target_side':'toward_route_interior','l1_geometry_unchanged':False,
                   'l2_soft_target_corner_policy':'full 0.30m inside junction; round join; no exemption or taper'}
        out['legacy_contract_success']=bool(out.get('success'))
        # Smac may return a geometrically close candidate whose discrete
        # collision semantics differ from the continuous final audit. Repair
        # only short, explicitly located windows and accept only after the
        # unchanged full sweep, curvature and traffic audits all pass.
        result=out.get('result')
        repairable={'RAW_STEERING_JUMP','STATIC_FOOTPRINT_COLLISION','SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT'}
        failures=set(filter(None,str(out.get('failure_code','')).split(';')))
        traffic_ok=out.get('traffic_audit',{}).get('valid') is True
        if result and result.points and not out.get('success') and traffic_ok and failures and failures<=repairable:
            from three_yd_v4.local_l3 import risk_segments,repair_windows
            from three_yd_v4.geometry_repair import repair_segment
            from arena_3d_v1.curvature_repair import truthful_points
            p=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points],float)
            bad,_=risk_segments(self.world,p,self.pending_l3_mask)
            windows,coverage,total=repair_windows(p,bad,margin=1.)
            log={'attempted':True,'windows':windows,'risk_segments':int(len(bad)),
                 'repair_length_m':coverage,'path_length_m':total,'calls':[],'success':False}
            available=session.budget.remaining() if hasattr(session,'budget') else 8.
            deadline=time.monotonic()+min(8.,max(0.,available))
            repair_started=time.monotonic();repaired_all=True
            try:
                for a,b in reversed(windows):
                    patch,detail=repair_segment(self.world,p,a,b,self.pending_l3_mask,deadline)
                    call={'anchors':[a,b],'detail':detail,'expanded':False}
                    if patch is None and time.monotonic()<deadline:
                        lengths=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);arc=np.r_[0.,np.cumsum(lengths)]
                        ea=max(0,int(np.searchsorted(arc,arc[a]-.35))-1)
                        eb=min(len(p)-1,int(np.searchsorted(arc,arc[b]+.35)))
                        patch,detail=repair_segment(self.world,p,ea,eb,self.pending_l3_mask,deadline)
                        call.update(anchors=[ea,eb],detail=detail,expanded=True)
                        a,b=ea,eb
                    log['calls'].append(call)
                    if patch is None:
                        repaired_all=False;break
                    p=np.vstack((p[:a],patch,p[b+1:]))
            finally:
                if hasattr(session,'budget'):session.budget.vehicle_used+=time.monotonic()-repair_started
            if repaired_all:
                points=truthful_points(p,result,0,len(p)-1)
                ca,ind,traffic,jump,remaining=self._audit(query,p,self.pending_l3_mask,auditor,points)
                log.update(success=not remaining,remaining_failures=remaining,
                           final_max_steering_jump_deg=jump,final_points=len(p))
                if not remaining:
                    result=replace(result,points=points,path_audit=ca,source='v5_bounded_smac_local_geometry_repaired')
                    out.update(result=result,success=True,failure_code='',geometry_success=True,
                        independent_audit=ind,traffic_audit=traffic,
                        constraint_audit={'max_raw_steering_jump_deg':jump,'steering_jump_valid':True,'traffic':traffic},
                        metrics=ca.metrics,legacy_contract_success=True)
                    self.trajectory=p;self.trajectory_version=0 if self.dynamic is None else self.dynamic.version
            log['wall_ms']=(time.monotonic()-repair_started)*1000
            out['local_geometry_repair']=log
        return out

    def update_dynamic(self,*args,**kwargs):
        raise NotImplementedError('V5_R1_STATIC_VALIDATION_ONLY')
