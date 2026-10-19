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
from arena_3d_v1.multires_grid import GridView,aggregate
from .request_roi import make_roi
from arena_3d_v1.l2_state_lifecycle import CompactCorridorGeometry,MutableStateBinding
from arena_3d_v1.semantic_reference import RoutePreference
from arena_3d_v1.pipeline import L1Plan
from three_yd_v4.guidance import TrafficAudit
from three_yd_v4.r5_guidance import cross_section
from three_yd_v4.r5_controller import SoftEdgeController,BoundedReferenceDStar,EdgeState
from three_yd_v4.r5_bounded import R5BoundedController
from .curve_semantic_binding import fresh_hash as fresh_semantic_hash
from .geometry import shortcut_path,corners,variable_neighborhood
from .r2_options import enabled as r2_enabled
from .r4_options import enabled as r4_enabled
from .r4_fields import resident as field_resident
from .rectangle_bridge import rectangle_candidate_bits,RectanglePoseCertifier,certify_relaxed_path,first_component_break
from .r6_cache import RoiGrid,get_cache
from .firstplan import enabled as first_enabled,sparse_dilate,state_coordinates
from .firstcost import enabled as firstcost_enabled,neutral_adapter
from .v9_maps import content_digest as v9_digest, seal_grid
from .v9_options import enabled as v9_enabled
from .v9_r9_membership import semantic_membership
from .v9_r9_scope import aggregate_scope


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
    before_all=np.searchsorted(dense_arc,dense_arc-join_window_m,side='left')
    after_all=np.searchsorted(dense_arc,dense_arc+join_window_m,side='right')-1
    from .guide_tangents import tangents
    directions,targets=tangents(points,dense_arc,before_all,after_all,offset_m)
    for index,(point,at_arc) in enumerate(zip(points,dense_arc)):
        d=directions[index];target=targets[index]
        candidates=[];source_xy=tuple(map(float,point))
        for rid,road_direction,poly in roads:
            signed=float(cv2.pointPolygonTest(poly,source_xy,True))
            candidates.append((abs(signed),rid,poly,signed,road_direction))
        _,rid,poly,signed,_=min(candidates,key=lambda x:x[0])
        corner_region_id=None;corner_signed_distance=-float('inf')
        for candidate_rid,candidate_poly in corner_polygons:
            value=float(cv2.pointPolygonTest(candidate_poly,source_xy,True))
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
        # A labelled road can touch several painted junctions without the
        # vehicle crossing every rasterized junction sliver.  Treat adjacent
        # same-road, same-direction uses as one road stage.  A centre sample
        # may also land one 0.05 m cell into the adjacent lane polygon while
        # the bounded vehicle path follows the selected road.  Only repair
        # those short raster-boundary excursions; never reclassify a real
        # detour, a turn, a direction violation, or a parking transit.
        if any(code in result.get('failures',()) for code in
               ('ROAD_SEQUENCE_INCOMPLETE','ROAD_SEQUENCE_OR_OWNERSHIP')):
            robust=self._robust_road_sequence(poses)
            result['r7_road_sequence_diagnostic']=robust
            if robust['valid']:
                failures=[code for code in result['failures'] if code not in
                          ('ROAD_SEQUENCE_INCOMPLETE','ROAD_SEQUENCE_OR_OWNERSHIP')]
                result.update(valid=not failures,failure=';'.join(failures),failures=failures,
                              road_sequence_raster_boundary_confirmed=True)
        if 'WRONG_TRAVEL_DIRECTION' in result.get('failures',[]):
            p,labels,bad=self._wrong_direction_indices(poses)
            if len(bad):
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

    def _robust_road_sequence(self,poses):
        p=densify(np.asarray(poses,float),ds=.025)
        rr,cc=self.world.cells(p)
        labels=self.topology.subregions[rr,cc].copy()
        roads=[]
        for rid,direction in self.roads:
            if roads and roads[-1][0]==rid:
                if float(np.dot(roads[-1][1],direction))<.999:
                    return {'valid':False,'reason':'SAME_ROAD_DIRECTION_CHANGE'}
                continue
            roads.append((rid,direction))
        expected=[rid for rid,_ in roads]
        selected=set(expected)
        lane_ids={rid for rid,item in self.topology.regions.items() if item['kind']=='lane'}
        bad=np.flatnonzero(np.isin(labels,list(lane_ids-selected)))
        corrected=[]
        if len(bad):
            lengths=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
            arc=np.r_[0.,np.cumsum(lengths)]
            starts=bad[np.r_[True,np.diff(bad)>1]]
            ends=bad[np.r_[np.diff(bad)>1,True]]
            for a,b in zip(starts,ends):
                run_length=float(arc[min(int(b)+1,len(arc)-1)]-arc[int(a)])
                if run_length>.15+1e-9:
                    return {'valid':False,'reason':'NONSELECTED_ROAD_TRAVEL',
                            'road_id':int(labels[a]),'length_m':run_length}
                # A micro excursion is only a raster-boundary ambiguity if
                # each sampled cell is adjacent to a selected road cell.
                candidates=None
                for i in range(int(a),int(b)+1):
                    r,c=int(rr[i]),int(cc[i])
                    near=set(map(int,np.unique(self.topology.subregions[
                        max(0,r-1):min(self.topology.subregions.shape[0],r+2),
                        max(0,c-1):min(self.topology.subregions.shape[1],c+2)])))&selected
                    candidates=near if candidates is None else candidates&near
                    if not candidates:
                        return {'valid':False,'reason':'FOREIGN_ROAD_NOT_AT_SELECTED_BOUNDARY',
                                'road_id':int(labels[i])}
                neighbors=[int(labels[i]) for i in (int(a)-1,int(b)+1)
                           if 0<=i<len(labels) and int(labels[i]) in candidates]
                replacement=neighbors[0] if neighbors else min(candidates)
                labels[int(a):int(b)+1]=replacement
                corrected.append({'foreign_road_id':int(self.topology.subregions[rr[a],cc[a]]),
                                  'selected_road_id':replacement,'length_m':run_length})
        visited=[]
        for rid in labels:
            if int(rid) not in lane_ids:
                continue
            if not visited or visited[-1]!=int(rid):visited.append(int(rid))
        return {'valid':visited==expected,'expected_distinct_roads':expected,
                'visited_distinct_roads':visited,'raster_boundary_runs':corrected,
                'reason':'MATCH' if visited==expected else 'DISTINCT_ROAD_SEQUENCE_MISMATCH'}


class FreeSpaceTraffic:
    """Only semantic traffic rules are absent; geometry audits stay separate."""

    def __init__(self, world, topology, route):
        self.world = world
        self.topology = topology
        self.route = route
        self.endpoint_parking = set()

    def audit(self, poses):
        return {'valid': True, 'failures': [], 'mode': 'free_space',
                'semantic_traffic_rules_applied': False}


class V5Controller(R5BoundedController):
    radii=(.8,1.2)
    # r12 diagnostic knob. The frozen r11 value remains the default; only
    # explicit, bounded experiments may widen the disconnected L2 point band.
    rectangle_check_radius_m=float(os.environ.get('THREEYD_R12_RECTANGLE_BAND_M','1.2'))
    max_search_cells=400000
    max_map_fraction=.1
    r6_contract='V5-r9-measured-cache-v1'
    optimize=os.environ.get('THREEYD_R9_OPTIMIZE','0')=='1'

    def _limit_l2_start_parking_to_owned_road(self):
        """Prevent a starting parking region from bypassing its owned road.

        This is a small endpoint access envelope, not a vehicle trajectory or
        a hard right-side line. The L2 search remains free inside the envelope.
        """
        item=self.topology.contexts[self.topology_route.signature]['endpoint_access']['start']
        rid=int(item['semantic_region_id'])
        if self.topology.regions[rid]['kind']!='parking_area':return
        guide=np.asarray(self.task['right_reference_polyline'],float)
        if len(guide)==0:return
        if os.environ.get('THREEYD_R13_NORTH_EXIT_GUIDE')=='1':
            if len(guide)<2:return
            tangent=guide[1]-guide[0];tangent/=np.linalg.norm(tangent)
            contact_delta=guide[0]-np.asarray(self.topology_route.start[:2],float)
            mismatch=abs(math.atan2(math.sin(math.atan2(contact_delta[1],contact_delta[0])-self.topology_route.start[2]),
                                    math.cos(math.atan2(contact_delta[1],contact_delta[0])-self.topology_route.start[2])))
            if abs(tangent[1])<.95 or mismatch<math.radians(150):return
        start=np.asarray(self.topology_route.start[:2],float)
        contact=guide[0]
        if not np.all(np.isfinite(contact)):return
        rr,cc=self.world.cells(np.vstack((start,contact)))
        radius_cells=int(math.ceil(1.2/self.world.map.resolution))
        r0=max(0,int(rr.min())-radius_cells);r1=min(self.priority.shape[0],int(rr.max())+radius_cells+1)
        c0=max(0,int(cc.min())-radius_cells);c1=min(self.priority.shape[1],int(cc.max())+radius_cells+1)
        seed=np.zeros((r1-r0,c1-c0),np.uint8)
        cv2.line(seed,(int(cc[0]-c0),int(rr[0]-r0)),(int(cc[1]-c0),int(rr[1]-r0)),1,1)
        y,x=np.mgrid[-radius_cells:radius_cells+1,-radius_cells:radius_cells+1]
        tube=cv2.dilate(seed,((x*x+y*y)<=radius_cells*radius_cells).astype(np.uint8))>0
        parking=self.topology.subregions==rid
        removed=int(np.count_nonzero(self.priority&parking))
        self.priority[parking]=False
        local=self.priority[r0:r1,c0:c1]
        access=tube&(self.topology.subregions[r0:r1,c0:c1]==rid)
        local|=access
        self.timing['r13_start_access']={'parking_region_id':rid,'owned_road_id':int(item['road_id']),
            'reference_contact_xy':contact.tolist(),'tube_radius_m':1.2,
            'formerly_allowed_parking_cells':removed,'allowed_parking_cells':int(access.sum()),
            'roi_rc':[r0,r1,c0,c1]}

    def _limit_l2_goal_parking_to_owned_road(self):
        """Keep a parking goal attached to its annotated road in L2."""
        item=self.topology.contexts[self.topology_route.signature]['endpoint_access']['goal']
        rid=int(item['semantic_region_id'])
        if self.topology.regions[rid]['kind']!='parking_area':return
        guide=np.asarray(self.task['right_reference_polyline'],float)
        if len(guide)==0:return
        goal=np.asarray(self.topology_route.goal[:2],float)
        contact=guide[-1]
        rr,cc=self.world.cells(np.vstack((goal,contact)))
        radius_cells=int(math.ceil(1.2/self.world.map.resolution))
        r0=max(0,int(rr.min())-radius_cells);r1=min(self.priority.shape[0],int(rr.max())+radius_cells+1)
        c0=max(0,int(cc.min())-radius_cells);c1=min(self.priority.shape[1],int(cc.max())+radius_cells+1)
        seed=np.zeros((r1-r0,c1-c0),np.uint8)
        cv2.line(seed,(int(cc[0]-c0),int(rr[0]-r0)),(int(cc[1]-c0),int(rr[1]-r0)),1,1)
        y,x=np.mgrid[-radius_cells:radius_cells+1,-radius_cells:radius_cells+1]
        tube=cv2.dilate(seed,((x*x+y*y)<=radius_cells*radius_cells).astype(np.uint8))>0
        parking=self.topology.subregions==rid
        removed=int(np.count_nonzero(self.priority&parking))
        self.priority[parking]=False
        access=tube&(self.topology.subregions[r0:r1,c0:c1]==rid)
        self.priority[r0:r1,c0:c1]|=access
        self.timing['r13_goal_access']={'parking_region_id':rid,'owned_road_id':int(item['road_id']),
            'reference_contact_xy':contact.tolist(),'tube_radius_m':1.2,
            'formerly_allowed_parking_cells':removed,'allowed_parking_cells':int(access.sum()),
            'roi_rc':[r0,r1,c0,c1]}

    def _r13_parking_pose_reference(self,xy):
        """Replace only an opposite-heading parking exit with a local guide.

        An analytic curve is a soft L2 reference and must still pass the
        unchanged Smac search, steering audit, traffic audit, and exact sweep.
        """
        tick=time.monotonic()
        access=self.topology.contexts[self.topology_route.signature]['endpoint_access']['start']
        rid=int(access['semantic_region_id']);road=int(access['road_id'])
        if self.topology.regions[rid]['kind']!='parking_area' or len(xy)<3:return xy
        delta=xy[1]-xy[0]
        mismatch=abs(math.atan2(math.sin(math.atan2(delta[1],delta[0])-self.topology_route.start[2]),
                                math.cos(math.atan2(delta[1],delta[0])-self.topology_route.start[2])))
        if mismatch<math.pi/2:return xy
        from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
        from arena_3d_v1.semantic_world import SweptChecker,densify
        from arena_3d_v1.curvature_repair import fit_connector,violating_segments
        checker=SweptChecker(self.world.hard|~self.plan.corridor_mask,
                             self.world.map.resolution,self.world.map.origin,*self.world.vehicle[:2])
        start=tuple(map(float,self.topology_route.start))
        rr,cc=self.world.cells(xy)
        road_nodes=np.flatnonzero(self.topology.subregions[rr,cc]==road)[:3]
        trials=0;selected=None;fit_failures=0
        for j in road_nodes:
            if j>=len(xy)-1:continue
            heading=math.atan2(float(xy[j+1,1]-xy[j,1]),float(xy[j+1,0]-xy[j,0]))
            goal=(float(xy[j,0]),float(xy[j,1]),heading)
            for radius in (.5,.65,.8,.45,.41):
                for word,params in dubins_choices(start,goal,radius):
                    length=radius*sum(params)
                    if length>8.:continue
                    trials+=1
                    edge=dubins_edge_from_parameters(start,goal,radius,word,params)
                    if not checker.check(densify(np.vstack((start,edge.samples)))):continue
                    try:
                        fitted,fit_info=fit_connector(start,goal,word,params,radius,0.,0.,
                            tick+1.5,1/self.world.vehicle[2])
                    except (ValueError,TimeoutError):
                        fit_failures+=1;continue
                    if not checker.check(densify(fitted)) or len(violating_segments(fitted,1/self.world.vehicle[2])[0]):
                        fit_failures+=1;continue
                    poses=fitted
                    pr,pc=self.world.cells(poses[:,:2]);labels=self.topology.subregions[pr,pc]
                    sequence=labels[np.r_[True,labels[1:]!=labels[:-1]]]
                    if sequence.tolist()!=[rid,road]:continue
                    selected=(j,fitted,length,word,radius,fit_info)
                    break
                if selected is not None:break
            if selected is not None:break
        log={'attempted':True,'initial_heading_mismatch_deg':math.degrees(mismatch),
             'candidate_checks':trials,'fit_failures':fit_failures,'selection_ms':(time.monotonic()-tick)*1000,
             'selected':selected is not None}
        if selected is None:
            self.timing['r13_endpoint_connector']=log
            raise RuntimeError('V5_L2_ENDPOINT_CONNECTOR_NOT_FOUND')
        j,fitted,length,word,radius,fit_info=selected
        self.r13_prefix_connector=fitted
        new=np.vstack((fitted[:,:2],xy[j+1:]))
        log.update(anchor_index=int(j),word=word,radius_m=radius,
                   connector_length_m=float(length),reference_points_added=int(len(new)-len(xy)),
                   fit=fit_info)
        self.timing['r13_endpoint_connector']=log
        return new

    def _r13_north_exit_reference(self,xy):
        """Experimental parking exit through a verified gap before road travel.

        This only supplies a vehicle-feasible *reference*. The original bounded
        Smac and independent strict audit still decide whether a path exists.
        """
        started=time.monotonic()
        access=self.topology.contexts[self.topology_route.signature]['endpoint_access']['start']
        rid=int(access['semantic_region_id']);road=int(access['road_id'])
        if self.topology.regions[rid]['kind']!='parking_area' or len(xy)<4:return xy
        first_delta=xy[1]-xy[0]
        mismatch=abs(math.atan2(math.sin(math.atan2(first_delta[1],first_delta[0])-self.topology_route.start[2]),
                                math.cos(math.atan2(first_delta[1],first_delta[0])-self.topology_route.start[2])))
        if mismatch<math.radians(150):return xy
        guide=np.asarray(self.task['right_reference_polyline'],float)
        tangent=guide[1]-guide[0];tangent/=np.linalg.norm(tangent)
        if abs(tangent[1])<.95:return xy
        inward=np.array([-tangent[1],tangent[0]])
        start=np.asarray(self.topology_route.start,float)
        contact=guide[0]
        gap=np.array([*(contact+1.1*inward-3.1*tangent),math.atan2(inward[1],inward[0])])
        downstream=np.array([*(contact+1.4*inward-.63*tangent),math.atan2(tangent[1],tangent[0])])
        from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
        from arena_3d_v1.semantic_world import SweptChecker,densify
        from arena_3d_v1.curvature_repair import fit_connector,violating_segments
        checker=SweptChecker(self.world.hard,
                             self.world.map.resolution,self.world.map.origin,*self.world.vehicle[:2])
        def fit_leg(a,b):
            for radius in (.65,.8,.5,.45,.4):
                for word,params in dubins_choices(tuple(a),tuple(b),radius):
                    if radius*sum(params)>9.:continue
                    edge=dubins_edge_from_parameters(tuple(a),tuple(b),radius,word,params)
                    if not checker.check(densify(np.vstack((a,edge.samples)))):continue
                    try:
                        fitted,log=fit_connector(tuple(a),tuple(b),word,params,radius,0.,0.,
                            started+1.8,1/self.world.vehicle[2])
                    except (ValueError,TimeoutError):continue
                    if checker.check(densify(fitted)) and not len(violating_segments(fitted,1/self.world.vehicle[2])[0]):
                        return fitted,{'word':word,'radius_m':radius,'fit':log}
            return None,None
        first,first_log=fit_leg(start,gap)
        if first is None:return xy
        second,second_log=fit_leg(gap,downstream)
        if second is None:return xy
        prefix=np.vstack((first[:-1],second))
        selected=None
        for index in range(2,min(len(xy)-1,16)):
            target=xy[index];next_point=xy[index+1]
            if np.dot(target-downstream[:2],tangent)<10.:continue
            target_heading=math.atan2(float(next_point[1]-target[1]),float(next_point[0]-target[0]))
            if abs(math.atan2(math.sin(target_heading-downstream[2]),math.cos(target_heading-downstream[2])))>.20:
                continue
            travel_sign=math.copysign(1.,float(target[1]-downstream[1]))
            run=np.linspace(0.,1.,21)
            slope=float(math.cos(downstream[2])/abs(math.sin(downstream[2])))
            ease=np.column_stack((downstream[0]+slope*run*(1-run)**2,
                                  downstream[1]+travel_sign*run))
            mid=np.array([downstream[0],target[1]-travel_sign*2.])
            if (mid[1]-ease[-1,1])*travel_sign<=0:continue
            count=max(2,int(math.ceil(np.linalg.norm(mid-ease[-1])/.05))+1)
            line=ease[-1]+(mid-ease[-1])*np.linspace(0.,1.,count)[:,None]
            count=max(2,int(math.ceil(np.linalg.norm(target-mid)/.05))+1)
            u=np.linspace(0.,1.,count)
            end=mid+(target-mid)*u[:,None]
            end[:,0]=mid[0]+(target[0]-mid[0])*(10*u**3-15*u**4+6*u**5)
            position=np.vstack((ease[:-1],line[:-1],end))
            dp=np.gradient(position,axis=0)
            sweep=np.column_stack((position,np.arctan2(dp[:,1],dp[:,0])))
            candidate=np.vstack((prefix[:-1],sweep))
            if not checker.check(densify(candidate)) or len(violating_segments(candidate,1/self.world.vehicle[2])[0]):continue
            rr,cc=self.world.cells(candidate[:,:2]);labels=self.topology.subregions[rr,cc]
            sequence=labels[np.r_[True,labels[1:]!=labels[:-1]]].tolist()
            if sequence[0]!=rid or road not in sequence or any(v not in (rid,road,0) for v in sequence):continue
            selected=(index,candidate,sequence)
            break
        if selected is None:return xy
        index,candidate,sequence=selected
        # The manually drawn parking/road polygons leave a one-cell unlabeled
        # seam at this verified transition. Admit only its local footprint,
        # rather than opening all unclassified map cells to L3.
        sr,sc=self.world.cells(candidate[:,:2]);seam_samples=self.topology.subregions[sr,sc]==0
        seam=np.zeros_like(self.world.hard,np.uint8)
        seam[sr[seam_samples],sc[seam_samples]]=1
        radius_cells=int(math.ceil(.40/self.world.map.resolution))
        yy,xx=np.mgrid[-radius_cells:radius_cells+1,-radius_cells:radius_cells+1]
        self.r13_semantic_seam=(cv2.dilate(seam,((xx*xx+yy*yy)<=radius_cells*radius_cells).astype(np.uint8))>0)&(self.topology.subregions==0)
        self.r13_semantic_road_id=road
        self.traffic.topology=copy.copy(self.topology)
        self.traffic.topology.subregions=self.topology.subregions.copy()
        self.traffic.topology.subregions[self.r13_semantic_seam]=road
        self.timing['r13_north_exit']={'selected':True,'first':first_log,'second':second_log,
            'join_index':int(index),'semantic_sequence':sequence,
            'local_length_m':float(np.linalg.norm(np.diff(candidate[:,:2],axis=0),axis=1).sum()),
            'seam_allowed_cells':int(self.r13_semantic_seam.sum()),
            'traffic_semantic_patch_cells':int(self.r13_semantic_seam.sum()),
            'wall_ms':(time.monotonic()-started)*1000}
        return np.vstack((candidate[:,:2],xy[index+1:]))

    def _r13_goal_pose_reference(self,xy):
        """Replace only a mismatched parking approach by a certified local guide."""
        started=time.monotonic()
        access=self.topology.contexts[self.topology_route.signature]['endpoint_access']['goal']
        rid=int(access['semantic_region_id']);road=int(access['road_id'])
        if self.topology.regions[rid]['kind']!='parking_area' or len(xy)<4:return xy
        delta=xy[-1]-xy[-2]
        yaw=self.topology_route.goal[2]
        mismatch=abs(math.atan2(math.sin(math.atan2(delta[1],delta[0])-yaw),
                                math.cos(math.atan2(delta[1],delta[0])-yaw)))
        if mismatch<math.radians(60):return xy
        from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
        from arena_3d_v1.semantic_world import SweptChecker,densify
        from arena_3d_v1.curvature_repair import fit_connector,violating_segments
        checker=SweptChecker(self.world.hard|~self.plan.corridor_mask,
                             self.world.map.resolution,self.world.map.origin,*self.world.vehicle[:2])
        rr,cc=self.world.cells(xy);labels=self.topology.subregions[rr,cc]
        goal=tuple(map(float,self.topology_route.goal))
        for index in range(len(xy)-3,max(-1,len(xy)-12),-1):
            if labels[index]!=road or labels[index+1]!=road:continue
            a,b=xy[index],xy[index+1]
            length=float(np.linalg.norm(b-a))
            if length<.6:continue
            direction=(b-a)/length
            heading=math.atan2(float(direction[1]),float(direction[0]))
            for back_m in (.5,.8,1.2):
                if back_m>=length:continue
                anchor=b-back_m*direction
                start=(float(anchor[0]),float(anchor[1]),heading)
                if not checker.check(np.asarray([start])):continue
                for radius in (.5,.65,.8,.45,.4):
                    for word,params in dubins_choices(start,goal,radius):
                        route_length=radius*sum(params)
                        if route_length>9.:continue
                        edge=dubins_edge_from_parameters(start,goal,radius,word,params)
                        if not checker.check(densify(np.vstack((start,edge.samples)))):continue
                        try:
                            fitted,fit_log=fit_connector(start,goal,word,params,radius,0.,0.,
                                started+1.2,1/self.world.vehicle[2])
                        except (ValueError,TimeoutError):continue
                        if not checker.check(densify(fitted)) or len(violating_segments(fitted,1/self.world.vehicle[2])[0]):continue
                        fr,fc=self.world.cells(fitted[:,:2]);f_labels=self.topology.subregions[fr,fc]
                        sequence=f_labels[np.r_[True,f_labels[1:]!=f_labels[:-1]]].tolist()
                        if sequence!=[road,rid]:continue
                        self.timing['r13_goal_pose']={'selected':True,'anchor_segment_index':index,
                            'anchor_pose':list(start),'word':word,'radius_m':radius,
                            'connector_length_m':route_length,'semantic_sequence':sequence,
                            'wall_ms':(time.monotonic()-started)*1000,'fit':fit_log}
                        return np.vstack((xy[:index+1],fitted[:,:2]))
        self.timing['r13_goal_pose']={'selected':False,'wall_ms':(time.monotonic()-started)*1000}
        return xy

    def _bind_endpoint_parking_access(self,xy):
        """Limit endpoint parking to its reference access tube.

        Parking remains free for endpoint connection, but it cannot become a
        long-range transit shortcut after the route has entered its owned road.
        """
        from .tail_access import reuse
        if reuse(self):return 0
        if firstcost_enabled('A') and neutral_adapter(self):
            # Frozen map-only adapter labels are exactly (~hard).astype(int16).
            # Its only region is a neutral junction; no parking or semantics.
            # Original selected-free dilation contains every selected-free cell,
            # so (free & dilate(free)) == free. Keep L3's separate scope intact.
            started=time.monotonic()
            from .neutral_masks import physical_masks
            self.global_drivable,physical=physical_masks(self.world.hard,self.topology.subregions)
            diagnostics={**dict(self.plan.diagnostics),'endpoint_parking_access_tube_m':1.2,
                         'whole_endpoint_parking_transit':False,'endpoint_access':{}}
            self.plan=replace(self.plan,corridor_mask=physical,diagnostics=diagnostics)
            self.server_l3_mask=physical;self.pending_l3_mask=physical;self._pending_l3_hash=''
            self.timing['r11_neutral_access_ms']=(time.monotonic()-started)*1000
            return 0
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
        access=(sparse_dilate if first_enabled("A") else cv2.dilate)(access,((x*x+y*y)<=radius_cells*radius_cells).astype(np.uint8))>0
        parking_ids=list(self.traffic.endpoint_parking)
        access &= semantic_membership(self.topology.subregions,parking_ids)
        lane_junction_ids=[rid for rid,r in self.topology.regions.items() if r['kind'] in ('lane','junction_area')]
        selected_ids=[rid for rid in self.topology_route.region_ids if self.topology.regions[rid]['kind'] in ('lane','junction_area')]
        selected=semantic_membership(self.topology.subregions,selected_ids)|access
        margin_cells=int(math.ceil(.45/self.world.map.resolution))
        margin=cv2.dilate(selected.astype(np.uint8),np.ones((2*margin_cells+1,2*margin_cells+1),np.uint8))>0
        seam=getattr(self,'r13_semantic_seam',None)
        self.global_drivable=(semantic_membership(self.topology.subregions,lane_junction_ids)|access|(seam if seam is not None else False))&margin
        physical=~self.world.hard&self.global_drivable
        from .v10_storage import retained_mask
        physical=retained_mask(physical)
        diagnostics={**dict(self.plan.diagnostics),'endpoint_parking_access_tube_m':1.2,
                     'whole_endpoint_parking_transit':False,'endpoint_access':details}
        self.plan=replace(self.plan,corridor_mask=physical,diagnostics=diagnostics)
        self.server_l3_mask=physical;self.pending_l3_mask=physical;self._pending_l3_hash=''
        return int(access.sum())

    def __init__(self,world,topology,route,*,cache_root,views=None,dynamic=None,weight=2.,**kwargs):
        tick=time.monotonic();cpu=time.process_time()
        from .tail_options import enabled as tail_enabled
        self.r4_semantic_body=bool(kwargs.get('r4_semantic_body',False) or tail_enabled('C'))
        self.world=world;self.topology=topology;self.topology_route=route;self.task=topology.task(route)
        self.root=Path(cache_root);self.root.mkdir(parents=True,exist_ok=True)
        self.views=views if views is not None else {};self.dynamic=dynamic;self.weight=weight;self.side='right'
        self.trajectory=None;self.trajectory_version=None;self.fallbacks=[];self._soft_grid_cache={};self._search_mask=None
        self.r13_prefix_connector=None
        self.r13_remainder_mask=None
        self.r13_semantic_seam=None
        self.r13_semantic_road_id=None
        from .v10_lifecycle import lifecycle
        self.timing={'activation_attempts':[]};self.lifecycle=lifecycle(self._clear_transients)
        self.generic_mode=getattr(topology,'mode',None)=='free_space'
        self.traffic=FreeSpaceTraffic(world,topology,route) if self.generic_mode else V5Traffic(world,topology,route)
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
        mask_started=time.monotonic()
        selected_physical={rid for rid in route.region_ids
            if topology.regions[rid]['kind'] in ('lane','junction_area','parking_area')}
        selected_physical|=self.traffic.endpoint_parking
        selected_mask=(topology.scope_mask(route) if self.generic_mode else
                       semantic_membership(topology.subregions,sorted(selected_physical)))
        all_physical=((topology.subregions==1) if firstcost_enabled('A') and neutral_adapter(self) else
            semantic_membership(topology.subregions,[rid for rid,r in topology.regions.items()
            if r['kind'] in ('lane','junction_area') or rid in self.traffic.endpoint_parking]))
        # Give the rectangular body and Nav2 inflation room at a semantic
        # boundary while still preventing multi-metre shortcuts through a
        # neighbouring, unselected road.
        margin_cells=int(math.ceil(.45/world.map.resolution))
        from .recovery_options import enabled as recovery_enabled
        if recovery_enabled('U'):
            from .recovery_masks import initial_masks
            self.global_drivable,physical=initial_masks(world.hard,selected_mask,all_physical,margin_cells)
        else:
            semantic_margin=cv2.dilate(selected_mask.astype(np.uint8),
                np.ones((2*margin_cells+1,2*margin_cells+1),np.uint8))>0
            self.global_drivable=all_physical&semantic_margin
            physical=~world.hard&self.global_drivable
            del semantic_margin
        from .request_cost import enabled as request_cost_enabled
        if request_cost_enabled('T'):
            # Drop the temporary owner after producing the retained drivable mask.
            # U may consume its buffer; otherwise it is no longer needed.
            del all_physical
        from .v10_storage import retained_mask
        physical=retained_mask(physical)
        self.plan=L1Plan(world.safe,physical,world.map.world_to_cell(*route.start[:2]),world.map.world_to_cell(*route.goal[:2]),world.map.sha256,world.map.origin,world.map.resolution,topology.key,(),canonical_hash(world.vehicle),route.signature,{'v5_runtime_mask':True})
        self.server_l3_mask=physical;self.pending_l3_mask=physical;self._pending_l3_hash=''
        if recovery_enabled('U') and not self.generic_mode and set(route.region_ids)==selected_physical:
            # The fresh selected mask has exactly the original priority IDs.
            # Margin construction has consumed it; no cache or other owner exists.
            self.priority=selected_mask
        else:
            consume_selected=(self.generic_mode and selected_mask.flags.owndata
                              and 'C' in os.environ.get('THREEYD_V10_R7_MODE','control'))
            self.priority=(selected_mask if consume_selected else selected_mask.copy() if self.generic_mode else
                           semantic_membership(topology.subregions,route.region_ids))
        if request_cost_enabled('T'):del selected_mask,physical
        if os.environ.get('THREEYD_R13_START_ACCESS_TUBE')=='1':
            self._limit_l2_start_parking_to_owned_road()
        from .tail_options import enabled as tail_enabled
        if os.environ.get('THREEYD_R13_GOAL_ACCESS_TUBE')=='1' or self.r4_semantic_body:
            self._limit_l2_goal_parking_to_owned_road()
        from .v10_storage import initial_scope as initial_search_scope
        self.priority,self.search_legal=initial_search_scope(self.priority,self.generic_mode)
        if not self.generic_mode:
            if recovery_enabled('U'):
                from .recovery_masks import dilated_boolean
                gap=dilated_boolean(self.search_legal,np.ones((7,7),np.uint8))
            else:
                gap=cv2.dilate(self.search_legal.astype(np.uint8),np.ones((7,7),np.uint8))>0
            self.search_legal|=gap&(topology.subregions==0)
            if request_cost_enabled('T'):del gap
        self.timing['r11_initial_masks_ms']=(time.monotonic()-mask_started)*1000
        before_restore=time.monotonic()
        self._static_route_cache_hit=self._restore_static_route()
        self.timing['route_lookup_actual_ms']=(time.monotonic()-before_restore)*1000
        if not self._static_route_cache_hit:
            from .tail_access import remember
            remember(self)
            self._activate_reference()
            if tail_enabled('L'):
                from .tail_parking import repair_reference
                repair_reference(self)
            self._store_static_route()
        if tail_enabled('J'):
            # Initial L2 construction is complete. Dynamic replans build a
            # fresh controller; cached-route restoration already releases both.
            self.priority=None;self.search_legal=None
        self.timing['l2_cache_source_response_ms']=self.initial_l2_result.response_ms
        self.timing['l2_current_search_response_ms']=0. if self._static_route_cache_hit else self.initial_l2_result.response_ms
        self.timing['adapter_calls']=[]
        if v9_enabled('A'):
            from .v9_maps import seal
            self.plan=replace(self.plan,corridor_mask=seal(self.plan.corridor_mask))
            self.reference_plan=replace(self.reference_plan,world_xy=seal(self.reference_plan.world_xy))
            self.reference_cells=seal(self.reference_cells)
            self.global_drivable=seal(self.global_drivable)
        self.timing.update(l2_total_ms=(time.monotonic()-tick)*1000,l2_cpu_ms=(time.process_time()-cpu)*1000,actual_resolution_m=.15,requested_resolution_m=.15,completion_mode=self.reference_mode,search_states=self.l2.geometry.state_count)

    def _dynamic_identity(self):
        if self.dynamic is None:return (0,'static')
        return (int(self.dynamic.version),v9_digest(self.dynamic.mask))

    def _route_cache_key(self):
        return (self.r6_contract,self.world.map.sha256,fresh_semantic_hash(self.world.semantic),
                canonical_hash(self.world.vehicle),self._dynamic_identity(),self.topology_route.signature,
                float(self.weight),tuple(self.topology_route.start),tuple(self.topology_route.goal),
                os.environ.get('THREEYD_R13_START_ACCESS_TUBE','0'),
                os.environ.get('THREEYD_R13_GOAL_ACCESS_TUBE','0'),
                os.environ.get('THREEYD_R13_ENDPOINT_CONNECTOR','0'),
                os.environ.get('THREEYD_R13_NORTH_EXIT_GUIDE','0'),
                os.environ.get('THREEYD_R13_GOAL_POSE_GUIDE','0'),self.r4_semantic_body)

    def _restore_static_route(self):
        cache=get_cache(self.views,'route-fields',max_entries=1)
        saved=cache.get(self._route_cache_key())
        if saved is None:
            from .request_cost import enabled as request_cost_enabled
            # This one-entry cache cannot serve the new task. Evict before
            # allocating its replacement, keeping identical-request hits.
            if request_cost_enabled('R'):
                cache.clear()
                # A different route needs newly constructed L2 fields. Its
                # one-entry composed-map cache cannot be retained for reuse
                # at the expense of overlapping old maps and new L2 arrays.
                # Native current-map/readback ownership remains in session.
                get_cache(self.views,'bounded-costmap',max_entries=1).clear()
            return False
        for name,value in saved.items():setattr(self,name,value)
        if getattr(self,'r13_semantic_seam',None) is not None:
            self.traffic.topology=copy.copy(self.topology)
            self.traffic.topology.subregions=self.topology.subregions.copy()
            self.traffic.topology.subregions[self.r13_semantic_seam]=self.r13_semantic_road_id
        # Each controller owns its publication cache and L3 query state even
        # when the immutable L2 route objects are reused.
        self._soft_grid_cache={};self._bounded_grid=None;self._search_mask=None
        from .v10_storage import retained_mask
        self.server_l3_mask=retained_mask(self.plan.corridor_mask);self.pending_l3_mask=self.server_l3_mask
        self.timing={'activation_attempts':[{'success':True,'cache_hit':True,
            'cache':'r6 route-conditioned L2 field and D* result'}],
            'r6_route_cache_hit':True,'r6_route_cache':cache.diagnostics()}
        return True

    def _store_static_route(self):
        names=('view','preference','l2','initial_l2_result','raw_reference_xy','reference_mode',
               'rectangle_bridge_certificates','rectangle_bridge_points','rectangle_bridge_cells',
               'l2_soft_target_xy','l2_guide_offset_records','l2_guide_offset_info','corner_records',
               'obstacle_repair_points','reference_cells','reference_plan','route','plan','global_drivable',
               'r13_prefix_connector','r13_semantic_seam','r13_semantic_road_id')
        cache=get_cache(self.views,'route-fields',max_entries=1)
        cache[self._route_cache_key()]={name:getattr(self,name) for name in names}
        self.timing.update(r6_route_cache_hit=False,r6_route_cache=cache.diagnostics())

    def _set_reference(self,xy):
        if v9_enabled('A'):
            from .v9_maps import seal
            # Seal before ReferenceRoute and the shared route-field cache bind
            # their arrays, so they cannot keep duplicate mutable buffers.
            self.plan=replace(self.plan,corridor_mask=seal(self.plan.corridor_mask))
            self.global_drivable=seal(self.global_drivable)
            xy=seal(xy)
        SoftEdgeController._set_reference(self,xy)
        if v9_enabled('A'):
            from .v9_maps import seal
            self.reference_cells=seal(self.reference_cells)

    def _clear_transients(self):
        """Release controller-owned query arrays; shared bounded caches survive."""
        self.trajectory=None;self._search_mask=None;self._bounded_grid=None
        self.pending_l3_mask=None;self.server_l3_mask=None
        self._v9_target=None
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
             v9_digest(self.reference_cells),v9_digest(mask),version,int(cap))
        if v9_enabled("A"):
            key += (v9_digest(self.world.hard),v9_digest(self.world.map.occupancy),
                    v9_digest(self.plan.corridor_mask),v9_digest(self.global_drivable))
        cache=get_cache(self.views,'bounded-costmap',max_entries=1)
        grid=cache.get(key)
        if grid is not None:
            self._bounded_grid=grid
            self.timing['r6_costmap_cache_hit']=True
            self.timing['r6_costmap_cache']=cache.diagnostics()
            return grid
        from .localfix_options import enabled as localfix_enabled
        if localfix_enabled('C'):cache.clear()
        if self.optimize:
            from .fast_costmap import compose_bounded
            grid=compose_bounded(self,mask,cap)
        else:grid=R5BoundedController.prepare_grid(self,True,cap)
        if self.optimize and v9_enabled('A'):
            from .curve_backend import settings
            from .v9_r5_options import enabled as full_storage_enabled
            if settings()[0]=='independent' and full_storage_enabled():
                from .compact_auxiliary import compact_owned_grid
                grid=compact_owned_grid(grid,derive_static=not self.generic_mode)
        grid=seal_grid(grid)
        cache[key]=grid
        self._bounded_grid=grid
        self.timing['r6_costmap_cache_hit']=False
        self.timing['r6_costmap_cache']=cache.diagnostics()
        return grid

    def _activate_reference(self):
        tick=time.monotonic();world=self.world
        from .tail_options import enabled as tail_enabled
        semantic_body=self.r4_semantic_body and not self.generic_mode
        body_scope=self.plan.corridor_mask
        if semantic_body and self.traffic.endpoint_parking:
            body_scope=body_scope&(~semantic_membership(self.topology.subregions,list(self.traffic.endpoint_parking))|self.priority)
        radius=math.hypot(*world.vehicle[:2])
        key=('v5_r4_conservative_circle',radius,0 if self.dynamic is None else self.dynamic.version)
        if semantic_body:key=(*key,'semantic_body',v9_digest(self.plan.corridor_mask))
        if key not in self.views:
            view=copy.copy(self.views.get(3) or GridView(world,3))
            fine_hard=world.hard if self.dynamic is None else world.hard|self.dynamic.mask
            if semantic_body:fine_hard=fine_hard|~body_scope
            expanded=inflate_circle(fine_hard,radius,world.map.resolution)
            hard=~aggregate(~expanded,3,all_free=True,pad_value=False)
            penalty=np.broadcast_to(np.frombuffer(bytes(4),np.float32),hard.shape)
            from .coarse_masks import pair as coarse_pair
            view.safe,view.hard=coarse_pair(hard);view.r4_obstacle_penalty=penalty
            view.key=canonical_hash([view.key,'v5-r4-conservative-circle',radius,digest(view.safe)])
            if not semantic_body:self.views[key]=view
            from .recovery_options import enabled as recovery_enabled
            if recovery_enabled('L'):
                # Inflation inputs have been consumed into the coarse view.
                # Rectangle recovery constructs its own current fine mask.
                del fine_hard,expanded
        circle_view=view if semantic_body else self.views[key]
        mask=aggregate_scope(self.search_legal,3)
        self.timing['r11_l2_obstacles_scope_ms']=(time.monotonic()-tick)*1000
        fields_started=time.monotonic()
        record={'resolution':.15,'former_circle_radius_m':radius,'scope_cells':int(mask.sum()),'state_dimensions':['x','y'],
                'hard_obstacle_model':'original circumscribed circle; every represented fine cell clear',
                'static_obstacle_model':'confirmed V5-r3 conservative primary',
                'l2_vehicle_feasibility_is_final':False,
                'success':False,'rectangle_recovery_attempted':False}
        self.timing['activation_attempts'].append(record)
        try:
            shape=circle_view.safe.shape
            from .r11_fields import bbox as scope_bbox
            # Normal circle planning only needs the exact bounds. Materialize
            # every scope coordinate only if rectangle recovery consumes it.
            bbox=scope_bbox(mask,halo=3)
            if bbox is None:raise RuntimeError('V5_L2_EMPTY_ROUTE_ROI')
            r0,r1,c0,c1=bbox;local_shape=(r1-r0,c1-c0)
            seed=np.ones(local_shape,np.uint8)
            line=np.asarray(self.task['right_reference_polyline'],float)
            if self.generic_mode:
                self.l2_guide_offset_records=[]
                self.l2_guide_offset_info={'mode':'free_space','straight_offset_m':0.,
                                           'traffic_direction_penalty':False,
                                           'corner_region_count':0}
            else:
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
            if r4_enabled('A'):
                from .r4_fields import road_fields
                normalized_local,eligible_local,direction_local=road_fields(self,circle_view,bbox)
            else:
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
            from .coarse_masks import false_field
            ambiguous=RoiGrid(false_field(local_shape),bbox,shape,False)
            pref=RoutePreference(potential,normalized,eligible,ambiguous,canonical_hash([self.task['signature'],'v7-free-space-no-offset' if self.generic_mode else 'r8-full-route-centerward-offset-0.30m',bbox,digest(potential_local)]),
                {'source':'neutral free-space L1 guide' if self.generic_mode else 'selected L1 right polyline shifted 0.30 m inward across roads and junctions','soft_not_hard':True,
                 'l1_geometry_unchanged':self.generic_mode,'centerward_offset_m':0. if self.generic_mode else .30,
                 'storage':'route-local ROI','bbox_rc':list(bbox),'resident_field_bytes':field_resident(potential_local,normalized_local,eligible_local,direction_local)})

            self.timing['r11_l2_fields_ms']=(time.monotonic()-fields_started)*1000
            def solve(safe,tag,use_direction=True):
                prepare_started=time.monotonic()
                from .coarse_masks import hard_for
                view=copy.copy(circle_view);view.safe=np.asarray(safe,bool)
                view.hard=hard_for(view.safe,circle_view.safe,circle_view.hard)
                view.key=canonical_hash([circle_view.key,tag,digest(view.safe)])
                roi=make_roi(view,mask,view.cell(self.topology_route.start),view.cell(self.topology_route.goal),self.task['signature'])
                geometry=CompactCorridorGeometry.build(roi)
                rc=(state_coordinates(geometry) if first_enabled("A") else
                    np.asarray([geometry.global_cell(i) for i in range(geometry.state_count)],np.int32))
                binding=MutableStateBinding(geometry.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,'static',canonical_hash([bbox,digest(potential_local),self.weight,'V7_FREE_SPACE_NO_OFFSET' if self.generic_mode else 'V5_R8_FULL_ROUTE_OFFSET_030',tag]))
                state_direction=direction[rc[:,0],rc[:,1]] if use_direction else np.zeros((len(rc),2),np.float32)
                from .fast_state import PrecomputedEdgeState
                state_class=PrecomputedEdgeState if self.optimize else EdgeState
                if self.optimize and r4_enabled('C'):
                    from .native_cold_search import NativeColdState
                    state_class=NativeColdState
                state=state_class(geometry,binding,search_potential[rc[:,0],rc[:,1]],self.weight,state_direction,np.zeros(len(rc)))
                planner=BoundedReferenceDStar(roi,geometry,state)
                self.timing['r11_l2_state_prepare_ms']=self.timing.get('r11_l2_state_prepare_ms',0.)+(time.monotonic()-prepare_started)*1000
                search_started=time.monotonic();result=planner.initialize()
                self.timing['r11_l2_search_ms']=self.timing.get('r11_l2_search_ms',0.)+(time.monotonic()-search_started)*1000
                if not result.success:raise RuntimeError(result.failure_code or 'V5_L2_NO_PATH')
                return view,geometry,planner,result,np.asarray(planner.path_global,int)

            # On a corroborated static map, retain the original conservative
            # route whenever it already works.  Only a disconnected/failed
            # original L2 search may use newly corroborated free pixels.
            original_circle_solution=None
            original_hard=getattr(world,'r12_original_hard',None)
            if original_hard is not None:
                original_fine_hard=original_hard if self.dynamic is None else original_hard|self.dynamic.mask
                if semantic_body:original_fine_hard=original_fine_hard|~body_scope
                original_expanded=inflate_circle(original_fine_hard,radius,world.map.resolution)
                original_safe=aggregate(~original_expanded,3,all_free=True,pad_value=False)
                original_components,_=ndimage.label(original_safe[r0:r1,c0:c1]&mask[r0:r1,c0:c1],np.ones((3,3),int))
                original_start=circle_view.cell(self.topology_route.start)
                original_goal=circle_view.cell(self.topology_route.goal)
                osl=(original_start[0]-r0,original_start[1]-c0)
                ogl=(original_goal[0]-r0,original_goal[1]-c0)
                if min(*osl,*ogl)>=0 and osl[0]<original_components.shape[0] and ogl[0]<original_components.shape[0] and osl[1]<original_components.shape[1] and ogl[1]<original_components.shape[1] and original_components[osl] and original_components[osl]==original_components[ogl]:
                    try:
                        original_circle_solution=solve(original_safe,'r12-original-circle-first')
                    except (ValueError,RuntimeError) as original_error:
                        record['r12_original_circle_failure']=str(original_error)
                else:
                    record['r12_original_circle_failure']='CIRCLE_DISCONNECTED'
            circle_error=None
            try:
                if original_circle_solution is not None:
                    view,geometry,planner,result,raw=original_circle_solution
                    record['r12_primary_map']='registered_original_pgm'
                else:
                    labels,_=ndimage.label(circle_view.safe[r0:r1,c0:c1]&mask[r0:r1,c0:c1],np.ones((3,3),int))
                    start=circle_view.cell(self.topology_route.start);goal=circle_view.cell(self.topology_route.goal)
                    sl=(start[0]-r0,start[1]-c0);gl=(goal[0]-r0,goal[1]-c0)
                    if min(*sl,*gl)<0 or sl[0]>=labels.shape[0] or gl[0]>=labels.shape[0] or sl[1]>=labels.shape[1] or gl[1]>=labels.shape[1] or not labels[sl] or labels[sl]!=labels[gl]:raise RuntimeError('V5_L2_CIRCLE_DISCONNECTED')
                    view,geometry,planner,result,raw=solve(circle_view.safe,'circle')
                    record['r12_primary_map']='corroborated_effective_map'
                self.rectangle_bridge_certificates=[];self.rectangle_bridge_points=np.empty((0,2),float)
                self.rectangle_bridge_cells=np.empty((0,2),int)
                self.reference_mode='v5_r4_conservative_circle'
            except (ValueError,RuntimeError) as error:
                circle_error=str(error);record['circle_failure']=circle_error;record['rectangle_recovery_attempted']=True
                fine_hard=world.hard if self.dynamic is None else world.hard|self.dynamic.mask
                if semantic_body:fine_hard=fine_hard|~body_scope
                from .r10_options import enabled as r10_enabled
                if not r10_enabled('B') and not r2_enabled('A'):
                    bits_key=('v5_r1_rectangle_bits',0 if self.dynamic is None else self.dynamic.version)
                    if bits_key not in self.views:
                        self.views[bits_key]=rectangle_candidate_bits(world,circle_view,fine_hard)
                    morphology_bits=np.asarray(self.views[bits_key],np.uint32)
                # The initial path is position-only and exists solely to locate
                # the circle-disconnected band.  It never becomes a published
                # reference and never opens an obstacle cell.
                # Coordinates were computed above for the exact same ROI.
                # Reuse them; no new rounding or footprint approximation.
                from .r3_options import enabled as r3_enabled
                if r4_enabled('A'):
                    from .r4_fields import point_free
                    scope_cells=np.column_stack(np.nonzero(mask))
                    point_safe=point_free(world,circle_view,scope_cells,fine_hard,shape)
                else:
                    if r3_enabled('A'):pr,pc=fr,fc
                    else:
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
                    if not r2_enabled('A'):
                        bits_key=('v5_r1_rectangle_bits',0 if self.dynamic is None else self.dynamic.version)
                        if bits_key not in self.views:
                            self.views[bits_key]=rectangle_candidate_bits(world,circle_view,fine_hard)
                        morphology_bits=np.asarray(self.views[bits_key],np.uint32)
                record['r2_skip_ordering_only_morphology']=r2_enabled('A')
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
                    exact_cache={} if semantic_body else self.views.setdefault(exact_key,{})
                    certifier=RectanglePoseCertifier(world,fine_hard,circle_view,bits,cache=exact_cache)
                    path_tangent={tuple(x['cell']):x['yaw_rad'] for x in certify_relaxed_path(point_raw,circle_view.safe,certifier)[0]}
                    candidates=np.column_stack(np.nonzero(band))
                    # Likely morphology passes first; this does not change the
                    # set eventually checked or the authoritative decision.
                    if not r2_enabled('A'):
                        likely=morphology_bits[candidates[:,0],candidates[:,1]]!=0 if len(candidates) else np.empty(0,bool)
                        candidates=np.concatenate((candidates[likely],candidates[~likely])) if len(candidates) else candidates
                    opened=np.zeros(shape,bool);rejected=0
                    from .pose_groups import enabled as grouped_enabled,certify_many
                    preferred_yaws=[path_tangent.get(tuple(map(int,cell))) for cell in candidates]
                    items=(certify_many(certifier,candidates,preferred_yaws) if grouped_enabled() else
                           [certifier.certify(cell,yaw) for cell,yaw in zip(candidates,preferred_yaws)])
                    for cell,item in zip(candidates,items):
                        if item is None:rejected+=1
                        else:
                            opened[tuple(cell)]=True;certificates.append(item)
                    record['r3_exact_pose_batch_calls']=certifier.batch_calls
                    record['r3_exact_pose_checks']=certifier.exact_pose_checks
                    record['r8_exact_pose_group_calls']=getattr(certifier,'native_group_calls',0)
                    record['rectangle_band_radius_m']=self.rectangle_check_radius_m
                    record['rectangle_band_candidates']=len(candidates)
                    record['rectangle_band_certified']=len(certificates)
                    record['rectangle_rejected_cells']=rejected
                    record['first_component_break']=first_component_break(point_raw,(circle_view.safe|opened)&mask,circle_view,certificates)
                    try:solution=solve(circle_view.safe|opened,'rect-certified-band')
                    except (ValueError,RuntimeError) as candidate_error:record['rectangle_failure']=str(candidate_error)
                from .localfix_options import enabled as localfix_enabled
                if solution is None and point_trial is not None and localfix_enabled('G') and self.generic_mode:
                    # A map-only L1 corridor is a search hint, not an obstacle.
                    # Try bounded local detours around diagnosed disconnects.
                    # Semantic maps never enter this path; every hard pixel,
                    # fixed pose and final L3 certificate remains unchanged.
                    original_mask=mask
                    allowed=circle_view.safe|opened
                    detours=[];fine_extension=np.zeros(world.hard.shape,bool)
                    for local_attempt in range(3):
                        gap=first_component_break(point_trial[-1],allowed&mask,circle_view,certificates)
                        if not gap.get('found'):break
                        center=(np.asarray(gap['source_pose'][:2])+np.asarray(gap['target_pose'][:2]))*.5
                        radius=(3.,5.,8.)[local_attempt]
                        from .local_detour import extend_hint
                        extend_hint(fine_extension,world,self.topology.subregions,center,radius)
                        trial_fine=self.search_legal|fine_extension
                        mask=aggregate_scope(trial_fine,3)
                        detail={'radius_m':radius,'center_xy':center.tolist(),'added_coarse_cells':int(mask.sum()-original_mask.sum()),'physical_obstacles_unchanged':True,'map_only_local_hint_detour':True}
                        try:solution=solve(allowed,'r6-local-hint-detour')
                        except (ValueError,RuntimeError) as err:detail['failure']=str(err)
                        detours.append(detail)
                        if solution is not None:
                            self.search_legal=trial_fine
                            self.global_drivable=self.global_drivable|fine_extension
                            from .v10_storage import retained_mask
                            physical=retained_mask(self.global_drivable&~world.hard)
                            self.plan=replace(self.plan,corridor_mask=physical)
                            self.server_l3_mask=physical;self.pending_l3_mask=physical
                            record['r6_local_detour_selected']=True
                            break
                    if solution is None:mask=original_mask
                    record['r6_local_detours']=detours
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

            post_started=time.monotonic()
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
            if os.environ.get('THREEYD_R13_NORTH_EXIT_GUIDE')=='1':
                xy=self._r13_north_exit_reference(xy)
            if os.environ.get('THREEYD_R13_GOAL_POSE_GUIDE')=='1':
                xy=self._r13_goal_pose_reference(xy)
            if os.environ.get('THREEYD_R13_ENDPOINT_CONNECTOR')=='1':
                xy=self._r13_parking_pose_reference(xy)
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
            self.timing['r11_l2_shortcut_endpoint_ms']=(time.monotonic()-post_started)*1000
            access_started=time.monotonic()
            record['endpoint_parking_access_cells']=self._bind_endpoint_parking_access(xy)
            self.timing['r11_l2_parking_bind_ms']=(time.monotonic()-access_started)*1000
            reference_started=time.monotonic();self._set_reference(xy)
            self.timing['r11_l2_set_reference_ms']=(time.monotonic()-reference_started)*1000
            record.update(success=True,states=geometry.state_count,raw_nodes=len(raw),straightened_nodes=len(xy),corner_count=len(self.corner_records),obstacle_repair_points=len(self.obstacle_repair_points),completion_mode=self.reference_mode,
                          r4_compact_fields=r4_enabled('A'),r6_roi_bbox_rc=list(bbox),r6_roi_cells=int(np.prod(local_shape)),r6_full_grid_cells=int(np.prod(shape)),r6_roi_fraction=float(np.prod(local_shape)/np.prod(shape)),
                          r6_roi_field_bytes=field_resident(potential_local,normalized_local,eligible_local,direction_local),
                          shortcut_cost_field=('V7 neutral L1 guide; hard obstacles enforced' if self.generic_mode else
                                               'r8 selected L1 guide inward offset 0.30m across roads and junctions; hard obstacles and direction enforced'),
                          l2_soft_target_offset_m=0. if self.generic_mode else .30,
                          l2_soft_target_side='none' if self.generic_mode else 'toward_route_interior',
                          l1_geometry_unchanged=self.generic_mode,
                          l2_soft_target_points=len(self.l2_soft_target_xy),
                          l2_soft_target_corner_policy=('none' if self.generic_mode else
                                                        'full 0.30m inside junction; round join; no exemption or taper'),
                          l2_soft_target_corner_count=self.l2_guide_offset_info['corner_region_count'])
            result.diagnostics.update(v5=record,heading_state=False,progress_state=False)
        except (ValueError,RuntimeError) as e:
            record['failure']=str(e)
            from arena_3d_v1.semantic_world import json_write
            json_write(self.root/('failed_reference_'+self.task['signature'][:12]+'.json'),self.timing)
            raise
        finally:record['wall_ms']=(time.monotonic()-tick)*1000

    def _target_mask(self):
        if self._search_mask is None:raise RuntimeError('GLOBAL_L3_SEARCH_FORBIDDEN')
        if v9_enabled('A'):
            key=(v9_digest(self.plan.corridor_mask),v9_digest(self._search_mask),
                 None if self.dynamic is None else v9_digest(self.dynamic.mask),
                 None if self.r13_remainder_mask is None else v9_digest(self.r13_remainder_mask))
            cached=getattr(self,'_v9_target',None)
            if cached is not None and cached[0]==key:
                target=cached[1]()
                if target is not None:return target
        from .target_mask import build as build_target_mask
        result=build_target_mask(self,SoftEdgeController._target_mask)
        if v9_enabled('A'):
            from .v9_maps import seal
            import weakref
            result=seal(result);self._v9_target=(key,weakref.ref(result))
        return result

    def sync_only(self,session):
        if not v9_enabled('A'):return super().sync_only(session)
        from contextlib import nullcontext
        with self.dynamic.lock if self.dynamic else nullcontext():
            version=0 if self.dynamic is None else self.dynamic.version
            grid=self.prepare_grid();mask=self._target_mask()
        session.set_semantic_costmap(grid);ack=session.update_local_mask(mask)
        if self.dynamic and self.dynamic.version!=version:
            ack.update(costmap_update_acknowledged=False,costmap_ack_status='MAP_VERSION_CHANGED_DURING_SYNC')
        return ack

    def _audit(self,*args,**kwargs):
        started=time.monotonic()
        try:return SoftEdgeController._audit(self,*args,**kwargs)
        finally:
            self.timing['strict_audit_calls']=self.timing.get('strict_audit_calls',0)+1
            self.timing['strict_audit_total_ms']=self.timing.get('strict_audit_total_ms',0.)+(time.monotonic()-started)*1000

    def plan_l3(self,query,session,auditor,spec,*,enabled=True,cap=140):
        from .fullroute import plan
        return plan(self,query,session,auditor,spec,enabled=enabled,cap=cap)

    def _plan_l3_uncached(self,query,session,auditor,spec,*,enabled=True,cap=140):
        from .curve_backend import identity, plan
        if identity()[1]!='control':return plan(self,query,session,auditor,spec,enabled=enabled,cap=cap)
        return self._plan_l3_legacy(query,session,auditor,spec,enabled=enabled,cap=cap)

    def _plan_l3_legacy(self,query,session,auditor,spec,*,enabled=True,cap=140):
        if not enabled:raise ValueError('L2_SOFT_COST_REQUIRED')
        forced=np.concatenate((np.asarray(getattr(self,'rectangle_bridge_points',[]),float).reshape(-1,2),
                               np.asarray(getattr(self,'obstacle_repair_points',[]),float).reshape(-1,2)))
        mask,info=variable_neighborhood(self.world,self.reference_plan.world_xy,self.corner_records,
                                        forced_turning_points=forced)
        from .v9_r8_fields import freeze_owned_scope
        self._search_mask=freeze_owned_scope(mask)
        from .v9_maps import immutable
        self.timing['r8_owned_search_mask_immutable']=immutable(self._search_mask)
        if hasattr(session,'bind_complete_route_scope'):
            session.bind_complete_route_scope(query,self._target_mask())
        connector=self.r13_prefix_connector
        full_target=self._target_mask() if connector is not None else None
        if connector is not None:
            from arena_3d_v1.semantic_world import SweptChecker
            parking_rid=int(self.topology.contexts[self.topology_route.signature]['endpoint_access']['start']['semantic_region_id'])
            remainder=full_target&(self.topology.subregions!=parking_rid)
            anchor_rc=self.world.cells(connector[-1:,:2]);ar=int(anchor_rc[0][0]);ac=int(anchor_rc[1][0])
            required_parking_radius=None
            for radius_m in (0.,.15,.25,.35,.45,.60):
                candidate=remainder.copy()
                if radius_m>0:
                    rad=int(math.ceil(radius_m/self.world.map.resolution))
                    r0=max(0,ar-rad);r1=min(candidate.shape[0],ar+rad+1)
                    c0=max(0,ac-rad);c1=min(candidate.shape[1],ac+rad+1)
                    yy,xx=np.mgrid[r0:r1,c0:c1]
                    near=((yy-ar)**2+(xx-ac)**2)*self.world.map.resolution**2<=radius_m**2
                    local=candidate[r0:r1,c0:c1]
                    local|=near&full_target[r0:r1,c0:c1]&(self.topology.subregions[r0:r1,c0:c1]==parking_rid)
                checker=SweptChecker(self.world.hard|~candidate,self.world.map.resolution,
                                     self.world.map.origin,*self.world.vehicle[:2])
                if checker.check(connector[-1:]):
                    remainder=candidate;required_parking_radius=radius_m;break
            if required_parking_radius is None:
                return {'success':False,'failure_code':'R13_PARKING_ANCHOR_OUTSIDE_REMAINDER_SCOPE',
                        'r13_parking_stitch':{'attempted':False,'anchor_pose_clear':False}}
            self.timing['r13_remainder_parking_radius_m']=required_parking_radius
            self.r13_remainder_mask=freeze_owned_scope(remainder)
        target=self._target_mask()
        if os.environ.get('THREEYD_R13_SCOPE_DIAG')=='1':
            from arena_3d_v1.semantic_world import SweptChecker
            probe={}
            scope_checker=SweptChecker(self.world.hard|~target,self.world.map.resolution,self.world.map.origin,*self.world.vehicle[:2])
            for role,pose in (('start',query.start),('goal',query.goal)):
                rr,cc=self.world.cells(np.asarray([pose[:2]],float));r=int(rr[0]);c=int(cc[0]);p=np.asarray([pose],float)
                probe[role]={'cell':[r,c],'hard':bool(self.world.hard[r,c]),
                             'base_mask':bool(self.plan.corridor_mask[r,c]),
                             'neighborhood_mask':bool(mask[r,c]),
                             'target_mask':bool(target[r,c]),
                             'static_pose_clear':bool(self.world.checker.check(p)),
                             'scope_pose_clear':bool(scope_checker.check(p))}
            components,component_count=ndimage.label(target,np.ones((3,3),np.uint8))
            start_component=int(components[tuple(probe['start']['cell'])])
            goal_component=int(components[tuple(probe['goal']['cell'])])
            probe['point_components']={'count':int(component_count),
                'start':start_component,'goal':goal_component,
                'same_nonzero':bool(start_component and start_component==goal_component)}
            np.savez_compressed(self.root/('r13_scope_'+self.topology_route.signature[:12]+'.npz'),mask=target)
            self.timing['r13_scope_probe']=probe
        count=int(target.sum());limit=min(self.max_search_cells,int(mask.size*self.max_map_fraction))
        info.update(search_cells=count,limit_cells=limit,full_map_cells=mask.size,map_fraction=count/mask.size,L2_soft_cost_enabled=True)
        self.variable_scope_info=info
        if count>limit:
            return {'success':False,'failure_code':'L3_SPATIAL_BUDGET','bounded_l3':info}
        run_query=query if connector is None else replace(query,start=tuple(map(float,connector[-1])))
        from .v9_r2_options import enabled as r2_enabled
        if v9_enabled('B') or v9_enabled('C') or r2_enabled('P'):
            from .v9_local import plan as local_plan
            local=local_plan(self,run_query,session,auditor,spec,cap=cap)
            if local is not None and (local.get('success') or any(x in local.get('failure_code','') for x in ('ACK','VERSION','BINDING'))):
                out=local
            else:
                out=None
        else:local=None;out=None
        segmented=False
        if self.generic_mode:
            from .bounded_segments import strategy_cache
            cache,cache_key=strategy_cache(self,run_query)
            segmented=bool(cache.get(cache_key))
        if out is not None:pass
        elif segmented:
            from .bounded_segments import recover
            out=recover(self,run_query,session,auditor,spec,
                        {'success':False,'called':False,'failure_code':'CACHED_SECTION_STRATEGY'},cap=cap)
        else:
            from .early_sections import plan as early_plan
            out=early_plan(self,run_query,session,auditor,spec,cap=cap)
            if out is None:
                out=SoftEdgeController.plan_l3(self,run_query,session,auditor,spec,enabled=True,cap=cap)
            if (self.generic_mode and not out.get('r3_early_sections') and not out.get('success')
                    and out.get('diagnostics',{}).get('termination_reason')=='SEARCH_ITERATION_LIMIT'):
                from .bounded_segments import recover
                out=recover(self,run_query,session,auditor,spec,out,cap=cap)
        if local is not None and out is not local:
            from .early_sections import merge_diagnostics
            out['diagnostics']=merge_diagnostics([local.get('diagnostics',{}),out.get('diagnostics',{})])
            out['v9_local']=local.get('v9_local',{})
            out['v9_local']['bounded_whole_fallback']=True
        out['bounded_l3']=info
        out['v5']={'heading_state':False,'progress_state':False,'corners':self.corner_records,'l1_points':len(self.task['right_reference_polyline']),'l2_points':len(self.reference_plan.world_xy),'rectangle_bridge_cells':len(getattr(self,'rectangle_bridge_cells',[])),'rectangle_bridge_static_pose_only':True,
                   'l2_soft_target_offset_m':0. if self.generic_mode else .30,'l2_soft_target_side':'none' if self.generic_mode else 'toward_route_interior','l1_geometry_unchanged':True,
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
            deadline=time.monotonic()+min(3. if connector is not None else 8.,max(0.,available))
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
        if connector is not None:
            self.r13_remainder_mask=None
            stitch={'attempted':True,'connector_points':len(connector),
                    'connector_length_m':float(np.linalg.norm(np.diff(connector[:,:2],axis=0),axis=1).sum()),
                    'local_vehicle_plan_success':bool(out.get('success'))}
            result=out.get('result')
            if result and result.points and out.get('success'):
                from arena_3d_v1.curvature_repair import truthful_points
                final_ack=self.sync_only(session)
                stitch['full_map_ack']=final_ack
                if final_ack.get('costmap_update_acknowledged') is not True:
                    out.update(success=False,failure_code='R13_FULL_COSTMAP_ACK_FAILED')
                    out['r13_parking_stitch']=stitch
                    return out
                self.pending_l3_mask=full_target
                self.server_l3_mask=full_target
                local=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points],float)
                seam_position_error=float(np.linalg.norm(connector[-1,:2]-local[0,:2]))
                seam_yaw_error=float(abs(math.atan2(math.sin(connector[-1,2]-local[0,2]),
                                                  math.cos(connector[-1,2]-local[0,2]))))
                full=np.vstack((connector[:-1],local)) if seam_position_error<1e-6 and seam_yaw_error<1e-6 else np.vstack((connector,local))
                points=truthful_points(full,result,0,len(full)-1)
                ca,ind,traffic,jump,failures=self._audit(query,full,self.pending_l3_mask,auditor,points)
                stitch.update(seam_position_error_m=seam_position_error,
                              seam_yaw_error_rad=seam_yaw_error,
                              final_failures=failures,success=not failures)
                out.update(success=not failures,failure_code=';'.join(failures),
                           geometry_success=bool(ca.final_valid_success and ind['valid']),
                           independent_audit=ind,traffic_audit=traffic,
                           constraint_audit={'max_raw_steering_jump_deg':jump,
                               'steering_jump_valid':jump<=15+1e-6,'traffic':traffic},metrics=ca.metrics)
                if not failures:
                    out['result']=replace(result,points=points,path_audit=ca,
                                          source='r13_validated_parking_prefix_and_bounded_smac')
                    self.trajectory=full
                    self.trajectory_version=0 if self.dynamic is None else self.dynamic.version
            out['r13_parking_stitch']=stitch
        return out

    def update_dynamic(self,*args,**kwargs):
        raise NotImplementedError('V5_R1_STATIC_VALIDATION_ONLY')
