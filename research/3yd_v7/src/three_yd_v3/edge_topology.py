"""Bridge directed boundary uses to the existing fine semantic raster.

Shared crossings below are DERIVED L2 interfaces. The L1 route is an ordered
list of uses of original edges, not a second region adjacency search.
"""
import json, math, time
import numpy as np
from arena_3d_v1.multires_topology import TopologyRoute
from arena_evaluation.semantic_map import canonical_hash
from .topology import OutlineTopology
from .edge_routing import EdgeRouter, AttachmentError


class EdgeTopology(OutlineTopology):
    def __init__(self, world, graph_path, cache_root, *, semantic_path=None, access_path=None):
        tick=time.monotonic()
        super().__init__(world, graph_path, cache_root)
        from pathlib import Path
        semantic_path=Path(semantic_path or Path(graph_path).with_name('semantic.json'))
        access_path=Path(access_path or Path(graph_path).with_name('parking_access.json'))
        self.router=EdgeRouter(self.graph,json.loads(semantic_path.read_text()),json.loads(access_path.read_text()) if access_path.exists() else None)
        self.key=canonical_hash([self.key,self.router.key,'endpoint-parking-free-exploration-v1']);self.edge_plans={}
        self.preparation_ms=(time.monotonic()-tick)*1000

    def plan(self, query, *, excluded_edge_transitions=(), **kwargs):
        regions=[]
        for p in (query.start,query.goal):
            rc=self.world.map.world_to_cell(*p[:2])
            if rc is None or not self.subregions[rc]:raise ValueError('V3_ENDPOINT_HAS_NO_SEMANTIC_REGION')
            regions.append(int(self.subregions[rc]))
        edge_plan=self.router.plan(query.query_id,query.start,query.goal,*regions,excluded=excluded_edge_transitions)
        stages=[]
        for use in edge_plan['ordered_edge_uses']:
            if not stages or stages[-1]['region_id']!=use['region_id']:
                stages.append({'region_id':use['region_id'],'kind':use['kind'],'edge_uses':[]})
            stages[-1]['edge_uses'].append(use)
        for endpoint,index in (('start',0),('goal',-1)):
            a=edge_plan[endpoint+'_attachment'];rid=a['semantic_region_id']
            if self.regions[rid]['kind']=='parking_area':
                stage={'region_id':rid,'kind':'parking_area','edge_uses':[],'access_attachment':a}
                if index==0:stages.insert(0,stage)
                else:stages.append(stage)
        gates=[]
        for i,(a,b) in enumerate(zip(stages,stages[1:])):
            u,v=a['region_id'],b['region_id']
            parking=a if a['kind']=='parking_area' else b if b['kind']=='parking_area' else None
            if parking:
                attachment=parking['access_attachment'];road=attachment['road_id']
                rc=self.parking_contacts.get((parking['region_id'],road))
                if rc is None:raise ValueError('OWNED_PARKING_ROAD_HAS_NO_CONTACT')
                xy=self.world.world_points(rc[:,0],rc[:,1]);side=attachment['screen_side']
                # Contact locations must belong to the explicitly assigned side
                # and the selected original edge interval, not all road contacts.
                use=self.router.uses[attachment['access_projection']['use_id']]
                from .channel import distances_to_gate
                near=distances_to_gate(xy,{'geometry':use.geometry})<=self.world.map.resolution*2
                xy=xy[near]
                if not len(xy):raise ValueError('OWNED_PARKING_SIDE_HAS_NO_CONTACT')
                gates.append({'gate_id':f'parking_{i}','from_region':u,'to_region':v,'temporary':True,
                              'contact_band_world_xy':xy.tolist(),'outline_edge_id':use.edge_id,
                              'role':'query_local_owned_parking_access','screen_side':attachment['access_screen_side'],
                              'travel_screen_side':side,'local_cross_road_allowed':attachment['local_cross_road_allowed']})
            else:
                left=a['edge_uses'][-1];right=b['edge_uses'][0]
                # Transition is at the immutable outline junction. For actual
                # centers, permit the incident shared boundary between regions.
                node=left['end_key']
                if node!=right['start_key'] or node[0]!='node':raise ValueError('EDGE_TRANSITION_NOT_ORIGINAL_NODE')
                shared=[e for e in self.graph['edges'] if set(e['regions'])=={u,v} and node[1] in (e['from'],e['to'])]
                if len(shared)!=1:raise ValueError('EDGE_TRANSITION_HAS_NO_UNIQUE_PHYSICAL_INTERFACE')
                edge=shared[0]
                gates.append({'gate_id':f"derived_{i}_e{edge['id']}",'from_region':u,'to_region':v,
                              'temporary':False,'outline_edge_id':edge['id'],'geometry':edge['geometry'],
                              'role':'derived_road_junction_crossing','original_transition_node':node[1],
                              'selected_side_edge_is_not_crossing':True})
        for i,stage in enumerate(stages):
            stage['stage']=i
            if stage['kind']=='lane':
                uses=stage['edge_uses'];vectors=[]
                for use in uses:
                    p=np.asarray(use['geometry']);vectors.extend(np.diff(p,axis=0))
                direction=np.asarray(vectors).sum(axis=0)
                if np.linalg.norm(direction)<1e-8:raise ValueError('LANE_DIRECTION_UNRESOLVED')
                axis=self.router.axes[stage['region_id']]
                axis=axis*(1 if direction@axis>=0 else -1)
                stage['travel_direction_xy']=axis.tolist()
                stage['screen_side']=uses[0]['screen_side']
        region_ids=tuple(s['region_id'] for s in stages)
        derived={'schema':'3YD-V3-edge-derived-execution-channel-v1','stages':stages,'gates':gates,
                 'edge_channel':edge_plan,'route_signature':edge_plan['signature'],
                 'boundary_use_is_reference_only':True,'parking_transit_allowed':False,'parking_endpoint_scope':'original endpoint parking region; contact restricted to assigned road and access-side edge'}
        self.edge_plans[edge_plan['signature']]=derived
        return TopologyRoute(region_ids,tuple(u['use_id'] for u in edge_plan['ordered_edge_uses']),
            tuple(self.parent_semantics[r] for r in region_ids),tuple(query.start),tuple(query.goal),edge_plan['signature'],
            {**edge_plan['diagnostics'],'edge_channel':edge_plan,'derived_stage_count':len(stages)},
            {'world_key':self.world.key,'topology_key':self.key,**self.world.binding})

    def channel_plan(self, route):return self.edge_plans[route.signature]

    # Endpoint parking regions remain available for L2/L3 obstacle exploration.
    # Their stage is terminal-only, and its crossing is restricted to the owned
    # road/access side. No parking-through transition is created.


class EdgeCandidateRecovery:
    """Bounded alternative ORIGINAL edge sequences, without modifying the graph."""
    def __init__(self, world, topology, cache_root, views, dynamic=None, max_attempts=4, controller_class=None):
        self.world=world;self.topology=topology;self.cache_root=cache_root;self.views=views
        self.dynamic=dynamic;self.max_attempts=max_attempts;self.controller_class=controller_class

    def plan(self,query,session,auditor,spec):
        tick=time.monotonic();queue=[frozenset()];seen=set();attempts=[];c=None;r=None;out={};searches=0
        while queue and len(attempts)<self.max_attempts and searches<32:
            exclusions=queue.pop(0);searches+=1
            try:r=self.topology.plan(query,excluded_edge_transitions=exclusions)
            except AttachmentError as error:
                out={'success':False,'failure_code':str(error),'attachment_error':error.details};break
            except (RuntimeError,ValueError) as error:
                out=out or {'success':False,'failure_code':str(error)};continue
            signature=r.interface_ids
            if signature in seen:continue
            seen.add(signature);c=None;started=time.monotonic()
            try:
                c=self.controller_class(self.world,self.topology,r,cache_root=self.cache_root,views=self.views,dynamic=self.dynamic)
                out=c.plan_l3(query,session,auditor,spec)
            except (RuntimeError,ValueError,TimeoutError) as error:
                if any(v in str(error).upper() for v in ('CORRUPT','BINDING','VERSION')):raise
                out={'success':False,'failure_code':str(error)}
            attempts.append({'regions':list(r.region_ids),'edge_uses':list(signature),'success':out.get('success',False),
                'geometry_success':out.get('geometry_success',False),'failure':out.get('failure_code'),
                'wall_ms':(time.monotonic()-started)*1000,'l1':r.diagnostics})
            if out.get('success') or out.get('geometry_success'):break
            if any(v in str(out.get('failure_code','')).upper() for v in ('ACK','CONTENT','CORRUPT','BINDING','VERSION')):break
            # Exclude a query-local transition only; no claim of permanent blockage.
            for a,b in zip(signature,signature[1:]):queue.append(exclusions|{(a,b)})
            if c:c.lifecycle.clear()
        if not out:out={'success':False,'failure_code':'EDGE_CANDIDATES_EXHAUSTED'}
        out['candidate_recovery']={'attempts':attempts,'searches':searches,'wall_ms':(time.monotonic()-tick)*1000,'persistent_topology_mutations':0}
        return c,r,out
