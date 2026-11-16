"""Adapt approved outline incidence to routes without modifying its nodes/edges.
Directed gate records are transient routing indexes, never new map vertices.
Gate midpoints are heuristic anchors only, not required vehicle waypoints.
"""
from pathlib import Path
from collections import defaultdict, OrderedDict
import json,math,time,itertools
import numpy as np
from scipy import ndimage
from arena_evaluation.semantic_map import canonical_hash
from arena_evaluation.semantic_rasterizer import rasterize_feature
from arena_3d_v1.semantic_graph import InterfacePose
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.semantic_world import digest,json_write

class OutlineTopology(SemanticTopology):
    SCHEMA='3YD-V3-r0-outline-incidence-v1'
    def __init__(self,world,graph_path,cache_root):
        tick=time.monotonic();self.world=world;self.cache_root=Path(cache_root)
        self.graph=json.loads(Path(graph_path).read_text());self.source_hash=canonical_hash(self.graph)
        self.key=canonical_hash([self.SCHEMA,self.source_hash,world.semantic.semantic_map_hash,world.rules])
        self.static_hard=world.hard.copy();self.query_cache=OrderedDict();self.mask_cache=OrderedDict();self.nodes={};self.region_nodes=defaultdict(lambda:{'in':[],'out':[]})
        self.regions={r['id']:r for r in self.graph['regions']}
        self.parent_semantics={rid:next(k for k,f in world.features.items() if f.properties.get('source_name')==r['name']) for rid,r in self.regions.items()}
        self.subregions=np.zeros(world.labels.shape,np.int16)
        area=lambda f:abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(f.coordinates,f.coordinates[1:]+f.coordinates[:1])))
        for rid in sorted(self.regions,key=lambda rid:({'parking_area':10,'lane':20,'junction_area':30}[self.regions[rid]['kind']],-area(world.features[self.parent_semantics[rid]]),rid)):
            f=world.features[self.parent_semantics[rid]]
            self.subregions[rasterize_feature(world.semantic,f)]=rid
        self.gates={}
        for edge in self.graph['edges']:
            if edge['kind']!='shared_outline' or len(edge['regions'])!=2:continue
            a,b=edge['regions'];pts=np.asarray(edge['geometry'],float);d=np.linalg.norm(np.diff(pts,axis=0),axis=1);s=np.r_[0,np.cumsum(d)];j=min(len(d)-1,np.searchsorted(s,s[-1]/2,side='right')-1)
            center=pts[j]+(pts[j+1]-pts[j])*(s[-1]/2-s[j])/max(d[j],1e-12)
            pid=int(edge['id']);self.gates[pid]={'regions':(a,b),'geometry':edge['geometry'],'outline_edge_id':edge['id'],'temporary':False}
            self._add_gate(pid,a,b,center,False)
        self.permanent_ids=set(self.nodes);self.permanent_gate_ids=set(self.gates)
        self.portal_count=len(self.gates);self.edges_count=len(self.graph['edges']);self.cache_hit=False
        self.parking_adjacencies={};self.parking_contacts={}
        # Query-only parking attachments from semantic contact, independent of occupancy.
        for rid,r in self.regions.items():
            if r['kind']!='parking_area':continue
            boundary=ndimage.binary_dilation(self.subregions==rid)&(self.subregions!=rid)
            adjacent={}
            for other in np.unique(self.subregions[boundary]):
                other=int(other)
                if other not in self.regions or self.regions[other]['kind']=='parking_area':continue
                rc=np.argwhere(boundary&(self.subregions==other));center=world.world_points(rc[:,0],rc[:,1]).mean(axis=0)
                adjacent[other]=center.tolist();self.parking_contacts[rid,other]=rc.astype(np.int32)
            self.parking_adjacencies[rid]=adjacent
        self.preparation_ms=(time.monotonic()-tick)*1000
        root=self.cache_root;root.mkdir(parents=True,exist_ok=True)
        json_write(root/'v3_adapter.json',{'schema':self.SCHEMA,'source_hash':self.source_hash,'outline_nodes':len(self.graph['nodes']),'outline_edges':self.edges_count,'gate_count':self.portal_count,'routing_arc_records':len(self.nodes),'new_map_nodes':0,'new_map_edges':0,'obstacles_used_for_l1':False,'parking_attachments':'query_only'})

    def _add_gate(self,pid,a,b,center,temporary):
        for direction,(u,v) in enumerate(((a,b),(b,a))):
            nid=2*pid+direction
            n=InterfacePose(nid,(float(center[0]),float(center[1]),0.),u,v,self.parent_semantics[u],self.parent_semantics[v],pid,temporary)
            self.nodes[nid]=n;self.region_nodes[u]['out'].append(nid);self.region_nodes[v]['in'].append(nid)

    def plan(self,query,**kwargs):
        # Serial query adapter; remove the preceding query's virtual parking links.
        for nid in list(self.nodes):
            if nid not in self.permanent_ids:
                n=self.nodes.pop(nid);self.region_nodes[n.from_region]['out'].remove(nid);self.region_nodes[n.to_region]['in'].remove(nid)
        self.gates={k:v for k,v in self.gates.items() if k in self.permanent_gate_ids}
        endpoint_regions=[]
        for pose in [query.start,query.goal]:
            rc=self.world.map.world_to_cell(*pose[:2])
            if rc is None or self.subregions[rc]==0:raise ValueError('V3_ENDPOINT_HAS_NO_SEMANTIC_REGION')
            endpoint_regions.append(int(self.subregions[rc]))
        for rid in sorted(set(endpoint_regions)):
            if rid not in self.parking_adjacencies:continue
            for other,center in self.parking_adjacencies[rid].items():
                pid=100000+rid*1000+other
                self.gates[pid]={'regions':(rid,other),'temporary':True,'outline_edge_id':None,'contact_band_cells':self.parking_contacts[rid,other].tolist()}
                self._add_gate(pid,rid,other,center,True)
        r=super().plan(query,**kwargs)
        r.diagnostics.update(approved_outline_unchanged=True,outline_nodes=len(self.graph['nodes']),outline_edges=len(self.graph['edges']),gate_midpoint_is_not_waypoint=True,temporary_parking_attachments=sum(v['temporary'] for v in self.gates.values()),source_graph_hash=self.source_hash)
        return r

    def fine_mask(self,route):
        """Body corridor uses the immutable static layer; overlays stay removable."""
        key=tuple(sorted(set(route.region_ids)))
        if key in self.mask_cache:return self.mask_cache[key]
        import cv2
        selected=np.isin(self.subregions,key)
        radius=math.ceil((self.world.vehicle[2]+math.hypot(*self.world.vehicle[:2])+.05+math.sqrt(2)*self.world.map.resolution/2)/self.world.map.resolution)
        r,c=np.nonzero(selected)
        if not len(r):raise ValueError('EMPTY_SEMANTIC_CORRIDOR')
        r0=max(0,int(r.min())-radius);r1=min(selected.shape[0],int(r.max())+radius+1);c0=max(0,int(c.min())-radius);c1=min(selected.shape[1],int(c.max())+radius+1)
        local=cv2.dilate(selected[r0:r1,c0:c1].astype(np.uint8),cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,)*2))>0
        mask=np.zeros(selected.shape,bool);mask[r0:r1,c0:c1]=local&~self.static_hard[r0:r1,c0:c1]
        if len(self.mask_cache)>=4:self.mask_cache.popitem(last=False)
        self.mask_cache[key]=mask
        return mask

    def channel_plan(self,route):
        """L1's output is an ordered region-and-gate contract, never a boundary path.
        Center and body masks are derived downstream in metres from this contract.
        """
        gates=[]
        for i,nid in enumerate(route.interface_ids):
            n=self.nodes[nid];gate=self.gates[n.portal]
            gates.append({'stage':i,'gate_id':n.portal,'from_region':n.from_region,'to_region':n.to_region,'outline_edge_id':gate['outline_edge_id'],'temporary':gate['temporary'],'geometry':gate.get('geometry'),'contact_band_cells':gate.get('contact_band_cells'),'required_crossing':'from_region_to_region_in_order','anchor_is_mandatory':False})
            if gate.get('contact_band_cells'):
                rc=np.asarray(gate['contact_band_cells'],dtype=np.int32)
                gates[-1]['contact_band_world_xy']=self.world.world_points(rc[:,0],rc[:,1]).tolist()
            gates[-1]['geometry_units']='world metres; temporary contact band is a set of fine-cell centers'
        stages=[]
        for i,rid in enumerate(route.region_ids):
            stages.append({'stage':i,'region_id':rid,'name':self.regions[rid]['name'],'kind':self.regions[rid]['kind'],'entry_gate':None if i==0 else gates[i-1]['gate_id'],'exit_gate':None if i==len(route.region_ids)-1 else gates[i]['gate_id']})
            stages[-1]['semantic_feature_id']=self.parent_semantics[rid]
            stages[-1]['geometry_ref']='inputs/semantic.json feature by semantic_feature_id; no subregion split'
        return {'schema':'3YD-V3-r0-channel-plan-v1','frame_id':'map','source_outline_hash':self.source_hash,'semantic_hash':self.world.semantic.semantic_map_hash,'route_signature':route.signature,'start':list(route.start),'goal':list(route.goal),'stages':stages,'gates':gates,'center_range':{'definition':'original semantic regions, preserving the ordered stage and gate obligations','region_ids':list(route.region_ids),'not_sufficient':'unordered union alone does not enforce passage order'},'body_check_range':{'definition':'downstream vehicle envelope around center range, intersected with current fine free space','not_center_permission':True},'resolution_m':self.world.map.resolution,'origin':self.world.map.origin,'l2_contract':'select free crossing locations anywhere on each gate; D* reference plus ordered-gate validation','l3_contract':'fine footprint/kinematics plus ordered-gate validation; failed validation cannot dispatch','status':'ordered-stage L2 plus independent L3 channel audit in this candidate; no continuous vehicle-space completeness claim'}

    def center_mask(self,route):
        """Semantic center range only; callers apply current occupancy separately."""
        return np.isin(self.subregions,route.region_ids)
