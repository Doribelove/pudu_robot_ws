"""Sparse semantic interfaces; no precomputed pairwise vehicle trajectories."""
from dataclasses import dataclass, asdict, field
from pathlib import Path
from collections import defaultdict, OrderedDict
import heapq, itertools, json, math, time
import numpy as np
import cv2
from .semantic_graph import decompose, SemanticPoseGraph, InterfacePose
from .semantic_world import digest, json_write
from arena_evaluation.semantic_map import canonical_hash

@dataclass
class TopologyRoute:
    region_ids: tuple
    interface_ids: tuple
    semantic_ids: tuple
    start: tuple
    goal: tuple
    signature: str
    diagnostics: dict
    binding: dict = field(default_factory=dict)

class SemanticTopology:
    SCHEMA = '3yd-static-semantic-interfaces-v2'
    def __init__(self, world, cache_root):
        tick=time.monotonic(); self.world=world; self.cache_root=Path(cache_root)
        self.key=canonical_hash([self.SCHEMA, world.key, 'one-sample-per-connected-portal'])
        self.nodes={}; self.region_nodes=defaultdict(lambda:{'in':[], 'out':[]})
        self.samples=1; self.query_cache=OrderedDict(); self.mask_cache=OrderedDict()
        root=self.cache_root/self.key; self.cache_hit=(root/'graph.json').exists()
        if self.cache_hit:
            meta=json.loads((root/'graph.json').read_text())
            content_hash=meta.pop('content_hash',None)
            # JSON object keys become strings on reload. Restore the original
            # integer-key type before verifying the build-time canonical hash.
            # Numeric/lexical ordering differs once region IDs reach 10.
            hash_meta={**meta,'parents':{int(k):v for k,v in meta['parents'].items()}}
            if content_hash!=canonical_hash(hash_meta):raise ValueError('TOPOLOGY_CACHE_MISMATCH')
            with np.load(root/'regions.npz',allow_pickle=False) as d:self.subregions=d['subregions']
            if meta['key']!=self.key or digest(self.subregions)!=meta['regions_hash']:raise ValueError('TOPOLOGY_CACHE_MISMATCH')
            self.parent_semantics={int(k):v for k,v in meta['parents'].items()}
            for item in meta['nodes']:
                item['pose']=tuple(item['pose']); n=InterfacePose(**item); self.nodes[n.id]=n
                self.region_nodes[n.to_region]['in'].append(n.id); self.region_nodes[n.from_region]['out'].append(n.id)
            self.portal_count=meta['portal_count']
        else:
            self.subregions,self.parent_semantics=decompose(world)
            SemanticPoseGraph._make_portals(self)
            root.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(root/'regions.npz',subregions=self.subregions)
            meta={'schema':self.SCHEMA,'key':self.key,'parents':self.parent_semantics,
                'nodes':[asdict(n) for n in self.nodes.values()], 'portal_count':self.portal_count,
                'regions_hash':digest(self.subregions),'kinematic_edges_precomputed':0}
            json_write(root/'graph.json',{**meta,'content_hash':canonical_hash(meta)})
        self.preparation_ms=(time.monotonic()-tick)*1000
        self.edges_count=len(self.nodes)

    def _advance(self, history, semantic):
        seq=list(history)
        if not seq or seq[-1]!=semantic:seq.append(semantic)
        if len(seq)>=3:
            names=[self.world.features[k].semantic_id if k in self.world.features else 'unlabelled' for k in seq[-3:]]
            if names in self.world.rules.get('forbidden_transitions',[]) or seq[-3:] in self.world.rules.get('forbidden_transitions',[]):return None
        return tuple(seq[-2:])

    def _direction_ok(self, region, start, end):
        k=self.parent_semantics[region]
        name=self.world.features[k].semantic_id if k in self.world.features else 'unlabelled'
        for rule in self.world.rules.get('one_way',[]):
            if rule['semantic_id']==name:
                yaw=float(rule['yaw'])
                if (end[0]-start[0])*math.cos(yaw)+(end[1]-start[1])*math.sin(yaw)<-1e-6:return False
        return True

    def plan(self, query, *, timeout=5., excluded_interfaces=()):
        tick=time.monotonic(); key=canonical_hash([self.key,query.start,query.goal,sorted(excluded_interfaces)])
        if key in self.query_cache:
            old=self.query_cache[key]
            return TopologyRoute(old.region_ids,old.interface_ids,old.semantic_ids,old.start,old.goal,old.signature,
                {**old.diagnostics,'cache_hit':True,'l1_ms':(time.monotonic()-tick)*1000},old.binding)
        cells=[]
        for pose in [query.start,query.goal]:
            rc=self.world.map.world_to_cell(*pose[:2])
            if rc is None or not self.world.safe[rc]:raise ValueError('TOPOLOGY_ENDPOINT_NOT_SAFE')
            cells.append(rc)
        start_region,goal_region=[int(self.subregions[rc]) for rc in cells]
        first=(-1,(self.parent_semantics[start_region],)); dist={first:0.}; previous={}; serial=itertools.count()
        queue=[(math.dist(query.start[:2],query.goal[:2]),0.,next(serial),first)]; expanded=0; final=None
        while queue:
            if time.monotonic()-tick>timeout:raise TimeoutError('TOPOLOGY_TIMEOUT')
            _,g,_,state=heapq.heappop(queue)
            if g!=dist.get(state):continue
            node,history=state
            if node==-2:final=state;break
            region=start_region if node==-1 else self.nodes[node].to_region
            pose=query.start if node==-1 else self.nodes[node].pose
            expanded+=1
            choices=[self.nodes[k] for k in self.region_nodes[region]['out'] if k not in excluded_interfaces]
            if region==goal_region:choices.append(None)
            for target in choices:
                point=query.goal if target is None else target.pose
                if not self._direction_ok(region,pose,point):continue
                next_hist=history if target is None else self._advance(history,target.semantic_to)
                if next_hist is None:continue
                nxt=(-2 if target is None else target.id,next_hist)
                ng=g+max(.001,math.dist(pose[:2],point[:2]))
                if ng<dist.get(nxt,math.inf):
                    dist[nxt]=ng;previous[nxt]=state
                    heapq.heappush(queue,(ng+(0. if target is None else math.dist(point[:2],query.goal[:2])),ng,next(serial),nxt))
        if final is None:raise RuntimeError('TOPOLOGY_NO_ROUTE')
        interfaces=[]; current=final
        while current!=first:
            if current[0]>=0:interfaces.append(current[0])
            current=previous[current]
        interfaces.reverse(); regions=[start_region]+[self.nodes[k].to_region for k in interfaces]
        semantic=[self.parent_semantics[r] for r in regions]
        result=TopologyRoute(tuple(regions),tuple(interfaces),tuple(semantic),tuple(query.start),tuple(query.goal),key,
            {'l1_ms':(time.monotonic()-tick)*1000,'cache_hit':False,'expanded':expanded,'candidate_only':True,
             'kinematic_validation':'deferred_to_fixed_L3_and_independent_audit'},
            {'world_key':self.world.key,'topology_key':self.key,**self.world.binding})
        if len(self.query_cache)>=32:self.query_cache.popitem(last=False)
        self.query_cache[key]=result
        return result

    def fine_mask(self, route):
        """Physical region corridor, independent of L2 resolution or preference."""
        key=tuple(sorted(set(route.region_ids)))
        if key in self.mask_cache:return self.mask_cache[key]
        selected=np.isin(self.subregions,key)
        rows,cols=np.nonzero(selected)
        # Candidate interfaces need a turning envelope, in addition to body
        # extent and the original grid safety allowance. Derive it in metres
        # from the unchanged vehicle rather than tune it per route.
        half_length,half_width,min_radius=self.world.vehicle
        margin=min_radius+math.hypot(half_length,half_width)+.05+math.sqrt(2)*self.world.map.resolution/2
        radius=int(math.ceil(margin/self.world.map.resolution))
        r0=max(0,int(rows.min())-radius);r1=min(selected.shape[0],int(rows.max())+radius+1)
        c0=max(0,int(cols.min())-radius);c1=min(selected.shape[1],int(cols.max())+radius+1)
        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,)*2)
        local=cv2.dilate(selected[r0:r1,c0:c1].astype(np.uint8),kernel)>0
        local&=~self.world.hard[r0:r1,c0:c1]
        # Semantic interfaces are center-route boundaries, not physical walls.
        # Preserve the restored body envelope across their boundary; static
        # obstacles remain hard and final center-path rule checks still apply.
        mask=np.zeros(selected.shape,bool);mask[r0:r1,c0:c1]=local
        if len(self.mask_cache)>=4:self.mask_cache.popitem(last=False)
        self.mask_cache[key]=mask
        return mask
