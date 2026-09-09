"""Bounded-memory tile components, skeleton polylines and exact seam portals.

The coarse graph is a quotient of 0.05m free-space components (one coarse
node per tile/component), not a collision map. Portal certificates name actual
adjacent free cells. Each tile skeleton is defined on its bounded core; halo
covers obstacle dilation, while explicit portal grafts remove dependence on
unbounded global thinning. No full-resolution global label/EDT/skeleton exists.
"""
from __future__ import annotations
from collections import OrderedDict
from dataclasses import asdict, dataclass, replace
import hashlib
import heapq
import json
import math
import os
from pathlib import Path
import resource
import time
from types import SimpleNamespace

import cv2
import numpy as np
from scipy import ndimage
from skimage.graph import MCP_Geometric
from skimage.morphology import skeletonize

from .topology import TopologyGraph,TopologyNode,TopologyEdge,_footprint_kernel,map_input_hash
from .reachable_endpoint_r3 import digest

VERSION='tiled_component_portal_skeleton_r3_v1'
NEIGHBORS=tuple((r,c) for r in (-1,0,1) for c in (-1,0,1) if r or c)


class TopologyDeadlineExceeded(TimeoutError):
    """The query may reuse completed tiles, but cannot proceed to planning."""


def check_deadline(deadline, stage):
    if deadline is not None and time.monotonic() >= deadline:
        raise TopologyDeadlineExceeded(stage)


@dataclass(frozen=True)
class TileConfig:
    tile_cells:int=512
    halo_cells:int=32
    padding_m:float=.05
    safety_margin_m:float=.05
    memory_tiles:int=4
    algorithm:str=VERSION


def atomic_json(path,payload):
    path=Path(path);tmp=path.with_name(path.name+f'.{os.getpid()}.tmp')
    with tmp.open('x') as f:json.dump(payload,f,sort_keys=True,separators=(',',':'));f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)


def read_json_or_invalid(path):
    try:return json.loads(Path(path).read_text())
    except (OSError,ValueError):return {}


def file_hash(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def compress_skeleton(skeleton,distance,map_,offset=(0,0),forced=()):
    """Trace every undirected pixel link once, retaining parallel edges."""
    pixels={tuple(map(int,p)) for p in np.argwhere(skeleton)}
    if not pixels:return TopologyGraph(),{}
    adj={p:sorted((p[0]+dr,p[1]+dc) for dr,dc in NEIGHBORS if (p[0]+dr,p[1]+dc) in pixels) for p in sorted(pixels)}
    keys={p for p,n in adj.items() if len(n)!=2}|(set(forced)&pixels)
    _,labels=cv2.connectedComponents(skeleton.astype(np.uint8),connectivity=8)
    # Every pure loop needs an anchor; self-edges are retained.
    for label in sorted(set(int(labels[p]) for p in pixels)):
        group=[p for p in pixels if labels[p]==label]
        if not any(p in keys for p in group):keys.add(min(group))
    nodes=[];ids={}
    for p in sorted(keys):
        ids[p]=len(nodes);r,c=p;x,y=map_.cell_to_world((r+offset[0],c+offset[1]));clear=float(distance[p])
        nodes.append(TopologyNode(len(nodes),x,y,c+offset[1],r+offset[0],len(adj[p]),clear,2*clear,int(labels[p])))
    edges=[];seen=set()
    for p in sorted(keys):
        for first in adj[p]:
            link=tuple(sorted((p,first)))
            if link in seen:continue
            seen.add(link);chain=[p,first];prev,cur=p,first
            while cur not in keys:
                next_=next(v for v in adj[cur] if v!=prev)
                seen.add(tuple(sorted((cur,next_))));chain.append(next_);prev,cur=cur,next_
            line=[list(map_.cell_to_world((r+offset[0],c+offset[1]))) for r,c in chain]
            length=sum(math.dist(a,b) for a,b in zip(line,line[1:]));clear=[float(distance[v]) for v in chain]
            edges.append(TopologyEdge(len(edges),ids[p],ids[cur],length,min(clear),float(np.mean(clear)),2*min(clear),len(chain),line))
    return TopologyGraph(nodes,edges),ids


class TiledTopology:
    def __init__(self,map_,footprint,cache,config=TileConfig()):
        self.map=map_;self.footprint=footprint;self.config=config
        kernel=_footprint_kernel(map_.resolution,footprint,config.padding_m,config.safety_margin_m)
        if config.halo_cells<kernel.shape[0]//2+2:raise ValueError('halo does not cover footprint and dilation boundary')
        if config.tile_cells<16 or config.memory_tiles<1:raise ValueError('invalid tile/memory configuration')
        self.kernel=kernel
        self.binding={'map_hash':map_input_hash(map_.yaml_path,map_.image_path),'resolution':map_.resolution,
                      'footprint':footprint,'config':asdict(config),'implementation_sha256':file_hash(__file__),'origin':map_.origin,'shape':[map_.height,map_.width]}
        self.key=digest(self.binding);self.cache=Path(cache)/self.key;self.cache.mkdir(parents=True,exist_ok=True)
        self.rows=math.ceil(map_.height/config.tile_cells);self.cols=math.ceil(map_.width/config.tile_cells)
        self.tiles=[(r,c) for r in range(self.rows) for c in range(self.cols)];self.lru=OrderedDict()
        self.coarse={};self.portals=[];self.portal_by_tile={t:[] for t in self.tiles};self.graph_lru=OrderedDict();self.refined_tiles=set()
        self.stats={'tile_count':len(self.tiles),'tile_completed_count':0,'tile_cache_hit_count':0,
                    'tile_cache_miss_count':0,'coarse_graph_build_ms':0.,'tile_refine_wall_ms':0.,
                    'topology_load_wall_ms':0.}

    def bounds(self,t):
        r,c=t;s=self.config.tile_cells
        return r*s,min((r+1)*s,self.map.height),c*s,min((c+1)*s,self.map.width)

    def _path(self,t):return self.cache/f'tile_{t[0]:04d}_{t[1]:04d}'

    def tile(self,t):
        if t in self.lru:
            value=self.lru.pop(t);self.lru[t]=value;return value
        load_started=time.monotonic()
        root=self._path(t);data=root.with_suffix('.npz');meta=root.with_suffix('.json')
        if data.exists() and meta.exists():
            info=read_json_or_invalid(meta)
            if info.get('binding')==self.key and info.get('sha256')==file_hash(data):
                with np.load(data,allow_pickle=False) as z:value={k:z[k] for k in z.files}
                self.stats['tile_cache_hit_count']+=1
                self.stats['topology_load_wall_ms']+=(time.monotonic()-load_started)*1000
            else:value=self._build_tile(t,data,meta)
        else:value=self._build_tile(t,data,meta)
        self.lru[t]=value
        while len(self.lru)>self.config.memory_tiles:self.lru.popitem(last=False)
        return value

    def _build_tile(self,t,data,meta):
        self.stats['tile_cache_miss_count']+=1;r0,r1,c0,c1=self.bounds(t);h=self.config.halo_cells
        er0,er1=max(0,r0-h),min(self.map.height,r1+h);ec0,ec1=max(0,c0-h),min(self.map.width,c1+h)
        raw=(self.map.occupancy[er0:er1,ec0:ec1]!=0).astype(np.uint8)
        inflated=cv2.dilate(raw,self.kernel,borderType=cv2.BORDER_CONSTANT,borderValue=1)
        free=(inflated[r0-er0:r1-er0,c0-ec0:c1-ec0]==0)
        _,labels=cv2.connectedComponents(free.astype(np.uint8),connectivity=8)
        # Unknown space beyond the loaded halo is an obstacle for this field.
        # This is a finite conservative lower bound, never global clearance.
        padded=np.pad((raw==0).astype(np.uint8),1,constant_values=0)
        distance=cv2.distanceTransform(padded,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)[1:-1,1:-1]*self.map.resolution
        distance=distance[r0-er0:r1-er0,c0-ec0:c1-ec0]
        value={'free':free,'labels':labels,'distance':distance.astype(np.float32)}
        tmp=data.with_name(data.name+f'.{os.getpid()}.tmp')
        with tmp.open('xb') as f:np.savez_compressed(f,**value);f.flush();os.fsync(f.fileno())
        os.replace(tmp,data);atomic_json(meta,{'binding':self.key,'sha256':file_hash(data),'components':int(labels.max()),'distance_semantics':'halo_boundary_obstacle_lower_bound'})
        return value

    def build_coarse(self):
        begin=time.monotonic()
        self.coarse={};self.portals=[];self.portal_by_tile={t:[] for t in self.tiles}
        for t in self.tiles:
            labels=self.tile(t)['labels']
            for label in range(1,int(labels.max())+1):self.coarse[(t[0],t[1],label)]=[]
        # Actual adjacent seam cells, grouped by the component pair. A portal
        # per run is enough; runs cannot jump across an obstacle.
        for t in self.tiles:
            for other,axis in [((t[0],t[1]+1),1),((t[0]+1,t[1]),0)]:
                if other not in self.portal_by_tile:continue
                a=self.tile(t)['labels'];b=self.tile(other)['labels'];pairs=[]
                n=a.shape[0] if axis else a.shape[1]
                for k in range(n):
                    for delta in (-1,0,1):
                        j=k+delta
                        if not 0<=j<(b.shape[0] if axis else b.shape[1]):continue
                        p=(k,a.shape[1]-1) if axis else (a.shape[0]-1,k)
                        q=(j,0) if axis else (0,j)
                        if a[p] and b[q]:pairs.append((int(a[p]),int(b[q]),k,j,p,q))
                grouped={}
                for row in pairs:grouped.setdefault(row[:2],[]).append(row)
                for pair,values in sorted(grouped.items()):
                    runs=[];run=[];prev=-9
                    for row in sorted(values,key=lambda v:(v[2],v[3])):
                        if row[2]>prev+1 and run:runs.append(run);run=[]
                        run.append(row);prev=row[2]
                    if run:runs.append(run)
                    for run in runs:
                        v=run[len(run)//2];self._portal(t,other,v[4],v[5],pair)
            # Diagonal tile corner adjacency is part of frozen 8-connectivity.
            for dc in (-1,1):
                other=(t[0]+1,t[1]+dc)
                if other not in self.portal_by_tile:continue
                a=self.tile(t)['labels'];b=self.tile(other)['labels']
                p=(a.shape[0]-1,0 if dc<0 else a.shape[1]-1);q=(0,b.shape[1]-1 if dc<0 else 0)
                if a[p] and b[q]:self._portal(t,other,p,q,(int(a[p]),int(b[q])))
        self.stats.update(coarse_graph_build_ms=(time.monotonic()-begin)*1000,
                          portal_count=len(self.portals),seam_count=len({tuple(v['tiles'][0]+v['tiles'][1]) for v in self.portals}))
        self._expected_seam_hash=digest(self.portals)
        certificate={'binding':self.key,'portals':self.portals,'hash':digest(self.portals),'validated':self.validate_seams()}
        atomic_json(self.cache/'seams.json',certificate)
        return certificate

    def _portal(self,t,u,p,q,pair):
        a=(*t,pair[0]);b=(*u,pair[1]);i=len(self.portals)
        r0,_,c0,_=self.bounds(t);r1,_,c1,_=self.bounds(u)
        gc=[p[0]+r0,p[1]+c0];gd=[q[0]+r1,q[1]+c1]
        entry={'id':i,'tiles':[list(t),list(u)],'cells':[gc,gd],'components':[list(a),list(b)]}
        self.portals.append(entry);self.portal_by_tile[t].append((i,p));self.portal_by_tile[u].append((i,q))
        self.coarse[a].append((b,i));self.coarse[b].append((a,i))

    def validate_seams(self):
        if getattr(self,'_expected_seam_hash',None)!=digest(self.portals):
            raise ValueError('missing, reordered or mutated seam portal')
        seen=set()
        for v in self.portals:
            a,b=v['cells'];key=tuple(a+b)
            if key in seen:raise ValueError('duplicate seam edge')
            seen.add(key)
            if max(abs(a[0]-b[0]),abs(a[1]-b[1]))!=1:raise ValueError('false nonadjacent seam')
            for t,p,component in zip(v['tiles'],v['cells'],v['components']):
                r0,_,c0,_=self.bounds(tuple(t));label=self.tile(tuple(t))['labels'][p[0]-r0,p[1]-c0]
                if label==0 or int(label)!=component[2]:raise ValueError('false component seam')
        return True

    def candidate_tiles(self,start,goal,endpoint_radius_m=25.,*,deadline=None):
        """Keep alternative endpoint components until SE(2) certification.

        Conservative all-heading erosion can split a narrow, pose-reachable
        endpoint neck. Selecting just its nearest component is therefore an
        invalid early rejection. Coarse candidates are a discovery set only.
        """
        check_deadline(deadline,'coarse_query_start')
        def candidates(p):
            cell=self.map.world_to_cell(*p[:2])
            if cell is None:return {}
            radius=math.ceil(endpoint_radius_m/self.map.resolution);result={}
            for tr in range(max(0,(cell[0]-radius)//self.config.tile_cells),min(self.rows,(cell[0]+radius)//self.config.tile_cells+1)):
                for tc in range(max(0,(cell[1]-radius)//self.config.tile_cells),min(self.cols,(cell[1]+radius)//self.config.tile_cells+1)):
                    check_deadline(deadline,'coarse_endpoint_tile')
                    t=(tr,tc);r0,_,c0,_=self.bounds(t);labels=self.tile(t)['labels']
                    rr,cc=np.ogrid[:labels.shape[0],:labels.shape[1]]
                    distance=(rr+r0-cell[0])**2+(cc+c0-cell[1])**2
                    valid=(distance<=radius**2)&(labels>0)
                    for label in sorted(map(int,np.unique(labels[valid]))):
                        result[(*t,label)]=math.sqrt(float(distance[valid&(labels==label)].min()))/self.config.tile_cells
            return result
        starts,goals=candidates(start),candidates(goal)
        check_deadline(deadline,'coarse_endpoint_candidates')
        if not starts or not goals:return []
        parent={node:None for node in starts};dist=dict(starts)
        queue=[(cost,node) for node,cost in starts.items()];heapq.heapify(queue)
        while queue:
            check_deadline(deadline,'coarse_query_search')
            cost,node=heapq.heappop(queue)
            if cost!=dist[node]:continue
            for nxt,_ in sorted(self.coarse.get(node,())):
                nc=cost+1.
                if nc<dist.get(nxt,math.inf)-1e-12:
                    dist[nxt]=nc;parent[nxt]=node;heapq.heappush(queue,(nc,nxt))
        targets=sorted((dist[g]+cost,g) for g,cost in goals.items() if g in dist)
        if not targets:return []
        selected={node[:2] for node in starts}|{node[:2] for node in goals}
        route=set()
        for _,target in targets[:8]:
            cur=target
            while cur is not None:route.add(cur[:2]);cur=parent[cur]
        selected.update(route)
        for r,c in route:
            for dr,dc in NEIGHBORS:
                if (r+dr,c+dc) in self.portal_by_tile:selected.add((r+dr,c+dc))
        check_deadline(deadline,'coarse_query_complete')
        return sorted(selected)

    def refine(self,t,*,deadline=None):
        check_deadline(deadline,'tile_refine_start')
        if t in self.graph_lru:
            graph,ids=self.graph_lru.pop(t);self.graph_lru[t]=(graph,ids);return graph,ids
        begin=time.monotonic();root=self._path(t);path=root.with_suffix('.graph.json')
        portal_binding=digest(self.portal_by_tile[t]);payload=None
        if path.exists():
            p=read_json_or_invalid(path)
            skpath=root.with_suffix('.skeleton.npz')
            if (p.get('binding')==self.key and p.get('portal_binding')==portal_binding
                and p.get('hash')==digest(p.get('graph')) and skpath.exists()
                and p.get('skeleton_hash')==file_hash(skpath)):payload=p
        if payload is None:
            data=self.tile(t);free=data['free'];skeleton=skeletonize(free)
            check_deadline(deadline,'tile_skeleton_complete')
            portals=[p for _,p in self.portal_by_tile[t]]
            # Multi-source geodesic grafts stay wholly in full-footprint free
            # space, even when nearest Euclidean skeleton lies behind a wall.
            if portals and np.any(skeleton):
                mcp=MCP_Geometric(np.where(free,1.,np.inf),fully_connected=True)
                costs,_=mcp.find_costs(starts=np.argwhere(skeleton),ends=portals)
                for p in portals:
                    check_deadline(deadline,'tile_portal_graft')
                    if not math.isfinite(float(costs[p])):raise ValueError('broken tile portal')
                    for cell in mcp.traceback(p):skeleton[cell]=True
            graph,ids=compress_skeleton(skeleton,data['distance'],self.map,(self.bounds(t)[0],self.bounds(t)[2]),portals)
            check_deadline(deadline,'tile_compression_complete')
            g={'nodes':[asdict(n) for n in graph.nodes],'edges':[asdict(e) for e in graph.edges],
               'portals':{str(i):ids[p] for i,p in self.portal_by_tile[t]}}
            payload={'binding':self.key,'portal_binding':portal_binding,'graph':g,'hash':digest(g)}
            # Tile artifact includes the exact skeleton used by compression.
            skpath=root.with_suffix('.skeleton.npz');tmp=skpath.with_suffix('.tmp')
            with tmp.open('wb') as f:np.savez_compressed(f,skeleton=skeleton)
            os.replace(tmp,skpath)
            payload['skeleton_hash']=file_hash(skpath)
            atomic_json(path,payload)
        self.refined_tiles.add(t);self.stats['tile_completed_count']=len(self.refined_tiles)
        g=payload['graph'];graph=TopologyGraph([TopologyNode(**n) for n in g['nodes']],[TopologyEdge(**e) for e in g['edges']]);ids={int(k):v for k,v in g['portals'].items()}
        self.stats['tile_refine_wall_ms']+=(time.monotonic()-begin)*1000
        self.graph_lru[t]=(graph,ids)
        while len(self.graph_lru)>self.config.memory_tiles:self.graph_lru.popitem(last=False)
        return graph,ids

    def artifact(self,tiles,*,deadline=None):
        check_deadline(deadline,'query_graph_start')
        nodes=[];edges=[];portals={};component_parent={}
        def root(a):
            component_parent.setdefault(a,a)
            while component_parent[a]!=a:
                component_parent[a]=component_parent[component_parent[a]];a=component_parent[a]
            return a
        selected=set(tiles)
        for t in sorted(selected):
            check_deadline(deadline,'query_graph_tile')
            graph,ids=self.refine(t,deadline=deadline);offset=len(nodes)
            for n in graph.nodes:
                v=replace(n,node_id=n.node_id+offset);nodes.append(v);root(v.node_id)
            for e in graph.edges:
                # Copy the mutable polyline containers, not each immutable scalar
                # through dataclasses.asdict/deepcopy. Query mutation stays local.
                v=replace(e,edge_id=len(edges),source=e.source+offset,target=e.target+offset,
                          polyline=[list(p) for p in e.polyline]);edges.append(v)
                a,b=root(v.source),root(v.target);component_parent[max(a,b)]=min(a,b)
            for pid,nid in ids.items():portals[(t,pid)]=nid+offset
        for p in self.portals:
            check_deadline(deadline,'query_graph_seam')
            t,u=map(tuple,p['tiles'])
            if t not in selected or u not in selected:continue
            a,b=portals[(t,p['id'])],portals[(u,p['id'])];pa,pb=nodes[a],nodes[b]
            length=math.hypot(pa.x-pb.x,pa.y-pb.y);clear=min(pa.clearance_m,pb.clearance_m)
            edges.append(TopologyEdge(len(edges),a,b,length,clear,clear,2*clear,2,[[pa.x,pa.y],[pb.x,pb.y]]))
            ra,rb=root(a),root(b);component_parent[max(ra,rb)]=min(ra,rb)
        components={v:i+1 for i,v in enumerate(sorted({root(n.node_id) for n in nodes}))}
        for n in nodes:n.component_id=components[root(n.node_id)]
        check_deadline(deadline,'query_graph_complete')
        return SimpleNamespace(graph=TopologyGraph(nodes,edges),hospital_map=self.map,
                               metadata={'map_sha256':self.binding['map_hash'],'binding':self.key,'tiles':sorted(selected),
                                         'coarse_failure':'ENDPOINT_COMPONENT_MISMATCH' if not selected else ''})

    def build_all(self,progress=None):
        wall=time.monotonic();cpu=time.process_time();self.build_coarse()
        for i,t in enumerate(self.tiles):
            self.refine(t);self.stats['tile_completed_count']=i+1
            if progress:progress({'tile':t,**self.stats,'elapsed_s':time.monotonic()-wall,
                                  'peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024})
        # Graph totals come from bounded reads; no merged high-resolution graph
        # is required to certify a completed large-map cache.
        n=e=0;hashes=[]
        for t in self.tiles:
            p=self._path(t).with_suffix('.graph.json');g=json.loads(p.read_text());n+=len(g['graph']['nodes']);e+=len(g['graph']['edges']);hashes.append(g['hash'])
        self.stats.update(topology_node_count=n,topology_edge_count=e+len(self.portals),
                          topology_build_wall_ms=(time.monotonic()-wall)*1000,
                          topology_build_cpu_ms=(time.process_time()-cpu)*1000,
                          topology_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                          topology_cache_bytes=sum(p.stat().st_size for p in self.cache.iterdir() if p.is_file()),
                          topology_hash=digest(hashes),seam_hash=digest(self.portals),seam_valid=self.validate_seams())
        atomic_json(self.cache/'complete.json',{'binding':self.binding,'stats':self.stats})
        return self.stats


def load_map_bounded(yaml_path,cache):
    """Decode PGM into disk-backed occupancy in bounded row chunks.

    The frozen large maps use binary P5 PGM, so pixel bytes can be mapped
    directly. PNG fallback is intentionally rejected instead of silently
    allocating a 100M-cell image and multiple global distance arrays.
    """
    import yaml
    from .planner_benchmark.map_utils import HospitalMap
    path=Path(yaml_path).resolve();cfg=yaml.safe_load(path.read_text());image=(path.parent/cfg['image']).resolve()
    with image.open('rb') as f:
        tokens=[]
        while len(tokens)<4:
            line=f.readline()
            if not line:raise ValueError('truncated PGM')
            if line.startswith(b'#'):continue
            tokens.extend(line.split())
        if tokens[0]!=b'P5' or int(tokens[3])!=255:raise ValueError('bounded loader requires P5 8-bit PGM')
        width,height=map(int,tokens[1:3]);offset=f.tell()
    if image.stat().st_size-offset!=width*height:raise ValueError('PGM payload shape mismatch')
    source=np.memmap(image,dtype=np.uint8,mode='r',offset=offset,shape=(height,width))
    directory=Path(cache)/('map_'+map_input_hash(path,image));directory.mkdir(parents=True,exist_ok=True)
    op=directory/'occupancy.bin';dp=directory/'distance.bin'
    binding={'map_hash':map_input_hash(path,image),'shape':[height,width],
             'loader_sha256':file_hash(__file__),'distance_semantics':'unused_precompute_placeholder'}
    metadata=read_json_or_invalid(directory/'complete.json')
    valid=(all(metadata.get(k)==v for k,v in binding.items()) and op.exists() and dp.exists()
           and op.stat().st_size==height*width and dp.stat().st_size==height*width*4
           and metadata.get('occupancy_sha256')==file_hash(op)
           and metadata.get('distance_sha256')==file_hash(dp))
    if not valid:
        suffix='.'+__import__('uuid').uuid4().hex+'.tmp'
        ot=op.with_name(op.name+suffix);dt=dp.with_name(dp.name+suffix)
        occupancy=np.memmap(ot,dtype=np.int8,mode='w+',shape=(height,width))
        distance=np.memmap(dt,dtype=np.float32,mode='w+',shape=(height,width))
        for r in range(0,height,256):
            probability=np.asarray(source[r:r+256],np.float32)/255
            if not cfg.get('negate',0):probability=1-probability
            occupancy[r:r+256]=np.where(probability>cfg.get('occupied_thresh',.65),100,
                                       np.where(probability<cfg.get('free_thresh',.196),0,-1))
            distance[r:r+256]=0
        occupancy.flush();distance.flush();del occupancy,distance
        os.replace(ot,op);os.replace(dt,dp)
        atomic_json(directory/'complete.json',{**binding,'occupancy_sha256':file_hash(op),
                                              'distance_sha256':file_hash(dp)})
    occupancy=np.memmap(op,dtype=np.int8,mode='r',shape=(height,width))
    distance=np.memmap(dp,dtype=np.float32,mode='r',shape=(height,width))
    return HospitalMap(path,image,float(cfg['resolution']),tuple(cfg['origin']),width,height,occupancy,distance)
