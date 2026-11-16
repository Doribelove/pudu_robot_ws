"""Semantic-region sweep decomposition and independently verified pose graph.

No skeleton or legacy topology is an input. Row runs stay in one subregion
until an obstacle/semantic split or merge changes connectivity. Raster rows
are processed at original resolution; they do not become graph nodes.
"""
from __future__ import annotations
from dataclasses import dataclass,asdict
from collections import defaultdict
from pathlib import Path
import heapq,itertools,math,time,json,os
import numpy as np
import cv2
from scipy import ndimage
from arena_evaluation.semantic_map import canonical_hash
from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
from .semantic_world import json_write,digest,wrap,SweptChecker
from .pipeline import L1Plan

@dataclass(frozen=True)
class InterfacePose:
 id:int
 pose:tuple
 from_region:int
 to_region:int
 semantic_from:int
 semantic_to:int
 portal:int
 temporary:bool=False

@dataclass
class Connection:
 id:int
 source:int
 target:int
 region:int
 length:float
 path:np.ndarray
 evidence:dict

@dataclass
class SemanticRoute:
 plan:L1Plan
 poses:np.ndarray
 nodes:list
 edges:list
 regions:list
 diagnostics:dict

def decompose(world):
 labels=world.labels;safe=world.safe
 sub=np.zeros(labels.shape,np.int32);nextid=1;parents={};previous=[]
 for r in range(labels.shape[0]):
  vals=np.where(safe[r],labels[r]+1,0)
  bounds=np.r_[0,np.flatnonzero(vals[1:]!=vals[:-1])+1,len(vals)]
  current=[(int(a),int(b),int(vals[a])) for a,b in zip(bounds,bounds[1:]) if vals[a]>0]
  # Inbound and outbound degrees define split/merge events, not area.
  ins=[[] for _ in current];outs=[0 for _ in previous]
  j=0
  for i,(a,b,k) in enumerate(current):
   while j<len(previous) and previous[j][1]<=a:j+=1
   jj=j
   while jj<len(previous) and previous[jj][0]<b:
    pa,pb,pk,pid=previous[jj]
    if pk==k and min(b,pb)>max(a,pa):ins[i].append(jj);outs[jj]+=1
    jj+=1
  row=[]
  for i,(a,b,k) in enumerate(current):
   if len(ins[i])==1 and outs[ins[i][0]]==1:pid=previous[ins[i][0]][3]
   else:pid=nextid;nextid+=1;parents[pid]=k-1
   sub[r,a:b]=pid;row.append((a,b,k,pid))
  previous=row
 # Contract small sweep slivers into adjacent regions with the SAME source
 # semantic label. This changes only bookkeeping, never an occupied cell or
 # semantic boundary. Large branches and obstacle-separated components stay.
 counts=np.bincount(sub.ravel());links=defaultdict(lambda:defaultdict(int))
 for a,b in [(sub[:-1],sub[1:]),(sub[:,:-1],sub[:,1:])]:
  mask=(a!=b)&(a>0)&(b>0);aa=a[mask];bb=b[mask]
  for (x,y),count in zip(*np.unique(np.sort(np.column_stack([aa,bb]),axis=1),axis=0,return_counts=True)):
   x,y=int(x),int(y)
   if parents[x]==parents[y]:links[x][y]+=int(count);links[y][x]+=int(count)
 root=np.arange(nextid)
 def find(x):
  while root[x]!=x:root[x]=root[root[x]];x=int(root[x])
  return x
 for sid in sorted(parents,key=lambda x:(counts[x],x)):
  r=find(sid)
  if r!=sid or counts[r]*world.map.resolution**2>=4.:continue
  candidates=[(v,counts[find(k)],find(k)) for k,v in links[sid].items() if find(k)!=r]
  if candidates:
   target=max(candidates)[2];root[r]=target;counts[target]+=counts[r]
 mapping=np.asarray([find(i) for i in range(nextid)],np.int32);sub=mapping[sub]
 parents={int(i):parents[int(i)] for i in np.unique(sub) if i}
 return sub,parents

class SemanticPoseGraph:
 SCHEMA='semantic-interface-pose-graph-v2'
 def __init__(self,world,cache_root,*,portal_samples=3,connection_timeout=.25):
  self.world=world;self.cache_root=Path(cache_root);self.samples=int(portal_samples);self.connection_timeout=connection_timeout
  self.config={'schema':self.SCHEMA,'portal_samples':self.samples,'connection_radius':world.vehicle[2]+.001,
    'decomposition':'same-semantic-run-split-merge-contract-slivers4m2','internal_poses':'max_clearance_long_region_slices_8m_four_headings',
    'parallel_connections':'validated_dubins_words_v2','rule_hash':canonical_hash(world.rules)}
  self.key=canonical_hash([world.key,self.config]);self.nodes={};self.edges={};self.adjacency=defaultdict(list)
  self.records=[];self.region_nodes=defaultdict(lambda:{'in':[],'out':[]});self.query_cache={}
  self.build_connections_ms=0.;self.connection_cache_hits=0;self.connection_attempts=0
  root=self.cache_root/self.key
  if (root/'graph.json').exists():
   started=time.monotonic();meta=json.loads((root/'graph.json').read_text())
   if meta['key']!=self.key or meta['geometry_sha256']!=__import__('hashlib').sha256((root/'geometry.npz').read_bytes()).hexdigest():raise ValueError('semantic graph cache integrity failure')
   self._edge_payload=np.load(root/'geometry.npz',allow_pickle=False);self.subregions=self._edge_payload['subregions']
   self.parent_semantics={}
   for n in meta['nodes']:
    n['pose']=tuple(n['pose']);v=InterfacePose(**n);self.nodes[v.id]=v
    self.parent_semantics[v.from_region]=v.semantic_from;self.parent_semantics[v.to_region]=v.semantic_to
    self.region_nodes[v.to_region]['in'].append(v.id);self.region_nodes[v.from_region]['out'].append(v.id)
   for e in meta['edges']:
    v=Connection(**e,path=None);self.edges[v.id]=v;self.adjacency[v.source].append(v.id)
   self.records=meta['rejections'];self.prepared_regions=set(self.region_nodes);self.portal_count=len({n.portal for n in self.nodes.values()})
   self.preparation_ms=(time.monotonic()-started)*1000;self.cache_hit=True
   return
  started=time.monotonic();self.subregions,self.parent_semantics=decompose(world)
  self._make_portals();self._make_internal_poses();self.preparation_ms=(time.monotonic()-started)*1000
  self.cache_hit=False
 def _make_portals(self):
  sub=self.subregions;groups=defaultdict(list);normals=defaultdict(list)
  for dr,dc in [(1,0),(0,1)]:
   a=sub[:sub.shape[0]-dr or None,:sub.shape[1]-dc or None]
   b=sub[dr:,dc:]
   r,c=np.nonzero((a!=b)&(a>0)&(b>0))
   for rr,cc in zip(r,c):
    aa,bb=int(a[rr,cc]),int(b[rr,cc]);key=tuple(sorted((aa,bb)))
    sign=1 if aa==key[0] else -1
    groups[key].append((rr+dr/2,cc+dc/2));normals[key].append((sign*dc,-sign*dr))
  portal=0
  for (a,b),points in sorted(groups.items()):
   if len(points)*self.world.map.resolution<.10:continue
   pts=np.asarray(points);norm=np.asarray(normals[(a,b)])
   # Separate physically disjoint shared boundaries; parallel edges retain
   # independent portal and connection IDs.
   order=np.lexsort((pts[:,1],pts[:,0]));pts=pts[order];norm=norm[order]
   chunks=np.split(np.arange(len(pts)),np.flatnonzero(np.linalg.norm(np.diff(pts,axis=0),axis=1)>3)+1)
   for chunk in chunks:
    if len(chunk)<2:continue
    portal+=1
    for pos in sorted(set(int(round(v)) for v in np.linspace(0,len(chunk)-1,self.samples+2)[1:-1])):
     ix=chunk[pos];r,c=pts[ix];xy=self.world.world_points([r],[c])[0]
     nearby=norm[chunk[max(0,pos-3):min(len(chunk),pos+4)]].sum(axis=0)
     yaw=math.atan2(nearby[1],nearby[0])
     for fr,to,t in [(a,b,yaw),(b,a,float(wrap(yaw+math.pi)))]:
      node=InterfacePose(len(self.nodes),tuple([*xy,t]),fr,to,self.parent_semantics[fr],self.parent_semantics[to],portal)
      self.nodes[node.id]=node;self.region_nodes[to]['in'].append(node.id);self.region_nodes[fr]['out'].append(node.id)
  self.portal_count=portal
 def _make_internal_poses(self):
  # A single long Dubins curve cannot follow every bend of a concave region.
  # Sparse interior poses provide separately validated curve splices. Sites
  # come directly from the region cells, not from a skeleton/topology input.
  res=self.world.map.resolution
  counts=np.bincount(self.subregions.ravel());boxes=ndimage.find_objects(self.subregions)
  portal=self.portal_count
  for region in sorted(self.parent_semantics):
   if counts[region]*res*res<16.:continue
   box=boxes[region-1]
   if box is None:continue
   shape=tuple(s.stop-s.start for s in box)
   axis=int(shape[1]>shape[0]);span=shape[axis]*res
   if span<8.:continue
   local=self.subregions[box]==region
   clearance=ndimage.distance_transform_edt(np.pad(local,1))[1:-1,1:-1]
   # Exclude narrow remnants; they already have boundary interfaces.
   minimum_clearance=.40/res
   for lo in range(0,shape[axis],int(8./res)):
    hi=min(shape[axis],lo+int(8./res));crop=clearance[lo:hi,:] if axis==0 else clearance[:,lo:hi]
    if crop.max(initial=0)<minimum_clearance:continue
    candidates=np.argwhere(crop>=crop.max()-.05/res)
    center=np.array(crop.shape)/2;rc=candidates[np.argmin(np.linalg.norm(candidates-center,axis=1))]
    rc[axis]+=lo;rc+=np.array([box[0].start,box[1].start])
    xy=self.world.world_points([rc[0]],[rc[1]])[0];portal+=1
    semantic=self.parent_semantics[region]
    for yaw in [0.,math.pi/2,math.pi,-math.pi/2]:
     n=InterfacePose(len(self.nodes),tuple([*xy,yaw]),region,region,semantic,semantic,portal)
     self.nodes[n.id]=n;self.region_nodes[region]['in'].append(n.id);self.region_nodes[region]['out'].append(n.id)
  self.portal_count=portal
 def _rule_allows(self,a,b):
  # Rules name source semantic regions and transition triples. Missing rules
  # mean no restriction was annotated, never evidence of one-way traffic.
  middle=self.parent_semantics[a.to_region]
  triple=[a.semantic_from,middle,b.semantic_to]
  semantic_ids=[self.world.features[k].semantic_id if k in self.world.features else 'unlabelled' for k in triple]
  for rule in self.world.rules.get('forbidden_transitions',[]):
   if rule==semantic_ids or rule==triple:return False
  forbidden=self.world.rules.get('forbidden_node_pairs',[])
  if [a.id,b.id] in forbidden:return False
  for rule in self.world.rules.get('one_way',[]):
   if rule['semantic_id']==semantic_ids[1]:
    yaw=float(rule['yaw']);delta=np.asarray(b.pose[:2])-a.pose[:2]
    if np.dot(delta,[math.cos(yaw),math.sin(yaw)]) < -1e-6:return False
  return True
 def _connection(self,a,b,*,record=True,all_valid=False):
  self.connection_attempts+=1;started=time.monotonic()
  if a.to_region!=b.from_region:raise ValueError('incompatible interface subregion')
  reason='UNVERIFIED_SAMPLING';region=a.to_region
  if not self._rule_allows(a,b):
   reason='FORBIDDEN_RULE';choices=[]
  else:choices=dubins_choices(a.pose,b.pose,self.config['connection_radius'])
  paths=[]
  for word,params in choices:
   if time.monotonic()-started>self.connection_timeout:reason='UNVERIFIED_TIMEOUT';break
   if self.config['connection_radius']*sum(params)>math.dist(a.pose[:2],b.pose[:2])*3+8:continue
   edge=dubins_edge_from_parameters(a.pose,b.pose,self.config['connection_radius'],word,params)
   if edge is None:continue
   p=np.vstack([a.pose,edge.samples]);r,c=self.world.cells(p)
   valid=(r>=0)&(c>=0)&(r<self.subregions.shape[0])&(c<self.subregions.shape[1])
   if not np.all(valid):continue
   inside=self.subregions[r,c]==region
   # The center may cross the exact interface within its endpoint collar.
   # Internal subdivision lines are not hard obstacles for the vehicle body.
   near_a=np.linalg.norm(p[:,:2]-np.asarray(a.pose[:2]),axis=1)<=.10
   near_b=np.linalg.norm(p[:,:2]-np.asarray(b.pose[:2]),axis=1)<=.10
   if not np.all(inside|near_a|near_b):continue
   if not self.world.checker.check(p):continue
   legal=True
   for rule in self.world.rules.get('one_way',[]):
    semantic=self.world.features.get(self.parent_semantics[region])
    if semantic and semantic.semantic_id==rule['semantic_id']:
     projection=np.diff(p[:,:2],axis=0)@np.array([math.cos(rule['yaw']),math.sin(rule['yaw'])])
     if np.any(projection < -1e-6):legal=False;break
   if not legal:continue
   paths.append((edge.length,p,word,params))
  if paths:
   variants=[];seen=set()
   for length,p,word,params in sorted(paths,key=lambda x:(x[0],x[2])):
    path_hash=digest(p)
    if path_hash in seen:continue
    seen.add(path_hash)
    evidence={'status':'VALIDATED','word':word,'parameters':list(params),'radius_m':self.config['connection_radius'],
     'forward_only':True,'swept_body':True,'rule_hash':self.world.binding['rules'],'vehicle':self.world.binding['vehicle'],
     'map_key':self.world.key,'path_sha256':path_hash,'validation_ms':(time.monotonic()-started)*1000}
    variants.append(Connection(-1,a.id,b.id,region,length,p,evidence))
   return variants if all_valid else variants[0]
  if record:self.records.append({'source':a.id,'target':b.id,'region':region,'status':reason})
  return [] if all_valid else None
 def _admit(self,edge):
  edge.id=len(self.edges);self.edges[edge.id]=edge;self.adjacency[edge.source].append(edge.id)
 def prepare_region(self,region):
  if region in getattr(self,'prepared_regions',set()):return
  if not hasattr(self,'prepared_regions'):self.prepared_regions=set()
  nodes=self.region_nodes[region]
  for ai in nodes['in']:
   for bi in nodes['out']:
    a,b=self.nodes[ai],self.nodes[bi]
    if a.portal==b.portal:continue
    for edge in self._connection(a,b,all_valid=True):self._admit(edge)
  self.prepared_regions.add(region)
 def prepare(self):
  if self.cache_hit:return
  started=time.monotonic()
  for region in sorted(self.region_nodes):self.prepare_region(region)
  self.build_connections_ms+=(time.monotonic()-started)*1000
  self.save()
 def save(self):
  root=self.cache_root/self.key;root.mkdir(parents=True,exist_ok=True)
  arrays={f'e{k}':v.path for k,v in self.edges.items()};arrays['subregions']=self.subregions
  np.savez_compressed(root/'geometry.npz',**arrays)
  json_write(root/'graph.json',{'key':self.key,'config':self.config,'world_binding':self.world.binding,
   'nodes':[asdict(n) for n in self.nodes.values()],
   'edges':[{k:v for k,v in asdict(e).items() if k!='path'} for e in self.edges.values()],
   'rejections':self.records,'geometry_sha256':__import__('hashlib').sha256((root/'geometry.npz').read_bytes()).hexdigest(),
   'preparation_ms':self.preparation_ms,'connection_preparation_ms':self.build_connections_ms})
 def _temp(self,pose,ident):
  cell=self.world.map.world_to_cell(*pose[:2])
  if cell is None or not self.world.safe[cell]:return None
  r=int(self.subregions[cell]);k=self.parent_semantics.get(r,int(self.world.labels[cell]));self.parent_semantics[r]=k
  return InterfacePose(ident,tuple(pose),r,r,k,k,-1,True)
 def plan(self,query,*,timeout=5.,blocked_cells=()):
  started=time.monotonic();blocked=tuple(sorted(set(tuple(map(int,c)) for c in blocked_cells)))
  key=canonical_hash([query.start,query.goal,self.key,blocked]);cached=self.query_cache.get(key)
  self.last_query_diagnostics={'dynamic_cells':len(blocked),'dynamic_rejected_edges':0,'status':'UNVERIFIED_SAMPLING'}
  if cached is not None:
   self.connection_cache_hits+=1
   return self._make_route(query,*cached,started=started,cache_hit=True)
  a,b=self._temp(query.start,-1),self._temp(query.goal,-2)
  if a is None or b is None:return None
  tempnodes={-1:a,-2:b};tempedges={};tempadj=defaultdict(list)
  def admit(e):
   if e is not None:e.id=-len(tempedges)-1;tempedges[e.id]=e;tempadj[e.source].append(e.id)
  for target in self.region_nodes[a.to_region]['out']:
   for edge in self._connection(a,self.nodes[target],all_valid=True):admit(edge)
  for source in self.region_nodes[b.from_region]['in']:
   for edge in self._connection(self.nodes[source],b,all_valid=True):admit(edge)
  if a.to_region==b.from_region:
   for edge in self._connection(a,b,all_valid=True):admit(edge)
  nodes={**self.nodes,**tempnodes};edges={**self.edges,**tempedges};serial=itertools.count()
  dynamic_checker=None;edge_valid={}
  if blocked:
   hard=self.world.hard.copy()
   for cell in blocked:hard[cell]=True
   dynamic_checker=SweptChecker(hard,self.world.map.resolution,self.world.map.origin,*self.world.vehicle[:2])
  bans=self.world.rules.get('forbidden_transitions',[])
  def advance(history,target):
   if not bans:return ()
   values=list(history)
   if not values or values[-1]!=target:values.append(target)
   if len(values)>=3:
    triple=values[-3:];names=[self.world.features[k].semantic_id if k in self.world.features else 'unlabelled' for k in triple]
    if triple in bans or names in bans:return None
   return tuple(values[-2:])
  initial=(-1,(a.semantic_to,) if bans else ())
  dist={initial:0.};prev={};queue=[(math.dist(a.pose[:2],b.pose[:2]),0.,next(serial),initial)];expanded=0;finished=None
  while queue:
   if time.monotonic()-started>timeout:
    self.last_query_diagnostics['status']='UNVERIFIED_TIMEOUT';return None
   _,g,_,state=heapq.heappop(queue);u,history=state
   if g!=dist.get(state):continue
   if u==-2:finished=state;break
   expanded+=1
   for eid in list(self.adjacency.get(u,[]))+tempadj.get(u,[]):
    e=edges[eid]
    next_history=advance(history,nodes[e.target].semantic_to)
    if next_history is None:continue
    successor=(e.target,next_history)
    if dynamic_checker is not None:
     if eid not in edge_valid:
      if e.path is None:e.path=self._edge_payload[f'e{e.id}']
      edge_valid[eid]=dynamic_checker.check(e.path)
      if not edge_valid[eid]:self.last_query_diagnostics['dynamic_rejected_edges']+=1
     if not edge_valid[eid]:continue
    ng=g+e.length
    if ng<dist.get(successor,math.inf):
     dist[successor]=ng;prev[successor]=(state,eid)
     h=math.dist(nodes[e.target].pose[:2],b.pose[:2]);heapq.heappush(queue,(ng+h,ng,next(serial),successor))
  if finished is None:return None
  cur=finished;route=[];ns=[cur[0]]
  while cur!=initial:
   cur,eid=prev[cur];route.append(edges[eid]);ns.append(cur[0])
  route.reverse();ns.reverse();selected_nodes=[nodes[k] for k in ns]
  # Keep only a bounded set of temporary endpoint/dynamic-version bindings.
  if len(self.query_cache)>=32:self.query_cache.pop(next(iter(self.query_cache)))
  self.query_cache[key]=(selected_nodes,route,expanded)
  return self._make_route(query,selected_nodes,route,expanded,started=started,cache_hit=False)
 def _make_route(self,query,nodes,edges,expanded,*,started,cache_hit):
  self.last_query_diagnostics['status']='VALIDATED_ROUTE'
  for e in edges:
   if e.path is None:e.path=self._edge_payload[f'e{e.id}']
  path=np.concatenate([e.path if i==0 else e.path[1:] for i,e in enumerate(edges)])
  # Same 2 m route / 4 m corner corridor semantics as r1. The complete
  # corridor is fixed before L2 preference and is identical for A/B/C.
  line=np.zeros(self.subregions.shape,np.uint8);r,c=self.world.cells(path);line[r,c]=1
  res=self.world.map.resolution
  kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*int(math.ceil(2/res))+1,)*2)
  mask=cv2.dilate(line,kernel)>0
  turn=np.zeros_like(line);dtheta=abs(wrap(np.diff(path[:,2])));ix=np.flatnonzero(dtheta>1e-5)
  if len(ix):
   turn[r[ix],c[ix]]=1
   kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*int(math.ceil(4/res))+1,)*2)
   mask|=cv2.dilate(turn,kernel)>0
  mask&=~self.world.hard
  route_ids=tuple(f'{e.region}:{e.source}>{e.target}:{e.evidence["path_sha256"][:12]}' for e in edges)
  signature=canonical_hash([self.key,route_ids,query.start,query.goal])
  plan=L1Plan(self.world.safe,mask,self.world.map.world_to_cell(*query.start[:2]),self.world.map.world_to_cell(*query.goal[:2]),
   self.world.map.sha256,self.world.map.origin,res,self.key,route_ids,canonical_hash(self.world.binding['vehicle']),signature,
   {'semantic_graph':True,'ordered_regions':[e.region for e in edges],'graph_nodes':len(self.nodes),'graph_edges':len(self.edges)})
  return SemanticRoute(plan,path,nodes,edges,[e.region for e in edges],{'search_ms':(time.monotonic()-started)*1000,'expanded_nodes':expanded,'query_cache_hit':cache_hit,**self.last_query_diagnostics})
