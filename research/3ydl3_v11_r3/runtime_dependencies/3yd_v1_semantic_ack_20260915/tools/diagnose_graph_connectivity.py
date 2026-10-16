import time,yaml,os,json,heapq
import numpy as np
from collections import deque,Counter,defaultdict
from scipy import ndimage
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write
from arena_3d_v1.semantic_graph import SemanticPoseGraph
from arena_evaluation.planner_benchmark.models import Query
os.sched_setaffinity(0,{14})
f=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(f['inputs']['map_yaml'],f['inputs']['semantic_path'],cache_root=WORK/'cache');g=SemanticPoseGraph(w,WORK/'cache/graph');out=[]
components,count=ndimage.label(w.safe)
regionadj=defaultdict(set)
for n in g.nodes.values():regionadj[n.from_region].add(n.to_region)
for d in yaml.safe_load((WORK/'config/queries.yaml').read_text())['queries'][13:15]:
 q=Query(d['query_id'],d['start'],d['goal'],seed=0);a,b=g._temp(q.start,-1),g._temp(q.goal,-2)
 src=[];dst=[]
 for n in g.region_nodes[a.to_region]['out']:
  if g._connection(a,g.nodes[n],record=False):src.append(n)
 for n in g.region_nodes[b.from_region]['in']:
  if g._connection(g.nodes[n],b,record=False):dst.append(n)
 reached=set(src);todo=deque(src)
 while todo:
  u=todo.popleft()
  for eid in g.adjacency[u]:
   v=g.edges[eid].target
   if v not in reached:reached.add(v);todo.append(v)
 # Region path ignores car kinematics and serves only to locate missing
 # motion-validation evidence, never an admissible L1 route.
 todo=deque([a.to_region]);par={a.to_region:None}
 while todo:
  u=todo.popleft()
  for v in regionadj[u]:
   if v not in par:par[v]=u;todo.append(v)
 chain=[];u=b.from_region
 if u in par:
  while u is not None:chain.append(u);u=par[u]
 chain.reverse()
 missing=[]
 for region in chain:
  ins=[n for n in g.region_nodes[region]['in'] if n in reached]
  missing.append({'region':region,'reached_in':len(ins),'all_in':len(g.region_nodes[region]['in']),'out':len(g.region_nodes[region]['out']),'semantic':g.parent_semantics[region]})
 p={'id':q.query_id,'src_count':len(src),'dst_count':len(dst),'reachable_nodes':len(reached),'reachable_dest_nodes':list(set(dst)&reached),'components':[int(components[w.map.world_to_cell(*pose[:2])]) for pose in [q.start,q.goal]],'region_chain':missing,'src':src,'dst':dst}
 out.append(p);print(json.dumps(p),flush=True)
json_write(WORK/'results/graph_connectivity_diagnostic_01.json',out)
