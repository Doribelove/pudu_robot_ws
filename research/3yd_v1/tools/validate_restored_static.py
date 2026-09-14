"""Verify the cache-read-only correction against every frozen real query."""
import ast,hashlib,json,shutil
from pathlib import Path
import numpy as np,yaml
from arena_3d_v1.semantic_world import SemanticWorld,WORK,digest,json_write
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.multires_pipeline import MultiresController
from arena_3d_v1.multires_grid import GridView
from arena_evaluation.planner_benchmark.models import Query
out=WORK/'results/cache_restore_validation_01';out.mkdir(exist_ok=False)
cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=out/'world')
g=SemanticTopology(w,WORK/'results/prep_formal_02_topology_015/graph');assert g.cache_hit
queries=yaml.safe_load((WORK/'config/queries.yaml').read_text())['queries'];views={f:GridView(w,f) for f in [1,3]};checks=[]
for arm,factor in [('topology_005',1),('topology_015',3)]:
 root=WORK/'results'/f'formal_01_{arm}';rows={r['query_id']:r for r in map(json.loads,(root/'runs.jsonl').read_text().splitlines()) if r['repetition']==1}
 cache=out/arm;shutil.copytree(root/'l2_cache',cache)
 for qd in queries:
  q=Query(qd['query_id'],tuple(map(float,qd['start'])),tuple(map(float,qd['goal'])),seed=qd.get('seed',20260914));r=g.plan(q)
  c=MultiresController(w,g,r,cache_root=cache,factor=factor,views=views);old=rows[q.query_id]
  check={'arm':arm,'query_id':q.query_id,'success':c.initial_l2_result.success,'same_route_signature':r.signature==old['reference']['route_signature'],
   'same_fine_hard_corridor':digest(c._target_mask())==old['hard_corridor_hash'],
   'same_l2_reference':np.array_equal(c.reference_cells,np.load(Path(old['case'])/'reference_fine_cells.npy')),
   'same_resolution':abs(c.view.map.resolution-old['multires']['actual_resolution_m'])<1e-8,
   'state_cache_hit':c.initial_l2_result.diagnostics['activation']['state_cache_hit']}
  check['pass']=all(v for k,v in check.items() if isinstance(v,bool));checks.append(check);c.lifecycle.clear();print(arm,q.query_id,check['pass'],flush=True)
old=WORK/'results/formal_01_topology_015/source_snapshot/multires_topology.py';new=WORK/'external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/multires_topology.py'
def methods(p):
 c=next(x for x in ast.parse(p.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='SemanticTopology')
 return {x.name:ast.dump(x,include_attributes=False) for x in c.body if isinstance(x,ast.FunctionDef)}
a,b=methods(old),methods(new);unchanged=[k for k in a if a[k]==b[k]];changed=[k for k in a if a[k]!=b[k]]
result={'count':len(checks),'passed':sum(c['pass'] for c in checks),'rows':checks,'unchanged_methods':unchanged,'changed_methods':changed,'correction':'Restore integer parent-region keys before checking the build-time hash; only cache-load branch in constructor changed. Formal query groups built graphs in-process, without using this branch.'}
assert changed==['__init__'];assert all(c['pass'] for c in checks)
json_write(WORK/'report/cache_restore_validation.json',result)
