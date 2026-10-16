import os,json,yaml
from pathlib import Path
import numpy as np
from arena_3d_v1.semantic_world import WORK,SemanticWorld,json_write,digest
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.multires_grid import GridView
from arena_3d_v1.recovery_pipeline import RecoveryController
from arena_evaluation.planner_benchmark.models import Query
os.sched_setaffinity(0,{14})
root=WORK/'results/formal_recovery_final';cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());w=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=root/'preparation_cache/world');g=SemanticTopology(w,root/'preparation_cache/graph',portal_samples=3);views={f:GridView(w,f) for f in (1,3)}
assert g.cache_hit
queries={q['query_id']:q for q in yaml.safe_load((WORK/'config/queries.yaml').read_text())['queries']};checks=[]
for row in map(json.loads,(root/'runs.jsonl').read_text().splitlines()):
 if row['repetition']!=1:continue
 qd=queries[row['query_id']];q=Query(qd['query_id'],tuple(map(float,qd['start'])),tuple(map(float,qd['goal'])),seed=qd.get('seed',0));route=g.plan(q);c=RecoveryController(w,g,route,cache_root=root/'l2_cache',views=views)
 path=Path(row['case']);same=np.array_equal(c.reference_cells,np.load(path/'reference_fine_cells.npy')) and digest(c.plan.corridor_mask)==row['hard_corridor_hash'];assert same,row['query_id'];assert c.timing['reference_cache_hit'],row['query_id']
 checks.append({'query':row['query_id'],'same_reference_and_corridor':same,'mode':c.timing['completion_mode'],'reference_cache_hit':c.timing['reference_cache_hit']});c.lifecycle.clear()
json_write(WORK/'report/cache_restore_validation.json',{'n':len(checks),'passed':sum(x['same_reference_and_corridor'] for x in checks),'graph_restore_hit':g.cache_hit,'checks':checks});print('cache restore',len(checks),'passed')
