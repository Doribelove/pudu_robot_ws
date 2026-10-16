import json,time,yaml
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker,densify,wrap
from evaluate_with_accel import evaluate
from arena_3d_v1.curvature_repair import violating_segments
root=Path('results/firstplan_diagnostic_v2/DP/mengjia'); rows=[json.loads(s) for s in (root/'runs.jsonl').read_text().splitlines()]
m=yaml.safe_load(Path('inputs/corroborated_map.yaml').read_text());output={}
for row in rows:
 if row['query_id'] not in ('Q04','Q06'):continue
 call=row['v9_local']['connections'][-1];p=np.array(call['reference_window_poses']);a=call['window_start_index'];b=call['window_end_index']
 mask=np.load(root/'cases'/f"{row['query_id']}_1"/'l3_scope.npz')['mask'];checker=SweptChecker(~mask,m['resolution'],m['origin'],.265,.225)
 chord=np.linalg.norm(p[b,:2]-p[a,:2]);best=None;success=None;tick=time.monotonic();trials=0
 print(row['query_id'],'anchor_clear',checker.check(p[a:a+1]),checker.check(p[b:b+1]),flush=True)
 for x,alpha,beta in ((x,alpha,beta) for x in (1.5,2.,2.5,3.) for alpha in (-4.,-2.,0.,2.,4.) for beta in (-4.,-2.,0.,2.,4.)):
  for y in (1.5,2.,2.5,3.):
   trials+=1;q,k=evaluate(p[a],p[b],0,0,(x*chord,y*chord),sample_limit=8192,tangent_acceleration=(alpha,beta))
   if max(abs(k))>2.49 or len(violating_segments(np.vstack((p[:a],q,p[b+1:])),2.5)[0]):continue
   ok=checker.check(densify(q))
   if ok:success={'x':float(x),'y':float(y),'max_k':float(max(abs(k))),'alpha':alpha,'beta':beta};break
  if success:break
 print('FOUND',success,'trials',trials,'ms',(time.monotonic()-tick)*1000,flush=True)
 output[row['query_id']]={'success':success,'trials':trials,'wall_ms':(time.monotonic()-tick)*1000,'anchor_clear':[bool(checker.check(p[a:a+1])),bool(checker.check(p[b:b+1]))]}
Path('report/JOINT_ACCEL_GRID_OFFLINE.json').write_text(json.dumps(output,indent=2))
