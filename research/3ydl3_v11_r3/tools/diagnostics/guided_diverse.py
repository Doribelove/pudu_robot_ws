import json,time,yaml,hashlib
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from arena_3d_v1.semantic_world import SweptChecker
import three_yd_v5.v9_guided_joint as module
src=Path(module.__file__).read_text();src=src.replace('for _,m in sorted(candidates)[:3]:','chosen=[]\n    for _,m in sorted(candidates):\n        if all(abs(arc[m]-arc[j])>=.5 for j in chosen):chosen.append(m)\n        if len(chosen)==3:break\n    for m in chosen:')
ns={'__package__':'three_yd_v5'};exec(src,ns);output={'source_sha256':hashlib.sha256(src.encode()).hexdigest(),'cases':{}}
for mid in ('mengjia','arena_large_250x300_2d_005'):
 root=Path('results/firstplan_diagnostic_v4/DP')/mid
 mp=Path('inputs/corroborated_map.yaml') if mid=='mengjia' else Path('inputs/free_maps')/mid/'map.yaml'
 if not mp.exists():
  from run_free_full import registered_input
  mp=registered_input(mid)[0]/'map.yaml'
 meta=yaml.safe_load(mp.read_text())
 for s in (root/'runs.jsonl').read_text().splitlines():
  r=json.loads(s);calls=r['v9_local'].get('connections',[])
  if not calls or calls[-1]['anchor_fit'].get('success'):continue
  c=calls[-1];p=np.array(c['reference_window_poses']);a=c['window_start_index'];b=c['window_end_index'];mask=np.load(root/'cases'/f"{r['query_id']}_1"/'l3_scope.npz')['mask'];checker=SweptChecker(~mask,meta['resolution'],meta['origin'],.265,.225);tick=time.monotonic()
  q,d=ns['guided_connect'](S(vehicle=(.265,.225,.4)),p,a,b,checker,time.monotonic()+.2)
  print(mid,r['query_id'],q is not None,round((time.monotonic()-tick)*1000),flush=True);output['cases'][mid+'_'+r['query_id']]={'detail':d,'wall_ms':(time.monotonic()-tick)*1000}
Path('report/GUIDED_DIVERSE_OFFLINE.json').write_text(json.dumps(output,indent=2))
