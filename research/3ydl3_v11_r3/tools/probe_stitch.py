import json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from three_yd_v5.free_space_topology import load_map_only_world,FreeSpaceTopology
from three_yd_v5.endpoint_repair import repair_path_extended
from arena_evaluation.planner_benchmark.models import Query
ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'results/diagnostic_segments_01'
world=load_map_only_world(ROOT/'inputs/arena_large_250x300_2d_005/map.yaml',ROOT/'results/offline_world')
row=next(x for x in json.loads((ROOT/'inputs/arena_large_250x300_2d_005/queries.json').read_text())['queries'] if x['id']=='A2B-03')
q=Query(row['id'],tuple(row['start']),tuple(row['goal']))
paths=[np.load(base/f'part{i}.npy') for i in range(4)]
p=np.vstack([paths[0]]+[v[1:] for v in paths[1:]])
mask=np.load(base/'scope.npz')['mask']
fixed,log=repair_path_extended(world,SimpleNamespace(audit=lambda p:{'valid':True}),q,p,mask,budget_s=5.)
(ROOT/'results/stitch_probe.json').write_text(json.dumps(log,indent=2))
print('success',fixed is not None,'ms',log['wall_ms'],'failure',log.get('failure'),'trials',len(log['trials']))
if fixed is not None:
 np.save(ROOT/'results/stitch_fixed.npy',fixed)
 print(world.audit(q,fixed,mask))
else: print(log['trials'][-10:])
