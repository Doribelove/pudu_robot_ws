import json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from three_yd_v5.free_space_topology import load_map_only_world,FreeSpaceTopology
from three_yd_v5.endpoint_repair import repair_path_extended
from arena_evaluation.planner_benchmark.models import Query
ROOT=Path(__file__).resolve().parents[1]
base=Path('/home/robot/workspaces/3yd_v8_r0_20261008/results/formal/arena_large_250x300_2d_005/cases/A2B-06_1')
world=load_map_only_world(ROOT/'inputs/arena_large_250x300_2d_005/map.yaml',ROOT/'results/offline_world')
row=next(x for x in json.loads((ROOT/'inputs/arena_large_250x300_2d_005/queries.json').read_text())['queries'] if x['id']=='A2B-06')
q=Query(row['id'],tuple(row['start']),tuple(row['goal']))
from three_yd_v5.geometry import variable_neighborhood
p=np.load(base/'rejected_candidate.npy')
topology=FreeSpaceTopology(world,ROOT/'results/offline_graph'); route=topology.routes(q,limit=1)[0]
mask,_=variable_neighborhood(world,np.load(base/'l2_reference_xy.npy'),json.loads((base/'l2_corners.json').read_text()))
mask &= ~world.hard & topology.scope_mask(route)
fixed,log=repair_path_extended(world,SimpleNamespace(audit=lambda p:{'valid':True}),q,p,mask,budget_s=5.)
(ROOT/'results/endpoint_probe.json').write_text(json.dumps(log,indent=2))
print('success',fixed is not None,'ms',log['wall_ms'],'failure',log.get('failure'),'trials',len(log['trials']))
if fixed is not None:
 np.save(ROOT/'results/endpoint_fixed.npy',fixed)
 print(world.audit(q,fixed,mask))
else: print(log['trials'][-10:])
