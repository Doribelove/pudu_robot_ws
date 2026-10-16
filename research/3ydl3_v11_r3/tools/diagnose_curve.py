"""Offline solver diagnostic on preserved L2/scope evidence; not performance data."""
import argparse,json,time
from pathlib import Path
import numpy as np
from arena_3d_v1.semantic_world import SemanticWorld
from three_yd_v5.free_space_topology import load_map_only_world
from three_yd_v5.curve_l3 import optimize,CONFIG
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--map-id',default='mengjia');p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=8);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
if a.map_id=='mengjia':w=SemanticWorld(ROOT/'inputs/corroborated_map.yaml',ROOT/'inputs/semantic.json',cache_root=a.output/'world')
else:w=load_map_only_world(ROOT/'inputs'/a.map_id/'map.yaml',a.output/'world')
rows=[json.loads(l) for l in (a.case.parent.parent/'runs.jsonl').read_text().splitlines()];row=next(r for r in rows if a.case.name.startswith(r['query_id']+'_'))
mask=np.load(a.case/'l3_scope.npz')['mask'];guide=np.load(a.case/'l2_reference_xy.npy')
path,curve,log=optimize(w,mask,guide,row['start'],row['goal'],time.monotonic()+a.seconds)
(a.output/'log.json').write_text(json.dumps(log,indent=2));(a.output/'curve.json').write_text(json.dumps(curve))
if path is not None:np.save(a.output/'path.npy',path)
print(log)
