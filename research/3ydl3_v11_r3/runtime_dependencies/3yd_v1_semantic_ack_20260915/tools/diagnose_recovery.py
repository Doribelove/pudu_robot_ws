import json,yaml,time
from pathlib import Path
import numpy as np
from scipy.ndimage import label
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.multires_grid import GridView,make_roi,guide_astar,aggregate
from arena_evaluation.planner_benchmark.models import Query
BASE=Path('/home/robot/workspaces/3yd_multires_static_20260914')
cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text()); w=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=WORK/'cache/world');g=SemanticTopology(w,WORK/'cache/graph');v=GridView(w,3);fine=GridView(w,1);out=[]
for qd in yaml.safe_load((WORK/'config/queries.yaml').read_text())['queries']:
 if qd['category'] not in ['parking_internal','lane_to_parking','parking_to_lane','lane_junction_turn','lane_multi_junction']:continue
 q=Query(qd['query_id'],tuple(qd['start']),tuple(qd['goal']),seed=0);r=g.plan(q);mask=g.fine_mask(r);coarse=v.corridor(mask)&v.safe
 labs,n=label(coarse);st,go=v.cell(q.start),v.cell(q.goal)
 row={'query':q.query_id,'regions':r.region_ids,'interfaces':r.interface_ids,'coarse_start_goal_components':[int(labs[st]),int(labs[go])],'start_goal_coarse_safe':[bool(coarse[st]),bool(coarse[go])]}
 for arm in ['baseline','topology_015']:
  case=BASE/'results'/f'formal_01_{arm}'/'cases'/f'{q.query_id}__1__right_0';p=np.load(case/'path.npy');rr,cc=w.cells(p)
  row[arm]={'outside_new_corridor_points':int((~mask[rr,cc]).sum()),'regions_seen':list(dict.fromkeys(g.subregions[rr,cc].tolist())),'length':float(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum())}
 # Locate the conservative loss on the existing fine reference.
 frc=np.load(BASE/'results/formal_01_topology_015/cases'/f'{q.query_id}__1__right_0'/'reference_fine_cells.npy');xy=w.world_points(frc[:,0],frc[:,1]);rr,cc=v.cells(xy)
 bad=~coarse[rr,cc];groups=[];beg=None
 for i in range(len(bad)+1):
  if i<len(bad) and bad[i] and beg is None:beg=i
  if (i==len(bad) or not bad[i]) and beg is not None:
   groups.append({'points':i-beg,'length_m':float(np.linalg.norm(np.diff(xy[max(0,beg-1):min(i+1,len(xy))],axis=0),axis=1).sum()),'bbox_xy':[xy[beg:i].min(axis=0).tolist(),xy[beg:i].max(axis=0).tolist()]});beg=None
 row['fine_reference_lost_in_coarse']=groups
 out.append(row);print(json.dumps(row),flush=True)
json_write(WORK/'report/diagnosis.json',out)
