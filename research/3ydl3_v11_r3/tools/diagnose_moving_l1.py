"""Read-only diagnosis; do not change frozen L1 or endpoint bindings."""
import json,time
from pathlib import Path
import numpy as np
from arena_3d_v1.semantic_world import SemanticWorld
from arena_evaluation.planner_benchmark.models import Query
from three_yd_v5.topology_default import DefaultTopology
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'report/Q09_L1_DIAGNOSIS';out.mkdir(exist_ok=True)
w=SemanticWorld(ROOT/'inputs/corroborated_map.yaml',ROOT/'inputs/semantic.json',cache_root=out/'world')
g=DefaultTopology(w,ROOT/'inputs/topology.json',out/'graph');results=[]
for qd in json.loads((ROOT/'datasets/moving_start_v1/mengjia.json').read_text())['scenarios']:
 if qd['id']!='Q09':continue
 q=Query('Q09',tuple(qd['start'][k] for k in ('x','y','yaw_rad')),tuple(qd['goal'][k] for k in ('x','y','yaw_rad')),seed=20260916)
 rc=w.map.world_to_cell(*q.start[:2]);rid=int(g.subregions[rc]);row={'step':qd['moving_start']['step'],'pose':q.start,'raster_region_id':rid,'raster_region':g.regions[rid]['name']}
 try:
  starts=g.endpoint_options(q,'start');goals=g.endpoint_options(q,'goal');row['start_options']=starts;row['goal_options']=goals
  if starts[0][1]==goals[0][1] and g.regions[rid]['kind']=='lane':
   road=starts[0][1];parking=goals[0][0];contact=np.asarray(g.contacts[road,parking]);b=contact[np.argmin(np.linalg.norm(contact-np.asarray(q.goal[:2]),axis=1))]
   axis=g.router.axes[road];advance=float((b-np.array(q.start[:2]))@axis);direction=axis*(1 if advance>0 else -1)
   row.update(nearest_goal_contact=b.tolist(),road_axis=axis.tolist(),advance_to_contact_m=advance,goal_binding_allows_direct_direction=g._allowed_direction(road,direction,goals[0][2]))
  routes=g.routes(q,limit=3);row['routes_found']=len(routes);row['search_stats']=g._probe_stats
 except Exception as e:row['error']=type(e).__name__+':'+str(e)
 results.append(row)
(out/'diagnosis.json').write_text(json.dumps(results,ensure_ascii=False,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x))+'\n')
for r in results:print({k:v for k,v in r.items() if k not in ('goal_options','start_options','search_stats')})
