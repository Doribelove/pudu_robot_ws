"""Fresh-process, all-cell/graph comparison; diagnostic, outside formal metrics."""
from pathlib import Path
import os,sys,json,hashlib,time,subprocess,argparse
E=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--map-id');ap.add_argument('--mode',choices=['parent','candidate']);args=ap.parse_args()
if args.mode:
 import numpy as np
 os.sched_setaffinity(0,{14})
 for k,v in {'THREEYD_V9_MODE':'A','THREEYD_V9_R1_MODE':'S','THREEYD_V9_R2_MODE':'D','THREEYD_V9_R3_MODE':'control','THREEYD_V9_R5_MODE':'C','THREEYD_V9_R6_MODE':'H','THREEYD_V9_R7_MODE':'NT','THREEYD_V9_R8_MODE':'OGR','THREEYD_V9_R9_MODE':'PLMX'}.items():os.environ[k]=v
 os.environ['THREEYD_V10_FIRSTREADY']='1' if args.mode=='candidate' else '0'
 from crossmap_inputs import load_world
 from arena_3d_v1.multires_grid import GridView
 from benchmark import Memory
 out=E/'results/preparation_equivalence'/args.map_id/args.mode;out.mkdir(parents=True,exist_ok=False)
 tick=time.monotonic()
 with Memory() as memory:
  _,mp,w,g,qs,qf=load_world(args.map_id,out);views={factor:GridView(w,factor) for factor in (1,3)}
 elapsed=time.monotonic()-tick
 def digest(a):return {'shape':list(a.shape),'dtype':str(a.dtype),'sha256':hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()}
 def native(v):
  if isinstance(v,np.ndarray):return v.tolist()
  if isinstance(v,np.generic):return v.item()
  if isinstance(v,dict):return [[native(k),native(x)] for k,x in sorted(v.items(),key=lambda pair:repr(pair[0]))]
  if isinstance(v,(tuple,list)):return [native(x) for x in v]
  if isinstance(v,set):return sorted(v)
  return v
 snapshot={'world':{k:digest(getattr(w,k)) for k in ('labels','hard','safe')},'subregions':digest(g.subregions),'views':{factor:{k:digest(getattr(view,k)) for k in ('labels','hard','safe')} for factor,view in views.items()},'topology':{k:native(getattr(g,k)) for k in ('graph','source_hash','access','parent_semantics','contacts','adj','key','_static_transitions','contact_centers')},'router_axes':native(g.router.axes),'router_polygons':native(g.router.polygons),'world_key':w.key,'poses':qs}
 proof={'map_id':args.map_id,'mode':args.mode,'wall_s':elapsed,'tree_RSS_peak_MiB':memory.peak/2**20,'diagnostic_only':True,'snapshot':snapshot,'construction':json.loads((out/'raster_storage.json').read_text())}
 (out/'comparison.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2));print(args.map_id,args.mode,elapsed,memory.peak/2**20,flush=True)
else:
 maps=[m for m in json.loads((E/'MAPS.json').read_text())['maps'] if m['kind']!='map_only' and (not args.map_id or args.map_id==m['map_id'])];results=[]
 for m in maps:
  mid=m['map_id'];pair={}
  for mode in ('parent','candidate'):
   target=E/'results/preparation_equivalence'/mid/mode/'comparison.json'
   if not target.exists():subprocess.run([sys.executable,__file__,'--map-id',mid,'--mode',mode],check=True)
   pair[mode]=json.loads(target.read_text())
  assert pair['parent']['snapshot']==pair['candidate']['snapshot'],('PREPARATION_DIFFERENCE',mid)
  assert pair['candidate']['construction']['enabled'] and pair['candidate']['construction']['safe_predicate']['direct']
  results.append({'map_id':mid,'all_world_cells_topology_and_views_equal':True,'cells':pair['parent']['snapshot']['world']['labels']['shape'][0]*pair['parent']['snapshot']['world']['labels']['shape'][1],'parent_s':pair['parent']['wall_s'],'candidate_s':pair['candidate']['wall_s'],'parent_RSS_MiB':pair['parent']['tree_RSS_peak_MiB'],'candidate_RSS_MiB':pair['candidate']['tree_RSS_peak_MiB'],'construction':pair['candidate']['construction']})
  print('EQUAL',mid,flush=True)
 (E/'report/PREPARATION_EQUIVALENCE.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
