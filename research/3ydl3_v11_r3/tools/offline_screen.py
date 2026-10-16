from pathlib import Path
import json,os
from replay_curve import run
ROOT=Path(__file__).resolve().parents[1]
os.sched_setaffinity(0,{14})
folder=ROOT/'results/offline_screen1';folder.mkdir(exist_ok=False)
cases=[]
for mid,qid in [('mentor_map_20260825_005','A2B-07'),('arena_medium_200x200_2d_005','A2B-04'),('3yd_synth_trizone_c_v2','Q10'),('pudu_wanda_3f','Q08')]:
 cases.append((mid,qid,next((ROOT/'results/diagnostic_control'/mid/'curve_inputs').glob(qid+'_*.npz'))))
variants={'control':{},'dense':{'constraint_step_m':.05,'adaptive_constraint_stations':True},'margin':{'curvature_target_ratio':.90},'tangent':{'free_endpoint_tangent_scale':True},'tangent_dense':{'free_endpoint_tangent_scale':True,'adaptive_constraint_stations':True,'constraint_step_m':.05}}
for mid,qid,path in cases:
 for name,config in variants.items():
  out=folder/mid/qid/name;out.mkdir(parents=True)
  result,record=run(path,config)
  (out/'result.json').write_text(json.dumps(record,indent=2)+'\n')
  if result[1] is not None:(out/'curve.json').write_text(json.dumps(result[1])+'\n')
  print(mid,qid,name,record['geometric_success'],round(record['elapsed_s'],3),record['log'].get('failure'),flush=True)
