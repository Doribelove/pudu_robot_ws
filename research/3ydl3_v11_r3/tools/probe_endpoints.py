import json,sys
from pathlib import Path
import numpy as np
from crossmap_inputs import load_world
from arena_3d_v1.semantic_world import SweptChecker
ROOT=Path(__file__).resolve().parents[1]
for mid,qids in {'arena_small_080x100_2d_005':['A2B-03'],'arena_medium_200x200_2d_005':['A2B-03','A2B-04'],'mentor_map_20260825_005':['A2B-03','A2B-07'],'pudu_wanda_3f':['Q08','Q09'],'3yd_synth_twinwing_b_v1':['Q06'],'3yd_synth_trizone_c_v2':['Q10']}.items():
 out=ROOT/'results/endpoint_audit'/mid;out.mkdir(parents=True,exist_ok=True)
 _,_,w,_,qs,_=load_world(mid,out);checker=SweptChecker(w.hard,w.map.resolution,w.map.origin,*w.vehicle[:2]);rows=[]
 for q in qs:
  if q['id'] not in qids:continue
  for name in ('start','goal'):
   v=q[name];v=np.array([v[k] for k in ('x','y','yaw_rad')] if isinstance(v,dict) else v)
   offsets=np.linspace(-1,1,41);p=np.tile(v,(len(offsets),1));p[:,:2]+=offsets[:,None]*[np.cos(v[2]),np.sin(v[2])]
   valid=[checker.check(a[None,:]) for a in p]
   record={'query':q['id'],'endpoint':name,'pose':v.tolist(),'body_valid':checker.check(v[None,:]),'forward_line_offsets':offsets.tolist(),'forward_line_valid':valid}
   rows.append(record);print(mid,q['id'],name,'body',record['body_valid'],'offset',[(round(a,2),b) for a,b in zip(offsets,valid) if abs(a)<=.15],flush=True)
 (out/'audit.json').write_text(json.dumps(rows,indent=2))
