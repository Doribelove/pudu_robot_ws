import sys,json,time
from pathlib import Path
import numpy as np
from crossmap_inputs import load_world
from three_yd_v5.curve_l3 import feasible_guide,interp
from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
from arena_3d_v1.semantic_world import SweptChecker,densify
ROOT=Path(__file__).resolve().parents[1];mid,qid=sys.argv[1:3];out=ROOT/'results/endpoint_choices'/f'{mid}_{qid}';out.mkdir(parents=True,exist_ok=True)
_,_,w,_,_,_=load_world(mid,out);z=np.load(ROOT/'results/inspect'/f'{mid}_{qid}'/'rank1.npz');start=z['start'];goal=z['goal'];p,s,_=feasible_guide(z['guide'],start,goal,w,z['physical'])
checks={k:SweptChecker(w.hard|~z[k] if k!='hard' else w.hard,w.map.resolution,w.map.origin,*w.vehicle[:2]) for k in ('hard','physical','mask')};found={k:[] for k in checks}
for distance in np.arange(.05,1.01,.05):
 at=s[-1]-distance;xy=interp(p,s,[at])[0];before,after=interp(p,s,[at-.05,at+.05]);yaw=np.arctan2(*(after-before)[::-1]);a=tuple(xy)+(yaw,)
 for radius in (.405,.41,.42,.45,.5,.6,.7,.8):
  for word,pars in dubins_choices(a,tuple(goal),radius):
   length=radius*sum(pars)
   if length>distance+7:continue
   edge=dubins_edge_from_parameters(a,tuple(goal),radius,word,pars);path=densify(np.vstack((a,edge.samples)))
   for name,check in checks.items():
    if check.check(path):found[name].append((length-distance,distance,radius,word,path.tolist()))
for name,rows in found.items():
 rows.sort();print(name,len(rows),[(r[:4]) for r in rows[:4]],flush=True)
 if rows:
  score,dist,radius,word,poses=rows[0];at=s[-1]-dist;guide=np.vstack((p[s<at],np.asarray(poses)[:,:2]));np.save(out/(name+'_guide.npy'),guide)
(out/'choices.json').write_text(json.dumps(found))
