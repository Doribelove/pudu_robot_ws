import sys,time,json
from pathlib import Path
import numpy as np
from crossmap_inputs import load_world
from three_yd_v5.curve_l3 import feasible_guide,interp,optimize,CONFIG
from three_yd_v5.curve_endpoint_v10 import connect
ROOT=Path(__file__).resolve().parents[1];PARENT=Path('/home/robot/workspaces/3ydl3_v9_crossmapfix_20261011');mid,qid=sys.argv[1:3]
out=ROOT/'results/endpoint_probe'/f'{mid}_{qid}';out.mkdir(parents=True,exist_ok=True)
_,_,w,_,_,_=load_world(mid,out);z=np.load(PARENT/'results/inspect'/f'{mid}_{qid}'/'rank1.npz');start=z['start'];goal=z['goal'];p,s,_=feasible_guide(z['guide'],start,goal,w,z['physical'])
for radius in (.45,.405,.5):
 for dist in (2.,3.,4.):
  at=s[-1]-dist;xy=interp(p,s,[at])[0];v=np.diff(interp(p,s,[at-.05,at+.05]),axis=0)[0];a=(*xy,np.arctan2(v[1],v[0]))
  tick=time.monotonic();path,log=connect(w,z['mask'],a,goal,tick+2.5,radius=radius);print('LOCAL',radius,dist,log,time.monotonic()-tick,flush=True)
  if path is None:continue
  guide=np.vstack((p[s<at],path[:,:2]));cfg=dict(CONFIG);cfg.update(adaptive_constraint_stations=True)
  pp,model,dd=optimize(w,z['mask'],guide,start,goal,time.monotonic()+15,cfg)
  print('OPT',radius,dist,dd.get('success'),dd.get('failure'),flush=True);tag=f'{radius}_{dist}';(out/(tag+'.json')).write_text(json.dumps({'search':log,'optimize':dd,'model':model}));np.save(out/(tag+'_guide.npy'),guide)
  if pp is not None:np.save(out/'path.npy',pp);sys.exit(0)
