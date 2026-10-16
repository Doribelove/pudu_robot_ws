import sys,time,json
from pathlib import Path
import numpy as np
from crossmap_inputs import load_world
from three_yd_v5.curve_l3 import feasible_guide,interp,optimize,CONFIG
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'experiments/diagnostic_rejected'))
from curve_endpoint_search import connect
ROOT=Path(__file__).resolve().parents[1];mid,qid=sys.argv[1:3];out=ROOT/'results/local_endpoint'/f'{mid}_{qid}';out.mkdir(parents=True,exist_ok=True)
_,_,w,_,_,_=load_world(mid,out);z=np.load(ROOT/'results/inspect'/f'{mid}_{qid}'/'rank1.npz');start=z['start'];goal=z['goal'];p,s,_=feasible_guide(z['guide'],start,goal,w,z['physical'])
for dist in (2.,3.,4.):
 at=s[-1]-dist;xy=interp(p,s,[at])[0];v=np.diff(interp(p,s,[at-.05,at+.05]),axis=0)[0];a=(*xy,np.arctan2(v[1],v[0]))
 tick=time.monotonic();path,log=connect(w,z['mask'],a,goal,tick+3);print('LOCAL',dist,log,time.monotonic()-tick,flush=True)
 if path is None:continue
 guide=np.vstack((p[s<at],path[:,:2]));cfg=dict(CONFIG);cfg.update(adaptive_constraint_stations=True)
 pp,model,dd=optimize(w,z['mask'],guide,start,goal,time.monotonic()+15,cfg)
 print('OPT',dist,dd.get('success'),dd.get('failure'),flush=True);(out/f'{dist}.json').write_text(json.dumps({'search':log,'optimize':dd,'model':model}));np.save(out/f'{dist}_guide.npy',guide)
 if pp is not None:np.save(out/'path.npy',pp);break
