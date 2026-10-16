"""Diagnostic: pose-aware bounded connections along the existing L2 order."""
import json,time,math,os
from pathlib import Path
import numpy as np
from replay_curve import load
from three_yd_v5.curve_l3 import clean_guide,interp
from three_yd_v5.failure_terminal import search
from three_yd_v5.v10_storage import collision_checker

R=Path(__file__).resolve().parents[1];O=R/'results/pose_chain_probe';O.mkdir(exist_ok=False)
os.sched_setaffinity(0,{14})
w,d,m=load(R/'results/scope_recovery_probe/pudu_wanda_3f/curve_inputs/Q09_004.npz')
p,s=clean_guide(d['guide'],d['start'],d['goal'],.22)
hard=w.hard|~d['physical'];checker=collision_checker(hard,w)
anchors=[d['start']];stations=[0.]
for nominal in np.arange(5.,s[-1]-3.,5.):
    choices=nominal+np.array([0.,-.5,.5,-1.,1.,-1.5,1.5,-2.,2.])
    for at in choices:
        if at-stations[-1]<2. or s[-1]-at<3.:continue
        xy=interp(p,s,[at])[0];before,after=interp(p,s,[at-.3,at+.3]);v=after-before
        if np.linalg.norm(before+after-2*xy)>.003:continue
        pose=np.r_[xy,math.atan2(v[1],v[0])]
        if not checker.check(pose[None,:]):continue
        anchors.append(pose);stations.append(at);break
anchors.append(d['goal']);stations.append(s[-1]);records=[];allpaths=[]
for j,(a,b) in enumerate(zip(anchors[:-1],anchors[1:])):
    record={'index':j,'start':a.tolist(),'goal':b.tolist(),'stations':stations[j:j+2],'trials':[]};path=None
    for radius in [.8,.5,.405]:
        tick=time.monotonic();seed,log=search(a,b,w,hard,.7,radius,max_nodes=300000)
        log.update(radius=radius,wall_ms=(time.monotonic()-tick)*1000);record['trials'].append(log)
        if seed is not None:path=seed;break
    record['body_feasible']=path is not None
    if path is not None:np.save(O/f'{j}.npy',path)
    records.append(record);allpaths.append(path)
    print(j,stations[j:j+2],path is not None,record['trials'][-1],flush=True)
    (O/'records.json').write_text(json.dumps(records,indent=2))
if all(x is not None for x in allpaths):
    path=np.vstack([allpaths[0],*[x[1:] for x in allpaths[1:]]])
    assert checker.check(path)
    np.save(O/'whole_pose_seed.npy',path)
    print('WHOLE_DISCRETE_BODY_VALID_NOT_NAVIGATION_SUCCESS',flush=True)
