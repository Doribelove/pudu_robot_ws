"""Diagnostic: sub-grid switching times at the unresolved fixed endpoint.

Search two bounded curvature arcs only. Success here remains an initializer,
not a certified spline, full navigation success, or infeasibility proof.
"""
import json,math,os
from pathlib import Path
import numpy as np
from replay_curve import load
from three_yd_v5.v10_storage import collision_checker

R=Path(__file__).resolve().parents[1]
os.sched_setaffinity(0,{16})
w,d,m=load(R/'results/diagnostic_initial/mentor_map_20260825_005/curve_inputs/A2B-07_000.npz')
a=d['goal'].copy();a[2]+=math.pi;check=collision_checker(w.hard,w)
out=R/'results/mentor_switch_control_probe_b';out.mkdir(exist_ok=False)

def advance(a,k,ss):
    yaw=a[2]+k*ss
    if k:return np.c_[a[0]+(np.sin(yaw)-math.sin(a[2]))/k,a[1]-(np.cos(yaw)-math.cos(a[2]))/k,yaw]
    return np.c_[a[0]+ss*math.cos(a[2]),a[1]+ss*math.sin(a[2]),yaw]

rows=[]
for sign in [1,-1]:
    for distance in np.arange(0,.701,.0025):
        first=advance(a,sign/w.vehicle[2],np.r_[np.arange(0,distance,.002),distance])
        if not check.check(first):continue
        length=(math.pi+distance/w.vehicle[2])*w.vehicle[2]
        second=advance(first[-1],-sign/w.vehicle[2],np.linspace(0,length,int(length/.002)+1))
        path=np.vstack([first,second[1:]])
        code=check.fn(check.hard,*check.hard.shape,path,len(path),check.res,*check.origin[:2],check.half_length,check.half_width)
        good=int(code)==0
        rows.append({'first_sign':sign,'first_distance':float(distance),'body_valid':good,'first_bad_index':int(code)})
        if good:np.save(out/f'{sign}_{distance:.4f}.npy',path)
(out/'records.json').write_text(json.dumps(rows,indent=2))
(out/'summary.json').write_text(json.dumps({'tested':len(rows),'body_valid':sum(x['body_valid'] for x in rows),'full_navigation':False,'physical_infeasibility_proven':False},indent=2))
print(json.loads((out/'summary.json').read_text()),flush=True)
