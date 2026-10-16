from pathlib import Path
import json,os
import numpy as np
from replay_curve import run
from three_yd_v5 import curve_l3
os.sched_setaffinity(0,{14})
ROOT=Path(__file__).resolve().parents[1]
root=ROOT/'results/certificate_diagnostic';root.mkdir(exist_ok=False)
original=curve_l3.certify;state={}
def diagnose(spline,*args,**kwargs):
 try:return original(spline,*args,**kwargs)
 except curve_l3.CurveFailure as e:
  knots=np.unique(spline.t);u=np.unique(np.concatenate([np.linspace(a,b,17) for a,b in zip(knots[:-1],knots[1:])]))
  v=spline(u,nu=1);a=spline(u,nu=2);speed=np.linalg.norm(v,axis=1);k=np.abs(v[:,0]*a[:,1]-v[:,1]*a[:,0])/np.maximum(speed,1e-15)**3
  index=int(np.argmax(k));record={'error':str(e),'max_sampled_curvature':float(k[index]),'u':float(u[index]),'xy':spline(u[index]).tolist(),'min_speed':float(speed.min()),'control_points':len(spline.c)}
  records=state['records'];out=state['out'];tag=str(len(records));records.append(record)
  (out/(tag+'.json')).write_text(json.dumps({'curve':{'knots':spline.t.tolist(),'coefficients':spline.c.tolist(),'degree':3},'diagnostic':record}))
  raise
curve_l3.certify=diagnose
for mid,q in [('3yd_synth_trizone_c_v2','Q10'),('pudu_wanda_3f','Q08')]:
 p=sorted((ROOT/'results/diagnostic_control'/mid/'curve_inputs').glob(q+'_*.npz'))[0]
 out=root/mid;out.mkdir();state.update(records=[],out=out)
 result,record=run(p,{})
 (out/'rejections.json').write_text(json.dumps(state['records'],indent=2))
 print(mid,[(r['error'],round(r['max_sampled_curvature'],3),[round(x,2) for x in r['xy']]) for r in state['records']],flush=True)
