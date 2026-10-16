from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from replay_curve import load
ROOT=Path(__file__).resolve().parents[1]
font_manager.fontManager.addfont('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc')
plt.rcParams.update({'font.family':'SimSun','axes.unicode_minus':False})
fig,axes=plt.subplots(1,2,figsize=(13,6))
for ax,(mid,qid) in zip(axes,[('3yd_synth_trizone_c_v2','Q10'),('pudu_wanda_3f','Q08')]):
 p=sorted((ROOT/'results/diagnostic_control'/mid/'curve_inputs').glob(qid+'_*.npz'))[0]
 world,a,meta=load(p);g=np.array([238.1,262.4,0.]) if mid=='3yd_synth_trizone_c_v2' else np.array([-44.3,0.,0.]);res=world.map.resolution;ox,oy=world.map.origin[:2];h,w=world.hard.shape
 c=int((g[0]-ox)/res);r=h-1-int((g[1]-oy)/res);d=int(5/res)
 r0=max(0,r-d);r1=min(h,r+d);c0=max(0,c-d);c1=min(w,c+d)
 bg=np.full((r1-r0,c1-c0,3),.92);hard=world.hard[r0:r1,c0:c1];mask=a['mask'][r0:r1,c0:c1]
 bg[mask]=[.8,.93,.84];bg[hard]=[.15,.15,.15]
 ax.imshow(bg,extent=(ox+c0*res,ox+c1*res,oy+(h-r1)*res,oy+(h-r0)*res),origin='upper')
 guide=a['guide'];ax.plot(guide[:,0],guide[:,1],color='#2077a4',label='L2参考')
 from scipy.interpolate import BSpline
 curve=json.loads((ROOT/'results/certificate_diagnostic'/mid/'0.json').read_text())['curve']
 spline=BSpline(curve['knots'],curve['coefficients'],3);u=np.linspace(spline.t[3],spline.t[-4],15000);v=spline(u)
 ax.plot(v[:,0],v[:,1],color='#bd2331',label='被拒绝候选')
 ax.set(xlim=(g[0]-5,g[0]+5),ylim=(g[1]-5,g[1]+5),aspect='equal',xlabel='x（米）',ylabel='y（米）',title=qid+' 转弯局部诊断')
 ax.legend(loc='upper left')
fig.suptitle('黑色：原障碍；浅绿：既有L3范围；红线：未经认证的候选曲线')
fig.tight_layout();fig.savefig(ROOT/'report/corner_diagnostic.png',dpi=140)
