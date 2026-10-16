import json,yaml
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties,fontManager
from three_yd_v5.v9_joints import evaluate
root=Path('results/firstplan_diagnostic_v2/DP/mengjia');row=next(json.loads(s) for s in (root/'runs.jsonl').read_text().splitlines() if json.loads(s)['query_id']=='Q04');c=row['v9_local']['connections'][-1];p=np.array(c['reference_window_poses']);a=c['window_start_index'];b=c['window_end_index'];m=yaml.safe_load(Path('inputs/corroborated_map.yaml').read_text());mask=np.load(root/'cases/Q04_1/l3_scope.npz')['mask'];res=m['resolution'];ox,oy=m['origin'][:2]
font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc';fontManager.addfont(font);plt.rcParams['font.family']='SimSun';fp=FontProperties(fname=font);assert fp.get_name()=='SimSun'
xlo,xhi=p[:,0].min()-.6,p[:,0].max()+.6;ylo,yhi=p[:,1].min()-.6,p[:,1].max()+.6
cl,ch=[int((x-ox)/res) for x in (xlo,xhi)];rl,rh=[mask.shape[0]-1-int((y-oy)/res) for y in (yhi,ylo)]
fig,ax=plt.subplots(figsize=(6,6));ax.imshow(mask[rl:rh+1,cl:ch+1],extent=(ox+cl*res,ox+(ch+1)*res,oy+(mask.shape[0]-rh-1)*res,oy+(mask.shape[0]-rl)*res),cmap='Greys_r',vmin=0,vmax=1)
ax.plot(p[:,0],p[:,1],'r-',label='原始参考路线');chord=np.linalg.norm(p[b,:2]-p[a,:2])
for x,y in ((1,1),(.5,1),(1,.5),(1.5,1.5),(2,2)):
 q,k=evaluate(p[a],p[b],0,0,(x*chord,y*chord));ax.plot(q[:,0],q[:,1],alpha=.6,label=f'导数长度 {x}/{y}')
ax.scatter(p[[a,b],0],p[[a,b],1],c='blue');ax.quiver(p[[a,b],0],p[[a,b],1],np.cos(p[[a,b],2]),np.sin(p[[a,b],2]),scale=7)
ax.set_title('Q04短直角：实际允许范围与连接候选');ax.set_xlabel('x（米）');ax.set_ylabel('y（米）');ax.legend(prop=fp,fontsize=8);fig.tight_layout();fig.savefig('report/q04_joint_diagnostic.png',dpi=160)
