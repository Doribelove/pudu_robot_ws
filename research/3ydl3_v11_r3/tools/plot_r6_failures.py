from pathlib import Path
import numpy as np,json
import matplotlib;matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.patches import Polygon
from replay_curve import load
R=Path(__file__).resolve().parents[1];font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc';font_manager.fontManager.addfont(font);plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name();plt.rcParams['axes.unicode_minus']=False
A=Path('/home/robot/workspaces/3ydl3_v10_r4_tailfix_20261012/results/diagnostic_control');fig,ax=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
for i,(mid,qid,title) in enumerate([('mentor_map_20260825_005','A2B-07','导师原图 A2B-07：终点'),('arena_medium_200x200_2d_005','A2B-04','Arena 中图 A2B-04：终点')]):
 w,d,m=load(next((A/mid/'curve_inputs').glob(qid+'_*.npz')));pose=d['goal'];res=w.map.resolution;ox,oy=w.map.origin[:2];h,wd=w.hard.shape
 x,y=pose[:2];c0=max(0,int((x-3-ox)/res));c1=min(wd,int((x+3-ox)/res));r0=max(0,h-int((y+3-oy)/res));r1=min(h,h-int((y-3-oy)/res));colors=np.ones((r1-r0,c1-c0,3));colors[~d['mask'][r0:r1,c0:c1]]=[.8,.88,1];colors[w.hard[r0:r1,c0:c1]]=[.25,.25,.25]
 ax[i].imshow(colors,extent=[ox+c0*res,ox+c1*res,oy+(h-r1)*res,oy+(h-r0)*res]);ax[i].plot(d['guide'][:,0],d['guide'][:,1],color='orange');ax[i].arrow(x,y,.6*np.cos(pose[2]),.6*np.sin(pose[2]),color='red',width=.03);ax[i].set_xlim(x-3,x+3);ax[i].set_ylim(y-3,y+3);ax[i].set_title(title)
p=R/'results/l2_probe/diagnostics';d=np.load(p/'failure.npz');m=json.loads((p/'failure.json').read_text());x,y=23.5,13.8;res=m['resolution'];ox,oy=m['origin'][:2];h,wd=d['fine_hard'].shape;c0=int((x-2-ox)/res);c1=int((x+2-ox)/res);r0=h-int((y+2-oy)/res);r1=h-int((y-2-oy)/res);ax[2].imshow(d['fine_hard'][r0:r1,c0:c1],cmap='gray_r',extent=[ox+c0*res,ox+c1*res,oy+(h-r1)*res,oy+(h-r0)*res],vmin=0,vmax=1)
raw=d['point_raw'];cx=(raw[:,1]+.5)*.15;cy=(d['circle_safe'].shape[0]-raw[:,0]-.5)*.15;ax[2].plot(cx,cy,color='orange');ax[2].set_xlim(x-2,x+2);ax[2].set_ylim(y-2,y+2);ax[2].set_title('导师原图 A2B-03：L2 断连位置')
for a in ax:a.set_xlabel('x / m');a.set_ylabel('y / m');a.set_aspect('equal')
fig.suptitle('诊断图：灰色为障碍，蓝色为局部范围外，橙色为参考线，红箭头为终点朝向');fig.savefig(R/'report/failure_locations.png',dpi=160)
