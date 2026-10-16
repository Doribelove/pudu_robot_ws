from pathlib import Path
import json,time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties,fontManager
from crossmap_inputs import load_world
from three_yd_v5.v10_storage import collision_checker
E=Path(__file__).resolve().parents[1];P=Path('/home/robot/workspaces/3ydl3_v9_crossmapfix_20261011/results/inspect');O=E/'results/endpoint_geometry';O.mkdir(parents=True,exist_ok=True)
font=Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc');fontManager.addfont(str(font));fp=FontProperties(fname=str(font));plt.rcParams.update({'font.family':fp.get_name(),'axes.unicode_minus':False})
logs=[]
for mid,qid in [('arena_medium_200x200_2d_005','A2B-04'),('mentor_map_20260825_005','A2B-07'),('mentor_map_20260825_005','A2B-03')]:
 out=O/f'{mid}_{qid}';out.mkdir(exist_ok=True);_,_,w,_,_,_=load_world(mid,out);z=np.load(P/out.name/'rank1.npz');goal=z['goal'];start=z['start'];guide=z['guide'];row={'map':mid,'query':qid,'goal':goal.tolist(),'vehicle':w.vehicle,'trials':{}}
 fig,axes=plt.subplots(1,2,figsize=(12,5.5))
 for ax,key in zip(axes,('physical','mask')):
  hard=w.hard|~z[key];checker=collision_checker(hard,w);r,c=w.cells(goal[None,:2]);r=int(r[0]);c=int(c[0]);n=80;r0=max(0,r-n);r1=min(hard.shape[0],r+n);c0=max(0,c-n);c1=min(hard.shape[1],c+n)
  img=np.ones((r1-r0,c1-c0,3));img[w.hard[r0:r1,c0:c1]]=.18;img[(~z[key]&~w.hard)[r0:r1,c0:c1]]=[1.,.8,.5]
  x0,y0=w.map.origin[:2];res=w.map.resolution;H=hard.shape[0];ax.imshow(img,extent=[x0+c0*res,x0+c1*res,y0+(H-r1)*res,y0+(H-r0)*res]);ax.plot(guide[:,0],guide[:,1],'-',label='L2参考线',color='C0');ax.scatter(*goal[:2],color='red');ax.arrow(*goal[:2],.5*np.cos(goal[2]),.5*np.sin(goal[2]),width=.025,color='red');ax.set_xlim(goal[0]-4,goal[0]+4);ax.set_ylim(goal[1]-4,goal[1]+4)
  a,b=w.vehicle[:2];body=np.array([[-a,-b],[-a,b],[a,b],[a,-b],[-a,-b]]);rot=np.array([[np.cos(goal[2]),-np.sin(goal[2])],[np.sin(goal[2]),np.cos(goal[2])]]);poly=body@rot.T+goal[:2];ax.plot(poly[:,0],poly[:,1],'r-')
  row['trials'][key]=[]
  for k in (0.,-2.5,2.5,-1.25,1.25):
   ss=-np.linspace(0,2,201);yaw=goal[2]+k*ss
   xy=goal[:2]+(np.c_[(np.sin(yaw)-np.sin(goal[2]))/k,(-np.cos(yaw)+np.cos(goal[2]))/k] if k else ss[:,None]*[np.cos(goal[2]),np.sin(goal[2])]);poses=np.c_[xy,yaw]
   good=0
   for j in range(1,len(poses)):
    if not checker.check(poses[j-1:j+1]):break
    good=j
   ax.plot(poses[:good+1,0],poses[:good+1,1],'-',linewidth=2)
   row['trials'][key].append({'curvature':k,'clear_backward_arc_m':float(-ss[good]),'goal_pose_clear':bool(checker.check(goal[None,:]))})
  ax.set_title('物理范围' if key=='physical' else '当前L3范围',fontproperties=fp);ax.set_xlabel('米',fontproperties=fp);ax.set_ylabel('米',fontproperties=fp);ax.set_aspect('equal');ax.legend(prop=fp)
 fig.suptitle(mid+' / '+qid+'：目标姿态与反向探测（非最终轨迹）',fontproperties=fp);fig.tight_layout();fig.savefig(out/'goal.png',dpi=150);plt.close(fig);logs.append(row);print(json.dumps(row),flush=True)
(O/'summary.json').write_text(json.dumps(logs,ensure_ascii=False,indent=2))
