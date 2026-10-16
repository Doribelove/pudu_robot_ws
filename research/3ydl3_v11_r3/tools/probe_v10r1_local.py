import sys,time,json,heapq,math
from pathlib import Path
import numpy as np
from crossmap_inputs import load_world
from three_yd_v5.curve_l3 import feasible_guide,interp,optimize,CONFIG
from three_yd_v5.v10_storage import collision_checker
from arena_evaluation.semantic_constraint_core import dubins_choices,dubins_edge_from_parameters
from arena_3d_v1.semantic_world import densify
E=Path(__file__).resolve().parents[1];mid,qid=sys.argv[1:3];out=E/'results/local_v1'/f'{mid}_{qid}';out.mkdir(parents=True,exist_ok=False);P=Path('/home/robot/workspaces/3ydl3_v9_crossmapfix_20261011/results/inspect')
_,_,w,_,_,_=load_world(mid,out);z=np.load(P/out.name/'rank1.npz');start=z['start'];goal=z['goal'];p,s,_=feasible_guide(z['guide'],start,goal,w,z['physical']);checker=collision_checker(w.hard|~z['mask'],w)
def connect(a,b,dsmax=.10,xybin=.035,nangle=160):
 deadline=time.monotonic()+12;radius=.405
 def key(p):return (round(p[0]/xybin),round(p[1]/xybin),round(p[2]%(2*np.pi)/(2*np.pi/nangle))%nangle)
 def hh(p):return np.linalg.norm(p[:2]-a[:2])+.15*abs(math.atan2(math.sin(p[2]-a[2]),math.cos(p[2]-a[2])))
 nodes=[(b,-1,None,0.)];queue=[(hh(b),0)];best={key(b):0.};expanded=0;shots=0
 while queue and expanded<30000 and time.monotonic()<deadline:
  _,idx=heapq.heappop(queue);pose,parent,edge,cost=nodes[idx]
  if cost>best.get(key(pose),float('inf'))+1e-8:continue
  expanded+=1
  if expanded%10==1:
   for word,pars in sorted(dubins_choices(tuple(a),tuple(pose),radius),key=lambda v:sum(v[1])):
    if radius*sum(pars)+cost>np.linalg.norm(a[:2]-b[:2])+7:continue
    curve=dubins_edge_from_parameters(tuple(a),tuple(pose),radius,word,pars);head=densify(np.vstack((a,curve.samples)));shots+=1
    if not checker.check(head):continue
    parts=[head];j=idx
    while nodes[j][1]>=0:parts.append(nodes[j][2][::-1]);j=nodes[j][1]
    result=np.vstack(parts);result=result[np.r_[True,np.linalg.norm(np.diff(result[:,:2],axis=0),axis=1)>1e-9]]
    return result,{'expanded':expanded,'shots':shots,'nodes':len(nodes)}
  ss=-np.linspace(0,dsmax,6)
  for k in np.linspace(-1/radius,1/radius,9):
   yaw=pose[2]+k*ss;xy=pose[:2]+(np.c_[(np.sin(yaw)-np.sin(pose[2]))/k,(-np.cos(yaw)+np.cos(pose[2]))/k] if abs(k)>1e-10 else ss[:,None]*[np.cos(pose[2]),np.sin(pose[2])]);edge=np.c_[xy,yaw];end=edge[-1];new=cost+dsmax;ky=key(end)
   if new>=best.get(ky,float('inf'))-1e-9 or new>7 or np.linalg.norm(end[:2]-b[:2])>3.5:continue
   if not checker.check(edge):continue
   best[ky]=new;nodes.append((end,idx,edge,new));heapq.heappush(queue,(new+2*hh(end),len(nodes)-1))
 return None,{'expanded':expanded,'shots':shots,'nodes':len(nodes),'queue':len(queue),'reason':'search_exhausted' if not queue else 'budget'}
logs=[]
for dist in (2.,3.):
 at=s[-1]-dist;xy=interp(p,s,[at])[0];d=np.diff(interp(p,s,[at-.05,at+.05]),axis=0)[0];a=np.r_[xy,np.arctan2(d[1],d[0])];tick=time.monotonic();path,log=connect(a,goal);log.update(distance=dist,wall_s=time.monotonic()-tick);print(log,flush=True);logs.append(log)
 if path is None:continue
 np.save(out/'local_path.npy',path);guide=np.vstack((p[s<at],path[:,:2]));np.save(out/'guide.npy',guide)
 for name,cfg in [('default',dict(CONFIG)),('adaptive',dict(CONFIG,adaptive_constraint_stations=True))]:
  pp,model,dd=optimize(w,z['mask'],guide,start,goal,time.monotonic()+15,cfg);print('OPT',name,dd.get('success'),dd.get('failure'),flush=True);(out/(name+'.json')).write_text(json.dumps({'model':model,'diagnostics':dd}));
  if pp is not None:np.save(out/(name+'_path.npy'),pp)
 break
(out/'search.json').write_text(json.dumps(logs,indent=2))
