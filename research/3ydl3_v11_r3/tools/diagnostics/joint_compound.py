from pathlib import Path
exec(open(str(Path(__file__).with_name('joint_fine_grid.py'))).read().split('for row in rows:')[0])
import math
for row in rows:
 if row['query_id'] not in ('Q04','Q06','Q10'):continue
 call=row['v9_local']['connections'][-1];p=np.array(call['reference_window_poses']);a=call['window_start_index'];b=call['window_end_index'];mask=np.load(root/'cases'/f"{row['query_id']}_1"/'l3_scope.npz')['mask'];checker=SweptChecker(~mask,m['resolution'],m['origin'],.265,.225)
 u=np.array([math.cos(p[a,2]),math.sin(p[a,2])]);v=np.array([math.cos(p[b,2]),math.sin(p[b,2])]);mat=np.column_stack((u,v));tick=time.monotonic();found=None
 if abs(np.linalg.det(mat))<.1:print(row['query_id'],'parallel');continue
 t,s=np.linalg.solve(mat,p[b,:2]-p[a,:2]);vertex=p[a,:2]+t*u
 for d in (.5,.6,.8,1.,1.2,1.5):
  if min(t,s)<d:continue
  start=np.r_[vertex-d*u,p[a,2]];goal=np.r_[vertex+d*v,p[b,2]]
  for coeff in (1.5,2.,2.5,3.):
   q,k=evaluate(start,goal,0,0,(coeff*d,coeff*d))
   left=np.c_[np.linspace(p[a,:2],start[:2],max(2,int((t-d)/.02)+2)),np.full(max(2,int((t-d)/.02)+2),p[a,2])]
   right=np.c_[np.linspace(goal[:2],p[b,:2],max(2,int((s-d)/.02)+2)),np.full(max(2,int((s-d)/.02)+2),p[b,2])]
   joined=np.vstack((left[:-1],q,right[1:]));splice=np.vstack((p[:a],joined,p[b+1:]))
   if max(abs(k))>2.49 or len(violating_segments(splice,2.5)[0]):continue
   if checker.check(densify(splice)):found={'d':d,'coeff':coeff,'max_k':float(max(abs(k)))};break
  if found:break
 print(row['query_id'],'found',found,'rays',t,s,'ms',(time.monotonic()-tick)*1000,flush=True)
 output[row['query_id']]={'success':found,'rays':[t,s],'wall_ms':(time.monotonic()-tick)*1000}
Path('report/JOINT_COMPOUND_OFFLINE.json').write_text(json.dumps(output,indent=2))
