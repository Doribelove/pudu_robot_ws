"""Actual public persistent API, no mocked planner/ACK, one isolated generation."""
import os,sys,tempfile,json,hashlib,math,threading
from pathlib import Path
import numpy as np
from PIL import Image
from trilayer_p6_runtime.api import MapOnlyRequest,SemanticRequest
from trilayer_p6_runtime.session import P6Session,SessionStateError
assert os.environ['ROS_LOCALHOST_ONLY']=='1' and os.environ['ROS_DOMAIN_ID']=='179'
mode=sys.argv[2];root=Path(tempfile.mkdtemp(prefix='trilayer-public-session-'+mode+'-'));size=400 if mode=='semantic' else 240
pixels=np.full((size,size),254,np.uint8);pixels[:4,:]=0;pixels[-4:,:]=0;pixels[:,:4]=0;pixels[:,-4:]=0
Image.fromarray(pixels).save(root/'map.pgm');mp=root/'map.yaml';mp.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
start,goal=((3.,10.,0.),(17.,10.,0.)) if mode=='semantic' else ((2.,4.,0.),(5.,4.,0.))
request=MapOnlyRequest(mp,start,goal,root/'runtime',query_id='persistent')
if mode=='semantic':
 regions=[{'id':i,'name':name,'kind':kind} for i,name,kind in [(1,'west','lane'),(2,'junction','junction_area'),(3,'east','lane')]]
 features=[]
 for r,(x0,y0,x1,y1) in zip(regions,[(1,8,8,12),(8,8,12,12),(12,8,19,12)]):features.append({'semantic_id':r['name'],'semantic_class':r['kind'],'geometry_type':'polygon','properties':{'source_name':r['name']},'coordinates':[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]})
 sem={'schema_version':'SemanticMapV1','frame_id':'map','resolution':.05,'origin':[0,0,0],'width':size,'height':size,'source_pdmap_hash':hashlib.sha256((root/'map.pgm').read_bytes()).hexdigest(),'features':features,'traffic_rules':{}}
 top={'regions':regions,'edges':[{'kind':'shared_outline','regions':[1,2],'geometry':[[8,8],[8,12]]},{'kind':'shared_outline','regions':[2,3],'geometry':[[12,8],[12,12]]}]}
 for name,obj in [('semantic.json',sem),('topology.json',top),('parking_access.json',{'bindings':[]})]:(root/name).write_text(json.dumps(obj))
 request=SemanticRequest(map_yaml=mp,start=start,goal=goal,output_dir=root/'runtime',query_id='persistent',semantic_json=root/'semantic.json',topology_json=root/'topology.json',parking_access_json=root/'parking_access.json')
s=P6Session(request,solver_root=Path(sys.argv[1]));events=[]
try:
 with s:
  first=s.plan();events.append(first);assert first['success'] and first['path'],first
  pid=first['native_pid'];ctx=s.map_context();assert ctx['hard_mask_sha256']==hashlib.sha256(np.flipud(pixels==0).astype(np.uint8).tobytes()).hexdigest()
  polygon=[[3.25,3.25],[3.75,3.25],[3.75,4.75],[3.25,4.75]] if mode=='map' else [[3,2],[3.5,2],[3.5,3],[3,3]]
  upd=s.update(1,upsert={'box':polygon},current_pose=start);events.append(upd);assert upd['result']['update']['accepted'];assert upd['success'],upd
  assert upd['dynamic_version']==1 and upd['trajectory_version']==1 and upd['native_pid']==pid
  assert s.current_trajectory()['available'];assert s.map_context()['effective_hard_mask_sha256']!=ctx['effective_hard_mask_sha256'];assert s.map_context()['static_hard_mask_sha256']==ctx['static_hard_mask_sha256'];assert s.map_context()['hard_mask_sha256']==ctx['static_hard_mask_sha256']
  duplicate=s.update(1,upsert={'box':polygon},current_pose=start);events.append(duplicate);assert duplicate['result']['update']['reason']=='DUPLICATE'
  stale=s.update(0);events.append(stale);assert stale['result']['update']['reason']=='STALE_SEQUENCE'
  negative_stale=s.update(-2);assert negative_stale['result']['update']['reason']=='STALE_SEQUENCE'
  gap=s.update(4);events.append(gap);assert gap['dynamic_version']==1
  clear=s.update(5,clear=['box'],observed_free=True,current_pose=start);events.append(clear);assert clear['success'] and clear['dynamic_version']==2
  assert s.map_context()['effective_hard_mask_sha256']==ctx['effective_hard_mask_sha256'];assert s.map_context()['static_hard_mask_sha256']==ctx['static_hard_mask_sha256']
  bad=s.update(6,clear=['box'],observed_free=False);events.append(bad);assert not bad['success'] and not bad['path'] and 'CLEAR_REQUIRES_FREE_OBSERVATION' in bad['exception'];assert not s.current_trajectory()['available']
  conflict=s.update(5,upsert={'box':polygon});events.append(conflict);assert not conflict['success'] and 'CONFLICTING_SAME_SEQUENCE' in conflict['exception'];assert not s.current_trajectory()['available']
  replanned=s.plan();events.append(replanned);assert replanned['success'] and replanned['native_pid']==pid
  try:s.update(True,upsert={'box':polygon})
  except ValueError:pass
  else:raise AssertionError('bool sequence accepted')
  assert not s.current_trajectory()['available']
  assert s.plan()['success']
  try:s.update(6,upsert={'box':[[0,0],[1,1],[0,1],[1,0]]})
  except ValueError:pass
  else:raise AssertionError('self crossing polygon accepted')
  assert not s.current_trajectory()['available']
  typed=s.update(6,upsert={'typed':[[1,1],[2,1],[2,2],[1,2]]},current_pose=start);assert typed['success']
  typed_conflict=s.update(6,upsert={'typed':[[1.,1.],[2.,1.],[2.,2.],[1.,2.]]},current_pose=start)
  assert not typed_conflict['success'] and 'CONFLICTING_SAME_SEQUENCE' in typed_conflict['exception']
  assert s.plan()['success']
  if mode=='semantic':
   try:s.plan(start=(3.1,10.,0.))
   except ValueError:pass
   else:raise AssertionError('unvalidated semantic endpoint override accepted')
   assert not s.current_trajectory()['available']
  else:
   shifted=s.plan((2.,5.,0.),(5.,5.,0.));assert shifted['success'];assert np.allclose(shifted['path'][-1],[5.,5.,0.]);assert shifted['native_pid']==pid
  # Direct operation-lock contention test; no competing ROS request is issued.
  s._lock.acquire()
  try:
   try:s.current_trajectory()
   except SessionStateError:pass
   else:raise AssertionError('concurrent operation accepted')
  finally:s._lock.release()
finally:
 closed=s.close();assert closed['worker_exit']==0 and not closed['cleanup_errors'],closed
assert s.close()==closed
for op in [s.open,s.plan,s.current_trajectory,s.map_context,lambda:s.update(7)]:
 try:op()
 except SessionStateError:pass
 else:raise AssertionError('closed session accepted operation')
(root/'events.json').write_text(json.dumps(events,indent=2))
print('PASS public persistent',mode,'same native PID',pid,'events',len(events),'normal close; evidence',root)
