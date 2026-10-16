"""Obstacle updates after parking/task progress; static replay, not execution."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
import argparse,os,json,hashlib,signal,time,shutil
from pathlib import Path
from dataclasses import replace
import numpy as np
from benchmark import Memory
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from three_yd_v5.v9_gate_init import DirectStorageAuditor as StorageAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.semantic_world import SemanticWorld,json_write
from arena_3d_v1.multires_grid import GridView
from three_yd_v5.free_space_topology import FreeSpaceTopology,load_map_only_world
from three_yd_v5.topology_default import DefaultTopology
from three_yd_v5.curve_session import CurveSession as PerformanceSession
from three_yd_v5.task_progress import TaskProgressService
from run_free_full import registered_input
from run_wanda3w import json_native,deadline_handler
from run_dynamic_fullroute import square
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--map-id',required=True);p.add_argument('--output',required=True,type=Path)
 p.add_argument('--l3-backend',choices=['control','independent','compatibility'],default='independent')
 p.add_argument('--suite',choices=['events','continuation','isolated','latest','recovery'],default='continuation');p.add_argument('--queries',nargs='+');a=p.parse_args()
 selected=json.loads((ROOT/'BASELINE.json').read_text())
 envkeys={'v9_mode':'THREEYD_V9_MODE','review_mode':'THREEYD_V9_R1_MODE',**{f'r{i}_mode':f'THREEYD_V9_R{i}_MODE' for i in (2,3,5,6,7,8,9)}}
 for key,name in envkeys.items():os.environ[name]=selected[key]
 os.environ['THREEYD_L3_BACKEND']=a.l3_backend;os.environ['ROS_DOMAIN_ID']='231'
 out=a.output;out.mkdir(parents=True,exist_ok=False)
 if a.map_id=='mengjia':
  mp=ROOT/'inputs/corroborated_map.yaml';w=SemanticWorld(mp,ROOT/'inputs/semantic.json',cache_root=out/'world')
  g=DefaultTopology(w,ROOT/'inputs/topology.json',out/'graph');qs=json.loads((ROOT/'inputs/queries.json').read_text())['scenarios']
 else:
  d,_,qs=registered_input(a.map_id);mp=d/'map.yaml';w=load_map_only_world(mp,out/'world');g=FreeSpaceTopology(w,out/'graph')
 def pose(v):return tuple(v[k] for k in ('x','y','yaw_rad')) if isinstance(v,dict) else tuple(v)
 queries=[Query(x['id'],pose(x['start']),pose(x['goal'])) for x in qs if not a.queries or x['id'] in a.queries]
 if a.suite in ('events','recovery'):queries=[next(q for q in queries if q.query_id==('Q09' if a.map_id=='mengjia' else 'A2B-01'))]
 ctx=runtime.MapContext(a.map_id,w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
 auditor=StorageAuditor(ctx,source_commit='3YDL3-V8-regression');spec=runtime.backend_availability()['hybrid_astar']
 sources=[Path(__file__).resolve(),*(ROOT/'src').rglob('*.py'),*(ROOT/'tools').glob('*.py'),ROOT/'env.bash',ROOT/'config/l3.json']
 snap=out/'source_snapshot';snap.mkdir()
 for f in sources:shutil.copy2(f,snap/str(f.relative_to(ROOT)).replace('/','_'))
 json_write(snap/'hashes.json',{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources})
 json_write(out/'protocol.json',{'version':'3YDL3-V8','map':a.map_id,'backend':a.l3_backend,'suite':a.suite,
  'queries':[q.query_id for q in queries],'latest_suite':'sequential actual 3cm/2deg offsets from each newly accepted curve, nominal cumulative progress 20/50/80 percent',
  'poses':'isolated: one 3cm/2deg request per fresh task at 20/50/80% of its latest accepted full path; continuation: sequential replay from the fixed INITIAL path',
  'comparison_scope':'functional regression; backend-specific trajectories are not paired identical moving-pose performance inputs',
  'CPU':sorted(os.sched_getaffinity(0)),'static_replay':True,'live_execution':False})
 s=PerformanceSession(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',
  smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,
  planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True)
 s.local_map_update_strategy='roi_ack';s.full_grid_settle_cycles=0
 service=TaskProgressService(w,g,out/'l2_cache',{3:GridView(w,3),1:GridView(w,1)},s,auditor,spec,max_region_routes=3 if a.map_id=='mengjia' else 1)
 events=[];current=queries[0]
 def save(label,fn):
  from three_yd_v5.curve_budget import L3RequestBudgetExceeded
  from three_yd_v5.v9_maps import content_digest
  s.reset_query_state(label,restore_base_map=False);tick=time.monotonic()
  try:
   with Memory() as mem:r=fn()
  except (Exception,L3RequestBudgetExceeded) as e:
   import traceback
   (out/(label+'_exception.txt')).write_text(traceback.format_exc());r={'success':False,'failure_code':type(e).__name__+':'+str(e)}
  r=dict(r);r.pop('result',None);path=service.current_trajectory()
  if path is not None:
   np.save(out/(label+'_path.npy'),path)
   body=w.audit(current,path,service.controller._target_mask());traffic=service.controller.traffic.audit(path)
   assert body['valid'] and traffic['valid'],(label,body,traffic)
   r['whole_suffix_body_audit']=body;r['original_task_traffic_audit']=traffic
  if 'success' in r:assert r['success'] or path is None,(label,'FAILED_REQUEST_EXPOSES_TRAJECTORY')
  if a.l3_backend=='independent':assert all(x.get('old_l3_calls',0)==0 for x in r.get('l3_backend',{}).get('attempts',[]))
  if service.controller is not None:
   c=service.controller;r['l1_signature']=str(c.topology_route.signature);r['l2_digest']=content_digest(c.reference_plan.world_xy)
  r.update(label=label,query_id=current.query_id,start=current.start,goal=current.goal,trajectory_available=path is not None,
    wall_ms=(time.monotonic()-tick)*1000,rss_peak_bytes=mem.peak)
  events.append(r)
  with (out/'events.jsonl').open('a') as journal:journal.write(json.dumps(r,ensure_ascii=False,sort_keys=True,allow_nan=False)+'\n')
  print(label,r.get('success'),r.get('action'),r.get('failure_code'),round(r['wall_ms']),flush=True);return r
 try:
  s.start();from native_snapshot import record
  record(out)
  if a.suite=='isolated':
   for q in queries:
    for fraction in (.2,.5,.8):
     service.close();s.active_controller=None;s.active_route=None
     service=TaskProgressService(w,g,out/'l2_cache',{3:GridView(w,3),1:GridView(w,1)},s,auditor,spec,max_region_routes=3 if a.map_id=='mengjia' else 1)
     current=q;label=q.query_id+'_isolated_p'+str(int(fraction*100))
     first=save(label+'_initial',lambda:service.plan(q))
     if not first.get('success'):
      events.append({'label':label+'_unformed','unformed':1,'reason':'initial strict failure'});continue
     base=service.current_trajectory().copy();arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1))]
     index=min(len(base)-2,int(np.searchsorted(arc,fraction*arc[-1])));reference=base[index].copy()
     sign=-1 if fraction==.5 else 1;v=reference.copy()
     v[:2]+=sign*.03*np.array([-np.sin(v[2]),np.cos(v[2])]);v[2]=(v[2]+sign*np.deg2rad(2)+np.pi)%(2*np.pi)-np.pi
     current=replace(q,start=tuple(v));got=save(label,lambda:service.plan(current))
     got['offset_reference_pose']=reference.tolist();got['actual_position_offset_m']=float(np.linalg.norm(v[:2]-reference[:2]))
     got['actual_yaw_offset_deg']=float(abs((v[2]-reference[2]+np.pi)%(2*np.pi)-np.pi)*180/np.pi)
     assert abs(got['actual_position_offset_m']-.03)<1e-10 and abs(got['actual_yaw_offset_deg']-2)<1e-10
  elif a.suite=='latest':
   for q in queries:
    service.close();s.active_controller=None;s.active_route=None
    service=TaskProgressService(w,g,out/'l2_cache',{3:GridView(w,3),1:GridView(w,1)},s,auditor,spec,max_region_routes=3 if a.map_id=='mengjia' else 1)
    current=q;first=save(q.query_id+'_latest_initial',lambda:service.plan(q));previous=0.
    if not first.get('success'):
     events.append({'label':q.query_id+'_latest_unformed','unformed':3,'reason':'initial strict failure'});continue
    for fraction in (.2,.5,.8):
     base=service.current_trajectory()
     if base is None:
      events.append({'label':q.query_id+'_latest_unformed','unformed':1,'reason':'previous continuation failed'});continue
     arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1))]
     local_fraction=(fraction-previous)/(1-previous);previous=fraction
     index=min(len(base)-2,int(np.searchsorted(arc,local_fraction*arc[-1])));reference=base[index].copy()
     sign=-1 if fraction==.5 else 1;v=reference.copy()
     v[:2]+=sign*.03*np.array([-np.sin(v[2]),np.cos(v[2])]);v[2]=(v[2]+sign*np.deg2rad(2)+np.pi)%(2*np.pi)-np.pi
     current=replace(q,start=tuple(v));got=save(q.query_id+'_latest_p'+str(int(fraction*100)),lambda:service.plan(current))
     got['offset_reference_pose']=reference.tolist();got['fraction_of_current_remaining_path']=local_fraction
     got['actual_position_offset_m']=float(np.linalg.norm(v[:2]-reference[:2]))
     got['actual_yaw_offset_deg']=float(abs((v[2]-reference[2]+np.pi)%(2*np.pi)-np.pi)*180/np.pi)
     assert abs(got['actual_position_offset_m']-.03)<1e-10 and abs(got['actual_yaw_offset_deg']-2)<1e-10
  elif a.suite=='continuation':
   for q in queries:
    for offset in (False,True):
     current=q;kind='offset' if offset else 'exact';label=q.query_id+'_'+kind
     first=save(label+'_initial',lambda:service.plan(q))
     if not first.get('success'):
      events.append({'label':label+'_unformed','unformed':3,'reason':'initial strict failure'});continue
     base=service.current_trajectory().copy();arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1))]
     for fraction in (.2,.5,.8):
      index=min(len(base)-2,int(np.searchsorted(arc,fraction*arc[-1])));v=base[index].copy()
      if offset:
       sign=-1 if fraction==.5 else 1;v[:2]+=sign*.03*np.array([-np.sin(v[2]),np.cos(v[2])]);v[2]=(v[2]+sign*np.deg2rad(2)+np.pi)%(2*np.pi)-np.pi
      before_signature=None if service.controller is None else str(service.controller.topology_route.signature)
      from three_yd_v5.v9_maps import content_digest
      before_l2=None if service.controller is None else content_digest(service.controller.reference_plan.world_xy)
      current=replace(q,start=tuple(v));got=save(label+'_p'+str(int(fraction*100)),lambda:service.plan(current))
      if got.get('success') and got.get('task_progress',{}).get('L1_search_called') is False:
       assert before_signature==got['l1_signature'] and before_l2==got['l2_digest']
  else:
   q=current;first=save('initial',lambda:service.plan(q))
   if first.get('success'):
    base=service.current_trajectory();arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1))]
    j=int(np.searchsorted(arc,.8*arc[-1]));k=int(np.searchsorted(arc,.2*arc[-1]));current=replace(q,start=tuple(base[j]))
    progress=save('progress80',lambda:service.plan(current))
    if progress.get('success'):
     tail=service.current_trajectory().copy();past=square(base[k,:2],.08)
     old=save('past_add',lambda:service.update(1,upsert={'past':past}));assert old['success'] and not old['replanned']
     assert np.array_equal(tail,service.current_trajectory())
     duplicate=save('duplicate',lambda:service.update(1,upsert={'past':past}));assert duplicate['update']['reason']=='DUPLICATE'
     stale=save('stale',lambda:service.update(0,upsert={}));assert not stale['update']['accepted']
     try:service.update(2,clear=['past'])
     except ValueError as e:
      assert str(e)=='CLEAR_REQUIRES_FREE_OBSERVATION';events.append({'label':'unobserved_clear_rejected','passed':True})
     else:raise AssertionError('unobserved clear accepted')
     save('past_clear',lambda:service.update(2,clear=['past'],observed_free=True))
     obstruction=square(tail[len(tail)//2,:2],.08)
     near=save('future_add',lambda:service.update(3,upsert={'future':obstruction}));assert near['replanned']
     save('future_clear',lambda:service.update(4,clear=['future'],observed_free=True))
     blocked=save('current_block',lambda:service.update(5,upsert={'current':square(current.start[:2],.4)}))
     assert not blocked['success'] and not blocked['trajectory_available']
     if a.suite=='recovery':
      shifted=np.array(current.start,float)
      shifted[:2]+=.03*np.array([-np.sin(shifted[2]),np.cos(shifted[2])]);shifted[2]=(shifted[2]+np.deg2rad(2)+np.pi)%(2*np.pi)-np.pi
      current=replace(current,start=tuple(shifted))
      failed_pose=save('blocked_offset',lambda:service.plan(current))
      assert not failed_pose['success'] and service.current_trajectory() is None
     recovered=save('current_clear',lambda:service.update(6,clear=['current'],observed_free=True))
     if a.suite=='recovery':
      final=service.current_trajectory();assert recovered['success'] and final is not None
      recovered['actual_start_error_m']=float(np.linalg.norm(final[0,:2]-shifted[:2]))
      recovered['actual_start_yaw_error_deg']=float(abs((final[0,2]-shifted[2]+np.pi)%(2*np.pi)-np.pi)*180/np.pi)
      assert recovered['actual_start_error_m']<1e-8 and recovered['actual_start_yaw_error_deg']<1e-6

  json_write(out/'events.json',events)
  json_write(out/'summary.json',{'version':'3YDL3-V8','backend':a.l3_backend,'suite':a.suite,'map':a.map_id,
   'requests':sum('success' in e for e in events),'success':sum(bool(e.get('success')) for e in events),
   'safety_assertions_passed':True,'static_replay':True,'live_execution':False})
 finally:service.close();s.close()
if __name__=='__main__':main()
