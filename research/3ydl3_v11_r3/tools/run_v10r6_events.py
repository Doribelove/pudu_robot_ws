"""Obstacle updates after parking/task progress; static replay, not execution."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
import argparse,os,json,hashlib,signal,time,shutil,sys
if os.environ.get('THREEYD_BENCH_SOURCE'):
 sys.path.insert(0,os.environ['THREEYD_BENCH_SOURCE']+'/tools')
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
ROOT=Path(os.environ.get('THREEYD_BENCH_SOURCE',Path(__file__).resolve().parents[1]))
def main():
 p=argparse.ArgumentParser();p.add_argument('--map-id',required=True);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
 out=a.output;out.mkdir(parents=True,exist_ok=False);os.environ['ROS_DOMAIN_ID']='220';signal.signal(signal.SIGALRM,deadline_handler)
 for key,value in {'THREEYD_L3_BACKEND':'independent','THREEYD_V9_MODE':'A','THREEYD_V9_R1_MODE':'S','THREEYD_V9_R2_MODE':'D','THREEYD_V9_R3_MODE':'control','THREEYD_V9_R5_MODE':'C','THREEYD_V9_R6_MODE':'H','THREEYD_V9_R7_MODE':'NT','THREEYD_V9_R8_MODE':'OGR','THREEYD_V9_R9_MODE':'PLMX','THREEYD_V8_R11_MODE':'AB'}.items():os.environ[key]=value
 from crossmap_inputs import load_world
 _,mp,w,g,queries,_=load_world(a.map_id,out)
 qd=next(x for x in queries if x['id']=='Q09') if a.map_id=='mengjia' else (next(x for x in queries if x['id']=='A2B-03') if a.map_id=='mentor_map_20260825_005' else queries[0])
 def pose(v):return tuple(v[k] for k in ('x','y','yaw_rad')) if isinstance(v,dict) else tuple(v)
 q=Query(qd['id'],pose(qd['start']),pose(qd['goal']))
 ctx=runtime.MapContext(a.map_id,w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
 auditor=StorageAuditor(ctx,source_commit='V10-r6-task-switch-events');spec=runtime.backend_availability()['hybrid_astar']
 json_write(out/'audit_storage.json',auditor.storage_diagnostics)
 snap=out/'source_snapshot';snap.mkdir()
 sources=[*(ROOT/'src').rglob('*.py'),*(ROOT/'tools').glob('*.py'),ROOT/'env.bash']
 for f in sources:shutil.copy2(f,snap/str(f.relative_to(ROOT)).replace('/','_'))
 json_write(snap/'hashes.json',{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources})
 s=PerformanceSession(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',
  smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,
  planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True)
 s.local_map_update_strategy='roi_ack';s.full_grid_settle_cycles=0
 service=TaskProgressService(w,g,out/'l2_cache',{3:GridView(w,3),1:GridView(w,1)},s,auditor,spec,
  max_region_routes=3 if a.map_id=='mengjia' else 1)
 json_write(out/'v9_preparation.json',{'r7_mode':os.environ.get('THREEYD_V9_R7_MODE'),'neutral_labels':getattr(w,'v9_r7_neutral_diagnostics',{'enabled':False,'reason':'semantic_world_or_control'}),'excluded_from_query_timing':True})
 events=[];current=q
 def save(label,fn):
  s.reset_query_state(label,restore_base_map=False);tick=time.monotonic()
  with Memory() as mem:
   signal.alarm(90)
   try:r=fn()
   finally:signal.alarm(0)
  r=dict(r);r.pop('result',None);path=service.current_trajectory()
  if path is not None:
   np.save(out/(label+'_path.npy'),path)
   body=w.audit(current,path,service.controller._target_mask());traffic=service.controller.traffic.audit(path)
   assert body['valid'] and traffic['valid'],(label,body,traffic)
   r['whole_suffix_body_audit']=body;r['original_task_traffic_audit']=traffic
  r.update(label=label,trajectory_available=path is not None,wall_ms=(time.monotonic()-tick)*1000,rss_peak_bytes=mem.peak)
  events.append(r);json_write(out/'events.json',events)
  print(label,r.get('success'),r.get('action'),r.get('failure_code'),round(r['wall_ms']),flush=True);return r
 try:
  s.start();from native_snapshot import record
  record(out)
  assert save('initial',lambda:service.plan(q))['success'];base=service.current_trajectory()
  if a.map_id=='mentor_map_20260825_005':
   assert service.controller.reference_mode=='v5_r4_conservative_circle_plus_certified_rectangle'
   blocked=save('block_recovered_start',lambda:service.update(1,upsert={'current':square(q.start[:2],.4)}))
   assert not blocked['success'] and not blocked['trajectory_available']
   restored=save('clear_recovered_start',lambda:service.update(2,clear=['current'],observed_free=True))
   assert restored['success'] and restored['trajectory_available']
   json_write(out/'summary.json',{'map':a.map_id,'r6_localfix_mode':os.environ.get('THREEYD_V10_R6_MODE'),'assertions_passed':True,'events':len(events),'static_event_replay':True,'live_execution':False})
   return
  from scipy.spatial import cKDTree
  candidates=[Query(x['id'],pose(x['start']),pose(x['goal'])) for x in queries if x['id']!=q.query_id]
  tree=cKDTree(base[:,:2]);separation=[tree.query(x.start[:2])[0] for x in candidates]
  b=candidates[int(np.argmax(separation))]
  assert max(separation)>1.,'No independent fixed-Q start for task-switch diagnostic'
  current=b;assert save('switch_A_to_B',lambda:service.plan(b))['success']
  current=q;assert save('switch_B_to_A',lambda:service.plan(q))['success']
  original_A=service.current_trajectory().copy()
  block=square(b.start[:2],.4)
  assert save('block_B_start',lambda:service.update(1,upsert={'switch_B':block}))['success']
  current=b;failed=save('switch_to_blocked_B',lambda:service.plan(b))
  assert not failed['success'] and not failed['trajectory_available']
  current=q;assert save('return_A_after_failed_B',lambda:service.plan(q))['success']
  assert save('clear_B_start',lambda:service.update(2,clear=['switch_B'],observed_free=True))['success']
  base=service.current_trajectory()
  arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1))];j=int(np.searchsorted(arc,.80*arc[-1]));k=int(np.searchsorted(arc,.20*arc[-1]))
  current=replace(q,start=tuple(base[j]));assert save('progress80',lambda:service.plan(current))['success']
  tail=service.current_trajectory().copy();past=square(base[k,:2],.08)
  old=save('past_add',lambda:service.update(3,upsert={'past':past}));assert old['success'] and not old['replanned']
  assert np.array_equal(tail,service.current_trajectory())
  duplicate=save('duplicate',lambda:service.update(3,upsert={'past':past}));assert duplicate['update']['reason']=='DUPLICATE'
  stale=save('stale',lambda:service.update(0,upsert={}));assert not stale['update']['accepted']
  try:service.update(4,clear=['past'])
  except ValueError as e:
   assert str(e)=='CLEAR_REQUIRES_FREE_OBSERVATION'
   events.append({'label':'unobserved_clear_rejected','passed':True,'failure':str(e)})
   json_write(out/'events.json',events)
  else:raise AssertionError('clear without visibility accepted')
  assert save('past_clear',lambda:service.update(4,clear=['past'],observed_free=True))['success']
  obstruction=square(tail[len(tail)//2,:2],.08)
  near=save('future_add',lambda:service.update(5,upsert={'future':obstruction}))
  assert near['replanned'] and (near['success'] or not near['trajectory_available'])
  assert save('future_clear',lambda:service.update(6,clear=['future'],observed_free=True))['success']
  blocked=save('current_block',lambda:service.update(7,upsert={'current':square(current.start[:2],.4)}))
  assert not blocked['success'] and not blocked['trajectory_available']
  assert save('current_clear',lambda:service.update(8,clear=['current'],observed_free=True))['success']
  json_write(out/'summary.json',{'version':json.loads((ROOT/'BASELINE.json').read_text())['version'],'r4_mode':os.environ.get('THREEYD_V10_R4_MODE'),'r5_recovery_mode':os.environ.get('THREEYD_V10_R5_MODE'),'r6_localfix_mode':os.environ.get('THREEYD_V10_R6_MODE'),'task_A':q.query_id,'task_B':b.query_id,'optimization_mode':os.environ.get('THREEYD_V8_R11_MODE'),'v9_mode':os.environ.get('THREEYD_V9_MODE'),'review_mode':os.environ.get('THREEYD_V9_R1_MODE'),'r2_mode':os.environ.get('THREEYD_V9_R2_MODE'),'r3_mode':os.environ.get('THREEYD_V9_R3_MODE'),'r5_mode':os.environ.get('THREEYD_V9_R5_MODE'),'r6_mode':os.environ.get('THREEYD_V9_R6_MODE'),'r7_mode':os.environ.get('THREEYD_V9_R7_MODE'),'r8_mode':os.environ.get('THREEYD_V9_R8_MODE','control'),'r9_mode':os.environ.get('THREEYD_V9_R9_MODE','control'),'CPU':sorted(os.sched_getaffinity(0)),'assertions_passed':True,'events':len(events),'map':a.map_id,
   'past_obstacle_replanned':old['replanned'],'future_obstacle_success':near['success'],
   'parking_context_retained':a.map_id=='mengjia','r3_request_mode':os.environ.get('THREEYD_V10_R3_MODE'),'static_event_replay':True,'live_execution':False})
 finally:service.close();s.close()
if __name__=='__main__':main()
