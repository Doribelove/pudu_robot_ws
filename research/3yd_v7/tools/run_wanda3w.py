"""3YD-V5-r13 Wanda3w ten-query regression against the frozen r11 input."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
import argparse,time,json,hashlib,os,gc,signal,traceback,math
from pathlib import Path
import numpy as np
import psutil
from benchmark import Memory
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.semantic_world import SemanticWorld,json_write,digest
from arena_3d_v1.multires_grid import GridView
from three_yd_v5.performance_session import PerformanceSession as SemanticAckSession
from three_yd_v5.topology_default import DefaultTopology as OutlineTopology, SEARCH_MODE
from three_yd_v5.recovery import V5Runner as V3CandidateRecovery
from three_yd_v5.controller import V5Controller as ChannelController
from three_yd_v3.dynamic import DynamicObstacles
from three_yd_v5.r10_options import mode as r10_mode
from three_yd_v5.r11_options import mode as r11_mode, enabled as r11_enabled
from three_yd_v5.r10_connector import stats as connector_stats
ROOT=Path(__file__).resolve().parents[1]
class QueryBudgetExceeded(BaseException):
 pass

def deadline_handler(signum,frame):
 # Backend RuntimeError/TimeoutError recovery must not swallow the query cap.
 raise QueryBudgetExceeded('QUERY_WALL_BUDGET')


def json_native(value,path='row'):
 if isinstance(value,np.ndarray):
  return value.tolist()
 if isinstance(value,np.generic):return value.item()
 if isinstance(value,dict):return {k:json_native(v,path+'.'+k) for k,v in value.items()}
 if isinstance(value,(list,tuple)):return [json_native(v,path+'[]') for v in value]
 return value

def path_metrics(c,p):
 ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);rr,cc=c.view.cells((p[:-1,:2]+p[1:,:2])/2)
 field=c.preference
 valid=(rr>=0)&(cc>=0)&(rr<field.eligible.shape[0])&(cc<field.eligible.shape[1]);good=np.zeros(len(rr),bool);good[valid]=field.eligible[rr[valid],cc[valid]]
 normal=np.full(len(rr),np.nan);normal[valid]=field.normalized_right[rr[valid],cc[valid]]
 good&=np.isfinite(normal);den=float(ds[good].sum());lane=np.isin(c.world.sample_labels(p[:-1]),[k for k,f in c.world.features.items() if f.semantic_class=='lane']);lane_length=float(ds[lane].sum())
 return {'length_m':float(ds.sum()),'right_evaluable_m':den,'lane_length_m':lane_length,'right_evaluable_lane_fraction':den/lane_length if lane_length else None,'right_side_fraction':float(ds[good&(normal<.5)].sum()/den) if den else None,'right_target_band_fraction':float(ds[good&(normal>=0.)&(normal<=.5)].sum()/den) if den else None,'right_metric':'R5 road cross-section: 0=right boundary, 1=left boundary; junction and parking excluded'}


def process_memory():
 result=[]
 for proc in [psutil.Process(),*psutil.Process().children(recursive=True)]:
  try:
   mem=proc.memory_full_info()
   result.append({'pid':proc.pid,'name':proc.name(),'rss':mem.rss,'pss':mem.pss,'uss':mem.uss})
  except psutil.Error:pass
 return result

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--profile',action='store_true');ap.add_argument('--approach-feedback',action='store_true');ap.add_argument('--reference-feedback',action='store_true');ap.add_argument('--l3-mode',choices=['bounded'],default='bounded');ap.add_argument('--output',required=True);ap.add_argument('--queries',default='all');ap.add_argument('--repeats',type=int,default=3);ap.add_argument('--map',default='corroborated_map.yaml');ap.add_argument('--uniform-scope',action='store_true');ap.add_argument('--weight',type=float,default=2.);ap.add_argument('--max-region-routes',type=int,default=3);ap.add_argument('--domain',type=int,default=228);ap.add_argument('--guard',type=int,default=90);ap.add_argument('--query-file');ap.add_argument('--interleave',action='store_true');ap.add_argument('--shuffle-seed',type=int,default=611);ap.add_argument('--memory-inventory',action='store_true');a=ap.parse_args()
 if not 0<=a.domain<=232:raise ValueError("ROS_DOMAIN_OUT_OF_TEST_RANGE")
 global ChannelController
 if a.approach_feedback:raise ValueError('V5_NO_APPROACH_FEEDBACK')
 if a.uniform_scope:
  import three_yd_v5.controller as module
  original=module.variable_neighborhood
  module.variable_neighborhood=lambda w,p,c:original(w,p,[],normal=.8,turning=.8)
 if a.reference_feedback:raise ValueError('USE_POSE_CONNECTION_FEEDBACK; UNBOUNDED_FEEDBACK_CONTROLLER_FORBIDDEN')
 out=Path(a.output);out.mkdir(parents=True,exist_ok=False);signal.signal(signal.SIGALRM,deadline_handler)
 os.environ['ROS_DOMAIN_ID']=str(a.domain);os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
 query_file=Path(a.query_file) if a.query_file else ROOT/'inputs/queries.json'
 qs=json.loads(query_file.read_text())['scenarios'];qs=[q for q in qs if a.queries=='all' or q['id'] in a.queries.split(',')]
 mp=ROOT/'inputs'/a.map;sp=ROOT/'inputs/semantic.json';t=time.monotonic()
 with Memory() as mem:
  w=SemanticWorld(mp,sp,cache_root=out/'world_cache');g=OutlineTopology(w,ROOT/'inputs/topology.json',out/'graph_cache');views={3:GridView(w,3),1:GridView(w,1)}
 prep={'wall_ms':(time.monotonic()-t)*1000,'rss_peak_bytes':mem.peak,'l1_build_ms':g.preparation_ms,'regions':len(g.regions),'region_adjacency_pairs':g.edges_count,'gates':g.portal_count};json_write(out/'preparation.json',prep)
 ctx=runtime.MapContext('mengjia',w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
 auditor=PathAuditor(ctx,source_commit='3YD-V6-r7');spec=runtime.backend_availability()['hybrid_astar'];dynamic=DynamicObstacles(w)
 session_type=SemanticAckSession
 if r11_enabled('C'):
  from three_yd_v5.r11_crop import StorageCropSession
  session_type=StorageCropSession
 session=session_type(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True)
 session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
 json_write(out/'protocol.json',{'version':'3YD-V6-r7','acceptance_gate':'unchanged vehicle sweep, curvature, steering and content ACK; r7 semantic road-stage boundary correction','l1_contract':'complete topology preserved; one selected incoming/outgoing boundary per junction; tangent-intersection trim; no branch geometry in a query output','l2_state':['x','y'],'l2_algorithm':'unchanged DStar Lite; V5-r4 circumscribed-circle primary; local exact rectangle bridge only after disconnection','l2_resolution_m':.15,'l2_soft_target':'entire selected L1 polyline shifted 0.30 m inward; round-like joins; junctions included','l2_soft_target_offset_m':.30,'l2_soft_target_corner_exemption':False,'l1_geometry_unchanged':True,'l2_hard_obstacle_model':'original circumscribed circle; every represented fine cell clear','l2_vehicle_feasibility_is_final':False,'additional_safety_buffer_m':0.,'rectangle_bridge_check_radius_m':ChannelController.rectangle_check_radius_m,'rectangle_pose_samples_per_cell':9*24,'rectangle_certificate_is_continuous_motion_proof':False,'coarse_aggregation':'all fine circle-inflated cells must be free','r6_optimizations_preserved':['version-bound candidate preflight and deterministic failure cache','one-entry route-conditioned L2 field and result LRU','one-entry byte-identical bounded costmap LRU with existing incremental ACK','bounded cache residency and controller transient cleanup','route-local L2 ROI field calculation'],'cache_invalidation':['map hash','semantic hash','vehicle hash','dynamic version and content','query pose','route signature'],'l3_contract':'unchanged 0.05 m Smac; endpoint parking access tubes; 0.8/1.2 m variable neighborhood; bounded local continuous-sweep repair; original independent audits','l3_resolution_m':.05,'l3_normal_radius_m':.8,'l3_corner_radius_m':.8 if a.uniform_scope else 1.2,'endpoint_parking_access_tube_m':1.2,'corner_threshold_deg':30.,'corner_plateau_arc_m':.6,'corner_transition_arc_m':1.2,'local_geometry_repair_budget_s':8.,'max_search_cells':ChannelController.max_search_cells,'max_map_fraction':ChannelController.max_map_fraction,'global_L3_fallback':False,'vehicle':w.vehicle,'map_file':str(mp),'map_sha256':w.map.sha256,'semantic_hash':w.semantic.semantic_map_hash,'queries_sha256':hashlib.sha256(query_file.read_bytes()).hexdigest(),'parking_access_sha256':hashlib.sha256((ROOT/'inputs/parking_access.json').read_bytes()).hexdigest(),'query_ids':[q['id'] for q in qs],'repeats':a.repeats,'CPU':sorted(os.sched_getaffinity(0)),'unknown_is_collision':True,'confirmed_map_interpretation':a.map=='corroborated_map.yaml','rss_scope':'worker and ROS descendants','guard_s':a.guard,'real_robot_execution':False,'dynamic_validation':False})
 protocol_path=out/'protocol.json';protocol=json.loads(protocol_path.read_text())
 protocol.update(l1_goal_potential=os.environ.get('THREEYD_L1_GOAL_POTENTIAL','1')=='1',l1_search=SEARCH_MODE,l1_first_budget_expansions=int(os.environ.get('THREEYD_L1_FIRST_BUDGET','50000')),l1_alternate_budget_expansions=int(os.environ.get('THREEYD_L1_ALTERNATE_BUDGET','10000')),l1_legacy_compat=os.environ.get('THREEYD_L1_LEGACY_COMPAT','1')=='1',l1_algorithm_source_sha256=hashlib.sha256((ROOT/'src/three_yd_v5/topology_astar.py').read_bytes()).hexdigest())
 for obsolete in ('l2_soft_target_corner_threshold_deg','l2_soft_target_corner_keep_radius_m','l2_soft_target_corner_transition_m'):
  protocol.pop(obsolete,None)
 protocol.update(r11_ablation=r11_mode(),query_file=str(query_file),interleaved=a.interleave,shuffle_seed=a.shuffle_seed,r10_ablation=r10_mode(),baseline='3YD-V5-r10-ABC',memory_profile=os.environ.get('THREEYD_R9_MEMORY_PROFILE','baseline'),native_allocator_mode=os.environ.get('THREEYD_R9_NATIVE_ALLOCATOR','inherit'),performance_mode='optimized' if ChannelController.optimize else 'measured-r8',
     timing_semantics='per-call wall clocks; cached source timings reported separately; sum all attempted candidates',
     transport_profile=os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
     allocator_environment={k:os.environ.get(k) for k in ('MALLOC_ARENA_MAX','MALLOC_TRIM_THRESHOLD_')},
     l2_soft_target='entire selected L1 right-reference polyline shifted 0.30 m toward route interior',l2_soft_target_corner_definition='junctions use the same full offset with round-like join',l2_soft_target_corner_transition_outside_polygon_m=0.)
 protocol.update(version='3YD-V6-r7',baseline='3YD-V5-r11-AB-frozen',
     experimental_flags={k:os.environ.get(k,'0') for k in
         ('THREEYD_R13_START_ACCESS_TUBE','THREEYD_R13_NORTH_EXIT_GUIDE')})
 protocol.update(v6_r6_l1={'lazy_first_phase_resume':os.environ.get('THREEYD_V6_LAZY_RESUME','1')=='1', 'dominance_count_buckets':os.environ.get('THREEYD_V6_DOMINANCE_BUCKETS','1')=='1', 'single_lane_certificate':os.environ.get('THREEYD_V6_SINGLE_LANE_CERTIFICATE','1')=='1', 'dominance_copy_on_delete':os.environ.get('THREEYD_V6_COPY_ON_DELETE','1')=='1', 'transition_table':os.environ.get('THREEYD_V6_TRANSITION_TABLE','1')=='1', 'directed_goal_potential':os.environ.get('THREEYD_V6_DIRECTED_POTENTIAL','1')=='1', 'prefix_sufficiency':os.environ.get('THREEYD_V6_PREFIX_SUFFICIENCY','1')=='1', 'precompiled_index':False, 'static_map_hash_cached':True, 'directed_all_pairs_precompute':os.environ.get('THREEYD_V6_PRECOMPUTED_POTENTIAL','1')=='1', 'skip_relaxed_potential':os.environ.get('THREEYD_V6_SKIP_RELAXED_POTENTIAL','1')=='1', 'potential_matrix_cap_mib':16})
 protocol['v6_r7_traffic']={'same_road_same_direction_stage_collapse':True,'foreign_lane_raster_run_max_m':0.15,'foreign_lane_requires_adjacent_selected_road_cell':True,'forbidden_turn_wrong_direction_and_parking_checks_preserved':True}
 json_write(protocol_path,protocol)
 snap=out/'source_snapshot';snap.mkdir();sources=[*(ROOT/'src/three_yd_v3').glob('*.py'),*(ROOT/'src/three_yd_v4').glob('*.py'),*(ROOT/'src/three_yd_v5').glob('*.py'),*(ROOT/'tools').glob('*.py'),ROOT/'env.bash'];import shutil
 for source in sources:shutil.copy2(source,snap/(source.parent.name+'_'+source.name))
 json_write(snap/'hashes.json',{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
 rows=[]
 try:
  startup_tick=time.monotonic();session.start()
  json_write(out/'session_startup.json',{'wall_ms':(time.monotonic()-startup_tick)*1000,'deferred_crop_start':r11_enabled('C')})
  if not r11_enabled('C'):__import__('native_snapshot').record(out)
  import random
  if a.interleave:
   schedule=[];rng=random.Random(a.shuffle_seed)
   for repeat_index in range(1,a.repeats+1):
    ordered=qs.copy();rng.shuffle(ordered);schedule.extend((qd,repeat_index) for qd in ordered)
  else:schedule=[(qd,None) for qd in qs]
  for qd,scheduled_repeat in schedule:
   q=Query(qd['id'],tuple(qd['start'][k] for k in ['x','y','yaw_rad']),tuple(qd['goal'][k] for k in ['x','y','yaw_rad']),seed=20260916)
   for repeat in ([scheduled_repeat] if scheduled_repeat is not None else range(1,a.repeats+1)):
    tag=f'{q.query_id}_{repeat}';case=out/'cases'/tag;case.mkdir(parents=True);c=None;r=None;outcome={};path=None
    profiler=None
    if a.profile:
     import cProfile
     profiler=cProfile.Profile();profiler.enable()
    connector_before=connector_stats()
    tile_before=len(getattr(session,'r10_tile_events',[]))
    tick=time.monotonic();cpu=time.process_time();row={'query_id':q.query_id,'repeat':repeat,'success':False,'geometry_success':False}
    with Memory() as memory:
     try:
      signal.alarm(a.guard);session.reset_query_state(tag,restore_base_map=False)
      c,r,outcome=V3CandidateRecovery(w,g,out/'l2_cache',views,dynamic,controller_class=ChannelController,reference_feedback=a.reference_feedback,approach_feedback=a.approach_feedback,weight=a.weight,max_region_routes=a.max_region_routes).plan(q,session,auditor,spec)
      path=outcome.pop('result',None);row.update(success=bool(outcome.get('success')),geometry_success=bool(outcome.get('geometry_success')),failure_code=outcome.get('failure_code',''))
     except QueryBudgetExceeded:
      row['failure_code']='QUERY_WALL_BUDGET'
      outcome={'success':False,'failure_code':'QUERY_WALL_BUDGET','budget_s':a.guard,'failure_is_physical_blockage':False}
     except Exception as e:
      row['failure_code']=type(e).__name__+':'+str(e);(case/'exception.txt').write_text(traceback.format_exc())
     finally:signal.alarm(0)
    if profiler:profiler.disable();profiler.dump_stats(str(case/'profile.pstats'))
    row.update(wall_ms=(time.monotonic()-tick)*1000,cpu_ms=(time.process_time()-cpu)*1000,tree_cpu_ms=memory.tree_cpu_ms,rss_peak_bytes=memory.peak)
    if c and hasattr(c,'initial_l2_result'):
     row['l2']={**c.initial_l2_result.diagnostics,'selected_backend':c.initial_l2_result.selected_backend,'response_ms':c.initial_l2_result.response_ms,'changed_cells':c.initial_l2_result.changed_cells};row['timing']=c.timing;np.save(case/'l2_reference.npy',c.reference_cells)
    if c and hasattr(c,'reference_plan'):
     np.save(case/'l2_reference_xy.npy',c.reference_plan.world_xy);np.save(case/'l2_raw_reference_xy.npy',c.raw_reference_xy)
     json_write(case/'l2_corners.json',c.corner_records)
     json_write(case/'l2_soft_target.json',{'offset_m':.30,'side':'toward_road_center','points':c.l2_soft_target_xy.tolist(),'records':c.l2_guide_offset_records,'corner_policy':c.l2_guide_offset_info})
     json_write(case/'l2_rectangle_bridge_certificates.json',getattr(c,'rectangle_bridge_certificates',[]))
     if c._search_mask is not None:np.savez_compressed(case/'l3_scope.npz',mask=c._target_mask())
    if r:
     row['region_sequence']=list(r.region_ids)
     json_write(case/'l1_output.json',g.channel_plan(r))
    if path and path.points:
     points=np.array([[p['x'],p['y'],p['yaw']] for p in path.points]);np.save(case/('path.npy' if row['success'] else 'rejected_candidate.npy'),points);json_write(case/('path_points.json' if row['success'] else 'rejected_candidate_points.json'),path.points)
     if c:row.update(path_metrics(c,points))
    row['legacy_contract_success']=bool(row['success'] and outcome.get('legacy_contract_success',outcome.get('traffic_audit',{}).get('nominal_route_valid',False)))
    row['approach_feedback']=outcome.get('approach_feedback',{});row['bounded_l3']=outcome.get('bounded_l3',{});row['feasibility_recovery']=outcome.get('feasibility_recovery',{});row['local_l3']=outcome.get('local_l3',{});row['audit']=outcome.get('independent_audit',{});row['constraints']=outcome.get('constraint_audit',{});row['l3']=outcome.get('diagnostics',{});row['recovery']=outcome.get('candidate_recovery',{});row['adapter']=outcome.get('reference_diagnostics',{})
    if c and 'l2' in row:
     row['l2']['source_response_ms']=row['l2']['response_ms']
     hit=bool(c.timing.get('r6_route_cache_hit'))
     row['l2']['current_search_response_ms']=0. if hit else row['l2']['response_ms']
     row['l2']['response_ms']=row['l2']['current_search_response_ms']
     row['l2']['reused_search_result']=hit
    row['curvature_repair']=outcome.get('curvature_repair',{})
    connector_after=connector_stats()
    row['r10']={'ablation':r10_mode(),'connector_cache':connector_after,'connector_delta':{k:connector_after[k]-connector_before[k] for k in ('calls','hits','failure_hits','misses','evictions')},'tile_updates':getattr(session,'r10_tile_events',[])[tile_before:]} 
    row['memory_processes']=process_memory()
    row['r11']={'mode':r11_mode()}
    if a.memory_inventory:
     from three_yd_v5.r11_memory import inventory
     record=inventory({'world':w,'map':w.map,'topology':g,'views':views,'controller':c,'session':session})
     json_write(case/'array_inventory.json',record)
     row['r11']['unique_array_bytes']=record['unique_array_bytes']
    row['performance_mode']='optimized' if ChannelController.optimize else 'measured-r8'
    row['v5']=outcome.get('v5',{});row['local_geometry_repair']=outcome.get('local_geometry_repair',{})
    if not c and r:
     failed=out/'l2_cache'/('failed_reference_'+r.signature[:12]+'.json')
     if failed.exists():row['timing']=json.loads(failed.read_text())
    json_write(case/'outcome.json',outcome);rows.append(row)
    with (out/'runs.jsonl').open('a') as f:f.write(json.dumps(json_native(row),allow_nan=False)+'\n')
    print(tag,row['success'],row['geometry_success'],row.get('failure_code'),round(row['wall_ms']),flush=True)
    if c:c.lifecycle.clear()
    c=None;path=None;outcome=None;gc.collect()
 finally:session.close()
 json_write(out/'summary.json',{'runs':len(rows),'strict_success':sum(r['success'] for r in rows),'geometry_success':sum(r['geometry_success'] for r in rows),'legacy_contract_success':sum(r['legacy_contract_success'] for r in rows),'failures':[r.get('failure_code') for r in rows if not r['success']]})
if __name__=='__main__':main()
