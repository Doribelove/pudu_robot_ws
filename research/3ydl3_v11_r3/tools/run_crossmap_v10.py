"""Persistent full-route task replay. Explicit CPU affinity, actual clocks, strict audits."""
if __name__=='__main__':
    import sys
    from owned_process import supervised_entry
    if not any(flag in sys.argv for flag in ('-h','--help')):
        supervised_entry()
import argparse,hashlib,json,os,signal,time,traceback,gc,shutil,sys
if os.environ.get("THREEYD_BENCH_SOURCE"):
    sys.path.insert(0, os.environ["THREEYD_BENCH_SOURCE"]+"/tools")
from pathlib import Path
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
from run_wanda3w import json_native,process_memory,path_metrics,deadline_handler,QueryBudgetExceeded
from three_yd_v5.curve_budget import L3RequestBudgetExceeded
ROOT=Path(os.environ.get('THREEYD_BENCH_SOURCE', str(Path(__file__).resolve().parents[1])))
VERSION=json.loads((ROOT/'BASELINE.json').read_text())['version']

def select_queries(queries,requested):
    if not requested:return queries
    available={q['id'] for q in queries};missing=set(requested)-available
    if missing:raise ValueError('UNKNOWN_QUERY_IDS:'+','.join(sorted(missing)))
    if len(set(requested))!=len(requested):raise ValueError('DUPLICATE_QUERY_ID_SELECTION')
    return [q for q in queries if q['id'] in requested]

def main():
    a=argparse.ArgumentParser();a.add_argument('--l3-backend',choices=['control','independent','compatibility'],default=os.environ.get('THREEYD_L3_BACKEND','control'));a.add_argument('--map-id',required=True);a.add_argument('--output',type=Path,required=True)
    a.add_argument('--workload',choices=['fixed','moving'],default='moving');a.add_argument('--arm',choices=['control','candidate'],default='candidate')
    a.add_argument('--query-file',type=Path);a.add_argument('--domain',type=int,default=220);a.add_argument('--queries',nargs='+');a.add_argument('--repeats',type=int,default=1)
    a.add_argument('--query-order',nargs='+',help='diagnostic permutation of selected original fixed queries')
    a.add_argument('--storage-ledger',action='store_true',help='diagnostic array inventory outside request timing')
    a.add_argument('--skip-repeat-after-failure',action='store_true')
    a.add_argument('--r9-mode',choices=['control','P','L','PL','M','PM','LM','PLM','U','PLU','PLMU','X','PLMX','PLMUX'],default=os.environ.get('THREEYD_V9_R9_MODE','control'))
    a.add_argument('--r8-mode',choices=['control','O','F','OF','G','OG','FG','OFG','R','GR','OGR'],default=os.environ.get('THREEYD_V9_R8_MODE','control'))
    a.add_argument('--r7-mode',choices=['control','N','T','NT'],default=os.environ.get('THREEYD_V9_R7_MODE','control'))
    a.add_argument('--r6-mode',choices=['control','H'],default=os.environ.get('THREEYD_V9_R6_MODE','control'))
    a.add_argument('--r5-mode',choices=['control','C'],default=os.environ.get('THREEYD_V9_R5_MODE','control'))
    a.add_argument('--r3-mode',choices=['control','I'],default=os.environ.get('THREEYD_V9_R3_MODE','control'))
    a.add_argument('--r2-mode',choices=['control','D','P','M','DP','DM','PM','DPM'],default=os.environ.get('THREEYD_V9_R2_MODE','control'))
    a.add_argument('--review-mode',choices=['control','S','J','K','SJ','SK','JK','SJK'],default=os.environ.get('THREEYD_V9_R1_MODE','control'))
    a.add_argument('--v9-mode',choices=['control','A','B','C','AB','AC','BC','ABC'],default=os.environ.get('THREEYD_V9_MODE','control'));a.add_argument('--profile',action='store_true');a.add_argument('--optimization-mode',choices=['control','A','B','AB'],default=None)
    a.add_argument('--pose-offset',type=float,default=0.);a.add_argument('--yaw-offset-deg',type=float,default=0.)
    args=a.parse_args();os.environ['THREEYD_L3_BACKEND']=args.l3_backend;os.environ['THREEYD_V9_R9_MODE']=args.r9_mode;os.environ['THREEYD_V9_R8_MODE']=args.r8_mode;os.environ['THREEYD_V9_R7_MODE']=args.r7_mode;os.environ['THREEYD_V9_R6_MODE']=args.r6_mode;os.environ['THREEYD_V9_R5_MODE']=args.r5_mode;os.environ['THREEYD_V9_R3_MODE']=args.r3_mode;os.environ['THREEYD_V9_MODE']=args.v9_mode;os.environ['THREEYD_V9_R1_MODE']=args.review_mode;os.environ['THREEYD_V9_R2_MODE']=args.r2_mode;os.environ['THREEYD_V8_R11_MODE']=args.optimization_mode or 'AB';out=args.output;out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.domain);signal.signal(signal.SIGALRM,deadline_handler)
    from three_yd_v5.curve_backend import settings
    if args.l3_backend!='control':
        from three_yd_v5 import curve_l3
        if os.environ.get("THREEYD_R4_CAPTURE"):
            from capture_curve_inputs import install
            install(os.environ["THREEYD_R4_CAPTURE"])
    if os.environ.get('THREEYD_R4_DIAG_FINE_POSE')=='1':
        from fine_rectangle_probe import install
        install()
    if os.environ.get('THREEYD_R6_CAPTURE_L2'):
        from capture_l2_failure import install
        install(os.environ['THREEYD_R6_CAPTURE_L2'])
    from crossmap_inputs import load_world
    mid=args.map_id
    semantic,mp,w,g,qs,qfile=load_world(mid,out)
    views={3:GridView(w,3),1:GridView(w,1)}
    if args.query_file:
        qfile=args.query_file.resolve();qs=json.loads(qfile.read_text())['queries']
    if args.workload=='moving':
        from moving_starts import load
        qfile=ROOT/'datasets/moving_start_v1'/(mid+'.json');qs=load(qfile,mid)
    qs=select_queries(qs,args.queries)
    if args.query_order:
        if len(args.query_order)!=len(qs) or set(args.query_order)!={q['id'] for q in qs}:raise ValueError('QUERY_ORDER_MUST_PERMUTE_REGISTERED_SELECTION')
        by_id={q['id']:q for q in qs};qs=[by_id[qid] for qid in args.query_order]
    if args.pose_offset or args.yaw_offset_deg:
        import copy
        qs=copy.deepcopy(qs)
        for item in qs:
            if item.get('moving_start',{}).get('kind')!='moving':continue
            v=item['start'];v=np.asarray([v[k] for k in ('x','y','yaw_rad')] if isinstance(v,dict) else v,float)
            sign=-1 if item['moving_start']['step']=='p50' else 1
            v[:2]+=sign*args.pose_offset*np.array([-np.sin(v[2]),np.cos(v[2])])
            v[2]=(v[2]+sign*np.deg2rad(args.yaw_offset_deg)+np.pi)%(2*np.pi)-np.pi
            item['start']=dict(zip(('x','y','yaw_rad'),v.tolist())) if semantic else v.tolist()
            item['moving_start']['pose_perturbation']={'lateral_m':sign*args.pose_offset,'yaw_deg':sign*args.yaw_offset_deg}
        json_write(out/'actual_queries.json',qs)
    ctx=runtime.MapContext(mid,w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
    auditor=StorageAuditor(ctx,source_commit='V9-r9-audit-L3')
    spec=runtime.backend_availability()['hybrid_astar']
    s=PerformanceSession(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',
        smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,
        planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True)
    s.local_map_update_strategy='roi_ack';s.full_grid_settle_cycles=0
    service=TaskProgressService(w,g,out/'l2_cache',views,s,auditor,spec,max_region_routes=3 if semantic else 1,continuation=True)
    json_write(out/'heap_preparation.json',{'excluded_from_query_timing':True,
        **getattr(service,'r5_heap_preparation',{'enabled':False})})
    json_write(out/'v9_preparation.json',{'r7_mode':args.r7_mode,'neutral_labels':getattr(w,'v9_r7_neutral_diagnostics',{'enabled':False,'reason':'semantic_world_or_control'}),'excluded_from_query_timing':True})
    json_write(out/'audit_storage.json',auditor.storage_diagnostics)
    memory_policy={'request_finish_heap_release':False}
    if (ROOT/'src/three_yd_v5/request_heap.py').exists():
        from three_yd_v5.request_heap import FREE_THRESHOLD,TOP_RESERVE
        memory_policy={'request_finish_heap_release':True,'free_threshold_bytes':FREE_THRESHOLD,
            'top_reserve_bytes':TOP_RESERVE,'inside_navigation_and_request_deadline':True}
    memory_policy['allocator_environment']={k:v for k,v in os.environ.items() if k.startswith('MALLOC_')}
    json_write(out/'protocol.json',{'runtime_memory_policy':memory_policy,'version':VERSION,'l3_backend':args.l3_backend,'curve_config':settings()[1],'curve_config_sha256':settings()[2],'map_id':mid,'arm':args.arm,'workload':args.workload,
        'query_file':str(qfile),'queries_sha256':hashlib.sha256(qfile.read_bytes()).hexdigest(),
        'map_sha256':w.map.sha256,'vehicle':w.vehicle,'CPU':sorted(os.sched_getaffinity(0)),
        'repeats':args.repeats,'profiler':args.profile,'v9_mode':args.v9_mode,'review_mode':args.review_mode,'r2_mode':args.r2_mode,'r3_mode':args.r3_mode,'r5_mode':args.r5_mode,'r6_mode':args.r6_mode,'r7_mode':args.r7_mode,'r8_mode':args.r8_mode,'r9_mode':args.r9_mode,'optimization_mode':os.environ['THREEYD_V8_R11_MODE'],'L1_frozen':True,'moving_window':False,'global_L3_fallback':False,
        'timing':'actual request wall; excludes one-time preparation/startup and export; reused layers report zero current work',
        'memory':'query worker and ROS descendants sampled peak RSS; PSS snapshot separately',
        'task_context':'same query ID + exact fixed goal; committed prefix only for original traffic audit',
        'pose_matching':'r10 exact/identical or strictly audited short connector; ambiguous visits rejected; no snapping',
        'pose_offset_m':args.pose_offset,'yaw_offset_deg':args.yaw_offset_deg,'actual_queries_exported':bool(args.pose_offset or args.yaw_offset_deg),
        'strict_audits':'unchanged continuous body, curvature, steering, road/parking/turn and exact content ACK',
        'real_execution':False,'diagnostic_subset':bool(args.queries),'actual_query_order':[q['id'] for q in qs],'query_order_diagnostic':bool(args.query_order),'r3_request_mode':os.environ.get('THREEYD_V10_R3_MODE','control'),'r4_mode':os.environ.get('THREEYD_V10_R4_MODE','control'),'r5_recovery_mode':os.environ.get('THREEYD_V10_R5_MODE','control'),'r7_tailcost_mode':os.environ.get('THREEYD_V10_R7_MODE','control'),'r6_localfix_mode':os.environ.get('THREEYD_V10_R6_MODE','control'),'diagnostic_r6_field_comparison':bool(os.environ.get('THREEYD_R6_COMPARE_FIELDS')),'diagnostic_curve_capture':bool(os.environ.get('THREEYD_R4_CAPTURE')),'diagnostic_fine_pose':os.environ.get('THREEYD_R4_DIAG_FINE_POSE')=='1','firstplan_mode':os.environ.get('THREEYD_V8_FIRST_MODE'),
        'common_harness_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'export_controller_references_released_before_next_request':True})
    snap=out/'source_snapshot';snap.mkdir()
    sources=[*(ROOT/'src').rglob('*.py'),*(ROOT/'tools').glob('*.py'),ROOT/'env.bash']
    for f in sources:shutil.copy2(f,snap/str(f.relative_to(ROOT)).replace('/','_'))
    json_write(snap/'hashes.json',{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources})
    rows=[]
    if os.environ.get('THREEYD_R6_COMPARE_FIELDS'):
        from compare_r6_fields import install
        install(out/'field_equivalence.jsonl')
        from compare_r7_occupancy import install as install_occupancy_oracle
        install_occupancy_oracle(out/'occupancy_equivalence.jsonl')
    if args.storage_ledger:
        from storage_ledger import inventory
        json_write(out/'storage_prepared.json',inventory({'world':w,'topology':g,'views':views,'service':service,'session':s}))
    try:
        s.start()
        json_write(out/'ready.json',{'ready':True,'navigation_only':True})
        from native_snapshot import record
        record(out)
        from three_yd_v5.v9_r9_lookup import lookup_stats
        from three_yd_v5.v9_r9_membership import membership_stats
        from three_yd_v5.v9_r9_buffers import buffer_stats
        from three_yd_v5.v9_r9_scope import scope_stats
        for qd in qs:
            def pose(v):return tuple(v[k] for k in ('x','y','yaw_rad')) if isinstance(v,dict) else tuple(v)
            q=Query(qd['id'],pose(qd['start']),pose(qd['goal']),seed=20260916 if semantic else 20260929)
            for rep in range(1,args.repeats+1):
                step=qd.get('moving_start',{}).get('step');tag=f'{q.query_id}_{step}_{rep}' if step else f'{q.query_id}_{rep}'
                case=out/'cases'/tag;case.mkdir(parents=True);res={};path=None
                row={'version':VERSION,'map_id':mid,'query_id':q.query_id,'repeat':rep,'moving_start':qd.get('moving_start'),
                    'start':q.start,'goal':q.goal,'success':False,'geometry_success':False}
                lookup_before=lookup_stats()
                membership_before=membership_stats()
                buffers_before=buffer_stats()
                scope_before=scope_stats()
                tick=time.monotonic();cpu=time.process_time()
                with Memory() as mem:
                    try:
                        signal.setitimer(signal.ITIMER_REAL,settings()[1]['total_request_budget_s']);s.reset_query_state(tag,restore_base_map=False)
                        if args.profile:
                            import cProfile
                            profiler=cProfile.Profile();profiler.enable()
                        try:res=service.plan(q)
                        finally:
                            if args.profile:
                                profiler.disable();profiler.dump_stats(str(case/'query.prof'))
                        path=res.pop('result',None)
                        row.update(success=bool(res.get('success')),geometry_success=bool(res.get('geometry_success')),failure_code=res.get('failure_code',''))
                        row.update(endpoint_tolerance=res.get('endpoint_tolerance',{}),
                                   exact_endpoint_success=res.get('exact_endpoint_success',bool(res.get('success'))),
                                   tolerant_endpoint_success=res.get('tolerant_endpoint_success',False))
                    except (Exception,QueryBudgetExceeded,L3RequestBudgetExceeded) as e:
                        row['failure_code']=type(e).__name__+':'+str(e);(case/'exception.txt').write_text(traceback.format_exc())
                        res={'success':False,'failure_code':row['failure_code'],'l3_backend':{'mode':args.l3_backend,'source':'no_result','partial_request_state':getattr(s,'_curve_request',{})}}
                    finally:signal.alarm(0)
                row.update(wall_ms=(time.monotonic()-tick)*1000,cpu_ms=(time.process_time()-cpu)*1000,tree_cpu_ms=mem.tree_cpu_ms,rss_peak_bytes=mem.peak)
                lookup_after=lookup_stats()
                row['r9_static_lookup']={**lookup_after,'native_LUT_calls_this_request':lookup_after['native_LUT_calls']-lookup_before['native_LUT_calls'],
                    'original_fallback_calls_this_request':lookup_after['original_fallback_calls']-lookup_before['original_fallback_calls'],
                    'inspection_outside_request_timing':True}
                membership_after=membership_stats()
                row['r9_semantic_membership']={**membership_after,
                    'native_membership_calls_this_request':membership_after['native_membership_calls']-membership_before['native_membership_calls'],
                    'original_fallback_calls_this_request':membership_after['original_fallback_calls']-membership_before['original_fallback_calls'],
                    'disabled_calls_this_request':membership_after['disabled_calls']-membership_before['disabled_calls'],
                    'lookup_cells_this_request':membership_after['lookup_cells']-membership_before['lookup_cells'],
                    'inspection_outside_request_timing':True}
                buffers_after=buffer_stats()
                row['r9_buffer_storage']={**buffers_after,
                    'scope_shared_calls_this_request':buffers_after['scope_shared_calls']-buffers_before['scope_shared_calls'],
                    'scope_duplicate_buffer_bytes_avoided_this_request':buffers_after['scope_duplicate_buffer_bytes_avoided']-buffers_before['scope_duplicate_buffer_bytes_avoided'],
                    'inspection_outside_request_timing':True}
                scope_after=scope_stats()
                row['r9_scope_aggregation']={**scope_after,
                    'exact_scope_calls_this_request':scope_after['exact_scope_calls']-scope_before['exact_scope_calls'],
                    'original_fallback_calls_this_request':scope_after['original_fallback_calls']-scope_before['original_fallback_calls'],
                    'disabled_calls_this_request':scope_after['disabled_calls']-scope_before['disabled_calls'],
                    'fine_cells_processed_this_request':scope_after['fine_cells_processed']-scope_before['fine_cells_processed'],
                    'logical_padded_bool_buffer_bytes_avoided_this_request':scope_after['logical_padded_bool_buffer_bytes_avoided']-scope_before['logical_padded_bool_buffer_bytes_avoided'],
                    'inspection_outside_request_timing':True}
                json_write(case/'outcome_before_export.json',res)
                c,r=service.controller,service.route
                from three_yd_v5.v9_r9_records import storage_evidence
                row['r9_record_storage']=storage_evidence(service.record)
                if c and hasattr(c,'initial_l2_result'):
                    reused=res.get('task_progress',{}).get('reused',False)
                    hit=reused or bool(c.timing.get('r6_route_cache_hit'))
                    row['l2']={**c.initial_l2_result.diagnostics,'response_ms':0. if hit else c.initial_l2_result.response_ms,
                        'source_response_ms':c.initial_l2_result.response_ms,'reused_search_result':hit}
                    row['timing']=dict(c.timing)
                if c and hasattr(c,'reference_plan'):
                    np.save(case/'l2_reference_xy.npy',c.reference_plan.world_xy)
                    json_write(case/'l2_soft_target.json',{'offset_m':c.l2_guide_offset_info['straight_offset_m'],'points':c.l2_soft_target_xy.tolist()})
                    if c._search_mask is not None:np.savez_compressed(case/'l3_scope.npz',mask=c._target_mask())
                if r:json_write(case/'l1_output.json',g.channel_plan(r))
                if path and path.points:
                    points=np.asarray([[p['x'],p['y'],p['yaw']] for p in path.points])
                    np.save(case/('path.npy' if row['success'] else 'rejected_candidate.npy'),points)
                    row['length_m']=float(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1).sum())
                    if semantic and c:row.update(path_metrics(c,points))
                for dst,src in [('audit','independent_audit'),('constraints','constraint_audit'),('l3','diagnostics'),('recovery','candidate_recovery'),
                    ('adapter','reference_diagnostics'),('bounded_l3','bounded_l3'),('fullroute','fullroute'),('task_progress','task_progress'),('v9_local','v9_local'),('v9_segment_cache','v9_segment_cache'),('curve_l3','curve_l3'),('l3_backend','l3_backend')]:row[dst]=res.get(src,{})
                row['memory_processes']=process_memory();json_write(case/'outcome.json',res)
                if (ROOT/'src/three_yd_v5/request_cost.py').exists():
                    from three_yd_v5.request_cost import stats as request_cost_stats
                    row['r3_request_optimization_cumulative']=request_cost_stats()
                if args.storage_ledger:
                    json_write(case/'storage.json',inventory({'world':w,'topology':g,'views':views,'service':service,'session':s}))
                with (out/'runs.jsonl').open('a') as f:f.write(json.dumps(json_native(row),allow_nan=False)+'\n')
                rows.append(row);print(tag,row['success'],row.get('failure_code'),round(row['wall_ms']),row['task_progress'].get('reused'),flush=True)
                # Export must not retain the previous controller during the
                # next request. This harness is shared by BOTH compared arms.
                path=None;res=None;c=None;r=None;gc.collect()
                if args.skip_repeat_after_failure and rep==1 and not row['success']:
                    json_write(case/'unformed_repeats.json',{'query_id':q.query_id,'count':max(0,args.repeats-1),'reason':'FIRST_REQUEST_FAILED_NO_REUSABLE_RESULT'})
                    break
        json_write(out/'summary.json',{'runs':len(rows),'success':sum(r['success'] for r in rows),
            'exact_endpoint_success':sum(r.get('exact_endpoint_success',False) for r in rows),
            'tolerant_endpoint_success':sum(r.get('tolerant_endpoint_success',False) for r in rows),
            'continuation_hits':sum(r['task_progress'].get('reused',False) for r in rows)})
    finally:
        service.close();s.close()
if __name__=='__main__':main()
