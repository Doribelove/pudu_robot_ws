"""Owned-process semantic/nonsemantic obstacle replay; all exports outside clocks."""
if __name__=='__main__':
    from owned_process import supervised_entry
    supervised_entry()
import argparse,hashlib,json,os,time,traceback
from pathlib import Path
import numpy as np
import psutil
from benchmark import Memory
from crossmap_inputs import load_world
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_3d_v1.multires_grid import GridView
from arena_3d_v1.semantic_world import json_write
from arena_evaluation.planner_benchmark.models import Query
from three_yd_v5.v9_gate_init import DirectStorageAuditor
from three_yd_v5.curve_session import CurveSession
from three_yd_v13.service import IncrementalService
from run_wanda3w import json_native

def cpu_tree():
    total=0.;root=psutil.Process()
    for p in [root,*root.children(recursive=True)]:
        try:c=p.cpu_times();total+=c.user+c.system
        except psutil.Error:pass
    return total

def main():
    import search_telemetry
    search_telemetry.install()
    ap=argparse.ArgumentParser();ap.add_argument('--map-id',required=True);ap.add_argument('--query',required=True)
    ap.add_argument('--arm',choices=['H-A','H-C','H-I'],required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--only-large',action='store_true');ap.add_argument('--only-small',action='store_true');ap.add_argument('--offset',type=float,default=4.);ap.add_argument('--fixed-start',action='store_true');ap.add_argument('--register-only',action='store_true');ap.add_argument('--events',type=Path);ap.add_argument('--domain',type=int,default=219)
    args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.domain);os.sched_setaffinity(0,{14})
    semantic,mp,w,g,qs,qfile=load_world(args.map_id,out)
    def pose(v):return tuple(v[k] for k in ('x','y','yaw_rad')) if isinstance(v,dict) else tuple(v)
    qd=next(q for q in qs if q['id']==args.query);q=Query(qd['id'],pose(qd['start']),pose(qd['goal']),seed=20260916 if semantic else 20260929)
    views={3:GridView(w,3),1:GridView(w,1)}
    ctx=runtime.MapContext(args.map_id,w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
    auditor=DirectStorageAuditor(ctx,source_commit='3YDL3-V13-P4-corridor-recovery')
    spec=runtime.backend_availability()['hybrid_astar']
    s=CurveSession(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',
        smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,
        planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True)
    s.local_map_update_strategy='roi_ack';s.full_grid_settle_cycles=0
    service=IncrementalService(w,g,out/'l2_cache',views,s,auditor,spec,arm=args.arm,max_region_routes=3 if semantic else 1)
    rows=[]
    from three_yd_v5.curve_backend import settings
    json_write(out/'protocol.json',{'version':'3YDL3-V13-P5-recovery-schedule','p5_schedule':os.environ.get('V13_P5_SCHEDULE','1')=='1','p5_cache':os.environ.get('V13_P5_CACHE','1')=='1','native_event_update':os.environ.get('V13_NATIVE_EVENT_UPDATE','1')=='1','indexed_heap':os.environ.get('V13_INDEXED_HEAP','1')=='1','native_extraction':os.environ.get('V13_NATIVE_PATH','1')=='1','native_segments':os.environ.get('V13_NATIVE_SEGMENTS','1')=='1','smart_rhs':os.environ.get('V13_SMART_RHS','1')=='1','skip_unchanged':os.environ.get('V13_SKIP_UNCHANGED','1')=='1','map_id':args.map_id,'query_id':q.query_id,
        'arm':args.arm,'semantic':semantic,'start':q.start,'goal':q.goal,'map_sha256':w.map.sha256,
        'query_sha256':hashlib.sha256(qfile.read_bytes()).hexdigest(),'CPU':[14],'vehicle':w.vehicle,
        'map_area_m2':w.map.width*w.map.height*w.map.resolution**2,'sizes_m':[.2,.8,2.],
        'L3_config_sha256':settings()[2],'L1_L3_algorithms_modified':False,
        'event_sequence':'initial; small/add-clear; medium/add-clear; large/add-clear; duplicate; stale; blocked-start/clear',
        'timing':'each actual initial/update request; excludes map loading/startup/post-request export; includes fallback/ACK/certification',
        'memory':'sampled sum of worker and descendant RSS; process CPU delta includes live descendants',
        'capture_trace':os.environ.get('V13_CAPTURE_TRACE')=='1','formal':os.environ.get('V13_FORMAL')=='1','moving_start':not args.fixed_start,'unknown_then_stationary':True})
    def measure(label,fn):
        s.reset_query_state(label,restore_base_map=False)
        telemetry_token,telemetry_rows=search_telemetry.start()
        ct=cpu_tree();tick=time.monotonic();res={}
        with Memory() as memory:
            try:res=fn()
            except BaseException as e:
                if isinstance(e,(KeyboardInterrupt,SystemExit)):raise
                res={'success':False,'failure_code':type(e).__name__+':'+str(e)}
                (out/(label+'_exception.txt')).write_text(traceback.format_exc())
                service.record=None
                if service.controller is not None:service.controller.trajectory=None;service.controller.trajectory_version=None
        elapsed=time.monotonic()-tick;cpu=cpu_tree()-ct
        search_counts=search_telemetry.finish(telemetry_token,telemetry_rows)
        path=service.current_trajectory();row={k:v for k,v in res.items() if k!='result'}
        row.update(all_L2_searches=search_counts,label=label,wall_ms=elapsed*1000,cpu_tree_s=cpu,rss_peak_bytes=memory.peak,
            average_cpu_cores=cpu/elapsed,trajectory_available=path is not None,map_version=service.dynamic.version)
        if path is not None:
            np.save(out/(label+'_path.npy'),path)
            c=service.controller
            np.save(out/(label+'_L2_ids.npy'),np.asarray(c.l2.state.current_path_ids,dtype=np.int64))
            audit=w.audit(service.query,path,c._target_mask())
            row['export_recheck']=audit
            assert audit['valid'],(label,audit)
            assert res.get('success') and c.trajectory_version==service.dynamic.version
            np.save(out/(label+'_L2.npy'),c.reference_plan.world_xy)
            row['l2_state_bytes']=c.l2.state_memory_bytes()
            row['l2_state_count']=c.l2.geometry.state_count
            row['L1_regions']=list(c.topology_route.region_ids)
            row['initial_l2_diagnostics']=c.initial_l2_result.diagnostics
            st=c.l2.state
            row['postclock_l2']={'geometry_hash':c.l2.geometry.binding.digest,'astar_queue_peak':getattr(st,'v13_queue_peak',None),'astar_queue_pops':getattr(st,'v13_queue_pops',None),'last_solver_unique':getattr(st,'v13_unique_expanded',None),'active_queue_entries':len(getattr(st,'queued',{})),'raw_queue_entries':len(getattr(st,'open',[])),'scope':'current controller at export; may differ from failed first attempt after domain recovery'}
            if os.environ.get('V13_VERIFY_EXTRACTION')=='1':
                from three_yd_v13.state import NativeIncrementalState
                from three_yd_v13.path_extract import supported,extract
                state=c.l2.state
                if isinstance(state,NativeIncrementalState) and supported(state):
                    before=(state.g.tobytes(),state.rhs.tobytes(),list(state.open),dict(state.queued))
                    expected=super(NativeIncrementalState,state).extract_path_ids()
                    actual=extract(state)
                    assert expected==actual,(label,'NATIVE_EXTRACTION_CHANGED_IDS')
                    assert before==(state.g.tobytes(),state.rhs.tobytes(),list(state.open),dict(state.queued))
                    row['extraction_equivalence']={'exact_ids':True,'readonly_search_state':True,'nodes':0 if actual is None else len(actual)}
            if os.environ.get('V13_CAPTURE_TRACE')=='1':
                state=c.l2.state;rc=np.column_stack((state._rows,state._cols)).astype(int)
                # Diagnostic replay only; trace export and map render are outside request clocks.
                graph_xy=c.view.world_points(rc[:,0],rc[:,1])
                trace=getattr(state,'v13_expansion_trace',np.empty(0,int))
                np.savez_compressed(out/(label+'_trace.npz'),xy=graph_xy,expanded=trace,
                    changed=np.asarray(getattr(state,'v13_changed_ids',[]),int),affected=np.asarray(getattr(state,'v13_affected_ids',[]),int),
                    blocked=np.flatnonzero(state.blocked),path_ids=np.asarray(state.current_path_ids,int))
                json_write(out/(label+'_L1.json'),{'xy':np.asarray(c.task.get('right_reference_polyline',[])).tolist()})
        elif res.get('success'):raise AssertionError('SUCCESS_WITHOUT_CURRENT_TRAJECTORY')
        json_write(out/(label+'.json'),json_native(row));rows.append(row)
        print(label,bool(row.get('success')),round(elapsed,3),row.get('failure_code',''),flush=True)
        return row
    try:
        s.start()
        from native_snapshot import record
        record(out)
        if os.environ.get('V13_CAPTURE_TRACE')=='1':
            from PIL import Image
            hard=np.asarray(w.hard,bool)
            image=np.where(hard[:,:,None],np.array([48,61,77],np.uint8),np.array([235,239,244],np.uint8))
            im=Image.fromarray(image);im.thumbnail((1800,1800));im.save(out/'map.png')
            # Derive projection from actual world.cells convention; no guessed Y direction.
            probes=w.map.world_to_cell if False else None
            xy=np.asarray([[0.,0.],[1.,0.],[0.,1.]])
            rr,cc=w.cells(xy)
            json_write(out/'map_display.json',{'width':w.map.width,'height':w.map.height,'resolution':w.map.resolution,
                'origin':w.map.origin,'probe_xy':xy.tolist(),'probe_rc':np.column_stack((rr,cc)).tolist()})
        initial=measure('initial',lambda:service.plan(q))
        if not initial.get('success'):
            json_write(out/'summary.json',{'pilot_initial_failed':True,'strict_recheck_passed':True,'all_failures_withdraw_trajectory':not initial['trajectory_available'],'rows':[initial]});return
        base=service.current_trajectory()
        if args.events:events=json.loads(args.events.read_text())
        else:
            # Mid-arc point, fixed before any obstacle result is observed.
            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1))]
            center=base[np.searchsorted(arc,arc[-1]/2),:2]
            rr,cc=w.cells(center.reshape(1,2))
            events={'placement':'original accepted path mid-arc; no selection by event success',
                'center_xy':center.tolist(),'static_center_clearance_m':float(w.map.distance_m[rr[0],cc[0]]),
                'bottleneck_width_estimate_m':float(2*w.map.distance_m[rr[0],cc[0]]),
                'width_estimate_is_not_exact_corridor_width':True,'sizes_m':[.2,.8,2.],
                'map_sha256':w.map.sha256,'query_id':q.query_id,'start':q.start,'goal':q.goal}
        assert events['map_sha256']==w.map.sha256 and events['query_id']==q.query_id
        json_write(out/'events_frozen.json',events)
        if args.register_only:
            from route_geometry import inspect_route
            json_write(out/'route_geometry.json',inspect_route(w,base,events))
            json_write(out/'summary.json',{'registration_only':True,'strict_recheck_passed':True,'all_failures_withdraw_trajectory':True,'rows':[initial]})
            return
        def square(center,width):
            x,y=center;h=width/2
            return [[x-h,y-h],[x+h,y-h],[x+h,y+h],[x-h,y+h]]
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(base[:,:2],axis=0),axis=1))]
        at=int(np.argmin(np.linalg.norm(base[:,:2]-events['center_xy'],axis=1)))
        pose_index=max(0,int(np.searchsorted(arc,max(0.,arc[at]-args.offset))))
        current_pose=tuple(q.start) if args.fixed_start else tuple(events.get('moving_start_pose',base[pose_index]))
        if not args.fixed_start:
            assert np.min(np.linalg.norm(base[:,:2]-np.asarray(current_pose)[:2],axis=1))<1e-7,'MOVING_POSE_NOT_ON_INITIAL_PATH'
        json_write(out/'moving_start.json',{'source':'exact pose on original accepted path',
            'index':pose_index,'pose':current_pose,'original_start':q.start,'original_goal':q.goal,
            'arc_from_original_start_m':float(arc[pose_index]),'arc_before_obstacle_m':float(arc[at]-arc[pose_index]),
            'physical_closed_loop':False,'kinematic_replan_only':True})
        seq=0
        for label,size in zip(['small','medium','large'],events['sizes_m']):
            if args.only_large and label!='large':continue
            if args.only_small and label!='small':continue
            seq+=1;poly=square(events['center_xy'],size)
            measure(label+'_add',lambda:service.update(seq,upsert={'test':poly},current_pose=(current_pose if seq==1 and not args.fixed_start else None)))
            seq+=1;measure(label+'_clear',lambda:service.update(seq,clear=['test'],observed_free=True))
        measure('duplicate',lambda:service.update(seq,clear=['test'],observed_free=True))
        measure('stale',lambda:service.update(seq-1))
        seq+=1;measure('start_blocked',lambda:service.update(seq,upsert={'start':square(current_pose[:2],.8)}))
        seq+=1;measure('start_clear',lambda:service.update(seq,clear=['start'],observed_free=True))
        json_write(out/'summary.json',{'rows':[{'label':r['label'],'success':r.get('success'),'wall_ms':r['wall_ms'],
            'rss_peak_bytes':r['rss_peak_bytes'],'failure_code':r.get('failure_code',''),'v13':r.get('v13')} for r in rows],
            'strict_recheck_passed':True,'all_failures_withdraw_trajectory':all(r.get('success') or not r['trajectory_available'] for r in rows)})
    finally:service.close();s.close()
if __name__=='__main__':main()
