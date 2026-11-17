#!/usr/bin/env python3
"""Exploratory single-session P6 deltas. Own synthetic map/domain184; 60s bound.
No public dynamic API or full P6 equivalence is asserted by this test.
"""
import os,sys,json,tempfile,hashlib,traceback
from pathlib import Path

def worker(out,solver_root):
    assert os.environ['ROS_DOMAIN_ID']=='184' and os.environ['ROS_LOCALHOST_ONLY']=='1'
    solver_root=Path(solver_root).resolve()
    assert solver_root.is_dir()
    sys.path.insert(0,str(solver_root))
    import osqp,qdldl,clarabel
    solver_sources={name:str(Path(module.__file__).resolve()) for name,module in [('osqp',osqp),('qdldl',qdldl),('clarabel',clarabel)]}
    assert all(Path(path).is_relative_to(solver_root) for path in solver_sources.values()),solver_sources
    from trilayer_p6_runtime import activate,RuntimeConfig
    from trilayer_p6_runtime.api import _json
    import numpy as np
    from PIL import Image
    pixels=np.full((240,240),254,np.uint8)
    pixels[:4,:]=0;pixels[-4:,:]=0;pixels[:,:4]=0;pixels[:,-4:]=0
    Image.fromarray(pixels).save(out/'map.pgm')
    mp=out/'map.yaml'
    mp.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    activate(RuntimeConfig('/global_costmap/trilayer_get_costmap',out/'cache'))
    from arena_evaluation import unified_four_backends_smoke as rt
    from three_yd_v5.free_space_topology import load_map_only_world,FreeSpaceTopology
    from three_yd_v5.v10_storage import prepare
    from three_yd_v5.curve_session import CurveSession
    from three_yd_v5.v12_topology import persistent
    from three_yd_v5.v12_fields import prepare_fields
    from arena_3d_v1.multires_grid import GridView
    from three_yd_v5.v9_gate_init import DirectStorageAuditor
    from three_yd_v13.service import IncrementalService
    from arena_evaluation.planner_benchmark.models import Query
    session=service=None;events=[];errors=[];failure=None
    def record(label,value):
        trajectory=service.current_trajectory()
        event={'label':label,'value':_json(value),'dynamic_version':service.dynamic.version,
               'trajectory_version':getattr(service.controller,'trajectory_version',None),
               'sequence':service.dynamic.sequence,'overlay_cells':int(service.dynamic.mask.sum()),
               'trajectory':None if trajectory is None else _json(trajectory),
               'session_identity':id(session),'native_pid':session.stack.process.pid,
               'ack_events':_json(session.ack_events)}
        events.append(event);(out/'events.json').write_text(json.dumps(events,indent=2,allow_nan=False))
        print(label, value.get('success'),value.get('failure_code'),value.get('action'),event['dynamic_version'],flush=True)
    try:
        world=load_map_only_world(mp,out/'world');prepare(world)
        ctx=rt.MapContext('dynamic_map',world.map,world.safe,world.map.distance_m,world.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
        session=CurveSession(ctx,out/'session',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True)
        session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0;session.start()
        graph=persistent(world,FreeSpaceTopology,out/'cache/indices/dynamic_map',None);prepare_fields(world,graph,out/'cache/planes')
        service=IncrementalService(world,graph,out/'l2_cache',{3:GridView(world,3),1:GridView(world,1)},session,DirectStorageAuditor(ctx,source_commit='P6-dynamic-exploration'),rt.backend_availability()['hybrid_astar'],arm='H-I',max_region_routes=1)
        query=Query('dynamic', (2.,4.,0.),(5.,4.,0.),seed=20260929)
        session.reset_query_state('dynamic',restore_base_map=False)
        first=service.plan(query);record('initial_plan',first)
        assert first.get('success') and first.get('independent_audit',{}).get('valid')
        polygon=[[3.25,3.25],[3.75,3.25],[3.75,4.75],[3.25,4.75]]
        update=service.update(1,upsert={'box':polygon},current_pose=query.start);record('upsert',update)
        assert update['update']['accepted'] and service.dynamic.version==1
        assert update.get('success') and update.get('independent_audit',{}).get('valid')
        assert update['action']=='incremental_L2_then_L3'
        assert any(3.25<=p[0]<=3.75 and 3.25<=p[1]<=4.75 for p in events[0]['trajectory'])
        assert not any(3.25<=p[0]<=3.75 and 3.25<=p[1]<=4.75 for p in events[-1]['trajectory'])
        assert len(session.ack_events)>len(events[0]['ack_events'])
        ack=session.ack_events[-1]
        assert ack['costmap_ack_status']=='exact_effective_master_content_verified'
        assert ack['costmap_ack_mismatch_cells']==0 and ack['semantic_exact_stable_observations']>=2
        version=service.dynamic.version;maskhash=service.dynamic.content_hash
        record('duplicate',service.update(1,upsert={'box':polygon},current_pose=query.start))
        assert events[-1]['value']['update']['reason']=='DUPLICATE' and service.dynamic.version==version
        record('stale',service.update(0))
        assert events[-1]['value']['update']['reason']=='STALE_SEQUENCE' and service.dynamic.content_hash==maskhash
        record('sequence_gap_no_observation',service.update(4))
        assert service.dynamic.content_hash==maskhash and service.dynamic.obstacles
        cleared=service.update(5,clear=['box'],observed_free=True,current_pose=query.start);record('observed_clear',cleared)
        assert cleared.get('success') and cleared.get('independent_audit',{}).get('valid')
        assert len(session.ack_events)>len(events[-2]['ack_events'])
        assert session.ack_events[-1]['costmap_ack_mismatch_cells']==0
        assert session.ack_events[-1]['costmap_ack_status']=='exact_effective_master_content_verified'
        assert session.ack_events[-1]['semantic_exact_stable_observations']>=2
        assert not service.dynamic.obstacles and not service.dynamic.mask.any() and service.dynamic.version==2
        try:service.update(6,clear=['box'],observed_free=False)
        except ValueError as exc:
            assert str(exc)=='CLEAR_REQUIRES_FREE_OBSERVATION';record('unobserved_clear_rejected',{'error':str(exc)})
        else:raise AssertionError('clear without free observation accepted')
        assert service.current_trajectory() is None
        try:service.update(5,upsert={'box':polygon})
        except ValueError as exc:
            assert str(exc)=='CONFLICTING_SAME_SEQUENCE';record('conflicting_sequence_rejected',{'error':str(exc)})
        else:raise AssertionError('conflicting same sequence accepted')
        assert service.dynamic.version==2 and service.dynamic.sequence==5 and not service.dynamic.mask.any()
        assert service.current_trajectory() is None
        for e in events:
            if e['label'] in ('initial_plan','upsert','observed_clear'):
                assert e['trajectory_version']==e['dynamic_version']
        assert len({x['native_pid'] for x in events})==1 and len({x['session_identity'] for x in events})==1
    except BaseException:failure=traceback.format_exc()
    finally:
        for name,obj in [('service',service),('session',session)]:
            if obj is not None:
                try:obj.close()
                except BaseException:errors.append(name+': '+traceback.format_exc())
    result={'solver_sources':solver_sources,'solver_root':str(solver_root),'events':events,'exception':failure,'cleanup_errors':errors,'worker_exit':getattr(getattr(getattr(session,'stack',None),'process',None),'returncode',None)}
    (out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    if failure:print(failure,flush=True)
    return int(bool(failure or errors or result['worker_exit']!=0))

def main():
    if len(sys.argv)>1 and sys.argv[1]=='--worker':return worker(Path(sys.argv[2]),sys.argv[3])
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--solver-root',type=Path,required=True)
    args=parser.parse_args()
    solver_root=args.solver_root.resolve()
    if not solver_root.is_dir():parser.error('solver root must exist')
    from dynamic_test_process import run_bounded
    # Refuse external discovery/profile injection; do not read these values.
    for name in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI','ROS_DISCOVERY_SERVER','ROS_STATIC_PEERS','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_FASTRTPS_USE_QOS_FROM_XML','ROS_SECURITY_ENABLE','ROS_SECURITY_STRATEGY','ROS_SECURITY_KEYSTORE','ROS_SECURITY_ENCLAVE_OVERRIDE','ROS_NAMESPACE','LD_PRELOAD','PYTHONHOME'):
        if os.environ.get(name):raise ValueError('Unsafe test environment override: '+name)
    out=Path(tempfile.mkdtemp(prefix='trilayer-dynamic-session-'))
    env=dict(os.environ,ROS_DOMAIN_ID='184',ROS_LOCALHOST_ONLY='1',PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    execution=run_bounded([sys.executable,str(Path(__file__).resolve()),'--worker',str(out),str(solver_root)],out/'worker.log',timeout=60.,env=env,cwd=out)
    (out/'execution.json').write_text(json.dumps(execution,indent=2))
    print(str(out),json.dumps(execution),flush=True)
    return int(execution['returncode']!=0 or execution['timed_out'] or bool(execution['remaining_processes']))
if __name__=='__main__':raise SystemExit(main())
