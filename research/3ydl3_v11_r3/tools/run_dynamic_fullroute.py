"""Deterministic obstacle-event replay; no moving robot or controller claims."""
if __name__=='__main__':
    from owned_process import supervised_entry
    supervised_entry()
import argparse,json,hashlib,os,signal,time,traceback
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from benchmark import Memory
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.semantic_world import SemanticWorld,json_write
from arena_3d_v1.multires_grid import GridView
from three_yd_v5.free_space_topology import FreeSpaceTopology,load_map_only_world
from three_yd_v5.topology_default import DefaultTopology
from three_yd_v5.performance_session import PerformanceSession
from three_yd_v5.fullroute_service import FullRouteService
from run_wanda3w import json_native,deadline_handler
from run_free_full import registered_input
ROOT=Path(__file__).resolve().parents[1]

def square(xy,half):
    x,y=map(float,xy)
    return [[x-half,y-half],[x+half,y-half],[x+half,y+half],[x-half,y+half]]

def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['semantic','free-space'],required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,default=220)
    a=p.parse_args();out=a.output;out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(a.domain);signal.signal(signal.SIGALRM,deadline_handler)
    if a.mode=='semantic':
        mid='mengjia';mp=ROOT/'inputs/corroborated_map.yaml'
        w=SemanticWorld(mp,ROOT/'inputs/semantic.json',cache_root=out/'world')
        g=DefaultTopology(w,ROOT/'inputs/topology.json',out/'graph')
        qd=next(q for q in json.loads((ROOT/'inputs/queries.json').read_text())['scenarios'] if q['id']=='Q06')
        q=Query(qd['id'],tuple(qd['start'][k] for k in ('x','y','yaw_rad')),tuple(qd['goal'][k] for k in ('x','y','yaw_rad')),seed=20260916)
    else:
        mid='arena_large_250x300_2d_005';directory,_,qs=registered_input(mid);mp=directory/'map.yaml'
        w=load_map_only_world(mp,out/'world');g=FreeSpaceTopology(w,out/'graph')
        qd=qs[0];q=Query(qd['id'],tuple(qd['start']),tuple(qd['goal']),seed=20260929)
    views={3:GridView(w,3),1:GridView(w,1)}
    context=runtime.MapContext(mid,w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
    auditor=PathAuditor(context,source_commit='V8-r7-fullroute-dynamic')
    spec=runtime.backend_availability()['hybrid_astar']
    s=PerformanceSession(context,out/'smac',map_yaml=mp,local_mask_updates=True,
        optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',
        optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,
        planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,
        tile_interval_s=.015,enable_semantic_repair=True)
    s.local_map_update_strategy='roi_ack';s.full_grid_settle_cycles=0
    service=FullRouteService(w,g,out/'l2_cache',views,s,auditor,spec,max_region_routes=3 if a.mode=='semantic' else 1)
    events=[]
    def save(label,fn):
        s.reset_query_state(label,restore_base_map=False);tick=time.monotonic()
        with Memory() as memory:
            signal.alarm(90)
            try:result=fn()
            finally:signal.alarm(0)
        result=dict(result);result.pop('result',None)
        result.update(label=label,wall_ms=(time.monotonic()-tick)*1000,rss_peak_bytes=memory.peak)
        path=service.current_trajectory()
        result['trajectory_available']=path is not None
        if path is not None:
            np.save(out/(label+'_path.npy'),path)
            result['whole_path_audit']=w.audit(q,path,service.controller._target_mask())
            assert result['whole_path_audit']['valid'],label
        events.append(result)
        json_write(out/'events.json',events)
        print(label,result.get('success'),result.get('action'),round(result['wall_ms']),flush=True)
        return result
    try:
        s.start()
        from native_snapshot import record
        record(out)
        initial=save('initial_full',lambda:service.plan(q));assert initial['success']
        base=service.current_trajectory().copy()
        repeat=save('repeat_full',lambda:service.plan(q));assert repeat['success']
        assert np.array_equal(base,service.current_trajectory())
        c=service.controller
        # Deterministic physically free point away from the selected route.
        rc=np.column_stack(np.nonzero(w.safe & ~c._search_mask))[::97]
        xy=w.world_points(rc[:,0],rc[:,1]);dist=cKDTree(base[:,:2]).query(xy)[0]
        free=xy[np.argmax(dist)]
        assert dist.max()>3
        far=square(free,.10)
        remote=save('remote_add',lambda:service.update(1,upsert={'remote':far}))
        assert remote['success'] and not remote['replanned'] and remote['impact']['kind']=='unrelated'
        assert np.array_equal(base,service.current_trajectory())
        duplicate=save('duplicate',lambda:service.update(1,upsert={'remote':far}))
        assert duplicate['update']['reason']=='DUPLICATE'
        stale=save('out_of_order',lambda:service.update(0,upsert={}))
        assert not stale['update']['accepted']
        try:service.update(2,clear=['remote'])
        except ValueError as e:
            assert str(e)=='CLEAR_REQUIRES_FREE_OBSERVATION'
            events.append({'label':'unobserved_clear_rejected','passed':True,'failure':str(e)})
        else:raise AssertionError('unobserved clear accepted')
        save('remote_clear',lambda:service.update(2,clear=['remote'],observed_free=True))
        block=square(base[len(base)//2,:2],.08)
        near=save('path_add',lambda:service.update(3,upsert={'path':block}))
        assert near['replanned'] and (near['success'] or not near['trajectory_available'])
        restored=save('path_clear',lambda:service.update(4,clear=['path'],observed_free=True))
        assert restored['success']
        endpoint=square(q.start[:2],.40)
        blocked=save('endpoint_block',lambda:service.update(5,upsert={'endpoint':endpoint}))
        assert not blocked['success'] and not blocked['trajectory_available']
        final=save('endpoint_clear',lambda:service.update(6,clear=['endpoint'],observed_free=True))
        assert final['success']
        json_write(out/'summary.json',{'mode':a.mode,'map':mid,'query':q.query_id,
            'events':len(events),'assertions_passed':True,'unrelated_replanned':remote['replanned'],
            'path_obstacle_replanned':near['replanned'],'path_obstacle_success':near['success'],
            'endpoint_block_stopped':not blocked['trajectory_available'],'clear_restored':final['success'],
            'real_execution':False,'dynamic_scope':'deterministic world-polygon replay, static poses',
            'L1_modified':False,'whole_route_only':True,'moving_window':False})
    finally:
        service.close();s.close()
if __name__=='__main__':main()
