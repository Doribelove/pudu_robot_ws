"""Isolated processes and identical fixed Smac for all static comparison arms."""
import argparse, gc, hashlib, json, math, os, signal, time, traceback
from pathlib import Path
import numpy as np
import psutil
import yaml
from benchmark import Memory, deadline_handler
from controlled_probe import controlled_inputs
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.semantic_world import SemanticWorld, WORK, json_write, digest
from arena_3d_v1.semantic_graph import SemanticPoseGraph
from arena_3d_v1.semantic_pipeline import SemanticR1Controller, ReferenceSmacSession
from arena_3d_v1.multires_topology import SemanticTopology
from arena_3d_v1.multires_pipeline import MultiresController
from arena_3d_v1.multires_grid import GridView

BASELINE=Path('/home/robot/workspaces/semantic_dual_map_r1_20260914')

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True)
    p.add_argument('--arm',choices=['baseline','topology_005','topology_015'],required=True)
    p.add_argument('--indices',default='0:20');p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--domain',type=int,default=223);p.add_argument('--scene',choices=['real','calibration','rotated','narrow'],default='real')
    args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{15});os.environ['ROS_DOMAIN_ID']=str(args.domain);os.environ['ROS_LOCALHOST_ONLY']='1'
    signal.signal(signal.SIGALRM,deadline_handler)
    cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text())
    if args.scene=='real':
        mp,sp=Path(cfg['inputs']['map_yaml']),Path(cfg['inputs']['semantic_path'])
        lo,hi=map(int,args.indices.split(':'));queries=yaml.safe_load((WORK/'config/queries.yaml').read_text())['queries'][lo:hi]
    elif args.scene=='narrow':
        from PIL import Image
        from arena_evaluation.semantic_map import SemanticMapV1,SemanticFeature
        inp=out/'inputs';inp.mkdir();grid=np.zeros((59,240),np.uint8);grid[20:39,20:220]=255
        Image.fromarray(grid).save(inp/'map.pgm');mp=inp/'map.yaml';sp=inp/'semantic.json'
        mp.write_text(yaml.safe_dump({'image':'map.pgm','resolution':.05,'origin':[-1.,-1.,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
        f=SemanticFeature('narrow','lane','polygon',[[0,0],[10,0],[10,.95],[0,.95],[0,0]],soft=True)
        json_write(sp,SemanticMapV1('map',.05,(-1.,-1.,0.),240,59,'narrow',[f]).to_dict())
        queries=[{'query_id':'narrow','start':(1.,.475,0.),'goal':(9.,.475,0.),'seed':20260914}]
    else:
        rotated=args.scene=='rotated';mp,sp=controlled_inputs(out/'inputs',rotated)
        queries=[{'query_id':args.scene,'start':(4.,5.,math.pi/2) if rotated else (5.,3.,0.),
            'goal':(4.,41.,math.pi/2) if rotated else (35.,3.,0.),'seed':20260914}]
    tick=time.monotonic();cpu=time.process_time()
    with Memory() as prep_mem:
        w=SemanticWorld(mp,sp,cache_root=out/'preparation_cache/world')
        if args.arm=='baseline':
            graph_cache=BASELINE/'cache/graph' if args.scene=='real' else out/'preparation_cache/graph'
            g=SemanticPoseGraph(w,graph_cache);g.prepare();views={}
        else:
            g=SemanticTopology(w,out/'preparation_cache/graph')
            factors=[1] if args.arm=='topology_005' else [1,3]
            views={f:GridView(w,f) for f in factors}
    prep={'wall_ms':(time.monotonic()-tick)*1000,'cpu_ms':(time.process_time()-cpu)*1000,'rss_peak_bytes':prep_mem.peak,
        'graph_cache_hit':g.cache_hit,'graph_preparation_ms':g.preparation_ms,
        'graph_nodes':len(g.nodes),'graph_edges':len(g.edges) if args.arm=='baseline' else g.edges_count,
        'graph_semantics':'validated_pose_connections' if args.arm=='baseline' else 'candidate_directed_semantic_interfaces',
        'preparation_cache_bytes':sum(x.stat().st_size for x in (out/'preparation_cache').rglob('*') if x.is_file())}
    json_write(out/'preparation.json',prep)
    ctx=runtime.MapContext(args.scene,w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
    auditor=PathAuditor(ctx,source_commit='3yd-static-multires-comparison');spec=runtime.backend_availability()['hybrid_astar']
    session=ReferenceSmacSession(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',
        smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,
        planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
    session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
    sources=[*WORK.glob('external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/*.py'),Path(__file__),WORK/'env.bash',WORK/'config/fastdds_large_map.xml']
    protocol={'arm':args.arm,'scene':args.scene,'queries':queries,'repeats':args.repeats,'cpu':15,'l3_resolution':.05,
        'vehicle':w.vehicle,'L2_weight':2.,'L3_cap':140,'saturation_m':2.,'static_only':True,
        'map_hash':w.map.sha256,'semantic_hash':w.semantic.semantic_map_hash,
        'query_file_sha256':hashlib.sha256((WORK/'config/queries.yaml').read_bytes()).hexdigest(),
        'source_hashes':{str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources},
        'timing':'query includes L1/controller/L3 and audits; metrics and case output happen after timer; preparation separate; all arms identical measurement',
        'baseline_graph_restore':'preexisting immutable baseline cache' if args.arm=='baseline' and args.scene=='real' else None}
    snap=out/'source_snapshot';snap.mkdir()
    for x in sources:(snap/x.name).write_bytes(x.read_bytes())
    json_write(out/'protocol.json',protocol);rows=[]
    try:
        session.start();protocol['smac_startup_ms']=session.stack_startup_time_ms;json_write(out/'protocol.json',protocol)
        for qd in queries:
            q=Query(qd['query_id'],tuple(map(float,qd['start'])),tuple(map(float,qd['goal'])),seed=qd.get('seed',20260914))
            for rep in range(1,args.repeats+1):
                sides=['right'] if args.scene in ('real','narrow') else ['right','left','right']
                for change,side in enumerate(sides):
                    tag=f'{q.query_id}__{rep}__{side}_{change}';case=out/'cases'/tag;case.mkdir(parents=True)
                    c=None;outcome={};path=None;r=None;tick=time.monotonic();cpu=time.process_time()
                    row={'query_id':q.query_id,'repetition':rep,'side':side,'switch_index':change,'arm':args.arm,'case':str(case),'success':False,'failure_code':''}
                    with Memory() as mem:
                        try:
                            signal.alarm(int(cfg['budgets']['new_query_pipeline_guard_s']));session.reset_query_state(tag,restore_base_map=False)
                            t=time.monotonic();lcpu=time.process_time();r=g.plan(q)
                            row['l1_ms']=(time.monotonic()-t)*1000;row['l1_cpu_ms']=(time.process_time()-lcpu)*1000
                            if r is None:raise RuntimeError('L1_NO_ROUTE')
                            row['l1_diagnostics']=r.diagnostics
                            t=time.monotonic();lcpu=time.process_time()
                            if args.arm=='baseline':c=SemanticR1Controller(w,r,cache_root=out/'l2_cache',side=side,l2_weight=2.)
                            else:c=MultiresController(w,g,r,cache_root=out/'l2_cache',factor=1 if args.arm=='topology_005' else 3,side=side,views=views)
                            row['l2_ms']=(time.monotonic()-t)*1000;row['l2_cpu_ms']=(time.process_time()-lcpu)*1000
                            row['l2']=c.initial_l2_result.diagnostics
                            outcome=c.plan_l3(q,session,auditor,spec,enabled=True,cap=140)
                            path=outcome.pop('result',None)
                            row['success']=bool(outcome.get('success'));row['failure_code']=outcome.get('failure_code','')
                        except Exception as e:
                            row['failure_code']=type(e).__name__+': '+str(e);(case/'exception.txt').write_text(traceback.format_exc())
                        finally:
                            signal.alarm(0);row['wall_ms']=(time.monotonic()-tick)*1000;row['cpu_ms']=(time.process_time()-cpu)*1000
                    row['process_tree_cpu_ms']=mem.tree_cpu_ms;row['rss_peak_bytes']=mem.peak
                    usage=[]
                    for proc in [psutil.Process(),*psutil.Process().children(recursive=True)]:
                        try:
                            mi=proc.memory_full_info();usage.append({'name':proc.name(),'rss_bytes':mi.rss,'pss_bytes':mi.pss})
                        except psutil.Error:pass
                    row['post_query_memory']=usage;row['post_query_tree_pss_bytes']=sum(v['pss_bytes'] for v in usage)
                    if c:
                        rc=np.asarray(c.l2.path_global)
                        np.save(case/'l2_native_cells.npy',rc)
                        if args.arm=='baseline':frc=rc
                        else:frc=c.reference_cells;row['multires']=c.timing;row['fallbacks']=c.fallbacks
                        np.save(case/'reference_fine_cells.npy',frc);np.save(case/'corridor.npy',c._target_mask())
                        row['hard_corridor_hash']=digest(c._target_mask())
                        if args.arm=='baseline' and rep==1 and side=='right':
                            np.savez_compressed(case/'evaluation_field.npz',q=c.preference.normalized_right,eligible=c.preference.eligible)
                    if path and path.points:
                        pts=np.asarray([[v['x'],v['y'],v['yaw']] for v in path.points]);np.save(case/'path.npy',pts)
                        row['pose_hash']=digest(pts);row['length_m']=float(np.linalg.norm(np.diff(pts[:,:2],axis=0),axis=1).sum())
                    row['l3']=outcome.get('diagnostics',{});row['reference']=outcome.get('reference_diagnostics',{});row['audit']=outcome.get('independent_audit',{})
                    json_write(case/'outcome.json',outcome);rows.append(row)
                    with (out/'runs.jsonl').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
                    print(args.arm,tag,row['success'],row['failure_code'],round(row['wall_ms']),flush=True)
                    if c:c.lifecycle.clear()
                    c=None;outcome=None;path=None;gc.collect()
    finally:session.close()
    json_write(out/'summary.json',{'count':len(rows),'success':sum(r['success'] for r in rows),'failures':[r['failure_code'] for r in rows if not r['success']]})

if __name__=='__main__':main()
