"""Frozen controls, same session, paired arms and explicit reference-only changes."""
import argparse,hashlib,json,math,os,time,shutil
from pathlib import Path
import numpy as np
from controlled_probe import controlled_inputs
from benchmark import arc_metrics,Memory
from arena_evaluation import unified_four_backends_smoke as runtime
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation.planner_benchmark.models import Query
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write,digest
from arena_3d_v1.semantic_graph import SemanticPoseGraph
from arena_3d_v1.semantic_pipeline import SemanticR1Controller,ReferenceSmacSession

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 snapshot=out/'source_snapshot';snapshot.mkdir();hashes={}
 for p in [*WORK.glob('external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/semantic_*.py'),WORK/'env.bash',WORK/'config/fastdds_large_map.xml',Path(__file__)]:
  hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();shutil.copy2(p,snapshot/p.name)
 json_write(out/'protocol.json',{'source_files':hashes,'calibration_and_rotated_heldout_repeats':3,'no_weights_changed_since_first_holdout_use':True,'first_heldout_use_retained_at':str(WORK/'results/controlled_formal_01'),'scope':'A, B, C right, C left, C right switchback, one session per scene','ros_domains_by_scene':{'calibration':220,'heldout':221}})
 selected=json.loads((WORK/'config/SELECTED_WEIGHTS.json').read_text());json_write(out/'selected_weights.json',selected)
 os.environ['ROS_DOMAIN_ID']='226';os.environ['ROS_LOCALHOST_ONLY']='1';os.sched_setaffinity(0,{15})
 rows=[]
 for heldout in [False,True]:
  os.environ['ROS_DOMAIN_ID']=str(220+int(heldout))
  scene='heldout' if heldout else 'calibration';mp,sp=controlled_inputs(WORK/'data'/scene,heldout)
  w=SemanticWorld(mp,sp,cache_root=WORK/'cache/controlled');g=SemanticPoseGraph(w,WORK/'cache/controlled_graph');g.prepare()
  q=Query(scene,(4.,5.,math.pi/2) if heldout else (5.,3.,0.),(4.,41.,math.pi/2) if heldout else (35.,3.,0.),seed=20260914)
  r=g.plan(q);ctx=runtime.MapContext(scene,w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
  auditor=PathAuditor(ctx,source_commit='isolated-r1-semantic-controls');spec=runtime.backend_availability()['hybrid_astar']
  session=ReferenceSmacSession(ctx,out/scene/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
  session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
  try:
   session.start()
   for rep in range(1,4):
    c=None
    for arm,side in [('A','right'),('B','right'),('C','right'),('C','left'),('C','right')]:
     tag=f'{scene}_{rep}_{arm}_{side}_{len(rows)}';case=out/'cases'/tag;case.mkdir(parents=True)
     started=time.monotonic();cpu=time.process_time();row={'scene':scene,'repetition':rep,'arm':arm,'side':side,'case':str(case)}
     with Memory() as mem:
      try:
       reference_only=arm=='C' and c is not None and c.lifecycle.weight>0 and prior_arm=='C'
       if reference_only:
        c.change_preference(side,2.);session.current_query_id=tag
       else:
        if c:c.lifecycle.clear()
        session.reset_query_state(tag,restore_base_map=False)
        c=SemanticR1Controller(w,r,cache_root=WORK/'cache/controlled_l2',side=side,l2_weight=0. if arm=='A' else 2.)
       row['reference_only_change']=reference_only;row['hard_changed_cells']=int(np.count_nonzero(c._target_mask()!=c.server_l3_mask))
       row['l2']=c.initial_l2_result.diagnostics;np.save(case/'l2.npy',np.asarray(c.l2.path_global));np.save(case/'corridor.npy',r.plan.corridor_mask)
       result=c.plan_l3(q,session,auditor,spec,enabled=arm=='C',cap=140);path=result.pop('result',None)
       if path and path.points:
        pts=np.array([[v['x'],v['y'],v['yaw']] for v in path.points]);np.save(case/'path.npy',pts)
        m=arc_metrics(w,pts,c.preference)
        # The left band's right-normalized interval is the mirror [.625,.875].
        if side=='left':
         old=c.preference.normalized_right;c.preference.normalized_right=1.-old
         m['target_band_ratio']=arc_metrics(w,pts,c.preference)['right_band_ratio'];c.preference.normalized_right=old
        else:m['target_band_ratio']=m['right_band_ratio']
        row['metrics']=m;row['pose_hash']=digest(pts)
        row['mean_transverse_position_m']=float(np.mean(pts[:,0] if heldout else pts[:,1]))
       row.update(success=bool(result.get('success')),failure_code=result.get('failure_code',''),outcome=result)
       json_write(case/'outcome.json',result)
      except Exception as e:row.update(success=False,failure_code=type(e).__name__+': '+str(e))
      row.update(wall_ms=(time.monotonic()-started)*1000,cpu_ms=(time.process_time()-cpu)*1000)
     row['peak_process_tree_rss_bytes']=mem.peak;rows.append(row)
     with (out/'runs.jsonl').open('a') as stream:stream.write(json.dumps(row,allow_nan=False)+'\n')
     print(tag,row['success'],row.get('metrics',{}).get('target_band_ratio'),row['failure_code'],flush=True)
     prior_arm=arm
    if c:c.lifecycle.clear()
  finally:session.close()
 json_write(out/'summary.json',{'count':len(rows),'success':sum(r['success'] for r in rows),'all_failures_retained':True})
if __name__=='__main__':main()
