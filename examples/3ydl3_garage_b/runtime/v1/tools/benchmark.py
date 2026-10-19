"""Honest paired runner: all outcomes, immutable inputs, explicit r1 baseline."""
import argparse,gc,hashlib,json,math,os,resource,signal,threading,time,shutil
from pathlib import Path
import numpy as np
import psutil,yaml
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation import unified_four_backends_smoke as runtime,topology
from arena_evaluation.path_audit import PathAuditor
from arena_evaluation import two_layer_v2_semantic_benchmark as context_util
from arena_3d_v1.r1_pipeline import Layered3DV1R1Controller
from arena_3d_v1.production_l1 import DeterministicGraphAStarL1
from arena_3d_v1.pipeline import ProductionL3Adapter,PipelineStep,corridor_dirty_transition
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write,digest
from arena_3d_v1.semantic_graph import SemanticPoseGraph
from arena_3d_v1.semantic_pipeline import SemanticR1Controller,ReferenceSmacSession
from arena_3d_v1.semantic_reference import preference_field,exact_baseline_costmap
from scipy.spatial import cKDTree

class Deadline(Exception):pass
def deadline_handler(*_):raise Deadline('QUERY_PIPELINE_40S_GUARD')
class Memory:
 def __enter__(self):
  self.stop=threading.Event();self.peak=0;self.root=psutil.Process();self.cpu_initial={};self.cpu_last={}
  for p in [self.root,*self.root.children(recursive=True)]:
   try:self.cpu_initial[p.pid]=sum(p.cpu_times()[:2])
   except psutil.Error:pass
  def sample():
   while not self.stop.is_set():
    total=0
    for p in [self.root,*self.root.children(recursive=True)]:
     try:
      total+=p.memory_info().rss;self.cpu_last[p.pid]=sum(p.cpu_times()[:2])
     except psutil.Error:pass
    self.peak=max(self.peak,total);self.stop.wait(.02)
  self.thread=threading.Thread(target=sample,daemon=True);self.thread.start();return self
 def __exit__(self,*_):
  self.stop.set();self.thread.join()
  self.tree_cpu_ms=sum(max(0.,v-self.cpu_initial.get(pid,0.)) for pid,v in self.cpu_last.items())*1000

def arc_metrics(world,poses,pref):
 p=np.asarray(poses);ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);arc=np.r_[0,np.cumsum(ds)]
 if arc[-1]<=0:return {}
 boundaries=np.r_[np.arange(0,arc[-1],.025),arc[-1]];mid=(boundaries[:-1]+boundaries[1:])/2;weights=np.diff(boundaries)
 ix=np.minimum(np.searchsorted(arc,mid,side='right')-1,len(ds)-1);f=np.divide(mid-arc[ix],ds[ix],out=np.zeros(len(ix)),where=ds[ix]>0)
 xy=p[ix,:2]+f[:,None]*(p[ix+1,:2]-p[ix,:2]);r,c=world.cells(xy)
 good=(r>=0)&(c>=0)&(r<world.map.height)&(c<world.map.width);q=np.full(len(r),np.nan);elig=np.zeros(len(r),bool)
 q[good]=pref.normalized_right[r[good],c[good]];elig[good]=pref.eligible[r[good],c[good]]
 target=(q>=.125)&(q<=.375);inside=elig&(mid>=5)&(mid<=arc[-1]-5)
 total=float(weights[inside].sum());alltotal=float(weights[elig].sum())
 return {'length_m':float(arc[-1]),'eligible_arc_length_m':alltotal,'trimmed_eligible_arc_length_m':total,
  'right_band_ratio':float(weights[inside&target].sum()/total) if total else None,
  'full_eligible_right_band_ratio':float(weights[elig&target].sum()/alltotal) if alltotal else None,
  'arc_sampling_m':.025,'partial_last_interval_included':True}

def baseline_step(c):
 target=c._target_mask();c.pending_l3_mask=target;dirty=corridor_dirty_transition(c.server_l3_mask,target);c._pending_l3_hash=dirty.target_hash
 return PipelineStep(None,None,c.initial_l2_result,False,False,True,'',target,dirty,c.plan.route_signature,{})

def reference_metrics(world,points,l2,pref):
 rc=np.asarray(l2);xy=world.world_points(rc[:,0],rc[:,1]);delta=np.diff(xy,axis=0)
 yaw=np.arctan2(delta[:,1],delta[:,0]);l2poses=np.column_stack([xy,np.r_[yaw,yaw[-1]]])
 m2=arc_metrics(world,l2poses,pref);m3=arc_metrics(world,points,pref)
 # Arc-midpoint samples prevent dense pose sampling from weighting turns.
 ds=np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1);s=np.r_[0,np.cumsum(ds)]
 t=np.arange(.0125,s[-1],.025);x=np.interp(t,s,points[:,0]);y=np.interp(t,s,points[:,1])
 distance=cKDTree(xy).query(np.column_stack([x,y]))[0]
 a=m2['right_band_ratio'];b=m3['right_band_ratio']
 return {'l2':m2,'l3':m3,'l2_to_l3_band_ratio':b/a if a and b is not None else None,
  'reference_deviation_mean_m':float(np.mean(distance)),'reference_deviation_p95_m':float(np.quantile(distance,.95)),
  'reference_deviation_max_m':float(np.max(distance)),'deviation_measurement':'arc_uniform_0.025m_to_l2_sampled_polyline_vertices'}

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--indices',default='8:16');parser.add_argument('--repeats',type=int,default=1);parser.add_argument('--arms',default='original_r1,A,B,C');parser.add_argument('--domain',type=int,default=226);args=parser.parse_args()
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 contract=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());freeze=json.loads((WORK/'config/FREEZE.json').read_text())
 assert hashlib.sha256((WORK/'config/acceptance.yaml').read_bytes()).hexdigest()==freeze['acceptance_sha256']
 assert hashlib.sha256((WORK/'config/queries.yaml').read_bytes()).hexdigest()==freeze['queries_sha256']
 os.environ['ROS_DOMAIN_ID']=str(args.domain);os.environ['ROS_LOCALHOST_ONLY']='1';os.sched_setaffinity(0,{15})
 signal.signal(signal.SIGALRM,deadline_handler)
 setup=time.monotonic();world=SemanticWorld(contract['inputs']['map_yaml'],contract['inputs']['semantic_path'],cache_root=WORK/'cache')
 graph=SemanticPoseGraph(world,WORK/'cache/graph');graph.prepare()
 ctx=context_util._context(Path(contract['inputs']['map_yaml']));auditor=PathAuditor(ctx,source_commit='isolated-explicit-r1-dual-map')
 tdir=Path('/home/robot/pudu_robot_ws/private_data/pudu_wanda_3f/results/real_ablation_r1_diag_v20_final8/topology_cache')
 artifact=topology.load_topology(tdir,ctx.hospital_map,runtime.FOOTPRINT,padding_m=.05,safety_margin_m=.05,allow_unknown=False)
 original=DeterministicGraphAStarL1(ctx,artifact,map_hash=ctx.map_sha256,topology_hash=hashlib.sha256((tdir/'topology_graph.json').read_bytes()).hexdigest())
 spec=runtime.backend_availability()['hybrid_astar']
 session=ReferenceSmacSession(ctx,out/'smac',map_yaml=Path(contract['inputs']['map_yaml']),local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
 session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
 data=yaml.safe_load((WORK/'config/queries.yaml').read_text())['queries'];lo,hi=map(int,args.indices.split(':'));queries=data[lo:hi];arms=args.arms.split(',')
 protocol={'acceptance':freeze,'query_ids':[q['query_id'] for q in queries],'repeats':args.repeats,'arms':arms,'cpu':15,'source_files':{},'graph_key':graph.key,'graph_nodes':len(graph.nodes),'graph_edges':len(graph.edges),'setup_wall_ms':(time.monotonic()-setup)*1000,'world_base_cache_hit':world.cache_hit,'graph_cache_hit':graph.cache_hit,
  'note':'Each row records actual cold/warm state. Query route preparation is shared before arms and recorded separately; no speedup claim.'}
 protocol['transport']={k:os.environ.get(k) for k in ['RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','ROS_LOCALHOST_ONLY','ROS_DOMAIN_ID']}
 protocol['search_and_smoothing_split']='not separately exposed by the frozen backend; combined planning time is recorded without inventing a split'
 (out/'source_snapshot').mkdir()
 for p in (WORK/'external/arena4_ws/src/arena/three_d_v1/arena_3d_v1').glob('*.py'):
  protocol['source_files'][str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();shutil.copy2(p,out/'source_snapshot'/p.name)
 shutil.copy2(__file__,out/'source_snapshot'/Path(__file__).name)
 for p in [WORK/'env.bash',WORK/'config/fastdds_large_map.xml',WORK/'config/SELECTED_WEIGHTS.json',WORK/'config/IMPLEMENTATION_FREEZE.json']:
  shutil.copy2(p,out/'source_snapshot'/p.name)
 json_write(out/'protocol.json',protocol);rows=[]
 try:
  session.start();protocol['smac_startup_ms']=session.stack_startup_time_ms
  protocol['loaded_backend_libraries']={}
  for line in Path(f'/proc/{session.planner_pid}/maps').read_text().splitlines():
   if any(k in line for k in ['nav2_smac_planner','nav2_costmap_2d','librmw_fastrtps']):
    p=Path(line.split()[-1])
    if p.is_file() and str(p) not in protocol['loaded_backend_libraries']:protocol['loaded_backend_libraries'][str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
  json_write(out/'protocol.json',protocol)
  for qdict in queries:
   q=Query(qdict['query_id'],tuple(map(float,qdict['start'])),tuple(map(float,qdict['goal'])),seed=qdict.get('seed',20260914))
   for repetition in range(args.repeats):
    before=time.monotonic();cpu_l1=time.process_time();route=graph.plan(q);semantic_l1_ms=(time.monotonic()-before)*1000;semantic_l1_cpu=(time.process_time()-cpu_l1)*1000
    before=time.monotonic();cpu_l1=time.process_time();original_plan=original.plan(q);original_l1_ms=(time.monotonic()-before)*1000;original_l1_cpu=(time.process_time()-cpu_l1)*1000
    pref=preference_field(world,route) if route else None
    print('QUERY',q.query_id,'repeat',repetition+1,'semantic_route',bool(route),'original_route',bool(original_plan),flush=True)
    order=arms if repetition%2==0 else list(reversed(arms))
    for arm in order:
     ident=f'{q.query_id}__{arm}__{repetition+1}';case=out/'cases'/ident;case.mkdir(parents=True)
     row={'query_id':q.query_id,'arm':arm,'repetition':repetition+1,'success':False,'failure_code':'','shared_l1_ms':original_l1_ms if arm=='original_r1' else semantic_l1_ms,
      'shared_l1_cpu_ms':original_l1_cpu if arm=='original_r1' else semantic_l1_cpu,'semantic_l1_diagnostics':route.diagnostics if route else graph.last_query_diagnostics,
      'cache_root':str(out/'l2_caches'/arm)}
     controller=None;started=time.monotonic();cpu=time.process_time()
     with Memory() as mem:
      try:
       signal.alarm(40)
       session.reset_query_state(ident,restore_base_map=False)
       if arm=='original_r1':
        if original_plan is None:raise RuntimeError('ORIGINAL_L1_UNVERIFIED_NO_ROUTE')
        tick=time.monotonic();l2cpu=time.process_time();controller=Layered3DV1R1Controller(original_plan,cache_root=out/'l2_caches'/arm);row['l2_activation_ms']=(time.monotonic()-tick)*1000;row['l2_activation_cpu_ms']=(time.process_time()-l2cpu)*1000
        if not controller.initial_l2_result.success:raise RuntimeError('ORIGINAL_L2_NO_PATH')
        baseline_grid=exact_baseline_costmap(session,controller._target_mask());session.set_semantic_costmap(baseline_grid)
        outcome=ProductionL3Adapter(controller,auditor).plan(baseline_step(controller),q,session,spec)
        outcome['reference_diagnostics']=baseline_grid.diagnostics
       else:
        if route is None:raise RuntimeError('SEMANTIC_L1_UNVERIFIED_NO_ROUTE')
        tick=time.monotonic();l2cpu=time.process_time();controller=SemanticR1Controller(world,route,cache_root=out/'l2_caches'/arm,l2_weight=0. if arm=='A' else 2.);row['l2_activation_ms']=(time.monotonic()-tick)*1000;row['l2_activation_cpu_ms']=(time.process_time()-l2cpu)*1000
        row['preference_build_ms']=controller.preference.diagnostics['build_ms']
        row['preference_build_cpu_ms']=controller.preference.diagnostics['cpu_ms']
        outcome=controller.plan_l3(q,session,auditor,spec,enabled=arm=='C',cap=140)
       row['l2']=controller.initial_l2_result.diagnostics
       if controller.l2.path_global:np.save(case/'l2.npy',np.asarray(controller.l2.path_global))
       result=outcome.pop('result',None)
       if result and result.points:
        points=np.asarray([[v['x'],v['y'],v['yaw']] for v in result.points]);np.save(case/'path.npy',points)
        json_write(case/'path.json',result.points)
        independent=outcome.get('independent_audit') or world.audit(q,points,controller._target_mask());outcome['independent_audit']=independent
        outcome['success']=bool(outcome.get('success') and independent['valid'])
        if not independent['valid']:outcome['failure_code']='INDEPENDENT_AUDIT_REJECTED'
        row['pose_hash']=digest(points)
        if pref:
         row['preference']=arc_metrics(world,points,pref)
         if controller.l2.path_global:row['reference_metrics']=reference_metrics(world,points,controller.l2.path_global,pref)
       row['success']=bool(outcome.get('success'));row['failure_code']=outcome.get('failure_code','')
       json_write(case/'outcome.json',outcome)
       row['outcome_path']=str(case/'outcome.json')
       row['l3_diagnostics']=outcome.get('diagnostics',{})
       row['reference_diagnostics']=outcome.get('reference_diagnostics',{})
       row['independent_audit']=outcome.get('independent_audit',{})
      except Exception as e:
       row['failure_code']=type(e).__name__+': '+str(e)
       json_write(case/'exception.json',{'type':type(e).__name__,'message':str(e)})
      finally:
       signal.alarm(0);row['request_wall_ms']=(time.monotonic()-started)*1000;row['request_cpu_ms']=(time.process_time()-cpu)*1000
       row['end_to_end_wall_ms']=row['request_wall_ms']+row['shared_l1_ms'];row['end_to_end_cpu_ms']=row['request_cpu_ms']+row['shared_l1_cpu_ms']
       if controller:controller.lifecycle.clear()
     row['peak_process_tree_rss_bytes']=mem.peak
     row['request_process_tree_cpu_ms']=mem.tree_cpu_ms
     row['end_to_end_process_tree_cpu_ms']=mem.tree_cpu_ms+row['shared_l1_cpu_ms']
     rows.append(row)
     with (out/'runs.jsonl').open('a') as stream:stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
     print('RUN',ident,row['success'],row['failure_code'],round(row['request_wall_ms']),flush=True)
     controller=None;gc.collect()
  json_write(out/'summary.json',{'count':len(rows),'success_count':sum(r['success'] for r in rows),'by_arm':{a:{'count':sum(r['arm']==a for r in rows),'success':sum(r['arm']==a and r['success'] for r in rows)} for a in arms},'all_failures_retained':True})
 finally:session.close()
if __name__=='__main__':main()
