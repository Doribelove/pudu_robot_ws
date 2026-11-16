"""C1 portable execution: original solver code, relocatable resources, Session-owned close."""
import os,sys,json,pathlib,hashlib,importlib.util,importlib.machinery,traceback,time,resource,io,tarfile,base64
ROOT=pathlib.Path('/isolated_bundle');OUT=pathlib.Path('/mnt/result');OUT.mkdir();SESSION=None;SERVICE=None;CONTEXTS=[];DESTROYED=set()
META={'version':'A2B-P6-closeout-C1','bootstrap_ack':False,'close_complete':False,'requests':0,'adaptations':[]}
def sha(p):return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def write(name,obj):(OUT/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2,default=lambda v:v.tolist() if hasattr(v,'tolist') else str(v))+'\n')
def mark(stage):print(json.dumps({'event':'stage','stage':stage,'t':time.monotonic()}),flush=True)
def main():
 global SESSION,SERVICE
 mode,mid,qid=sys.argv[1].split(':');META.update(mode=mode,map_id=mid,query_id=qid)
 assert not pathlib.Path('/home/robot').exists()
 assert os.stat('/proc/self/ns/net').st_ino!=int(os.environ['HOST_NET_NS_INODE'])
 assert os.stat('/proc/self/ns/ipc').st_ino!=int(os.environ['HOST_IPC_NS_INODE'])
 assert resource.getrlimit(resource.RLIMIT_AS)==(4*1024**3,4*1024**3)
 if mode.startswith('fixture'):
  import subprocess
  probe=pathlib.Path('/mnt/quota_probe');probe.write_bytes(bytes(2*1024**2))
  readonly=[]
  for name in ['/isolated_bundle/readonly_probe','/usr/readonly_probe','/review/readonly_probe']:
   try:pathlib.Path(name).write_text('x')
   except OSError as e:readonly.append({'path':name,'errno':e.errno})
   else:raise RuntimeError('HOST_WRITE_SUCCEEDED')
  META['fixture']={'readonly_rejections':readonly,'tmpfs_available':os.statvfs('/mnt').f_bavail*os.statvfs('/mnt').f_frsize,'AS':resource.getrlimit(resource.RLIMIT_AS),'FSIZE':resource.getrlimit(resource.RLIMIT_FSIZE),'NOFILE':resource.getrlimit(resource.RLIMIT_NOFILE),'private_net':True,'private_ipc':True,'PID':os.getpid()}
  if mode=='fixture-orphan':
   child=subprocess.Popen(['/usr/bin/sleep','40'],start_new_session=True);META['fixture']['intentional_orphan_pid']=child.pid
  META['bootstrap_ack']=None;return
 for row in json.loads((ROOT/'audit/ASSEMBLED_MANIFEST.json').read_text())['files']:
  p=(ROOT/row['destination']).resolve();assert p.is_relative_to(ROOT) and sha(p)==row['sha256'],str(p)
 env=json.loads((ROOT/'support/entry_v9r1/ENVIRONMENT_CONTRACT_V9.json').read_text())['values']
 for k,v in env.items():
  if k.startswith(('THREEYD_','V13_')):os.environ[k]=v
 # Mutable caches relocate into bounded scratch; all map and source inputs stay read-only.
 os.environ['THREEYD_V12_INDEX_ROOT']='/mnt/indices';os.environ['THREEYD_V12_PLANE_ROOT']='/mnt/planes'
 sys.path[:]=['/review']+[str(ROOT/p) for p in ['runtime/profiles/p6/src','runtime/base/src','runtime/base/runtime/base/src','runtime/base/runtime','runtime/base/deps','numeric','ros/lib/python3.10/site-packages','tools','external/home/robot/workspaces/3ydl3_v12_r0_completion_b_20261020/tools']]+['/usr/lib/python3.10','/usr/lib/python3.10/lib-dynload','/usr/lib/python3/dist-packages']
 from diagnostic_support_v18 import ExactSemanticLoader
 loaders=[]
 class Finder:
  def find_spec(self,name,path=None,target=None):
   if name!='arena_3d_v1.semantic_world':return None
   spec=importlib.machinery.PathFinder.find_spec(name,path,target);spec.loader=ExactSemanticLoader(name,spec.origin,loaders,META['adaptations']);return spec
 sys.meta_path.insert(0,Finder())
 import arena_3d_v1.semantic_world as sw
 import arena_3d_v1.semantic_pipeline as sp
 sw.WORK=sp.WORK=ROOT/'runtime/resources/semantic'
 from arena_evaluation import unified_four_backends_smoke as rt
 from three_yd_v5.free_space_topology import load_map_only_world,FreeSpaceTopology
 from three_yd_v5.v10_storage import prepare,prepare_topology
 from three_yd_v5.curve_session import CurveSession
 spec=importlib.util.spec_from_file_location('portable_entry',ROOT/'support/entry_v9r1/portable_entry_v9r2.py');entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
 verified=entry.verify_only(ROOT);assert verified['pass'];write('input_verification.json',verified)
 docs=entry.render_overlay_in_memory(ROOT)
 mapping={old:str(pathlib.Path('/mnt/overlay')/pathlib.Path(old).relative_to(ROOT/'support/entry_v9r1/overlay')) for old in docs}
 for old,body in docs.items():
  for k,v in mapping.items():body=body.replace(k,v)
  dest=pathlib.Path(mapping[old]);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(body)
 registry=json.loads(pathlib.Path(mapping[str(ROOT/'support/entry_v9r1/overlay/B/MAPS.json')]).read_text());m=next(x for x in registry['maps'] if x['map_id']==mid);mp=pathlib.Path(m['map_yaml']);semantic=m['kind']!='map_only';inp=pathlib.Path(m['inputs'])
 mark('world.start')
 if semantic:
  from three_yd_v5.v10_firstready import firstready_construction
  with firstready_construction():world=sw.SemanticWorld(mp,inp/'semantic.json',cache_root=OUT/'world');prepare(world)
 else:world=load_map_only_world(mp,OUT/'world');prepare(world)
 mark('world.ready')
 ctx=rt.MapContext(mid,world.map,world.safe,world.map.distance_m,world.map.sha256,sha(mp),mp)
 import rclpy.context as context_module
 OriginalContext=context_module.Context
 class OwnedContext(OriginalContext):
  def __init__(self,*a,**k):
   super().__init__(*a,**k);CONTEXTS.append(self)
  def destroy(self):
   result=super().destroy();DESTROYED.add(id(self));return result
 context_module.Context=OwnedContext
 SESSION=CurveSession(ctx,OUT/'session',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,tile_interval_s=.015,enable_semantic_repair=True)
 SESSION.local_map_update_strategy='roi_ack';SESSION.full_grid_settle_cycles=0
 mark('session.constructed');SESSION.start();mark('session.started')
 expected_boot_hash=hashlib.sha256(bytes([254])*256).hexdigest()
 META['bootstrap_evidence']=[x for x in SESSION.ack_events if x.get('costmap_ack_status')=='cold_exact_content_verified']
 META['bootstrap_expected']={'shape':[16,16],'cells':256,'SHA256':expected_boot_hash,'minimum_stable_observations':2}
 META['bootstrap_ack']=any(x.get('costmap_update_acknowledged') is True and x.get('costmap_ack_mismatch_cells')==0 and x.get('costmap_ack_checked_cells')==256 and x.get('costmap_ack_stable_observations',-1)>=2 and x.get('server_costmap_content_hash')==expected_boot_hash for x in META['bootstrap_evidence']);assert META['bootstrap_ack'],'bootstrap exact256 ACK missing'
 write('bootstrap_ack.json',SESSION.ack_events)
 if mode=='bootstrap':return
 mark('topology.start')
 graph_input=inp
 if semantic and not (inp/'parking_access.json').is_file():
  missing=pathlib.Path('/supplement/inputs')/mid/'parking_access.json';assert missing.is_file(),'missing semantic companion'
  graph_input=pathlib.Path('/mnt/topology_inputs')/mid;graph_input.mkdir(parents=True)
  for name in ['topology.json','semantic.json']:(graph_input/name).symlink_to(inp/name)
  (graph_input/'parking_access.json').symlink_to(missing)
  META['semantic_companion']={'path':str(missing),'sha256':sha(missing)}
 from three_yd_v5.v12_topology import persistent
 from three_yd_v5.topology_default import DefaultTopology
 graph=persistent(world,DefaultTopology if semantic else FreeSpaceTopology,pathlib.Path('/mnt/indices')/mid,graph_input/'topology.json' if semantic else None)
 if semantic:prepare_topology(graph)
 from three_yd_v5.v12_fields import prepare_fields
 fields=prepare_fields(world,graph,pathlib.Path('/mnt/planes'));write('block_fields.json',fields)
 from arena_3d_v1.multires_grid import GridView
 from three_yd_v5.v9_gate_init import DirectStorageAuditor
 from three_yd_v13.service import IncrementalService
 from arena_evaluation.planner_benchmark.models import Query
 auditor=DirectStorageAuditor(ctx,source_commit='P6-C1-portable');views={3:GridView(world,3),1:GridView(world,1)}
 SERVICE=IncrementalService(world,graph,OUT/'l2_cache',views,SESSION,auditor,rt.backend_availability()['hybrid_astar'],arm='H-I',max_region_routes=3 if semantic else 1)
 qd=next(x for x in m['poses'] if x['id']==qid)
 def pose(v):return tuple(v[k] for k in ('x','y','yaw_rad')) if isinstance(v,dict) else tuple(v)
 q=Query(qid,pose(qd['start']),pose(qd['goal']),seed=20260916 if semantic else 20260929)
 SESSION.reset_query_state(qid,restore_base_map=False);mark('request.start');tick=time.monotonic();cpu=time.process_time();META['requests']+=1
 result=SERVICE.plan(q);META['request_wall_s']=time.monotonic()-tick;META['request_cpu_s']=time.process_time()-cpu;path=result.pop('result',None);write('outcome.json',result)
 if path and path.points:write('path.json',path.points)
 c=SERVICE.controller;state=getattr(getattr(c,'l2',None),'state',None)
 META['runtime_identity']={k:type(v).__module__+'.'+type(v).__name__ for k,v in [('service',SERVICE),('controller',c),('L2',getattr(c,'l2',None)),('state',state)]};META['success']=result.get('success');META['failure_code']=result.get('failure_code');META['exact_endpoint_success']=result.get('exact_endpoint_success');META['tolerant_endpoint_success']=result.get('tolerant_endpoint_success');META['strict_audit']=result.get('constraint_audit');META['independent_audit']=result.get('independent_audit')
 if c and hasattr(c,'initial_l2_result'):write('l2.json',c.initial_l2_result.diagnostics)
 write('all_ack_events.json',SESSION.ack_events);mark('request.returned')
 assert result.get('success'),'planning failure:'+str(result.get('failure_code'))

if __name__=='__main__':
 failure=None
 try:main()
 except BaseException as e:failure={'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()}
 finally:
  cleanup_errors=[]
  for label,obj in [('service',SERVICE),('session',SESSION)]:
   if obj is not None:
    try:obj.close()
    except BaseException as e:cleanup_errors.append({'owner':label,'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()})
  # Successful Session.close destroys its own Context. Only unclosed construction/failure leftovers are reclaimed here.
  for ctx in CONTEXTS:
   if id(ctx) not in DESTROYED:
    try:
     if ctx.ok():ctx.shutdown()
     ctx.destroy()
    except BaseException as e:cleanup_errors.append({'owner':'unclosed_context','type':type(e).__name__,'message':str(e)})
  META['context_ownership']={'constructed':len(CONTEXTS),'destroyed':len(DESTROYED),'single_session_owner':True}
  META['close_complete']=not cleanup_errors and len(CONTEXTS)==len(DESTROYED);META['cleanup_errors']=cleanup_errors
  META['failure']=failure;META['pass']=failure is None and (META.get('mode','').startswith('fixture') or META['bootstrap_ack']) and META['close_complete'];write('worker_result.json',META)
  loaded={}
  for n,m in sorted(sys.modules.items()):
   f=getattr(m,'__file__',None)
   if f and pathlib.Path(f).is_file():loaded[n]={'file':f,'sha256':sha(f)}
  write('loaded_modules.json',loaded)
  members=[p for p in sorted(OUT.rglob('*')) if p.is_file() and p.stat().st_size<8*1024**2]
  assert len(members)<=4096 and sum(p.stat().st_size for p in members)<=64*1024**2
  b=io.BytesIO()
  with tarfile.open(fileobj=b,mode='w:gz') as tar:
   for p in members:tar.add(p,arcname=str(p.relative_to(OUT)),recursive=False)
  assert len(b.getvalue())<24*1024**2
  print(json.dumps({'event':'artifacts','tar_gz':base64.b64encode(b.getvalue()).decode()}),flush=True)
  print(json.dumps({'event':'terminal',**META},default=str),flush=True)
 raise SystemExit(0 if META['pass'] else 1)
