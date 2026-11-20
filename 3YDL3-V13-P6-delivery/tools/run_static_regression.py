"""Source-locked P6 initial-navigation regression. Serial, new outputs only."""
from pathlib import Path
import argparse, fcntl, hashlib, json, os, shlex, subprocess, sys
ROOT=Path(__file__).resolve().parents[1]
B=Path('/home/robot/workspaces/3ydl3_v12_r0_completion_b_20261020')
FLAGS=['--l3-backend','independent','--workload','fixed','--v9-mode','A','--review-mode','S','--r2-mode','D','--r3-mode','control','--r5-mode','C','--r6-mode','H','--r7-mode','NT','--r8-mode','OGR','--r9-mode','PLMX','--optimization-mode','AB','--repeats','1']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify(package):
 source=ROOT/'source/3YDL3-V13-L2';manifest=json.loads((source/'MANIFEST.json').read_text())
 assert sha(package/'MANIFEST.json')==sha(source/'MANIFEST.json')
 for rel,h in manifest['files'].items():assert sha(source/rel)==h and sha(package/rel)==h,rel
 assets=json.loads((ROOT/'docs/RUNTIME_ASSETS.json').read_text())
 for rel,item in assets['additions'].items():assert sha(package/rel)==item['sha256']==sha(item['source']),rel
 registry=json.loads((B/'MAPS.json').read_text());assert len(registry['maps'])==10
 for m in registry['maps']:
  assert len(m['poses'])==10,m['map_id']
  for p,h in m['input_sha256'].items():assert sha(p)==h,p
 return registry

def environment(package):
 env={k:v for k,v in os.environ.items()
      if k == 'V13_RUNTIME_ENV' or not k.startswith(('THREEYD_','V13_'))}
 env.update(THREEYD_BENCH_CPU='14',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',
  THREEYD_BENCH_SOURCE=str(B),V13_CLOSEOUT_PACKAGE=str(package),V13_PROFILE='p6',V13_P6_HINT_MODE='local',V13_P7_OBSTACLE_L1='0',
  V13_FORMAL='1',V13_CAPTURE_TRACE='0',THREEYD_L3_BACKEND='independent',
  THREEYD_V10_R3_MODE='ABRTHI',THREEYD_V10_R4_MODE='PHFBDJNTV',THREEYD_V10_R5_MODE='MLSEKU',THREEYD_V10_R6_MODE='DCGXY',THREEYD_V10_R7_MODE='ABCD',THREEYD_V10_R8_MODE='STPBE',
  THREEYD_ENDPOINT_TOLERANCE='1',THREEYD_L2_ROUTE_QUALITY='1',THREEYD_V12_BLOCK_FIELDS='1',THREEYD_V12_NATIVE_COMPACT='1',THREEYD_V12_PACK_SEARCH='1',THREEYD_V12_DERIVE_STATIC='1',
  THREEYD_V12_TILED_SWEEP='0',THREEYD_V12_CAPTURE_NATIVE='0',THREEYD_V12_PROVENANCE='1',THREEYD_V12_FALLBACK='1',
  THREEYD_V12_PLANE_ROOT=str(ROOT/'storage/planes'),THREEYD_V12_INDEX_ROOT=str(ROOT/'storage/indices'))
 return env

def shell(package,command):
 paths=[package/'profiles/p6/src',package/'base/src',package/'base/runtime/base/src',package/'base/runtime',package/'base/deps',ROOT/'tools',B/'tools']
 bindings=environment(package)
 explicit={k:v for k,v in bindings.items() if k.startswith(('THREEYD_','V13_')) or k in ('PYTHONDONTWRITEBYTECODE','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}
 exports='\n'.join('export '+k+'='+shlex.quote(v) for k,v in explicit.items())
 return 'set -e\nsource '+shlex.quote(str(package/'env.bash'))+'\n'+exports+'\nexport PYTHONPATH='+shlex.quote(os.pathsep.join(map(str,paths)))+':"${PYTHONPATH:-}"\nexec '+shlex.join(command)

def run_batch(package,out,mid,queries,domain):
 command=['/usr/bin/python3',str(ROOT/'tools/p6_static_harness.py'),'--map-id',mid,'--output',str(out),'--domain',str(domain),*FLAGS]
 if queries:command+=['--queries',*queries]
 out.parent.mkdir(parents=True,exist_ok=True)
 assert not out.exists() and not out.with_suffix('.log').exists(),out
 print('START',mid,queries or 'original ten queries',str(out),flush=True)
 with open('/tmp/3yd_v10_r4_bench.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  with out.with_suffix('.log').open('x') as log:
   proc=subprocess.run(['bash','-c',shell(package,command)],env=environment(package),stdout=log,stderr=subprocess.STDOUT)
 journal_path=out.with_suffix('.supervisor.json')
 assert proc.returncode==0,('HARNESS_FAILED',str(out.with_suffix('.log')),proc.returncode)
 journal=json.loads(journal_path.read_text());assert journal['returncode']==0 and journal['cleanup']['complete'],journal_path
 summary=json.loads((out/'summary.json').read_text());assert summary['runs']==(len(queries) if queries else 10) and summary['continuation_hits']==0,summary
 for name in ['P6_RUNTIME_BEFORE.json','P6_RUNTIME_AFTER.json']:assert (out/name).is_file(),name
 rows=[json.loads(l) for l in (out/'runs.jsonl').read_text().splitlines()]
 for row in rows:
  ident=row['p6_runtime_identity'];assert ident['service_class']=='three_yd_v13.service.IncrementalService' and ident['arm']=='H-I',ident
  if row['success']:assert ident['controller_class']=='three_yd_v13.incremental.IncrementalController',ident
 print('DONE',mid,summary,flush=True)
 return {'path':str(out),'map_id':mid,**summary}

def main():
 p=argparse.ArgumentParser();p.add_argument('--phase',choices=['check','pilot','formal','verify'],required=True);p.add_argument('--output',type=Path);p.add_argument('--package',type=Path,default=ROOT/'runtime/3YDL3-V13-L2');p.add_argument('--pilot-result',type=Path);p.add_argument('--domain',type=int,default=226);a=p.parse_args();package=a.package.resolve();registry=verify(package)
 if a.phase=='verify':print(json.dumps({'source_runtime_registry_verified':True,'maps':10,'queries':100}));return
 assert a.output is not None,'--output is required'
 out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
 if a.phase=='check':
  with (out/'IMPORT_CHECK.json').open('x') as f:proc=subprocess.run(['bash','-c',shell(package,['/usr/bin/python3',str(ROOT/'tools/p6_runtime_binding.py')])],env=environment(package),stdout=f,stderr=subprocess.PIPE,text=True)
  (out/'stderr.log').write_text(proc.stderr)
  assert proc.returncode==0,('IMPORT_CHECK_FAILED',str(out/'stderr.log'))
  print('CHECK_PASS',out);return
 if a.phase=='formal':
  assert a.pilot_result is not None,'--pilot-result is required for formal'
  pilot=json.loads((a.pilot_result/'COMPLETE.json').read_text());assert pilot['complete'] and all(x['success']==x['runs'] for x in pilot['batches']),'PILOT_NOT_PASSED'
 protocol={'version':'3YDL3-V13-P6','fixed_commit':'d8c01c5a71a67c2e85c250e8a8148bda9724dc68','profile':'p6','arm':'H-I','phase':a.phase,'package':str(package),'runtime_assets_sha256':sha(ROOT/'docs/RUNTIME_ASSETS.json'),'map_registry':str(B/'MAPS.json'),'map_registry_sha256':sha(B/'MAPS.json'),'queries':registry,'requested_rounds':3 if a.phase=='formal' else 1,'CPU':[14],'ROS_DOMAIN_ID':a.domain,'map_loading_startup_excluded':True,'request_fallback_ACK_certification_included':True,'no_moving_obstacle_events':True,'continuation_hits_required':0,'historical_B_results_not_same_batch_control':True,'tools_sha256':{str(p.relative_to(ROOT)):sha(p) for p in sorted((ROOT/'tools').glob('*.py'))}}
 (out/'PROTOCOL.json').write_text(json.dumps(protocol,indent=2)+'\n');done=[]
 if a.phase=='pilot':
  jobs=[('arena_small_080x100_2d_005',['A2B-01']),('mengjia',['Q03']),('pudu_wanda_3f',['Q09']),('mentor_map_20260825_005',['A2B-07'])]
  for mid,qs in jobs:done.append(run_batch(package,out/mid,mid,qs,a.domain));(out/'PROGRESS.json').write_text(json.dumps(done,indent=2)+'\n')
 else:
  maps=registry['maps']
  for rnd in range(1,4):
   order=maps[rnd-1:]+maps[:rnd-1]
   for m in order:
    done.append(run_batch(package,out/f'round{rnd}'/m['map_id']/'P6',m['map_id'],None,a.domain));(out/'PROGRESS.json').write_text(json.dumps(done,indent=2)+'\n')
 verify(package)
 (out/'COMPLETE.json').write_text(json.dumps({'complete':True,'source_and_registry_unchanged':True,'batches':done,'total_requests':sum(x['runs'] for x in done),'successes':sum(x['success'] for x in done),'all_results_retained':True},indent=2)+'\n')
 print('COMPLETE',out,flush=True)
if __name__=='__main__':main()
