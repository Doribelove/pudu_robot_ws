"""Serial, interleaved fresh processes using the actual V7 and V8 workspaces."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,time
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path(json.loads((ROOT/'PARENT.json').read_text())['parent'])
ROOTS={'v7':PARENT,'v8':ROOT}
MAPS=('mengjia','arena_large_250x300_2d_005')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,d):
 p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def hashes(root):
 files=[p for folder in ('src','tools','config','native','install','inputs','datasets','deps') for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)]
 files.extend(root/p for p in ('env.bash','run.sh','BASELINE.json','L1_LOCK.json'))
 return {str(p.relative_to(root)):sha(p) for p in files}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
 ap.add_argument('--rounds',type=int,default=3);ap.add_argument('--versions',nargs='+',choices=ROOTS,default=['v7','v8'])
 ap.add_argument('--queries',nargs='+');ap.add_argument('--domain',type=int,default=230);ap.add_argument('--resume',action='store_true')
 a=ap.parse_args();assert set(a.versions)==set(ROOTS) and len(a.versions)==2 and 0<=a.domain<=232
 out=a.output.resolve()
 if not a.resume:
  out.mkdir(parents=True,exist_ok=False)
  protocol={'schema':'3YDL3-V8-direct-parent-interleaved-v1','rounds':a.rounds,'versions':a.versions,
    'maps':MAPS,'roots':{k:str(v) for k,v in ROOTS.items()},'requested_queries':a.queries,'domain':a.domain,
    'source_hashes':{k:hashes(root) for k,root in ROOTS.items()},'CPU':[14],
    'first':'one query per fresh process; full request wall, CPU and process tree recorded',
    'repeat':'two immediately after strict first only in round0; failure costs and unformed requests retained',
    'order':'round/query/map nested, versions rotated by round+query+map',
    'os_cache':'not flushed; fresh process is not cold OS','diagnostic_subset':bool(a.queries),
    'l3_backend':'independent for BOTH versions; historical V9 control is not used'}
  save(out/'protocol.json',protocol)
  for version,root in ROOTS.items():
   for folder in ('src','tools','config','native'):
    shutil.copytree(root/folder,out/'source'/version/folder,ignore=shutil.ignore_patterns('__pycache__'))
 protocol=json.loads((out/'protocol.json').read_text())
 for key in ('rounds','versions','domain'):assert protocol[key]==getattr(a,key)
 assert protocol['requested_queries']==a.queries
 for version,expected in protocol['source_hashes'].items():
  assert hashes(ROOTS[version])==expected,('SOURCE_CHANGED',version)
 from run_free_full import registered_input
 queries={'mengjia':[q['id'] for q in json.loads((ROOT/'inputs/queries.json').read_text())['scenarios']]}
 queries[MAPS[1]]=[q['id'] for q in registered_input(MAPS[1])[2]]
 save(out/'query_ids.json',queries)
 for round_index in range(a.rounds):
  for index in range(10):
   for im,mid in enumerate(MAPS):
    q=queries[mid][index]
    if a.queries and q not in a.queries:continue
    shift=(round_index+index+im)%2;order=a.versions[shift:]+a.versions[:shift]
    for version in order:
     batch=out/f'round{round_index+1}'/mid/q/version;meta=batch.parent/(version+'.launcher.json')
     if meta.exists():
      old=json.loads(meta.read_text())
      if old.get('completed') and old.get('returncode')==0 and (batch/'summary.json').is_file():continue
      raise RuntimeError('INCOMPLETE_BATCH_REQUIRES_INSPECTION:'+str(batch))
     command=[str(ROOTS[version]/'run.sh'),'--map-id',mid,'--queries',q,'--l3-backend','independent',
       '--repeats','3' if round_index==0 else '1','--skip-repeat-after-failure','--cold-memory',
       '--domain',str(a.domain),'--output',str(batch)]
     env=dict(os.environ);env['THREEYD_BENCH_CPU']='14';tick=time.monotonic();env['THREEYD_L3_PROCESS_STARTED']=str(tick)
     save(meta,{'command':command,'version':version,'started_monotonic':tick,'completed':False})
     with (batch.parent/(version+'.log')).open('w') as log:
      done=subprocess.run(command,cwd=ROOTS[version],env=env,stdout=log,stderr=subprocess.STDOUT)
     record=json.loads(meta.read_text());record.update(completed=True,returncode=done.returncode,process_wall_s=time.monotonic()-tick)
     if (batch/'runs.jsonl').exists():
      rows=[json.loads(v) for v in (batch/'runs.jsonl').read_text().splitlines()]
      record.update(runs=len(rows),strict=sum(r['success'] for r in rows))
     save(meta,record);print(round_index+1,mid,q,version,record.get('strict'),round(record['process_wall_s'],1),flush=True)
     assert done.returncode==0,('BATCH_FAILED',str(batch))
 for version,expected in protocol['source_hashes'].items():assert hashes(ROOTS[version])==expected,version
 save(out/'COMPLETE.json',{'complete':True,'finished_monotonic':time.monotonic()})
if __name__=='__main__':main()
