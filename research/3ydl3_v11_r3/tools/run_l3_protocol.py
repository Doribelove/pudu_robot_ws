"""New-process interleaved fixed-query protocol; never mixes historical repeats."""
import argparse,hashlib,json,os,subprocess,sys,time,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MAPS=('mengjia','arena_large_250x300_2d_005')

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')
def source_files():return [p for folder in ('src','tools','config','native','install','inputs') for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)]+[ROOT/'env.bash',ROOT/'run.sh',ROOT/'BASELINE.json']

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--rounds',type=int,default=3)
    ap.add_argument('--modes',nargs='+',default=['control','independent','compatibility']);ap.add_argument('--queries',nargs='+');ap.add_argument('--resume',action='store_true');ap.add_argument('--domain',type=int,default=231)
    a=ap.parse_args()
    if not 0<=a.domain<=232:ap.error('ROS domain must be within 0..232 for the installed DDS transport')
    out=a.output.resolve()
    if a.resume:assert (out/'protocol.json').is_file()
    else:
        out.mkdir(parents=True,exist_ok=False)
        protocol={'schema':'3YDL3-V8-new-process-interleaved-v1','rounds':a.rounds,'modes':a.modes,'maps':MAPS,
          'source_hashes':{str(p.relative_to(ROOT)):sha(p) for p in source_files()},'config':json.loads((ROOT/'config/l3.json').read_text()),
          'independent_first_per_mode':60 if not a.queries and a.rounds==3 else None,'planned_repeats_per_mode':40 if not a.queries else None,
          'first_sample_is_new_process':True,'repeats':'only round0 immediately after its first strict success, two per query; failed first gets two explicit unformed records',
          'cpu':[14],'os_cache':'not flushed; cold process with OS filesystem/page cache uncontrolled',
          'diagnostic_subset':bool(a.queries) or a.rounds!=3,'order':'round/query/map nested; mode order rotated by round+query+map',
          'requested_queries':a.queries,'domain':a.domain,'created_monotonic':time.monotonic()}
        save(out/'protocol.json',protocol)
        for folder in ('src','tools','config'):
            shutil.copytree(ROOT/folder,out/'source'/folder,ignore=shutil.ignore_patterns('__pycache__'))
    protocol=json.loads((out/'protocol.json').read_text());assert protocol['modes']==a.modes and protocol['rounds']==a.rounds and protocol['requested_queries']==a.queries and protocol['domain']==a.domain
    for name,expected in protocol['source_hashes'].items():assert sha(ROOT/name)==expected,('SOURCE_CHANGED_AFTER_FREEZE',name)
    from run_free_full import registered_input
    qs={'mengjia':[x['id'] for x in json.loads((ROOT/'inputs/queries.json').read_text())['scenarios']]}
    qs[MAPS[1]]=[x['id'] for x in registered_input(MAPS[1])[2]]
    assert all(len(v)==10 and len(set(v))==10 for v in qs.values())
    save(out/'query_ids.json',qs)
    for rep in range(a.rounds):
        for index in range(10):
            for im,mid in enumerate(MAPS):
                q=qs[mid][index]
                if a.queries and q not in a.queries:continue
                shift=(rep+index+im)%len(a.modes);order=a.modes[shift:]+a.modes[:shift]
                for mode in order:
                    batch=out/f'round{rep+1}'/mid/q/mode;meta=batch.parent/(mode+'.launcher.json')
                    if meta.exists():
                        previous=json.loads(meta.read_text())
                        if previous.get('completed') and (batch/'summary.json').is_file():continue
                        raise RuntimeError('INCOMPLETE_BATCH_REQUIRES_EXPLICIT_INSPECTION:'+str(batch))
                    batch.parent.mkdir(parents=True,exist_ok=True)
                    command=[str(ROOT/'run.sh'),'--map-id',mid,'--queries',q,'--l3-backend',mode,'--repeats','3' if rep==0 else '1',
                             '--skip-repeat-after-failure','--cold-memory','--domain',str(a.domain),'--output',str(batch)]
                    env=dict(os.environ);env['THREEYD_BENCH_CPU']='14';started=time.monotonic();env['THREEYD_L3_PROCESS_STARTED']=str(started)
                    save(meta,{'command':command,'started_monotonic':started,'completed':False,'round':rep+1,'mode':mode,'query_id':q,'map_id':mid})
                    with (batch.parent/(mode+'.log')).open('w') as log:
                        done=subprocess.run(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
                    record=json.loads(meta.read_text());record.update(returncode=done.returncode,process_wall_ms=(time.monotonic()-started)*1000,completed=True)
                    if (batch/'runs.jsonl').exists():
                        rows=[json.loads(v) for v in (batch/'runs.jsonl').read_text().splitlines()]
                        record['runs']=len(rows);record['strict']=sum(r['success'] for r in rows);record['first_success']=bool(rows and rows[0]['success'])
                    save(meta,record);print(rep+1,mid,q,mode,record.get('strict'),round(record['process_wall_ms']/1000,1),flush=True)
                    if done.returncode:raise RuntimeError('BATCH_PROCESS_FAILURE:'+str(batch))
                    for name in ('config/l3.json','src/three_yd_v5/curve_l3.py','src/three_yd_v5/curve_backend.py'):
                        assert sha(ROOT/name)==protocol['source_hashes'][name],name
    save(out/'COMPLETE.json',{'complete':True,'finished_monotonic':time.monotonic()})
if __name__=='__main__':main()
