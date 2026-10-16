import os,subprocess,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];stages=[];log=[]
for mode in ('A','AB'):
 for branch,mid in [('semantic','mengjia'),('free-space','arena_large_250x300_2d_005')]:stages.append((mode,branch,mid,1,'ablation/'+mode))
for branch,mid,n in [('semantic','mengjia',3),('free-space','arena_large_250x300_2d_005',3),('semantic','3yd_synth_multiloop_a_v1',1),('free-space','arena_medium_200x200_2d_005',1)]:stages.append(('ABC',branch,mid,n,'formal' if n==3 else 'additional'))
for mode,branch,mid,n,prefix in stages:
 out=ROOT/'results'/prefix/mid;started=time.time();print('START',mode,mid,n,flush=True)
 env=dict(os.environ,THREEYD_V8_R6_MODE=mode,THREEYD_R6_INVENTORY='0',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
 with (ROOT/'report'/f'run_{mode}_{mid}.log').open('w') as stream:
  r=subprocess.run(['taskset','-c','12','./run.sh','--mode',branch,'--map-id',mid,'--repeats',str(n),'--output',str(out)],cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
 log.append({'mode':mode,'map_id':mid,'repeats':n,'output':str(out),'start':started,'end':time.time(),'exit':r.returncode});(ROOT/'report/evaluation_stages.json').write_text(json.dumps(log,indent=2))
 if r.returncode:raise SystemExit(r.returncode)
 rows=[json.loads(s) for s in (out/'runs.jsonl').read_text().splitlines()];print('DONE',mode,mid,sum(x['success'] for x in rows),'/',len(rows),flush=True)
 if len(rows)!=10*n or (mid!='arena_medium_200x200_2d_005' and not all(x['success'] for x in rows)):raise SystemExit('SUCCESS_GATE_FAILED')
