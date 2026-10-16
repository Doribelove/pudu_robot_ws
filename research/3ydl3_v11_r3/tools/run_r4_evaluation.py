"""Fresh-process serial r3-control and independent storage/search ablations."""
import os,json,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
stages=[]
for mode in ('control','A','B','C','ABC'):
    for branch,mid in [('free-space','arena_large_250x300_2d_005'),('semantic','mengjia')]:
        repeats=3 if mode in ('control','ABC') else 1
        stages.append((mode,branch,mid,repeats,f'results/{"formal" if mode=="ABC" else mode}/{mid}'))
for mode in ('control','ABC'):
    for branch,mid in [('semantic','3yd_synth_multiloop_a_v1'),('free-space','arena_medium_200x200_2d_005')]:
        stages.append((mode,branch,mid,1,f'results/additional/{mode}/{mid}'))
log=[]
for mode,branch,mid,repeats,out in stages:
    print('START',mode,mid,repeats,flush=True)
    started=time.time();env=dict(os.environ,THREEYD_V8_R4_MODE=mode,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    logfile=ROOT/'report'/('run_'+mode+'_'+mid+'.log')
    with logfile.open('w') as stream:
        result=subprocess.run(['taskset','-c','12','./run.sh','--mode',branch,'--map-id',mid,'--repeats',str(repeats),'--output',out],cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
    log.append({'mode':mode,'branch':branch,'map_id':mid,'repeats':repeats,'output':out,'start':started,'end':time.time(),'exit_code':result.returncode})
    (ROOT/'report/evaluation_stages.json').write_text(json.dumps(log,indent=2))
    if result.returncode:raise SystemExit(result.returncode)
    rows=[json.loads(s) for s in (ROOT/out/'runs.jsonl').read_text().splitlines()]
    if len(rows)!=10*repeats:raise SystemExit('INCOMPLETE:'+out)
    print('DONE',mode,mid,sum(x['success'] for x in rows),'/',len(rows),flush=True)
    if mid in ('mengjia','arena_large_250x300_2d_005') and not all(x['success'] for x in rows):
        raise SystemExit('REGRESSION:'+out)
