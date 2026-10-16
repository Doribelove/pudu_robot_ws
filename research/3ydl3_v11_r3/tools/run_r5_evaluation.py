import os,subprocess,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
log=[]
stages=[('diagnostic','free-space','arena_medium_200x200_2d_005',1,['--queries','A2B-06','A2B-08'])]
for branch,mid in [('semantic','mengjia'),('free-space','arena_large_250x300_2d_005'),('semantic','3yd_synth_multiloop_a_v1'),('free-space','arena_medium_200x200_2d_005')]:
    stages.append(('formal' if mid in ('mengjia','arena_large_250x300_2d_005') else 'additional',branch,mid,3 if mid in ('mengjia','arena_large_250x300_2d_005') else 1,[]))
for stage,branch,mid,repeats,extra in stages:
    out=f'results/{stage}/{mid}';started=time.time()
    print('START r5',stage,mid,flush=True)
    env=dict(os.environ,THREEYD_V8_R4_MODE='ABC',THREEYD_V8_R5_REPAIR='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    with (ROOT/'report'/f'run_{stage}_{mid}.log').open('w') as stream:
        result=subprocess.run(['taskset','-c','12','./run.sh','--mode',branch,'--map-id',mid,'--repeats',str(repeats),'--output',out,*extra],cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
    log.append({'stage':stage,'branch':branch,'map_id':mid,'repeats':repeats,'output':out,'start':started,'end':time.time(),'exit_code':result.returncode})
    (ROOT/'report/evaluation_stages.json').write_text(json.dumps(log,indent=2))
    if result.returncode:raise SystemExit(result.returncode)
    rows=[json.loads(s) for s in (ROOT/out/'runs.jsonl').read_text().splitlines()]
    print('DONE r5',stage,mid,sum(x['success'] for x in rows),'/',len(rows),flush=True)
    expected=2 if stage=='diagnostic' else 10*repeats
    if len(rows)!=expected:raise SystemExit('INCOMPLETE_FIXED_SET')
    if stage in ('diagnostic','formal') and not all(x['success'] for x in rows):raise SystemExit('CASE_REPAIR_OR_FORMAL_FAILED:'+out)
