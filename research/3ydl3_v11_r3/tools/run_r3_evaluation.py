"""Serial independent A/B experiments and full existing-map acceptance."""
import json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
stages=[
 ('control','free-space','arena_large_250x300_2d_005','results/control/arena_large_250x300_2d_005'),
 ('A','free-space','arena_large_250x300_2d_005','results/ablation_A/arena_large_250x300_2d_005'),
 ('B','free-space','arena_large_250x300_2d_005','results/ablation_B/arena_large_250x300_2d_005'),
 ('AB','free-space','arena_large_250x300_2d_005','results/formal/arena_large_250x300_2d_005'),
 ('control','semantic','mengjia','results/control/mengjia'),
 ('AB','semantic','mengjia','results/formal/mengjia'),
]
records=[]
for mode,branch,mid,out in stages:
 print('START',mode,branch,out,flush=True)
 env=dict(os.environ,THREEYD_V8_R3_MODE=mode,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
 started=time.time()
 result=subprocess.run(['taskset','-c','12','./run.sh','--mode',branch,'--map-id',mid,'--repeats','3','--output',out],cwd=ROOT,env=env)
 records.append({'r3_ablation':mode,'branch':branch,'map_id':mid,'output':out,'start':started,'end':time.time(),'exit_code':result.returncode})
 (ROOT/'report/evaluation_stages.json').write_text(json.dumps(records,indent=2))
 if result.returncode:raise SystemExit(result.returncode)
 rows=[json.loads(s) for s in (ROOT/out/'runs.jsonl').read_text().splitlines()]
 if len(rows)!=30 or not all(r['success'] for r in rows):raise SystemExit('STRICT_FAILURE: '+out)
 print('COMPLETE',mode,branch,'30/30 strict',flush=True)
