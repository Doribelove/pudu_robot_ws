"""Serial recovery ablations and full fixed-map comparison. No overlapping ROS jobs."""
import json,os,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
stages=[
 ('B','free-space','arena_large_250x300_2d_005','results/diagnostic/B_A03',['--queries','A2B-03']),
 ('AB','free-space','arena_large_250x300_2d_005','results/diagnostic/AB_A03',['--queries','A2B-03']),
 ('control','free-space','arena_large_250x300_2d_005','results/control/arena_large_250x300_2d_005',[]),
 ('AB','free-space','arena_large_250x300_2d_005','results/formal/arena_large_250x300_2d_005',[]),
 ('control','semantic','mengjia','results/control/mengjia',[]),
 ('AB','semantic','mengjia','results/formal/mengjia',[]),
]
records=[]
for ablation,mode,mid,out,rest in stages:
 print('START',ablation,mode,out,flush=True)
 env=dict(os.environ,THREEYD_V8_R2_MODE=ablation,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
 started=time.time()
 result=subprocess.run(['taskset','-c','12','./run.sh','--mode',mode,'--map-id',mid,'--repeats','3','--output',out,*rest],cwd=ROOT,env=env)
 summary=ROOT/out/'summary.json'
 item={'ablation':ablation,'mode':mode,'map_id':mid,'output':out,'started':started,'finished':time.time(),'exit_code':result.returncode,'summary':json.loads(summary.read_text()) if summary.exists() else None}
 records.append(item);(ROOT/'report/evaluation_stages.json').write_text(json.dumps(records,indent=2))
 if result.returncode:raise SystemExit(result.returncode)
 rows=[json.loads(line) for line in (ROOT/out/'runs.jsonl').read_text().splitlines()]
 if not all(r['success'] for r in rows):raise SystemExit('STRICT_FAILURE: '+out)
 print('COMPLETE',ablation,mode,len(rows),'strict',flush=True)
