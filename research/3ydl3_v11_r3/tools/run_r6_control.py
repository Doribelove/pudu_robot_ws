import subprocess,json,os,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];parent=Path('/home/robot/workspaces/3yd_v8_r5_20261008');stages=[]
for branch,mid,n in [('semantic','mengjia',3),('free-space','arena_large_250x300_2d_005',3),('semantic','3yd_synth_multiloop_a_v1',1),('free-space','arena_medium_200x200_2d_005',1)]:
 out=ROOT/'results/control'/mid;print('START control',mid,flush=True);began=time.time()
 with (ROOT/'report'/('control_'+mid+'.log')).open('w') as log:
  p=subprocess.run(['taskset','-c','12',str(parent/'run.sh'),'--mode',branch,'--map-id',mid,'--repeats',str(n),'--output',str(out)],cwd=parent,env=dict(os.environ,THREEYD_V8_R4_MODE='ABC',THREEYD_V8_R5_REPAIR='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1'),stdout=log,stderr=subprocess.STDOUT)
 stages.append({'map':mid,'exit':p.returncode,'start':began,'end':time.time()});(ROOT/'report/control_stages.json').write_text(json.dumps(stages,indent=2))
 if p.returncode:raise SystemExit(p.returncode)
 rows=[json.loads(l) for l in (out/'runs.jsonl').read_text().splitlines()];print('DONE',mid,sum(x['success'] for x in rows),'/',len(rows),flush=True)
