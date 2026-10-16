"""Fresh cases after development, plus final implementation performance run."""
import time,json,shlex,subprocess
from arena_3d_v1.semantic_world import WORK,json_write

until=time.monotonic()+1800
while True:
 p=WORK/'report/final_v2_dispatch.jsonl'
 rows=[json.loads(s) for s in p.read_text().splitlines()] if p.exists() else []
 if any(r['name']=='holdout_v2_larger_room_rot_candidate' for r in rows):break
 if time.monotonic()>until:raise RuntimeError('Previous validation batch not complete')
 time.sleep(1)
jobs=[]
for case in ['sealed_offset','sealed_near_goal']:
 for rotated in [False,True]:
  for baseline in [True,False]:
   jobs.append(('sealed_'+case+('_rot' if rotated else '')+('_r1' if baseline else '_candidate'),
    'tools/turnaround_matrix.py',['--case',case,'--repeats','3','--domain','100']+(['--rotated'] if rotated else [])+(['--baseline'] if baseline else [])))
jobs.append(('formal_candidate_v2','tools/candidate_benchmark.py',['--arm','recovery','--indices','0:20','--repeats','3','--domain','100']))
json_write(WORK/'report/sealed_jobs.json',jobs)
for name,script,opts in jobs:
 out=WORK/'results'/name
 if out.exists():raise RuntimeError(f'Output exists: {out}')
 command='source '+shlex.quote(str(WORK/'env.bash'))+'\nOPENBLAS_NUM_THREADS=1 /usr/bin/python3 '+shlex.join([script,'--output',str(out),*opts])
 started=time.monotonic();print('START',name,flush=True)
 with (WORK/'report'/f'{name}.log').open('w') as f:code=subprocess.call(['/bin/bash','-c',command],cwd=WORK,stdout=f,stderr=subprocess.STDOUT)
 row={'name':name,'returncode':code,'wall_s':time.monotonic()-started,'command':command}
 with (WORK/'report/sealed_dispatch.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
 print('FINISH',name,code,round(row['wall_s'],1),flush=True)
