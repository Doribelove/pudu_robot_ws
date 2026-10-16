"""Validate the additional cold-publication repair; preserve earlier failure."""
import json,time,subprocess,shlex
from pathlib import Path
from arena_3d_v1.semantic_world import WORK,json_write

def rows(path):
 return [json.loads(s) for s in path.read_text().splitlines()] if path.exists() else []

until=time.monotonic()+900
while not any(r['name']=='formal_candidate' for r in rows(WORK/'report/planning_dispatch.jsonl')):
 if time.monotonic()>until:raise RuntimeError('Previous formal batch did not finish')
 time.sleep(1)
jobs=[('final_calibration_v2','tools/candidate_benchmark.py',['--arm','recovery','--scene','calibration','--repeats','3','--domain','99']),
      ('ack_fault_injection_v2','tools/ack_fault_probe.py',['--domain','99']),
      ('startup_v2_small','tools/startup_stress.py',['--arm','stable','--cycles','90','--domain','99','--abrupt-every','10']),
      ('startup_v2_real','tools/startup_stress.py',['--arm','stable','--cycles','10','--domain','99','--real','--abrupt-every','5'])]
for scene in ['return_bulb','return_bulb_rotated','rotated','narrow','two_routes','two_routes_optimistic_edge']:
 jobs.append(('final_v2_'+scene,'tools/candidate_benchmark.py',['--arm','recovery','--scene',scene,'--repeats','3','--domain','99']))
for case in ['shifted','yaw','smaller_room','larger_room']:
 for rotated in [False,True]:
  jobs.append(('holdout_v2_'+case+('_rot' if rotated else '')+'_candidate','tools/turnaround_matrix.py',
   ['--case',case,'--repeats','3','--domain','99']+(['--rotated'] if rotated else [])))
json_write(WORK/'report/final_v2_jobs.json',jobs)
for name,script,opts in jobs:
 out=WORK/'results'/name
 if out.exists():raise RuntimeError(f'Output exists: {out}')
 command='source '+shlex.quote(str(WORK/'env.bash'))+'\nOPENBLAS_NUM_THREADS=1 /usr/bin/python3 '+shlex.join([script,'--output',str(out),*opts])
 started=time.monotonic();print('START',name,flush=True)
 with (WORK/'report'/f'{name}.log').open('w') as f:
  code=subprocess.call(['/bin/bash','-c',command],cwd=WORK,stdout=f,stderr=subprocess.STDOUT)
 record={'name':name,'returncode':code,'wall_s':time.monotonic()-started,'command':command}
 with (WORK/'report/final_v2_dispatch.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
 print('FINISH',name,code,round(record['wall_s'],1),flush=True)
