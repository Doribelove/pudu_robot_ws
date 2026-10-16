"""Validate bounded repair of large post-startup mask gaps; preserve v2 failures."""
import json,time,subprocess,shlex,hashlib
from arena_3d_v1.semantic_world import WORK,json_write
jobs=[('ack_fault_injection_v3','tools/ack_fault_probe.py',['--domain','101']),
 ('startup_v3_small','tools/startup_stress.py',['--arm','stable','--cycles','90','--domain','101','--abrupt-every','10']),
 ('startup_v3_real','tools/startup_stress.py',['--arm','stable','--cycles','10','--domain','101','--real','--abrupt-every','5']),
 ('final_calibration_v3','tools/candidate_benchmark.py',['--arm','recovery','--scene','calibration','--repeats','3','--domain','101']),
 ('final_v3_return_bulb','tools/candidate_benchmark.py',['--arm','recovery','--scene','return_bulb','--repeats','3','--domain','101']),
 ('formal_candidate_v3','tools/candidate_benchmark.py',['--arm','recovery','--indices','0:20','--repeats','3','--domain','101'])]
json_write(WORK/'report/final_v3_jobs.json',jobs)
for name,script,opts in jobs:
 out=WORK/'results'/name
 if out.exists():raise RuntimeError(f'Output exists: {out}')
 command='source '+shlex.quote(str(WORK/'env.bash'))+'\nOPENBLAS_NUM_THREADS=1 /usr/bin/python3 '+shlex.join([script,'--output',str(out),*opts])
 started=time.monotonic();print('START',name,flush=True)
 with (WORK/'report'/f'{name}.log').open('w') as f:code=subprocess.call(['/bin/bash','-c',command],cwd=WORK,stdout=f,stderr=subprocess.STDOUT)
 record={'name':name,'returncode':code,'wall_s':time.monotonic()-started,'command':command}
 with (WORK/'report/final_v3_dispatch.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
 print('FINISH',name,code,round(record['wall_s'],1),flush=True)
