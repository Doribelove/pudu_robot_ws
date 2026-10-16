"""Predefined diagnostics, stress tests and ABBA fixed-query comparison."""
import json,os,subprocess,sys,time,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def save(p,d):p.write_text(json.dumps(d,indent=2)+'\n')
jobs=[]
for pub in ['observed_baseline','paced','repair']:
 jobs.append(('ablation_'+pub,'semantic_benchmark.py',['--publication',pub,'--arm','recovery','--indices','0:8','--repeats','1','--domain','104']))
for fault in ['drop_tile','stale_tile','corrupt_reads','stale_timestamp','sequence_change']:
 for pub in (['baseline','combined'] if fault in ['drop_tile','stale_tile'] else ['combined']):
  jobs.append(('fault_'+fault+'_'+pub,'semantic_stress.py',['--fault',fault,'--updates','4','--publication',pub,'--domain','104']))
for scene,updates in [('calibration',40),('rotated',40),('real',20)]:
 jobs.append(('stress_'+scene,'semantic_stress.py',['--scene',scene,'--updates',str(updates),'--domain','104']))
jobs.append(('resource_cycles','lifecycle_matrix.py',['--domain','104']))
for i,pub in enumerate(['baseline','combined','combined','baseline']):
 jobs.append((f'formal_{i+1}_{pub}','semantic_benchmark.py',['--publication',pub,'--arm','recovery','--indices','0:20','--repeats','3','--domain','104']))
for scene in ['calibration','rotated','return_bulb','return_bulb_rotated','narrow','two_routes_optimistic_edge']:
 jobs.append(('control_'+scene,'semantic_benchmark.py',['--arm','recovery','--scene',scene,'--repeats','3','--domain','104']))
save(ROOT/'report/VALIDATION_JOBS.json',jobs)
sources=[*ROOT.glob('external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/*.py'),*ROOT.glob('tools/*.py'),*ROOT.glob('config/*'),ROOT/'env.bash',ROOT/'build/libsemantic_sweep.so']
save(ROOT/'report/IMPLEMENTATION_FREEZE.json',{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.is_file()})
for name,script,args in jobs:
 output=ROOT/'results'/name
 if output.exists():raise FileExistsError(output)
 command=[sys.executable,str(ROOT/'tools'/script),'--output',str(output),*args]
 print('START',name,flush=True);start=time.monotonic()
 with (ROOT/'report'/f'{name}.log').open('w') as f:code=subprocess.call(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 result={'name':name,'command':command,'returncode':code,'elapsed_s':time.monotonic()-start}
 with (ROOT/'report/dispatch.jsonl').open('a') as f:f.write(json.dumps(result)+'\n')
 print('FINISH',name,code,round(result['elapsed_s'],1),flush=True)
 if code!=0:raise RuntimeError(f'Validation subprocess failed: {name}; investigate without overwriting it')
