"""Frozen final lock + pacing + bounded repair verification and ABBA comparison."""
import json,os,subprocess,sys,time,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def save(p,d):p.write_text(json.dumps(d,indent=2)+'\n')
jobs=[]
# First verify loss/staleness recovery and fail-closed behavior.
for fault in ['drop_tile','stale_tile','corrupt_reads','stale_timestamp','sequence_change']:
 jobs.append(('final_fault_'+fault,'semantic_stress.py',['--snapshot','locked','--publication','combined','--fault',fault,'--updates','4','--domain','108']))
for scene in ['calibration','rotated']:
 jobs.append(('final_stress_'+scene,'semantic_stress.py',['--snapshot','locked','--publication','combined','--scene',scene,'--updates','40','--domain','108']))
# All four complete batches, no fastest-run selection.
for i,(pub,snapshot) in enumerate([('baseline','system'),('combined','locked'),('combined','locked'),('baseline','system')]):
 jobs.append((f'final_{i+1}_{pub}','semantic_benchmark.py',['--snapshot',snapshot,'--publication',pub,'--arm','recovery','--indices','0:20','--repeats','3','--domain','108']))
for scene in ['calibration','rotated','return_bulb','return_bulb_rotated','narrow','two_routes_optimistic_edge']:
 jobs.append(('final_control_'+scene,'semantic_benchmark.py',['--snapshot','locked','--publication','combined','--arm','recovery','--scene',scene,'--repeats','3','--domain','108']))
jobs.append(('final_resource_cycles','lifecycle_matrix.py',['--snapshot','locked','--domain','108']))
save(ROOT/'report/FINAL_VALIDATION_JOBS.json',jobs)
sources=[*ROOT.glob('external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/*.py'),*ROOT.glob('tools/*.py'),*ROOT.glob('config/*'),ROOT/'env.bash',ROOT/'build/libsemantic_sweep.so',*ROOT.glob('install/nav2_snapshot_local/lib/*.so'),*ROOT.glob('external/nav2_snapshot_local/nav2_costmap_2d/src/*.cpp')]
save(ROOT/'report/FINAL_IMPLEMENTATION_FREEZE.json',{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.is_file()})
for name,script,args in jobs:
 output=ROOT/'results'/name
 if output.exists():raise FileExistsError(output)
 command=[sys.executable,str(ROOT/'tools'/script),'--output',str(output),*args]
 print('START',name,flush=True);start=time.monotonic()
 with (ROOT/'report'/f'{name}.log').open('w') as f:code=subprocess.call(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
 summary=json.loads((output/'summary.json').read_text()) if (output/'summary.json').exists() else {}
 expected_negative=any(x in name for x in ['corrupt_reads','stale_timestamp','sequence_change'])
 if expected_negative:
  passed=summary.get('success')==1 and summary.get('executed')==2 and len(summary.get('failures',[]))==1
 elif script=='semantic_stress.py':
  passed=summary.get('success')==summary.get('requested_updates') and summary.get('success',0)>0
  if 'fault_' in name:
   rows=[json.loads(l) for l in (output/'updates.jsonl').read_text().splitlines()];passed=passed and rows[-1]['injected_tiles']==1
 elif script=='lifecycle_matrix.py':passed=summary.get('cycles')==summary.get('success')==12
 else:passed=summary.get('success')==summary.get('count') and summary.get('success',0)>0
 result={'name':name,'command':command,'returncode':code,'elapsed_s':time.monotonic()-start,'summary':summary,'assertions_passed':passed,'expected_negative':expected_negative}
 with (ROOT/'report/final_dispatch.jsonl').open('a') as f:f.write(json.dumps(result)+'\n')
 print('FINISH',name,code,passed,round(result['elapsed_s'],1),flush=True)
 if code!=0 or not passed:raise RuntimeError(f'Validation failed: {name}; preserve results and investigate')
