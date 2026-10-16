"""Serial, recorded validation. Failed jobs are retained and never retried."""
import argparse,subprocess,sys,time,json,shlex,hashlib
from pathlib import Path
from arena_3d_v1.semantic_world import WORK,json_write

def main():
 p=argparse.ArgumentParser();p.add_argument('--phase',choices=['startup','planning'],required=True);args=p.parse_args()
 jobs=[]
 if args.phase=='startup':
  jobs=[('startup_final_small','tools/startup_stress.py',['--arm','stable','--cycles','90','--domain','97','--abrupt-every','10']),
        ('startup_final_real','tools/startup_stress.py',['--arm','stable','--cycles','10','--domain','97','--real','--abrupt-every','5'])]
 else:
  jobs.append(('ack_fault_injection','tools/ack_fault_probe.py',['--domain','98']))
  for scene in ['return_bulb','return_bulb_rotated','calibration','rotated','narrow','two_routes','two_routes_optimistic_edge']:
   jobs.append(('final_'+scene,'tools/candidate_benchmark.py',['--arm','recovery','--scene',scene,'--repeats','3','--domain','98']))
  for case in ['shifted','yaw','smaller_room','larger_room']:
   for rotated in [False,True]:
    for baseline in [True,False]:
     name='holdout_'+case+('_rot' if rotated else '')+('_r1' if baseline else '_candidate')
     opts=['--case',case,'--repeats','3','--domain','98']+(['--rotated'] if rotated else [])+(['--baseline'] if baseline else [])
     jobs.append((name,'tools/turnaround_matrix.py',opts))
  jobs.append(('formal_r1','/home/robot/workspaces/3yd_v1_static_recovery_20260914/tools/static_benchmark.py',['--arm','recovery','--indices','0:20','--repeats','3','--domain','98']))
  jobs.append(('formal_candidate','tools/candidate_benchmark.py',['--arm','recovery','--indices','0:20','--repeats','3','--domain','98']))
 json_write(WORK/'report'/f'{args.phase}_jobs.json',jobs)
 for name,script,opts in jobs:
  out=WORK/'results'/name;log=WORK/'report'/f'{name}.log'
  if out.exists():raise RuntimeError(f'Output already exists: {out}')
  env=Path(script).parents[1]/'env.bash' if Path(script).is_absolute() else WORK/'env.bash'
  command='source '+shlex.quote(str(env))+'\nOPENBLAS_NUM_THREADS=1 /usr/bin/python3 '+shlex.join([str(script),'--output',str(out),*opts])
  t=time.monotonic();print('START',name,flush=True)
  with log.open('w') as stream:
   code=subprocess.call(['/bin/bash','-c',command],cwd=WORK,stdout=stream,stderr=subprocess.STDOUT)
  row={'name':name,'returncode':code,'wall_s':time.monotonic()-t,'command':command,'log':str(log)}
  with (WORK/'report'/f'{args.phase}_dispatch.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
  print('FINISH',name,code,round(row['wall_s'],1),flush=True)

if __name__=='__main__':main()
