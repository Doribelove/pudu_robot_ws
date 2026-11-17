#!/usr/bin/env python3
"""Real CLI integration, optional explicit source-built solver environment."""
import argparse,json,os,subprocess,sys,tempfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--solver-root',required=True);p.add_argument('--case',choices=['obstacle','small_budget','custom','timeout'],required=True);a=p.parse_args()
root=Path(tempfile.mkdtemp(prefix='trilayer-entry-contract-'));out=root/'out'
cmd=[sys.executable,str(Path(__file__).parents[1]/'scripts/plan'),'--demo',
     'straight' if a.case=='custom' else ('obstacle' if a.case=='timeout' else a.case),
     '--domain-id','181','--output',str(out),'--solver-root',a.solver_root]
if a.case=='custom':cmd+=['--start','3','6','0','--goal','7','6','0']
if a.case=='timeout':cmd+=['--timeout','1']
env=dict(os.environ);env['OMP_NUM_THREADS']='1';env['OPENBLAS_NUM_THREADS']='1'
r=subprocess.run(cmd,env=env,timeout=85,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
print(r.stdout);print('EVIDENCE',out)
result=json.loads((out/'result.json').read_text());execution=result['execution']
assert not execution['remaining_processes'],execution
if a.case=='timeout':
 assert r.returncode==124 and not result['success'] and result['failure_code']=='WORKER_TIMEOUT'
elif a.case=='small_budget':
 assert r.returncode==1 and not result['success'] and result['failure_code']=='L3_SPATIAL_BUDGET'
 assert result['worker_exit']==0 and result['cleanup_errors']==[]
else:
 assert r.returncode==0 and result['success'],result
 assert result['worker_exit']==0 and result['cleanup_errors']==[]
 expected=((3.,6.,0.),(7.,6.,0.)) if a.case=='custom' else ((2.,4.,0.),(5.,4.,0.))
 assert tuple(result['path'][0])==expected[0] and tuple(result['path'][-1])==expected[1]
 if a.case=='obstacle':
  assert result['result']['curve_l3']['iterations']
  assert all(i['qp_solver']=='clarabel' and i['status']=='Solved' for i in result['result']['curve_l3']['iterations'])
 assert result['result']['independent_audit']['valid']
print('PASS actual CLI',a.case)
