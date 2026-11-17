#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Bounded owner for the real worker + real planner_server contract."""
import os,sys,tempfile,subprocess,time,shutil,json
from pathlib import Path
from trilayer_planner_service.process import run_bounded
root=Path(tempfile.mkdtemp(prefix='trilayer-nav2-contract-'));out=root/'worker'
env=dict(os.environ,ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='185',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1')
log=open(root/'owner.log','w')
worker=subprocess.Popen([sys.executable,'-m','trilayer_planner_service.cli','--output',str(out),'--domain-id','187','--solver-root',sys.argv[1],'--lifetime','150'],env=env,stdout=log,stderr=log)
try:
 deadline=time.monotonic()+15
 while not (out/'ready.json').exists():
  if worker.poll() is not None:raise RuntimeError('worker exited before ready: '+str(root))
  if time.monotonic()>deadline:raise TimeoutError('worker readiness timeout')
  time.sleep(.05)
 host_execution=run_bounded([sys.executable,str(Path(__file__).with_name('planner_server_smoke.py').resolve()),str(out/'service.sock')],root/'host-test.log',timeout=120,grace=5.,env=env,cwd=root)
 (root/'host-execution.json').write_text(json.dumps(host_execution,indent=2)+'\n')
 print((root/'host-test.log').read_text(),flush=True)
 assert host_execution['returncode']==0 and not host_execution['timed_out'] and not host_execution['interrupted'] and not host_execution['remaining_processes'],host_execution
 requests=[json.loads(p.read_text()) for p in sorted(out.glob('request-*.json'))]
 assert len(requests)==4 and all(r['success'] for r in requests)
 assert requests[0]['native_pid']==requests[1]['native_pid']!=requests[2]['native_pid']
 print('PASS same-snapshot PID reuse, changed-snapshot cold PID rebuild; evidence',root,flush=True)
finally:
 if worker.poll() is None:worker.terminate()
 try:worker.wait(timeout=15)
 except subprocess.TimeoutExpired:worker.kill();worker.wait(timeout=5);raise RuntimeError('owner did not cleanly stop')
 log.close()
 if (out/'execution.json').exists():
  execution=json.loads((out/'execution.json').read_text())
  assert not execution['remaining_processes'],execution
