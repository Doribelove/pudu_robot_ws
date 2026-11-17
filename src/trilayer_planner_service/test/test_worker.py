#!/usr/bin/env python3
"""Real socket/P6 integration. Domain186 only; no robot/controller."""
import base64,hashlib,json,os,socket,subprocess,sys,tempfile,time
from pathlib import Path
import numpy as np
from trilayer_planner_service.protocol import encode,read_line,MAX_RESPONSE
root=Path(tempfile.mkdtemp(prefix='tri-svc-test-'));solver=Path(sys.argv[1]).resolve()
def launch(name,timeout=55,lifetime=65,min_free=512):
 out=root/name
 process=subprocess.Popen([sys.executable,'-m','trilayer_planner_service.cli','--output',str(out),'--domain-id','186','--solver-root',str(solver),'--request-timeout',str(timeout),'--lifetime',str(lifetime),'--min-free-mib',str(min_free)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
 for _ in range(200):
  if (out/'ready.json').exists():return out,process
  if process.poll() is not None:raise RuntimeError(process.communicate()[0].decode())
  time.sleep(.025)
 raise RuntimeError('worker did not start')
def call(out,req,disconnect=False):
 with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
  s.settimeout(60);s.connect(str(out/'service.sock'));s.sendall(encode(req))
  if disconnect:return
  return read_line(s,MAX_RESPONSE)
def plan(i,obstacle=False):
 costs=np.zeros((240,240),np.uint8);costs[:4,:]=254;costs[-4:,:]=254;costs[:,:4]=254;costs[:,-4:]=254
 if obstacle:costs[65:96,65:76]=254
 raw=costs.tobytes();hard=(costs>=253).astype(np.uint8).tobytes()
 c={'frame_id':'map','width':240,'height':240,'resolution':.05,'origin':[0.,0.],'costmap_sha256':hashlib.sha256(raw).hexdigest(),'hard_mask_sha256':hashlib.sha256(hard).hexdigest()}
 return {'v':1,'id':i,'op':'plan','start':[2.,4.,0.],'goal':[5.,4.,0.],'context':c,'snapshot':base64.b64encode(raw).decode()}
outputs=[];process=None
try:
 out,process=launch('normal',lifetime=0)
 time.sleep(1.2)
 assert process.poll() is None
 assert (out.stat().st_mode & 0o777)==0o700 and ((out/'service.sock').stat().st_mode & 0o777)==0o600
 for i in range(2):
  r=call(out,plan(str(i)));outputs.append(r);assert r['ok'] and r['audit_valid'],r
 assert outputs[0]['native_pid']==outputs[1]['native_pid'] and outputs[1]['session_reused']
 r=call(out,{'v':1,'id':'bad','op':'plan','unexpected':True});assert not r['ok'] and not r['path']
 r=call(out,plan('changed',True));outputs.append(r);assert r['ok'] and r['session_generation']==2 and not r['session_reused'],r
 assert r['native_pid']!=outputs[0]['native_pid']
 # Actual plan disconnect: response cannot be consumed, subsequent status sees intact owner.
 call(out,plan('disconnected',True),True)
 status=call(out,{'v':1,'id':'status','op':'status'});assert status['active'] and status['session_generation']==2,status
 r=call(out,{'v':1,'id':'close','op':'close'});assert r['ok'],r
 assert process.wait(timeout=15)==0
 execution=json.loads((out/'execution.json').read_text());assert not execution['remaining_processes'],execution
 out,process=launch('signal',lifetime=0)
 r=call(out,plan('signalplan'));assert r['ok']
 process.terminate();assert process.wait(timeout=20)==0
 signal_execution=json.loads((out/'execution.json').read_text())
 assert signal_execution['interrupted'] and not signal_execution['remaining_processes']
 assert not (out/'service.sock').exists()
 assert json.loads((out/'closed.json').read_text())['shutdown_reason']=='SIGNAL'
 out,process=launch('timeout',1,lifetime=0)
 try:call(out,plan('deadline'))
 except (OSError,ValueError):pass
 assert process.wait(timeout=20)==124
 execution=json.loads((out/'execution.json').read_text());assert execution['timed_out'] and not execution['remaining_processes'],execution
 out,process=launch('lowspace',lifetime=0,min_free=10**12)
 status=call(out,{'v':1,'id':'space-status','op':'status'});assert status['ok']
 lowspace=call(out,plan('lowspace'))
 assert lowspace['failure_code']=='DISK_SPACE_LOW' and not lowspace['ok'] and not lowspace['path'],lowspace
 assert process.wait(timeout=15)==1
 lowspace_execution=json.loads((out/'execution.json').read_text())
 assert not lowspace_execution['remaining_processes'] and not (out/'service.sock').exists()
 assert not list(out.glob('generation-*'))
 (root/'summary.json').write_text(json.dumps({'plans':outputs,'signal_execution':signal_execution,'timeout_execution':execution,'persistent_idle_seconds':1.2,'lowspace_response':lowspace,'lowspace_execution':lowspace_execution},indent=2))
 print('PASS',root)
finally:
 if process is not None and process.poll() is None:
  process.terminate()
  try:process.wait(timeout=20)
  except subprocess.TimeoutExpired:process.kill();process.wait()
