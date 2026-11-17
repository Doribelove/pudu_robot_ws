"""Interrupt actual API after observing native nodes active; no lifecycle mocks."""
from pathlib import Path
import os,sys,tempfile,subprocess,time,signal,json
import psutil
root=Path(tempfile.mkdtemp(prefix='trilayer-interrupt-test-'));out=root/'request'
script=Path(__file__).with_name('test_semantic_service.py')
with (root/'child.log').open('w') as log:
 child=subprocess.Popen([sys.executable,str(script),sys.argv[1],'baseline',str(out)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 descendants=[]
 try:
  deadline=time.monotonic()+15
  while time.monotonic()<deadline:
   if child.poll() is not None:raise AssertionError('child exited before interruption: '+(root/'child.log').read_text())
   logs=list((out/'runtime/session/logs').glob('*.log'))
   if any('Managed nodes are active' in p.read_text(errors='replace') for p in logs):break
   time.sleep(.001)
  else:raise AssertionError('did not observe actual native lifecycle activation')
  descendants=psutil.Process(child.pid).children(recursive=True)
  assert descendants,'native worker child absent'
  child.send_signal(signal.SIGINT)
  child.wait(timeout=12)
  evidence=json.loads((out/'runtime/result.json').read_text())
  assert evidence['success'] is False
  assert 'KeyboardInterrupt' in evidence['exception']
  assert evidence['cleanup_errors']==[] and evidence['worker_exit']==0,evidence
  assert all(not p.is_running() for p in descendants),'owned native worker survived API cleanup'
  assert (out/'semantic.json').is_file() and (out/'map.pgm').is_file(),'input files deleted'
  print('PASS real post-activation interrupt, captured exception, worker exit0, no descendants;',root)
 finally:
  if child.poll() is None:
   try:descendants+=psutil.Process(child.pid).children(recursive=True)
   except psutil.Error:pass
   child.kill();child.wait(timeout=3)
  for p in descendants:
   try:
    if p.is_running():p.kill()
   except psutil.Error:pass
