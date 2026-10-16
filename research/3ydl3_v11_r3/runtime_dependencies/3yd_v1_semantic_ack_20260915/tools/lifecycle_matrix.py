"""Fixed fresh-process matrix, including dead-supervisor journal recovery."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
from owned_process import recover_pending,live,save
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--domain',type=int,default=106);p.add_argument('--snapshot',choices=['system','locked'],default='system');a=p.parse_args()
root=Path(a.output).resolve();root.mkdir(parents=True,exist_ok=False);rows=[]
modes=['normal','kill_worker','normal','terminate_supervisor','normal','kill_supervisor','normal','kill_worker','normal','terminate_supervisor','kill_supervisor','normal']
for i,mode in enumerate(modes):
 out=root/f'{i:02d}_{mode}';command=[sys.executable,str(Path(__file__).with_name('resource_worker.py')),'--output',str(out),'--domain',str(a.domain),'--snapshot',a.snapshot]+([] if mode=='normal' else ['--wait'])
 before=os.statvfs('/dev/shm');row={'index':i,'mode':mode,'shm_before':(before.f_blocks-before.f_bfree)*before.f_frsize};t=time.monotonic()
 with (root/f'{i:02d}.log').open('w') as f:
  child=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT)
  until=time.monotonic()+20
  while not (out/'ready.json').exists() and child.poll() is None and time.monotonic()<until:time.sleep(.1)
  row['ready']=(out/'ready.json').exists()
  if not row['ready']:
   child.terminate();child.wait(timeout=10);raise RuntimeError(f'Worker did not reach ACK: {out}')
  if mode!='normal':
   time.sleep(.3)  # ensure resource journal has observed the live ROS tree
   if mode=='kill_worker':os.kill(json.loads((out/'ready.json').read_text())['pid'],signal.SIGKILL)
   elif mode=='terminate_supervisor':child.terminate()
   else:child.kill()
  row['supervisor_returncode']=child.wait(timeout=20)
 # Dead-supervisor cleanup is automatically run when the following normal
 # supervised worker starts; retain the unrecovered journal for that step.
 journal=out.with_name(out.name+'.supervisor.json');d=json.loads(journal.read_text())
 row['journal']=str(journal);row['state_immediate']=d['state'];row['elapsed_s']=time.monotonic()-t
 after=os.statvfs('/dev/shm');row['shm_after']=(after.f_blocks-after.f_bfree)*after.f_frsize;rows.append(row);save(root/'progress.json',rows);print(i,mode,row['state_immediate'],round(row['elapsed_s'],1),flush=True)
recover_pending(root)
for row in rows:
 d=json.loads(Path(row['journal']).read_text());c=d.get('recovered_cleanup',d.get('cleanup',{}))
 row.update(final_state=d['state'],cleanup_complete=c.get('complete',False),remaining_processes=[r for r in d['processes'].values() if live(r)],remaining_recorded_files=[s for s in d['resources'] if (Path('/dev/shm')/s).exists()],removed_bytes=sum(x['allocated_bytes'] for x in c.get('removed',[])))
 row['success']=row['ready'] and row['final_state']=='complete' and row['cleanup_complete'] and not row['remaining_processes'] and not row['remaining_recorded_files']
save(root/'summary.json',{'cycles':len(rows),'success':sum(r['success'] for r in rows),'rows':rows});print('COMPLETE',sum(r['success'] for r in rows),len(rows),flush=True)
