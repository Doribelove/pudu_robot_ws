"""Single-worker private-namespace runner; all writable runtime data in bounded tmpfs."""
import os,sys,json,pathlib,time,signal,subprocess,selectors,resource,ctypes,hashlib,argparse,select,base64,io,tarfile,shutil
O=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(O))
from owned_tree_v18 import children_of,kill_owned
from parent_death_v18 import arm_parent_death,install_cancel_handlers,child_preexec,check_cancel,begin_cleanup,finish_cleanup,CANCEL
from seccomp_policy_v11 import export_filter

def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['bootstrap','plan','fixture','fixture-orphan']);p.add_argument('--map',default='arena_small_080x100_2d_005');p.add_argument('--query',default='Q01');a=p.parse_args()
 attempt=a.mode+'_'+str(time.time_ns());d=O.parent/'runs'/attempt;d.mkdir(parents=True)
 source={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in O.iterdir() if x.is_file()};(d/'SOURCE_LOCK.json').write_text(json.dumps(source,indent=2)+'\n')
 (d/'source').mkdir()
 for name in source:shutil.copy2(O/name,d/'source'/name)
 supplement=json.loads((O.parent/'supplement/MANIFEST.json').read_text());(d/'SUPPLEMENT_LOCK.json').write_text(json.dumps(supplement,indent=2)+'\n')
 install_cancel_handlers();arm_parent_death(os.getppid(),signal.SIGTERM);assert ctypes.CDLL(None).prctl(36,1,0,0,0)==0
 parent=os.getpid();mp=O.parent/('mount_'+attempt);mp.mkdir();fd=None;proc=None;sel=selectors.DefaultSelector();buf=bytearray();reason='completed';t=time.monotonic();peak=0;observed={};errors=[];before={}
 def limits():
  for k,v in [(resource.RLIMIT_AS,4*1024**3),(resource.RLIMIT_CPU,150),(resource.RLIMIT_CORE,0),(resource.RLIMIT_FSIZE,512*1024**2),(resource.RLIMIT_NOFILE,256)]:resource.setrlimit(k,(v,v))
 try:
  fd,blocked=export_filter();argv=['/usr/bin/unshare','--user','--map-root-user','--mount','--pid','--fork','--kill-child=KILL','--ipc','--net','--mount-proc','/bin/sh',str(O/'namespace.sh'),str(mp),str(fd),str(1024**3),'/review/worker.py',a.mode+':'+a.map+':'+a.query,str(os.stat('/proc/self/ns/net').st_ino),str(os.stat('/proc/self/ns/ipc').st_ino)]
  (d/'COMMAND.json').write_text(json.dumps(argv,indent=2)+'\n')
  proc=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True,pass_fds=(fd,),preexec_fn=lambda:child_preexec(parent,limits));sel.register(proc.stdout,selectors.EVENT_READ)
  while True:
   check_cancel();owned=children_of(parent);observed.update(owned);rss=tasks=0
   for pid in owned:
    try:
     fields={x.split(':',1)[0]:x.split(':',1)[1].strip() for x in pathlib.Path('/proc/'+str(pid)+'/status').read_text().splitlines() if ':' in x};tasks+=int(fields.get('Threads','0'));rss+=int(fields.get('VmRSS','0 kB').split()[0])*1024
    except (OSError,ValueError):pass
   peak=max(peak,rss)
   if time.monotonic()-t>180:reason='wall_timeout';break
   if rss>3*1024**3 or tasks>96 or len(owned)>12:reason='tree_resource_limit';break
   if proc.poll() is not None and not sel.get_map():break
   for key,_ in sel.select(.05):
    b=os.read(key.fd,65536)
    if not b:sel.unregister(key.fileobj);continue
    if len(buf)+len(b)>40*1024**2:reason='stdout_limit';break
    buf.extend(b)
   if reason!='completed':break
 except BaseException as e:reason=type(e).__name__+':'+str(e)
 finally:
  begin_cleanup();before=children_of(parent)
  try:killed=kill_owned(parent,proc.pid if proc else -1)
  except BaseException as e:killed=[];errors.append(str(e))
  if proc:
   try:proc.wait(timeout=3)
   except BaseException as e:errors.append(str(e))
   if proc.stdout:proc.stdout.close()
  if fd is not None:os.close(fd)
  sel.close();finish_cleanup()
 remaining=children_of(parent);mounts=[x for x in pathlib.Path('/proc/self/mountinfo').read_text().splitlines() if str(mp) in x];contents=list(mp.iterdir())
 text=buf.decode(errors='replace');(d/'stdout.jsonl').write_text(text)
 terminal=None
 for line in text.splitlines():
  try:r=json.loads(line)
  except Exception:continue
  if r.get('event')=='terminal':terminal=r
  if r.get('event')=='artifacts':
   raw=base64.b64decode(r['tar_gz']);ar=tarfile.open(fileobj=io.BytesIO(raw),mode='r:gz')
   members=ar.getmembers();assert len(members)<=4096 and sum(m.size for m in members)<=64*1024**2
   current=sum(p.stat().st_size for p in O.parent.rglob('*') if p.is_file());assert current+sum(m.size for m in members)<=400*1024**2
   for m in members:
    dest=(d/'artifacts'/m.name).resolve();assert dest.is_relative_to((d/'artifacts').resolve()) and m.isfile()
    dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(ar.extractfile(m).read())
 report={'version':'A2B-P6-closeout-C1','mode':a.mode,'map':a.map,'query':a.query,'reason':reason,'returncode':proc.returncode if proc else None,'elapsed_s':time.monotonic()-t,'peak_tree_RSS_bytes':peak,'terminal':terminal,'owned_seen':observed,'before_cleanup':before,'remaining':remaining,'killed':killed,'cleanup_errors':errors,'host_mounts':mounts,'host_mount_contents':[str(x) for x in contents],'cleanup_pass':not remaining and not mounts and not contents and not errors and not before and not killed,'rescue_cleanup_complete':not remaining and not mounts and not contents and not errors,'source_unchanged':all(hashlib.sha256((O/k).read_bytes()).hexdigest()==v for k,v in source.items())}
 report['pass']=reason=='completed' and report['returncode']==0 and bool(terminal and terminal.get('pass')) and report['cleanup_pass'] and report['source_unchanged'];(d/'RESULT.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'directory':str(d),**report},ensure_ascii=False,default=str));return 0 if report['pass'] else 1
if __name__=='__main__':raise SystemExit(main())
