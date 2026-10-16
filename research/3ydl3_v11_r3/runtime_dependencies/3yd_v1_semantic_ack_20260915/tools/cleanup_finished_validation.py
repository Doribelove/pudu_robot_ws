"""Reap only transport segments recorded by finished validation subprocesses.

Some segments outlive the in-process Context destructor. This test-runner
cleanup is separate from StableSmacSession.close(), and makes no guarantee
about unattended production process supervision.
"""
import json,os,re
from pathlib import Path
import psutil
from arena_3d_v1.semantic_world import WORK,json_write

def main():
 roots=['startup_v3_small','startup_v3_real','final_calibration_v3','final_v3_return_bulb','formal_candidate_v3']
 dispatch=[json.loads(s) for s in (WORK/'report/final_v3_dispatch.jsonl').read_text().splitlines()]
 assert all(any(d['name']==name for d in dispatch) for name in roots),'A validation child is still running'
 owned=set()
 for name in roots:
  for f in (WORK/'results'/name).rglob('shutdown.json'):
   for value in json.loads(f.read_text()).get('owned_shm',[]):
    if re.fullmatch(r'fastrtps_[A-Za-z0-9_]+',value):owned.add(value)
 protected=set();unreadable=[];exempt=[]
 for proc in psutil.process_iter(['pid','uids','name','status']):
  try:
   for line in Path(f'/proc/{proc.pid}/maps').read_text().splitlines():
    if '/dev/shm/' in line:protected.add(line.split('/dev/shm/',1)[1].split(' ')[0])
   for fd in Path(f'/proc/{proc.pid}/fd').iterdir():
    try:
     target=os.readlink(fd)
     if target.startswith('/dev/shm/'):protected.add(target[len('/dev/shm/'):].split(' ')[0])
    except OSError:pass
  except PermissionError:
   if proc.info.get('uids') and proc.info['uids'].real==os.getuid() and proc.info['status']!=psutil.STATUS_ZOMBIE:
    if proc.info['name'] in {'(sd-pam)','ssh-agent','fusermount3'}:exempt.append({'pid':proc.pid,'name':proc.info['name']})
    else:unreadable.append(proc.pid)
  except OSError:pass
 removed=[];protected|={name+'_el' for name in protected}
 if not unreadable:
  for name in sorted((owned|{name+'_el' for name in owned})-protected):
   p=Path('/dev/shm')/name
   try:
    st=p.lstat()
    if p.is_file() and not p.is_symlink() and st.st_uid==os.getuid():
     p.unlink();removed.append({'name':name,'allocated_bytes':st.st_blocks*512,'logical_bytes':st.st_size})
   except FileNotFoundError:pass
 result={'finished_roots':roots,'observed_owned_names':sorted(owned),'protected':sorted(protected),'skipped_unreadable_pids':unreadable,'exempt_system_helpers':exempt,'removed':removed,
  'remaining_owned':[name for name in sorted(owned) if (Path('/dev/shm')/name).exists()],
  'scope':'Post-child-exit cleanup in validation runner; not a claim that session.close leaves zero shared memory.'}
 json_write(WORK/'report/post_process_cleanup.json',result);print(json.dumps({k:result[k] for k in ['removed','remaining_owned','skipped_unreadable_pids']},indent=2))
if __name__=='__main__':main()
