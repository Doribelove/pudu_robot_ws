"""Supervisor-owned children and journaled cleanup; no ROS initialization here."""
import json,os,re,signal,subprocess,sys,time,uuid
from pathlib import Path
import psutil

RESOURCE=re.compile(r'fastrtps_[A-Za-z0-9_]+$')
HELPERS={'(sd-pam)','ssh-agent','fusermount3'}

def save(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(data,indent=2));os.replace(tmp,path)

def fingerprint(path):
    try:
        s=Path(path).lstat()
        if Path(path).is_file() and not Path(path).is_symlink():return [s.st_dev,s.st_ino,s.st_uid]
    except OSError:pass
    return None

def live(record):
    try:
        p=psutil.Process(record['pid'])
        return p.create_time()==record['created'] and p.status()!=psutil.STATUS_ZOMBIE
    except psutil.Error:return False

def mapped_names(pid):
    names=set()
    try:
        for line in Path(f'/proc/{pid}/maps').read_text().splitlines():
            if '/dev/shm/' in line:
                name=line.split('/dev/shm/',1)[1].split(' ')[0]
                if RESOURCE.fullmatch(name):names.add(name)
    except OSError:pass
    return names

def protected_resources():
    protected=set();unreadable=[];helpers=[]
    for p in psutil.process_iter(['pid','uids','name','status']):
        try:
            for line in Path(f'/proc/{p.pid}/maps').read_text().splitlines():
                if '/dev/shm/' in line:protected.add(line.split('/dev/shm/',1)[1].split(' ')[0])
            for fd in Path(f'/proc/{p.pid}/fd').iterdir():
                try:
                    name=os.readlink(fd)
                    if name.startswith('/dev/shm/'):protected.add(name[len('/dev/shm/'):].split(' ')[0])
                except OSError:pass
        except PermissionError:
            if p.info.get('uids') and p.info['uids'].real==os.getuid() and p.info['status']!=psutil.STATUS_ZOMBIE:
                if p.info['name'] in HELPERS:helpers.append({'pid':p.pid,'name':p.info['name']})
                else:unreadable.append(p.pid)
        except OSError:pass
    protected|={s+'_el' for s in protected}
    return protected,unreadable,helpers

def cleanup(record):
    survivors=[r for r in record['processes'].values() if live(r)]
    if survivors:return {'complete':False,'reason':'recorded_process_still_alive','survivors':survivors,'removed':[]}
    protected,unreadable,helpers=protected_resources();removed=[];retained=[];replaced=[]
    for name,identity in record['resources'].items():
        if not RESOURCE.fullmatch(name):continue
        p=Path('/dev/shm')/name;current=fingerprint(p)
        if current is None:continue
        if current!=identity:replaced.append(name);continue
        if name in protected or unreadable:retained.append(name);continue
        if current[2]!=os.getuid():retained.append(name);continue
        try:
            # Match device/inode again immediately before unlink, and never
            # remove a different generation reusing the same transport name.
            if fingerprint(p)!=identity:replaced.append(name);continue
            st=p.stat();p.unlink();removed.append({'name':name,'allocated_bytes':st.st_blocks*512})
        except FileNotFoundError:pass
    return {'complete':not retained,'removed':removed,'retained':retained,'replaced_names_left_untouched':replaced,
      'unreadable_same_user_pids':unreadable,'exempt_system_helpers':helpers}

def recover_pending(folder):
    recovered=[]
    for path in sorted(Path(folder).glob('*.supervisor.json')):
        try:record=json.loads(path.read_text())
        except (OSError,ValueError):continue
        if record.get('schema')!='3yd-owned-process-v1' or record.get('state')=='complete':continue
        # Do not stop somebody else's live run, including another supervisor.
        if record.get('supervisor') and live(record['supervisor']):continue
        if any(live(r) for r in record.get('processes',{}).values()):
            if record.get('orphan_policy')!='terminate_and_reap':continue
            # Only a dead supervisor's recorded generations are reclaimed.
            # Discover any children of those still-live generations first.
            for r in list(record['processes'].values()):
                if not live(r):continue
                try:
                    parent=psutil.Process(r['pid'])
                    for child in parent.children(recursive=True):
                        record['processes'][str(child.pid)]={'pid':child.pid,'created':child.create_time()}
                except psutil.Error:pass
            record['orphan_recovery_signals']=[]
            for sig,grace in [(signal.SIGINT,3.),(signal.SIGKILL,2.)]:
                for r in record['processes'].values():
                    if live(r):
                        try:os.kill(r['pid'],sig);record['orphan_recovery_signals'].append({'pid':r['pid'],'signal':int(sig)})
                        except ProcessLookupError:pass
                until=time.monotonic()+grace
                while time.monotonic()<until and any(live(r) for r in record['processes'].values()):time.sleep(.05)
        result=cleanup(record);record['recovered_cleanup']=result
        if result['complete']:record['state']='complete'
        save(path,record);recovered.append({'journal':str(path),'cleanup':result})
    return recovered

def supervise(command,journal,*,cwd=None,env=None,termination_grace_s=8.):
    journal=Path(journal)
    if journal.exists():raise FileExistsError(journal)
    journal.parent.mkdir(parents=True,exist_ok=True)
    prior=recover_pending(journal.parent)
    existing={p.name:fingerprint(p) for p in Path('/dev/shm').glob('fastrtps_*')}
    record={'schema':'3yd-owned-process-v1','state':'running','command':command,
      'supervisor':{'pid':os.getpid(),'created':psutil.Process().create_time()},'orphan_policy':'terminate_and_reap','processes':{},'resources':{},
      'preexisting_resources':existing,'recovered_previous':prior,'started':time.time(),'signals':[]}
    requested=[None];old_handlers={}
    def on_signal(signum,frame):requested[0]=signum
    for sig in [signal.SIGINT,signal.SIGTERM]:old_handlers[sig]=signal.signal(sig,on_signal)
    process=None;termination_started=None
    def observe():
        record['supervisor_peak_rss_bytes']=max(record.get('supervisor_peak_rss_bytes',0),psutil.Process().memory_info().rss)
        seeds=[psutil.Process(r['pid']) for r in list(record['processes'].values()) if live(r)]
        for parent in seeds:
            try:children=parent.children(recursive=True)
            except psutil.Error:children=[]
            for p in [parent,*children]:
                try:
                    info={'pid':p.pid,'created':p.create_time()};record['processes'][str(p.pid)]=info
                    for name in mapped_names(p.pid):
                        identity=fingerprint(Path('/dev/shm')/name)
                        if identity and identity[2]==os.getuid() and existing.get(name)!=identity:
                            record['resources'][name]=identity
                            lock=name+'_el';lock_id=fingerprint(Path('/dev/shm')/lock)
                            if lock_id and existing.get(lock)!=lock_id:record['resources'][lock]=lock_id
                except psutil.Error:pass
    def signal_owned(sig):
        for r in list(record['processes'].values()):
            if live(r):
                try:os.kill(r['pid'],sig);record['signals'].append({'pid':r['pid'],'signal':int(sig)})
                except ProcessLookupError:pass
    try:
        child_env=dict(os.environ if env is None else env);child_env['THREEYD_SUPERVISED']=str(journal.resolve())
        save(journal,record)
        process=subprocess.Popen(command,cwd=cwd,env=child_env,start_new_session=True)
        record['processes'][str(process.pid)]={'pid':process.pid,'created':psutil.Process(process.pid).create_time()}
        save(journal,record)
        while process.poll() is None:
            observe();save(journal,record)
            if requested[0] is not None:
                if termination_started is None:
                    termination_started=time.monotonic();process.send_signal(requested[0])
                    record['signals'].append({'pid':process.pid,'signal':requested[0]})
                elif time.monotonic()-termination_started>=termination_grace_s:signal_owned(signal.SIGKILL)
            time.sleep(.1)
        code=process.wait();observe();record['returncode']=code
        # A killed worker cannot execute its finally block. Stop its recorded
        # children before considering shared-memory files for deletion.
        if any(live(r) for r in record['processes'].values()):
            signal_owned(signal.SIGINT);until=time.monotonic()+3
            while time.monotonic()<until and any(live(r) for r in record['processes'].values()):observe();time.sleep(.1)
            signal_owned(signal.SIGKILL)
            until=time.monotonic()+2
            while time.monotonic()<until and any(live(r) for r in record['processes'].values()):time.sleep(.05)
        record['cleanup']=cleanup(record);record['state']='complete' if record['cleanup']['complete'] else 'cleanup_pending'
        record['finished']=time.time();save(journal,record)
        return code if code else (0 if record['cleanup']['complete'] else 70)
    finally:
        # Unexpected supervisor exceptions leave a recoverable journal and
        # release owned processes; signals never become blanket machine kills.
        if process is not None and process.poll() is None:
            signal_owned(signal.SIGKILL)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
            record['cleanup']=cleanup(record);record['state']='complete' if record['cleanup']['complete'] else 'cleanup_pending'
            save(journal,record)
        for sig,handler in old_handlers.items():signal.signal(sig,handler)

def supervised_entry():
    marker=os.environ.get('THREEYD_SUPERVISED')
    if marker:
        p=Path(marker)
        if p.exists():
            record=json.loads(p.read_text())
            if record.get('state')=='running' and record.get('supervisor',{}).get('pid')==os.getppid() and live(record['supervisor']):return
        raise RuntimeError('Supervision marker has no live parent ownership journal')
    if '--output' not in sys.argv:raise ValueError('--output is required for supervised execution')
    output=Path(sys.argv[sys.argv.index('--output')+1]).resolve()
    if output.exists():raise FileExistsError(output)
    os.sched_setaffinity(0,{int(os.environ.get('THREEYD_BENCH_CPU','12'))})
    journal=output.parent/(output.name+'.supervisor.json')
    code=supervise([sys.executable,str(Path(sys.argv[0]).resolve()),*sys.argv[1:]],journal,cwd=os.getcwd())
    raise SystemExit(code)
