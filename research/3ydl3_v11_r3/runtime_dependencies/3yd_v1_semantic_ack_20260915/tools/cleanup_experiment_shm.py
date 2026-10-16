"""Remove unused current-user FastDDS segments from this experiment period."""
import os,json,time
from pathlib import Path
import psutil
from arena_3d_v1.semantic_world import WORK
for process in psutil.process_iter(['name','cmdline']):
    args=process.info['cmdline'] or []
    if process.info['name'] in ('planner_server','map_server') or any(a.endswith('/static_benchmark.py') or a=='tools/static_benchmark.py' for a in args):
        raise RuntimeError('Refusing cleanup while a planning benchmark/server is running')
marker=(WORK/'V1_BASELINE_MANIFEST.json').stat().st_mtime;protected=set();exempt=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        for line in (proc/'maps').read_text().splitlines():
            if '/dev/shm/' in line:protected.add(line.split('/dev/shm/',1)[1].split(' ')[0])
        for fd in (proc/'fd').iterdir():
            try:
                target=os.readlink(fd)
                if target.startswith('/dev/shm/'):protected.add(target[len('/dev/shm/'):].split(' ')[0])
            except OSError:pass
    except PermissionError:
        if proc.stat().st_uid==os.getuid():
            name=(proc/'comm').read_text().strip();status=(proc/'status').read_text()
            if name not in ['(sd-pam)','ssh-agent'] and '\nState:\tZ' not in status:raise
            exempt.append({'pid':proc.name,'name':name})
    except (OSError,ProcessLookupError):pass
candidates=[]
for p in Path('/dev/shm').iterdir():
    if p.name.startswith('fastrtps_') and p.is_file():
        st=p.stat()
        if st.st_uid==os.getuid() and st.st_mtime>=marker and p.name not in protected:candidates.append((p,st))
record={'scope':'unmapped/unopened current-user FastDDS files since this experiment began; no planning servers running','marker_unix':marker,'protected':sorted(protected),'unrelated_protected_processes':exempt,'files':[{'name':p.name,'bytes':st.st_size} for p,st in candidates]}
(WORK/'report'/f'shm_cleanup_{time.time_ns()}.json').write_text(json.dumps(record,indent=2))
for p,st in candidates:p.unlink()
print('cleaned unused FastDDS GiB',sum(st.st_size for p,st in candidates)/2**30)
