"""Linux process identity tracking for one supervised planning worker.

Only a worker created by the caller and its verified descendants are owned.
Names, ROS domains, campaign paths, and reused PIDs never grant ownership.
"""
from __future__ import annotations
from pathlib import Path
import os
import signal
import time


def process_stat(pid):
    try:
        raw=Path(f'/proc/{pid}/stat').read_text()
        fields=raw[raw.rfind(')')+2:].split()
        return {'pid':int(pid),'state':fields[0],'ppid':int(fields[1]),
                'start_ticks':int(fields[19]),'rss_bytes':int(fields[21])*os.sysconf('SC_PAGE_SIZE')}
    except (OSError,ValueError,IndexError):return None


def process_table():
    rows={}
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            row=process_stat(int(path.name))
            if row is not None:rows[row['pid']]=row
    return rows


def signal_identity(pid,start_ticks,signum):
    # Hold the kernel process identity across validation and signaling. A
    # check followed by kill(pid) alone has a PID-reuse race.
    if not hasattr(os,'pidfd_open') or not hasattr(signal,'pidfd_send_signal'):
        raise RuntimeError('LINUX_PIDFD_REQUIRED_FOR_CONFIRMED_RETIREMENT')
    try:fd=os.pidfd_open(pid)
    except ProcessLookupError:return False
    try:
        row=process_stat(pid)
        if row is None or row['start_ticks']!=start_ticks or row['state'] in {'Z','X'}:
            return False
        try:signal.pidfd_send_signal(fd,signum);return True
        except ProcessLookupError:return False
    finally:os.close(fd)


class OwnedProcessTree:
    def __init__(self,root_pid):
        if root_pid==os.getpid():raise ValueError('SUPERVISOR_CANNOT_OWN_ITSELF')
        row=process_stat(root_pid)
        if row is None:raise ProcessLookupError(root_pid)
        self.root_pid=root_pid;self.identities={root_pid:row['start_ticks']}

    def capture(self):
        table=process_table()
        children={}
        for pid,row in table.items():children.setdefault(row['ppid'],[]).append(pid)
        pending=[pid for pid,born in self.identities.items()
                 if pid in table and table[pid]['start_ticks']==born]
        seen=set()
        while pending:
            pid=pending.pop()
            if pid in seen:continue
            seen.add(pid)
            for child in children.get(pid,[]):
                self.identities[child]=table[child]['start_ticks'];pending.append(child)
        active=[row for pid,row in table.items() if self.identities.get(pid)==row['start_ticks']
                and row['state'] not in {'Z','X'}]
        return {'rss_bytes':sum(v['rss_bytes'] for v in active),'active':active}

    def _signal(self,pid,signum):
        row=process_stat(pid)
        if row is None or row['state'] in {'Z','X'} or row['start_ticks']!=self.identities.get(pid):
            return False
        return signal_identity(pid,self.identities[pid],signum)

    def retire(self,timeout_s=2.):
        """Confirm termination before allowing a replacement session to start.

        On a timeout/cancel the old session is uncertain. Freeze verified
        descendants before terminating them so the retiring worker cannot
        continue to launch new ROS processes during cleanup.
        """
        started=time.monotonic();deadline=started+timeout_s;signals=[]
        previous=None
        while time.monotonic()<deadline:
            snapshot=self.capture()
            active=tuple(sorted((v['pid'],v['start_ticks']) for v in snapshot['active']))
            for pid,_ in active:
                if self._signal(pid,signal.SIGSTOP):signals.append([pid,'SIGSTOP'])
            if active==previous:break
            previous=active
        self.capture()
        for pid in sorted(self.identities,reverse=True):
            if self._signal(pid,signal.SIGKILL):signals.append([pid,'SIGKILL'])
        while time.monotonic()<deadline:
            remaining=self.capture()['active']
            if not remaining:break
            time.sleep(.01)
        remaining=self.capture()['active']
        return {'confirmed':not remaining,'remaining':remaining,
                'owned_identities':dict(self.identities),'signals':signals,
                'retirement_wall_ms':(time.monotonic()-started)*1000}
