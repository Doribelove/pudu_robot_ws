"""Copied static identity-based cleanup functions, no runtime invocation."""
import os,signal,time
from pathlib import Path
from parent_death_v18 import check_cancel
def children_of(root):
    check_cancel();rows={}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            raw=(entry/'stat').read_text();fields=raw[raw.rfind(')')+2:].split()
            rows[int(entry.name)]=(int(fields[1]),int(fields[19]))
        except (OSError,ValueError,IndexError):continue
    check_cancel();found={root};old=None
    while old!=found:
        old=set(found);found|={pid for pid,(ppid,_) in rows.items() if ppid in found}
    return {pid:rows[pid][1] for pid in found if pid!=root and pid in rows}

def kill_owned(root, leader):
    """Use pidfd identity and current ancestry, never name-based pkill."""
    killed=set()
    for _ in range(12):
        targets=children_of(root)
        for pid,born in targets.items():
            fd=None
            try:
                fd=os.pidfd_open(pid)
                now=children_of(root)
                if now.get(pid)==born:signal.pidfd_send_signal(fd,signal.SIGKILL);killed.add(pid)
            except (OSError,ProcessLookupError):pass
            finally:
                if fd is not None:os.close(fd)
        for pid in targets:
            if pid==leader:continue
            try:os.waitpid(pid,os.WNOHANG)
            except ChildProcessError:pass
        if not children_of(root):break
        time.sleep(.02)
    return sorted(killed)

