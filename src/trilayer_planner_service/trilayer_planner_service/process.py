# SPDX-License-Identifier: Apache-2.0
"""Bounded execution and cleanup of this invocation's processes only."""
import os,signal,subprocess,time
import psutil


def run_bounded(command,log_file,*,timeout,env,grace=5.,cwd=None,deadline_file=None):
    tracked={};timed_out=False;interrupted=False
    with open(log_file,'w') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                               env=env,cwd=cwd,start_new_session=True)
        try:root=psutil.Process(child.pid)
        except psutil.NoSuchProcess:root=None
        def collect():
            # A persistent broker can create many cold generations; discard only
            # confirmed dead handles, not live detached descendants.
            for key,p in list(tracked.items()):
                try:
                    if not p.is_running() or p.create_time()!=key[1] or p.status()==psutil.STATUS_ZOMBIE:tracked.pop(key,None)
                except psutil.NoSuchProcess:tracked.pop(key,None)
                except psutil.Error:pass
            try:
                if root is None:return
                for p in root.children(recursive=True):tracked[(p.pid,p.create_time())]=p
            except psutil.Error:pass
        def terminate():
            collect()
            if child.poll() is None:
                try:child.send_signal(signal.SIGINT)
                except ProcessLookupError:pass
                try:child.wait(timeout=grace)
                except subprocess.TimeoutExpired:
                    child.kill();child.wait(timeout=grace)
            for (pid,created),p in reversed(list(tracked.items())):
                try:
                    if p.is_running() and p.create_time()==created and p.status()!=psutil.STATUS_ZOMBIE:p.terminate()
                except psutil.Error:pass
            _,alive=psutil.wait_procs(list(tracked.values()),timeout=grace)
            for p in alive:
                try:
                    if (p.pid,p.create_time()) in tracked and p.status()!=psutil.STATUS_ZOMBIE:p.kill()
                except psutil.Error:pass
            psutil.wait_procs(alive,timeout=grace)
        def cancel(signum,frame):
            raise KeyboardInterrupt
        previous_term=signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGTERM,cancel)
        deadline=None if timeout==0 else time.monotonic()+timeout
        try:
            while child.poll() is None:
                collect()
                request_expired=False
                if deadline_file is not None:
                    import json
                    try:
                        with open(deadline_file) as f: request_deadline=json.load(f).get('deadline_monotonic')
                        request_expired=request_deadline is not None and time.monotonic()>request_deadline
                    except (OSError,ValueError,TypeError):pass
                if (deadline is not None and time.monotonic()>=deadline) or request_expired:
                    timed_out=True;break
                time.sleep(.03)
        except KeyboardInterrupt:interrupted=True
        finally:
            # Finish bounded cleanup even when a second Ctrl-C arrives.
            previous=signal.getsignal(signal.SIGINT)
            signal.signal(signal.SIGINT,signal.SIG_IGN)
            signal.signal(signal.SIGTERM,signal.SIG_IGN)
            try:terminate()
            finally:
                signal.signal(signal.SIGINT,previous)
                signal.signal(signal.SIGTERM,previous_term)
        remaining=[]
        for key,p in tracked.items():
            try:
                if p.is_running() and (p.pid,p.create_time())==key and p.status()!=psutil.STATUS_ZOMBIE:remaining.append(p.pid)
            except psutil.Error:pass
    return {'returncode':child.returncode,'timed_out':timed_out,
            'interrupted':interrupted,'remaining_processes':remaining}
