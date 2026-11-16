"""Hard per-request wall deadline, shared by all legal-route attempts/fallbacks."""
from contextlib import contextmanager
from contextvars import ContextVar
import signal,threading,time

_DEADLINE=ContextVar('global_navigation_deadline',default=None)

def request_deadline():return _DEADLINE.get()

class L3RequestBudgetExceeded(BaseException):pass

@contextmanager
def request_guard(seconds):
    if threading.current_thread() is not threading.main_thread():
        raise RuntimeError('BOUNDED_L3_SERVICE_REQUIRES_MAIN_THREAD')
    old_handler=signal.getsignal(signal.SIGALRM);old_delay,old_repeat=signal.getitimer(signal.ITIMER_REAL)
    started=time.monotonic()
    def timeout(*_):raise L3RequestBudgetExceeded('L3_TOTAL_REQUEST_WALL_BUDGET')
    signal.signal(signal.SIGALRM,timeout)
    duration=min(seconds,old_delay) if old_delay else seconds
    inherited=_DEADLINE.get()
    token=_DEADLINE.set(min(started+duration,inherited) if inherited is not None else started+duration)
    signal.setitimer(signal.ITIMER_REAL,duration)
    interrupted=False
    try:yield
    except BaseException:
        interrupted=True
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        _DEADLINE.reset(token)
        signal.signal(signal.SIGALRM,old_handler)
        if old_delay:
            remaining=old_delay-(time.monotonic()-started)
            if remaining>0:signal.setitimer(signal.ITIMER_REAL,remaining,old_repeat)
            elif not interrupted:
                # Fail synchronously if cleanup crossed the caller's timer.
                # Do not schedule a second alarm while unwinding an already
                # delivered deadline exception: it masks its failure record.
                raise L3RequestBudgetExceeded('L3_TOTAL_REQUEST_WALL_BUDGET')
