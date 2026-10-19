"""Hard per-request wall deadline, shared by all legal-route attempts/fallbacks."""
from contextlib import contextmanager
import signal,threading,time

class L3RequestBudgetExceeded(BaseException):pass

@contextmanager
def request_guard(seconds):
    if threading.current_thread() is not threading.main_thread():
        raise RuntimeError('BOUNDED_L3_SERVICE_REQUIRES_MAIN_THREAD')
    old_handler=signal.getsignal(signal.SIGALRM);old_delay,old_repeat=signal.getitimer(signal.ITIMER_REAL)
    started=time.monotonic()
    def timeout(*_):raise L3RequestBudgetExceeded('L3_TOTAL_REQUEST_WALL_BUDGET')
    signal.signal(signal.SIGALRM,timeout)
    signal.setitimer(signal.ITIMER_REAL,min(seconds,old_delay) if old_delay else seconds)
    try:yield
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        signal.signal(signal.SIGALRM,old_handler)
        if old_delay:signal.setitimer(signal.ITIMER_REAL,max(.000001,old_delay-(time.monotonic()-started)),old_repeat)
