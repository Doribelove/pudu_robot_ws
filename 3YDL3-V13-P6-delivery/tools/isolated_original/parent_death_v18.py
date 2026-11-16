"""Kernel parent-death primitives, no action on import."""
import ctypes,os,signal,threading
PR_SET_PDEATHSIG=1

def arm_parent_death(expected_parent,sig,*,getppid=os.getppid,setter=None,terminate=None):
 """Arm then check parent identity: closes death-before-prctl fork race."""
 if setter is None:
  lib=ctypes.CDLL(None,use_errno=True)
  setter=lambda s:lib.prctl(PR_SET_PDEATHSIG,int(s),0,0,0)
 if terminate is None:terminate=lambda:os.kill(os.getpid(),signal.SIGKILL)
 if int(expected_parent)<=1:raise RuntimeError('Explicit live owner parent required')
 if setter(sig)!=0:raise OSError(ctypes.get_errno(),'PR_SET_PDEATHSIG')
 if getppid()!=expected_parent:
  terminate();raise RuntimeError('Parent changed during fork/exec startup')

class CancellationRequested(BaseException):pass
CANCEL={'requested':False,'signal':None,'received_monotonic':None,'cleanup_started':None,'cleanup_finished':None}
CLEANING=False
def check_cancel():
 if CANCEL['requested'] and not CLEANING:raise CancellationRequested('SUPERVISOR_CANCEL_SIGNAL_'+str(CANCEL['signal']))
def receive_cancel(sig,frame=None):
 import time
 if not CANCEL['requested']:CANCEL.update(requested=True,signal=int(sig),received_monotonic=time.monotonic())
 check_cancel()
def begin_cleanup():
 global CLEANING
 import time
 CLEANING=True;CANCEL['cleanup_started']=time.monotonic()
def finish_cleanup():
 import time
 CANCEL['cleanup_finished']=time.monotonic()
def install_cancel_handlers():
 signal.signal(signal.SIGTERM,receive_cancel);signal.signal(signal.SIGHUP,receive_cancel)

def child_preexec(expected_parent,limit_setter):
 # Must be called by fresh single-thread supervisor; preexec is otherwise unsafe.
 arm_parent_death(expected_parent,signal.SIGKILL)
 limit_setter()
