"""Supervised, serialized static planning with a bounded request queue.

The ROS engine lives in a separate process. Late results are never accepted;
an uncertain or cancelled engine is retired and its process identities are
checked before a new generation can become READY.
"""
from __future__ import annotations
from collections import deque
from concurrent.futures import Future
from dataclasses import dataclass
from pathlib import Path
import json
import math
import multiprocessing
import os
import pickle
import queue
import threading
import tempfile
import time
from .owned_processes_r3 import OwnedProcessTree,process_stat

MAX_RESULT_BYTES=64*1024**2


def _engine_worker(connection,config,output,temporary_root):
    os.environ['TMPDIR']=temporary_root;tempfile.tempdir=temporary_root
    from .static_planner_engine_r3 import EngineConfig,StaticPlannerEngine
    engine=None
    try:
        engine=StaticPlannerEngine(EngineConfig(**config),output)
        connection.send_bytes(pickle.dumps({'event':'ready','value':engine.start()},protocol=5))
        while True:
            if not engine.backend_available():raise RuntimeError('BACKEND_PROCESS_DIED')
            if not connection.poll(.1):continue
            request=pickle.loads(connection.recv_bytes(16*1024))
            if request['command']=='close':break
            if request['command']!='plan':raise ValueError('UNKNOWN_WORKER_COMMAND')
            result=engine.plan(request['request'],request['deadline'])
            payload=pickle.dumps({'event':'result','value':result},protocol=5)
            if len(payload)>MAX_RESULT_BYTES:raise ValueError('RESULT_TRANSPORT_CAPACITY')
            connection.send_bytes(payload)
    except BaseException as exc:
        try:connection.send_bytes(pickle.dumps({'event':'error','error':repr(exc)},protocol=5))
        except (OSError,EOFError):pass
    finally:
        if engine is not None:engine.close()
        connection.close()


class WorkerProcess:
    def __init__(self,config,output):
        context=multiprocessing.get_context('spawn')
        # AF_UNIX paths are limited to 108 bytes. Keep the private temporary
        # root short; its parent owns cleanup even after a forced retirement.
        self.temporary=tempfile.TemporaryDirectory(prefix='pln-svc-',dir='/tmp')
        parent,child=context.Pipe(duplex=True)
        self.connection=parent;self.events=queue.Queue(maxsize=2)
        self.process=context.Process(target=_engine_worker,args=(child,config,str(output),self.temporary.name))
        self.process.start();child.close()
        self.tree=OwnedProcessTree(self.process.pid)
        self.reader=threading.Thread(target=self._receive,daemon=True,name='pln-worker-results')
        self.reader.start()

    def _receive(self):
        try:
            while True:
                event=pickle.loads(self.connection.recv_bytes(MAX_RESULT_BYTES))
                self.events.put(event,timeout=1.)
                if event['event']=='error':break
        except (EOFError,OSError,ValueError,queue.Full) as exc:
            try:self.events.put_nowait({'event':'error','error':repr(exc)})
            except queue.Full:pass

    def send(self,request,deadline):
        payload=pickle.dumps({'command':'plan','request':request,'deadline':deadline},protocol=5)
        if len(payload)>16*1024:raise ValueError('REQUEST_TRANSPORT_CAPACITY')
        self.connection.send_bytes(payload)

    def poll(self):
        try:return self.events.get_nowait()
        except queue.Empty:return None

    def rss(self):
        return self.tree.capture()['rss_bytes']

    def retire(self):
        result=self.tree.retire(timeout_s=2.)
        self.process.join(timeout=.2)
        self.connection.close();self.reader.join(timeout=.2)
        result['reader_stopped']=not self.reader.is_alive()
        result['confirmed']=result['confirmed'] and result['reader_stopped']
        if result['confirmed']:
            self.temporary.cleanup();result['temporary_cleanup_confirmed']=True
        return result


@dataclass
class PendingRequest:
    request: dict
    future: Future
    received: float
    deadline: float
    started: float|None=None
    cancelled: bool=False


class StaticPlannerSupervisor:
    def __init__(self,config,output,*,queue_capacity=4,startup_timeout_s=1500.,
                 recovery_timeout_s=90.,rss_limit_bytes=10*1024**3,
                 output_limit_bytes=512*1024**2,output_limit_files=4096,worker_factory=WorkerProcess):
        if queue_capacity<1 or queue_capacity>16:raise ValueError('QUEUE_CAPACITY_INVALID')
        if not 0<rss_limit_bytes<=10*1024**3:raise ValueError('RSS_LIMIT_INVALID')
        if not 1024<=output_limit_bytes<=1024**3 or not 16<=output_limit_files<=8192:
            raise ValueError('OUTPUT_LIMIT_INVALID')
        if not all(math.isfinite(t) and t>0 for t in [startup_timeout_s,recovery_timeout_s]):
            raise ValueError('INITIALIZATION_TIMEOUT_INVALID')
        self.config=dict(config);self.output=Path(output).resolve()
        self.queue_capacity=queue_capacity;self.startup_timeout=startup_timeout_s
        self.recovery_timeout=recovery_timeout_s;self.rss_limit=rss_limit_bytes
        self.output_limit_bytes=output_limit_bytes;self.output_limit_files=output_limit_files
        self.worker_factory=worker_factory;self.lock=threading.RLock()
        self.pending=deque();self.active=None;self.ids=set();self.finished=deque(maxlen=256)
        self.generation=0;self.worker=None;self.state='STOPPED';self.ready_receipt=None
        self.journal_error=None
        self.stopping=threading.Event();self.thread=None
        self.stats={'completed':0,'recovery_count':0,'peak_sampled_tree_rss_bytes':0,
                    'late_results_rejected':0,'queue_rejections':0,
                    'output_bytes':0,'output_files':0}

    def _output_usage(self):
        # These are service-owned output paths only. No historical evidence
        # is overwritten or deleted to make room for another request.
        total=0;files=0
        for directory,_,names in os.walk(self.output,followlinks=False):
            for name in names:
                try:total+=os.lstat(Path(directory)/name).st_size;files+=1
                except FileNotFoundError:continue
                if total>self.output_limit_bytes or files>self.output_limit_files:
                    return total,files
        return total,files

    def _event(self,event,**values):
        row={'event':event,'monotonic':time.monotonic(),'generation':self.generation,**values}
        try:
            with (self.output/'service_events.jsonl').open('a') as stream:
                stream.write(json.dumps(row,sort_keys=True,allow_nan=False)+'\n')
        except OSError as exc:
            # Logging failure must never bypass process retirement or strand
            # a queued Future. Retain the error in status and fail closed.
            self.journal_error=repr(exc);self.state='FAILED';self.stopping.set()
            self.ready_receipt=None
            return False
        return True

    def start(self):
        if self.thread is not None:raise RuntimeError('SERVICE_ALREADY_STARTED')
        self.output.mkdir(parents=True,exist_ok=False);self.state='INITIALIZING'
        self.thread=threading.Thread(target=self._loop,daemon=True,name='pln-supervisor')
        self.thread.start()

    def status(self):
        with self.lock:
            return {'state':self.state,'generation':self.generation,'queued':len(self.pending),
                    'active_request_id':self.active.request['request_id'] if self.active else None,
                    'ready_receipt':self.ready_receipt,'journal_error':self.journal_error,**self.stats}

    def submit(self,request):
        rid=request.get('request_id')
        if not isinstance(rid,str) or not rid or len(rid)>128:raise ValueError('INVALID_REQUEST_ID')
        for key in ['start','goal']:
            pose=request.get(key)
            if not isinstance(pose,(list,tuple)) or len(pose)!=3 or any(
                    isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in pose):
                raise ValueError('INVALID_ENDPOINT_POSE')
        # Copy only the supported primitive request fields before queueing.
        frozen={'request_id':rid,'start':list(request['start']),'goal':list(request['goal'])}
        now=time.monotonic();future=Future();task=PendingRequest(frozen,future,now,now+7.)
        with self.lock:
            if self.stopping.is_set() or self.state not in {'READY','BUSY'}:
                raise RuntimeError('SERVICE_NOT_READY')
            if rid in self.ids or rid in self.finished:raise ValueError('DUPLICATE_REQUEST_ID')
            if len(self.pending)>=self.queue_capacity:
                self.stats['queue_rejections']+=1;raise RuntimeError('REQUEST_QUEUE_FULL')
            self.pending.append(task);self.ids.add(rid)
        future.add_done_callback(lambda value:self.cancel(rid) if value.cancelled() else None)
        return future

    def cancel(self,request_id):
        with self.lock:
            for task in ([self.active] if self.active else [])+list(self.pending):
                if task.request['request_id']==request_id:
                    task.cancelled=True;return True
        return False

    def _finish(self,task,result):
        completed=time.monotonic();rid=task.request['request_id']
        if result.get('request_id',rid)!=rid:raise RuntimeError('WORKER_REQUEST_BINDING_MISMATCH')
        if result.get('final_valid_success'):
            attempts=result.get('attempts',[])
            if not (result.get('static_footprint_valid') is True and
                    result.get('kinematic_valid') is True and result.get('canonical_path_hash') and
                    result.get('points') and any(a.get('planner_search_started') for a in attempts)):
                raise RuntimeError('WORKER_CANONICAL_CERTIFICATE_REQUIRED')
            for attempt in attempts:
                if attempt.get('planner_search_started') and not (
                        attempt.get('costmap_update_acknowledged') is True and
                        all(attempt.get(k)==0 for k in ['costmap_ack_mismatch_cells','hard_mismatch',
                            'soft_mismatch','stale_cells','hash_mismatch','sequence_mismatch']) and
                        attempt.get('publication',{}).get('request_id')==rid):
                    raise RuntimeError('WORKER_EXACT_ACK_REQUIRED')
        if result.get('final_valid_success') and completed>task.deadline:
            result['rejected_candidate_points']=result.pop('points',[])
            result.update(final_valid_success=False,points=[],failure_code='REQUEST_DEADLINE')
            self.stats['late_results_rejected']+=1
        result.update(request_id=rid,service_generation=self.generation,
                      service_queue_wait_ms=((task.started or completed)-task.received)*1000,
                      service_end_to_end_ms=(completed-task.received)*1000)
        with self.lock:
            self.ids.discard(rid);self.finished.append(rid);self.stats['completed']+=1
            if self.active is task:self.active=None
        logged=self._event('request_complete',request_id=rid,valid=result.get('final_valid_success',False),
                           failure=result.get('failure_code',''),service_end_to_end_ms=result['service_end_to_end_ms'])
        if not logged:
            result['rejected_candidate_points']=result.get('points',[])
            result.update(final_valid_success=False,points=[],failure_code='SERVICE_JOURNAL_UNAVAILABLE')
        if not task.future.done():task.future.set_result(result)

    def _fail(self,task,reason,**values):
        self._finish(task,{'final_valid_success':False,'points':[],'failure_code':reason,**values})

    def _recover(self,reason):
        with self.lock:self.state='RECOVERING';self.ready_receipt=None
        self._event('retire_begin',reason=reason)
        result=self.worker.retire() if self.worker else {'confirmed':True}
        self._event('retire_complete',reason=reason,retirement=result)
        if not result['confirmed']:
            self.state='FAILED';raise RuntimeError('OWNED_PROCESS_RETIREMENT_UNCONFIRMED')
        self.worker=None;self.stats['recovery_count']+=1

    def _loop(self):
        ready_deadline=0.;last_resource=0.;last_output=0.
        try:
            while not self.stopping.is_set():
                if self.worker is None:
                    self.generation+=1
                    self.worker=self.worker_factory(self.config,self.output/f'session_{self.generation:04d}')
                    ready_deadline=time.monotonic()+(self.startup_timeout if self.generation==1 else self.recovery_timeout)
                    self._event('worker_started')
                now=time.monotonic()
                if now-last_output>=1.:
                    last_output=now;size,count=self._output_usage()
                    self.stats.update(output_bytes=size,output_files=count)
                    if size>self.output_limit_bytes or count>self.output_limit_files:
                        if self.active:self._fail(self.active,'SERVICE_OUTPUT_CAPACITY')
                        self._recover('SERVICE_OUTPUT_CAPACITY');self.state='FAILED';break
                if now-last_resource>=.1:
                    last_resource=now
                    parent=process_stat(os.getpid())
                    rss=self.worker.rss()+(parent['rss_bytes'] if parent else 0)
                    self.stats['peak_sampled_tree_rss_bytes']=max(self.stats['peak_sampled_tree_rss_bytes'],rss)
                    if rss>self.rss_limit:
                        if self.active:self._fail(self.active,'SERVICE_RESOURCE_LIMIT');self.active=None
                        self._recover('SERVICE_RESOURCE_LIMIT')
                        self.state='FAILED';break
                event=self.worker.poll()
                if event:
                    if event['event']=='ready':
                        self.ready_receipt=event['value'];self.state='READY';self._event('ready')
                    elif event['event']=='result':
                        if self.active is None:raise RuntimeError('UNEXPECTED_WORKER_RESULT')
                        task=self.active;result=event['value']
                        # Linearize cancellation and delivery under the same
                        # lock. A successful cancel must not race with a valid
                        # result that already passed an earlier flag check.
                        with self.lock:
                            cancelled=task.cancelled
                            if not cancelled:
                                if result.get('session_recovery_required'):
                                    self.state='RECOVERING';self.ready_receipt=None
                                self._finish(task,result)
                        if cancelled:
                            self._recover('USER_CANCELLED');self._fail(task,'USER_CANCELLED',retirement_confirmed=True)
                        else:
                            if result.get('session_recovery_required'):self._recover('SESSION_UNCERTAIN')
                            else:self.state='FAILED' if self.stopping.is_set() else 'READY'
                    else:
                        if self.active:self._fail(self.active,'WORKER_FAILURE',detail=event.get('error',''));self.active=None
                        if self.state in {'INITIALIZING','RECOVERING'}:
                            self._recover('INITIALIZATION_FAILURE');self.state='FAILED';break
                        self._recover('WORKER_FAILURE')
                if self.worker is None:continue
                now=time.monotonic()
                if self.state in {'INITIALIZING','RECOVERING'} and now>=ready_deadline:
                    self._recover('READY_DEADLINE');self.state='FAILED';break
                if self.active and (self.active.cancelled or now>=self.active.deadline):
                    task=self.active
                    if task.cancelled:
                        self._recover('USER_CANCELLED');self._fail(task,'USER_CANCELLED',retirement_confirmed=True)
                    else:
                        self._fail(task,'REQUEST_DEADLINE',retirement_pending=True)
                        self._recover('REQUEST_DEADLINE')
                    continue
                with self.lock:
                    while self.pending and (self.pending[0].cancelled or now>=self.pending[0].deadline):
                        task=self.pending.popleft()
                        self._fail(task,'USER_CANCELLED' if task.cancelled else 'REQUEST_DEADLINE_IN_QUEUE')
                    if self.state=='READY' and self.active is None and self.pending:
                        self.active=self.pending.popleft();self.active.started=time.monotonic();self.state='BUSY'
                        self.worker.send(self.active.request,self.active.deadline)
                time.sleep(.01)
        except BaseException as exc:
            self.state='FAILED';self._event('service_error',error=repr(exc))
        finally:
            if self.worker is not None:
                try:
                    result=self.worker.retire();self._event('shutdown_retirement',retirement=result)
                    if not result['confirmed']:self.state='FAILED'
                except BaseException as exc:
                    self.state='FAILED';self._event('shutdown_retirement_error',error=repr(exc))
            with self.lock:
                tasks=([self.active] if self.active else [])+list(self.pending)
                self.active=None;self.pending.clear()
                for task in tasks:
                    if not task.future.done():self._fail(task,'SERVICE_STOPPED')
                if self.journal_error:self.state='FAILED'
                elif self.state!='FAILED':self.state='STOPPED'
                self.ready_receipt=None

    def close(self):
        self.stopping.set()
        if self.thread is not None:self.thread.join(timeout=5.)
        if self.thread is not None and self.thread.is_alive():raise RuntimeError('SERVICE_STOP_UNCONFIRMED')
