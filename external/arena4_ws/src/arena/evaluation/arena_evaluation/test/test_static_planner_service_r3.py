"""Supervisor safety gates and recovery use a deterministic fake ROS worker."""
from collections import deque
import time

import pytest

from arena_evaluation.static_planner_service_r3 import StaticPlannerSupervisor


class Worker:
    def __init__(self):
        self.events=deque([{'event':'ready','value':{'state':'READY'}}])
        self.sent=[];self.retired=False;self.confirmed=True;self.memory=1
    def poll(self):return self.events.popleft() if self.events else None
    def send(self,request,deadline):self.sent.append((request,deadline))
    def rss(self):return self.memory
    def retire(self):self.retired=True;return {'confirmed':self.confirmed}


def until(predicate,timeout=1.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.005)
    assert predicate()


@pytest.fixture
def service(tmp_path):
    workers=[]
    def factory(*args):
        if workers:assert workers[-1].retired
        w=Worker();workers.append(w);return w
    s=StaticPlannerSupervisor({},tmp_path/'service',worker_factory=factory,
                              startup_timeout_s=.2,recovery_timeout_s=.2)
    s.start();until(lambda:s.status()['state']=='READY')
    yield s,workers
    s.close()


def req(rid):return {'request_id':rid,'start':[1.,2.,0.],'goal':[5.,6.,0.]}


def valid(rid):
    attempt={'planner_search_started':True,'costmap_update_acknowledged':True,
             'publication':{'request_id':rid}}
    attempt.update({k:0 for k in ['costmap_ack_mismatch_cells','hard_mismatch',
                                 'soft_mismatch','stale_cells','hash_mismatch','sequence_mismatch']})
    return {'request_id':rid,'final_valid_success':True,'static_footprint_valid':True,
            'kinematic_valid':True,'canonical_path_hash':'certified',
            'points':[[1,2,0],[2,2,0]],'attempts':[attempt]}


def test_requests_are_serial_and_inputs_are_frozen(service):
    s,workers=service;original=req('a');a=s.submit(original);original['start'][0]=999
    b=s.submit(req('b'));until(lambda:len(workers[0].sent)==1)
    assert workers[0].sent[0][0]['start'][0]==1.
    workers[0].events.append({'event':'result','value':valid('a')})
    assert a.result(timeout=1)['final_valid_success']
    until(lambda:len(workers[0].sent)==2)
    assert not b.done()
    workers[0].events.append({'event':'result','value':valid('b')})
    assert b.result(timeout=1)['final_valid_success']


def test_queue_has_fixed_capacity_and_duplicate_guard(service):
    s,workers=service;s.submit(req('a'));until(lambda:bool(workers[0].sent))
    for i in range(s.queue_capacity):s.submit(req(str(i)))
    with pytest.raises(RuntimeError,match='QUEUE_FULL'):s.submit(req('overflow'))
    with pytest.raises(ValueError,match='DUPLICATE'):s.submit(req('a'))
    assert s.status()['queued']==s.queue_capacity


@pytest.mark.parametrize('field',['hard_mismatch','soft_mismatch','stale_cells','hash_mismatch','sequence_mismatch','costmap_ack_mismatch_cells'])
def test_any_ack_mismatch_cannot_expose_path(service,field):
    s,workers=service;f=s.submit(req('a'));until(lambda:bool(workers[0].sent))
    result=valid('a');result['attempts'][0][field]=1
    workers[0].events.append({'event':'result','value':result})
    r=f.result(timeout=1);assert not r['final_valid_success'] and not r['points']
    until(lambda:workers[0].retired)


@pytest.mark.parametrize('mutation',['wrong_request','wrong_publication','missing_audit','missing_points'])
def test_result_binding_and_canonical_certificate_required(service,mutation):
    s,workers=service;f=s.submit(req('a'));until(lambda:bool(workers[0].sent))
    result=valid('a')
    if mutation=='wrong_request':result['request_id']='wrong'
    if mutation=='wrong_publication':result['attempts'][0]['publication']['request_id']='wrong'
    if mutation=='missing_audit':result['canonical_path_hash']=''
    if mutation=='missing_points':result['points']=[]
    workers[0].events.append({'event':'result','value':result})
    assert not f.result(timeout=1)['final_valid_success']


def test_cancelled_active_worker_retires_before_ready(service):
    s,workers=service;f=s.submit(req('a'));until(lambda:bool(workers[0].sent))
    assert s.cancel('a')
    r=f.result(timeout=1);assert r['failure_code']=='USER_CANCELLED' and r['retirement_confirmed']
    until(lambda:len(workers)==2 and s.status()['state']=='READY')
    assert workers[0].retired and not workers[1].sent


def test_future_cancel_also_cancels_native_work(service):
    s,workers=service;f=s.submit(req('a'));until(lambda:bool(workers[0].sent))
    assert f.cancel()
    until(lambda:workers[0].retired)
    assert f.cancelled()


def test_uncertain_result_cannot_reuse_worker(service):
    s,workers=service;f=s.submit(req('a'));until(lambda:bool(workers[0].sent))
    workers[0].events.append({'event':'result','value':{'final_valid_success':False,
                            'failure_code':'EXACT_ACK_FAILED_CLOSED','session_recovery_required':True}})
    assert not f.result(timeout=1)['final_valid_success']
    until(lambda:len(workers)==2 and s.status()['state']=='READY')
    assert workers[0].retired


def test_expired_queue_request_never_reaches_worker(service):
    s,workers=service;s.submit(req('a'));until(lambda:bool(workers[0].sent))
    f=s.submit(req('b'))
    with s.lock:s.pending[0].deadline=time.monotonic()-1
    assert f.result(timeout=1)['failure_code']=='REQUEST_DEADLINE_IN_QUEUE'
    assert len(workers[0].sent)==1


def test_late_valid_result_is_rejected_and_preserved(service):
    s,workers=service;f=s.submit(req('a'));until(lambda:bool(workers[0].sent))
    with s.lock:
        s.active.deadline=time.monotonic()-1
        workers[0].events.append({'event':'result','value':valid('a')})
    r=f.result(timeout=1)
    assert not r['final_valid_success'] and not r['points']
    assert r['failure_code']=='REQUEST_DEADLINE'


def test_unconfirmed_retirement_forbids_replacement(service):
    s,workers=service;workers[0].confirmed=False
    f=s.submit(req('a'));until(lambda:bool(workers[0].sent));s.cancel('a')
    assert not f.result(timeout=1)['final_valid_success']
    until(lambda:s.status()['state']=='FAILED')
    assert len(workers)==1


def test_resource_limit_stops_service_without_search(service):
    s,workers=service;workers[0].memory=11*1024**3
    until(lambda:s.status()['state']=='FAILED')
    assert workers[0].retired and not workers[0].sent


@pytest.mark.parametrize('value',[float('nan'),float('inf'),True,'1',None])
def test_invalid_pose_never_queues(service,value):
    s,_=service;r=req('a');r['start'][0]=value
    with pytest.raises(ValueError):s.submit(r)
    assert s.status()['queued']==0


def test_journal_failure_never_strands_future_or_skips_retirement(service,monkeypatch):
    from pathlib import Path
    s,workers=service;f=s.submit(req('a'));until(lambda:bool(workers[0].sent))
    original=Path.open
    def fail_log(path,*args,**kwargs):
        if path.name=='service_events.jsonl':raise OSError('injected disk failure')
        return original(path,*args,**kwargs)
    monkeypatch.setattr(Path,'open',fail_log)
    workers[0].events.append({'event':'result','value':valid('a')})
    r=f.result(timeout=1)
    assert not r['final_valid_success'] and r['failure_code']=='SERVICE_JOURNAL_UNAVAILABLE'
    until(lambda:workers[0].retired and s.status()['state']=='FAILED')
    assert s.status()['journal_error'] and not s.status()['ready_receipt']


def test_output_capacity_stops_without_deleting_evidence(service,monkeypatch):
    s,workers=service
    preserved=s.output/'preserved_failure.log';preserved.write_text('failure evidence')
    monkeypatch.setattr(s,'_output_usage',lambda:(s.output_limit_bytes+1,1))
    until(lambda:s.status()['state']=='FAILED',timeout=2.)
    assert workers[0].retired and preserved.read_text()=='failure evidence'
