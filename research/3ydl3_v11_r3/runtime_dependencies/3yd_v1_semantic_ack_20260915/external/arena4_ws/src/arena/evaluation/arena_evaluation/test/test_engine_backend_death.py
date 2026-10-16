from types import SimpleNamespace
import time
import pytest
from arena_evaluation import static_planner_engine_r3 as module


def engine_fixture():
    engine=module.StaticPlannerEngine(SimpleNamespace(query_budget_s=7.),'/tmp/unused-pln-test')
    engine.state='READY';engine._planner_identity=(12345,678)
    engine.session=SimpleNamespace(planner_pid=12345,_unresolved_timeout=False,
        _budget_state_uncertain=False,context_uncertain=False)
    engine.ctx=engine.spec=engine.auditor=engine.preparer=None
    return engine


@pytest.mark.parametrize('row',[None,{'pid':12345,'start_ticks':678,'state':'Z'},
                                {'pid':12345,'start_ticks':679,'state':'S'}])
def test_dead_or_reused_backend_prevents_search_and_invalidates_session(monkeypatch,row):
    engine=engine_fixture();called=[]
    monkeypatch.setattr(module,'process_stat',lambda pid:row,raising=False)
    monkeypatch.setattr(module.runner,'run_query',lambda *a,**kw:called.append(1) or
        {'final_valid_success':False,'failure_code':'ACK_TIMEOUT','points':[]})
    result=engine.plan({'request_id':'death','start':[0,0,0],'goal':[1,1,0]},time.monotonic()+7)
    assert not called
    assert result['failure_code']=='BACKEND_PROCESS_DIED'
    assert result['session_recovery_required'] is True and result['points']==[]
    assert result['planner_search_started'] is False and engine.state=='FAILED'


def test_backend_death_during_request_cannot_deliver_path(monkeypatch):
    engine=engine_fixture();table=[{'pid':12345,'start_ticks':678,'state':'S'}]
    monkeypatch.setattr(module,'process_stat',lambda pid:table[0],raising=False)
    def run(*args,**kwargs):
        table[0]=None
        return {'final_valid_success':True,'failure_code':'','points':[{'x':1}],
                'attempts':[{'planner_search_started':True}]}
    monkeypatch.setattr(module.runner,'run_query',run)
    result=engine.plan({'request_id':'death','start':[0,0,0],'goal':[1,1,0]},time.monotonic()+7)
    assert result['failure_code']=='BACKEND_PROCESS_DIED'
    assert result['session_recovery_required'] is True and not result['final_valid_success']
    assert result['points']==[] and result['rejected_candidate_points']==[{'x':1}]
    assert engine.state=='FAILED'


def test_idle_worker_reports_backend_death_without_waiting_for_request(monkeypatch,tmp_path):
    import pickle
    from arena_evaluation import static_planner_service_r3 as service
    calls=[];sent=[]
    class Engine:
        def __init__(self,*args):pass
        def start(self):return {'state':'READY'}
        def backend_available(self):return False
        def close(self):calls.append('engine_closed')
    class Connection:
        def send_bytes(self,value):sent.append(pickle.loads(value))
        def recv_bytes(self,*args):raise AssertionError('MUST_NOT_WAIT_FOR_REQUEST')
        def close(self):calls.append('connection_closed')
    monkeypatch.setattr(module,'StaticPlannerEngine',Engine)
    monkeypatch.setattr(module,'EngineConfig',lambda **kw:kw)
    monkeypatch.setattr(service.tempfile,'tempdir',service.tempfile.tempdir)
    monkeypatch.setenv('TMPDIR',str(tmp_path))
    service._engine_worker(Connection(),{},tmp_path/'out',str(tmp_path))
    assert sent[0]['event']=='ready'
    assert sent[1]['event']=='error' and 'BACKEND_PROCESS_DIED' in sent[1]['error']
    assert calls==['engine_closed','connection_closed']
