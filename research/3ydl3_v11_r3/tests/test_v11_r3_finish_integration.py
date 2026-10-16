from contextlib import contextmanager
from types import SimpleNamespace as S, MethodType
import pytest
from three_yd_v5 import curve_backend,curve_budget,endpoint_tolerance,tail_filehash,request_heap
from three_yd_v5.task_progress import TaskProgressService


@pytest.mark.parametrize('mode',['independent','control'])
def test_finish_hook_is_inside_complete_request_clock_and_configuration_guard(monkeypatch,mode):
    calls=[]
    @contextmanager
    def guard(*args):
        calls.append('clock-start')
        try:yield
        finally:calls.append('clock-stop')
    @contextmanager
    def settings_scope():
        try:yield
        finally:calls.append('verify-config')
    @contextmanager
    def file_scope():yield
    monkeypatch.setattr(curve_budget,'request_guard',guard)
    monkeypatch.setattr(curve_backend,'settings',lambda:(mode,{'total_request_budget_s':30.},'same-config'))
    monkeypatch.setattr(curve_backend,'settings_scope',settings_scope)
    monkeypatch.setattr(tail_filehash,'request_scope',file_scope)
    monkeypatch.setattr(curve_backend,'request_start',lambda s:calls.append('request-start'))
    def finish(s,out):calls.append('verify-result');return out
    monkeypatch.setattr(curve_backend,'request_finish',finish)
    monkeypatch.setattr(endpoint_tolerance,'execute',lambda service,q,plan:plan(q))
    def release():calls.append('heap-release');return {'triggered':True}
    monkeypatch.setattr(request_heap,'release_retired_heap',release)
    def plan(q):calls.append('plan');return {'success':True}
    service=S(_plan=plan,_retain=lambda:calls.append('retain'),controller=None)
    service._guarded_plan=MethodType(TaskProgressService._guarded_plan,service)
    out=TaskProgressService.plan(service,None)
    assert out['success']
    expected=['clock-start','request-start','plan','verify-result','retain']
    if mode=='independent':expected+=['heap-release'];assert out['r11_request_heap']['triggered']
    else:assert 'r11_request_heap' not in out
    assert calls==expected+['verify-config','clock-stop']
