from types import SimpleNamespace as S
import time
import pytest
from three_yd_v5.request_schedule import scheduled_plan
from three_yd_v5 import curve_backend


@pytest.mark.parametrize('success,yielded,failure,cost,retries',[
    (True,True,'',1.,False),(False,True,'CURVE_BUDGET',1.,True),
    (False,False,'CURVE_NO_CERTIFIED_SOLUTION',1.,False),
    (False,True,'CURVE_MAP_ACK_FAILED',1.,False),
    (False,True,'CURVE_BUDGET',15.,False)])
def test_fallback_preserves_consumed_budget_and_all_attempts(monkeypatch,success,yielded,failure,cost,retries):
    monkeypatch.setattr(curve_backend,'settings',lambda:('independent',{'new_l3_budget_s':15.},'same-config'))
    state={'deadline':time.monotonic()+30,'new_used_s':0.,'attempts':[]}
    deadline=state['deadline'];session=S(_curve_request=state);calls=[]
    def execute(*args):
        calls.append(getattr(session,'_r11_primary_slice_s',None))
        assert session._curve_request is state and state['deadline']==deadline
        if len(calls)==2:assert state['new_used_s']==cost
        state['new_used_s']+=cost if len(calls)==1 else .2
        state['attempts'].append({'r11_primary_yielded':yielded})
        return None,None,{'success':success or len(calls)==2,'failure_code':failure,
            'candidate_recovery':{'attempts':[{'call':len(calls)}],'l1_actual_ms':3.,'wall_ms':10.}}
    runner=S(topology=S(mode='semantic'),_plan_original=execute)
    out=scheduled_plan(runner,None,session,None,None)[2]
    assert calls==([1.,None] if retries else [1.])
    assert not hasattr(session,'_r11_primary_slice_s')
    assert out['r11_request_schedule']['original_schedule_retried']==retries
    if retries:
        assert out['candidate_recovery']['attempts']==[{'call':1},{'call':2}]
        assert out['candidate_recovery']['l1_actual_ms']==6.
        assert state['new_used_s']==cost+.2


def test_exception_clears_private_schedule_without_changing_shared_budget(monkeypatch):
    monkeypatch.setattr(curve_backend,'settings',lambda:('independent',{'new_l3_budget_s':15.},'x'))
    session=S(_curve_request={'attempts':[],'deadline':100.,'new_used_s':3.})
    def fail(*args):raise ValueError('CONTENT_VERSION_CHANGED')
    with pytest.raises(ValueError,match='CONTENT_VERSION_CHANGED'):
        scheduled_plan(S(topology=S(mode='semantic'),_plan_original=fail),None,session,None,None)
    assert not hasattr(session,'_r11_primary_slice_s')
    assert session._curve_request['new_used_s']==3.
