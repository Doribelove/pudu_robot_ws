import math
from types import SimpleNamespace
import pytest
from arena_evaluation.request_budget_r3 import retry_admission


def test_observed_large_roi_retry_rejected_before_costmap():
    result=retry_admission(.8115, confirmed_search_s=5.,
                           corridor_ms=194.5, costmap_ms=617.2)
    assert not result['admitted']
    assert result['required_s'] > 3.
    assert result['minimum_search_s'] == .5


def test_confirmed_small_budget_avoids_unnecessary_reconfiguration_reserve():
    result=retry_admission(1.2, confirmed_search_s=.5, corridor_ms=20., costmap_ms=100.)
    assert result['admitted']
    assert result['server_budget_strategy']=='reuse_confirmed'
    assert result['required_s']==pytest.approx(.95)


def test_reconfiguration_and_nonzero_preparation_are_both_reserved():
    result=retry_admission(2., confirmed_search_s=5., corridor_ms=20., costmap_ms=100.)
    assert not result['admitted']
    assert result['required_s']==pytest.approx(2.2)
    assert retry_admission(2.21,confirmed_search_s=5.,corridor_ms=20.,costmap_ms=100.)['admitted']


def test_observed_slow_reconfiguration_increases_required_budget():
    fast=retry_admission(4., confirmed_search_s=5.)
    slow=retry_admission(4., confirmed_search_s=5., reconfigure_peak_s=2.)
    assert slow['required_s']>fast['required_s']


@pytest.mark.parametrize('value',[-.1,math.inf,math.nan])
def test_invalid_remaining_budget_is_rejected(value):
    with pytest.raises(ValueError):retry_admission(value)


def test_runner_does_not_construct_or_publish_rejected_retry(monkeypatch):
    from arena_evaluation import two_layer_v1_r3_benchmark as runner
    clock=SimpleNamespace(now=0.)
    monkeypatch.setattr(runner.time,'monotonic',lambda:clock.now)
    monkeypatch.setattr(runner.baseline,'_session_log_cursor',lambda s:0)
    monkeypatch.setattr(runner.baseline,'_session_log_delta',lambda *a:'')
    monkeypatch.setattr(runner.baseline,'_parse_smac_benchmark_metrics',lambda x:{})
    monkeypatch.setattr(runner.baseline,'_classify_smac_failure',lambda *a:('SMAC_MAX_ITERATIONS',None,''))
    class Session:
        _enforce_server_budget=True
        _confirmed_server_budget=5.
        calls=0
        builds=0
        def begin_request(self,*a):pass
        def plan(self,*a,**kw):
            self.calls+=1;clock.now=6.19
            return runner.legacy.PlanResult(failure_code='NO_PATH',diagnostics={
                'planner_search_started':True,'total_costmap_update_ms':617.2})
    session=Session()
    def corridor(*a,**kw):
        session.builds+=1;clock.now+=.1945;return None,{}
    session._corridor_cache=SimpleNamespace(get=corridor)
    ctx=SimpleNamespace(map_id='synthetic',hospital_map=SimpleNamespace(width=100,height=100))
    query=SimpleNamespace(query_id='arbitrary-id')
    selector=lambda *a,**k:(None,None,SimpleNamespace(polyline=[(0.,0.),(1.,1.)]),'')
    result=runner.run_query(ctx,query,None,selector,session,SimpleNamespace(),None,'request')
    assert session.calls==session.builds==1
    assert result['failure_code']=='REQUEST_BUDGET_TOO_SMALL_FOR_RETRY'
    assert result['primary_failure']=='SMAC_MAX_ITERATIONS'
    assert result['retry_rejected_before_corridor']
    assert not result['corridor_retry_used']
    assert result['l3_calls']==1
