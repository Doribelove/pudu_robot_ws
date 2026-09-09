import math
import json
import time
from types import SimpleNamespace

from arena_evaluation.reachable_endpoint_r3 import ReachableEndpointSelector
from test_reachable_endpoint_r3 import _map, _topology, _query, FOOTPRINT


def fixture(monkeypatch):
    topology=_topology(_map(),[(1,[(2.,4.),(8.,4.)])])
    selector=ReachableEndpointSelector(topology,FOOTPRINT)
    start={'component':1,'point':(2.,4.),'turn_rad':0.,'length_m':1.,'failure':''}
    goal={'component':1,'point':(3.,4.),'turn_rad':4*math.pi,'length_m':20.,'failure':'','hybrid_attempted':True}
    records=[goal]+[{'component':1,'point':(x,4.),'tangent_yaw':0.,'failure':'ENDPOINT_LOCAL_SE2_NO_PATH'} for x in (3.1,4.2,5.4,6.6,7.8)]
    monkeypatch.setattr(selector,'_route',lambda a,b:(object(),(start,goal)) if b and b[0] is goal else (None,None))
    monkeypatch.setattr(selector.connector,'free_component',lambda _:None)
    ss={'attempts':0,'expanded':0,'generated':0};gs={'attempts':1,'expanded':2500,'generated':1}
    return selector,start,goal,records,ss,gs


def test_quality_refinement_shares_original_four_attempt_cap(monkeypatch):
    s,start,goal,records,ss,gs=fixture(monkeypatch);calls=[]
    def fail(a,b,deadline):
        calls.append((a,b,deadline));return None,{'expanded':2500,'generated':1}
    monkeypatch.setattr(s.connector,'connect',fail)
    end=time.monotonic()+5.;goals=[goal]
    result=s._improve_detours(SimpleNamespace(start=(2.,4.,0.),goal=(8.,4.,0.)),[start],goals,[],records,ss,gs,end)
    assert len(calls)==3 and gs['attempts']==4 and gs['expanded']==10000
    assert all(math.dist(a[:2],goal['point'])>=1. for a,_,_ in calls)
    assert all(deadline<=end-2. for _,_,deadline in calls)
    assert goals==[goal] and result['quality_connector_attempts']==3


def test_quality_exhaustion_does_not_discard_existing_certificate(monkeypatch):
    s,start,goal,records,ss,gs=fixture(monkeypatch)
    monkeypatch.setattr(s.connector,'connect',lambda *args:(None,{'expanded':1,'generated':1,'budget_exhausted':True}))
    goals=[goal]
    result=s._improve_detours(SimpleNamespace(start=(2.,4.,0.),goal=(8.,4.,0.)),[start],goals,[],records,ss,gs,time.monotonic()+5.)
    assert goals==[goal] and result['quality_connector_budget_exhausted']
    assert all(not r.get('stats',{}).get('budget_exhausted') for r in records)
    assert any(r.get('stats',{}).get('quality_budget_exhausted') for r in records)


def test_remaining_request_budget_is_reserved_for_smac(monkeypatch):
    s,start,goal,records,ss,gs=fixture(monkeypatch)
    def forbidden(*a):raise AssertionError('quality refinement consumed reserved L3 time')
    monkeypatch.setattr(s.connector,'connect',forbidden)
    result=s._improve_detours(SimpleNamespace(start=(2.,4.,0.),goal=(8.,4.,0.)),[start],[goal],[],records,ss,gs,time.monotonic()+1.9)
    assert result['quality_connector_attempts']==0


def test_cache_hit_reports_no_new_quality_search():
    topology=_topology(_map(),[(1,[(2.,4.),(8.,4.)])])
    selector=ReachableEndpointSelector(topology,FOOTPRINT);query=_query()
    assert selector(topology,query)[2] is not None
    key=next(iter(selector.cache));payload=json.loads(selector.cache[key])
    payload['certificate']['diagnostics']['quality_connector_attempts']=2
    payload['certificate']['diagnostics']['quality_connector_budget_exhausted']=True
    selector.cache[key]=json.dumps(payload)
    timing={};assert selector(topology,query,timing=timing)[2] is not None
    assert timing['endpoint_connector_cache_hit']
    assert timing['quality_connector_attempts']==0 and not timing['quality_connector_budget_exhausted']
    assert selector.last_certificate['diagnostics']['quality_connector_attempts']==2
