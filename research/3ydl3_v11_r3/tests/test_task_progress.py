from dataclasses import replace
from types import SimpleNamespace as S
from collections import deque
import threading
import numpy as np
import pytest
from three_yd_v5.task_progress import match_pose,ProgressTraffic,TaskProgressService
from test_v8_r7_fullroute import fake_controller


def test_match_does_not_snap_or_ignore_heading():
    p=np.array([[0,0,0],[1,0,0],[2,0,0]],float)
    assert match_pose(p,(1,0,0))[0]==1
    assert match_pose(p,(1.001,0,0))[0] is None
    assert match_pose(p,(1,0,.01))[0] is None
    assert match_pose(p,(1,0,2*np.pi))[0]==1


def test_ambiguous_visit_rejected():
    p=np.array([[0,0,0],[1,0,0],[2,0,0],[1,0,0]])
    assert match_pose(p,(1,0,0))==(None,'AMBIGUOUS_ROUTE_VISIT')


def test_adjacent_duplicate_not_a_branch():
    assert match_pose(np.array([[0,0,0],[1,0,0],[1,0,0],[2,0,0]]),(1,0,0))[0]==1


def test_original_traffic_sees_prefix_and_still_rejects_suffix():
    seen=[]
    base=S(audit=lambda p:(seen.append(p.copy()) or {'valid':False,'failures':['WRONG_TRAVEL_DIRECTION']}),endpoint_parking={38})
    t=ProgressTraffic(base,[[0,0,0]])
    r=t.audit([[1,0,0],[.5,0,np.pi]])
    assert r['failures']==['WRONG_TRAVEL_DIRECTION'] and not r['valid']
    assert np.array_equal(seen[0],np.array([[0,0,0],[1,0,0],[.5,0,np.pi]]))
    assert t.endpoint_parking=={38}


def fake_service():
    c,q,session,record=fake_controller()
    q=S(query_id='q',start=q.start,goal=q.goal)
    p=np.array([[.5,.5,0],[.75,.5,0],[1,.5,0]])
    record['result']=replace(record['result'],points=[dict(zip(('x','y','yaw'),v)) for v in p])
    service=TaskProgressService.__new__(TaskProgressService)
    service.lock=threading.RLock();service.dynamic=S(lock=threading.RLock(),version=0)
    c.dynamic=service.dynamic;c.trajectory=p.copy();c.trajectory_version=0;c.timing={'historical_ms':99999}
    service.controller=c;service.query=q;service.task_query=q;service.record=record;service.route='r'
    service.continuation=True;service.remaining=p.copy();service.prefix=np.empty((0,3))
    service.base_traffic=S(audit=lambda p:{'valid':True,'failures':[]});service.session=session
    service.auditor=None;service.progress_m=0.
    service._solve=lambda q:{'success':False,'failure_code':'FRESH_PLAN_REQUIRED'}
    return service,q,p


def test_continuation_audits_actual_suffix_and_drops_old_clocks():
    s,q,p=fake_service();seen=[];old=s.controller._audit
    s.controller._audit=lambda query,poses,*a:(seen.append((query,poses.copy())) or old(query,poses,*a))
    current=S(query_id=q.query_id,start=tuple(p[1]),goal=q.goal)
    r=s.plan(current)
    assert r['success'] and r['task_progress']['reused']
    assert np.array_equal(seen[0][1],p[1:]) and seen[0][0] is current
    assert r['candidate_recovery']['l1_actual_ms']==0
    assert 'historical_ms' not in s.controller.timing
    assert np.array_equal(s.prefix,p[:1])


@pytest.mark.parametrize('change',['goal','id','backtrack','pose','heading'])
def test_incompatible_request_requires_fresh_plan(change):
    s,q,p=fake_service()
    if change=='backtrack':
        s.plan(S(query_id=q.query_id,start=tuple(p[1]),goal=q.goal))
    x=S(query_id=q.query_id,start=q.start,goal=q.goal)
    if change=='goal':x.goal=(2,.5,0)
    elif change=='id':x.query_id='new'
    elif change=='pose':x.start=(.501,.5,0)
    elif change=='heading':x.start=(.5,.5,.1)
    r=s.plan(x)
    assert not r['success'] and not r['task_progress']['reused']


def test_ack_failure_withdraws_current_trajectory():
    s,q,p=fake_service();s.controller.sync_only=lambda _: {'costmap_update_acknowledged':False}
    r=s.plan(S(query_id=q.query_id,start=tuple(p[1]),goal=q.goal))
    assert not r['success'] and s.current_trajectory() is None


def test_context_prefix_never_supplies_body_or_curvature_permission():
    s,q,p=fake_service();old=s.controller._audit
    s.controller._audit=lambda *a:(*old(*a)[:4],['SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT'])
    r=s.plan(S(query_id=q.query_id,start=tuple(p[1]),goal=q.goal))
    assert not r['success'] and s.current_trajectory() is None
