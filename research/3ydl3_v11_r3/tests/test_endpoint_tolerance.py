from dataclasses import replace
from types import SimpleNamespace as S
import math
import time
import numpy as np
import pytest
from arena_evaluation.planner_benchmark.models import Query
from three_yd_v5 import endpoint_tolerance as et
from three_yd_v5.v10_storage import collision_checker
from test_curve_l3 import world


def service():
    w=world()
    s=S(world=w,controller=None,record=None,remaining=None,retained_record=None,retained_path=None)
    s.session=S(_curve_request={'deadline':time.monotonic()+10,'new_used_s':1.,'config':{'new_l3_budget_s':15.}})
    s.path=None;s.current_trajectory=lambda:s.path
    return s


def query():return Query('q',(2.,2.,0.),(8.,2.,0.))


def test_exact_success_never_searches_candidates(monkeypatch):
    s=service();q=query();calls=[]
    monkeypatch.setattr(et,'candidates',lambda *_:pytest.fail('fallback on exact success'))
    out=et.execute(s,q,lambda x:calls.append(x) or {'success':True})
    assert calls==[q] and out['exact_endpoint_success'] and not out['tolerant_endpoint_success']


@pytest.mark.parametrize('failure',['CURVE_MAP_ACK_FAILED','REFERENCE_BINDING_MISMATCH',
    'MAP_VERSION_CHANGED','CURVE_BUDGET','COSTMAP_CONTENT_CORRUPT','UNEXPECTED_INTERNAL_FAILURE'])
def test_infrastructure_or_budget_errors_cannot_trigger_tolerance(monkeypatch,failure):
    s=service();q=query()
    monkeypatch.setattr(et,'candidates',lambda *_:pytest.fail('invalid retry'))
    out=et.execute(s,q,lambda _:dict(success=False,failure_code=failure))
    assert not out['success'] and len(out['endpoint_tolerance']['attempts'])==1


def test_fallback_retains_original_query_and_shared_clocks(monkeypatch):
    s=service();q=query();r=replace(q,start=(2.12,2.,math.radians(-30)),goal=(8.,2.2,math.radians(30)))
    clock=s.session._curve_request;calls=[]
    monkeypatch.setattr(et,'candidates',lambda *_:iter([r]))
    def solve(x):
        calls.append(x);assert s.session._curve_request is clock
        clock['new_used_s']+=1
        if x is q:return {'success':False,'failure_code':'CURVE_NO_CERTIFIED_SOLUTION'}
        s.path=np.asarray([x.start,x.goal]);s.query=x
        return {'success':True,'legacy_contract_success':True}
    out=et.execute(s,q,solve)
    assert calls==[q,r] and clock['new_used_s']==3
    assert q.start==(2.,2.,0.) and q.goal==(8.,2.,0.)
    assert out['tolerant_endpoint_success'] and not out['legacy_contract_success']
    assert out['endpoint_tolerance']['offsets']['goal']['position_m']==pytest.approx(.2)
    assert out['endpoint_tolerance']['offsets']['start']['yaw_deg']==pytest.approx(30)


def test_actual_delivered_pose_not_just_candidate_is_checked(monkeypatch):
    s=service();q=query();r=replace(q,goal=(8.1,2.,0.))
    monkeypatch.setattr(et,'candidates',lambda *_:iter([r]))
    def solve(x):
        if x is q:return {'success':False,'failure_code':'CURVE_NO_CERTIFIED_SOLUTION'}
        s.path=np.array([x.start,(8.2001,2.,0.)])
        return {'success':True}
    with pytest.raises(ValueError,match='OUTSIDE_PERMISSION'):et.execute(s,q,solve)


def test_wraparound_angle_and_euclidean_disk():
    assert et.deviation((1,1,math.radians(179)),(1,1,math.radians(-151)))['yaw_deg']==pytest.approx(30)
    s=service();cfg=et.policy();checker=collision_checker(s.world.hard,s.world)
    options=et.pose_options(query().start,checker,cfg,time.monotonic()+2)
    assert options and any(et.deviation(query().start,p)['position_m']>.199 for _,p in options)
    for _,p in options:
        d=et.deviation(query().start,p)
        assert d['position_m']<=.2+1e-12 and d['yaw_deg']<=30+1e-12


def test_both_invalid_footprints_can_be_relocated_without_body_shrink():
    s=service();q=query();w=s.world
    # Obstacles overlap the right edge of each rectangle. The centers remain
    # clear; candidate screening must use the complete vehicle footprint.
    for x in (2.24,8.24):
        w.hard[300-1-int(2/.05),int(x/.05)]=True
    checker=collision_checker(w.hard,w)
    assert not checker.check(np.asarray([q.start])) and not checker.check(np.asarray([q.goal]))
    poses=list(et.candidates(s,q,et.policy()))
    assert poses
    assert w.vehicle==(.265,.225,.4)
    for p in poses:
        for role in ('start','goal'):
            assert checker.check(np.asarray([getattr(p,role)]))
            assert et.deviation(getattr(q,role),getattr(p,role))['position_m']<=.2+1e-9


def test_blocked_disk_is_truthful_failure_and_withdraws_old_path():
    s=service();s.world.hard[:]=True;q=query()
    s.controller=S(trajectory=np.ones((2,3)),trajectory_version=0)
    s.record={'old':True};s.retained_record={'old':True}
    out=et.execute(s,q,lambda _:dict(success=False,failure_code='STATIC_FOOTPRINT_COLLISION'))
    assert not out['success'] and out['endpoint_tolerance']['resolved'] is None
    assert s.record is None and s.retained_record is None and s.controller.trajectory is None


def test_budget_is_not_refilled_after_exact_failure(monkeypatch):
    s=service();q=query();calls=[]
    monkeypatch.setattr(et,'candidates',lambda *_:iter([replace(q,goal=(8.1,2.,0.))]))
    def solve(x):
        calls.append(x);s.session._curve_request['new_used_s']=15
        return {'success':False,'failure_code':'CURVE_NO_CERTIFIED_SOLUTION'}
    assert not et.execute(s,q,solve)['success'] and calls==[q]


def test_update_cannot_accumulate_offsets():
    s=service();q=query();s.query=replace(q,goal=(8.19,2.,0.));s.path=np.array([q.start,s.query.goal])
    s.endpoint_resolution={}
    out=et.after_update(s,{'success':True},q)
    assert out['endpoint_tolerance']['offsets']['goal']['position_m']==pytest.approx(.19)
    s.query=replace(q,goal=(8.21,2.,0.));s.path=np.array([q.start,s.query.goal])
    with pytest.raises(ValueError,match='OUTSIDE_PERMISSION'):et.after_update(s,{'success':True},q)


def test_tolerance_cannot_exceed_permission(monkeypatch):
    original=et.json.loads
    monkeypatch.setattr(et.json,'loads',lambda raw:{**original(raw),'yaw_deg':31.})
    with pytest.raises(ValueError,match='PERMISSION_LIMIT'):et.policy()


def test_committed_parking_progress_cannot_be_reinterpreted_as_fresh_task(monkeypatch):
    s=service();q=query();s.prefix=np.ones((10,3));s.endpoint_resolution={}
    monkeypatch.setattr(et,'candidates',lambda *_:pytest.fail('lost traffic history'))
    out=et.execute(s,q,lambda _:dict(success=False,failure_code='CURVE_NO_CERTIFIED_SOLUTION'))
    assert out['endpoint_tolerance_skip']=='COMMITTED_TASK_PROGRESS_CONTEXT'
    s._plan=lambda _:pytest.fail('lost traffic history during update')
    out=et.after_update(s,{'success':False,'failure_code':'CURVE_NO_CERTIFIED_SOLUTION'},q)
    assert not out['success']


def test_known_invalid_parking_candidate_is_rejected_not_treated_as_permission():
    assert et.eligible('PARKING_OWNERSHIP_UNRESOLVED')
    assert et.eligible('PARKING_BINDING_POSITION_MISMATCH')
    assert not et.eligible('REFERENCE_BINDING_MISMATCH')


def test_exact_failed_update_is_not_executed_twice(monkeypatch):
    s=service();q=query();s.query=q;r=replace(q,goal=(8.1,2.,0.));calls=[]
    s.endpoint_resolution={}
    monkeypatch.setattr(et,'candidates',lambda *_:iter([r]))
    def solve(x):
        calls.append(x);s.query=x;s.path=np.array([x.start,x.goal]);return {'success':True}
    s._plan=solve
    out=et.after_update(s,{'success':False,'failure_code':'CURVE_NO_CERTIFIED_SOLUTION','wall_ms':5000},q)
    assert calls==[r] and out['tolerant_endpoint_success']
    assert out['endpoint_tolerance']['attempts'][0]['source']=='already_executed_exact_update'
