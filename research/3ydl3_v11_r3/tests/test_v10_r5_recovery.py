"""Ownership, candidate coverage and budget guards for r5 recovery."""
from types import SimpleNamespace as S
import gc
import weakref
import pytest
from three_yd_v5.recovery import V5Runner


def exercise(monkeypatch,mode,early_success=False,first_failure='CURVE_NO_CERTIFIED_SOLUTION'):
    monkeypatch.setenv('THREEYD_V10_R4_MODE','FB')
    monkeypatch.setenv('THREEYD_V10_R5_MODE',mode)
    monkeypatch.setenv('THREEYD_V6_LAZY_RESUME','1')
    routes=[S(signature=str(i),region_ids=(1,),start=(0,0,0),goal=(1,0,0)) for i in range(1,4)]
    ctx={'endpoint_access':{k:{'semantic_region_id':1,'road_id':1} for k in ('start','goal')},'roads':[(1,(1,0))]}
    topology=S(routes=lambda q,limit:routes[:limit],contexts={r.signature:ctx for r in routes},regions={1:{'kind':'lane'}})
    world=S(map=S(sha256='map'),semantic=S(semantic_map_hash='sem'),vehicle=(.265,.225,.4))
    session=S(_curve_request={'new_used_s':0.});calls=[];refs=[];alive_at_construction=[]
    class Controller:
        def __init__(self,w,t,r,**kwargs):
            alive_at_construction.append(sum(x() is not None for x in refs))
            refs.append(weakref.ref(self))
            self.topology_route=r;self.timing={};self.lifecycle=S(clear=lambda:None)
            self.r4_semantic_body=kwargs.get('r4_semantic_body',False)
        def plan_l3(self,q,s,a,spec):
            phase='early' if s._r5_speculative_seed else ('seed' if s._r4_seed_phase else 'primary')
            calls.append((self.topology_route.signature,phase));s._curve_request['new_used_s']+=.1
            ok=early_success and phase=='early' and self.topology_route.signature=='2'
            failure=first_failure if self.topology_route.signature=='1' and phase=='primary' else 'CURVE_NO_CERTIFIED_SOLUTION'
            return {'success':ok,'failure_code':'' if ok else failure}
    runner=V5Runner(world,topology,None,{},controller_class=Controller)
    c,r,out=runner.plan(S(start=(0,0,0),goal=(1,0,0)),session,None,None)
    return calls,alive_at_construction,out


def test_failed_speculation_retains_all_original_candidates_and_full_recovery(monkeypatch):
    calls,alive,out=exercise(monkeypatch,'ME')
    assert calls==[('1','primary'),('1','early'),('2','early'),('3','early'),
                   ('2','primary'),('3','primary'),('1','seed'),('2','seed'),('3','seed')]
    assert not out['success']
    assert len(out['candidate_recovery']['attempts'])==9
    assert all(n==0 for n in alive)


def test_early_success_only_stops_after_controller_strict_success(monkeypatch):
    calls,alive,out=exercise(monkeypatch,'ME',True)
    assert calls==[('1','primary'),('1','early'),('2','early')]
    assert out['success'] and out['candidate_recovery']['attempts'][-1]['success']


def test_replaced_controller_released_before_next_construction(monkeypatch):
    calls,alive,out=exercise(monkeypatch,'M')
    assert calls==[('1','primary'),('2','primary'),('3','primary'),('1','seed'),('2','seed'),('3','seed')]
    assert all(n==0 for n in alive)


def test_late_curve_failure_does_not_trigger_extra_speculative_round(monkeypatch):
    calls,alive,out=exercise(monkeypatch,'ME',first_failure='CURVE_STRICT_REJECTED:WRONG_TRAVEL_DIRECTION')
    assert not any(phase=='early' for _,phase in calls)
    assert [route for route,phase in calls if phase=='primary']==['1','2','3']
    assert [route for route,phase in calls if phase=='seed']==['2','3']


@pytest.mark.parametrize('mode',['MLSE','MLSEK'])
def test_modes_keep_physical_constraints_and_total_budgets(monkeypatch,mode):
    from three_yd_v5.curve_backend import settings
    monkeypatch.setenv('THREEYD_V10_R5_MODE','control');base=settings()[1]
    monkeypatch.setenv('THREEYD_V10_R5_MODE',mode);candidate=settings()[1]
    assert all(candidate[k]==v for k,v in base.items() if k!='r5_mode')
    assert candidate['total_request_budget_s']==30. and candidate['new_l3_budget_s']==15.
    assert candidate['r5_early_seed']


def test_speculative_seed_has_one_attempt_with_original_deadline(monkeypatch):
    from three_yd_v5 import curve_l3
    import time
    calls=[]
    def solver(*args):
        calls.append(args);return None,None,{'failure':'CURVE_NO_CERTIFIED_SOLUTION','wall_ms':1.}
    monkeypatch.setattr(curve_l3,'_optimize_once',solver)
    deadline=time.monotonic()+.2
    cfg=dict(curve_l3.CONFIG,r4_failure_recovery=True,r4_recovery_only=True,r5_speculative_seed=True)
    curve_l3.optimize(None,None,None,None,None,deadline,cfg)
    assert len(calls)==1 and calls[0][-2]<=deadline
    assert calls[0][-1]['tracking_weight']==curve_l3.CONFIG['tracking_weight']
