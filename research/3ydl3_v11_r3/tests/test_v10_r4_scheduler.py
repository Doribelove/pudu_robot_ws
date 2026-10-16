from types import SimpleNamespace as S
import time
import pytest
from three_yd_v5.recovery import V5Runner
from three_yd_v5 import curve_l3


@pytest.mark.parametrize('success_primary',[True,False])
def test_all_original_routes_precede_seed_recovery(monkeypatch,success_primary):
    monkeypatch.setenv('THREEYD_V10_R4_MODE','F')
    monkeypatch.setenv('THREEYD_V6_LAZY_RESUME','1')
    routes=[S(signature=str(i),region_ids=(1,),start=(0,0,0),goal=(1,0,0)) for i in range(1,4)]
    ctx={'endpoint_access':{k:{'semantic_region_id':1,'road_id':1} for k in ('start','goal')},'roads':[(1,(1,0))]}
    topology=S(routes=lambda query,limit:routes[:limit],contexts={r.signature:ctx for r in routes},regions={1:{'kind':'lane'}})
    world=S(map=S(sha256='map'),semantic=S(semantic_map_hash='sem'),vehicle=(.265,.225,.4))
    session=S(_curve_request={'new_used_s':0.})
    calls=[]
    class Controller:
        def __init__(self,w,t,r,**kwargs):
            self.topology_route=r;self.timing={};self.lifecycle=S(clear=lambda:None)
        def plan_l3(self,q,s,a,spec):
            phase=s._r4_seed_phase;calls.append((self.topology_route.signature,phase))
            s._curve_request['new_used_s']+=.1
            ok=(success_primary and self.topology_route.signature=='3' and not phase) or (not success_primary and self.topology_route.signature=='2' and phase)
            return {'success':ok,'failure_code':'' if ok else 'CURVE_NO_CERTIFIED_SOLUTION'}
    runner=V5Runner(world,topology,None,{},controller_class=Controller)
    c,r,out=runner.plan(S(start=(0,0,0),goal=(1,0,0)),session,None,None)
    assert out['success']
    assert calls==([('1',False),('2',False),('3',False)] if success_primary else
                   [('1',False),('2',False),('3',False),('1',True),('2',True)])
    assert out['candidate_recovery']['max_region_routes']==3


def test_successful_primary_spline_is_untouched(monkeypatch):
    result=(object(),{'original':True},{'wall_ms':12.,'success':True})
    calls=[]
    def solver(*args):calls.append(args);return result
    monkeypatch.setattr(curve_l3,'_optimize_once',solver)
    now=time.monotonic();cfg=dict(curve_l3.CONFIG,r4_failure_recovery=True)
    actual=curve_l3.optimize(None,None,None,None,None,now+15,cfg)
    assert actual is result and len(calls)==1
    assert now<calls[0][-2]<=now+3.1


def test_recovery_cannot_extend_existing_deadline(monkeypatch):
    deadlines=[]
    def solver(*args):
        deadlines.append(args[-2]);return None,None,{'failure':'CURVE_NO_CERTIFIED_SOLUTION','wall_ms':1.}
    monkeypatch.setattr(curve_l3,'_optimize_once',solver)
    deadline=time.monotonic()+.2
    curve_l3.optimize(None,None,None,None,None,deadline,dict(curve_l3.CONFIG,r4_failure_recovery=True,r4_recovery_only=True))
    assert len(deadlines)==2 and all(x<=deadline for x in deadlines)
