import gc,time,weakref
from types import SimpleNamespace as S
import numpy as np
from three_yd_v5.v10_lifecycle import lifecycle
from three_yd_v5 import curve_l3 as l3

def world():
    return S(hard=np.zeros((300,300),bool),map=S(resolution=.05,origin=(0.,0.,0.)),vehicle=(.265,.225,.4),key='test')

def test_callback_does_not_keep_retired_controller_alive():
    class Owner:
        def __init__(self):self.calls=0;self.lifecycle=lifecycle(self.clear)
        def clear(self):self.calls+=1
    gc.collect();gc.disable()
    try:
        o=Owner();cb=o.lifecycle.clear;cb();assert o.calls==1
        ref=weakref.ref(o);del o;assert ref() is None;cb()
    finally:gc.enable()

def test_feasible_quadratic_candidate_has_full_curve_certificate(monkeypatch):
    import three_yd_v5.curve_qp as qp
    monkeypatch.setattr(qp,'clarabel_setup',lambda *a,**k: (_ for _ in ()).throw(AssertionError('unneeded QP')))
    w=world();p,m,d=l3.optimize(w,~w.hard,[[2,3],[12,3]],(2,3,0),(12,3,0),time.monotonic()+5,dict(l3.CONFIG,certified_quadratic_start=True))
    assert p is not None and d['accepted_quadratic_initialization'] and not d['iterations']
    assert d['certificate']['intervals']>0
    np.testing.assert_allclose(p[[0,-1]],[[2,3,0],[12,3,0]],atol=1e-10)

def test_rejected_initial_certificate_preserves_original_sqp(monkeypatch):
    w=world();args=(w,~w.hard,[[2,3],[12,3]],(2,3,0),(12,3,0))
    old,_,_=l3.optimize(*args,time.monotonic()+5,dict(l3.CONFIG,certified_quadratic_start=False))
    cert=l3.certify;calls=[]
    def reject_first(*a,**k):
        calls.append(1)
        if len(calls)==1:raise l3.CurveFailure('CURVE_INTERVAL_CURVATURE_OR_REGULARITY')
        return cert(*a,**k)
    monkeypatch.setattr(l3,'certify',reject_first)
    new,_,d=l3.optimize(*args,time.monotonic()+5,dict(l3.CONFIG,certified_quadratic_start=True))
    assert not d['accepted_quadratic_initialization'];assert len(calls)>=2
    np.testing.assert_array_equal(new,old)

def test_strict_rejection_of_initial_curve_uses_original_sqp(monkeypatch,tmp_path):
    from three_yd_v5 import curve_backend as backend,geometry
    from three_yd_v5.v9_maps import content_digest
    from arena_evaluation.planner_benchmark.models import Query
    monkeypatch.setenv('THREEYD_L3_BACKEND','independent')
    w=world();mask=~w.hard;guide=np.array([[2.,3.],[12.,3.]])
    monkeypatch.setattr(geometry,'variable_neighborhood',lambda *a,**k:(mask,{}))
    audit_calls=[]
    ca=S(metrics={},diagnostics=lambda:{})
    def audit(*args):
        audit_calls.append(1)
        return ca,{}, {'valid':len(audit_calls)>1},0.,(['WRONG_TRAVEL_DIRECTION'] if len(audit_calls)==1 else [])
    c=S(world=w,dynamic=None,reference_plan=S(world_key=w.key,fine_corridor_hash=content_digest(mask),world_xy=guide),plan=S(corridor_mask=mask),corner_records=[],max_search_cells=1000000,max_map_fraction=1.,root=tmp_path,timing={},_bounded_grid=S(diagnostics={}),_target_mask=lambda:mask,sync_only=lambda session:{'costmap_update_acknowledged':True},_audit=audit)
    s=S(bound_vehicle=w.vehicle);q=Query('test',(2.,3.,0.),(12.,3.,0.))
    result=backend.plan(c,q,s,None,None)
    assert result['success'] and len(audit_calls)==2
    assert 'quadratic_strict_fallback' in result['curve_l3']
    assert not result['curve_l3']['accepted_quadratic_initialization']
