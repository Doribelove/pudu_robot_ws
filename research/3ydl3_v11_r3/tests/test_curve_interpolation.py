from types import SimpleNamespace as S
import math
import numpy as np
import pytest
from arena_3d_v1 import semantic_world as parent
from three_yd_v5 import curve_interpolation as ci
from three_yd_v5.curve_records import repack_or_reuse
from three_yd_v5.v9_r9_records import pack_internal

@pytest.mark.parametrize('seed',range(12))
def test_interpolation_matches_original_bits_and_interval_counts(seed):
    rng=np.random.RandomState(seed)
    p=np.c_[np.cumsum(rng.normal(0,.06,(100,2)),axis=0),rng.uniform(-math.pi,math.pi,100)]
    p[1]=p[0];p[2,2]=p[1,2]+math.pi
    ds=[.025,.05,.1][seed%3];dyaw=[math.pi/180,math.pi/90][seed%2]
    a=ci.ORIGINAL(p,ds,dyaw);b=ci.exact_interpolation(p,ds,dyaw)
    assert a.shape==b.shape
    assert np.array_equal(a.view(np.uint64),b.view(np.uint64))

def test_interpolation_cache_rehashes_mutable_input_and_detaches_output():
    ci.clear();p=np.array([[1.,2.,0.],[4.,2.,0.]])
    a=ci.fast_densify(p);a[0,0]=99
    b=ci.fast_densify(p);assert b[0,0]==1 and ci.stats()['hits']==1
    p[0,1]=2.1;c=ci.fast_densify(p)
    assert ci.stats()['misses']==2 and c[0,1]==2.1
    ci.fast_densify(p,ds=.05);assert ci.stats()['misses']==3
    assert ci.stats()['audit_or_ACK_results_cached'] is False

def test_reuse_of_pose_interpolation_does_not_reuse_body_permission(monkeypatch):
    ci.clear();monkeypatch.setattr(parent,'densify',ci.ORIGINAL)
    monkeypatch.setenv('THREEYD_L3_BACKEND','independent');assert ci.install()
    p=np.array([[1.1,2.,0.],[5.1,2.,0.]])
    hard=np.zeros((140,160),bool)
    w=S(hard=hard,map=S(resolution=.05,origin=(0,0,0)),vehicle=(.265,.225,.4),
        sample_labels=lambda v:np.zeros(len(v),int),features={},rules={})
    q=S(start=tuple(p[0]),goal=tuple(p[-1]))
    calls=[];original_check=parent.SweptChecker.check
    def check(self,poses):calls.append(1);return original_check(self,poses)
    monkeypatch.setattr(parent.SweptChecker,'check',check)
    assert parent.SemanticWorld.audit(w,q,p,np.ones_like(hard))['valid']
    hard[99,75]=True
    rejected=parent.SemanticWorld.audit(w,q,p,np.ones_like(hard))
    assert not rejected['valid'] and 'SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT' in rejected['failures']
    assert len(calls)==2 and ci.stats()['hits']==1

def test_control_dispatch_keeps_original_computation(monkeypatch):
    ci.clear();monkeypatch.setenv('THREEYD_L3_BACKEND','control')
    p=np.array([[1.,2.,0.],[4.,2.,0.]])
    assert np.array_equal(ci.dispatch(p),ci.ORIGINAL(p))
    assert ci.stats()['hits']==ci.stats()['misses']==0

def test_guarded_traffic_aliases_share_only_interpolation(monkeypatch):
    from three_yd_v4 import guidance
    from three_yd_v5 import controller
    ci.clear();monkeypatch.setattr(parent,'densify',ci.ORIGINAL)
    monkeypatch.setattr(guidance,'densify',ci.ORIGINAL)
    monkeypatch.setattr(controller,'densify',ci.ORIGINAL)
    monkeypatch.setenv('THREEYD_L3_BACKEND','independent');assert ci.install()
    p=np.array([[1.,2.,0.],[4.,2.,0.]])
    expected=ci.ORIGINAL(p)
    for function in (parent.densify,guidance.densify,controller.densify):
        actual=function(p);assert np.array_equal(actual.view(np.uint64),expected.view(np.uint64))
    assert ci.stats()['misses']==1 and ci.stats()['hits']==2
    assert set(ci.stats()['guarded_aliases'])=={'three_yd_v4.guidance','three_yd_v5.controller'}
    custom=lambda *args:None;monkeypatch.setattr(guidance,'densify',custom)
    assert ci.install() and guidance.densify is custom

def test_checked_point_repack_preserves_bits_and_mutable_delivery(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_R9_MODE','PLMX')
    points=[{'x':-0.,'y':2.,'yaw':0.,'source':'curve'},{'x':1.,'y':2.,'yaw':0.,'source':'curve'}]
    source=pack_internal(points);delivery=source.packed.decode()
    assert repack_or_reuse(delivery,source) is source
    delivery[0]['x']=0.;changed=repack_or_reuse(delivery,source)
    assert changed is not source and not np.signbit(changed.packed.columns[0][1][0])
    assert np.signbit(source.packed.columns[0][1][0])
    delivery[0]['source']='changed';changed=repack_or_reuse(delivery,source)
    assert changed is not source
