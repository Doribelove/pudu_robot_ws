from types import SimpleNamespace as S
import numpy as np
from three_yd_v5.v9_local import plan
from arena_3d_v1.semantic_world import digest


def setup():
    mask=np.ones((180,180),bool);world=S(key='same',hard=~mask,vehicle=(.265,.225,.4),map=S(resolution=.05,origin=(0.,0.)))
    c=S(world=world,dynamic=None,plan=S(corridor_mask=mask),initial_l2_result=S(success=True),
        reference_plan=S(world_key='same',fine_corridor_hash=digest(mask),world_xy=np.array([[1.,2.],[4.,2.],[4.,5.]])),
        _target_mask=lambda:mask,trajectory=None,trajectory_version=None,_bounded_grid=S(diagnostics={}))
    q=S(query_id='corner',start=(1.,2.,0.),goal=(4.,5.,np.pi/2));s=S(bound_vehicle=world.vehicle)
    ca=S(metrics={},diagnostics=lambda:{},final_valid_success=True)
    return c,q,s,S(backend='frozen',version='frozen'),ca


def test_preflight_solves_real_corner_then_actual_ack_and_complete_audit(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_MODE','A');monkeypatch.setenv('THREEYD_V9_R2_MODE','P')
    c,q,s,spec,ca=setup();calls=[]
    c.sync_only=lambda s:(calls.append('ACK') or {'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0})
    c._audit=lambda *a:(calls.append('full_audit') or (ca,{'valid':True},{'valid':True},0.,[]))
    out=plan(c,q,s,None,spec)
    assert out['success'] and calls==['ACK','full_audit']
    assert out['v9_local']['analytic_preflight'] and out['v9_local']['connections']
    assert all(call['anchor_fit']['method'] in ('analytic_G2_quintic','compound_G2_corner','guided_G2') for call in out['v9_local']['connections'])
    assert all(not call['Smac_called'] for call in out['v9_local']['connections'])


def test_no_complete_geometry_cannot_publish_ack_or_claim_physical_blockage(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_MODE','A');monkeypatch.setenv('THREEYD_V9_R2_MODE','P')
    c,q,s,spec,ca=setup();calls=[]
    from three_yd_v5 import v9_guided_joint
    monkeypatch.setattr(v9_guided_joint,'guided_connect',lambda *a,**k:(None,{'success':False,'failure':'NO_VALID_ANALYTIC_JOINT'}))
    c.sync_only=lambda s:calls.append('ACK');c._audit=lambda *a:calls.append('audit')
    out=plan(c,q,s,None,spec)
    assert not out['success'] and not calls and c.trajectory is None
    assert out['failure_code']=='V9_FAST_PREFLIGHT_NO_COMPLETE_GEOMETRY'
    assert not out['v9_local']['actual_map_ACK_performed']


def test_preflight_cannot_bypass_failed_ack_or_final_audit(monkeypatch):
    monkeypatch.setenv('THREEYD_V9_MODE','A');monkeypatch.setenv('THREEYD_V9_R2_MODE','P')
    c,q,s,spec,ca=setup();calls=[]
    c.sync_only=lambda s:{'costmap_update_acknowledged':False};c._audit=lambda *a:calls.append('audit')
    out=plan(c,q,s,None,spec);assert not out['success'] and not calls
    c.sync_only=lambda s:{'costmap_update_acknowledged':True}
    c._audit=lambda *a:(ca,{'valid':False},{'valid':True},0.,['BODY_REJECTED'])
    out=plan(c,q,s,None,spec);assert not out['success'] and c.trajectory is None


def test_matching_preparation_is_lazy_but_actual_candidates_are_swept(monkeypatch):
    from test_v9_l3 import fake
    from three_yd_v5.v9_segments import propose,store
    from three_yd_v5 import v9_segments
    monkeypatch.setenv('THREEYD_V9_MODE','AB');monkeypatch.setenv('THREEYD_V9_R2_MODE','M')
    c=fake();p=np.c_[np.arange(1.,31.,.05),np.full(600,2.5),np.zeros(600)];mask=np.ones(c.world.hard.shape,bool)
    calls=[];old=v9_segments.SweptChecker
    def counted(*a):calls.append('constructed');return old(*a)
    monkeypatch.setattr(v9_segments,'SweptChecker',counted)
    _,log=propose(c,p,mask);assert not calls and log['matched_segments']==0
    store(c,{'success':True,'result':S(points=[dict(zip(('x','y','yaw'),v)) for v in p])})
    _,log=propose(c,p,mask);assert calls==['constructed'] and log['matched_segments']>0
    calls.clear();_,log=propose(c,p,np.zeros_like(mask));assert calls==['constructed'] and log['matched_segments']==0


def test_analytic_long_connection_does_not_inherit_old_optimizer_state_limit():
    import time
    from three_yd_v5.v9_joints import connect
    from arena_3d_v1.semantic_world import SweptChecker
    world=S(vehicle=(.265,.225,.4));mask=np.zeros((200,500),np.uint8)
    checker=SweptChecker(mask,.05,(0.,0.),*world.vehicle[:2])
    p=np.array([[1.,2.,0.],[1.1,2.,0.],[15.1,2.,.02],[15.2,2.002,.02]])
    q,d=connect(world,p,1,2,checker,time.monotonic()+1,maximum_length=17.7,sample_limit=8192)
    assert d['success'] and q is not None and d['length_m']>6
    assert np.array_equal(q[0],p[1]) and np.array_equal(q[-1],p[2])
