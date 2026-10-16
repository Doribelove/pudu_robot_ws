from types import SimpleNamespace as S
from copy import deepcopy
import numpy as np
import pytest
from arena_evaluation.unified_four_backends_smoke import PlanResult
from three_yd_v5.fullroute import RouteCache,record_from,validated_reuse,binding
from three_yd_v5.fullroute_service import impact
from arena_3d_v1.semantic_world import digest


def test_cache_copy_and_byte_limit():
    cache=RouteCache(2,2000)
    value={'points':np.zeros((10,3))}
    assert cache.put('a',value)
    value['points'][0,0]=9
    got=cache.get('a');assert got['points'][0,0]==0
    got['points'][0,0]=8;assert cache.get('a')['points'][0,0]==0
    assert not cache.put('b',{'huge':np.zeros(10000)})
    assert cache.bytes<=2000 and len(cache.items)==1


def test_cache_lru():
    c=RouteCache(2,10000)
    c.put('a',1);c.put('b',2);c.get('a');c.put('c',3)
    assert c.get('b') is None and c.get('a')==1 and c.evictions==1


def test_only_success_and_no_historical_clocks():
    r=PlanResult(points=[{'x':0.,'y':0.,'yaw':0.}],diagnostics={'planning_time_ms':999},expanded_states=500)
    assert record_from({'success':False,'result':r}) is None
    saved=record_from({'success':True,'result':r})
    assert saved['result'].diagnostics=={} and saved['result'].expanded_states==0
    assert r.expanded_states==500


def fake_controller():
    mask=np.ones((30,30),bool)
    w=S(key='w',map=S(sha256='s',resolution=.05,origin=(0,0),occupancy=np.zeros(mask.shape,np.int8)),
        vehicle=(.265,.225,.4),hard=~mask,labels=np.ones(mask.shape,np.int16),rules={},semantic=S(semantic_map_hash='s'))
    q=S(start=(.5,.5,0.),goal=(1.,.5,0.))
    ref=S(world_key='w',world_xy=np.array([[.5,.5],[1.,.5]]),fine_corridor_hash=digest(mask))
    ca=S(final_valid_success=True,metrics={'length_m':.5})
    c=S(world=w,dynamic=None,reference_plan=ref,plan=S(corridor_mask=mask),
        max_search_cells=1000,max_map_fraction=1.,topology_route=S(signature='r'),weight=2.,
        trajectory=None,trajectory_version=None,_bounded_grid=S(diagnostics={}),
        _target_mask=lambda:mask,sync_only=lambda s:{'costmap_update_acknowledged':True},
        _audit=lambda *a:(ca,{'valid':True},{'valid':True},0.,[]))
    points=[{'x':.5,'y':.5,'yaw':0.},{'x':1.,'y':.5,'yaw':0.}]
    return c,q,S(bound_vehicle=w.vehicle),{'result':PlanResult(points=points)}


def test_reuse_zero_backend_calls_and_current_audit():
    c,q,s,r=fake_controller();out=validated_reuse(c,q,s,None,r)
    assert out['success'] and out['diagnostics']['backend_call_count']==0
    assert np.array_equal(c.trajectory,np.array([[.5,.5,0],[1,.5,0]]))


def test_ack_failure_stops_before_audit():
    c,q,s,r=fake_controller();c.sync_only=lambda s:{'costmap_update_acknowledged':False}
    c._audit=lambda *a:pytest.fail('must not audit with unconfirmed map')
    out=validated_reuse(c,q,s,None,r)
    assert not out['success'] and 'ACK' in out['failure_code'] and c.trajectory is None


def test_collision_rejection_never_publishes():
    c,q,s,r=fake_controller();old=c._audit
    c._audit=lambda *a:(*old(*a)[:4],['SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT'])
    out=validated_reuse(c,q,s,None,r)
    assert not out['success'] and c.trajectory is None


def test_midcall_map_change_rejected():
    c,q,s,r=fake_controller();c.dynamic=S(version=1)
    def audit(*a):
        c.dynamic.version=2
        return S(final_valid_success=True,metrics={}),{'valid':True},{'valid':True},0.,[]
    c._audit=audit
    out=validated_reuse(c,q,s,None,r)
    assert 'MAP_VERSION' in out['failure_code'] and c.trajectory is None


@pytest.mark.parametrize('field',['occupancy','hard','labels','goal','vehicle','reference','rules'])
def test_real_content_changes_invalidate(field):
    c,q,s,r=fake_controller();before=binding(c,q,s,'s',True,140)
    if field=='occupancy':c.world.map.occupancy[0,0]=100
    elif field=='hard':c.world.hard[0,0]=True
    elif field=='labels':c.world.labels[0,0]=2
    elif field=='goal':q.goal=(1.1,.5,0.)
    elif field=='vehicle':c.world.vehicle=(.3,.25,.5)
    elif field=='reference':c.reference_plan.world_xy[0,0]=.6
    else:c.world.rules={'one_way':['changed']}
    assert before!=binding(c,q,s,'s',True,140)


def test_impact_includes_reference_body_margin_and_ignores_removal():
    c,q,s,r=fake_controller()
    c.world.cells=lambda p:(np.asarray(p[:,1]/.05,int),np.asarray(p[:,0]/.05,int))
    c._search_mask=np.ones((30,30),bool)
    added=np.zeros((30,30),bool);added[12,15]=True
    result=impact(c,added,[12,13,15,16]);assert result['reference_affected']
    assert impact(c,np.zeros_like(added),[12,13,15,16])['kind']=='no_new_blockage'


def test_service_does_not_publish_partial_result():
    import threading
    from three_yd_v5.fullroute_service import FullRouteService
    s=FullRouteService.__new__(FullRouteService);s.lock=threading.RLock()
    s.dynamic=S(lock=threading.RLock(),version=0)
    s.record=None;s.controller=S(current_trajectory=lambda:np.ones((2,3)))
    assert s.current_trajectory() is None


def test_current_path_version_is_immediately_invalidated():
    import threading
    from three_yd_v5.fullroute_service import FullRouteService
    s=FullRouteService.__new__(FullRouteService)
    s.dynamic=S(lock=threading.RLock(),version=1)
    s.record={};s.controller=S(trajectory_version=0,trajectory=np.ones((2,3)))
    assert s.current_trajectory() is None
    s.controller.trajectory_version=1
    got=s.current_trajectory();assert np.array_equal(got,s.controller.trajectory)
    got[:]=0;assert s.controller.trajectory.sum()==6


def test_event_history_is_bounded_and_does_not_retain_diagnostic_payload():
    from collections import deque
    from three_yd_v5.fullroute_service import FullRouteService
    s=FullRouteService.__new__(FullRouteService);s.events=deque(maxlen=32)
    for i in range(100):s._remember({'action':'check','update':{'version':i},'diagnostics':{'large':np.ones(10000)}})
    assert len(s.events)==32 and s.events[0]['update']['version']==68
    assert all('diagnostics' not in e for e in s.events)


def test_nondefault_cost_cap_uses_original_planning(monkeypatch):
    from three_yd_v5.fullroute import plan
    monkeypatch.setenv('THREEYD_V8_FULLROUTE_REUSE','1')
    c=S(_plan_l3_uncached=lambda *a,**kw:{'cap':kw['cap']})
    assert plan(c,None,None,None,None,cap=180)=={'cap':180}
