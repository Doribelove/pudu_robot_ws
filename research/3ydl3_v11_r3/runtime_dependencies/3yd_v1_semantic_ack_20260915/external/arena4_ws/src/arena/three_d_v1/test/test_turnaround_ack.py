import math,time
from types import SimpleNamespace
import numpy as np
import pytest
from arena_3d_v1.curvature_repair import fit_connector,violating_segments,truthful_points,ordered_occurrences,TurnaroundController
from arena_3d_v1.recovery_pipeline import RecoveryController
from arena_3d_v1.stable_session import StableSmacSession
from arena_evaluation.semantic_constraint_core import dubins_choices


def test_connector_preserves_endpoint_pose_and_curvature_without_metadata_clipping():
    start=np.array([3.,3.,0.]);goal=np.array([3.,3.8,math.pi])
    valid=None
    for word,params in dubins_choices(start,goal,.6):
        try:p,stats=fit_connector(start,goal,word,params,.6,0.,0.,time.monotonic()+2,2.5)
        except ValueError:continue
        bad,_=violating_segments(p,2.5)
        if not len(bad):valid=p;break
    assert valid is not None
    assert np.max(abs(valid[0]-start))<1e-12
    assert np.linalg.norm(valid[-1,:2]-goal[:2])<1e-12
    source=SimpleNamespace(source='smac',planner_backend='smac',backend_version='fixed')
    points=truthful_points(valid,source,0,len(valid)-1)
    delta=np.diff(valid,axis=0);yaw=np.arctan2(np.sin(delta[:,2]),np.cos(delta[:,2]))
    expected=np.arctan(.5*yaw/np.linalg.norm(delta[:,:2],axis=1))
    assert np.allclose([x['steering'] for x in points[:-1]],expected)
    assert np.max(abs(np.diff([x['steering'] for x in points])))<=math.radians(15)+1e-6
    assert all(x['planner_backend']=='bounded_forward_curvature_fit' for x in points)


def test_fit_budget_and_state_cap_are_enforced():
    with pytest.raises(TimeoutError,match='BUDGET'):
        fit_connector((0,0,0),(1,0,0),'LSL',(0,2,0),.5,0,0,time.monotonic()-1,2.5)
    with pytest.raises(ValueError,match='STATE_BUDGET'):
        fit_connector((0,0,0),(100,0,0),'LSL',(0,200,0),.5,0,0,time.monotonic()+1,2.5)


def test_repair_window_includes_unclipped_steering_discontinuity():
    p=[np.zeros(3)]
    for k in [1.,1.,-1.,-1.]:
        x,y,h=p[-1];delta=k*.05
        p.append(np.array([x+(math.sin(h+delta)-math.sin(h))/k,
                           y+(-math.cos(h+delta)+math.cos(h))/k,h+delta]))
    bad,_=violating_segments(np.asarray(p),2.5)
    assert 1 in bad and 2 in bad


@pytest.mark.parametrize('failure',[['SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT'],['REVERSE'],['CURVATURE','REVERSE']])
def test_noncurvature_failures_do_not_enter_repair(monkeypatch,failure):
    outcome={'success':False,'result':object(),'independent_audit':{'failures':failure}}
    monkeypatch.setattr(RecoveryController,'plan_l3',lambda *a,**k:outcome)
    controller=object.__new__(TurnaroundController)
    assert controller.plan_l3(None,None,None,None) is outcome
    assert 'local_curvature_repair' not in outcome


def test_unacknowledged_map_cannot_enter_repair(monkeypatch):
    outcome={'success':False,'result':object(),'independent_audit':{'failures':['CURVATURE']},'diagnostics':{'costmap_update_acknowledged':False}}
    monkeypatch.setattr(RecoveryController,'plan_l3',lambda *a,**k:outcome)
    assert object.__new__(TurnaroundController).plan_l3(None,None,None,None) is outcome
    assert 'local_curvature_repair' not in outcome


def test_repeated_region_occurrence_is_not_a_set_membership_check():
    region=np.array([[1,2,3]])
    world=SimpleNamespace(cells=lambda p:(np.zeros(len(p),int),np.clip(p[:,0].astype(int),0,2)))
    topology=SimpleNamespace(world=world,subregions=region)
    route=SimpleNamespace(region_ids=(1,2,1))
    assert not ordered_occurrences(topology,route,np.array([[0,0,0],[1,0,0]]))[0]
    assert ordered_occurrences(topology,route,np.array([[0,0,0],[1,0,0],[0,0,0]]))[0]


def test_lost_first_readback_is_cancelled_and_next_request_can_succeed(monkeypatch):
    import arena_3d_v1.stable_session as module
    clock=[0.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    response=SimpleNamespace(map=SimpleNamespace(metadata=SimpleNamespace(size_x=2,size_y=1,update_time=SimpleNamespace(sec=1,nanosec=0)),data=[254,254]))
    class Future:
        def __init__(self,ready):self.ready=ready;self.cancelled=False
        def done(self):return self.ready
        def cancel(self):self.cancelled=True
        def result(self):return response
    class Client:
        def __init__(self):self.futures=[];self.removed=[]
        def wait_for_service(self,timeout_sec):return True
        def call_async(self,_):
            future=Future(bool(self.futures));self.futures.append(future);return future
        def remove_pending_request(self,future):self.removed.append(future)
    s=object.__new__(StableSmacSession);s._get_costmap_client=Client();s.readbacks=[]
    s.GetCostmap=SimpleNamespace(Request=lambda:None);s.ctx=SimpleNamespace(hospital_map=SimpleNamespace(width=2,height=1))
    def spin(timeout_sec):clock[0]+=max(.001,timeout_sec)
    s.client=SimpleNamespace(executor=SimpleNamespace(spin_once=spin))
    with pytest.raises(RuntimeError,match='attempt timed out'):s._server_costmap_snapshot(3.)
    assert clock[0]<=.751 and s._get_costmap_client.futures[0].cancelled
    assert len(s._get_costmap_client.removed)==1
    grid,stamp=s._server_costmap_snapshot(3.)
    assert grid.tolist()==[[254,254]] and stamp==1_000_000_000
    assert s.readbacks[0]['pending_request_removed'] and s.readbacks[1]['success']


def test_readback_shape_mismatch_remains_failure(monkeypatch):
    s=object.__new__(StableSmacSession);s.readbacks=[];s.ctx=SimpleNamespace(hospital_map=SimpleNamespace(width=2,height=1))
    future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(map=SimpleNamespace(metadata=SimpleNamespace(size_x=3,size_y=1))))
    s._get_costmap_client=SimpleNamespace(wait_for_service=lambda **k:True,call_async=lambda _:future)
    s.GetCostmap=SimpleNamespace(Request=lambda:None);s.client=object()
    with pytest.raises(RuntimeError,match='shape mismatch'):s._server_costmap_snapshot(time.monotonic()+3)


@pytest.mark.parametrize('apply_reassertion',[True,False])
def test_large_cold_initialization_gap_is_reasserted_but_never_assumed_applied(monkeypatch,apply_reassertion):
    import arena_3d_v1.stable_session as module
    clock=[0.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    s=object.__new__(StableSmacSession);s.costmap_ack_timeout_s=.3;s._costmap_ack_sequence=0
    master=np.zeros((100,100),np.uint8);published=[]
    def spin(timeout_sec):clock[0]+=max(.001,timeout_sec)
    s.client=SimpleNamespace(executor=SimpleNamespace(spin_once=spin))
    s._server_costmap_snapshot=lambda deadline:(master.copy(),int(clock[0]*1e9))
    def publish(expected,changed):
        published.append(int(changed.sum()))
        if apply_reassertion:master[:]=254
        return expected.copy(),{'roi_message_count':1,'roi_published_cells':int(changed.sum())}
    s._publish_dirty_roi=publish
    result=s._mask_content_ack(np.full((100,100),100,np.int8),np.ones((100,100),bool))
    assert 1<=len(published)<=2 and published[0]==10000
    assert result['costmap_update_acknowledged'] is apply_reassertion
    if apply_reassertion:assert result['costmap_ack_stable_observations']==2
    else:assert result['costmap_ack_mismatch_cells']==10000 and s._costmap_ack_sequence==0


@pytest.mark.parametrize('apply_reassertion',[True,False])
def test_large_open_mask_gap_requires_server_evidence_and_preserves_inflation(monkeypatch,apply_reassertion):
    import arena_3d_v1.stable_session as module
    clock=[0.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    s=object.__new__(StableSmacSession);s.costmap_ack_timeout_s=.3;s._costmap_ack_sequence=0
    expected=np.zeros((100,100),np.int8);expected[:10]=100
    master=np.full(expected.shape,254,np.uint8);published=[]
    def spin(timeout_sec):clock[0]+=max(.001,timeout_sec)
    s.client=SimpleNamespace(executor=SimpleNamespace(spin_once=spin))
    s._server_costmap_snapshot=lambda deadline:(master.copy(),int(clock[0]*1e9))
    def publish(values,changed):
        published.append(changed.copy())
        if apply_reassertion:master[changed]=200  # opened but inflated
        return values.copy(),{'roi_message_count':1,'roi_published_cells':int(changed.sum())}
    s._publish_dirty_roi=publish
    result=s._mask_content_ack(expected,np.ones(expected.shape,bool))
    assert 1<=len(published)<=2 and published[0].sum()==9000
    assert not published[0][:10].any()
    assert result['costmap_update_acknowledged'] is apply_reassertion
    assert clock[0]<=.301
    if apply_reassertion:
        assert result['costmap_ack_status']=='mask_content_verified'
        assert result['costmap_ack_stable_observations']==2
    else:assert result['costmap_ack_mismatch_cells']==9000 and s._costmap_ack_sequence==0
