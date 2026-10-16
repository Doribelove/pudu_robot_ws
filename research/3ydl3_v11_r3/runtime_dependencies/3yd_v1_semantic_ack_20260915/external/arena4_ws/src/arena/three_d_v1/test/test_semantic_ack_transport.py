import time
from types import SimpleNamespace
import numpy as np
import pytest
from arena_3d_v1.semantic_ack_session import PacedPublisher,SemanticAckSession
from arena_3d_v1.stable_session import StableSmacSession
from arena_evaluation.unified_four_backends_smoke import SmacSession


def test_pacing_is_actual_elapsed_time_even_with_ready_callbacks(monkeypatch):
    import arena_3d_v1.semantic_ack_session as mod
    clock=[0.];monkeypatch.setattr(mod.time,'monotonic',lambda:clock[0])
    def spin(timeout_sec):clock[0]+=.0001  # callback returns far before timeout
    session=SimpleNamespace(tile_interval_s=.015,_exact_repair_context=None,tile_events=[],current_query_id='x',_semantic_publication_sequence=1,client=SimpleNamespace(executor=SimpleNamespace(spin_once=spin)))
    sent=[];pub=PacedPublisher(session,SimpleNamespace(publish=lambda m:sent.append(clock[0])),'initial')
    message=SimpleNamespace(x=0,y=0,width=1,height=1,data=[0])
    for _ in range(3):pub.publish(message)
    assert min(np.diff(sent))>=.015
    assert len(session.tile_events)==3


def test_repair_publication_cannot_exceed_ack_deadline(monkeypatch):
    import arena_3d_v1.semantic_ack_session as mod
    monkeypatch.setattr(mod.time,'monotonic',lambda:3.)
    session=SimpleNamespace(tile_interval_s=.015,_exact_repair_context={'repairing':True,'deadline':2.})
    pub=PacedPublisher(session,SimpleNamespace(publish=lambda m:pytest.fail('published after deadline')),'repair')
    with pytest.raises(RuntimeError,match='deadline'):pub.publish(None)


def fixture_session(monkeypatch,apply):
    import arena_3d_v1.semantic_ack_session as mod
    clock=[0.];monkeypatch.setattr(mod.time,'monotonic',lambda:clock[0])
    session=object.__new__(SemanticAckSession);semantic=SimpleNamespace(policy_hash='p')
    session._semantic_costmap=semantic;session._content_signature=lambda s:('p','g','m');session._semantic_publication_sequence=7
    session.enable_semantic_repair=True;session.tile_interval_s=0.;session.tile_events=[];session.semantic_repair_events=[]
    session.current_query_id='q';session.inflation_radius_m=.55;session.ctx=SimpleNamespace(hospital_map=SimpleNamespace(resolution=.05))
    master=np.zeros((40,40),np.uint8);master[15:25,15:25]=95
    target=np.zeros_like(master);expected=np.zeros(master.shape,np.int8)
    session._exact_repair_context={'semantic':semantic,'signature':('p','g','m'),'expected':expected,'master':target,'affected':np.ones(master.shape,bool),
      'sequence':7,'deadline':3.,'started':0.,'repairs':0,'repairing':False,'last_bad':None,'same_observations':0,'last_send':0.,'next_probe_at':.4,'events':[]}
    def snapshot(self,deadline):clock[0]+=.15;return master.copy(),int(clock[0]*1e9)
    monkeypatch.setattr(StableSmacSession,'_server_costmap_snapshot',snapshot)
    session._local_update_publisher=object();calls=[]
    def low_level(self,grid,dirty):
        calls.append(dirty.copy())
        if apply:master[dirty]=target[dirty]
        return grid.copy(),{'roi_message_count':1,'roi_published_cells':int(dirty.sum())}
    monkeypatch.setattr(SmacSession,'_publish_dirty_roi',low_level)
    return session,master,calls

@pytest.mark.parametrize('apply',[True,False])
def test_gap_reassertion_keeps_sequence_and_requires_actual_new_content(monkeypatch,apply):
    session,master,calls=fixture_session(monkeypatch,apply)
    seen=[]
    for _ in range(8):seen.append(session._server_costmap_snapshot(3.)[0])
    assert 1<=len(calls)<=2
    assert session._semantic_publication_sequence==7
    assert np.any(seen[1]!=0)  # publishing is never converted into a matching readback
    assert bool(np.all(seen[-1]==0)) is apply
    assert calls[0][3:37,3:37].all()  # includes 0.55 m inflation support + one cell


def test_changed_publication_cannot_be_repaired(monkeypatch):
    session,master,calls=fixture_session(monkeypatch,True);session._semantic_publication_sequence+=1
    with pytest.raises(RuntimeError,match='publication changed'):session._server_costmap_snapshot(3.)
    assert not calls
