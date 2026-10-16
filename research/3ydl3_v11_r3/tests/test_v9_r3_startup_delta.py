import time
import numpy as np
import pytest
from arena_3d_v1.semantic_ack_session import SemanticAckSession
from arena_evaluation.semantic_smac_session_r2 import ExactSemanticSmacSessionR2
from arena_3d_v1.stable_session import StableSmacSession
from three_yd_v5.r6_session import MapFlowMixin
from three_yd_v5.v9_r3_session import StartupDeltaReader,StartupDeltaPerformanceSession

def fake(monkeypatch):
 monkeypatch.setenv('THREEYD_V9_R3_MODE','I');monkeypatch.setenv('THREEYD_V8_R6_MODE','ABC')
 s=StartupDeltaPerformanceSession.__new__(StartupDeltaPerformanceSession)
 s.readbacks=[];s._r6_fast_readback=False;s._exact_repair_context=None;s.enable_semantic_repair=True
 data=np.arange(24,dtype=np.uint8).reshape(4,6)
 def raw(self,deadline):
  assert self._r6_fast_readback
  self.readbacks.append({'real_read':True});return data,381
 monkeypatch.setattr(MapFlowMixin,'_server_costmap_snapshot',raw)
 return s,data

def test_immutable_real_startup_snapshot_and_flag_restoration(monkeypatch):
 s,data=fake(monkeypatch)
 actual,stamp=StartupDeltaReader._server_costmap_snapshot(s,time.monotonic()+1)
 assert np.array_equal(actual,data) and stamp==381 and not s._r6_fast_readback
 data[:]=0;assert actual[1,1]==7
 with pytest.raises(ValueError):actual.flags.writeable=True
 assert s.readbacks[-1]['r3_immutable_snapshot']
 def bad(*args):raise RuntimeError('actual service failed')
 monkeypatch.setattr(MapFlowMixin,'_server_costmap_snapshot',bad)
 with pytest.raises(RuntimeError,match='actual service failed'):StartupDeltaReader._server_costmap_snapshot(s,time.monotonic()+1)
 assert not s._r6_fast_readback

def test_original_semantic_repair_wrapper_keeps_publication_guard(monkeypatch):
 s,data=fake(monkeypatch)
 mro=StartupDeltaPerformanceSession.mro()
 assert mro.index(SemanticAckSession)<mro.index(StartupDeltaReader)<mro.index(StableSmacSession)
 actual,stamp=SemanticAckSession._server_costmap_snapshot(s,time.monotonic()+1)
 assert stamp==381 and np.array_equal(actual,data)
 from types import SimpleNamespace as S
 s._semantic_costmap=S(policy_hash='p');s._semantic_publication_sequence=1
 s._exact_repair_context={'semantic':object(),'signature':['p'],'sequence':1}
 with pytest.raises(RuntimeError,match='publication changed'):SemanticAckSession._server_costmap_snapshot(s,time.monotonic()+1)

def test_disabled_or_native_disabled_uses_parent_reader(monkeypatch):
 s,data=fake(monkeypatch);seen=[]
 monkeypatch.setattr(StableSmacSession,'_server_costmap_snapshot',lambda self,d:(seen.append(d) or (data,53)))
 for mode,native in [('control','ABC'),('I','A')]:
  monkeypatch.setenv('THREEYD_V9_R3_MODE',mode);monkeypatch.setenv('THREEYD_V8_R6_MODE',native)
  actual,stamp=StartupDeltaReader._server_costmap_snapshot(s,7)
  assert actual is data and stamp==53
 assert seen==[7,7]
