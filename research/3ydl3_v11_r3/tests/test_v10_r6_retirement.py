from types import SimpleNamespace as S
from dataclasses import replace
import weakref,gc
import numpy as np
import pytest
from arena_evaluation.semantic_costmap_composer import SemanticCostmap
from three_yd_v5.source_retirement import detach,retire
from three_yd_v5.r4_tiles import TiledSoft
from three_yd_v5.v9_maps import seal,immutable
from three_yd_v5.r6_ack import grid_digest

@pytest.mark.parametrize('dtype',[bool,np.int8,np.uint8,np.float32])
@pytest.mark.parametrize('reverse',[False,True])
def test_detach_small_native_crop_keeps_bytes_without_global_owner(dtype,reverse):
 source=np.arange(1024*512,dtype=np.int32).reshape(1024,512).astype(dtype)
 source=seal(source);view=source[120:150,44:90]
 if reverse:view=view[::-1]
 actual=detach(view)
 assert np.array_equal(actual,view) and actual.dtype==view.dtype and actual.shape==view.shape
 assert grid_digest(actual)==grid_digest(view) and immutable(actual)
 assert not np.shares_memory(actual,source)
 with pytest.raises(ValueError):actual.flags.writeable=True

def test_small_owned_or_broadcast_storage_is_retained():
 a=seal(np.ones((5,7),bool));assert detach(a) is a
 a=np.broadcast_to(np.frombuffer(bytes([1]),bool),(1024,1024));assert detach(a) is a
 assert detach(None) is None


def fixture():
 source=seal(np.arange(1024*512,dtype=np.uint8).reshape(1024,512));view=source[10:30,30:55]
 hard=seal(source>127)[10:30,30:55]
 soft=TiledSoft(view.shape,140.,~hard,[((0,20,0,25),view.astype(np.float32))])
 grid=SemanticCostmap(view,view.view(np.int8),view,soft,hard,np.broadcast_to(True,view.shape),'policy','semantic','reference',True,{'source':'unchanged'})
 s=S(r5_enabled=True,_r5_global_grid=object(),_semantic_costmap=grid,_r5_grid_cache=object(),_r5_mask_cache=object(),_action_in_progress=False,
     _current_grid=view[::-1].view(np.int8),_current_allowed_mask=hard[::-1],_last_exact_expected_master=view[::-1],
     _costmap_state_trusted=True,_last_exact_signature=('policy','grid','master'),_semantic_publication_sequence=77,_costmap_ack_sequence=43)
 return s,source


def test_retirement_keeps_native_ack_content_and_sequences():
 s,source=fixture();old=s._semantic_costmap;before={k:getattr(s,k).copy() for k in ('_current_grid','_current_allowed_mask','_last_exact_expected_master')}
 assert retire(s)
 assert s._r5_global_grid is s._r5_grid_cache is s._r5_mask_cache is None
 assert s._costmap_state_trusted and s._last_exact_signature==('policy','grid','master')
 assert s._semantic_publication_sequence==77 and s._costmap_ack_sequence==43
 for k,v in before.items():assert np.array_equal(getattr(s,k),v)
 for k in ('internal_cost','occupancy_grid','expected_master_cost','hard_semantic_mask','affected_mask'):
  assert np.array_equal(getattr(old,k),getattr(s._semantic_costmap,k))
 assert s._semantic_costmap.diagnostics==old.diagnostics
 assert np.array_equal(s._semantic_costmap.soft_cost[:,:],old.soft_cost[:,:])
 assert not np.shares_memory(s._semantic_costmap.occupancy_grid,source)
 assert not retire(s)

@pytest.mark.parametrize('flag',['_action_in_progress','_r7_batch','_v9_batch'])
def test_active_publication_cannot_be_retired(flag):
 s,_=fixture();setattr(s,flag,True)
 original=s._r5_global_grid
 with pytest.raises(RuntimeError,match='R6_RETIRE_DURING_ACTIVE_PUBLICATION'):retire(s)
 assert s._r5_global_grid is original

@pytest.mark.parametrize('fails',[False,True])
def test_missed_single_entry_costmap_evicted_before_new_composition(monkeypatch,fails):
 from three_yd_v5 import controller,fast_costmap
 from three_yd_v5.r6_cache import get_cache
 monkeypatch.setenv('THREEYD_V10_R6_MODE','C')
 monkeypatch.setattr(controller,'v9_enabled',lambda flag:False)
 monkeypatch.setattr(controller,'seal_grid',lambda g:g)
 c=S(world=S(map=S(sha256='map')),views={},topology_route=S(signature='new'),r6_contract='test',
     reference_cells=np.zeros((2,2),int),timing={},optimize=True)
 c._target_mask=lambda:np.ones((3,4),bool);c._dynamic_identity=lambda:(0,'static')
 cache=get_cache(c.views,'bounded-costmap',max_entries=1)
 class Grid:pass
 prior=Grid();reference=weakref.ref(prior);cache['old']=prior;del prior
 new=Grid();calls=[]
 def build(*args):
  calls.append(True);assert reference() is None and not cache
  if fails:raise RuntimeError('composition failure')
  return new
 monkeypatch.setattr(fast_costmap,'compose_bounded',build)
 if fails:
  with pytest.raises(RuntimeError,match='composition failure'):controller.V5Controller._prepare_grid_cached(c)
  assert not cache
 else:
  assert controller.V5Controller._prepare_grid_cached(c) is new
  assert controller.V5Controller._prepare_grid_cached(c) is new
 assert calls==[True]
