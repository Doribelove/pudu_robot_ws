from types import SimpleNamespace as S
from dataclasses import dataclass
import numpy as np
import pytest
from three_yd_v5.v9_maps import seal,content_digest,immutable,stats,clear
from three_yd_v5.v9_segments import SegmentCache,cache_for,store,propose
from three_yd_v5.v9_local import heading_error
from three_yd_v5.performance_session import PerformanceSession


def test_only_byte_backing_is_immutable_and_alias_mutation_changes_hash(monkeypatch):
 monkeypatch.setenv('THREEYD_V9_MODE','A');clear()
 a=np.zeros((60,90),np.int8);alias=a.view();a.flags.writeable=False
 old=content_digest(a);alias[0,0]=100
 assert content_digest(a)!=old and not immutable(a)
 b=seal(a);assert immutable(b) and np.array_equal(a,b)
 with pytest.raises(ValueError):b.flags.writeable=True
 old=content_digest(b);assert content_digest(b)==old
 assert stats()['immutable_hash_entries']==1
 del b
 import gc;gc.collect();assert stats()['immutable_hash_entries']==0


def test_mutable_maps_and_replacements_are_never_cached(monkeypatch):
 monkeypatch.setenv('THREEYD_V9_MODE','A')
 a=np.ones((80,91),np.uint8);b=seal(a);before=content_digest(a)
 a[44,77]=2;assert content_digest(a)!=before
 assert content_digest(b)==before
 assert content_digest(seal(a))==content_digest(a)
 assert content_digest(np.flipud(seal(a)))==content_digest(np.flipud(a))


def test_segment_cache_capacity_clear_and_binding():
 c=SegmentCache();c.bind('a')
 for i in range(200):c[i]={'poses':np.zeros((50,3))}
 assert len(c)==32 and c.bytes<=4*2**20 and c.evictions==168
 c.bind('b');assert len(c)==0 and c.bytes==0 and c.invalidations==1
 c[1]=1;c.clear();assert len(c)==0 and c.bytes==0


def fake():
 shape=(100,1000)
 w=S(key='x',map=S(sha256='m',resolution=.05,origin=(0.,0.),occupancy=np.zeros(shape,np.int8)),
     hard=np.zeros(shape,bool),labels=np.ones(shape,np.int16),vehicle=(.265,.225,.4),rules={},
     semantic=S(semantic_map_hash='s'))
 c=S(world=w,views={},max_search_cells=400000,max_map_fraction=.1,generic_mode=True,
     topology_route=S(signature='original-task'))
 return c


def test_segments_rebind_live_map_vehicle_rules_and_search_range(monkeypatch):
 monkeypatch.setenv('THREEYD_V9_MODE','B');c=fake();cache=cache_for(c)
 changes=[lambda: c.world.hard.__setitem__((30,50),True),
          lambda: c.world.map.occupancy.__setitem__((31,50),100),
          lambda: c.world.labels.__setitem__((32,50),2),
          lambda: setattr(c.world,'vehicle',(.3,.25,.4)),
          lambda: c.world.rules.update(one_way=True),
          lambda: setattr(c,'max_search_cells',200000)]
 for change in changes:
  cache['a']=1;change();assert cache_for(c) is cache and len(cache)==0
 assert cache.invalidations==len(changes)


def test_different_task_overlap_is_proposal_rechecked_in_current_mask(monkeypatch):
 monkeypatch.setenv('THREEYD_V9_MODE','B');c=fake()
 p=np.c_[np.arange(1.,31.,.05),np.full(600,2.5),np.zeros(600)]
 result=S(points=[dict(zip(('x','y','yaw'),v)) for v in p])
 assert store(c,{'success':False,'result':result})['entries']==0
 store(c,{'success':True,'result':result})
 c.topology_route.signature='different-goal-and-start-task'
 q=np.c_[np.arange(.5,32.,.05),np.full(630,2.55),np.zeros(630)]
 mask=np.ones(c.world.hard.shape,bool)
 proposal,log=propose(c,q,mask)
 assert log['matched_segments']>0 and log['matched_length_m']>=4
 assert np.allclose(proposal[0],q[0]) and np.allclose(proposal[-1],q[-1])
 blocked=np.zeros_like(mask)
 _,bad=propose(c,q,blocked);assert bad['matched_segments']==0
 reverse=q[::-1].copy();reverse[:,2]=np.pi
 _,bad=propose(c,reverse,mask);assert bad['matched_segments']==0


def test_shared_batch_requires_real_ack_same_map_scope_sequence_and_releases():
 s=PerformanceSession.__new__(PerformanceSession);s._semantic_costmap=S(signature='same')
 s._content_signature=lambda v:v.signature;s._semantic_publication_sequence=4;s._costmap_state_trusted=True
 mask=np.ones((30,50),bool)
 with pytest.raises(RuntimeError):
  with s.v9_batch(mask,{'costmap_update_acknowledged':False}):pass
 ack={'costmap_update_acknowledged':True,'costmap_ack_sequence':9}
 with s.v9_batch(mask,ack):
  d=s.update_local_mask(mask);assert d['v9_shared_batch_ack'] and d['local_map_update_ms']==0
  changed=mask.copy();changed[0,0]=False
  with pytest.raises(RuntimeError):s.update_local_mask(changed)
  s._semantic_publication_sequence=5
  with pytest.raises(RuntimeError):s.update_local_mask(mask)
 assert s._v9_batch is None
 with pytest.raises(ValueError):
  with s.v9_batch(mask,ack):raise ValueError('caller aborted')
 assert s._v9_batch is None


def test_heading_follows_geometry_and_detects_false_yaw():
 p=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
 assert heading_error(p)==0.
 p[1,1]=1;assert heading_error(p)>np.deg2rad(5)


def test_complete_audit_is_required_after_current_ack_and_cannot_publish_partial(monkeypatch):
 from three_yd_v5.v9_local import plan
 from arena_3d_v1.semantic_world import digest
 monkeypatch.setenv('THREEYD_V9_MODE','C')
 mask=np.ones((100,200),bool);world=S(key='w',hard=~mask,vehicle=(.265,.225,.4),
   map=S(resolution=.05,origin=(0.,0.)))
 c=S(world=world,dynamic=None,plan=S(corridor_mask=mask),initial_l2_result=S(success=True),
     reference_plan=S(world_key='w',fine_corridor_hash=digest(mask),world_xy=np.array([[1.,2.],[7.,2.]])),
     _target_mask=lambda:mask,trajectory=None,trajectory_version=None,_bounded_grid=S(diagnostics={}))
 q=S(query_id='straight',start=(1.,2.,0.),goal=(7.,2.,0.))
 s=S(bound_vehicle=world.vehicle)
 called=[]
 c.sync_only=lambda s:(called.append('actual_ack') or {'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0})
 ca=S(metrics={},diagnostics=lambda:{},final_valid_success=True)
 def audit(*a):
  called.append('whole_audit');assert len(a[1])>100
  return ca,{'valid':True},{'valid':True},0.,[]
 c._audit=audit;spec=S(backend='Smac',version='frozen')
 out=plan(c,q,s,None,spec)
 assert out['success'] and called==['actual_ack','whole_audit'] and len(c.trajectory)>100
 c._audit=lambda *a:(ca,{'valid':False},{'valid':True},0.,['BODY_REJECTED'])
 out=plan(c,q,s,None,spec)
 assert not out['success'] and c.trajectory is None and 'BODY_REJECTED' in out['failure_code']
 c.sync_only=lambda s:{'costmap_update_acknowledged':False}
 c._audit=lambda *a:pytest.fail('unconfirmed map reached audit')
 out=plan(c,q,s,None,spec)
 assert not out['success'] and 'ACK' in out['failure_code'] and c.trajectory is None
 c.reference_plan.fine_corridor_hash='stale'
 with pytest.raises(ValueError,match='REFERENCE_BINDING'):plan(c,q,s,None,spec)


def test_scope_reuse_is_weak_and_releases_at_query_lifecycle(monkeypatch):
 from three_yd_v5.controller import V5Controller
 import gc
 monkeypatch.setenv('THREEYD_V9_MODE','A')
 c=V5Controller.__new__(V5Controller);c.dynamic=None;c.r13_remainder_mask=None
 mask=np.ones((30,50),bool);c.plan=S(corridor_mask=mask);c._search_mask=mask.copy()
 v=c._target_mask();assert immutable(v) and c._target_mask() is v
 ref=c._v9_target[1];del v;gc.collect();assert ref() is None
 v=c._target_mask();mask[10,20]=False
 current=c._target_mask();assert not current[10,20] and current is not v
 c._soft_grid_cache={'old':1};c._clear_transients()
 assert c._v9_target is None and not c._soft_grid_cache and c.pending_l3_mask is None
