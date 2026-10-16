from dataclasses import replace
import numpy as np
import pytest
from test_v9_r5_fullstorage import make
from three_yd_v5.v9_maps import seal,content_digest
from three_yd_v5.v9_r7_batch import BoundStorageSession,field_binding
from three_yd_v5.performance_session import PerformanceSession


def prepared(monkeypatch):
    s,q,mask,g=make(monkeypatch);s.__class__=BoundStorageSession
    monkeypatch.setenv('THREEYD_V9_R7_MODE','T')
    s.bind_complete_route_scope(q,mask);native=s._prepare_native(mask)
    s._r5_switching=False;s._costmap_state_trusted=True;s._semantic_publication_sequence=7
    s._last_exact_signature=s._content_signature(s._semantic_costmap)
    s._last_exact_expected_master=np.flipud(s._semantic_costmap.expected_master_cost).copy()
    s._current_grid=np.flipud(s._semantic_costmap.occupancy_grid)
    s._current_allowed_mask=native
    policy,grid,master=s._last_exact_signature
    ack={'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0,
         'semantic_exact_stable_observations':2,'semantic_policy_hash':policy,
         'semantic_expected_grid_hash':grid,'semantic_expected_master_hash':master,
         'server_costmap_content_hash':content_digest(np.flipud(s._semantic_costmap.expected_master_cost)),
         'costmap_ack_sequence':9,'semantic_publication_sequence':7}
    ack['semantic_expected_server_content_hash']=ack['server_costmap_content_hash']
    s._last_exact_ack=dict(ack)
    return s,mask,g,native,ack


def test_parent_global_binding_reproduces_rejection(monkeypatch):
    s,mask,g,native,ack=prepared(monkeypatch)
    with PerformanceSession.v9_batch(s,mask,ack):
        with pytest.raises(RuntimeError,match='V9_BATCH_MAP_BINDING_CHANGED'):
            PerformanceSession.update_local_mask(s,native)
    assert s._v9_batch is None


def test_native_batch_pins_both_bindings_and_parent_guard(monkeypatch):
    s,mask,g,native,ack=prepared(monkeypatch);original=s._semantic_costmap
    with s.v9_batch(mask,ack):
        assert s._prepare_native(mask) is s._r7_batch['native_mask']
        for incoming in (mask,native,mask):
            r=s.update_local_mask(incoming)
            assert r['costmap_update_acknowledged'] and r['v9_shared_batch_ack']
            assert r['r7_global_and_native_batch_binding_verified']
            assert r['local_map_update_messages']==0
            assert s._semantic_costmap is original
        with pytest.raises(RuntimeError,match='NESTED'): 
            with s.v9_batch(mask,ack):pass
    assert s._r7_batch is None and s._v9_batch is None
    g.soft_cost[:]=17;s._prepare_native(mask)
    assert np.all(s._semantic_costmap.soft_cost==17)
    assert s._semantic_costmap is not original


@pytest.mark.parametrize('fault',['global_mask','native_mask','task','bounds','scope','origin',
    'resolution','global_origin','global_map_hash','vehicle','grid_wrapper','native_wrapper',
    'global_policy','native_policy','global_internal','global_hard','global_affected','global_soft',
    'native_internal','native_soft','sequence','trust','switching','bookkeeping','diagnostics'])
def test_binding_change_never_inherits_ack(monkeypatch,fault):
    s,mask,g,native,ack=prepared(monkeypatch)
    with pytest.raises(RuntimeError,match='R7_BATCH_'):
        with s.v9_batch(mask,ack):
            incoming=mask
            if fault=='global_mask':incoming=mask.copy();incoming[0,0]=True
            elif fault=='native_mask':incoming=native.copy();incoming[1,1]=~incoming[1,1]
            elif fault=='task':s.r5_task_key=('changed',)
            elif fault=='bounds':s.r5_native_bounds=(0,1,0,1)
            elif fault=='scope':s.r5_scope_bounds=(0,2,0,2)
            elif fault=='origin':s.ctx.hospital_map.origin=(1.,2.,0.)
            elif fault=='resolution':s.ctx.hospital_map.resolution=.1
            elif fault=='global_origin':s.global_ctx.hospital_map.origin=(2.,3.,0.)
            elif fault=='global_map_hash':s.global_ctx.map_sha256='changed'
            elif fault=='vehicle':s.bound_vehicle=(1.,1.,1.)
            elif fault=='grid_wrapper':s._r5_global_grid=replace(g)
            elif fault=='native_wrapper':s._semantic_costmap=replace(s._semantic_costmap)
            elif fault=='global_policy':g.policy_hash='changed'
            elif fault=='native_policy':s._semantic_costmap.policy_hash='changed'
            elif fault=='global_internal':g.internal_cost[0,0]=1
            elif fault=='global_hard':g.hard_semantic_mask[0,0]=~g.hard_semantic_mask[0,0]
            elif fault=='global_affected':g.affected_mask[0,0]=False
            elif fault=='global_soft':g.soft_cost[0,0]=1
            elif fault=='native_internal':s._semantic_costmap.internal_cost[0,0]=2
            elif fault=='native_soft':s._semantic_costmap.soft_cost[0,0]=2
            elif fault=='sequence':s._semantic_publication_sequence+=1
            elif fault=='trust':s._costmap_state_trusted=False
            elif fault=='switching':s._r5_switching=True
            elif fault=='bookkeeping':s._current_allowed_mask=s._current_allowed_mask.copy();s._current_allowed_mask[0,0]=~s._current_allowed_mask[0,0]
            elif fault=='diagnostics':g.diagnostics['search_bound_hash']='changed'
            s.update_local_mask(incoming)
    assert s._r7_batch is None and s._v9_batch is None


@pytest.mark.parametrize('fault',['missing','unacknowledged','mismatch','unstable','falsehash','sequence','policy','stored_ack','stored_master'])
def test_real_current_stable_ack_required(monkeypatch,fault):
    s,mask,g,native,ack=prepared(monkeypatch)
    if fault=='missing':ack={}
    elif fault=='unacknowledged':ack['costmap_update_acknowledged']=False
    elif fault=='mismatch':ack['costmap_ack_mismatch_cells']=1
    elif fault=='unstable':ack['semantic_exact_stable_observations']=1
    elif fault=='falsehash':ack['server_costmap_content_hash']='invented'
    elif fault=='sequence':ack['semantic_publication_sequence']+=1
    elif fault=='policy':ack['semantic_policy_hash']='changed'
    elif fault=='stored_ack':s._last_exact_ack=None
    elif fault=='stored_master':s._last_exact_expected_master[0,0]=0
    with pytest.raises(RuntimeError,match='CURRENT_REAL_ACK'):
        with s.v9_batch(mask,ack):pass
    assert getattr(s,'_r7_batch',None) is None


def test_release_on_body_error_and_exit_checks(monkeypatch):
    s,mask,g,native,ack=prepared(monkeypatch)
    with pytest.raises(ValueError,match='body'):
        with s.v9_batch(mask,ack):raise ValueError('body')
    assert s._r7_batch is None and s._v9_batch is None
    with pytest.raises(RuntimeError,match='R7_BATCH_'):
        with s.v9_batch(mask,ack):s._semantic_publication_sequence+=1
    assert s._r7_batch is None and s._v9_batch is None


def test_source_setter_during_batch_refuses_rebind(monkeypatch):
    s,mask,g,native,ack=prepared(monkeypatch)
    with s.v9_batch(mask,ack):
        with pytest.raises(RuntimeError,match='REBIND'):s.set_semantic_costmap(g)
        assert s._r5_global_grid is g


def test_sparse_soft_binding_never_materializes_dense_field(monkeypatch):
    from three_yd_v5.r4_tiles import TiledSoft
    drive=np.ones((100,120),bool);values=np.zeros((4,6),np.float32)
    v=TiledSoft(drive.shape,140.,drive,[((5,9,7,13),values)])
    monkeypatch.setattr(TiledSoft,'__array__',lambda *args,**kwargs:pytest.fail('dense materialization'))
    initial=field_binding(v);values[0,0]=2
    assert field_binding(v)!=initial
    initial=field_binding(v);drive[0,0]=False
    assert field_binding(v)!=initial


def test_control_uses_original_batch_even_with_storage(monkeypatch):
    s,mask,g,native,ack=prepared(monkeypatch)
    monkeypatch.setenv('THREEYD_V9_R7_MODE','N')
    with s.v9_batch(mask,ack):
        with pytest.raises(RuntimeError,match='V9_BATCH_MAP_BINDING_CHANGED'):
            s.update_local_mask(mask)
    assert s._v9_batch is None


@pytest.mark.parametrize('mode,n,t',[('control',False,False),('N',True,False),('T',False,True),('NT',True,True)])
def test_independent_candidate_flags(monkeypatch,mode,n,t):
    from three_yd_v5.v9_r7_options import enabled
    monkeypatch.setenv('THREEYD_V9_R7_MODE',mode)
    assert enabled('N')==n and enabled('T')==t
