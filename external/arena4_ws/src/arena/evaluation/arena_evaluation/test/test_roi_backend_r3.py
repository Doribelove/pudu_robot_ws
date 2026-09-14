from types import SimpleNamespace
from dataclasses import replace
import numpy as np
import pytest
from arena_evaluation.roi_backend_r3 import (RoiBounds,RoiMap,route_bounds,crop_context,
    MAX_CELLS,canonical)
from arena_evaluation import two_layer_v1_r3_benchmark as runner
from arena_evaluation.exact_ack_r3 import Publication,compare_exact,grid_hash
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.bounded_context_r3 import BoundedField


def context(h=700,w=900,origin=(-12.5,23.25,0.)):
    m=HospitalMap(None,None,.05,origin,w,h,np.zeros((h,w),np.int8),np.ones((h,w),np.float32))
    m.occupancy[270:410,460:470]=100
    m.distance_m=BoundedField(m,'occupied_distance')
    return SimpleNamespace(hospital_map=m,map_sha256='a'*64,bounded_memory=True)


def test_capacity_integer_overflow_and_memory_bound():
    assert not RoiBounds(0,0,8000,12000).capacity()['supported']
    assert not RoiBounds(0,0,8193,8192).capacity(64*1024**2)['supported']
    assert RoiBounds(0,0,8192,8192).capacity()['supported']
    assert not RoiBounds(0,0,10,10).capacity(99)['supported']
    assert runner.pinned_smac_capacity(1,89478485)['supported']
    assert not runner.pinned_smac_capacity(1,89478486)['supported']


def test_nonzero_origin_and_top_bottom_roundtrip():
    m=context().hospital_map;b=RoiBounds(111,127,300,450);r=RoiMap(m,b)
    for cell in [(0,0),(299,449),(17,230)]:
        xy=r.cell_to_world(cell)
        assert r.world_to_cell(*xy)==cell
        assert m.world_to_cell(*xy)==(cell[0]+111,cell[1]+127)
    assert r.world_to_cell(*m.cell_to_world((110,127))) is None
    assert np.shares_memory(r.occupancy,m.occupancy)


@pytest.mark.parametrize('expansion',[False,True])
def test_crop_corridor_exact_frozen_geometry(expansion):
    ctx=context();m=ctx.hospital_map
    pts=[m.cell_to_world(c) for c in [(220,230),(260,320),(420,420),(490,570)]]
    route=SimpleNamespace(polyline=pts)
    b=route_bounds(m,route);local=crop_context(ctx,b)
    full,_=runner.corridor(ctx,route,expansion)
    cropped,_=runner.corridor(local,route,expansion)
    assert np.array_equal(cropped,full[b.top:b.top+b.height,b.left:b.left+b.width])
    assert full.sum()==cropped.sum()  # No allowed cells silently removed.


def test_oversized_roi_not_silently_truncated():
    # No global arrays are needed for the capacity computation.
    m=SimpleNamespace(resolution=.05,origin=(0,0,0),height=10000,width=10000,
                      world_to_cell=lambda x,y:(int(y),int(x)))
    b=route_bounds(m,SimpleNamespace(polyline=[(0,0),(9999,9999)]))
    assert b==RoiBounds(0,0,10000,10000)
    assert not b.capacity()['supported']


@pytest.mark.parametrize('field,value',[('backend_context','other'),('map_hash','other'),
    ('roi_bbox',(0,0,2,2)),('sequence',7),('request_id','other')])
def test_context_stale_or_wrong_binding_never_ack(field,value):
    a=np.zeros((3,4),np.uint8);h=grid_hash(a)
    token=Publication(1,h,(0,0,4,3),h,(3,4),'a'*64,'q','context1')
    assert compare_exact(token,token,a,a)['acknowledged']
    assert not compare_exact(replace(token,**{field:value}),token,a,a)['acknowledged']


def test_same_input_binding_deterministic_and_origin_change_invalidates():
    a={'bbox':[1,2,3,4],'origin':[.05,.1,0.],'generation':1}
    assert canonical(a)==canonical(dict(reversed(list(a.items()))))
    assert canonical(a)!=canonical({**a,'origin':[.1,.1,0.]})


def test_window_minimum_preserves_global_batch_and_none():
    from arena_evaluation.roi_backend_r3 import WindowField
    class Field:
        def minimum_at(self,cells):
            self.seen=cells
            return 0. if None in cells else .123
    field=Field();window=WindowField(field,RoiBounds(100,200,20,30))
    assert window.minimum_at([(1,2),(4,5)])==.123
    assert field.seen==[(101,202),(104,205)]
    assert window.minimum_at([None])==0.


def test_cropped_minimum_equals_global_canonical_value():
    from arena_evaluation.roi_backend_r3 import WindowField
    ctx=context();b=RoiBounds(200,200,300,300)
    cells=[(270,459),(390,471),(210,230),(450,470)]
    local=[(r-b.top,c-b.left) for r,c in cells]
    assert WindowField(ctx.hospital_map.distance_m,b).minimum_at(local)==ctx.hospital_map.distance_m.minimum_at(cells)


def test_memory_rejection_is_distinct_from_index_overflow():
    v=RoiBounds(0,0,8000,10000).capacity(64*1024**2)
    assert v['index_supported'] and not v['memory_supported']
    assert v['failure_code']=='ROI_MEMORY_CELL_LIMIT'
    assert RoiBounds(0,0,8000,12000).capacity()['failure_code']=='PINNED_SMAC_INDEX_CAPACITY'


def test_unconfirmed_context_cannot_invoke_parent_search(monkeypatch):
    from arena_evaluation.roi_backend_r3 import RoiExactAckSmacSession
    from arena_evaluation.exact_ack_r3 import ExactAckSmacSession
    s=object.__new__(RoiExactAckSmacSession);s.context_uncertain=True;s.backend_context_binding='valid-old'
    def forbidden(*a,**k):raise AssertionError('search entered with uncertain context')
    monkeypatch.setattr(ExactAckSmacSession,'plan',forbidden)
    result=s.plan(None,SimpleNamespace(backend='smac',version='pinned'))
    assert result.failure_code=='ROI_CONTEXT_NOT_CONFIRMED'
    assert result.diagnostics['planner_search_started'] is False


def test_collision_rejection_does_not_mislabel_corridor_membership():
    from arena_evaluation.roi_backend_r3 import GlobalRoiAuditor
    from arena_evaluation.path_audit import PathAuditResult
    class Auditor:
        def audit(self,*a):
            return PathAuditResult(within_mask=True,mask_hash='mask',metrics={
                'static_footprint_valid':False,'kinematic_valid':True,'failure_code':'STATIC_FOOTPRINT_COLLISION'},
                timings={'canonical_path_audit_ms':1.})
    session=SimpleNamespace(local_auditor=Auditor(),backend_context_binding=canonical({'generation':1,'bbox':[1,2,3,4]}))
    wrapped=GlobalRoiAuditor(Auditor(),session)
    result=wrapped.audit(None,[],None)
    assert result.within_mask and not result.final_valid_success
    assert result.metrics['final_valid_success'] is False
    first=result.mask_hash
    session.backend_context_binding=canonical({'generation':2,'bbox':[1,2,3,4]})
    assert wrapped.audit(None,[],None).mask_hash==first


def test_local_audit_rejection_remains_fail_closed():
    from arena_evaluation.roi_backend_r3 import GlobalRoiAuditor
    from arena_evaluation.path_audit import PathAuditResult
    class Auditor:
        def __init__(self,valid):self.valid=valid
        def audit(self,*a):
            return PathAuditResult(within_mask=True,metrics={'static_footprint_valid':self.valid,
                'kinematic_valid':True,'failure_code':'' if self.valid else 'STATIC_FOOTPRINT_COLLISION'},
                timings={'canonical_path_audit_ms':1.})
    session=SimpleNamespace(local_auditor=Auditor(False),backend_context_binding=canonical({'generation':1}))
    result=GlobalRoiAuditor(Auditor(True),session).audit(None,[],None)
    assert not result.final_valid_success
    assert result.metrics['failure_code']=='STATIC_FOOTPRINT_COLLISION'


def test_80_micell_context_limit_has_uint32_headroom():
    assert MAX_CELLS==80*1024**2
    maximum=RoiBounds(0,0,8192,10240).capacity()
    assert maximum['supported'] and maximum['cells']==MAX_CELLS
    assert maximum['exclusive_index_limit']-maximum['required_states']>250_000_000
    assert not RoiBounds(0,0,8192,10241).capacity()['memory_supported']
    assert RoiBounds(0,0,8000,10000).capacity()['supported']
