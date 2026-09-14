from types import SimpleNamespace
import numpy as np
import pytest
from arena_evaluation.roi_backend_r3 import (
    RoiExactAckSmacSession,RoiBounds,route_bounds,MAX_CELLS)


def session(h,w,ready=32*1024**2):
    m=SimpleNamespace(height=h,width=w,resolution=.05,origin=(0.,0.,0.),
                      world_to_cell=lambda x,y:(int(y/.05),int(x/.05)))
    s=object.__new__(RoiExactAckSmacSession)
    s.global_ctx=SimpleNamespace(hospital_map=m)
    s.max_cells=MAX_CELLS;s.ready_context_max_cells=ready
    return s


def test_bounded_map_preserves_global_lattice_for_all_routes():
    s=session(2000,2000)
    for points in [[(10.,10.),(15.,15.)],[(70.,10.),(20.,50.)]]:
        assert s.bounds_for_route(SimpleNamespace(polyline=points))==RoiBounds(0,0,2000,2000)


def test_large_maps_still_use_complete_bounded_route_roi():
    s=session(10000,10000);route=SimpleNamespace(polyline=[(30.,20.),(40.,60.)])
    assert s.bounds_for_route(route)==route_bounds(s.global_ctx.hospital_map,route)
    assert s.bounds_for_route(route).cells<MAX_CELLS


def test_zero_eager_limit_retains_frozen_query_roi_behavior():
    s=session(2000,2000,ready=0);route=SimpleNamespace(polyline=[(10.,10.),(15.,15.)])
    assert s.bounds_for_route(route)==route_bounds(s.global_ctx.hospital_map,route)
    assert s.bounds_for_route(route)!=RoiBounds(0,0,2000,2000)


def test_ready_staging_requires_complete_exact_update(monkeypatch):
    from arena_evaluation import roi_backend_r3 as roi
    s=session(10,12);events=[]
    s.begin_request=lambda rid,deadline:events.append(('begin',rid))
    monkeypatch.setattr(roi,'crop_context',lambda ctx,bounds:ctx)
    s.activate_context=lambda ctx,bounds:events.append(('stage',bounds.cells))
    def update(mask):
        assert mask.shape==(10,12) and not np.any(mask)
        events.append(('exact_update',mask.size))
        return {'costmap_update_acknowledged':True,'costmap_ack_mismatch_cells':0}
    s.update_local_mask=update
    result=s.prepare_ready_baseline(1.)
    assert [e[0] for e in events]==['begin','stage','exact_update']
    assert result['costmap_update_acknowledged']
    assert not result['query_dependent'] and not result['planner_search_started']
    assert not hasattr(s,'_ready_ack_window_s')


def test_ready_ack_failure_never_becomes_ready(monkeypatch):
    from arena_evaluation import roi_backend_r3 as roi
    s=session(10,12);s.begin_request=lambda *a:None
    monkeypatch.setattr(roi,'crop_context',lambda ctx,bounds:ctx)
    s.activate_context=lambda *a:None
    def fail(mask):raise roi.ExactAckFailure('EXACT_ACK_FAILED_CLOSED')
    s.update_local_mask=fail
    with pytest.raises(roi.ExactAckFailure):s.prepare_ready_baseline(1.)
    assert not hasattr(s,'ready_baseline_receipt')
    assert not hasattr(s,'_ready_ack_window_s')
