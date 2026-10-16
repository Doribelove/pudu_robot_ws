from types import SimpleNamespace
import math
import numpy as np
from arena_3d_v1.multires_grid import make_roi
from arena_3d_v1.l2_state_lifecycle import CompactCorridorGeometry,MutableStateBinding
from arena_3d_v1.semantic_l2 import weighted_astar
from three_yd_v4.r5_controller import EdgeState
from three_yd_v5.fast_state import PrecomputedEdgeState
from three_yd_v5.fast_costmap import compose_bounded
from three_yd_v4.r5_bounded import R5BoundedController
from three_yd_v5.controller import V5Controller


def states():
    safe=np.ones((18,24),bool);safe[4:14,10]=False;safe[9,10]=True
    view=SimpleNamespace(key='test',safe=safe,factor=3,vehicle=(.265,.225,.4),
        map=SimpleNamespace(height=18,width=24,origin=(0.,0.,0.),resolution=.15))
    roi=make_roi(view,np.ones_like(safe),(8,2),(9,21),'test')
    g=CompactCorridorGeometry.build(roi)
    binding=MutableStateBinding(g.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,'static')
    rng=np.random.default_rng(193)
    potential=rng.uniform(0.,4.,g.state_count)
    direction=np.zeros((g.state_count,2));direction[:,0]=1.
    progress=np.zeros(g.state_count)
    return [cls(g,binding,potential,2.,direction,progress) for cls in (EdgeState,PrecomputedEdgeState)]


def check_edges(a,b):
    for u in range(a.geometry.state_count):
        for v in a.geometry.neighbor_ids(u):
            assert a._edge_cost(u,v)==b._edge_cost(u,v)


def test_exact_edge_parity_and_search_tie_order():
    old,new=states();check_edges(old,new)
    for s in (old,new):
        assert not s.compute_shortest_path(timeout_s=5.).timeout_triggered
    assert old.extract_path_ids()==new.extract_path_ids()
    assert np.array_equal(old.g,new.g)
    assert old.total_expanded_nodes==new.total_expanded_nodes
    assert weighted_astar(old)[0]==weighted_astar(new)[0]


def test_obstacle_and_potential_updates_invalidate_fast_weights():
    old,new=states()
    for s in (old,new):s.set_blocked_ids({7,20,41})
    check_edges(old,new)
    for s in (old,new):s.set_blocked_ids(set())
    check_edges(old,new)
    p=old.potential.copy();p[3:25]+=1.
    for s in (old,new):s.replace_potential(p)
    check_edges(old,new)


def test_fused_costmap_all_bytes_match_r8():
    shape=(120,160);rng=np.random.default_rng(1)
    occupancy=np.zeros(shape,np.int8);occupancy[rng.random(shape)<.03]=100
    occupancy[:8]=-1;hard=occupancy!=0
    mask=np.zeros(shape,bool);mask[10:112,8:150]=True
    drivable=mask.copy();corridor=~hard&mask
    c=V5Controller.__new__(V5Controller)
    c.world=SimpleNamespace(hard=hard,semantic=SimpleNamespace(semantic_map_hash='s'),
        map=SimpleNamespace(occupancy=occupancy,resolution=.05))
    c.global_drivable=drivable;c.plan=SimpleNamespace(corridor_mask=corridor)
    c.dynamic=None;c.reference_cells=np.array([[15,15],[95,85],[95,135]])
    c.task={'signature':'t'};c._soft_grid_cache={};c._bounded_grid=None;c._target_mask=lambda:mask
    old=R5BoundedController.prepare_grid(c);new=compose_bounded(c,mask)
    for field in ('internal_cost','occupancy_grid','expected_master_cost','soft_cost','hard_semantic_mask'):
        assert np.array_equal(getattr(old,field),getattr(new,field)),field
    assert old.policy_hash==new.policy_hash


def test_cache_hit_reports_current_time_without_mutating_source():
    c=V5Controller.__new__(V5Controller);c.timing={'adapter_calls':[],'r6_costmap_cache_hit':True}
    from dataclasses import dataclass
    @dataclass
    class Grid:
        diagnostics:dict
    source=Grid({'adapter_ms':524.})
    c._prepare_grid_cached=lambda enabled,cap:source
    result=c.prepare_grid()
    assert result.diagnostics['adapter_cache_hit']
    assert result.diagnostics['adapter_source_build_ms']==524.
    assert result.diagnostics['adapter_actual_ms']<100.
    assert source.diagnostics=={'adapter_ms':524.}


def test_native_allocator_environment_restored_on_start_failure(monkeypatch):
    from three_yd_v5.performance_session import PerformanceSession
    from three_yd_v4.raw_diagnostics import RawDiagnosticSession
    import os,pytest
    monkeypatch.setenv('THREEYD_R9_OPTIMIZE','1')
    monkeypatch.setenv('THREEYD_R9_NATIVE_ALLOCATOR','default')
    monkeypatch.setenv('MALLOC_ARENA_MAX','2')
    monkeypatch.setenv('MALLOC_TRIM_THRESHOLD_','131072')
    def failed_start(self):
        assert 'MALLOC_ARENA_MAX' not in os.environ
        assert 'MALLOC_TRIM_THRESHOLD_' not in os.environ
        raise RuntimeError('test startup failure')
    monkeypatch.setattr(RawDiagnosticSession,'start',failed_start)
    with pytest.raises(RuntimeError,match='test startup failure'):
        PerformanceSession.__new__(PerformanceSession).start()
    assert os.environ['MALLOC_ARENA_MAX']=='2'
    assert os.environ['MALLOC_TRIM_THRESHOLD_']=='131072'
