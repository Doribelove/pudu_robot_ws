from types import SimpleNamespace
import numpy as np
import pytest
from arena_3d_v1.multires_grid import make_roi
from arena_3d_v1.l2_state_lifecycle import CompactCorridorGeometry,MutableStateBinding
from three_yd_v5.fast_state import PrecomputedEdgeState
from three_yd_v5.native_cold_search import NativeColdState


def pair(seed):
    rng=np.random.default_rng(seed);safe=rng.random((31,47))>.10
    safe[1,1]=safe[-2,-2]=True
    view=SimpleNamespace(key='test',safe=safe,factor=3,vehicle=(.265,.225,.4),map=SimpleNamespace(height=31,width=47,origin=(0.,0.,0.),resolution=.15))
    roi=make_roi(view,np.ones_like(safe),(1,1),(29,45),'test')
    g=CompactCorridorGeometry.build(roi);binding=MutableStateBinding(g.binding.digest,roi.binding.start_cell,roi.binding.goal_cell,'static')
    p=rng.uniform(0.,4.,g.state_count) if seed%2 else np.zeros(g.state_count)
    direction=np.zeros((g.state_count,2));direction[:,0]=float(seed%3==0)
    return [cls(g,binding,p,2.,direction,np.zeros(g.state_count)) for cls in (PrecomputedEdgeState,NativeColdState)]

@pytest.mark.parametrize('seed',range(12))
def test_native_full_state_path_and_queue_parity(seed):
    old,new=pair(seed)
    a=old.compute_shortest_path(timeout_s=5.);b=new.compute_shortest_path(timeout_s=5.)
    assert not a.timeout_triggered and not b.timeout_triggered
    assert a.expanded_nodes==b.expanded_nodes and a.queue_pushes==b.queue_pushes
    assert np.array_equal(old.g,new.g) and np.array_equal(old.rhs,new.rhs)
    assert old.extract_path_ids()==new.extract_path_ids()
    assert sorted(old.open)==sorted(new.open) and old.queued==new.queued
    assert new.native_cold_used
    # Incremental updates after native initialization must retain a valid
    # resumable D* state. Unblocked cells and old queued entries matter too.
    for s in (old,new):
        s.initialized=True;s.set_blocked_ids({5,12,35})
        s.compute_shortest_path(timeout_s=5.)
    assert np.array_equal(old.g,new.g) and old.extract_path_ids()==new.extract_path_ids()


def test_native_budget_stop_is_resumable_and_not_no_path_certificate():
    old,new=pair(3)
    a=old.compute_shortest_path(max_expansions=3);b=new.compute_shortest_path(max_expansions=3)
    assert a.timeout_triggered and b.timeout_triggered
    assert np.array_equal(old.g,new.g) and sorted(old.open)==sorted(new.open)
    for s in (old,new):s.compute_shortest_path(timeout_s=5.)
    assert old.extract_path_ids()==new.extract_path_ids() and np.array_equal(old.g,new.g)
