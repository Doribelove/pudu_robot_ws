import math
import numpy as np
import pytest
from test_v8_r4_native import pair
from arena_3d_v1.semantic_l2 import weighted_astar as reference
from three_yd_v5.tail_astar import weighted_astar


@pytest.mark.parametrize('seed',range(20))
def test_native_astar_same_path_cost_and_queue_counts(monkeypatch,seed):
    monkeypatch.setenv('THREEYD_V10_R4_MODE','N')
    old,new=pair(seed)
    a,aa=reference(old,timeout_s=5.);b,bb=weighted_astar(new,timeout_s=5.)
    assert a==b
    for k in ('path','cost','expanded_nodes','generated_nodes','timeout_triggered'):
        assert getattr(aa,k)==getattr(bb,k)
    assert new.r4_native_astar_used
    assert np.array_equal(old.g,new.g) and np.array_equal(old.rhs,new.rhs)


def test_dynamic_and_custom_costs_use_current_original_search(monkeypatch):
    monkeypatch.setenv('THREEYD_V10_R4_MODE','N');old,new=pair(4)
    for s in (old,new):s.set_blocked_ids({5,12,35})
    a,aa=reference(old);b,bb=weighted_astar(new)
    assert a==b and aa.cost==bb.cost and not hasattr(new,'r4_native_astar_used')
    old,new=pair(4)
    for s in (old,new):s._edge_cost=lambda a,b: math.inf
    a,aa=reference(old);b,bb=weighted_astar(new)
    assert a==b and aa.cost==bb.cost and not hasattr(new,'r4_native_astar_used')


def test_native_timeout_is_not_physical_blockage(monkeypatch):
    monkeypatch.setenv('THREEYD_V10_R4_MODE','N');_,s=pair(2)
    ids,result=weighted_astar(s,timeout_s=0.)
    assert ids is None and result.timeout_triggered and result.expanded_nodes==0
