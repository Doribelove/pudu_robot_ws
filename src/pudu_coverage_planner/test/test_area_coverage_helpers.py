import numpy as np

from pudu_coverage_planner.area_coverage_node import AreaCoverageNode


def test_reachable_component_uses_start_component():
    mask = np.zeros((8, 12), dtype=bool)
    mask[1:5, 1:5] = True
    mask[2:4, 9:11] = True

    selected = AreaCoverageNode._reachable_component(mask, (9.0, 2.0))

    assert int(selected.sum()) == 4
    assert selected[2, 9]
    assert not selected[1, 1]


def test_reachable_component_uses_largest_without_pose():
    mask = np.zeros((8, 12), dtype=bool)
    mask[1:5, 1:5] = True
    mask[2:4, 9:11] = True

    selected = AreaCoverageNode._reachable_component(mask, None)

    assert int(selected.sum()) == 16
    assert selected[1, 1]
    assert not selected[2, 9]


def test_reachable_component_snaps_to_nearest_free_space():
    mask = np.zeros((8, 12), dtype=bool)
    mask[1:5, 1:5] = True
    mask[2:4, 9:11] = True

    selected = AreaCoverageNode._reachable_component(mask, (7.5, 2.0))

    assert int(selected.sum()) == 4
    assert selected[2, 9]
