import numpy as np
import pytest

from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_3d_v1.pipeline import L1Plan, Layered3DV1Controller


def make_plan(route="r1", *, barrier_gap=True):
    free = np.ones((31, 41), dtype=bool)
    if barrier_gap:
        free[15, :] = False
        free[15, 18:23] = True
    corridor = np.ones_like(free)
    return L1Plan(
        static_safe_free=free, corridor_mask=corridor,
        start_cell=(28, 3), goal_cell=(2, 37),
        map_hash="map-v1", map_origin=(0.0, 0.0, 0.0), resolution=0.05,
        topology_hash="topology-v1", route_edge_ids=(route,),
        footprint_hash="jackal", route_signature=route,
    )


def snap(index, occupied):
    return DynamicSnapshot.from_cells(
        f"S{index}", occupied, timestamp=float(index),
        map_version="map-v1", map_shape=(31, 41),
    )


def test_pipeline_skip_then_l2_and_content_ack_commit():
    controller = Layered3DV1Controller(
        make_plan(), dynamic_inflation_radius_cells=0,
        dstar_wall_budget_ms=1000.0, verify_l2_oracle=True,
    )
    path = controller.l2.path_global
    assert path
    cell = path[len(path) // 2]
    first = controller.process_snapshot(snap(1, [cell]), now=1.0)
    assert not first.l3_required
    second = controller.process_snapshot(snap(2, [cell]), now=2.0)
    assert second.l3_required
    assert second.l2_result and second.l2_result.success
    assert second.dirty_roi and second.dirty_roi.closed_cells == 1
    before = controller.server_l3_mask.copy()
    with pytest.raises(ValueError):
        controller.acknowledge_l3_mask("wrong")
    assert np.array_equal(before, controller.server_l3_mask)
    controller.acknowledge_l3_mask(second.dirty_roi.target_hash)
    assert controller.server_l3_mask[cell] == 0


def test_l2_no_path_invokes_l1_graph_astar_callback_and_rebinds():
    controller = Layered3DV1Controller(
        make_plan(), dynamic_inflation_radius_cells=0,
        dstar_wall_budget_ms=1000.0, dstar_max_expansions=100_000,
    )
    barrier = [(15, column) for column in range(18, 23)]
    controller.process_snapshot(snap(1, barrier), now=1.0)
    replacement = make_plan(route="r2", barrier_gap=False)
    replacement.static_safe_free[15, :] = True
    called = []

    def replan(blocked):
        called.append(tuple(blocked))
        # A different topology route is represented by a safe opening on the
        # other side of the dynamic cells.
        return replacement

    result = controller.process_snapshot(snap(2, barrier), l1_replan=replan, now=2.0)
    assert called
    assert result.l1_graph_astar_called
    assert result.l1_reroute_succeeded
    assert result.l3_required
    assert result.route_signature == "r2"


def test_runtime_contract_is_latest_production_substrate():
    controller = Layered3DV1Controller(
        make_plan(), dynamic_inflation_radius_cells=0,
    )
    contract = controller.runtime_contract
    assert contract["l1"] == "deterministic_graph_astar"
    assert contract["l2"] == "persistent_dstar_lite_cropped_corridor_roi"
    assert contract["explicitly_not_derived_from"] == "3D-V0"
    assert contract["smac_angle_quantization_bins"] == 48
    assert contract["server_content_ack_required_before_smac"] is True
    assert contract["fixed_settle_cycles_after_ack"] == 0
