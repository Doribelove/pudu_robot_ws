from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_3d_v1.pipeline import L1Plan, ProductionL3Adapter
from arena_3d_v1.r1_pipeline import Layered3DV1R1Controller


def make_plan(route="r1"):
    free = np.ones((21, 31), dtype=bool)
    corridor = np.ones_like(free)
    return L1Plan(
        static_safe_free=free,
        corridor_mask=corridor,
        start_cell=(18, 2),
        goal_cell=(2, 28),
        map_hash="map-v1",
        map_origin=(0.0, 0.0, 0.0),
        resolution=0.05,
        topology_hash="topology-v1",
        route_edge_ids=(route,),
        footprint_hash="jackal",
        route_signature=route,
    )


def snapshot(index, occupied):
    return DynamicSnapshot.from_cells(
        f"S{index}", occupied, timestamp=float(index),
        map_version="map-v1", map_shape=(21, 31),
    )


def test_r1_preserves_confirmation_roi_ack_and_runtime_contract(tmp_path):
    controller = Layered3DV1R1Controller(
        make_plan(), cache_root=tmp_path,
        dynamic_inflation_radius_cells=0,
        dstar_wall_budget_ms=1000.0,
        verify_l2_oracle=True,
    )
    path = controller.l2.path_global
    cell = path[len(path) // 2]
    pending = controller.process_snapshot(snapshot(1, [cell]), now=1.0)
    assert pending.scheduler.reason == "DUPLICATE_OR_UNCONFIRMED_OBSERVATION"
    confirmed = controller.process_snapshot(snapshot(2, [cell]), now=2.0)
    assert confirmed.l3_required
    assert confirmed.dirty_roi.closed_cells == 1
    with pytest.raises(ValueError):
        controller.acknowledge_l3_mask("wrong")
    controller.acknowledge_l3_mask(confirmed.dirty_roi.target_hash)
    contract = controller.runtime_contract
    assert contract["revision_id"] == "r1-l2-state-lifecycle-soak"
    assert contract["max_active_dstar_states"] == 1
    assert contract["server_content_ack_required_before_smac"] is True
    assert contract["fixed_settle_cycles_after_ack"] == 0
    assert contract["smac_angle_quantization_bins"] == 48
    assert contract["canonical_path_audit_reused"] is True


def test_production_l3_adapter_reuses_one_canonical_audit(tmp_path):
    controller = Layered3DV1R1Controller(
        make_plan(), cache_root=tmp_path,
        dynamic_inflation_radius_cells=0,
        dstar_wall_budget_ms=1000.0,
    )
    path = controller.l2.path_global
    cell = path[len(path) // 2]
    controller.process_snapshot(snapshot(1, [cell]), now=1.0)
    step = controller.process_snapshot(snapshot(2, [cell]), now=2.0)

    result = SimpleNamespace(
        diagnostics={
            "costmap_update_acknowledged": True,
            "costmap_ack_mismatch_cells": 0,
        },
        planner_success=True,
        points=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        failure_code="",
        path_audit=None,
    )

    class Session:
        local_map_update_strategy = "roi_ack"
        full_grid_settle_cycles = 0

        def plan(self, *args, **kwargs):
            return result

    class Auditor:
        calls = 0

        def audit(self, query, points, mask):
            self.calls += 1
            return SimpleNamespace(
                metrics={
                    "static_footprint_valid": True,
                    "kinematic_valid": True,
                    "failure_code": "",
                },
                within_mask=True,
                diagnostics=lambda: {"audit_instance": self.calls},
            )

    auditor = Auditor()
    outcome = ProductionL3Adapter(controller, auditor).plan(
        step, SimpleNamespace(), Session(), SimpleNamespace(),
    )
    assert outcome["success"]
    assert outcome["canonical_path_audit_reused"]
    assert auditor.calls == 1
    assert result.path_audit is not None
