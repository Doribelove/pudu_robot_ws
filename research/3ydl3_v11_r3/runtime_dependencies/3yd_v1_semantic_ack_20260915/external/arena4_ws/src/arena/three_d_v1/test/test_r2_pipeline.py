from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_3d_v1.l2_incremental import CorridorROI
from arena_3d_v1.pipeline import L1Plan, ProductionL3Adapter
from arena_3d_v1.r2_pipeline import Layered3DV1R2Controller
from arena_3d_v1.r2_state_lifecycle import R2L2StateLifecycleManager


def make_plan(route="r2"):
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


def make_roi(plan):
    return CorridorROI.from_global(
        plan.static_safe_free, plan.corridor_mask,
        plan.start_cell, plan.goal_cell,
        binding_fields=plan.binding_fields(),
    )


def snapshot(index, occupied):
    return DynamicSnapshot.from_cells(
        f"S{index}", occupied, timestamp=float(index),
        map_version="map-v1", map_shape=(21, 31),
    )


def prepared_controller(tmp_path):
    plan = make_plan()
    manager = R2L2StateLifecycleManager(tmp_path, max_active_states=1)
    assert manager.prebuild(make_roi(plan), verify_oracle=True).success
    return Layered3DV1R2Controller(
        plan, cache_root=tmp_path, lifecycle_manager=manager,
        dynamic_inflation_radius_cells=0,
        dstar_wall_budget_ms=1000.0,
        verify_l2_oracle=True,
    )


def test_r2_preserves_roi_ack_contract_and_emits_phase_telemetry(tmp_path):
    controller = prepared_controller(tmp_path)
    path = controller.l2.path_global
    cell = path[len(path) // 2]
    pending = controller.process_snapshot(snapshot(1, [cell]), now=1.0)
    assert pending.scheduler.reason == "DUPLICATE_OR_UNCONFIRMED_OBSERVATION"
    confirmed = controller.process_snapshot(snapshot(2, [cell]), now=2.0)
    assert confirmed.l3_required
    assert confirmed.dirty_roi.closed_cells == 1
    for field in (
        "confirmation_wall_ms", "scheduler_wall_ms", "target_mask_wall_ms",
        "l2_dispatch_wall_ms", "dirty_roi_wall_ms", "pipeline_response_ms",
        "end_to_end_pre_l3_ms",
    ):
        assert field in confirmed.diagnostics
        assert confirmed.diagnostics[field] >= 0
    assert confirmed.diagnostics["pipeline_response_ms"] >= confirmed.diagnostics["l2_dispatch_wall_ms"]
    with pytest.raises(ValueError):
        controller.acknowledge_l3_mask("wrong")
    controller.acknowledge_l3_mask(confirmed.dirty_roi.target_hash)
    contract = controller.runtime_contract
    assert contract["revision_id"] == "r2-production-acceptance-real-replay"
    assert contract["online_synchronous_dstar_build"] is False
    assert contract["smac_angle_quantization_bins"] == 48
    assert contract["smac_motion_model"] == "DUBIN"
    assert contract["canonical_path_audit_reused"] is True


def test_r2_cache_miss_controller_remains_safe_and_never_builds(tmp_path):
    controller = Layered3DV1R2Controller(
        make_plan(), cache_root=tmp_path,
        dynamic_inflation_radius_cells=0,
        verify_l2_oracle=True,
    )
    assert controller.initial_l2_result.success
    assert controller.initial_l2_result.selected_backend == "deterministic_grid_astar_cache_miss"
    assert controller.lifecycle.synchronous_dstar_build_count == 0
    assert len(controller.lifecycle.active) == 0


def test_r2_production_l3_adapter_reuses_one_canonical_audit(tmp_path):
    controller = prepared_controller(tmp_path)
    path = controller.l2.path_global
    cell = path[len(path) // 2]
    controller.process_snapshot(snapshot(1, [cell]), now=1.0)
    step = controller.process_snapshot(snapshot(2, [cell]), now=2.0)
    result = SimpleNamespace(
        diagnostics={"costmap_update_acknowledged": True, "costmap_ack_mismatch_cells": 0},
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
                metrics={"static_footprint_valid": True, "kinematic_valid": True, "failure_code": ""},
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
