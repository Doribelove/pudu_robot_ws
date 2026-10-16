"""3D-V1/r1 composition root with compact cached L2 state lifecycle."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

import numpy as np

from arena_evaluation.dynamic_snapshot import DynamicSnapshot

from .dynamic_policy import DynamicGridConfirmation, RelevanceScheduler
from .l2_incremental import Cell, L2PlanResult
from .l2_state_lifecycle import (
    ARCHITECTURE_ID,
    DEFAULT_DYNAMIC_BASELINE,
    L2StateLifecycleManager,
    PROTOCOL_ID,
    REVISION_ID,
)
from .pipeline import DirtyROI, L1Plan, PipelineStep, corridor_dirty_transition


PARENT_ARCHITECTURE = "3D-V1-r0-production-substrate-l2-dstar-v1"
L1Replan = Callable[[Sequence[Cell]], Optional[L1Plan]]


class Layered3DV1R1Controller:
    """Frozen r0 policy with a compact cached and LRU-bounded L2 backend."""

    def __init__(
        self,
        initial_plan: L1Plan,
        *,
        cache_root: Path,
        max_active_states: int = 1,
        dynamic_inflation_radius_cells: int = 7,
        confidence_threshold: float = 0.60,
        dstar_wall_budget_ms: float = 500.0,
        dstar_max_expansions: int = 20_000,
        dstar_attempt_max_changed_cells: int = 2,
        verify_l2_oracle: bool = False,
        lifecycle_manager: Optional[L2StateLifecycleManager] = None,
    ) -> None:
        self.dynamic_inflation_radius_cells = int(dynamic_inflation_radius_cells)
        self.dstar_wall_budget_ms = float(dstar_wall_budget_ms)
        self.dstar_max_expansions = int(dstar_max_expansions)
        self.dstar_attempt_max_changed_cells = max(0, int(dstar_attempt_max_changed_cells))
        self.verify_l2_oracle = bool(verify_l2_oracle)
        self.confirmation = DynamicGridConfirmation(
            map_version=initial_plan.map_hash,
            map_shape=initial_plan.corridor_mask.shape,
            inflation_radius_cells=dynamic_inflation_radius_cells,
            confidence_threshold=confidence_threshold,
        )
        self.lifecycle = lifecycle_manager or L2StateLifecycleManager(
            cache_root,
            max_active_states=max_active_states,
            dstar_wall_budget_ms=dstar_wall_budget_ms,
            dstar_max_expansions=dstar_max_expansions,
        )
        self.plan: L1Plan
        self.scheduler: RelevanceScheduler
        self.l2: Any
        self.l1_rebind_count = 0
        self.server_l3_mask = np.asarray(initial_plan.corridor_mask, dtype=bool).copy()
        self.pending_l3_mask = self.server_l3_mask.copy()
        self._pending_l3_hash = ""
        self.initial_l2_result = self._bind_l1_plan(initial_plan)

    @property
    def runtime_contract(self) -> Mapping[str, Any]:
        return {
            "architecture_id": ARCHITECTURE_ID,
            "revision_id": REVISION_ID,
            "protocol_id": PROTOCOL_ID,
            "parent_architecture": PARENT_ARCHITECTURE,
            "explicitly_not_derived_from": "3D-V0",
            "map_resolution_m": 0.05,
            "l1": "deterministic_graph_astar",
            "l2": "compact_cached_persistent_dstar_lite_cropped_corridor_roi",
            "l2_multiresolution": False,
            "corridor_profile": "topology_turn_adaptive_2m_4m",
            "max_active_dstar_states": self.lifecycle.max_active_states,
            "hard_max_active_dstar_states": self.lifecycle.HARD_MAX_ACTIVE_STATES,
            "roi_max_message_bytes": 128 * 1024,
            "server_content_ack_required_before_smac": True,
            "fixed_settle_cycles_after_ack": 0,
            "smac_angle_quantization_bins": 48,
            "smac_motion_model": "DUBIN",
            "allow_reverse": False,
            "allow_in_place_rotation": False,
            "canonical_path_audit_reused": True,
        }

    def _bind_l1_plan(
        self,
        plan: L1Plan,
        blocked_cells: Sequence[Cell] = (),
    ) -> L2PlanResult:
        if abs(float(plan.resolution) - 0.05) > 1.0e-12:
            raise ValueError("3D-V1-r1 accepts only the project 0.05 m/cell map")
        from .l2_incremental import CorridorROI
        roi = CorridorROI.from_global(
            plan.static_safe_free, plan.corridor_mask,
            plan.start_cell, plan.goal_cell,
            binding_fields=plan.binding_fields(),
        )
        self.plan = plan
        self.scheduler = RelevanceScheduler(plan.corridor_mask)
        self.l2, result, activation = self.lifecycle.activate(
            roi,
            dynamic_baseline_version=DEFAULT_DYNAMIC_BASELINE,
            verify_oracle=self.verify_l2_oracle,
        )
        if blocked_cells:
            result = self.l2.update(
                blocked_cells,
                verify_oracle=self.verify_l2_oracle,
                force_cold_astar=True,
            )
        result.diagnostics["activation"] = activation.as_dict()
        return result

    def _target_mask(self) -> np.ndarray:
        target = np.asarray(self.plan.corridor_mask, dtype=bool).copy()
        for cell in self.confirmation.blocked_cells:
            if target[cell]:
                target[cell] = False
        return target

    def process_snapshot(
        self,
        snapshot: DynamicSnapshot,
        *,
        l1_replan: Optional[L1Replan] = None,
        now: Optional[float] = None,
    ) -> PipelineStep:
        started_ns = time.monotonic_ns()
        update = self.confirmation.consume(snapshot, now=now)
        decision = self.scheduler.decide(update, self.l2.path_global)
        target = self._target_mask()
        self.pending_l3_mask = target
        if not decision.invoke_l2:
            return PipelineStep(
                update, decision, None, False, False, False,
                update.rejection_reason if not update.accepted else "",
                None, None, self.plan.route_signature,
                {
                    "scheduler_skip": True,
                    "pipeline_response_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                    "pending_l3_dirty_cells": int(np.count_nonzero(
                        self.server_l3_mask ^ self.pending_l3_mask
                    )),
                    "active_state_count": len(self.lifecycle.active),
                    "resident_bytes": self.lifecycle.resident_bytes,
                },
            )

        l2_result = self.l2.update(
            update.blocked_cells,
            verify_oracle=self.verify_l2_oracle,
            force_cold_astar=(
                bool(update.newly_freed_sources)
                or len(update.newly_blocked_sources) > self.dstar_attempt_max_changed_cells
                or not self.l2.dstar_ready
            ),
        )
        l1_called = False
        reroute_succeeded = False
        failure = ""
        if not l2_result.success:
            if l1_replan is None:
                failure = "L2_NO_PATH_NEEDS_L1_REROUTE"
            else:
                l1_called = True
                replacement = l1_replan(update.blocked_cells)
                if replacement is None:
                    failure = "L1_NO_ROUTE"
                else:
                    previous_binding = self.l2.binding_hash
                    l2_result = self._bind_l1_plan(replacement, update.blocked_cells)
                    self.l1_rebind_count += 1
                    reroute_succeeded = l2_result.success
                    if self.l2.binding_hash == previous_binding:
                        raise AssertionError("L1 reroute returned an unchanged L2 binding")
                    failure = "" if reroute_succeeded else "L2_NO_PATH_AFTER_L1_REROUTE"
                    target = self._target_mask()
                    self.pending_l3_mask = target

        l3_required = bool(l2_result.success and not failure)
        dirty: Optional[DirtyROI] = (
            corridor_dirty_transition(self.server_l3_mask, target)
            if l3_required else None
        )
        if dirty is not None:
            if dirty.old_state_residual_cells:
                raise AssertionError("old/new ROI transition failed to close stale cells")
            self._pending_l3_hash = dirty.target_hash
        return PipelineStep(
            update, decision, l2_result, l1_called, reroute_succeeded,
            l3_required, failure, target if l3_required else None, dirty,
            self.plan.route_signature,
            {
                "scheduler_skip": False,
                "l1_rebind_count": self.l1_rebind_count,
                "pipeline_response_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                "content_ack_required_before_smac": True,
                "fixed_settle_cycles": 0,
                "dstar_attempt_max_changed_cells": self.dstar_attempt_max_changed_cells,
                "dstar_selected": l2_result.selected_backend == "compact_persistent_dstar",
                "active_state_count": len(self.lifecycle.active),
                "resident_bytes": self.lifecycle.resident_bytes,
                "peak_active_state_count": self.lifecycle.peak_active_state_count,
                "peak_resident_bytes": self.lifecycle.peak_resident_bytes,
            },
        )

    def service_l2_resync(self) -> L2PlanResult:
        return self.l2.service_resync()

    def acknowledge_l3_mask(self, content_hash: str) -> None:
        if not self._pending_l3_hash:
            raise RuntimeError("no L3 mask is awaiting acknowledgement")
        if str(content_hash) != self._pending_l3_hash:
            raise ValueError("server content ACK hash does not match the target L3 mask")
        self.server_l3_mask = self.pending_l3_mask.copy()
        self._pending_l3_hash = ""


__all__ = ["Layered3DV1R1Controller", "PARENT_ARCHITECTURE"]
