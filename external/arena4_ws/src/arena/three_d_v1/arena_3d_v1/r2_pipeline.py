"""3D-V1/r2 production-acceptance composition root and phase telemetry."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from arena_evaluation.dynamic_snapshot import DynamicSnapshot

from .l2_incremental import Cell
from .pipeline import DirtyROI, L1Plan, PipelineStep, corridor_dirty_transition
from .r1_pipeline import L1Replan, Layered3DV1R1Controller
from .r2_state_lifecycle import (
    ARCHITECTURE_ID,
    PROTOCOL_ID,
    REVISION_ID,
    R2L2StateLifecycleManager,
)


PARENT_ARCHITECTURE = "3D-V1-r1-l2-state-lifecycle-soak"


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


class Layered3DV1R2Controller(Layered3DV1R1Controller):
    """r1 production contracts plus packed/cache-only r2 L2 lifecycle."""

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
        lifecycle_manager: Optional[R2L2StateLifecycleManager] = None,
    ) -> None:
        lifecycle = lifecycle_manager or R2L2StateLifecycleManager(
            cache_root,
            max_active_states=max_active_states,
            dstar_wall_budget_ms=dstar_wall_budget_ms,
            dstar_max_expansions=dstar_max_expansions,
            require_prebuilt_cache=True,
        )
        super().__init__(
            initial_plan,
            cache_root=cache_root,
            max_active_states=max_active_states,
            dynamic_inflation_radius_cells=dynamic_inflation_radius_cells,
            confidence_threshold=confidence_threshold,
            dstar_wall_budget_ms=dstar_wall_budget_ms,
            dstar_max_expansions=dstar_max_expansions,
            dstar_attempt_max_changed_cells=dstar_attempt_max_changed_cells,
            verify_l2_oracle=verify_l2_oracle,
            lifecycle_manager=lifecycle,
        )

    @property
    def runtime_contract(self) -> Mapping[str, Any]:
        contract = dict(super().runtime_contract)
        contract.update({
            "architecture_id": ARCHITECTURE_ID,
            "revision_id": REVISION_ID,
            "protocol_id": PROTOCOL_ID,
            "parent_architecture": PARENT_ARCHITECTURE,
            "l2": "verified-cache-only-compact-dstar+packed-static-roi+deterministic-astar-miss",
            "online_synchronous_dstar_build": False,
            "cache_miss_backend": "deterministic_grid_astar",
            "static_roi_storage": "little-endian-bitset",
        })
        return contract

    def process_snapshot(
        self,
        snapshot: DynamicSnapshot,
        *,
        l1_replan: Optional[L1Replan] = None,
        now: Optional[float] = None,
    ) -> PipelineStep:
        pipeline_started = time.monotonic_ns()
        started = time.monotonic_ns()
        update = self.confirmation.consume(snapshot, now=now)
        confirmation_wall_ms = _elapsed_ms(started)
        started = time.monotonic_ns()
        decision = self.scheduler.decide(update, self.l2.path_global)
        scheduler_wall_ms = _elapsed_ms(started)
        started = time.monotonic_ns()
        target = self._target_mask()
        target_mask_wall_ms = _elapsed_ms(started)
        self.pending_l3_mask = target
        if not decision.invoke_l2:
            pipeline_ms = _elapsed_ms(pipeline_started)
            return PipelineStep(
                update, decision, None, False, False, False,
                update.rejection_reason if not update.accepted else "",
                None, None, self.plan.route_signature,
                {
                    "scheduler_skip": True,
                    "confirmation_wall_ms": confirmation_wall_ms,
                    "scheduler_wall_ms": scheduler_wall_ms,
                    "target_mask_wall_ms": target_mask_wall_ms,
                    "l2_dispatch_wall_ms": 0.0,
                    "l1_replan_wall_ms": 0.0,
                    "dirty_roi_wall_ms": 0.0,
                    "pipeline_response_ms": pipeline_ms,
                    "end_to_end_pre_l3_ms": pipeline_ms,
                    "pending_l3_dirty_cells": int(np.count_nonzero(
                        self.server_l3_mask ^ self.pending_l3_mask
                    )),
                    "active_state_count": len(self.lifecycle.active),
                    "resident_bytes": self.lifecycle.resident_bytes,
                    "lifecycle": dict(self.lifecycle.telemetry()),
                },
            )

        started = time.monotonic_ns()
        l2_result = self.l2.update(
            update.blocked_cells,
            verify_oracle=self.verify_l2_oracle,
            force_cold_astar=(
                bool(update.newly_freed_sources)
                or len(update.newly_blocked_sources) > self.dstar_attempt_max_changed_cells
                or not self.l2.dstar_ready
            ),
        )
        l2_dispatch_wall_ms = _elapsed_ms(started)
        l1_called = False
        reroute_succeeded = False
        failure = ""
        l1_replan_wall_ms = 0.0
        if not l2_result.success:
            if l1_replan is None:
                failure = "L2_NO_PATH_NEEDS_L1_REROUTE"
            else:
                l1_called = True
                started = time.monotonic_ns()
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
                l1_replan_wall_ms = _elapsed_ms(started)

        l3_required = bool(l2_result.success and not failure)
        started = time.monotonic_ns()
        dirty: Optional[DirtyROI] = (
            corridor_dirty_transition(self.server_l3_mask, target)
            if l3_required else None
        )
        dirty_roi_wall_ms = _elapsed_ms(started)
        if dirty is not None:
            if dirty.old_state_residual_cells:
                raise AssertionError("old/new ROI transition failed to close stale cells")
            self._pending_l3_hash = dirty.target_hash
        pipeline_ms = _elapsed_ms(pipeline_started)
        return PipelineStep(
            update, decision, l2_result, l1_called, reroute_succeeded,
            l3_required, failure, target if l3_required else None, dirty,
            self.plan.route_signature,
            {
                "scheduler_skip": False,
                "l1_rebind_count": self.l1_rebind_count,
                "confirmation_wall_ms": confirmation_wall_ms,
                "scheduler_wall_ms": scheduler_wall_ms,
                "target_mask_wall_ms": target_mask_wall_ms,
                "l2_dispatch_wall_ms": l2_dispatch_wall_ms,
                "l1_replan_wall_ms": l1_replan_wall_ms,
                "dirty_roi_wall_ms": dirty_roi_wall_ms,
                "pipeline_response_ms": pipeline_ms,
                "end_to_end_pre_l3_ms": pipeline_ms,
                "content_ack_required_before_smac": True,
                "fixed_settle_cycles": 0,
                "dstar_attempt_max_changed_cells": self.dstar_attempt_max_changed_cells,
                "dstar_selected": l2_result.selected_backend == "compact_persistent_dstar",
                "active_state_count": len(self.lifecycle.active),
                "resident_bytes": self.lifecycle.resident_bytes,
                "peak_active_state_count": self.lifecycle.peak_active_state_count,
                "peak_resident_bytes": self.lifecycle.peak_resident_bytes,
                "lifecycle": dict(self.lifecycle.telemetry()),
            },
        )


__all__ = ["Layered3DV1R2Controller", "PARENT_ARCHITECTURE"]
