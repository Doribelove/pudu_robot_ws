"""3D-V1 composition root and production L3 boundary.

The controller is deliberately ROS-independent.  It owns the dynamic state,
scheduler and L2 lifetime, while the adapter at the bottom calls the already
validated ROI/ACK Smac session and canonical PathAudit implementation.
"""

from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from arena_evaluation.dynamic_snapshot import DynamicSnapshot

from . import (
    ARCHITECTURE_ID,
    IMPLEMENTATION_REVISION,
    PARENT_ARCHITECTURE,
    PROTOCOL_VERSION,
)
from .dynamic_policy import (
    ConfirmedGridUpdate,
    DynamicGridConfirmation,
    RelevanceScheduler,
    SchedulerDecision,
)
from .l2_incremental import Cell, CorridorROI, L2PlanResult, PersistentCorridorDStar


def _grid_hash(mask: np.ndarray) -> str:
    return hashlib.sha256(
        np.ascontiguousarray(np.asarray(mask, dtype=np.uint8)).tobytes()
    ).hexdigest()


@dataclass(frozen=True)
class L1Plan:
    """Output contract of the latest deterministic Graph A* layer."""

    static_safe_free: np.ndarray
    corridor_mask: np.ndarray
    start_cell: Cell
    goal_cell: Cell
    map_hash: str
    map_origin: Tuple[float, float, float]
    resolution: float
    topology_hash: str
    route_edge_ids: Tuple[str, ...]
    footprint_hash: str
    route_signature: str
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def binding_fields(self) -> Mapping[str, Any]:
        return {
            "map_hash": self.map_hash,
            "map_origin": self.map_origin,
            "resolution": self.resolution,
            "topology_hash": self.topology_hash,
            "route_edge_ids": self.route_edge_ids,
            "corridor_mask_hash": _grid_hash(self.corridor_mask),
            "footprint_hash": self.footprint_hash,
        }


@dataclass(frozen=True)
class DirtyROI:
    bbox: Optional[Tuple[int, int, int, int]]
    changed_cells: int
    closed_cells: int
    opened_cells: int
    target_hash: str
    old_state_residual_cells: int


def corridor_dirty_transition(old_mask: np.ndarray, new_mask: np.ndarray) -> DirtyROI:
    """Exact old/new dirty union used by the ROI content-ACK publisher."""
    old = np.asarray(old_mask, dtype=bool)
    new = np.asarray(new_mask, dtype=bool)
    if old.shape != new.shape:
        raise ValueError("old and new L3 masks must have identical shapes")
    dirty = old ^ new
    rows, columns = np.nonzero(dirty)
    bbox = None if not len(rows) else (
        int(rows.min()), int(rows.max()) + 1,
        int(columns.min()), int(columns.max()) + 1,
    )
    applied = old.copy()
    applied[old & ~new] = False
    applied[new & ~old] = True
    return DirtyROI(
        bbox=bbox,
        changed_cells=int(np.count_nonzero(dirty)),
        closed_cells=int(np.count_nonzero(old & ~new)),
        opened_cells=int(np.count_nonzero(new & ~old)),
        target_hash=_grid_hash(new),
        old_state_residual_cells=int(np.count_nonzero(applied ^ new)),
    )


@dataclass(frozen=True)
class PipelineStep:
    snapshot_update: ConfirmedGridUpdate
    scheduler: SchedulerDecision
    l2_result: Optional[L2PlanResult]
    l1_graph_astar_called: bool
    l1_reroute_succeeded: bool
    l3_required: bool
    failure_code: str
    target_l3_mask: Optional[np.ndarray]
    dirty_roi: Optional[DirtyROI]
    route_signature: str
    diagnostics: Mapping[str, Any] = field(default_factory=dict)


L1Replan = Callable[[Sequence[Cell]], Optional[L1Plan]]


class Layered3DV1Controller:
    """Dynamic scheduler around L1 Graph A*, persistent L2 D*, and L3 Smac."""

    def __init__(
        self,
        initial_plan: L1Plan,
        *,
        dynamic_inflation_radius_cells: int = 7,
        confidence_threshold: float = 0.60,
        dstar_wall_budget_ms: float = 500.0,
        dstar_max_expansions: int = 20_000,
        dstar_attempt_max_changed_cells: int = 2,
        verify_l2_oracle: bool = False,
    ) -> None:
        self.dynamic_inflation_radius_cells = int(dynamic_inflation_radius_cells)
        self.dstar_wall_budget_ms = float(dstar_wall_budget_ms)
        self.dstar_max_expansions = int(dstar_max_expansions)
        self.dstar_attempt_max_changed_cells = max(
            0, int(dstar_attempt_max_changed_cells),
        )
        self.verify_l2_oracle = bool(verify_l2_oracle)
        self.confirmation = DynamicGridConfirmation(
            map_version=initial_plan.map_hash,
            map_shape=initial_plan.corridor_mask.shape,
            inflation_radius_cells=dynamic_inflation_radius_cells,
            confidence_threshold=confidence_threshold,
        )
        self.plan: L1Plan
        self.scheduler: RelevanceScheduler
        self.l2: PersistentCorridorDStar
        self.l1_rebind_count = 0
        self.server_l3_mask = np.asarray(initial_plan.corridor_mask, dtype=bool).copy()
        self.pending_l3_mask = self.server_l3_mask.copy()
        self._pending_l3_hash = ""
        self.initial_l2_result = self._bind_l1_plan(initial_plan, ())

    @property
    def runtime_contract(self) -> Mapping[str, Any]:
        return {
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "parent_architecture": PARENT_ARCHITECTURE,
            "explicitly_not_derived_from": "3D-V0",
            "protocol_version": PROTOCOL_VERSION,
            "map_resolution_m": 0.05,
            "l1": "deterministic_graph_astar",
            "l2": "persistent_dstar_lite_cropped_corridor_roi",
            "l2_multiresolution": False,
            "corridor_profile": "topology_turn_adaptive_2m_4m",
            "roi_max_message_bytes": 128 * 1024,
            "server_content_ack_required_before_smac": True,
            "fixed_settle_cycles_after_ack": 0,
            "smac_angle_quantization_bins": 48,
            "canonical_path_audit_reused": True,
        }

    def _bind_l1_plan(
        self, plan: L1Plan, blocked_cells: Sequence[Cell],
    ) -> L2PlanResult:
        if abs(float(plan.resolution) - 0.05) > 1.0e-12:
            raise ValueError("3D-V1 accepts only the project 0.05 m/cell map")
        roi = CorridorROI.from_global(
            plan.static_safe_free, plan.corridor_mask,
            plan.start_cell, plan.goal_cell,
            binding_fields=plan.binding_fields(),
        )
        self.plan = plan
        self.scheduler = RelevanceScheduler(plan.corridor_mask)
        self.l2 = PersistentCorridorDStar(
            roi,
            dstar_wall_budget_ms=self.dstar_wall_budget_ms,
            dstar_max_expansions=self.dstar_max_expansions,
        )
        self.l2.prime_blocked(blocked_cells)
        return self.l2.initialize(verify_oracle=self.verify_l2_oracle)

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
                },
            )

        l2_result = self.l2.update(
            update.blocked_cells,
            verify_oracle=self.verify_l2_oracle,
            force_cold_astar=(
                bool(update.newly_freed_sources)
                or len(update.newly_blocked_sources)
                > self.dstar_attempt_max_changed_cells
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
        dirty = corridor_dirty_transition(self.server_l3_mask, target) if l3_required else None
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
                "dstar_selected": l2_result.selected_backend == "persistent_dstar",
            },
        )

    def service_l2_resync(self) -> L2PlanResult:
        """Service explicitly-accounted D* maintenance during a quiet period."""
        return self.l2.service_resync()

    def acknowledge_l3_mask(self, content_hash: str) -> None:
        """Commit server state only after the ROI service reports content ACK."""
        if not self._pending_l3_hash:
            raise RuntimeError("no L3 mask is awaiting acknowledgement")
        if str(content_hash) != self._pending_l3_hash:
            raise ValueError("server content ACK hash does not match the target L3 mask")
        self.server_l3_mask = self.pending_l3_mask.copy()
        self._pending_l3_hash = ""


class ProductionL3Adapter:
    """Thin adapter over the validated ROI/ACK Smac and PathAudit runtime."""

    def __init__(self, controller: Layered3DV1Controller, auditor: Any) -> None:
        self.controller = controller
        self.auditor = auditor

    def plan(self, step: PipelineStep, query: Any, session: Any, smac_spec: Any) -> Dict[str, Any]:
        if not step.l3_required or step.target_l3_mask is None or step.dirty_roi is None:
            return {"called": False, "failure_code": step.failure_code}
        if getattr(session, "local_map_update_strategy", "") != "roi_ack":
            raise ValueError("3D-V1 requires the production ROI/ACK update strategy")
        if int(getattr(session, "full_grid_settle_cycles", -1)) != 0:
            raise ValueError("3D-V1 forbids fixed post-ACK settle cycles")
        result = session.plan(
            query, smac_spec, source="l3_prime_smac_hybrid",
            allowed_mask=step.target_l3_mask,
            window_start_index=0, window_end_index=-1,
            window_path_length_m=0.0,
            skip_path_mask_validation=True,
        )
        diagnostics = dict(result.diagnostics or {})
        if diagnostics.get("costmap_update_acknowledged") is not True:
            return {
                "called": True, "success": False,
                "failure_code": "COSTMAP_CONTENT_ACK_FAILED",
                "diagnostics": diagnostics,
            }
        # The session ACK compares the expected and server-side *costmap*
        # content hashes.  ``target_hash`` is the bool allowed-mask hash, so
        # the two hash domains must not be compared directly.
        self.controller.acknowledge_l3_mask(step.dirty_roi.target_hash)
        if not result.planner_success or not result.points:
            return {
                "called": True, "success": False,
                "failure_code": str(result.failure_code or "L3_PRIME_FAILED"),
                "diagnostics": diagnostics,
            }
        audit = self.auditor.audit(query, result.points, step.target_l3_mask)
        metrics = dict(getattr(audit, "metrics", {}) or {})
        valid = bool(
            metrics.get("static_footprint_valid")
            and metrics.get("kinematic_valid")
            and getattr(audit, "within_mask", False)
            and not metrics.get("failure_code")
        )
        result.path_audit = audit
        return {
            "called": True,
            "success": valid,
            "failure_code": "" if valid else str(
                metrics.get("failure_code") or "FINAL_VALIDATION_FAILED"
            ),
            "canonical_path_audit_reused": True,
            "metrics": metrics,
            "diagnostics": {**diagnostics, **dict(audit.diagnostics())},
            "result": result,
        }


__all__ = [
    "DirtyROI", "L1Plan", "Layered3DV1Controller", "PipelineStep",
    "ProductionL3Adapter", "corridor_dirty_transition",
]
