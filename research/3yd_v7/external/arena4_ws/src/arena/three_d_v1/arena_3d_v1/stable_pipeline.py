"""Stable production facade over the frozen, accepted 3D-V1/r2 implementation."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping, Optional

from arena_evaluation.dynamic_snapshot import DynamicSnapshot

from .l2_incremental import L2PlanResult
from .pipeline import L1Plan, PipelineStep, ProductionL3Adapter
from .r1_pipeline import L1Replan
from .r2_pipeline import Layered3DV1R2Controller
from .r2_state_lifecycle import R2L2StateLifecycleManager
from .stable_contract import (
    ARCHITECTURE_ID,
    PRODUCTION_BASELINE_ID,
    PROTOCOL_ID,
    RELEASE_CANDIDATE_ID,
    SOURCE_REVISION,
)


def _actual_cache_status(manager: R2L2StateLifecycleManager) -> str:
    activation = manager.last_activation
    if activation is None:
        return "NOT_EVALUATED"
    if getattr(activation, "active_hit", False):
        return "HIT_ACTIVE_MEMORY"
    geometry = getattr(activation, "geometry_cache", None)
    state = getattr(activation, "state_cache", None)
    if bool(getattr(geometry, "hit", False)) and bool(getattr(state, "hit", False)):
        return "HIT_VERIFIED_R2_CACHE"
    reasons = [
        str(getattr(geometry, "reject_reason", "") or ""),
        str(getattr(state, "reject_reason", "") or ""),
    ]
    reason = "+".join(item for item in reasons if item) or "CACHE_MISS"
    return f"REJECTED_TO_ASTAR:{reason}"


class Layered3DV1StableController(Layered3DV1R2Controller):
    """Only default production controller; behavior is delegated to frozen r2."""

    def __init__(
        self,
        initial_plan: L1Plan,
        *,
        cache_root: Path,
        declared_cache_status: str = "AUTO",
        stable_config_sha256: str = "",
        max_active_states: int = 1,
        dynamic_inflation_radius_cells: int = 7,
        confidence_threshold: float = 0.60,
        dstar_wall_budget_ms: float = 500.0,
        dstar_max_expansions: int = 20_000,
        dstar_attempt_max_changed_cells: int = 2,
        verify_l2_oracle: bool = False,
        lifecycle_manager: Optional[R2L2StateLifecycleManager] = None,
    ) -> None:
        self.declared_cache_status = str(declared_cache_status)
        self.stable_config_sha256 = str(stable_config_sha256)
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
            lifecycle_manager=lifecycle_manager,
        )
        initial_reason = (
            "CACHE_MISS_OR_REJECT"
            if "astar" in self.initial_l2_result.selected_backend.lower()
            else "NONE"
        )
        self.initial_l2_result = self._annotate_result(
            self.initial_l2_result, fallback_reason=initial_reason
        )

    @property
    def cache_status(self) -> str:
        actual = _actual_cache_status(self.lifecycle)
        if self.declared_cache_status == "AUTO":
            return actual
        if (
            self.declared_cache_status == "HIT_VERIFIED_STABLE_BUNDLE"
            and actual.startswith("REJECTED_TO_ASTAR")
        ):
            return f"RUNTIME_{actual}"
        return self.declared_cache_status

    @property
    def runtime_contract(self) -> Mapping[str, Any]:
        contract = dict(super().runtime_contract)
        contract.update({
            "architecture_id": ARCHITECTURE_ID,
            "production_baseline_id": PRODUCTION_BASELINE_ID,
            "source_revision": SOURCE_REVISION,
            "release_candidate": RELEASE_CANDIDATE_ID,
            "protocol_id": PROTOCOL_ID,
            "revision_id": PRODUCTION_BASELINE_ID,
            "selected_backend": self.initial_l2_result.selected_backend,
            "fallback_reason": self.initial_l2_result.diagnostics.get(
                "fallback_reason", "NONE"
            ),
            "cache_status": self.cache_status,
            "minimum_turning_radius_m": 0.40,
            "maximum_curvature_1pm": 2.50,
            "allow_reverse": False,
            "allow_in_place_rotation": False,
            "pure_dstar_production_mode": False,
            "stable_config_sha256": self.stable_config_sha256,
        })
        return contract

    def _annotate_result(
        self, result: L2PlanResult, *, fallback_reason: str
    ) -> L2PlanResult:
        diagnostics = dict(result.diagnostics)
        diagnostics.update({
            "architecture_id": ARCHITECTURE_ID,
            "production_baseline_id": PRODUCTION_BASELINE_ID,
            "source_revision": SOURCE_REVISION,
            "selected_backend": result.selected_backend,
            "fallback_reason": str(fallback_reason),
            "cache_status": self.cache_status,
            "online_synchronous_dstar_build": 0,
        })
        return replace(result, diagnostics=diagnostics)

    def _fallback_reason(self, step: PipelineStep) -> str:
        result = step.l2_result
        if result is None:
            return "NONE" if not step.failure_code else step.failure_code
        backend = result.selected_backend.lower()
        if "dstar" in backend and "astar" not in backend:
            return "NONE"
        if "cache_miss" in backend or self.cache_status.startswith("REJECTED"):
            return "CACHE_MISS_OR_REJECT"
        if step.snapshot_update.newly_freed_sources:
            return "RECOVERY"
        if len(step.snapshot_update.newly_blocked_sources) > self.dstar_attempt_max_changed_cells:
            return "LARGE_CHANGE"
        if bool(getattr(result.dstar_stats, "timeout_triggered", False)):
            return "DSTAR_TIMEOUT"
        if not self.l2.dstar_ready:
            return "DSTAR_NOT_READY"
        if step.failure_code:
            return step.failure_code
        return "POLICY_FORCED_DETERMINISTIC_ASTAR"

    def process_snapshot(
        self,
        snapshot: DynamicSnapshot,
        *,
        l1_replan: Optional[L1Replan] = None,
        now: Optional[float] = None,
    ) -> PipelineStep:
        step = super().process_snapshot(snapshot, l1_replan=l1_replan, now=now)
        selected_backend = (
            "scheduler_skip" if step.l2_result is None
            else step.l2_result.selected_backend
        )
        fallback_reason = self._fallback_reason(step)
        result = (
            None if step.l2_result is None
            else self._annotate_result(step.l2_result, fallback_reason=fallback_reason)
        )
        diagnostics = dict(step.diagnostics)
        diagnostics.update({
            "architecture_id": ARCHITECTURE_ID,
            "production_baseline_id": PRODUCTION_BASELINE_ID,
            "source_revision": SOURCE_REVISION,
            "selected_backend": selected_backend,
            "fallback_reason": fallback_reason,
            "cache_status": self.cache_status,
            "online_synchronous_dstar_build": 0,
        })
        return replace(step, l2_result=result, diagnostics=diagnostics)


class StableProductionL3Adapter(ProductionL3Adapter):
    """Stable name for the frozen ROI/ACK/Smac/PathAudit boundary."""

    def __init__(self, controller: Layered3DV1StableController, auditor: Any) -> None:
        if not isinstance(controller, Layered3DV1StableController):
            raise TypeError("stable L3 adapter requires the stable production controller")
        super().__init__(controller, auditor)


__all__ = ["Layered3DV1StableController", "StableProductionL3Adapter"]
