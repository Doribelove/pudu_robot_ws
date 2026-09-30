"""Observable, hard-invariant-preserving preference relaxation for 2A-V2."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple


HARD_INVARIANTS = {
    "hard_semantics": True,
    "static_obstacles": True,
    "footprint_collision_check": True,
    "allow_reverse": False,
    "allow_in_place_rotation": False,
    "minimum_turning_radius_m": 0.40,
    "maximum_curvature_1pm": 2.50,
    "legal_endpoints": True,
    "no_stopping_goal_rejected": True,
}


@dataclass
class RelaxationAttempt:
    relaxation_level: str
    trigger_reason: str
    failure_stage_before_relaxation: str
    relaxed_parameters: Dict[str, Any]
    hard_constraints: Dict[str, Any]
    hard_constraints_held: bool
    success: bool
    failure_code: str


@dataclass
class RelaxationResult:
    success: bool
    relaxation_level: str
    failure_code: str
    value: Any = None
    attempts: List[RelaxationAttempt] = field(default_factory=list)


class PreferenceRelaxationController:
    def __init__(
        self, *, enabled: bool = True, mode: str = "graceful",
        levels: Sequence[str] = ("R0", "R1", "R2", "R3", "R4"),
    ) -> None:
        if mode not in {"graceful", "strict"}:
            raise ValueError("preference relaxation mode must be graceful or strict")
        supplied = list(levels)
        if not supplied or supplied[0] != "R0" or any(
            value not in {"R0", "R1", "R2", "R3", "R4"} for value in supplied
        ):
            raise ValueError("relaxation levels must start at R0 and contain only R0..R4")
        self.enabled = bool(enabled)
        self.mode = mode
        self.levels = supplied if self.enabled and mode == "graceful" else ["R0"]

    @staticmethod
    def parameters(level: str) -> Dict[str, Any]:
        return {
            "R0": {"lateral_weight_scale": 1.0, "selective_disable": False, "roi_mode": "base"},
            "R1": {"lateral_weight_scale": 0.35, "selective_disable": False, "roi_mode": "base"},
            "R2": {"lateral_weight_scale": 0.35, "selective_disable": True, "roi_mode": "base"},
            "R3": {"lateral_weight_scale": 0.35, "selective_disable": True, "roi_mode": "expanded"},
            "R4": {"lateral_weight_scale": 0.35, "selective_disable": True, "roi_mode": "full_map"},
        }[level]

    def run(
        self,
        attempt: Callable[[str, Mapping[str, Any]], Tuple[bool, Any, str, str, bool]],
    ) -> RelaxationResult:
        """Run bounded attempts.

        The callback returns ``(success, value, failure_code, failure_stage,
        hard_constraints_held)``.  A hard violation terminates immediately;
        only soft-preference/search-space failures may advance a level.
        """
        records: List[RelaxationAttempt] = []
        trigger = "initial_request"
        previous_stage = "none"
        last_code = "SEMANTIC_PLANNING_FAILED"
        for level in self.levels:
            params = self.parameters(level)
            success, value, failure_code, failure_stage, hard_held = attempt(level, params)
            last_code = str(failure_code or last_code)
            record = RelaxationAttempt(
                relaxation_level=level,
                trigger_reason=trigger,
                failure_stage_before_relaxation=previous_stage,
                relaxed_parameters=dict(params),
                hard_constraints=dict(HARD_INVARIANTS),
                hard_constraints_held=bool(hard_held),
                success=bool(success),
                failure_code="" if success else last_code,
            )
            records.append(record)
            if not hard_held:
                return RelaxationResult(False, level, "HARD_CONSTRAINT_VIOLATION", value, records)
            if success:
                return RelaxationResult(True, level, "", value, records)
            trigger = last_code
            previous_stage = str(failure_stage or "unknown")
        return RelaxationResult(False, self.levels[-1], last_code, None, records)


__all__ = [
    "HARD_INVARIANTS", "RelaxationAttempt", "RelaxationResult",
    "PreferenceRelaxationController",
]
