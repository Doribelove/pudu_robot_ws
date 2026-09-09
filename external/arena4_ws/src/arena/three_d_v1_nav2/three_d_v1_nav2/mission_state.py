"""Pure fail-closed state machine for eight independent NavigateToPose goals."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence


class MissionState(str, Enum):
    WAIT_STACK_ACTIVE = "WAIT_STACK_ACTIVE"
    SET_INITIAL_POSE = "SET_INITIAL_POSE"
    WAIT_LOCALIZATION = "WAIT_LOCALIZATION"
    SEND_GOAL = "SEND_GOAL"
    ACTIVE = "ACTIVE"
    VERIFY_SUCCEEDED = "VERIFY_SUCCEEDED"
    STOPPED_AND_AUDITED = "STOPPED_AND_AUDITED"
    ADVANCE = "ADVANCE"
    COMPLETE = "COMPLETE"
    HALT_CURRENT_QUERY = "HALT_CURRENT_QUERY"


@dataclass(frozen=True)
class Acceptance:
    action_succeeded: bool
    within_position_tolerance: bool
    within_yaw_tolerance: bool
    stopped: bool
    canonical_final_audit_passed: bool

    @property
    def all_passed(self) -> bool:
        return all((
            self.action_succeeded, self.within_position_tolerance,
            self.within_yaw_tolerance, self.stopped, self.canonical_final_audit_passed,
        ))


class MissionMachine:
    """Index advances only after the exact five-part acceptance conjunction."""

    def __init__(self, query_ids: Sequence[str]) -> None:
        if not query_ids or len(query_ids) != len(set(query_ids)):
            raise ValueError("mission needs a non-empty, unique ordered query list")
        self.query_ids = tuple(query_ids)
        self.query_index = 0
        self.state = MissionState.WAIT_STACK_ACTIVE
        self.goal_uuid: Optional[str] = None
        self.used_goal_uuids: set[str] = set()
        self.failure_code = ""

    @property
    def query_id(self) -> str:
        return self.query_ids[self.query_index]

    def stack_active(self) -> None:
        self._require(MissionState.WAIT_STACK_ACTIVE)
        self.state = MissionState.SET_INITIAL_POSE

    def initial_pose_set(self) -> None:
        self._require(MissionState.SET_INITIAL_POSE)
        self.state = MissionState.WAIT_LOCALIZATION

    def localization_ready(self) -> None:
        self._require(MissionState.WAIT_LOCALIZATION)
        self.state = MissionState.SEND_GOAL

    def goal_sent(self, goal_uuid: str) -> None:
        self._require(MissionState.SEND_GOAL)
        if not goal_uuid or goal_uuid in self.used_goal_uuids:
            self.halt("DUPLICATE_OR_EMPTY_GOAL_UUID")
            return
        self.goal_uuid = goal_uuid
        self.used_goal_uuids.add(goal_uuid)
        self.state = MissionState.ACTIVE

    def action_result(self, succeeded: bool, failure_code: str = "NAV2_ACTION_FAILED") -> None:
        self._require(MissionState.ACTIVE)
        if not succeeded:
            self.halt(failure_code)
            return
        self.state = MissionState.VERIFY_SUCCEEDED

    def verify(self, acceptance: Acceptance) -> None:
        self._require(MissionState.VERIFY_SUCCEEDED)
        if not acceptance.all_passed:
            failed = [name for name, passed in (
                ("ACTION", acceptance.action_succeeded),
                ("POSITION_TOLERANCE", acceptance.within_position_tolerance),
                ("YAW_TOLERANCE", acceptance.within_yaw_tolerance),
                ("STOP", acceptance.stopped),
                ("FINAL_PATH_AUDIT", acceptance.canonical_final_audit_passed),
            ) if not passed]
            self.halt("ACCEPTANCE_FAILED_" + "_".join(failed))
            return
        self.state = MissionState.STOPPED_AND_AUDITED

    def advance(self) -> None:
        self._require(MissionState.STOPPED_AND_AUDITED)
        self.state = MissionState.ADVANCE
        if self.query_index + 1 >= len(self.query_ids):
            self.state = MissionState.COMPLETE
            return
        self.query_index += 1
        self.goal_uuid = None
        self.state = MissionState.SET_INITIAL_POSE

    def halt(self, failure_code: str) -> None:
        if self.state in (MissionState.COMPLETE, MissionState.HALT_CURRENT_QUERY):
            return
        self.failure_code = failure_code or "UNSPECIFIED_FAILURE"
        self.state = MissionState.HALT_CURRENT_QUERY

    def _require(self, expected: MissionState) -> None:
        if self.state != expected:
            current = self.state
            self.halt(f"INVALID_TRANSITION_{current.value}_EXPECTED_{expected.value}")
            raise RuntimeError(f"invalid mission transition from {current.value}; expected {expected.value}")
