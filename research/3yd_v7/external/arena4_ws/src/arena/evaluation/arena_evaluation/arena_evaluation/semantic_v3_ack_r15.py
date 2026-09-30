"""Single-observation exact-content ACK for the 2A-V3 r15 fast loop."""
from __future__ import annotations

from typing import Any, Optional

import numpy as np

from .semantic_v3_ack_r14 import DeterministicReinflationSessionR14


class SingleObservationExactAckSessionR15(DeterministicReinflationSessionR14):
    """Keep r14's deterministic reset/replay but halve full-master RPCs.

    One fresh, sequence-bound full-master observation is sufficient to prove
    exact byte equality at the planning barrier.  Requiring a second identical
    8.6M-cell response did not strengthen the safety invariant and caused the
    long-run service queue failure measured by r14.  No mismatch, stale cell,
    timeout, or hash discrepancy is accepted; timeout repair remains disabled.
    """

    PUBLICATION_VERSION = "2A-V3-r15-single-observation-exact-ack-v1"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.exact_stable_observations = 1

    def _wait_for_costmap_ack(
        self, expected: np.ndarray, changed: np.ndarray,
        *, timeout_s: Optional[float] = None,
    ) -> dict[str, Any]:
        semantic_bound = getattr(self, "_semantic_costmap", None) is not None
        result = super()._wait_for_costmap_ack(
            expected, changed, timeout_s=timeout_s,
        )
        # Session.start() publishes a deliberately lethal non-semantic base
        # grid before a request is bound.  Its inherited occupancy ACK has no
        # semantic observation count and is outside this r15 invariant.
        if not semantic_bound:
            return result
        if result.get("costmap_update_acknowledged") is not True:
            raise RuntimeError("R15_SINGLE_OBSERVATION_EXACT_ACK_FAILED_CLOSED")
        if int(result.get("semantic_exact_stable_observations", 0)) != 1:
            self._costmap_state_trusted = False
            self._force_full_next_update = True
            raise RuntimeError("R15_EXACT_OBSERVATION_COUNT_MISMATCH")
        result.update({
            "r15_exact_observation_count": 1,
            "r15_ack_transport": "ONE_FRESH_FULL_MASTER_SERVICE_RESPONSE",
            "r15_timeout_repair_allowed": False,
        })
        return result


__all__ = ["SingleObservationExactAckSessionR15"]
