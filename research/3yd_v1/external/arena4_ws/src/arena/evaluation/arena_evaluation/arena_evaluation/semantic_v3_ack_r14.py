"""Deterministic reinflation/reset publication protocol for 2A-V3 r14."""
from __future__ import annotations

import time
from typing import Any, Optional

import numpy as np

from .two_layer_v3_semantic_r13_online import RoutePhaseV3PlannerSession


class DeterministicReinflationSessionR14(RoutePhaseV3PlannerSession):
    """Publish the exact ROI, reset, then replay the complete current source.

    The r13 seam repair was safe but an occasional incremental InflationLayer
    boundary did not converge before the two-second ACK deadline, forcing a
    complete-map resend.  r14 sends the same bounded, overlapped source tiles
    and then invokes the public global-costmap clear service exactly once.  The
    Clearing every layer invalidates master cells outside the dirty ROI.  The
    pinned Humble StaticLayer does not expose a synchronous "full reinflation
    completed" acknowledgement, so replaying only the ROI after a clear races
    its next update cycle.  r14 therefore publishes the bounded change first,
    invokes the public clear service exactly once, and then publishes the
    complete *current* source grid once before exact full-master readback.  The
    full replay is a predetermined reset transaction, never a timeout repair.
    There is no fixed settle and no successful planning after a mismatch.
    """

    PUBLICATION_VERSION = "2A-V3-r14-roi-reset-full-source-replay-v2"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._r14_roi_publication_pending = False
        self._r14_semantic_state_activated = False

    def _begin_publication(self, changed: np.ndarray, *, full: bool = False) -> None:
        super()._begin_publication(changed, full=full)

    def _server_costmap_snapshot(self, deadline: float) -> tuple[np.ndarray, int]:
        """Read one master snapshot and cancel a late client future.

        The inherited implementation leaves a timed-out 8.6 MB GetCostmap
        future outstanding.  In a long run those responses queue behind later
        ACK requests.  r14 keeps at most one client future and never treats a
        timeout as successful evidence.
        """
        if self._get_costmap_client is None or self.client is None:
            raise RuntimeError("global costmap readback service is unavailable")
        remaining = max(0.0, deadline-time.monotonic())
        if not self._get_costmap_client.wait_for_service(timeout_sec=min(.2, remaining)):
            raise RuntimeError("global costmap readback service unavailable")
        future = self._get_costmap_client.call_async(self.GetCostmap.Request())
        while not future.done() and time.monotonic() < deadline:
            self.client.executor.spin_once(
                timeout_sec=min(.01, max(0.0, deadline-time.monotonic())),
            )
        if not future.done():
            future.cancel()
            raise RuntimeError("global costmap readback timed out")
        response = future.result()
        message = getattr(response, "map", None)
        metadata = getattr(message, "metadata", None)
        width = int(getattr(metadata, "size_x", 0))
        height = int(getattr(metadata, "size_y", 0))
        if width != int(self.ctx.hospital_map.width) or height != int(self.ctx.hospital_map.height):
            raise RuntimeError(f"global costmap readback shape mismatch: {width}x{height}")
        data = np.frombuffer(bytes(getattr(message, "data", b"")), dtype=np.uint8)
        if data.size != width*height:
            raise RuntimeError("global costmap readback data length mismatch")
        update_time = getattr(metadata, "update_time", None)
        timestamp_ns = (
            int(getattr(update_time, "sec", 0))*1_000_000_000
            + int(getattr(update_time, "nanosec", 0))
        )
        return data.reshape((height, width)), int(timestamp_ns)

    def _publish_dirty_roi(
        self, expected: np.ndarray, changed: np.ndarray,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        applied, telemetry = super()._publish_dirty_roi(expected, changed)
        if int(telemetry.get("roi_changed_cells", 0)) == 0:
            telemetry.update({
                "deterministic_reinflation_reset_count": 0,
                "deterministic_reinflation_reset_ms": 0.0,
                "deterministic_reinflation_ordering_barrier": "UNCHANGED_EXACT_REUSE",
            })
            return applied, telemetry
        # update_local_mask calls _wait_for_costmap_ack immediately after this
        # method.  Mark that boundary so the clear service can be executed
        # before, rather than after, the exact server-content observation.
        self._r14_roi_publication_pending = True
        return applied, telemetry

    def _publish_full_grid(self, values: np.ndarray, *, clear_costmap: bool = True) -> float:
        # The inherited full-grid path already clears before publishing.  It
        # must not be mistaken for an ROI publication awaiting reinflation.
        self._r14_roi_publication_pending = False
        return super()._publish_full_grid(values, clear_costmap=clear_costmap)

    def _wait_for_costmap_ack(
        self, expected: np.ndarray, changed: np.ndarray,
        *, timeout_s: Optional[float] = None,
    ) -> dict[str, Any]:
        reset_count = 0
        reset_ms = 0.0
        barrier_ms = 0.0
        replay: dict[str, Any] = {}
        if self._r14_roi_publication_pending:
            started = time.monotonic_ns()
            reset_ms = float(self._clear_global_costmap())
            barrier_ms = (time.monotonic_ns()-started)/1.0e6
            reset_count = 1
            self._r14_roi_publication_pending = False
            # ClearEntireCostmap invalidates unchanged master cells as well as
            # the dirty ROI.  Replaying only the dirty tiles made restoration
            # depend on a later asynchronous StaticLayer update.  Publish the
            # complete current source once, by protocol, so exact readback has
            # a deterministic predecessor state.  Bypass this class's wrapper
            # to avoid arming another reset.
            full_publication_ms = RoutePhaseV3PlannerSession._publish_full_grid(
                self, expected, clear_costmap=False,
            )
            replay = {
                "roi_message_count": 1,
                "roi_published_cells": int(np.asarray(expected).size),
                "local_map_serialization_ms": 0.0,
                "local_map_publication_ms": float(full_publication_ms),
                "replay_scope": "FULL_CURRENT_SOURCE_GRID",
            }
        result = super()._wait_for_costmap_ack(
            expected, changed, timeout_s=timeout_s,
        )
        result.update({
            "deterministic_reinflation_reset_count": reset_count,
            "deterministic_reinflation_reset_ms": reset_ms,
            "deterministic_reinflation_total_barrier_ms": barrier_ms,
            "deterministic_reinflation_ordering_barrier": (
                "ROI_TILES_PUBLISHED_THEN_CLEAR_SERVICE_COMPLETED"
                if reset_count else "FULL_PUBLICATION_OWNS_RESET_OR_UNCHANGED"
            ),
            "deterministic_reinflation_replay_count": int(bool(replay)),
            "deterministic_reinflation_replay_messages": int(
                replay.get("roi_message_count", 0)
            ),
            "deterministic_reinflation_replay_cells": int(
                replay.get("roi_published_cells", 0)
            ),
            "deterministic_reinflation_replay_scope": replay.get(
                "replay_scope", "NONE"
            ),
            "deterministic_reinflation_replay_serialization_ms": float(
                replay.get("local_map_serialization_ms", 0.0)
            ),
            "deterministic_reinflation_replay_publication_ms": float(
                replay.get("local_map_publication_ms", 0.0)
            ),
        })
        if not bool(result.get("costmap_update_acknowledged")):
            # Raising here prevents the inherited update_local_mask from
            # performing its timeout-driven full-grid repair branch.
            self._costmap_state_trusted = False
            self._force_full_next_update = True
            raise RuntimeError("R14_DETERMINISTIC_REINFLATION_ACK_FAILED_CLOSED")
        result.update({
            "verified_master_snapshot_source": "EXPECTED_MASTER_PROVEN_EQUAL_BY_EXACT_ACK",
            "post_ack_get_costmap_requests": 0,
        })
        return result

    def verified_master_snapshot(
        self, ack: Any, *, timeout_s: float = 1.0,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Return expected master bytes only after observed exact equality.

        The parent ACK compares all hard, soft, stale and ordinary affected
        cells, requires two stable observations, and binds the complete server
        hash and publication sequence.  The expected master is therefore the
        exact effective content consumed at that barrier; retaining or reading
        the 8.6 MB service response again cannot strengthen that proof.
        """
        del timeout_s
        semantic = self._semantic_costmap
        required = {
            "costmap_update_acknowledged": True,
            "costmap_ack_semantics": "exact_effective_master",
            "costmap_ack_hard_mismatch_cells": 0,
            "costmap_ack_soft_exact_mismatch_cells": 0,
            "costmap_ack_stale_roi_cells": 0,
            "costmap_ack_hash_mismatch": 0,
            "costmap_ack_sequence_mismatch": 0,
        }
        if semantic is None:
            raise RuntimeError("semantic costmap is not bound")
        for key, expected in required.items():
            if ack.get(key) != expected:
                raise RuntimeError(f"V3 effective-content ACK failed for {key}")
        publication_sequence = int(ack.get("semantic_publication_sequence", -1))
        if publication_sequence != int(self._semantic_publication_sequence):
            raise RuntimeError("R14 exact-ACK publication sequence mismatch")
        from .semantic_rasterizer import grid_hash
        expected_server = np.ascontiguousarray(
            np.flipud(np.asarray(semantic.expected_master_cost, dtype=np.uint8)),
        )
        server_hash = str(ack.get("server_costmap_content_hash", ""))
        expected_hash = grid_hash(expected_server)
        if (
            server_hash != expected_hash
            or expected_hash != str(ack.get("semantic_expected_server_content_hash", ""))
        ):
            self._costmap_state_trusted = False
            self._force_full_next_update = True
            raise RuntimeError("R14 exact-ACK full hash binding mismatch")
        top_order = np.ascontiguousarray(semantic.expected_master_cost, dtype=np.uint8)
        if grid_hash(top_order) != str(semantic.expected_master_hash):
            raise RuntimeError("R14 cached top-row master binding hash mismatch")
        return top_order, {
            "verified_master_hash": grid_hash(top_order),
            "verified_server_content_hash": server_hash,
            "verified_master_mismatch_cells": 0,
            "verified_server_update_time_ns": int(
                ack.get("server_costmap_update_time_ns", -1)
            ),
            "verified_publication_sequence": publication_sequence,
            "verified_master_snapshot_source": "EXPECTED_MASTER_PROVEN_EQUAL_BY_EXACT_ACK",
            "post_ack_get_costmap_requests": 0,
        }

    def update_local_mask(self, allowed_mask: Any, **kwargs: Any) -> dict[str, Any]:
        semantic_activation = bool(
            self._semantic_costmap is not None
            and not self._r14_semantic_state_activated
            and not kwargs.get("initialization", False)
        )
        if semantic_activation:
            # The inherited startup state is intentionally all lethal.  Its
            # first transition has no previous real request whose inflation
            # bounds can be reused, so activate the first request with one
            # proactive full publication.  This is not a timeout repair; all
            # subsequent transitions use the bounded deterministic ROI flow.
            kwargs["force_full"] = True
        result = super().update_local_mask(allowed_mask, **kwargs)
        # Initial full publication is expected at session activation.  Once a
        # trusted state exists, any r13 timeout-driven full repair is excluded
        # from r14 evidence and fails the request before a planner is called.
        if (
            result.get("local_map_update_mode") == "roi_ack_full_fallback"
            or result.get("roi_ack_initial_status")
        ):
            self._costmap_state_trusted = False
            self._force_full_next_update = True
            raise RuntimeError("R14_DETERMINISTIC_REINFLATION_ACK_FAILED_CLOSED")
        if semantic_activation:
            self._r14_semantic_state_activated = True
            result.update({
                "r14_initial_semantic_full_activation": True,
                "r14_timeout_full_repair": False,
                "local_map_update_mode": "deterministic_initial_full_activation",
                "local_map_update_fallback": False,
                "local_map_update_fallback_reason": "",
            })
        return result


__all__ = ["DeterministicReinflationSessionR14"]
