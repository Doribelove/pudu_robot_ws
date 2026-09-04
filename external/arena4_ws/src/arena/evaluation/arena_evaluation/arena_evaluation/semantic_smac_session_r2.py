"""Exact effective-content ACK for PLN-02 2A-V2 r2.

The r1 session intentionally accepted a StaticLayer-to-InflationLayer interval.
This r2 session instead gates planning on the exact master costmap consumed by
Smac.  It also binds no-op reuse to the publication and content that produced
the successful exact observation.
"""

from __future__ import annotations

import hashlib
import time
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np

from .semantic_costmap_composer import LETHAL_OBSTACLE, occupancy_to_static_layer
from .semantic_rasterizer import grid_hash
from .semantic_smac_session import SemanticSmacSession


class ExactSemanticSmacSessionR2(SemanticSmacSession):
    """Fail-closed server ACK with an exact, sequence-bound no-op cache."""

    PUBLICATION_VERSION = "2A-V2-semantic-costmap-r2-exact-v1"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self._semantic_publication_sequence = 0
        self._active_publication_bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)
        self._active_publication_baseline_timestamp_ns = -1
        self._last_exact_expected_master: Optional[np.ndarray] = None
        self._last_exact_signature: Optional[Tuple[str, str, str]] = None
        self._last_exact_ack: Optional[Dict[str, Any]] = None
        self._exact_ack_cache: Dict[Tuple[Any, ...], Dict[str, Any]] = {}
        self.exact_stable_observations = 2
        super().__init__(*args, **kwargs)
        # A RELIABLE writer is compatible with the pinned StaticLayer's
        # BEST_EFFORT request and removes avoidable local writer-side drops.
        # Content ACK remains the correctness barrier; QoS is not treated as
        # delivery proof.  Reuse the already constructed reliable profile so
        # this stays isolated from the r1 session implementation.
        self._map_update_qos = self._map_qos

    @staticmethod
    def _content_signature(semantic: Any) -> Tuple[str, str, str]:
        return (
            str(semantic.policy_hash),
            str(semantic.expected_grid_hash),
            str(semantic.expected_master_hash),
        )

    @staticmethod
    def _bbox_for_mask(mask: np.ndarray) -> Tuple[int, int, int, int]:
        cells = np.argwhere(np.asarray(mask, dtype=bool))
        if not cells.size:
            return (0, 0, 0, 0)
        y0, x0 = cells.min(axis=0)
        y1, x1 = cells.max(axis=0) + 1
        return (int(x0), int(y0), int(x1 - x0), int(y1 - y0))

    def _begin_publication(self, changed: np.ndarray, *, full: bool = False) -> None:
        if self._semantic_costmap is None:
            return
        self._semantic_publication_sequence += 1
        self._active_publication_baseline_timestamp_ns = int(
            getattr(self, "_last_server_update_time_ns", -1)
        )
        if full:
            self._active_publication_bbox = (
                0, 0, int(self.ctx.hospital_map.width), int(self.ctx.hospital_map.height),
            )
        else:
            self._active_publication_bbox = self._bbox_for_mask(changed)

    def start(self) -> None:
        super().start()
        # SmacSession's cold initialization is an all-lethal map and is
        # content-ACKed by the non-semantic parent path.  Retain that exact
        # effective state so the first semantic publication can compute the
        # old/new master dirty union.
        shape = (int(self.ctx.hospital_map.height), int(self.ctx.hospital_map.width))
        self._last_exact_expected_master = np.full(shape, LETHAL_OBSTACLE, dtype=np.uint8)

    def _publish_dirty_roi(
        self, expected: np.ndarray, changed: np.ndarray,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        self._begin_publication(np.asarray(changed, dtype=bool), full=False)
        return super()._publish_dirty_roi(expected, changed)

    def _publish_full_grid(self, values: np.ndarray, *, clear_costmap: bool = True) -> float:
        self._begin_publication(np.ones(np.asarray(values).shape, dtype=bool), full=True)
        return super()._publish_full_grid(values, clear_costmap=clear_costmap)

    def _current_source_grid_hash(self) -> str:
        semantic = self._semantic_costmap
        if semantic is None:
            return ""
        published = np.ascontiguousarray(
            np.flipud(np.asarray(semantic.occupancy_grid, dtype=np.int8)),
        )
        return hashlib.sha256(published.tobytes()).hexdigest()

    def _exact_cache_key(
        self, *, source_grid_hash: str, expected_master_hash: str,
        policy_hash: str, publication_sequence: int,
        roi_bbox: Tuple[int, int, int, int], server_content_hash: str,
    ) -> Tuple[Any, ...]:
        return (
            self.PUBLICATION_VERSION,
            int(publication_sequence),
            str(policy_hash),
            str(source_grid_hash),
            str(expected_master_hash),
            tuple(int(value) for value in roi_bbox),
            str(server_content_hash),
        )

    def update_local_mask(self, allowed_mask: Any, **kwargs: Any) -> Dict[str, Any]:
        """Only permit no-op reuse of the same exact-ACKed effective content."""
        semantic = self._semantic_costmap
        signature = self._content_signature(semantic) if semantic is not None else None
        if (
            semantic is not None
            and getattr(self, "enable_mask_reuse_noop", False)
            and signature != self._last_exact_signature
        ):
            # The parent would otherwise trust byte-identical source content
            # without proof that its effective master state was exact.
            self._force_full_next_update = True
        result = super().update_local_mask(allowed_mask, **kwargs)
        if semantic is None or result.get("local_map_update_mode") != "reuse_noop":
            return result
        if signature != self._last_exact_signature or self._last_exact_ack is None:
            self._costmap_state_trusted = False
            self._force_full_next_update = True
            raise RuntimeError("exact semantic ACK evidence missing for no-op reuse")
        evidence = dict(self._last_exact_ack)
        source_hash = self._current_source_grid_hash()
        if (
            source_hash != evidence.get("semantic_source_grid_hash")
            or str(semantic.expected_master_hash) != evidence.get("semantic_expected_master_hash")
            or str(semantic.policy_hash) != evidence.get("semantic_policy_hash")
        ):
            self._costmap_state_trusted = False
            self._force_full_next_update = True
            raise RuntimeError("exact semantic no-op key mismatch")
        cache_key = self._exact_cache_key(
            source_grid_hash=source_hash,
            expected_master_hash=str(semantic.expected_master_hash),
            policy_hash=str(semantic.policy_hash),
            publication_sequence=int(evidence.get("semantic_publication_sequence", -1)),
            roi_bbox=tuple(evidence.get("semantic_ack_roi_bbox") or ()),
            server_content_hash=str(evidence.get("server_costmap_content_hash", "")),
        )
        if (
            int(evidence.get("semantic_publication_sequence", -1))
            != int(self._semantic_publication_sequence)
            or tuple(evidence.get("semantic_ack_roi_bbox") or ())
            != tuple(self._active_publication_bbox)
            or cache_key not in self._exact_ack_cache
        ):
            self._costmap_state_trusted = False
            self._force_full_next_update = True
            raise RuntimeError("exact semantic no-op complete key mismatch")
        for key, value in evidence.items():
            if key.startswith("costmap_") or key.startswith("semantic_") or key.startswith("server_"):
                result[key] = value
        result.update({
            "costmap_update_acknowledged": True,
            "costmap_ack_status": "reused_exact_effective_content_ack",
            "costmap_ack_wait_ms": 0.0,
            "costmap_ack_attempts": 0,
            "semantic_ack_reused": True,
            "semantic_noop_complete_key_verified": True,
        })
        self._local_mask_info = dict(result)
        self._append_ack_trace("semantic_exact_ack_successes.jsonl", result)
        return result

    def _wait_for_costmap_ack(
        self, expected: np.ndarray, changed: np.ndarray, *, timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        semantic = self._semantic_costmap
        if semantic is None:
            return super()._wait_for_costmap_ack(expected, changed, timeout_s=timeout_s)

        started_ns = time.monotonic_ns()
        deadline = time.monotonic() + float(timeout_s or self.costmap_ack_timeout_s)
        expected_occupancy = np.asarray(expected, dtype=np.int8)
        expected_master = np.flipud(
            np.asarray(semantic.expected_master_cost, dtype=np.uint8),
        )
        expected_static = occupancy_to_static_layer(expected_occupancy)
        changed_source = np.asarray(changed, dtype=bool)
        hard_semantic = np.flipud(np.asarray(semantic.hard_semantic_mask, dtype=bool))
        soft_semantic = np.flipud(np.asarray(semantic.soft_cost > 0, dtype=bool))
        previous_master = self._last_exact_expected_master
        if previous_master is None or previous_master.shape != expected_master.shape:
            effective_dirty = np.ones(expected_master.shape, dtype=bool)
        else:
            effective_dirty = previous_master != expected_master
        affected = changed_source | effective_dirty | hard_semantic | soft_semantic
        hard = affected & (hard_semantic | (expected_occupancy == 100))
        soft = affected & soft_semantic & ~hard
        stale = effective_dirty
        ordinary = affected & ~hard & ~soft
        affected_cells = int(np.count_nonzero(affected))
        hard_cells = int(np.count_nonzero(hard))
        soft_cells = int(np.count_nonzero(soft))
        stale_cells = int(np.count_nonzero(stale))
        source_grid_hash = hashlib.sha256(
            np.ascontiguousarray(expected_occupancy).tobytes()
        ).hexdigest()
        expected_master_hash = str(semantic.expected_master_hash)
        expected_server_content_hash = grid_hash(expected_master)
        publication_sequence = int(self._semantic_publication_sequence)
        roi_bbox = tuple(int(value) for value in self._active_publication_bbox)

        attempts = 0
        stable_matches = 0
        last_hard = hard_cells
        last_soft = soft_cells
        last_stale = stale_cells
        last_ordinary = int(np.count_nonzero(ordinary))
        last_timestamp = -1
        last_affected_hash = ""
        last_full_hash = ""
        last_error = ""
        last_server: Optional[np.ndarray] = None
        required_stable = max(1, int(self.exact_stable_observations))

        if self.client is not None:
            self.client.executor.spin_once(
                timeout_sec=min(0.025, max(0.0, deadline - time.monotonic())),
            )
        while time.monotonic() < deadline:
            attempts += 1
            try:
                server, timestamp = self._server_costmap_snapshot(deadline)
            except RuntimeError as exc:
                last_error = str(exc)
                stable_matches = 0
                continue
            last_server = server
            last_timestamp = int(timestamp)
            timestamp_fresh = (
                self._active_publication_baseline_timestamp_ns < 0
                or last_timestamp > self._active_publication_baseline_timestamp_ns
            )
            exact = server == expected_master
            last_hard = int(np.count_nonzero(hard & ~exact))
            last_soft = int(np.count_nonzero(soft & ~exact))
            last_stale = int(np.count_nonzero(stale & ~exact))
            last_ordinary = int(np.count_nonzero(ordinary & ~exact))
            affected_values = np.ascontiguousarray(server[affected], dtype=np.uint8)
            last_affected_hash = hashlib.sha256(affected_values.tobytes()).hexdigest()
            last_full_hash = grid_hash(server)
            sequence_matches = (
                publication_sequence == int(self._semantic_publication_sequence)
            )
            if (
                timestamp_fresh
                and sequence_matches
                and last_hard == 0 and last_soft == 0
                and last_stale == 0 and last_ordinary == 0
            ):
                stable_matches += 1
                if stable_matches < required_stable:
                    if self.client is not None:
                        self.client.executor.spin_once(
                            timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())),
                        )
                    continue
                self._last_server_update_time_ns = last_timestamp
                self._costmap_ack_sequence += 1
                self._semantic_roi_sequence += 1
                result = {
                    "costmap_update_acknowledged": True,
                    "costmap_ack_status": "exact_effective_master_content_verified",
                    "costmap_ack_wait_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                    "costmap_ack_attempts": attempts,
                    "costmap_ack_checked_cells": affected_cells,
                    "costmap_ack_mismatch_cells": 0,
                    "costmap_ack_hard_checked_cells": hard_cells,
                    "costmap_ack_hard_mismatch_cells": 0,
                    "costmap_ack_soft_checked_cells": soft_cells,
                    "costmap_ack_soft_mismatch_cells": 0,
                    "costmap_ack_soft_exact_mismatch_cells": 0,
                    "costmap_ack_soft_exact_mismatch_ratio": 0.0,
                    "costmap_ack_stale_checked_cells": stale_cells,
                    "costmap_ack_stale_roi_cells": 0,
                    "costmap_ack_hash_mismatch": 0,
                    "costmap_ack_sequence_mismatch": 0,
                    "costmap_ack_semantics": "exact_effective_master",
                    "costmap_ack_sequence": int(self._costmap_ack_sequence),
                    "semantic_roi_sequence": int(self._semantic_roi_sequence),
                    "semantic_publication_sequence": publication_sequence,
                    "semantic_publication_version": self.PUBLICATION_VERSION,
                    "semantic_policy_hash": str(semantic.policy_hash),
                    "semantic_source_grid_hash": source_grid_hash,
                    "semantic_expected_grid_hash": str(semantic.expected_grid_hash),
                    "semantic_expected_master_hash": expected_master_hash,
                    "semantic_expected_server_content_hash": expected_server_content_hash,
                    "semantic_ack_roi_bbox": list(roi_bbox),
                    "semantic_effective_dirty_bbox": list(self._bbox_for_mask(effective_dirty)),
                    "semantic_exact_stable_observations": stable_matches,
                    "server_costmap_update_time_ns": last_timestamp,
                    "server_costmap_content_hash": last_full_hash,
                    "server_affected_content_hash": last_affected_hash,
                }
                cache_key = self._exact_cache_key(
                    source_grid_hash=source_grid_hash,
                    expected_master_hash=expected_master_hash,
                    policy_hash=str(semantic.policy_hash),
                    publication_sequence=publication_sequence,
                    roi_bbox=roi_bbox,
                    server_content_hash=last_full_hash,
                )
                result["semantic_exact_ack_key_hash"] = hashlib.sha256(
                    repr(cache_key).encode("utf-8")
                ).hexdigest()
                self._exact_ack_cache[cache_key] = dict(result)
                # Keep only the current state; this is a one-entry, bounded
                # evidence cache rather than an unbounded per-query history.
                if len(self._exact_ack_cache) > 1:
                    self._exact_ack_cache = {cache_key: dict(result)}
                self._last_exact_expected_master = expected_master.copy()
                self._last_exact_signature = self._content_signature(semantic)
                self._last_exact_ack = dict(result)
                self._semantic_ack_cache.clear()
                self._append_ack_trace("semantic_exact_ack_successes.jsonl", result)
                return result
            stable_matches = 0
            if self.client is not None:
                self.client.executor.spin_once(
                    timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())),
                )

        mismatch = (
            affected & (last_server != expected_master)
            if last_server is not None else affected
        )
        result = {
            "costmap_update_acknowledged": False,
            "costmap_ack_status": "exact_effective_master_mismatch",
            "costmap_ack_wait_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
            "costmap_ack_attempts": attempts,
            "costmap_ack_checked_cells": affected_cells,
            # hard/soft/stale are overlapping audit classes.  The aggregate
            # is the unique effective-content mismatch count.
            "costmap_ack_mismatch_cells": int(np.count_nonzero(mismatch)),
            "costmap_ack_hard_checked_cells": hard_cells,
            "costmap_ack_hard_mismatch_cells": last_hard,
            "costmap_ack_soft_checked_cells": soft_cells,
            "costmap_ack_soft_mismatch_cells": last_soft,
            "costmap_ack_soft_exact_mismatch_cells": last_soft,
            "costmap_ack_soft_exact_mismatch_ratio": float(last_soft / soft_cells) if soft_cells else 0.0,
            "costmap_ack_stale_checked_cells": stale_cells,
            "costmap_ack_stale_roi_cells": last_stale,
            "costmap_ack_hash_mismatch": int(
                last_full_hash != expected_server_content_hash
            ),
            "costmap_ack_sequence_mismatch": int(
                publication_sequence != self._semantic_publication_sequence
            ),
            "costmap_ack_semantics": "exact_effective_master",
            "semantic_publication_sequence": publication_sequence,
            "semantic_publication_version": self.PUBLICATION_VERSION,
            "semantic_policy_hash": str(semantic.policy_hash),
            "semantic_source_grid_hash": source_grid_hash,
            "semantic_expected_grid_hash": str(semantic.expected_grid_hash),
            "semantic_expected_master_hash": expected_master_hash,
            "semantic_expected_server_content_hash": expected_server_content_hash,
            "semantic_ack_roi_bbox": list(roi_bbox),
            "semantic_effective_dirty_bbox": list(self._bbox_for_mask(effective_dirty)),
            "server_costmap_update_time_ns": last_timestamp,
            "server_costmap_content_hash": last_full_hash,
            "server_affected_content_hash": last_affected_hash,
            "costmap_ack_error": last_error,
        }
        self._costmap_state_trusted = False
        self._force_full_next_update = True
        self._last_exact_signature = None
        self._last_exact_ack = None
        if last_server is not None:
            samples = []
            for row, col in np.argwhere(mismatch)[:32]:
                radius = 16
                y0, y1 = max(0, int(row) - radius), min(expected_occupancy.shape[0], int(row) + radius + 1)
                x0, x1 = max(0, int(col) - radius), min(expected_occupancy.shape[1], int(col) + radius + 1)
                sources = np.argwhere(expected_occupancy[y0:y1, x0:x1] == 100)
                nearest_source = None
                if sources.size:
                    source_rows = sources[:, 0] + y0
                    source_cols = sources[:, 1] + x0
                    distances = np.hypot(source_rows - int(row), source_cols - int(col))
                    source_index = int(np.argmin(distances))
                    source_row, source_col = int(source_rows[source_index]), int(source_cols[source_index])
                    nearest_source = {
                        "row": source_row, "col": source_col,
                        "distance_cells": float(distances[source_index]),
                        "received_master": int(last_server[source_row, source_col]),
                    }
                samples.append({
                    "row": int(row), "col": int(col),
                    "kind": (
                        "hard" if hard[row, col] else
                        "soft" if soft[row, col] else
                        "stale" if stale[row, col] else "ordinary"
                    ),
                    "published_occupancy": int(expected_occupancy[row, col]),
                    "expected_static": int(expected_static[row, col]),
                    "expected_master": int(expected_master[row, col]),
                    "received_master": int(last_server[row, col]),
                    "nearest_expected_lethal_source": nearest_source,
                })
            result["mismatch_samples"] = samples
        self._append_ack_trace("semantic_exact_ack_failures.jsonl", result)
        return result


__all__ = ["ExactSemanticSmacSessionR2"]
