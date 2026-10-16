"""Semantic OccupancyGrid adapter for the frozen 2A-V1-r2 Smac session."""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np
import yaml

from .semantic_costmap_composer import (
    LETHAL_OBSTACLE, SemanticCostmap, occupancy_to_static_layer,
)
from .unified_four_backends_smoke import SmacSession


class SemanticSmacSession(SmacSession):
    """Publish non-trinary semantic grids while retaining r2 lifecycle logic."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self._semantic_output = Path(args[1] if len(args) > 1 else kwargs["output"])
        super().__init__(*args, **kwargs)
        self._semantic_costmap: Optional[SemanticCostmap] = None
        self._semantic_ack_cache: Dict[Tuple[str, str, str], Dict[str, Any]] = {}
        self._semantic_roi_sequence = 0
        params = yaml.safe_load(self.params_file.read_text(encoding="utf-8")) or {}
        costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
        # The pinned Humble StaticLayer declares these three parameters on the
        # parent costmap node (see static_layer.cpp::getParameters), while some
        # newer layouts expose plugin-scoped keys.  Set both so the serialized
        # config is explicit and the active pinned implementation is truly
        # non-trinary.
        costmap.update({
            "trinary_costmap": False,
            "lethal_cost_threshold": 100,
            "unknown_cost_value": 255,
        })
        static = costmap.setdefault("static_layer", {})
        static.update({
            "trinary_costmap": False,
            "lethal_cost_threshold": 100,
            "unknown_cost_value": 255,
        })
        self.params_file.write_text(yaml.safe_dump(params, sort_keys=False), encoding="utf-8")
        self.smac_config_hash = hashlib.sha256(self.params_file.read_bytes()).hexdigest()

    def set_semantic_costmap(self, costmap: SemanticCostmap) -> None:
        expected_shape = (self.ctx.hospital_map.height, self.ctx.hospital_map.width)
        if costmap.occupancy_grid.shape != expected_shape:
            raise ValueError("semantic costmap shape mismatch")
        self._semantic_costmap = costmap

    @staticmethod
    def _semantic_ack_key(costmap: SemanticCostmap) -> Tuple[str, str, str]:
        return (
            str(costmap.policy_hash), str(costmap.expected_grid_hash),
            str(costmap.expected_master_hash),
        )

    def update_local_mask(self, allowed_mask: Any, **kwargs: Any) -> Dict[str, Any]:
        """Carry the last semantic ACK evidence across an exact no-op reuse."""
        result = super().update_local_mask(allowed_mask, **kwargs)
        semantic = self._semantic_costmap
        if semantic is None or result.get("local_map_update_mode") != "reuse_noop":
            return result
        prior = self._semantic_ack_cache.get(self._semantic_ack_key(semantic))
        if prior is None:
            return result
        for key in (
            "costmap_ack_hard_checked_cells", "costmap_ack_hard_mismatch_cells",
            "costmap_ack_soft_checked_cells", "costmap_ack_soft_mismatch_cells",
            "costmap_ack_soft_exact_mismatch_cells", "costmap_ack_soft_exact_mismatch_ratio",
            "costmap_ack_semantics", "semantic_roi_sequence", "semantic_publication_version",
            "semantic_policy_hash", "semantic_expected_grid_hash", "semantic_expected_master_hash",
            "server_costmap_update_time_ns", "server_costmap_content_hash",
        ):
            if key in prior:
                result[key] = prior[key]
        result.update({
            "costmap_update_acknowledged": True,
            "costmap_ack_status": "reused_server_verified_semantic_state",
            "costmap_ack_wait_ms": 0.0,
            "costmap_ack_attempts": 0,
            "semantic_ack_reused": True,
        })
        self._local_mask_info = dict(result)
        return result

    def _append_ack_trace(self, filename: str, payload: Mapping[str, Any]) -> None:
        trace_path = self._semantic_output / "logs" / filename
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        with trace_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({
                "query_id": self.current_query_id,
                "timestamp_ns": time.time_ns(),
                **dict(payload),
            }, sort_keys=True) + "\n")

    def reset_query_state(self, query_id: str, *, restore_base_map: bool = True) -> Dict[str, Any]:
        # A previous query's semantic grid is never valid reset state for the
        # next query.  If the parent detects an untrusted costmap and performs
        # a full base-map repair, it must use the frozen r2 base occupancy.
        self._semantic_costmap = None
        return super().reset_query_state(query_id, restore_base_map=restore_base_map)

    def _grid_for_mask(self, allowed_mask: Any) -> Tuple[np.ndarray, np.ndarray]:
        if self._semantic_costmap is None:
            return super()._grid_for_mask(allowed_mask)
        mask = np.asarray(allowed_mask, dtype=bool)
        values = np.asarray(self._semantic_costmap.occupancy_grid, dtype=np.int8).copy()
        values[~mask] = 100
        return mask, np.flipud(values)

    def _wait_for_costmap_ack(
        self, expected: np.ndarray, changed: np.ndarray, *, timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        if self._semantic_costmap is None:
            return super()._wait_for_costmap_ack(expected, changed, timeout_s=timeout_s)
        started_ns = time.monotonic_ns()
        deadline = time.monotonic() + float(timeout_s or self.costmap_ack_timeout_s)
        expected_occupancy = np.asarray(expected, dtype=np.int8)
        changed_mask = np.asarray(changed, dtype=bool)
        semantic = self._semantic_costmap
        expected_master = np.flipud(np.asarray(semantic.expected_master_cost, dtype=np.uint8))
        expected_static = occupancy_to_static_layer(expected_occupancy)
        hard_semantic = np.flipud(np.asarray(semantic.hard_semantic_mask, dtype=bool))
        soft_semantic = np.flipud(np.asarray(semantic.soft_cost > 0, dtype=bool))
        # Every changed cell is a correctness barrier.  Semantic cells are
        # rechecked even when byte-identical to the previous publication.
        affected = changed_mask | hard_semantic | soft_semantic
        hard = affected & (hard_semantic | (expected_occupancy == 100))
        soft = (
            affected & soft_semantic & ~hard
            & (expected_occupancy > 0) & (expected_occupancy < 100)
        )
        ordinary = affected & ~hard & ~soft
        attempts = 0
        last_hard = int(np.count_nonzero(hard))
        last_soft = int(np.count_nonzero(soft))
        last_soft_exact = last_soft
        last_ordinary = int(np.count_nonzero(ordinary))
        last_timestamp = -1
        last_hash = ""
        last_error = ""
        last_server: Optional[np.ndarray] = None
        if self.client is not None:
            self.client.executor.spin_once(timeout_sec=min(0.025, max(0.0, deadline - time.monotonic())))
        while time.monotonic() < deadline:
            attempts += 1
            try:
                server, timestamp = self._server_costmap_snapshot(deadline)
            except RuntimeError as exc:
                last_error = str(exc)
                continue
            last_timestamp = int(timestamp)
            last_server = server
            hard_mismatch = int(np.count_nonzero(server[hard] != LETHAL_OBSTACLE))
            soft_exact_mismatch = int(np.count_nonzero(server[soft] != expected_master[soft]))
            # StaticLayer must preserve the mapped soft value.  InflationLayer
            # may raise it up to the deterministic full-map master value; its
            # incremental update bounds can yield a lower decay shell at a
            # handful of boundary cells, but never below the static value.
            soft_mismatch = int(np.count_nonzero(
                (server[soft] < expected_static[soft])
                | (server[soft] > expected_master[soft])
                | (server[soft] >= LETHAL_OBSTACLE)
            ))
            # Non-semantic opened cells may be inflated but must not become
            # lethal.  Exact inflation is required for semantic soft cells.
            ordinary_mismatch = int(np.count_nonzero(
                np.where(expected_occupancy[ordinary] == 100,
                         server[ordinary] != LETHAL_OBSTACLE,
                         server[ordinary] == LETHAL_OBSTACLE)
            ))
            last_hard, last_soft, last_ordinary = hard_mismatch, soft_mismatch, ordinary_mismatch
            last_soft_exact = soft_exact_mismatch
            last_hash = hashlib.sha256(np.ascontiguousarray(server[affected]).tobytes()).hexdigest()
            if hard_mismatch == 0 and soft_mismatch == 0 and ordinary_mismatch == 0:
                self._last_server_update_time_ns = last_timestamp
                self._costmap_ack_sequence += 1
                self._semantic_roi_sequence += 1
                result = {
                    "costmap_update_acknowledged": True,
                    "costmap_ack_status": "semantic_hard_exact_soft_static_to_inflation_bounds_verified",
                    "costmap_ack_wait_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
                    "costmap_ack_attempts": attempts,
                    "costmap_ack_checked_cells": int(np.count_nonzero(affected)),
                    "costmap_ack_mismatch_cells": 0,
                    "costmap_ack_hard_checked_cells": int(np.count_nonzero(hard)),
                    "costmap_ack_hard_mismatch_cells": 0,
                    "costmap_ack_soft_checked_cells": int(np.count_nonzero(soft)),
                    "costmap_ack_soft_mismatch_cells": 0,
                    "costmap_ack_soft_exact_mismatch_cells": soft_exact_mismatch,
                    "costmap_ack_soft_exact_mismatch_ratio": (
                        float(soft_exact_mismatch / np.count_nonzero(soft))
                        if np.count_nonzero(soft) else 0.0
                    ),
                    "costmap_ack_semantics": "interval_not_exact",
                    "costmap_ack_sequence": self._costmap_ack_sequence,
                    "semantic_roi_sequence": self._semantic_roi_sequence,
                    "semantic_publication_version": "2A-V2-semantic-costmap-v1",
                    "semantic_policy_hash": semantic.policy_hash,
                    "semantic_expected_grid_hash": semantic.expected_grid_hash,
                    "semantic_expected_master_hash": semantic.expected_master_hash,
                    "server_costmap_update_time_ns": last_timestamp,
                    "server_costmap_content_hash": last_hash,
                }
                self._semantic_ack_cache[self._semantic_ack_key(semantic)] = dict(result)
                self._append_ack_trace("semantic_ack_successes.jsonl", result)
                return result
        result = {
            "costmap_update_acknowledged": False,
            "costmap_ack_status": "semantic_hard_soft_mismatch",
            "costmap_ack_wait_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
            "costmap_ack_attempts": attempts,
            "costmap_ack_checked_cells": int(np.count_nonzero(affected)),
            "costmap_ack_mismatch_cells": last_hard + last_soft + last_ordinary,
            "costmap_ack_hard_checked_cells": int(np.count_nonzero(hard)),
            "costmap_ack_hard_mismatch_cells": last_hard,
            "costmap_ack_soft_checked_cells": int(np.count_nonzero(soft)),
            "costmap_ack_soft_mismatch_cells": last_soft,
            "costmap_ack_soft_exact_mismatch_cells": last_soft_exact,
            "costmap_ack_soft_exact_mismatch_ratio": (
                float(last_soft_exact / np.count_nonzero(soft))
                if np.count_nonzero(soft) else 0.0
            ),
            "costmap_ack_semantics": "interval_not_exact",
            "semantic_roi_sequence": self._semantic_roi_sequence,
            "semantic_publication_version": "2A-V2-semantic-costmap-v1",
            "semantic_policy_hash": semantic.policy_hash,
            "semantic_expected_grid_hash": semantic.expected_grid_hash,
            "semantic_expected_master_hash": semantic.expected_master_hash,
            "server_costmap_update_time_ns": last_timestamp,
            "server_costmap_content_hash": last_hash,
            "costmap_ack_error": last_error,
        }
        if last_server is not None:
            mismatch = (
                (hard & (last_server != LETHAL_OBSTACLE))
                | (soft & (
                    (last_server < expected_static)
                    | (last_server > expected_master)
                    | (last_server >= LETHAL_OBSTACLE)
                ))
                | (ordinary & np.where(
                    expected_occupancy == 100,
                    last_server != LETHAL_OBSTACLE,
                    last_server == LETHAL_OBSTACLE,
                ))
            )
            samples = []
            for row, col in np.argwhere(mismatch)[:32]:
                samples.append({
                    "row": int(row), "col": int(col),
                    "kind": "hard" if hard[row, col] else ("soft" if soft[row, col] else "ordinary"),
                    "published_occupancy": int(expected_occupancy[row, col]),
                    "expected_static_lower_bound": int(expected_static[row, col]),
                    "expected_master": int(expected_master[row, col]),
                    "received_master": int(last_server[row, col]),
                })
            self._append_ack_trace(
                "semantic_ack_failures.jsonl",
                {**result, "mismatch_samples": samples},
            )
        return result


__all__ = ["SemanticSmacSession"]
