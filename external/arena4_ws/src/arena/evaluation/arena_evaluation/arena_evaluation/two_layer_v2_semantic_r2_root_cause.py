"""Read-only r1 reproduction profiler for the 2A-V2/r2 root-cause gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import cv2
import numpy as np

from . import two_layer_v2_semantic_r1_benchmark as r1
from .semantic_costmap_composer import LETHAL_OBSTACLE, occupancy_to_static_layer
from .semantic_smac_session import SemanticSmacSession
from .semantic_rasterizer import grid_hash


ARCHITECTURE_ID = "2A-V2"
IMPLEMENTATION_REVISION = "r2-direction-ack-latency"
PROTOCOL_ID = "PLN-02-2A-V2-R2-DIRECTION-ACK-LATENCY-V1"


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value)


def _pair_counts(source: np.ndarray, target: np.ndarray, mask: np.ndarray) -> list[Dict[str, int]]:
    source_values = np.asarray(source, dtype=np.uint8)[mask].astype(np.int32)
    target_values = np.asarray(target, dtype=np.uint8)[mask].astype(np.int32)
    if not len(source_values):
        return []
    encoded = source_values * 256 + target_values
    counts = np.bincount(encoded, minlength=65536)
    return [
        {"source": int(index // 256), "target": int(index % 256), "count": int(count)}
        for index, count in enumerate(counts) if count
    ]


class RootCauseSemanticSmacSession(SemanticSmacSession):
    """Capture post-ACK effective master content without changing r1 acceptance."""

    def _capture_effective_content(
        self, expected_occupancy: np.ndarray, changed: np.ndarray,
        ack: Mapping[str, Any], *, observation: str,
    ) -> None:
        semantic = self._semantic_costmap
        if semantic is None or self.client is None:
            return
        try:
            server, timestamp = self._server_costmap_snapshot(time.monotonic() + 1.0)
        except RuntimeError as exc:
            self._append_ack_trace("r2_root_cause_capture_errors.jsonl", {
                "observation": observation, "error": str(exc),
            })
            return
        expected_occupancy = np.asarray(expected_occupancy, dtype=np.int8)
        changed = np.asarray(changed, dtype=bool)
        expected_static = occupancy_to_static_layer(expected_occupancy)
        expected_master = np.flipud(np.asarray(semantic.expected_master_cost, dtype=np.uint8))
        hard_semantic = np.flipud(np.asarray(semantic.hard_semantic_mask, dtype=bool))
        soft_semantic = np.flipud(np.asarray(semantic.soft_cost > 0, dtype=bool))
        hard = changed | hard_semantic
        hard &= (hard_semantic | (expected_occupancy == 100))
        soft = soft_semantic & ~hard & (expected_occupancy > 0) & (expected_occupancy < 100)
        affected = changed | hard_semantic | soft_semantic
        exact_mismatch = affected & (server != expected_master)
        soft_exact_mismatch = soft & exact_mismatch
        hard_exact_mismatch = hard & (server != LETHAL_OBSTACLE)
        stale_changed = changed & (server != expected_master)
        static_dominant = soft & (expected_master == expected_static)
        inflation_dominant = soft & (expected_master > expected_static)

        cells = np.argwhere(affected)
        bbox = None if not len(cells) else [
            int(cells[:, 0].min()), int(cells[:, 0].max()) + 1,
            int(cells[:, 1].min()), int(cells[:, 1].max()) + 1,
        ]
        sequence = int(ack.get("semantic_roi_sequence") or self._semantic_roi_sequence)
        name = f"{sequence:04d}_{_safe_name(self.current_query_id)}_{observation}"
        output = self._semantic_output / "r2_root_cause"
        output.mkdir(parents=True, exist_ok=True)
        if bbox is not None:
            r0, r1_value, c0, c1 = bbox
            np.savez_compressed(
                output / f"{name}.npz",
                expected_occupancy=expected_occupancy[r0:r1_value, c0:c1],
                expected_static=expected_static[r0:r1_value, c0:c1],
                expected_master=expected_master[r0:r1_value, c0:c1],
                actual_master=server[r0:r1_value, c0:c1],
                soft_mask=soft[r0:r1_value, c0:c1],
                changed_mask=changed[r0:r1_value, c0:c1],
                exact_mismatch=exact_mismatch[r0:r1_value, c0:c1],
                bbox=np.asarray(bbox, dtype=np.int32),
            )
        heatmap = np.zeros((*server.shape, 3), dtype=np.uint8)
        heatmap[soft_exact_mismatch] = (0, 0, 255)
        heatmap[stale_changed] = (0, 255, 255)
        heatmap[hard_exact_mismatch] = (255, 0, 0)
        preview = cv2.resize(
            heatmap, (max(1, heatmap.shape[1] // 4), max(1, heatmap.shape[0] // 4)),
            interpolation=cv2.INTER_NEAREST,
        )
        cv2.imwrite(str(output / f"{name}_mismatch.png"), preview)
        record = {
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "protocol_id": PROTOCOL_ID,
            "query_id": self.current_query_id,
            "observation": observation,
            "semantic_roi_sequence": sequence,
            "server_timestamp_ns": int(timestamp),
            "policy_hash": semantic.policy_hash,
            "source_grid_hash": semantic.expected_grid_hash,
            "expected_master_hash": semantic.expected_master_hash,
            "actual_full_master_hash": grid_hash(server),
            "affected_content_hash": hashlib.sha256(
                np.ascontiguousarray(server[affected]).tobytes()
            ).hexdigest(),
            "bbox": bbox,
            "affected_cells": int(np.count_nonzero(affected)),
            "soft_cells": int(np.count_nonzero(soft)),
            "soft_exact_mismatch_cells": int(np.count_nonzero(soft_exact_mismatch)),
            "hard_exact_mismatch_cells": int(np.count_nonzero(hard_exact_mismatch)),
            "stale_changed_cells": int(np.count_nonzero(stale_changed)),
            "static_dominant_soft_cells": int(np.count_nonzero(static_dominant)),
            "static_dominant_mismatch_cells": int(np.count_nonzero(static_dominant & exact_mismatch)),
            "inflation_dominant_soft_cells": int(np.count_nonzero(inflation_dominant)),
            "inflation_dominant_mismatch_cells": int(np.count_nonzero(inflation_dominant & exact_mismatch)),
            "occupancy_to_static": _pair_counts(expected_occupancy, expected_static, soft),
            "static_to_expected_master": _pair_counts(expected_static, expected_master, soft),
            "expected_to_actual_master": _pair_counts(expected_master, server, soft),
            "actual_minus_expected_histogram": {
                str(int(value)): int(count)
                for value, count in zip(*np.unique(
                    server[soft].astype(np.int16) - expected_master[soft].astype(np.int16),
                    return_counts=True,
                ))
            },
            "r1_ack_status": ack.get("costmap_ack_status"),
            "r1_ack_semantics": ack.get("costmap_ack_semantics"),
            "r1_interval_accepted": ack.get("costmap_update_acknowledged"),
        }
        with (output / "effective_content_migrations.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, sort_keys=True) + "\n")

    def _wait_for_costmap_ack(
        self, expected: np.ndarray, changed: np.ndarray, *, timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        result = super()._wait_for_costmap_ack(expected, changed, timeout_s=timeout_s)
        self._capture_effective_content(expected, changed, result, observation="post_r1_interval_ack")
        return result

    def update_local_mask(self, allowed_mask: Any, **kwargs: Any) -> Dict[str, Any]:
        result = super().update_local_mask(allowed_mask, **kwargs)
        if result.get("local_map_update_mode") == "reuse_noop" and self._semantic_costmap is not None:
            _mask, expected = self._grid_for_mask(allowed_mask)
            self._capture_effective_content(
                expected, np.zeros(expected.shape, dtype=bool), result,
                observation="post_r1_noop_reuse",
            )
        return result


def run(args: argparse.Namespace) -> Path:
    output = args.output_dir.resolve()
    original = r1.SemanticSmacSession
    r1.SemanticSmacSession = RootCauseSemanticSmacSession
    try:
        r1.run_real_ablation(
            extracted_dir=args.extracted_dir.resolve(),
            semantic_map_path=args.semantic_map.resolve(),
            topology_cache=args.topology_cache.resolve(),
            output=output,
            config_path=args.config.resolve(),
            warmups=0,
            repetitions=1,
            ros_domain_id=args.ros_domain_id,
            arms=[value for value in args.arms.split(",") if value],
            query_ids=None,
        )
    finally:
        r1.SemanticSmacSession = original
    (output / "root_cause_profile_protocol.json").write_text(json.dumps({
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "profile_subject": "frozen_2A-V2_r1",
        "acceptance_semantics_unchanged": True,
        "arms": args.arms.split(","),
        "ros_domain_id": args.ros_domain_id,
    }, indent=2) + "\n", encoding="utf-8")
    return output


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=r1.DEFAULT_CONFIG)
    parser.add_argument("--extracted-dir", type=Path, required=True)
    parser.add_argument("--semantic-map", type=Path, required=True)
    parser.add_argument("--topology-cache", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--arms", default="E3,E4")
    parser.add_argument("--ros-domain-id", type=int, default=134)
    args = parser.parse_args(argv)
    print(f"2A-V2/r2 root-cause profile: {run(args)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
