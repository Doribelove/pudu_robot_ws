"""2D-V3 integration contract built on the frozen 2D-V2 runtime substrate.

ROI/ACK, adaptive 2/4 m corridors, 48 heading bins, and canonical PathAudit
remain owned by the shared V2 modules.  This file adds only V3 routing policy
composition and a bounded compressed mask cache for dynamic route churn.
"""

from __future__ import annotations

import hashlib
import json
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np

from . import layered_2d_v2_pipeline as v2
from .hybrid_l1_router import BudgetConfig, HybridL1Router


ARCHITECTURE_ID = "2D-V3"
IMPLEMENTATION_REVISION = "r0-hybrid-tail-bounded-v1"
PARENT_ARCHITECTURE = "2D-V2-r0"
PROTOCOL_VERSION = "PLN-02-EXP-V1"


def stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str,
    ).encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class PackedMaskROI:
    bbox: Tuple[int, int, int, int]
    shape: Tuple[int, int]
    packed: bytes
    allowed_cells: int
    mask_hash: str

    @classmethod
    def from_mask(cls, mask: np.ndarray) -> "PackedMaskROI":
        value = np.asarray(mask, dtype=bool)
        rows, columns = np.nonzero(value)
        if not len(rows):
            bbox = (0, 0, 0, 0)
            roi = np.zeros((0, 0), dtype=bool)
        else:
            bbox = (int(rows.min()), int(rows.max()) + 1,
                    int(columns.min()), int(columns.max()) + 1)
            roi = value[bbox[0]:bbox[1], bbox[2]:bbox[3]]
        packed = np.packbits(roi.reshape(-1)).tobytes()
        digest = hashlib.sha256()
        digest.update(json.dumps([*bbox, *roi.shape], separators=(",", ":")).encode("ascii"))
        digest.update(packed)
        return cls(bbox, tuple(int(item) for item in roi.shape), packed,
                   int(np.count_nonzero(roi)), digest.hexdigest())

    @property
    def memory_bytes(self) -> int:
        return int(len(self.packed) + 8 * 6 + len(self.mask_hash))

    def unpack(self) -> np.ndarray:
        if not self.shape[0] or not self.shape[1]:
            return np.zeros(self.shape, dtype=bool)
        count = self.shape[0] * self.shape[1]
        return np.unpackbits(np.frombuffer(self.packed, dtype=np.uint8), count=count).reshape(self.shape).astype(bool)


class DynamicRouteMaskLRU:
    """Memory-capped cache of edge/turn primitives and composed routes."""

    def __init__(self, *, map_shape: Sequence[int], binding: Mapping[str, Any],
                 memory_cap_bytes: int = 128 * 1024 * 1024) -> None:
        self.map_shape = (int(map_shape[0]), int(map_shape[1]))
        self.binding = dict(binding)
        self.binding_hash = stable_hash(self.binding)
        self.memory_cap_bytes = max(1, int(memory_cap_bytes))
        self.primitives: Dict[str, PackedMaskROI] = {}
        self.route_cache: "OrderedDict[str, PackedMaskROI]" = OrderedDict()
        self.primitive_bytes = 0
        self.route_bytes = 0
        self.route_hits = 0
        self.route_misses = 0
        self.evictions = 0
        self.peak_bytes = 0

    @property
    def memory_bytes(self) -> int:
        return int(self.primitive_bytes + self.route_bytes)

    def validate_binding(self, binding: Mapping[str, Any]) -> None:
        if stable_hash(dict(binding)) != self.binding_hash:
            raise ValueError("dynamic mask cache binding mismatch")

    def put_primitive(self, primitive_id: str, mask: np.ndarray) -> PackedMaskROI:
        value = np.asarray(mask, dtype=bool)
        if value.shape != self.map_shape:
            raise ValueError("primitive mask shape does not match map binding")
        key = str(primitive_id)
        if key in self.primitives:
            return self.primitives[key]
        packed = PackedMaskROI.from_mask(value)
        self.primitives[key] = packed
        self.primitive_bytes += packed.memory_bytes
        self.peak_bytes = max(self.peak_bytes, self.memory_bytes)
        return packed

    def _route_key(
        self, route_edge_ids: Sequence[str], turn_support_ids: Sequence[str],
        endpoint_id: str, snapshot_id: str, blocked_digest: str,
    ) -> str:
        return stable_hash({
            "binding_hash": self.binding_hash,
            "route_edge_ids": list(route_edge_ids),
            "turn_support_ids": list(turn_support_ids),
            "endpoint_id": str(endpoint_id),
            "snapshot_id": str(snapshot_id),
            "blocked_edge_digest": str(blocked_digest),
        })

    def _place(self, destination: np.ndarray, primitive: PackedMaskROI) -> None:
        r0, r1, c0, c1 = primitive.bbox
        if r1 <= r0 or c1 <= c0:
            return
        destination[r0:r1, c0:c1] |= primitive.unpack()

    def compose(
        self, *, route_edge_ids: Sequence[str], turn_support_ids: Sequence[str] = (),
        endpoint_id: str = "", snapshot_id: str = "", blocked_digest: str = "",
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        key = self._route_key(
            route_edge_ids, turn_support_ids, endpoint_id, snapshot_id, blocked_digest,
        )
        cached = self.route_cache.get(key)
        if cached is not None:
            self.route_cache.move_to_end(key)
            self.route_hits += 1
            result = np.zeros(self.map_shape, dtype=bool)
            self._place(result, cached)
            return result, {"cache_hit": True, "route_cache_key": key,
                            **self.diagnostics()}
        self.route_misses += 1
        result = np.zeros(self.map_shape, dtype=bool)
        ids = [*(f"edge:{edge}" for edge in route_edge_ids),
               *(f"turn:{turn}" for turn in turn_support_ids)]
        if endpoint_id:
            ids.append(f"endpoint:{endpoint_id}")
        missing = [primitive_id for primitive_id in ids if primitive_id not in self.primitives]
        if missing:
            raise KeyError(f"missing dynamic corridor primitives: {missing}")
        for primitive_id in ids:
            self._place(result, self.primitives[primitive_id])
        packed = PackedMaskROI.from_mask(result)
        # Route entries alone are evictable; primitives are the bounded reusable
        # substrate and are never silently dropped mid-map binding.
        while self.route_cache and self.memory_bytes + packed.memory_bytes > self.memory_cap_bytes:
            _old_key, old = self.route_cache.popitem(last=False)
            self.route_bytes -= old.memory_bytes
            self.evictions += 1
        if self.memory_bytes + packed.memory_bytes <= self.memory_cap_bytes:
            self.route_cache[key] = packed
            self.route_bytes += packed.memory_bytes
        self.peak_bytes = max(self.peak_bytes, self.memory_bytes)
        return result, {"cache_hit": False, "route_cache_key": key,
                        **self.diagnostics()}

    def invalidate(self, new_binding: Mapping[str, Any]) -> None:
        self.binding = dict(new_binding)
        self.binding_hash = stable_hash(self.binding)
        self.primitives.clear()
        self.route_cache.clear()
        self.primitive_bytes = 0
        self.route_bytes = 0

    def diagnostics(self) -> Dict[str, Any]:
        return {
            "binding_hash": self.binding_hash,
            "primitive_count": len(self.primitives),
            "route_entry_count": len(self.route_cache),
            "primitive_bytes": self.primitive_bytes,
            "route_bytes": self.route_bytes,
            "memory_bytes": self.memory_bytes,
            "memory_cap_bytes": self.memory_cap_bytes,
            "peak_bytes": self.peak_bytes,
            "route_hits": self.route_hits,
            "route_misses": self.route_misses,
            "evictions": self.evictions,
        }


class Layered2DV3Pipeline:
    """Composition root; V2 owns downstream runtime, V3 owns L1 decisions."""

    def __init__(self, *, v2_runtime: Any, l1_router: HybridL1Router,
                 mask_cache: Optional[DynamicRouteMaskLRU] = None) -> None:
        self.v2_runtime = v2_runtime
        self.l1_router = l1_router
        self.mask_cache = mask_cache

    @staticmethod
    def corridor_transition(old_mask: np.ndarray, new_mask: np.ndarray) -> Dict[str, Any]:
        return v2.corridor_dirty_transition(old_mask, new_mask)

    @property
    def runtime_contract(self) -> Mapping[str, Any]:
        return {
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "parent_architecture": PARENT_ARCHITECTURE,
            "l2_enabled": False,
            "smac_planner": "SmacPlannerHybrid",
            "motion_model": "DUBIN",
            "angle_quantization_bins": v2.ANGLE_QUANTIZATION_BINS,
            "corridor_profile": v2.CORRIDOR_PROFILE,
            "roi_max_message_bytes": v2.ROI_MAX_MESSAGE_BYTES,
            "server_content_ack_required_before_smac": True,
            "fixed_settle_cycles": 0,
            "normal_costmap_clear_count": 0,
            "canonical_path_audit_reused": True,
        }


PathAuditor = v2.PathAuditor
SmacSession = v2.SmacSession


__all__ = [
    "ARCHITECTURE_ID", "IMPLEMENTATION_REVISION", "PARENT_ARCHITECTURE",
    "PROTOCOL_VERSION", "PackedMaskROI", "DynamicRouteMaskLRU",
    "Layered2DV3Pipeline", "BudgetConfig", "HybridL1Router", "PathAuditor",
    "SmacSession",
]
