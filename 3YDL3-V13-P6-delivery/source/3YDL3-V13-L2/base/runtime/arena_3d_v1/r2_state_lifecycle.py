"""Production-acceptance L2 lifecycle for 3D-V1/r2.

r2 keeps the r1 compact D* algorithm unchanged.  It bit-packs only the
immutable static ROI mask and admits D* online only from a fully verified
prebuilt geometry + mutable-state cache.  A miss or rejected cache dispatches
the current request directly to the frozen deterministic grid A* path; it
never performs a hidden synchronous D* build.
"""

from __future__ import annotations

import gc
import math
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np

from arena_evaluation.dstar_lite import DStarSearchStats, INF

from .l2_incremental import Cell, CorridorROI, GridAStarResult, L2PlanResult, deterministic_grid_astar
from .l2_state_lifecycle import (
    DEFAULT_DYNAMIC_BASELINE,
    CacheTelemetry,
    CompactCorridorGeometry,
    CompactDStarState,
    CompactGeometryBinding,
    CompactPersistentCorridorDStar,
    L2StateLifecycleManager,
    MutableStateBinding,
    _elapsed_ms,
    _rss_bytes,
)


ARCHITECTURE_ID = "3D-V1"
REVISION_ID = "r2-production-acceptance-real-replay"
PROTOCOL_ID = "PLN-02-3D-V1-R2-PRODUCTION-ACCEPTANCE-V1"
ALGORITHM_VERSION = "compact-dstar-lite-corner-safe-f64-v2+r2-packed-static-mask"
STATIC_MASK_SCHEMA = "3D-V1-r2-packed-static-roi-little-v1"
FORMAT_VERSION = 3


class PackedStaticMask:
    """Logical 2-D bool mask backed by a little-endian bit-packed array."""

    def __init__(self, mask: np.ndarray) -> None:
        started = time.monotonic_ns()
        source = np.ascontiguousarray(mask, dtype=np.bool_)
        if source.ndim != 2:
            raise ValueError("static ROI mask must be 2-D")
        self.shape = (int(source.shape[0]), int(source.shape[1]))
        self.size = int(source.size)
        self.packed = np.ascontiguousarray(
            np.packbits(source.reshape(-1), bitorder="little"), dtype=np.uint8,
        )
        self.true_count = int(np.count_nonzero(source))
        self.pack_ms = _elapsed_ms(started)
        self.materialize_count = 0
        self.materialize_total_ms = 0.0
        self.last_materialize_ms = 0.0
        self.peak_materialized_bytes = 0

    @property
    def nbytes(self) -> int:
        return int(self.packed.nbytes)

    @property
    def logical_nbytes(self) -> int:
        return self.size

    def copy(self, order: str = "C") -> np.ndarray:
        del order
        started = time.monotonic_ns()
        result = np.unpackbits(
            self.packed, count=self.size, bitorder="little",
        ).astype(np.bool_, copy=False).reshape(self.shape)
        elapsed = _elapsed_ms(started)
        self.materialize_count += 1
        self.materialize_total_ms += elapsed
        self.last_materialize_ms = elapsed
        self.peak_materialized_bytes = max(self.peak_materialized_bytes, int(result.nbytes))
        return result

    def reshape(self, *shape: int) -> np.ndarray:
        return self.copy().reshape(*shape)

    def __array__(self, dtype: Optional[np.dtype] = None) -> np.ndarray:
        result = self.copy()
        return result if dtype is None else result.astype(dtype, copy=False)

    def __getitem__(self, key: Any) -> Any:
        if isinstance(key, tuple) and len(key) == 2 and all(
            isinstance(value, (int, np.integer)) for value in key
        ):
            row, column = int(key[0]), int(key[1])
            if row < 0:
                row += self.shape[0]
            if column < 0:
                column += self.shape[1]
            if not (0 <= row < self.shape[0] and 0 <= column < self.shape[1]):
                raise IndexError("packed static mask index out of range")
            linear = row * self.shape[1] + column
            return bool((int(self.packed[linear >> 3]) >> (linear & 7)) & 1)
        return self.copy()[key]

    def telemetry(self) -> Dict[str, Any]:
        return {
            "schema": STATIC_MASK_SCHEMA,
            "logical_shape": list(self.shape),
            "logical_bytes": self.logical_nbytes,
            "storage_bytes": self.nbytes,
            "true_count": self.true_count,
            "pack_ms": self.pack_ms,
            "materialize_count": self.materialize_count,
            "materialize_total_ms": self.materialize_total_ms,
            "last_materialize_ms": self.last_materialize_ms,
            "peak_materialized_bytes": self.peak_materialized_bytes,
        }


def pack_corridor_roi(roi: CorridorROI) -> CorridorROI:
    if isinstance(roi.base_free, PackedStaticMask):
        return roi
    packed = PackedStaticMask(np.asarray(roi.base_free, dtype=np.bool_))
    return CorridorROI(
        bbox=roi.bbox,
        base_free=packed,  # type: ignore[arg-type]
        start_local=roi.start_local,
        goal_local=roi.goal_local,
        binding=roi.binding,
        global_corridor_cells=roi.global_corridor_cells,
    )


def _r2_state_binding(geometry: CompactCorridorGeometry, roi: CorridorROI, baseline: str) -> MutableStateBinding:
    return MutableStateBinding(
        geometry_hash=geometry.binding.digest,
        start_cell=roi.binding.start_cell,
        goal_cell=roi.binding.goal_cell,
        dynamic_baseline_version=str(baseline),
        algorithm_version=ALGORITHM_VERSION,
        format_version=FORMAT_VERSION,
    )


class R2CompactPersistentCorridorDStar(CompactPersistentCorridorDStar):
    """r1's exact D* state with an r2 packed immutable ROI mask."""

    @property
    def static_mask(self) -> PackedStaticMask:
        mask = self.roi.base_free
        if not isinstance(mask, PackedStaticMask):
            raise AssertionError("r2 planner lost its packed static mask")
        return mask

    def _result(self, **kwargs: Any) -> L2PlanResult:  # type: ignore[override]
        result = super()._result(**kwargs)
        diagnostics = dict(result.diagnostics)
        diagnostics.update({
            "static_mask": self.static_mask.telemetry(),
            "static_mask_storage_bytes": self.static_mask.nbytes,
            "static_mask_logical_bytes": self.static_mask.logical_nbytes,
            "synchronous_dstar_build_performed": False,
            "r2_algorithm_version": ALGORITHM_VERSION,
        })
        return L2PlanResult(
            success=result.success,
            path=result.path,
            failure_code=result.failure_code,
            selected_backend=result.selected_backend,
            response_ms=result.response_ms,
            dstar_stats=result.dstar_stats,
            fallback_stats=result.fallback_stats,
            changed_cells=result.changed_cells,
            state_reused=result.state_reused,
            partial_dstar_result_returned=result.partial_dstar_result_returned,
            oracle_cost_error=result.oracle_cost_error,
            diagnostics=diagnostics,
        )


class CacheMissAStarPlanner:
    """Packed-mask deterministic A* adapter used when D* admission fails."""

    def __init__(self, roi: CorridorROI, *, reject_reason: str) -> None:
        self.roi = pack_corridor_roi(roi)
        self.reject_reason = str(reject_reason)
        self.blocked_global: Set[Cell] = set()
        self.current_path_local: Optional[List[Cell]] = None
        self.fallback_count = 0
        self.resync_count = 0
        self.reinitialize_count = 0
        self.cache_telemetry: Dict[str, Any] = {}

    @property
    def static_mask(self) -> PackedStaticMask:
        mask = self.roi.base_free
        if not isinstance(mask, PackedStaticMask):
            raise AssertionError("cache-miss planner lost its packed mask")
        return mask

    @property
    def planner(self) -> "CacheMissAStarPlanner":
        return self

    @property
    def binding_hash(self) -> str:
        return self.roi.binding.digest

    @property
    def dstar_ready(self) -> bool:
        return False

    @property
    def path_global(self) -> Optional[List[Cell]]:
        if self.current_path_local is None:
            return None
        return [self.roi.to_global(cell) for cell in self.current_path_local]

    @property
    def dynamic_blocked_local(self) -> Set[Cell]:
        return {
            self.roi.to_local(cell) for cell in self.blocked_global
            if self.roi.contains_global(cell) and self.static_mask[self.roi.to_local(cell)]
        }

    @property
    def current_free(self) -> np.ndarray:
        result = self.static_mask.copy()
        for cell in self.dynamic_blocked_local:
            result[cell] = False
        return result

    def _translate_blocked(self, blocked_global: Iterable[Cell]) -> Set[Cell]:
        return {
            (int(cell[0]), int(cell[1])) for cell in blocked_global
            if self.roi.contains_global((int(cell[0]), int(cell[1])))
            and self.static_mask[self.roi.to_local((int(cell[0]), int(cell[1])))]
        }

    def _search_result(self, started_ns: int, *, changed: int, backend: str, verify_oracle: bool) -> L2PlanResult:
        fallback = deterministic_grid_astar(
            self.current_free, self.roi.start_local, self.roi.goal_local,
        )
        self.current_path_local = None if fallback.path is None else list(fallback.path)
        self.fallback_count += 1
        diagnostics = {
            "binding_hash": self.binding_hash,
            "roi_bbox": list(self.roi.bbox),
            "roi_shape": list(self.roi.shape),
            "roi_array_cells": self.static_mask.size,
            "corridor_cells": self.static_mask.true_count,
            "global_corridor_cells": self.roi.global_corridor_cells,
            "dynamic_blocked_cells": len(self.dynamic_blocked_local),
            "dstar_ready": False,
            "reinitialize_count": 0,
            "fallback_count": self.fallback_count,
            "resync_count": self.resync_count,
            "state_memory_bytes": self.state_memory_bytes(),
            "static_mask": self.static_mask.telemetry(),
            "cache_admission": "REJECTED_TO_DETERMINISTIC_ASTAR",
            "cache_reject_reason": self.reject_reason,
            "synchronous_dstar_build_performed": False,
            "cache": dict(self.cache_telemetry),
        }
        return L2PlanResult(
            success=fallback.path is not None,
            path=self.path_global,
            failure_code="" if fallback.path is not None else "L2_NO_PATH_IN_CORRIDOR",
            selected_backend=backend,
            response_ms=_elapsed_ms(started_ns),
            dstar_stats=DStarSearchStats(),
            fallback_stats=fallback,
            changed_cells=int(changed),
            state_reused=False,
            partial_dstar_result_returned=False,
            oracle_cost_error=0.0 if verify_oracle else None,
            diagnostics=diagnostics,
        )

    def initialize(self, *, verify_oracle: bool = False) -> L2PlanResult:
        return self._search_result(
            time.monotonic_ns(), changed=0,
            backend="deterministic_grid_astar_cache_miss",
            verify_oracle=verify_oracle,
        )

    def update(
        self,
        blocked_global: Iterable[Cell],
        *,
        verify_oracle: bool = False,
        force_cold_astar: bool = False,
    ) -> L2PlanResult:
        del force_cold_astar
        started = time.monotonic_ns()
        updated = self._translate_blocked(blocked_global)
        changed = len(self.blocked_global.symmetric_difference(updated))
        self.blocked_global = updated
        if not changed:
            return L2PlanResult(
                success=self.current_path_local is not None,
                path=self.path_global,
                failure_code="" if self.current_path_local is not None else "L2_NO_PATH_IN_CORRIDOR",
                selected_backend="scheduler_reuse_cache_miss_astar",
                response_ms=_elapsed_ms(started),
                dstar_stats=DStarSearchStats(),
                changed_cells=0,
                state_reused=True,
                partial_dstar_result_returned=False,
                oracle_cost_error=0.0 if verify_oracle else None,
                diagnostics={
                    "cache_admission": "REJECTED_TO_DETERMINISTIC_ASTAR",
                    "cache_reject_reason": self.reject_reason,
                    "synchronous_dstar_build_performed": False,
                    "state_memory_bytes": self.state_memory_bytes(),
                    "static_mask": self.static_mask.telemetry(),
                },
            )
        return self._search_result(
            started, changed=changed,
            backend="deterministic_grid_astar_cache_miss",
            verify_oracle=verify_oracle,
        )

    def service_resync(self) -> L2PlanResult:
        self.resync_count += 1
        return self._search_result(
            time.monotonic_ns(), changed=0,
            backend="deterministic_grid_astar_cache_miss_resync",
            verify_oracle=False,
        )

    def state_memory_bytes(self) -> int:
        path_bytes = 0 if self.current_path_local is None else len(self.current_path_local) * 4
        return int(
            self.static_mask.nbytes + path_bytes
            + sys.getsizeof(self.blocked_global) + len(self.blocked_global) * 72
        )


@dataclass(frozen=True)
class R2ActivationTelemetry:
    active_hit: bool
    geometry_cache: CacheTelemetry
    state_cache: CacheTelemetry
    admission_backend: str
    cache_required: bool
    synchronous_dstar_build_performed: bool
    cache_decision_ms: float
    fallback_search_ms: float
    static_mask_pack_ms: float
    static_mask_storage_bytes: int
    static_mask_logical_bytes: int
    activate_ms: float
    evict_ms: float
    evicted_key: str
    released_resident_bytes: int
    active_state_count: int
    resident_bytes: int
    fallback_planner_resident_bytes: int
    rss_before_bytes: int
    rss_after_bytes: int

    def as_dict(self) -> Dict[str, Any]:
        return {
            "active_hit": self.active_hit,
            "geometry_cache_hit": self.geometry_cache.hit,
            "geometry_cache_reject_reason": self.geometry_cache.reject_reason,
            "geometry_restore_ms": self.geometry_cache.wall_ms,
            "geometry_cache_bytes": self.geometry_cache.bytes_on_disk,
            "state_cache_hit": self.state_cache.hit,
            "state_cache_reject_reason": self.state_cache.reject_reason,
            "state_restore_ms": self.state_cache.wall_ms,
            "state_cache_bytes": self.state_cache.bytes_on_disk,
            "admission_backend": self.admission_backend,
            "cache_required": self.cache_required,
            "synchronous_dstar_build_performed": self.synchronous_dstar_build_performed,
            "cache_decision_ms": self.cache_decision_ms,
            "fallback_search_ms": self.fallback_search_ms,
            "static_mask_pack_ms": self.static_mask_pack_ms,
            "static_mask_storage_bytes": self.static_mask_storage_bytes,
            "static_mask_logical_bytes": self.static_mask_logical_bytes,
            "activate_ms": self.activate_ms,
            "evict_ms": self.evict_ms,
            "evicted_key": self.evicted_key,
            "released_resident_bytes": self.released_resident_bytes,
            "active_state_count": self.active_state_count,
            "resident_bytes": self.resident_bytes,
            "fallback_planner_resident_bytes": self.fallback_planner_resident_bytes,
            "rss_before_bytes": self.rss_before_bytes,
            "rss_after_bytes": self.rss_after_bytes,
        }


@dataclass(frozen=True)
class PrebuildTelemetry:
    geometry_cache_hit: bool
    state_cache_hit: bool
    geometry_build_ms: float
    first_solve_ms: float
    geometry_serialize_ms: float
    state_serialize_ms: float
    total_ms: float
    resident_bytes: int
    success: bool

    def as_dict(self) -> Dict[str, Any]:
        return dict(self.__dict__)


class R2L2StateLifecycleManager(L2StateLifecycleManager):
    """Verified-cache-only online admission plus explicit offline prebuild."""

    def __init__(self, *args: Any, require_prebuilt_cache: bool = True, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.require_prebuilt_cache = bool(require_prebuilt_cache)
        self.cache_admission_count = 0
        self.cache_reject_to_astar_count = 0
        self.synchronous_dstar_build_count = 0
        self.prebuild_count = 0

    def prebuild(
        self,
        roi: CorridorROI,
        *,
        dynamic_baseline_version: str = DEFAULT_DYNAMIC_BASELINE,
        verify_oracle: bool = False,
    ) -> PrebuildTelemetry:
        """Build/verify one cache outside the online request path."""
        total_started = time.monotonic_ns()
        geometry_binding = CompactGeometryBinding.from_roi(
            roi, safety_policy_hash=self.safety_policy_hash,
        )
        geometry, geometry_restore = self.geometry_cache.restore(geometry_binding)
        geometry_build_ms = 0.0
        geometry_serialize_ms = 0.0
        if geometry is None:
            started = time.monotonic_ns()
            geometry = CompactCorridorGeometry.build(
                roi, safety_policy_hash=self.safety_policy_hash,
            )
            geometry_build_ms = _elapsed_ms(started)
            geometry_serialize_ms = self.geometry_cache.save(geometry).wall_ms
        state_binding = _r2_state_binding(geometry, roi, dynamic_baseline_version)
        state, state_restore = self.state_cache.restore(geometry, state_binding)
        first_solve_ms = 0.0
        state_serialize_ms = 0.0
        packed_roi = pack_corridor_roi(roi)
        if state is None:
            state = CompactDStarState(geometry, state_binding)
        planner = R2CompactPersistentCorridorDStar(
            packed_roi, geometry, state,
            dstar_wall_budget_ms=self.dstar_wall_budget_ms,
            dstar_max_expansions=self.dstar_max_expansions,
        )
        result = planner.initialize(verify_oracle=verify_oracle)
        first_solve_ms = result.dstar_stats.search_time_ms
        if not state_restore.hit and state.ready:
            state_serialize_ms = self.state_cache.save(state).wall_ms
        resident = planner.state_memory_bytes()
        self.prebuild_count += 1
        telemetry = PrebuildTelemetry(
            geometry_cache_hit=geometry_restore.hit,
            state_cache_hit=state_restore.hit,
            geometry_build_ms=geometry_build_ms,
            first_solve_ms=first_solve_ms,
            geometry_serialize_ms=geometry_serialize_ms,
            state_serialize_ms=state_serialize_ms,
            total_ms=_elapsed_ms(total_started),
            resident_bytes=resident,
            success=result.success,
        )
        del planner, state, geometry
        gc.collect()
        return telemetry

    def activate(
        self,
        roi: CorridorROI,
        *,
        dynamic_baseline_version: str = DEFAULT_DYNAMIC_BASELINE,
        blocked_global: Iterable[Cell] = (),
        verify_oracle: bool = False,
    ) -> Tuple[Any, L2PlanResult, R2ActivationTelemetry]:
        total_started = time.monotonic_ns()
        rss_before = _rss_bytes()
        geometry_binding = CompactGeometryBinding.from_roi(
            roi, safety_policy_hash=self.safety_policy_hash,
        )
        provisional_state_binding = MutableStateBinding(
            geometry_hash=geometry_binding.digest,
            start_cell=roi.binding.start_cell,
            goal_cell=roi.binding.goal_cell,
            dynamic_baseline_version=str(dynamic_baseline_version),
            algorithm_version=ALGORITHM_VERSION,
            format_version=FORMAT_VERSION,
        )
        key = provisional_state_binding.digest
        self.activation_count += 1
        blocked_values = tuple(blocked_global)

        if key in self.active:
            planner = self.active.pop(key)
            self.active[key] = planner
            self.active_hit_count += 1
            result = planner.initialize(verify_oracle=verify_oracle)
            empty = CacheTelemetry(False, "ACTIVE_MEMORY_HIT", 0.0)
            mask = planner.static_mask
            telemetry = R2ActivationTelemetry(
                True, empty, empty, "compact_dstar_active_hit", True, False,
                0.0, 0.0, 0.0, mask.nbytes, mask.logical_nbytes,
                _elapsed_ms(total_started), 0.0, "", 0,
                len(self.active), self.resident_bytes, 0,
                rss_before, _rss_bytes(),
            )
            planner.cache_telemetry = telemetry.as_dict()
            self.last_activation = telemetry  # type: ignore[assignment]
            return planner, result, telemetry

        evict_ms, evicted_key, released = self._evict_if_needed(key)
        decision_started = time.monotonic_ns()
        geometry, geometry_telemetry = self.geometry_cache.restore(geometry_binding)
        if geometry is None:
            state_telemetry = CacheTelemetry(False, "GEOMETRY_UNAVAILABLE", 0.0)
            state = None
        else:
            state_binding = _r2_state_binding(geometry, roi, dynamic_baseline_version)
            if state_binding.digest != key:
                raise AssertionError("r2 provisional state binding changed after restore")
            state, state_telemetry = self.state_cache.restore(geometry, state_binding)
        decision_ms = _elapsed_ms(decision_started)

        if geometry is None or state is None:
            self.cache_reject_to_astar_count += 1
            reason = (
                f"GEOMETRY:{geometry_telemetry.reject_reason}"
                if geometry is None else f"STATE:{state_telemetry.reject_reason}"
            )
            planner = CacheMissAStarPlanner(roi, reject_reason=reason)
            if blocked_values:
                planner.blocked_global = planner._translate_blocked(blocked_values)
            fallback_started = time.monotonic_ns()
            result = planner.initialize(verify_oracle=verify_oracle)
            fallback_ms = _elapsed_ms(fallback_started)
            mask = planner.static_mask
            telemetry = R2ActivationTelemetry(
                False, geometry_telemetry, state_telemetry,
                "deterministic_grid_astar_cache_miss", True, False,
                decision_ms, fallback_ms, mask.pack_ms, mask.nbytes, mask.logical_nbytes,
                _elapsed_ms(total_started), evict_ms, evicted_key, released,
                len(self.active), self.resident_bytes, planner.state_memory_bytes(),
                rss_before, _rss_bytes(),
            )
            planner.cache_telemetry = telemetry.as_dict()
            self.last_activation = telemetry  # type: ignore[assignment]
            return planner, result, telemetry

        packed_roi = pack_corridor_roi(roi)
        planner = R2CompactPersistentCorridorDStar(
            packed_roi, geometry, state,
            dstar_wall_budget_ms=self.dstar_wall_budget_ms,
            dstar_max_expansions=self.dstar_max_expansions,
        )
        if blocked_values:
            if state.initialized:
                raise ValueError("cached mutable state cannot accept non-empty activation baseline")
            planner.prime_blocked(blocked_values)
        result = planner.initialize(verify_oracle=verify_oracle)
        self.active[key] = planner
        self.cache_admission_count += 1
        self.peak_active_state_count = max(self.peak_active_state_count, len(self.active))
        self.peak_resident_bytes = max(self.peak_resident_bytes, self.resident_bytes)
        mask = planner.static_mask
        telemetry = R2ActivationTelemetry(
            False, geometry_telemetry, state_telemetry,
            "compact_dstar_verified_cache", True, False,
            decision_ms, 0.0, mask.pack_ms, mask.nbytes, mask.logical_nbytes,
            _elapsed_ms(total_started), evict_ms, evicted_key, released,
            len(self.active), self.resident_bytes, 0,
            rss_before, _rss_bytes(),
        )
        planner.cache_telemetry = telemetry.as_dict()
        self.last_activation = telemetry  # type: ignore[assignment]
        return planner, result, telemetry

    def telemetry(self) -> Mapping[str, Any]:
        return {
            "activation_count": self.activation_count,
            "active_hit_count": self.active_hit_count,
            "cache_admission_count": self.cache_admission_count,
            "cache_reject_to_astar_count": self.cache_reject_to_astar_count,
            "synchronous_dstar_build_count": self.synchronous_dstar_build_count,
            "prebuild_count": self.prebuild_count,
            "eviction_count": self.eviction_count,
            "active_state_count": len(self.active),
            "resident_bytes": self.resident_bytes,
            "peak_active_state_count": self.peak_active_state_count,
            "peak_resident_bytes": self.peak_resident_bytes,
        }


__all__ = [
    "ALGORITHM_VERSION", "ARCHITECTURE_ID", "CacheMissAStarPlanner",
    "FORMAT_VERSION", "PROTOCOL_ID", "PackedStaticMask", "PrebuildTelemetry",
    "R2ActivationTelemetry", "R2CompactPersistentCorridorDStar",
    "R2L2StateLifecycleManager", "REVISION_ID", "STATIC_MASK_SCHEMA",
    "pack_corridor_roi",
]
