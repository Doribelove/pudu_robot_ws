"""Compact, verified and bounded L2 state lifecycle for 3D-V1/r1.

Only statically footprint-safe corridor cells receive compact integer state
IDs. Immutable geometry and mutable goal-rooted D* state have independent,
content-addressed cache manifests. Dynamic occupancy is a boolean overlay;
the static geometry is never mutated.
"""

from __future__ import annotations

import gc
import hashlib
import heapq
import json
import math
import os
import resource
import tempfile
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Set, Tuple

import numpy as np

from arena_evaluation.dstar_lite import DStarSearchStats, INF

from .l2_incremental import (
    Cell,
    CorridorROI,
    GridAStarResult,
    L2PlanResult,
    deterministic_grid_astar,
)


ARCHITECTURE_ID = "3D-V1"
REVISION_ID = "r1-l2-state-lifecycle-soak"
PROTOCOL_ID = "PLN-02-3D-V1-R1-L2-LIFECYCLE-V1"
GEOMETRY_SCHEMA = "3D-V1-r1-compact-geometry-v2"
STATE_SCHEMA = "3D-V1-r1-compact-dstar-state-v2"
ALGORITHM_VERSION = "compact-dstar-lite-corner-safe-f64-v2"
ADJACENCY_RULE = "8-neighbor-euclidean-corner-safe-int16-delta-v2"
FORMAT_VERSION = 2
DEFAULT_DYNAMIC_BASELINE = "empty-dynamic-overlay-v1"
SAFETY_POLICY = "footprint-safe-static-mask+dynamic-radius-7-v1"
OFFSETS: Tuple[Cell, ...] = (
    (-1, -1), (-1, 0), (-1, 1), (0, -1),
    (0, 1), (1, -1), (1, 0), (1, 1),
)
OFFSET_INDEX = {offset: index for index, offset in enumerate(OFFSETS)}
NO_NEIGHBOR_DELTA = np.iinfo(np.int16).min


def _elapsed_ms(started_ns: int) -> float:
    return (time.monotonic_ns() - started_ns) / 1.0e6


def _stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str,
    ).encode("utf-8")).hexdigest()


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _rss_bytes() -> int:
    try:
        for line in Path("/proc/self/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _atomic_npz(path: Path, **arrays: np.ndarray) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Optional[Path] = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w+b", prefix=path.name + ".", suffix=".tmp",
            dir=path.parent, delete=False,
        ) as stream:
            temporary = Path(stream.name)
            np.savez_compressed(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        return int(path.stat().st_size)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def _atomic_json(path: Path, value: Mapping[str, Any]) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")
    temporary: Optional[Path] = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w+b", prefix=path.name + ".", suffix=".tmp",
            dir=path.parent, delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        return len(payload)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


@dataclass(frozen=True)
class CompactGeometryBinding:
    map_hash: str
    map_shape: Tuple[int, int]
    map_origin: Tuple[float, float, float]
    resolution: float
    topology_hash: str
    route_edge_ids: Tuple[str, ...]
    corridor_mask_hash: str
    footprint_hash: str
    safety_policy_hash: str
    adjacency_rule: str = ADJACENCY_RULE
    format_version: int = FORMAT_VERSION

    @classmethod
    def from_roi(
        cls, roi: CorridorROI, *, safety_policy_hash: str = SAFETY_POLICY,
    ) -> "CompactGeometryBinding":
        binding = roi.binding
        return cls(
            map_hash=binding.map_hash,
            map_shape=binding.map_shape,
            map_origin=binding.map_origin,
            resolution=binding.resolution,
            topology_hash=binding.topology_hash,
            route_edge_ids=binding.route_edge_ids,
            corridor_mask_hash=binding.corridor_mask_hash,
            footprint_hash=binding.footprint_hash,
            safety_policy_hash=str(safety_policy_hash),
        )

    def as_dict(self) -> Dict[str, Any]:
        return {
            "map_hash": self.map_hash,
            "map_shape": list(self.map_shape),
            "map_origin": list(self.map_origin),
            "resolution": self.resolution,
            "topology_hash": self.topology_hash,
            "route_edge_ids": list(self.route_edge_ids),
            "corridor_mask_hash": self.corridor_mask_hash,
            "footprint_hash": self.footprint_hash,
            "safety_policy_hash": self.safety_policy_hash,
            "adjacency_rule": self.adjacency_rule,
            "format_version": self.format_version,
        }

    @property
    def digest(self) -> str:
        return _stable_hash(self.as_dict())


@dataclass(frozen=True)
class MutableStateBinding:
    geometry_hash: str
    start_cell: Cell
    goal_cell: Cell
    dynamic_baseline_version: str
    algorithm_version: str = ALGORITHM_VERSION
    format_version: int = FORMAT_VERSION

    def as_dict(self) -> Dict[str, Any]:
        return {
            "geometry_hash": self.geometry_hash,
            "start_cell": list(self.start_cell),
            "goal_cell": list(self.goal_cell),
            "dynamic_baseline_version": self.dynamic_baseline_version,
            "algorithm_version": self.algorithm_version,
            "format_version": self.format_version,
        }

    @property
    def digest(self) -> str:
        return _stable_hash(self.as_dict())


@dataclass(frozen=True)
class CacheTelemetry:
    hit: bool
    reject_reason: str
    wall_ms: float
    bytes_on_disk: int = 0
    content_hash: str = ""


@dataclass
class CompactCorridorGeometry:
    binding: CompactGeometryBinding
    bbox: Tuple[int, int, int, int]
    roi_shape: Tuple[int, int]
    state_cells_linear: np.ndarray
    neighbor_deltas: np.ndarray
    global_corridor_cells: int
    build_diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self._linear_view = memoryview(self.state_cells_linear)
        self._neighbor_view = memoryview(self.neighbor_deltas.reshape(-1))

    @classmethod
    def build(
        cls,
        roi: CorridorROI,
        *,
        safety_policy_hash: str = SAFETY_POLICY,
    ) -> "CompactCorridorGeometry":
        total_started = time.monotonic_ns()
        started = time.monotonic_ns()
        linear = np.ascontiguousarray(
            np.flatnonzero(roi.base_free.reshape(-1)), dtype=np.int32,
        )
        width = int(roi.shape[1])
        rows = linear // width
        columns = linear % width
        enumeration_ms = _elapsed_ms(started)
        started = time.monotonic_ns()
        state_id_grid = np.full(roi.shape, -1, dtype=np.int32)
        state_id_grid[rows, columns] = np.arange(len(rows), dtype=np.int32)
        mapping_ms = _elapsed_ms(started)
        started = time.monotonic_ns()
        neighbors = np.full(
            (len(linear), len(OFFSETS)), NO_NEIGHBOR_DELTA, dtype=np.int16,
        )
        flat_grid = state_id_grid.reshape(-1)
        source_ids = np.arange(len(linear), dtype=np.int32)
        height = int(roi.shape[0])
        for offset_index, (drow, dcolumn) in enumerate(OFFSETS):
            valid = (
                (rows + drow >= 0) & (rows + drow < height)
                & (columns + dcolumn >= 0) & (columns + dcolumn < width)
            )
            candidate_ids = source_ids[valid]
            target_linear = (
                (rows[valid] + drow) * width + columns[valid] + dcolumn
            )
            targets = flat_grid[target_linear]
            traversable = targets >= 0
            if drow and dcolumn:
                side_a = flat_grid[rows[valid] * width + columns[valid] + dcolumn]
                side_b = flat_grid[(rows[valid] + drow) * width + columns[valid]]
                traversable &= (side_a >= 0) & (side_b >= 0)
            candidate_ids = candidate_ids[traversable]
            targets = targets[traversable]
            deltas = targets.astype(np.int64) - candidate_ids.astype(np.int64)
            if len(deltas) and (
                int(deltas.min()) <= int(NO_NEIGHBOR_DELTA)
                or int(deltas.max()) > int(np.iinfo(np.int16).max)
            ):
                raise ValueError("neighbor state-ID delta exceeds int16 compact format")
            neighbors[candidate_ids, offset_index] = deltas.astype(np.int16)
        adjacency_ms = _elapsed_ms(started)
        del state_id_grid, flat_grid
        return cls(
            binding=CompactGeometryBinding.from_roi(
                roi, safety_policy_hash=safety_policy_hash,
            ),
            bbox=roi.bbox,
            roi_shape=roi.shape,
            state_cells_linear=linear,
            neighbor_deltas=neighbors,
            global_corridor_cells=roi.global_corridor_cells,
            build_diagnostics={
                "safe_cell_enumeration_ms": enumeration_ms,
                "cell_to_state_mapping_ms": mapping_ms,
                "geometry_build_ms": _elapsed_ms(total_started),
                "safe_cell_count": int(len(linear)),
                "roi_array_cells": int(roi.base_free.size),
                "adjacency_build_ms": adjacency_ms,
                "adjacency_materialized": True,
                "adjacency_encoding": "int16_state_id_delta",
            },
        )

    @property
    def state_count(self) -> int:
        return int(self.state_cells_linear.shape[0])

    @property
    def resident_bytes(self) -> int:
        return int(self.state_cells_linear.nbytes + self.neighbor_deltas.nbytes)

    def contains_global(self, cell: Cell) -> bool:
        row, column = int(cell[0]), int(cell[1])
        return self.bbox[0] <= row < self.bbox[1] and self.bbox[2] <= column < self.bbox[3]

    def global_to_local(self, cell: Cell) -> Cell:
        if not self.contains_global(cell):
            raise ValueError(f"global cell outside compact ROI: {cell}")
        return int(cell[0]) - self.bbox[0], int(cell[1]) - self.bbox[2]

    def local_to_global(self, cell: Cell) -> Cell:
        return int(cell[0]) + self.bbox[0], int(cell[1]) + self.bbox[2]

    def state_id_from_local(self, cell: Cell, *, required: bool = True) -> int:
        row, column = int(cell[0]), int(cell[1])
        if not (0 <= row < self.roi_shape[0] and 0 <= column < self.roi_shape[1]):
            if required:
                raise ValueError(f"local cell outside compact ROI: {cell}")
            return -1
        linear = row * int(self.roi_shape[1]) + column
        result = int(np.searchsorted(self.state_cells_linear, linear))
        if result >= self.state_count or int(self._linear_view[result]) != linear:
            result = -1
        if required and result < 0:
            raise ValueError(f"cell is not statically footprint-safe: {cell}")
        return result

    def state_id_from_global(self, cell: Cell, *, required: bool = True) -> int:
        if not self.contains_global(cell):
            if required:
                raise ValueError(f"global cell outside compact ROI: {cell}")
            return -1
        return self.state_id_from_local(self.global_to_local(cell), required=required)

    def local_cell(self, state_id: int) -> Cell:
        if not 0 <= int(state_id) < self.state_count:
            raise ValueError(f"invalid compact state id: {state_id}")
        linear = int(self._linear_view[int(state_id)])
        return divmod(linear, int(self.roi_shape[1]))

    def global_cell(self, state_id: int) -> Cell:
        return self.local_to_global(self.local_cell(state_id))

    def neighbor_ids(self, state_id: int) -> Iterator[int]:
        state_id = int(state_id)
        base = state_id * len(OFFSETS)
        for offset_index in range(len(OFFSETS)):
            delta = self._neighbor_view[base + offset_index]
            if delta != NO_NEIGHBOR_DELTA:
                yield state_id + delta

    def neighbor_id_at(self, state_id: int, offset_index: int) -> int:
        delta = self._neighbor_view[int(state_id) * len(OFFSETS) + int(offset_index)]
        return -1 if delta == NO_NEIGHBOR_DELTA else int(state_id) + delta

    def state_ids_from_global(self, cells: Iterable[Cell]) -> Set[int]:
        width = int(self.roi_shape[1])
        values = sorted({
            (int(cell[0]) - self.bbox[0]) * width + int(cell[1]) - self.bbox[2]
            for cell in cells if self.contains_global(cell)
        })
        if not values:
            return set()
        query = np.asarray(values, dtype=np.int64)
        indices = np.searchsorted(self.state_cells_linear, query)
        valid = indices < self.state_count
        valid_indices = indices[valid]
        valid_query = query[valid]
        matches = self.state_cells_linear[valid_indices] == valid_query
        return {int(value) for value in valid_indices[matches]}

    def state_ids_for_local_path(self, cells: Sequence[Cell]) -> List[int]:
        if not cells:
            return []
        coordinates = np.asarray(cells, dtype=np.int64)
        if coordinates.ndim != 2 or coordinates.shape[1] != 2:
            raise ValueError("path cells must have shape (N, 2)")
        if (
            np.any(coordinates[:, 0] < 0)
            or np.any(coordinates[:, 0] >= self.roi_shape[0])
            or np.any(coordinates[:, 1] < 0)
            or np.any(coordinates[:, 1] >= self.roi_shape[1])
        ):
            raise ValueError("path contains a cell outside compact ROI")
        linear = coordinates[:, 0] * int(self.roi_shape[1]) + coordinates[:, 1]
        indices = np.searchsorted(self.state_cells_linear, linear)
        if (
            np.any(indices >= self.state_count)
            or not np.array_equal(self.state_cells_linear[indices], linear)
        ):
            raise ValueError("path contains a statically unsafe cell")
        return [int(value) for value in indices]

    def arrays_hash(self) -> str:
        digest = hashlib.sha256()
        for value in (
            np.asarray(self.bbox, dtype=np.int32),
            np.asarray(self.roi_shape, dtype=np.int32),
            self.state_cells_linear,
            self.neighbor_deltas,
        ):
            digest.update(np.ascontiguousarray(value).tobytes())
        return digest.hexdigest()


class VerifiedGeometryCache:
    def __init__(self, root: Path) -> None:
        self.root = Path(root).resolve()

    def _paths(self, digest: str) -> Tuple[Path, Path]:
        directory = self.root / "geometry" / digest
        return directory / "payload.npz", directory / "manifest.json"

    def save(self, geometry: CompactCorridorGeometry) -> CacheTelemetry:
        started = time.monotonic_ns()
        payload, manifest_path = self._paths(geometry.binding.digest)
        payload_bytes = _atomic_npz(
            payload,
            bbox=np.asarray(geometry.bbox, dtype=np.int32),
            roi_shape=np.asarray(geometry.roi_shape, dtype=np.int32),
            state_cells_linear=geometry.state_cells_linear,
            neighbor_deltas=geometry.neighbor_deltas,
            global_corridor_cells=np.asarray([geometry.global_corridor_cells], dtype=np.int64),
        )
        payload_hash = _file_hash(payload)
        manifest = {
            "schema_version": GEOMETRY_SCHEMA,
            "binding": geometry.binding.as_dict(),
            "binding_hash": geometry.binding.digest,
            "arrays_hash": geometry.arrays_hash(),
            "payload_sha256": payload_hash,
            "payload_bytes": payload_bytes,
            "state_count": geometry.state_count,
        }
        total_bytes = payload_bytes + _atomic_json(manifest_path, manifest)
        return CacheTelemetry(False, "", _elapsed_ms(started), total_bytes, payload_hash)

    def restore(
        self, expected: CompactGeometryBinding,
    ) -> Tuple[Optional[CompactCorridorGeometry], CacheTelemetry]:
        started = time.monotonic_ns()
        payload, manifest_path = self._paths(expected.digest)
        if not payload.is_file() or not manifest_path.is_file():
            return None, CacheTelemetry(False, "CACHE_MISS", _elapsed_ms(started))
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest.get("schema_version") != GEOMETRY_SCHEMA:
                raise ValueError("SCHEMA_MISMATCH")
            if manifest.get("binding_hash") != expected.digest:
                raise ValueError("BINDING_HASH_MISMATCH")
            if manifest.get("binding") != expected.as_dict():
                raise ValueError("BINDING_FIELDS_MISMATCH")
            payload_hash = _file_hash(payload)
            if payload_hash != manifest.get("payload_sha256"):
                raise ValueError("CONTENT_HASH_MISMATCH")
            with np.load(payload, allow_pickle=False) as values:
                bbox = tuple(int(value) for value in values["bbox"])
                roi_shape = tuple(int(value) for value in values["roi_shape"])
                linear = np.ascontiguousarray(values["state_cells_linear"], dtype=np.int32)
                neighbors = np.ascontiguousarray(values["neighbor_deltas"], dtype=np.int16)
                global_cells = int(values["global_corridor_cells"][0])
            if len(bbox) != 4 or len(roi_shape) != 2:
                raise ValueError("ARRAY_SHAPE_MISMATCH")
            if linear.ndim != 1:
                raise ValueError("STATE_CELL_SHAPE_MISMATCH")
            if neighbors.shape != (len(linear), len(OFFSETS)):
                raise ValueError("NEIGHBOR_SHAPE_MISMATCH")
            if len(linear) and (
                int(linear[0]) < 0
                or int(linear[-1]) >= int(roi_shape[0]) * int(roi_shape[1])
                or np.any(linear[1:] <= linear[:-1])
            ):
                raise ValueError("STATE_CELL_ORDER_MISMATCH")
            geometry = CompactCorridorGeometry(
                expected, bbox, roi_shape, linear, neighbors, global_cells,
                {"restored": True},
            )
            if geometry.arrays_hash() != manifest.get("arrays_hash"):
                raise ValueError("ARRAY_CONTENT_HASH_MISMATCH")
            if geometry.state_count != int(manifest.get("state_count", -1)):
                raise ValueError("STATE_COUNT_MISMATCH")
            if geometry.state_count:
                sources = np.repeat(
                    np.arange(geometry.state_count, dtype=np.int64), len(OFFSETS),
                )
                flat_delta = neighbors.reshape(-1).astype(np.int64)
                present = flat_delta != int(NO_NEIGHBOR_DELTA)
                targets = sources[present] + flat_delta[present]
                if np.any(targets < 0) or np.any(targets >= geometry.state_count):
                    raise ValueError("NEIGHBOR_TARGET_OUT_OF_RANGE")
            bytes_on_disk = int(payload.stat().st_size + manifest_path.stat().st_size)
            return geometry, CacheTelemetry(
                True, "", _elapsed_ms(started), bytes_on_disk, payload_hash,
            )
        except Exception as exc:
            return None, CacheTelemetry(False, str(exc) or type(exc).__name__, _elapsed_ms(started))


@dataclass(frozen=True)
class CompactAStarResult:
    path_state_ids: Optional[List[int]]
    path_global: Optional[List[Cell]]
    cost: float
    expanded_nodes: int
    generated_nodes: int
    search_time_ms: float
    timeout_triggered: bool = False


def deterministic_compact_astar(
    geometry: CompactCorridorGeometry,
    blocked: np.ndarray,
    start_id: int,
    goal_id: int,
    *,
    timeout_s: Optional[float] = None,
    max_expansions: Optional[int] = None,
) -> CompactAStarResult:
    """Deterministic array-backed A* used as a parity oracle in tests."""
    started_ns = time.monotonic_ns()
    blocked = np.asarray(blocked, dtype=bool)
    if blocked.shape != (geometry.state_count,):
        raise ValueError("blocked state array has wrong shape")
    if blocked[start_id] or blocked[goal_id]:
        return CompactAStarResult(None, None, INF, 0, 0, _elapsed_ms(started_ns))
    deadline = None if timeout_s is None else time.monotonic() + max(0.0, float(timeout_s))
    distance = np.full(geometry.state_count, INF, dtype=np.float64)
    previous = np.full(geometry.state_count, -1, dtype=np.int32)
    start_cell = geometry.local_cell(start_id)
    goal_cell = geometry.local_cell(goal_id)
    distance[start_id] = 0.0
    serial = 0
    queue: List[Tuple[float, float, int, int, int, int]] = [(
        math.hypot(start_cell[0] - goal_cell[0], start_cell[1] - goal_cell[1]),
        0.0, serial, start_cell[0], start_cell[1], start_id,
    )]
    expanded = 0
    generated = 1
    timed_out = False
    while queue:
        if deadline is not None and time.monotonic() >= deadline:
            timed_out = True
            break
        _estimate, cost, _serial, _row, _column, state_id = heapq.heappop(queue)
        if cost != float(distance[state_id]):
            continue
        if state_id == goal_id:
            break
        if max_expansions is not None and expanded >= max(0, int(max_expansions)):
            timed_out = True
            break
        expanded += 1
        first = geometry.local_cell(state_id)
        for target in geometry.neighbor_ids(state_id):
            if blocked[state_id] or blocked[target]:
                continue
            second = geometry.local_cell(target)
            if first[0] != second[0] and first[1] != second[1]:
                drow = second[0] - first[0]
                dcolumn = second[1] - first[1]
                side_a = geometry.neighbor_id_at(
                    state_id, OFFSET_INDEX[(0, dcolumn)],
                )
                side_b = geometry.neighbor_id_at(
                    state_id, OFFSET_INDEX[(drow, 0)],
                )
                if side_a < 0 or side_b < 0 or blocked[side_a] or blocked[side_b]:
                    continue
                step = math.sqrt(2.0)
            else:
                step = 1.0
            candidate = cost + step
            if candidate < float(distance[target]):
                distance[target] = candidate
                previous[target] = state_id
                target_cell = geometry.local_cell(target)
                serial += 1
                heapq.heappush(queue, (
                    candidate + math.hypot(
                        target_cell[0] - goal_cell[0], target_cell[1] - goal_cell[1],
                    ),
                    candidate, serial, target_cell[0], target_cell[1], target,
                ))
                generated += 1
    if timed_out or not math.isfinite(float(distance[goal_id])):
        return CompactAStarResult(
            None, None, INF, expanded, generated, _elapsed_ms(started_ns), timed_out,
        )
    path = [goal_id]
    cursor = goal_id
    while cursor != start_id:
        cursor = int(previous[cursor])
        if cursor < 0:
            return CompactAStarResult(
                None, None, INF, expanded, generated, _elapsed_ms(started_ns), timed_out,
            )
        path.append(cursor)
    path.reverse()
    return CompactAStarResult(
        path, [geometry.global_cell(value) for value in path],
        float(distance[goal_id]), expanded, generated, _elapsed_ms(started_ns), False,
    )


class CompactDStarState:
    """Goal-rooted D* Lite over compact state IDs and float64 arrays."""

    def __init__(
        self,
        geometry: CompactCorridorGeometry,
        binding: MutableStateBinding,
    ) -> None:
        started = time.monotonic_ns()
        self.geometry = geometry
        self.binding = binding
        self.start_id = geometry.state_id_from_global(binding.start_cell)
        self.goal_id = geometry.state_id_from_global(binding.goal_cell)
        count = geometry.state_count
        self.blocked = np.zeros(count, dtype=np.bool_)
        self.g = np.full(count, INF, dtype=np.float64)
        self.rhs = np.full(count, INF, dtype=np.float64)
        self.rhs[self.goal_id] = 0.0
        self.queued: Dict[int, Tuple[float, float, int]] = {}
        self.open: List[Tuple[float, float, int, int]] = []
        self.serial = 0
        self.km = 0.0
        self.current_path_ids: Optional[List[int]] = None
        self.queue_push_count = 0
        self.queue_pop_count = 0
        self.update_vertex_count = 0
        self.predecessor_visit_count = 0
        self.neighbor_visit_count = 0
        self.total_expanded_nodes = 0
        self.update_count = 0
        self.reinitialize_count = 0
        self.ready = True
        self.initialized = False
        self.cache_pristine = True
        self.invalid_extraction_injected = False
        self._refresh_views()
        self._push(self.goal_id)
        self.constructor_ms = _elapsed_ms(started)

    def _refresh_views(self) -> None:
        self._blocked_view = memoryview(self.blocked)
        self._g_view = memoryview(self.g)
        self._rhs_view = memoryview(self.rhs)

    @property
    def resident_bytes(self) -> int:
        arrays = (self.blocked, self.g, self.rhs)
        path_bytes = 0 if self.current_path_ids is None else len(self.current_path_ids) * 4
        heap_bytes = len(self.open) * 48
        queued_bytes = len(self.queued) * 112
        return int(
            sum(value.nbytes for value in arrays)
            + path_bytes + heap_bytes + queued_bytes
        )

    def _heuristic(self, first: int, second: int) -> float:
        first_cell = self.geometry.local_cell(first)
        second_cell = self.geometry.local_cell(second)
        return math.hypot(first_cell[0] - second_cell[0], first_cell[1] - second_cell[1])

    def _calculate_key(self, state_id: int) -> Tuple[float, float]:
        best = min(self._g_view[state_id], self._rhs_view[state_id])
        return best + self._heuristic(self.start_id, state_id) + self.km, best

    def _push(self, state_id: int) -> None:
        if self._g_view[state_id] == self._rhs_view[state_id]:
            self.queued.pop(state_id, None)
            return
        key = self._calculate_key(state_id)
        existing = self.queued.get(state_id)
        if existing is not None and existing[:2] == key:
            return
        token = self.serial
        self.queued[state_id] = (key[0], key[1], token)
        heapq.heappush(self.open, (key[0], key[1], token, state_id))
        self.serial += 1
        self.queue_push_count += 1

    def _edge_cost(self, first: int, second: int) -> float:
        if self._blocked_view[first] or self._blocked_view[second]:
            return INF
        first_cell = self.geometry.local_cell(first)
        second_cell = self.geometry.local_cell(second)
        drow = second_cell[0] - first_cell[0]
        dcolumn = second_cell[1] - first_cell[1]
        if drow and dcolumn:
            side_a = self.geometry.neighbor_id_at(
                first, OFFSET_INDEX[(0, dcolumn)],
            )
            side_b = self.geometry.neighbor_id_at(
                first, OFFSET_INDEX[(drow, 0)],
            )
            if (
                side_a < 0 or side_b < 0
                or self._blocked_view[side_a] or self._blocked_view[side_b]
            ):
                return INF
            return math.sqrt(2.0)
        return 1.0

    def update_vertex(self, state_id: int) -> None:
        self.update_vertex_count += 1
        if state_id != self.goal_id:
            best = INF
            if not self._blocked_view[state_id]:
                neighbor_base = state_id * len(OFFSETS)
                neighbor_view = self.geometry._neighbor_view
                for offset_index, (drow, dcolumn) in enumerate(OFFSETS):
                    delta = neighbor_view[neighbor_base + offset_index]
                    if delta == NO_NEIGHBOR_DELTA:
                        continue
                    successor = state_id + delta
                    self.neighbor_visit_count += 1
                    if self._blocked_view[successor]:
                        continue
                    if drow and dcolumn:
                        side_a = self.geometry.neighbor_id_at(
                            state_id, OFFSET_INDEX[(0, dcolumn)],
                        )
                        side_b = self.geometry.neighbor_id_at(
                            state_id, OFFSET_INDEX[(drow, 0)],
                        )
                        if (
                            side_a < 0 or side_b < 0
                            or self._blocked_view[side_a] or self._blocked_view[side_b]
                        ):
                            continue
                        value = math.sqrt(2.0) + self._g_view[successor]
                    else:
                        value = 1.0 + self._g_view[successor]
                    if value < best:
                        best = value
            self._rhs_view[state_id] = best
        self._push(state_id)

    def set_blocked_ids(self, new_blocked_ids: Set[int]) -> int:
        old_ids = set(int(value) for value in np.flatnonzero(self.blocked))
        changed = old_ids.symmetric_difference(new_blocked_ids)
        if not changed:
            return 0
        self.cache_pristine = False
        for state_id in changed:
            self._blocked_view[state_id] = state_id in new_blocked_ids
        affected: Set[int] = set(changed)
        for state_id in changed:
            affected.update(self.geometry.neighbor_ids(state_id))
        for state_id in sorted(affected):
            self.update_vertex(state_id)
        self.update_count += 1
        return len(changed)

    def compute_shortest_path(
        self,
        *,
        timeout_s: Optional[float] = None,
        max_expansions: Optional[int] = None,
    ) -> DStarSearchStats:
        started = time.monotonic_ns()
        initial_queue_size = len(self.open)
        pushes_before = self.queue_push_count
        pops_before = self.queue_pop_count
        updates_before = self.update_vertex_count
        predecessors_before = self.predecessor_visit_count
        expanded = 0
        generated = 0
        deadline = None if timeout_s is None else time.monotonic() + max(0.0, float(timeout_s))
        timeout = False
        while self.open:
            top_key = (self.open[0][0], self.open[0][1])
            start_key = self._calculate_key(self.start_id)
            if not (
                top_key < start_key
                or self._rhs_view[self.start_id] != self._g_view[self.start_id]
            ):
                break
            if deadline is not None and time.monotonic() >= deadline:
                timeout = True
                break
            if max_expansions is not None and expanded >= max(0, int(max_expansions)):
                timeout = True
                break
            old_1, old_2, token, state_id = heapq.heappop(self.open)
            self.queue_pop_count += 1
            if self.queued.get(state_id) != (old_1, old_2, token):
                continue
            del self.queued[state_id]
            key_new = self._calculate_key(state_id)
            if (old_1, old_2) < key_new:
                self._push(state_id)
                continue
            if self._g_view[state_id] > self._rhs_view[state_id]:
                self._g_view[state_id] = self._rhs_view[state_id]
                expanded += 1
                for predecessor in self.geometry.neighbor_ids(state_id):
                    self.predecessor_visit_count += 1
                    self.update_vertex(predecessor)
            else:
                self._g_view[state_id] = INF
                expanded += 1
                self.update_vertex(state_id)
                for predecessor in self.geometry.neighbor_ids(state_id):
                    self.predecessor_visit_count += 1
                    self.update_vertex(predecessor)
            generated += 1
        elapsed = _elapsed_ms(started)
        self.total_expanded_nodes += expanded
        no_path = not math.isfinite(self._g_view[self.start_id])
        return DStarSearchStats(
            expanded_nodes=expanded,
            generated_nodes=generated,
            queue_pops=self.queue_pop_count - pops_before,
            queue_pushes=self.queue_push_count - pushes_before,
            initial_queue_size=initial_queue_size,
            final_queue_size=len(self.open),
            update_vertex_count=self.update_vertex_count - updates_before,
            search_time_ms=elapsed,
            timeout_triggered=timeout,
            no_path=no_path,
        )

    def extract_path_ids(self, *, max_length: Optional[int] = None) -> Optional[List[int]]:
        if self.invalid_extraction_injected:
            return None
        if not math.isfinite(self._g_view[self.start_id]):
            return None
        limit = max_length or max(2, self.geometry.state_count * 2)
        path = [self.start_id]
        current = self.start_id
        visited = {current}
        while current != self.goal_id and len(path) < limit:
            choices: List[Tuple[float, int, int, int]] = []
            for successor in self.geometry.neighbor_ids(current):
                value = self._edge_cost(current, successor) + self._g_view[successor]
                if math.isfinite(value):
                    row, column = self.geometry.local_cell(successor)
                    choices.append((value, row, column, successor))
            if not choices:
                return None
            _value, _row, _column, next_id = min(choices)
            if next_id in visited:
                return None
            path.append(next_id)
            visited.add(next_id)
            current = next_id
        return path if current == self.goal_id else None

    def path_is_valid(self, path_ids: Optional[Sequence[int]]) -> bool:
        if not path_ids:
            return False
        if int(path_ids[0]) != self.start_id or int(path_ids[-1]) != self.goal_id:
            return False
        for state_id in path_ids:
            if self._blocked_view[int(state_id)]:
                return False
        for first, second in zip(path_ids, path_ids[1:]):
            if int(second) not in set(self.geometry.neighbor_ids(int(first))):
                return False
            if not math.isfinite(self._edge_cost(int(first), int(second))):
                return False
        return True

    def path_cost(self, path_ids: Optional[Sequence[int]]) -> float:
        if not path_ids:
            return INF
        return float(sum(
            self._edge_cost(int(first), int(second))
            for first, second in zip(path_ids, path_ids[1:])
        ))

    def global_path(self, path_ids: Optional[Sequence[int]] = None) -> Optional[List[Cell]]:
        path = self.current_path_ids if path_ids is None else path_ids
        if path is None:
            return None
        return [self.geometry.global_cell(int(value)) for value in path]

    def rebuild_open_from_inconsistent(self) -> int:
        self.open.clear()
        self.queued.clear()
        self.serial = 0
        for state_id in np.flatnonzero(self.g != self.rhs):
            self._push(int(state_id))
        return len(self.open)


class VerifiedMutableStateCache:
    def __init__(self, root: Path) -> None:
        self.root = Path(root).resolve()

    def _paths(self, digest: str) -> Tuple[Path, Path]:
        directory = self.root / "state" / digest
        return directory / "payload.npz", directory / "manifest.json"

    @staticmethod
    def _arrays_hash(state: CompactDStarState) -> str:
        digest = hashlib.sha256()
        for value in (state.blocked, state.g, state.rhs):
            digest.update(np.ascontiguousarray(value).tobytes())
        path = np.asarray(state.current_path_ids or [], dtype=np.int32)
        digest.update(path.tobytes())
        return digest.hexdigest()

    def save(self, state: CompactDStarState) -> CacheTelemetry:
        started = time.monotonic_ns()
        if not state.initialized or not state.ready:
            return CacheTelemetry(False, "STATE_NOT_CONVERGED", _elapsed_ms(started))
        if not state.cache_pristine:
            return CacheTelemetry(False, "STATE_DIRTY_DYNAMIC_BASELINE", _elapsed_ms(started))
        payload, manifest_path = self._paths(state.binding.digest)
        path = np.asarray(state.current_path_ids or [], dtype=np.int32)
        payload_bytes = _atomic_npz(
            payload,
            blocked=np.packbits(state.blocked, bitorder="little"),
            g=state.g,
            rhs=state.rhs,
            current_path_ids=path,
            state_count=np.asarray([state.geometry.state_count], dtype=np.int64),
        )
        payload_hash = _file_hash(payload)
        manifest = {
            "schema_version": STATE_SCHEMA,
            "binding": state.binding.as_dict(),
            "binding_hash": state.binding.digest,
            "payload_sha256": payload_hash,
            "payload_bytes": payload_bytes,
            "arrays_hash": self._arrays_hash(state),
            "state_count": state.geometry.state_count,
            "path_count": int(len(path)),
            "converged": True,
        }
        total_bytes = payload_bytes + _atomic_json(manifest_path, manifest)
        return CacheTelemetry(False, "", _elapsed_ms(started), total_bytes, payload_hash)

    def restore(
        self,
        geometry: CompactCorridorGeometry,
        expected: MutableStateBinding,
    ) -> Tuple[Optional[CompactDStarState], CacheTelemetry]:
        started = time.monotonic_ns()
        payload, manifest_path = self._paths(expected.digest)
        if not payload.is_file() or not manifest_path.is_file():
            return None, CacheTelemetry(False, "CACHE_MISS", _elapsed_ms(started))
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest.get("schema_version") != STATE_SCHEMA:
                raise ValueError("SCHEMA_MISMATCH")
            if manifest.get("binding_hash") != expected.digest:
                raise ValueError("BINDING_HASH_MISMATCH")
            if manifest.get("binding") != expected.as_dict():
                raise ValueError("BINDING_FIELDS_MISMATCH")
            if manifest.get("converged") is not True:
                raise ValueError("NON_CONVERGED_STATE")
            payload_hash = _file_hash(payload)
            if payload_hash != manifest.get("payload_sha256"):
                raise ValueError("CONTENT_HASH_MISMATCH")
            with np.load(payload, allow_pickle=False) as values:
                state_count = int(values["state_count"][0])
                g = np.ascontiguousarray(values["g"], dtype=np.float64)
                rhs = np.ascontiguousarray(values["rhs"], dtype=np.float64)
                packed = np.ascontiguousarray(values["blocked"], dtype=np.uint8)
                path_ids = np.ascontiguousarray(values["current_path_ids"], dtype=np.int32)
            if state_count != geometry.state_count or int(manifest.get("state_count", -1)) != state_count:
                raise ValueError("STATE_COUNT_MISMATCH")
            if g.shape != (state_count,) or rhs.shape != (state_count,):
                raise ValueError("STATE_ARRAY_SHAPE_MISMATCH")
            if np.any(np.isnan(g)) or np.any(np.isnan(rhs)):
                raise ValueError("NAN_STATE_VALUE")
            state = CompactDStarState(geometry, expected)
            state.g = g
            state.rhs = rhs
            state.blocked = np.unpackbits(
                packed, count=state_count, bitorder="little",
            ).astype(np.bool_, copy=False)
            state._refresh_views()
            state.current_path_ids = [int(value) for value in path_ids]
            state.initialized = True
            state.ready = True
            state.cache_pristine = True
            if state._rhs_view[state.goal_id] != 0.0:
                raise ValueError("GOAL_RHS_MISMATCH")
            if state.current_path_ids and not state.path_is_valid(state.current_path_ids):
                raise ValueError("CACHED_PATH_INVALID")
            if self._arrays_hash(state) != manifest.get("arrays_hash"):
                raise ValueError("ARRAY_CONTENT_HASH_MISMATCH")
            state.rebuild_open_from_inconsistent()
            bytes_on_disk = int(payload.stat().st_size + manifest_path.stat().st_size)
            return state, CacheTelemetry(
                True, "", _elapsed_ms(started), bytes_on_disk, payload_hash,
            )
        except Exception as exc:
            return None, CacheTelemetry(False, str(exc) or type(exc).__name__, _elapsed_ms(started))


class CompactPersistentCorridorDStar:
    """r1 planner adapter preserving the r0 L2 public contract."""

    def __init__(
        self,
        roi: CorridorROI,
        geometry: CompactCorridorGeometry,
        state: CompactDStarState,
        *,
        dstar_wall_budget_ms: float = 500.0,
        dstar_max_expansions: int = 20_000,
    ) -> None:
        self.roi = roi
        self.geometry = geometry
        self.state = state
        self.dstar_wall_budget_ms = max(0.01, float(dstar_wall_budget_ms))
        self.dstar_max_expansions = max(1, int(dstar_max_expansions))
        self.fallback_count = 0
        self.resync_count = 0
        self.cache_telemetry: Dict[str, Any] = {}

    @property
    def planner(self) -> CompactDStarState:
        return self.state

    @property
    def binding_hash(self) -> str:
        return self.state.binding.digest

    @property
    def dstar_ready(self) -> bool:
        return self.state.ready

    @property
    def path_global(self) -> Optional[List[Cell]]:
        return self.state.global_path()

    @property
    def dynamic_blocked_local(self) -> Set[Cell]:
        return {
            self.geometry.local_cell(int(value))
            for value in np.flatnonzero(self.state.blocked)
        }

    @property
    def current_free(self) -> np.ndarray:
        result = self.roi.base_free.copy()
        blocked_ids = np.flatnonzero(self.state.blocked)
        if len(blocked_ids):
            result.reshape(-1)[self.geometry.state_cells_linear[blocked_ids]] = False
        return result

    def _translate_blocked(self, blocked_global: Iterable[Cell]) -> Set[int]:
        return self.geometry.state_ids_from_global(
            (int(raw[0]), int(raw[1])) for raw in blocked_global
        )

    def _result(
        self,
        *,
        started_ns: int,
        path_ids: Optional[List[int]],
        failure: str,
        backend: str,
        stats: DStarSearchStats,
        fallback: Optional[GridAStarResult] = None,
        changed: int = 0,
        reused: bool = False,
        oracle_cost_error: Optional[float] = None,
    ) -> L2PlanResult:
        self.state.current_path_ids = None if path_ids is None else list(path_ids)
        return L2PlanResult(
            success=path_ids is not None,
            path=self.state.global_path(path_ids),
            failure_code=failure,
            selected_backend=backend,
            response_ms=_elapsed_ms(started_ns),
            dstar_stats=stats,
            fallback_stats=fallback,
            changed_cells=int(changed),
            state_reused=bool(reused),
            partial_dstar_result_returned=False,
            oracle_cost_error=oracle_cost_error,
            diagnostics={
                "binding_hash": self.binding_hash,
                "geometry_hash": self.geometry.binding.digest,
                "roi_bbox": list(self.roi.bbox),
                "roi_shape": list(self.roi.shape),
                "roi_array_cells": int(self.roi.base_free.size),
                "corridor_cells": self.geometry.state_count,
                "global_corridor_cells": self.roi.global_corridor_cells,
                "dynamic_blocked_cells": int(np.count_nonzero(self.state.blocked)),
                "dstar_ready": self.state.ready,
                "reinitialize_count": self.state.reinitialize_count,
                "fallback_count": self.fallback_count,
                "resync_count": self.resync_count,
                "geometry_resident_bytes": self.geometry.resident_bytes,
                "mutable_state_resident_bytes": self.state.resident_bytes,
                "state_memory_bytes": self.state_memory_bytes(),
                "cache": dict(self.cache_telemetry),
                "predecessor_visits_total": self.state.predecessor_visit_count,
                "neighbor_visits_total": self.state.neighbor_visit_count,
            },
        )

    def initialize(self, *, verify_oracle: bool = False) -> L2PlanResult:
        started_ns = time.monotonic_ns()
        if self.state.initialized:
            path_ids = self.state.current_path_ids
            return self._result(
                started_ns=started_ns,
                path_ids=path_ids,
                failure="" if path_ids is not None else "L2_NO_PATH_IN_CORRIDOR",
                backend="compact_dstar_cache_restore",
                stats=DStarSearchStats(),
                reused=True,
                oracle_cost_error=0.0 if verify_oracle else None,
            )
        stats = self.state.compute_shortest_path()
        path_ids = self.state.extract_path_ids()
        self.state.initialized = True
        self.state.ready = not stats.timeout_triggered
        oracle_error: Optional[float] = None
        if verify_oracle:
            oracle = deterministic_grid_astar(
                self.current_free, self.roi.start_local, self.roi.goal_local,
            )
            if (path_ids is None) != (oracle.path is None):
                raise AssertionError("compact D* reachability differs from grid A* oracle")
            oracle_error = 0.0 if path_ids is None else abs(
                self.state.path_cost(path_ids) - oracle.cost
            )
            if oracle_error > 1.0e-9:
                raise AssertionError("compact D* cost differs from grid A* oracle")
        return self._result(
            started_ns=started_ns,
            path_ids=path_ids,
            failure="" if path_ids is not None else "L2_NO_PATH_IN_CORRIDOR",
            backend="compact_persistent_dstar",
            stats=stats,
            oracle_cost_error=oracle_error,
        )

    def prime_blocked(self, blocked_global: Iterable[Cell]) -> int:
        if self.state.initialized:
            raise RuntimeError("prime_blocked() is only valid before initialize()")
        return self.state.set_blocked_ids(self._translate_blocked(blocked_global))

    def update(
        self,
        blocked_global: Iterable[Cell],
        *,
        verify_oracle: bool = False,
        force_cold_astar: bool = False,
    ) -> L2PlanResult:
        if not self.state.initialized:
            raise RuntimeError("initialize() must complete before dynamic updates")
        started_ns = time.monotonic_ns()
        changed = self.state.set_blocked_ids(self._translate_blocked(blocked_global))
        if not changed:
            return self._result(
                started_ns=started_ns,
                path_ids=self.state.current_path_ids,
                failure="" if self.state.current_path_ids else "L2_NO_PATH_IN_CORRIDOR",
                backend="scheduler_reuse",
                stats=DStarSearchStats(),
                changed=0,
                reused=True,
                oracle_cost_error=0.0 if verify_oracle else None,
            )
        if force_cold_astar:
            self.state.ready = False
            self.fallback_count += 1
            fallback = deterministic_grid_astar(
                self.current_free, self.roi.start_local, self.roi.goal_local,
            )
            path_ids = (
                None if fallback.path is None
                else self.geometry.state_ids_for_local_path(fallback.path)
            )
            return self._result(
                started_ns=started_ns,
                path_ids=path_ids,
                failure="" if path_ids is not None else "L2_NO_PATH_IN_CORRIDOR",
                backend="deterministic_grid_astar_direct",
                stats=DStarSearchStats(),
                fallback=fallback,
                changed=changed,
                reused=False,
                oracle_cost_error=0.0 if verify_oracle else None,
            )
        if self.state.ready:
            stats = self.state.compute_shortest_path(
                timeout_s=self.dstar_wall_budget_ms / 1000.0,
                max_expansions=self.dstar_max_expansions,
            )
            if not stats.timeout_triggered:
                path_ids = self.state.extract_path_ids()
                if path_ids is None and not stats.no_path:
                    self.state.ready = False
                elif path_ids is not None and not self.state.path_is_valid(path_ids):
                    self.state.ready = False
                else:
                    oracle_error: Optional[float] = None
                    if verify_oracle:
                        oracle = deterministic_grid_astar(
                            self.current_free, self.roi.start_local, self.roi.goal_local,
                        )
                        if (path_ids is None) != (oracle.path is None):
                            raise AssertionError("compact D* reachability differs from grid A*")
                        oracle_error = 0.0 if path_ids is None else abs(
                            self.state.path_cost(path_ids) - oracle.cost
                        )
                        if oracle_error > 1.0e-9:
                            raise AssertionError("compact D* cost differs from grid A*")
                    return self._result(
                        started_ns=started_ns,
                        path_ids=path_ids,
                        failure="" if path_ids is not None else "L2_NO_PATH_IN_CORRIDOR",
                        backend="compact_persistent_dstar",
                        stats=stats,
                        changed=changed,
                        reused=True,
                        oracle_cost_error=oracle_error,
                    )
        else:
            stats = DStarSearchStats(timeout_triggered=True)
        self.state.ready = False
        self.fallback_count += 1
        fallback = deterministic_grid_astar(
            self.current_free, self.roi.start_local, self.roi.goal_local,
        )
        path_ids = (
            None if fallback.path is None
            else self.geometry.state_ids_for_local_path(fallback.path)
        )
        return self._result(
            started_ns=started_ns,
            path_ids=path_ids,
            failure="" if path_ids is not None else "L2_NO_PATH_IN_CORRIDOR",
            backend="deterministic_grid_astar_fallback",
            stats=stats,
            fallback=fallback,
            changed=changed,
            reused=False,
            oracle_cost_error=0.0 if verify_oracle else None,
        )

    def service_resync(self) -> L2PlanResult:
        started_ns = time.monotonic_ns()
        if self.state.ready:
            return self._result(
                started_ns=started_ns,
                path_ids=self.state.current_path_ids,
                failure="" if self.state.current_path_ids is not None else "L2_NO_PATH_IN_CORRIDOR",
                backend="resync_not_required",
                stats=DStarSearchStats(),
                reused=True,
            )
        self.resync_count += 1
        stats = self.state.compute_shortest_path()
        path_ids = self.state.extract_path_ids()
        self.state.ready = not stats.timeout_triggered and (
            path_ids is not None or stats.no_path
        )
        return self._result(
            started_ns=started_ns,
            path_ids=path_ids,
            failure="" if path_ids is not None else "L2_NO_PATH_IN_CORRIDOR",
            backend="compact_persistent_dstar_resync",
            stats=stats,
        )

    def state_memory_bytes(self) -> int:
        return int(
            self.roi.base_free.nbytes
            + self.geometry.resident_bytes
            + self.state.resident_bytes
        )


@dataclass(frozen=True)
class ActivationTelemetry:
    active_hit: bool
    geometry_cache: CacheTelemetry
    state_cache: CacheTelemetry
    geometry_build_ms: float
    state_build_ms: float
    state_serialize_ms: float
    activate_ms: float
    evict_ms: float
    evicted_key: str
    released_resident_bytes: int
    active_state_count: int
    resident_bytes: int
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
            "geometry_build_ms": self.geometry_build_ms,
            "state_build_ms": self.state_build_ms,
            "state_serialize_ms": self.state_serialize_ms,
            "activate_ms": self.activate_ms,
            "evict_ms": self.evict_ms,
            "evicted_key": self.evicted_key,
            "released_resident_bytes": self.released_resident_bytes,
            "active_state_count": self.active_state_count,
            "resident_bytes": self.resident_bytes,
            "rss_before_bytes": self.rss_before_bytes,
            "rss_after_bytes": self.rss_after_bytes,
        }


class L2StateLifecycleManager:
    """Verified disk cache plus configurable 1/2-entry mutable-state LRU."""

    HARD_MAX_ACTIVE_STATES = 2

    def __init__(
        self,
        cache_root: Path,
        *,
        max_active_states: int = 1,
        dstar_wall_budget_ms: float = 500.0,
        dstar_max_expansions: int = 20_000,
        safety_policy_hash: str = SAFETY_POLICY,
    ) -> None:
        if not 1 <= int(max_active_states) <= self.HARD_MAX_ACTIVE_STATES:
            raise ValueError("max_active_states must be 1 or 2")
        self.cache_root = Path(cache_root).resolve()
        self.max_active_states = int(max_active_states)
        self.dstar_wall_budget_ms = float(dstar_wall_budget_ms)
        self.dstar_max_expansions = int(dstar_max_expansions)
        self.safety_policy_hash = str(safety_policy_hash)
        self.geometry_cache = VerifiedGeometryCache(self.cache_root)
        self.state_cache = VerifiedMutableStateCache(self.cache_root)
        self.active: "OrderedDict[str, CompactPersistentCorridorDStar]" = OrderedDict()
        self.activation_count = 0
        self.active_hit_count = 0
        self.eviction_count = 0
        self.peak_active_state_count = 0
        self.peak_resident_bytes = 0
        self.last_activation: Optional[ActivationTelemetry] = None

    @property
    def resident_bytes(self) -> int:
        return int(sum(value.state_memory_bytes() for value in self.active.values()))

    def _evict_if_needed(self, incoming_key: str) -> Tuple[float, str, int]:
        if incoming_key in self.active or len(self.active) < self.max_active_states:
            return 0.0, "", 0
        started = time.monotonic_ns()
        key, planner = self.active.popitem(last=False)
        released = planner.state_memory_bytes()
        if planner.state.ready and planner.state.cache_pristine:
            self.state_cache.save(planner.state)
        del planner
        gc.collect()
        self.eviction_count += 1
        return _elapsed_ms(started), key, released

    def activate(
        self,
        roi: CorridorROI,
        *,
        dynamic_baseline_version: str = DEFAULT_DYNAMIC_BASELINE,
        blocked_global: Iterable[Cell] = (),
        verify_oracle: bool = False,
    ) -> Tuple[CompactPersistentCorridorDStar, L2PlanResult, ActivationTelemetry]:
        total_started = time.monotonic_ns()
        rss_before = _rss_bytes()
        geometry_binding = CompactGeometryBinding.from_roi(
            roi, safety_policy_hash=self.safety_policy_hash,
        )
        state_binding = MutableStateBinding(
            geometry_hash=geometry_binding.digest,
            start_cell=roi.binding.start_cell,
            goal_cell=roi.binding.goal_cell,
            dynamic_baseline_version=str(dynamic_baseline_version),
        )
        key = state_binding.digest
        self.activation_count += 1
        if key in self.active:
            planner = self.active.pop(key)
            self.active[key] = planner
            self.active_hit_count += 1
            result = planner.initialize(verify_oracle=verify_oracle)
            empty = CacheTelemetry(False, "ACTIVE_MEMORY_HIT", 0.0)
            telemetry = ActivationTelemetry(
                True, empty, empty, 0.0, 0.0, 0.0,
                _elapsed_ms(total_started), 0.0, "", 0,
                len(self.active), self.resident_bytes, rss_before, _rss_bytes(),
            )
            planner.cache_telemetry = telemetry.as_dict()
            self.last_activation = telemetry
            return planner, result, telemetry

        evict_ms, evicted_key, released = self._evict_if_needed(key)
        geometry, geometry_telemetry = self.geometry_cache.restore(geometry_binding)
        geometry_build_ms = 0.0
        if geometry is None:
            started = time.monotonic_ns()
            geometry = CompactCorridorGeometry.build(
                roi, safety_policy_hash=self.safety_policy_hash,
            )
            geometry_build_ms = _elapsed_ms(started)
            self.geometry_cache.save(geometry)

        state, state_telemetry = self.state_cache.restore(geometry, state_binding)
        state_build_ms = 0.0
        serialize_ms = 0.0
        blocked_values = tuple(blocked_global)
        if state is None:
            started = time.monotonic_ns()
            state = CompactDStarState(geometry, state_binding)
            state_build_ms = _elapsed_ms(started)
        planner = CompactPersistentCorridorDStar(
            roi, geometry, state,
            dstar_wall_budget_ms=self.dstar_wall_budget_ms,
            dstar_max_expansions=self.dstar_max_expansions,
        )
        if blocked_values:
            if state.initialized:
                # A cache bound to an empty dynamic baseline cannot silently
                # activate against a non-empty overlay.
                raise ValueError("cached mutable state cannot accept non-empty activation baseline")
            planner.prime_blocked(blocked_values)
        result = planner.initialize(verify_oracle=verify_oracle)
        if not state_telemetry.hit and state.ready:
            saved = self.state_cache.save(state)
            serialize_ms = saved.wall_ms
        self.active[key] = planner
        self.peak_active_state_count = max(self.peak_active_state_count, len(self.active))
        self.peak_resident_bytes = max(self.peak_resident_bytes, self.resident_bytes)
        telemetry = ActivationTelemetry(
            False, geometry_telemetry, state_telemetry,
            geometry_build_ms, state_build_ms, serialize_ms,
            _elapsed_ms(total_started), evict_ms, evicted_key, released,
            len(self.active), self.resident_bytes, rss_before, _rss_bytes(),
        )
        planner.cache_telemetry = telemetry.as_dict()
        self.last_activation = telemetry
        return planner, result, telemetry

    def save_active(self) -> List[CacheTelemetry]:
        return [self.state_cache.save(planner.state) for planner in self.active.values()]

    def clear(self) -> Mapping[str, Any]:
        started = time.monotonic_ns()
        released = self.resident_bytes
        for planner in self.active.values():
            if planner.state.ready and planner.state.cache_pristine:
                self.state_cache.save(planner.state)
        self.active.clear()
        gc.collect()
        return {
            "clear_ms": _elapsed_ms(started),
            "released_resident_bytes": released,
            "active_state_count": 0,
            "resident_bytes": self.resident_bytes,
        }


__all__ = [
    "ADJACENCY_RULE", "ALGORITHM_VERSION", "ARCHITECTURE_ID",
    "ActivationTelemetry", "CacheTelemetry", "CompactAStarResult",
    "CompactCorridorGeometry", "CompactDStarState",
    "CompactGeometryBinding", "CompactPersistentCorridorDStar",
    "DEFAULT_DYNAMIC_BASELINE", "FORMAT_VERSION", "GEOMETRY_SCHEMA",
    "L2StateLifecycleManager", "MutableStateBinding", "PROTOCOL_ID",
    "REVISION_ID", "SAFETY_POLICY", "STATE_SCHEMA",
    "VerifiedGeometryCache", "VerifiedMutableStateCache",
    "deterministic_compact_astar",
]
