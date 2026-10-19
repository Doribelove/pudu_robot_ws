"""Hash-bound, bounded static-geometry cache for 2A-V3 engineering runs."""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import time
from typing import Any, Mapping

import numpy as np

from .edge_semantic_annotator import (
    EdgeSemanticAnnotation, EdgeSemanticAnnotator, RegionCoverage,
    SemanticEdgeRouter, topology_graph_hash,
)
from .semantic_map import SemanticMapV1, canonical_hash, sha256_file
from .semantic_rasterizer import RasterizedSemantics, SemanticRasterizer
from .topology import load_topology
from . import two_layer_v2_semantic_r1_benchmark as r1


SCHEMA_VERSION = "PLN-02-2A-V3-STATIC-CACHE-V1"
LAST_CACHE_TELEMETRY: dict[str, Any] = {}


class BoundedLRU:
    """Small explicit LRU whose admission is bounded by count and bytes."""

    def __init__(self, *, capacity: int, maximum_bytes: int) -> None:
        if capacity < 1 or maximum_bytes < 1:
            raise ValueError("LRU bounds must be positive")
        self.capacity, self.maximum_bytes = int(capacity), int(maximum_bytes)
        self._items: OrderedDict[str, tuple[Any, int]] = OrderedDict()
        self.resident_bytes = 0
        self.evictions = 0

    def get(self, key: str) -> Any | None:
        item = self._items.pop(str(key), None)
        if item is None:
            return None
        self._items[str(key)] = item
        return item[0]

    def put(self, key: str, value: Any, resident_bytes: int) -> bool:
        size = int(resident_bytes)
        if size < 0 or size > self.maximum_bytes:
            return False
        prior = self._items.pop(str(key), None)
        if prior is not None:
            self.resident_bytes -= prior[1]
        while self._items and (
            len(self._items) >= self.capacity
            or self.resident_bytes+size > self.maximum_bytes
        ):
            _old_key, (_old_value, old_size) = self._items.popitem(last=False)
            self.resident_bytes -= old_size
            self.evictions += 1
        self._items[str(key)] = (value, size)
        self.resident_bytes += size
        return True

    @property
    def active_count(self) -> int:
        return len(self._items)


@dataclass(frozen=True)
class StaticCacheTelemetry:
    key: str
    status: str
    reject_reason: str
    restore_ms: float
    build_ms: float
    serialize_ms: float
    cache_files_bytes: int
    mapped_array_bytes: int
    edge_annotation_count: int
    cache_entry_count: int
    evicted_entries: int

    def to_dict(self) -> dict[str, Any]:
        return dict(self.__dict__)


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _cache_key(
    *, ctx: Any, semantic_map: SemanticMapV1, topology: Any,
    config: Mapping[str, Any],
) -> str:
    return canonical_hash({
        "schema_version": SCHEMA_VERSION,
        "map_hash": ctx.map_sha256,
        "map_yaml_hash": ctx.map_yaml_sha256,
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "topology_hash": topology_graph_hash(topology),
        "footprint": config["protocol"]["footprint"],
        "semantic_safety_margin_m": config["protocol"]["semantic_safety_margin_m"],
        "edge_policy": config["l1_edge_cost"],
        "resolution_m": config["protocol"]["resolution_m"],
    })


def _array_files(raster: RasterizedSemantics) -> dict[str, np.ndarray]:
    values = {
        "class_grid": raster.class_grid,
        "priority_grid": raster.priority_grid,
        "hard_mask": raster.hard_mask,
        "hard_footprint_mask": raster.hard_footprint_mask,
        "no_stopping_mask": raster.no_stopping_mask,
    }
    values.update({f"mask__{name}": value for name, value in raster.masks.items()})
    return values


def _serialize(
    target: Path, *, key: str, raster: RasterizedSemantics,
    annotator: EdgeSemanticAnnotator, maximum_entries: int,
) -> tuple[int, int]:
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{key[:12]}-", dir=target.parent))
    try:
        file_hashes: dict[str, str] = {}
        mapped_bytes = 0
        for name, value in sorted(_array_files(raster).items()):
            path = temporary/f"{name}.npy"
            np.save(path, np.ascontiguousarray(value), allow_pickle=False)
            file_hashes[path.name] = _hash_file(path)
            mapped_bytes += int(np.asarray(value).nbytes)
        annotations = [
            value.to_dict() for _key, value in sorted(annotator._cache.items())
        ]
        annotation_path = temporary/"edge_annotations.json.gz"
        with gzip.open(annotation_path, "wt", encoding="utf-8", compresslevel=6) as stream:
            json.dump(annotations, stream, sort_keys=True, separators=(",", ":"))
        file_hashes[annotation_path.name] = _hash_file(annotation_path)
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "key": key,
            "raster": {
                "semantic_map_hash": raster.semantic_map_hash,
                "policy_hash": raster.policy_hash,
                "resolution": raster.resolution,
                "origin": list(raster.origin),
                "width": raster.width,
                "height": raster.height,
                "metadata": raster.metadata,
                "mask_names": sorted(raster.masks),
            },
            "edge_annotation_count": len(annotations),
            "mapped_array_bytes": mapped_bytes,
            "files": file_hashes,
        }
        manifest_path = temporary/"manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True)+"\n")
        if target.exists():
            # Another process may have completed the identical immutable key.
            shutil.rmtree(temporary)
        else:
            os.replace(temporary, target)
        _evict_disk_entries(target.parent, keep=target.name, maximum_entries=maximum_entries)
        total = sum(path.stat().st_size for path in target.iterdir() if path.is_file())
        return int(total), int(mapped_bytes)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)


def _evict_disk_entries(root: Path, *, keep: str, maximum_entries: int) -> int:
    entries = sorted(
        (path for path in root.iterdir() if path.is_dir() and not path.name.startswith(".")),
        key=lambda path: (path.stat().st_mtime_ns, path.name),
    )
    evicted = 0
    while len(entries) > max(1, int(maximum_entries)):
        victim = next((path for path in entries if path.name != keep), None)
        if victim is None:
            break
        shutil.rmtree(victim)
        entries.remove(victim)
        evicted += 1
    return evicted


def _load(target: Path, *, expected_key: str) -> tuple[RasterizedSemantics, list[EdgeSemanticAnnotation], int, int]:
    manifest_path = target/"manifest.json"
    if not manifest_path.exists():
        raise ValueError("CACHE_MANIFEST_MISSING")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("CACHE_SCHEMA_MISMATCH")
    if manifest.get("key") != expected_key:
        raise ValueError("CACHE_KEY_MISMATCH")
    for name, expected in manifest.get("files", {}).items():
        path = target/name
        if not path.is_file() or _hash_file(path) != expected:
            raise ValueError(f"CACHE_CONTENT_HASH_MISMATCH:{name}")
    meta = manifest["raster"]
    arrays = {
        path.stem: np.load(path, mmap_mode="r", allow_pickle=False)
        for path in target.glob("*.npy")
    }
    masks = {name: arrays[f"mask__{name}"].astype(bool, copy=False) for name in meta["mask_names"]}
    raster = RasterizedSemantics(
        semantic_map_hash=str(meta["semantic_map_hash"]),
        policy_hash=str(meta["policy_hash"]), resolution=float(meta["resolution"]),
        origin=tuple(meta["origin"]), width=int(meta["width"]), height=int(meta["height"]),
        masks=masks, class_grid=arrays["class_grid"], priority_grid=arrays["priority_grid"],
        hard_mask=arrays["hard_mask"].astype(bool, copy=False),
        hard_footprint_mask=arrays["hard_footprint_mask"].astype(bool, copy=False),
        no_stopping_mask=arrays["no_stopping_mask"].astype(bool, copy=False),
        metadata=dict(meta.get("metadata") or {}),
    )
    with gzip.open(target/"edge_annotations.json.gz", "rt", encoding="utf-8") as stream:
        raw = json.load(stream)
    annotations = []
    for item in raw:
        payload = dict(item)
        payload["region_coverage"] = [RegionCoverage(**value) for value in payload["region_coverage"]]
        annotations.append(EdgeSemanticAnnotation(**payload))
    size = sum(path.stat().st_size for path in target.iterdir() if path.is_file())
    return raster, annotations, int(size), int(manifest["mapped_array_bytes"])


def prepare_static_cached(
    extracted_dir: Path, semantic_map_path: Path, topology_cache: Path,
    config: Mapping[str, Any], *, output: Path | None = None,
    cache_root: Path | None = None, maximum_disk_entries: int = 2,
) -> tuple[Any, ...]:
    """Drop-in, online-only replacement for r1 ``_prepare``.

    Query generation is deliberately omitted: r14 always consumes an explicit,
    hash-frozen query file.  Raster and edge annotations are immutable and
    restored only after schema/key/content validation.
    """
    global LAST_CACHE_TELEMETRY
    started = time.monotonic()
    context_started = time.monotonic()
    ctx = r1._context((Path(extracted_dir)/"optemap.yaml").resolve())
    semantic_map = SemanticMapV1.load(Path(semantic_map_path).resolve())
    semantic_map.validate_against_map(ctx.hospital_map)
    topology = load_topology(
        Path(topology_cache), ctx.hospital_map, config["protocol"]["footprint"],
        padding_m=.05, safety_margin_m=.05, allow_unknown=False,
    )
    context_ms = (time.monotonic()-context_started)*1000.0
    key = _cache_key(ctx=ctx, semantic_map=semantic_map, topology=topology, config=config)
    root = Path(cache_root or (
        Path(extracted_dir).parent/"results"/"2a_v3_r14_static_cache"
    )).resolve()
    target = root/key
    restore_ms = build_ms = serialize_ms = 0.0
    reject_reason = ""
    status = "MISS"
    try:
        restore_started = time.monotonic()
        raster, annotations, cache_bytes, mapped_bytes = _load(target, expected_key=key)
        restore_ms = (time.monotonic()-restore_started)*1000.0
        status = "HIT"
    except (OSError, ValueError, json.JSONDecodeError) as error:
        reject_reason = str(error) if target.exists() else "CACHE_ENTRY_ABSENT"
        build_started = time.monotonic()
        raster = SemanticRasterizer(
            footprint=config["protocol"]["footprint"],
            safety_margin_m=float(config["protocol"]["semantic_safety_margin_m"]),
        ).rasterize(semantic_map, hospital_map=ctx.hospital_map)
        annotator = EdgeSemanticAnnotator(
            ctx.hospital_map, semantic_map, raster,
            base_map_hash=ctx.map_sha256, topology_hash=topology_graph_hash(topology),
            policy=config["l1_edge_cost"],
        )
        annotator.precompute(topology.graph.edges)
        build_ms = (time.monotonic()-build_started)*1000.0
        serialize_started = time.monotonic()
        cache_bytes, mapped_bytes = _serialize(
            target, key=key, raster=raster, annotator=annotator,
            maximum_entries=maximum_disk_entries,
        )
        serialize_ms = (time.monotonic()-serialize_started)*1000.0
        annotations = list(annotator._cache.values())
        status = "REBUILT" if target.exists() and reject_reason != "CACHE_ENTRY_ABSENT" else "MISS_BUILT"
    annotator = EdgeSemanticAnnotator(
        ctx.hospital_map, semantic_map, raster,
        base_map_hash=ctx.map_sha256, topology_hash=topology_graph_hash(topology),
        policy=config["l1_edge_cost"],
    )
    annotator._cache = {
        (int(value.edge_id), bool(value.traversal_reversed)): value for value in annotations
    }
    expected_count = 2*len(topology.graph.edges)
    if len(annotator._cache) != expected_count:
        raise ValueError("CACHE_EDGE_ANNOTATION_COVERAGE_MISMATCH")
    router = SemanticEdgeRouter(topology, annotator)
    telemetry = StaticCacheTelemetry(
        key=key, status=status, reject_reason=reject_reason,
        restore_ms=restore_ms, build_ms=build_ms, serialize_ms=serialize_ms,
        cache_files_bytes=cache_bytes, mapped_array_bytes=mapped_bytes,
        edge_annotation_count=len(annotator._cache),
        cache_entry_count=sum(path.is_dir() for path in root.iterdir()),
        evicted_entries=0,
    ).to_dict()
    telemetry.update({
        "context_and_topology_ms": context_ms,
        "total_prepare_ms": (time.monotonic()-started)*1000.0,
        "cache_root": str(root),
    })
    LAST_CACHE_TELEMETRY = telemetry
    # Preserve the legacy tuple shape.  r14 gets queries only from its frozen
    # file, so the query bundle is intentionally empty.
    return (
        ctx, semantic_map, raster, topology, annotator, router, ([], {}, {}),
        float(build_ms), float(context_ms), float(restore_ms),
    )


__all__ = [
    "SCHEMA_VERSION", "BoundedLRU", "StaticCacheTelemetry",
    "prepare_static_cached", "LAST_CACHE_TELEMETRY",
]
