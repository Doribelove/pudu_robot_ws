"""Verified L1-plan artifact used by stable plan and cache CLIs."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import numpy as np

from .pipeline import L1Plan
from .stable_contract import (
    ARCHITECTURE_ID,
    PLAN_BUNDLE_SCHEMA,
    PRODUCTION_BASELINE_ID,
    SOURCE_REVISION,
    sha256_file,
)


Pose = Tuple[float, float, float]


def _hash_value(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str,
    ).encode("utf-8")).hexdigest()


def _array_hash(value: np.ndarray) -> str:
    array = np.ascontiguousarray(value)
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode("ascii"))
    digest.update(np.asarray(array.shape, dtype=np.int64).tobytes())
    digest.update(array.tobytes())
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(str(path), os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _atomic_npz(path: Path, **arrays: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            np.savez_compressed(stream, **arrays)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


@dataclass(frozen=True)
class VerifiedPlanBundle:
    plan: L1Plan
    start_pose: Pose
    goal_pose: Pose
    manifest: Mapping[str, Any]
    root: Path


def write_plan_bundle(
    root: Path,
    plan: L1Plan,
    *,
    start_pose: Sequence[float],
    goal_pose: Sequence[float],
) -> Path:
    """Write one immutable stable L1 artifact with an atomic payload/manifest."""
    bundle = Path(root).resolve()
    if bundle.exists() and any(bundle.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty plan bundle: {bundle}")
    bundle.mkdir(parents=True, exist_ok=True)
    start = tuple(float(value) for value in start_pose)
    goal = tuple(float(value) for value in goal_pose)
    if len(start) != 3 or len(goal) != 3:
        raise ValueError("start_pose and goal_pose must be x,y,yaw triples")
    static = np.ascontiguousarray(plan.static_safe_free, dtype=np.bool_)
    corridor = np.ascontiguousarray(plan.corridor_mask, dtype=np.bool_)
    if static.shape != corridor.shape:
        raise ValueError("static and corridor masks must have identical shapes")
    payload = bundle / "plan_payload.npz"
    _atomic_npz(payload, static_safe_free=static, corridor_mask=corridor)
    fields: Dict[str, Any] = {
        "map_hash": plan.map_hash,
        "map_shape": list(static.shape),
        "map_origin": list(plan.map_origin),
        "resolution": float(plan.resolution),
        "topology_hash": plan.topology_hash,
        "route_edge_ids": list(plan.route_edge_ids),
        "corridor_mask_hash": _array_hash(corridor),
        "static_safe_free_hash": _array_hash(static),
        "start_cell": list(plan.start_cell),
        "goal_cell": list(plan.goal_cell),
        "start_pose": list(start),
        "goal_pose": list(goal),
        "footprint_hash": plan.footprint_hash,
        "route_signature": plan.route_signature,
    }
    manifest = {
        "schema_version": PLAN_BUNDLE_SCHEMA,
        "architecture_id": ARCHITECTURE_ID,
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "source_revision": SOURCE_REVISION,
        "binding": fields,
        "binding_hash": _hash_value(fields),
        "payload": payload.name,
        "payload_sha256": sha256_file(payload),
        "payload_bytes": payload.stat().st_size,
        "diagnostics": dict(plan.diagnostics),
    }
    _atomic_json(bundle / "plan_manifest.json", manifest)
    return bundle


def load_plan_bundle(root: Path) -> VerifiedPlanBundle:
    bundle = Path(root).resolve()
    manifest_path = bundle / "plan_manifest.json"
    if not manifest_path.is_file():
        raise ValueError(f"plan manifest missing: {manifest_path}")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ValueError(f"invalid plan manifest: {exc}") from exc
    for key, expected in (
        ("schema_version", PLAN_BUNDLE_SCHEMA),
        ("architecture_id", ARCHITECTURE_ID),
        ("production_baseline_id", PRODUCTION_BASELINE_ID),
        ("source_revision", SOURCE_REVISION),
    ):
        if manifest.get(key) != expected:
            raise ValueError(f"plan manifest {key} mismatch")
    binding = manifest.get("binding") or {}
    if manifest.get("binding_hash") != _hash_value(binding):
        raise ValueError("plan binding hash mismatch")
    payload = bundle / str(manifest.get("payload", ""))
    if not payload.is_file() or sha256_file(payload) != manifest.get("payload_sha256"):
        raise ValueError("plan payload missing or content hash mismatch")
    try:
        with np.load(payload, allow_pickle=False) as values:
            static = np.ascontiguousarray(values["static_safe_free"], dtype=np.bool_)
            corridor = np.ascontiguousarray(values["corridor_mask"], dtype=np.bool_)
    except Exception as exc:
        raise ValueError(f"invalid plan payload: {exc}") from exc
    if list(static.shape) != binding.get("map_shape") or static.shape != corridor.shape:
        raise ValueError("plan array shape mismatch")
    if _array_hash(static) != binding.get("static_safe_free_hash"):
        raise ValueError("static mask hash mismatch")
    if _array_hash(corridor) != binding.get("corridor_mask_hash"):
        raise ValueError("corridor mask hash mismatch")
    plan = L1Plan(
        static_safe_free=static,
        corridor_mask=corridor,
        start_cell=tuple(int(value) for value in binding["start_cell"]),
        goal_cell=tuple(int(value) for value in binding["goal_cell"]),
        map_hash=str(binding["map_hash"]),
        map_origin=tuple(float(value) for value in binding["map_origin"]),
        resolution=float(binding["resolution"]),
        topology_hash=str(binding["topology_hash"]),
        route_edge_ids=tuple(str(value) for value in binding["route_edge_ids"]),
        footprint_hash=str(binding["footprint_hash"]),
        route_signature=str(binding["route_signature"]),
        diagnostics=dict(manifest.get("diagnostics") or {}),
    )
    return VerifiedPlanBundle(
        plan=plan,
        start_pose=tuple(float(value) for value in binding["start_pose"]),
        goal_pose=tuple(float(value) for value in binding["goal_pose"]),
        manifest=manifest,
        root=bundle,
    )


__all__ = ["Pose", "VerifiedPlanBundle", "load_plan_bundle", "write_plan_bundle"]
