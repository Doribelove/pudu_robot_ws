"""Production cache bundle, offline prebuild, verification, and inventory."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml

from .l2_incremental import CorridorROI
from .l2_state_lifecycle import (
    ADJACENCY_RULE,
    DEFAULT_DYNAMIC_BASELINE,
    GEOMETRY_SCHEMA,
    SAFETY_POLICY,
    STATE_SCHEMA,
    CompactGeometryBinding,
)
from .pipeline import L1Plan
from .production_io import Pose, load_plan_bundle
from .r2_state_lifecycle import (
    ALGORITHM_VERSION,
    STATIC_MASK_SCHEMA,
    R2L2StateLifecycleManager,
    _r2_state_binding,
)
from .stable_contract import (
    ARCHITECTURE_ID,
    CACHE_BUNDLE_SCHEMA,
    DEFAULT_STABLE_CONFIG,
    PRODUCTION_BASELINE_ID,
    PROTOCOL_ID,
    SOURCE_REVISION,
    load_stable_config,
    sha256_file,
)


def _stable_hash(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str,
    ).encode("utf-8")).hexdigest()


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


class StableCacheBundleError(ValueError):
    """A present bundle has a global version/source/config mismatch."""


@dataclass(frozen=True)
class CacheRouteStatus:
    status: str
    reason: str
    binding_hash: str
    production_ready: bool
    deep_verified: bool = False
    selected_backend: str = ""
    oracle_cost_error: Optional[float] = None

    @property
    def accepted(self) -> bool:
        return self.status == "HIT_VERIFIED" and self.production_ready

    def as_dict(self) -> Dict[str, Any]:
        return dict(self.__dict__)


def route_binding(
    plan: L1Plan,
    *,
    start_pose: Sequence[float],
    goal_pose: Sequence[float],
    dynamic_baseline_version: str = DEFAULT_DYNAMIC_BASELINE,
) -> Dict[str, Any]:
    start = tuple(float(value) for value in start_pose)
    goal = tuple(float(value) for value in goal_pose)
    if len(start) != 3 or len(goal) != 3:
        raise ValueError("start_pose and goal_pose must be x,y,yaw triples")
    fields = dict(plan.binding_fields())
    return {
        "map_hash": plan.map_hash,
        "map_shape": list(plan.static_safe_free.shape),
        "map_origin": list(plan.map_origin),
        "resolution": float(plan.resolution),
        "topology_hash": plan.topology_hash,
        "route_edge_ids": list(plan.route_edge_ids),
        "route_signature": plan.route_signature,
        "corridor_mask_hash": fields["corridor_mask_hash"],
        "start_cell": list(plan.start_cell),
        "goal_cell": list(plan.goal_cell),
        "start_position": list(start[:2]),
        "start_yaw": start[2],
        "goal_position": list(goal[:2]),
        "goal_yaw": goal[2],
        "footprint_hash": plan.footprint_hash,
        "safety_policy": SAFETY_POLICY,
        "adjacency_rule": ADJACENCY_RULE,
        "dynamic_baseline_version": str(dynamic_baseline_version),
        "algorithm_version": ALGORITHM_VERSION,
        "geometry_schema": GEOMETRY_SCHEMA,
        "state_schema": STATE_SCHEMA,
        "static_mask_schema": STATIC_MASK_SCHEMA,
        "cache_bundle_schema": CACHE_BUNDLE_SCHEMA,
        "source_revision": SOURCE_REVISION,
    }


class StableCacheBundle:
    """Versioned wrapper around the accepted r2 geometry/state cache."""

    MANIFEST = "manifest.json"

    def __init__(self, root: Path, *, config_path: Optional[Path] = None) -> None:
        self.root = Path(root).resolve()
        self.config = load_stable_config(config_path)
        self.config_path = Path(self.config["_config_path"])
        self.config_sha256 = str(self.config["_config_sha256"])
        self.cache_root = self.root / "cache"
        self.entries_root = self.root / "entries"

    @property
    def manifest_path(self) -> Path:
        return self.root / self.MANIFEST

    def _expected_header(self) -> Dict[str, Any]:
        return {
            "schema_version": CACHE_BUNDLE_SCHEMA,
            "architecture_id": ARCHITECTURE_ID,
            "production_baseline_id": PRODUCTION_BASELINE_ID,
            "source_revision": SOURCE_REVISION,
            "protocol_id": PROTOCOL_ID,
            "stable_config_sha256": self.config_sha256,
            "algorithm_version": ALGORITHM_VERSION,
            "geometry_schema": GEOMETRY_SCHEMA,
            "state_schema": STATE_SCHEMA,
            "static_mask_schema": STATIC_MASK_SCHEMA,
            "adjacency_rule": ADJACENCY_RULE,
        }

    def initialize(self) -> Mapping[str, Any]:
        if self.manifest_path.exists():
            return self.validate_global_manifest()
        self.root.mkdir(parents=True, exist_ok=True)
        manifest = {
            **self._expected_header(),
            "production_ready": False,
            "entry_count": 0,
            "cache_root": "cache",
            "entries_root": "entries",
            "atomic_write": "temp+fsync+atomic-rename",
        }
        _atomic_json(self.manifest_path, manifest)
        return manifest

    def validate_global_manifest(self) -> Mapping[str, Any]:
        if not self.manifest_path.is_file():
            raise FileNotFoundError(f"stable cache bundle missing: {self.manifest_path}")
        try:
            manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise StableCacheBundleError(f"invalid bundle manifest: {exc}") from exc
        for key, expected in self._expected_header().items():
            if manifest.get(key) != expected:
                raise StableCacheBundleError(
                    f"bundle {key} mismatch: expected {expected!r}, got {manifest.get(key)!r}"
                )
        if manifest.get("cache_root") != "cache" or manifest.get("entries_root") != "entries":
            raise StableCacheBundleError("bundle path layout mismatch")
        return manifest

    def _entry_path(self, digest: str) -> Path:
        return self.entries_root / f"{digest}.json"

    def _roi(self, plan: L1Plan) -> CorridorROI:
        return CorridorROI.from_global(
            plan.static_safe_free,
            plan.corridor_mask,
            plan.start_cell,
            plan.goal_cell,
            binding_fields=plan.binding_fields(),
        )

    def prebuild_route(
        self,
        plan: L1Plan,
        *,
        start_pose: Sequence[float],
        goal_pose: Sequence[float],
        dynamic_baseline_version: str = DEFAULT_DYNAMIC_BASELINE,
        verify_oracle: bool = True,
    ) -> Mapping[str, Any]:
        """Offline-only build followed by a verified warm restore."""
        self.initialize()
        self.validate_global_manifest()
        fields = route_binding(
            plan,
            start_pose=start_pose,
            goal_pose=goal_pose,
            dynamic_baseline_version=dynamic_baseline_version,
        )
        digest = _stable_hash(fields)
        roi = self._roi(plan)
        policy = self.config["policy"]
        cache = self.config["cache"]
        manager = R2L2StateLifecycleManager(
            self.cache_root,
            max_active_states=int(cache["max_active_states_default"]),
            dstar_wall_budget_ms=float(policy["dstar_wall_budget_ms"]),
            dstar_max_expansions=int(policy["dstar_max_expansions"]),
        )
        prebuild = manager.prebuild(
            roi,
            dynamic_baseline_version=dynamic_baseline_version,
            verify_oracle=verify_oracle,
        )
        manager.clear()
        planner, activation_result, activation = manager.activate(
            roi,
            dynamic_baseline_version=dynamic_baseline_version,
            verify_oracle=verify_oracle,
        )
        geometry_binding = CompactGeometryBinding.from_roi(
            roi, safety_policy_hash=manager.safety_policy_hash,
        )
        geometry, geometry_status = manager.geometry_cache.restore(geometry_binding)
        if geometry is None:
            raise RuntimeError(f"offline geometry verification failed: {geometry_status.reject_reason}")
        state_binding = _r2_state_binding(geometry, roi, dynamic_baseline_version)
        files = [
            self.cache_root / "geometry" / geometry_binding.digest / "payload.npz",
            self.cache_root / "geometry" / geometry_binding.digest / "manifest.json",
            self.cache_root / "state" / state_binding.digest / "payload.npz",
            self.cache_root / "state" / state_binding.digest / "manifest.json",
        ]
        file_records = []
        for path in files:
            if not path.is_file():
                raise RuntimeError(f"prebuilt cache artifact missing: {path}")
            file_records.append({
                "path": str(path.relative_to(self.root)),
                "sha256": sha256_file(path),
                "bytes": path.stat().st_size,
            })
        ready = bool(
            prebuild.success
            and activation.geometry_cache.hit
            and activation.state_cache.hit
            and activation_result.success
            and float(activation_result.oracle_cost_error or 0.0) == 0.0
            and manager.synchronous_dstar_build_count == 0
        )
        entry = {
            "schema_version": CACHE_BUNDLE_SCHEMA,
            "architecture_id": ARCHITECTURE_ID,
            "production_baseline_id": PRODUCTION_BASELINE_ID,
            "source_revision": SOURCE_REVISION,
            "binding": fields,
            "binding_hash": digest,
            "geometry_binding_hash": geometry_binding.digest,
            "state_binding_hash": state_binding.digest,
            "production_ready": ready,
            "offline_prebuild": prebuild.as_dict(),
            "warm_activation": activation.as_dict(),
            "warm_selected_backend": activation_result.selected_backend,
            "oracle_cost_error": activation_result.oracle_cost_error,
            "online_synchronous_dstar_build": manager.synchronous_dstar_build_count,
            "files": file_records,
        }
        manager.clear()
        if not ready:
            raise RuntimeError("cache route failed production-ready verification")
        _atomic_json(self._entry_path(digest), entry)
        inventory = self.inventory()
        global_manifest = dict(self.validate_global_manifest())
        global_manifest.update({
            "production_ready": bool(inventory) and all(item["valid"] for item in inventory),
            "entry_count": len(inventory),
            "inventory_hash": _stable_hash(inventory),
        })
        _atomic_json(self.manifest_path, global_manifest)
        return entry

    def _verify_entry_files(self, entry: Mapping[str, Any]) -> Tuple[bool, str]:
        if entry.get("schema_version") != CACHE_BUNDLE_SCHEMA:
            return False, "ENTRY_SCHEMA_MISMATCH"
        binding = entry.get("binding") or {}
        if entry.get("binding_hash") != _stable_hash(binding):
            return False, "ENTRY_BINDING_HASH_MISMATCH"
        if entry.get("production_ready") is not True:
            return False, "ENTRY_NOT_PRODUCTION_READY"
        for record in entry.get("files") or ():
            path = (self.root / str(record.get("path", ""))).resolve()
            try:
                path.relative_to(self.root)
            except ValueError:
                return False, "ENTRY_PATH_ESCAPES_BUNDLE"
            if not path.is_file():
                return False, "ENTRY_FILE_MISSING"
            if path.stat().st_size != int(record.get("bytes", -1)):
                return False, "ENTRY_FILE_SIZE_MISMATCH"
            if sha256_file(path) != record.get("sha256"):
                return False, "ENTRY_FILE_HASH_MISMATCH"
        return True, ""

    def verify_route(
        self,
        plan: L1Plan,
        *,
        start_pose: Sequence[float],
        goal_pose: Sequence[float],
        dynamic_baseline_version: str = DEFAULT_DYNAMIC_BASELINE,
        deep: bool = False,
    ) -> CacheRouteStatus:
        self.validate_global_manifest()
        fields = route_binding(
            plan,
            start_pose=start_pose,
            goal_pose=goal_pose,
            dynamic_baseline_version=dynamic_baseline_version,
        )
        digest = _stable_hash(fields)
        path = self._entry_path(digest)
        if not path.is_file():
            return CacheRouteStatus("MISS", "ROUTE_ENTRY_MISSING", digest, False)
        try:
            entry = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            return CacheRouteStatus("REJECTED", f"ENTRY_PARSE:{exc}", digest, False)
        if entry.get("binding") != fields:
            return CacheRouteStatus("REJECTED", "ENTRY_BINDING_FIELDS_MISMATCH", digest, False)
        valid, reason = self._verify_entry_files(entry)
        if not valid:
            return CacheRouteStatus("REJECTED", reason, digest, False)
        if not deep:
            return CacheRouteStatus("HIT_VERIFIED", "", digest, True)
        manager = R2L2StateLifecycleManager(self.cache_root, max_active_states=1)
        _planner, result, activation = manager.activate(
            self._roi(plan),
            dynamic_baseline_version=dynamic_baseline_version,
            verify_oracle=True,
        )
        accepted = bool(
            activation.geometry_cache.hit
            and activation.state_cache.hit
            and result.success
            and float(result.oracle_cost_error or 0.0) == 0.0
            and manager.synchronous_dstar_build_count == 0
        )
        manager.clear()
        return CacheRouteStatus(
            "HIT_VERIFIED" if accepted else "REJECTED",
            "" if accepted else "DEEP_R2_RESTORE_FAILED",
            digest,
            accepted,
            deep_verified=accepted,
            selected_backend=result.selected_backend,
            oracle_cost_error=result.oracle_cost_error,
        )

    def inventory(self) -> List[Dict[str, Any]]:
        if not self.entries_root.is_dir():
            return []
        result: List[Dict[str, Any]] = []
        for path in sorted(self.entries_root.glob("*.json")):
            try:
                entry = json.loads(path.read_text(encoding="utf-8"))
                valid, reason = self._verify_entry_files(entry)
                result.append({
                    "entry": path.name,
                    "binding_hash": str(entry.get("binding_hash", "")),
                    "route_signature": str((entry.get("binding") or {}).get("route_signature", "")),
                    "valid": valid,
                    "reject_reason": reason,
                    "production_ready": entry.get("production_ready") is True,
                    "resident_bytes": int((entry.get("offline_prebuild") or {}).get("resident_bytes", 0)),
                })
            except Exception as exc:
                result.append({
                    "entry": path.name,
                    "binding_hash": "",
                    "route_signature": "",
                    "valid": False,
                    "reject_reason": f"ENTRY_PARSE:{exc}",
                    "production_ready": False,
                    "resident_bytes": 0,
                })
        return result

    def purge_obsolete_candidates(self, expected_binding_hashes: Iterable[str]) -> List[Dict[str, Any]]:
        """Report only. This method deliberately never deletes cache data."""
        expected = {str(value) for value in expected_binding_hashes}
        return [
            {**item, "action": "REPORT_ONLY_NOT_DELETED"}
            for item in self.inventory()
            if item["binding_hash"] not in expected
        ]


def _write_report(path: Optional[Path], value: Any) -> None:
    rendered = yaml.safe_dump(value, sort_keys=False)
    if path is None:
        print(rendered, end="")
    else:
        path.resolve().parent.mkdir(parents=True, exist_ok=True)
        path.resolve().write_text(rendered, encoding="utf-8")


def prebuild_main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Offline cache prebuild for 3D-V1-r2-stable. Production runtime never "
            "builds D* synchronously and falls back to deterministic A* on miss/reject."
        )
    )
    parser.add_argument("--cache-bundle", type=Path, required=True)
    parser.add_argument("--plan-bundle", type=Path, action="append", required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_STABLE_CONFIG)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    bundle = StableCacheBundle(args.cache_bundle, config_path=args.config)
    rows = []
    for plan_path in args.plan_bundle:
        verified = load_plan_bundle(plan_path)
        entry = bundle.prebuild_route(
            verified.plan,
            start_pose=verified.start_pose,
            goal_pose=verified.goal_pose,
        )
        rows.append({
            "plan_bundle": str(verified.root),
            "binding_hash": entry["binding_hash"],
            "production_ready": entry["production_ready"],
            "offline_prebuild": entry["offline_prebuild"],
            "warm_activation": entry["warm_activation"],
        })
    _write_report(args.report, rows)


def verify_main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Verify/inventory a 3D-V1-r2-stable cache bundle; purge-obsolete is "
            "report-only and never deletes files."
        )
    )
    parser.add_argument("--cache-bundle", type=Path, required=True)
    parser.add_argument("--plan-bundle", type=Path, action="append", default=[])
    parser.add_argument("--config", type=Path, default=DEFAULT_STABLE_CONFIG)
    parser.add_argument("--deep", action="store_true")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--purge-obsolete-report", action="store_true")
    args = parser.parse_args()
    bundle = StableCacheBundle(args.cache_bundle, config_path=args.config)
    global_manifest = bundle.validate_global_manifest()
    routes = []
    expected: List[str] = []
    for plan_path in args.plan_bundle:
        verified = load_plan_bundle(plan_path)
        status = bundle.verify_route(
            verified.plan,
            start_pose=verified.start_pose,
            goal_pose=verified.goal_pose,
            deep=args.deep,
        )
        routes.append({"plan_bundle": str(verified.root), **status.as_dict()})
        expected.append(status.binding_hash)
    report: Dict[str, Any] = {
        "architecture_id": ARCHITECTURE_ID,
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "global_production_ready": global_manifest.get("production_ready") is True,
        "inventory": bundle.inventory(),
        "routes": routes,
    }
    if args.purge_obsolete_report:
        report["purge_obsolete_candidates"] = bundle.purge_obsolete_candidates(expected)
        report["purge_performed"] = False
    _write_report(args.report, report)


__all__ = [
    "CacheRouteStatus", "StableCacheBundle", "StableCacheBundleError",
    "prebuild_main", "route_binding", "verify_main",
]
