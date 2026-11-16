"""Frozen identifiers and configuration validation for the 3D-V1 stable runtime."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import yaml


ARCHITECTURE_ID = "3D-V1"
PRODUCTION_BASELINE_ID = "3D-V1-r2-stable"
SOURCE_REVISION = "r2-production-acceptance"
RELEASE_CANDIDATE_ID = "3D-V1-r2-stable-rc1"
PROTOCOL_ID = "PLN-02-3D-V1-R2-STABLE-PROMOTION-V1"
CACHE_BUNDLE_SCHEMA = "3D-V1-r2-stable-cache-bundle-v1"
PLAN_BUNDLE_SCHEMA = "3D-V1-r2-stable-l1-plan-v1"

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_STABLE_CONFIG = PACKAGE_ROOT / "config/three_d_v1_stable.yaml"


class StableConfigurationError(ValueError):
    """The stable configuration does not match the compiled stable contract."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require(mapping: Mapping[str, Any], key: str, expected: Any) -> None:
    actual = mapping.get(key)
    if actual != expected:
        raise StableConfigurationError(
            f"stable config mismatch for {key}: expected {expected!r}, got {actual!r}"
        )


def load_stable_config(path: Optional[Path] = None) -> Dict[str, Any]:
    """Load and fail closed on any stable/source/contract mismatch."""
    config_path = Path(path or DEFAULT_STABLE_CONFIG).resolve()
    if not config_path.is_file():
        raise StableConfigurationError(f"stable config missing: {config_path}")
    value = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(value, dict):
        raise StableConfigurationError("stable config root must be a mapping")
    for key, expected in (
        ("architecture_id", ARCHITECTURE_ID),
        ("production_baseline_id", PRODUCTION_BASELINE_ID),
        ("source_revision", SOURCE_REVISION),
        ("release_candidate", RELEASE_CANDIDATE_ID),
        ("protocol_id", PROTOCOL_ID),
    ):
        _require(value, key, expected)

    forbidden = {"workload", "calibration", "heldout", "arms", "parameter_search"}
    present = sorted(forbidden.intersection(value))
    if present:
        raise StableConfigurationError(
            f"research-only sections are forbidden in stable config: {present}"
        )

    policy = value.get("policy") or {}
    runtime = value.get("runtime") or {}
    cache = value.get("cache") or {}
    vehicle = value.get("vehicle_contract") or {}
    _require(policy, "online_synchronous_dstar_build", False)
    _require(policy, "pure_dstar_production_mode", False)
    _require(policy, "cache_miss_backend", "deterministic_grid_astar")
    _require(policy, "partial_dstar_result_allowed", False)
    _require(runtime, "map_resolution_m", 0.05)
    _require(runtime, "corridor_profile", "topology_turn_adaptive_2m_4m")
    _require(runtime, "fixed_settle_cycles_after_ack", 0)
    _require(runtime, "server_effective_content_ack_required", True)
    _require(cache, "schema", CACHE_BUNDLE_SCHEMA)
    _require(cache, "max_active_states_default", 1)
    _require(cache, "max_active_states_hard", 2)
    _require(vehicle, "angle_quantization_bins", 48)
    _require(vehicle, "motion_model", "DUBIN")
    _require(vehicle, "allow_reverse", False)
    _require(vehicle, "allow_in_place_rotation", False)
    _require(vehicle, "minimum_turning_radius_m", 0.40)
    _require(vehicle, "maximum_curvature_1pm", 2.50)

    source_bindings = value.get("source_bindings") or {}
    package = Path(__file__).resolve().parent
    for filename, expected in source_bindings.items():
        source = package / str(filename)
        if not source.is_file():
            raise StableConfigurationError(f"bound r2 source missing: {source}")
        actual = sha256_file(source)
        if actual != str(expected):
            raise StableConfigurationError(
                f"bound r2 source changed: {filename}: expected {expected}, got {actual}"
            )
    value["_config_path"] = str(config_path)
    value["_config_sha256"] = sha256_file(config_path)
    return value


def stable_contract() -> Dict[str, Any]:
    """Small side-effect-free contract used by imports, CLIs, and telemetry."""
    return {
        "architecture_id": ARCHITECTURE_ID,
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "source_revision": SOURCE_REVISION,
        "release_candidate": RELEASE_CANDIDATE_ID,
        "protocol_id": PROTOCOL_ID,
        "default_revision": PRODUCTION_BASELINE_ID,
        "online_synchronous_dstar_build": False,
        "pure_dstar_production_mode": False,
        "cache_miss_backend": "deterministic_grid_astar",
    }


__all__ = [
    "ARCHITECTURE_ID", "CACHE_BUNDLE_SCHEMA", "DEFAULT_STABLE_CONFIG",
    "PLAN_BUNDLE_SCHEMA", "PRODUCTION_BASELINE_ID", "PROTOCOL_ID",
    "RELEASE_CANDIDATE_ID", "SOURCE_REVISION", "StableConfigurationError",
    "load_stable_config", "sha256_file", "stable_contract",
]
