"""Single revision resolver and factory for the 3D-V1 production runtime."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Type

from .pipeline import L1Plan
from .production_cache import StableCacheBundle, StableCacheBundleError
from .stable_contract import (
    DEFAULT_STABLE_CONFIG,
    PRODUCTION_BASELINE_ID,
    SOURCE_REVISION,
    load_stable_config,
)
from .stable_pipeline import Layered3DV1StableController


class RevisionSelectionError(ValueError):
    """A revision is unavailable in the requested runtime mode."""


@dataclass(frozen=True)
class RevisionResolution:
    requested: str
    resolved: str
    mode: str
    production_default: bool
    legacy_explicit_only: bool


_STABLE_ALIASES = {
    "", "default", "current", "stable", "r2-stable", PRODUCTION_BASELINE_ID.lower(),
}
_LEGACY_ALIASES = {
    "r0": "r0-production-substrate-l2-dstar-v1",
    "r0-production-substrate-l2-dstar-v1": "r0-production-substrate-l2-dstar-v1",
    "r1": "r1-l2-state-lifecycle-soak",
    "r1-l2-state-lifecycle-soak": "r1-l2-state-lifecycle-soak",
    "r2": "r2-production-acceptance-real-replay",
    "r2-production-acceptance": "r2-production-acceptance-real-replay",
    "r2-production-acceptance-real-replay": "r2-production-acceptance-real-replay",
}


def resolve_revision(
    revision: Optional[str] = None, *, mode: str = "production"
) -> RevisionResolution:
    normalized_mode = str(mode).strip().lower()
    if normalized_mode not in {"production", "benchmark", "reproduction"}:
        raise RevisionSelectionError(f"unknown runtime mode: {mode}")
    requested = "" if revision is None else str(revision).strip()
    key = requested.lower()
    if "pure" in key and "dstar" in key:
        raise RevisionSelectionError("pure D* has no 3D-V1 production or legacy runtime")
    if key in _STABLE_ALIASES:
        return RevisionResolution(
            requested=requested or "<unspecified>",
            resolved=PRODUCTION_BASELINE_ID,
            mode=normalized_mode,
            production_default=revision is None or not requested,
            legacy_explicit_only=False,
        )
    if key not in _LEGACY_ALIASES:
        raise RevisionSelectionError(f"unknown 3D-V1 revision: {requested}")
    resolved = _LEGACY_ALIASES[key]
    if normalized_mode == "production":
        raise RevisionSelectionError(
            f"{requested} is legacy/research-only; production accepts only "
            f"{PRODUCTION_BASELINE_ID}. Use mode='benchmark' or an explicit legacy CLI."
        )
    return RevisionResolution(
        requested=requested,
        resolved=resolved,
        mode=normalized_mode,
        production_default=False,
        legacy_explicit_only=True,
    )


def controller_class(
    revision: Optional[str] = None, *, mode: str = "production"
) -> Type[Any]:
    resolution = resolve_revision(revision, mode=mode)
    if resolution.resolved == PRODUCTION_BASELINE_ID:
        return Layered3DV1StableController
    if resolution.resolved == "r0-production-substrate-l2-dstar-v1":
        from .pipeline import Layered3DV1Controller
        return Layered3DV1Controller
    if resolution.resolved == "r1-l2-state-lifecycle-soak":
        from .r1_pipeline import Layered3DV1R1Controller
        return Layered3DV1R1Controller
    if resolution.resolved == "r2-production-acceptance-real-replay":
        from .r2_pipeline import Layered3DV1R2Controller
        return Layered3DV1R2Controller
    raise AssertionError(f"unhandled revision: {resolution.resolved}")


def _fallback_only_cache_root() -> Path:
    # R2 activation is read-only on a miss. /dev/null as a non-directory makes
    # every low-level geometry/state lookup fail closed without creating files.
    return Path("/dev/null/three_d_v1_stable_cache_miss")


def create_controller(
    initial_plan: L1Plan,
    *,
    revision: Optional[str] = None,
    mode: str = "production",
    cache_bundle: Optional[Path] = None,
    start_pose: Optional[Sequence[float]] = None,
    goal_pose: Optional[Sequence[float]] = None,
    config_path: Optional[Path] = None,
    verify_l2_oracle: bool = False,
    backend: Optional[str] = None,
    **legacy_kwargs: Any,
) -> Any:
    """Create stable by default; legacy implementations require benchmark mode."""
    resolution = resolve_revision(revision, mode=mode)
    if backend and "pure" in backend.lower() and "dstar" in backend.lower():
        raise RevisionSelectionError("stable production runtime rejects pure D*")
    if resolution.resolved != PRODUCTION_BASELINE_ID:
        cls = controller_class(revision, mode=mode)
        return cls(initial_plan, verify_l2_oracle=verify_l2_oracle, **legacy_kwargs)
    if legacy_kwargs:
        raise RevisionSelectionError(
            f"stable production factory rejects research parameter overrides: {sorted(legacy_kwargs)}"
        )

    config = load_stable_config(config_path or DEFAULT_STABLE_CONFIG)
    cache_status = "MISS:NO_STABLE_CACHE_BUNDLE"
    cache_root = _fallback_only_cache_root()
    if cache_bundle is not None:
        bundle = StableCacheBundle(cache_bundle, config_path=Path(config["_config_path"]))
        if bundle.manifest_path.is_file():
            bundle.validate_global_manifest()
            if start_pose is None or goal_pose is None:
                cache_status = "REJECTED:ENDPOINT_POSES_REQUIRED"
            else:
                status = bundle.verify_route(
                    initial_plan,
                    start_pose=start_pose,
                    goal_pose=goal_pose,
                    deep=False,
                )
                cache_status = (
                    "HIT_VERIFIED_STABLE_BUNDLE" if status.accepted
                    else f"{status.status}:{status.reason}"
                )
                if status.accepted:
                    cache_root = bundle.cache_root
        else:
            cache_status = "MISS:BUNDLE_MANIFEST_MISSING"

    policy: Mapping[str, Any] = config["policy"]
    cache: Mapping[str, Any] = config["cache"]
    controller = Layered3DV1StableController(
        initial_plan,
        cache_root=cache_root,
        declared_cache_status=cache_status,
        stable_config_sha256=str(config["_config_sha256"]),
        max_active_states=int(cache["max_active_states_default"]),
        dynamic_inflation_radius_cells=7,
        confidence_threshold=float(policy["confidence_threshold"]),
        dstar_wall_budget_ms=float(policy["dstar_wall_budget_ms"]),
        dstar_max_expansions=int(policy["dstar_max_expansions"]),
        dstar_attempt_max_changed_cells=int(policy["dstar_changed_source_eligibility_max"]),
        verify_l2_oracle=verify_l2_oracle,
    )
    if controller.lifecycle.synchronous_dstar_build_count != 0:
        raise AssertionError("stable runtime performed a synchronous D* build")
    return controller


def production_selection() -> Mapping[str, Any]:
    resolution = resolve_revision()
    return {
        "requested_revision": resolution.requested,
        "resolved_revision": resolution.resolved,
        "mode": resolution.mode,
        "production_baseline_id": PRODUCTION_BASELINE_ID,
        "source_revision": SOURCE_REVISION,
        "legacy_requires_explicit_benchmark_mode": True,
    }


__all__ = [
    "RevisionResolution", "RevisionSelectionError", "StableCacheBundleError",
    "controller_class", "create_controller", "production_selection", "resolve_revision",
]
