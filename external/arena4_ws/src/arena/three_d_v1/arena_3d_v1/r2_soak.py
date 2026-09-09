"""Long-running r2 soak using the frozen r1 workload and oracle harness.

The harness body is intentionally reused without editing r1. This module
injects only the r2 controller/lifecycle implementation, performs explicit
offline prebuild before the online loop, and rewrites generated metadata to
the r2 protocol.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, ClassVar, Iterable, Sequence

import yaml

from . import r1_soak as frozen_harness
from .l2_incremental import Cell, CorridorROI
from .r2_pipeline import Layered3DV1R2Controller
from .r2_stage_a import DEFAULT_FROZEN_CONFIG, _load_frozen_config, _sha256
from .r2_state_lifecycle import (
    ARCHITECTURE_ID,
    PROTOCOL_ID,
    REVISION_ID,
    CacheMissAStarPlanner,
    R2L2StateLifecycleManager,
    STATIC_MASK_SCHEMA,
)


DEFAULT_QUERIES = ("A2B-07", "A2B-11", "A2B-17")


class _SoakLifecycleManager(R2L2StateLifecycleManager):
    """Limit cache construction to the harness's explicit prebuild phase."""

    latest: ClassVar["_SoakLifecycleManager | None"] = None

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.offline_prebuild_phase = True
        self.prebuild_records: list[dict[str, Any]] = []
        type(self).latest = self

    def activate(
        self,
        roi: CorridorROI,
        *,
        dynamic_baseline_version: str = "EMPTY_CONFIRMED_DYNAMIC_V1",
        blocked_global: Iterable[Cell] = (),
        verify_oracle: bool = False,
    ) -> tuple[Any, Any, Any]:
        planner, result, telemetry = super().activate(
            roi,
            dynamic_baseline_version=dynamic_baseline_version,
            blocked_global=blocked_global,
            verify_oracle=verify_oracle,
        )
        if isinstance(planner, CacheMissAStarPlanner) and self.offline_prebuild_phase:
            prebuild = self.prebuild(
                roi,
                dynamic_baseline_version=dynamic_baseline_version,
                verify_oracle=verify_oracle,
            )
            self.prebuild_records.append({
                "binding_hash": roi.binding.digest,
                **prebuild.as_dict(),
            })
            planner, result, telemetry = super().activate(
                roi,
                dynamic_baseline_version=dynamic_baseline_version,
                blocked_global=blocked_global,
                verify_oracle=verify_oracle,
            )
        return planner, result, telemetry

    def clear(self) -> dict[str, Any]:
        # The frozen harness calls clear once between offline prebuild and the
        # online snapshot loop. Later clears keep this flag false.
        self.offline_prebuild_phase = False
        return super().clear()


def _patch_artifacts(output: Path, frozen_config: Path) -> None:
    manager = _SoakLifecycleManager.latest
    if manager is None:
        raise RuntimeError("r2 soak lifecycle manager was not created")
    manifest_path = output / "manifest.yaml"
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    manifest.update({
        "architecture_id": ARCHITECTURE_ID,
        "revision_id": REVISION_ID,
        "protocol_id": PROTOCOL_ID,
        "harness_parent": "3D-V1-r1 frozen workload/oracle soak harness",
        "online_cache_policy": "verified-cache-only; miss-to-deterministic-grid-astar",
        "static_mask_schema": STATIC_MASK_SCHEMA,
        "synchronous_online_dstar_build_count": manager.synchronous_dstar_build_count,
        "frozen_config": str(frozen_config.resolve()),
        "frozen_config_sha256": _sha256(frozen_config),
    })
    manifest_path.write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8",
    )
    (output / "r2_prebuild_telemetry.yaml").write_text(
        yaml.safe_dump({
            "phase": "offline_before_online_soak",
            "records": manager.prebuild_records,
            "online_synchronous_dstar_build_count": manager.synchronous_dstar_build_count,
            "lifecycle_final": dict(manager.telemetry()),
        }, sort_keys=False),
        encoding="utf-8",
    )
    verification_path = output / "verification.yaml"
    verification = yaml.safe_load(verification_path.read_text(encoding="utf-8")) or {}
    verification.update({
        "r2_verified_cache_only": True,
        "online_synchronous_dstar_build_zero": manager.synchronous_dstar_build_count == 0,
        "packed_static_mask": True,
    })
    verification["soak_pass"] = bool(
        verification.get("soak_pass")
        and verification["online_synchronous_dstar_build_zero"]
    )
    verification_path.write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    report_path = output / "final_report.md"
    report = report_path.read_text(encoding="utf-8").replace(
        "# 3D-V1-r1 high-dynamic soak", "# 3D-V1-r2 high-dynamic soak",
    )
    report += (
        "\n- lifecycle: verified-cache-only online admission; cache miss falls back to deterministic grid A*\n"
        f"- static mask: `{STATIC_MASK_SCHEMA}`\n"
        f"- synchronous online D* builds: `{manager.synchronous_dstar_build_count}`\n"
    )
    report_path.write_text(report, encoding="utf-8")
    reproduction = output / "reproduction_command.txt"
    reproduction.write_text(
        reproduction.read_text(encoding="utf-8").replace(
            "-m arena_3d_v1.r1_soak", "-m arena_3d_v1.r2_soak",
        ),
        encoding="utf-8",
    )


def run(
    output: Path,
    *,
    query_ids: Sequence[str] = DEFAULT_QUERIES,
    min_snapshots: int = 5_000,
    max_snapshots: int = 20_000,
    max_duration_s: float = 7_200.0,
    route_switch_interval: int = 500,
    oracle_sample_interval: int = 100,
    frozen_config_path: Path = DEFAULT_FROZEN_CONFIG,
) -> Path:
    previous = {
        "manager": frozen_harness.L2StateLifecycleManager,
        "controller": frozen_harness.Layered3DV1R1Controller,
        "architecture": frozen_harness.ARCHITECTURE_ID,
        "revision": frozen_harness.REVISION_ID,
        "protocol": frozen_harness.PROTOCOL_ID,
        "load_frozen_config": frozen_harness._load_frozen_config,
    }
    frozen_harness.L2StateLifecycleManager = _SoakLifecycleManager
    frozen_harness.Layered3DV1R1Controller = Layered3DV1R2Controller
    frozen_harness.ARCHITECTURE_ID = ARCHITECTURE_ID
    frozen_harness.REVISION_ID = REVISION_ID
    frozen_harness.PROTOCOL_ID = PROTOCOL_ID
    frozen_harness._load_frozen_config = _load_frozen_config
    try:
        result = frozen_harness.run(
            output,
            query_ids=query_ids,
            min_snapshots=min_snapshots,
            max_snapshots=max_snapshots,
            max_duration_s=max_duration_s,
            route_switch_interval=route_switch_interval,
            oracle_sample_interval=oracle_sample_interval,
            frozen_config_path=frozen_config_path,
        )
        _patch_artifacts(result, frozen_config_path)
        return result
    finally:
        frozen_harness.L2StateLifecycleManager = previous["manager"]
        frozen_harness.Layered3DV1R1Controller = previous["controller"]
        frozen_harness.ARCHITECTURE_ID = previous["architecture"]
        frozen_harness.REVISION_ID = previous["revision"]
        frozen_harness.PROTOCOL_ID = previous["protocol"]
        frozen_harness._load_frozen_config = previous["load_frozen_config"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--query-ids", default=",".join(DEFAULT_QUERIES))
    parser.add_argument("--min-snapshots", type=int, default=5_000)
    parser.add_argument("--max-snapshots", type=int, default=20_000)
    parser.add_argument("--max-duration-s", type=float, default=7_200.0)
    parser.add_argument("--route-switch-interval", type=int, default=500)
    parser.add_argument("--oracle-sample-interval", type=int, default=100)
    parser.add_argument("--frozen-config", type=Path, default=DEFAULT_FROZEN_CONFIG)
    args = parser.parse_args()
    query_ids = tuple(item.strip() for item in args.query_ids.split(",") if item.strip())
    try:
        run(
            args.output_dir,
            query_ids=query_ids,
            min_snapshots=args.min_snapshots,
            max_snapshots=args.max_snapshots,
            max_duration_s=args.max_duration_s,
            route_switch_interval=args.route_switch_interval,
            oracle_sample_interval=args.oracle_sample_interval,
            frozen_config_path=args.frozen_config,
        )
    except Exception as exc:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "INTERRUPTED_RUN.md").write_text(
            f"# Interrupted run\n\n`{type(exc).__name__}: {exc}`\n",
            encoding="utf-8",
        )
        (args.output_dir / "stderr.log").write_text(
            f"{type(exc).__name__}: {exc}\n", encoding="utf-8",
        )
        raise


if __name__ == "__main__":
    main()
