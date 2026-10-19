"""Default-reference audit and release validation for 3D-V1-r2-stable."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence

import yaml

from .production_runtime import controller_class, production_selection, resolve_revision
from .stable_contract import (
    DEFAULT_STABLE_CONFIG,
    PRODUCTION_BASELINE_ID,
    load_stable_config,
    stable_contract,
)
from .stable_pipeline import Layered3DV1StableController


ROOT = Path("/home/robot/pudu_robot_ws")
TEXT_SUFFIXES = {".py", ".yaml", ".yml", ".md", ".xml", ".cfg", ".txt", ".sh"}
PRUNE_NAMES = {".git", "build", "install", "log", "__pycache__", ".pytest_cache", "private_data"}


def _candidate_files(root: Path) -> Iterable[Path]:
    for path in root.rglob("*"):
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        relative = path.relative_to(root)
        if any(part in PRUNE_NAMES for part in relative.parts):
            continue
        if relative.parts and relative.parts[0] == "experiments":
            continue
        yield path


def _classification(path: Path, line: str) -> str:
    value = str(path)
    name = path.name.lower()
    lowered = line.lower()
    if "/artifacts/3d_v1_" in value:
        return "historical_evidence"
    if name in {
        "pln-02_3d_v1_r0_design_implementation_report.md",
        "pln-02_3d_v1_r1_l2_lifecycle_final_report.md",
        "pln-02_3d_v1_r2_production_acceptance_final_report.md",
    }:
        return "historical_evidence"
    if name.startswith(("r1_", "r2_", "test_r1_", "test_r2_")):
        return "legacy_revision_source"
    if name in {
        "pipeline.py", "l2_incremental.py", "dynamic_policy.py", "production_l1.py",
        "stage_a_benchmark.py", "real_stage_a_benchmark.py", "stage_b_smoke.py",
        "three_d_v1_r0.yaml", "three_d_v1_r1_l2_lifecycle.yaml",
        "three_d_v1_r2_production_acceptance.yaml",
    }:
        return "legacy_revision_source"
    if name == "setup.py" and any(token in lowered for token in (
        "three_d_v1_stage_a =", "three_d_v1_real_stage_a =", "three_d_v1_stage_b_smoke =",
    )):
        return "legacy_compatibility_alias"
    if name == "setup.py" and "three_d_v1_" in lowered and "r" in lowered:
        return "explicit_legacy_cli"
    if any(token in name for token in ("stable", "production_runtime", "production_cache", "production_cli", "production_validation")):
        return "stable_default_or_binding"
    if name == "__init__.py":
        return "stable_default_or_legacy_internal_constant"
    if "architecture_2a_v2" in name:
        return "other_architecture_cross_reference"
    return "reviewed_nondefault_reference"


def default_reference_audit(root: Path = ROOT) -> List[Dict[str, Any]]:
    markers = ("3d-v1", "3d_v1", "three_d_v1")
    rows: List[Dict[str, Any]] = []
    for path in sorted(_candidate_files(Path(root).resolve())):
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError):
            continue
        for number, line in enumerate(lines, 1):
            lowered = line.lower()
            if not any(marker in lowered for marker in markers):
                continue
            classification = _classification(path, line)
            legacy = any(token in lowered for token in (
                "r0-production", "r1-l2", "r2-production-acceptance-real-replay",
                "three_d_v1_stage_a =", "three_d_v1_real_stage_a =",
                "three_d_v1_stage_b_smoke =",
            ))
            default_claim = "default" in lowered or "production_baseline" in lowered
            violation = bool(
                legacy and default_claim and classification in {
                    "reviewed_nondefault_reference", "other_architecture_cross_reference"
                }
            )
            rows.append({
                "path": str(path),
                "line": number,
                "reference": line.strip(),
                "classification": classification,
                "default_violation": violation,
            })
    return rows


def write_reference_audit(output: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    with (output / "default_reference_audit.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["path", "line", "reference", "classification", "default_violation"],
        )
        writer.writeheader()
        writer.writerows(rows)
    (output / "default_reference_audit.json").write_text(
        json.dumps(list(rows), indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )


def validate_release(output: Path, *, root: Path = ROOT) -> Mapping[str, Any]:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty validation output: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config = load_stable_config()
    rows = default_reference_audit(root)
    write_reference_audit(output, rows)
    setup_text = (
        root / "external/arena4_ws/src/arena/three_d_v1/setup.py"
    ).read_text(encoding="utf-8")
    forbidden_loaded = sorted(
        name for name in sys.modules
        if name.startswith("arena_3d_v1.")
        and any(token in name for token in ("stage_a", "stage_b", "profile", "soak", "calibration"))
    )
    checks = {
        "default_resolves_stable": resolve_revision().resolved == PRODUCTION_BASELINE_ID,
        "default_controller_class_is_stable": controller_class() is Layered3DV1StableController,
        "stable_config_valid": config["production_baseline_id"] == PRODUCTION_BASELINE_ID,
        "stable_plan_cli_present": "three_d_v1_plan = arena_3d_v1.production_cli:main" in setup_text,
        "stable_cache_prebuild_cli_present": (
            "three_d_v1_cache_prebuild = arena_3d_v1.production_cache:prebuild_main" in setup_text
        ),
        "stable_cache_verify_cli_present": (
            "three_d_v1_cache_verify = arena_3d_v1.production_cache:verify_main" in setup_text
        ),
        "stable_validate_release_cli_present": (
            "three_d_v1_validate_release = arena_3d_v1.production_validation:main" in setup_text
        ),
        "reference_audit_has_no_default_violation": not any(row["default_violation"] for row in rows),
        "production_import_loaded_no_research_runner": not forbidden_loaded,
        "pure_dstar_disabled": stable_contract()["pure_dstar_production_mode"] is False,
        "online_synchronous_build_disabled": (
            stable_contract()["online_synchronous_dstar_build"] is False
        ),
    }
    verification = {
        **stable_contract(),
        **production_selection(),
        "reference_count": len(rows),
        "forbidden_research_modules_loaded": forbidden_loaded,
        "checks": checks,
        "release_validation_pass": all(checks.values()),
    }
    (output / "verification.yaml").write_text(
        yaml.safe_dump(verification, sort_keys=False), encoding="utf-8",
    )
    (output / "manifest.yaml").write_text(
        yaml.safe_dump({
            **stable_contract(),
            "stable_config": str(DEFAULT_STABLE_CONFIG),
            "stable_config_sha256": config["_config_sha256"],
            "reference_audit_csv": "default_reference_audit.csv",
            "reference_audit_json": "default_reference_audit.json",
        }, sort_keys=False),
        encoding="utf-8",
    )
    (output / "reproduction_command.txt").write_text(
        "cd /home/robot/pudu_robot_ws\n"
        "PYTHONPATH=/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/three_d_v1:"
        "/home/robot/pudu_robot_ws/external/arena4_ws/src/arena/evaluation/arena_evaluation "
        f"/usr/bin/python3 -m arena_3d_v1.production_validation --output-dir {output}\n",
        encoding="utf-8",
    )
    (output / "stdout.log").write_text(
        f"release_validation_pass={verification['release_validation_pass']} references={len(rows)}\n",
        encoding="utf-8",
    )
    (output / "stderr.log").write_text("", encoding="utf-8")
    return verification


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            f"Validate {PRODUCTION_BASELINE_ID} default selection. Stable fails closed "
            "for legacy/pure-D* requests and never builds D* online."
        )
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, default=ROOT)
    args = parser.parse_args()
    verification = validate_release(args.output_dir, root=args.workspace)
    if not verification["release_validation_pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
