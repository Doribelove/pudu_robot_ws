"""Offline parking-aisle calibration for the 2A-V3 r17 research arm."""
from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path
import platform
import resource
import shutil
import time
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from . import semantic_applicability_v3 as applicability
from . import two_layer_v3_semantic_r13_benchmark as r13
from . import two_layer_v3_semantic_r14_benchmark as r14
from . import two_layer_v3_semantic_r14_expanded as expanded
from . import two_layer_v3_semantic_r15_fast as r15
from .path_audit import PathAuditor
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import sha256_file
from .semantic_parking_aisle_r17 import (
    ParkingAislePolicyR17,
    build_route_local_aisle_field,
)
from .semantic_parking_reference_v3 import ContinuousParkingReferenceBuilder
from .semantic_query_defaults import load_query_set
from .semantic_route_phase_compact_r14 import CompactRoutePhaseWorldR14
from .semantic_route_phase_v3 import LazyRoutePhaseSearch, OrientedRoute
from .semantic_transition_contract import audit_transition_samples, resample_path


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r17-parking-aisle-medial-reference"
PROTOCOL_ID = "PLN-02-2A-V3-R17-PARKING-AISLE-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R17-RESULT-V1"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config/two_layer_v3_semantic_r17_parking_aisle.yaml"


def identity() -> dict[str, str]:
    return {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
    }


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _load(config_path: Path = DEFAULT_CONFIG):
    resolved = Path(config_path).resolve()
    config = yaml.safe_load(resolved.read_text(encoding="utf-8")) or {}
    for key, value in identity().items():
        if config.get(key) != value:
            raise ValueError(f"r17 identity mismatch for {key}")
    parent_r16_path = (resolved.parent / config["parent_r16"]["path"]).resolve()
    if sha256_file(parent_r16_path) != str(config["parent_r16"]["sha256"]):
        raise ValueError("r17 parent r16 hash mismatch")
    parent_r16 = yaml.safe_load(parent_r16_path.read_text(encoding="utf-8")) or {}
    parent_r15_path = (
        parent_r16_path.parent / parent_r16["parent_r15"]["path"]
    ).resolve()
    if sha256_file(parent_r15_path) != str(parent_r16["parent_r15"]["sha256"]):
        raise ValueError("r17 transitive r15 hash mismatch")
    parent = r15._load(parent_r15_path)
    r15_config, algorithm, parent_algorithm, r1_parent = parent[:4]
    frozen = config["frozen_bindings"]
    actual = {
        "map_hash": r15_config["frozen_bindings"]["map_hash"],
        "semantic_map_hash": r15_config["frozen_bindings"]["semantic_map_hash"],
        "expanded32_query_hash": r15_config["frozen_bindings"]["expanded32_query_hash"],
    }
    for key, value in actual.items():
        if str(frozen[key]) != str(value):
            raise ValueError(f"r17 frozen binding drift for {key}")
    return (
        config,
        r15_config,
        algorithm,
        parent_algorithm,
        r1_parent,
        parent_r16_path,
        parent_r15_path,
        resolved,
    )


def _aisle_policy(config: Mapping[str, Any]) -> ParkingAislePolicyR17:
    values = dict(config["parking_aisle_metric"])
    values.pop("metric_revision")
    values.pop("relation_to_r2")
    return ParkingAislePolicyR17(**values)


def _audit_field(world, points, parking_deviation: np.ndarray) -> dict[str, Any]:
    sampled_points, station = resample_path(points)
    samples = np.asarray(
        [[point[key] for key in ("x", "y", "yaw")] for point in sampled_points],
        dtype=np.float64,
    )
    rows, cols, inside = world.cells(samples)
    if not np.all(inside):
        return {
            "semantic_gate_passed": False,
            "contract_metric_status": "INVALID_CONTRACT_METRIC",
            "failure_reason": "PATH_OUTSIDE_INPUT_CROP",
        }
    lane = np.isin(world.grids["labels"][rows, cols], world.selected_lanes)
    parking = np.isin(
        world.grids["parking_components"][rows, cols], world.selected_parking
    )
    return audit_transition_samples(
        path_length_m=float(station[-1]),
        station_m=station,
        lane_mask=lane,
        lane_error_m=world.grids["error"][rows, cols],
        lane_correct_side=world.grids["correct"][rows, cols],
        parking_mask=parking,
        parking_normalized_deviation=parking_deviation[rows, cols],
        raw_xy=samples[:, :2],
    )


def _report(rows: Sequence[Mapping[str, Any]], gates: Mapping[str, Any]) -> str:
    lines = [
        "# 2A-V3 r17 parking aisle calibration",
        "",
        "R17 does not rewrite the frozen R2 component metric. It evaluates a separately named route-local aisle metric and reports R2 compatibility beside it.",
        "",
        "| query | field target | reference target | aisle path band | aisle P50 | R2 component band | R2 component P50 | witness | failure |",
        "|---|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for row in rows:
        def value(key: str) -> str:
            item = row.get(key)
            return "N/A" if item is None else f"{float(item):.3f}"
        lines.append(
            "| {query} | {field} | {reference} | {local_band} | {local_p50} | {r2_band} | {r2_p50} | {witness} | {failure} |".format(
                query=row["query_id"],
                field=value("field_target_station_ratio"),
                reference=value("reference_target_ratio"),
                local_band=value("aisle_center_band_ratio"),
                local_p50=value("aisle_deviation_p50"),
                r2_band=value("r2_component_center_band_ratio"),
                r2_p50=value("r2_component_deviation_p50"),
                witness=str(bool(row.get("gate_passed"))).lower(),
                failure=row.get("failure_code") or "",
            )
        )
    lines.extend(
        [
            "",
            "## Gate",
            "",
            f"- offline three-query gate: `{str(gates['offline_three_query_gate_passed']).lower()}`",
            f"- online ROS allowed next: `{str(gates['online_targeted_allowed']).lower()}`",
            "- selected8/expanded32 remain blocked until the online targeted gate.",
            "- r13 B research acceptance remains unchanged; this calibration cannot grant production promotion.",
            "",
        ]
    )
    return "\n".join(lines)


def run(*, output: Path, config_path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "witnesses").mkdir()
    started = time.monotonic()
    (
        config,
        r15_config,
        algorithm,
        parent_algorithm,
        parent,
        parent_r16_path,
        parent_r15_path,
        resolved_config,
    ) = _load(config_path)
    query_path = (
        resolved_config.parent / r15_config["frozen_bindings"]["expanded32_path"]
    ).resolve()
    queries, _intents, query_meta = load_query_set(
        query_path,
        actual_map_hash=config["frozen_bindings"]["map_hash"],
        actual_semantic_map_hash=config["frozen_bindings"]["semantic_map_hash"],
    )
    wanted = list(config["calibration"]["query_ids"])
    by_id = {query.query_id: query for query in queries}
    if any(query_id not in by_id for query_id in wanted):
        raise ValueError("r17 calibration contains unknown frozen query")
    sources = [
        Path(__file__).resolve(),
        Path(__file__).with_name("semantic_parking_aisle_r17.py"),
        Path(__file__).with_name("semantic_route_phase_v3.py"),
        resolved_config,
        parent_r16_path,
        parent_r15_path,
        query_path,
    ]
    hashes = {str(path): sha256_file(path) for path in sources}
    protocol = {
        **identity(),
        "schema_version": SCHEMA_VERSION,
        "configuration": config,
        "query_hash": query_meta["query_hash"],
        "source_hashes_at_start": hashes,
        "root_git": r15._git_capture(PACKAGE_ROOT.parents[5]),
        "evaluation_git": r15._git_capture(PACKAGE_ROOT.parent),
        "processes_before": applicability._process_audit(),
        "python": platform.python_version(),
    }
    _write_json(output / "protocol.json", protocol)
    (output / "reproduction_command.txt").write_text(
        f"/usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r17_parking_aisle --output {output} --config {resolved_config}\n",
        encoding="utf-8",
    )
    snapshot = output / "source_snapshot"
    snapshot.mkdir()
    for index, source in enumerate(sources):
        shutil.copy2(source, snapshot / f"{index:02d}_{source.name}")

    prepared, static_prepare_ms = r15._prepare_static(
        algorithm=algorithm, parent=parent, output=output
    )
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    builder = expanded.AuditOnlyPreferenceBuilderR14(
        ctx.hospital_map,
        raster,
        policy=parent["regional_preference"],
        semantic_map=semantic_map,
    )
    composer = SemanticCostmapComposerR2(
        policy=parent["l3_soft_cost"], inflation_cache_capacity=2
    )
    selector = r15._selector(parent, topology, router)
    aisle_policy = _aisle_policy(config)
    reference_builder = ContinuousParkingReferenceBuilder(
        r14.ParkingReferencePolicy(**dict(config["parking_reference"]))
    )
    base_se2_policy = r13._policy(parent_algorithm)
    connection = config["se2_local_connection"]
    se2_policy = replace(
        base_se2_policy,
        maximum_station_skip=int(connection["maximum_station_skip"]),
        maximum_local_edge_length_m=float(connection["maximum_local_edge_length_m"]),
        maximum_local_edge_ratio=float(connection["maximum_local_edge_ratio"]),
    )
    canonical_auditor = PathAuditor(ctx, source_commit="2A-V3-r17-offline")
    rows = []
    for query_id in wanted:
        query_started = time.monotonic()
        query = by_id[query_id]
        row: dict[str, Any] = {
            **identity(), "query_id": query_id, "category": query.category
        }
        try:
            (
                _route,
                orientation,
                _composition,
                _publication,
                full_allowed,
                metadata,
                arrays,
            ) = r15._prepare_query_state(
                query=query,
                query_hash=query_meta["query_hash"],
                selector=selector,
                ctx=ctx,
                semantic_map=semantic_map,
                raster=raster,
                topology=topology,
                route_phase_algorithm=parent_algorithm,
                parent=parent,
                builder=builder,
                composer=composer,
            )
            world = CompactRoutePhaseWorldR14(
                output,
                query_id,
                arrays_override=arrays,
                meta_override=metadata,
                full_occupancy=ctx.hospital_map.occupancy,
                full_allowed=full_allowed,
            )
            world._canonical_auditor = canonical_auditor
            route = OrientedRoute(
                metadata["route_polyline"],
                world.start,
                world.goal,
                endpoint_attachment_limit_m=base_se2_policy.endpoint_attachment_limit_m,
            )
            original_component_deviation = world.grids["parking_deviation"].copy()
            field = build_route_local_aisle_field(world, route, aisle_policy)
            row["field"] = field.summary()
            row["field_target_station_ratio"] = field.diagnostics[
                "active_target_available_station_ratio"
            ]
            if not field.gate_passed:
                raise RuntimeError(field.failure_code)
            world.grids["parking_deviation"] = field.deviation
            selected_parking = np.isin(
                world.grids["parking_components"], np.asarray(world.selected_parking)
            )
            world.grids["allowed"][
                selected_parking & ~np.isfinite(field.deviation)
            ] = False
            reference = reference_builder.build(world, route)
            row["reference"] = reference.to_dict()
            row["reference_target_ratio"] = reference.target_sample_ratio
            if (
                not reference.gate_passed
                or reference.target_sample_ratio
                <= float(config["gates"]["reference_target_ratio_min_exclusive"])
            ):
                raise RuntimeError(
                    reference.failure_code or "PARKING_AISLE_REFERENCE_GATE_FAILED"
                )
            guide = OrientedRoute(
                reference.polyline,
                world.start,
                world.goal,
                endpoint_attachment_limit_m=base_se2_policy.endpoint_attachment_limit_m,
            )
            search_started = time.monotonic()
            searcher = LazyRoutePhaseSearch(world, guide, se2_policy)
            witness, evaluations, search = searcher.search(
                semantic_map,
                allow_safe_soft_fallback=False,
                prefer_lazy=True,
                fallback_first=False,
            )
            row["search_ms"] = (time.monotonic() - search_started) * 1000.0
            row["search"] = search
            row["candidate_evaluation_count"] = len(evaluations)
            _write_json(
                output / f"{query_id}_candidate_evaluations.json",
                expanded._json_safe(evaluations),
            )
            candidate = witness or searcher.best_failed
            if candidate is None:
                raise RuntimeError("NO_ROUTE_PHASE_SE2_CANDIDATE")
            points = candidate["points"]
            r2_compatibility = _audit_field(
                world, points, original_component_deviation
            )
            row["r2_component_compatibility"] = r2_compatibility
            local_parking = candidate["semantics"]["active_window"]["classes"][
                "parking"
            ]
            r2_parking = (
                r2_compatibility.get("active_window", {})
                .get("classes", {})
                .get("parking", {})
            )
            row.update(
                {
                    "aisle_center_band_ratio": local_parking.get(
                        "parking_center_band_ratio"
                    ),
                    "aisle_deviation_p50": local_parking.get(
                        "parking_center_normalized_deviation_p50"
                    ),
                    "r2_component_center_band_ratio": r2_parking.get(
                        "parking_center_band_ratio"
                    ),
                    "r2_component_deviation_p50": r2_parking.get(
                        "parking_center_normalized_deviation_p50"
                    ),
                    "maximum_curvature_1pm": candidate.get(
                        "maximum_control_curvature_1pm"
                    ),
                    "revisit_passed": candidate.get("revisit", {}).get(
                        "revisit_screen_passed"
                    ),
                    "ordered_progress_passed": candidate.get(
                        "ordered_progress", {}
                    ).get("ordered_progress_gate_passed"),
                    "trace_replay_exact": candidate.get("trace_replay_exact"),
                    "gate_passed": witness is not None,
                    "failure_code": "" if witness is not None else "SE2_WITNESS_GATE_FAILED",
                }
            )
            stored = dict(candidate)
            _write_json(
                output / "witnesses" / f"{query_id}_path.json",
                stored.pop("points"),
            )
            _write_json(
                output / "witnesses" / f"{query_id}_controls.json",
                {"edges": stored.pop("controls")},
            )
            stored["semantic_metric_revision"] = config["parking_aisle_metric"][
                "metric_revision"
            ]
            stored["r2_component_compatibility"] = r2_compatibility
            _write_json(
                output / "witnesses" / f"{query_id}_audit.json", stored
            )
        except (RuntimeError, ValueError) as error:
            row.setdefault("gate_passed", False)
            row.setdefault("failure_code", type(error).__name__)
            row["failure_detail"] = str(error)
        row["wall_ms"] = (time.monotonic() - query_started) * 1000.0
        row["peak_rss_bytes"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
        rows.append(row)
        _write_json(output / f"result_{query_id}.json", expanded._json_safe(row))
        if (
            config["calibration"]["early_stop_on_first_failure"]
            and row.get("gate_passed") is not True
        ):
            break

    all_three = len(rows) == len(wanted) and all(
        row.get("gate_passed") is True for row in rows
    )
    gates = {
        "offline_three_query_gate_passed": all_three,
        "required_query_count": len(wanted),
        "executed_query_count": len(rows),
        "online_targeted_allowed": all_three,
        "selected8_allowed": False,
        "expanded32_allowed": False,
        "r13_b_research_acceptance_preserved": True,
        "production_promotion_allowed": False,
        "hard_stop_triggered": not all_three,
    }
    flat = [
        {
            "query_id": row["query_id"],
            "gate_passed": row.get("gate_passed"),
            "failure_code": row.get("failure_code"),
            "field_target_station_ratio": row.get("field_target_station_ratio"),
            "reference_target_ratio": row.get("reference_target_ratio"),
            "aisle_center_band_ratio": row.get("aisle_center_band_ratio"),
            "aisle_deviation_p50": row.get("aisle_deviation_p50"),
            "r2_component_center_band_ratio": row.get(
                "r2_component_center_band_ratio"
            ),
            "r2_component_deviation_p50": row.get(
                "r2_component_deviation_p50"
            ),
            "maximum_curvature_1pm": row.get("maximum_curvature_1pm"),
            "revisit_passed": row.get("revisit_passed"),
            "ordered_progress_passed": row.get("ordered_progress_passed"),
            "trace_replay_exact": row.get("trace_replay_exact"),
            "search_ms": row.get("search_ms"),
            "wall_ms": row.get("wall_ms"),
            "peak_rss_bytes": row.get("peak_rss_bytes"),
        }
        for row in rows
    ]
    with (output / "parking_aisle_results.csv").open(
        "x", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    _write_json(output / "gate_results.json", gates)
    final = {
        **identity(),
        "schema_version": SCHEMA_VERSION,
        "gate_results": gates,
        "rows": rows,
        "static_prepare_ms": static_prepare_ms,
        "wall_s": time.monotonic() - started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "processes_after": applicability._process_audit(),
    }
    _write_json(output / "final_result.json", expanded._json_safe(final))
    (output / "r17_calibration_report.md").write_text(
        _report(rows, gates), encoding="utf-8"
    )
    _write_json(output / "artifact_hashes.json", applicability._manifest_files(output))
    return final


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    result = run(output=args.output, config_path=args.config)
    print(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False))
    return 0 if result["gate_results"]["offline_three_query_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ARCHITECTURE_ID",
    "IMPLEMENTATION_REVISION",
    "PROTOCOL_ID",
    "identity",
    "run",
    "main",
]
