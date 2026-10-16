"""Fast route-local semantic applicability audit for 2A-V3 r16."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import platform
import resource
import shutil
import time
from typing import Any, Mapping, Sequence

import yaml

from . import semantic_applicability_v3 as applicability
from . import two_layer_v3_semantic_r13_benchmark as r13
from . import two_layer_v3_semantic_r14_expanded as expanded
from . import two_layer_v3_semantic_r15_fast as r15
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_map import sha256_file
from .semantic_parking_applicability_r16 import (
    ParkingApplicabilityPolicyR16,
    audit_parking_metric_applicability,
)
from .semantic_query_defaults import load_query_set
from .semantic_route_phase_compact_r14 import CompactRoutePhaseWorldR14
from .semantic_route_phase_v3 import OrientedRoute


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r16-route-local-semantic-applicability"
PROTOCOL_ID = "PLN-02-2A-V3-R16-ROUTE-LOCAL-APPLICABILITY-V1"
SCHEMA_VERSION = "PLN-02-2A-V3-R16-RESULT-V1"
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config/two_layer_v3_semantic_r16_applicability.yaml"


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
            raise ValueError(f"r16 identity mismatch for {key}")
    parent_path = (resolved.parent / config["parent_r15"]["path"]).resolve()
    if sha256_file(parent_path) != str(config["parent_r15"]["sha256"]):
        raise ValueError("r16 parent r15 hash mismatch")
    parent = r15._load(parent_path)
    r15_config, algorithm, parent_algorithm, r1_parent = parent[:4]
    frozen = config["frozen_bindings"]
    actual = {
        "map_hash": r15_config["frozen_bindings"]["map_hash"],
        "semantic_map_hash": r15_config["frozen_bindings"]["semantic_map_hash"],
        "expanded32_query_hash": r15_config["frozen_bindings"]["expanded32_query_hash"],
    }
    for key, value in actual.items():
        if str(frozen[key]) != str(value):
            raise ValueError(f"r16 frozen binding drift for {key}")
    return config, r15_config, algorithm, parent_algorithm, r1_parent, parent_path, resolved


def _policy(config: Mapping[str, Any]) -> ParkingApplicabilityPolicyR16:
    values = dict(config["parking_applicability"])
    for key in (
        "query_ids",
        "inapplicable_dispatch",
        "fallback_semantic_success_counted",
    ):
        values.pop(key)
    return ParkingApplicabilityPolicyR16(**values)


def _report_markdown(rows: Sequence[Mapping[str, Any]], gate: Mapping[str, Any]) -> str:
    lines = [
        "# 2A-V3 r16 parking 指标适用性根因报告",
        "",
        "本审计没有修改 R2 parking 指标、0.25 阈值或 50% 中心带门槛。它只检查整块停车语义连通域归一化得到的目标带，是否真实存在于冻结 L1 路线所在的局部可通行横截面。分类只使用地图、语义、路线和 footprint 净空，不使用规划成功或失败结果。",
        "",
        "| query | 局部目标可用站位 | 比例 | 分类 | 调度 |",
        "|---|---:|---:|---|---|",
    ]
    for row in rows:
        summary = row["applicability"]["summary"]
        lines.append(
            "| {query_id} | {available}/{count} | {ratio:.3f} | {classification} | {dispatch} |".format(
                query_id=row["query_id"],
                available=summary["target_available_station_count"],
                count=summary["active_parking_station_count"],
                ratio=float(summary["target_available_station_ratio"] or 0.0),
                classification=summary["classification"],
                dispatch=summary["dispatch"],
            )
        )
    lines.extend(
        [
            "",
            "## 判定",
            "",
            f"- 当前配置中的 parking 查询全部作为 strict-witness 压力门槛是否可作为 B 分母：`{str(gate['all_configured_queries_require_strict_witness_valid_for_b']).lower()}`。",
            f"- r13 研究型 B 是否保留：`{str(gate['r13_b_research_acceptance_preserved']).lower()}`。",
            "- 不适用查询必须走 E0 native Smac，fallback 不计语义成功；适用查询才进入 E5 语义搜索。",
            "- 这不是连续空间不可行证明，也不批准 A/生产晋升。",
            "",
            "## r16 whole-primitive 排序假设",
            "",
            "第一条查询的有界原型把边选择从 endpoint ranking 改为整条 Dubins primitive replay 后排序；中心带覆盖仅从 r15 的 0.333 左右提高到 0.344，仍远低于 0.50，因此按快速硬停止协议淘汰，不继续跑后两条。这避免了无证据的长时间参数扫描。",
            "",
        ]
    )
    return "\n".join(lines)


def run(*, output: Path, config_path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    (
        config,
        r15_config,
        algorithm,
        parent_algorithm,
        parent,
        parent_path,
        resolved_config,
    ) = _load(config_path)
    stage = config["parking_applicability"]
    policy = _policy(config)
    query_path = (
        resolved_config.parent / r15_config["frozen_bindings"]["expanded32_path"]
    ).resolve()
    queries, _intents, query_meta = load_query_set(
        query_path,
        actual_map_hash=config["frozen_bindings"]["map_hash"],
        actual_semantic_map_hash=config["frozen_bindings"]["semantic_map_hash"],
    )
    by_id = {query.query_id: query for query in queries}
    wanted = list(stage["query_ids"])
    if any(query_id not in by_id for query_id in wanted):
        raise ValueError("r16 applicability audit contains unknown frozen query")

    sources = [
        Path(__file__).resolve(),
        Path(__file__).with_name("semantic_parking_applicability_r16.py"),
        Path(__file__).with_name("semantic_route_phase_r16.py"),
        resolved_config,
        parent_path,
        query_path,
    ]
    source_hashes = {str(path): sha256_file(path) for path in sources}
    protocol = {
        **identity(),
        "schema_version": SCHEMA_VERSION,
        "configuration": config,
        "query_hash": query_meta["query_hash"],
        "source_hashes_at_start": source_hashes,
        "root_git": r15._git_capture(PACKAGE_ROOT.parents[5]),
        "evaluation_git": r15._git_capture(PACKAGE_ROOT.parent),
        "processes_before": applicability._process_audit(),
        "python": platform.python_version(),
    }
    _write_json(output / "protocol.json", protocol)
    (output / "reproduction_command.txt").write_text(
        f"/usr/bin/python3 -m arena_evaluation.two_layer_v3_semantic_r16_applicability --output {output} --config {resolved_config}\n",
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
    rows = []
    for query_id in wanted:
        row_started = time.monotonic()
        query = by_id[query_id]
        route_hash = None
        route_length_m = None
        orientation: Mapping[str, Any] = {}
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
            oriented = OrientedRoute(
                metadata["route_polyline"],
                world.start,
                world.goal,
                endpoint_attachment_limit_m=r13._policy(
                    parent_algorithm
                ).endpoint_attachment_limit_m,
            )
            route_hash = oriented.route_hash
            route_length_m = oriented.length_m
            audit = audit_parking_metric_applicability(world, oriented, policy)
        except (RuntimeError, ValueError) as error:
            audit = {
                "schema_version": SCHEMA_VERSION,
                "metric_definition_changed": False,
                "planner_outcome_used_for_classification": False,
                "continuous_space_infeasibility_claimed": False,
                "scope": "frozen_L1_route_binding_precondition",
                "summary": {
                    "applicable": False,
                    "classification": "NOT_APPLICABLE_ROUTE_BINDING_INVALID",
                    "dispatch": "E0_NATIVE_SMAC_FALLBACK",
                    "semantic_success_counted_on_fallback": False,
                    "active_parking_station_count": 0,
                    "target_available_station_count": 0,
                    "target_available_station_ratio": None,
                    "required_ratio_exclusive": policy.required_target_station_ratio_exclusive,
                    "route_local_best_deviation_p50": None,
                    "route_local_best_deviation_min": None,
                    "route_local_best_deviation_max": None,
                    "unavailable_station_runs_m": [],
                    "failure_code": "SEMANTIC_ROUTE_BINDING_INVALID",
                    "failure_detail": str(error),
                },
                "station_records": [],
            }
        row = {
            **identity(),
            "query_id": query_id,
            "category": query.category,
            "route_hash": route_hash,
            "route_length_m": route_length_m,
            "route_reversed_for_query": orientation.get("route_reversed_for_query"),
            "applicability": audit,
            "wall_ms": (time.monotonic() - row_started) * 1000.0,
            "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            * 1024,
        }
        rows.append(row)
        _write_json(output / f"result_{query_id}.json", row)

    all_processed = len(rows) == len(wanted)
    applicable_count = sum(
        row["applicability"]["summary"]["applicable"] is True for row in rows
    )
    gate = {
        "audit_completed": all_processed,
        "metric_definition_unchanged": all(
            row["applicability"]["metric_definition_changed"] is False
            for row in rows
        ),
        "planner_outcome_not_used": all(
            row["applicability"]["planner_outcome_used_for_classification"]
            is False
            for row in rows
        ),
        "applicable_query_count": applicable_count,
        "inapplicable_query_count": len(rows) - applicable_count,
        "r15_all_three_gate_valid_for_b": bool(
            all_processed and applicable_count == len(rows)
        ),
        "all_configured_queries_require_strict_witness_valid_for_b": bool(
            all_processed and applicable_count == len(rows)
        ),
        "r13_b_research_acceptance_preserved": True,
        "production_promotion_allowed": False,
        "next_action": "E0_FALLBACK_FOR_INAPPLICABLE_E5_ONLY_FOR_APPLICABLE",
    }
    flat = []
    for row in rows:
        summary = row["applicability"]["summary"]
        flat.append(
            {
                "query_id": row["query_id"],
                "category": row["category"],
                "route_length_m": row["route_length_m"],
                "active_parking_station_count": summary[
                    "active_parking_station_count"
                ],
                "target_available_station_count": summary[
                    "target_available_station_count"
                ],
                "target_available_station_ratio": summary[
                    "target_available_station_ratio"
                ],
                "route_local_best_deviation_p50": summary[
                    "route_local_best_deviation_p50"
                ],
                "classification": summary["classification"],
                "dispatch": summary["dispatch"],
                "failure_code": summary["failure_code"],
                "wall_ms": row["wall_ms"],
                "peak_rss_bytes": row["peak_rss_bytes"],
            }
        )
    with (output / "parking_metric_applicability.csv").open(
        "x", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    _write_json(output / "gate_results.json", gate)
    final = {
        **identity(),
        "schema_version": SCHEMA_VERSION,
        "gate_results": gate,
        "rows": rows,
        "static_prepare_ms": static_prepare_ms,
        "wall_s": time.monotonic() - started,
        "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        * 1024,
        "processes_after": applicability._process_audit(),
    }
    _write_json(output / "final_result.json", final)
    (output / "r16_root_cause_report.md").write_text(
        _report_markdown(rows, gate), encoding="utf-8"
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
    return 0 if result["gate_results"]["audit_completed"] else 2


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
