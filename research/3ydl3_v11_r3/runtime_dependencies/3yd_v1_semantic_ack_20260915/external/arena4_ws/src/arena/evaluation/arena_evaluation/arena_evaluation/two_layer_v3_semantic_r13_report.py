"""Build the immutable paired evidence bundle for 2A-V3 r13.

The command is deliberately read-only with respect to planner result trees. It
accepts the separately launched E0, E4-r3 and E5-r13 runs, verifies their
query/repetition pairing, and writes a new aggregate directory.  It never
loads a saved path as planner input; saved paths are used only for reporting
and overlays after all planning has completed.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import shlex
import shutil
import statistics
from typing import Any, Mapping, Sequence

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from . import semantic_applicability_v3 as applicability
from .semantic_map import sha256_file


ARCHITECTURE_ID = "2A-V3"
IMPLEMENTATION_REVISION = "r13-route-phase-multisemantic-state-lattice"
PROTOCOL_ID = "PLN-02-2A-V3-R13-PAIRED-REPORT-V1"


def _json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _rows(root: Path) -> list[dict[str, str]]:
    with (Path(root)/"runs.csv").open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _measured(root: Path, *, candidate: bool) -> list[dict[str, str]]:
    key = "mode" if candidate else "run_mode"
    rows = [row for row in _rows(root) if row.get(key) == "measured"]
    if not rows:
        raise ValueError(f"no measured rows: {root}")
    return rows


def _truth(value: Any) -> bool:
    return value is True or str(value).lower() == "true"


def _zero(value: Any) -> bool:
    if value in (None, ""):
        return False
    if str(value).lower() == "false":
        return True
    try:
        return float(value) == 0.0
    except (TypeError, ValueError):
        return False


def _paired_key(row: Mapping[str, Any]) -> tuple[str, int]:
    return str(row["query_id"]), int(row["repetition"])


def _latency(rows: Sequence[Mapping[str, str]], key: str) -> dict[str, Any]:
    values = [float(row[key]) for row in rows]
    return {
        "n": len(values), "p50_ms": statistics.median(values),
        "p95_ms": float(np.percentile(values, 95)),
        "p99_ms": float(np.percentile(values, 99)),
        "percentile_interpretation": "P95_P99_DEBUG_ONLY" if len(values) < 100 else "FORMAL",
    }


def _safety(rows: Sequence[Mapping[str, str]], *, candidate: bool) -> dict[str, Any]:
    keys = (
        ("collision_violations", "kinematic_violations", "hard_semantic_violations",
         "no_stopping_goal_violations", "reverse_distance_m", "rotate_in_place_count")
        if candidate else
        ("collision_violation_count", "kinematic_violation_count",
         "hard_semantic_violation_count", "no_stopping_goal_violation",
         "reverse_distance_m", "in_place_rotation_count")
    )
    violations = {
        key: sum(not _zero(row.get(key)) for row in rows)
        for key in keys
    }
    return {"row_violation_counts": violations, "gate_passed": not any(violations.values())}


def _arm_summary(root: Path, *, candidate: bool) -> dict[str, Any]:
    rows = _measured(root, candidate=candidate)
    wall_key = "request_wall_ms" if candidate else "cumulative_request_wall_ms"
    result = {
        "root": str(Path(root).resolve()),
        "measured_count": len(rows),
        "final_valid_count": sum(_truth(row.get("final_valid_success")) for row in rows),
        "r0_count": sum(row.get("relaxation_level") == "R0" for row in rows),
        "latency": _latency(rows, wall_key),
        "safety": _safety(rows, candidate=candidate),
        "runs_csv_sha256": sha256_file(Path(root)/"runs.csv"),
    }
    if candidate:
        result.update({
            "semantic_success_count": sum(_truth(row.get("semantic_success_counted")) for row in rows),
            "safe_soft_fallback_count": sum(_truth(row.get("safe_soft_fallback")) for row in rows),
            "exact_ack": _json(Path(root)/"exact_ack_summary.json"),
            "path_echo_count": sum(_truth(row.get("path_echo_verified")) for row in rows),
        })
    else:
        summary = _json(Path(root)/"summary.json")[0]
        result["exact_ack"] = {
            "hard_exact_mismatch_cells": summary["costmap_ack_hard_mismatch_cells"],
            "soft_exact_mismatch_cells": summary["costmap_ack_soft_exact_mismatch_cells"],
        }
    return result


def _query_rows(
    e0_rows: Sequence[Mapping[str, str]], e4_rows: Sequence[Mapping[str, str]],
    e5_rows: Sequence[Mapping[str, str]],
) -> list[dict[str, Any]]:
    query_order = []
    for row in e5_rows:
        if row["query_id"] not in query_order:
            query_order.append(row["query_id"])
    output = []
    for query_id in query_order:
        arms = {
            "e0": [row for row in e0_rows if row["query_id"] == query_id],
            "e4_r3": [row for row in e4_rows if row["query_id"] == query_id],
            "e5_r13": [row for row in e5_rows if row["query_id"] == query_id],
        }
        if any(len(rows) != 3 for rows in arms.values()):
            raise ValueError(f"{query_id}: expected exactly three paired measured rows per arm")
        output.append({
            "query_id": query_id,
            "e0_final_valid": sum(_truth(row["final_valid_success"]) for row in arms["e0"]),
            "e4_r3_final_valid": sum(_truth(row["final_valid_success"]) for row in arms["e4_r3"]),
            "e5_r13_final_valid": sum(_truth(row["final_valid_success"]) for row in arms["e5_r13"]),
            "e0_p50_ms": statistics.median(float(row["cumulative_request_wall_ms"]) for row in arms["e0"]),
            "e4_r3_p50_ms": statistics.median(float(row["cumulative_request_wall_ms"]) for row in arms["e4_r3"]),
            "e5_r13_p50_ms": statistics.median(float(row["request_wall_ms"]) for row in arms["e5_r13"]),
            "e5_semantic_success": sum(_truth(row["semantic_success_counted"]) for row in arms["e5_r13"]),
            "e5_safe_soft_fallback": sum(_truth(row["safe_soft_fallback"]) for row in arms["e5_r13"]),
            "e5_lane_correct_side_ratio": arms["e5_r13"][0].get("lane_correct_side_ratio", ""),
            "e5_lane_target_band_ratio": arms["e5_r13"][0].get("lane_target_band_ratio", ""),
            "e5_lane_lateral_error_p50_m": arms["e5_r13"][0].get("lane_lateral_error_p50_m", ""),
            "e5_parking_center_band_ratio": arms["e5_r13"][0].get("parking_center_band_ratio", ""),
            "e5_parking_normalized_deviation_p50": arms["e5_r13"][0].get("parking_normalized_deviation_p50", ""),
        })
    return output


def _path(root: Path, arm: str, query_id: str, *, candidate: bool) -> list[dict[str, Any]]:
    path = (
        Path(root)/"requests"/f"measured_01_{query_id}"/"path.json"
        if candidate else Path(root)/"paths"/f"{arm}_{query_id}_measured_1.json"
    )
    return _json(path)


def _overlays(
    output: Path, query_rows: Sequence[Mapping[str, Any]], *, map_yaml: Path,
    e0: Path, e4: Path, e5: Path,
) -> None:
    descriptor = yaml.safe_load(Path(map_yaml).read_text(encoding="utf-8"))
    image_path = (Path(map_yaml).parent/descriptor["image"]).resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"cannot load map image {image_path}")
    resolution = float(descriptor["resolution"])
    origin = tuple(map(float, descriptor["origin"][:2]))
    extent = [origin[0], origin[0]+image.shape[1]*resolution,
              origin[1], origin[1]+image.shape[0]*resolution]
    target = output/"overlays"
    target.mkdir()
    colors = {"E0": "#3572C6", "E4-r3": "#ED8B24", "E5-r13": "#159447"}
    for row in query_rows:
        query_id = str(row["query_id"])
        paths = {
            "E0": _path(e0, "E0", query_id, candidate=False),
            "E4-r3": _path(e4, "E4", query_id, candidate=False),
            "E5-r13": _path(e5, "E5", query_id, candidate=True),
        }
        all_xy = []
        fig, ax = plt.subplots(figsize=(8.5, 6.5), constrained_layout=True)
        ax.imshow(image, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=255)
        for name, points in paths.items():
            xy = np.asarray([[float(p["x"]), float(p["y"])] for p in points])
            all_xy.append(xy)
            ax.plot(xy[:, 0], xy[:, 1], color=colors[name], linewidth=1.6, label=name)
            if len(xy) > 2:
                index = min(len(xy)-2, max(0, len(xy)//2))
                delta = xy[index+1]-xy[index]
                ax.arrow(xy[index, 0], xy[index, 1], delta[0], delta[1],
                         color=colors[name], head_width=.30, length_includes_head=True)
        first = all_xy[0][0]
        last = all_xy[0][-1]
        ax.scatter([first[0]], [first[1]], c="#00B8D9", marker="o", s=65, label="start")
        ax.scatter([last[0]], [last[1]], c="#D81B60", marker="*", s=100, label="goal")
        merged = np.vstack(all_xy)
        margin = 2.0
        ax.set_xlim(float(merged[:, 0].min()-margin), float(merged[:, 0].max()+margin))
        ax.set_ylim(float(merged[:, 1].min()-margin), float(merged[:, 1].max()+margin))
        ax.set_aspect("equal")
        ax.set_title(f"{query_id}: measured repetition 1")
        ax.set_xlabel("map x (m)")
        ax.set_ylabel("map y (m)")
        ax.legend(loc="best", fontsize=8)
        fig.savefig(target/f"{query_id}.png", dpi=150)
        plt.close(fig)


def build_report(
    *, output: Path, selected_e0: Path, selected_e4: Path, selected_e5: Path,
    targeted_e0: Path, targeted_e4: Path, targeted_e5: Path,
    cold_e0: Path, cold_e4: Path, cold_e5: Path, map_yaml: Path,
) -> dict[str, Any]:
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    roots = {name: Path(value).resolve() for name, value in {
        "selected_e0": selected_e0, "selected_e4": selected_e4, "selected_e5": selected_e5,
        "targeted_e0": targeted_e0, "targeted_e4": targeted_e4, "targeted_e5": targeted_e5,
        "cold_e0": cold_e0, "cold_e4": cold_e4, "cold_e5": cold_e5,
    }.items()}
    for name, root in roots.items():
        if not (root/"runs.csv").is_file():
            raise ValueError(f"missing runs.csv for {name}: {root}")
        if (root/"EXCLUDED.json").exists():
            raise ValueError(f"refusing excluded input for {name}: {root}")

    selected = {
        "E0": _arm_summary(roots["selected_e0"], candidate=False),
        "E4-r3": _arm_summary(roots["selected_e4"], candidate=False),
        "E5-r13": _arm_summary(roots["selected_e5"], candidate=True),
    }
    targeted = {
        "E0": _arm_summary(roots["targeted_e0"], candidate=False),
        "E4-r3": _arm_summary(roots["targeted_e4"], candidate=False),
        "E5-r13": _arm_summary(roots["targeted_e5"], candidate=True),
    }
    selected_rows = _query_rows(
        _measured(roots["selected_e0"], candidate=False),
        _measured(roots["selected_e4"], candidate=False),
        _measured(roots["selected_e5"], candidate=True),
    )
    targeted_rows = _query_rows(
        _measured(roots["targeted_e0"], candidate=False),
        _measured(roots["targeted_e4"], candidate=False),
        _measured(roots["targeted_e5"], candidate=True),
    )
    e0_keys = {_paired_key(r) for r in _measured(roots["selected_e0"], candidate=False) if _truth(r["final_valid_success"])}
    e5_valid = {_paired_key(r) for r in _measured(roots["selected_e5"], candidate=True) if _truth(r["final_valid_success"])}
    no_e0_regression = e0_keys <= e5_valid
    measured_ratio = selected["E5-r13"]["latency"]["p50_ms"] / selected["E0"]["latency"]["p50_ms"]
    cold = {
        "E0_p50_ms": _json(roots["cold_e0"]/"summary.json")[0]["cumulative_request_wall_ms"]["p50"],
        "E4-r3_p50_ms": _json(roots["cold_e4"]/"summary.json")[0]["cumulative_request_wall_ms"]["p50"],
        "E5-r13_p50_ms": _json(roots["cold_e5"]/"performance_summary.json")["request_p50_ms"],
    }
    cold["E5_over_E0_ratio"] = cold["E5-r13_p50_ms"]/cold["E0_p50_ms"]
    gates = {
        "targeted_e5_strict_9_of_9": targeted["E5-r13"]["semantic_success_count"] == 9,
        "targeted_e5_final_valid_9_of_9": targeted["E5-r13"]["final_valid_count"] == 9,
        "selected8_e5_final_valid_24_of_24": selected["E5-r13"]["final_valid_count"] == 24,
        "selected8_no_e0_success_regression": no_e0_regression,
        "selected8_e5_exact_ack": selected["E5-r13"]["exact_ack"].get("gate_passed") is True,
        "targeted_e5_exact_ack": targeted["E5-r13"]["exact_ack"].get("gate_passed") is True,
        "all_arms_safety": all(x["safety"]["gate_passed"] for group in (selected, targeted) for x in group.values()),
        "measured_request_ratio_le_2": measured_ratio <= 2.0,
        "cold_request_ratio_le_2": cold["E5_over_E0_ratio"] <= 2.0,
    }
    gates["b_gate_passed"] = all(gates.values())
    result = {
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "qualification": "B_RESEARCH_ACCEPTANCE" if gates["b_gate_passed"] else "BELOW_B",
        "production_promotion": False,
        "selected8": selected,
        "targeted": targeted,
        "selected8_query_rows": selected_rows,
        "targeted_query_rows": targeted_rows,
        "performance": {"selected8_measured_E5_over_E0_ratio": measured_ratio, "cold": cold},
        "gates": gates,
        "limitations": [
            "selected8 strict semantic success is limited to two of eight queries",
            "safe fallbacks are final-valid but never counted as semantic success",
            "P95/P99 are debug-only because each arm has fewer than 100 measured samples",
            "no 30-50 query production-scale expansion was run",
        ],
        "input_roots": {key: str(value) for key, value in roots.items()},
    }
    applicability._write_json(output/"same_round_comparison.json", result)
    applicability._write_json(output/"gate_results.json", gates)
    applicability._write_json(output/"performance_summary.json", result["performance"])
    with (output/"query_comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected_rows[0]))
        writer.writeheader(); writer.writerows(selected_rows)
    with (output/"targeted_comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(targeted_rows[0]))
        writer.writeheader(); writer.writerows(targeted_rows)
    _overlays(output, selected_rows, map_yaml=map_yaml, e0=roots["selected_e0"],
              e4=roots["selected_e4"], e5=roots["selected_e5"])
    command = " ".join(shlex.quote(item) for item in [
        "/usr/bin/python3", "-m", "arena_evaluation.two_layer_v3_semantic_r13_report",
        "--output", str(output), "--selected-e0", str(selected_e0),
        "--selected-e4", str(selected_e4), "--selected-e5", str(selected_e5),
        "--targeted-e0", str(targeted_e0), "--targeted-e4", str(targeted_e4),
        "--targeted-e5", str(targeted_e5), "--cold-e0", str(cold_e0),
        "--cold-e4", str(cold_e4), "--cold-e5", str(cold_e5),
        "--map-yaml", str(map_yaml),
    ])
    (output/"reproduction_command.txt").write_text(command+"\n", encoding="utf-8")
    applicability._write_json(output/"process_audit.json", applicability._process_audit())
    main_report = applicability.ROOT/"docs"/"PLN-02_ARCHITECTURE_2A_V3.md"
    interface_report = applicability.ROOT/"docs"/"PLN-02_2A_V3_SMAC_INTERFACE_AUDIT.md"
    sources = [
        Path(__file__).resolve(), Path(__file__).with_name("two_layer_v3_semantic_r13_online.py"),
        Path(__file__).with_name("two_layer_v3_semantic_r13_benchmark.py"),
        Path(__file__).with_name("semantic_route_phase_v3.py"),
        Path(__file__).parents[1]/"config"/"two_layer_v3_semantic_r13_online.yaml",
        Path(__file__).parents[1]/"config"/"two_layer_v3_semantic_r13_route_phase.yaml",
        Path(__file__).parents[1]/"config"/"pudu_wanda_3f_v3_r10_targeted_applicable_positive_v1.yaml",
        Path(__file__).parents[1]/"config"/"pudu_wanda_3f_selected8_gt50m_r2_v2.yaml",
        main_report, interface_report,
    ]
    snapshot = output/"source_snapshot"
    snapshot.mkdir()
    snapshot_hashes = {}
    for source in sources:
        source = source.resolve()
        shutil.copy2(source, snapshot/source.name)
        snapshot_hashes[str(source)] = sha256_file(source)
    shutil.copy2(main_report, output/"final_report.md")
    applicability._write_json(output/"source_snapshot_manifest.json", snapshot_hashes)
    applicability._write_json(output/"artifact_hashes.json", applicability._manifest_files(output))
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    for name in ("selected-e0", "selected-e4", "selected-e5", "targeted-e0",
                 "targeted-e4", "targeted-e5", "cold-e0", "cold-e4", "cold-e5"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    args = parser.parse_args(argv)
    result = build_report(
        output=args.output, selected_e0=args.selected_e0, selected_e4=args.selected_e4,
        selected_e5=args.selected_e5, targeted_e0=args.targeted_e0,
        targeted_e4=args.targeted_e4, targeted_e5=args.targeted_e5,
        cold_e0=args.cold_e0, cold_e4=args.cold_e4, cold_e5=args.cold_e5,
        map_yaml=args.map_yaml,
    )
    print(json.dumps({"qualification": result["qualification"], "gates": result["gates"]}, indent=2))
    return 0 if result["gates"]["b_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
