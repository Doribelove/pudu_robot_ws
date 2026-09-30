"""PLN-02 2A-V2 r3 lane-relative viability-guide benchmark entry point."""

from __future__ import annotations

import json
import math
import shlex
import sys
from contextlib import contextmanager
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, Iterator, Optional, Sequence

import cv2
import numpy as np
import yaml

from . import two_layer_v2_semantic_r1_benchmark as r1
from . import two_layer_v2_semantic_r2_benchmark as r2
from .regional_preference_r1 import expand_roi_to_route_lane_instances, orient_route_for_query
from .regional_preference_r3 import RegionalPreferenceBuilderR3
from .semantic_costmap_r2 import SemanticCostmapComposerR2
from .semantic_smac_session_r2 import ExactSemanticSmacSessionR2
from .planner_benchmark.models import Query
from .semantic_map import canonical_hash


ARCHITECTURE_ID = "2A-V2"
IMPLEMENTATION_REVISION = "r3-lane-relative-viability-guide"
PROTOCOL_ID = "PLN-02-2A-V2-R3-LANE-VIABILITY-GUIDE-V1"
PARENT_ARCHITECTURE = "2A-V2-r2"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_v2_semantic_r3.yaml"
DEFAULT_MIRROR_SELECTION_POLICY = (
    Path(__file__).resolve().parents[1]
    / "config/pudu_wanda_3f_r3_mirror_selection_policy_v3.yaml"
)
ROOT = Path(__file__).resolve().parents[7]
SOURCE_FILES = (
    Path(__file__),
    Path(__file__).with_name("regional_preference_r3.py"),
    Path(__file__).with_name("regional_preference_r2.py"),
    Path(__file__).with_name("semantic_costmap_r2.py"),
    Path(__file__).with_name("semantic_query_defaults.py"),
    Path(__file__).with_name("semantic_smac_session_r2.py"),
    Path(__file__).with_name("two_layer_v2_semantic_r1_benchmark.py"),
    Path(__file__).with_name("two_layer_v2_semantic_r2_benchmark.py"),
    Path(__file__).resolve().parents[1] / "src/nav2_effective_costmap.cpp",
    Path(__file__).resolve().parents[1] / "test/test_two_layer_v2_semantic_r3.py",
    Path(__file__).resolve().parents[1] / "setup.py",
    DEFAULT_CONFIG,
    r2.DEFAULT_QUERY_SET_PATH,
    DEFAULT_MIRROR_SELECTION_POLICY,
    Path(__file__).resolve().parents[1] / "config/pudu_wanda_3f_r3_mirror_selection_policy_v1.yaml",
    Path(__file__).resolve().parents[1] / "config/pudu_wanda_3f_r3_mirror_selection_policy_v2.yaml",
    Path(__file__).resolve().parents[1] / "config/pudu_wanda_3f_r3_frozen_mirror_query_v1.yaml",
    Path(__file__).resolve().parents[1] / "config/two_layer_v2_semantic_r3_calibration_freeze.yaml",
    Path(__file__).resolve().parents[1] / "config/pudu_wanda_3f_r3_targeted_preflight3_v1.yaml",
    ROOT / "docs/PLN-02_ARCHITECTURE_2A_V2_R3_RESEARCH_PLAN.md",
)


@contextmanager
def _r3_bindings() -> Iterator[None]:
    r1_values: Dict[str, Any] = {
        "ARCHITECTURE_ID": ARCHITECTURE_ID,
        "IMPLEMENTATION_REVISION": IMPLEMENTATION_REVISION,
        "PARENT_ARCHITECTURE": PARENT_ARCHITECTURE,
        "DEFAULT_CONFIG": DEFAULT_CONFIG,
        "RegionalPreferenceBuilderR1": RegionalPreferenceBuilderR3,
        "SemanticCostmapComposer": SemanticCostmapComposerR2,
        "SemanticSmacSession": ExactSemanticSmacSessionR2,
    }
    r2_values: Dict[str, Any] = {
        "ARCHITECTURE_ID": ARCHITECTURE_ID,
        "IMPLEMENTATION_REVISION": IMPLEMENTATION_REVISION,
        "PROTOCOL_ID": PROTOCOL_ID,
        "PARENT_ARCHITECTURE": PARENT_ARCHITECTURE,
        "DEFAULT_CONFIG": DEFAULT_CONFIG,
        "SOURCE_FILES": SOURCE_FILES,
    }
    old_r1 = {name: getattr(r1, name) for name in r1_values}
    old_r2 = {name: getattr(r2, name) for name in r2_values}
    try:
        for name, value in r1_values.items():
            setattr(r1, name, value)
        for name, value in r2_values.items():
            setattr(r2, name, value)
        yield
    finally:
        for name, value in old_r1.items():
            setattr(r1, name, value)
        for name, value in old_r2.items():
            setattr(r2, name, value)


def _parser():
    parser = r2._parser()
    parser.description = (
        "Run PLN-02 static 2A-V2/r3 directed lane-viability certification "
        "and guide validation"
    )
    parser.set_defaults(config=DEFAULT_CONFIG)
    for action in parser._actions:
        if action.dest == "mode":
            action.choices = tuple(action.choices) + ("mirror-selection",)
        if action.dest == "query_set":
            action.help = (
                "frozen query-set YAML for offline-diagnostic or real-ablation; "
                "defaults to the map-bound eight-query set"
            )
    parser.add_argument(
        "--mirror-selection-policy", type=Path,
        default=DEFAULT_MIRROR_SELECTION_POLICY,
    )
    return parser


def _write_guide_facet(ctx: Any, query: Any, route: Any, diagnostics: Dict[str, Any], path: Path) -> None:
    base = cv2.imread(str(ctx.hospital_map.image_path), cv2.IMREAD_GRAYSCALE)
    image = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
    route_cells = []
    for point in route.polyline:
        cell = ctx.hospital_map.world_to_cell(float(point[0]), float(point[1]))
        if cell is not None:
            route_cells.append((cell[1], cell[0]))
    guide_cell_segments = []
    for segment in diagnostics.get("guide_polylines_world", []):
        cells = []
        for point in segment:
            cell = ctx.hospital_map.world_to_cell(float(point[0]), float(point[1]))
            if cell is not None:
                cells.append((cell[1], cell[0]))
        if cells:
            guide_cell_segments.append(cells)
    guide_cells = [cell for segment in guide_cell_segments for cell in segment]
    if len(route_cells) >= 2:
        cv2.polylines(image, [np.asarray(route_cells, np.int32)], False, (0, 0, 0), 3, cv2.LINE_AA)
    for segment in guide_cell_segments:
        if len(segment) >= 2:
            cv2.polylines(image, [np.asarray(segment, np.int32)], False, (0, 180, 0), 4, cv2.LINE_AA)
    endpoint_cells = []
    for pose, color in ((query.start, (255, 0, 0)), (query.goal, (0, 0, 255))):
        cell = ctx.hospital_map.world_to_cell(float(pose[0]), float(pose[1]))
        if cell is not None:
            cv2.circle(image, (cell[1], cell[0]), 8, color, -1, cv2.LINE_AA)
            endpoint_cells.append((cell[1], cell[0]))
    all_cells = route_cells + guide_cells + endpoint_cells
    if all_cells:
        coords = np.asarray(all_cells, dtype=np.int32)
        margin = 100
        col0 = max(0, int(coords[:, 0].min()) - margin)
        col1 = min(image.shape[1], int(coords[:, 0].max()) + margin + 1)
        row0 = max(0, int(coords[:, 1].min()) - margin)
        row1 = min(image.shape[0], int(coords[:, 1].max()) + margin + 1)
        image = image[row0:row1, col0:col1]
    cv2.imwrite(str(path), image)


def _augment_offline_guide_diagnostics(
    *, extracted_dir: Path, semantic_map_path: Path, topology_cache: Path,
    output: Path, config_path: Path, preference_policy_overrides: Dict[str, Any],
) -> None:
    config = r1._load_config(config_path, preference_policy_overrides)
    prepared = r1._prepare(
        extracted_dir, semantic_map_path, topology_cache, config, output=output,
    )
    ctx, semantic_map, raster, topology, _annotator, router, query_bundle = prepared[:7]
    queries = query_bundle[0]
    selector = r1._semantic_selector(topology, router)
    builder = RegionalPreferenceBuilderR3(
        ctx.hospital_map, raster, policy=config["regional_preference"], semantic_map=semantic_map,
    )
    facets = output / "guide_facets"
    facets.mkdir(exist_ok=True)
    records = []
    for query in queries:
        _, _, route, reason = selector(
            topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
        )
        if route is None:
            records.append({"query_id": query.query_id, "route_found": False, "reason": reason})
            continue
        route, orientation = orient_route_for_query(route, query)
        allowed = r1.r2_runtime._raw_corridor_mask(
            ctx, topology, route, query, float(config["roi"]["r0_padding_m"]),
        )
        allowed, roi_diagnostics = expand_roi_to_route_lane_instances(
            ctx.hospital_map, raster, semantic_map, route.polyline, allowed,
            free_mask=r1.r2_runtime._raw_free_mask(ctx),
            route_probe_radius_m=float(config["roi"].get("lane_route_probe_radius_m", 0.50)),
        )
        field = builder.build(
            route.polyline, goal=query.goal, allowed_mask=allowed,
            route_diagnostics=orientation,
        )
        guide_diagnostics = {
            key: value for key, value in field.diagnostics.items()
            if (
                key.startswith("viability_")
                or (
                    key.startswith("guide_")
                    and key not in {"guide_polyline_world", "guide_polylines_world"}
                )
            )
        }
        record = {
            "query_id": query.query_id,
            "category": query.category,
            "route_found": True,
            **orientation,
            **roi_diagnostics,
            **guide_diagnostics,
        }
        records.append(record)
        _write_guide_facet(
            ctx, query, route, field.diagnostics,
            facets / f"{query.query_id}.png",
        )
    target_records = [
        record for record in records
        if record.get("query_id") in {
            "real-lane-forward", "real-lane-reverse",
            "cmp2-01-lane-north", "cmp2-02-lane-south",
            "r3-mirror-1-positive", "r3-mirror-2-negative",
        }
    ]
    targets = {record["query_id"]: record for record in target_records}
    expected_target_count = 2
    viability_gate = bool(len(targets) == expected_target_count and all(
        record.get("viability_gate_passed") is True for record in targets.values()
    ))
    guide_gate = bool(len(targets) == 2 and all(
        record.get("guide_status") == "BUILT"
        and float(record.get("guide_correct_side_ratio") or 0.0) >= 0.80
        and float(record.get("guide_target_error_p50_m") or float("inf")) <= 0.50
        and float(record.get("guide_max_curvature_1pm") or float("inf")) <= 2.50
        for record in targets.values()
    ))
    payload = {
        "schema_version": "2A-V2-r3-offline-guide-diagnostic-v1",
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "offline_viability_gate_passed": viability_gate,
        "offline_guide_gate_passed": guide_gate,
        "cold_start": {
            "semantic_raster_ms": prepared[7],
            "topology_load_ms": prepared[8],
            "semantic_edge_precompute_ms": prepared[9],
        },
        "records": records,
    }
    (output / "r3_guide_diagnostics.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    source_payload = json.loads((extracted_dir / "ATLAS_DATA").read_text(encoding="utf-8"))
    source_zones = source_payload.get("map", {}).get("zones", [])
    selected_ids = sorted({
        semantic_id
        for record in records
        for semantic_id in record.get("viability_lane_instance_ids", [])
    })
    semantic_features = {
        feature.semantic_id: feature for feature in semantic_map.features
        if feature.semantic_class == "lane"
    }
    source_by_id = {
        str(zone.get("id")): {"source_index": index, **zone}
        for index, zone in enumerate(source_zones) if isinstance(zone, dict)
    }
    lane_audit = {
        "schema_version": "2A-V2-r3-lane-source-audit-v1",
        "source_pdmap_hash": semantic_map.source_pdmap_hash,
        "source_atlas_hash": r1.sha256_file(extracted_dir / "ATLAS_DATA"),
        "semantic_map_hash": semantic_map.semantic_map_hash,
        "selected_lane_ids": selected_ids,
        "lanes": [
            {
                "semantic_id": semantic_id,
                "source": source_by_id.get(semantic_id),
                "converted": asdict(semantic_features[semantic_id])
                if semantic_id in semantic_features else None,
                "source_conversion_match": bool(
                    semantic_id in source_by_id and semantic_id in semantic_features
                ),
            }
            for semantic_id in selected_ids
        ],
    }
    (output / "lane_source_audit.json").write_text(
        json.dumps(lane_audit, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _undirected_route_hash(polyline: Sequence[Sequence[float]]) -> str:
    points = [[float(point[0]), float(point[1])] for point in polyline]
    reverse = list(reversed(points))
    return canonical_hash(min(points, reverse))


def _endpoint_yaws(polyline: Sequence[Sequence[float]]) -> tuple[float, float]:
    points = [[float(point[0]), float(point[1])] for point in polyline]
    if len(points) < 2:
        return 0.0, 0.0
    start_index = next(
        (index for index in range(1, len(points)) if points[index] != points[0]), 1,
    )
    end_index = next(
        (index for index in range(len(points) - 2, -1, -1) if points[index] != points[-1]),
        len(points) - 2,
    )
    return (
        math.atan2(
            points[start_index][1] - points[0][1],
            points[start_index][0] - points[0][0],
        ),
        math.atan2(
            points[-1][1] - points[end_index][1],
            points[-1][0] - points[end_index][0],
        ),
    )


def _run_mirror_selection(
    *, extracted_dir: Path, semantic_map_path: Path, topology_cache: Path,
    output: Path, config_path: Path, selection_policy_path: Path,
    preference_policy_overrides: Dict[str, Any],
) -> None:
    """Enumerate the pre-frozen topology-quantile pool and freeze one mirror."""
    config = r1._load_config(config_path, preference_policy_overrides)
    selection = yaml.safe_load(selection_policy_path.read_text(encoding="utf-8"))
    prepared = r1._prepare(
        extracted_dir, semantic_map_path, topology_cache, config, output=output,
    )
    ctx, semantic_map, raster, topology, _annotator, router = prepared[:6]
    bindings = selection["bindings"]
    if ctx.map_sha256 != bindings["map_hash"]:
        raise RuntimeError("mirror selection map hash mismatch")
    if semantic_map.semantic_map_hash != bindings["semantic_map_hash"]:
        raise RuntimeError("mirror selection semantic-map hash mismatch")

    builder = RegionalPreferenceBuilderR3(
        ctx.hospital_map, raster,
        policy={**config["regional_preference"], "viability_certificate_only": True},
        semantic_map=semantic_map,
    )
    candidate_policy = selection["candidate_generation"]
    minimum_clearance = float(candidate_policy["minimum_endpoint_clearance_m"])
    minimum_separation = float(candidate_policy["minimum_euclidean_separation_m"])
    maximum_separation = float(candidate_policy["maximum_euclidean_separation_m"])
    quantiles = [float(value) for value in candidate_policy["longitudinal_quantiles"]]
    window = 2.5
    by_label: Dict[int, list[Any]] = {}
    for node in topology.graph.nodes:
        cell = ctx.hospital_map.world_to_cell(float(node.x), float(node.y))
        if cell is None or float(node.clearance_m) < minimum_clearance:
            continue
        label = int(builder._lane_labels[cell])
        if label > 0:
            by_label.setdefault(label, []).append(node)

    endpoint_pairs: list[tuple[int, Any, Any]] = []
    lane_sampling = []
    for label, nodes in sorted(by_label.items()):
        coordinates = np.asarray([[float(node.x), float(node.y)] for node in nodes], np.float64)
        centered = coordinates - np.mean(coordinates, axis=0)
        _values, vectors = np.linalg.eigh(centered.T @ centered)
        axis = vectors[:, -1]
        dominant = int(np.argmax(np.abs(axis)))
        if axis[dominant] < 0.0:
            axis = -axis
        projection = coordinates @ axis
        low, high = float(np.min(projection)), float(np.max(projection))
        representatives = []
        for quantile in quantiles:
            target = low + quantile * (high - low)
            choices = [
                (node, float(value)) for node, value in zip(nodes, projection)
                if abs(float(value) - target) <= window + 1.0e-12
            ]
            if not choices:
                continue
            selected_node, selected_projection = min(
                choices,
                key=lambda item: (
                    -float(item[0].clearance_m), abs(item[1] - target), int(item[0].node_id),
                ),
            )
            if not representatives or selected_node.node_id != representatives[-1][0].node_id:
                representatives.append((selected_node, selected_projection, quantile))
        pair_count = 0
        for first, second in zip(representatives, representatives[1:]):
            distance = math.hypot(
                float(second[0].x) - float(first[0].x),
                float(second[0].y) - float(first[0].y),
            )
            if minimum_separation <= distance <= maximum_separation:
                endpoint_pairs.append((label, first[0], second[0]))
                pair_count += 1
        lane_sampling.append({
            "lane_label": label,
            "lane_semantic_id": builder._lane_instance_ids.get(label, ""),
            "eligible_topology_node_count": len(nodes),
            "principal_axis": [float(axis[0]), float(axis[1])],
            "representative_node_ids": [int(item[0].node_id) for item in representatives],
            "candidate_pair_count": pair_count,
        })

    selector = r1._semantic_selector(topology, router)
    minimum_lane_fraction = float(candidate_policy["require_route_lane_station_fraction"])
    maximum_attachment = float(
        candidate_policy.get("maximum_l1_endpoint_attachment_distance_m", float("inf"))
    )
    minimum_route_length = float(
        candidate_policy.get("minimum_certified_l1_route_length_m", 0.0)
    )
    eligibility_policy = selection["eligibility"]
    robust_correct = float(eligibility_policy.get("correct_side_ratio_min", 0.80))
    robust_target = float(eligibility_policy.get("target_band_station_ratio_min", 0.50))
    candidate_records = []
    for pair_index, (label, first, second) in enumerate(endpoint_pairs):
        directed_records = []
        for direction_name, start_node, goal_node in (
            ("positive", first, second), ("negative", second, first),
        ):
            direct_yaw = math.atan2(
                float(goal_node.y) - float(start_node.y),
                float(goal_node.x) - float(start_node.x),
            )
            query = Query(
                query_id=f"mirror-candidate-{pair_index:03d}-{direction_name}",
                start=[float(start_node.x), float(start_node.y), direct_yaw],
                goal=[float(goal_node.x), float(goal_node.y), direct_yaw],
                category="lane_mirror_calibration",
                seed=int(config["experiment"]["seed"]),
            )
            _, _, route, reason = selector(
                topology, query, cache_mode=r1.r2_runtime.CACHE_MODE_OPTIMIZED, timing={},
            )
            if route is None:
                directed_records.append({
                    "direction": direction_name, "route_found": False, "reason": reason,
                })
                continue
            route, orientation = orient_route_for_query(route, query)
            route_cells = [
                ctx.hospital_map.world_to_cell(float(point[0]), float(point[1]))
                for point in route.polyline
            ]
            route_labels = [
                int(builder._lane_labels[cell]) for cell in route_cells if cell is not None
            ]
            lane_fraction = (
                float(sum(value == label for value in route_labels) / len(route_labels))
                if route_labels else 0.0
            )
            allowed = r1.r2_runtime._raw_corridor_mask(
                ctx, topology, route, query, float(config["roi"]["r0_padding_m"]),
            )
            allowed, roi_diagnostics = expand_roi_to_route_lane_instances(
                ctx.hospital_map, raster, semantic_map, route.polyline, allowed,
                free_mask=r1.r2_runtime._raw_free_mask(ctx),
                route_probe_radius_m=float(config["roi"].get("lane_route_probe_radius_m", 0.50)),
            )
            field = builder.build(
                route.polyline, goal=query.goal, allowed_mask=allowed,
                route_diagnostics=orientation,
            )
            start_yaw, goal_yaw = _endpoint_yaws(route.polyline)
            directed_records.append({
                "direction": direction_name,
                "route_found": True,
                "reason": reason,
                "start": [float(start_node.x), float(start_node.y), start_yaw],
                "goal": [float(goal_node.x), float(goal_node.y), goal_yaw],
                "route_length_m": float(sum(
                    math.dist(left[:2], right[:2])
                    for left, right in zip(route.polyline, route.polyline[1:])
                )),
                "route_lane_station_fraction": lane_fraction,
                "undirected_route_hash": _undirected_route_hash(route.polyline),
                **orientation,
                **roi_diagnostics,
                **{
                    key: value for key, value in field.diagnostics.items()
                    if key.startswith("viability_")
                },
            })
        complete = len(directed_records) == 2 and all(
            record.get("route_found") is True for record in directed_records
        )
        route_match = complete and len({
            record["undirected_route_hash"] for record in directed_records
        }) == 1
        eligible = bool(
            complete and route_match
            and all(record["route_lane_station_fraction"] >= minimum_lane_fraction for record in directed_records)
            and all(record.get("viability_gate_passed") is True for record in directed_records)
            and all(float(record["route_start_distance_m"]) <= maximum_attachment for record in directed_records)
            and all(float(record["route_end_distance_m"]) <= maximum_attachment for record in directed_records)
            and all(float(record["route_length_m"]) >= minimum_route_length for record in directed_records)
            and all(
                float(record["viability_full_path_correct_side_station_ratio"]) >= robust_correct
                for record in directed_records
            )
            and all(
                float(record["viability_full_path_target_station_ratio"]) >= robust_target
                for record in directed_records
            )
        )
        minimum_correct = min(
            (float(record.get("viability_full_path_correct_side_station_ratio", 0.0))
             for record in directed_records), default=0.0,
        )
        minimum_target = min(
            (float(record.get("viability_full_path_target_station_ratio", 0.0))
             for record in directed_records), default=0.0,
        )
        candidate_records.append({
            "candidate_id": f"mirror-candidate-{pair_index:03d}",
            "lane_label": label,
            "lane_semantic_id": builder._lane_instance_ids.get(label, ""),
            "endpoint_node_ids": [int(first.node_id), int(second.node_id)],
            "minimum_endpoint_clearance_m": min(
                float(first.clearance_m), float(second.clearance_m),
            ),
            "eligible": eligible,
            "same_undirected_l1_route": route_match,
            "minimum_bidirectional_correct_side_ratio": minimum_correct,
            "minimum_bidirectional_target_band_ratio": minimum_target,
            "directions": directed_records,
        })

    eligible_records = [record for record in candidate_records if record["eligible"]]
    selected = min(
        eligible_records,
        key=lambda record: (
            -record["minimum_bidirectional_correct_side_ratio"],
            -record["minimum_bidirectional_target_band_ratio"],
            -record["minimum_endpoint_clearance_m"],
            -min(item["route_length_m"] for item in record["directions"]),
            record["lane_semantic_id"],
            tuple(record["directions"][0]["start"][:2]),
        ),
        default=None,
    )
    result = {
        "schema_version": "2A-V2-r3-mirror-selection-result-v1",
        "architecture_id": ARCHITECTURE_ID,
        "implementation_revision": IMPLEMENTATION_REVISION,
        "protocol_id": PROTOCOL_ID,
        "selection_policy": selection,
        "selection_policy_sha256": r1.sha256_file(selection_policy_path),
        "cold_start": {
            "semantic_raster_ms": prepared[7],
            "topology_load_ms": prepared[8],
            "semantic_edge_precompute_ms": prepared[9],
        },
        "lane_sampling": lane_sampling,
        "candidate_count": len(candidate_records),
        "eligible_candidate_count": len(eligible_records),
        "selection_status": "FROZEN" if selected is not None else "NO_ELIGIBLE_MIRROR_PAIR",
        "selected_candidate_id": selected["candidate_id"] if selected is not None else None,
        "selected": selected,
        "candidates": candidate_records,
    }
    (output / "mirror_selection_result.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if selected is not None:
        frozen = {
            "schema_version": "PLN-02-2A-V2-R3-FROZEN-MIRROR-QUERY-V1",
            "architecture_id": ARCHITECTURE_ID,
            "implementation_revision": IMPLEMENTATION_REVISION,
            "protocol_id": PROTOCOL_ID,
            "map_hash": ctx.map_sha256,
            "semantic_map_hash": semantic_map.semantic_map_hash,
            "selection_policy_sha256": r1.sha256_file(selection_policy_path),
            "source_candidate_id": selected["candidate_id"],
            "queries": [
                {
                    "query_id": f"r3-mirror-{index + 1}-{record['direction']}",
                    "start": record["start"],
                    "goal": record["goal"],
                    "category": "lane_mirror_regression",
                    "seed": int(config["experiment"]["seed"]),
                    "validation_status": "OFFLINE_DIRECTED_VIABILITY_CERTIFIED",
                }
                for index, record in enumerate(selected["directions"])
            ],
        }
        frozen["query_hash"] = canonical_hash(frozen["queries"])
        (output / "frozen_mirror_query_set.yaml").write_text(
            yaml.safe_dump(frozen, allow_unicode=True, sort_keys=False), encoding="utf-8",
        )


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    args = _parser().parse_args(arguments)
    overrides = json.loads(args.preference_policy_json)
    reproduction = "two_layer_v2_semantic_r3_benchmark " + " ".join(
        shlex.quote(value) for value in arguments
    )
    if args.mode in {"convert", "synthetic-smoke", "mirror-selection"} and (
        args.query_set is not None or args.generate_query_set
    ):
        raise SystemExit(
            "--query-set/--generate-query-set apply only to offline-diagnostic "
            "or real-ablation mode"
        )
    try:
        query_path, require_default_contract, legacy_stage5_gates = r2._real_query_set_selection(
            args.query_set, args.generate_query_set,
        )
    except ValueError as error:
        raise SystemExit(str(error)) from error
    with _r3_bindings():
        if args.mode == "convert":
            return r1.main(arguments)
        if args.mode == "synthetic-smoke":
            r1.run_synthetic_smoke(
                output=args.output_dir.resolve(), config_path=args.config.resolve(),
                preference_policy_overrides=overrides,
            )
            real = False
        else:
            if args.extracted_dir is None or args.semantic_map is None or args.topology_cache is None:
                raise SystemExit(
                    f"{args.mode} requires --extracted-dir, --semantic-map and --topology-cache"
                )
            common = {
                "extracted_dir": args.extracted_dir.resolve(),
                "semantic_map_path": args.semantic_map.resolve(),
                "topology_cache": args.topology_cache.resolve(),
                "output": args.output_dir.resolve(),
                "config_path": args.config.resolve(),
                "preference_policy_overrides": overrides,
            }
            if args.mode == "mirror-selection":
                r1._refuse_nonempty(common["output"])
                common["output"].mkdir(parents=True)
                with r2._query_set_binding(
                    r2.DEFAULT_QUERY_SET_PATH, require_default_contract=True,
                ):
                    _run_mirror_selection(
                        extracted_dir=common["extracted_dir"],
                        semantic_map_path=common["semantic_map_path"],
                        topology_cache=common["topology_cache"],
                        output=common["output"], config_path=common["config_path"],
                        selection_policy_path=args.mirror_selection_policy.resolve(),
                        preference_policy_overrides=overrides,
                    )
                real = False
            elif args.mode == "offline-diagnostic":
                r1._refuse_nonempty(common["output"])
                common["output"].mkdir(parents=True)
                if query_path is None:
                    _augment_offline_guide_diagnostics(
                        extracted_dir=common["extracted_dir"],
                        semantic_map_path=common["semantic_map_path"],
                        topology_cache=common["topology_cache"],
                        output=common["output"], config_path=common["config_path"],
                        preference_policy_overrides=overrides,
                    )
                else:
                    with r2._query_set_binding(
                        query_path, require_default_contract=require_default_contract,
                    ):
                        _augment_offline_guide_diagnostics(
                            extracted_dir=common["extracted_dir"],
                            semantic_map_path=common["semantic_map_path"],
                            topology_cache=common["topology_cache"],
                            output=common["output"], config_path=common["config_path"],
                            preference_policy_overrides=overrides,
                        )
                real = False
            else:
                arms = [value.strip() for value in args.arms.split(",") if value.strip()]
                unknown = sorted(set(arms) - set(r1.ARM_ORDER))
                if unknown:
                    raise SystemExit(f"unknown arms: {unknown}")
                if query_path is None:
                    r1.run_real_ablation(
                        **common, warmups=args.warmups, repetitions=args.repetitions,
                        ros_domain_id=args.ros_domain_id, arms=arms,
                        query_ids=[value.strip() for value in args.query_ids.split(",") if value.strip()] or None,
                    )
                else:
                    with r2._query_set_binding(
                        query_path,
                        require_default_contract=require_default_contract,
                    ):
                        r1.run_real_ablation(
                            **common, warmups=args.warmups, repetitions=args.repetitions,
                            ros_domain_id=args.ros_domain_id, arms=arms,
                            query_ids=[value.strip() for value in args.query_ids.split(",") if value.strip()] or None,
                        )
                real = True
        r2._postprocess(
            args.output_dir.resolve(), reproduction, real=real,
            legacy_stage5_gates=legacy_stage5_gates,
        )
        if not real:
            for artifact in ("synthetic_smoke.json", "direction_diagnostics.json"):
                path = args.output_dir.resolve() / artifact
                if path.exists():
                    payload = json.loads(path.read_text())
                    payload["schema_version"] = f"2A-V2-r3-{artifact.removesuffix('.json').replace('_', '-')}-v1"
                    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    print(f"2A-V2/r3 output: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
