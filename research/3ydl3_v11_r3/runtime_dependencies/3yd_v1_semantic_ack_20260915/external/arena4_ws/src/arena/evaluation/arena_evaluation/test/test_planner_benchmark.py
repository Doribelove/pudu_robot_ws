from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml

from arena_evaluation.planner_benchmark.cross_report import build_cross_report
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.planner_benchmark.models import Query, RESULT_CODES
from arena_evaluation.planner_benchmark.path_metrics import analyze_path, interpolate_path, path_length
from arena_evaluation.planner_benchmark.resources import discover_process, read_snapshot
from arena_evaluation.planner_benchmark.runner import _action_status_text, _write_rows, classify_action_result, validate_queries
from arena_evaluation.fixed_resolution_map import prepare_fixed_resolution_map
from arena_evaluation.planner_benchmark.config import stack_parameters


FOOTPRINT = [[0.5, 0.3], [0.5, -0.3], [-0.5, -0.3], [-0.5, 0.3]]


def _write_map(tmp_path: Path, pixels: np.ndarray, *, resolution: float = 1.0) -> Path:
    from PIL import Image
    import yaml

    image_path = tmp_path / "test.pgm"
    Image.fromarray(pixels.astype(np.uint8)).save(image_path)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(yaml.safe_dump({
        "image": image_path.name,
        "resolution": resolution,
        "origin": [-2.0, -2.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }))
    return yaml_path


def test_world_pixel_conversion_flips_image_y(tmp_path):
    map_yaml = _write_map(tmp_path, np.full((4, 4), 254, dtype=np.uint8))
    hospital_map = HospitalMap.load(map_yaml)
    assert hospital_map.world_to_cell(-1.5, -1.5) == (3, 0)
    assert hospital_map.world_to_cell(-1.5, 1.5) == (0, 0)
    assert hospital_map.cell_to_world((3, 0)) == pytest.approx((-1.5, -1.5))


def test_fixed_resolution_map_exact_replication_and_metadata(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    pixels = np.array([[0, 127], [205, 254]], dtype=np.uint8)
    source_image = source / "map.pgm"
    from PIL import Image
    Image.fromarray(pixels).save(source_image, format="PPM")
    (source / "map.yaml").write_text(yaml.safe_dump({
        "image": "map.pgm", "resolution": 0.1, "origin": [-1.0, -1.0, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }))
    output = prepare_fixed_resolution_map(source / "map.yaml", 0.05, tmp_path / "derived")
    derived = np.asarray(Image.open(output / "map.pgm").convert("L"))
    assert derived.shape == (4, 4)
    assert np.array_equal(derived, np.repeat(np.repeat(pixels, 2, axis=0), 2, axis=1))
    metadata = yaml.safe_load((output / "metadata.yaml").read_text())
    assert metadata["target_size"] == [4, 4]
    assert metadata["physical_extent_m"] == [0.2, 0.2]
    assert metadata["dynamic_obstacles"] is False
    assert metadata["derived_occupied_cell_count"] == metadata["occupied_cell_count"] * 4
    assert metadata["derived_unknown_cell_count"] == metadata["unknown_cell_count"] * 4


def test_stack_parameters_uses_protocol_cells_and_origin():
    params = stack_parameters(
        protocol={"resolution": 0.05, "width_cells": 1600, "height_cells": 1200,
                  "origin": [-3.0, -4.0, 0.0], "footprint": [[0.1, 0.1]],
                  "variants": {"product": {}}},
        planner_config={"config_variant": "product", "planner_plugins": ["GridBased"]},
    )
    costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
    assert costmap["width"] == pytest.approx(80.0)
    assert costmap["height"] == pytest.approx(60.0)
    assert costmap["origin_x"] == -3.0
    assert costmap["origin_y"] == -4.0


def test_footprint_collision_and_clearance(tmp_path):
    pixels = np.full((8, 8), 254, dtype=np.uint8)
    pixels[4, 4] = 0
    hospital_map = HospitalMap.load(_write_map(tmp_path, pixels))
    assert hospital_map.footprint_collision((2.5, 1.5, 0.0), FOOTPRINT)
    assert not hospital_map.footprint_collision((-0.5, 0.5, 0.0), FOOTPRINT)
    assert hospital_map.clearance(-0.5, 0.5) > 0


def test_interpolation_and_xy_length():
    points = [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 1.0, "y": 0.0, "yaw": 0.0}]
    sampled = interpolate_path(points, 0.25)
    assert len(sampled) == 5
    assert path_length(sampled) == pytest.approx(1.0)


def test_path_metrics_rotation_and_reverse(tmp_path):
    hospital_map = HospitalMap.load(_write_map(tmp_path, np.full((20, 20), 254, dtype=np.uint8), resolution=0.2))
    query = Query("q", [0.0, 0.0, math.pi], [1.0, 0.0, math.pi])
    points = [
        {"x": 0.0, "y": 0.0, "yaw": math.pi},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 1.0, "y": 0.0, "yaw": 0.0},
    ]
    metric = analyze_path(
        run_id="r", query=query, planner_id="smac", config_variant="normalized",
        points=points, hospital_map=hospital_map, footprint=[[0.05, 0.05], [0.05, -0.05], [-0.05, -0.05], [-0.05, 0.05]],
        preferred_minimum_turning_radius=0.4, allow_unknown=False,
    )
    assert metric.in_place_rotation_count == 1
    assert metric.preferred_radius_violation_count == 0
    assert metric.reverse_distance_m == pytest.approx(0.0)


def test_proc_stat_and_action_mapping():
    snapshot = read_snapshot(__import__("os").getpid())
    assert snapshot is not None
    assert snapshot.cpu_user_ms is not None
    assert _action_status_text(4) == "SUCCEEDED"
    assert _action_status_text(6) == "ABORTED"


def test_invalid_start_and_goal_are_not_silently_replaced(tmp_path):
    pixels = np.full((8, 8), 254, dtype=np.uint8)
    pixels[0, 0] = 0
    hospital_map = HospitalMap.load(_write_map(tmp_path, pixels))
    start_invalid = Query("bad-start", [-100.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    goal_invalid = Query("bad-goal", [0.0, 0.0, 0.0], [100.0, 0.0, 0.0])
    assert hospital_map.validate_query(start_invalid, FOOTPRINT, 0.0, True).start_status == "OUT_OF_BOUNDS"
    assert hospital_map.validate_query(goal_invalid, FOOTPRINT, 0.0, True).goal_status == "OUT_OF_BOUNDS"


def test_blocked_query_reports_disconnected(tmp_path):
    pixels = np.full((8, 8), 254, dtype=np.uint8)
    pixels[:, 4] = 0
    hospital_map = HospitalMap.load(_write_map(tmp_path, pixels))
    query = Query("disconnected", [-1.5, 0.5, 0.0], [1.5, 0.5, 0.0])
    validation = hospital_map.validate_query(query, [[0.05, 0.05], [0.05, -0.05], [-0.05, -0.05], [-0.05, 0.05]], 0.0, False)
    assert validation.validation_status == "INVALID"
    assert not validation.connected


def test_empty_path_has_no_false_quality_metrics(tmp_path):
    hospital_map = HospitalMap.load(_write_map(tmp_path, np.full((8, 8), 254, dtype=np.uint8)))
    metric = analyze_path(
        run_id="empty", query=Query("q", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]), planner_id="navfn", config_variant="product",
        points=[], hospital_map=hospital_map, footprint=[[0.05, 0.05], [0.05, -0.05], [-0.05, -0.05], [-0.05, 0.05]],
        preferred_minimum_turning_radius=0.4, allow_unknown=False,
    )
    assert metric.path_length_m is None
    assert metric.footprint_collision_count == 0


def test_yaw_interpolation_wraps_short_way():
    points = [{"x": 0.0, "y": 0.0, "yaw": 3.1}, {"x": 1.0, "y": 0.0, "yaw": -3.1}]
    sampled = interpolate_path(points, 0.25)
    assert abs(sampled[2]["yaw"]) > 3.0


def test_clearance_percentiles_are_monotonic(tmp_path):
    hospital_map = HospitalMap.load(_write_map(tmp_path, np.full((20, 20), 254, dtype=np.uint8), resolution=0.2))
    query = Query("q", [-1.0, -1.0, 0.0], [1.0, 1.0, 0.0])
    points = [{"x": -1.0, "y": -1.0, "yaw": 0.0}, {"x": 1.0, "y": 1.0, "yaw": 0.0}]
    metric = analyze_path(
        run_id="clear", query=query, planner_id="navfn", config_variant="product", points=points,
        hospital_map=hospital_map, footprint=[[0.05, 0.05], [0.05, -0.05], [-0.05, -0.05], [-0.05, 0.05]],
        preferred_minimum_turning_radius=0.4, allow_unknown=False,
    )
    assert metric.minimum_clearance_m <= metric.clearance_p05_m <= metric.clearance_p50_m


def test_direction_switch_and_reverse_segment(tmp_path):
    hospital_map = HospitalMap.load(_write_map(tmp_path, np.full((20, 20), 254, dtype=np.uint8), resolution=0.2))
    query = Query("reverse", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    points = [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": -0.5, "y": 0.0, "yaw": 0.0}, {"x": 0.0, "y": 0.0, "yaw": 0.0}]
    metric = analyze_path(
        run_id="reverse", query=query, planner_id="smac", config_variant="normalized", points=points,
        hospital_map=hospital_map, footprint=[[0.05, 0.05], [0.05, -0.05], [-0.05, -0.05], [-0.05, 0.05]],
        preferred_minimum_turning_radius=0.4, allow_unknown=False,
    )
    assert metric.reverse_distance_m > 0
    assert metric.direction_switch_count == 1


def test_result_codes_cover_timeout_and_empty_path():
    assert classify_action_result(4, path_point_count=0) == "EMPTY_PATH"
    assert classify_action_result(4, path_point_count=2) == "SUCCEEDED"
    assert classify_action_result(0, path_point_count=0) == "EXCEPTION"
    assert "CLIENT_TIMEOUT" in RESULT_CODES


def test_missing_proc_is_structured():
    assert discover_process("planner_server", cmdline_contains="benchmark-id-that-does-not-exist") is None


def test_csv_schema_is_key_based(tmp_path):
    path = tmp_path / "rows.csv"
    _write_rows(path, [{"run_id": "r1", "query_id": "q0", "result_code": "SUCCEEDED"}])
    header = path.read_text().splitlines()[0].split(",")
    assert header == ["run_id", "query_id", "result_code"]


def test_validate_queries_exposes_variant(tmp_path):
    hospital_map = HospitalMap.load(_write_map(tmp_path, np.full((8, 8), 254, dtype=np.uint8)))
    protocol = {"footprint": [[0.05, 0.05], [0.05, -0.05], [-0.05, -0.05], [-0.05, 0.05]], "minimum_endpoint_clearance_m": 0.0, "variants": {"normalized": {"allow_unknown": False}}}
    validation = validate_queries(protocol=protocol, queries=[Query("q", [-1.0, -1.0, 0.0], [0.0, 0.0, 0.0])], hospital_map=hospital_map, config_variants=["normalized"])
    assert validation[0].config_variant == "normalized"


def test_cross_report_uses_measured_rows_and_recomputes_path_ratios(tmp_path):
    def write_run(directory, planner_id, lengths, *, failed=False, collision_index=None):
        directory.mkdir()
        run_rows = [{
            "run_id": f"{planner_id}-warmup", "query_id": "q00", "planner_id": planner_id,
            "config_variant": "product", "run_mode": "warmup", "result_code": "SUCCEEDED",
            "planning_time_ms": 999.0,
        }]
        metric_rows = []
        for index, length in enumerate(lengths):
            run_id = f"{planner_id}-{index}"
            result_code = "ACTION_ABORTED" if failed and index == len(lengths) - 1 else "SUCCEEDED"
            run_rows.append({
                "run_id": run_id, "query_id": "q00", "planner_id": planner_id,
                "config_variant": "product", "run_mode": "measured", "result_code": result_code,
                "planning_time_ms": float(index + 1),
            })
            if result_code == "SUCCEEDED":
                metric_rows.append({
                    "run_id": run_id, "query_id": "q00", "planner_id": planner_id,
                    "config_variant": "product", "path_length_m": length,
                    "footprint_collision_count": int(index == collision_index),
                })
        pd.DataFrame(run_rows).to_csv(directory / "planner_runs.csv", index=False)
        pd.DataFrame(metric_rows).to_csv(directory / "path_metrics.csv", index=False)

    write_run(tmp_path / "stage3_navfn_product", "navfn_astar", [10.0, 10.0])
    write_run(
        tmp_path / "stage3_smac_product", "smac_hybrid", [8.0, 9.0, 10.0],
        failed=True, collision_index=0,
    )
    output = build_cross_report(tmp_path)

    summary = pd.read_csv(output / "baseline_summary.csv")
    navfn_time = summary[(summary["planner"] == "navfn") & (summary["metric"] == "planning_time_ms")].iloc[0]
    assert navfn_time["count"] == 2
    by_query = pd.read_csv(output / "baseline_by_query.csv")
    smac_navfn_ratio = by_query[
        (by_query["planner"] == "smac_hybrid") & (by_query["metric"] == "length_over_navfn")
    ].iloc[0]
    assert smac_navfn_ratio["mean"] == pytest.approx(0.85)
    navfn_shortest_ratio = by_query[
        (by_query["planner"] == "navfn")
        & (by_query["metric"] == "length_over_shortest_observed_valid")
    ].iloc[0]
    assert navfn_shortest_ratio["mean"] == pytest.approx(10.0 / 9.0)
    smac_shortest_ratio = by_query[
        (by_query["planner"] == "smac_hybrid")
        & (by_query["metric"] == "length_over_shortest_observed_valid")
    ].iloc[0]
    assert smac_shortest_ratio["count"] == 1
    assert smac_shortest_ratio["mean"] == pytest.approx(1.0)

    failures = pd.read_csv(output / "baseline_failure_summary.csv")
    assert failures[["query_id", "run_mode", "result_code", "count"]].to_dict("records") == [{
        "query_id": "q00", "run_mode": "measured", "result_code": "ACTION_ABORTED", "count": 1,
    }]
