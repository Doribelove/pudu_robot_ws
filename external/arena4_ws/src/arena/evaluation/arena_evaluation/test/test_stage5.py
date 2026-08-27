from pathlib import Path

import pandas as pd
import pytest
import yaml

from arena_evaluation.planner_benchmark.config import load_queries
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.planner_benchmark.runner import BenchmarkInputError, run_benchmark
from arena_evaluation.stage5_report import build_stage5_report, validity_columns
from arena_evaluation.topology import astar_grid, load_topology
from arena_evaluation.topology_cli import run_topology_benchmark, _assert_static_cli, corridor_padding_schedule


FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]


def _workspace_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "experiments/maps/hospital_005/map.yaml").exists():
            return parent
    raise RuntimeError("Stage 5 workspace artifacts are unavailable")


def test_v2_q00_q05_endpoints_meet_fixed_clearance():
    root = _workspace_root()
    hospital_map = HospitalMap.load(root / "experiments/maps/hospital_005/map.yaml")
    _, queries = load_queries(root / "experiments/planner_benchmark/hospital_005/queries_v2.yaml")
    selected = {query.query_id: query for query in queries if query.query_id in {"q00", "q05"}}
    assert set(selected) == {"q00", "q05"}
    for query in selected.values():
        validation = hospital_map.validate_query(query, FOOTPRINT, 0.5, allow_unknown=False)
        assert validation.validation_status == "VALID"
        assert validation.start_clearance_m >= 0.5
        assert validation.goal_clearance_m >= 0.5


def test_q04_conservative_topology_mismatch_is_reproducible():
    root = _workspace_root()
    hospital_map = HospitalMap.load(root / "experiments/maps/hospital_005/map.yaml")
    artifact = load_topology(
        root / "experiments/topology_benchmark/hospital_005/stage5_full_v2/topology",
        hospital_map, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    _, queries = load_queries(root / "experiments/planner_benchmark/hospital_005/queries_v2.yaml")
    query = next(query for query in queries if query.query_id == "q04")
    start = hospital_map.world_to_cell(query.start[0], query.start[1])
    goal = hospital_map.world_to_cell(query.goal[0], query.goal[1])
    assert astar_grid(artifact.free_mask, start, goal) is None


def test_action_success_and_static_validity_are_separate():
    frame = validity_columns(pd.DataFrame([
        {"result_code": "SUCCEEDED", "footprint_collision_count": 0},
        {"result_code": "SUCCEEDED", "footprint_collision_count": 3},
        {"result_code": "ACTION_ABORTED", "footprint_collision_count": None},
    ]))
    assert frame["action_success"].tolist() == [True, True, False]
    assert frame["static_footprint_valid"].tolist() == [True, False, False]
    assert frame["final_valid_success"].tolist() == [True, False, False]


def test_dynamic_protocols_are_rejected_before_execution(tmp_path):
    protocol = tmp_path / "dynamic.yaml"
    protocol.write_text(yaml.safe_dump({"dynamic_obstacles": True}))
    with pytest.raises(BenchmarkInputError, match="dynamic_obstacles"):
        run_benchmark(
            protocol_path=protocol, queries_path=tmp_path / "missing.yaml",
            output_dir=tmp_path / "planner", planners=["navfn"],
            config_variants=["product"], warmups=0, repetitions=0, timeout=1.0,
        )
    with pytest.raises(ValueError, match="dynamic_obstacles"):
        run_topology_benchmark(
            map_name="hospital_005", protocol_path=protocol,
            queries_path=tmp_path / "missing.yaml", output_dir=tmp_path / "topology",
            topology_dir=None, modes=["full_grid"], query_ids=None, repetitions=1,
            build_only=True, corridor_padding_m=1.0, attach_radius_m=5.0,
        )


def test_stage5_report_refuses_to_overwrite_existing_output(tmp_path):
    output = tmp_path / "existing"
    output.mkdir()
    (output / "sentinel").write_text("keep")
    with pytest.raises(ValueError, match="refusing to overwrite"):
        build_stage5_report(
            tmp_path / "missing-planner", tmp_path / "missing-topology",
            tmp_path / "missing-protocol", tmp_path / "missing-queries", output,
        )
    assert (output / "sentinel").read_text() == "keep"


def test_stage6_cli_requires_static_marker():
    from arena_evaluation.topology_cli import main
    assert main(["--output-dir", "x"]) == 2


def test_stage6_output_modes_are_explicit_and_non_overwriting(tmp_path):
    # Existing outputs are always rejected by the topology runner.
    output = tmp_path / "existing"
    output.mkdir()
    (output / "query_runs.csv").write_text("sentinel")
    with pytest.raises(ValueError, match="refusing to overwrite"):
        run_topology_benchmark(
            map_name="hospital_005", protocol_path=Path("experiments/planner_benchmark/hospital_005/topology_protocol_v2.yaml"),
            queries_path=Path("experiments/planner_benchmark/hospital_005/queries_v2.yaml"), output_dir=output,
            topology_dir=Path("experiments/topology_benchmark/hospital_005/stage5_full_v2/topology"),
            modes=["full_grid"], query_ids=["q00"], repetitions=1, build_only=False,
            corridor_padding_m=1.0, attach_radius_m=5.0,
        )
