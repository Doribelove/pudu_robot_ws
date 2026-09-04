import json
from pathlib import Path

import pytest
import yaml

from arena_evaluation import layered_2d_v1_pipeline as r1_pipeline
from arena_evaluation import two_layer_2d_v1_4x_dynamic_incremental_benchmark as benchmark
from arena_evaluation import two_layer_2d_v1_dynamic_incremental_benchmark as parent
from arena_evaluation import dynamic_incremental_value as value


@pytest.fixture(scope="module")
def prepared_4x():
    ctx, queries, metadata, artifact, topology_info = benchmark._load_4x_inputs()
    graph_view = r1_pipeline.build_static_topology_view(artifact)
    graph_view.metadata["topology_cache_key"] = topology_info["topology_cache_key"]
    graphs, pipeline = benchmark._build_query_graphs(
        graph_view, topology_info, ctx, queries, benchmark._load_static_reference(),
    )
    edge_cells = parent._edge_cells(graph_view)
    exclusive, any_witness, _reverse = parent._witness_maps(edge_cells)
    specs, selected = benchmark._build_scenarios(
        graphs, exclusive, any_witness, edge_count=len(edge_cells),
        seed=benchmark.DEFAULT_SEED, main_query_count=10,
    )
    return ctx, artifact, topology_info, graphs, edge_cells, specs, selected, pipeline


def test_4x_input_and_cache_binding_are_isolated():
    ctx, queries, metadata, artifact, info = benchmark._load_4x_inputs()
    assert ctx.map_id == "mentor_map_20260825_005_4x_area"
    assert artifact.free_mask.shape == (3024, 6574)
    assert len(artifact.graph.nodes) == 4376
    assert len(artifact.graph.edges) == 4562
    assert len(queries) == 20
    assert info["source_topology_cache_key"] != parent.FROZEN_TOPOLOGY_KEY
    assert info["topology_cache_key"] != parent.FROZEN_TOPOLOGY_KEY
    binding = info["cache_binding"]
    assert binding["shape"] == [3024, 6574]
    assert binding["implementation_revision"] == "r4"
    assert binding["map_hash"] == benchmark.sha256_file(benchmark.FOUR_X_MAP_PGM)


def test_absolute_and_ratio_change_points_are_frozen_and_deduplicated():
    assert benchmark.ABSOLUTE_POINTS == (1, 2, 5, 20, 100)
    points = benchmark.ratio_change_points(4562)
    assert [row["changed_edge_count"] for row in points] == [2, 4, 11, 42, 210]
    assert len({row["changed_edge_count"] for row in points}) == len(points)
    assert all(row["changed_edge_count"] >= 1 for row in points)


def test_4x_scenarios_cover_semantics_scales_and_ten_queries(prepared_4x):
    _ctx, _artifact, _info, _graphs, _cells, specs, selected, _pipeline = prepared_4x
    assert len(selected) == 10
    families = [spec.scale_family for spec in specs]
    assert families.count("absolute") == 5
    assert families.count("ratio") == 5
    categories = {spec.scenario.category for spec in specs}
    assert {"outside_path", "path_nonbridge_alternate", "moving_e1_e2_e3",
            "disappearance_recovery", "bridge_or_min_cut_no_route"}.issubset(categories)
    assert {spec.scenario.query_id for spec in specs if spec.scenario.analysis_group == "negative_control"} == {"A2B-07", "A2B-16"}
    assert any(spec.scenario.query_id == "A2B-19" and spec.scenario.analysis_group == "smac_long_tail_control" for spec in specs)


def test_persistent_dstar_oracle_parity_block_no_route_and_recovery(tmp_path, prepared_4x):
    ctx, artifact, _info, graphs, edge_cells, specs, _selected, _pipeline = prepared_4x
    spec = next(spec for spec in specs if spec.scenario.category == "bridge_or_min_cut_no_route")
    _path, payloads = parent._write_event_stream(
        tmp_path, spec.scenario, map_version=ctx.map_sha256,
        map_shape=artifact.free_mask.shape, seed=benchmark.DEFAULT_SEED,
    )
    rows = value.run_paired_episode(
        graphs[spec.scenario.query_id].template, payloads, edge_cells,
        map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
        arm_order=benchmark.PRIMARY_ARMS,
    )
    parent._annotate_rows(rows, spec.scenario, graphs[spec.scenario.query_id],
                          run_mode="measured", repetition=1,
                          topology_edge_count=len(edge_cells))
    correctness = benchmark._correctness_rows(rows)
    assert all(row["all_correct"] for row in correctness)
    inc = [row for row in rows if row["arm"] == value.INCREMENTAL_DSTAR]
    assert any(row["reachable"] is False and row["failure_code"] == "L1_NO_ROUTE" for row in inc)
    assert inc[-1]["reachable"] is True
    assert all(row["reinitialize_call_count"] == 0 and not row["implicit_reinitialize"] for row in inc)
    assert len({row["planner_identity_stable"] for row in inc[1:]}) == 1


def test_all_4x_scenarios_have_paired_oracle_parity_without_reinitialize(tmp_path, prepared_4x):
    ctx, artifact, _info, graphs, edge_cells, specs, _selected, _pipeline = prepared_4x
    correctness = []
    for index, spec in enumerate(specs):
        scenario = spec.scenario
        scenario_dir = tmp_path / f"{index:02d}"
        scenario_dir.mkdir()
        _path, payloads = parent._write_event_stream(
            scenario_dir, scenario, map_version=ctx.map_sha256,
            map_shape=artifact.free_mask.shape, seed=benchmark.DEFAULT_SEED,
        )
        rows = value.run_paired_episode(
            graphs[scenario.query_id].template, payloads, edge_cells,
            map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
            arm_order=benchmark.PRIMARY_ARMS if index % 2 == 0 else tuple(reversed(benchmark.PRIMARY_ARMS)),
        )
        parent._annotate_rows(
            rows, scenario, graphs[scenario.query_id], run_mode="measured",
            repetition=1, topology_edge_count=len(edge_cells),
        )
        correctness.extend(benchmark._correctness_rows(rows))
    assert len(correctness) == len(specs) * 21
    assert all(row["all_correct"] for row in correctness)


def test_stage_a_failure_can_never_admit_ros():
    assert benchmark._stage_b_allowed({"stage_a_pass": False}, False) is False
    assert benchmark._stage_b_allowed({"stage_a_pass": True}, True) is False
    assert benchmark._stage_b_allowed({"stage_a_pass": True}, False) is True


def test_formal_artifact_and_source_snapshot_validation(tmp_path):
    for name in benchmark.REQUIRED:
        (tmp_path / name).write_text("x\n", encoding="utf-8")
    events = tmp_path / "dynamic_event_streams"
    events.mkdir()
    (events / "S.json").write_text(json.dumps({"snapshots": [{}] * 21}), encoding="utf-8")
    source = tmp_path / "source_snapshot" / "source.py"
    source.parent.mkdir()
    source.write_text("pass\n", encoding="utf-8")
    manifest = {"files": [{"snapshot": "source_snapshot/source.py",
                            "sha256": benchmark.sha256_file(source)}]}
    (tmp_path / "source_snapshot_manifest.yaml").write_text(
        yaml.safe_dump(manifest), encoding="utf-8")
    result = benchmark._validate_artifacts(tmp_path)
    assert result["passed"] is True
    assert not result["bad_source_hashes"]
