import csv
import json

import yaml

from arena_evaluation import two_layer_2d_v1_dynamic_incremental_benchmark as benchmark


def test_cli_contract_and_formal_repetition_guard(tmp_path):
    args = benchmark.build_parser().parse_args(["--stage-a-only"])
    assert args.repetitions == 20
    assert args.main_query_count == 8
    assert args.ros_domain_id == 97
    assert args.stage_a_only is True


def test_formal_csv_manifest_and_source_snapshot_helpers(tmp_path):
    rows_path = tmp_path / "paired_algorithm_runs.csv"
    benchmark._write_csv(rows_path, [{
        "arm": "incremental_dstar", "changed_edge_ids": ["a"],
        "snapshot_id": "S1",
    }])
    with rows_path.open(newline="", encoding="utf-8") as handle:
        row = next(csv.DictReader(handle))
    assert json.loads(row["changed_edge_ids"]) == ["a"]

    event = tmp_path / "event.json"
    event.write_text('{"snapshots": []}\n', encoding="utf-8")
    snapshot = benchmark._snapshot_sources(tmp_path, [event])
    manifest = yaml.safe_load((tmp_path / "source_snapshot_manifest.yaml").read_text())
    assert snapshot["file_count"] == manifest["file_count"]
    assert snapshot["file_count"] >= 2
    for item in manifest["files"]:
        copied = tmp_path / item["snapshot"]
        assert copied.is_file()
        assert benchmark.sha256_file(copied) == item["sha256"]


def test_event_stream_has_s0_through_s20_and_fixed_snapshot_identity(tmp_path):
    scenario = benchmark.Scenario(
        "DYN-TEST", "A2B-01", "changed_edges_5", "main",
        ("a", "b", "c", "d", "e"),
        {"a": (0, 0), "b": (0, 1), "c": (0, 2), "d": (0, 3), "e": (0, 4)},
    )
    path, payloads = benchmark._write_event_stream(
        tmp_path, scenario, map_version="map-v1", map_shape=(2, 5), seed=7,
    )
    saved = json.loads(path.read_text())
    assert len(payloads) == 21
    assert [item["snapshot_id"] for item in saved["snapshots"]] == [f"DYN-TEST-S{i}" for i in range(21)]
    assert all(json.loads(payload)["snapshot_hash"] for payload in payloads)


def test_frozen_r2_generates_required_dynamic_coverage_and_oracle_parity(tmp_path):
    frozen = benchmark._load_frozen_validity()
    queries, _metadata = benchmark.task_source._load_tasks()
    ctx = benchmark.task_source._context()
    artifact, topology_info, _audit = benchmark.r2_benchmark._load_frozen_r1_topology(
        ctx, benchmark.DEFAULT_CACHE_ROOT,
    )
    graph_view = benchmark.r1_pipeline.build_static_topology_view(artifact)
    graph_view.metadata["topology_cache_key"] = topology_info["topology_cache_key"]
    query_graphs = benchmark._build_query_graphs(
        graph_view, topology_info, ctx, queries, frozen,
    )
    edge_cells = benchmark._edge_cells(graph_view)
    exclusive, any_witness, _reverse = benchmark._witness_maps(edge_cells)
    scenarios = benchmark._build_scenarios(
        query_graphs, frozen, exclusive, any_witness,
        seed=benchmark.DEFAULT_SEED, main_query_count=8,
    )
    main = [scenario for scenario in scenarios if scenario.analysis_group == "main"]
    assert len(main) == 8
    assert len({scenario.query_id for scenario in main}) == 8
    assert {len(scenario.target_edges) for scenario in main}.issuperset({1, 5, 20, 100})
    assert any(scenario.min_cut_size > 0 for scenario in main)
    assert {scenario.query_id for scenario in scenarios if scenario.analysis_group == "negative_control"} == {"A2B-07", "A2B-16"}
    assert any(scenario.query_id == "A2B-19" and scenario.analysis_group == "smac_long_tail_control" for scenario in scenarios)

    scenario = next(item for item in main if item.min_cut_size > 0)
    _path, payloads = benchmark._write_event_stream(
        tmp_path, scenario, map_version=ctx.map_sha256,
        map_shape=artifact.free_mask.shape, seed=benchmark.DEFAULT_SEED,
    )
    rows = benchmark.value.run_paired_episode(
        query_graphs[scenario.query_id].template, payloads, edge_cells,
        map_version=ctx.map_sha256, map_shape=artifact.free_mask.shape,
    )
    benchmark._annotate_rows(
        rows, scenario, query_graphs[scenario.query_id], run_mode="measured",
        repetition=1, topology_edge_count=len(edge_cells),
    )
    correctness = benchmark._correctness_rows(rows)
    assert all(row["all_correct"] for row in correctness)
    assert any(not row["reachable"] for row in rows)
