import pytest
import hashlib

from arena_evaluation import two_layer_2d_v1_r2_formal_benchmark as formal


def test_r2_config_freezes_the_r1_architecture_and_static_protocol():
    config = formal._load_r2_config()
    assert config["architecture_id"] == "2D-V1"
    assert config["implementation_revision"] == "r2"
    assert config["topology"]["refined_topology"] is False
    assert config["topology"]["search"] == "graph_dstar_lite"
    assert config["layers"]["l2_enabled"] is False
    assert config["layers"]["motion_model"] == "DUBIN"
    assert config["formal_experiment"]["dynamic_obstacles"] is False
    assert config["formal_experiment"]["smac_parameters_changed_from_r1"] is False


def test_cli_requires_explicit_static_experiment_acknowledgement():
    parser = formal.build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args([])
    args = parser.parse_args(["--no-dynamic-obstacles"])
    assert args.no_dynamic_obstacles is True
    assert args.warmups == 3
    assert args.repetitions == 5


def test_online_accounting_adds_only_non_overlapping_aggregates():
    reset = {
        "query_session_reset_ms": 10.0,
        "query_session_reset_costmap_clear_ms": 1000.0,
    }
    diagnostics = {
        "l1_total_time_ms": 20.0,
        "attachment_lookup_time_ms": 1000.0,
        "dstar_lite_search_time_ms": 1000.0,
        "route_polyline_construction_time_ms": 3.0,
        "corridor_mask_total_time_ms": 4.0,
        "corridor_mask_dilation_ms": 1000.0,
        "endpoint_diagnostics_time_ms": 5.0,
        "local_map_update_ms": 6.0,
        "local_costmap_clear_ms": 1000.0,
        "l3_action_wall_ms": 7.0,
        "planning_time_ms": 1000.0,
        "path_within_mask_check_ms": 8.0,
        "pipeline_validation_time_ms": 9.0,
        "canonical_validation_time_ms": 1000.0,
        "dynamic_collision_diagnostics_time_ms": 1.0,
    }
    assert formal._online_accounted_ms(reset, diagnostics, 2.0, 1.0) == 76.0


def test_phase_summary_has_p50_p95_p99_for_every_declared_phase():
    rows = [
        {"run_mode": "warmup", "online_wall_ms": 9999.0},
        {"run_mode": "measured", **{field: 1.0 for field in formal.PHASE_FIELDS}},
        {"run_mode": "measured", **{field: 3.0 for field in formal.PHASE_FIELDS}},
    ]
    summary = formal._phase_summary(rows)
    assert [row["phase"] for row in summary] == list(formal.PHASE_FIELDS)
    assert all(row["p50_ms"] == 2.0 for row in summary)
    assert all(row["p95_ms"] is not None for row in summary)
    assert all(row["p99_ms"] is not None for row in summary)


def test_source_snapshot_preserves_exact_bytes(tmp_path):
    source = tmp_path / "input.py"
    source.write_bytes(b"print('frozen')\n")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    output = tmp_path / "experiment"
    output.mkdir()
    result = formal._snapshot_sources(output, {str(source): digest}, "aggregate")
    snapshot = output / result["files"][0]["snapshot_path"]
    assert snapshot.read_bytes() == source.read_bytes()
    assert result["files"][0]["sha256"] == digest
