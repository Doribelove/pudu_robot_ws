from pathlib import Path
from types import SimpleNamespace

from arena_evaluation import two_layer_formal_benchmark as formal


def test_formal_identity_and_fixed_protocol():
    assert formal.ARCHITECTURE_ID == "2A-V0"
    assert formal.IMPLEMENTATION_REVISION == "r3"
    assert formal.PROTOCOL_VERSION == "PLN-02-EXP-V1"
    assert formal.PADDING_SCHEDULE == (2.0, 4.0, 6.0)
    assert formal.CORRIDOR_SEMANTICS == "raw_map_smac_aligned"


def test_two_layer_call_annotation_disables_l2():
    row = {"l3_call_count": 1, "action_success": True, "final_valid_success": True,
           "result_code": "SUCCEEDED", "reason_code": "", "last_layer": "L3_PRIME",
           "cache_mode": "optimized", "topology_cache_hit": True}
    call = formal._annotate_call(row, {"stage": "L3_PRIME", "physical_backend_call_count": 1},
                                 SimpleNamespace(query_id="A2B-01"), Path("/tmp/formal-test"))
    assert call["architecture_id"] == "2A-V0"
    assert call["implementation_revision"] == "r3"
    assert call["l2_called"] is False
    assert call["l2_call_count"] == 0
    assert call["l3_call_count"] == 1


def test_metric_availability_is_explicit():
    parser = formal.build_parser()
    args = parser.parse_args(["--no-dynamic-obstacles"])
    assert args.cache_mode == "optimized"
    assert args.warmups == 3
    assert args.repetitions == 5


def test_historical_summary_selects_two_layer_row():
    historical = formal.ROOT / "experiments/layered_planner_benchmark/l1_l2_l3_vs_l1_l3prime_mentor_map_20260825_005_4x_area_20_v1/paired_summary.csv"
    summary = formal._historical_two_layer_summary(historical)
    assert summary["architecture"] == "two_layer"
    assert summary["final_valid_count"] == "13"
    assert summary["p50_ms"] == "1746.9264495"


def test_preflight_cache_build_count_is_configurable():
    parser = formal.build_parser()
    args = parser.parse_args(["--no-dynamic-obstacles", "--preflight-cache-build-count", "1"])
    assert args.preflight_cache_build_count == 1


def test_row_annotation_adds_protocol_aliases_without_changing_pose_values(tmp_path):
    query = SimpleNamespace(query_id="A2B-01", start=(1.0, 2.0, 0.1), goal=(3.0, 4.0, 0.2))
    row = formal._annotate_row(
        tmp_path,
        {"start_x": 1.0, "start_y": 2.0, "start_yaw": 0.1, "goal_x": 3.0, "goal_y": 4.0,
         "goal_yaw": 0.2, "query_hash": "qhash", "pipeline_wall_time_ms": 1.0,
         "pipeline_cpu_total_ms": 1.0, "planner_success": True, "final_valid_success": True,
         "l3_prime_call_count": 1, "l1_success": True, "static_footprint_valid": True,
         "kinematic_valid": True, "reverse_distance_m": 0.0, "heading_discontinuity_count": 0},
        query, {}, {"topology_cache_hit": True}, "optimized", 10.0,
    )
    assert row["query_sha256"] == "qhash"
    assert row["start"] == "[1.0,2.0,0.1]"
    assert row["goal"] == "[3.0,4.0,0.2]"
    assert row["heading_jump_count"] == 0
    assert row["reverse_length_m"] == 0.0
