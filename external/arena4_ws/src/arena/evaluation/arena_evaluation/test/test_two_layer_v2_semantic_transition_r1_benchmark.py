from __future__ import annotations

import json
import subprocess
import sys

import pytest

from arena_evaluation import two_layer_v2_semantic_transition_r1_benchmark as benchmark


def test_online_invocation_fails_before_creating_output(tmp_path, capsys):
    output = tmp_path / "must_not_be_created"
    result = benchmark.main(["--mode", "real-ablation", "--output-dir", str(output)])
    report = json.loads(capsys.readouterr().out)
    assert result == 2
    assert not output.exists()
    assert report["status"] == "NOT_RUN_OFFLINE_GATE_FAILED"
    assert report["online_adapter_status"] == "NOT_IMPLEMENTED_ONLINE_ADAPTER"
    assert report["online_started"] is False
    assert report["promotion_gate_passed"] is False


@pytest.mark.parametrize("payload", [{}, [], {"gate_passed": True}, {
    "gate_passed": True, "contract_revision": benchmark.CONTRACT_REVISION,
    "all_targeted_passed": True,
    "query_ids": list(benchmark.TARGETED_QUERY_IDS),
}])
def test_external_gate_claim_cannot_enable_online_or_promotion(tmp_path, payload):
    gate = tmp_path / "claimed_gate.json"
    gate.write_text(json.dumps(payload))
    report = benchmark.disabled_status(gate)
    assert report["offline_gate_status"] == "UNVERIFIED_EXTERNAL_REPORT"
    assert report["online_started"] is False
    assert report["promotion_gate_passed"] is False
    assert "verified_targeted_offline_3_of_3" in report["missing_gates"]
    assert "same_round_48_bin_three_arm_comparison" in report["missing_gates"]
    assert "selected8_all_queries_all_measured_repeats" in report["missing_gates"]
    assert "per_repeat_e0_success_retention" in report["missing_gates"]


def test_invalid_gate_is_fail_closed(tmp_path):
    gate = tmp_path / "invalid.json"
    gate.write_text("not-json")
    report = benchmark.disabled_status(gate)
    assert report["offline_gate_status"] == "UNREADABLE_OR_INVALID"
    assert report["online_started"] is False
    assert report["promotion_gate_passed"] is False


def test_help_and_import_do_not_load_planning_or_ros_modules():
    check = subprocess.run(
        [sys.executable, "-c", (
            "import sys; "
            "from arena_evaluation import two_layer_v2_semantic_transition_r1_benchmark as b; "
            "assert 'rclpy' not in sys.modules; "
            "assert 'arena_evaluation.two_layer_v2_semantic_r2_benchmark' not in sys.modules; "
            "b.main(['--help'])"
        )],
        check=False, capture_output=True, text=True,
    )
    assert check.returncode == 0, check.stderr
    assert "never starts ROS" in check.stdout
