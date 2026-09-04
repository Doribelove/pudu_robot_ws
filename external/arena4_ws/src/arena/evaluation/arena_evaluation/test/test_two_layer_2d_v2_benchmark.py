from pathlib import Path

import yaml

from arena_evaluation import two_layer_2d_v2_static_benchmark as static
from arena_evaluation import two_layer_2d_v1_4x_dynamic_incremental_benchmark as dynamic


def test_config_contract_and_candidate_status():
    config = yaml.safe_load(static.CONFIG.read_text())
    assert config["architecture_id"] == "2D-V2"
    assert config["implementation_revision"] == "r0-enhanced-runtime-v1"
    assert config["status"] == "candidate"
    assert config["runtime"]["fixed_settle_cycles_after_ack"] == 0
    assert config["smac"]["angle_quantization_bins"] == 48


def test_stage_a_failure_hard_blocks_stage_b():
    assert dynamic._stage_b_allowed({"stage_a_pass": False}, False) is False
    assert dynamic._stage_b_allowed({"stage_a_pass": True}, True) is False


def test_static_artifact_validator_detects_missing(tmp_path: Path):
    result = static._artifact_validation(tmp_path)
    assert not result["passed"]
    assert "final_report.md" in result["missing"]
