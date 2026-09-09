from pathlib import Path

import numpy as np
import pytest
import yaml

from arena_evaluation import semantic_v3_ack_r14
from arena_evaluation.semantic_parking_reference_r15 import (
    METHOD_ID,
    lexicographic_target_astar,
)
from arena_evaluation.semantic_v3_ack_r15 import SingleObservationExactAckSessionR15
from arena_evaluation.two_layer_v3_semantic_r15_fast import (
    ARCHITECTURE_ID,
    IMPLEMENTATION_REVISION,
    PROTOCOL_ID,
    _ack_gate,
    _load,
    percentile,
)


CONFIG = Path(__file__).resolve().parents[1] / "config/two_layer_v3_semantic_r15_fast.yaml"


def test_r15_identity_and_frozen_parent_hashes():
    config, algorithm, _parent_algorithm, _parent, selected8, expanded32, resolved = _load(CONFIG)
    assert resolved == CONFIG.resolve()
    assert config["architecture_id"] == ARCHITECTURE_ID == "2A-V3"
    assert config["implementation_revision"] == IMPLEMENTATION_REVISION
    assert config["protocol_id"] == PROTOCOL_ID
    assert algorithm["frozen_bindings"]["yaw_bins"] == 48
    assert algorithm["frozen_bindings"]["motion_model"] == "DUBIN"
    assert algorithm["frozen_bindings"]["allow_reverse"] is False
    assert algorithm["frozen_bindings"]["allow_in_place_rotation"] is False
    assert selected8.is_file() and expanded32.is_file()


def test_fast_protocol_has_hard_stop_and_one_formal_run():
    config = yaml.safe_load(CONFIG.read_text())
    protocol = config["fast_iteration"]
    assert protocol["early_stop_on_first_hard_failure"] is True
    assert protocol["full_regression_only_after_candidate_gate"] is True
    assert protocol["formal_run_limit_per_revision"] == 1
    assert protocol["ack_stress"]["exact_observations_per_change"] == 1
    assert protocol["parking_oracle"]["require_all_three"] is True
    assert protocol["formal32"]["enabled_only_after_ack_parking_sentinel_medium_gates"] is True


def test_lexicographic_parking_solver_prevents_diagonal_corner_cut():
    free = np.array([[True, False], [False, True]], dtype=bool)
    deviation = np.zeros((2, 2), dtype=np.float32)
    path, expanded, objective = lexicographic_target_astar(
        free, deviation, deviation <= 0.25, (0, 0), (1, 1), 20,
    )
    assert path is None
    assert expanded == 1
    assert objective is None


def test_lexicographic_parking_solver_prefers_center_band_over_shortcut():
    free = np.ones((5, 7), dtype=bool)
    # The direct middle row is outside the target.  Rows 1 and 3 are target
    # corridors, so the target-first objective must take a longer route.
    deviation = np.ones((5, 7), dtype=np.float32)
    deviation[1, :] = 0.0
    deviation[3, :] = 0.0
    deviation[2, 0] = deviation[2, -1] = 0.0
    target = deviation <= 0.25
    path, _expanded, objective = lexicographic_target_astar(
        free, deviation, target, (2, 0), (2, 6), 1000,
    )
    assert path is not None and objective is not None
    assert objective[0] == pytest.approx(0.0)
    assert any(row != 2 for row, _col in path[1:-1])


def test_lexicographic_parking_solver_mirror_has_mirrored_result():
    free = np.zeros((7, 9), dtype=bool)
    free[1:6, 1:8] = True
    # A unique target corridor avoids relying on a tie that raster ordering
    # may legitimately resolve differently after reflection.
    deviation = np.ones_like(free, dtype=np.float32)
    deviation[1, 1:8] = 0.0
    deviation[1:6, 1] = 0.0
    deviation[1:6, 7] = 0.0
    target = deviation <= 0.25
    first, _, first_objective = lexicographic_target_astar(
        free, deviation, target, (5, 1), (5, 7), 1000,
    )
    mirrored_free = np.fliplr(free)
    mirrored_deviation = np.fliplr(deviation)
    mirrored, _, mirrored_objective = lexicographic_target_astar(
        mirrored_free, mirrored_deviation, mirrored_deviation <= 0.25,
        (5, 7), (5, 1), 1000,
    )
    assert first is not None and mirrored is not None
    assert first_objective == pytest.approx(mirrored_objective)
    assert [(row, 8 - col) for row, col in first] == mirrored


def test_r15_method_is_not_the_frozen_r14_weighted_reference():
    assert METHOD_ID == "parking_component_lexicographic_centre_reference_r15_v1"


def test_single_observation_ack_keeps_fail_closed_result(monkeypatch):
    def accepted(_self, _expected, _changed, *, timeout_s=None):
        del timeout_s
        return {
            "costmap_update_acknowledged": True,
            "semantic_exact_stable_observations": 1,
        }

    monkeypatch.setattr(semantic_v3_ack_r14.DeterministicReinflationSessionR14,
                        "_wait_for_costmap_ack", accepted)
    session = object.__new__(SingleObservationExactAckSessionR15)
    session._semantic_costmap = object()
    result = session._wait_for_costmap_ack(np.zeros((1, 1)), np.ones((1, 1), bool))
    assert result["r15_exact_observation_count"] == 1
    assert result["r15_timeout_repair_allowed"] is False


def test_single_observation_ack_rejects_wrong_observation_count(monkeypatch):
    def wrong_count(_self, _expected, _changed, *, timeout_s=None):
        del timeout_s
        return {
            "costmap_update_acknowledged": True,
            "semantic_exact_stable_observations": 2,
        }

    monkeypatch.setattr(semantic_v3_ack_r14.DeterministicReinflationSessionR14,
                        "_wait_for_costmap_ack", wrong_count)
    session = object.__new__(SingleObservationExactAckSessionR15)
    session._semantic_costmap = object()
    with pytest.raises(RuntimeError, match="OBSERVATION_COUNT"):
        session._wait_for_costmap_ack(np.zeros((1, 1)), np.ones((1, 1), bool))
    assert session._costmap_state_trusted is False
    assert session._force_full_next_update is True


def test_nonsemantic_startup_ack_is_not_relabelled(monkeypatch):
    def base_ack(_self, _expected, _changed, *, timeout_s=None):
        del timeout_s
        return {"costmap_update_acknowledged": True, "costmap_ack_status": "base"}

    monkeypatch.setattr(semantic_v3_ack_r14.DeterministicReinflationSessionR14,
                        "_wait_for_costmap_ack", base_ack)
    session = object.__new__(SingleObservationExactAckSessionR15)
    session._semantic_costmap = None
    result = session._wait_for_costmap_ack(np.zeros((1, 1)), np.ones((1, 1), bool))
    assert result == {"costmap_update_acknowledged": True, "costmap_ack_status": "base"}


def test_ack_gate_requires_all_rows_exact_and_bounded():
    config = yaml.safe_load(CONFIG.read_text())
    row = {
        "wall_ms": 100.0, "current_rss_bytes": 500_000_000,
        "peak_rss_bytes": 600_000_000, "acknowledged": True,
        "exact_observations": 1, "hard_mismatch": 0, "soft_mismatch": 0,
        "stale_cells": 0, "hash_mismatch": 0, "sequence_mismatch": 0,
        "full_repair": False,
    }
    passed = _ack_gate([row, row], config, 2)
    assert passed["ack_stress_passed"] is True
    failed = _ack_gate([dict(row, soft_mismatch=1), row], config, 2)
    assert failed["ack_stress_passed"] is False
    assert failed["hard_stop_triggered"] is True


def test_percentile_empty_and_exact():
    assert percentile([], 99) is None
    assert percentile([1.0, 2.0, 3.0], 50) == 2.0
