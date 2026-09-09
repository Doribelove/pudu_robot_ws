from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation.se2_semantic_guide import (
    build_smac_dubin_primitives,
    sample_primitive,
    wrap_angle,
)
from arena_evaluation import semantic_transition_se2_chain_reachability as probe
from arena_evaluation.semantic_transition_se2_chain_reachability import (
    ReachabilityFailure,
    ReachabilityPolicy,
    _predecessor_pose,
    _validate_transition,
    analyze_world,
)


class FakeMap:
    resolution = 0.05
    full_origin = (0.0, 0.0)
    full_height = 100
    row0 = 0
    col0 = 0
    height = 100
    width = 100

    def __init__(self):
        self.distance_m = np.full((self.height, self.width), 10.0, dtype=np.float32)

    def world_to_cell(self, x, y):
        col = int(np.floor(float(x) / self.resolution))
        row = self.full_height - 1 - int(np.floor(float(y) / self.resolution))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None


class FakeWorld:
    def __init__(self, *, target=True, goal=None):
        policy = ReachabilityPolicy(maximum_states_per_direction=5000, timeout_per_direction_s=5.0)
        straight = build_smac_dubin_primitives(policy.smac_policy)[0]
        distance = 8.0 * straight.arc_length_m
        self.start = (1.0, 2.0, 0.0)
        self.goal = goal or (1.0 + distance, 2.0, 0.0)
        self.map = FakeMap()
        shape = (self.map.height, self.map.width)
        self.selected = [7]
        self.master = np.zeros(shape, dtype=np.uint8)
        self.grids = {
            "labels": np.full(shape, 7, dtype=np.int16),
            "allowed": np.ones(shape, dtype=bool),
            "hard": np.zeros(shape, dtype=bool),
            "correct": np.full(shape, bool(target), dtype=bool),
            "error": np.full(shape, 0.1 if target else 1.0, dtype=np.float32),
        }
        self.meta = {
            "route_polyline": [list(self.start[:2]), list(self.goal[:2])],
            "expected_master_hash": "fake-master",
        }
        self.query = SimpleNamespace(query_id="synthetic-positive")
        self.validation_dense_flags = []

    def cells(self, samples):
        poses = np.asarray(samples)
        cols = np.floor(poses[:, 0] / self.map.resolution).astype(np.int64)
        rows = self.map.full_height - 1 - np.floor(poses[:, 1] / self.map.resolution).astype(np.int64)
        inside = (rows >= 0) & (rows < self.map.height) & (cols >= 0) & (cols < self.map.width)
        return rows, cols, inside

    def collision_free(self, samples):
        rows, cols, inside = self.cells(samples)
        return bool(np.all(inside) and np.all(self.master[rows, cols] < 253))

    def validate_edge(self, edge, *, dense=False):
        self.validation_dense_flags.append(bool(dense))
        rows, cols, inside = self.cells(np.vstack((edge.start, edge.samples)))
        if not np.all(inside):
            return False
        if np.any(self.master[rows, cols] >= 253):
            return False
        if np.any(self.grids["labels"][rows, cols] != 7):
            return False
        edge.n = len(edge.samples)
        correct = self.grids["correct"][rows[1:], cols[1:]]
        target = correct & (self.grids["error"][rows[1:], cols[1:]] <= 0.5)
        edge.correct = int(correct.sum())
        edge.target = int(target.sum())
        return True


def small_policy(**changes):
    values = dict(
        local_crop_padding_m=0.53,
        maximum_states_per_direction=5000,
        timeout_per_direction_s=5.0,
        maximum_peak_rss_mib=2048.0,
    )
    values.update(changes)
    return ReachabilityPolicy(**values)


def test_formal_primitive_contract_is_48_bin_forward_only_and_conservative_radius():
    policy = small_policy()
    primitives = build_smac_dubin_primitives(policy.smac_policy)
    assert policy.yaw_bins == 48
    assert policy.turning_radius_m == pytest.approx(0.401)
    assert [item.name for item in primitives] == ["FORWARD", "FORWARD_LEFT", "FORWARD_RIGHT"]
    assert [item.delta_yaw_bins for item in primitives] == [0, 2, -2]
    assert max(abs(item.curvature_1pm) for item in primitives) < 2.50


@pytest.mark.parametrize("primitive_index", [0, 1, 2])
def test_backward_coreach_uses_exact_inverse_of_a_forward_primitive(primitive_index):
    policy = small_policy()
    primitive = build_smac_dubin_primitives(policy.smac_policy)[primitive_index]
    child = (2.0, 2.0, np.pi / 2.0)
    predecessor = _predecessor_pose(child, primitive, policy.smac_policy)
    replay = sample_primitive(predecessor, primitive, policy.smac_policy)
    assert replay[-1][:2] == pytest.approx(child[:2], abs=1.0e-12)
    assert abs(wrap_angle(replay[-1][2] - child[2])) <= 1.0e-12


def test_transition_rejects_goal_plane_overshoot_before_constraint_world_acceptance():
    world = FakeWorld()
    policy = small_policy()
    primitive = build_smac_dubin_primitives(policy.smac_policy)[0]
    source = (world.goal[0] - 0.5 * primitive.arc_length_m, world.goal[1], 0.0)
    samples = sample_primitive(source, primitive, policy.smac_policy)
    local = np.ones_like(world.master, dtype=bool)
    valid, reason = _validate_transition(
        world,
        source,
        samples,
        primitive,
        local_mask=local,
        lane_label=7,
        terminal_tangent=np.asarray((1.0, 0.0)),
        policy=policy,
    )
    assert not valid
    assert reason == "GOAL_PLANE_OVERSHOOT"
    assert world.validation_dense_flags == []


def test_admitted_transition_invokes_dense_constraint_world_validation():
    world = FakeWorld()
    policy = small_policy()
    primitive = build_smac_dubin_primitives(policy.smac_policy)[0]
    source = world.start
    samples = sample_primitive(source, primitive, policy.smac_policy)
    local = np.ones_like(world.master, dtype=bool)
    valid, reason = _validate_transition(
        world,
        source,
        samples,
        primitive,
        local_mask=local,
        lane_label=7,
        terminal_tangent=np.asarray((1.0, 0.0)),
        policy=policy,
    )
    assert valid
    assert reason == ""
    assert world.validation_dense_flags == [True]


def test_finite_forward_and_backward_sets_intersect_on_raw_target_states_deterministically():
    world = FakeWorld(target=True)
    policy = small_policy()
    straight = build_smac_dubin_primitives(policy.smac_policy)[0]
    first, first_arrays = analyze_world(world, policy, primitives=(straight,))
    second, second_arrays = analyze_world(world, policy, primitives=(straight,))
    assert first["analysis_disposition"] == "NON_DECISION_EXPLORATORY"
    assert not first["hard_stop_eligible"]
    assert not first["acceptance_evidence"]
    assert not first["c1_infeasibility_evidence"]
    assert first["traversals_open_exhausted_within_underapproximation"]
    assert first["observed_target_key_intersection_nondecisive"]
    assert first["observed_exact_representative_target_join_nondecisive"]
    assert first["result_code"] == "NON_DECISION_EXPLORATORY_EXACT_REPRESENTATIVE_TARGET_JOIN_OBSERVED"
    assert first["intersection"]["observed_raw_target_key_intersection_count"] > 0
    assert not first["replayable_witness_generated"]
    assert not first["smac_analytic_expansion_implemented"]
    assert first["intersection"] == second["intersection"]
    for name in first_arrays:
        assert np.array_equal(first_arrays[name], second_arrays[name])


def test_empty_raw_target_reports_reachability_gate_failure_without_resource_claim():
    world = FakeWorld(target=False)
    policy = small_policy()
    straight = build_smac_dubin_primitives(policy.smac_policy)[0]
    result, arrays = analyze_world(world, policy, primitives=(straight,))
    assert result["traversals_open_exhausted_within_underapproximation"]
    assert result["result_code"] == "NON_DECISION_EXPLORATORY_NO_TARGET_KEY_INTERSECTION_OBSERVED"
    assert not result["observed_target_key_intersection_nondecisive"]
    assert not result["hard_stop_eligible"]
    assert not result["c1_infeasibility_evidence"]
    assert result["r2_resource_analysis_status"] == "NOT_RUN_NONDECISION_EXPLORATORY"
    assert arrays["observed_target_key_intersection"].shape == (0, 3)


def test_exact_lane_instance_is_not_replaced_by_selected_lane_union():
    world = FakeWorld()
    goal_cell = world.map.world_to_cell(*world.goal[:2])
    world.grids["labels"][goal_cell] = 8
    world.selected = [7, 8]
    with pytest.raises(ReachabilityFailure) as error:
        analyze_world(world, small_policy())
    assert error.value.code == "ENDPOINT_LANE_INSTANCE_MISMATCH"


def test_non_bin_endpoint_yaw_fails_instead_of_changing_query():
    world = FakeWorld(goal=(1.8, 2.0, 0.01))
    with pytest.raises(ReachabilityFailure) as error:
        analyze_world(world, small_policy())
    assert error.value.code == "ENDPOINT_YAW_NOT_ON_48_BIN"


def test_policy_caps_prevent_unbounded_search():
    with pytest.raises(ValueError, match="1000000"):
        ReachabilityPolicy(maximum_states_per_direction=1_000_001)
    with pytest.raises(ValueError, match="120"):
        ReachabilityPolicy(timeout_per_direction_s=121.0)


def test_state_limit_is_inconclusive_not_an_infeasibility_claim():
    world = FakeWorld(target=True)
    policy = small_policy(maximum_states_per_direction=1)
    result, _ = analyze_world(world, policy)
    assert not result["traversals_open_exhausted_within_underapproximation"]
    assert result["result_code"] == "NON_DECISION_EXPLORATORY_TRAVERSAL_BOUNDED"
    assert not result["hard_stop_eligible"]
    assert not result["continuous_space_witness_proved"]


@pytest.mark.parametrize("intersection", [False, True])
def test_cli_always_returns_inconclusive_for_current_nondecision_probe(monkeypatch, tmp_path, intersection):
    monkeypatch.setattr(probe, "run", lambda *args, **kwargs: {
        "result_code": "NON_DECISION_EXPLORATORY_TEST",
        "analysis_disposition": "NON_DECISION_EXPLORATORY",
        "hard_stop_eligible": False,
        "observed_target_key_intersection_nondecisive": intersection,
        "replayable_witness_generated": False,
        "wall_s": 0.0,
    })
    code = probe.main([
        "--inputs", str(tmp_path),
        "--output", str(tmp_path / "unused"),
    ])
    assert code == 3


def test_run_records_input_hashes_before_and_after(monkeypatch, tmp_path):
    inputs = tmp_path / "inputs"
    inputs.mkdir()
    (inputs / "synthetic-positive.json").write_text("{}", encoding="utf-8")
    (inputs / "synthetic-positive.npz").write_bytes(b"not-read-by-fake-world")
    monkeypatch.setattr(probe, "ConstraintWorld", lambda *_: FakeWorld())
    output = tmp_path / "output"
    result = probe.run(
        inputs,
        "synthetic-positive",
        output,
        small_policy(maximum_states_per_direction=1),
    )
    status = probe.json.loads((output / "STATUS.json").read_text())
    assert result["analysis_disposition"] == "NON_DECISION_EXPLORATORY"
    assert result["input_hashes_at_start"] == result["input_hashes_at_end"]
    assert not any(result["hash_drift"].values())
    assert status["status"] == "NON_DECISION_EXPLORATORY"
    assert not status["hard_stop_eligible"]


def test_run_marks_unexpected_exception_excluded(monkeypatch, tmp_path):
    inputs = tmp_path / "inputs"
    inputs.mkdir()
    (inputs / "synthetic-positive.json").write_text("{}", encoding="utf-8")
    (inputs / "synthetic-positive.npz").write_bytes(b"not-read")

    def fail(*_):
        raise RuntimeError("synthetic failure")

    monkeypatch.setattr(probe, "ConstraintWorld", fail)
    output = tmp_path / "excluded"
    result = probe.run(inputs, "synthetic-positive", output, small_policy())
    status = probe.json.loads((output / "STATUS.json").read_text())
    assert result["analysis_disposition"] == "EXCLUDED"
    assert result["result_code"] == "EXCLUDED_EXCEPTION"
    assert status["status"] == "EXCLUDED"
    assert status["exception"]["exception_type"] == "RuntimeError"
    assert (output / "exception.txt").is_file()


def test_run_marks_input_drift_excluded(monkeypatch, tmp_path):
    inputs = tmp_path / "inputs"
    inputs.mkdir()
    input_json = inputs / "synthetic-positive.json"
    input_json.write_text("{}", encoding="utf-8")
    (inputs / "synthetic-positive.npz").write_bytes(b"not-read-by-fake-world")
    monkeypatch.setattr(probe, "ConstraintWorld", lambda *_: FakeWorld())
    original_analyze = probe.analyze_world

    def mutate_after_analysis(world, policy):
        result, arrays = original_analyze(world, policy)
        input_json.write_text('{"changed": true}', encoding="utf-8")
        return result, arrays

    monkeypatch.setattr(probe, "analyze_world", mutate_after_analysis)
    output = tmp_path / "drifted"
    result = probe.run(
        inputs,
        "synthetic-positive",
        output,
        small_policy(maximum_states_per_direction=1),
    )
    status = probe.json.loads((output / "STATUS.json").read_text())
    assert result["analysis_disposition"] == "EXCLUDED"
    assert result["result_code"] == "EXCLUDED_INPUT_OR_SOURCE_DRIFT"
    assert result["hash_drift"] == {"input_changed": True, "source_changed": False}
    assert status["status"] == "EXCLUDED"


def test_source_contains_no_positive_coordinates_or_historical_path_loader():
    source = Path(__file__).parents[1] / "arena_evaluation/semantic_transition_se2_chain_reachability.py"
    text = source.read_text()
    assert "-25.750998" not in text
    assert "path.json\").read" not in text
    assert "controls.json\").read" not in text
    assert '"saved_path_or_historical_witness_read": False' in text
    assert "finite_search_complete" not in text
    assert "NO_RAW_TARGET_ON_FINITE_SE2_CHAIN" not in text
    assert '"hard_stop_eligible": False' in text
    assert '"smac_analytic_expansion_implemented": False' in text
