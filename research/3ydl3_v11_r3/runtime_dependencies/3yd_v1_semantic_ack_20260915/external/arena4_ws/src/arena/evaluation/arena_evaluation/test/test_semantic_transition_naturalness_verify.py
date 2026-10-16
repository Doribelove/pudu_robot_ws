import csv
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from arena_evaluation import semantic_transition_naturalness_verify as verifier


def _evaluation(*, hard=True, binding=True, semantic=True, natural=True, after_goal=0, length=10.0):
    strict = hard and binding and semantic and natural
    return {
        "gates": {
            "hard_safety_gate_passed": hard,
            "targeted_binding_gate_passed": binding,
            "r2_semantic_gate_passed": semantic,
            "naturalness_gate_passed": natural,
            "strict_acceptance_gate_passed": strict,
        },
        "r2_semantic_audit": {
            "active_window": {
                "classes": {
                    "lane": {
                        "correct_side_ratio": 1.0,
                        "target_band_ratio": 1.0 if semantic else 0.0,
                        "lateral_error_p50_m": 0.1 if semantic else 2.0,
                    }
                }
            }
        },
        "naturalness_audit": {
            "input_complete": True,
            "audit_passed": natural,
            "status": "NO_SUSPECT_EXCURSION" if natural else "DETOUR_JUSTIFICATION_REQUIRED",
            "failure_codes": [] if natural else ["GOAL_PLANE_OVERSHOOT"],
            "maximum_goal_plane_overshoot_m": 0.0 if natural else 2.0,
            "cumulative_backward_route_progress_m": 0.0 if natural else 2.0,
            "target_sample_attribution": {
                "after_goal_plane_count": after_goal,
            },
        },
        "revisit_audit": {
            "revisit_screen_passed": natural,
            "status": "NO_REVISIT_DETECTED" if natural else "DETOUR_JUSTIFICATION_REQUIRED",
        },
        "path_length_m": length,
    }


def test_shortcut_summary_adds_evidence_but_never_conflates_gates():
    candidate = _evaluation(hard=True, semantic=True, natural=False, after_goal=20, length=30.0)
    rows = [
        {
            "strictly_shorter": True,
            "hard_safety_gate_passed": True,
            "targeted_binding_gate_passed": True,
            "r2_semantic_gate_passed": False,
            "strict_acceptance_gate_passed": False,
        }
    ]
    result = verifier._shortcut_summary(rows, candidate)
    assert result["hard_detour_necessity_disproven"]
    assert result["semantic_window_padding_evidence"]
    assert result["failure_codes"] == [
        "SAFE_SHORTCUT_DISPROVES_HARD_DETOUR_NECESSITY",
        "SEMANTIC_WINDOW_PADDING_EVIDENCE",
    ]
    assert "gate_passed" not in result


def test_shortcut_summary_rejects_unbound_shortcut_as_necessity_evidence():
    candidate = _evaluation(hard=True, semantic=True, natural=False, after_goal=20)
    rows = [{
        "strictly_shorter": True,
        "hard_safety_gate_passed": True,
        "targeted_binding_gate_passed": False,
        "r2_semantic_gate_passed": False,
        "strict_acceptance_gate_passed": False,
    }]
    result = verifier._shortcut_summary(rows, candidate)
    assert not result["hard_detour_necessity_disproven"]
    assert not result["semantic_window_padding_evidence"]


def test_shortcut_enumeration_includes_start_and_every_nonterminal_control_knot(monkeypatch):
    edges = [
        SimpleNamespace(length=4.0, goal=(4.0, 0.0, 0.0)),
        SimpleNamespace(length=4.0, goal=(8.0, 0.0, 0.0)),
        SimpleNamespace(length=2.0, goal=(10.0, 0.0, 0.0)),
    ]
    world = SimpleNamespace(start=(0.0, 0.0, 0.0), goal=(10.0, 0.0, 0.0))

    monkeypatch.setattr(verifier, "dubins_choices", lambda start, goal, radius: [("LSL", (1, 1, 1)), ("RSR", (1, 1, 1))])

    def edge(start, goal, radius, choice):
        return SimpleNamespace(word=("LSL", "RSR")[choice], length=float(goal[0] - start[0]), goal=goal)

    monkeypatch.setattr(verifier, "dubins_edge", edge)
    monkeypatch.setattr(
        verifier,
        "_evaluate_edges",
        lambda world, semantic_map, actual, control_replay_passed: _evaluation(
            length=sum(item.length for item in actual)
        ),
    )
    rows = verifier._enumerate_shortcuts(world, object(), edges)
    assert len(rows) == 6
    assert [(row["origin_kind"], row["kept_original_edge_count"]) for row in rows] == [
        ("START_TO_GOAL", 0),
        ("START_TO_GOAL", 0),
        ("CONTROL_KNOT_TO_GOAL", 1),
        ("CONTROL_KNOT_TO_GOAL", 1),
        ("CONTROL_KNOT_TO_GOAL", 2),
        ("CONTROL_KNOT_TO_GOAL", 2),
    ]


def test_write_artifacts_is_write_once_and_hashes_outputs(tmp_path, monkeypatch):
    source = tmp_path / "source.py"
    necessity = tmp_path / "necessity.py"
    revisit = tmp_path / "revisit.py"
    dependency = tmp_path / "dependency.py"
    source.write_text("source\n", encoding="utf-8")
    necessity.write_text("necessity\n", encoding="utf-8")
    revisit.write_text("revisit\n", encoding="utf-8")
    dependency.write_text("dependency\n", encoding="utf-8")
    snapshot = verifier._capture_snapshot({
        "verifier_source": source,
        "necessity_audit_source": necessity,
        "revisit_audit_source": revisit,
        "gate_dependency::fake.py": dependency,
    })
    result = {
        "query_id": "q",
        "bindings": {"path_sha256": "abc"},
        "verification_snapshot": snapshot,
        "gate_dependency_sha256": {"fake.py": snapshot["sha256"]["gate_dependency::fake.py"]},
        "gates": {
            "hard_safety_gate_passed": True,
            "targeted_binding_gate_passed": True,
            "r2_semantic_gate_passed": True,
            "naturalness_gate_passed": False,
            "strict_acceptance_gate_passed": False,
        },
    }
    shortcut = {
        "origin_kind": "START_TO_GOAL",
        "kept_original_edge_count": 0,
        "connector_choice_index": 0,
        "connector_word": "LSL",
        "connector_length_m": 2.0,
        "shortcut_path_length_m": 2.0,
        "length_saved_m": 8.0,
        "strictly_shorter": True,
        "hard_safety_gate_passed": True,
        "targeted_binding_gate_passed": True,
        "r2_semantic_gate_passed": False,
        "naturalness_gate_passed": True,
        "strict_acceptance_gate_passed": False,
        "lane_correct_side_ratio": 0.0,
        "lane_target_band_ratio": 0.0,
        "lane_lateral_error_p50_m": 4.0,
        "naturalness_status": "NO_SUSPECT_EXCURSION",
        "naturalness_failure_codes": [],
        "revisit_screen_passed": True,
        "revisit_status": "NO_REVISIT_DETECTED",
        "maximum_goal_plane_overshoot_m": 0.0,
        "cumulative_backward_route_progress_m": 0.0,
        "target_credit_after_goal_plane_count": 0,
    }
    output = tmp_path / "result"
    verifier.write_artifacts(output, result, [shortcut])
    assert json.loads((output / "verification.json").read_text())["gates"]["strict_acceptance_gate_passed"] is False
    assert json.loads((output / "manifest.json").read_text())["write_once"] is True
    with (output / "shortcut_evidence.csv").open(newline="") as stream:
        assert next(csv.DictReader(stream))["origin_kind"] == "START_TO_GOAL"
    with pytest.raises(FileExistsError):
        verifier.write_artifacts(output, result, [shortcut])


def test_snapshot_change_fails_closed(tmp_path):
    source = tmp_path / "input.json"
    source.write_text("one\n", encoding="utf-8")
    snapshot = verifier._capture_snapshot({"input": source})
    source.write_text("two\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="changed"):
        verifier._assert_snapshot_unchanged(snapshot)


def test_legacy_optimizer_length_bound_is_not_a_hard_safety_gate():
    canonical = {
        "final_valid_success": True,
        "static_footprint_valid": True,
        "kinematic_valid": True,
        "reverse_distance_m": 0,
        "in_place_rotation_count": 0,
        "maximum_curvature": 1.0,
    }
    audit = {
        "canonical": canonical,
        "padded_effective_master_collision_free": True,
        "no_stopping_goal_violation": False,
        "maximum_control_curvature_1pm": 1.0,
    }
    checks, passed = verifier._hard_safety_evidence(
        audit, {"hard_feature_gate_passed": True}
    )
    assert passed
    assert "path_length_within_bound" not in checks


def test_main_returns_failure_when_naturalness_fails(tmp_path, monkeypatch, capsys):
    result = {
        "query_id": "positive",
        "bindings": {},
        "gates": {
            "hard_safety_gate_passed": True,
            "targeted_binding_gate_passed": True,
            "r2_semantic_gate_passed": True,
            "naturalness_gate_passed": False,
            "strict_acceptance_gate_passed": False,
        },
    }
    shortcut = {
        "naturalness_failure_codes": [],
    }
    monkeypatch.setattr(verifier, "verify_candidate", lambda **kwargs: (result, [shortcut]))
    recorded = {}
    monkeypatch.setattr(
        verifier,
        "write_artifacts",
        lambda output, actual, shortcuts: recorded.update(output=output, actual=actual, shortcuts=shortcuts),
    )
    code = verifier.main([
        "--inputs", str(tmp_path),
        "--query", "positive",
        "--candidate", str(tmp_path),
        "--output", str(tmp_path / "new"),
    ])
    assert code == 2
    assert recorded["actual"]["gates"]["hard_safety_gate_passed"]
    assert not recorded["actual"]["gates"]["naturalness_gate_passed"]
    assert "strict_acceptance_gate_passed" in capsys.readouterr().out


def test_candidate_files_accepts_frozen_certificate_name(tmp_path):
    (tmp_path / "path.json").write_text("[]\n", encoding="utf-8")
    (tmp_path / "certificate.json").write_text("{}\n", encoding="utf-8")
    path_file, controls_file = verifier._candidate_files(tmp_path)
    assert path_file.name == "path.json"
    assert controls_file.name == "certificate.json"


def test_candidate_files_fails_closed_when_controls_are_missing(tmp_path):
    (tmp_path / "path.json").write_text("[]\n", encoding="utf-8")
    with pytest.raises(FileNotFoundError, match="controls.json or certificate.json"):
        verifier._candidate_files(tmp_path)


def test_module_never_defines_an_unqualified_gate_field():
    source = Path(verifier.__file__).read_text(encoding="utf-8")
    # The string appears in the explanatory docstring only; dictionary keys
    # must always carry their layer name.
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith(('"', "'")) and ":" in stripped:
            assert not stripped.startswith(('"gate_passed"', "'gate_passed'"))


def test_shortcut_row_reports_revisit_gate_separately():
    evaluation = _evaluation(hard=True, semantic=True, natural=False)
    row = verifier._shortcut_row(
        origin_kind="START_TO_GOAL",
        kept_edges=0,
        choice_index=0,
        connector=SimpleNamespace(word="LSL", length=1.0),
        evaluation=evaluation,
        original_length_m=2.0,
    )
    assert row["revisit_screen_passed"] is False
    assert row["revisit_status"] == "DETOUR_JUSTIFICATION_REQUIRED"
