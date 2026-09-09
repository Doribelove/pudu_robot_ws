from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from arena_evaluation import semantic_route_phase_v3 as subject
from arena_evaluation import two_layer_v3_semantic_r13_benchmark as runner
from arena_evaluation import two_layer_v3_semantic_r13_online as online


def _semantic_audit(*, hard_failure=False):
    failures = ["R2_SEMANTIC_GATE"]
    if hard_failure:
        failures.append("PADDED_MASTER_COLLISION")
    return {
        "gate_passed": False,
        "failure_codes": failures,
        "semantics": {
            "semantic_gate_passed": False,
            "active_window": {
                "classes": {
                    "lane": {"semantic_gate_passed": False},
                    "parking": {"semantic_gate_passed": None},
                },
            },
        },
    }


def test_route_phase_identity_and_motion_contract_are_explicit():
    assert runner.ARCHITECTURE_ID == "2A-V3"
    assert runner.IMPLEMENTATION_REVISION == "r13-route-phase-multisemantic-state-lattice"
    assert subject.YAW_BIN_COUNT == 48
    policy = subject.RoutePhasePolicy()
    assert policy.turning_radius_m >= 0.40


def test_policy_rejects_relaxed_turning_radius_and_unbounded_resources():
    with pytest.raises(ValueError, match="turning radius"):
        subject.RoutePhasePolicy(turning_radius_m=0.399)
    with pytest.raises(ValueError, match="resource bounds"):
        subject.RoutePhasePolicy(maximum_expanded_labels=0)


def test_phase_requires_stable_positive_semantic_instance():
    assert subject.Phase("transfer").instance == 0
    assert subject.Phase("lane", 7) == subject.Phase("lane", 7)
    with pytest.raises(ValueError, match="positive instance"):
        subject.Phase("parking", 0)
    with pytest.raises(ValueError, match="unknown route phase"):
        subject.Phase("junction", 1)


def test_safe_soft_fallback_is_final_valid_but_never_semantic_success():
    searcher = object.__new__(subject.LazyRoutePhaseSearch)
    searcher.shortest_fallback = lambda _semantic_map: _semantic_audit()
    result = searcher.safe_soft_fallback(
        object(),
        {"applicable_semantic_classes": ["lane"], "records": []},
    )
    assert result["gate_passed"] is True
    assert result["final_valid_gate_passed"] is True
    assert result["strict_semantic_gate_passed"] is False
    assert result["semantic_success_counted"] is False
    assert result["safe_soft_fallback"] is True
    assert result["raw_failure_codes"] == ["R2_SEMANTIC_GATE"]
    assert result["failure_codes"] == []


def test_safe_soft_fallback_fails_closed_on_any_hard_failure():
    searcher = object.__new__(subject.LazyRoutePhaseSearch)
    searcher.shortest_fallback = lambda _semantic_map: _semantic_audit(hard_failure=True)
    assert searcher.safe_soft_fallback(
        object(), {"applicable_semantic_classes": ["lane"], "records": []},
    ) is None


def test_fallback_first_requires_explicit_soft_fallback_authorization():
    searcher = object.__new__(subject.LazyRoutePhaseSearch)
    with pytest.raises(ValueError, match="explicit safe-soft-fallback"):
        searcher.search(object(), fallback_first=True)


def test_targeted_scope_binds_approved_positive_and_original_negatives():
    config, _parent, query_path = runner._load_config(runner.DEFAULT_CONFIG)
    queries, content_hash, source = runner._query_scope(
        config=config,
        query_path=query_path,
        scope="targeted",
        map_path=runner.DEFAULT_EXTRACTED / "optemap.pgm",
    )
    assert content_hash == config["targeted_query_set"]["query_hash"]
    assert Path(source).name == "pudu_wanda_3f_v3_r10_targeted_applicable_positive_v1.yaml"
    assert [query.query_id for query in queries] == [
        "v3-applicable-mirror-positive",
        "r3-mirror-2-negative",
        "cmp2-02-lane-south",
    ]


def test_selected8_scope_keeps_frozen_order_and_hash():
    config, _parent, query_path = runner._load_config(runner.DEFAULT_CONFIG)
    queries, content_hash, _source = runner._query_scope(
        config=config,
        query_path=query_path,
        scope="selected8",
        map_path=runner.DEFAULT_EXTRACTED / "optemap.pgm",
    )
    assert content_hash == "7e2a5ddb7a91b175779c0cfc1063dad77bf1c926ee52be94c350203204bac43e"
    assert len(queries) == 8
    assert queries[0].query_id == "cmp2-01-lane-north"
    assert queries[-1].query_id == "cmp2-08-parking-internal"


def test_route_hash_keeps_topology_and_polyline_bindings_separate():
    route = SimpleNamespace(
        node_ids=[1, 2], edge_ids=[3], polyline=[[0.0, 0.0], [1.0, 0.0]],
    )
    topology_hash = runner.r1._path_hash(route)
    polyline_hash = runner.canonical_hash(route.polyline)
    assert topology_hash != polyline_hash


def test_online_dispatch_never_allows_targeted_soft_fallback():
    config, _path, _algorithm, _parent, _queries, _single, _single_path = online._load_config(
        online.DEFAULT_CONFIG,
    )
    targeted = online._query_plan(
        "targeted", "v3-applicable-mirror-positive", config,
    )
    assert targeted["strict_semantic_required"] is True
    assert targeted["allow_safe_soft_fallback"] is False
    fallback = online._query_plan("selected8", "cmp2-06-lane-to-parking", config)
    assert fallback["strict_semantic_required"] is False
    assert fallback["allow_safe_soft_fallback"] is True
    assert fallback["fallback_first"] is True


def test_online_metric_row_preserves_semantic_failure_on_valid_fallback():
    query = SimpleNamespace(query_id="q", category="parking")
    witness = {
        "gate_passed": True,
        "semantic_success_counted": False,
        "safe_negative_fallback": False,
        "safe_soft_fallback": True,
        "fallback_reason": "APPLICABLE_R2_SEMANTIC_TARGET_UNSATISFIED",
        "arc_length_m": 4.0,
        "maximum_control_curvature_1pm": 2.0,
        "semantics": {"active_window": {"classes": {
            "lane": {"semantic_gate_passed": None},
            "parking": {
                "semantic_gate_passed": False,
                "parking_center_band_ratio": .3,
                "parking_center_normalized_deviation_p50": .6,
            },
        }}},
        "canonical": {
            "final_valid_success": True, "static_footprint_valid": True,
            "kinematic_valid": True, "reverse_distance_m": 0,
            "in_place_rotation_count": 0,
        },
        "hard_features": {
            "hard_feature_gate_passed": True,
            "no_stopping_task_endpoint_violations": 0,
        },
        "ordered_progress": {"ordered_progress_gate_passed": True},
        "revisit": {"revisit_screen_passed": True},
    }
    row = online._row_from_witness(query, witness)
    assert row["final_valid_success"] is True
    assert row["strict_semantic_gate_passed"] is False
    assert row["semantic_success_counted"] is False
    assert row["parking_semantic_gate_passed"] is False


def test_r13_roi_tiles_repair_every_inflation_seam_and_boundary():
    class Stamp:
        def to_msg(self):
            return object()

    class Message:
        def __init__(self):
            self.header = SimpleNamespace(frame_id="", stamp=None)
            self.x = self.y = self.width = self.height = 0
            self.data = None

    published = []
    session = object.__new__(online.RoutePhaseV3PlannerSession)
    session._current_grid = np.zeros((80, 100), dtype=np.int8)
    session._begin_publication = lambda changed, full=False: None
    session.OccupancyGridUpdate = Message
    session._local_update_publisher = SimpleNamespace(
        publish=lambda message: published.append((
            message.y, message.height, bytes(message.data),
        )),
    )
    session.client = SimpleNamespace(
        node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: Stamp())),
        executor=SimpleNamespace(spin_once=lambda timeout_sec: None),
    )
    session.roi_max_payload_bytes = 100
    session.roi_tile_overlap_rows = 2
    session.roi_seam_repair_margin_rows = 3
    session.roi_publish_pacing_s = 0.0
    expected = np.arange(8000, dtype=np.int16).reshape(80, 100).astype(np.int8)
    applied, diagnostics = session._publish_dirty_roi(
        expected, np.ones(expected.shape, dtype=bool),
    )
    assert np.array_equal(applied, expected)
    assert diagnostics["roi_seam_repair_messages"] > 2
    assert diagnostics["roi_message_count"] == len(published)
    assert diagnostics["roi_tile_overlap_rows"] == 2
    assert diagnostics["roi_seam_repair_margin_rows"] == 3
    assert max(height*100 for _row, height, _data in published) <= 1024
    assert published[-1][0]+published[-1][1] == 80
