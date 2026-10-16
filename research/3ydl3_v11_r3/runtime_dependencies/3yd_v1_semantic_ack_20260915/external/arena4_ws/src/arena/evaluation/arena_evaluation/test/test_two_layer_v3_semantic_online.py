from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from arena_evaluation import two_layer_v3_semantic_online as subject
from arena_evaluation.semantic_rasterizer import grid_hash
from arena_evaluation.semantic_v3_online_session import VerifiedV3PlannerSession


def _exact_ack(server, sequence=3):
    return {
        "costmap_update_acknowledged": True,
        "costmap_ack_semantics": "exact_effective_master",
        "costmap_ack_hard_mismatch_cells": 0,
        "costmap_ack_soft_exact_mismatch_cells": 0,
        "costmap_ack_stale_roi_cells": 0,
        "costmap_ack_hash_mismatch": 0,
        "costmap_ack_sequence_mismatch": 0,
        "server_costmap_content_hash": grid_hash(server),
        "semantic_expected_server_content_hash": grid_hash(server),
        "semantic_publication_sequence": sequence,
    }


def test_online_config_binds_r10_algorithm_and_frozen_targeted():
    config, algorithm_path, targeted_path, algorithm, parent = subject._load_online_config(
        subject.DEFAULT_CONFIG,
    )
    assert config["architecture_id"] == "2A-V3"
    assert algorithm["implementation_revision"] == "r10-local-station-prefilter-strict-final"
    assert algorithm_path.name == "two_layer_v3_semantic_r10_local_projection.yaml"
    assert targeted_path.name == "pudu_wanda_3f_v3_r10_targeted_applicable_positive_v1.yaml"
    assert parent["protocol"]["allow_reverse"] is False
    assert parent["protocol"]["minimum_turning_radius_m"] == 0.4


def test_verified_master_snapshot_returns_exact_top_row_order():
    top = np.asarray([[1, 2, 3], [4, 5, 6]], dtype=np.uint8)
    server = np.ascontiguousarray(np.flipud(top))
    session = object.__new__(VerifiedV3PlannerSession)
    session._semantic_costmap = SimpleNamespace(
        expected_master_cost=top,
        expected_master_hash=grid_hash(top),
    )
    session._semantic_publication_sequence = 3
    session._costmap_state_trusted = True
    session._force_full_next_update = False
    session._server_costmap_snapshot = lambda deadline: (server.copy(), 42)
    result, evidence = session.verified_master_snapshot(_exact_ack(server))
    assert np.array_equal(result, top)
    assert evidence["verified_master_mismatch_cells"] == 0
    assert evidence["verified_master_hash"] == grid_hash(top)


def test_verified_master_snapshot_fails_closed_after_server_change():
    top = np.asarray([[1, 2], [3, 4]], dtype=np.uint8)
    server = np.ascontiguousarray(np.flipud(top))
    changed = server.copy()
    changed[0, 0] += 1
    session = object.__new__(VerifiedV3PlannerSession)
    session._semantic_costmap = SimpleNamespace(
        expected_master_cost=top,
        expected_master_hash=grid_hash(top),
    )
    session._semantic_publication_sequence = 3
    session._costmap_state_trusted = True
    session._force_full_next_update = False
    session._server_costmap_snapshot = lambda deadline: (changed, 43)
    with pytest.raises(RuntimeError, match="changed or is not exact"):
        session.verified_master_snapshot(_exact_ack(server))
    assert session._costmap_state_trusted is False
    assert session._force_full_next_update is True


def test_verified_master_snapshot_rejects_interval_or_stale_ack():
    session = object.__new__(VerifiedV3PlannerSession)
    session._semantic_costmap = SimpleNamespace(expected_master_cost=np.zeros((1, 1), np.uint8))
    with pytest.raises(RuntimeError, match="ACK failed"):
        session.verified_master_snapshot({"costmap_update_acknowledged": True})


def test_path_content_hash_ignores_transport_serialization_padding():
    pose = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=-2.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.5, w=0.5),
        ),
    )
    first = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map", stamp=SimpleNamespace(sec=7, nanosec=11),
        ),
        poses=[pose],
    )
    second_pose = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=-2.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.5, w=0.5),
        ),
    )
    second = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map", stamp=SimpleNamespace(sec=7, nanosec=11),
        ),
        poses=[second_pose],
    )
    assert VerifiedV3PlannerSession._path_content_hash(first) == (
        VerifiedV3PlannerSession._path_content_hash(second)
    )
    second.poses[0].pose.position.x = 1.01
    assert VerifiedV3PlannerSession._path_content_hash(first) != (
        VerifiedV3PlannerSession._path_content_hash(second)
    )


def test_metric_row_requires_strict_witness():
    query = SimpleNamespace(query_id="q", category="lane")
    row = subject._metric_row(query, {"failure_code": "NO_ROUTE"}, None)
    assert row["final_valid_success"] is False
    assert row["failure_code"] == "NO_ROUTE"


def test_online_config_hash_binding_rejects_tamper(tmp_path: Path):
    config = subject.applicability._load_extended_mapping(
        subject.DEFAULT_CONFIG, "extends_online_config",
    )
    config.pop("extends_online_config", None)
    config["algorithm_config"]["path"] = str(
        subject.DEFAULT_CONFIG.parent / config["algorithm_config"]["path"]
    )
    config["targeted_query_set"]["path"] = str(
        subject.DEFAULT_CONFIG.parent / config["targeted_query_set"]["path"]
    )
    config["predecessor_online_config"]["path"] = str(
        subject.DEFAULT_CONFIG.parent / config["predecessor_online_config"]["path"]
    )
    config["algorithm_config"]["sha256"] = "0" * 64
    copied = tmp_path / "online.yaml"
    copied.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(ValueError, match="frozen online binding changed"):
        subject._load_online_config(copied)
