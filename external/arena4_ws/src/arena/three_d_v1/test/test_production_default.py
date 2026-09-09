import importlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

import arena_3d_v1
from arena_3d_v1.pipeline import L1Plan
from arena_3d_v1.production_cache import StableCacheBundle, StableCacheBundleError
from arena_3d_v1.production_io import load_plan_bundle, write_plan_bundle
from arena_3d_v1.production_runtime import (
    RevisionSelectionError,
    controller_class,
    create_controller,
    resolve_revision,
)
from arena_3d_v1.production_validation import default_reference_audit
from arena_3d_v1.stable_contract import (
    CACHE_BUNDLE_SCHEMA,
    PRODUCTION_BASELINE_ID,
    SOURCE_REVISION,
    load_stable_config,
)
from arena_3d_v1.stable_pipeline import Layered3DV1StableController


def make_plan(route="stable", start=(18, 2), goal=(2, 28)):
    free = np.ones((21, 31), dtype=bool)
    corridor = np.ones_like(free)
    return L1Plan(
        static_safe_free=free,
        corridor_mask=corridor,
        start_cell=start,
        goal_cell=goal,
        map_hash="map-stable-v1",
        map_origin=(1.0, 2.0, 0.0),
        resolution=0.05,
        topology_hash="topology-stable-v1",
        route_edge_ids=(route,),
        footprint_hash="jackal-footprint-v1",
        route_signature=route,
    )


def test_default_import_factory_config_and_schema_are_stable():
    assert arena_3d_v1.REVISION_ID == PRODUCTION_BASELINE_ID
    assert arena_3d_v1.Layered3DV1Controller is Layered3DV1StableController
    assert resolve_revision().resolved == PRODUCTION_BASELINE_ID
    assert controller_class() is Layered3DV1StableController
    config = load_stable_config()
    assert config["production_baseline_id"] == PRODUCTION_BASELINE_ID
    assert config["source_revision"] == SOURCE_REVISION
    assert config["cache"]["schema"] == CACHE_BUNDLE_SCHEMA


@pytest.mark.parametrize("revision", ["r0", "r1", "r2", "r2-production-acceptance"])
def test_production_rejects_legacy_revisions(revision):
    with pytest.raises(RevisionSelectionError, match="legacy/research-only"):
        resolve_revision(revision, mode="production")
    assert resolve_revision(revision, mode="benchmark").legacy_explicit_only


@pytest.mark.parametrize("revision", ["pure-dstar", "PURE_DSTAR_LITE"])
def test_pure_dstar_is_rejected(revision):
    with pytest.raises(RevisionSelectionError, match=r"pure D\*"):
        resolve_revision(revision, mode="production")


def test_stable_cache_miss_falls_back_without_build_and_has_telemetry(tmp_path):
    controller = create_controller(make_plan(), cache_bundle=tmp_path)
    result = controller.initial_l2_result
    assert isinstance(controller, Layered3DV1StableController)
    assert result.success
    assert result.selected_backend == "deterministic_grid_astar_cache_miss"
    assert controller.lifecycle.synchronous_dstar_build_count == 0
    assert not list(tmp_path.rglob("payload.npz"))
    assert result.diagnostics["architecture_id"] == "3D-V1"
    assert result.diagnostics["production_baseline_id"] == PRODUCTION_BASELINE_ID
    assert result.diagnostics["source_revision"] == SOURCE_REVISION
    assert result.diagnostics["fallback_reason"] == "CACHE_MISS_OR_REJECT"
    assert result.diagnostics["cache_status"].startswith("MISS:")


def test_plan_bundle_and_stable_cache_bind_endpoint_yaw_and_fail_closed(tmp_path):
    plan = make_plan()
    plan_root = write_plan_bundle(
        tmp_path / "plan", plan,
        start_pose=(1.1, 2.2, 0.3), goal_pose=(3.3, 4.4, -0.7),
    )
    verified = load_plan_bundle(plan_root)
    bundle = StableCacheBundle(tmp_path / "cache-bundle")
    entry = bundle.prebuild_route(
        verified.plan, start_pose=verified.start_pose, goal_pose=verified.goal_pose,
    )
    assert entry["production_ready"]
    good = bundle.verify_route(
        verified.plan, start_pose=verified.start_pose, goal_pose=verified.goal_pose, deep=True,
    )
    changed_yaw = bundle.verify_route(
        verified.plan,
        start_pose=(1.1, 2.2, 0.31),
        goal_pose=verified.goal_pose,
    )
    assert good.accepted and good.deep_verified
    assert changed_yaw.status == "MISS"
    controller = create_controller(
        verified.plan,
        cache_bundle=bundle.root,
        start_pose=verified.start_pose,
        goal_pose=verified.goal_pose,
        verify_l2_oracle=True,
    )
    assert controller.initial_l2_result.success
    assert controller.initial_l2_result.selected_backend in {
        "compact_persistent_dstar", "compact_dstar_cache_restore",
    }
    assert controller.cache_status == "HIT_VERIFIED_STABLE_BUNDLE"
    assert controller.lifecycle.synchronous_dstar_build_count == 0


def test_corrupt_route_cache_is_rejected_to_astar_not_rebuilt(tmp_path):
    plan = make_plan()
    bundle = StableCacheBundle(tmp_path / "cache-bundle")
    entry = bundle.prebuild_route(
        plan, start_pose=(0.0, 0.0, 0.0), goal_pose=(1.0, 1.0, 0.0),
    )
    payload_record = next(record for record in entry["files"] if record["path"].endswith("payload.npz"))
    payload = bundle.root / payload_record["path"]
    payload.write_bytes(payload.read_bytes()[:17])
    status = bundle.verify_route(
        plan, start_pose=(0.0, 0.0, 0.0), goal_pose=(1.0, 1.0, 0.0),
    )
    assert status.status == "REJECTED"
    assert status.reason in {"ENTRY_FILE_SIZE_MISMATCH", "ENTRY_FILE_HASH_MISMATCH"}
    controller = create_controller(
        plan,
        cache_bundle=bundle.root,
        start_pose=(0.0, 0.0, 0.0),
        goal_pose=(1.0, 1.0, 0.0),
        verify_l2_oracle=True,
    )
    assert controller.initial_l2_result.selected_backend == "deterministic_grid_astar_cache_miss"
    assert controller.lifecycle.synchronous_dstar_build_count == 0


def test_global_cache_revision_mismatch_rejects_startup(tmp_path):
    plan = make_plan()
    bundle = StableCacheBundle(tmp_path / "cache-bundle")
    bundle.initialize()
    manifest = json.loads(bundle.manifest_path.read_text())
    manifest["source_revision"] = "wrong"
    bundle.manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(StableCacheBundleError, match="source_revision mismatch"):
        create_controller(
            plan,
            cache_bundle=bundle.root,
            start_pose=(0.0, 0.0, 0.0),
            goal_pose=(1.0, 1.0, 0.0),
        )


def test_plan_payload_corruption_is_rejected(tmp_path):
    root = write_plan_bundle(
        tmp_path / "plan", make_plan(),
        start_pose=(0.0, 0.0, 0.0), goal_pose=(1.0, 1.0, 0.0),
    )
    payload = root / "plan_payload.npz"
    payload.write_bytes(payload.read_bytes()[:11])
    with pytest.raises(ValueError, match="payload"):
        load_plan_bundle(root)


def test_cache_inventory_and_purge_are_report_only(tmp_path):
    plan = make_plan()
    bundle = StableCacheBundle(tmp_path / "cache-bundle")
    entry = bundle.prebuild_route(
        plan, start_pose=(0.0, 0.0, 0.0), goal_pose=(1.0, 1.0, 0.0),
    )
    before = sorted(path for path in bundle.root.rglob("*") if path.is_file())
    candidates = bundle.purge_obsolete_candidates([])
    after = sorted(path for path in bundle.root.rglob("*") if path.is_file())
    assert candidates and candidates[0]["action"] == "REPORT_ONLY_NOT_DELETED"
    assert candidates[0]["binding_hash"] == entry["binding_hash"]
    assert before == after


def test_default_reference_scan_has_no_unhandled_default_legacy_reference():
    rows = default_reference_audit(Path("/home/robot/pudu_robot_ws"))
    assert rows
    assert not [row for row in rows if row["default_violation"]]


def test_default_import_does_not_load_research_runners():
    importlib.reload(arena_3d_v1)
    forbidden = [
        name for name in sys.modules
        if name.startswith("arena_3d_v1.")
        and any(token in name for token in ("stage_a", "stage_b", "profile", "soak", "calibration"))
    ]
    assert forbidden == []
