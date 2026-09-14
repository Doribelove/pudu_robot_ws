import gc
import json
import weakref
from dataclasses import replace

import numpy as np
import pytest

from arena_3d_v1.l2_incremental import CorridorROI, deterministic_grid_astar
from arena_3d_v1.l2_state_lifecycle import (
    DEFAULT_DYNAMIC_BASELINE,
    FORMAT_VERSION,
    GEOMETRY_SCHEMA,
    CompactCorridorGeometry,
    CompactDStarState,
    CompactGeometryBinding,
    CompactPersistentCorridorDStar,
    L2StateLifecycleManager,
    MutableStateBinding,
    VerifiedGeometryCache,
    VerifiedMutableStateCache,
    deterministic_compact_astar,
)


def make_roi(free, start, goal, *, route="r1", map_hash="map-v1"):
    corridor = np.ones_like(free, dtype=bool)
    return CorridorROI.from_global(
        free, corridor, start, goal,
        binding_fields={
            "map_hash": map_hash,
            "map_origin": (0.0, 0.0, 0.0),
            "resolution": 0.05,
            "topology_hash": "topology-v1",
            "route_edge_ids": (route,),
            "footprint_hash": "jackal",
        },
    )


def make_planner(roi, *, budget=1000.0, expansions=100_000):
    geometry = CompactCorridorGeometry.build(roi)
    binding = MutableStateBinding(
        geometry.binding.digest,
        roi.binding.start_cell,
        roi.binding.goal_cell,
        DEFAULT_DYNAMIC_BASELINE,
    )
    state = CompactDStarState(geometry, binding)
    return CompactPersistentCorridorDStar(
        roi, geometry, state,
        dstar_wall_budget_ms=budget,
        dstar_max_expansions=expansions,
    )


@pytest.mark.parametrize("seed", range(8))
def test_compact_graph_matches_grid_oracle_reachability_cost_and_path(seed):
    rng = np.random.default_rng(seed)
    free = rng.random((19, 23)) > 0.20
    free[18, 0] = True
    free[0, 22] = True
    roi = make_roi(free, (18, 0), (0, 22))
    geometry = CompactCorridorGeometry.build(roi)
    compact = deterministic_compact_astar(
        geometry, np.zeros(geometry.state_count, dtype=bool),
        geometry.state_id_from_global((18, 0)),
        geometry.state_id_from_global((0, 22)),
    )
    oracle = deterministic_grid_astar(free, (18, 0), (0, 22))
    assert (compact.path_global is None) == (oracle.path is None)
    if oracle.path is not None:
        assert compact.cost == oracle.cost
        assert compact.path_global == oracle.path


def test_dynamic_diagonal_corner_cutting_is_forbidden():
    free = np.ones((3, 3), dtype=bool)
    roi = make_roi(free, (2, 0), (0, 2))
    planner = make_planner(roi)
    assert planner.initialize(verify_oracle=True).success
    result = planner.update([(1, 0), (2, 1)], verify_oracle=True)
    assert not result.success
    assert result.failure_code == "L2_NO_PATH_IN_CORRIDOR"


def test_no_route_then_recovery_and_no_partial_result():
    free = np.zeros((5, 9), dtype=bool)
    free[2, :] = True
    roi = make_roi(free, (2, 0), (2, 8))
    planner = make_planner(roi)
    assert planner.initialize(verify_oracle=True).success
    blocked = planner.update([(2, 4)], verify_oracle=True)
    assert not blocked.success
    assert blocked.partial_dstar_result_returned is False
    recovered = planner.update([], verify_oracle=True, force_cold_astar=True)
    assert recovered.success
    assert recovered.selected_backend == "deterministic_grid_astar_direct"
    resync = planner.service_resync()
    assert resync.success
    assert planner.state.reinitialize_count == 0


def test_timeout_and_invalid_extraction_fall_back_without_partial():
    free = np.ones((51, 51), dtype=bool)
    roi = make_roi(free, (50, 0), (0, 50))
    timeout_planner = make_planner(roi, budget=0.01, expansions=1)
    assert timeout_planner.initialize().success
    cell = timeout_planner.path_global[len(timeout_planner.path_global) // 2]
    timeout = timeout_planner.update([cell], verify_oracle=True)
    assert timeout.success
    assert timeout.selected_backend == "deterministic_grid_astar_fallback"
    assert timeout.partial_dstar_result_returned is False

    invalid_planner = make_planner(roi)
    assert invalid_planner.initialize().success
    cell = invalid_planner.path_global[len(invalid_planner.path_global) // 2]
    invalid_planner.state.invalid_extraction_injected = True
    invalid = invalid_planner.update([cell], verify_oracle=True)
    assert invalid.success
    assert invalid.selected_backend == "deterministic_grid_astar_fallback"
    assert invalid.partial_dstar_result_returned is False


def test_geometry_key_is_sensitive_to_every_binding_field():
    free = np.ones((7, 9), dtype=bool)
    original = CompactCorridorGeometry.build(make_roi(free, (6, 0), (0, 8))).binding
    variants = [
        replace(original, map_hash="different"),
        replace(original, map_shape=(8, 9)),
        replace(original, map_origin=(1.0, 0.0, 0.0)),
        replace(original, resolution=0.051),
        replace(original, topology_hash="different"),
        replace(original, route_edge_ids=("different",)),
        replace(original, corridor_mask_hash="different"),
        replace(original, footprint_hash="different"),
        replace(original, safety_policy_hash="different"),
        replace(original, adjacency_rule="different"),
        replace(original, format_version=FORMAT_VERSION + 1),
    ]
    assert len({original.digest, *(variant.digest for variant in variants)}) == len(variants) + 1
    state = MutableStateBinding(original.digest, (6, 0), (0, 8), "baseline")
    state_variants = [
        replace(state, geometry_hash="different"),
        replace(state, start_cell=(5, 0)),
        replace(state, goal_cell=(0, 7)),
        replace(state, dynamic_baseline_version="different"),
        replace(state, algorithm_version="different"),
        replace(state, format_version=FORMAT_VERSION + 1),
    ]
    assert len({state.digest, *(variant.digest for variant in state_variants)}) == len(state_variants) + 1


def test_corrupt_truncated_and_schema_mismatch_geometry_cache_rejected(tmp_path):
    roi = make_roi(np.ones((7, 9), dtype=bool), (6, 0), (0, 8))
    geometry = CompactCorridorGeometry.build(roi)
    cache = VerifiedGeometryCache(tmp_path)
    cache.save(geometry)
    payload, manifest_path = cache._paths(geometry.binding.digest)

    original_payload = payload.read_bytes()
    payload.write_bytes(original_payload[:17])
    assert cache.restore(geometry.binding)[1].reject_reason == "CONTENT_HASH_MISMATCH"

    payload.write_bytes(original_payload)
    manifest = json.loads(manifest_path.read_text())
    manifest["schema_version"] = GEOMETRY_SCHEMA + "-future"
    manifest_path.write_text(json.dumps(manifest))
    assert cache.restore(geometry.binding)[1].reject_reason == "SCHEMA_MISMATCH"

    cache.save(geometry)
    payload.write_bytes(b"corrupt")
    assert cache.restore(geometry.binding)[1].reject_reason == "CONTENT_HASH_MISMATCH"


def test_corrupt_and_schema_mismatch_mutable_state_cache_rejected(tmp_path):
    roi = make_roi(np.ones((7, 9), dtype=bool), (6, 0), (0, 8))
    planner = make_planner(roi)
    assert planner.initialize().success
    cache = VerifiedMutableStateCache(tmp_path)
    cache.save(planner.state)
    payload, manifest_path = cache._paths(planner.state.binding.digest)

    payload.write_bytes(payload.read_bytes()[:23])
    assert cache.restore(planner.geometry, planner.state.binding)[1].reject_reason == "CONTENT_HASH_MISMATCH"

    cache.save(planner.state)
    manifest = json.loads(manifest_path.read_text())
    manifest["schema_version"] = "future-state-schema"
    manifest_path.write_text(json.dumps(manifest))
    assert cache.restore(planner.geometry, planner.state.binding)[1].reject_reason == "SCHEMA_MISMATCH"

    cache.save(planner.state)
    wrong = replace(planner.state.binding, dynamic_baseline_version="other")
    restored, telemetry = cache.restore(planner.geometry, wrong)
    assert restored is None
    assert telemetry.reject_reason == "CACHE_MISS"


def test_lru_admission_hit_eviction_and_release(tmp_path):
    free = np.ones((9, 11), dtype=bool)
    first = make_roi(free, (8, 0), (0, 10), route="r1")
    second = make_roi(free, (8, 1), (0, 9), route="r2")
    manager = L2StateLifecycleManager(tmp_path, max_active_states=1)
    planner1, result1, _ = manager.activate(first)
    assert result1.success
    _, _, hit = manager.activate(first)
    assert hit.active_hit
    reference = weakref.ref(planner1)
    del planner1
    planner2, result2, evicted = manager.activate(second)
    assert result2.success
    assert evicted.evicted_key
    assert evicted.released_resident_bytes > 0
    assert len(manager.active) == 1
    gc.collect()
    assert reference() is None
    assert manager.peak_active_state_count <= 1
    assert planner2.binding_hash != evicted.evicted_key


def test_route_endpoint_and_corridor_states_never_alias(tmp_path):
    free = np.ones((9, 11), dtype=bool)
    manager = L2StateLifecycleManager(tmp_path, max_active_states=2)
    route_a = make_roi(free, (8, 0), (0, 10), route="r1")
    route_b = make_roi(free, (8, 0), (0, 10), route="r2")
    endpoint_b = make_roi(free, (8, 1), (0, 10), route="r1")
    planner_a, _, _ = manager.activate(route_a)
    planner_b, _, _ = manager.activate(route_b)
    planner_endpoint, _, _ = manager.activate(endpoint_b)
    assert len({planner_a.binding_hash, planner_b.binding_hash, planner_endpoint.binding_hash}) == 3
    assert len(manager.active) == 2
    assert manager.peak_active_state_count <= 2


def test_hard_lru_limit_is_enforced(tmp_path):
    with pytest.raises(ValueError):
        L2StateLifecycleManager(tmp_path, max_active_states=3)
