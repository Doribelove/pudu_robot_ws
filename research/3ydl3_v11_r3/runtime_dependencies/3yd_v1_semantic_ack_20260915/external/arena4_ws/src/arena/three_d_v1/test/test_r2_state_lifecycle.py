import gc
import json
import weakref

import numpy as np

from arena_3d_v1.l2_incremental import CorridorROI, deterministic_grid_astar
from arena_3d_v1.r2_state_lifecycle import (
    CacheMissAStarPlanner,
    PackedStaticMask,
    R2L2StateLifecycleManager,
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


def test_packed_static_mask_exact_parity_and_storage_reduction():
    rng = np.random.default_rng(7)
    source = rng.random((47, 59)) > 0.31
    packed = PackedStaticMask(source)
    assert np.array_equal(packed.copy(), source)
    assert packed.nbytes == (source.size + 7) // 8
    assert packed.logical_nbytes == source.nbytes
    assert packed.nbytes < source.nbytes / 7
    for cell in ((0, 0), (13, 17), (46, 58)):
        assert packed[cell] == source[cell]
    assert packed.materialize_count == 1


def test_verified_prebuild_warm_activation_and_dynamic_oracle_parity(tmp_path):
    free = np.ones((31, 41), dtype=bool)
    roi = make_roi(free, (28, 3), (2, 37))
    manager = R2L2StateLifecycleManager(tmp_path, max_active_states=1)
    prebuild = manager.prebuild(roi, verify_oracle=True)
    assert prebuild.success
    assert prebuild.first_solve_ms > 0
    planner, initial, activation = manager.activate(roi, verify_oracle=True)
    assert initial.success
    assert activation.geometry_cache.hit
    assert activation.state_cache.hit
    assert activation.synchronous_dstar_build_performed is False
    assert manager.synchronous_dstar_build_count == 0
    assert planner.state.g.dtype == np.float64
    assert planner.state.rhs.dtype == np.float64
    assert planner.static_mask.nbytes == (roi.base_free.size + 7) // 8
    assert planner.state_memory_bytes() < roi.base_free.nbytes + planner.geometry.resident_bytes + planner.state.resident_bytes

    cell = planner.path_global[len(planner.path_global) // 2]
    result = planner.update([cell], verify_oracle=True)
    assert result.success
    assert result.partial_dstar_result_returned is False
    assert result.diagnostics["synchronous_dstar_build_performed"] is False


def test_cache_miss_goes_directly_to_astar_without_admitting_or_building(tmp_path):
    free = np.ones((27, 35), dtype=bool)
    roi = make_roi(free, (24, 2), (2, 32))
    manager = R2L2StateLifecycleManager(tmp_path, max_active_states=1)
    planner, result, activation = manager.activate(roi, verify_oracle=True)
    oracle = deterministic_grid_astar(free, (24, 2), (2, 32))
    assert isinstance(planner, CacheMissAStarPlanner)
    assert result.success
    assert result.path == [roi.to_global(cell) for cell in oracle.path]
    assert result.selected_backend == "deterministic_grid_astar_cache_miss"
    assert activation.admission_backend == "deterministic_grid_astar_cache_miss"
    assert activation.synchronous_dstar_build_performed is False
    assert manager.synchronous_dstar_build_count == 0
    assert manager.cache_reject_to_astar_count == 1
    assert len(manager.active) == 0
    assert not list(tmp_path.glob("geometry/*/payload.npz"))
    assert not list(tmp_path.glob("state/*/payload.npz"))


def test_corrupted_prebuilt_state_is_rejected_to_astar_not_rebuilt(tmp_path):
    free = np.ones((19, 29), dtype=bool)
    roi = make_roi(free, (17, 1), (1, 27))
    manager = R2L2StateLifecycleManager(tmp_path)
    assert manager.prebuild(roi).success
    state_payload = next(tmp_path.glob("state/*/payload.npz"))
    state_payload.write_bytes(state_payload.read_bytes()[:31])
    planner, result, activation = manager.activate(roi, verify_oracle=True)
    assert isinstance(planner, CacheMissAStarPlanner)
    assert result.success
    assert "CONTENT_HASH_MISMATCH" in activation.state_cache.reject_reason
    assert activation.synchronous_dstar_build_performed is False
    assert manager.synchronous_dstar_build_count == 0


def test_r2_lru_route_endpoint_isolation_and_release(tmp_path):
    free = np.ones((21, 31), dtype=bool)
    first = make_roi(free, (18, 2), (2, 28), route="r1")
    second = make_roi(free, (18, 3), (2, 27), route="r2")
    manager = R2L2StateLifecycleManager(tmp_path, max_active_states=1)
    assert manager.prebuild(first).success
    assert manager.prebuild(second).success
    planner1, _, _ = manager.activate(first)
    reference = weakref.ref(planner1)
    first_key = planner1.binding_hash
    del planner1
    planner2, result2, activation2 = manager.activate(second)
    assert result2.success
    assert activation2.evicted_key == first_key
    assert activation2.released_resident_bytes > 0
    assert planner2.binding_hash != first_key
    assert len(manager.active) == 1
    gc.collect()
    assert reference() is None
    cleared = manager.clear()
    assert cleared["resident_bytes"] == 0


def test_schema_mismatch_rejects_to_astar(tmp_path):
    free = np.ones((13, 17), dtype=bool)
    roi = make_roi(free, (11, 1), (1, 15))
    manager = R2L2StateLifecycleManager(tmp_path)
    assert manager.prebuild(roi).success
    manifest_path = next(tmp_path.glob("state/*/manifest.json"))
    manifest = json.loads(manifest_path.read_text())
    manifest["schema_version"] = "future"
    manifest_path.write_text(json.dumps(manifest))
    planner, result, activation = manager.activate(roi)
    assert isinstance(planner, CacheMissAStarPlanner)
    assert result.success
    assert activation.state_cache.reject_reason == "SCHEMA_MISMATCH"
