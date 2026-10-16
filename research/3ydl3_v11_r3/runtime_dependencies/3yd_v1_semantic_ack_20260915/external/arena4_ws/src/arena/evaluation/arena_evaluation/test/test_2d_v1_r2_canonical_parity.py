import json
from pathlib import Path

import pytest

from arena_evaluation import canonical_path_validation as canonical
from arena_evaluation import two_layer_2d_v1_formal_benchmark as benchmark
from arena_evaluation import unified_four_backends_smoke as legacy


R1_EXPERIMENT = Path(
    "/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/"
    "2d_v1_mentor_map_20260825_005_20_performance_r1_maskaligned_v2"
)


@pytest.mark.skipif(not (R1_EXPERIMENT / "paths").is_dir(), reason="r1 saved paths unavailable")
def test_all_r1_saved_paths_match_legacy_validation_and_collision_diagnostics():
    ctx = benchmark._context()
    queries, _metadata = benchmark._load_tasks()
    query_by_id = {query.query_id: query for query in queries}
    checked = 0
    a2b07_checked = 0
    for path_file in sorted((R1_EXPERIMENT / "paths").glob("*.json")):
        query_id = next((query_id for query_id in query_by_id if f"_{query_id}_" in path_file.name), None)
        assert query_id is not None
        points = json.loads(path_file.read_text(encoding="utf-8"))
        old = legacy.validate_path(ctx, query_by_id[query_id], points)
        old_collision = benchmark._collision_diagnostics(ctx, points)
        new = canonical.canonical_validate_path(ctx, query_by_id[query_id], points)
        for field in (
            "static_footprint_valid", "kinematic_valid", "path_length_m",
            "minimum_clearance_m", "maximum_curvature", "curvature_p95",
            "heading_discontinuity_count", "position_discontinuity_count",
            "steering_jump_count", "reverse_distance_m", "in_place_rotation_count",
            "start_position_error_m", "start_yaw_error_rad", "goal_position_error_m",
            "goal_yaw_error_rad", "failure_code", "failure_detail",
        ):
            if isinstance(old[field], float):
                assert new[field] == pytest.approx(old[field], rel=0.0, abs=1.0e-12), (path_file, field)
            else:
                assert new[field] == old[field], (path_file, field)
        assert new["final_valid_success"] == bool(
            old["static_footprint_valid"] and old["kinematic_valid"]
        )
        for field in ("collision_count", "collision_segment_indices", "collision_positions"):
            assert new[field] == old_collision[field], (path_file, field)
        if query_id == "A2B-07":
            assert new["failure_code"] == "STATIC_FOOTPRINT_COLLISION"
            assert new["collision_count"] > 0
            a2b07_checked += 1
        checked += 1
    assert checked == 152
    assert a2b07_checked == 8
