import cv2
import numpy as np

from pudu_coverage_planner.coverage_geometry import (
    erode_mask,
    plan_coverage,
    polygon_to_mask,
    split_path_at_turns,
)


def assert_paths_are_inside(mask, paths):
    assert paths
    for path in paths:
        assert len(path) >= 2
        for x, y in path:
            ix = int(round(x))
            iy = int(round(y))
            assert 0 <= iy < mask.shape[0]
            assert 0 <= ix < mask.shape[1]
            assert mask[iy, ix]


def test_obstacle_splits_sweep_without_crossing_it():
    free = np.zeros((120, 160), dtype=bool)
    free[10:110, 10:150] = True
    free[40:80, 70:90] = False

    plan = plan_coverage(
        free,
        lane_spacing_cells=12,
        point_spacing_cells=3.0,
        min_cell_area_cells=20,
        start=(20.0, 20.0),
    )

    assert len(plan.cell_paths) >= 2
    assert_paths_are_inside(free, plan.cell_paths)


def test_polygon_inflation_keeps_robot_center_inside_boundary():
    polygon = [(8.0, 8.0), (91.0, 8.0), (91.0, 71.0), (8.0, 71.0)]
    area = polygon_to_mask((80, 100), polygon)
    navigable = erode_mask(area, 5)
    plan = plan_coverage(navigable, 10, 2.0, 10)

    assert np.count_nonzero(navigable) < np.count_nonzero(area)
    assert_paths_are_inside(navigable, plan.cell_paths)


def test_rotated_room_is_covered_along_a_low_turn_axis():
    base = np.zeros((160, 160), dtype=np.uint8)
    cv2.rectangle(base, (35, 65), (125, 95), 1, thickness=-1)
    transform = cv2.getRotationMatrix2D((80, 80), 30.0, 1.0)
    room = cv2.warpAffine(base, transform, (160, 160), flags=cv2.INTER_NEAREST).astype(bool)

    plan = plan_coverage(room, 8, 2.0, 20)

    assert min(abs(plan.sweep_rotation_deg - 30.0),
               abs(plan.sweep_rotation_deg - 120.0)) <= 15.0
    assert_paths_are_inside(room, plan.cell_paths)


def test_tight_lane_connectors_are_not_sent_as_one_u_turn_path():
    u_path = [
        (0.0, 0.0), (5.0, 0.0),
        (5.0, 1.0),
        (0.0, 1.0),
        (0.0, 2.0),
        (5.0, 2.0),
    ]

    pieces = split_path_at_turns(
        u_path, turn_angle_rad=0.60, min_length_cells=3.0)

    assert len(pieces) == 3
    assert all(len(piece) == 2 for piece in pieces)
