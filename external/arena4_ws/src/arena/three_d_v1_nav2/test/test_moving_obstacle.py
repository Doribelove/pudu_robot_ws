import pytest

from three_d_v1_nav2.moving_obstacle_node import (
    four_independent_specs,
    segment_pose,
    triangular_position,
)


def test_triangular_motion_is_continuous_and_reverses_at_bounds():
    assert triangular_position(0.0, 5.8, 10.2, 0.3) == pytest.approx(
        (5.8, 1))
    one_way = (10.2 - 5.8) / 0.3
    assert triangular_position(one_way, 5.8, 10.2, 0.3) == pytest.approx(
        (10.2, 1))
    halfway_back = one_way + one_way / 2.0
    assert triangular_position(
        halfway_back, 5.8, 10.2, 0.3) == pytest.approx((8.0, -1))
    assert triangular_position(
        2.0 * one_way, 5.8, 10.2, 0.3) == pytest.approx((5.8, 1))


@pytest.mark.parametrize(
    "lower,upper,speed",
    [(1.0, 1.0, 0.3), (2.0, 1.0, 0.3), (1.0, 2.0, 0.0)],
)
def test_triangular_motion_refuses_invalid_contract(lower, upper, speed):
    with pytest.raises(ValueError):
        triangular_position(0.0, lower, upper, speed)


def test_four_independent_profile_has_four_unique_crossings_and_phases():
    specs = four_independent_specs()
    assert len(specs) == 4
    assert len({spec.entity for spec in specs}) == 4
    assert len({spec.speed_mps for spec in specs}) == 4
    assert len({spec.phase_s for spec in specs}) == 4
    assert [spec.confirmation for spec in specs] == [
        "initial", "delayed", "delayed", "delayed"]
    for spec in specs:
        x, y, direction = segment_pose(spec, 0.0)
        assert direction in (-1, 1)
        assert min(spec.x0, spec.x1) - 1e-9 <= x <= max(spec.x0, spec.x1) + 1e-9
        assert min(spec.y0, spec.y1) - 1e-9 <= y <= max(spec.y0, spec.y1) + 1e-9
        assert spec.span_m > 1.0
