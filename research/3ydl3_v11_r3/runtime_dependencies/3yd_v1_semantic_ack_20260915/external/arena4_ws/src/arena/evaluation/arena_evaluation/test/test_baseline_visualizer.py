import math

import pytest

from nav_msgs.msg import OccupancyGrid

from arena_evaluation.baseline_visualizer import (
    costmap_is_ready,
    pose_stamped,
    select_query,
    select_queries,
)
from arena_evaluation.planner_benchmark.models import Query


def test_select_query_returns_requested_query():
    queries = [
        Query("q00", [0.0, 0.0, 0.0], [1.0, 1.0, 0.0]),
        Query("q03", [2.0, 3.0, 0.5], [4.0, 5.0, -0.5]),
    ]

    assert select_query(queries, "q03").query_id == "q03"


def test_select_query_rejects_unknown_id():
    with pytest.raises(ValueError, match="unknown query_id"):
        select_query([Query("q00", [0.0] * 3, [1.0] * 3)], "q99")


def test_select_queries_preserves_all_query_order():
    queries = [
        Query("q00", [0.0] * 3, [1.0] * 3),
        Query("q01", [2.0] * 3, [3.0] * 3),
    ]

    assert [query.query_id for query in select_queries(queries, "all")] == [
        "q00",
        "q01",
    ]
    assert [query.query_id for query in select_queries(queries, "q01")] == [
        "q01"
    ]


def test_pose_stamped_converts_yaw_to_quaternion():
    pose = pose_stamped([1.5, -2.5, math.pi / 2.0])

    assert pose.header.frame_id == "map"
    assert pose.pose.position.x == pytest.approx(1.5)
    assert pose.pose.position.y == pytest.approx(-2.5)
    assert pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_costmap_requires_free_and_lethal_static_cells():
    message = OccupancyGrid()
    message.info.width = 2
    message.info.height = 2

    message.data = [-1, -1, -1, -1]
    assert not costmap_is_ready(message)

    message.data = [0, 0, 0, 0]
    assert not costmap_is_ready(message)

    message.data = [0, 0, 99, 100]
    assert costmap_is_ready(message)


def test_costmap_rejects_incomplete_grid():
    message = OccupancyGrid()
    message.info.width = 2
    message.info.height = 2
    message.data = [0, 100]

    assert not costmap_is_ready(message)
