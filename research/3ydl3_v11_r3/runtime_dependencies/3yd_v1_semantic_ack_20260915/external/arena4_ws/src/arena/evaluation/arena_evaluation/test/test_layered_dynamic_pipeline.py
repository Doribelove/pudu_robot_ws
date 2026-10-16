from types import SimpleNamespace

import numpy as np

from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.layered_dynamic_pipeline import L3Result, LayeredDynamicPipeline
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation.topology import TopologyArtifact, TopologyEdge, TopologyGraph, TopologyNode
from arena_evaluation.layered_dynamic_pipeline_cli import main


class FakeMap:
    resolution = 1.0
    origin = (0.0, 0.0, 0.0)
    width = 15
    height = 15

    def world_to_cell(self, x, y):
        row, col = int(round(y)), int(round(x))
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def cell_to_world(self, cell):
        return float(cell[1]), float(cell[0])

    def footprint_collision(self, *args, **kwargs):
        return False


class FakeL3:
    def __init__(self):
        self.calls = 0

    def plan(self, query, grid_path, snapshot, *, corridor_mask, topology_artifact):
        self.calls += 1
        points = []
        for index, cell in enumerate(grid_path):
            x, y = topology_artifact.hospital_map.cell_to_world(cell)
            points.append({"x": x, "y": y, "yaw": 0.0})
        return L3Result(True, points, diagnostics={"snapshot_id": snapshot.snapshot_id})


def _artifact():
    nodes = [
        TopologyNode(index, float(index + 1), 7.0, index + 1, 7, 2, 2.0, 4.0, 0)
        for index in range(5)
    ]
    edges = [
        TopologyEdge(index, index, index + 1, 1.0, 2.0, 2.0, 4.0, 2, [[float(index + 1), 7.0], [float(index + 2), 7.0]])
        for index in range(4)
    ]
    free = np.ones((15, 15), dtype=bool)
    graph = TopologyGraph(nodes, edges)
    return TopologyArtifact(FakeMap(), free, free.copy(), np.ones_like(free, dtype=np.float32), np.ones_like(free, dtype=np.int32), graph, {})


def test_pipeline_initial_and_unaffected_dynamic_update():
    l3 = FakeL3()
    pipeline = LayeredDynamicPipeline(_artifact(), footprint=[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]], l3_planner=l3, corridor_padding_m=1.0, endpoint_radius_m=1.1)
    query = Query("q", [1.0, 7.0, 0.0], [5.0, 7.0, 0.0], "", 0, "VALID")
    initial = pipeline.plan_initial(query)
    assert initial.success
    assert l3.calls == 1
    unaffected = pipeline.update_dynamic(query, DynamicSnapshot.from_cells("off", [(0, 0)], map_shape=(15, 15)))
    assert unaffected.success
    assert unaffected.diagnostics["dynamic_replan_triggered"] is False
    assert l3.calls == 1


def test_pipeline_triggers_l2_repair_for_ahead_obstacle():
    l3 = FakeL3()
    pipeline = LayeredDynamicPipeline(_artifact(), footprint=[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]], l3_planner=l3, corridor_padding_m=1.0, endpoint_radius_m=1.1)
    query = Query("q", [1.0, 7.0, 0.0], [5.0, 7.0, 0.0], "", 0, "VALID")
    initial = pipeline.plan_initial(query)
    assert initial.success
    updated = pipeline.update_dynamic(query, DynamicSnapshot.from_cells("block", [(7, 3)], map_shape=(15, 15)))
    assert updated.diagnostics["dynamic_replan_triggered"] is True
    assert updated.success
    assert l3.calls == 2


def test_3d_v0_cli_demo(capsys):
    assert main(["--demo"]) == 0
    assert '"architecture_id": "3D-V0"' in capsys.readouterr().out
