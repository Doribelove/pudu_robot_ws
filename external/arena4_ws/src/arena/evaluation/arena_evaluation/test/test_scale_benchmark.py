from pathlib import Path

import yaml

from arena_evaluation.scale_benchmark import (
    MAP_SPECS,
    _scaled_queries,
    _protocol,
    prepare_inputs,
)


def test_scaled_queries_preserve_yaw_and_scale_xy():
    payload = _scaled_queries("hospital_200x200_005")
    q00 = next(item for item in payload["queries"] if item["query_id"] == "q00")
    assert q00["start"] == [2.625, 3.375, -2.658]
    assert q00["goal"] == [-27.875, -12.875, 0.483]


def test_protocol_has_static_scale_contract():
    protocol = _protocol("hospital_400x400_005")
    assert protocol["dynamic_obstacles"] is False
    assert protocol["resolution"] == 0.05
    assert protocol["grid_cells"] == 8000 * 8000
    assert protocol["modes"] == ["full_grid", "topology_guided_grid_fallback", "layered_hard_radius_l3"]


def test_prepare_inputs_does_not_touch_source_maps(tmp_path):
    before = Path(MAP_SPECS["hospital_005"]["map_yaml"]).read_bytes()
    prepare_inputs(tmp_path, ["hospital_005"])
    after = Path(MAP_SPECS["hospital_005"]["map_yaml"]).read_bytes()
    assert before == after
    assert (tmp_path / "hospital_005" / "protocol.yaml").exists()
    assert yaml.safe_load((tmp_path / "hospital_005" / "queries.yaml").read_text())["map"] == "hospital_005"
