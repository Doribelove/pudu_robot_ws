from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from arena_evaluation.padded_map import prepare_padded_map


SOURCE = Path("/home/robot/pudu_robot_ws/experiments/maps/hospital_005/map.yaml")


def test_padded_map_preserves_source_block_and_world_bounds(tmp_path):
    output = prepare_padded_map(SOURCE, 100.0, None, tmp_path / "hospital_boundary_100", target_resolution=0.05)
    source_yaml = yaml.safe_load(SOURCE.read_text())
    source_image = np.asarray(Image.open(SOURCE.parent / source_yaml["image"]).convert("L"))
    target_image = np.asarray(Image.open(output / "map.pgm").convert("L"))
    assert target_image.shape == (2000, 2000)
    derived_source = target_image[200:1800, 200:1800]
    assert not np.array_equal(derived_source, source_image)
    config = yaml.safe_load((output / "map.yaml").read_text())
    assert config["origin"] == [-50.0, -50.0, 0.0]
    metadata = yaml.safe_load((output / "metadata.yaml").read_text())
    assert metadata["source_world_bounds"] == [-40.0, -40.0, 40.0, 40.0]
    assert metadata["target_world_bounds"] == [-50.0, -50.0, 50.0, 50.0]
    assert metadata["source_block_preserved_exactly"] is False
    assert metadata["source_unchanged_outside_gates"] is True
    assert metadata["gate_count"] == 10
    assert metadata["gate_width_m"] == 1.0
    assert metadata["outer_free_region_connected"] is True
    assert metadata["outer_free_region_connected_to_source_query_space"] is True
    assert yaml.safe_load((output / "validation.yaml").read_text())["source_block_exact_match"] is False


def test_padded_map_boundary_is_free_and_dynamic_free(tmp_path):
    output = prepare_padded_map(SOURCE, 200.0, None, tmp_path / "hospital_boundary_200")
    image = np.asarray(Image.open(output / "map.pgm").convert("L"))
    assert image[0, 0] == 254
    assert image[100, 100] == 254
    metadata = yaml.safe_load((output / "metadata.yaml").read_text())
    assert metadata["boundary_semantics"] == "free"
    assert metadata["dynamic_obstacles"] is False
    validation = yaml.safe_load((output / "validation.yaml").read_text())
    assert validation["outer_free_region_connected_to_source_query_space"] is True


def test_no_gate_mode_keeps_source_block_exact(tmp_path):
    output = prepare_padded_map(
        SOURCE, 100.0, None, tmp_path / "hospital_boundary_no_gates",
        target_resolution=0.05, connect_gates=False,
    )
    source_yaml = yaml.safe_load(SOURCE.read_text())
    source_image = np.asarray(Image.open(SOURCE.parent / source_yaml["image"]).convert("L"))
    target_image = np.asarray(Image.open(output / "map.pgm").convert("L"))
    assert np.array_equal(target_image[200:1800, 200:1800], source_image)
    metadata = yaml.safe_load((output / "metadata.yaml").read_text())
    assert metadata["gate_count"] == 0
    assert metadata["outer_free_region_connected_to_source_query_space"] is False
