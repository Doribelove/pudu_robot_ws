from __future__ import annotations

import math

import pytest
import numpy as np
import yaml
from PIL import Image

from arena_evaluation.relaxed_single_planner_benchmark import (
    RelaxedAckermannConfig,
    _derived_radius,
    _as_bool,
    _code_manifest,
    _hybrid_astar,
    MapContext,
    FOOTPRINT,
)
from arena_evaluation.planner_benchmark.map_utils import HospitalMap, sha256_file
from arena_evaluation.planner_benchmark.models import Query
from arena_evaluation.topology import preprocess_static_map


def test_relaxed_radius_is_derived_from_wheelbase_and_steering():
    radius, curvature = _derived_radius(0.50, 60.0)
    assert radius == pytest.approx(0.2886751346, rel=1e-8)
    assert curvature == pytest.approx(3.464101615, rel=1e-8)
    config = RelaxedAckermannConfig()
    assert config.minimum_turning_radius_m == pytest.approx(radius, rel=1e-8)
    assert config.maximum_curvature_per_m == pytest.approx(curvature, rel=1e-8)


def test_relaxed_lattice_metadata_is_fixed_and_forward_only():
    config = RelaxedAckermannConfig()
    assert config.hybrid_step_size_m == pytest.approx(0.25)
    assert config.angle_resolution_deg == pytest.approx(5.0)
    assert config.angle_bins == 72
    assert config.steering_angles_deg == (-60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0)
    assert config.integration_sample_spacing_m <= 0.05
    assert config.allow_reverse is False
    assert config.allow_in_place_rotation is False


def test_inconsistent_derived_vehicle_parameters_are_rejected():
    with pytest.raises(ValueError):
        RelaxedAckermannConfig(minimum_turning_radius_m=0.4)
    with pytest.raises(ValueError):
        RelaxedAckermannConfig(maximum_curvature_per_m=2.5)


def test_csv_false_values_are_not_truthy():
    assert _as_bool(False) is False
    assert _as_bool("False") is False
    assert _as_bool("0") is False
    assert _as_bool("true") is True


def test_hybrid_timeout_keeps_v2_diagnostics(tmp_path):
    image_path = tmp_path / "map.pgm"
    Image.fromarray(np.full((160, 160), 254, dtype=np.uint8)).save(image_path)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(yaml.safe_dump({
        "image": image_path.name, "resolution": 0.05, "origin": [-4.0, -4.0, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }))
    hospital_map = HospitalMap.load(yaml_path)
    _, free, distance, _ = preprocess_static_map(hospital_map, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False)
    context = MapContext("tiny", hospital_map, free, distance, sha256_file(image_path), sha256_file(yaml_path), {})
    query = Query("q", [-3.0, -3.0, 0.0], [3.0, 3.0, 0.0])
    result = _hybrid_astar(context, query, RelaxedAckermannConfig(), 0.0)
    assert result.failure_code == "TIMEOUT"
    assert result.angle_resolution_deg == pytest.approx(5.0)
    assert result.step_size_m == pytest.approx(0.25)
    assert result.diagnostics["angle_bins"] == 72
    assert result.diagnostics["steering_angles_deg"] == list(RelaxedAckermannConfig().steering_angles_deg)


def test_code_manifest_refresh_preserves_completed_run_provenance(tmp_path):
    (tmp_path / "protocol.yaml").write_text("schema_version: 2\n", encoding="utf-8")
    (tmp_path / "core_queries_v1.yaml").write_text("queries: []\n", encoding="utf-8")
    (tmp_path / "code_manifest.yaml").write_text(yaml.safe_dump({
        "command": ["planner", "--stage", "run"],
        "started_at": "2026-08-24T00:00:00+00:00",
        "ended_at": "2026-08-24T01:00:00+00:00",
    }), encoding="utf-8")
    _code_manifest(tmp_path, ["planner", "--stage", "report"])
    recorded = yaml.safe_load((tmp_path / "code_manifest.yaml").read_text(encoding="utf-8"))
    assert recorded["command"] == ["planner", "--stage", "run"]
    assert recorded["started_at"] == "2026-08-24T00:00:00+00:00"
    assert recorded["ended_at"] == "2026-08-24T01:00:00+00:00"
