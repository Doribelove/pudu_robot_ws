from types import SimpleNamespace

import numpy as np

from arena_evaluation import l1_l3_corridor_hybrid_smoke as candidate
from arena_evaluation import unified_four_backends_smoke as legacy
from arena_evaluation.planner_benchmark.models import Query


class _Map:
    occupancy = np.zeros((4, 4), dtype=np.int8)
    resolution = 0.05

    def world_to_cell(self, x, y):
        return (1, 1) if float(x) < 0.5 else (0, 0)


def _context():
    return SimpleNamespace(
        hospital_map=_Map(),
        free_mask=np.ones((4, 4), dtype=bool),
        map_sha256="map",
        map_yaml_sha256="yaml",
    )


def _query():
    return Query("q02", [0.0, 0.0, 0.0], [0.1, 0.1, 0.0])


def _spec():
    return legacy.BackendSpec(
        "hybrid_astar", "Nav2 SmacPlannerHybrid DUBIN", "test", True, "", True,
    )


def test_l1_l3_never_calls_grid_and_passes_corridor_mask(monkeypatch):
    ctx = _context()
    topology = SimpleNamespace()
    route = SimpleNamespace(node_ids=[1, 2], edge_ids=[3], length_m=2.0, min_width_m=1.0)
    allowed = np.zeros((4, 4), dtype=bool)
    allowed[1, 1] = True

    monkeypatch.setattr(legacy, "plan_grid_astar", lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("L2 must not be called")))
    monkeypatch.setattr(legacy, "attach_pose", lambda *args, **kwargs: SimpleNamespace(node_id=1, component_id=0))
    monkeypatch.setattr(legacy, "search_topology", lambda *args, **kwargs: route)
    monkeypatch.setattr(candidate, "corridor_mask", lambda *args, **kwargs: allowed)

    class Session:
        def __init__(self):
            self.allowed = None
            self.calls = 0

        def plan(self, query, spec, **kwargs):
            self.calls += 1
            self.allowed = kwargs["allowed_mask"]
            return legacy.PlanResult(
                planner_success=False,
                failure_code="CLIENT_TIMEOUT",
                planner_backend=spec.backend,
                backend_version=spec.version,
                source=candidate.L3_PRIME_SOURCE,
                diagnostics={"backend_called": True, "backend_call_count": 1},
            )

    session = Session()
    result, diagnostics = candidate.plan_l1_l3_corridor_hybrid(
        ctx, _query(), topology, session, _spec(),
        corridor_semantics="inflated_l1_legacy",
    )
    assert session.calls == 1
    assert np.array_equal(session.allowed, allowed)
    assert diagnostics["l2_called"] is False
    assert diagnostics["l2_call_count"] == 0
    assert diagnostics["l3_prime_call_count"] == 1
    assert result.failure_code == "PLANNER_TIMEOUT"


def test_path_leaving_corridor_cannot_be_success(monkeypatch):
    ctx = _context()
    topology = SimpleNamespace()
    route = SimpleNamespace(node_ids=[1], edge_ids=[], length_m=1.0, min_width_m=1.0)
    allowed = np.zeros((4, 4), dtype=bool)
    allowed[1, 1] = True
    monkeypatch.setattr(legacy, "attach_pose", lambda *args, **kwargs: SimpleNamespace(node_id=1, component_id=0))
    monkeypatch.setattr(legacy, "search_topology", lambda *args, **kwargs: route)
    monkeypatch.setattr(candidate, "corridor_mask", lambda *args, **kwargs: allowed)

    points = [{
        "x": 0.0, "y": 0.0, "yaw": 0.0,
        "source": candidate.L3_PRIME_SOURCE,
        "motion_direction": "forward", "steering": 0.0,
        "planner_backend": "Nav2 SmacPlannerHybrid DUBIN",
        "backend_version": "test", "source_commit": "test", "path_hash": "test",
    }, {
        "x": 1.0, "y": 1.0, "yaw": 0.0,
        "source": candidate.L3_PRIME_SOURCE,
        "motion_direction": "forward", "steering": 0.0,
        "planner_backend": "Nav2 SmacPlannerHybrid DUBIN",
        "backend_version": "test", "source_commit": "test", "path_hash": "test",
    }]

    class Session:
        def plan(self, query, spec, **kwargs):
            return legacy.PlanResult(
                planner_success=True, points=points,
                planner_backend=spec.backend, backend_version=spec.version,
                source=candidate.L3_PRIME_SOURCE,
                diagnostics={"backend_called": True, "backend_call_count": 1},
            )

    result, diagnostics = candidate.plan_l1_l3_corridor_hybrid(
        ctx, _query(), topology, Session(), _spec(),
        corridor_semantics="inflated_l1_legacy",
    )
    assert result.planner_success is False
    assert result.failure_code == "L3_PRIME_PATH_OUTSIDE_CORRIDOR"
    assert diagnostics["l3_prime_call_count"] == 1


def test_candidate_protocol_disables_optional_backends():
    parser = candidate.build_parser()
    args = parser.parse_args(["--no-dynamic-obstacles"])
    assert args.map_ids is None
    assert candidate.ARCHITECTURE == "l1_l3_corridor_hybrid"
    assert candidate.RAW_QUERY_IDS == ("q02", "q06", "q07", "q09")


def test_raw_smac_aligned_mask_uses_raw_free_cells_and_fixed_profile(monkeypatch):
    ctx = _context()
    ctx.hospital_map.occupancy[0, 0] = 100
    topology = SimpleNamespace(free_mask=np.ones((4, 4), dtype=bool))
    route = SimpleNamespace(node_ids=[1], edge_ids=[], length_m=1.0, min_width_m=1.0,
                            polyline=[[0.0, 0.0]])
    monkeypatch.setattr(legacy, "attach_pose", lambda *args, **kwargs: SimpleNamespace(node_id=1, component_id=0))
    monkeypatch.setattr(legacy, "search_topology", lambda *args, **kwargs: route)

    class Session:
        def plan(self, query, spec, **kwargs):
            self.allowed = kwargs["allowed_mask"]
            return legacy.PlanResult(
                planner_success=False, failure_code="NO_PATH_IN_CORRIDOR",
                planner_backend=spec.backend, backend_version=spec.version,
                source=candidate.L3_PRIME_SOURCE,
                diagnostics={"backend_called": True, "backend_call_count": 1},
            )

    session = Session()
    _result, diagnostics = candidate.plan_l1_l3_corridor_hybrid(
        ctx, _query(), topology, session, _spec(), corridor_padding_m=2.0,
        corridor_semantics=candidate.CORRIDOR_SEMANTICS,
    )
    assert session.allowed.shape == ctx.hospital_map.occupancy.shape
    assert not session.allowed[0, 0]
    assert diagnostics["corridor_semantics"] == candidate.CORRIDOR_SEMANTICS
    assert diagnostics["corridor_padding_m"] == 2.0
    assert diagnostics["raw_start_occupancy"] == 0
    assert diagnostics["smac_start_cost"] == "not_available"


def test_failure_classifier_distinguishes_lethal_and_no_path():
    lethal, started, _detail = candidate._classify_smac_failure(
        "ACTION_ABORTED", {}, "GridBased: Starting point in lethal space!"
    )
    assert lethal == "START_IN_LETHAL_SPACE"
    assert started is False
    no_path, started, _detail = candidate._classify_smac_failure(
        "ACTION_ABORTED", {}, "Smac: no valid path found"
    )
    assert no_path == "NO_PATH_IN_CORRIDOR"
    assert started is True


def test_profile_sweep_is_fixed_and_contains_requested_padding_values():
    specs = candidate._profile_specs()
    assert [item[2] for item in specs if item[1] == candidate.CORRIDOR_SEMANTICS] == [1.0, 2.0, 4.0]
    assert specs[0][1] == "raw_full_map"


def test_local_attempt_validation_adds_provenance_without_rewriting_pose(monkeypatch):
    ctx = _context()
    topology = SimpleNamespace()
    route = SimpleNamespace(node_ids=[1], edge_ids=[], length_m=1.0, min_width_m=1.0)
    allowed = np.ones((4, 4), dtype=bool)
    monkeypatch.setattr(legacy, "attach_pose", lambda *args, **kwargs: SimpleNamespace(node_id=1, component_id=0))
    monkeypatch.setattr(legacy, "search_topology", lambda *args, **kwargs: route)
    monkeypatch.setattr(candidate, "corridor_mask", lambda *args, **kwargs: allowed)
    original = [{"x": 0.0, "y": 0.0, "yaw": 0.0, "steering": 0.0,
                 "motion_direction": "forward", "source": candidate.L3_PRIME_SOURCE,
                 "planner_backend": "smac", "backend_version": "test"},
                {"x": 0.1, "y": 0.1, "yaw": 0.0, "steering": 0.0,
                 "motion_direction": "forward", "source": candidate.L3_PRIME_SOURCE,
                 "planner_backend": "smac", "backend_version": "test"}]

    class Session:
        def plan(self, query, spec, **kwargs):
            return legacy.PlanResult(
                planner_success=True, points=[dict(item) for item in original],
                planner_backend=spec.backend, backend_version=spec.version,
                source=candidate.L3_PRIME_SOURCE,
                diagnostics={"backend_called": True, "backend_call_count": 1},
            )

    monkeypatch.setattr(legacy, "validate_path", lambda _ctx, _query, points: {
        "static_footprint_valid": True, "kinematic_valid": True,
        "final_valid_success": True, "failure_code": "", "failure_detail": "",
    } if all("source_commit" in point and "path_hash" in point for point in points) else {
        "static_footprint_valid": False, "kinematic_valid": False,
        "final_valid_success": False, "failure_code": "PATH_SCHEMA_INVALID", "failure_detail": "",
    })
    result, diagnostics = candidate.plan_l1_l3_corridor_hybrid(
        ctx, _query(), topology, Session(), _spec(),
        corridor_semantics="inflated_l1_legacy", validate_each_attempt=True,
    )
    assert result.planner_success
    assert diagnostics["l3_prime_call_count"] == 1
    assert result.points[0]["x"] == original[0]["x"]
    assert result.points[-1]["y"] == original[-1]["y"]


def test_last_costmap_and_action_state_are_promoted_from_attempt(monkeypatch):
    ctx = _context()
    topology = SimpleNamespace()
    route = SimpleNamespace(node_ids=[1], edge_ids=[], length_m=1.0, min_width_m=1.0)
    allowed = np.ones((4, 4), dtype=bool)
    monkeypatch.setattr(legacy, "attach_pose", lambda *args, **kwargs: SimpleNamespace(node_id=1, component_id=0))
    monkeypatch.setattr(legacy, "search_topology", lambda *args, **kwargs: route)
    monkeypatch.setattr(candidate, "corridor_mask", lambda *args, **kwargs: allowed)

    class Session:
        def plan(self, query, spec, **kwargs):
            return legacy.PlanResult(
                planner_success=False, failure_code="ACTION_ABORTED",
                planner_backend=spec.backend, backend_version=spec.version,
                source=candidate.L3_PRIME_SOURCE,
                diagnostics={
                    "backend_called": True, "backend_call_count": 1,
                    "action_status": "ABORTED", "action_result_code": "ACTION_ABORTED",
                    "previous_mask_hash": "before", "applied_mask_hash": "after",
                    "expected_mask_hash": "expected", "local_map_update_ms": 12.5,
                    "local_map_update_messages": 2, "local_map_update_mode": "delta",
                    "local_map_update_fallback": False,
                },
            )

    result, diagnostics = candidate.plan_l1_l3_corridor_hybrid(
        ctx, _query(), topology, Session(), _spec(),
        corridor_semantics="inflated_l1_legacy", padding_schedule_m=(1.0, 2.0),
    )
    assert result.failure_code == "ACTION_ABORTED"
    assert diagnostics["costmap_update_before_hash"] == "before"
    assert diagnostics["costmap_update_after_hash"] == "after"
    assert diagnostics["costmap_update_expected_hash"] == "expected"
    assert diagnostics["costmap_update_time_ms"] == 12.5
    assert diagnostics["costmap_update_messages"] == 2
    assert diagnostics["costmap_update_mode"] == "delta"
    assert diagnostics["action_status"] == "ABORTED"
    assert diagnostics["action_result_code"] == "ACTION_ABORTED"
