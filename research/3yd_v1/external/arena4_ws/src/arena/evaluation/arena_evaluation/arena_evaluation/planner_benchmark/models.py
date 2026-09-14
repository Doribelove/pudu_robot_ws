from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


RESULT_CODES = {
    "SUCCEEDED",
    "SERVER_UNAVAILABLE",
    "ACTION_REJECTED",
    "ACTION_ABORTED",
    "ACTION_CANCELED",
    "CLIENT_TIMEOUT",
    "EMPTY_PATH",
    "INVALID_START",
    "INVALID_GOAL",
    "EXCEPTION",
}


@dataclass(frozen=True)
class Query:
    query_id: str
    start: List[float]
    goal: List[float]
    category: str = "unspecified"
    seed: int = 0
    validation_status: str = "UNVALIDATED"

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class QueryValidation:
    query_id: str
    config_variant: str
    validation_status: str
    start_status: str
    goal_status: str
    connected: bool
    start_clearance_m: Optional[float] = None
    goal_clearance_m: Optional[float] = None
    reason: str = ""
    suggested_start: Optional[List[float]] = None
    suggested_goal: Optional[List[float]] = None

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ResourceSnapshot:
    cpu_user_ms: Optional[float] = None
    cpu_system_ms: Optional[float] = None
    rss_bytes: Optional[int] = None
    pss_bytes: Optional[int] = None


@dataclass
class ResourceMeasurement:
    planner_cpu_user_ms: Optional[float] = None
    planner_cpu_system_ms: Optional[float] = None
    planner_cpu_total_ms: Optional[float] = None
    planner_cpu_percent: Optional[float] = None
    planner_rss_before_bytes: Optional[int] = None
    planner_rss_peak_bytes: Optional[int] = None
    planner_pss_before_bytes: Optional[int] = None
    planner_pss_peak_bytes: Optional[int] = None
    stack_rss_before_bytes: Optional[int] = None
    stack_rss_peak_bytes: Optional[int] = None
    stack_pss_before_bytes: Optional[int] = None
    stack_pss_peak_bytes: Optional[int] = None
    sample_interval_ms: Optional[float] = None
    process_error: str = ""


@dataclass
class RunRecord:
    run_id: str
    timestamp: str
    map_id: str
    map_sha256: str
    query_id: str
    query_category: str
    planner_id: str
    config_variant: str
    planner_config_sha256: str
    repetition: int
    run_mode: str
    start_x: float
    start_y: float
    start_yaw: float
    goal_x: float
    goal_y: float
    goal_yaw: float
    action_status: str = ""
    result_code: str = "EXCEPTION"
    result_detail: str = ""
    planning_time_ms: Optional[float] = None
    wall_time_ms: Optional[float] = None
    cpu_user_ms: Optional[float] = None
    cpu_system_ms: Optional[float] = None
    cpu_total_ms: Optional[float] = None
    cpu_percent: Optional[float] = None
    planner_rss_before_bytes: Optional[int] = None
    planner_rss_peak_bytes: Optional[int] = None
    planner_pss_before_bytes: Optional[int] = None
    planner_pss_peak_bytes: Optional[int] = None
    stack_rss_before_bytes: Optional[int] = None
    stack_rss_peak_bytes: Optional[int] = None
    stack_pss_before_bytes: Optional[int] = None
    stack_pss_peak_bytes: Optional[int] = None
    sample_interval_ms: Optional[float] = None
    path_point_count: int = 0
    path_file: str = ""
    resource_error: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class PathMetric:
    run_id: str
    query_id: str
    planner_id: str
    config_variant: str
    path_length_m: Optional[float] = None
    euclidean_distance_m: Optional[float] = None
    length_over_euclidean: Optional[float] = None
    length_over_navfn: Optional[float] = None
    length_over_shortest_observed_valid: Optional[float] = None
    minimum_clearance_m: Optional[float] = None
    clearance_p05_m: Optional[float] = None
    clearance_p50_m: Optional[float] = None
    footprint_collision_count: int = 0
    footprint_collision_length_m: float = 0.0
    heading_change_p95_rad: Optional[float] = None
    heading_change_max_rad: Optional[float] = None
    curvature_p95_per_m: Optional[float] = None
    curvature_max_per_m: Optional[float] = None
    preferred_radius_violation_count: int = 0
    in_place_rotation_count: int = 0
    reverse_distance_m: float = 0.0
    reverse_ratio: float = 0.0
    direction_switch_count: int = 0
    goal_position_error_m: Optional[float] = None
    goal_yaw_error_rad: Optional[float] = None

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)
