"""Auditable four-backend and three-layer static A2B smoke benchmark.

This entry point is intentionally separate from the historical benchmark
runners.  A backend is called only when its mature implementation is actually
available.  In particular, an unavailable Smac/OMPL adapter is reported as a
structured failure; a grid path is never silently reused for an RRT or Hybrid
label.  The smoke output is diagnostic and must not be used as a performance
ranking.
"""

from __future__ import annotations

import argparse
from array import array
import csv
import datetime as dt
import hashlib
import importlib.util
import json
import math
import os
import re
import resource
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml
from PIL import Image

from .planner_benchmark.map_utils import HospitalMap, sha256_file
from .planner_benchmark.models import Query
from .planner_benchmark.resources import read_snapshot
from .topology import (
    TOPOLOGY_ALGORITHM_VERSION,
    TopologyArtifact,
    astar_grid,
    attach_pose,
    build_topology,
    corridor_mask,
    save_topology,
    search_topology,
)


ROOT = Path("/home/robot/pudu_robot_ws")
OUTPUT_NAME = "unified_four_backends_smoke_v3"
SOURCE_QUERIES = ROOT / "experiments/planner_benchmark/hospital_005/queries_v2.yaml"
MAP_PATHS = {
    "hospital_005": ROOT / "experiments/maps/hospital_005/map.yaml",
    "hospital_boundary_100x100_005": ROOT / "experiments/maps/hospital_boundary_100x100_005/map.yaml",
}
TIMEOUTS = {"hospital_005": 5.0, "hospital_boundary_100x100_005": 5.0}
FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]
WHEELBASE_M = 0.50
COLLISION_SAMPLE_SPACING_M = 0.05
COLLISION_YAW_SAMPLE_STEP_RAD = math.radians(5.0)
MAX_PATH_SAMPLE_SPACING_M = 0.15
KINEMATIC_NUMERICAL_TOLERANCE = 1.0e-3
ALGORITHMS = ("grid_astar", "geometric_rrt_star", "hybrid_astar", "kinodynamic_rrt")
LAYERED_MODES = (
    "full_grid",
    "topology_guided_grid",
    "topology_guided_grid_fallback",
    "l1_l2_geometric_rrt_star",
    "l3_hybrid_repair",
    "l3_kinodynamic_rrt",
)
CACHE_MODE_BASELINE = "baseline"
CACHE_MODE_OPTIMIZED = "optimized"


def _strict_smac_config_path() -> Path:
    source_path = Path(__file__).resolve().parents[1] / "config" / "planner_benchmark_strict_forward_smac_hybrid.yaml"
    if source_path.exists():
        return source_path
    from ament_index_python.packages import get_package_share_directory

    installed_path = (
        Path(get_package_share_directory("arena_evaluation"))
        / "config"
        / "planner_benchmark_strict_forward_smac_hybrid.yaml"
    )
    if not installed_path.exists():
        raise FileNotFoundError(f"Smac planner config not found: {installed_path}")
    return installed_path


@dataclass(frozen=True)
class BackendSpec:
    name: str
    backend: str
    version: str
    available: bool
    reason: str
    mature: bool = True


@dataclass
class MapContext:
    map_id: str
    hospital_map: HospitalMap
    free_mask: np.ndarray
    distance_m: np.ndarray
    map_sha256: str
    map_yaml_sha256: str
    map_yaml: Optional[Path] = None


@dataclass
class PlanResult:
    planner_success: bool = False
    points: Optional[List[Dict[str, Any]]] = None
    failure_code: str = "NO_PATH"
    failure_detail: str = ""
    planner_backend: str = ""
    backend_version: str = ""
    source: str = ""
    expanded_states: int = 0
    generated_states: int = 0
    samples: Optional[int] = None
    rewires: Optional[int] = None
    first_solution_time_ms: Optional[float] = None
    diagnostics: Dict[str, Any] = None  # type: ignore[assignment]


SMAC_PARAMETER_PROFILES: Dict[str, Dict[str, Any]] = {
    "baseline": {},
    "angle_bins_48": {"angle_quantization_bins": 48},
    "downsample_2": {"downsample_costmap": True, "downsampling_factor": 2},
    "lighter_smoother": {
        "smoother": {"max_iterations": 200, "do_refinement": True, "refinement_num": 1},
    },
    "bounded_search": {
        "max_iterations": 200000,
        "max_on_approach_iterations": 500,
        "max_planning_time": 1.0,
    },
}

# Incremental costmap patches may coalesce only within this local envelope.
# Keeping the bounds explicit prevents a chain of distant windows from
# degenerating into a full-map update while still reducing message overhead
# for fragmented cells in one window.
DELTA_MAX_PATCH_GAP_CELLS = 64
DELTA_MAX_PATCH_EXPANSION = 2.5


def _grid_digest(grid: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(grid, dtype=np.int8).tobytes()).hexdigest()


def delta_patch_rectangles(
    previous_grid: np.ndarray, expected_grid: np.ndarray,
) -> List[Tuple[int, int, int, int]]:
    """Return tight map-coordinate patches for close and open regions.

    A single bounding box over all changed cells can span unrelated windows
    and turn an incremental update into an effectively full-map update.  Label
    each connected changed component independently so distant regions remain
    separate patches.  Four-connectivity matches the cell adjacency used by
    the occupancy grid and keeps diagonal corners as distinct updates.
    """
    previous = np.asarray(previous_grid, dtype=np.int8)
    expected = np.asarray(expected_grid, dtype=np.int8)
    if previous.shape != expected.shape:
        raise ValueError("delta grids must have the same shape")
    changed = previous != expected
    if not np.any(changed):
        return []
    rectangles: List[Tuple[int, int, int, int]] = []
    # Treat closing and opening as separate regions. This prevents an opening
    # component from being merged with a closing component during a state
    # switch, while the component labelling below prevents distant windows
    # within one region from sharing a large bounding box.
    try:
        from scipy import ndimage
    except ImportError:  # pragma: no cover - scipy ships with project deps
        ndimage = None

    def rect_area(rectangle: Tuple[int, int, int, int]) -> int:
        return rectangle[2] * rectangle[3]

    def merge_local_rectangles(
        candidates: Sequence[Tuple[int, int, int, int]],
    ) -> List[Tuple[int, int, int, int]]:
        """Coalesce nearby components without spanning unrelated windows."""
        pending = sorted(candidates, key=lambda item: (item[1], item[0]))
        merged: List[Tuple[int, int, int, int]] = []
        for candidate in pending:
            absorbed = False
            for index, current in enumerate(merged):
                current_x, current_y, current_w, current_h = current
                candidate_x, candidate_y, candidate_w, candidate_h = candidate
                gap_x = max(
                    0,
                    current_x - (candidate_x + candidate_w),
                    candidate_x - (current_x + current_w),
                )
                gap_y = max(
                    0,
                    current_y - (candidate_y + candidate_h),
                    candidate_y - (current_y + current_h),
                )
                union_x = min(current_x, candidate_x)
                union_y = min(current_y, candidate_y)
                union_right = max(current_x + current_w, candidate_x + candidate_w)
                union_bottom = max(current_y + current_h, candidate_y + candidate_h)
                union = (
                    union_x, union_y, union_right - union_x, union_bottom - union_y,
                )
                # A component may be joined to its local neighbour, but a
                # large empty gap or excessive bounding-box expansion keeps
                # distant windows as separate messages.
                if (
                    max(gap_x, gap_y) <= DELTA_MAX_PATCH_GAP_CELLS
                    and rect_area(union)
                    <= DELTA_MAX_PATCH_EXPANSION * (rect_area(current) + rect_area(candidate))
                ):
                    merged[index] = union
                    absorbed = True
                    break
            if not absorbed:
                merged.append(candidate)
        return merged

    for region in (changed & (expected == 100), changed & (expected != 100)):
        if not np.any(region):
            continue
        region_rectangles: List[Tuple[int, int, int, int]] = []
        if ndimage is not None:
            labels, component_count = ndimage.label(
                region, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8),
            )
            for component_slice in ndimage.find_objects(labels)[:int(component_count)]:
                if component_slice is None:
                    continue
                row_slice, column_slice = component_slice
                region_rectangles.append((
                    int(column_slice.start), int(row_slice.start),
                    int(column_slice.stop - column_slice.start),
                    int(row_slice.stop - row_slice.start),
                ))
        else:
            # Keep a dependency-free fallback for minimal test/install
            # environments. It is intentionally used only when scipy is
            # absent; a set-backed flood fill preserves the same four-neighbor
            # component semantics as ``ndimage.label``.
            remaining = {tuple(int(value) for value in item) for item in np.argwhere(region)}
            while remaining:
                seed = remaining.pop()
                stack = [seed]
                min_row = max_row = seed[0]
                min_col = max_col = seed[1]
                while stack:
                    row, column = stack.pop()
                    min_row, max_row = min(min_row, row), max(max_row, row)
                    min_col, max_col = min(min_col, column), max(max_col, column)
                    for neighbour in (
                        (row - 1, column), (row + 1, column),
                        (row, column - 1), (row, column + 1),
                    ):
                        if neighbour in remaining:
                            remaining.remove(neighbour)
                            stack.append(neighbour)
                region_rectangles.append((
                    min_col, min_row, max_col - min_col + 1, max_row - min_row + 1,
                ))
        rectangles.extend(merge_local_rectangles(region_rectangles))
    return sorted(rectangles, key=lambda item: (item[1], item[0], item[3], item[2]))


def apply_delta_rectangles(
    previous_grid: np.ndarray, expected_grid: np.ndarray,
    rectangles: Sequence[Tuple[int, int, int, int]],
) -> np.ndarray:
    """Apply map-coordinate rectangles; used by runtime verification and tests."""
    previous = np.asarray(previous_grid, dtype=np.int8)
    expected = np.asarray(expected_grid, dtype=np.int8)
    if previous.shape != expected.shape:
        raise ValueError("delta grids must have the same shape")
    applied = previous.copy()
    for x, y, width, height in rectangles:
        if x < 0 or y < 0 or width <= 0 or height <= 0:
            raise ValueError("invalid delta rectangle")
        if x + width > expected.shape[1] or y + height > expected.shape[0]:
            raise ValueError("delta rectangle is outside the map")
        applied[y:y + height, x:x + width] = expected[y:y + height, x:x + width]
    return applied


class SmacSession:
    """One static Nav2 planner stack reused within a smoke map."""

    def __init__(
        self, ctx: MapContext, output: Path, *, map_yaml: Optional[Path] = None,
        log_tag: Optional[str] = None, local_mask_updates: bool = False,
        optimization_profile: str = "v6_compatible", smac_parameter_profile: str = "baseline",
        optimization_stage: str = "step3_delta_map",
        enable_mask_reuse_noop: bool = False,
    ):
        from .planner_benchmark.config import load_yaml, stack_parameters
        from .planner_benchmark.runner import BenchmarkStack, ComputePathClient

        import rclpy
        from map_msgs.msg import OccupancyGridUpdate
        from nav2_msgs.srv import ClearEntireCostmap
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from rclpy.context import Context

        # Each stack owns a fresh Context so repeated local L3 windows can be
        # initialized after the previous lifecycle stack was shut down.
        self.rclpy = rclpy
        self.ctx = ctx
        if optimization_profile not in {"v6_compatible", "v7_candidate"}:
            raise ValueError(f"unsupported optimization profile: {optimization_profile}")
        if smac_parameter_profile not in SMAC_PARAMETER_PROFILES:
            raise ValueError(f"unsupported Smac parameter profile: {smac_parameter_profile}")
        if optimization_stage not in {
            "baseline", "step1_skip_simplification", "step2_light_reset", "step3_delta_map",
        }:
            raise ValueError(f"unsupported optimization stage: {optimization_stage}")
        if optimization_profile == "v6_compatible":
            # The rollback profile is a behavioral contract, not merely a
            # label.  Keep the strict baseline Smac parameters even when a
            # caller accidentally carries a candidate parameter selection.
            smac_parameter_profile = "baseline"
            optimization_stage = "baseline"
        self.optimization_profile = optimization_profile
        self.optimization_stage = optimization_stage
        self.smac_parameter_profile = smac_parameter_profile
        self.local_map_update_strategy = (
            "delta" if optimization_stage == "step3_delta_map" else "v6_full"
        )
        self.OccupancyGridUpdate = OccupancyGridUpdate
        self.OccupancyGrid = OccupancyGrid
        self.ClearEntireCostmap = ClearEntireCostmap
        self._map_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.context = Context()
        rclpy.init(context=self.context)
        config_path = _strict_smac_config_path()
        planner_config = load_yaml(config_path)
        # Let Smac emit its own smoothed geometric path.  This keeps steering
        # and curvature tied to planner output rather than post-hoc metadata
        # rewriting in the layered stitcher; hard validation still rejects any
        # collision or curvature violation.
        if isinstance(planner_config.get("GridBased"), dict):
            planner_config["GridBased"]["smooth_path"] = True
            overrides = SMAC_PARAMETER_PROFILES[smac_parameter_profile]
            for key, value in overrides.items():
                if key == "smoother":
                    planner_config["GridBased"].setdefault("smoother", {}).update(value)
                else:
                    planner_config["GridBased"][key] = value
        protocol = {
            "resolution": 0.05,
            "width_cells": ctx.hospital_map.width,
            "height_cells": ctx.hospital_map.height,
            "origin": list(ctx.hospital_map.origin),
            "footprint": FOOTPRINT,
            "variants": {"strict_forward": {"allow_unknown": False, "inflation_radius": 0.55, "cost_scaling_factor": 3.0}},
            # The fixed layered latency mode updates the static layer in-place
            # for each local repair.  The legacy one-shot/query mode leaves
            # this disabled and therefore preserves its original semantics.
            "static_layer_subscribe_to_updates": bool(local_mask_updates),
            "costmap_update_frequency": (
                100.0 if local_mask_updates and optimization_stage == "step3_delta_map"
                else (20.0 if local_mask_updates else 1.0)
            ),
        }
        params = stack_parameters(protocol=protocol, planner_config=planner_config)
        logs = output / "logs"
        logs.mkdir(parents=True, exist_ok=True)
        params_file = logs / f"smac_strict_{ctx.map_id}.yaml"
        params_file.write_text(yaml.safe_dump(params, sort_keys=False), encoding="utf-8")
        self.params_file = params_file
        self.smac_config_hash = hashlib.sha256(params_file.read_bytes()).hexdigest()
        tag = re.sub(r"[^A-Za-z0-9_.-]+", "_", log_tag or ctx.map_id)
        node_tag = re.sub(r"[^A-Za-z0-9_]+", "_", tag)
        self.stack = BenchmarkStack(
            map_yaml=map_yaml or ctx.map_yaml or MAP_PATHS[ctx.map_id], params_file=params_file,
            log_file=logs / f"smac_strict_{tag}.log",
        )
        self.client_type = ComputePathClient
        self.client_node_name = f"planner_benchmark_client_{node_tag}_{os.getpid()}"
        self.client = None
        self.planner_pid = 0
        self.stack_pids: List[int] = []
        self.stack_startup_time_ms = 0.0
        self.stack_shutdown_time_ms = 0.0
        self.session_start_count = 0
        self.session_close_count = 0
        self.session_restart_count = 0
        self.restart_reasons: List[str] = []
        self.current_query_id = ""
        self.supports_local_mask = bool(protocol["static_layer_subscribe_to_updates"])
        self._local_update_publisher = None
        self._local_map_publisher = None
        self._clear_costmap_client = None
        self._local_mask_info: Dict[str, Any] = {}
        self._current_allowed_mask: Optional[np.ndarray] = None
        self._current_grid: Optional[np.ndarray] = None
        self._costmap_state_trusted = False
        self._force_full_next_update = False
        self._action_in_progress = False
        # Optional r1 optimization.  Kept disabled by default so the V0/V7
        # session semantics remain unchanged.
        self.enable_mask_reuse_noop = bool(enable_mask_reuse_noop)
        self._last_update_had_fallback = False

    def start(self) -> None:
        started_ns = time.monotonic_ns()
        self.stack.start(timeout=90.0)
        self.planner_pid, self.stack_pids, process_error = self.stack.pids()
        if process_error:
            self.stack.stop()
            raise RuntimeError(process_error)
        self.client = self.client_type(
            timeout=7.0, node_name=self.client_node_name, context=self.context,
        )
        if self.supports_local_mask:
            self._local_update_publisher = self.client.node.create_publisher(
                self.OccupancyGridUpdate, "/map_updates", 10,
            )
            # StaticLayer handles a complete OccupancyGrid deterministically;
            # publishing it alongside the update message avoids relying on a
            # particular Nav2 minor version's update callback scheduling.
            self._local_map_publisher = self.client.node.create_publisher(
                self.OccupancyGrid, "/map", self._map_qos,
            )
            self._clear_costmap_client = self.client.node.create_client(
                self.ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap",
            )
        self.stack_startup_time_ms = (time.monotonic_ns() - started_ns) / 1.0e6
        self.session_start_count += 1

        if self.supports_local_mask and self.local_map_update_strategy == "delta":
            # The cold-start phase initializes a deterministic all-lethal map.
            # Online requests then only open/close their local regions.
            lethal = np.zeros(
                (self.ctx.hospital_map.height, self.ctx.hospital_map.width), dtype=bool,
            )
            self.update_local_mask(lethal, force_full=True, initialization=True)
            self._local_mask_info = {}
            self.stack_startup_time_ms = (time.monotonic_ns() - started_ns) / 1.0e6
        elif self.supports_local_mask:
            base_mask = np.asarray(self.ctx.hospital_map.occupancy == 0, dtype=bool)
            self._current_allowed_mask, self._current_grid = self._grid_for_mask(base_mask)
            self._costmap_state_trusted = True

    def reset_query_state(self, query_id: str, *, restore_base_map: bool = True) -> Dict[str, Any]:
        """Start one serial query without carrying request diagnostics forward.

        Nav2 creates fresh action state for every ComputePath request.  The
        shared context deliberately retains only the immutable base map and
        planner processes; the next L3 request replaces the complete local
        mask before planning.
        """
        if self.client is None or not self.context.ok():
            raise RuntimeError("Smac map session is not active")
        reset_started_ns = time.monotonic_ns()
        fallback = False
        fallback_reason = ""
        if not restore_base_map and self._action_in_progress:
            fallback = True
            fallback_reason = "previous_action_not_finished"
        elif not restore_base_map and self.supports_local_mask and not self._costmap_state_trusted:
            fallback = True
            fallback_reason = "costmap_state_untrusted"
        if fallback:
            restore_base_map = True
            self._force_full_next_update = True
        self.current_query_id = str(query_id)
        self._local_mask_info = {}
        if restore_base_map and self.supports_local_mask:
            base_mask = np.asarray(self.ctx.hospital_map.occupancy == 0, dtype=bool)
            self.update_local_mask(base_mask, force_full=True)
            self._local_mask_info = {}
        return {
            "query_session_reused": self.session_start_count == 1,
            "session_start_count": self.session_start_count,
            "session_close_count": self.session_close_count,
            "session_restart_count": self.session_restart_count,
            "restart_reason": ";".join(self.restart_reasons),
            "query_session_reset_mode": "v6_full" if restore_base_map else "light",
            "session_reset_fallback": fallback,
            "session_reset_fallback_reason": fallback_reason,
            "query_session_reset_ms": (time.monotonic_ns() - reset_started_ns) / 1.0e6,
        }

    def close(self) -> None:
        started_ns = time.monotonic_ns()
        if self.client is not None:
            if self._local_update_publisher is not None:
                self.client.node.destroy_publisher(self._local_update_publisher)
                self._local_update_publisher = None
            if self._local_map_publisher is not None:
                self.client.node.destroy_publisher(self._local_map_publisher)
                self._local_map_publisher = None
            self._clear_costmap_client = None
            self.client.close()
            self.client = None
        self.stack.stop()
        time.sleep(0.5)
        if self.context.ok():
            self.context.shutdown()
        self.stack_shutdown_time_ms = (time.monotonic_ns() - started_ns) / 1.0e6
        self.session_close_count += 1
        self.current_query_id = ""
        self._costmap_state_trusted = False
        self._current_allowed_mask = None
        self._current_grid = None

    def _grid_for_mask(self, allowed_mask: Any) -> Tuple[np.ndarray, np.ndarray]:
        mask = np.asarray(allowed_mask, dtype=bool)
        if mask.shape != (self.ctx.hospital_map.height, self.ctx.hospital_map.width):
            raise ValueError("local mask shape does not match the map")
        values = np.where(mask, np.asarray(self.ctx.hospital_map.occupancy, dtype=np.int16), 100)
        values = np.where(values < 0, -1, np.clip(values, 0, 100)).astype(np.int8)
        return mask, np.flipud(values)

    def _clear_global_costmap(self) -> float:
        clear_started_ns = time.monotonic_ns()
        if self._clear_costmap_client is not None and self._clear_costmap_client.wait_for_service(timeout_sec=0.5):
            clear_future = self._clear_costmap_client.call_async(self.ClearEntireCostmap.Request())
            deadline = time.monotonic() + 1.0
            while not clear_future.done() and time.monotonic() < deadline:
                self.client.executor.spin_once(timeout_sec=0.01)
            if not clear_future.done():
                raise RuntimeError("global costmap clear service timed out")
        return (time.monotonic_ns() - clear_started_ns) / 1.0e6

    def _publish_full_grid(self, values: np.ndarray, *, clear_costmap: bool = True) -> float:
        message = self.OccupancyGridUpdate()
        message.header.frame_id = "map"
        message.x = 0
        message.y = 0
        message.width = int(self.ctx.hospital_map.width)
        message.height = int(self.ctx.hospital_map.height)
        serialized_data = array("b", np.ascontiguousarray(values, dtype=np.int8).tobytes())
        message.data = serialized_data
        full_map = self.OccupancyGrid()
        full_map.header.frame_id = "map"
        full_map.header.stamp = self.client.node.get_clock().now().to_msg()
        full_map.info.resolution = float(self.ctx.hospital_map.resolution)
        full_map.info.width = int(self.ctx.hospital_map.width)
        full_map.info.height = int(self.ctx.hospital_map.height)
        full_map.info.origin.position.x = float(self.ctx.hospital_map.origin[0])
        full_map.info.origin.position.y = float(self.ctx.hospital_map.origin[1])
        full_map.info.origin.orientation.w = 1.0
        full_map.data = serialized_data
        clear_ms = self._clear_global_costmap() if clear_costmap else 0.0
        self._local_map_publisher.publish(full_map)
        self._local_update_publisher.publish(message)
        settle_cycles = int(getattr(self, "full_grid_settle_cycles", 3 if self.local_map_update_strategy == "v6_full" else 2))
        for _ in range(max(0, settle_cycles)):
            self.client.executor.spin_once(timeout_sec=0.01)
        return clear_ms

    def _publish_delta_updates(
        self, expected: np.ndarray, rectangles: Sequence[Tuple[int, int, int, int]],
    ) -> np.ndarray:
        if self._current_grid is None:
            raise RuntimeError("delta update has no initialized grid")
        applied = apply_delta_rectangles(self._current_grid, expected, rectangles)
        for x, y, width, height in rectangles:
            patch = np.ascontiguousarray(expected[y:y + height, x:x + width], dtype=np.int8)
            message = self.OccupancyGridUpdate()
            message.header.frame_id = "map"
            message.x = int(x)
            message.y = int(y)
            message.width = int(width)
            message.height = int(height)
            message.data = array("b", patch.tobytes())
            self._local_update_publisher.publish(message)
            # StaticLayer and the master costmap run in the planner process.
            # Let each close/open region cross three 100 Hz costmap periods before
            # publishing the next region, otherwise only the last bounds can be
            # visible to Smac on some Nav2 Humble builds.
            for _ in range(6):
                self.client.executor.spin_once(timeout_sec=0.005)
        return applied

    def update_local_mask(
        self, allowed_mask: Any, *, window_start_index: int = -1,
        window_end_index: int = -1, window_path_length_m: Optional[float] = None,
        force_full: bool = False, initialization: bool = False,
        fallback_reason: str = "",
    ) -> Dict[str, Any]:
        """Publish one full-size static-layer update without restarting Nav2.

        ``OccupancyGridUpdate`` uses map coordinates whose row zero is the
        bottom of the map.  HospitalMap stores image rows top-to-bottom, so
        the mask is flipped before publishing.  Every cell outside the local
        window is lethal, making it impossible for Smac to route around the
        requested window while retaining the original static map geometry.
        """
        if not self.supports_local_mask or self._local_update_publisher is None or self._local_map_publisher is None or self.client is None:
            raise RuntimeError("Smac session does not support local costmap updates")
        mask, values = self._grid_for_mask(allowed_mask)
        started_ns = time.monotonic_ns()
        expected_hash = _grid_digest(values)
        previous_hash = _grid_digest(self._current_grid) if self._current_grid is not None else ""
        can_reuse = bool(
            getattr(self, "enable_mask_reuse_noop", False) and not force_full and not initialization
            and not getattr(self, "_force_full_next_update", False) and getattr(self, "_costmap_state_trusted", False)
            and self._current_grid is not None and not getattr(self, "_last_update_had_fallback", False)
            and previous_hash == expected_hash
        )
        if can_reuse:
            elapsed_ms = (time.monotonic_ns() - started_ns) / 1.0e6
            allowed_cells = int(np.count_nonzero(mask))
            self._local_mask_info = {
                "local_mask_hash": expected_hash,
                "local_map_width_cells": int(mask.shape[1]),
                "local_map_height_cells": int(mask.shape[0]),
                "local_window_allowed_cells": allowed_cells,
                "local_window_start_index": int(window_start_index),
                "local_window_end_index": int(window_end_index),
                "local_window_path_length_m": window_path_length_m,
                "local_map_update_ms": elapsed_ms,
                "local_costmap_clear_ms": 0.0,
                "local_map_update_mode": "reuse_noop",
                "local_map_update_messages": 0,
                "local_map_update_cells": 0,
                "local_map_update_bytes": 0,
                "local_map_update_fallback": False,
                "local_map_update_fallback_reason": "",
                "local_map_update_skipped": True,
                "previous_mask_hash": previous_hash,
                "expected_mask_hash": expected_hash,
                "applied_mask_hash": previous_hash,
            }
            return dict(self._local_mask_info)
        mode = "v6_full"
        messages = 1
        cells = int(values.size)
        update_bytes = int(values.nbytes)
        clear_ms = 0.0
        fallback = False
        actual_fallback_reason = fallback_reason
        use_delta = (
            self.local_map_update_strategy == "delta" and not force_full
            and not self._force_full_next_update and self._costmap_state_trusted
            and self._current_grid is not None
        )
        if use_delta:
            mode = "delta"
            try:
                rectangles = delta_patch_rectangles(self._current_grid, values)
                messages = len(rectangles)
                cells = sum(width * height for _x, _y, width, height in rectangles)
                update_bytes = cells
                applied = self._publish_delta_updates(values, rectangles)
                applied_hash = _grid_digest(applied)
                if applied_hash != expected_hash:
                    raise RuntimeError("delta applied-grid hash mismatch")
            except (OSError, RuntimeError, ValueError) as exc:
                fallback = True
                actual_fallback_reason = actual_fallback_reason or str(exc)
                mode = "full_fallback"
                clear_ms = self._publish_full_grid(values)
                applied = values.copy()
                messages += 2
                cells += int(values.size)
                update_bytes += int(values.nbytes)
        else:
            mode = "delta_initial_full" if initialization else (
                "full_fallback" if self.local_map_update_strategy == "delta" else "v6_full"
            )
            fallback = bool(self.local_map_update_strategy == "delta" and not initialization)
            if fallback and not actual_fallback_reason:
                actual_fallback_reason = "forced_full_update"
            clear_ms = self._publish_full_grid(values)
            applied = values.copy()
            messages = 2
            cells = int(values.size) * 2
            update_bytes = int(values.nbytes) * 2
        applied_hash = _grid_digest(applied)
        self._current_allowed_mask = mask.copy()
        self._current_grid = applied
        self._costmap_state_trusted = applied_hash == expected_hash
        self._force_full_next_update = False
        self._last_update_had_fallback = bool(fallback)
        elapsed_ms = (time.monotonic_ns() - started_ns) / 1.0e6
        self._local_mask_info = {
            "local_mask_hash": expected_hash,
            "local_map_width_cells": int(mask.shape[1]),
            "local_map_height_cells": int(mask.shape[0]),
            "local_window_allowed_cells": int(np.count_nonzero(mask)),
            "local_window_start_index": int(window_start_index),
            "local_window_end_index": int(window_end_index),
            "local_window_path_length_m": window_path_length_m,
            "local_map_update_ms": elapsed_ms,
            "local_costmap_clear_ms": clear_ms,
            "local_map_update_mode": mode,
            "local_map_update_messages": messages,
            "local_map_update_cells": cells,
            "local_map_update_bytes": update_bytes,
            "local_map_update_fallback": fallback,
            "local_map_update_fallback_reason": actual_fallback_reason,
            "local_map_update_skipped": False,
            "previous_mask_hash": previous_hash,
            "expected_mask_hash": expected_hash,
            "applied_mask_hash": applied_hash,
        }
        return dict(self._local_mask_info)

    def _path_within_allowed_mask(
        self, points: Sequence[Mapping[str, Any]], allowed_mask: np.ndarray,
    ) -> bool:
        if not points:
            return False
        spacing = max(0.01, float(self.ctx.hospital_map.resolution) * 0.5)
        for first, second in zip(points, points[1:]):
            dx = float(second["x"]) - float(first["x"])
            dy = float(second["y"]) - float(first["y"])
            steps = max(1, int(math.ceil(math.hypot(dx, dy) / spacing)))
            for step in range(steps + 1):
                fraction = step / steps
                cell = self.ctx.hospital_map.world_to_cell(
                    float(first["x"]) + fraction * dx,
                    float(first["y"]) + fraction * dy,
                )
                if cell is None or not bool(allowed_mask[cell]):
                    return False
        if len(points) == 1:
            cell = self.ctx.hospital_map.world_to_cell(float(points[0]["x"]), float(points[0]["y"]))
            return cell is not None and bool(allowed_mask[cell])
        return True

    def plan(
        self, query: Query, spec: BackendSpec, *, source: str = "hybrid_astar",
        allowed_mask: Any = None, window_start_index: int = -1,
        window_end_index: int = -1, window_path_length_m: Optional[float] = None,
        force_full_update: bool = False,
    ) -> PlanResult:
        if self.client is None:
            return unavailable_plan(spec, source=source)
        local_update_ms = 0.0
        local_mask_info: Dict[str, Any] = {}
        if allowed_mask is not None:
            local_mask_info = self.update_local_mask(
                allowed_mask, window_start_index=window_start_index,
                window_end_index=window_end_index,
                window_path_length_m=window_path_length_m,
                force_full=bool(force_full_update),
            )
            local_update_ms = float(local_mask_info.get("local_map_update_ms") or 0.0)
        def call_action() -> Tuple[str, str, float, Any, List[Dict[str, Any]], Any, Optional[float]]:
            self._action_in_progress = True
            try:
                status, result_code, wall_ms, measurement, raw_points, action_result = self.client.plan(
                    query, planner_pid=self.planner_pid, stack_pids=self.stack_pids, sample_interval_ms=5.0,
                )
            finally:
                self._action_in_progress = False
            if result_code == "CLIENT_TIMEOUT":
                self._costmap_state_trusted = False
                self._last_update_had_fallback = True
            points = _annotate_smac_points(raw_points or [], spec, source)
            planning_ms = None
            duration = getattr(action_result, "planning_time", None)
            if duration is not None:
                planning_ms = float(getattr(duration, "sec", 0)) * 1000.0 + float(getattr(duration, "nanosec", 0)) / 1e6
            return status, result_code, wall_ms, measurement, points, action_result, planning_ms

        action_results = [call_action()]
        status, result_code, wall_ms, measurement, points, action_result, planning_ms = action_results[-1]
        path_left_mask = bool(
            allowed_mask is not None and result_code == "SUCCEEDED" and points
            and not self._path_within_allowed_mask(points, np.asarray(allowed_mask, dtype=bool))
        )
        if path_left_mask and self.local_map_update_strategy == "delta":
            first_update = dict(local_mask_info)
            fallback_update = self.update_local_mask(
                allowed_mask, window_start_index=window_start_index,
                window_end_index=window_end_index,
                window_path_length_m=window_path_length_m,
                force_full=True, fallback_reason="returned_path_left_allowed_mask",
            )
            local_update_ms += float(fallback_update.get("local_map_update_ms") or 0.0)
            local_mask_info = {
                **fallback_update,
                "local_map_update_ms": local_update_ms,
                "local_map_update_messages": int(first_update.get("local_map_update_messages") or 0)
                + int(fallback_update.get("local_map_update_messages") or 0),
                "local_map_update_cells": int(first_update.get("local_map_update_cells") or 0)
                + int(fallback_update.get("local_map_update_cells") or 0),
                "local_map_update_bytes": int(first_update.get("local_map_update_bytes") or 0)
                + int(fallback_update.get("local_map_update_bytes") or 0),
            }
            action_results.append(call_action())
            status, result_code, wall_ms, measurement, points, action_result, planning_ms = action_results[-1]
            path_left_mask = bool(
                result_code == "SUCCEEDED" and points
                and not self._path_within_allowed_mask(points, np.asarray(allowed_mask, dtype=bool))
            )
        if path_left_mask:
            result_code = "L3_PATH_OUTSIDE_LOCAL_MASK"
            points = []
        total_wall_ms = sum(float(item[2]) for item in action_results)
        total_planning_ms = sum(float(item[6] or 0.0) for item in action_results)
        backend_attempts = [
            {
                "backend_action_index": index,
                "action_status": item[0],
                "action_result_code": item[1],
                "wall_time_ms": item[2],
                "planning_time_ms": item[6],
                "path_mask_valid": not (
                    index == 0 and len(action_results) > 1
                ) and not path_left_mask,
            }
            for index, item in enumerate(action_results)
        ]
        diagnostics = {
            "action_status": status, "action_result_code": result_code,
            "backend_called": True,
            "planning_time_ms": total_planning_ms, "planner_cpu_total_ms": getattr(measurement, "planner_cpu_total_ms", None),
            "planner_rss_peak_bytes": getattr(measurement, "planner_rss_peak_bytes", None),
            "planner_pss_peak_bytes": getattr(measurement, "planner_pss_peak_bytes", None),
            "stack_rss_peak_bytes": getattr(measurement, "stack_rss_peak_bytes", None),
            "stack_pss_peak_bytes": getattr(measurement, "stack_pss_peak_bytes", None),
            "wall_time_ms": total_wall_ms,
            "l3_action_wall_ms": total_wall_ms,
            "l3_planning_time_ms": total_planning_ms,
            "l3_process_overhead_ms": max(0.0, total_wall_ms - total_planning_ms),
            "backend_call_count": len(action_results),
            "backend_action_attempts": backend_attempts,
            "returned_path_within_mask": not path_left_mask,
            **local_mask_info,
        }
        success = result_code == "SUCCEEDED" and bool(points)
        return PlanResult(
            planner_success=success, points=points or None,
            failure_code="" if success else result_code,
            failure_detail="" if success else f"Nav2 action status={status}",
            planner_backend=spec.backend, backend_version=spec.version,
            source=source, diagnostics=diagnostics,
        )


def prepare_local_smac_context(
    ctx: MapContext,
    query: Query,
    allowed_mask: np.ndarray,
    output: Path,
    *,
    map_tag: Optional[str] = None,
) -> Tuple[MapContext, Path, Dict[str, float]]:
    """Build one masked static map for a query and reuse it for all actions.

    The caller owns the returned ``SmacSession`` lifecycle.  Keeping map
    construction here makes the legacy one-shot API and the fixed layered
    pipeline share exactly the same occupancy semantics.
    """
    started_ns = time.monotonic_ns()
    local_root = output / "local_maps" / ctx.map_id / (map_tag or query.query_id)
    local_root.mkdir(parents=True, exist_ok=True)
    original = np.asarray(Image.open(ctx.hospital_map.image_path).convert("L"))
    masked = np.where(np.asarray(allowed_mask, dtype=bool), original, 0).astype(np.uint8)
    image_path = local_root / "map.pgm"
    Image.fromarray(masked).save(image_path)
    source_config = yaml.safe_load(ctx.hospital_map.yaml_path.read_text(encoding="utf-8")) or {}
    source_config["image"] = image_path.name
    yaml_path = local_root / "map.yaml"
    yaml_path.write_text(yaml.safe_dump(source_config, sort_keys=False), encoding="utf-8")
    local_map = HospitalMap.load(yaml_path)
    local_ctx = MapContext(
        f"{ctx.map_id}_{query.query_id}", local_map, np.asarray(allowed_mask, dtype=bool),
        ctx.distance_m, sha256_file(image_path), sha256_file(yaml_path), yaml_path,
    )
    return local_ctx, yaml_path, {
        "l3_local_map_build_ms": (time.monotonic_ns() - started_ns) / 1.0e6,
    }


def _wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _delta(a: float, b: float) -> float:
    return _wrap(float(a) - float(b))


def _annotate_smac_points(points: Sequence[Mapping[str, Any]], spec: BackendSpec, source: str) -> List[Dict[str, Any]]:
    annotated: List[Dict[str, Any]] = []
    for index, point in enumerate(points):
        if index + 1 < len(points):
            next_point = points[index + 1]
            distance = math.hypot(float(next_point["x"]) - float(point["x"]), float(next_point["y"]) - float(point["y"]))
            yaw_change = _delta(float(next_point["yaw"]), float(point["yaw"]))
            steering = math.atan(0.50 * yaw_change / distance) if distance > 1.0e-9 else 0.0
            projection = (float(next_point["x"]) - float(point["x"])) * math.cos(float(point["yaw"])) + (float(next_point["y"]) - float(point["y"])) * math.sin(float(point["yaw"]))
            direction = "forward" if projection >= -1.0e-6 else "reverse"
        else:
            steering = float(annotated[-1]["steering"]) if annotated else 0.0
            direction = str(annotated[-1]["motion_direction"]) if annotated else "forward"
        annotated.append({
            "x": float(point["x"]), "y": float(point["y"]), "yaw": _wrap(float(point["yaw"])),
            "velocity": 0.0, "steering": steering, "source": source,
            "motion_direction": direction, "planner_backend": spec.backend,
            "backend_version": spec.version,
        })
    steering_step_limit = math.radians(15.0)
    for index in range(1, len(annotated)):
        previous = float(annotated[index - 1]["steering"])
        delta = float(annotated[index]["steering"]) - previous
        annotated[index]["steering"] = previous + max(
            -steering_step_limit, min(steering_step_limit, delta)
        )
    return annotated


def _queries() -> Dict[str, Query]:
    payload = yaml.safe_load(SOURCE_QUERIES.read_text(encoding="utf-8")) or {}
    return {
        str(item["query_id"]): Query(
            query_id=str(item["query_id"]),
            start=[float(v) for v in item["start"]],
            goal=[float(v) for v in item["goal"]],
            category=str(item.get("category", "unspecified")),
            seed=int(item.get("seed", payload.get("seed", 20260821))),
            validation_status=str(item.get("validation_status", "UNVALIDATED")),
        )
        for item in payload.get("queries", [])
    }


def backend_availability() -> Dict[str, BackendSpec]:
    """Detect callable mature backends, distinguishing installed libraries.

    ROS Smac and OMPL shared libraries are useful evidence, but they are not a
    Python-callable planner endpoint.  Treating them as available here would
    make a smoke result irreproducible and would violate the backend contract.
    """
    ompl_adapter = importlib.util.find_spec("arena_evaluation._ompl_planner_backend") is not None
    ompl_version = "unknown"
    if ompl_adapter:
        try:
            from . import _ompl_planner_backend
            ompl_version = str(_ompl_planner_backend.version())
        except (ImportError, RuntimeError):
            ompl_adapter = False
    smac_libs = [
        Path("/opt/ros/humble/lib/libnav2_smac_planner.so"),
        Path("/opt/ros/humble/lib/libnav2_smac_planner_lattice.so"),
    ]
    smac_installed = any(path.exists() for path in smac_libs)
    smac_callable = smac_installed and shutil.which("ros2") is not None and importlib.util.find_spec("rclpy") is not None
    smac_version = "ROS humble"
    package_xml = Path("/opt/ros/humble/share/nav2_smac_planner/package.xml")
    if package_xml.exists():
        match = re.search(r"<version>([^<]+)</version>", package_xml.read_text(encoding="utf-8"))
        if match:
            smac_version = match.group(1)
    return {
        "grid_astar": BackendSpec(
            "grid_astar", "arena_evaluation.topology.astar_grid", TOPOLOGY_ALGORITHM_VERSION, True,
            "in-repository deterministic 8-neighbor Euclidean A*", mature=True,
        ),
        "geometric_rrt_star": BackendSpec(
            "geometric_rrt_star", "OMPL geometric::RRTstar", ompl_version, ompl_adapter,
            "compiled project adapter available" if ompl_adapter else "compiled OMPL adapter missing; run colcon build", mature=True,
        ),
        "hybrid_astar": BackendSpec(
            "hybrid_astar", "Nav2 SmacPlannerHybrid DUBIN", smac_version, smac_callable,
            "static Nav2 planner stack available" if smac_callable else ("Smac plugin installed but ROS Python environment is not sourced" if smac_installed else "Nav2 Smac plugin missing"),
            mature=True,
        ),
        "kinodynamic_rrt": BackendSpec(
            "kinodynamic_rrt", "OMPL control::SST", ompl_version, ompl_adapter,
            "compiled bicycle-control SST adapter available" if ompl_adapter else "compiled OMPL adapter missing; run colcon build",
            mature=True,
        ),
    }


def _context(map_id: str) -> MapContext:
    hospital_map = HospitalMap.load(MAP_PATHS[map_id])
    if not math.isclose(hospital_map.resolution, 0.05, abs_tol=1e-12):
        raise ValueError(f"{map_id}: resolution must be exactly 0.05 m/cell")
    # Build the footprint-inflated free mask once per map.  No planner is
    # allowed to replace this with a point-robot or reduced-footprint mask.
    from .topology import preprocess_static_map

    _, free_mask, distance_m, _ = preprocess_static_map(
        hospital_map, FOOTPRINT, padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
    )
    return MapContext(
        map_id, hospital_map, free_mask, distance_m,
        sha256_file(hospital_map.image_path), sha256_file(hospital_map.yaml_path), hospital_map.yaml_path,
    )


def _path_hash(points: Sequence[Mapping[str, Any]]) -> str:
    normalized = [{key: value for key, value in point.items() if key != "path_hash"} for point in points]
    payload = json.dumps(normalized, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _enrich_path(points: Optional[List[Dict[str, Any]]], source_commit: Optional[str]) -> str:
    if not points:
        return ""
    for point in points:
        point["source_commit"] = source_commit or "unknown"
    digest = _path_hash(points)
    for point in points:
        point["path_hash"] = digest
    return digest


def _signed_curvature(a: Mapping[str, Any], b: Mapping[str, Any], c: Mapping[str, Any]) -> float:
    ab = math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
    bc = math.hypot(float(c["x"]) - float(b["x"]), float(c["y"]) - float(b["y"]))
    ac = math.hypot(float(c["x"]) - float(a["x"]), float(c["y"]) - float(a["y"]))
    if min(ab, bc, ac) <= 1.0e-12:
        return 0.0
    cross = (
        (float(b["x"]) - float(a["x"])) * (float(c["y"]) - float(a["y"]))
        - (float(b["y"]) - float(a["y"])) * (float(c["x"]) - float(a["x"]))
    )
    return 2.0 * cross / (ab * bc * ac)


def _annotate_geometric_metadata(points: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Derive truthful non-search metadata for a two-dimensional path."""
    curvatures = [0.0] * len(points)
    for index in range(1, len(points) - 1):
        curvatures[index] = _signed_curvature(points[index - 1], points[index], points[index + 1])
    if len(points) > 1:
        curvatures[0] = curvatures[1]
        curvatures[-1] = curvatures[-2]
    for index, point in enumerate(points):
        point["velocity"] = 0.0
        point["steering"] = math.atan(WHEELBASE_M * curvatures[index])
        if index + 1 < len(points):
            following = points[index + 1]
            projection = (
                (float(following["x"]) - float(point["x"])) * math.cos(float(point["yaw"]))
                + (float(following["y"]) - float(point["y"])) * math.sin(float(point["yaw"]))
            )
            point["motion_direction"] = "forward" if projection >= -1.0e-6 else "reverse"
        elif index:
            point["motion_direction"] = points[index - 1]["motion_direction"]
        else:
            point["motion_direction"] = "forward"
    return points


def _points_from_cells(ctx: MapContext, cells: Sequence[Tuple[int, int]], query: Query, backend: str, source: str) -> List[Dict[str, Any]]:
    points: List[Dict[str, Any]] = []
    for index, cell in enumerate(cells):
        x, y = ctx.hospital_map.cell_to_world(cell)
        if index == 0:
            yaw = query.start[2]
        elif index == len(cells) - 1:
            yaw = query.goal[2]
        else:
            x0, y0 = ctx.hospital_map.cell_to_world(cells[index - 1])
            x1, y1 = ctx.hospital_map.cell_to_world(cells[index + 1])
            yaw = math.atan2(y1 - y0, x1 - x0)
        points.append({
            "x": float(x), "y": float(y), "yaw": _wrap(yaw), "source": source,
            "motion_direction": "forward", "steering": 0.0, "velocity": 0.0,
            "planner_backend": backend, "backend_version": TOPOLOGY_ALGORITHM_VERSION,
        })
    return _annotate_geometric_metadata(points)


def plan_grid_astar(ctx: MapContext, query: Query, timeout_s: float, *, allowed_mask: Optional[np.ndarray] = None, source: str = "grid") -> PlanResult:
    started = time.monotonic()
    start = ctx.hospital_map.world_to_cell(*query.start[:2])
    goal = ctx.hospital_map.world_to_cell(*query.goal[:2])
    if start is None or goal is None or not ctx.free_mask[start] or not ctx.free_mask[goal]:
        return PlanResult(
            failure_code="INVALID_ENDPOINT", failure_detail="endpoint outside footprint-inflated free space",
            planner_backend="arena_evaluation.topology.astar_grid", backend_version=TOPOLOGY_ALGORITHM_VERSION,
            source=source, diagnostics={"backend_called": False},
        )
    result = astar_grid(ctx.free_mask, start, goal, allowed_mask=allowed_mask, resolution=0.05, return_stats=True, timeout_s=timeout_s)
    if result.path is None:
        return PlanResult(
            failure_code=result.failure_code or "NO_PATH", failure_detail=result.failure_code or "no path",
            planner_backend="arena_evaluation.topology.astar_grid", backend_version=TOPOLOGY_ALGORITHM_VERSION,
            source=source, expanded_states=result.expanded_nodes, generated_states=result.generated_nodes,
            diagnostics={"backend_called": True, "planning_time_ms": (time.monotonic() - started) * 1000.0},
        )
    return PlanResult(
        True, _points_from_cells(ctx, result.path, query, "arena_evaluation.topology.astar_grid", source),
        "", "", "arena_evaluation.topology.astar_grid", TOPOLOGY_ALGORITHM_VERSION, source,
        result.expanded_nodes, result.generated_nodes,
        diagnostics={"backend_called": True, "planning_time_ms": (time.monotonic() - started) * 1000.0},
    )


def unavailable_plan(spec: BackendSpec, *, source: str = "") -> PlanResult:
    return PlanResult(
        planner_success=False, points=None, failure_code="BACKEND_UNAVAILABLE",
        failure_detail=spec.reason, planner_backend=spec.backend,
        backend_version=spec.version, source=source,
        diagnostics={"mature_backend": spec.mature, "available": spec.available, "backend_called": False},
    )


def _write_mask(path: Path, mask: np.ndarray) -> None:
    height, width = mask.shape
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + np.where(mask, 255, 0).astype(np.uint8).tobytes())


def plan_ompl(
    ctx: MapContext,
    query: Query,
    algorithm: str,
    spec: BackendSpec,
    timeout_s: float,
    *,
    source: str,
    allowed_mask: Optional[np.ndarray] = None,
) -> PlanResult:
    """Invoke one compiled OMPL request in a fresh process."""
    if not spec.available:
        return unavailable_plan(spec, source=source)
    backend_algorithm = "rrt_star" if algorithm == "geometric_rrt_star" else "sst"
    with tempfile.TemporaryDirectory(prefix="pln02_ompl_") as temporary:
        temp = Path(temporary)
        path_file = temp / "path.yaml"
        summary_file = temp / "summary.yaml"
        command = [
            sys.executable, "-m", "arena_evaluation.ompl_backend_cli",
            "--algorithm", backend_algorithm,
            "--map-yaml", str(ctx.map_yaml or MAP_PATHS[ctx.map_id]),
            "--start", *(str(value) for value in query.start),
            "--goal", *(str(value) for value in query.goal),
            "--seed", str(query.seed), "--timeout", str(timeout_s),
            "--path-output", str(path_file), "--summary-output", str(summary_file),
        ]
        if allowed_mask is not None:
            mask_path = temp / "allowed.pgm"
            _write_mask(mask_path, allowed_mask)
            command.extend(["--allowed-mask", str(mask_path)])
        started = time.monotonic()
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        snapshots = []
        timed_out = False
        while process.poll() is None:
            snapshot = read_snapshot(process.pid)
            if snapshot is not None:
                snapshots.append(snapshot)
            if time.monotonic() - started > timeout_s + 2.0:
                timed_out = True
                process.kill()
                break
            time.sleep(0.005)
        stdout, stderr = process.communicate()
        if timed_out:
            return PlanResult(
                failure_code="BACKEND_TIMEOUT", failure_detail="OMPL adapter exceeded timeout guard",
                planner_backend=spec.backend, backend_version=spec.version, source=source,
                diagnostics={"backend_called": True, "command": command, "stderr": stderr[-1000:]},
            )
        if process.returncode != 0 or not summary_file.exists():
            return PlanResult(
                failure_code="BACKEND_EXCEPTION",
                failure_detail=(stderr or stdout or "OMPL adapter produced no summary")[-2000:],
                planner_backend=spec.backend, backend_version=spec.version, source=source,
                diagnostics={"backend_called": True, "returncode": process.returncode, "command": command},
            )
        summary = yaml.safe_load(summary_file.read_text(encoding="utf-8")) or {}
        raw_points = yaml.safe_load(path_file.read_text(encoding="utf-8")) if path_file.exists() else []
        points = []
        for point in raw_points or []:
            points.append({
                "x": float(point["x"]), "y": float(point["y"]), "yaw": _wrap(float(point["yaw"])),
                "velocity": float(point.get("velocity", 0.0)), "steering": float(point.get("steering", 0.0)),
                "source": source, "motion_direction": "forward",
                "planner_backend": str(summary.get("planner_backend", spec.backend)),
                "backend_version": str(summary.get("backend_version", spec.version)),
            })
        if algorithm == "geometric_rrt_star":
            _annotate_geometric_metadata(points)
        rss_values = [item.rss_bytes for item in snapshots if item.rss_bytes is not None]
        pss_values = [item.pss_bytes for item in snapshots if item.pss_bytes is not None]
        cpu_total_ms = None
        if snapshots:
            first, last = snapshots[0], snapshots[-1]
            cpu_total_ms = max(0.0, (last.cpu_user_ms or 0.0) + (last.cpu_system_ms or 0.0) - (first.cpu_user_ms or 0.0) - (first.cpu_system_ms or 0.0))
        backend_note = str(summary.get("failure_detail", "") or "")
        diagnostics = {
            "backend_called": True,
            "planning_time_ms": summary.get("planning_time_ms"),
            "iterations": summary.get("iterations"),
            "generated_states": summary.get("generated_states"),
            "planner_rss_peak_bytes": max(rss_values) if rss_values else None,
            "planner_pss_peak_bytes": max(pss_values) if pss_values else None,
            "planner_cpu_total_ms": cpu_total_ms,
            "monitor_sample_count": len(snapshots),
            "adapter_stderr": stderr[-1000:],
            "backend_note": backend_note,
        }
        return PlanResult(
            planner_success=bool(summary.get("planner_success", False)),
            points=points or None,
            failure_code=str(summary.get("failure_code", "") or ""),
            failure_detail=backend_note if summary.get("failure_code") else "",
            planner_backend=str(summary.get("planner_backend", spec.backend)),
            backend_version=str(summary.get("backend_version", spec.version)),
            source=source,
            generated_states=int(summary.get("generated_states") or 0),
            samples=(int(summary["samples"]) if summary.get("samples") is not None else None),
            rewires=(int(summary["rewires"]) if summary.get("rewires") is not None else None),
            first_solution_time_ms=(float(summary["first_solution_time_ms"]) if summary.get("first_solution_time_ms") is not None else None),
            diagnostics=diagnostics,
        )


def plan_local_smac(
    ctx: MapContext,
    query: Query,
    spec: BackendSpec,
    allowed_mask: np.ndarray,
    output: Path,
) -> PlanResult:
    """Run Smac against a persisted map whose outside-window cells are occupied."""
    if not spec.available:
        return unavailable_plan(spec, source="l3_hybrid_smac")
    local_root = output / "local_maps" / ctx.map_id / query.query_id
    local_root.mkdir(parents=True, exist_ok=True)
    original = np.asarray(Image.open(ctx.hospital_map.image_path).convert("L"))
    masked = np.where(allowed_mask, original, 0).astype(np.uint8)
    image_path = local_root / "map.pgm"
    Image.fromarray(masked).save(image_path)
    source_config = yaml.safe_load(ctx.hospital_map.yaml_path.read_text(encoding="utf-8")) or {}
    source_config["image"] = image_path.name
    yaml_path = local_root / "map.yaml"
    yaml_path.write_text(yaml.safe_dump(source_config, sort_keys=False), encoding="utf-8")
    local_map = HospitalMap.load(yaml_path)
    local_ctx = MapContext(
        f"{ctx.map_id}_{query.query_id}", local_map, allowed_mask,
        ctx.distance_m, sha256_file(image_path), sha256_file(yaml_path), yaml_path,
    )
    session: Optional[SmacSession] = None
    try:
        session = SmacSession(local_ctx, output, map_yaml=yaml_path, log_tag=f"local_{ctx.map_id}_{query.query_id}")
        session.start()
        return session.plan(query, spec, source="l3_hybrid_smac")
    except (RuntimeError, OSError) as exc:
        return PlanResult(
            failure_code="LOCAL_HYBRID_BACKEND_START_FAILED", failure_detail=str(exc),
            planner_backend=spec.backend, backend_version=spec.version,
            source="l3_hybrid_smac", diagnostics={"backend_called": False},
        )
    finally:
        if session is not None:
            session.close()


def plan_single(
    ctx: MapContext, query: Query, algorithm: str, specs: Mapping[str, BackendSpec], timeout_s: float,
    smac_session: Optional[SmacSession] = None,
) -> PlanResult:
    if algorithm == "grid_astar":
        return plan_grid_astar(ctx, query, timeout_s)
    if algorithm in {"geometric_rrt_star", "kinodynamic_rrt"}:
        return plan_ompl(ctx, query, algorithm, specs[algorithm], timeout_s, source=algorithm)
    if algorithm == "hybrid_astar" and smac_session is not None:
        return smac_session.plan(query, specs[algorithm])
    return unavailable_plan(specs[algorithm], source="")


def _curvature(a: Mapping[str, Any], b: Mapping[str, Any], c: Mapping[str, Any]) -> float:
    return abs(_signed_curvature(a, b, c))


def _collision_check_poses(points: Sequence[Mapping[str, Any]]) -> Iterable[Tuple[float, float, float]]:
    """Sample every translational and rotational segment for footprint checks."""
    if not points:
        return
    yield float(points[0]["x"]), float(points[0]["y"]), float(points[0]["yaw"])
    for first, second in zip(points, points[1:]):
        dx = float(second["x"]) - float(first["x"])
        dy = float(second["y"]) - float(first["y"])
        dyaw = _delta(float(second["yaw"]), float(first["yaw"]))
        steps = max(
            1,
            int(math.ceil(math.hypot(dx, dy) / COLLISION_SAMPLE_SPACING_M)),
            int(math.ceil(abs(dyaw) / COLLISION_YAW_SAMPLE_STEP_RAD)),
        )
        for step in range(1, steps + 1):
            fraction = step / steps
            yield (
                float(first["x"]) + fraction * dx,
                float(first["y"]) + fraction * dy,
                _wrap(float(first["yaw"]) + fraction * dyaw),
            )


def validate_path(ctx: MapContext, query: Query, points: Optional[Sequence[Mapping[str, Any]]]) -> Dict[str, Any]:
    values: Dict[str, Any] = {
        "static_footprint_valid": False, "kinematic_valid": False,
        "path_length_m": None, "minimum_clearance_m": None,
        "curvature_p95": None, "maximum_curvature": None,
        "heading_discontinuity_count": 0, "reverse_distance_m": 0.0,
        "in_place_rotation_count": 0, "position_discontinuity_count": 0, "steering_jump_count": 0,
        "start_position_error_m": None, "start_yaw_error_rad": None,
        "goal_position_error_m": None, "goal_yaw_error_rad": None,
        "failure_code": "EMPTY_PATH", "failure_detail": "path is empty",
    }
    if not points:
        return values
    required = ("x", "y", "yaw", "source", "motion_direction", "steering", "planner_backend", "backend_version", "source_commit", "path_hash")
    if any(any(field not in point for field in required) for point in points):
        values.update(failure_code="PATH_SCHEMA_INVALID", failure_detail="required path field missing")
        return values
    failures: List[str] = []
    kinematic_failures: List[str] = []
    collisions = sum(
        ctx.hospital_map.footprint_collision(pose, FOOTPRINT, unknown_is_collision=True)
        for pose in _collision_check_poses(points)
    )
    lengths = [math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(points, points[1:])]
    curvatures = [_curvature(a, b, c) for a, b, c in zip(points, points[1:], points[2:])]
    heading_jumps = sum(abs(_delta(float(b["yaw"]), float(a["yaw"]))) > math.radians(25.0) for a, b in zip(points, points[1:]))
    steering_jumps = sum(abs(float(b["steering"]) - float(a["steering"])) > math.radians(15.0) + 1.0e-6 for a, b in zip(points, points[1:]))
    rotations = sum(d <= 1.0e-9 and abs(_delta(float(b["yaw"]), float(a["yaw"]))) > 1.0e-6 for a, b, d in zip(points, points[1:], lengths))
    reverse = sum(d for a, b, d in zip(points, points[1:], lengths) if str(a["motion_direction"]) != "forward" or str(b["motion_direction"]) != "forward" or (d > 1e-9 and (float(b["x"]) - float(a["x"])) * math.cos(float(a["yaw"])) + (float(b["y"]) - float(a["y"])) * math.sin(float(a["yaw"])) < -1e-6))
    discontinuities = sum(d > MAX_PATH_SAMPLE_SPACING_M + 1.0e-9 for d in lengths)
    start_error = math.hypot(float(points[0]["x"]) - query.start[0], float(points[0]["y"]) - query.start[1])
    start_yaw_error = abs(_delta(float(points[0]["yaw"]), query.start[2]))
    goal_error = math.hypot(float(points[-1]["x"]) - query.goal[0], float(points[-1]["y"]) - query.goal[1])
    goal_yaw_error = abs(_delta(float(points[-1]["yaw"]), query.goal[2]))
    max_curvature = max(curvatures, default=0.0)
    if collisions: failures.append("STATIC_FOOTPRINT_COLLISION")
    if reverse > 1.0e-6: kinematic_failures.append("REVERSE_MOTION")
    if rotations: kinematic_failures.append("IN_PLACE_ROTATION_FORBIDDEN")
    # Smac's discretized primitive can overshoot the analytic curvature by a
    # sub-millimetre-scale numerical amount.  Keep the hard 2.50 1/m protocol
    # while allowing the documented 1e-3 numerical tolerance.
    if max_curvature > 2.5 + KINEMATIC_NUMERICAL_TOLERANCE: kinematic_failures.append("MAXIMUM_CURVATURE_VIOLATION")
    if heading_jumps: kinematic_failures.append("HEADING_DISCONTINUITY")
    if steering_jumps: kinematic_failures.append("STEERING_DISCONTINUITY")
    if discontinuities: kinematic_failures.append("POSITION_DISCONTINUITY")
    if start_error > 0.25: kinematic_failures.append("START_POSITION_ERROR")
    if start_yaw_error > math.radians(10.0): kinematic_failures.append("START_YAW_ERROR")
    if goal_error > 0.25: kinematic_failures.append("ENDPOINT_POSITION_ERROR")
    if goal_yaw_error > math.radians(10.0): kinematic_failures.append("ENDPOINT_YAW_ERROR")
    failures.extend(kinematic_failures)
    values.update(
        static_footprint_valid=collisions == 0,
        kinematic_valid=not kinematic_failures,
        path_length_m=sum(lengths),
        minimum_clearance_m=min((ctx.hospital_map.clearance(float(p["x"]), float(p["y"])) or 0.0) for p in points),
        curvature_p95=float(np.percentile(curvatures, 95)) if curvatures else 0.0,
        maximum_curvature=max_curvature,
        heading_discontinuity_count=int(heading_jumps), reverse_distance_m=reverse,
        in_place_rotation_count=int(rotations), position_discontinuity_count=int(discontinuities), steering_jump_count=int(steering_jumps),
        start_position_error_m=start_error, start_yaw_error_rad=start_yaw_error,
        goal_position_error_m=goal_error, goal_yaw_error_rad=goal_yaw_error,
        failure_code=failures[0] if failures else "", failure_detail=", ".join(failures),
    )
    return values


def _record_backend_call(
    diagnostics: Dict[str, Any], result: PlanResult, role: str, **details: Any,
) -> None:
    result_diagnostics = result.diagnostics or {}
    diagnostics.setdefault("backend_calls", []).append({
        "role": role,
        "planner_backend": result.planner_backend,
        "backend_version": result.backend_version,
        "called": bool(result_diagnostics.get("backend_called", False)),
        "planner_success": result.planner_success,
        "failure_code": result.failure_code,
        **details,
    })


def plan_layered(
    ctx: MapContext, query: Query, mode: str, specs: Mapping[str, BackendSpec], timeout_s: float,
    topology: Optional[TopologyArtifact], output: Optional[Path] = None,
    capture_allowed_mask: bool = False,
    cache_mode: str = CACHE_MODE_BASELINE,
) -> Tuple[PlanResult, Dict[str, Any]]:
    if cache_mode not in {CACHE_MODE_BASELINE, CACHE_MODE_OPTIMIZED}:
        raise ValueError(f"unsupported cache mode: {cache_mode}")
    diagnostics: Dict[str, Any] = {
        "layer_mode": mode, "l1_route": False, "l2_attempts": [],
        "l3_attempted": False, "backend_calls": [],
        "cache_mode": cache_mode,
        "l1_attachment_lookup_ms": 0.0,
        "l1_candidate_collision_check_ms": 0.0,
        "l1_adjacency_build_ms": 0.0,
        "l1_route_search_ms": 0.0,
        "l1_route_construction_ms": 0.0,
        "l1_graph_search_ms": 0.0,
        "l1_total_time_ms": 0.0,
        "l1_start_candidate_count": 0,
        "l1_goal_candidate_count": 0,
        "l1_candidate_pair_attempts": 0,
        "topology_adjacency_cache_hit": bool(getattr(getattr(topology, "graph", None), "adjacency_cache_hit", False)),
        "endpoint_spatial_index_cache_hit": False,
        "endpoint_candidate_cache_hit": False,
        "route_cache_hit": False,
    }
    if mode == "full_grid":
        result = plan_grid_astar(ctx, query, timeout_s, source="full_grid")
        _record_backend_call(diagnostics, result, "l2_full_grid")
        result.diagnostics = {**(result.diagnostics or {}), **diagnostics}
        return result, diagnostics
    if topology is None:
        return PlanResult(failure_code="TOPOLOGY_BUILD_FAILED", failure_detail="topology artifact unavailable", source="l1"), diagnostics
    l1_started = time.monotonic_ns()
    endpoint_started = time.monotonic_ns()
    start_cell = ctx.hospital_map.world_to_cell(*query.start[:2])
    goal_cell = ctx.hospital_map.world_to_cell(*query.goal[:2])
    if cache_mode == CACHE_MODE_OPTIMIZED:
        # Import lazily: the optimized helper imports this legacy module for
        # constants and path validation, so importing it at module load time
        # would create a circular import.
        from . import l1_l3_corridor_hybrid_smoke as optimized_candidate

        attach_timing: Dict[str, Any] = {}
        start_attachment, goal_attachment, route, attach_reason = optimized_candidate._select_route_with_endpoint_attach(
            topology, query, cache_mode=optimized_candidate.CACHE_MODE_OPTIMIZED,
            timing=attach_timing,
        )
        diagnostics.update({
            "l1_attachment_lookup_ms": float(attach_timing.get("start_lookup_ms", 0.0))
            + float(attach_timing.get("goal_lookup_ms", 0.0)),
            "l1_candidate_collision_check_ms": float(attach_timing.get("start_collision_check_ms", 0.0))
            + float(attach_timing.get("goal_collision_check_ms", 0.0)),
            "l1_start_candidate_count": int(attach_timing.get("start_candidate_count", 0)),
            "l1_goal_candidate_count": int(attach_timing.get("goal_candidate_count", 0)),
            "endpoint_spatial_index_cache_hit": bool(attach_timing.get("endpoint_spatial_index_cache_hit", False)),
            "endpoint_candidate_cache_hit": bool(attach_timing.get("endpoint_candidate_cache_hit", False)),
            "route_cache_hit": bool(attach_timing.get("route_cache_hit", False)),
            "l1_adjacency_build_ms": float(attach_timing.get("adjacency_build_ms", 0.0)),
            "l1_route_search_ms": float(attach_timing.get("route_search_ms", 0.0)),
            "l1_route_construction_ms": float(attach_timing.get("route_construction_ms", 0.0)),
            "l1_candidate_pair_attempts": int(attach_timing.get("candidate_pair_attempts", 0)),
            "topology_adjacency_cache_hit": bool(attach_timing.get(
                "topology_adjacency_cache_hit",
                diagnostics["topology_adjacency_cache_hit"],
            )),
            "endpoint_attach_reason": attach_reason,
        })
    else:
        # Preserve the frozen V7 baseline attach semantics exactly.  The
        # timings below are observational only and do not alter selection.
        attach_started = time.monotonic_ns()
        start_attachment = attach_pose(topology, query.start, FOOTPRINT)
        goal_attachment = attach_pose(topology, query.goal, FOOTPRINT)
        diagnostics["l1_attachment_lookup_ms"] = (time.monotonic_ns() - attach_started) / 1.0e6
        diagnostics["l1_start_candidate_count"] = int(start_attachment is not None)
        diagnostics["l1_goal_candidate_count"] = int(goal_attachment is not None)
        route = None
        attach_reason = "legacy_attach"
        if start_attachment is not None and goal_attachment is not None:
            adjacency_started = time.monotonic_ns()
            topology.graph.adjacency()
            diagnostics["l1_adjacency_build_ms"] = (time.monotonic_ns() - adjacency_started) / 1.0e6
            route_started = time.monotonic_ns()
            route = search_topology(topology, start_attachment.node_id, goal_attachment.node_id)
            diagnostics["l1_route_search_ms"] = (time.monotonic_ns() - route_started) / 1.0e6
            diagnostics["topology_adjacency_cache_hit"] = bool(
                getattr(getattr(topology, "graph", None), "adjacency_cache_hit", False)
            )
    diagnostics["l1_graph_search_ms"] = float(
        diagnostics["l1_attachment_lookup_ms"]
        + diagnostics["l1_candidate_collision_check_ms"]
        + diagnostics["l1_adjacency_build_ms"]
        + diagnostics["l1_route_search_ms"]
        + diagnostics["l1_route_construction_ms"]
    )
    diagnostics["l1_total_time_ms"] = float((time.monotonic_ns() - l1_started) / 1.0e6)
    if start_cell is None or goal_cell is None or start_attachment is None or goal_attachment is None:
        diagnostics["failure_code"] = "TOPOLOGY_ENDPOINT_NOT_ATTACHABLE"
        if mode == "topology_guided_grid_fallback":
            result = plan_grid_astar(ctx, query, timeout_s, source="full_grid_fallback")
            _record_backend_call(diagnostics, result, "l2_full_grid_fallback", fallback_trigger="TOPOLOGY_ENDPOINT_NOT_ATTACHABLE")
            result.diagnostics = {**(result.diagnostics or {}), **diagnostics}
            return result, diagnostics
        return PlanResult(failure_code="TOPOLOGY_ENDPOINT_NOT_ATTACHABLE", failure_detail="L1 could not attach endpoint", source="l1"), diagnostics
    # Baseline route was searched above; optimized route is selected by the
    # multi-goal helper.  Keep this guard to make malformed custom helpers a
    # structured failure instead of performing a hidden second search.
    if route is None:
        diagnostics["failure_code"] = "TOPOLOGY_NO_ROUTE"
        if mode == "topology_guided_grid_fallback":
            result = plan_grid_astar(ctx, query, timeout_s, source="full_grid_fallback")
            _record_backend_call(diagnostics, result, "l2_full_grid_fallback", fallback_trigger="TOPOLOGY_NO_ROUTE")
            result.diagnostics = {**(result.diagnostics or {}), **diagnostics}
            return result, diagnostics
        return PlanResult(failure_code="TOPOLOGY_NO_ROUTE", failure_detail="L1 graph route absent", source="l1"), diagnostics
    diagnostics.update({"l1_route": True, "topology_node_ids": route.node_ids, "topology_edge_ids": route.edge_ids, "corridor_min_width_m": route.min_width_m})
    if mode == "l1_l2_geometric_rrt_star":
        padding = max(1.0, min(4.0, route.min_width_m / 2.0))
        allowed = corridor_mask(topology, route, start_cell, goal_cell, padding)
        diagnostics["rrt_corridor_padding_m"] = padding
        diagnostics["rrt_allowed_cells"] = int(np.count_nonzero(allowed))
        result = plan_ompl(
            ctx, query, "geometric_rrt_star", specs["geometric_rrt_star"], timeout_s,
            source="topology_guided_rrt_star", allowed_mask=allowed,
        )
        _record_backend_call(diagnostics, result, "l2_corridor_rrt_star", corridor_padding_m=padding)
        diagnostics["failure_code"] = result.failure_code
        result.diagnostics.update(diagnostics)
        return result, diagnostics

    l2_candidate: Optional[PlanResult] = None
    for padding in (1.0, 2.0, 4.0):
        allowed = corridor_mask(topology, route, start_cell, goal_cell, padding)
        candidate = plan_grid_astar(ctx, query, timeout_s, allowed_mask=allowed, source="topology_guided_grid")
        _record_backend_call(diagnostics, candidate, "l2_corridor_grid", corridor_padding_m=padding)
        diagnostics["l2_attempts"].append({"padding_m": padding, "failure_code": candidate.failure_code, "expanded_states": candidate.expanded_states})
        if candidate.planner_success:
            l2_candidate = candidate
            if capture_allowed_mask:
                # Runtime-only object used by the fixed efficiency pipeline
                # for collision-preserving L2 simplification.  It is removed
                # before CSV/report serialization.
                diagnostics["_allowed_mask_runtime"] = allowed
            break
    if mode in {"l3_hybrid_repair", "l3_kinodynamic_rrt"}:
        diagnostics["l3_attempted"] = True
        if l2_candidate is None or not l2_candidate.points:
            return PlanResult(failure_code="L2_PATH_UNAVAILABLE", failure_detail="local L3 repair has no L2 input path", source="l3", diagnostics=diagnostics), diagnostics
        points = l2_candidate.points
        trigger = next((index for index, triple in enumerate(zip(points, points[1:], points[2:]), start=1) if _curvature(*triple) > 2.5 + 1.0e-6), len(points) // 2)
        first = trigger
        distance = 0.0
        while first > 0 and distance < 2.0:
            distance += math.hypot(points[first]["x"] - points[first - 1]["x"], points[first]["y"] - points[first - 1]["y"])
            first -= 1
        last = trigger
        distance = 0.0
        while last + 1 < len(points) and distance < 2.0:
            distance += math.hypot(points[last + 1]["x"] - points[last]["x"], points[last + 1]["y"] - points[last]["y"])
            last += 1
        local_query = Query(
            query_id=f"{query.query_id}_{mode}_{first}_{last}",
            start=[float(points[first]["x"]), float(points[first]["y"]), float(points[first]["yaw"])],
            goal=[float(points[last]["x"]), float(points[last]["y"]), float(points[last]["yaw"])],
            category="local_repair", seed=query.seed,
        )
        rows, cols = np.indices(ctx.free_mask.shape)
        first_cell = ctx.hospital_map.world_to_cell(local_query.start[0], local_query.start[1])
        last_cell = ctx.hospital_map.world_to_cell(local_query.goal[0], local_query.goal[1])
        if first_cell is None or last_cell is None:
            return PlanResult(failure_code="LOCAL_WINDOW_INVALID", failure_detail="repair window endpoint outside map", source="l3", diagnostics=diagnostics), diagnostics
        margin_cells = int(math.ceil(1.5 / ctx.hospital_map.resolution))
        local_mask = ctx.free_mask & (rows >= min(first_cell[0], last_cell[0]) - margin_cells) & (rows <= max(first_cell[0], last_cell[0]) + margin_cells) & (cols >= min(first_cell[1], last_cell[1]) - margin_cells) & (cols <= max(first_cell[1], last_cell[1]) + margin_cells)
        diagnostics.update({"local_window_start_index": first, "local_window_end_index": last, "local_window_allowed_cells": int(np.count_nonzero(local_mask))})
        if output is None:
            hybrid_result = PlanResult(
                failure_code="LOCAL_HYBRID_OUTPUT_REQUIRED", failure_detail="hard-bounded Smac adapter requires an output directory",
                planner_backend=specs["hybrid_astar"].backend, backend_version=specs["hybrid_astar"].version,
                source="l3_hybrid_smac", diagnostics={},
            )
        else:
            hybrid_result = plan_local_smac(ctx, local_query, specs["hybrid_astar"], local_mask, output)
        _record_backend_call(diagnostics, hybrid_result, "l3_local_hybrid")
        diagnostics["hybrid_failure_code"] = hybrid_result.failure_code
        if hybrid_result.planner_success and hybrid_result.points:
            stitched = list(points[:first]) + hybrid_result.points + list(points[last + 1:])
            hybrid_result.points = stitched
            hybrid_result.source = "layered_l1_l2_l3_hybrid"
            hybrid_result.diagnostics.update(diagnostics)
            return hybrid_result, diagnostics
        if mode == "l3_hybrid_repair":
            diagnostics["failure_code"] = hybrid_result.failure_code
            hybrid_result.diagnostics.update(diagnostics)
            return hybrid_result, diagnostics
        diagnostics["kinodynamic_fallback_attempted"] = True
        result = plan_ompl(
            ctx, local_query, "kinodynamic_rrt", specs["kinodynamic_rrt"], timeout_s,
            source="l3_kinodynamic_sst", allowed_mask=local_mask,
        )
        _record_backend_call(
            diagnostics, result, "l3_local_kinodynamic_fallback",
            fallback_trigger=hybrid_result.failure_code,
        )
        diagnostics["failure_code"] = result.failure_code
        if not result.planner_success or not result.points:
            result.diagnostics.update(diagnostics)
            return result, diagnostics
        stitched = list(points[:first]) + result.points + list(points[last + 1:])
        result.points = stitched
        result.source = "layered_l1_l2_l3_sst"
        result.diagnostics.update(diagnostics)
        return result, diagnostics
    if l2_candidate is not None:
        l2_candidate.diagnostics.update(diagnostics)
        return l2_candidate, diagnostics
    if mode == "topology_guided_grid_fallback":
        candidate = plan_grid_astar(ctx, query, timeout_s, source="full_grid_fallback")
        _record_backend_call(diagnostics, candidate, "l2_full_grid_fallback", fallback_trigger="CORRIDOR_NO_PATH")
        candidate.diagnostics.update(diagnostics)
        return candidate, diagnostics
    diagnostics["failure_code"] = "CORRIDOR_NO_PATH"
    return PlanResult(failure_code="CORRIDOR_NO_PATH", failure_detail="all configured corridor widths failed", source="l2", diagnostics=diagnostics), diagnostics


def _write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    materialized = list(rows)
    fields: List[str] = []
    for row in materialized:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or ["empty"])
        writer.writeheader()
        def encode(value: Any) -> str:
            return json.dumps(value, sort_keys=True, default=lambda item: item.item() if isinstance(item, np.generic) else str(item))
        for row in materialized:
            writer.writerow({key: encode(value) if isinstance(value, (dict, list)) else (value.item() if isinstance(value, np.generic) else value) for key, value in row.items()})


def _refuse_nonempty(path: Path) -> None:
    if path.exists() and any(path.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output: {path}")


def _source_commit() -> Optional[str]:
    try:
        value = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        return value or None
    except (OSError, subprocess.SubprocessError):
        return None


def _numeric_values(rows: Sequence[Mapping[str, Any]], field: str) -> List[float]:
    return [float(row[field]) for row in rows if row.get(field) is not None]


def _percentile_text(rows: Sequence[Mapping[str, Any]], field: str, percentile: float) -> str:
    values = _numeric_values(rows, field)
    return "n/a" if not values else f"{float(np.percentile(values, percentile)):.2f}"


def _write_final_report(
    output: Path,
    run_rows: Sequence[Mapping[str, Any]],
    specs: Mapping[str, BackendSpec],
    topologies: Mapping[str, TopologyArtifact],
    *,
    warmups: int,
    repetitions: int,
) -> None:
    measured = [row for row in run_rows if row.get("run_mode") == "measured"]
    singles = [row for row in measured if row.get("algorithm") in ALGORITHMS]
    layered = [row for row in measured if row.get("algorithm") in LAYERED_MODES]
    query_ids = sorted({str(row["query_id"]) for row in run_rows})
    map_ids = list(topologies)
    lines = [
        "# PLN-02 Unified Four-Backend and Three-Layer Smoke Report",
        "",
        "This report describes a bounded static-map smoke diagnostic. It is not a formal performance ranking.",
        "",
        "## 1. Scope and protocol",
        "",
        f"The run covers {', '.join(f'`{item}`' for item in map_ids)} and queries {', '.join(f'`{item}`' for item in query_ids)}, with {warmups} warmup and {repetitions} measured single-planner repetition(s). `dynamic_obstacles=false`; resolution is fixed at 0.05 m/cell; the complete Jackal rectangle is checked along every path segment; reverse and in-place rotation are forbidden; `Rmin=0.40 m` and maximum curvature `2.50 1/m` are hard acceptance limits. q08 is never replaced and is skipped only where endpoint validation rejects it.",
        "",
        "## 2. Actual backend identities",
        "",
        "| Baseline | Actual call target | Version | Callable |",
        "|---|---|---:|---:|",
    ]
    for algorithm in ALGORITHMS:
        spec = specs[algorithm]
        lines.append(f"| `{algorithm}` | `{spec.backend}` | `{spec.version}` | {spec.available} |")
    lines.extend([
        "",
        "Grid A* searches only `(x,y)` cells. RRT* is OMPL `geometric::RRTstar`. Hybrid A* is Nav2 `SmacPlannerHybrid` with forward-only `DUBIN` primitives. The kinodynamic backend is OMPL `control::SST` over `(x,y,yaw,velocity,steering)` with acceleration and steering-rate controls; it is not AO-RRT*.",
        "",
        "## 3. Native single-planner performance diagnostics",
        "",
        "| Algorithm | Attempts | Planner success | Successful planning P50 ms | Call wall P50 ms |",
        "|---|---:|---:|---:|---:|",
    ])
    for algorithm in ALGORITHMS:
        rows = [row for row in singles if row["algorithm"] == algorithm]
        successful_rows = [row for row in rows if row["planner_success"]]
        lines.append(
            f"| `{algorithm}` | {len(rows)} | {sum(bool(row['planner_success']) for row in rows)} | "
            f"{_percentile_text(successful_rows, 'planning_time_ms', 50)} | "
            f"{_percentile_text(rows, 'wall_time_ms', 50)} |"
        )
    lines.extend([
        "",
        "These timings are smoke observations only. RRT* `samples` is the OMPL main-loop sampling iteration count and `generated_states` is the retained tree vertex count. OMPL does not expose an exact RRT* rewire counter. SST exposes neither an exact sample count nor rewiring, so those fields are null rather than inferred from vertices.",
        "",
        "## 4. Native and final path validity",
        "",
        "| Algorithm | Native path valid | Static footprint valid | Kinematic valid | Final valid |",
        "|---|---:|---:|---:|---:|",
    ])
    for algorithm in ALGORITHMS:
        rows = [row for row in singles if row["algorithm"] == algorithm]
        lines.append(
            f"| `{algorithm}` | {sum(bool(row['native_path_valid']) for row in rows)}/{len(rows)} | "
            f"{sum(bool(row['static_footprint_valid']) for row in rows)}/{len(rows)} | "
            f"{sum(bool(row['kinematic_valid']) for row in rows)}/{len(rows)} | "
            f"{sum(bool(row['final_valid_success']) for row in rows)}/{len(rows)} |"
        )
    lines.extend([
        "",
        "For the two geometric baselines, `native_path_valid` means a nonempty, statically valid native geometric path; `final_valid_success` additionally requires the common hard kinematic acceptance. No shared fallback or common Hybrid repair is applied to single-planner rows.",
        "",
        "## 5. L1/L2/L3 composed-system results",
        "",
        "| Mode | Attempts | Planner success | Static valid | Final valid | Pipeline wall P50 ms |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for mode in LAYERED_MODES:
        rows = [row for row in layered if row["algorithm"] == mode]
        lines.append(
            f"| `{mode}` | {len(rows)} | {sum(bool(row['planner_success']) for row in rows)} | "
            f"{sum(bool(row['static_footprint_valid']) for row in rows)} | "
            f"{sum(bool(row['final_valid_success']) for row in rows)} | "
            f"{_percentile_text(rows, 'pipeline_wall_time_ms', 50)} |"
        )
    lines.extend(["", "L1 artifacts record skeleton graph nodes, edges, connected components, channel width and clearance. L2 tries 1 m, 2 m and 4 m topology corridors before its documented full-grid fallback. L1+L2 RRT* samples only inside the persisted corridor mask. L3 writes a hard-bounded static map, calls local Smac first, and calls SST only after local Smac failure.", ""])
    for map_id, topology in topologies.items():
        metadata = topology.metadata
        lines.append(
            f"- `{map_id}` L1: {metadata.get('graph_nodes')} nodes, {metadata.get('graph_edges')} edges, "
            f"{metadata.get('graph_components')} components, precompute {float(metadata.get('precompute_wall_time_ms', 0.0)):.2f} ms."
        )
    lines.extend([
        "",
        "## 6. Provenance and surrogate boundary",
        "",
        "`protocol.yaml`, `manifest.yaml`, `source_manifest.yaml`, the topology/local-map artifacts, per-point backend/source fields, path hashes, and the expanded backend call log are the provenance source of truth. Historical in-repository bicycle and simplified RRT implementations remain reference/surrogate code only and are excluded from every row in this run.",
        "",
        "## 7. Why yaw is local and its cost",
        "",
        "L1 graph search and L2 grid/RRT* stay two-dimensional because yaw would multiply the state space across the full 80-100 m maps. L3 introduces yaw, velocity and steering only around a violating window. The table above reports full pipeline wall time, while backend-specific planning/RSS/PSS fields retain the local Hybrid or SST cost; neither is mixed with Gazebo, navigation execution or local velocity control.",
        "",
        "## 8. Diagnostic comparison boundary",
        "",
        "The smoke records full-grid A*, full-map Smac, full-map SST, topology-guided grid/RRT*, and local L3 attempts under one validator. Relative speed or memory benefit is not claimed unless both compared branches return final-valid paths. A failed or invalid fast path is not treated as a performance win.",
        "",
        "## 9. Failures and applicability",
        "",
    ])
    failure_counts: Dict[Tuple[str, str], int] = {}
    for row in measured:
        code = str(row.get("failure_code") or "")
        if code:
            key = (str(row["algorithm"]), code)
            failure_counts[key] = failure_counts.get(key, 0) + 1
    for (algorithm, code), count in sorted(failure_counts.items(), key=lambda item: (-item[1], item[0])):
        lines.append(f"- `{algorithm}` / `{code}`: {count} measured row(s).")
    final_by_map = {
        map_id: sum(bool(row["final_valid_success"]) for row in measured if row["map_id"] == map_id)
        for map_id in topologies
    }
    kinematic_baseline_gate = all(
        any(
            bool(row["final_valid_success"])
            for row in singles
            if row["map_id"] == map_id and row["algorithm"] == algorithm
        )
        for map_id in topologies
        for algorithm in ("hybrid_astar", "kinodynamic_rrt")
    )
    layered_gate = all(
        any(bool(row["final_valid_success"]) for row in layered if row["map_id"] == map_id)
        for map_id in topologies
    )
    smoke_gate = (
        all(spec.available for spec in specs.values())
        and kinematic_baseline_gate
        and layered_gate
    )
    lines.extend([
        "",
        "## 10. Formal-experiment gate",
        "",
        f"Backend availability is {'complete' if all(spec.available for spec in specs.values()) else 'incomplete'}. Final-valid measured rows by map: "
        + ", ".join(f"`{map_id}`={count}" for map_id, count in final_by_map.items()) + ".",
        f"Full-map kinematic-baseline gate: {kinematic_baseline_gate}. Layered final-valid gate: {layered_gate}.",
        "",
        ("The smoke validity gate passed; tests and build still have to pass before a formal four-map launch." if smoke_gate else "The smoke validity gate did not pass. Do not launch the 200 m/400 m or formal four-map experiment; the failure modes above remain blockers."),
    ])
    (output / "final_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_smoke(output: Path, *, map_ids: Sequence[str] = tuple(MAP_PATHS), query_ids: Optional[Sequence[str]] = None, warmups: int = 1, repetitions: int = 2) -> Path:
    _refuse_nonempty(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "paths").mkdir()
    queries = _queries()
    selected = [queries[qid] for qid in (query_ids or tuple(queries))]
    os.environ.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    specs = backend_availability()
    contexts = {map_id: _context(map_id) for map_id in map_ids}
    topologies: Dict[str, TopologyArtifact] = {}
    for map_id in map_ids:
        topology_started_ns = time.monotonic_ns()
        topology_before = resource.getrusage(resource.RUSAGE_SELF)
        topology = build_topology(
            contexts[map_id].hospital_map, FOOTPRINT,
            padding_m=0.05, safety_margin_m=0.05, allow_unknown=False,
        )
        topology_after = resource.getrusage(resource.RUSAGE_SELF)
        topology.metadata["precompute_wall_time_ms"] = (time.monotonic_ns() - topology_started_ns) / 1e6
        topology.metadata["precompute_cpu_time_ms"] = max(
            0.0,
            (
                topology_after.ru_utime - topology_before.ru_utime
                + topology_after.ru_stime - topology_before.ru_stime
            ) * 1000.0,
        )
        topologies[map_id] = topology
    for map_id, topology in topologies.items():
        save_topology(topology, output / "topology" / map_id)
    started = dt.datetime.now(dt.timezone.utc).isoformat()
    source_commit = _source_commit()
    protocol = {
        "schema_version": 1, "experiment": "pln02_unified_four_backends_smoke_v3", "dynamic_obstacles": False,
        "resolution": 0.05, "minimum_turning_radius_m": 0.40, "maximum_curvature": 2.50,
        "allow_reverse": False, "allow_in_place_rotation": False, "footprint": FOOTPRINT,
        "footprint_padding_m": 0.05, "additional_safety_margin_m": 0.05,
        "collision_sample_spacing_m": COLLISION_SAMPLE_SPACING_M,
        "collision_yaw_sample_step_deg": math.degrees(COLLISION_YAW_SAMPLE_STEP_RAD),
        "maximum_path_sample_spacing_m": MAX_PATH_SAMPLE_SPACING_M,
        "endpoint_position_tolerance_m": 0.25, "endpoint_yaw_tolerance_deg": 10.0,
        "steering_continuity_step_deg": 15.0, "random_seed": 20260821,
        "warmup_runs": warmups, "measured_runs": repetitions, "query_policy": "q00-q09; q08 retained in validation and skipped when endpoint-invalid",
        "algorithms": list(ALGORITHMS), "layered_modes": list(LAYERED_MODES), "formal_performance_conclusions": False,
        "backend_contracts": {
            "grid_astar": {"state": ["x", "y"], "connectivity": 8},
            "geometric_rrt_star": {"state": ["x", "y"], "backend": "OMPL geometric::RRTstar", "range_m": 1.0, "goal_bias": 0.10},
            "hybrid_astar": {"state": ["x", "y", "yaw"], "backend": "Nav2 SmacPlannerHybrid", "motion_model": "DUBIN", "minimum_turning_radius_m": 0.40},
            "kinodynamic_rrt": {"state": ["x", "y", "yaw", "velocity", "steering"], "control": ["acceleration", "steering_rate"], "backend": "OMPL control::SST", "wheelbase_m": 0.50},
        },
    }
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    (output / "core_queries.yaml").write_text(SOURCE_QUERIES.read_text(encoding="utf-8"), encoding="utf-8")
    availability_rows = []
    for spec in specs.values():
        availability_rows.append({"algorithm": spec.name, "planner_backend": spec.backend, "backend_version": spec.version, "available": spec.available, "mature": spec.mature, "reason": spec.reason})
    backend_gate = "All four mature single-planner backends are callable." if all(spec.available for spec in specs.values()) else "No formal ranking is permitted while a mature backend is unavailable."
    (output / "backend_availability.md").write_text("# Backend availability\n\n" + "\n".join(f"- **{r['algorithm']}**: `{r['planner_backend']}` {r['backend_version']}; available={r['available']}; {r['reason']}" for r in availability_rows) + f"\n\n{backend_gate}\n", encoding="utf-8")
    run_rows: List[Dict[str, Any]] = []
    metric_rows: List[Dict[str, Any]] = []
    failure_rows: List[Dict[str, Any]] = []
    call_rows: List[Dict[str, Any]] = []
    repetitions_spec = [("warmup", warmups), ("measured", repetitions)]
    smac_results: Dict[Tuple[str, str, str, int], PlanResult] = {}
    if specs["hybrid_astar"].available:
        for map_id, ctx in contexts.items():
            session: Optional[SmacSession] = None
            try:
                session = SmacSession(ctx, output)
                session.start()
                for query in selected:
                    validation = ctx.hospital_map.validate_query(query, FOOTPRINT, 0.5, allow_unknown=False)
                    if validation.validation_status != "VALID":
                        continue
                    for run_mode, count in repetitions_spec:
                        for repetition in range(1, count + 1):
                            smac_results[(map_id, query.query_id, run_mode, repetition)] = session.plan(query, specs["hybrid_astar"])
            except (RuntimeError, OSError) as exc:
                failure = PlanResult(
                    failure_code="BACKEND_START_FAILED", failure_detail=str(exc),
                    planner_backend=specs["hybrid_astar"].backend,
                    backend_version=specs["hybrid_astar"].version,
                )
                for query in selected:
                    for run_mode, count in repetitions_spec:
                        for repetition in range(1, count + 1):
                            smac_results[(map_id, query.query_id, run_mode, repetition)] = failure
            finally:
                if session is not None:
                    session.close()
    for map_id, ctx in contexts.items():
        for query in selected:
            validation = ctx.hospital_map.validate_query(query, FOOTPRINT, 0.5, allow_unknown=False).as_dict()
            invalid_query = validation["validation_status"] != "VALID"
            for algorithm in ALGORITHMS:
                for run_mode, count in repetitions_spec:
                    for repetition in range(1, count + 1):
                        started_ns = time.monotonic_ns()
                        before = resource.getrusage(resource.RUSAGE_SELF)
                        if invalid_query:
                            result = PlanResult(failure_code="INVALID_QUERY", failure_detail=validation.get("reason", ""), planner_backend=specs[algorithm].backend, backend_version=specs[algorithm].version)
                        elif algorithm == "hybrid_astar" and (map_id, query.query_id, run_mode, repetition) in smac_results:
                            result = smac_results[(map_id, query.query_id, run_mode, repetition)]
                        else:
                            result = plan_single(ctx, query, algorithm, specs, TIMEOUTS[map_id])
                        after = resource.getrusage(resource.RUSAGE_SELF)
                        elapsed_ms = (time.monotonic_ns() - started_ns) / 1e6
                        points = result.points
                        path_hash = _enrich_path(points, source_commit)
                        metrics = validate_path(ctx, query, points)
                        path_file = ""
                        if points and run_mode == "measured":
                            path_file = f"paths/{map_id}_{query.query_id}_{algorithm}_{repetition}.json"
                            (output / path_file).write_text(json.dumps(points, sort_keys=True), encoding="utf-8")
                        failure_code = result.failure_code or metrics["failure_code"]
                        failure_detail = result.failure_detail if result.failure_code else metrics["failure_detail"]
                        result_diagnostics = result.diagnostics or {}
                        planner_cpu_ms = result_diagnostics.get("planner_cpu_total_ms")
                        planner_rss = result_diagnostics.get("planner_rss_peak_bytes")
                        planner_pss = result_diagnostics.get("planner_pss_peak_bytes")
                        row = {
                            "run_id": f"{map_id}_{query.query_id}_{algorithm}_{run_mode}_{repetition}", "map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode, "repetition": repetition,
                            "planner_success": result.planner_success, "action_success": bool(points), "native_path_valid": bool(points and metrics["static_footprint_valid"] and (algorithm in {"grid_astar", "geometric_rrt_star"} or metrics["kinematic_valid"])), "static_footprint_valid": metrics["static_footprint_valid"], "kinematic_valid": metrics["kinematic_valid"], "final_valid_success": bool(points and metrics["static_footprint_valid"] and metrics["kinematic_valid"]),
                            "failure_code": failure_code, "failure_detail": failure_detail, "planning_time_ms": result_diagnostics.get("planning_time_ms"), "wall_time_ms": result_diagnostics.get("wall_time_ms", elapsed_ms), "cpu_total_ms": planner_cpu_ms if planner_cpu_ms is not None else max(0.0, (after.ru_utime - before.ru_utime + after.ru_stime - before.ru_stime) * 1000.0), "planner_rss_peak_bytes": planner_rss if planner_rss is not None else resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024, "planner_pss_peak_bytes": planner_pss, "stack_rss_peak_bytes": result_diagnostics.get("stack_rss_peak_bytes", planner_rss), "stack_pss_peak_bytes": result_diagnostics.get("stack_pss_peak_bytes", planner_pss),
                            "path_length_m": metrics["path_length_m"], "minimum_clearance_m": metrics["minimum_clearance_m"], "curvature_p95": metrics["curvature_p95"], "maximum_curvature": metrics["maximum_curvature"], "heading_discontinuity_count": metrics["heading_discontinuity_count"], "steering_jump_count": metrics["steering_jump_count"], "reverse_distance_m": metrics["reverse_distance_m"], "in_place_rotation_count": metrics["in_place_rotation_count"], "expanded_states": result.expanded_states, "generated_states": result.generated_states, "samples": result.samples, "rewires": result.rewires, "first_solution_time_ms": result.first_solution_time_ms,
                            "planner_backend": result.planner_backend or specs[algorithm].backend, "backend_version": result.backend_version or specs[algorithm].version, "source": result.source, "source_commit": source_commit, "path_hash": path_hash, "path_file": path_file, "query_validation": validation,
                        }
                        run_rows.append(row)
                        row.update({
                            "position_discontinuity_count": metrics["position_discontinuity_count"],
                            "start_position_error_m": metrics["start_position_error_m"],
                            "start_yaw_error_rad": metrics["start_yaw_error_rad"],
                            "goal_position_error_m": metrics["goal_position_error_m"],
                            "goal_yaw_error_rad": metrics["goal_yaw_error_rad"],
                            "diagnostics": result_diagnostics,
                        })
                        metric_rows.append({"run_id": row["run_id"], **metrics})
                        if failure_code:
                            failure_rows.append({"map_id": map_id, "query_id": query.query_id, "algorithm": algorithm, "run_mode": run_mode, "failure_code": failure_code, "failure_detail": row["failure_detail"]})
                        call_rows.append({
                            "run_id": row["run_id"], "algorithm": algorithm, "call_index": 1,
                            "role": "single_planner", "planner_backend": row["planner_backend"],
                            "backend_version": row["backend_version"],
                            "backend_available": specs[algorithm].available,
                            "called": bool(result_diagnostics.get("backend_called", False)) and not invalid_query,
                            "planner_success": result.planner_success, "fallback_used": False,
                            "fallback_trigger": "", "path_hash": row["path_hash"],
                            "failure_code": result.failure_code,
                            "validation_failure_code": metrics["failure_code"],
                        })
    # Layered smoke is recorded in the same protocol but separate rows so
    # single-algorithm and composed-system results cannot be conflated.
    for map_id, ctx in contexts.items():
        for query in selected:
            if ctx.hospital_map.validate_query(query, FOOTPRINT, 0.5, allow_unknown=False).validation_status != "VALID":
                continue
            for mode in LAYERED_MODES:
                pipeline_started_ns = time.monotonic_ns()
                pipeline_before = resource.getrusage(resource.RUSAGE_SELF)
                result, diagnostics = plan_layered(ctx, query, mode, specs, TIMEOUTS[map_id], topologies[map_id], output)
                pipeline_after = resource.getrusage(resource.RUSAGE_SELF)
                pipeline_wall_time_ms = (time.monotonic_ns() - pipeline_started_ns) / 1e6
                pipeline_cpu_total_ms = max(
                    0.0,
                    (
                        pipeline_after.ru_utime - pipeline_before.ru_utime
                        + pipeline_after.ru_stime - pipeline_before.ru_stime
                    ) * 1000.0,
                )
                layered_path_hash = _enrich_path(result.points, source_commit)
                metrics = validate_path(ctx, query, result.points)
                run_id = f"{map_id}_{query.query_id}_{mode}_measured_1"
                result_diagnostics = result.diagnostics or {}
                path_file = ""
                if result.points:
                    path_file = f"paths/{map_id}_{query.query_id}_{mode}_1.json"
                    (output / path_file).write_text(json.dumps(result.points, sort_keys=True), encoding="utf-8")
                row = {"run_id": run_id, "map_id": map_id, "query_id": query.query_id, "algorithm": mode, "run_mode": "measured", "repetition": 1, "planner_success": result.planner_success, "action_success": bool(result.points), "native_path_valid": bool(result.points and metrics["static_footprint_valid"] and metrics["kinematic_valid"]), "static_footprint_valid": metrics["static_footprint_valid"], "kinematic_valid": metrics["kinematic_valid"], "final_valid_success": bool(result.points and metrics["static_footprint_valid"] and metrics["kinematic_valid"]), "planning_time_ms": result_diagnostics.get("planning_time_ms"), "wall_time_ms": result_diagnostics.get("wall_time_ms"), "pipeline_wall_time_ms": pipeline_wall_time_ms, "cpu_total_ms": result_diagnostics.get("planner_cpu_total_ms"), "pipeline_cpu_total_ms": pipeline_cpu_total_ms, "planner_rss_peak_bytes": result_diagnostics.get("planner_rss_peak_bytes"), "planner_pss_peak_bytes": result_diagnostics.get("planner_pss_peak_bytes"), "stack_rss_peak_bytes": result_diagnostics.get("stack_rss_peak_bytes", result_diagnostics.get("planner_rss_peak_bytes")), "stack_pss_peak_bytes": result_diagnostics.get("stack_pss_peak_bytes", result_diagnostics.get("planner_pss_peak_bytes")), "expanded_states": result.expanded_states, "generated_states": result.generated_states, "samples": result.samples, "rewires": result.rewires, "first_solution_time_ms": result.first_solution_time_ms, "planner_backend": result.planner_backend, "backend_version": result.backend_version, "source": result.source, "source_commit": source_commit, "path_hash": layered_path_hash, "path_file": path_file, "diagnostics": diagnostics, **metrics}
                # ``metrics`` reports path validity, while the planner result
                # reports backend availability. Preserve the latter when no
                # path exists instead of allowing EMPTY_PATH to hide it.
                row["failure_code"] = result.failure_code or metrics["failure_code"]
                row["failure_detail"] = result.failure_detail or metrics["failure_detail"]
                run_rows.append(row)
                metric_rows.append({"run_id": run_id, **metrics})
                backend_available_by_name = {spec.backend: spec.available for spec in specs.values()}
                calls = list(diagnostics.get("backend_calls") or [])
                if not calls:
                    calls = [{
                        "role": "no_path_backend_called", "planner_backend": "not_called",
                        "backend_version": "", "called": False, "planner_success": False,
                        "failure_code": row["failure_code"],
                    }]
                for call_index, call in enumerate(calls, start=1):
                    role = str(call.get("role", ""))
                    call_rows.append({
                        "run_id": run_id, "algorithm": mode, "call_index": call_index,
                        "role": role, "planner_backend": call.get("planner_backend", ""),
                        "backend_version": call.get("backend_version", ""),
                        "backend_available": backend_available_by_name.get(str(call.get("planner_backend", "")), True),
                        "called": bool(call.get("called", False)),
                        "planner_success": bool(call.get("planner_success", False)),
                        "fallback_used": "fallback" in role,
                        "fallback_trigger": call.get("fallback_trigger", ""),
                        "corridor_padding_m": call.get("corridor_padding_m"),
                        "path_hash": row["path_hash"], "failure_code": call.get("failure_code", ""),
                    })
                if row["failure_code"]:
                    failure_rows.append({"map_id": map_id, "query_id": query.query_id, "algorithm": mode, "run_mode": "measured", "failure_code": row["failure_code"], "failure_detail": row["failure_detail"]})
    _write_csv(output / "runs.csv", run_rows)
    _write_csv(output / "path_metrics.csv", metric_rows)
    failure_counts: Dict[Tuple[str, str, str], int] = {}
    failure_details: Dict[Tuple[str, str, str], str] = {}
    for failure in failure_rows:
        key = (str(failure["algorithm"]), str(failure["failure_code"]), str(failure["run_mode"]))
        failure_counts[key] = failure_counts.get(key, 0) + 1
        failure_details.setdefault(key, str(failure.get("failure_detail", "")))
    _write_csv(output / "failure_summary.csv", [
        {"algorithm": key[0], "failure_code": key[1], "run_mode": key[2], "count": count, "example_detail": failure_details[key]}
        for key, count in sorted(failure_counts.items())
    ])
    _write_csv(output / "backend_call_log.csv", call_rows)
    source_paths = [
        Path(__file__).resolve(),
        Path(__file__).resolve().parents[1] / "src" / "ompl_planner_backend.cpp",
        Path(__file__).resolve().parent / "ompl_backend_cli.py",
        Path(__file__).resolve().parents[1] / "setup.py",
        Path(__file__).resolve().parents[1] / "package.xml",
        _strict_smac_config_path(),
        Path(__file__).resolve().parent / "topology.py",
        Path(__file__).resolve().parent / "planner_benchmark" / "runner.py",
        Path(__file__).resolve().parents[1] / "test" / "test_unified_four_backends_smoke.py",
        SOURCE_QUERIES,
    ]
    source_files = {str(path): sha256_file(path) for path in source_paths}
    code_hash = hashlib.sha256("\n".join(f"{path}\0{digest}" for path, digest in sorted(source_files.items())).encode("utf-8")).hexdigest()
    source_manifest = {"source_commit": source_commit, "code_hash": code_hash, "source_files": source_files, "map_hashes": {m: contexts[m].map_sha256 for m in map_ids}}
    (output / "source_manifest.yaml").write_text(yaml.safe_dump(source_manifest, sort_keys=False), encoding="utf-8")
    manifest = {"schema_version": 1, "experiment": "pln02_unified_four_backends_smoke_v3", "created_at": started, "ended_at": dt.datetime.now(dt.timezone.utc).isoformat(), "map_ids": list(map_ids), "query_ids": [q.query_id for q in selected], "warmup_runs": warmups, "measured_runs": repetitions, "dynamic_obstacles": False, "no_formal_performance_conclusions": True, "backend_availability": {name: spec.available for name, spec in specs.items()}, "code_hash": code_hash, "run_count": len(run_rows)}
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    measured_single = [row for row in run_rows if row.get("run_mode") == "measured" and row.get("algorithm") in ALGORITHMS]
    report = ["# Unified four-backend smoke report", "", "This is a smoke diagnostic, not a formal performance result.", "", "## Backend gate", ""]
    report.extend(f"- `{name}`: available={spec.available}; `{spec.backend}`; {spec.reason}" for name, spec in specs.items())
    report.extend(["", "## Native single-planner results", ""])
    for algorithm in ALGORITHMS:
        rows = [row for row in measured_single if row["algorithm"] == algorithm]
        report.append(f"- `{algorithm}`: attempts={len(rows)}, planner_success={sum(bool(row['planner_success']) for row in rows)}, native_path_valid={sum(bool(row['native_path_valid']) for row in rows)}, final_valid_success={sum(bool(row['final_valid_success']) for row in rows)}")
    report.extend(["", "## Interpretation", "", "- All four labels have distinct call sites; no single-algorithm run uses another planner's fallback path.", "- Geometric RRT* is OMPL `geometric::RRTstar`; kinodynamic planning is OMPL `control::SST`, not AO-RRT*.", "- Historical in-repository bicycle/RRT implementations remain reference-only and are excluded.", "- L1 and L2 omit yaw to keep their search spaces two-dimensional; L3 introduces yaw and vehicle state only in a persisted hard-bounded repair window.", "- q08 remains in the input/validation set and is skipped only on maps where endpoint validation is invalid.", "- Formal four-map ranking remains blocked until both 80 m and 100 m smoke validity gates, tests and build pass."])
    (output / "smoke_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    _write_final_report(
        output, run_rows, specs, topologies,
        warmups=warmups, repetitions=repetitions,
    )
    try:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except ImportError:
        pass
    return output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the auditable PLN-02 four-backend and layered static smoke")
    parser.add_argument("--output-dir", default=str(ROOT / "experiments/layered_planner_benchmark" / OUTPUT_NAME))
    parser.add_argument("--map-id", action="append", choices=list(MAP_PATHS), dest="map_ids")
    parser.add_argument("--query-id", action="append", choices=list(_queries()), dest="query_ids")
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--repetitions", type=int, default=2)
    parser.add_argument("--no-dynamic-obstacles", action="store_true", required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        output = run_smoke(Path(args.output_dir).resolve(), map_ids=args.map_ids or tuple(MAP_PATHS), query_ids=args.query_ids, warmups=args.warmups, repetitions=args.repetitions)
    except (OSError, ValueError, KeyError) as exc:
        print(f"unified_four_backends_smoke: ERROR: {exc}")
        return 2
    print(f"smoke output: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
