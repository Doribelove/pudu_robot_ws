from __future__ import annotations
from array import array
import hashlib
import json
import math
import os
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple
import numpy as np
import yaml
from .planner_benchmark.map_utils import HospitalMap
from .planner_benchmark.models import Query
FOOTPRINT = [[0.255, 0.215], [0.255, -0.215], [-0.255, -0.215], [-0.255, 0.215]]
COLLISION_SAMPLE_SPACING_M = 0.05
COLLISION_YAW_SAMPLE_STEP_RAD = math.radians(5.0)
MAX_PATH_SAMPLE_SPACING_M = 0.15
KINEMATIC_NUMERICAL_TOLERANCE = 0.001

def _strict_smac_config_path() -> Path:
    source_path = Path(__file__).resolve().parents[2] / 'config' / 'planner_benchmark_strict_forward_smac_hybrid.yaml'
    if source_path.exists():
        return source_path
    from ament_index_python.packages import get_package_share_directory
    installed_path = Path(get_package_share_directory('arena_evaluation')) / 'config' / 'planner_benchmark_strict_forward_smac_hybrid.yaml'
    if not installed_path.exists():
        raise FileNotFoundError(f'Smac planner config not found: {installed_path}')
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
    failure_code: str = 'NO_PATH'
    failure_detail: str = ''
    planner_backend: str = ''
    backend_version: str = ''
    source: str = ''
    expanded_states: int = 0
    generated_states: int = 0
    samples: Optional[int] = None
    rewires: Optional[int] = None
    first_solution_time_ms: Optional[float] = None
    diagnostics: Dict[str, Any] = None
    path_audit: Any = None
SMAC_PARAMETER_PROFILES: Dict[str, Dict[str, Any]] = {'baseline': {}, 'angle_bins_48': {'angle_quantization_bins': 48}, 'downsample_2': {'downsample_costmap': True, 'downsampling_factor': 2}, 'lighter_smoother': {'smoother': {'max_iterations': 200, 'do_refinement': True, 'refinement_num': 1}}, 'bounded_search': {'max_iterations': 200000, 'max_on_approach_iterations': 500, 'max_planning_time': 1.0}}
DELTA_MAX_PATCH_GAP_CELLS = 64
DELTA_MAX_PATCH_EXPANSION = 2.5

def _grid_digest(grid: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(grid, dtype=np.int8).tobytes()).hexdigest()

def delta_patch_rectangles(previous_grid: np.ndarray, expected_grid: np.ndarray) -> List[Tuple[int, int, int, int]]:
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
        raise ValueError('delta grids must have the same shape')
    changed = previous != expected
    if not np.any(changed):
        return []
    rectangles: List[Tuple[int, int, int, int]] = []
    try:
        from scipy import ndimage
    except ImportError:
        ndimage = None

    def rect_area(rectangle: Tuple[int, int, int, int]) -> int:
        return rectangle[2] * rectangle[3]

    def merge_local_rectangles(candidates: Sequence[Tuple[int, int, int, int]]) -> List[Tuple[int, int, int, int]]:
        """Coalesce nearby components without spanning unrelated windows."""
        pending = sorted(candidates, key=lambda item: (item[1], item[0]))
        merged: List[Tuple[int, int, int, int]] = []
        for candidate in pending:
            absorbed = False
            for (index, current) in enumerate(merged):
                (current_x, current_y, current_w, current_h) = current
                (candidate_x, candidate_y, candidate_w, candidate_h) = candidate
                gap_x = max(0, current_x - (candidate_x + candidate_w), candidate_x - (current_x + current_w))
                gap_y = max(0, current_y - (candidate_y + candidate_h), candidate_y - (current_y + current_h))
                union_x = min(current_x, candidate_x)
                union_y = min(current_y, candidate_y)
                union_right = max(current_x + current_w, candidate_x + candidate_w)
                union_bottom = max(current_y + current_h, candidate_y + candidate_h)
                union = (union_x, union_y, union_right - union_x, union_bottom - union_y)
                if max(gap_x, gap_y) <= DELTA_MAX_PATCH_GAP_CELLS and rect_area(union) <= DELTA_MAX_PATCH_EXPANSION * (rect_area(current) + rect_area(candidate)):
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
            (labels, component_count) = ndimage.label(region, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8))
            for component_slice in ndimage.find_objects(labels)[:int(component_count)]:
                if component_slice is None:
                    continue
                (row_slice, column_slice) = component_slice
                region_rectangles.append((int(column_slice.start), int(row_slice.start), int(column_slice.stop - column_slice.start), int(row_slice.stop - row_slice.start)))
        else:
            remaining = {tuple((int(value) for value in item)) for item in np.argwhere(region)}
            while remaining:
                seed = remaining.pop()
                stack = [seed]
                min_row = max_row = seed[0]
                min_col = max_col = seed[1]
                while stack:
                    (row, column) = stack.pop()
                    (min_row, max_row) = (min(min_row, row), max(max_row, row))
                    (min_col, max_col) = (min(min_col, column), max(max_col, column))
                    for neighbour in ((row - 1, column), (row + 1, column), (row, column - 1), (row, column + 1)):
                        if neighbour in remaining:
                            remaining.remove(neighbour)
                            stack.append(neighbour)
                region_rectangles.append((min_col, min_row, max_col - min_col + 1, max_row - min_row + 1))
        rectangles.extend(merge_local_rectangles(region_rectangles))
    return sorted(rectangles, key=lambda item: (item[1], item[0], item[3], item[2]))

def apply_delta_rectangles(previous_grid: np.ndarray, expected_grid: np.ndarray, rectangles: Sequence[Tuple[int, int, int, int]]) -> np.ndarray:
    """Apply map-coordinate rectangles; used by runtime verification and tests."""
    previous = np.asarray(previous_grid, dtype=np.int8)
    expected = np.asarray(expected_grid, dtype=np.int8)
    if previous.shape != expected.shape:
        raise ValueError('delta grids must have the same shape')
    applied = previous.copy()
    for (x, y, width, height) in rectangles:
        if x < 0 or y < 0 or width <= 0 or (height <= 0):
            raise ValueError('invalid delta rectangle')
        if x + width > expected.shape[1] or y + height > expected.shape[0]:
            raise ValueError('delta rectangle is outside the map')
        applied[y:y + height, x:x + width] = expected[y:y + height, x:x + width]
    return applied

class SmacSession:
    """One static Nav2 planner stack reused within a smoke map."""

    def __init__(self, ctx: MapContext, output: Path, *, map_yaml: Optional[Path]=None, log_tag: Optional[str]=None, local_mask_updates: bool=False, optimization_profile: str='v6_compatible', smac_parameter_profile: str='baseline', optimization_stage: str='step3_delta_map', enable_mask_reuse_noop: bool=False, planner_parameter_overrides: Optional[Mapping[str, Any]]=None, costmap_ack_timeout_s: float=0.6):
        from .planner_benchmark.config import load_yaml, stack_parameters
        from .planner_benchmark.runner import BenchmarkStack, ComputePathClient
        import rclpy
        from map_msgs.msg import OccupancyGridUpdate
        from nav2_msgs.srv import ClearEntireCostmap, GetCostmap
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from rclpy.context import Context
        self.rclpy = rclpy
        self.ctx = ctx
        if optimization_profile not in {'v6_compatible', 'v7_candidate'}:
            raise ValueError(f'unsupported optimization profile: {optimization_profile}')
        if smac_parameter_profile not in SMAC_PARAMETER_PROFILES:
            raise ValueError(f'unsupported Smac parameter profile: {smac_parameter_profile}')
        if optimization_stage not in {'baseline', 'step1_skip_simplification', 'step2_light_reset', 'step3_delta_map'}:
            raise ValueError(f'unsupported optimization stage: {optimization_stage}')
        if optimization_profile == 'v6_compatible':
            smac_parameter_profile = 'baseline'
            optimization_stage = 'baseline'
        self.optimization_profile = optimization_profile
        self.optimization_stage = optimization_stage
        self.smac_parameter_profile = smac_parameter_profile
        self.local_map_update_strategy = 'delta' if optimization_stage == 'step3_delta_map' else 'v6_full'
        self.OccupancyGridUpdate = OccupancyGridUpdate
        self.OccupancyGrid = OccupancyGrid
        self.ClearEntireCostmap = ClearEntireCostmap
        self.GetCostmap = GetCostmap
        self._map_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map_update_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
        self.context = Context()
        rclpy.init(context=self.context)
        config_path = _strict_smac_config_path()
        planner_config = load_yaml(config_path)
        if isinstance(planner_config.get('GridBased'), dict):
            planner_config['GridBased']['smooth_path'] = True
            overrides = SMAC_PARAMETER_PROFILES[smac_parameter_profile]
            for (key, value) in overrides.items():
                if key == 'smoother':
                    planner_config['GridBased'].setdefault('smoother', {}).update(value)
                else:
                    planner_config['GridBased'][key] = value
            for (key, value) in dict(planner_parameter_overrides or {}).items():
                if key == 'smoother' and isinstance(value, Mapping):
                    planner_config['GridBased'].setdefault('smoother', {}).update(dict(value))
                else:
                    planner_config['GridBased'][str(key)] = value
        protocol = {'resolution': 0.05, 'width_cells': ctx.hospital_map.width, 'height_cells': ctx.hospital_map.height, 'origin': list(ctx.hospital_map.origin), 'footprint': FOOTPRINT, 'variants': {'strict_forward': {'allow_unknown': False, 'inflation_radius': 0.55, 'cost_scaling_factor': 3.0}}, 'static_layer_subscribe_to_updates': bool(local_mask_updates), 'costmap_update_frequency': 100.0 if local_mask_updates and optimization_stage == 'step3_delta_map' else 20.0 if local_mask_updates else 1.0}
        params = stack_parameters(protocol=protocol, planner_config=planner_config)
        logs = output / 'logs'
        logs.mkdir(parents=True, exist_ok=True)
        params_file = logs / f'smac_strict_{ctx.map_id}.yaml'
        params_file.write_text(yaml.safe_dump(params, sort_keys=False), encoding='utf-8')
        self.params_file = params_file
        self.smac_config_hash = hashlib.sha256(params_file.read_bytes()).hexdigest()
        tag = re.sub('[^A-Za-z0-9_.-]+', '_', log_tag or ctx.map_id)
        node_tag = re.sub('[^A-Za-z0-9_]+', '_', tag)
        resolved_map_yaml = map_yaml or ctx.map_yaml
        if resolved_map_yaml is None:
            raise ValueError('Explicit map_yaml is required; no bundled map registry')
        self.stack = BenchmarkStack(map_yaml=resolved_map_yaml, params_file=params_file, log_file=logs / f'smac_strict_{tag}.log')
        self.client_type = ComputePathClient
        self.client_node_name = f'planner_benchmark_client_{node_tag}_{os.getpid()}'
        self.client = None
        self.planner_pid = 0
        self.stack_pids: List[int] = []
        self.stack_startup_time_ms = 0.0
        self.stack_shutdown_time_ms = 0.0
        self.session_start_count = 0
        self.session_close_count = 0
        self.session_restart_count = 0
        self.restart_reasons: List[str] = []
        self.current_query_id = ''
        self.supports_local_mask = bool(protocol['static_layer_subscribe_to_updates'])
        self._local_update_publisher = None
        self._local_map_publisher = None
        self._clear_costmap_client = None
        self._get_costmap_client = None
        self._local_mask_info: Dict[str, Any] = {}
        self._current_allowed_mask: Optional[np.ndarray] = None
        self._current_grid: Optional[np.ndarray] = None
        self._costmap_state_trusted = False
        self._force_full_next_update = False
        self._action_in_progress = False
        self.enable_mask_reuse_noop = bool(enable_mask_reuse_noop)
        self._last_update_had_fallback = False
        self._last_publish_timing: Dict[str, float] = {}
        self._last_delta_publish_timing: Dict[str, float] = {}
        self.costmap_ack_timeout_s = max(0.05, float(costmap_ack_timeout_s))
        self.roi_max_payload_bytes = 128000
        self.roi_publish_pacing_s = 0.001
        self._last_server_update_time_ns = -1
        self._costmap_ack_sequence = 0

    def start(self) -> None:
        started_ns = time.monotonic_ns()
        self.stack.start(timeout=90.0)
        (self.planner_pid, self.stack_pids, process_error) = self.stack.pids()
        if process_error:
            self.stack.stop()
            raise RuntimeError(process_error)
        self.client = self.client_type(timeout=7.0, node_name=self.client_node_name, context=self.context)
        if self.supports_local_mask:
            self._local_update_publisher = self.client.node.create_publisher(self.OccupancyGridUpdate, '/map_updates', self._map_update_qos)
            self._local_map_publisher = self.client.node.create_publisher(self.OccupancyGrid, '/map', self._map_qos)
            self._clear_costmap_client = self.client.node.create_client(self.ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
            self._get_costmap_client = self.client.node.create_client(self.GetCostmap, '/global_costmap/get_costmap')
        self.stack_startup_time_ms = (time.monotonic_ns() - started_ns) / 1000000.0
        self.session_start_count += 1
        if self.supports_local_mask and self.local_map_update_strategy in {'delta', 'roi_ack'}:
            lethal = np.zeros((self.ctx.hospital_map.height, self.ctx.hospital_map.width), dtype=bool)
            self.update_local_mask(lethal, force_full=True, initialization=True)
            self._local_mask_info = {}
            self.stack_startup_time_ms = (time.monotonic_ns() - started_ns) / 1000000.0
        elif self.supports_local_mask:
            base_mask = np.asarray(self.ctx.hospital_map.occupancy == 0, dtype=bool)
            (self._current_allowed_mask, self._current_grid) = self._grid_for_mask(base_mask)
            self._costmap_state_trusted = True

    def reset_query_state(self, query_id: str, *, restore_base_map: bool=True) -> Dict[str, Any]:
        """Start one serial query without carrying request diagnostics forward.

        Nav2 creates fresh action state for every ComputePath request.  The
        shared context deliberately retains only the immutable base map and
        planner processes; the next L3 request replaces the complete local
        mask before planning.
        """
        if self.client is None or not self.context.ok():
            raise RuntimeError('Smac map session is not active')
        reset_started_ns = time.monotonic_ns()
        fallback = False
        fallback_reason = ''
        if not restore_base_map and self._action_in_progress:
            fallback = True
            fallback_reason = 'previous_action_not_finished'
        elif not restore_base_map and self.supports_local_mask and (not self._costmap_state_trusted):
            fallback = True
            fallback_reason = 'costmap_state_untrusted'
        if fallback:
            restore_base_map = True
            self._force_full_next_update = True
        self.current_query_id = str(query_id)
        self._local_mask_info = {}
        if restore_base_map and self.supports_local_mask:
            base_mask = np.asarray(self.ctx.hospital_map.occupancy == 0, dtype=bool)
            reset_map_info = self.update_local_mask(base_mask, force_full=True)
            self._local_mask_info = {}
        else:
            reset_map_info = {}
        return {'query_session_reused': self.session_start_count == 1, 'session_start_count': self.session_start_count, 'session_close_count': self.session_close_count, 'session_restart_count': self.session_restart_count, 'restart_reason': ';'.join(self.restart_reasons), 'query_session_reset_mode': 'v6_full' if restore_base_map else 'light', 'session_reset_fallback': fallback, 'session_reset_fallback_reason': fallback_reason, 'query_session_reset_ms': (time.monotonic_ns() - reset_started_ns) / 1000000.0, 'query_session_reset_local_map_generation_ms': reset_map_info.get('local_map_generation_ms', 0.0), 'query_session_reset_local_map_serialization_ms': reset_map_info.get('local_map_serialization_ms', 0.0), 'query_session_reset_local_map_publication_ms': reset_map_info.get('local_map_publication_ms', 0.0), 'query_session_reset_costmap_clear_ms': reset_map_info.get('local_costmap_clear_ms', 0.0), 'query_session_reset_costmap_settle_ms': reset_map_info.get('costmap_settle_ms', 0.0)}

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
            self._get_costmap_client = None
            self.client.close()
            self.client = None
        self.stack.stop()
        time.sleep(0.5)
        if self.context.ok():
            self.context.shutdown()
        self.stack_shutdown_time_ms = (time.monotonic_ns() - started_ns) / 1000000.0
        self.session_close_count += 1
        self.current_query_id = ''
        self._costmap_state_trusted = False
        self._current_allowed_mask = None
        self._current_grid = None

    def _grid_for_mask(self, allowed_mask: Any) -> Tuple[np.ndarray, np.ndarray]:
        mask = np.asarray(allowed_mask, dtype=bool)
        if mask.shape != (self.ctx.hospital_map.height, self.ctx.hospital_map.width):
            raise ValueError('local mask shape does not match the map')
        values = np.where(mask, np.asarray(self.ctx.hospital_map.occupancy, dtype=np.int16), 100)
        values = np.where(values < 0, -1, np.clip(values, 0, 100)).astype(np.int8)
        return (mask, np.flipud(values))

    def _clear_global_costmap(self) -> float:
        clear_started_ns = time.monotonic_ns()
        if self._clear_costmap_client is not None and self._clear_costmap_client.wait_for_service(timeout_sec=0.5):
            clear_future = self._clear_costmap_client.call_async(self.ClearEntireCostmap.Request())
            deadline = time.monotonic() + 1.0
            while not clear_future.done() and time.monotonic() < deadline:
                self.client.executor.spin_once(timeout_sec=0.01)
            if not clear_future.done():
                raise RuntimeError('global costmap clear service timed out')
        return (time.monotonic_ns() - clear_started_ns) / 1000000.0

    def _server_costmap_snapshot(self, deadline: float) -> Tuple[np.ndarray, int]:
        """Read the planner server's master costmap through its service."""
        if self._get_costmap_client is None or self.client is None:
            raise RuntimeError('global costmap readback service is unavailable')
        remaining = max(0.0, deadline - time.monotonic())
        if not self._get_costmap_client.wait_for_service(timeout_sec=min(0.2, remaining)):
            raise RuntimeError('global costmap readback service unavailable')
        future = self._get_costmap_client.call_async(self.GetCostmap.Request())
        while not future.done() and time.monotonic() < deadline:
            self.client.executor.spin_once(timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))
        if not future.done():
            raise RuntimeError('global costmap readback timed out')
        response = future.result()
        message = getattr(response, 'map', None)
        metadata = getattr(message, 'metadata', None)
        width = int(getattr(metadata, 'size_x', 0))
        height = int(getattr(metadata, 'size_y', 0))
        if width != int(self.ctx.hospital_map.width) or height != int(self.ctx.hospital_map.height):
            raise RuntimeError(f'global costmap readback shape mismatch: {width}x{height}')
        data = np.frombuffer(bytes(getattr(message, 'data', b'')), dtype=np.uint8)
        if data.size != width * height:
            raise RuntimeError('global costmap readback data length mismatch')
        update_time = getattr(metadata, 'update_time', None)
        update_time_ns = int(getattr(update_time, 'sec', 0)) * 1000000000 + int(getattr(update_time, 'nanosec', 0))
        return (data.reshape((height, width)), update_time_ns)

    def _wait_for_costmap_ack(self, expected: np.ndarray, changed: np.ndarray, *, timeout_s: Optional[float]=None) -> Dict[str, Any]:
        """Wait until Nav2's master costmap reflects every changed static cell.

        The server costmap includes inflation, so raw byte equality with the
        static grid is intentionally not used.  A closed corridor cell must be
        lethal (254), while an opened raw-free cell must be non-lethal.  This
        checks the complete changed set returned by the server, rather than a
        client-side predicted grid hash.
        """
        ack_started_ns = time.monotonic_ns()
        deadline = time.monotonic() + float(timeout_s or self.costmap_ack_timeout_s)
        expected_grid = np.asarray(expected, dtype=np.int8)
        changed_mask = np.asarray(changed, dtype=bool)
        checked = int(np.count_nonzero(changed_mask))
        if checked == 0:
            return {'costmap_update_acknowledged': True, 'costmap_ack_status': 'unchanged_verified_state', 'costmap_ack_wait_ms': 0.0, 'costmap_ack_attempts': 0, 'costmap_ack_checked_cells': 0, 'costmap_ack_mismatch_cells': 0, 'server_costmap_update_time_ns': self._last_server_update_time_ns, 'server_costmap_content_hash': ''}
        if self.client is not None:
            self.client.executor.spin_once(timeout_sec=min(0.025, max(0.0, deadline - time.monotonic())))
        attempts = 0
        last_mismatch = checked
        last_update_time_ns = -1
        last_hash = ''
        last_error = ''
        repair_count = 0
        repair_messages = 0
        repair_cells = 0
        repair_serialization_ms = 0.0
        repair_publication_ms = 0.0
        while time.monotonic() < deadline:
            attempts += 1
            try:
                (server, update_time_ns) = self._server_costmap_snapshot(deadline)
            except RuntimeError as exc:
                last_error = str(exc)
                continue
            server_values = np.ascontiguousarray(server[changed_mask], dtype=np.uint8)
            expected_values = expected_grid[changed_mask]
            expected_closed = expected_values == 100
            matches = np.where(expected_closed, server_values == 254, server_values < 254)
            last_mismatch = int(matches.size - np.count_nonzero(matches))
            last_update_time_ns = int(update_time_ns)
            last_hash = hashlib.sha256(server_values.tobytes()).hexdigest()
            if last_mismatch == 0:
                self._last_server_update_time_ns = last_update_time_ns
                self._costmap_ack_sequence += 1
                return {'costmap_update_acknowledged': True, 'costmap_ack_status': 'server_content_verified', 'costmap_ack_wait_ms': (time.monotonic_ns() - ack_started_ns) / 1000000.0, 'costmap_ack_attempts': attempts, 'costmap_ack_checked_cells': checked, 'costmap_ack_mismatch_cells': 0, 'costmap_ack_sequence': self._costmap_ack_sequence, 'server_costmap_update_time_ns': last_update_time_ns, 'server_costmap_content_hash': last_hash, 'costmap_ack_repair_count': repair_count, 'costmap_ack_repair_messages': repair_messages, 'costmap_ack_repair_cells': repair_cells, 'costmap_ack_repair_serialization_ms': repair_serialization_ms, 'costmap_ack_repair_publication_ms': repair_publication_ms}
            elapsed_s = (time.monotonic_ns() - ack_started_ns) / 1000000000.0
            repair_limit = max(5000, int(math.ceil(checked * 0.05)))
            if repair_count < 2 and elapsed_s >= 0.05 and (last_mismatch <= repair_limit):
                mismatch_mask = np.zeros(changed_mask.shape, dtype=bool)
                changed_indices = np.flatnonzero(changed_mask)
                mismatch_mask.flat[changed_indices[~matches]] = True
                (_applied, repair_info) = self._publish_dirty_roi(expected_grid, mismatch_mask)
                repair_count += 1
                repair_messages += int(repair_info.get('roi_message_count', 0))
                repair_cells += int(repair_info.get('roi_published_cells', 0))
                repair_serialization_ms += float(repair_info.get('local_map_serialization_ms', 0.0))
                repair_publication_ms += float(repair_info.get('local_map_publication_ms', 0.0))
                if self.client is not None:
                    self.client.executor.spin_once(timeout_sec=min(0.025, max(0.0, deadline - time.monotonic())))
            if self.client is not None:
                self.client.executor.spin_once(timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))
        return {'costmap_update_acknowledged': False, 'costmap_ack_status': 'server_content_mismatch' if not last_error else 'readback_error', 'costmap_ack_wait_ms': (time.monotonic_ns() - ack_started_ns) / 1000000.0, 'costmap_ack_attempts': attempts, 'costmap_ack_checked_cells': checked, 'costmap_ack_mismatch_cells': last_mismatch, 'costmap_ack_sequence': self._costmap_ack_sequence, 'server_costmap_update_time_ns': last_update_time_ns, 'server_costmap_content_hash': last_hash, 'costmap_ack_error': last_error, 'costmap_ack_repair_count': repair_count, 'costmap_ack_repair_messages': repair_messages, 'costmap_ack_repair_cells': repair_cells, 'costmap_ack_repair_serialization_ms': repair_serialization_ms, 'costmap_ack_repair_publication_ms': repair_publication_ms}

    def _publish_dirty_roi(self, expected: np.ndarray, changed: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Publish one merged ROI containing both corridor closes and opens."""
        if self._current_grid is None:
            raise RuntimeError('ROI update has no initialized grid')
        changed_cells = np.argwhere(np.asarray(changed, dtype=bool))
        if changed_cells.size == 0:
            return (self._current_grid.copy(), {'local_map_serialization_ms': 0.0, 'local_map_publication_ms': 0.0, 'costmap_settle_ms': 0.0, 'roi_bbox': [0, 0, 0, 0], 'roi_changed_cells': 0})
        min_y = int(changed_cells[:, 0].min())
        max_y = int(changed_cells[:, 0].max()) + 1
        min_x = int(changed_cells[:, 1].min())
        max_x = int(changed_cells[:, 1].max()) + 1
        width = max_x - min_x
        max_payload_bytes = max(1024, int(getattr(self, 'roi_max_payload_bytes', 128000)))
        rows_per_tile = max(1, max_payload_bytes // max(1, width))
        serialization_ms = 0.0
        publication_ms = 0.0
        message_count = 0
        for tile_y in range(min_y, max_y, rows_per_tile):
            tile_y1 = min(max_y, tile_y + rows_per_tile)
            serialize_started_ns = time.monotonic_ns()
            patch = np.ascontiguousarray(expected[tile_y:tile_y1, min_x:max_x], dtype=np.int8)
            message = self.OccupancyGridUpdate()
            message.header.frame_id = 'map'
            message.header.stamp = self.client.node.get_clock().now().to_msg()
            message.x = min_x
            message.y = tile_y
            message.width = width
            message.height = tile_y1 - tile_y
            message.data = array('b', patch.tobytes())
            serialization_ms += (time.monotonic_ns() - serialize_started_ns) / 1000000.0
            publish_started_ns = time.monotonic_ns()
            self._local_update_publisher.publish(message)
            publication_ms += (time.monotonic_ns() - publish_started_ns) / 1000000.0
            message_count += 1
            pacing_s = max(0.0, float(getattr(self, 'roi_publish_pacing_s', 0.001)))
            if pacing_s > 0.0:
                executor = getattr(self.client, 'executor', None)
                if executor is not None:
                    executor.spin_once(timeout_sec=pacing_s)
                else:
                    time.sleep(pacing_s)
        applied = self._current_grid.copy()
        applied[min_y:max_y, min_x:max_x] = expected[min_y:max_y, min_x:max_x]
        return (applied, {'local_map_serialization_ms': serialization_ms, 'local_map_publication_ms': publication_ms, 'costmap_settle_ms': 0.0, 'roi_bbox': [min_x, min_y, max_x - min_x, max_y - min_y], 'roi_changed_cells': int(changed_cells.shape[0]), 'roi_published_cells': int((max_y - min_y) * width), 'roi_message_count': message_count, 'roi_max_message_bytes': int(min(max_payload_bytes, max(1, width) * rows_per_tile)), 'roi_publish_pacing_ms': float(getattr(self, 'roi_publish_pacing_s', 0.001)) * 1000.0})

    def _publish_full_grid(self, values: np.ndarray, *, clear_costmap: bool=True) -> float:
        serialize_started_ns = time.monotonic_ns()
        message = self.OccupancyGridUpdate()
        message.header.frame_id = 'map'
        message.x = 0
        message.y = 0
        message.width = int(self.ctx.hospital_map.width)
        message.height = int(self.ctx.hospital_map.height)
        serialized_data = array('b', np.ascontiguousarray(values, dtype=np.int8).tobytes())
        message.data = serialized_data
        full_map = self.OccupancyGrid()
        full_map.header.frame_id = 'map'
        full_map.header.stamp = self.client.node.get_clock().now().to_msg()
        full_map.info.resolution = float(self.ctx.hospital_map.resolution)
        full_map.info.width = int(self.ctx.hospital_map.width)
        full_map.info.height = int(self.ctx.hospital_map.height)
        full_map.info.origin.position.x = float(self.ctx.hospital_map.origin[0])
        full_map.info.origin.position.y = float(self.ctx.hospital_map.origin[1])
        full_map.info.origin.orientation.w = 1.0
        full_map.data = serialized_data
        serialization_ms = (time.monotonic_ns() - serialize_started_ns) / 1000000.0
        clear_ms = self._clear_global_costmap() if clear_costmap else 0.0
        publish_started_ns = time.monotonic_ns()
        self._local_map_publisher.publish(full_map)
        self._local_update_publisher.publish(message)
        publication_ms = (time.monotonic_ns() - publish_started_ns) / 1000000.0
        settle_started_ns = time.monotonic_ns()
        settle_cycles = int(getattr(self, 'full_grid_settle_cycles', 3 if self.local_map_update_strategy == 'v6_full' else 2))
        for _ in range(max(0, settle_cycles)):
            self.client.executor.spin_once(timeout_sec=0.01)
        self._last_publish_timing = {'local_map_serialization_ms': serialization_ms, 'local_map_publication_ms': publication_ms, 'costmap_settle_ms': (time.monotonic_ns() - settle_started_ns) / 1000000.0}
        return clear_ms

    def _publish_delta_updates(self, expected: np.ndarray, rectangles: Sequence[Tuple[int, int, int, int]]) -> np.ndarray:
        if self._current_grid is None:
            raise RuntimeError('delta update has no initialized grid')
        serialize_ms = 0.0
        publication_ms = 0.0
        settle_ms = 0.0
        applied = apply_delta_rectangles(self._current_grid, expected, rectangles)
        for (x, y, width, height) in rectangles:
            serialize_started_ns = time.monotonic_ns()
            patch = np.ascontiguousarray(expected[y:y + height, x:x + width], dtype=np.int8)
            message = self.OccupancyGridUpdate()
            message.header.frame_id = 'map'
            message.x = int(x)
            message.y = int(y)
            message.width = int(width)
            message.height = int(height)
            message.data = array('b', patch.tobytes())
            serialize_ms += (time.monotonic_ns() - serialize_started_ns) / 1000000.0
            publish_started_ns = time.monotonic_ns()
            self._local_update_publisher.publish(message)
            publication_ms += (time.monotonic_ns() - publish_started_ns) / 1000000.0
            settle_started_ns = time.monotonic_ns()
            for _ in range(6):
                self.client.executor.spin_once(timeout_sec=0.005)
            settle_ms += (time.monotonic_ns() - settle_started_ns) / 1000000.0
        self._last_delta_publish_timing = {'local_map_serialization_ms': serialize_ms, 'local_map_publication_ms': publication_ms, 'costmap_settle_ms': settle_ms}
        return applied

    def update_local_mask(self, allowed_mask: Any, *, window_start_index: int=-1, window_end_index: int=-1, window_path_length_m: Optional[float]=None, force_full: bool=False, initialization: bool=False, fallback_reason: str='') -> Dict[str, Any]:
        """Publish one full-size static-layer update without restarting Nav2.

        ``OccupancyGridUpdate`` uses map coordinates whose row zero is the
        bottom of the map.  HospitalMap stores image rows top-to-bottom, so
        the mask is flipped before publishing.  Every cell outside the local
        window is lethal, making it impossible for Smac to route around the
        requested window while retaining the original static map geometry.
        """
        if not self.supports_local_mask or self._local_update_publisher is None or self._local_map_publisher is None or (self.client is None):
            raise RuntimeError('Smac session does not support local costmap updates')
        started_ns = time.monotonic_ns()
        generation_started_ns = time.monotonic_ns()
        (mask, values) = self._grid_for_mask(allowed_mask)
        generation_ms = (time.monotonic_ns() - generation_started_ns) / 1000000.0
        hash_started_ns = time.monotonic_ns()
        expected_hash = _grid_digest(values)
        previous_hash = _grid_digest(self._current_grid) if self._current_grid is not None else ''
        grid_hash_ms = (time.monotonic_ns() - hash_started_ns) / 1000000.0
        can_reuse = bool(getattr(self, 'enable_mask_reuse_noop', False) and (not force_full) and (not initialization) and (not getattr(self, '_force_full_next_update', False)) and getattr(self, '_costmap_state_trusted', False) and (self._current_grid is not None) and (not getattr(self, '_last_update_had_fallback', False)) and (previous_hash == expected_hash))
        if can_reuse:
            elapsed_ms = (time.monotonic_ns() - started_ns) / 1000000.0
            allowed_cells = int(np.count_nonzero(mask))
            self._local_mask_info = {'local_mask_hash': expected_hash, 'local_map_width_cells': int(mask.shape[1]), 'local_map_height_cells': int(mask.shape[0]), 'local_window_allowed_cells': allowed_cells, 'local_window_start_index': int(window_start_index), 'local_window_end_index': int(window_end_index), 'local_window_path_length_m': window_path_length_m, 'local_map_update_ms': elapsed_ms, 'local_map_generation_ms': generation_ms, 'local_map_serialization_ms': 0.0, 'local_map_publication_ms': 0.0, 'local_costmap_clear_ms': 0.0, 'costmap_settle_ms': 0.0, 'local_map_hash_ms': grid_hash_ms, 'local_map_update_mode': 'reuse_noop', 'local_map_update_messages': 0, 'local_map_update_cells': 0, 'local_map_update_bytes': 0, 'local_map_update_fallback': False, 'local_map_update_fallback_reason': '', 'local_map_update_skipped': True, 'previous_mask_hash': previous_hash, 'expected_mask_hash': expected_hash, 'applied_mask_hash': previous_hash, 'costmap_update_acknowledged': True if self.local_map_update_strategy == 'roi_ack' else 'not_available', 'costmap_ack_status': 'reused_server_verified_state' if self.local_map_update_strategy == 'roi_ack' else 'not_requested', 'costmap_ack_wait_ms': 0.0, 'costmap_ack_attempts': 0, 'costmap_ack_checked_cells': 0, 'costmap_ack_mismatch_cells': 0, 'costmap_ack_sequence': getattr(self, '_costmap_ack_sequence', 0), 'server_costmap_update_time_ns': getattr(self, '_last_server_update_time_ns', -1), 'server_costmap_content_hash': ''}
            return dict(self._local_mask_info)
        mode = 'v6_full'
        messages = 1
        cells = int(values.size)
        update_bytes = int(values.nbytes)
        clear_ms = 0.0
        publish_timing = {'local_map_serialization_ms': 0.0, 'local_map_publication_ms': 0.0, 'costmap_settle_ms': 0.0}
        ack_info: Dict[str, Any] = {'costmap_update_acknowledged': 'not_available', 'costmap_ack_status': 'not_requested', 'costmap_ack_wait_ms': 0.0, 'costmap_ack_attempts': 0, 'costmap_ack_checked_cells': 0, 'costmap_ack_mismatch_cells': 0, 'server_costmap_update_time_ns': 'not_available', 'server_costmap_content_hash': ''}
        fallback = False
        actual_fallback_reason = fallback_reason
        changed = np.ones(values.shape, dtype=bool) if self._current_grid is None else np.asarray(self._current_grid != values, dtype=bool)
        use_roi_ack = self.local_map_update_strategy == 'roi_ack' and (not force_full) and (not self._force_full_next_update) and self._costmap_state_trusted and (self._current_grid is not None)
        use_delta = self.local_map_update_strategy == 'delta' and (not force_full) and (not self._force_full_next_update) and self._costmap_state_trusted and (self._current_grid is not None)
        if use_roi_ack:
            mode = 'roi_ack'
            (applied, publish_timing) = self._publish_dirty_roi(values, changed)
            messages = int(publish_timing.get('roi_message_count', 0))
            cells = int(publish_timing.get('roi_published_cells', 0))
            update_bytes = cells
            ack_info = self._wait_for_costmap_ack(values, changed)
            messages += int(ack_info.get('costmap_ack_repair_messages', 0))
            cells += int(ack_info.get('costmap_ack_repair_cells', 0))
            update_bytes += int(ack_info.get('costmap_ack_repair_cells', 0))
            publish_timing['local_map_serialization_ms'] = float(publish_timing.get('local_map_serialization_ms', 0.0)) + float(ack_info.get('costmap_ack_repair_serialization_ms', 0.0))
            publish_timing['local_map_publication_ms'] = float(publish_timing.get('local_map_publication_ms', 0.0)) + float(ack_info.get('costmap_ack_repair_publication_ms', 0.0))
            if not bool(ack_info.get('costmap_update_acknowledged')):
                fallback = True
                actual_fallback_reason = str(ack_info.get('costmap_ack_status') or 'roi_ack_failed')
                mode = 'roi_ack_full_fallback'
                clear_ms = self._publish_full_grid(values)
                full_timing = dict(getattr(self, '_last_publish_timing', {}) or {})
                publish_timing = {key: float(publish_timing.get(key, 0.0)) + float(full_timing.get(key, 0.0)) for key in ('local_map_serialization_ms', 'local_map_publication_ms', 'costmap_settle_ms')} | {'roi_bbox': publish_timing.get('roi_bbox', [0, 0, 0, 0]), 'roi_changed_cells': publish_timing.get('roi_changed_cells', 0), 'roi_published_cells': publish_timing.get('roi_published_cells', 0), 'roi_message_count': publish_timing.get('roi_message_count', 0), 'roi_max_message_bytes': publish_timing.get('roi_max_message_bytes', 0), 'roi_publish_pacing_ms': publish_timing.get('roi_publish_pacing_ms', 0.0)}
                applied = values.copy()
                messages += 2
                cells += int(values.size) * 2
                update_bytes += int(values.nbytes) * 2
                full_ack = self._wait_for_costmap_ack(values, np.ones(values.shape, dtype=bool))
                ack_info = {**full_ack, 'roi_ack_initial_status': ack_info.get('costmap_ack_status', ''), 'roi_ack_initial_mismatch_cells': ack_info.get('costmap_ack_mismatch_cells', 0), 'roi_ack_initial_attempts': ack_info.get('costmap_ack_attempts', 0), 'roi_ack_initial_wait_ms': ack_info.get('costmap_ack_wait_ms', 0.0), 'roi_ack_initial_error': ack_info.get('costmap_ack_error', '')}
                if not bool(full_ack.get('costmap_update_acknowledged')):
                    self._costmap_state_trusted = False
                    self._force_full_next_update = True
                    self._last_update_had_fallback = True
                    raise RuntimeError('costmap full fallback ACK failed: ' + str(full_ack.get('costmap_ack_status') or full_ack.get('costmap_ack_error') or 'unknown'))
        elif use_delta:
            mode = 'delta'
            try:
                rectangles = delta_patch_rectangles(self._current_grid, values)
                messages = len(rectangles)
                cells = sum((width * height for (_x, _y, width, height) in rectangles))
                update_bytes = cells
                applied = self._publish_delta_updates(values, rectangles)
                publish_timing = dict(getattr(self, '_last_delta_publish_timing', {}) or publish_timing)
                applied_hash = _grid_digest(applied)
                if applied_hash != expected_hash:
                    raise RuntimeError('delta applied-grid hash mismatch')
            except (OSError, RuntimeError, ValueError) as exc:
                fallback = True
                actual_fallback_reason = actual_fallback_reason or str(exc)
                mode = 'full_fallback'
                clear_ms = self._publish_full_grid(values)
                publish_timing = dict(getattr(self, '_last_publish_timing', {}) or publish_timing)
                applied = values.copy()
                messages += 2
                cells += int(values.size)
                update_bytes += int(values.nbytes)
        else:
            incremental_mode = self.local_map_update_strategy in {'delta', 'roi_ack'}
            mode = ('roi_ack_initial_full' if self.local_map_update_strategy == 'roi_ack' else 'delta_initial_full') if initialization else 'full_fallback' if incremental_mode else 'v6_full'
            fallback = bool(incremental_mode and (not initialization))
            if fallback and (not actual_fallback_reason):
                actual_fallback_reason = 'forced_full_update'
            clear_ms = self._publish_full_grid(values)
            publish_timing = dict(getattr(self, '_last_publish_timing', {}) or publish_timing)
            applied = values.copy()
            messages = 2
            cells = int(values.size) * 2
            update_bytes = int(values.nbytes) * 2
            if self.local_map_update_strategy == 'roi_ack':
                ack_info = self._wait_for_costmap_ack(values, changed)
                if not bool(ack_info.get('costmap_update_acknowledged')):
                    self._costmap_state_trusted = False
                    self._force_full_next_update = True
                    self._last_update_had_fallback = True
                    raise RuntimeError('costmap full update ACK failed: ' + str(ack_info.get('costmap_ack_status') or ack_info.get('costmap_ack_error') or 'unknown'))
        applied_hash = _grid_digest(applied)
        self._current_allowed_mask = mask.copy()
        self._current_grid = applied
        self._costmap_state_trusted = bool(applied_hash == expected_hash and (self.local_map_update_strategy != 'roi_ack' or ack_info.get('costmap_update_acknowledged') is True))
        self._force_full_next_update = False
        self._last_update_had_fallback = bool(fallback)
        elapsed_ms = (time.monotonic_ns() - started_ns) / 1000000.0
        self._local_mask_info = {'local_mask_hash': expected_hash, 'local_map_width_cells': int(mask.shape[1]), 'local_map_height_cells': int(mask.shape[0]), 'local_window_allowed_cells': int(np.count_nonzero(mask)), 'local_window_start_index': int(window_start_index), 'local_window_end_index': int(window_end_index), 'local_window_path_length_m': window_path_length_m, 'local_map_update_ms': elapsed_ms, 'local_map_generation_ms': generation_ms, 'local_map_serialization_ms': float(publish_timing.get('local_map_serialization_ms', 0.0)), 'local_map_publication_ms': float(publish_timing.get('local_map_publication_ms', 0.0)), 'local_costmap_clear_ms': clear_ms, 'costmap_settle_ms': float(publish_timing.get('costmap_settle_ms', 0.0)), 'local_map_update_mode': mode, 'local_map_update_messages': messages, 'local_map_update_cells': cells, 'local_map_update_bytes': update_bytes, 'local_map_update_fallback': fallback, 'local_map_update_fallback_reason': actual_fallback_reason, 'local_map_update_skipped': False, 'previous_mask_hash': previous_hash, 'expected_mask_hash': expected_hash, 'applied_mask_hash': applied_hash, 'local_map_hash_ms': grid_hash_ms, **publish_timing, **ack_info}
        return dict(self._local_mask_info)

    def _path_within_allowed_mask(self, points: Sequence[Mapping[str, Any]], allowed_mask: np.ndarray) -> bool:
        if not points:
            return False
        spacing = max(0.01, float(self.ctx.hospital_map.resolution) * 0.5)
        for (first, second) in zip(points, points[1:]):
            dx = float(second['x']) - float(first['x'])
            dy = float(second['y']) - float(first['y'])
            steps = max(1, int(math.ceil(math.hypot(dx, dy) / spacing)))
            for step in range(steps + 1):
                fraction = step / steps
                cell = self.ctx.hospital_map.world_to_cell(float(first['x']) + fraction * dx, float(first['y']) + fraction * dy)
                if cell is None or not bool(allowed_mask[cell]):
                    return False
        if len(points) == 1:
            cell = self.ctx.hospital_map.world_to_cell(float(points[0]['x']), float(points[0]['y']))
            return cell is not None and bool(allowed_mask[cell])
        return True

    def plan(self, query: Query, spec: BackendSpec, *, source: str='hybrid_astar', allowed_mask: Any=None, window_start_index: int=-1, window_end_index: int=-1, window_path_length_m: Optional[float]=None, force_full_update: bool=False, skip_path_mask_validation: bool=False) -> PlanResult:
        if self.client is None:
            return unavailable_plan(spec, source=source)
        local_update_ms = 0.0
        local_mask_info: Dict[str, Any] = {}
        if allowed_mask is not None:
            local_mask_info = self.update_local_mask(allowed_mask, window_start_index=window_start_index, window_end_index=window_end_index, window_path_length_m=window_path_length_m, force_full=bool(force_full_update))
            local_update_ms = float(local_mask_info.get('local_map_update_ms') or 0.0)
        action_phase_timings: List[Dict[str, float]] = []

        def call_action() -> Tuple[str, str, float, Any, List[Dict[str, Any]], Any, Optional[float]]:
            self._action_in_progress = True
            try:
                (status, result_code, wall_ms, measurement, raw_points, action_result) = self.client.plan(query, planner_pid=self.planner_pid, stack_pids=self.stack_pids, sample_interval_ms=5.0)
            finally:
                self._action_in_progress = False
            client_timing = dict(getattr(self.client, 'last_timing', {}) or {})
            annotation_started_ns = time.monotonic_ns()
            if result_code == 'CLIENT_TIMEOUT':
                self._costmap_state_trusted = False
                self._last_update_had_fallback = True
            points = _annotate_smac_points(raw_points or [], spec, source)
            client_timing['point_annotation_ms'] = (time.monotonic_ns() - annotation_started_ns) / 1000000.0
            action_phase_timings.append(client_timing)
            planning_ms = None
            duration = getattr(action_result, 'planning_time', None)
            if duration is not None:
                planning_ms = float(getattr(duration, 'sec', 0)) * 1000.0 + float(getattr(duration, 'nanosec', 0)) / 1000000.0
            return (status, result_code, wall_ms, measurement, points, action_result, planning_ms)
        action_results = [call_action()]
        (status, result_code, wall_ms, measurement, points, action_result, planning_ms) = action_results[-1]
        mask_check_started_ns = time.monotonic_ns()
        path_left_mask = bool(not skip_path_mask_validation and allowed_mask is not None and (result_code == 'SUCCEEDED') and points and (not self._path_within_allowed_mask(points, np.asarray(allowed_mask, dtype=bool))))
        path_within_mask_check_ms = (time.monotonic_ns() - mask_check_started_ns) / 1000000.0
        if path_left_mask and self.local_map_update_strategy == 'delta':
            first_update = dict(local_mask_info)
            fallback_update = self.update_local_mask(allowed_mask, window_start_index=window_start_index, window_end_index=window_end_index, window_path_length_m=window_path_length_m, force_full=True, fallback_reason='returned_path_left_allowed_mask')
            local_update_ms += float(fallback_update.get('local_map_update_ms') or 0.0)
            local_mask_info = {**fallback_update, 'local_map_update_ms': local_update_ms, 'local_map_update_messages': int(first_update.get('local_map_update_messages') or 0) + int(fallback_update.get('local_map_update_messages') or 0), 'local_map_update_cells': int(first_update.get('local_map_update_cells') or 0) + int(fallback_update.get('local_map_update_cells') or 0), 'local_map_update_bytes': int(first_update.get('local_map_update_bytes') or 0) + int(fallback_update.get('local_map_update_bytes') or 0)}
            action_results.append(call_action())
            (status, result_code, wall_ms, measurement, points, action_result, planning_ms) = action_results[-1]
            mask_check_started_ns = time.monotonic_ns()
            path_left_mask = bool(result_code == 'SUCCEEDED' and points and (not self._path_within_allowed_mask(points, np.asarray(allowed_mask, dtype=bool))))
            path_within_mask_check_ms += (time.monotonic_ns() - mask_check_started_ns) / 1000000.0
        if path_left_mask:
            result_code = 'L3_PATH_OUTSIDE_LOCAL_MASK'
            points = []
        total_wall_ms = sum((float(item[2]) for item in action_results))
        total_planning_ms = sum((float(item[6] or 0.0) for item in action_results))
        backend_attempts = [{'backend_action_index': index, 'action_status': item[0], 'action_result_code': item[1], 'wall_time_ms': item[2], 'planning_time_ms': item[6], 'path_mask_valid': not (index == 0 and len(action_results) > 1) and (not path_left_mask)} for (index, item) in enumerate(action_results)]
        diagnostics = {'action_status': status, 'action_result_code': result_code, 'backend_called': True, 'planning_time_ms': total_planning_ms, 'planner_cpu_total_ms': getattr(measurement, 'planner_cpu_total_ms', None), 'planner_rss_peak_bytes': getattr(measurement, 'planner_rss_peak_bytes', None), 'planner_pss_peak_bytes': getattr(measurement, 'planner_pss_peak_bytes', None), 'stack_rss_peak_bytes': getattr(measurement, 'stack_rss_peak_bytes', None), 'stack_pss_peak_bytes': getattr(measurement, 'stack_pss_peak_bytes', None), 'wall_time_ms': total_wall_ms, 'l3_action_wall_ms': total_wall_ms, 'l3_planning_time_ms': total_planning_ms, 'l3_process_overhead_ms': max(0.0, total_wall_ms - total_planning_ms), 'backend_call_count': len(action_results), 'backend_action_attempts': backend_attempts, 'returned_path_within_mask': not path_left_mask, 'path_within_mask_check_ms': path_within_mask_check_ms, 'session_path_mask_validation_skipped': bool(skip_path_mask_validation), 'ros_path_conversion_ms': sum((float(item.get('ros_path_conversion_ms', 0.0)) for item in action_phase_timings)), 'point_annotation_ms': sum((float(item.get('point_annotation_ms', 0.0)) for item in action_phase_timings)), 'action_server_wait_ms': sum((float(item.get('action_server_wait_ms', 0.0)) for item in action_phase_timings)), 'action_goal_send_wait_ms': sum((float(item.get('action_goal_send_wait_ms', 0.0)) for item in action_phase_timings)), 'action_result_wait_ms': sum((float(item.get('action_result_wait_ms', 0.0)) for item in action_phase_timings)), **local_mask_info}
        success = result_code == 'SUCCEEDED' and bool(points)
        return PlanResult(planner_success=success, points=points or None, failure_code='' if success else result_code, failure_detail='' if success else f'Nav2 action status={status}', planner_backend=spec.backend, backend_version=spec.version, source=source, diagnostics=diagnostics)

def _wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi

def _delta(a: float, b: float) -> float:
    return _wrap(float(a) - float(b))

def _annotate_smac_points(points: Sequence[Mapping[str, Any]], spec: BackendSpec, source: str) -> List[Dict[str, Any]]:
    annotated: List[Dict[str, Any]] = []
    for (index, point) in enumerate(points):
        if index + 1 < len(points):
            next_point = points[index + 1]
            distance = math.hypot(float(next_point['x']) - float(point['x']), float(next_point['y']) - float(point['y']))
            yaw_change = _delta(float(next_point['yaw']), float(point['yaw']))
            steering = math.atan(0.5 * yaw_change / distance) if distance > 1e-09 else 0.0
            projection = (float(next_point['x']) - float(point['x'])) * math.cos(float(point['yaw'])) + (float(next_point['y']) - float(point['y'])) * math.sin(float(point['yaw']))
            direction = 'forward' if projection >= -1e-06 else 'reverse'
        else:
            steering = float(annotated[-1]['steering']) if annotated else 0.0
            direction = str(annotated[-1]['motion_direction']) if annotated else 'forward'
        annotated.append({'x': float(point['x']), 'y': float(point['y']), 'yaw': _wrap(float(point['yaw'])), 'velocity': 0.0, 'steering': steering, 'source': source, 'motion_direction': direction, 'planner_backend': spec.backend, 'backend_version': spec.version})
    steering_step_limit = math.radians(15.0)
    for index in range(1, len(annotated)):
        previous = float(annotated[index - 1]['steering'])
        delta = float(annotated[index]['steering']) - previous
        annotated[index]['steering'] = previous + max(-steering_step_limit, min(steering_step_limit, delta))
    return annotated

def _path_hash(points: Sequence[Mapping[str, Any]]) -> str:
    normalized = [{key: value for (key, value) in point.items() if key != 'path_hash'} for point in points]
    payload = json.dumps(normalized, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()

def _signed_curvature(a: Mapping[str, Any], b: Mapping[str, Any], c: Mapping[str, Any]) -> float:
    ab = math.hypot(float(b['x']) - float(a['x']), float(b['y']) - float(a['y']))
    bc = math.hypot(float(c['x']) - float(b['x']), float(c['y']) - float(b['y']))
    ac = math.hypot(float(c['x']) - float(a['x']), float(c['y']) - float(a['y']))
    if min(ab, bc, ac) <= 1e-12:
        return 0.0
    cross = (float(b['x']) - float(a['x'])) * (float(c['y']) - float(a['y'])) - (float(b['y']) - float(a['y'])) * (float(c['x']) - float(a['x']))
    return 2.0 * cross / (ab * bc * ac)

def unavailable_plan(spec: BackendSpec, *, source: str='') -> PlanResult:
    return PlanResult(planner_success=False, points=None, failure_code='BACKEND_UNAVAILABLE', failure_detail=spec.reason, planner_backend=spec.backend, backend_version=spec.version, source=source, diagnostics={'mature_backend': spec.mature, 'available': spec.available, 'backend_called': False})

def _curvature(a: Mapping[str, Any], b: Mapping[str, Any], c: Mapping[str, Any]) -> float:
    return abs(_signed_curvature(a, b, c))
