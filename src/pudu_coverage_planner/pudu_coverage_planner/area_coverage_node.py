"""Known-map full-area coverage orchestration for an already active Nav2 stack."""

import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Point32, PointStamped, PolygonStamped, PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .coverage_geometry import (
    dilate_mask,
    erode_mask,
    plan_coverage,
    polygon_to_mask,
    split_path_at_turns,
)


GridPoint = Tuple[float, float]
WorldPoint = Tuple[float, float]


class AreaCoverageNode(Node):
    """Plan and execute Boustrophedon coverage over a user-selected polygon."""

    def __init__(self) -> None:
        super().__init__("area_coverage")
        self._declare_parameters()
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.area_topic = str(self.get_parameter("area_topic").value)
        self.clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, latched_qos)
        self.create_subscription(
            PolygonStamped, self.area_topic, self._area_callback, 10)
        self.create_subscription(
            PointStamped, self.clicked_point_topic, self._clicked_point_callback, 10)

        self.path_pub = self.create_publisher(Path, "/coverage/path", latched_qos)
        self.traveled_path_pub = self.create_publisher(
            Path, "/coverage/traveled_path", latched_qos)
        self.uncovered_cells_pub = self.create_publisher(
            GridCells, "/coverage/uncovered_cells", latched_qos)
        self.covered_cells_pub = self.create_publisher(
            GridCells, "/coverage/covered_cells", latched_qos)
        self.pose_arrows_pub = self.create_publisher(
            MarkerArray, "/coverage/pose_arrows", latched_qos)
        self.area_pub = self.create_publisher(
            PolygonStamped, "/coverage/clicked_polygon", latched_qos)
        self.grid_pub = self.create_publisher(
            OccupancyGrid, "/coverage/grid", latched_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/coverage/markers", latched_qos)
        self.progress_pub = self.create_publisher(Float32, "/coverage/progress", 10)
        self.status_pub = self.create_publisher(String, "/coverage/status", latched_qos)

        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.follow_client = ActionClient(self, FollowPath, "/follow_path")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, "/coverage/plan", self._plan_service)
        self.create_service(Trigger, "/coverage/plan_map", self._plan_map_service)
        self.create_service(Trigger, "/coverage/start", self._start_service)
        self.create_service(Trigger, "/coverage/pause", self._pause_service)
        self.create_service(Trigger, "/coverage/resume", self._resume_service)
        self.create_service(Trigger, "/coverage/cancel", self._cancel_service)
        self.create_service(Trigger, "/coverage/clear", self._clear_service)
        self.create_service(Trigger, "/coverage/close_area", self._close_area_service)
        self.create_service(Trigger, "/coverage/query", self._query_service)

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_data: Optional[np.ndarray] = None
        self.area_world: List[WorldPoint] = []
        self.clicked_world: List[WorldPoint] = []
        self.target_mask: Optional[np.ndarray] = None
        self.navigable_mask: Optional[np.ndarray] = None
        self.covered_mask: Optional[np.ndarray] = None
        self.cell_paths: List[Path] = []
        self.execution_paths: List[Path] = []
        self.current_path: Optional[Path] = None
        self.plan_source = "none"
        self.state = "waiting_for_map"
        self.detail = "waiting for OccupancyGrid"
        self.repair_pass = 0
        self.failed_cells = 0
        self.sequence_token = 0
        self.active_goal_handle = None
        self.last_tf_warning_ns = 0
        self.last_progress = 0.0
        self.traveled_path = Path()
        self.traveled_path.header.frame_id = self.global_frame
        self.pose_arrow_samples: List[PoseStamped] = []
        self.last_trail_pose: Optional[Tuple[float, float, float]] = None
        self.last_trail_time_ns = 0
        self.last_pose_arrow_time_ns = 0
        self.distance_since_arrow = 0.0
        self.last_visual_covered_count = -1
        self.last_visual_target_count = -1

        self.create_timer(0.10, self._coverage_timer)
        self.create_timer(0.10, self._motion_history_timer)
        self.create_timer(0.50, self._publish_timer)
        self._publish_status()
        self.get_logger().info(
            "Area coverage ready: planning the reachable /map component by "
            "default; publish a PolygonStamped to %s for a manual sub-area"
            % self.area_topic)

    def _declare_parameters(self) -> None:
        defaults = {
            "map_topic": "/map",
            "area_topic": "/coverage/area",
            "clicked_point_topic": "/clicked_point",
            "global_frame": "map",
            "robot_base_frame": "base_link",
            "occupied_threshold": 50,
            "unknown_is_obstacle": True,
            "robot_radius": 0.22,
            "coverage_radius": 0.38,
            "lane_spacing": 0.38,
            "path_point_spacing": 0.08,
            "turn_split_angle": 0.60,
            "min_sweep_segment_length": 0.45,
            "min_cell_area": 0.10,
            "min_repair_area": 0.04,
            "completion_threshold": 0.98,
            "max_repair_passes": 2,
            "controller_id": "FollowPath",
            "goal_checker_id": "general_goal_checker",
            "auto_plan_on_area": True,
            "auto_plan_full_map": True,
            "auto_execute_on_area": False,
            "trail_sample_distance": 0.03,
            "trail_sample_period": 0.25,
            "max_trail_points": 20000,
            "visualization_cell_size": 0.15,
            "pose_arrow_trigger": "distance",
            "pose_arrow_distance": 0.50,
            "pose_arrow_period": 5.0,
            "max_pose_arrows": 1000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        expected = int(msg.info.width * msg.info.height)
        if expected == 0 or len(msg.data) != expected or msg.info.resolution <= 0.0:
            self.get_logger().error("Rejected invalid OccupancyGrid")
            return
        self.map_msg = msg
        self.map_data = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width))
        if self.state == "waiting_for_map":
            self._set_state("idle", "map received; waiting for coverage area")
            if bool(self.get_parameter("auto_plan_full_map").value):
                success, message = self._make_map_plan(reset_coverage=True)
                if not success:
                    self.get_logger().warning(
                        "Automatic map coverage planning deferred: %s" % message)

    def _area_callback(self, msg: PolygonStamped) -> None:
        frame = msg.header.frame_id or self.global_frame
        if frame != self.global_frame:
            self.get_logger().error(
                "Coverage polygon frame '%s' must be '%s'" %
                (frame, self.global_frame))
            return
        points = [(float(point.x), float(point.y)) for point in msg.polygon.points]
        if len(points) < 3:
            self.get_logger().error("Coverage polygon requires at least three points")
            return
        self.area_world = points
        self.plan_source = "polygon"
        self.clicked_world = []
        self._publish_area_polygon()
        if bool(self.get_parameter("auto_plan_on_area").value):
            success, message = self._make_plan(reset_coverage=True)
            if success and bool(self.get_parameter("auto_execute_on_area").value):
                self._begin_execution()
            elif not success:
                self.get_logger().error(message)

    def _clicked_point_callback(self, msg: PointStamped) -> None:
        frame = msg.header.frame_id or self.global_frame
        if frame != self.global_frame:
            self.get_logger().error(
                "Clicked point frame '%s' must be '%s'" %
                (frame, self.global_frame))
            return
        self.clicked_world.append((float(msg.point.x), float(msg.point.y)))
        self._publish_area_polygon(use_clicked=True)
        self._set_state(
            "selecting_area",
            "%d polygon vertices; call /coverage/close_area after the last point"
            % len(self.clicked_world),
        )

    def _plan_service(self, _request: Trigger.Request, response: Trigger.Response):
        if self.plan_source == "map" or len(self.area_world) < 3:
            response.success, response.message = self._make_map_plan(
                reset_coverage=True)
        else:
            response.success, response.message = self._make_plan(
                reset_coverage=True)
        return response

    def _plan_map_service(self, _request: Trigger.Request, response: Trigger.Response):
        response.success, response.message = self._make_map_plan(
            reset_coverage=True)
        return response

    def _start_service(self, _request: Trigger.Request, response: Trigger.Response):
        if not self.cell_paths:
            if bool(self.get_parameter("auto_plan_full_map").value):
                success, message = self._make_map_plan(reset_coverage=True)
            else:
                success, message = self._make_plan(reset_coverage=True)
            if not success:
                response.success = False
                response.message = message
                return response
        response.success, response.message = self._begin_execution()
        return response

    def _pause_service(self, _request: Trigger.Request, response: Trigger.Response):
        if self.state not in ("connecting", "sweeping"):
            response.success = False
            response.message = "coverage is not executing"
            return response
        self.sequence_token += 1
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        self.active_goal_handle = None
        self.execution_paths = []
        self.current_path = None
        self._publish_path([])
        self._publish_markers([])
        self._set_state("paused", "execution paused; covered cells are preserved")
        response.success = True
        response.message = self.detail
        return response

    def _resume_service(self, _request: Trigger.Request, response: Trigger.Response):
        if self.state != "paused":
            response.success = False
            response.message = "coverage is not paused"
            return response
        paths = self._make_residual_plan()
        if not paths:
            progress = self._progress()
            threshold = float(self.get_parameter("completion_threshold").value)
            terminal_state = "completed" if progress >= threshold else "partial"
            self._set_state(
                terminal_state,
                "no reachable residual sweep remains at %.1f%%" %
                (100.0 * progress),
            )
            response.success = True
            response.message = self.detail
            return response
        self.cell_paths = paths
        response.success, response.message = self._begin_execution()
        return response

    def _cancel_service(self, _request: Trigger.Request, response: Trigger.Response):
        self.sequence_token += 1
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        self.active_goal_handle = None
        self.execution_paths = []
        self.current_path = None
        self._publish_path([])
        self._publish_markers([])
        self._set_state("cancelled", "execution cancelled; coverage history preserved")
        response.success = True
        response.message = self.detail
        return response

    def _clear_service(self, _request: Trigger.Request, response: Trigger.Response):
        self.sequence_token += 1
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        self.active_goal_handle = None
        self.area_world = []
        self.clicked_world = []
        self.target_mask = None
        self.navigable_mask = None
        self.covered_mask = None
        self.cell_paths = []
        self.execution_paths = []
        self.current_path = None
        self.plan_source = "none"
        self.repair_pass = 0
        self.failed_cells = 0
        self._reset_visual_history()
        self._set_state("idle", "coverage area and history cleared")
        self._publish_area_polygon()
        self._publish_path([])
        self._publish_markers([])
        self._publish_coverage_cells(force=True)
        response.success = True
        response.message = self.detail
        return response

    def _close_area_service(self, _request: Trigger.Request, response: Trigger.Response):
        if len(self.clicked_world) < 3:
            response.success = False
            response.message = "select at least three RViz points first"
            return response
        self.area_world = list(self.clicked_world)
        self.plan_source = "polygon"
        self.clicked_world = []
        self._publish_area_polygon()
        response.success, response.message = self._make_plan(reset_coverage=True)
        if response.success and bool(self.get_parameter("auto_execute_on_area").value):
            self._begin_execution()
        return response

    def _query_service(self, _request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = self._status_text()
        return response

    def _make_plan(self, reset_coverage: bool) -> Tuple[bool, str]:
        if self.map_msg is None or self.map_data is None:
            return False, "no OccupancyGrid has been received"
        if len(self.area_world) < 3:
            return False, "no closed coverage polygon; publish /coverage/area first"
        if self.state in ("connecting", "sweeping"):
            return False, "cancel or pause the current execution before replanning"

        area_grid = [self._world_to_grid(point) for point in self.area_world]
        polygon_mask = polygon_to_mask(self.map_data.shape, area_grid)
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        if bool(self.get_parameter("unknown_is_obstacle").value):
            free_mask = (self.map_data >= 0) & (self.map_data < occupied_threshold)
        else:
            free_mask = self.map_data < occupied_threshold
        raw_target = polygon_mask & free_mask
        radius_cells = int(math.ceil(
            float(self.get_parameter("robot_radius").value) /
            self.map_msg.info.resolution))
        navigable = erode_mask(polygon_mask, radius_cells) & erode_mask(
            free_mask, radius_cells)
        if not np.any(raw_target):
            return False, "selected polygon contains no known free map cells"
        if not np.any(navigable):
            return False, "selected polygon is too small after robot-footprint inflation"

        self.plan_source = "polygon"
        return self._commit_plan(
            raw_target, navigable, reset_coverage, "selected polygon")

    def _make_map_plan(self, reset_coverage: bool) -> Tuple[bool, str]:
        """Plan the robot-reachable component of known free OccupancyGrid space."""
        if self.map_msg is None or self.map_data is None:
            return False, "no OccupancyGrid has been received"
        if self.state in ("connecting", "sweeping"):
            return False, "cancel or pause the current execution before replanning"

        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        if bool(self.get_parameter("unknown_is_obstacle").value):
            free_mask = (self.map_data >= 0) & (self.map_data < occupied_threshold)
        else:
            free_mask = self.map_data < occupied_threshold
        if not np.any(free_mask):
            return False, "the map contains no known free cells"

        robot_grid: Optional[GridPoint] = None
        robot_pose = self._robot_world_pose(warn=False)
        if robot_pose is not None:
            robot_grid = self._world_to_grid((robot_pose[0], robot_pose[1]))
        reachable_free = self._reachable_component(free_mask, robot_grid)

        radius_cells = int(math.ceil(
            float(self.get_parameter("robot_radius").value) /
            self.map_msg.info.resolution))
        navigable_candidates = erode_mask(reachable_free, radius_cells)
        navigable = self._reachable_component(
            navigable_candidates, robot_grid)
        if not np.any(navigable):
            return False, "no robot-sized reachable free component exists in the map"

        # Expand safe robot-center cells back toward walls so the displayed and
        # measured scan area represents reachable floor, not only center poses.
        raw_target = reachable_free & dilate_mask(navigable, radius_cells)
        self.area_world = self._outer_boundary(raw_target)
        self.clicked_world = []
        self.plan_source = "map"
        self._publish_area_polygon()
        return self._commit_plan(
            raw_target, navigable, reset_coverage, "reachable map")

    def _commit_plan(
        self,
        raw_target: np.ndarray,
        navigable: np.ndarray,
        reset_coverage: bool,
        source: str,
    ) -> Tuple[bool, str]:
        paths = self._plan_mask(navigable)
        if not paths:
            return False, "no sweep path could be generated for the %s" % source
        self.target_mask = raw_target
        self.navigable_mask = navigable
        if reset_coverage or self.covered_mask is None:
            self.covered_mask = np.zeros_like(raw_target, dtype=bool)
            self.repair_pass = 0
            self.failed_cells = 0
            self._reset_visual_history()
        self.cell_paths = paths
        self.execution_paths = []
        self.current_path = None
        self._publish_path(paths)
        self._publish_markers(paths)
        self._publish_coverage_cells(force=True)
        assert self.map_msg is not None
        area_m2 = (
            float(np.count_nonzero(raw_target)) *
            self.map_msg.info.resolution ** 2)
        self._set_state(
            "planned",
            "%s %.1f m^2: %d sweep segments, %d poses; call /coverage/start"
            % (
                source,
                area_m2,
                len(paths),
                sum(len(path.poses) for path in paths),
            ),
        )
        return True, self.detail

    @staticmethod
    def _reachable_component(
        mask: np.ndarray, start: Optional[GridPoint]
    ) -> np.ndarray:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), connectivity=8)
        if count <= 1:
            return np.zeros_like(mask, dtype=bool)

        selected = 0
        if start is not None:
            x = int(round(start[0]))
            y = int(round(start[1]))
            if 0 <= y < mask.shape[0] and 0 <= x < mask.shape[1]:
                selected = int(labels[y, x])
            if selected == 0:
                rows, columns = np.nonzero(labels)
                if rows.size:
                    nearest = int(np.argmin(
                        (columns.astype(float) - start[0]) ** 2 +
                        (rows.astype(float) - start[1]) ** 2))
                    selected = int(labels[rows[nearest], columns[nearest]])
        if selected == 0:
            selected = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        return labels == selected

    def _outer_boundary(self, mask: np.ndarray) -> List[WorldPoint]:
        contours, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        contour = max(contours, key=cv2.contourArea)
        perimeter = cv2.arcLength(contour, True)
        simplified = cv2.approxPolyDP(
            contour, max(1.0, 0.002 * perimeter), True)
        return [
            self._grid_to_world((float(point[0][0]), float(point[0][1])))
            for point in simplified
        ]

    def _plan_mask(self, mask: np.ndarray) -> List[Path]:
        assert self.map_msg is not None
        resolution = self.map_msg.info.resolution
        lane_cells = max(1, int(round(
            float(self.get_parameter("lane_spacing").value) / resolution)))
        point_cells = max(0.5,
            float(self.get_parameter("path_point_spacing").value) / resolution)
        min_area_cells = max(1, int(round(
            float(self.get_parameter("min_cell_area").value) /
            (resolution * resolution))))
        start = None
        pose = self._robot_world_pose(warn=False)
        if pose is not None:
            start = self._world_to_grid((pose[0], pose[1]))
        plan = plan_coverage(
            mask, lane_cells, point_cells, min_area_cells, start=start)
        execution_paths: List[Path] = []
        for points in plan.cell_paths:
            usable = split_path_at_turns(
                points,
                float(self.get_parameter("turn_split_angle").value),
                float(self.get_parameter("min_sweep_segment_length").value) /
                resolution,
            )
            execution_paths.extend(
                self._grid_points_to_path(piece) for piece in usable)
        self.get_logger().info(
            "Coverage sweep angle %.0f deg, %d cells -> %d executable segments" %
            (plan.sweep_rotation_deg, len(plan.cell_paths), len(execution_paths)))
        return execution_paths

    def _make_residual_plan(self) -> List[Path]:
        if (self.target_mask is None or self.covered_mask is None or
                self.navigable_mask is None or self.map_msg is None):
            return []
        residual = self.target_mask & ~self.covered_mask
        min_cells = max(1, int(round(
            float(self.get_parameter("min_repair_area").value) /
            (self.map_msg.info.resolution ** 2))))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            residual.astype(np.uint8), connectivity=8)
        filtered = np.zeros_like(residual)
        for label in range(1, count):
            if int(stats[label, cv2.CC_STAT_AREA]) >= min_cells:
                filtered[labels == label] = True
        coverage_cells = int(math.ceil(
            float(self.get_parameter("coverage_radius").value) /
            self.map_msg.info.resolution))
        repair_centers = self.navigable_mask & dilate_mask(filtered, coverage_cells)
        if not np.any(repair_centers):
            return []
        paths = self._plan_mask(repair_centers)
        self._publish_path(paths)
        self._publish_markers(paths)
        return paths

    def _begin_execution(self) -> Tuple[bool, str]:
        if self.state in ("connecting", "sweeping"):
            return False, "coverage is already executing"
        if not self.navigate_client.server_is_ready():
            return False, "Nav2 /navigate_to_pose action is not ready"
        if not self.follow_client.server_is_ready():
            return False, "Nav2 /follow_path action is not ready"
        if not self.cell_paths:
            return False, "there is no coverage plan"
        self.sequence_token += 1
        token = self.sequence_token
        self.execution_paths = list(self.cell_paths)
        self.current_path = None
        self.repair_pass = 0
        self.failed_cells = 0
        now_ns = self.get_clock().now().nanoseconds
        self.last_trail_time_ns = now_ns
        self.last_pose_arrow_time_ns = now_ns
        self._set_state("connecting", "starting coverage execution")
        self._run_next_cell(token)
        return True, "coverage execution started"

    def _run_next_cell(self, token: int) -> None:
        if token != self.sequence_token:
            return
        if not self.execution_paths:
            self.current_path = None
            self._finish_pass(token)
            return
        self.current_path = self.execution_paths.pop(0)
        self._publish_upcoming_path()
        goal = NavigateToPose.Goal()
        goal.pose = self.current_path.poses[0]
        self._set_state(
            "connecting",
            "moving to cell entrance; %d cells remain" %
            (len(self.execution_paths) + 1),
        )
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done, saved_token=token: self._navigate_goal_response(
                done, saved_token))

    def _navigate_goal_response(self, future, token: int) -> None:
        if token != self.sequence_token:
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # ROS action transport errors
            self._cell_failed(token, "connector request error: %s" % error)
            return
        if not goal_handle.accepted:
            self._cell_failed(token, "connector goal rejected")
            return
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, saved_token=token: self._navigate_result(done, saved_token))

    def _navigate_result(self, future, token: int) -> None:
        if token != self.sequence_token:
            return
        self.active_goal_handle = None
        try:
            status = future.result().status
        except Exception as error:
            self._cell_failed(token, "connector result error: %s" % error)
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._cell_failed(token, "could not reach cell entrance (status %d)" % status)
            return
        self._send_follow_path(token)

    def _send_follow_path(self, token: int) -> None:
        if token != self.sequence_token or self.current_path is None:
            return
        goal = FollowPath.Goal()
        goal.path = self.current_path
        goal.controller_id = str(self.get_parameter("controller_id").value)
        goal.goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self._set_state(
            "sweeping",
            "sweeping cell; %d cells remain" % (len(self.execution_paths) + 1),
        )
        future = self.follow_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done, saved_token=token: self._follow_goal_response(
                done, saved_token))

    def _follow_goal_response(self, future, token: int) -> None:
        if token != self.sequence_token:
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._cell_failed(token, "FollowPath request error: %s" % error)
            return
        if not goal_handle.accepted:
            self._cell_failed(token, "FollowPath goal rejected")
            return
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, saved_token=token: self._follow_result(done, saved_token))

    def _follow_result(self, future, token: int) -> None:
        if token != self.sequence_token:
            return
        self.active_goal_handle = None
        try:
            status = future.result().status
        except Exception as error:
            self._cell_failed(token, "FollowPath result error: %s" % error)
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._cell_failed(token, "FollowPath failed (status %d)" % status)
            return
        self.current_path = None
        self._run_next_cell(token)

    def _cell_failed(self, token: int, reason: str) -> None:
        if token != self.sequence_token:
            return
        self.active_goal_handle = None
        self.current_path = None
        self.failed_cells += 1
        self.get_logger().warning("Coverage cell skipped: %s" % reason)
        self._run_next_cell(token)

    def _finish_pass(self, token: int) -> None:
        if token != self.sequence_token:
            return
        progress = self._progress()
        threshold = float(self.get_parameter("completion_threshold").value)
        max_repairs = int(self.get_parameter("max_repair_passes").value)
        if progress >= threshold:
            self._publish_path([])
            self._publish_markers([])
            self._set_state(
                "completed", "coverage %.1f%% complete" % (100.0 * progress))
            return
        if self.repair_pass < max_repairs:
            paths = self._make_residual_plan()
            if paths:
                self.repair_pass += 1
                self.cell_paths = paths
                self.execution_paths = list(paths)
                self._set_state(
                    "connecting",
                    "repair pass %d/%d for %.1f%% residual"
                    % (self.repair_pass, max_repairs, 100.0 * (1.0 - progress)),
                )
                self._run_next_cell(token)
                return
        self._set_state(
            "partial",
            "coverage stopped at %.1f%% after %d repair passes and %d failed cells"
            % (100.0 * progress, self.repair_pass, self.failed_cells),
        )
        self._publish_path([])
        self._publish_markers([])

    def _coverage_timer(self) -> None:
        if self.state != "sweeping" or self.target_mask is None:
            return
        pose = self._robot_world_pose(warn=True)
        if pose is None or self.map_msg is None:
            return
        gx, gy = self._world_to_grid((pose[0], pose[1]))
        radius_cells = int(math.ceil(
            float(self.get_parameter("coverage_radius").value) /
            self.map_msg.info.resolution))
        center_x = int(round(gx))
        center_y = int(round(gy))
        y0 = max(0, center_y - radius_cells)
        y1 = min(self.target_mask.shape[0], center_y + radius_cells + 1)
        x0 = max(0, center_x - radius_cells)
        x1 = min(self.target_mask.shape[1], center_x + radius_cells + 1)
        if x0 >= x1 or y0 >= y1:
            return
        ys, xs = np.ogrid[y0:y1, x0:x1]
        disk = ((xs - gx) ** 2 + (ys - gy) ** 2) <= radius_cells ** 2
        assert self.covered_mask is not None
        self.covered_mask[y0:y1, x0:x1] |= disk & self.target_mask[y0:y1, x0:x1]

    def _motion_history_timer(self) -> None:
        """Record the real TF trajectory and leave periodic pose-arrow breadcrumbs."""
        if self.state not in ("connecting", "sweeping"):
            return
        robot_pose = self._robot_world_pose(warn=False)
        if robot_pose is None:
            return

        now = self.get_clock().now()
        now_ns = now.nanoseconds
        x, y, yaw = robot_pose
        step_distance = 0.0
        if self.last_trail_pose is not None:
            step_distance = math.hypot(
                x - self.last_trail_pose[0], y - self.last_trail_pose[1])
        elapsed = (now_ns - self.last_trail_time_ns) * 1e-9
        min_distance = max(
            0.0, float(self.get_parameter("trail_sample_distance").value))
        min_period = max(
            0.0, float(self.get_parameter("trail_sample_period").value))
        if (self.last_trail_pose is not None and
                step_distance < min_distance and elapsed < min_period):
            return

        pose = self._make_pose_stamped(x, y, yaw, now.to_msg())
        self.traveled_path.header.stamp = now.to_msg()
        self.traveled_path.poses.append(pose)
        max_points = max(2, int(self.get_parameter("max_trail_points").value))
        if len(self.traveled_path.poses) > max_points:
            self.traveled_path.poses = self.traveled_path.poses[-max_points:]
        self.traveled_path_pub.publish(self.traveled_path)

        self.distance_since_arrow += step_distance
        self.last_trail_pose = (x, y, yaw)
        self.last_trail_time_ns = now_ns

        arrow_period = max(
            0.0, float(self.get_parameter("pose_arrow_period").value))
        arrow_distance = max(
            0.0, float(self.get_parameter("pose_arrow_distance").value))
        elapsed_arrow = (now_ns - self.last_pose_arrow_time_ns) * 1e-9
        distance_due = self.distance_since_arrow >= arrow_distance
        time_due = elapsed_arrow >= arrow_period
        trigger = str(self.get_parameter("pose_arrow_trigger").value).lower()
        if trigger == "time":
            arrow_due = time_due
        elif trigger == "either":
            arrow_due = distance_due or time_due
        else:
            arrow_due = distance_due
        if not self.pose_arrow_samples:
            arrow_due = True
        if arrow_due:
            self.pose_arrow_samples.append(pose)
            max_arrows = max(
                1, int(self.get_parameter("max_pose_arrows").value))
            if len(self.pose_arrow_samples) > max_arrows:
                self.pose_arrow_samples = self.pose_arrow_samples[-max_arrows:]
            self.distance_since_arrow = 0.0
            self.last_pose_arrow_time_ns = now_ns
            self._publish_pose_arrows()

    def _publish_timer(self) -> None:
        progress = self._progress()
        self.last_progress = progress
        self.progress_pub.publish(Float32(data=float(progress)))
        self.status_pub.publish(String(data=self._status_text()))
        self._publish_coverage_grid()
        self._publish_coverage_cells()

    def _progress(self) -> float:
        if self.target_mask is None or self.covered_mask is None:
            return 0.0
        total = int(np.count_nonzero(self.target_mask))
        if total == 0:
            return 0.0
        covered = int(np.count_nonzero(self.covered_mask & self.target_mask))
        return covered / total

    def _set_state(self, state: str, detail: str) -> None:
        changed = state != self.state or detail != self.detail
        self.state = state
        self.detail = detail
        self._publish_status()
        if changed:
            self.get_logger().info("Coverage [%s]: %s" % (state, detail))

    def _status_text(self) -> str:
        return "%s | %.1f%% | %s" % (
            self.state, 100.0 * self._progress(), self.detail)

    def _publish_status(self) -> None:
        self.status_pub.publish(String(data=self._status_text()))

    def _publish_coverage_grid(self) -> None:
        if self.map_msg is None or self.target_mask is None or self.covered_mask is None:
            return
        output = OccupancyGrid()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.global_frame
        output.info = self.map_msg.info
        data = np.full(self.target_mask.shape, -1, dtype=np.int8)
        data[self.target_mask] = 0
        data[self.covered_mask & self.target_mask] = 100
        output.data = data.reshape(-1).tolist()
        self.grid_pub.publish(output)

    def _publish_coverage_cells(self, force: bool = False) -> None:
        if (self.map_msg is None or self.target_mask is None or
                self.covered_mask is None):
            if force:
                empty = GridCells()
                empty.header.stamp = self.get_clock().now().to_msg()
                empty.header.frame_id = self.global_frame
                self.uncovered_cells_pub.publish(empty)
                self.covered_cells_pub.publish(empty)
                self.last_visual_covered_count = -1
                self.last_visual_target_count = -1
            return

        covered_mask = self.covered_mask & self.target_mask
        covered_count = int(np.count_nonzero(covered_mask))
        target_count = int(np.count_nonzero(self.target_mask))
        if (not force and covered_count == self.last_visual_covered_count and
                target_count == self.last_visual_target_count):
            return
        self.uncovered_cells_pub.publish(
            self._mask_to_grid_cells(self.target_mask & ~covered_mask))
        self.covered_cells_pub.publish(self._mask_to_grid_cells(covered_mask))
        self.last_visual_covered_count = covered_count
        self.last_visual_target_count = target_count

    def _mask_to_grid_cells(self, mask: np.ndarray) -> GridCells:
        assert self.map_msg is not None
        msg = GridCells()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        map_resolution = float(self.map_msg.info.resolution)
        requested_size = max(
            map_resolution,
            float(self.get_parameter("visualization_cell_size").value),
        )
        stride = max(1, int(round(requested_size / map_resolution)))
        height, width = mask.shape
        padded_height = int(math.ceil(height / stride) * stride)
        padded_width = int(math.ceil(width / stride) * stride)
        padded = np.zeros((padded_height, padded_width), dtype=bool)
        padded[:height, :width] = mask
        reduced = padded.reshape(
            padded_height // stride, stride,
            padded_width // stride, stride,
        ).any(axis=(1, 3))
        block_rows, block_columns = np.nonzero(reduced)
        rows = np.minimum(
            block_rows * stride + 0.5 * (stride - 1), height - 1)
        columns = np.minimum(
            block_columns * stride + 0.5 * (stride - 1), width - 1)
        cell_size = map_resolution * stride
        msg.cell_width = cell_size
        msg.cell_height = cell_size
        msg.cells = [
            Point(x=world[0], y=world[1], z=0.015)
            for world in (
                self._grid_to_world((float(column), float(row)))
                for row, column in zip(rows, columns)
            )
        ]
        return msg

    def _reset_visual_history(self) -> None:
        self.traveled_path = Path()
        self.traveled_path.header.frame_id = self.global_frame
        self.traveled_path.header.stamp = self.get_clock().now().to_msg()
        self.pose_arrow_samples = []
        self.last_trail_pose = None
        self.last_trail_time_ns = 0
        self.last_pose_arrow_time_ns = 0
        self.distance_since_arrow = 0.0
        self.last_visual_covered_count = -1
        self.last_visual_target_count = -1
        self.traveled_path_pub.publish(self.traveled_path)
        self._publish_pose_arrows()

    def _publish_pose_arrows(self) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for index, pose in enumerate(self.pose_arrow_samples):
            arrow = Marker()
            arrow.header.frame_id = self.global_frame
            arrow.header.stamp = stamp
            arrow.ns = "coverage_pose_arrows"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = pose.pose.position.x
            arrow.pose.position.y = pose.pose.position.y
            arrow.pose.position.z = 0.09
            arrow.pose.orientation = pose.pose.orientation
            arrow.scale.x = 0.32
            arrow.scale.y = 0.09
            arrow.scale.z = 0.09
            arrow.color = ColorRGBA(r=0.95, g=0.12, b=0.80, a=0.95)
            array.markers.append(arrow)
        self.pose_arrows_pub.publish(array)

    def _publish_area_polygon(self, use_clicked: bool = False) -> None:
        points = self.clicked_world if use_clicked else self.area_world
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.polygon.points = [
            Point32(x=float(x), y=float(y), z=0.0) for x, y in points]
        self.area_pub.publish(msg)

    def _publish_path(self, paths: Sequence[Path]) -> None:
        combined = Path()
        combined.header.stamp = self.get_clock().now().to_msg()
        combined.header.frame_id = self.global_frame
        for path in paths:
            combined.poses.extend(path.poses)
        self.path_pub.publish(combined)

    def _publish_upcoming_path(self) -> None:
        paths: List[Path] = []
        if self.current_path is not None:
            paths.append(self.current_path)
        paths.extend(self.execution_paths)
        self._publish_path(paths)
        self._publish_markers(paths)

    def _publish_markers(self, paths: Sequence[Path]) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        if self.area_world:
            boundary = Marker()
            boundary.header.frame_id = self.global_frame
            boundary.header.stamp = self.get_clock().now().to_msg()
            boundary.ns = "coverage_area"
            boundary.id = 0
            boundary.type = Marker.LINE_STRIP
            boundary.action = Marker.ADD
            boundary.scale.x = 0.04
            boundary.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=1.0)
            boundary.pose.orientation.w = 1.0
            closed = self.area_world + [self.area_world[0]]
            boundary.points = [Point(x=x, y=y, z=0.04) for x, y in closed]
            array.markers.append(boundary)
        for index, path in enumerate(paths):
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "coverage_cells"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.025
            marker.color = ColorRGBA(r=1.0, g=0.72, b=0.05, a=0.95)
            marker.pose.orientation.w = 1.0
            marker.points = [
                Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.05)
                for pose in path.poses]
            array.markers.append(marker)
        self.marker_pub.publish(array)

    def _world_to_grid(self, point: WorldPoint) -> GridPoint:
        assert self.map_msg is not None
        origin = self.map_msg.info.origin
        yaw = self._yaw_from_quaternion(
            origin.orientation.x, origin.orientation.y,
            origin.orientation.z, origin.orientation.w)
        dx = point[0] - origin.position.x
        dy = point[1] - origin.position.y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            (cosine * dx + sine * dy) / self.map_msg.info.resolution - 0.5,
            (-sine * dx + cosine * dy) / self.map_msg.info.resolution - 0.5,
        )

    def _grid_to_world(self, point: GridPoint) -> WorldPoint:
        assert self.map_msg is not None
        origin = self.map_msg.info.origin
        yaw = self._yaw_from_quaternion(
            origin.orientation.x, origin.orientation.y,
            origin.orientation.z, origin.orientation.w)
        local_x = (point[0] + 0.5) * self.map_msg.info.resolution
        local_y = (point[1] + 0.5) * self.map_msg.info.resolution
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            origin.position.x + cosine * local_x - sine * local_y,
            origin.position.y + sine * local_x + cosine * local_y,
        )

    def _grid_points_to_path(self, points: Sequence[GridPoint]) -> Path:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        world_points = [self._grid_to_world(point) for point in points]
        for index, point in enumerate(world_points):
            if index + 1 < len(world_points):
                next_point = world_points[index + 1]
                yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
            elif index > 0:
                previous = world_points[index - 1]
                yaw = math.atan2(point[1] - previous[1], point[0] - previous[0])
            else:
                yaw = 0.0
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            path.poses.append(pose)
        return path

    def _make_pose_stamped(self, x: float, y: float, yaw: float, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _robot_world_pose(self, warn: bool) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as error:
            now_ns = self.get_clock().now().nanoseconds
            if warn and now_ns - self.last_tf_warning_ns > 5_000_000_000:
                self.get_logger().warning("Coverage TF unavailable: %s" % error)
                self.last_tf_warning_ns = now_ns
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x), float(translation.y),
            self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y),
                          1.0 - 2.0 * (y * y + z * z))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AreaCoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
