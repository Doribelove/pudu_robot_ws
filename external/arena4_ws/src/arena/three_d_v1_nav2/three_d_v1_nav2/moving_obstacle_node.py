"""Deterministic moving-obstacle harness for the frozen query-04 Nav2 run.

The default ``single`` profile preserves the accepted one-obstacle experiment.
The ``four_independent`` profile keeps that obstacle and adds three independently
phased crossing obstacles on the accepted L3 route. The original envelope is
confirmed first; the three additions are confirmed later as one exact snapshot
so the online bridge produces an auditable stop--replan--resume event.
"""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from arena_evaluation.dynamic_snapshot import DynamicSnapshot
from arena_evaluation.planner_benchmark.map_utils import HospitalMap

from .contracts import EXPECTED_MAP_SHA256


QUERY_ID = "cmp2-04-multi-junction"
ENTITY_NAME = "seq8_moving_obstacle"


@dataclass(frozen=True)
class ObstacleSpec:
    """One deterministic moving Gazebo box and its full swept segment."""

    entity: str
    x0: float
    y0: float
    x1: float
    y1: float
    speed_mps: float
    phase_s: float
    side_m: float
    color_rgb: tuple[float, float, float]
    confirmation: str

    @property
    def span_m(self) -> float:
        return math.hypot(self.x1 - self.x0, self.y1 - self.y0)


def triangular_position(
    elapsed_s: float, lower: float, upper: float, speed: float,
) -> tuple[float, int]:
    """Return position and direction for a constant-speed triangular wave."""
    if not upper > lower:
        raise ValueError("upper must be greater than lower")
    if not speed > 0.0:
        raise ValueError("speed must be positive")
    span = upper - lower
    phase = (max(0.0, elapsed_s) * speed) % (2.0 * span)
    if phase <= span:
        return lower + phase, 1
    return upper - (phase - span), -1


def segment_pose(spec: ObstacleSpec, elapsed_s: float) -> tuple[float, float, int]:
    """Interpolate an independently phased triangular wave on a 2-D segment."""
    distance, direction = triangular_position(
        elapsed_s + spec.phase_s, 0.0, spec.span_m, spec.speed_mps)
    fraction = distance / spec.span_m
    return (
        spec.x0 + fraction * (spec.x1 - spec.x0),
        spec.y0 + fraction * (spec.y1 - spec.y0),
        direction,
    )


def swept_segment_cells(
    grid: HospitalMap, spec: ObstacleSpec, spacing_m: float,
) -> list[tuple[int, int]]:
    """Sample a complete 2-D center envelope with exact end points."""
    if not spacing_m > 0.0:
        raise ValueError("invalid swept-envelope spacing")
    count = max(1, math.ceil(spec.span_m / spacing_m))
    return sorted({
        grid.world_to_cell(
            spec.x0 + (spec.x1 - spec.x0) * index / count,
            spec.y0 + (spec.y1 - spec.y0) * index / count,
        )
        for index in range(count + 1)
    })


def swept_envelope_cells(
    grid: HospitalMap, x_min: float, x_max: float, y: float, spacing_m: float,
) -> list[tuple[int, int]]:
    """Backward-compatible helper for the accepted horizontal profile."""
    if not x_max > x_min:
        raise ValueError("invalid swept-envelope geometry")
    return swept_segment_cells(
        grid,
        ObstacleSpec(
            ENTITY_NAME, x_min, y, x_max, y, 0.30, 0.0, 0.40,
            (1.0, 0.05, 0.05), "initial"),
        spacing_m,
    )


def four_independent_specs(side_m: float = 0.40) -> tuple[ObstacleSpec, ...]:
    """Frozen four-entity layout: original entity plus three L3 crossings."""
    return (
        ObstacleSpec(
            "seq8_moving_obstacle_0", 5.8, 54.116896, 10.2, 54.116896,
            0.30, 0.0, side_m, (1.0, 0.05, 0.05), "initial"),
        ObstacleSpec(
            "seq8_moving_obstacle_1", 9.557352111708319,
            58.898494148878136, 10.716461916091681, 59.20908143352187,
            0.22, 1.8, side_m, (0.05, 0.45, 1.0), "delayed"),
        ObstacleSpec(
            "seq8_moving_obstacle_2", 8.372704683889175,
            48.705595723092124, 9.530340486310823, 48.38955878070788,
            0.27, 3.1, side_m, (0.75, 0.10, 1.0), "delayed"),
        ObstacleSpec(
            "seq8_moving_obstacle_3", 2.2211112577785057,
            26.5966615003528, 2.539693382021494, 25.4397235460472,
            0.31, 4.4, side_m, (0.05, 0.85, 0.35), "delayed"),
    )


def obstacle_sdf(
    entity_name: str, side_m: float, color_rgb: tuple[float, float, float],
) -> str:
    """Build a colored, lidar-visible box model."""
    sdf = ET.Element("sdf", version="1.6")
    model = ET.SubElement(sdf, "model", name=entity_name)
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="body")
    color = f"{color_rgb[0]} {color_rgb[1]} {color_rgb[2]} 1.0"
    for kind in ("collision", "visual"):
        entity = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(entity, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = f"{side_m} {side_m} 1.0"
        if kind == "visual":
            material = ET.SubElement(entity, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color
            ET.SubElement(material, "emissive").text = (
                f"{0.25 * color_rgb[0]} {0.25 * color_rgb[1]} "
                f"{0.25 * color_rgb[2]} 1.0")
    return ET.tostring(sdf, encoding="unicode")


class MovingObstacle(Node):
    """Move one or four Gazebo obstacles and publish auditable state."""

    def __init__(self) -> None:
        super().__init__("three_d_v1_moving_obstacle")
        for name, default in (
            ("run_dir", ""),
            ("result_dir", ""),
            ("query_id", QUERY_ID),
            ("profile", "single"),
            ("x_min", 5.8),
            ("x_max", 10.2),
            ("y", 54.116896),
            ("speed_mps", 0.30),
            ("side_m", 0.40),
            ("update_rate_hz", 10.0),
            ("confirmation_interval_s", 0.45),
            ("delayed_confirmation_s", 8.0),
            ("envelope_spacing_m", 0.40),
        ):
            self.declare_parameter(name, default)
        self.run_dir = Path(str(self.get_parameter("run_dir").value))
        self.output = Path(str(self.get_parameter("result_dir").value))
        self.query_id = str(self.get_parameter("query_id").value)
        self.profile = str(self.get_parameter("profile").value)
        self.rate = float(self.get_parameter("update_rate_hz").value)
        self.confirmation_interval = float(
            self.get_parameter("confirmation_interval_s").value)
        self.delayed_confirmation = float(
            self.get_parameter("delayed_confirmation_s").value)
        self.envelope_spacing = float(
            self.get_parameter("envelope_spacing_m").value)
        side = float(self.get_parameter("side_m").value)
        if self.profile == "single":
            self.specs = (ObstacleSpec(
                ENTITY_NAME,
                float(self.get_parameter("x_min").value),
                float(self.get_parameter("y").value),
                float(self.get_parameter("x_max").value),
                float(self.get_parameter("y").value),
                float(self.get_parameter("speed_mps").value),
                0.0, side, (1.0, 0.05, 0.05), "initial"),)
        elif self.profile == "four_independent":
            self.specs = four_independent_specs(side)
        else:
            raise RuntimeError(f"unsupported moving-obstacle profile: {self.profile}")
        if self.query_id != QUERY_ID:
            raise RuntimeError(
                "moving-obstacle profile is frozen to cmp2-04-multi-junction")
        if not self.output.is_dir():
            raise RuntimeError("result_dir must exist")
        grid = HospitalMap.load(
            self.run_dir / "derived_map/extracted/optemap.yaml")
        self.map_shape = grid.occupancy.shape
        self.cells_by_entity = {
            spec.entity: swept_segment_cells(grid, spec, self.envelope_spacing)
            for spec in self.specs
        }
        initial_cells = sorted({
            cell for spec in self.specs if spec.confirmation == "initial"
            for cell in self.cells_by_entity[spec.entity]
        })
        all_cells = sorted({
            cell for cells in self.cells_by_entity.values() for cell in cells
        })
        self.observation_plan = [
            (0.0, "initial", initial_cells),
            (self.confirmation_interval, "initial", initial_cells),
        ]
        if self.profile == "four_independent":
            self.observation_plan.extend([
                (self.delayed_confirmation, "four_combined", all_cells),
                (self.delayed_confirmation + self.confirmation_interval,
                 "four_combined", all_cells),
            ])
        self.state = "WAIT_STACK_ACTIVE"
        self.mission_state = ""
        self.started_mono = None
        self.observation_count = 0
        self.pending_pose: dict[str, object] = {}
        self.service_stats = {
            spec.entity: {"successes": 0, "failures": 0}
            for spec in self.specs
        }
        self.snapshot_pub = self.create_publisher(
            String, "/three_d_v1/dynamic_snapshot", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/three_d_v1/moving_obstacle_markers", 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, "/three_d_v1/moving_obstacle_pose", 10)
        self.states_pub = self.create_publisher(
            String, "/three_d_v1/moving_obstacle_states", 10)
        self.event_pub = self.create_publisher(
            String, "/three_d_v1/moving_obstacle_event", 10)
        self.create_subscription(
            String, "/three_d_v1/mission_state", self._mission, 10)
        self.pose_client = self.create_client(
            SetEntityPose, "/world/default/set_pose")
        self.trace_file = (self.output / "moving_obstacle_trace.csv").open(
            "w", newline="")
        self.trace = csv.DictWriter(self.trace_file, fieldnames=(
            "wall_time_ns", "ros_time_ns", "mission_state", "entity",
            "obstacle_index", "x", "y", "direction", "command_sequence",
            "set_pose_successes", "set_pose_failures"))
        self.trace.writeheader()
        self.command_sequence = 0
        self.spawn_records: list[dict] = []
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

    def _mission(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if payload.get("query_id") in (None, "", self.query_id):
                self.mission_state = str(payload.get("state", ""))
        except (TypeError, ValueError):
            self.mission_state = "INVALID_STATE_MESSAGE"

    def _spawn(self, spec: ObstacleSpec) -> None:
        x, y, _ = segment_pose(spec, 0.0)
        content = obstacle_sdf(spec.entity, spec.side_m, spec.color_rgb)
        request = (
            f"sdf: {json.dumps(content)} name: {json.dumps(spec.entity)} "
            f"allow_renaming: false pose {{ position {{ x: {x} "
            f"y: {y} z: 0.5 }} }}"
        )
        command = [
            "gz", "service", "-s", "/world/default/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
            "--req", request,
        ]
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=10,
            env=dict(
                os.environ,
                GZ_PARTITION=os.environ.get(
                    "GZ_PARTITION", "nav2_actual2p5_20260907"),
            ),
        )
        output = result.stdout + result.stderr
        if result.returncode or "data: true" not in output.lower():
            raise RuntimeError(
                f"Gazebo spawn failed for {spec.entity}: " + output)
        self.spawn_records.append({
            "entity": spec.entity,
            "command": command,
            "returncode": result.returncode,
            "output": output,
            "initial_pose": [x, y, 0.5],
            "side_m": spec.side_m,
        })

    def _publish_observation(
        self, scope: str, cells: list[tuple[int, int]],
    ) -> None:
        snapshot = DynamicSnapshot.from_cells(
            f"moving-{scope}-{self.observation_count}", cells,
            timestamp=time.time(), map_version=EXPECTED_MAP_SHA256,
            map_shape=self.map_shape)
        self.snapshot_pub.publish(String(data=json.dumps(snapshot.as_dict())))
        event = {
            "event": "GLOBAL_SWEPT_ENVELOPE_OBSERVATION",
            "index": self.observation_count,
            "scope": scope,
            "entity_count": 1 if scope == "initial" else len(self.specs),
            "snapshot": snapshot.as_dict(),
        }
        self.event_pub.publish(String(data=json.dumps(event)))
        with (self.output / "moving_obstacle_observations.jsonl").open(
            "a") as stream:
            stream.write(json.dumps(event) + "\n")
        self.observation_count += 1

    def _collect_pose_results(self) -> None:
        for entity, future in list(self.pending_pose.items()):
            if not future.done():
                continue
            try:
                response = future.result()
                key = (
                    "successes" if response is not None and response.success
                    else "failures")
            except Exception:
                key = "failures"
            self.service_stats[entity][key] += 1
            del self.pending_pose[entity]

    def _send_pose(self, spec: ObstacleSpec, x: float, y: float) -> None:
        if (
            not self.pose_client.service_is_ready()
            or spec.entity in self.pending_pose
        ):
            return
        request = SetEntityPose.Request()
        request.entity = Entity(name=spec.entity, type=Entity.MODEL)
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = 0.5
        request.pose.orientation.w = 1.0
        self.pending_pose[spec.entity] = self.pose_client.call_async(request)

    def _publish_visuals(
        self, positions: list[tuple[ObstacleSpec, float, float, int]],
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = []
        states = []
        delayed_confirmed = self.observation_count >= len(self.observation_plan)
        for index, (spec, x, y, direction) in enumerate(positions):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.5
            pose.orientation.w = 1.0
            states.append({
                "entity": spec.entity, "index": index, "x": x, "y": y,
                "direction": direction, "speed_mps": spec.speed_mps,
                "confirmation": spec.confirmation,
            })
            if index == 0:
                stamped = PoseStamped()
                stamped.header.frame_id = "map"
                stamped.header.stamp = stamp
                stamped.pose = pose
                self.pose_pub.publish(stamped)

            current = Marker()
            current.header.frame_id = "map"
            current.header.stamp = stamp
            current.ns = "moving_obstacle_current"
            current.id = index
            current.type = Marker.CUBE
            current.action = Marker.ADD
            current.pose = pose
            current.scale.x = spec.side_m
            current.scale.y = spec.side_m
            current.scale.z = 1.0
            current.color.r, current.color.g, current.color.b = spec.color_rgb
            current.color.a = 1.0
            markers.append(current)

            envelope = Marker()
            envelope.header = current.header
            envelope.ns = "moving_obstacle_envelope"
            envelope.id = 100 + index
            envelope.type = Marker.CUBE
            envelope.action = Marker.ADD
            envelope.pose.position.x = 0.5 * (spec.x0 + spec.x1)
            envelope.pose.position.y = 0.5 * (spec.y0 + spec.y1)
            envelope.pose.position.z = 0.025
            yaw = math.atan2(spec.y1 - spec.y0, spec.x1 - spec.x0)
            envelope.pose.orientation.z = math.sin(0.5 * yaw)
            envelope.pose.orientation.w = math.cos(0.5 * yaw)
            envelope.scale.x = spec.span_m + spec.side_m
            envelope.scale.y = spec.side_m + 0.70
            envelope.scale.z = 0.05
            envelope.color.r, envelope.color.g, envelope.color.b = spec.color_rgb
            envelope.color.a = 0.28
            markers.append(envelope)

            label = Marker()
            label.header = current.header
            label.ns = "moving_obstacle_label"
            label.id = 200 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 1.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.30
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            arrow = ">" if direction > 0 else "<"
            status = (
                "CONFIRMED"
                if spec.confirmation == "initial" or delayed_confirmed
                else "TRACKING")
            label.text = f"O{index + 1} {arrow} {spec.speed_mps:.2f} {status}"
            markers.append(label)
        self.marker_pub.publish(MarkerArray(markers=markers))
        self.states_pub.publish(String(data=json.dumps({
            "ros_time_ns": self.get_clock().now().nanoseconds,
            "mission_state": self.mission_state,
            "obstacles": states,
        })))

    def _tick(self) -> None:
        self._collect_pose_results()
        if self.state == "WAIT_STACK_ACTIVE":
            if self.mission_state != "ACTIVE":
                return
            for spec in self.specs:
                self._spawn(spec)
            if not self.pose_client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("Gazebo set_pose bridge is unavailable")
            self.started_mono = time.monotonic()
            self.state = "MOVING"
            self.event_pub.publish(String(data=json.dumps({
                "event": "MOVING_OBSTACLES_STARTED",
                "profile": self.profile,
                "entity_count": len(self.specs),
            })))
        if self.state != "MOVING":
            return
        elapsed = time.monotonic() - self.started_mono
        while (
            self.observation_count < len(self.observation_plan)
            and elapsed >= self.observation_plan[self.observation_count][0]
        ):
            _, scope, cells = self.observation_plan[self.observation_count]
            self._publish_observation(scope, cells)

        positions = []
        now_wall = time.time_ns()
        now_ros = self.get_clock().now().nanoseconds
        self.command_sequence += 1
        for index, spec in enumerate(self.specs):
            x, y, direction = segment_pose(spec, elapsed)
            positions.append((spec, x, y, direction))
            self._send_pose(spec, x, y)
            stats = self.service_stats[spec.entity]
            self.trace.writerow({
                "wall_time_ns": now_wall,
                "ros_time_ns": now_ros,
                "mission_state": self.mission_state,
                "entity": spec.entity,
                "obstacle_index": index,
                "x": format(x, ".17g"),
                "y": format(y, ".17g"),
                "direction": direction,
                "command_sequence": self.command_sequence,
                "set_pose_successes": stats["successes"],
                "set_pose_failures": stats["failures"],
            })
        self._publish_visuals(positions)
        self.trace_file.flush()
        if self.mission_state in ("COMPLETE", "HALT_CURRENT_QUERY"):
            self.state = "FINISHED"
            self.timer.cancel()
            obstacles = []
            for spec in self.specs:
                obstacles.append({
                    **asdict(spec),
                    "span_m": spec.span_m,
                    "envelope_cells": self.cells_by_entity[spec.entity],
                    "set_pose_successes": self.service_stats[spec.entity]["successes"],
                    "set_pose_failures": self.service_stats[spec.entity]["failures"],
                })
            summary = {
                "profile": self.profile,
                "query_id": self.query_id,
                "entity_count": len(self.specs),
                "obstacles": obstacles,
                "spawn_records": self.spawn_records,
                "update_rate_hz": self.rate,
                "observation_count": self.observation_count,
                "observation_plan": [
                    {"elapsed_s": delay, "scope": scope,
                     "cell_count": len(cells)}
                    for delay, scope, cells in self.observation_plan
                ],
                "set_pose_successes": sum(
                    value["successes"] for value in self.service_stats.values()),
                "set_pose_failures": sum(
                    value["failures"] for value in self.service_stats.values()),
                "terminal_mission_state": self.mission_state,
                "observation_source": (
                    "scripted conservative swept-envelope ground truth with "
                    "delayed confirmation; instantaneous physical poses are "
                    "observed by Gazebo lidar/local costmap"),
            }
            # Retain accepted single-profile keys for audit compatibility.
            if len(self.specs) == 1:
                spec = self.specs[0]
                summary.update({
                    "entity": spec.entity, "x_min": spec.x0,
                    "x_max": spec.x1, "y": spec.y0,
                    "side_m": spec.side_m, "speed_mps": spec.speed_mps,
                    "envelope_cells": self.cells_by_entity[spec.entity],
                })
            (self.output / "moving_obstacle_summary.json").write_text(
                json.dumps(summary, indent=2) + "\n")
            self.event_pub.publish(String(data=json.dumps({
                "event": "MOVING_OBSTACLES_FINISHED", **summary})))

    def close(self) -> None:
        if not self.trace_file.closed:
            self.trace_file.close()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = MovingObstacle()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
