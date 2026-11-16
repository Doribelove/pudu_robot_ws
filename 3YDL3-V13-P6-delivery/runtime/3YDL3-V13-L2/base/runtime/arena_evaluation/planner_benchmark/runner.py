from __future__ import annotations

import csv
import datetime as dt
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import yaml

from .config import (
    file_sha256,
    load_protocol,
    load_queries,
    load_yaml,
    resolve_path,
    stack_parameters,
    variant_config_path,
)
from .map_utils import HospitalMap
from .models import PathMetric, Query, QueryValidation, RunRecord
from .path_metrics import analyze_path, path_from_message, save_path
from .resources import ResourceMonitor, discover_process


class BenchmarkInputError(RuntimeError):
    pass


def classify_action_result(status: int, *, path_point_count: int, accepted: bool = True) -> str:
    if not accepted:
        return "ACTION_REJECTED"
    if status == 4:
        return "SUCCEEDED" if path_point_count else "EMPTY_PATH"
    if status == 5:
        return "ACTION_CANCELED"
    if status == 6:
        return "ACTION_ABORTED"
    return "EXCEPTION"


def validate_queries(
    *,
    protocol: Dict[str, Any],
    queries: Sequence[Query],
    hospital_map: HospitalMap,
    config_variants: Sequence[str],
) -> List[QueryValidation]:
    validations: List[QueryValidation] = []
    footprint = protocol["footprint"]
    clearance = float(protocol.get("minimum_endpoint_clearance_m", 0.0))
    for variant in config_variants:
        allow_unknown = bool(protocol["variants"][variant].get("allow_unknown", True))
        for query in queries:
            validation = hospital_map.validate_query(query, footprint, clearance, allow_unknown)
            validation.config_variant = variant
            # Keep the variant in the report while preserving the requested
            # public status values (INVALID_START/INVALID_GOAL).
            if validation.validation_status != "VALID":
                if validation.start_status != "VALID":
                    validation.validation_status = "INVALID_START"
                elif validation.goal_status != "VALID":
                    validation.validation_status = "INVALID_GOAL"
            validations.append(validation)
    return validations


class BenchmarkStack:
    def __init__(self, *, map_yaml: Path, params_file: Path, log_file: Path):
        self.map_yaml = map_yaml
        self.params_file = params_file
        self.log_file = log_file
        self.process: Optional[subprocess.Popen] = None
        self.log_stream = None

    def start(self, timeout: float = 45.0) -> None:
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        self.log_stream = self.log_file.open("w", encoding="utf-8")
        command = [
            "ros2", "launch", "arena_evaluation", "planner_benchmark_stack.launch.py",
            f"map_yaml:={self.map_yaml}",
            f"params_file:={self.params_file}",
            "use_sim_time:=false",
        ]
        self.process = subprocess.Popen(
            command,
            stdout=self.log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=os.environ.copy(),
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise BenchmarkInputError(f"planner stack exited with code {self.process.returncode}; see {self.log_file}")
            started = self._started_processes()
            if all(name in started for name in ("planner_server", "map_server", "lifecycle_manager")) and self._log_contains("Managed nodes are active"):
                time.sleep(1.0)
                return
            time.sleep(0.2)
        raise BenchmarkInputError(f"planner stack did not expose planner_server within {timeout}s; see {self.log_file}")

    def pids(self) -> Tuple[int, List[int], str]:
        started = self._started_processes()
        missing = [name for name in ("planner_server", "map_server", "lifecycle_manager") if name not in started]
        if missing or "planner_server" not in started:
            return 0, [], "missing process(es): " + ", ".join(missing)
        dead = [name for name, pid in started.items() if not Path(f"/proc/{pid}").exists()]
        if dead:
            return 0, [], "benchmark process(es) disappeared: " + ", ".join(dead)
        return started["planner_server"], [started[name] for name in ("map_server", "planner_server", "lifecycle_manager")], ""

    def _log_contains(self, text: str) -> bool:
        try:
            return text in self.log_file.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return False

    def _started_processes(self) -> Dict[str, int]:
        try:
            content = self.log_file.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return {}
        result: Dict[str, int] = {}
        for name in ("planner_server", "map_server", "lifecycle_manager"):
            match = re.search(rf"\[{re.escape(name)}-[^]]+\]: process started with pid \[(\d+)\]", content)
            if match:
                result[name] = int(match.group(1))
        return result

    def stop(self) -> None:
        if self.process is not None:
            for signum, timeout in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
                try:
                    # The launch parent can exit before all ROS children. Its
                    # process group remains the authoritative cleanup target.
                    os.killpg(self.process.pid, signum)
                except ProcessLookupError:
                    break
                try:
                    self.process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
                # Even after the launch parent exits, give the process group
                # one final signal on the next iteration to reap late children.
                if signum == signal.SIGTERM:
                    continue
                if signum == signal.SIGKILL:
                    break
        if self.log_stream is not None:
            self.log_stream.close()

    def __enter__(self) -> "BenchmarkStack":
        self.start()
        return self

    def __exit__(self, *_args: Any) -> None:
        self.stop()


class ComputePathClient:
    def __init__(
        self,
        timeout: float,
        node_name: str = "planner_benchmark_client",
        *,
        context: Any = None,
    ):
        try:
            import rclpy
            from nav2_msgs.action import ComputePathToPose
            from rclpy.action import ActionClient
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except ImportError as exc:  # pragma: no cover - only reached outside ROS
            raise RuntimeError(f"ROS action dependencies are unavailable: {exc}") from exc
        self.rclpy = rclpy
        self.context = context
        self.ComputePathToPose = ComputePathToPose
        self.node = Node(node_name, context=context)
        self.client = ActionClient(self.node, ComputePathToPose, "/compute_path_to_pose")
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.timeout = float(timeout)
        self.last_timing: Dict[str, float] = {}

    def close(self) -> None:
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()

    def plan(
        self,
        query: Query,
        *,
        planner_pid: int,
        stack_pids: Sequence[int],
        sample_interval_ms: float,
    ) -> Tuple[str, str, float, Optional[float], Optional[List[Dict[str, float]]], Any]:
        started_ns = time.monotonic_ns()
        deadline = time.monotonic() + self.timeout
        server_wait_started_ns = time.monotonic_ns()
        server_ready = self.client.wait_for_server(timeout_sec=max(0.0, deadline - time.monotonic()))
        self.last_timing = {
            "action_server_wait_ms": (time.monotonic_ns() - server_wait_started_ns) / 1.0e6,
            "action_goal_send_wait_ms": 0.0,
            "action_result_wait_ms": 0.0,
            "ros_path_conversion_ms": 0.0,
        }
        if not server_ready:
            return "", "SERVER_UNAVAILABLE", (time.monotonic_ns() - started_ns) / 1e6, None, None, None
        goal = self.ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = query.goal[0]
        goal.goal.pose.position.y = query.goal[1]
        goal.goal.pose.orientation.z = __import__("math").sin(query.goal[2] / 2.0)
        goal.goal.pose.orientation.w = __import__("math").cos(query.goal[2] / 2.0)
        goal.start.header.frame_id = "map"
        goal.start.pose.position.x = query.start[0]
        goal.start.pose.position.y = query.start[1]
        goal.start.pose.orientation.z = __import__("math").sin(query.start[2] / 2.0)
        goal.start.pose.orientation.w = __import__("math").cos(query.start[2] / 2.0)
        goal.planner_id = "GridBased"
        goal.use_start = True
        monitor = ResourceMonitor(planner_pid, stack_pids, sample_interval_ms)
        monitor.start()
        send_started_ns = time.monotonic_ns()
        send_future = self.client.send_goal_async(goal)
        self._spin_until(send_future, deadline)
        self.last_timing["action_goal_send_wait_ms"] = (time.monotonic_ns() - send_started_ns) / 1.0e6
        wall_after_send_ms = (time.monotonic_ns() - started_ns) / 1e6
        if not send_future.done():
            measurement = monitor.finish(wall_after_send_ms)
            return "", "CLIENT_TIMEOUT", wall_after_send_ms, measurement, None, None
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            measurement = monitor.finish(wall_after_send_ms)
            return "", "ACTION_REJECTED", wall_after_send_ms, measurement, None, None
        result_started_ns = time.monotonic_ns()
        result_future = goal_handle.get_result_async()
        self._spin_until(result_future, deadline)
        self.last_timing["action_result_wait_ms"] = (time.monotonic_ns() - result_started_ns) / 1.0e6
        wall_time_ms = (time.monotonic_ns() - started_ns) / 1e6
        if not result_future.done():
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            measurement = monitor.finish(wall_time_ms)
            return "", "CLIENT_TIMEOUT", wall_time_ms, measurement, None, None
        wrapper = result_future.result()
        status = int(getattr(wrapper, "status", 0))
        status_text = _action_status_text(status)
        result = getattr(wrapper, "result", None)
        planning_time_ms = None
        points = None
        if result is not None:
            duration = getattr(result, "planning_time", None)
            if duration is not None:
                planning_time_ms = float(getattr(duration, "sec", 0)) * 1000.0 + float(getattr(duration, "nanosec", 0)) / 1e6
            path = getattr(result, "path", None)
            conversion_started_ns = time.monotonic_ns()
            points = path_from_message(path) if path is not None else []
            self.last_timing["ros_path_conversion_ms"] = (time.monotonic_ns() - conversion_started_ns) / 1.0e6
        measurement = monitor.finish(wall_time_ms)
        return status_text, classify_action_result(status, path_point_count=len(points or [])), wall_time_ms, measurement, points, result

    def _spin_until(self, future: Any, deadline: float) -> None:
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))


def run_benchmark(
    *,
    protocol_path: str | Path,
    queries_path: str | Path,
    output_dir: str | Path,
    planners: Sequence[str],
    config_variants: Sequence[str],
    warmups: int,
    repetitions: int,
    timeout: float,
    validate_only: bool = False,
    query_ids: Optional[Sequence[str]] = None,
) -> Path:
    protocol_file, protocol = load_protocol(protocol_path)
    if bool(protocol.get("dynamic_obstacles", False)):
        raise BenchmarkInputError("dynamic_obstacles must be false for the static planner benchmark")
    queries_file, queries = load_queries(queries_path)
    if query_ids:
        selected = set(query_ids)
        queries = [query for query in queries if query.query_id in selected]
        if not queries:
            raise BenchmarkInputError("none of the requested query IDs exist in the query set")
    if not queries:
        raise BenchmarkInputError("query set is empty")
    map_yaml = resolve_path(protocol["map_yaml"], base=protocol_file.parent)
    hospital_map = HospitalMap.load(map_yaml)
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output / "planner_runs.csv").exists() and not validate_only:
        raise BenchmarkInputError(f"refusing to overwrite existing benchmark output: {output}")
    validations = validate_queries(
        protocol=protocol,
        queries=queries,
        hospital_map=hospital_map,
        config_variants=config_variants,
    )
    _write_rows(output / "query_validation.csv", [item.as_dict() for item in validations])
    _write_rows(output / "queries.csv", [query.as_dict() for query in queries])
    _write_rows(output / "maps.csv", [{
        "map_id": protocol.get("map", "hospital"),
        "map_yaml": str(map_yaml),
        "map_sha256": hospital_map.sha256,
        "width": hospital_map.width,
        "height": hospital_map.height,
        "resolution": hospital_map.resolution,
        "origin": json.dumps(hospital_map.origin),
    }])
    protocol_copy = dict(protocol)
    protocol_copy["protocol_file"] = str(protocol_file)
    (output / "protocol.yaml").write_text(yaml.safe_dump(protocol_copy, sort_keys=False))
    _write_manifest(output, protocol, protocol_file, queries_file, map_yaml, planners, config_variants, warmups, repetitions, timeout, validate_only=validate_only)
    invalid = [item for item in validations if item.validation_status != "VALID"]
    if invalid:
        details = "; ".join(f"{item.query_id}:{item.validation_status}:{item.reason}" for item in invalid)
        raise BenchmarkInputError("query validation failed; no benchmark started: " + details)
    if validate_only:
        return output

    completed = False
    try:
        import rclpy
    except ImportError as exc:  # pragma: no cover
        raise BenchmarkInputError(f"rclpy is required for benchmark execution: {exc}") from exc
    rclpy.init()
    client = None
    records: List[RunRecord] = []
    path_metrics: List[PathMetric] = []
    try:
        client = ComputePathClient(timeout=timeout)
        for planner in planners:
            for variant in config_variants:
                planner_config_file = variant_config_path(variant, planner)
                planner_config = load_yaml(planner_config_file)
                params = stack_parameters(protocol=protocol, planner_config=planner_config)
                params_file = output / "logs" / f"stack_params_{planner}_{variant}.yaml"
                params_file.parent.mkdir(parents=True, exist_ok=True)
                params_file.write_text(yaml.safe_dump(params, sort_keys=False))
                for query in queries:
                    log_file = output / "logs" / f"stack_{planner}_{variant}_{query.query_id}.log"
                    stack = BenchmarkStack(map_yaml=map_yaml, params_file=params_file, log_file=log_file)
                    try:
                        # Large static maps spend significant time loading the
                        # image and allocating the costmap before lifecycle
                        # activation. Keep the per-request timeout separate
                        # from this cold-start allowance.
                        stack_start_timeout = max(60.0, min(180.0, timeout * 2.0))
                        stack.start(timeout=stack_start_timeout)
                        planner_pid, stack_pids, process_error = stack.pids()
                        if process_error:
                            raise BenchmarkInputError(process_error)
                        modes = [("cold", 0)] + [("warmup", index + 1) for index in range(warmups)] + [("measured", index + 1) for index in range(repetitions)]
                        for run_mode, repetition in modes:
                            record, path, metric = _run_one(
                                client=client,
                                query=query,
                                planner=planner_config.get("planner_id", planner),
                                variant=variant,
                                planner_config_file=planner_config_file,
                                protocol=protocol,
                                hospital_map=hospital_map,
                                output=output,
                                repetition=repetition,
                                run_mode=run_mode,
                                planner_pid=planner_pid,
                                stack_pids=stack_pids,
                            )
                            records.append(record)
                            if metric is not None:
                                path_metrics.append(metric)
                    finally:
                        stack.stop()
        completed = True
    finally:
        if client is not None:
            client.close()
        rclpy.shutdown()
        # Persist records even when a later query or cold start fails. This
        # preserves structured partial evidence and prevents an interrupted
        # scale run from being mistaken for an empty experiment.
        _write_rows(output / "planner_runs.csv", [record.as_dict() for record in records])
        _populate_relative_lengths(path_metrics, records)
        _write_rows(output / "path_metrics.csv", [metric.as_dict() for metric in path_metrics])
    return output


def _run_one(
    *,
    client: ComputePathClient,
    query: Query,
    planner: str,
    variant: str,
    planner_config_file: Path,
    protocol: Dict[str, Any],
    hospital_map: HospitalMap,
    output: Path,
    repetition: int,
    run_mode: str,
    planner_pid: int,
    stack_pids: Sequence[int],
) -> Tuple[RunRecord, Optional[List[Dict[str, float]]], Optional[PathMetric]]:
    run_id = f"{query.query_id}_{planner}_{variant}_{run_mode}_{repetition}_{time.time_ns()}"
    timestamp = dt.datetime.now(dt.timezone.utc).isoformat()
    action_status, result_code, wall_time_ms, measurement, points, result = client.plan(
        query,
        planner_pid=planner_pid,
        stack_pids=stack_pids,
        sample_interval_ms=float(protocol.get("sample_interval_ms", 10.0)),
    )
    path_file = ""
    metric = None
    if points:
        path_path = output / "paths" / f"{run_id}.json.gz"
        save_path(path_path, points)
        path_file = str(path_path.relative_to(output))
        if result_code == "SUCCEEDED":
            metric = analyze_path(
                run_id=run_id,
                query=query,
                planner_id=planner,
                config_variant=variant,
                points=points,
                hospital_map=hospital_map,
                footprint=protocol["footprint"],
                preferred_minimum_turning_radius=float(protocol["preferred_minimum_turning_radius"]),
                allow_unknown=bool(protocol["variants"][variant].get("allow_unknown", True)),
            )
    record = RunRecord(
        run_id=run_id,
        timestamp=timestamp,
        map_id=str(protocol.get("map", "hospital")),
        map_sha256=hospital_map.sha256,
        query_id=query.query_id,
        query_category=query.category,
        planner_id=planner,
        config_variant=variant,
        planner_config_sha256=file_sha256(planner_config_file),
        repetition=repetition,
        run_mode=run_mode,
        start_x=query.start[0],
        start_y=query.start[1],
        start_yaw=query.start[2],
        goal_x=query.goal[0],
        goal_y=query.goal[1],
        goal_yaw=query.goal[2],
        action_status=action_status,
        result_code=result_code,
        result_detail=("action result has no error_code/error_msg fields" if result is not None and result_code != "SUCCEEDED" else ""),
        planning_time_ms=_planning_time(result),
        wall_time_ms=wall_time_ms,
        planner_rss_before_bytes=measurement.planner_rss_before_bytes if measurement else None,
        planner_rss_peak_bytes=measurement.planner_rss_peak_bytes if measurement else None,
        planner_pss_before_bytes=measurement.planner_pss_before_bytes if measurement else None,
        planner_pss_peak_bytes=measurement.planner_pss_peak_bytes if measurement else None,
        stack_rss_before_bytes=measurement.stack_rss_before_bytes if measurement else None,
        stack_rss_peak_bytes=measurement.stack_rss_peak_bytes if measurement else None,
        stack_pss_before_bytes=measurement.stack_pss_before_bytes if measurement else None,
        stack_pss_peak_bytes=measurement.stack_pss_peak_bytes if measurement else None,
        sample_interval_ms=measurement.sample_interval_ms if measurement else None,
        path_point_count=len(points or []),
        path_file=path_file,
        resource_error=measurement.process_error if measurement else "",
    )
    if measurement is not None:
        record.cpu_user_ms = measurement.planner_cpu_user_ms
        record.cpu_system_ms = measurement.planner_cpu_system_ms
        record.cpu_total_ms = measurement.planner_cpu_total_ms
        record.cpu_percent = measurement.planner_cpu_percent
    return record, points, metric


def _planning_time(result: Any) -> Optional[float]:
    if result is None:
        return None
    duration = getattr(result, "planning_time", None)
    if duration is None:
        return None
    return float(getattr(duration, "sec", 0)) * 1000.0 + float(getattr(duration, "nanosec", 0)) / 1e6


def _action_status_text(status: int) -> str:
    return {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }.get(status, f"STATUS_{status}")


def _write_rows(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fieldnames: List[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _populate_relative_lengths(metrics: Sequence[PathMetric], records: Sequence[RunRecord]) -> None:
    modes = {record.run_id: record.run_mode for record in records}
    measured = [metric for metric in metrics if modes.get(metric.run_id) == "measured" and metric.path_length_m is not None and metric.footprint_collision_count == 0]
    for metric in metrics:
        peers = [item for item in measured if item.query_id == metric.query_id and item.config_variant == metric.config_variant]
        navfn = [item.path_length_m for item in peers if "navfn" in item.planner_id and item.path_length_m is not None]
        shortest = [item.path_length_m for item in peers if item.path_length_m is not None]
        if metric.path_length_m is not None and navfn:
            metric.length_over_navfn = metric.path_length_m / min(navfn)
        if metric.path_length_m is not None and shortest:
            metric.length_over_shortest_observed_valid = metric.path_length_m / min(shortest)


def _write_manifest(output: Path, protocol: Dict[str, Any], protocol_file: Path, queries_file: Path, map_yaml: Path, planners: Sequence[str], variants: Sequence[str], warmups: int, repetitions: int, timeout: float, *, validate_only: bool = False) -> None:
    manifest = {
        "schema_version": 1,
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "protocol_file": str(protocol_file),
        "queries_file": str(queries_file),
        "map_yaml": str(map_yaml),
        "map_sha256": file_sha256(map_yaml.parent / yaml.safe_load(map_yaml.read_text())["image"]),
        "planners": list(planners),
        "config_variants": list(variants),
        "warmups": warmups,
        "repetitions": repetitions,
        "timeout": timeout,
        "odom_navigation_data": False,
        "validate_only": validate_only,
    }
    (output / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
