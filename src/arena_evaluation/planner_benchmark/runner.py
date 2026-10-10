from __future__ import annotations
import os
import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple
from .models import Query
from .path_metrics import path_from_message
from .resources import ResourceMonitor

class BenchmarkInputError(RuntimeError):
    pass

def classify_action_result(status: int, *, path_point_count: int, accepted: bool=True) -> str:
    if not accepted:
        return 'ACTION_REJECTED'
    if status == 4:
        return 'SUCCEEDED' if path_point_count else 'EMPTY_PATH'
    if status == 5:
        return 'ACTION_CANCELED'
    if status == 6:
        return 'ACTION_ABORTED'
    return 'EXCEPTION'

class BenchmarkStack:

    def __init__(self, *, map_yaml: Path, params_file: Path, log_file: Path):
        self.map_yaml = map_yaml
        self.params_file = params_file
        self.log_file = log_file
        self.process: Optional[subprocess.Popen] = None
        self.log_stream = None

    def start(self, timeout: float=45.0) -> None:
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        self.log_stream = self.log_file.open('w', encoding='utf-8')
        command = ['ros2', 'launch', 'arena_evaluation', 'planner_benchmark_stack.launch.py', f'map_yaml:={self.map_yaml}', f'params_file:={self.params_file}', 'use_sim_time:=false']
        self.process = subprocess.Popen(command, stdout=self.log_stream, stderr=subprocess.STDOUT, start_new_session=True, env=os.environ.copy())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise BenchmarkInputError(f'planner stack exited with code {self.process.returncode}; see {self.log_file}')
            started = self._started_processes()
            if all((name in started for name in ('planner_server', 'map_server', 'lifecycle_manager'))) and self._log_contains('Managed nodes are active'):
                time.sleep(1.0)
                return
            time.sleep(0.2)
        raise BenchmarkInputError(f'planner stack did not expose planner_server within {timeout}s; see {self.log_file}')

    def pids(self) -> Tuple[int, List[int], str]:
        started = self._started_processes()
        missing = [name for name in ('planner_server', 'map_server', 'lifecycle_manager') if name not in started]
        if missing or 'planner_server' not in started:
            return (0, [], 'missing process(es): ' + ', '.join(missing))
        dead = [name for (name, pid) in started.items() if not Path(f'/proc/{pid}').exists()]
        if dead:
            return (0, [], 'benchmark process(es) disappeared: ' + ', '.join(dead))
        return (started['planner_server'], [started[name] for name in ('map_server', 'planner_server', 'lifecycle_manager')], '')

    def _log_contains(self, text: str) -> bool:
        try:
            return text in self.log_file.read_text(encoding='utf-8', errors='replace')
        except OSError:
            return False

    def _started_processes(self) -> Dict[str, int]:
        try:
            content = self.log_file.read_text(encoding='utf-8', errors='replace')
        except OSError:
            return {}
        result: Dict[str, int] = {}
        for name in ('planner_server', 'map_server', 'lifecycle_manager'):
            match = re.search(f'\\[{re.escape(name)}-[^]]+\\]: process started with pid \\[(\\d+)\\]', content)
            if match:
                result[name] = int(match.group(1))
        return result

    def stop(self) -> None:
        if self.process is not None:
            for (signum, timeout) in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
                try:
                    os.killpg(self.process.pid, signum)
                except ProcessLookupError:
                    break
                try:
                    self.process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
                if signum == signal.SIGTERM:
                    continue
                if signum == signal.SIGKILL:
                    break
        if self.log_stream is not None:
            self.log_stream.close()

    def __enter__(self) -> 'BenchmarkStack':
        self.start()
        return self

    def __exit__(self, *_args: Any) -> None:
        self.stop()

class ComputePathClient:

    def __init__(self, timeout: float, node_name: str='planner_benchmark_client', *, context: Any=None):
        try:
            import rclpy
            from nav2_msgs.action import ComputePathToPose
            from rclpy.action import ActionClient
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except ImportError as exc:
            raise RuntimeError(f'ROS action dependencies are unavailable: {exc}') from exc
        self.rclpy = rclpy
        self.context = context
        self.ComputePathToPose = ComputePathToPose
        self.node = Node(node_name, context=context)
        self.client = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.timeout = float(timeout)
        self.last_timing: Dict[str, float] = {}

    def close(self) -> None:
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()

    def plan(self, query: Query, *, planner_pid: int, stack_pids: Sequence[int], sample_interval_ms: float) -> Tuple[str, str, float, Optional[float], Optional[List[Dict[str, float]]], Any]:
        started_ns = time.monotonic_ns()
        deadline = time.monotonic() + self.timeout
        server_wait_started_ns = time.monotonic_ns()
        server_ready = self.client.wait_for_server(timeout_sec=max(0.0, deadline - time.monotonic()))
        self.last_timing = {'action_server_wait_ms': (time.monotonic_ns() - server_wait_started_ns) / 1000000.0, 'action_goal_send_wait_ms': 0.0, 'action_result_wait_ms': 0.0, 'ros_path_conversion_ms': 0.0}
        if not server_ready:
            return ('', 'SERVER_UNAVAILABLE', (time.monotonic_ns() - started_ns) / 1000000.0, None, None, None)
        goal = self.ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = query.goal[0]
        goal.goal.pose.position.y = query.goal[1]
        goal.goal.pose.orientation.z = __import__('math').sin(query.goal[2] / 2.0)
        goal.goal.pose.orientation.w = __import__('math').cos(query.goal[2] / 2.0)
        goal.start.header.frame_id = 'map'
        goal.start.pose.position.x = query.start[0]
        goal.start.pose.position.y = query.start[1]
        goal.start.pose.orientation.z = __import__('math').sin(query.start[2] / 2.0)
        goal.start.pose.orientation.w = __import__('math').cos(query.start[2] / 2.0)
        goal.planner_id = 'GridBased'
        goal.use_start = True
        monitor = ResourceMonitor(planner_pid, stack_pids, sample_interval_ms)
        monitor.start()
        send_started_ns = time.monotonic_ns()
        send_future = self.client.send_goal_async(goal)
        self._spin_until(send_future, deadline)
        self.last_timing['action_goal_send_wait_ms'] = (time.monotonic_ns() - send_started_ns) / 1000000.0
        wall_after_send_ms = (time.monotonic_ns() - started_ns) / 1000000.0
        if not send_future.done():
            measurement = monitor.finish(wall_after_send_ms)
            return ('', 'CLIENT_TIMEOUT', wall_after_send_ms, measurement, None, None)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            measurement = monitor.finish(wall_after_send_ms)
            return ('', 'ACTION_REJECTED', wall_after_send_ms, measurement, None, None)
        result_started_ns = time.monotonic_ns()
        result_future = goal_handle.get_result_async()
        self._spin_until(result_future, deadline)
        self.last_timing['action_result_wait_ms'] = (time.monotonic_ns() - result_started_ns) / 1000000.0
        wall_time_ms = (time.monotonic_ns() - started_ns) / 1000000.0
        if not result_future.done():
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            measurement = monitor.finish(wall_time_ms)
            return ('', 'CLIENT_TIMEOUT', wall_time_ms, measurement, None, None)
        wrapper = result_future.result()
        status = int(getattr(wrapper, 'status', 0))
        status_text = _action_status_text(status)
        result = getattr(wrapper, 'result', None)
        planning_time_ms = None
        points = None
        if result is not None:
            duration = getattr(result, 'planning_time', None)
            if duration is not None:
                planning_time_ms = float(getattr(duration, 'sec', 0)) * 1000.0 + float(getattr(duration, 'nanosec', 0)) / 1000000.0
            path = getattr(result, 'path', None)
            conversion_started_ns = time.monotonic_ns()
            points = path_from_message(path) if path is not None else []
            self.last_timing['ros_path_conversion_ms'] = (time.monotonic_ns() - conversion_started_ns) / 1000000.0
        measurement = monitor.finish(wall_time_ms)
        return (status_text, classify_action_result(status, path_point_count=len(points or [])), wall_time_ms, measurement, points, result)

    def _spin_until(self, future: Any, deadline: float) -> None:
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))

def _action_status_text(status: int) -> str:
    return {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, f'STATUS_{status}')
