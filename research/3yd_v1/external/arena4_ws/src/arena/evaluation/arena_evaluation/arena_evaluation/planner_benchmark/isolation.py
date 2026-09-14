"""Independent child-process execution and /proc resource sampling.

This utility is for one benchmark request at a time.  The parent process owns
the sampling loop, while the child executes exactly one callable invocation.
It is deliberately generic so the four planner implementations can share the
same measurement semantics without introducing a second recorder framework.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import queue
import resource
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Optional, Sequence

from .resources import read_snapshot


@dataclass
class IsolatedRunResult:
    """Result and resource fields for one isolated request."""

    value: Any = None
    exception_type: str = ""
    exception_message: str = ""
    wall_time_ms: float = 0.0
    # ``wall_time_ms`` is the child request elapsed time when the child sends a
    # result.  The parent-side duration, including /proc sampling and the
    # result handshake, is retained separately so timeout audits do not confuse
    # monitoring overhead with planner execution.
    monitor_wall_time_ms: float = 0.0
    planner_elapsed_ms: Optional[float] = None
    child_elapsed_ms: Optional[float] = None
    monitor_overhead_ms: Optional[float] = None
    cpu_user_ms: Optional[float] = None
    cpu_system_ms: Optional[float] = None
    process_rss_before_bytes: Optional[int] = None
    process_rss_peak_bytes: Optional[int] = None
    process_rss_after_bytes: Optional[int] = None
    process_pss_before_bytes: Optional[int] = None
    process_pss_peak_bytes: Optional[int] = None
    process_pss_after_bytes: Optional[int] = None
    rss_delta_bytes: Optional[int] = None
    pss_delta_bytes: Optional[int] = None
    sample_interval_ms: float = 5.0
    sample_count: int = 0
    sampling_limited: bool = False
    timed_out: bool = False

    @property
    def cpu_total_ms(self) -> Optional[float]:
        if self.cpu_user_ms is None and self.cpu_system_ms is None:
            return None
        return float(self.cpu_user_ms or 0.0) + float(self.cpu_system_ms or 0.0)


def _child_entry(conn: Any, target: Callable[..., Any], args: tuple[Any, ...], kwargs: dict[str, Any]) -> None:
    started = time.monotonic_ns()
    usage_before = resource.getrusage(resource.RUSAGE_SELF)
    before_snapshot = read_snapshot(os.getpid())
    try:
        value = target(*args, **kwargs)
        error_type = ""
        error_message = ""
    except BaseException as exc:  # send structured failure; never kill parent
        value = None
        error_type = type(exc).__name__
        error_message = str(exc)
    usage = resource.getrusage(resource.RUSAGE_SELF)
    after_snapshot = read_snapshot(os.getpid())
    payload = {
        "value": value,
        "exception_type": error_type,
        "exception_message": error_message,
        # Exclude process bootstrap/import CPU. The parent is responsible for
        # keeping map preparation outside the target when it needs a separate
        # preparation timing; this delta is the isolated request CPU.
        "cpu_user_ms": max(0.0, float(usage.ru_utime - usage_before.ru_utime) * 1000.0),
        "cpu_system_ms": max(0.0, float(usage.ru_stime - usage_before.ru_stime) * 1000.0),
        "elapsed_ms": (time.monotonic_ns() - started) / 1e6,
        "before_snapshot": _snapshot_payload(before_snapshot),
        "after_snapshot": _snapshot_payload(after_snapshot),
    }
    try:
        conn.send(payload)
        # Keep the process observable until the parent has collected the final
        # /proc snapshot. The parent releases this handshake; the timeout
        # prevents a crashed parent from leaking a child indefinitely.
        if conn.poll(5.0):
            conn.recv()
    except (BrokenPipeError, EOFError, OSError):
        pass
    finally:
        conn.close()


def run_isolated(
    target: Callable[..., Any],
    args: Sequence[Any] = (),
    kwargs: Optional[Mapping[str, Any]] = None,
    *,
    timeout_s: Optional[float] = None,
    sample_interval_ms: float = 5.0,
    start_method: str = "fork",
) -> IsolatedRunResult:
    """Execute one callable in a fresh child and sample its resources.

    ``target`` should perform only the planner request; map loading or other
    preparation can be done by the parent before calling this function.  A
    timeout terminates the child and is reported as ``timed_out`` rather than
    being conflated with a planner failure.  The helper never reuses a child
    process between calls.
    """

    interval_ms = max(1.0, float(sample_interval_ms))
    interval_s = interval_ms / 1000.0
    context = mp.get_context(start_method)
    # A duplex pipe provides a short result/ack handshake. Without it, a fast
    # worker can exit before the required after-RSS/PSS sample is read.
    parent_conn, child_conn = context.Pipe(duplex=True)
    process = context.Process(target=_child_entry, args=(child_conn, target, tuple(args), dict(kwargs or {})))
    started = time.monotonic_ns()
    process.start()
    child_conn.close()
    pid = int(process.pid or 0)
    # The request deadline starts after the child exists. Process creation is
    # reported in wall_time_ms only when the caller includes it separately;
    # timeout enforcement itself must not consume the planner request budget.
    request_started = time.monotonic_ns()
    before = read_snapshot(pid) if pid else None
    samples = []
    deadline = None if timeout_s is None else time.monotonic() + max(0.0, float(timeout_s))
    timeout_triggered = False
    payload_received = False
    payload: dict[str, Any] = {}
    while process.is_alive():
        snapshot = read_snapshot(pid) if pid else None
        if snapshot is not None:
            samples.append(snapshot)
        if not payload_received and parent_conn.poll():
            try:
                payload = parent_conn.recv()
                payload_received = True
            except (EOFError, OSError):
                payload = {}
        # Once the child has returned a payload, the request itself is
        # complete. Handle the result/after-snapshot handshake before the
        # deadline check so sampling overhead at the boundary cannot turn a
        # completed request into a false timeout.
        if payload_received:
            final_snapshot = read_snapshot(pid) if pid else None
            if final_snapshot is not None:
                samples.append(final_snapshot)
            try:
                parent_conn.send(True)
            except (BrokenPipeError, OSError):
                pass
            break
        if deadline is not None and time.monotonic() >= deadline:
            timeout_triggered = True
            process.terminate()
            process.join(timeout=max(0.2, interval_s * 4.0))
            if process.is_alive():
                process.kill()
                process.join(timeout=1.0)
            break
        time.sleep(interval_s)
    process.join(timeout=max(0.1, interval_s * 2.0))
    # A fast child can disappear before the final /proc snapshot.  Preserve
    # the last observable sample and mark the result sampling-limited.
    # The explicit final sample is normally the last item while the child is
    # held at the result handshake. Use it as the after snapshot after join.
    after = samples[-1] if payload_received and samples else (read_snapshot(pid) if pid and process.is_alive() else None)
    sample_count = len(samples)
    # A timeout may terminate the child before it sends a payload. For a
    # normal request the payload was consumed in the loop above.
    if not payload_received:
        try:
            if parent_conn.poll(0.05):
                payload = parent_conn.recv()
        except (EOFError, OSError, queue.Empty):
            payload = {}
    try:
        parent_conn.close()
    except OSError:
        pass
    monitor_wall_ms = (time.monotonic_ns() - request_started) / 1e6
    planner_elapsed_ms = _as_float(payload.get("elapsed_ms")) if payload_received else None
    # A normal child result carries its own monotonic elapsed time.  Use that
    # as the request wall time; retain the parent duration independently.  For
    # a killed child there is no trustworthy child elapsed value, so the
    # monitor duration remains the only observable wall time.
    wall_ms = planner_elapsed_ms if planner_elapsed_ms is not None else monitor_wall_ms
    timed_out = timeout_triggered
    result = IsolatedRunResult(
        value=payload.get("value"),
        exception_type=str(payload.get("exception_type", "")),
        exception_message=str(payload.get("exception_message", "")),
        wall_time_ms=float(wall_ms),
        monitor_wall_time_ms=float(monitor_wall_ms),
        planner_elapsed_ms=planner_elapsed_ms,
        child_elapsed_ms=_as_float(payload.get("elapsed_ms")),
        cpu_user_ms=_as_float(payload.get("cpu_user_ms")),
        cpu_system_ms=_as_float(payload.get("cpu_system_ms")),
        process_rss_before_bytes=_snapshot_value(before, "rss_bytes"),
        process_rss_peak_bytes=_peak_snapshot(samples, "rss_bytes"),
        process_rss_after_bytes=_snapshot_value(after, "rss_bytes") or _payload_snapshot_value(payload.get("after_snapshot"), "rss_bytes"),
        process_pss_before_bytes=_snapshot_value(before, "pss_bytes"),
        process_pss_peak_bytes=_peak_snapshot(samples, "pss_bytes"),
        process_pss_after_bytes=_snapshot_value(after, "pss_bytes") or _payload_snapshot_value(payload.get("after_snapshot"), "pss_bytes"),
        sample_interval_ms=interval_ms,
        sample_count=sample_count,
        sampling_limited=sample_count < 2 or monitor_wall_ms < interval_ms,
        timed_out=timed_out,
    )
    if result.process_rss_before_bytes is None:
        result.process_rss_before_bytes = _payload_snapshot_value(payload.get("before_snapshot"), "rss_bytes")
    if result.process_pss_before_bytes is None:
        result.process_pss_before_bytes = _payload_snapshot_value(payload.get("before_snapshot"), "pss_bytes")
    if result.process_rss_peak_bytes is None:
        result.process_rss_peak_bytes = result.process_rss_after_bytes
    if result.process_pss_peak_bytes is None:
        result.process_pss_peak_bytes = result.process_pss_after_bytes
    if result.process_rss_before_bytes is not None and result.process_rss_peak_bytes is not None:
        result.rss_delta_bytes = result.process_rss_peak_bytes - result.process_rss_before_bytes
    if result.process_pss_before_bytes is not None and result.process_pss_peak_bytes is not None:
        result.pss_delta_bytes = result.process_pss_peak_bytes - result.process_pss_before_bytes
    if result.child_elapsed_ms is not None:
        # ``wall_time_ms`` is intentionally the child request duration.  The
        # parent-side monitor duration includes /proc snapshots and the pipe
        # handshake, so use that separate clock when attributing overhead.
        result.monitor_overhead_ms = max(0.0, float(result.monitor_wall_time_ms) - float(result.child_elapsed_ms))
    return result


def _snapshot_value(snapshot: Any, field: str) -> Optional[int]:
    if snapshot is None:
        return None
    value = getattr(snapshot, field, None)
    return int(value) if value is not None else None


def _snapshot_payload(snapshot: Any) -> dict[str, Optional[int]]:
    if snapshot is None:
        return {"rss_bytes": None, "pss_bytes": None}
    return {
        "rss_bytes": _snapshot_value(snapshot, "rss_bytes"),
        "pss_bytes": _snapshot_value(snapshot, "pss_bytes"),
    }


def _payload_snapshot_value(snapshot: Any, field: str) -> Optional[int]:
    if not isinstance(snapshot, dict):
        return None
    value = snapshot.get(field)
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _peak_snapshot(samples: Sequence[Any], field: str) -> Optional[int]:
    values = [_snapshot_value(item, field) for item in samples]
    values = [value for value in values if value is not None]
    return max(values) if values else None


def _as_float(value: Any) -> Optional[float]:
    try:
        return None if value is None else float(value)
    except (TypeError, ValueError):
        return None
