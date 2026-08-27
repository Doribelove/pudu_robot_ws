from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

from .models import ResourceMeasurement, ResourceSnapshot


CLK_TCK = float(os.sysconf(os.sysconf_names["SC_CLK_TCK"]))


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    comm: str
    cmdline: str


def process_info(pid: int) -> Optional[ProcessInfo]:
    proc = Path(f"/proc/{pid}")
    try:
        comm = (proc / "comm").read_text().strip()
        raw_cmdline = (proc / "cmdline").read_bytes().replace(b"\x00", b" ").decode(errors="replace").strip()
        return ProcessInfo(pid=pid, comm=comm, cmdline=raw_cmdline)
    except (FileNotFoundError, PermissionError, OSError):
        return None


def discover_process(name: str, *, cmdline_contains: str = "") -> Optional[ProcessInfo]:
    candidates: List[ProcessInfo] = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(proc.name)
        except ValueError:
            continue
        info = process_info(pid)
        if info is None or info.comm != name:
            continue
        if cmdline_contains and cmdline_contains not in info.cmdline:
            continue
        candidates.append(info)
    if not candidates:
        return None
    # The benchmark launches one fresh instance per stack. If another ROS graph
    # is present, the most recently started matching process is the least
    # surprising choice; the caller still records an explicit discovery error
    # when no process exists.
    return max(candidates, key=lambda item: item.pid)


def read_snapshot(pid: int) -> Optional[ResourceSnapshot]:
    proc = Path(f"/proc/{pid}")
    try:
        stat = (proc / "stat").read_text()
        close_paren = stat.rfind(")")
        if close_paren < 0:
            return None
        fields = stat[close_paren + 2 :].split()
        # fields[0] is state (field 3); utime/stime are fields 14/15.
        user_ticks = int(fields[11])
        system_ticks = int(fields[12])
        rss_bytes = None
        for line in (proc / "status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                rss_bytes = int(line.split()[1]) * 1024
                break
        pss_bytes = None
        try:
            for line in (proc / "smaps_rollup").read_text().splitlines():
                if line.startswith("Pss:"):
                    pss_bytes = int(line.split()[1]) * 1024
                    break
        except (FileNotFoundError, PermissionError, OSError):
            pass
        return ResourceSnapshot(
            cpu_user_ms=user_ticks / CLK_TCK * 1000.0,
            cpu_system_ms=system_ticks / CLK_TCK * 1000.0,
            rss_bytes=rss_bytes,
            pss_bytes=pss_bytes,
        )
    except (FileNotFoundError, PermissionError, OSError, ValueError, IndexError):
        return None


class ResourceMonitor:
    """Sample planner and stack resources around exactly one action request."""

    def __init__(self, planner_pid: int, stack_pids: Sequence[int], interval_ms: float = 10.0):
        self.planner_pid = planner_pid
        self.stack_pids = tuple(dict.fromkeys(int(pid) for pid in stack_pids))
        self.interval_s = max(float(interval_ms), 1.0) / 1000.0
        self.before_planner: Optional[ResourceSnapshot] = None
        self.before_stack: Optional[ResourceSnapshot] = None
        self.samples: List[tuple[float, Optional[ResourceSnapshot], Optional[ResourceSnapshot]]] = []
        self.errors: List[str] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.before_planner = read_snapshot(self.planner_pid)
        self.before_stack = _sum_snapshots(self.stack_pids)
        if self.before_planner is None:
            self.errors.append(f"planner process {self.planner_pid} disappeared before request")
        if self.before_stack is None:
            self.errors.append("one or more planning stack processes disappeared before request")
        self._thread = threading.Thread(target=self._sample_loop, name="planner-resource-monitor", daemon=True)
        self._thread.start()

    def finish(self, wall_time_ms: float) -> ResourceMeasurement:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.1, self.interval_s * 5.0))
        self.samples.append((time.monotonic(), read_snapshot(self.planner_pid), _sum_snapshots(self.stack_pids)))
        after_planner = next((item[1] for item in reversed(self.samples) if item[1] is not None), None)
        measurement = ResourceMeasurement(sample_interval_ms=self.interval_s * 1000.0)
        if self.before_planner is None or after_planner is None:
            self.errors.append("planner process disappeared while sampling")
        else:
            measurement.planner_cpu_user_ms = max(0.0, after_planner.cpu_user_ms - self.before_planner.cpu_user_ms)
            measurement.planner_cpu_system_ms = max(0.0, after_planner.cpu_system_ms - self.before_planner.cpu_system_ms)
            measurement.planner_cpu_total_ms = measurement.planner_cpu_user_ms + measurement.planner_cpu_system_ms
            if wall_time_ms > 0:
                measurement.planner_cpu_percent = measurement.planner_cpu_total_ms / wall_time_ms * 100.0
        measurement.planner_rss_before_bytes = _value(self.before_planner, "rss_bytes")
        measurement.planner_pss_before_bytes = _value(self.before_planner, "pss_bytes")
        measurement.planner_rss_peak_bytes = _peak(self.samples, 1, "rss_bytes")
        measurement.planner_pss_peak_bytes = _peak(self.samples, 1, "pss_bytes")
        measurement.stack_rss_before_bytes = _value(self.before_stack, "rss_bytes")
        measurement.stack_pss_before_bytes = _value(self.before_stack, "pss_bytes")
        measurement.stack_rss_peak_bytes = _peak(self.samples, 2, "rss_bytes")
        measurement.stack_pss_peak_bytes = _peak(self.samples, 2, "pss_bytes")
        measurement.process_error = "; ".join(dict.fromkeys(self.errors))
        return measurement

    def _sample_loop(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            planner = read_snapshot(self.planner_pid)
            stack = _sum_snapshots(self.stack_pids)
            if planner is None:
                self.errors.append(f"planner process {self.planner_pid} disappeared during sampling")
            if stack is None:
                self.errors.append("one or more planning stack processes disappeared during sampling")
            self.samples.append((now, planner, stack))
            self._stop.wait(self.interval_s)


def _sum_snapshots(pids: Sequence[int]) -> Optional[ResourceSnapshot]:
    snapshots = [read_snapshot(pid) for pid in pids]
    if not snapshots or any(snapshot is None for snapshot in snapshots):
        return None
    typed = [snapshot for snapshot in snapshots if snapshot is not None]
    rss_values = [snapshot.rss_bytes for snapshot in typed]
    pss_values = [snapshot.pss_bytes for snapshot in typed]
    return ResourceSnapshot(
        cpu_user_ms=sum(float(snapshot.cpu_user_ms or 0.0) for snapshot in typed),
        cpu_system_ms=sum(float(snapshot.cpu_system_ms or 0.0) for snapshot in typed),
        rss_bytes=(sum(int(value) for value in rss_values if value is not None) if all(value is not None for value in rss_values) else None),
        pss_bytes=(sum(int(value) for value in pss_values if value is not None) if all(value is not None for value in pss_values) else None),
    )


def _value(snapshot: Optional[ResourceSnapshot], field: str) -> Optional[int]:
    if snapshot is None:
        return None
    value = getattr(snapshot, field)
    return int(value) if value is not None else None


def _peak(samples: Sequence[tuple[float, Optional[ResourceSnapshot], Optional[ResourceSnapshot]]], index: int, field: str) -> Optional[int]:
    values = []
    for sample in samples:
        snapshot = sample[index]
        if snapshot is not None and getattr(snapshot, field) is not None:
            values.append(int(getattr(snapshot, field)))
    return max(values) if values else None
