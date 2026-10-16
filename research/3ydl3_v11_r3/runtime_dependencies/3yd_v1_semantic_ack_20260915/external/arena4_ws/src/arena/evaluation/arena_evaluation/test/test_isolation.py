from __future__ import annotations

import time

from arena_evaluation.planner_benchmark.isolation import run_isolated


def _quick_worker(value: int) -> int:
    return value * 2


def _sampled_worker(value: int) -> int:
    time.sleep(0.04)
    return value


def _slow_worker() -> None:
    time.sleep(0.5)


def test_run_isolated_executes_one_fresh_child_and_returns_value():
    result = run_isolated(_quick_worker, args=(21,), sample_interval_ms=5.0)
    assert result.value == 42
    assert result.exception_type == ""
    assert result.cpu_total_ms is not None
    # A very short request may have fewer than two /proc samples; that is an
    # explicit diagnosis rather than a fabricated peak value.
    assert isinstance(result.sampling_limited, bool)


def test_run_isolated_samples_rss_pss_and_cpu_for_longer_request():
    result = run_isolated(_sampled_worker, args=(7,), sample_interval_ms=5.0)
    assert result.value == 7
    assert result.sample_count >= 2
    assert result.sample_interval_ms == 5.0
    assert result.cpu_user_ms is not None
    assert result.process_rss_before_bytes is not None
    assert result.process_rss_peak_bytes is not None
    assert result.process_rss_after_bytes is not None
    assert result.process_pss_after_bytes is not None
    assert result.rss_delta_bytes is not None
    assert result.child_elapsed_ms is not None
    assert result.monitor_wall_time_ms >= result.child_elapsed_ms
    assert result.monitor_overhead_ms is not None


def test_run_isolated_reports_timeout_separately():
    result = run_isolated(_slow_worker, timeout_s=0.02, sample_interval_ms=5.0)
    assert result.timed_out is True
    assert result.value is None
    assert result.exception_type == ""
