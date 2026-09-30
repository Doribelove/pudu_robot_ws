"""Admission estimates for the frozen seven-second planning success deadline.

These estimates reject hopeless retries before allocating or publishing a new
corridor. They do not extend the deadline or replace the checks before search.
"""
from __future__ import annotations
import math


def retry_admission(remaining_s, *, confirmed_search_s=None,
                    reconfigure_peak_s=0., corridor_ms=0., costmap_ms=0.):
    values = [remaining_s, reconfigure_peak_s, corridor_ms, costmap_ms]
    if any(not math.isfinite(v) or v < 0 for v in values):
        raise ValueError('RETRY_BUDGET_INPUT_INVALID')
    if confirmed_search_s is not None and (
            not math.isfinite(confirmed_search_s) or confirmed_search_s <= 0):
        raise ValueError('CONFIRMED_SEARCH_BUDGET_INVALID')
    reserve = max(1.25, reconfigure_peak_s + .3)
    # Reconfiguration uses the existing half-second quantum. The existing
    # post-RPC check also keeps .3 seconds beyond the confirmed search limit.
    search_required = reserve + .5 + .3
    strategy = 'reconfigure'
    if confirmed_search_s is not None and confirmed_search_s + .3 <= search_required:
        search_required = confirmed_search_s + .3
        strategy = 'reuse_confirmed'
    # The preceding attempt is the same route/ROI. Use both observed costs,
    # with headroom, rather than pretending the next update is free. This is
    # an estimate, not a worst-case bound; the absolute deadline remains final.
    preparation_estimate = 1.25 * (corridor_ms + costmap_ms) / 1000.
    required = preparation_estimate + search_required
    return {
        'admitted': remaining_s >= required,
        'remaining_s': remaining_s,
        'required_s': required,
        'preparation_estimate_s': preparation_estimate,
        'minimum_search_s': .5,
        'search_and_control_required_s': search_required,
        'reconfiguration_reserve_s': reserve,
        'server_budget_strategy': strategy,
        'estimate_policy': 'previous_same_route_cost_plus_25_percent_v1',
    }
