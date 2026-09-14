"""Lane-relative, curvature-feasible semantic guide fields for 2A-V2 r3.

r2 correctly computes the query-oriented right boundary, but a collection of
low-cost cells does not prove that a forward-only vehicle can enter and follow
that collection.  r3 therefore builds a small station/lateral lattice inside
each selected lane instance.  The lattice is only a *guide generator*: the
unchanged Smac Hybrid planner remains the path generator and canonical
PathAudit remains authoritative.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from bisect import bisect_left, bisect_right
import hashlib
import math
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy import ndimage

from .regional_preference import PreferenceField
from .regional_preference_r1 import CroppedGrid, _huber_plateau
from .regional_preference_r2 import DEFAULT_R2_POLICY, RegionalPreferenceBuilderR2
from .semantic_map import canonical_hash


DEFAULT_R3_POLICY: Dict[str, Any] = {
    **DEFAULT_R2_POLICY,
    "guide_enabled": True,
    "guide_station_spacing_m": 0.40,
    "guide_lateral_sample_spacing_m": 0.10,
    "guide_station_snap_m": 0.20,
    "guide_max_lateral_probe_m": 12.0,
    "guide_min_segment_length_m": 2.0,
    "guide_max_lateral_slope": 0.85,
    "guide_max_curvature_1pm": 2.50,
    "guide_beam_width": 160,
    "guide_target_error_weight": 10.0,
    "guide_wrong_side_penalty": 8.0,
    "guide_lateral_rate_weight": 0.8,
    "guide_curvature_weight": 0.4,
    "guide_endpoint_anchor_length_m": 1.5,
    "guide_endpoint_anchor_weight": 3.0,
    "guide_tube_half_width_m": 0.30,
    "guide_tube_tolerance_m": 0.20,
    "guide_tube_huber_delta_m": 0.50,
    "guide_tube_error_scale_m": 1.50,
    "guide_cost_cap": 64,
    # The viability gate is deliberately expressed in the same physical
    # target band as the frozen online metric.  It certifies the directed
    # lane lattice before the soft guide is allowed to influence planning.
    "viability_target_error_max_m": 0.50,
    "viability_min_safe_station_ratio": 0.80,
    "viability_min_correct_side_ratio": 0.80,
    # A strict majority in the <=0.50 m band conservatively certifies the
    # frozen median-error <=0.50 m online gate.
    "viability_min_target_band_ratio": 0.50,
    "viability_certificate_only": False,
    # A circumscribed Jackal footprint plus the frozen 0.05 m safety margin.
    # It is used only to reject guide samples; final feasibility still belongs
    # to Smac and canonical PathAudit.
    "guide_static_clearance_m": math.hypot(0.255 + 0.05, 0.215 + 0.05),
}


@dataclass(frozen=True)
class _Station:
    s: float
    x: float
    y: float
    tx: float
    ty: float
    lane_label: int


@dataclass(frozen=True)
class _Candidate:
    offset: float
    x: float
    y: float
    row: int
    col: int
    error: float
    correct_side: bool


@dataclass(frozen=True)
class LaneViabilityGuide:
    mask: np.ndarray
    covered_mask: np.ndarray
    cost: np.ndarray
    diagnostics: Dict[str, Any]


@dataclass(frozen=True)
class DirectedLaneViabilityCertificate:
    """Fail-closed evidence for one start-to-goal, lane-relative target band."""

    gate_passed: bool
    classification: str
    diagnostics: Dict[str, Any]
    witness_segments: Tuple[Tuple[_Candidate, ...], ...] = ()


def _hospital_map_hash(hospital_map: Any) -> str:
    try:
        value = str(hospital_map.sha256)
        if value:
            return value
    except (AttributeError, OSError):
        pass
    occupancy = np.ascontiguousarray(hospital_map.occupancy)
    return canonical_hash({
        "resolution": float(hospital_map.resolution),
        "origin": [float(value) for value in hospital_map.origin],
        "shape": list(occupancy.shape),
        "occupancy_sha256": hashlib.sha256(occupancy.tobytes()).hexdigest(),
    })


def _densify_route(route: Sequence[Sequence[float]], spacing_m: float) -> list[_Station]:
    points = [(float(point[0]), float(point[1])) for point in route]
    points = [point for index, point in enumerate(points) if index == 0 or point != points[index - 1]]
    if len(points) < 2:
        return []
    spacing = max(float(spacing_m), 1.0e-3)
    segments = []
    cumulative = 0.0
    for first, second in zip(points, points[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        if length <= 1.0e-9:
            continue
        segments.append((cumulative, cumulative + length, first, dx, dy, length))
        cumulative += length
    if not segments:
        return []
    sample_s = np.arange(0.0, cumulative, spacing, dtype=np.float64).tolist()
    if not sample_s or cumulative - sample_s[-1] > 1.0e-9:
        sample_s.append(cumulative)
    stations: list[_Station] = []
    segment_index = 0
    for station_s in sample_s:
        while (
            segment_index + 1 < len(segments)
            and station_s > segments[segment_index][1] + 1.0e-9
        ):
            segment_index += 1
        start_s, end_s, first, dx, dy, length = segments[segment_index]
        distance = min(max(station_s - start_s, 0.0), end_s - start_s)
        ratio = distance / length
        stations.append(_Station(
            float(station_s), first[0] + ratio * dx, first[1] + ratio * dy,
            dx / length, dy / length, 0,
        ))
    return stations


def _unique_nearby_label(labels: np.ndarray, cell: Tuple[int, int], radius: int) -> int:
    row, col = cell
    direct = int(labels[row, col])
    if direct > 0:
        return direct
    row0, row1 = max(0, row - radius), min(labels.shape[0], row + radius + 1)
    col0, col1 = max(0, col - radius), min(labels.shape[1], col + radius + 1)
    values = sorted(int(value) for value in np.unique(labels[row0:row1, col0:col1]) if int(value) > 0)
    return values[0] if len(values) == 1 else 0


def _curvature(first: _Candidate, second: _Candidate, third: _Candidate) -> float:
    a = math.hypot(second.x - first.x, second.y - first.y)
    b = math.hypot(third.x - second.x, third.y - second.y)
    c = math.hypot(third.x - first.x, third.y - first.y)
    denominator = a * b * c
    if denominator <= 1.0e-12:
        return float("inf")
    twice_area = abs(
        (second.x - first.x) * (third.y - first.y)
        - (second.y - first.y) * (third.x - first.x)
    )
    return 2.0 * twice_area / denominator


def _candidate_cost(
    candidate: _Candidate, station: _Station, start_s: float, end_s: float,
    policy: Mapping[str, Any],
) -> float:
    edge_distance = min(station.s - start_s, end_s - station.s)
    anchor_length = max(float(policy["guide_endpoint_anchor_length_m"]), 1.0e-6)
    anchor = max(0.0, 1.0 - edge_distance / anchor_length)
    return (
        float(policy["guide_target_error_weight"]) * candidate.error * candidate.error
        + (0.0 if candidate.correct_side else float(policy["guide_wrong_side_penalty"]))
        + float(policy["guide_endpoint_anchor_weight"]) * anchor * candidate.offset * candidate.offset
    )


def _candidate_segment_valid(
    first: _Candidate, second: _Candidate, station: _Station,
    safe: np.ndarray, labels: np.ndarray, lane_label: int,
    max_lateral_slope: float,
) -> bool:
    ds = math.hypot(second.x - first.x, second.y - first.y)
    if ds <= 1.0e-6:
        return False
    forward = (
        (second.x - first.x) * station.tx
        + (second.y - first.y) * station.ty
    )
    if forward <= 0.02:
        return False
    if abs(second.offset - first.offset) > max_lateral_slope * ds + 0.05:
        return False
    # Integer Bresenham avoids allocating two NumPy arrays for each of the
    # hundreds of thousands of short lattice edges on a real 155 m route.
    col, row = first.col, first.row
    end_col, end_row = second.col, second.row
    delta_col, delta_row = abs(end_col - col), abs(end_row - row)
    step_col = 1 if col < end_col else -1
    step_row = 1 if row < end_row else -1
    error = delta_col - delta_row
    while True:
        if not bool(safe[row, col]) or int(labels[row, col]) != lane_label:
            return False
        if col == end_col and row == end_row:
            return True
        twice_error = 2 * error
        if twice_error > -delta_row:
            error -= delta_row
            col += step_col
        if twice_error < delta_col:
            error += delta_col
            row += step_row


def _build_adjacency(
    stations: Sequence[_Station], candidates: Sequence[Sequence[_Candidate]],
    safe: np.ndarray, labels: np.ndarray, policy: Mapping[str, Any],
) -> list[Dict[int, list[int]]]:
    """Build deterministic, corner/semantic-safe links between adjacent stations."""
    max_slope = float(policy["guide_max_lateral_slope"])
    adjacency: list[Dict[int, list[int]]] = []
    for station_index in range(len(stations) - 1):
        first_values = candidates[station_index]
        second_values = candidates[station_index + 1]
        second_offsets = [value.offset for value in second_values]
        station_delta = max(
            stations[station_index + 1].s - stations[station_index].s, 1.0e-6,
        )
        offset_limit = max_slope * station_delta + 0.10
        links: Dict[int, list[int]] = {}
        for first_index, first in enumerate(first_values):
            begin = bisect_left(second_offsets, first.offset - offset_limit)
            end = bisect_right(second_offsets, first.offset + offset_limit)
            valid = [
                second_index for second_index in range(begin, end)
                if _candidate_segment_valid(
                    first, second_values[second_index], stations[station_index],
                    safe, labels, stations[station_index].lane_label, max_slope,
                )
            ]
            if valid:
                links[first_index] = valid
        adjacency.append(links)
    return adjacency


def _longest_curvature_feasible_run(
    stations: Sequence[_Station], candidates: Sequence[Sequence[_Candidate]],
    safe: np.ndarray, labels: np.ndarray, policy: Mapping[str, Any],
) -> Tuple[int, Optional[Tuple[int, int]]]:
    """Return the exact longest station interval connected by the target lattice.

    Unlike the guide optimizer this reachability calculation is not beam
    pruned and has no preference objective.  It is therefore a certificate of
    the discretized geometry, not evidence that one particular optimizer
    happened to find a low-cost path.
    """
    if len(stations) < 2:
        return (1, (0, 0)) if len(stations) == 1 and candidates[0] else (0, None)
    adjacency = _build_adjacency(stations, candidates, safe, labels, policy)
    max_curvature = float(policy["guide_max_curvature_1pm"])
    states: Dict[Tuple[int, int], int] = {}
    best_length = 0
    best_interval: Optional[Tuple[int, int]] = None
    for station_index in range(1, len(stations)):
        next_states: Dict[Tuple[int, int], int] = {}
        if station_index >= 2:
            for previous_key, start_index in states.items():
                first = candidates[station_index - 2][previous_key[0]]
                second = candidates[station_index - 1][previous_key[1]]
                for third_index in adjacency[station_index - 1].get(previous_key[1], []):
                    third = candidates[station_index][third_index]
                    curvature = _curvature(first, second, third)
                    if not math.isfinite(curvature) or curvature > max_curvature + 1.0e-9:
                        continue
                    key = (previous_key[1], third_index)
                    if key not in next_states or start_index < next_states[key]:
                        next_states[key] = start_index
        # A valid target segment may begin after an earlier discontinuity.
        for first_index, seconds in adjacency[station_index - 1].items():
            for second_index in seconds:
                key = (first_index, second_index)
                start_index = station_index - 1
                if key not in next_states or start_index < next_states[key]:
                    next_states[key] = start_index
        states = next_states
        for start_index in states.values():
            length = station_index - start_index + 1
            interval = (start_index, station_index)
            if length > best_length or (
                length == best_length and (best_interval is None or interval < best_interval)
            ):
                best_length = length
                best_interval = interval
    if best_length == 0:
        for index, values in enumerate(candidates):
            if values:
                return 1, (index, index)
    return best_length, best_interval


def _pareto_metric_pairs_on_full_path(
    stations: Sequence[_Station], candidates: Sequence[Sequence[_Candidate]],
    safe: np.ndarray, labels: np.ndarray, policy: Mapping[str, Any],
    target_error_max_m: float,
    adjacency: Optional[Sequence[Mapping[int, Sequence[int]]]] = None,
) -> Tuple[list[Tuple[int, int]], bool, Optional[list[_Candidate]], Tuple[int, int]]:
    """Return nondominated (target-band, correct-side) counts on full paths.

    Short departures from the target band are allowed because the frozen
    online gates are aggregate metrics.  Every represented path still spans
    the whole semantic lane run and every edge remains safe, forward,
    same-instance and curvature bounded.  No beam or cost objective is used.
    """
    if len(stations) < 3 or any(not values for values in candidates):
        return [], False, None, (0, 0)
    if adjacency is None:
        adjacency = _build_adjacency(stations, candidates, safe, labels, policy)
    max_curvature = float(policy["guide_max_curvature_1pm"])
    start_s, end_s = stations[0].s, stations[-1].s

    def metric_value(candidate: _Candidate) -> Tuple[int, int]:
        correct = int(candidate.correct_side)
        target = int(
            candidate.correct_side
            and candidate.error <= float(target_error_max_m) + 1.0e-9
        )
        return target, correct

    def pareto(values: Mapping[Tuple[int, int], float]) -> Dict[Tuple[int, int], float]:
        result: Dict[Tuple[int, int], float] = {}
        best_correct = -1
        for target, correct in sorted(values, reverse=True):
            if correct > best_correct:
                result[(target, correct)] = values[(target, correct)]
                best_correct = correct
        return result

    states: Dict[Tuple[int, int], Dict[Tuple[int, int], float]] = {}
    for first_index, first in enumerate(candidates[0]):
        for second_index in adjacency[0].get(first_index, []):
            second = candidates[1][second_index]
            first_target, first_correct = metric_value(first)
            second_target, second_correct = metric_value(second)
            rate = (second.offset - first.offset) / max(stations[1].s - stations[0].s, 1.0e-6)
            pair = (first_target + second_target, first_correct + second_correct)
            states[(first_index, second_index)] = {pair: (
                _candidate_cost(first, stations[0], start_s, end_s, policy)
                + _candidate_cost(second, stations[1], start_s, end_s, policy)
                + float(policy["guide_lateral_rate_weight"]) * rate * rate
            )}
    if not states:
        return [], False, None, (0, 0)
    parents: Dict[
        int, Dict[Tuple[Tuple[int, int], Tuple[int, int]], Tuple[Tuple[int, int], Tuple[int, int]]]
    ] = {}
    for station_index in range(2, len(stations)):
        pending: Dict[Tuple[int, int], Dict[Tuple[int, int], float]] = {}
        pending_parents: Dict[
            Tuple[Tuple[int, int], Tuple[int, int]], Tuple[Tuple[int, int], Tuple[int, int]]
        ] = {}
        ds_station = max(stations[station_index].s - stations[station_index - 1].s, 1.0e-6)
        for previous_key, metric_costs in states.items():
            first = candidates[station_index - 2][previous_key[0]]
            second = candidates[station_index - 1][previous_key[1]]
            for third_index in adjacency[station_index - 1].get(previous_key[1], []):
                third = candidates[station_index][third_index]
                curvature = _curvature(first, second, third)
                if not math.isfinite(curvature) or curvature > max_curvature + 1.0e-9:
                    continue
                key = (previous_key[1], third_index)
                third_target, third_correct = metric_value(third)
                rate = (third.offset - second.offset) / ds_station
                step_cost = (
                    _candidate_cost(third, stations[station_index], start_s, end_s, policy)
                    + float(policy["guide_lateral_rate_weight"]) * rate * rate
                    + float(policy["guide_curvature_weight"]) * curvature * curvature
                )
                target_values = pending.setdefault(key, {})
                for previous_pair, previous_cost in metric_costs.items():
                    pair = (
                        previous_pair[0] + third_target,
                        previous_pair[1] + third_correct,
                    )
                    cost = previous_cost + step_cost
                    if pair not in target_values or cost < target_values[pair] - 1.0e-12:
                        target_values[pair] = cost
                        pending_parents[(key, pair)] = (previous_key, previous_pair)
        if not pending:
            return [], False, None, (0, 0)
        states = {key: pareto(values) for key, values in pending.items()}
        parents[station_index] = {
            (key, pair): pending_parents[(key, pair)]
            for key, values in states.items() for pair in values
        }

    final_values: Dict[Tuple[int, int], float] = {}
    final_locations: Dict[Tuple[int, int], Tuple[int, int]] = {}
    for key, values in states.items():
        for pair, cost in values.items():
            if pair not in final_values or cost < final_values[pair] - 1.0e-12:
                final_values[pair] = cost
                final_locations[pair] = key
    final_values = pareto(final_values)
    station_count = len(stations)
    required_target = math.floor(
        float(policy["viability_min_target_band_ratio"]) * station_count
    ) + 1
    required_correct = math.ceil(
        float(policy["viability_min_correct_side_ratio"]) * station_count - 1.0e-12
    )
    passing = [
        pair for pair in final_values
        if pair[0] >= required_target and pair[1] >= required_correct
    ]
    selected_pair = min(
        passing,
        key=lambda pair: (final_values[pair], -pair[0], -pair[1], pair),
        default=max(
            final_values,
            key=lambda pair: (
                min(pair[0] / max(required_target, 1), pair[1] / max(required_correct, 1)),
                pair[0], pair[1], -final_values[pair],
            ),
            default=(0, 0),
        ),
    )
    key = final_locations[selected_pair]
    indices = [0] * station_count
    indices[-2], indices[-1] = key
    pair = selected_pair
    for station_index in range(station_count - 1, 1, -1):
        previous_key, previous_pair = parents[station_index][(key, pair)]
        indices[station_index - 2] = previous_key[0]
        key, pair = previous_key, previous_pair
    witness = [
        candidates[index][candidate_index]
        for index, candidate_index in enumerate(indices)
    ]
    return list(final_values), True, witness, selected_pair


def _solve_segment(
    stations: Sequence[_Station], candidates: Sequence[Sequence[_Candidate]],
    safe: np.ndarray, labels: np.ndarray, policy: Mapping[str, Any],
    adjacency: Optional[Sequence[Mapping[int, Sequence[int]]]] = None,
) -> Optional[list[_Candidate]]:
    if len(stations) < 3 or any(not values for values in candidates):
        return None
    max_curvature = float(policy["guide_max_curvature_1pm"])
    start_s, end_s = stations[0].s, stations[-1].s
    if adjacency is None:
        adjacency = _build_adjacency(stations, candidates, safe, labels, policy)
    states: Dict[Tuple[int, int], float] = {}
    for first_index, first in enumerate(candidates[0]):
        first_cost = _candidate_cost(first, stations[0], start_s, end_s, policy)
        for second_index in adjacency[0].get(first_index, []):
            second = candidates[1][second_index]
            rate = (second.offset - first.offset) / max(stations[1].s - stations[0].s, 1.0e-6)
            states[(first_index, second_index)] = (
                first_cost
                + _candidate_cost(second, stations[1], start_s, end_s, policy)
                + float(policy["guide_lateral_rate_weight"]) * rate * rate
            )
    if not states:
        return None
    parents: Dict[int, Dict[Tuple[int, int], Tuple[int, int]]] = {}
    beam_width = max(1, int(policy["guide_beam_width"]))
    for station_index in range(2, len(stations)):
        if len(states) > beam_width:
            states = dict(sorted(states.items(), key=lambda item: (item[1], item[0]))[:beam_width])
        next_states: Dict[Tuple[int, int], float] = {}
        parent_layer: Dict[Tuple[int, int], Tuple[int, int]] = {}
        ds_station = max(stations[station_index].s - stations[station_index - 1].s, 1.0e-6)
        for previous_key, prior_cost in sorted(states.items()):
            first = candidates[station_index - 2][previous_key[0]]
            second = candidates[station_index - 1][previous_key[1]]
            for third_index in adjacency[station_index - 1].get(previous_key[1], []):
                third = candidates[station_index][third_index]
                curvature = _curvature(first, second, third)
                if not math.isfinite(curvature) or curvature > max_curvature + 1.0e-9:
                    continue
                rate = (third.offset - second.offset) / ds_station
                cost = (
                    prior_cost
                    + _candidate_cost(third, stations[station_index], start_s, end_s, policy)
                    + float(policy["guide_lateral_rate_weight"]) * rate * rate
                    + float(policy["guide_curvature_weight"]) * curvature * curvature
                )
                key = (previous_key[1], third_index)
                if key not in next_states or cost < next_states[key] - 1.0e-12:
                    next_states[key] = cost
                    parent_layer[key] = previous_key
        if not next_states:
            return None
        states = next_states
        parents[station_index] = parent_layer
    best_key = min(states, key=lambda key: (states[key], key))
    indices = [0] * len(stations)
    indices[-2], indices[-1] = best_key
    key = best_key
    for station_index in range(len(stations) - 1, 1, -1):
        previous_key = parents[station_index][key]
        indices[station_index - 2] = previous_key[0]
        key = previous_key
    return [candidates[index][candidate_index] for index, candidate_index in enumerate(indices)]


def _certify_directed_target(
    runs: Sequence[Sequence[_Station]],
    run_candidates: Sequence[Sequence[Sequence[_Candidate]]],
    run_adjacencies: Sequence[Sequence[Mapping[int, Sequence[int]]]],
    safe: np.ndarray,
    labels: np.ndarray,
    policy: Mapping[str, Any],
    *,
    map_hash: str,
    semantic_map_hash: str,
    route_hash: str,
    roi_hash: str,
    lane_instance_ids: Mapping[int, str],
) -> DirectedLaneViabilityCertificate:
    """Certify local availability and the two frozen online direction metrics."""
    started = time.monotonic_ns()
    target_error = float(policy["viability_target_error_max_m"])
    minimum_safe = float(policy["viability_min_safe_station_ratio"])
    minimum_correct = float(policy["viability_min_correct_side_ratio"])
    minimum_target = float(policy["viability_min_target_band_ratio"])
    run_records: list[Dict[str, Any]] = []
    lane_station_count = 0
    safe_station_count = 0
    target_station_count = 0
    selected_target_count = 0
    selected_correct_count = 0
    pareto_pair_count = 0
    every_run_has_full_path = True
    every_run_meets_metrics = True
    witness_segments: list[Tuple[_Candidate, ...]] = []
    for run_index, (run, candidates, adjacency) in enumerate(
        zip(runs, run_candidates, run_adjacencies)
    ):
        target_candidates = [
            [
                value for value in values
                if value.correct_side and value.error <= target_error + 1.0e-9
            ]
            for values in candidates
        ]
        run_station_count = len(run)
        run_safe_count = sum(bool(values) for values in candidates)
        run_target_count = sum(bool(values) for values in target_candidates)
        run_required_target = math.floor(minimum_target * run_station_count) + 1
        run_required_correct = math.ceil(minimum_correct * run_station_count - 1.0e-12)
        preferred_witness = _solve_segment(
            run, candidates, safe, labels, policy, adjacency,
        )
        preferred_pair = (
            sum(
                value.correct_side and value.error <= target_error + 1.0e-9
                for value in preferred_witness
            ),
            sum(value.correct_side for value in preferred_witness),
        ) if preferred_witness is not None else (0, 0)
        preferred_passed = bool(
            preferred_witness is not None
            and preferred_pair[0] >= run_required_target
            and preferred_pair[1] >= run_required_correct
        )
        if preferred_passed:
            metric_pairs = [preferred_pair]
            full_path = True
            witness = preferred_witness
            selected_pair = preferred_pair
            search_mode = "CONSTRUCTIVE_COST_WITNESS"
        else:
            metric_pairs, full_path, witness, selected_pair = _pareto_metric_pairs_on_full_path(
                run, candidates, safe, labels, policy, target_error, adjacency,
            )
            search_mode = "EXACT_PARETO_FALLBACK"
        best_target = max((pair[0] for pair in metric_pairs), default=0)
        best_correct = max((pair[1] for pair in metric_pairs), default=0)
        every_run_has_full_path &= full_path
        run_meets_metrics = bool(
            full_path
            and selected_pair[0] >= run_required_target
            and selected_pair[1] >= run_required_correct
        )
        every_run_meets_metrics &= run_meets_metrics
        selected_target_count += selected_pair[0]
        selected_correct_count += selected_pair[1]
        pareto_pair_count += len(metric_pairs)
        if witness is not None:
            witness_segments.append(tuple(witness))
        lane_station_count += run_station_count
        safe_station_count += run_safe_count
        target_station_count += run_target_count
        label = int(run[0].lane_label) if run else 0
        run_record = {
            "run_index": run_index,
            "lane_label": label,
            "lane_semantic_id": lane_instance_ids.get(label, ""),
            "start_s_m": float(run[0].s) if run else None,
            "end_s_m": float(run[-1].s) if run else None,
            "length_m": float(run[-1].s - run[0].s) if len(run) >= 2 else 0.0,
            "station_count": run_station_count,
            "safe_candidate_station_count": run_safe_count,
            "target_candidate_station_count": run_target_count,
            "target_candidate_station_ratio": (
                float(run_target_count / run_station_count) if run_station_count else 0.0
            ),
            "full_run_curvature_feasible": full_path,
            "maximum_target_station_count_on_full_path": best_target,
            "maximum_target_station_ratio_on_full_path": (
                float(best_target / run_station_count) if run_station_count else 0.0
            ),
            "maximum_correct_side_station_count_on_full_path": best_correct,
            "maximum_correct_side_station_ratio_on_full_path": (
                float(best_correct / run_station_count) if run_station_count else 0.0
            ),
            "pareto_metric_pair_count": len(metric_pairs),
            "certificate_search_mode": search_mode,
            "selected_target_station_count": selected_pair[0],
            "selected_correct_side_station_count": selected_pair[1],
            "selected_metric_pair_passed": run_meets_metrics,
        }
        run_records.append(run_record)

    safe_ratio = float(safe_station_count / lane_station_count) if lane_station_count else 0.0
    target_ratio = float(target_station_count / lane_station_count) if lane_station_count else 0.0
    required_target_count = math.floor(minimum_target * lane_station_count) + 1
    required_correct_count = math.ceil(minimum_correct * lane_station_count - 1.0e-12)
    full_path_target_count = selected_target_count
    full_path_correct_count = selected_correct_count
    full_path_target_ratio = (
        float(full_path_target_count / lane_station_count) if lane_station_count else 0.0
    )
    full_path_correct_ratio = (
        float(full_path_correct_count / lane_station_count) if lane_station_count else 0.0
    )
    if lane_station_count == 0:
        classification = "NO_ELIGIBLE_LANE_RUN"
    elif safe_ratio < minimum_safe:
        classification = "DIRECTED_SAFE_LANE_FRAGMENTED"
    elif not every_run_has_full_path:
        classification = "DIRECTED_LANE_CURVATURE_DISCONNECTED"
    elif target_ratio <= minimum_target:
        classification = "DIRECTED_TARGET_LOCALLY_UNAVAILABLE"
    elif not every_run_meets_metrics:
        classification = "DIRECTED_ONLINE_METRIC_PAIR_INFEASIBLE"
    else:
        classification = "DIRECTED_TARGET_VIABLE"
    gate_passed = classification == "DIRECTED_TARGET_VIABLE"
    policy_binding = {
        key: policy[key] for key in (
            "guide_station_spacing_m",
            "guide_lateral_sample_spacing_m",
            "guide_max_lateral_probe_m",
            "guide_min_segment_length_m",
            "guide_max_lateral_slope",
            "guide_max_curvature_1pm",
            "guide_static_clearance_m",
            "viability_target_error_max_m",
            "viability_min_safe_station_ratio",
            "viability_min_correct_side_ratio",
            "viability_min_target_band_ratio",
        )
    }
    diagnostics: Dict[str, Any] = {
        "viability_schema_version": "2A-V2-directed-lane-viability-certificate-v1",
        "viability_gate_passed": gate_passed,
        "viability_classification": classification,
        "viability_evidence_scope": "discretized_static_lane_lattice_not_final_vehicle_path",
        "viability_direction_source": "query_start_to_goal_oriented_l1_route_tangent",
        "viability_target_band_definition": (
            f"same_lane_instance AND d_right<=d_left AND abs(d_right-0.40)<={target_error:.3f}m"
        ),
        "viability_lane_station_count": lane_station_count,
        "viability_safe_candidate_station_count": safe_station_count,
        "viability_safe_candidate_station_ratio": safe_ratio,
        "viability_target_candidate_station_count": target_station_count,
        "viability_target_candidate_station_ratio": target_ratio,
        "viability_full_path_target_station_count": full_path_target_count,
        "viability_full_path_target_station_ratio": full_path_target_ratio,
        "viability_full_path_correct_side_station_count": full_path_correct_count,
        "viability_full_path_correct_side_station_ratio": full_path_correct_ratio,
        "viability_required_target_station_count": required_target_count,
        "viability_required_correct_side_station_count": required_correct_count,
        "viability_pareto_metric_pair_count": pareto_pair_count,
        "viability_all_lane_runs_meet_metrics": every_run_meets_metrics,
        "viability_min_safe_station_ratio": minimum_safe,
        "viability_min_correct_side_ratio": minimum_correct,
        "viability_min_target_band_ratio": minimum_target,
        "viability_lane_instance_ids": sorted({
            record["lane_semantic_id"] for record in run_records if record["lane_semantic_id"]
        }),
        "viability_lane_runs": run_records,
        "viability_map_hash": map_hash,
        "viability_semantic_map_hash": semantic_map_hash,
        "viability_route_hash": route_hash,
        "viability_roi_hash": roi_hash,
        "viability_policy_hash": canonical_hash(policy_binding),
        "viability_build_ms": (time.monotonic_ns() - started) / 1.0e6,
    }
    certificate_content = {
        key: value for key, value in diagnostics.items()
        if key not in {"viability_build_ms", "viability_certificate_hash"}
    }
    diagnostics["viability_certificate_hash"] = canonical_hash(certificate_content)
    return DirectedLaneViabilityCertificate(
        gate_passed, classification, diagnostics, tuple(witness_segments),
    )


def build_lane_viability_guide(
    hospital_map: Any, base: PreferenceField, lane_labels: np.ndarray,
    hard_footprint_mask: np.ndarray, route: Sequence[Sequence[float]],
    allowed_mask: np.ndarray, policy: Mapping[str, Any],
    lane_instance_ids: Optional[Mapping[int, str]] = None,
    semantic_map_hash: str = "",
) -> LaneViabilityGuide:
    """Build a continuous guide tube without changing any hard constraint."""
    started = time.monotonic_ns()
    shape = tuple(int(value) for value in allowed_mask.shape)
    empty = np.zeros(shape, dtype=bool)
    empty_cost = np.zeros(shape, dtype=np.uint8)
    if not bool(policy.get("guide_enabled", True)) or len(route) < 2:
        return LaneViabilityGuide(empty, empty.copy(), empty_cost, {
            "guide_status": "DISABLED_OR_EMPTY_ROUTE", "guide_build_ms": 0.0,
        })

    error_grid = base.lane_error_m
    correct_grid = base.lane_correct_side
    if isinstance(error_grid, CroppedGrid):
        bounds = error_grid.bounds
        error = np.asarray(error_grid.values, dtype=np.float32)
        correct = np.asarray(correct_grid.values, dtype=bool)
    else:
        bounds = (0, shape[0], 0, shape[1])
        error = np.asarray(error_grid, dtype=np.float32)
        correct = np.asarray(correct_grid, dtype=bool)
    row0, row1, col0, col1 = bounds
    target = np.s_[row0:row1, col0:col1]
    labels = np.asarray(lane_labels[target], dtype=np.int32)
    allowed = np.asarray(allowed_mask[target], dtype=bool)
    hard = np.asarray(hard_footprint_mask[target], dtype=bool)
    static_clearance = np.asarray(hospital_map.distance_m[target], dtype=np.float32)
    safe = allowed & ~hard & (static_clearance >= float(policy["guide_static_clearance_m"]))

    stations = _densify_route(route, float(policy["guide_station_spacing_m"]))
    snap_cells = max(0, int(math.ceil(
        float(policy["guide_station_snap_m"]) / float(hospital_map.resolution)
    )))
    labelled_stations: list[_Station] = []
    for station in stations:
        cell = hospital_map.world_to_cell(station.x, station.y)
        label = 0
        if cell is not None and row0 <= cell[0] < row1 and col0 <= cell[1] < col1:
            local = (cell[0] - row0, cell[1] - col0)
            label = _unique_nearby_label(labels, local, snap_cells)
        labelled_stations.append(replace(station, lane_label=label))

    # Runs are never allowed to cross a semantic lane-instance boundary or a
    # junction/parking gap.  That keeps direction and preference propagation
    # local to the actual route-lane instance selected by L1.
    runs: list[list[_Station]] = []
    current: list[_Station] = []
    for station in labelled_stations:
        if station.lane_label <= 0 or (current and station.lane_label != current[-1].lane_label):
            if current:
                runs.append(current)
            current = []
        if station.lane_label > 0:
            current.append(station)
    if current:
        runs.append(current)

    max_probe = float(policy["guide_max_lateral_probe_m"])
    lateral_step = max(float(policy["guide_lateral_sample_spacing_m"]), hospital_map.resolution)
    offsets = np.arange(-max_probe, max_probe + 0.5 * lateral_step, lateral_step, dtype=np.float64)
    candidate_started = time.monotonic_ns()
    selected_segments: list[Tuple[int, list[_Candidate]]] = []
    failed_runs = 0
    candidate_count = 0
    target_candidate_station_count = 0
    candidate_station_count = 0
    longest_target_station_run = 0
    eligible_runs: list[list[_Station]] = []
    eligible_run_candidates: list[list[list[_Candidate]]] = []
    min_segment_length = float(policy["guide_min_segment_length_m"])
    for run in runs:
        if len(run) < 3 or run[-1].s - run[0].s < min_segment_length:
            continue
        per_station: list[list[_Candidate]] = []
        for station in run:
            values: list[_Candidate] = []
            seen_cells = set()
            # Positive offset is the start->goal right normal (ty, -tx).
            for offset in offsets:
                x = station.x + float(offset) * station.ty
                y = station.y - float(offset) * station.tx
                cell = hospital_map.world_to_cell(x, y)
                if cell is None or not (row0 <= cell[0] < row1 and col0 <= cell[1] < col1):
                    continue
                local = (cell[0] - row0, cell[1] - col0)
                if local in seen_cells or int(labels[local]) != station.lane_label or not bool(safe[local]):
                    continue
                value = float(error[local])
                if not math.isfinite(value):
                    continue
                seen_cells.add(local)
                values.append(_Candidate(
                    float(offset), x, y, local[0], local[1], value, bool(correct[local]),
                ))
            values.sort(key=lambda item: (item.offset, item.row, item.col))
            candidate_count += len(values)
            per_station.append(values)

        eligible_runs.append(run)
        eligible_run_candidates.append(per_station)

        candidate_station_count += sum(bool(values) for values in per_station)
        current_target_run = 0
        for values in per_station:
            has_target = any(value.correct_side and value.error <= 0.50 for value in values)
            target_candidate_station_count += int(has_target)
            current_target_run = current_target_run + 1 if has_target else 0
            longest_target_station_run = max(longest_target_station_run, current_target_run)
    candidate_build_ms = (time.monotonic_ns() - candidate_started) / 1.0e6
    adjacency_started = time.monotonic_ns()
    run_adjacencies = [
        _build_adjacency(run, candidates, safe, labels, policy)
        for run, candidates in zip(eligible_runs, eligible_run_candidates)
    ]
    adjacency_build_ms = (time.monotonic_ns() - adjacency_started) / 1.0e6

    route_hash = canonical_hash([[float(point[0]), float(point[1])] for point in route])
    roi_hash = hashlib.sha256(
        np.packbits(np.ascontiguousarray(allowed_mask, dtype=np.uint8)).tobytes()
    ).hexdigest()
    certificate = _certify_directed_target(
        eligible_runs, eligible_run_candidates, run_adjacencies,
        safe, labels, policy,
        map_hash=_hospital_map_hash(hospital_map),
        semantic_map_hash=str(semantic_map_hash),
        route_hash=route_hash,
        roi_hash=roi_hash,
        lane_instance_ids=dict(lane_instance_ids or {}),
    )
    certificate_runs = certificate.diagnostics["viability_lane_runs"]
    target_only_full_run_count = sum(
        bool(record["full_run_curvature_feasible"])
        and float(record["maximum_target_station_ratio_on_full_path"]) > float(
            policy["viability_min_target_band_ratio"]
        )
        for record in certificate_runs
    )
    target_only_failed_run_count = len(certificate_runs) - target_only_full_run_count

    guide_solve_started = time.monotonic_ns()
    if (
        not bool(policy.get("viability_certificate_only", False))
        and certificate.gate_passed
    ):
        selected_segments.extend(
            (run[0].lane_label, list(witness))
            for run, witness in zip(eligible_runs, certificate.witness_segments)
        )
    failed_runs = max(0, len(eligible_runs) - len(selected_segments))
    guide_solve_ms = (time.monotonic_ns() - guide_solve_started) / 1.0e6

    raster_started = time.monotonic_ns()
    guide_local = np.zeros(error.shape, dtype=np.uint8)
    covered_local = np.zeros(error.shape, dtype=bool)
    guide_points: list[list[float]] = []
    guide_polylines: list[list[list[float]]] = []
    curvatures: list[float] = []
    guide_errors: list[float] = []
    guide_correct: list[bool] = []
    selected_labels = set()
    for label, selected in selected_segments:
        selected_labels.add(int(label))
        segment_points = [[candidate.x, candidate.y] for candidate in selected]
        guide_polylines.append(segment_points)
        for first, second in zip(selected, selected[1:]):
            cv2.line(
                guide_local, (first.col, first.row), (second.col, second.row),
                1, thickness=1, lineType=cv2.LINE_8,
            )
        for candidate in selected:
            guide_local[candidate.row, candidate.col] = 1
            guide_points.append([candidate.x, candidate.y])
            guide_errors.append(candidate.error)
            guide_correct.append(candidate.correct_side)
        curvatures.extend(
            _curvature(first, second, third)
            for first, second, third in zip(selected, selected[1:], selected[2:])
        )

    if selected_labels:
        covered_local = safe & np.isin(labels, sorted(selected_labels)) & np.isfinite(error)
    tube_radius_cells = max(1, int(math.ceil(
        float(policy["guide_tube_half_width_m"]) / hospital_map.resolution
    )))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * tube_radius_cells + 1, 2 * tube_radius_cells + 1),
    )
    tube = cv2.dilate(guide_local, kernel).astype(bool) & covered_local
    if np.any(guide_local):
        distance = ndimage.distance_transform_edt(
            guide_local == 0, sampling=float(hospital_map.resolution),
        ).astype(np.float32)
        normalized = _huber_plateau(
            distance,
            tolerance=float(policy["guide_tube_tolerance_m"]),
            delta=float(policy["guide_tube_huber_delta_m"]),
            scale=float(policy["guide_tube_error_scale_m"]),
        )
        guide_cost_local = np.clip(
            float(policy["guide_cost_cap"]) * normalized,
            0.0, float(policy["guide_cost_cap"]),
        ).astype(np.uint8)
        guide_cost_local[~covered_local] = 0
    else:
        guide_cost_local = np.zeros(error.shape, dtype=np.uint8)

    mask = np.zeros(shape, dtype=bool)
    covered = np.zeros(shape, dtype=bool)
    cost = np.zeros(shape, dtype=np.uint8)
    mask[target] = tube
    covered[target] = covered_local
    cost[target] = guide_cost_local
    guide_hash = hashlib.sha256(np.ascontiguousarray(guide_local).tobytes()).hexdigest()
    rasterize_ms = (time.monotonic_ns() - raster_started) / 1.0e6
    diagnostics = {
        "guide_status": "BUILT" if selected_segments else "NO_FEASIBLE_GUIDE",
        "guide_method": "lane_relative_station_lateral_curvature_lattice_v1",
        "guide_is_final_path": False,
        "guide_fail_closed_behavior": "retain_r2_field",
        "guide_station_count": len(labelled_stations),
        "guide_labelled_station_count": sum(station.lane_label > 0 for station in labelled_stations),
        "guide_lane_run_count": len(runs),
        "guide_feasible_segment_count": len(selected_segments),
        "guide_failed_lane_run_count": failed_runs,
        "guide_candidate_count": candidate_count,
        "guide_candidate_build_ms": candidate_build_ms,
        "guide_adjacency_build_ms": adjacency_build_ms,
        "guide_solve_ms": guide_solve_ms,
        "guide_rasterize_ms": rasterize_ms,
        "guide_candidate_station_count": candidate_station_count,
        "guide_target_candidate_station_count": target_candidate_station_count,
        "guide_target_candidate_station_ratio": (
            float(target_candidate_station_count / candidate_station_count)
            if candidate_station_count else 0.0
        ),
        "guide_longest_target_station_run": longest_target_station_run,
        "guide_longest_target_station_run_ratio": (
            float(longest_target_station_run / candidate_station_count)
            if candidate_station_count else 0.0
        ),
        "guide_target_only_full_lane_run_count": target_only_full_run_count,
        "guide_target_only_failed_lane_run_count": target_only_failed_run_count,
        "guide_centerline_cell_count": int(np.count_nonzero(guide_local)),
        "guide_tube_cell_count": int(np.count_nonzero(tube)),
        "guide_covered_cell_count": int(np.count_nonzero(covered_local)),
        "guide_correct_side_ratio": float(np.mean(guide_correct)) if guide_correct else None,
        "guide_target_error_p50_m": float(np.median(guide_errors)) if guide_errors else None,
        "guide_max_curvature_1pm": float(max(curvatures, default=0.0)),
        "guide_curvature_limit_1pm": float(policy["guide_max_curvature_1pm"]),
        "guide_lane_instance_ids": sorted(int(value) for value in selected_labels),
        "guide_hash": guide_hash,
        "guide_polyline_world": guide_points,
        "guide_polylines_world": guide_polylines,
        "guide_build_ms": (time.monotonic_ns() - started) / 1.0e6,
        **certificate.diagnostics,
    }
    diagnostics["guide_acceptance_gate_passed"] = bool(
        diagnostics["guide_status"] == "BUILT"
        and certificate.gate_passed
        and (diagnostics["guide_correct_side_ratio"] or 0.0) >= 0.80
        and (diagnostics["guide_target_error_p50_m"] or float("inf")) <= 0.50
        and diagnostics["guide_max_curvature_1pm"] <= float(policy["guide_max_curvature_1pm"])
    )
    return LaneViabilityGuide(mask, covered, cost, diagnostics)


class RegionalPreferenceBuilderR3(RegionalPreferenceBuilderR2):
    """Replace fragmented lane minima with a verified continuous soft tube."""

    def __init__(
        self, hospital_map: Any, raster: Any, *, policy: Optional[Mapping[str, Any]] = None,
        semantic_map: Any = None,
    ) -> None:
        merged = {**DEFAULT_R3_POLICY, **dict(policy or {})}
        super().__init__(hospital_map, raster, policy=merged, semantic_map=semantic_map)
        self.policy = merged
        self.policy_hash = canonical_hash(self.policy)

    def build(
        self, route: Sequence[Sequence[float]], *, goal: Optional[Sequence[float]] = None,
        allowed_mask: Optional[np.ndarray] = None, relaxation_level: str = "R0",
        planning_preference_enabled: bool = True,
        route_diagnostics: Optional[Mapping[str, Any]] = None,
    ) -> PreferenceField:
        base = super().build(
            route, goal=goal, allowed_mask=allowed_mask,
            relaxation_level=relaxation_level,
            planning_preference_enabled=planning_preference_enabled,
            route_diagnostics=route_diagnostics,
        )
        shape = (int(self.raster.height), int(self.raster.width))
        allowed = np.ones(shape, dtype=bool) if allowed_mask is None else np.asarray(allowed_mask, dtype=bool)
        guide = build_lane_viability_guide(
            self.hospital_map, base, self._lane_labels,
            np.asarray(self.raster.hard_footprint_mask, dtype=bool),
            route, allowed, self.policy, self._lane_instance_ids,
            str(getattr(self.raster, "semantic_map_hash", "")),
        )
        cost = np.asarray(base.cost, dtype=np.uint8).copy()
        active = np.asarray(base.active_lateral_mask, dtype=bool).copy()
        guide_applied = bool(
            planning_preference_enabled and relaxation_level == "R0"
            and guide.diagnostics.get("guide_acceptance_gate_passed") is True
        )
        if guide_applied:
            cost[guide.covered_mask] = guide.cost[guide.covered_mask]
            active[guide.covered_mask] = True
        diagnostics = {
            **base.diagnostics,
            **guide.diagnostics,
            "geometry_revision": "r3_lane_relative_viability_guide",
            "guide_applied_to_planning": guide_applied,
            "guide_soft_only": True,
            "guide_cost_cap": int(self.policy["guide_cost_cap"]),
        }
        return replace(
            base, cost=cost, active_lateral_mask=active,
            policy_hash=self.policy_hash, diagnostics=diagnostics,
        )


__all__ = [
    "DEFAULT_R3_POLICY", "DirectedLaneViabilityCertificate", "LaneViabilityGuide",
    "RegionalPreferenceBuilderR3",
    "build_lane_viability_guide",
]
