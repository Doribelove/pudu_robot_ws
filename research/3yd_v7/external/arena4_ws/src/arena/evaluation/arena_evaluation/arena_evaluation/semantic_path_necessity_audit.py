"""Fail-closed geometry screen for unnecessary path excursions.

This module deliberately does not decide whether a detected detour is truly
necessary.  It identifies evidence that requires a separate, map-bound
shortcut or obstacle certificate before a path can be accepted:

* crossing the directed goal plane and continuing beyond it;
* cumulative regression along the terminal route direction;
* nonlocal, oppositely directed visits whose real padded Jackal footprints
  overlap; and
* target-band samples collected beyond the directed goal plane.

The screen is independent of semantic thresholds and planner scoring.  A
missing or malformed input fails closed.  ``target_mask`` is expected to use
the same samples as ``path`` (normally the frozen 0.025 m semantic samples).
"""
from __future__ import annotations

import heapq
import math
from typing import Any, Sequence

import numpy as np


FOOTPRINT_HALF_LENGTH_M = 0.265
FOOTPRINT_HALF_WIDTH_M = 0.225
FOOTPRINT_LENGTH_M = 2.0 * FOOTPRINT_HALF_LENGTH_M
FOOTPRINT_AREA_M2 = 4.0 * FOOTPRINT_HALF_LENGTH_M * FOOTPRINT_HALF_WIDTH_M
MINIMUM_TURNING_RADIUS_M = 0.40
# A route farther away than the frozen R2 maximum endpoint-transition extent
# cannot truthfully define the audited path's terminal direction.  A nonzero
# goal gap must additionally lie on the terminal route ray, within one padded
# footprint half-width laterally; the 6 m value alone is not treated as proof.
ROUTE_ENDPOINT_BINDING_LIMIT_M = 6.0
ROUTE_GOAL_RAY_LATERAL_LIMIT_M = FOOTPRINT_HALF_WIDTH_M
# Two footprint poses are not separate visits until enough arc length has
# elapsed to execute the shortest legal 180-degree heading reversal.
MIN_NONLOCAL_ARC_SEPARATION_M = math.pi * MINIMUM_TURNING_RADIUS_M
MAX_GEOMETRY_SAMPLE_SPACING_M = 0.025
GEOMETRY_EPS = 1.0e-9
MAX_RECORDED_OVERLAP_EVENTS = 128


def _cross(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _wrap(angle: float | np.ndarray):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _footprint_polygon(pose: Sequence[float]) -> np.ndarray:
    """Return the fixed padded Jackal rectangle in counter-clockwise order."""
    x, y, yaw = map(float, pose[:3])
    corners = np.asarray((
        (-FOOTPRINT_HALF_LENGTH_M, -FOOTPRINT_HALF_WIDTH_M),
        (FOOTPRINT_HALF_LENGTH_M, -FOOTPRINT_HALF_WIDTH_M),
        (FOOTPRINT_HALF_LENGTH_M, FOOTPRINT_HALF_WIDTH_M),
        (-FOOTPRINT_HALF_LENGTH_M, FOOTPRINT_HALF_WIDTH_M),
    ))
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray(((c, -s), (s, c)))
    return corners @ rotation.T + np.asarray((x, y))


def _line_intersection(start: np.ndarray, end: np.ndarray,
                       clip_start: np.ndarray, clip_end: np.ndarray) -> np.ndarray:
    direction = end - start
    clip_direction = clip_end - clip_start
    denominator = _cross(direction, clip_direction)
    if abs(denominator) <= GEOMETRY_EPS:
        # This branch is reached only at a clipping transition.  Returning the
        # endpoint is deterministic and avoids amplifying parallel roundoff.
        return end.copy()
    parameter = _cross(clip_start - start, clip_direction) / denominator
    return start + parameter * direction


def _clip_convex(subject: np.ndarray, clip: np.ndarray) -> np.ndarray:
    output = np.asarray(subject, dtype=float)
    for clip_start, clip_end in zip(clip, np.roll(clip, -1, axis=0)):
        if not len(output):
            break
        source = output
        output_points = []
        edge = clip_end - clip_start

        def inside(point):
            return _cross(edge, point - clip_start) >= -GEOMETRY_EPS

        previous = source[-1]
        previous_inside = inside(previous)
        for current in source:
            current_inside = inside(current)
            if current_inside != previous_inside:
                output_points.append(_line_intersection(previous, current, clip_start, clip_end))
            if current_inside:
                output_points.append(current)
            previous, previous_inside = current, current_inside
        output = np.asarray(output_points, dtype=float).reshape((-1, 2))
    return output


def _polygon_area(polygon: np.ndarray) -> float:
    if len(polygon) < 3:
        return 0.0
    return abs(float(np.dot(polygon[:, 0], np.roll(polygon[:, 1], -1))
                     - np.dot(polygon[:, 1], np.roll(polygon[:, 0], -1)))) * 0.5


def padded_footprint_overlap_area(pose_a: Sequence[float], pose_b: Sequence[float]) -> float:
    """Return exact convex-polygon overlap area for the frozen padded footprint."""
    a, b = np.asarray(pose_a, dtype=float), np.asarray(pose_b, dtype=float)
    if a.shape != (3,) or b.shape != (3,) or not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise ValueError("poses must be finite x/y/yaw triples")
    return _polygon_area(_clip_convex(_footprint_polygon(a), _footprint_polygon(b)))


def _invalid(*codes: str) -> dict[str, Any]:
    unique = list(dict.fromkeys(codes)) or ["INVALID_INPUT"]
    return {
        "audit_passed": False,
        "input_complete": False,
        "status": "INVALID_INPUT",
        "failure_codes": unique,
        "requires_detour_justification": True,
        "scope": "pure_geometry_screen_not_a_detour_necessity_proof",
        "footprint_half_extents_m": [FOOTPRINT_HALF_LENGTH_M, FOOTPRINT_HALF_WIDTH_M],
    }


def _terminal_direction(start: np.ndarray, goal: np.ndarray,
                        route_polyline: Sequence[Sequence[float]] | None):
    direct = goal - start
    direct_norm = float(np.linalg.norm(direct))
    if direct_norm <= GEOMETRY_EPS:
        return None, {"failure_code": "COINCIDENT_START_GOAL"}
    if route_polyline is None:
        return direct / direct_norm, {
            "direction_source": "start_to_goal",
            "route_reversed_for_query": False,
        }
    try:
        route = np.asarray(route_polyline, dtype=float)
    except (TypeError, ValueError):
        return None, {"failure_code": "INVALID_ROUTE_POLYLINE"}
    if route.ndim != 2 or route.shape[1] < 2 or len(route) < 2 or not np.all(np.isfinite(route[:, :2])):
        return None, {"failure_code": "INVALID_ROUTE_POLYLINE"}
    route = route[:, :2]
    # Consecutive duplicates carry no direction information and otherwise let
    # a one-cell endpoint attachment dominate the terminal tangent.
    keep = np.r_[True, np.linalg.norm(np.diff(route, axis=0), axis=1) > GEOMETRY_EPS]
    route = route[keep]
    if len(route) < 2:
        return None, {"failure_code": "ROUTE_TERMINAL_TANGENT_UNDEFINED"}
    normal = float(np.linalg.norm(route[0] - start) + np.linalg.norm(route[-1] - goal))
    reverse = float(np.linalg.norm(route[-1] - start) + np.linalg.norm(route[0] - goal))
    reversed_for_query = reverse < normal
    if reversed_for_query:
        route = route[::-1]
    start_distance = float(np.linalg.norm(route[0] - start))
    end_distance = float(np.linalg.norm(route[-1] - goal))
    direction_info = {
        "direction_source": "oriented_route_terminal_tangent",
        "route_reversed_for_query": reversed_for_query,
        "route_normal_endpoint_sum_m": normal,
        "route_reversed_endpoint_sum_m": reverse,
        "route_start_distance_m": start_distance,
        "route_end_distance_m": end_distance,
        "route_endpoint_binding_limit_m": ROUTE_ENDPOINT_BINDING_LIMIT_M,
    }
    if max(start_distance, end_distance) > ROUTE_ENDPOINT_BINDING_LIMIT_M + GEOMETRY_EPS:
        return None, {**direction_info, "failure_code": "ROUTE_ENDPOINT_UNBOUND"}

    # Endpoint attachment commonly contributes a final diagonal map-cell
    # segment.  Define the goal plane from at least one full padded-footprint
    # length of the oriented route so quantisation cannot rotate it by 45 deg.
    distances = np.linalg.norm(np.diff(route, axis=0), axis=1)
    index = len(route) - 2
    accumulated = float(distances[index])
    while index > 0 and accumulated < FOOTPRINT_LENGTH_M:
        index -= 1
        accumulated += float(distances[index])
    delta = route[-1] - route[index]
    norm = float(np.linalg.norm(delta))
    if norm <= GEOMETRY_EPS:
        return None, {**direction_info, "failure_code": "ROUTE_TERMINAL_TANGENT_UNDEFINED"}
    tangent = delta / norm
    goal_gap = goal - route[-1]
    goal_gap_longitudinal = float(np.dot(goal_gap, tangent))
    goal_gap_lateral = abs(_cross(tangent, goal_gap))
    direction_info.update({
        "route_goal_gap_longitudinal_m": goal_gap_longitudinal,
        "route_goal_gap_lateral_m": goal_gap_lateral,
        "route_goal_ray_lateral_limit_m": ROUTE_GOAL_RAY_LATERAL_LIMIT_M,
    })
    if end_distance > GEOMETRY_EPS and (
        goal_gap_longitudinal < -GEOMETRY_EPS
        or goal_gap_lateral > ROUTE_GOAL_RAY_LATERAL_LIMIT_M + GEOMETRY_EPS
    ):
        return None, {**direction_info, "failure_code": "ROUTE_GOAL_RAY_UNBOUND"}
    return tangent, {
        **direction_info,
        "terminal_tangent_window_m": accumulated,
        "terminal_tangent_window_source": "one_full_padded_footprint_length",
    }


def _dense_geometry(path: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    dense = [path[0].copy()]
    station = [0.0]
    total = 0.0
    for first, second in zip(path, path[1:]):
        delta_xy = second[:2] - first[:2]
        distance = float(np.linalg.norm(delta_xy))
        delta_yaw = float(_wrap(second[2] - first[2]))
        # Necessity uses the contract's fixed 0.025 m arc-length resolution.
        # The separate full safety audit remains responsible for its <=1-degree
        # interpolation rule.
        count = max(1, math.ceil(distance / (MAX_GEOMETRY_SAMPLE_SPACING_M * (1.0 + 1.0e-9))))
        for step in range(1, count + 1):
            fraction = step / count
            pose = first + np.asarray((delta_xy[0], delta_xy[1], delta_yaw)) * fraction
            pose[2] = _wrap(pose[2])
            dense.append(pose)
            total += distance / count
            station.append(total)
    return np.asarray(dense), np.asarray(station)


def _regression_runs(
    progress: np.ndarray,
    station: np.ndarray,
) -> tuple[float, float, list[dict[str, Any]]]:
    delta = np.diff(progress)
    regressing = delta < -GEOMETRY_EPS
    runs = []
    index = 0
    while index < len(regressing):
        if not regressing[index]:
            index += 1
            continue
        end = index
        while end + 1 < len(regressing) and regressing[end + 1]:
            end += 1
        arc_span = float(station[end + 1] - station[index])
        loss = float(progress[index] - progress[end + 1])
        nonlocal_run = arc_span + GEOMETRY_EPS >= MIN_NONLOCAL_ARC_SEPARATION_M
        runs.append({
            "start_index": index,
            "end_index": end + 1,
            "start_station_m": float(station[index]),
            "end_station_m": float(station[end + 1]),
            "arc_span_m": arc_span,
            "route_progress_loss_m": loss,
            "nonlocal": nonlocal_run,
        })
        index = end + 1
    total = float(np.maximum(-delta, 0.0).sum())
    nonlocal_total = float(sum(run["route_progress_loss_m"] for run in runs if run["nonlocal"]))
    return total, nonlocal_total, runs


def _rectangle_overlap_score(first: np.ndarray, second: np.ndarray) -> float | None:
    """Exact rectangle SAT with a cheap normalized penetration score."""
    first_forward = np.asarray((math.cos(first[2]), math.sin(first[2])))
    first_lateral = np.asarray((-first_forward[1], first_forward[0]))
    second_forward = np.asarray((math.cos(second[2]), math.sin(second[2])))
    second_lateral = np.asarray((-second_forward[1], second_forward[0]))
    delta = second[:2] - first[:2]
    margins = []
    for axis in (first_forward, first_lateral, second_forward, second_lateral):
        first_radius = (FOOTPRINT_HALF_LENGTH_M * abs(float(np.dot(first_forward, axis)))
                        + FOOTPRINT_HALF_WIDTH_M * abs(float(np.dot(first_lateral, axis))))
        second_radius = (FOOTPRINT_HALF_LENGTH_M * abs(float(np.dot(second_forward, axis)))
                         + FOOTPRINT_HALF_WIDTH_M * abs(float(np.dot(second_lateral, axis))))
        reach = first_radius + second_radius
        margin = reach - abs(float(np.dot(delta, axis)))
        # Strictly positive penetration on every separating axis is equivalent
        # to positive-area overlap for two non-degenerate rectangles.
        if margin <= GEOMETRY_EPS:
            return None
        margins.append(margin / reach)
    return min(margins)


def _reverse_overlap_events(path: np.ndarray, station: np.ndarray) -> tuple[int, list[dict[str, Any]]]:
    # The spatial bucket size is the exact padded-footprint diameter.  It is an
    # acceleration bound only; acceptance uses convex-polygon intersection.
    radius = math.hypot(FOOTPRINT_HALF_LENGTH_M, FOOTPRINT_HALF_WIDTH_M)
    diameter = 2.0 * radius
    buckets: dict[tuple[int, int], list[int]] = {}
    count = 0
    strongest = []
    for current, pose in enumerate(path):
        key = tuple(np.floor(pose[:2] / diameter).astype(np.int64))
        candidates = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                candidates.extend(buckets.get((key[0] + dx, key[1] + dy), ()))
        for previous in sorted(candidates):
            arc_separation = float(station[current] - station[previous])
            # "Nonlocal" is derived from the frozen Rmin=0.40 m: a separate
            # opposing visit must be farther apart than the shortest legal
            # 180-degree turning arc.  This avoids flagging overlap internal to
            # one continuous hairpin without introducing a tuned distance.
            if arc_separation + GEOMETRY_EPS < MIN_NONLOCAL_ARC_SEPARATION_M:
                continue
            heading_dot = math.cos(float(_wrap(pose[2] - path[previous, 2])))
            if heading_dot >= -GEOMETRY_EPS:
                continue
            if float(np.linalg.norm(pose[:2] - path[previous, :2])) > diameter + GEOMETRY_EPS:
                continue
            overlap_score = _rectangle_overlap_score(path[previous], pose)
            if overlap_score is None:
                continue
            count += 1
            heading_delta = abs(float(_wrap(pose[2] - path[previous, 2])))
            center_distance = float(np.linalg.norm(pose[:2] - path[previous, :2]))
            event = {
                "first_index": previous,
                "second_index": current,
                "first_station_m": float(station[previous]),
                "second_station_m": float(station[current]),
                "arc_separation_m": arc_separation,
                "center_distance_m": center_distance,
                "heading_delta_abs_rad": heading_delta,
                "heading_dot": heading_dot,
                "sat_minimum_normalized_penetration": overlap_score,
                "first_pose": path[previous].tolist(),
                "second_pose": pose.tolist(),
            }
            ranking = (overlap_score, -center_distance, -float(station[previous]),
                       -float(station[current]), count)
            item = (*ranking, event)
            if len(strongest) < MAX_RECORDED_OVERLAP_EVENTS:
                heapq.heappush(strongest, item)
            elif ranking > strongest[0][:-1]:
                heapq.heapreplace(strongest, item)
        buckets.setdefault(key, []).append(current)
    recorded = []
    for *_, event in strongest:
        area = padded_footprint_overlap_area(event["first_pose"], event["second_pose"])
        event["footprint_overlap_area_m2"] = area
        event["footprint_overlap_fraction"] = area / FOOTPRINT_AREA_M2
        recorded.append(event)
    recorded.sort(key=lambda event: (-event["footprint_overlap_area_m2"],
                                     event["first_station_m"], event["second_station_m"]))
    return count, recorded


def audit_path_necessity(
    path: Sequence[Sequence[float]],
    *,
    start: Sequence[float],
    goal: Sequence[float],
    target_mask: Sequence[bool] | None,
    route_polyline: Sequence[Sequence[float]] | None = None,
) -> dict[str, Any]:
    """Screen a path for unjustified excursions, returning JSON-safe evidence.

    ``audit_passed`` means only that this screen found no suspicious geometry.
    A false result requires a separate map-bound justification; it does not by
    itself prove that a natural or feasible semantic path is impossible.
    """
    try:
        poses = np.asarray(path, dtype=float)
    except (TypeError, ValueError):
        return _invalid("INVALID_PATH_SHAPE")
    if poses.ndim != 2 or poses.shape[1] < 3 or len(poses) < 2:
        return _invalid("PATH_YAW_MISSING" if poses.ndim == 2 and poses.shape[1] == 2
                        else "INVALID_PATH_SHAPE")
    poses = poses[:, :3]
    if not np.all(np.isfinite(poses)):
        return _invalid("NONFINITE_PATH")
    try:
        start_xy, goal_xy = np.asarray(start, dtype=float)[:2], np.asarray(goal, dtype=float)[:2]
    except (TypeError, ValueError, IndexError):
        return _invalid("INVALID_START_OR_GOAL")
    if start_xy.shape != (2,) or not np.all(np.isfinite(start_xy)):
        return _invalid("INVALID_START")
    if goal_xy.shape != (2,) or not np.all(np.isfinite(goal_xy)):
        return _invalid("INVALID_GOAL")
    if target_mask is None:
        return _invalid("TARGET_MASK_MISSING")
    target = np.asarray(target_mask)
    if target.ndim != 1 or len(target) != len(poses):
        return _invalid("TARGET_MASK_LENGTH_MISMATCH")
    if not np.all(np.isin(target, (False, True))):
        return _invalid("TARGET_MASK_NOT_BOOLEAN")
    target = target.astype(bool)
    input_failures = []
    if not np.array_equal(poses[0, :2], start_xy):
        input_failures.append("PATH_START_MISMATCH")
    if not np.array_equal(poses[-1, :2], goal_xy):
        input_failures.append("PATH_GOAL_MISMATCH")
    segment_length = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)
    if np.any(segment_length <= GEOMETRY_EPS):
        input_failures.append("ZERO_TRANSLATION_STEP")
    direction, direction_info = _terminal_direction(start_xy, goal_xy, route_polyline)
    if direction is None:
        input_failures.append(direction_info["failure_code"])
    if input_failures:
        return _invalid(*input_failures)

    station = np.r_[0.0, np.cumsum(segment_length)]
    signed_goal_plane = (poses[:, :2] - goal_xy) @ direction
    maximum_overshoot = max(0.0, float(np.max(signed_goal_plane)))
    maximum_index = int(np.argmax(signed_goal_plane))
    beyond = signed_goal_plane > GEOMETRY_EPS
    on_plane = np.abs(signed_goal_plane) <= GEOMETRY_EPS
    first_beyond = int(np.flatnonzero(beyond)[0]) if np.any(beyond) else None
    progress = (poses[:, :2] - start_xy) @ direction
    backward_progress, nonlocal_backward_progress, regression_runs = _regression_runs(
        progress, station
    )

    dense_path, dense_station = _dense_geometry(poses)
    overlap_count, overlap_events = _reverse_overlap_events(dense_path, dense_station)

    target_count = int(np.count_nonzero(target))
    target_before = int(np.count_nonzero(target & (signed_goal_plane < -GEOMETRY_EPS)))
    target_on = int(np.count_nonzero(target & on_plane))
    target_after = int(np.count_nonzero(target & beyond))
    failure_codes = []
    if maximum_overshoot > GEOMETRY_EPS:
        failure_codes.append("GOAL_PLANE_OVERSHOOT")
    repeated_local_regression = backward_progress + GEOMETRY_EPS >= MIN_NONLOCAL_ARC_SEPARATION_M
    if nonlocal_backward_progress > GEOMETRY_EPS or repeated_local_regression:
        failure_codes.append("BACKWARD_ROUTE_PROGRESS")
    if overlap_count:
        failure_codes.append("NONLOCAL_REVERSE_FOOTPRINT_OVERLAP")
    if target_after:
        failure_codes.append("TARGET_CREDIT_AFTER_GOAL_PLANE")
    passed = not failure_codes
    return {
        "audit_passed": passed,
        "input_complete": True,
        "status": "NO_SUSPECT_EXCURSION" if passed else "DETOUR_JUSTIFICATION_REQUIRED",
        "failure_codes": failure_codes,
        "requires_detour_justification": not passed,
        "scope": "pure_geometry_screen_not_a_detour_necessity_proof",
        "path_pose_count": len(poses),
        "path_length_m": float(station[-1]),
        "dense_geometry_pose_count": len(dense_path),
        "direction_xy": direction.tolist(),
        **direction_info,
        "goal_plane_definition": "dot(position-goal,directed_terminal_tangent)=0; positive is beyond goal",
        "maximum_goal_plane_overshoot_m": maximum_overshoot,
        "maximum_overshoot_station_m": float(station[maximum_index]),
        "maximum_overshoot_pose": poses[maximum_index].tolist(),
        "first_beyond_goal_plane_index": first_beyond,
        "first_beyond_goal_plane_station_m": float(station[first_beyond]) if first_beyond is not None else None,
        "cumulative_backward_route_progress_m": backward_progress,
        "nonlocal_backward_route_progress_m": nonlocal_backward_progress,
        "repeated_local_regression_limit_m": MIN_NONLOCAL_ARC_SEPARATION_M,
        "repeated_local_regression_limit_exceeded": repeated_local_regression,
        "backward_progress_runs": regression_runs,
        "backward_progress_failure_definition": (
            "a contiguous regression run spanning at least pi*Rmin of path arc, or "
            "cumulative shorter regressions reaching pi*Rmin; smaller endpoint-attachment "
            "curvature remains diagnostic only"
        ),
        "nonlocal_definition": (
            "arc separation >= pi*Rmin (Rmin=0.40 m), opposing headings, and positive "
            "exact convex-footprint intersection area"
        ),
        "nonlocal_reverse_footprint_overlap_count": overlap_count,
        "recorded_overlap_event_count": len(overlap_events),
        "overlap_events_truncated": overlap_count > len(overlap_events),
        "nonlocal_reverse_footprint_overlaps": overlap_events,
        "footprint_half_extents_m": [FOOTPRINT_HALF_LENGTH_M, FOOTPRINT_HALF_WIDTH_M],
        "footprint_area_m2": FOOTPRINT_AREA_M2,
        "target_sample_attribution": {
            "target_sample_count": target_count,
            "before_goal_plane_count": target_before,
            "on_goal_plane_count": target_on,
            "after_goal_plane_count": target_after,
            "after_goal_plane_ratio": target_after / target_count if target_count else None,
        },
    }
