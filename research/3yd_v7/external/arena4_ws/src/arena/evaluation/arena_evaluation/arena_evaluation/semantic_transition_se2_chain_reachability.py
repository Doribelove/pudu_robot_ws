"""Non-decision exploratory 48-bin Dubins reach/coreach diagnostic.

This request-derived probe follows the PLN-02 positive-query goal-plane audit.
It reads the current request's route and constraint grids, never a saved path
or witness, and leaves every input raster unchanged.

The finite state identity is the same ``(map row, map column, yaw bin)``
quotient used by the existing isolated Smac-compatible oracle.  Each direction
keeps the first deterministic continuous representative for a state key.  A
forward expansion applies the frozen Smac Dubin primitive.  A backward
expansion analytically inverts that primitive, then validates the corresponding
*forward* sweep.  Thus backward traversal does not grant reverse vehicle
motion.  Every admitted transition is checked by ``ConstraintWorld`` with its
dense padded-footprint/effective-master audit, is confined to one lane feature
instance, and stays on or before the request goal plane.

This is explicitly a first-representative *under-approximation*.  It does not
implement Smac analytic expansion and emits no replayable parent-chain
certificate.  Its key intersection is neither a continuous-space
infeasibility proof nor a path witness: continuous representatives attached to
the same quotient key can differ within a map cell.  Presence and absence are
both non-decision exploratory observations and cannot support acceptance, C1,
or a hard stop.  No semantic label/top-k resource pruning is performed.  Exact
R2 path-resource optimisation is deliberately outside this probe.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import platform
import resource
import shutil
import time
import traceback
from types import SimpleNamespace
from typing import Mapping, Sequence

import numpy as np

from .se2_semantic_guide import (
    SE2GuidePolicy,
    SmacDubinPrimitive,
    bin_to_yaw,
    build_smac_dubin_primitives,
    sample_primitive,
    wrap_angle,
    yaw_to_bin,
)
from .semantic_constraint_core import ConstraintWorld
from .semantic_map import canonical_hash, sha256_file
from .semantic_transition_goal_plane_topology import _oriented_terminal_tangent
from .semantic_transition_ordered_corridor import CorridorFailure, OrientedRoute


PROTOCOL_ID = "PLN-02-SEMANTIC-SE2-CHAIN-NONDECISION-EXPLORATORY-R0-V1"
METHOD_ID = "first_representative_48bin_dubin_underapprox_probe_v1"
QUERY_ID = "r3-mirror-1-positive"
YAW_BINS = 48
TURNING_RADIUS_M = 0.401
HARD_MAXIMUM_CURVATURE_1PM = 2.50
PADDED_FOOTPRINT = ((0.265, 0.225), (0.265, -0.225), (-0.265, -0.225), (-0.265, 0.225))

StateKey = tuple[int, int, int]
Pose = tuple[float, float, float]


class ReachabilityFailure(RuntimeError):
    """Fail-closed setup error with a stable result code."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = str(code)
        self.detail = str(detail)


@dataclass(frozen=True)
class ReachabilityPolicy:
    """Frozen bounds for the one-off positive-query diagnostic."""

    yaw_bins: int = YAW_BINS
    resolution_m: float = 0.05
    turning_radius_m: float = TURNING_RADIUS_M
    primitive_sample_spacing_m: float = 0.025
    local_crop_padding_m: float = 6.0
    goal_plane_epsilon_m: float = 1.0e-9
    exact_representative_join_tolerance_m: float = 1.0e-9
    maximum_states_per_direction: int = 1_000_000
    timeout_per_direction_s: float = 120.0
    maximum_peak_rss_mib: float = 2048.0
    route_endpoint_attachment_limit_m: float = 0.75

    def __post_init__(self) -> None:
        if self.yaw_bins != YAW_BINS:
            raise ValueError("this diagnostic requires exactly 48 yaw bins")
        if not math.isclose(self.resolution_m, 0.05, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError("this diagnostic requires the frozen 0.05 m map")
        if not math.isclose(self.turning_radius_m, TURNING_RADIUS_M, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError("this diagnostic requires the frozen 0.401 m implementation radius")
        if self.primitive_sample_spacing_m <= 0.0:
            raise ValueError("primitive sample spacing must be positive")
        if self.local_crop_padding_m < 0.53:
            raise ValueError("local crop padding must cover at least one padded footprint length")
        if self.goal_plane_epsilon_m < 0.0 or self.exact_representative_join_tolerance_m < 0.0:
            raise ValueError("numeric tolerances must be non-negative")
        if not 1 <= self.maximum_states_per_direction <= 1_000_000:
            raise ValueError("maximum states per direction must be in [1, 1000000]")
        if not 0.0 < self.timeout_per_direction_s <= 120.0:
            raise ValueError("per-direction timeout must be in (0, 120] seconds")
        if not 128.0 <= self.maximum_peak_rss_mib <= 2048.0:
            raise ValueError("peak RSS bound must be in [128, 2048] MiB")
        if self.route_endpoint_attachment_limit_m < 0.0:
            raise ValueError("route attachment limit must be non-negative")

    @property
    def smac_policy(self) -> SE2GuidePolicy:
        # 0.401 m is conservative relative to the hard 0.40 m minimum; its
        # realised primitive curvature is therefore slightly below 2.50 1/m.
        return replace(
            SE2GuidePolicy(),
            resolution_m=self.resolution_m,
            yaw_bins=self.yaw_bins,
            minimum_turning_radius_m=self.turning_radius_m,
            maximum_curvature_1pm=1.0 / self.turning_radius_m,
            primitive_sample_spacing_m=self.primitive_sample_spacing_m,
            max_iterations=self.maximum_states_per_direction,
            timeout_s=self.timeout_per_direction_s,
        )


@dataclass
class DirectionSearch:
    direction: str
    representatives: dict[StateKey, Pose]
    traversal_open_exhausted: bool
    stop_code: str
    expanded_state_count: int
    generated_transition_count: int
    accepted_tree_edge_count: int
    rejection_counts: dict[str, int]
    alias_count: int
    alias_position_residual_max_m: float
    alias_position_residual_sum_m: float
    wall_s: float


def _peak_rss_mib() -> float:
    # Linux ru_maxrss is KiB.  The project is pinned to Linux/ROS Humble.
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def _hash_rows(rows: np.ndarray, width: int) -> str:
    array = np.asarray(rows)
    if array.ndim != 2 or array.shape[1] != width:
        array = np.empty((0, width), dtype=np.int32)
    array = np.asarray(array, dtype="<i4")
    if len(array):
        order = np.lexsort(tuple(array[:, index] for index in reversed(range(width))))
        array = np.ascontiguousarray(array[order])
    else:
        array = np.ascontiguousarray(array)
    return hashlib.sha256(array.tobytes()).hexdigest()


def _keys_array(keys: Sequence[StateKey] | set[StateKey]) -> np.ndarray:
    if not keys:
        return np.empty((0, 3), dtype=np.int32)
    return np.asarray(sorted(keys), dtype=np.int32).reshape((-1, 3))


def _cells_array(cells: Sequence[tuple[int, int]] | set[tuple[int, int]]) -> np.ndarray:
    if not cells:
        return np.empty((0, 2), dtype=np.int32)
    return np.asarray(sorted(cells), dtype=np.int32).reshape((-1, 2))


def _pose_key(world, pose: Sequence[float], yaw_bins: int = YAW_BINS) -> StateKey | None:
    cell = world.map.world_to_cell(float(pose[0]), float(pose[1]))
    if cell is None:
        return None
    return int(cell[0]), int(cell[1]), yaw_to_bin(float(pose[2]), yaw_bins)


def _predecessor_pose(child: Pose, primitive: SmacDubinPrimitive, policy: SE2GuidePolicy) -> Pose:
    """Exact predecessor whose forward primitive terminates at ``child``."""

    child_bin = yaw_to_bin(child[2], policy.yaw_bins)
    predecessor_bin = (child_bin - primitive.delta_yaw_bins) % policy.yaw_bins
    predecessor_yaw = bin_to_yaw(predecessor_bin, policy.yaw_bins)
    if primitive.name == "FORWARD":
        dx = primitive.arc_length_m * math.cos(predecessor_yaw)
        dy = primitive.arc_length_m * math.sin(predecessor_yaw)
    else:
        angle = abs(primitive.delta_yaw_rad)
        radius = policy.minimum_turning_radius_m
        if primitive.name == "FORWARD_LEFT":
            dx = radius * (math.sin(predecessor_yaw + angle) - math.sin(predecessor_yaw))
            dy = radius * (-math.cos(predecessor_yaw + angle) + math.cos(predecessor_yaw))
        elif primitive.name == "FORWARD_RIGHT":
            dx = radius * (math.sin(predecessor_yaw) - math.sin(predecessor_yaw - angle))
            dy = radius * (math.cos(predecessor_yaw - angle) - math.cos(predecessor_yaw))
        else:  # Defensive: only the three frozen primitives are admissible.
            raise ValueError(f"unsupported primitive {primitive.name}")
    return (
        float(child[0]) - dx,
        float(child[1]) - dy,
        predecessor_yaw,
    )


def _cell_centres(world) -> tuple[np.ndarray, np.ndarray]:
    rows = np.arange(world.map.height, dtype=np.float64)
    cols = np.arange(world.map.width, dtype=np.float64)
    x = world.map.full_origin[0] + (world.map.col0 + cols + 0.5) * world.map.resolution
    y = world.map.full_origin[1] + (
        world.map.full_height - world.map.row0 - rows - 0.5
    ) * world.map.resolution
    return x, y


def _local_masks(world, route: OrientedRoute, terminal_tangent: np.ndarray,
                 lane_label: int, policy: ReachabilityPolicy) -> tuple[np.ndarray, np.ndarray, dict]:
    x, y = _cell_centres(world)
    padding = float(policy.local_crop_padding_m)
    xmin, ymin = np.min(route.points, axis=0) - padding
    xmax, ymax = np.max(route.points, axis=0) + padding
    bbox = (
        (x[None, :] >= xmin) & (x[None, :] <= xmax)
        & (y[:, None] >= ymin) & (y[:, None] <= ymax)
    )
    # Cell-centre inclusion uses one cell of indexing slack at the terminal
    # plane.  The actual continuous pose and every primitive sample are checked
    # against the strict epsilon separately; this slack cannot admit overshoot.
    progress = (
        (x[None, :] - float(world.goal[0])) * float(terminal_tangent[0])
        + (y[:, None] - float(world.goal[1])) * float(terminal_tangent[1])
    )
    same_lane = np.asarray(world.grids["labels"] == int(lane_label), dtype=bool)
    static_legal = (
        same_lane
        & np.asarray(world.grids["allowed"], dtype=bool)
        & ~np.asarray(world.grids["hard"], dtype=bool)
        & (np.asarray(world.master) < 253)
    )
    local = static_legal & bbox & (progress <= policy.resolution_m)
    raw_target = (
        same_lane
        & np.asarray(world.grids["correct"], dtype=bool)
        & (np.asarray(world.grids["error"], dtype=np.float64) <= 0.50 + 1.0e-12)
    )
    local_target = raw_target & local
    pre_goal_target = local_target & (progress <= policy.goal_plane_epsilon_m)
    after_goal_target = local_target & (progress > policy.goal_plane_epsilon_m)
    diagnostics = {
        "local_crop_bbox_world": [float(xmin), float(ymin), float(xmax), float(ymax)],
        "local_crop_cell_count": int(np.count_nonzero(local)),
        "same_lane_static_legal_cell_count": int(np.count_nonzero(static_legal)),
        "raw_target_cell_count_same_lane_full_crop": int(np.count_nonzero(raw_target)),
        "raw_target_cell_count_local": int(np.count_nonzero(local_target)),
        "raw_target_cell_count_local_pre_or_on_goal_plane": int(np.count_nonzero(pre_goal_target)),
        "raw_target_cell_count_local_after_goal_plane": int(np.count_nonzero(after_goal_target)),
        "local_mask_sha256": hashlib.sha256(np.ascontiguousarray(local).tobytes()).hexdigest(),
        "raw_target_mask_sha256": hashlib.sha256(np.ascontiguousarray(raw_target).tobytes()).hexdigest(),
        "local_raw_target_mask_sha256": hashlib.sha256(
            np.ascontiguousarray(local_target).tobytes()
        ).hexdigest(),
        "goal_plane_cell_index_slack_m": policy.resolution_m,
        "primitive_sample_goal_plane_epsilon_m": policy.goal_plane_epsilon_m,
    }
    return local, raw_target, diagnostics


def _validate_transition(
    world,
    source: Pose,
    samples: Sequence[Pose],
    primitive: SmacDubinPrimitive,
    *,
    local_mask: np.ndarray,
    lane_label: int,
    terminal_tangent: np.ndarray,
    policy: ReachabilityPolicy,
) -> tuple[bool, str]:
    poses = np.vstack((np.asarray(source, dtype=np.float64), np.asarray(samples, dtype=np.float64)))
    progress = (
        (poses[:, 0] - float(world.goal[0])) * float(terminal_tangent[0])
        + (poses[:, 1] - float(world.goal[1])) * float(terminal_tangent[1])
    )
    if np.any(progress > policy.goal_plane_epsilon_m):
        return False, "GOAL_PLANE_OVERSHOOT"
    rows, cols, inside = world.cells(poses)
    if not np.all(inside):
        return False, "OUTSIDE_CONSTRAINT_CROP"
    if np.any(~local_mask[rows, cols]):
        return False, "OUTSIDE_LOCAL_SAME_LANE_CROP"
    if np.any(np.asarray(world.grids["labels"])[rows, cols] != int(lane_label)):
        return False, "LANE_INSTANCE_ESCAPE"
    if abs(float(primitive.curvature_1pm)) > HARD_MAXIMUM_CURVATURE_1PM + 1.0e-12:
        return False, "CURVATURE_INFEASIBLE"
    probe = SimpleNamespace(
        start=tuple(map(float, source)),
        samples=np.asarray(samples, dtype=np.float64),
        n=0,
        correct=0,
        target=0,
    )
    if not world.validate_edge(probe, dense=True):
        return False, "CONSTRAINT_WORLD_DENSE_REJECTED"
    return True, ""


def _search_direction(
    world,
    seed: Pose,
    *,
    direction: str,
    primitives: Sequence[SmacDubinPrimitive],
    local_mask: np.ndarray,
    lane_label: int,
    terminal_tangent: np.ndarray,
    policy: ReachabilityPolicy,
) -> DirectionSearch:
    if direction not in ("forward_from_start", "backward_from_goal"):
        raise ValueError(f"unknown direction {direction}")
    started = time.monotonic()
    seed_key = _pose_key(world, seed, policy.yaw_bins)
    if seed_key is None or not bool(local_mask[seed_key[:2]]):
        raise ReachabilityFailure("ENDPOINT_OUTSIDE_LOCAL_CROP", f"{direction} seed is not local/legal")
    representatives: dict[StateKey, Pose] = {seed_key: seed}
    queue: deque[StateKey] = deque((seed_key,))
    rejected: Counter[str] = Counter()
    expanded = generated = accepted = aliases = 0
    alias_sum = alias_max = 0.0
    traversal_open_exhausted = True
    stop_code = "OPEN_EXHAUSTED"

    while queue:
        if time.monotonic() - started >= policy.timeout_per_direction_s:
            traversal_open_exhausted = False
            stop_code = "DIRECTION_TIMEOUT"
            break
        if len(representatives) >= policy.maximum_states_per_direction:
            traversal_open_exhausted = False
            stop_code = "DIRECTION_STATE_LIMIT"
            break
        if _peak_rss_mib() >= policy.maximum_peak_rss_mib:
            traversal_open_exhausted = False
            stop_code = "PROCESS_RSS_LIMIT"
            break
        key = queue.popleft()
        pose = representatives[key]
        expanded += 1
        for primitive in primitives:
            generated += 1
            if direction == "forward_from_start":
                source = pose
                samples = sample_primitive(source, primitive, policy.smac_policy)
                candidate = tuple(map(float, samples[-1]))
            else:
                candidate = _predecessor_pose(pose, primitive, policy.smac_policy)
                source = candidate
                samples = sample_primitive(source, primitive, policy.smac_policy)
                endpoint = samples[-1]
                endpoint_error = math.hypot(endpoint[0] - pose[0], endpoint[1] - pose[1])
                endpoint_yaw_error = abs(wrap_angle(endpoint[2] - pose[2]))
                if endpoint_error > 1.0e-9 or endpoint_yaw_error > 1.0e-12:
                    rejected["INVERSE_REPLAY_MISMATCH"] += 1
                    continue

            candidate_key = _pose_key(world, candidate, policy.yaw_bins)
            if candidate_key is None:
                rejected["OUTSIDE_CONSTRAINT_CROP"] += 1
                continue
            valid, reason = _validate_transition(
                world,
                source,
                samples,
                primitive,
                local_mask=local_mask,
                lane_label=lane_label,
                terminal_tangent=terminal_tangent,
                policy=policy,
            )
            if not valid:
                rejected[reason] += 1
                continue
            prior = representatives.get(candidate_key)
            if prior is not None:
                aliases += 1
                residual = math.hypot(prior[0] - candidate[0], prior[1] - candidate[1])
                alias_sum += residual
                alias_max = max(alias_max, residual)
                rejected["EXISTING_QUOTIENT_KEY"] += 1
                continue
            representatives[candidate_key] = candidate
            queue.append(candidate_key)
            accepted += 1

    return DirectionSearch(
        direction=direction,
        representatives=representatives,
        traversal_open_exhausted=traversal_open_exhausted,
        stop_code=stop_code,
        expanded_state_count=expanded,
        generated_transition_count=generated,
        accepted_tree_edge_count=accepted,
        rejection_counts=dict(sorted(rejected.items())),
        alias_count=aliases,
        alias_position_residual_max_m=alias_max,
        alias_position_residual_sum_m=alias_sum,
        wall_s=time.monotonic() - started,
    )


def _direction_summary(search: DirectionSearch) -> dict:
    keys = _keys_array(set(search.representatives))
    return {
        "direction": search.direction,
        "traversal_open_exhausted": search.traversal_open_exhausted,
        "stop_code": search.stop_code,
        "reachable_quotient_state_count": len(search.representatives),
        "reachable_state_keys_sha256": _hash_rows(keys, 3),
        "expanded_state_count": search.expanded_state_count,
        "generated_transition_count": search.generated_transition_count,
        "accepted_tree_edge_count": search.accepted_tree_edge_count,
        "rejection_counts": search.rejection_counts,
        "existing_quotient_key_transition_count": search.alias_count,
        "existing_key_representative_position_residual_mean_m": (
            search.alias_position_residual_sum_m / search.alias_count if search.alias_count else 0.0
        ),
        "existing_key_representative_position_residual_max_m": (
            search.alias_position_residual_max_m
        ),
        "wall_s": search.wall_s,
    }


def analyze_world(
    world,
    policy: ReachabilityPolicy | None = None,
    *,
    primitives: Sequence[SmacDubinPrimitive] | None = None,
) -> tuple[dict, dict[str, np.ndarray]]:
    """Run the deterministic finite reachability analysis on an already bound world."""

    policy = policy or ReachabilityPolicy()
    if not math.isclose(float(world.map.resolution), policy.resolution_m, rel_tol=0.0, abs_tol=1.0e-12):
        raise ReachabilityFailure("MAP_RESOLUTION_MISMATCH", "bound world is not the frozen 0.05 m map")
    smac_policy = policy.smac_policy
    smac_policy.validate()
    if tuple(tuple(float(value) for value in point) for point in smac_policy.padded_footprint) != PADDED_FOOTPRINT:
        raise ReachabilityFailure(
            "FOOTPRINT_BINDING_MISMATCH",
            "formal primitive checker is not bound to the frozen 0.53 x 0.45 m padded footprint",
        )
    formal_primitives = tuple(primitives or build_smac_dubin_primitives(smac_policy))
    if tuple(item.primitive_id for item in formal_primitives) != tuple(range(len(formal_primitives))):
        raise ReachabilityFailure("PRIMITIVE_BINDING_MISMATCH", "primitive IDs must be dense and ordered")
    if any(abs(item.curvature_1pm) > HARD_MAXIMUM_CURVATURE_1PM + 1.0e-12 for item in formal_primitives):
        raise ReachabilityFailure("CURVATURE_INFEASIBLE", "formal primitive exceeds 2.50 1/m")

    start = tuple(map(float, world.start))
    goal = tuple(map(float, world.goal))
    for name, pose in (("start", start), ("goal", goal)):
        snapped = bin_to_yaw(yaw_to_bin(pose[2], policy.yaw_bins), policy.yaw_bins)
        if abs(wrap_angle(snapped - pose[2])) > 1.0e-12:
            raise ReachabilityFailure(
                "ENDPOINT_YAW_NOT_ON_48_BIN",
                f"{name} yaw would require an endpoint change: yaw={pose[2]:.12f}, bin_yaw={snapped:.12f}",
            )
    start_key = _pose_key(world, start, policy.yaw_bins)
    goal_key = _pose_key(world, goal, policy.yaw_bins)
    if start_key is None or goal_key is None:
        raise ReachabilityFailure("ENDPOINT_OUTSIDE_CONSTRAINT_CROP", "query endpoint is outside input crop")
    labels = np.asarray(world.grids["labels"])
    start_label = int(labels[start_key[:2]])
    goal_label = int(labels[goal_key[:2]])
    selected = {int(value) for value in world.selected}
    if start_label <= 0 or start_label != goal_label or start_label not in selected:
        raise ReachabilityFailure(
            "ENDPOINT_LANE_INSTANCE_MISMATCH",
            f"same direct lane instance required: start={start_label}, goal={goal_label}, selected={sorted(selected)}",
        )
    if not world.collision_free(np.asarray((start, goal), dtype=np.float64)):
        raise ReachabilityFailure("ENDPOINT_FOOTPRINT_INVALID", "start or goal fails padded effective-master check")

    route = OrientedRoute(
        world.meta["route_polyline"],
        start,
        goal,
        endpoint_attachment_limit_m=policy.route_endpoint_attachment_limit_m,
    )
    _, terminal_tangent, tangent_diagnostics = _oriented_terminal_tangent(
        route.points,
        start,
        goal,
        footprint_length_m=0.53,
    )
    terminal_tangent = np.asarray(terminal_tangent, dtype=np.float64)
    local_mask, raw_target, crop_diagnostics = _local_masks(
        world, route, terminal_tangent, start_label, policy,
    )
    if not bool(local_mask[start_key[:2]]) or not bool(local_mask[goal_key[:2]]):
        raise ReachabilityFailure("ENDPOINT_OUTSIDE_LOCAL_CROP", "start or goal excluded by frozen local mask")

    forward = _search_direction(
        world,
        start,
        direction="forward_from_start",
        primitives=formal_primitives,
        local_mask=local_mask,
        lane_label=start_label,
        terminal_tangent=terminal_tangent,
        policy=policy,
    )
    backward = _search_direction(
        world,
        goal,
        direction="backward_from_goal",
        primitives=formal_primitives,
        local_mask=local_mask,
        lane_label=start_label,
        terminal_tangent=terminal_tangent,
        policy=policy,
    )

    forward_keys = set(forward.representatives)
    backward_keys = set(backward.representatives)
    chain_keys = forward_keys & backward_keys
    observed_target_intersection_keys = {
        key for key in chain_keys if bool(raw_target[key[0], key[1]])
    }
    observed_target_intersection_cells = {
        (key[0], key[1]) for key in observed_target_intersection_keys
    }

    exact_join_keys: set[StateKey] = set()
    exact_target_join_keys: set[StateKey] = set()
    join_residuals = []
    for key in sorted(chain_keys):
        left = forward.representatives[key]
        right = backward.representatives[key]
        residual = math.hypot(left[0] - right[0], left[1] - right[1])
        yaw_residual = abs(wrap_angle(left[2] - right[2]))
        join_residuals.append(residual)
        if (
            residual <= policy.exact_representative_join_tolerance_m
            and yaw_residual <= 1.0e-12
        ):
            exact_join_keys.add(key)
            if key in observed_target_intersection_keys:
                exact_target_join_keys.add(key)

    traversals_open_exhausted = bool(
        forward.traversal_open_exhausted and backward.traversal_open_exhausted
    )
    if not traversals_open_exhausted:
        result_code = "NON_DECISION_EXPLORATORY_TRAVERSAL_BOUNDED"
    elif not observed_target_intersection_keys:
        result_code = "NON_DECISION_EXPLORATORY_NO_TARGET_KEY_INTERSECTION_OBSERVED"
    elif not exact_target_join_keys:
        result_code = "NON_DECISION_EXPLORATORY_TARGET_KEY_INTERSECTION_OBSERVED"
    else:
        result_code = "NON_DECISION_EXPLORATORY_EXACT_REPRESENTATIVE_TARGET_JOIN_OBSERVED"

    chain_array = _keys_array(chain_keys)
    target_intersection_array = _keys_array(observed_target_intersection_keys)
    exact_join_array = _keys_array(exact_join_keys)
    exact_target_array = _keys_array(exact_target_join_keys)
    target_cells_array = _cells_array(observed_target_intersection_cells)
    result = {
        "protocol_id": PROTOCOL_ID,
        "method_id": METHOD_ID,
        "query_id": str(world.query.query_id),
        "result_code": result_code,
        "analysis_disposition": "NON_DECISION_EXPLORATORY",
        "hard_stop_eligible": False,
        "acceptance_evidence": False,
        "c1_infeasibility_evidence": False,
        "traversals_open_exhausted_within_underapproximation": traversals_open_exhausted,
        "observed_target_key_intersection_nondecisive": bool(observed_target_intersection_keys),
        "observed_exact_representative_target_join_nondecisive": bool(exact_target_join_keys),
        "continuous_space_witness_proved": False,
        "replayable_witness_generated": False,
        "parent_chain_certificate_emitted": False,
        "smac_analytic_expansion_implemented": False,
        "r2_resource_analysis_status": "NOT_RUN_NONDECISION_EXPLORATORY",
        "r2_resource_analysis_reason": (
            "first-representative under-approximation has no replayable continuously joined "
            "parent chain, and the R2 short/long active window is path-history dependent"
        ),
        "lane_instance_label": start_label,
        "forward": _direction_summary(forward),
        "backward": _direction_summary(backward),
        "intersection": {
            "quotient_state_count": len(chain_keys),
            "quotient_state_keys_sha256": _hash_rows(chain_array, 3),
            "observed_raw_target_key_intersection_count": len(observed_target_intersection_keys),
            "observed_raw_target_key_intersection_sha256": _hash_rows(target_intersection_array, 3),
            "observed_raw_target_original_cell_count": len(observed_target_intersection_cells),
            "raw_target_original_cells_sha256": _hash_rows(target_cells_array, 2),
            "observed_exact_representative_join_state_count": len(exact_join_keys),
            "exact_representative_join_keys_sha256": _hash_rows(exact_join_array, 3),
            "observed_exact_representative_raw_target_state_count": len(exact_target_join_keys),
            "exact_representative_raw_target_keys_sha256": _hash_rows(exact_target_array, 3),
            "representative_join_position_residual_min_m": (
                float(min(join_residuals)) if join_residuals else None
            ),
            "representative_join_position_residual_p50_m": (
                float(np.median(np.asarray(join_residuals))) if join_residuals else None
            ),
            "representative_join_position_residual_max_m": (
                float(max(join_residuals)) if join_residuals else None
            ),
            "quotient_position_alias_bound_m": math.sqrt(2.0) * policy.resolution_m,
        },
        "crop": crop_diagnostics,
        "route": {
            "route_hash": route.route_hash,
            "route_length_m": route.length_m,
            "start_attachment_distance_m": route.start_attachment_distance_m,
            "goal_attachment_distance_m": route.goal_attachment_distance_m,
            "terminal_tangent": terminal_tangent.tolist(),
            **tangent_diagnostics,
        },
        "primitive_contract": {
            "source": "se2_semantic_guide.build_smac_dubin_primitives",
            "yaw_bins": policy.yaw_bins,
            "map_resolution_m": policy.resolution_m,
            "turning_radius_m": policy.turning_radius_m,
            "hard_minimum_turning_radius_m": 0.40,
            "hard_maximum_curvature_1pm": HARD_MAXIMUM_CURVATURE_1PM,
            "realised_turn_curvature_1pm": 1.0 / policy.turning_radius_m,
            "allow_reverse": False,
            "allow_in_place_rotation": False,
            "padded_footprint": [list(item) for item in PADDED_FOOTPRINT],
            "primitives": [asdict(item) for item in formal_primitives],
            "dense_constraint_world_validation": True,
            "effective_master_hash": world.meta.get("expected_master_hash"),
            "same_lane_feature_instance": True,
            "strict_continuous_goal_plane_overshoot_rejected": True,
        },
        "finite_scope": {
            "state_identity": "original_0.05m_map_cell_row_col_plus_48bin_yaw",
            "representative_rule": "first_valid_continuous_pose_in_deterministic_BFS_primitive_order",
            "search_coverage": "strict under-approximation because later continuous representatives for an existing key are discarded",
            "forward_set": "first-representative primitive exploration from exact start",
            "backward_set": "first-representative reverse exploration of validated forward primitives from exact goal",
            "intersection_kind": "non-decision observation in two independently selected representative sets",
            "no_semantic_top_k_pruning": True,
            "smac_analytic_expansion_absent": True,
            "replayable_parent_chain_absent": True,
            "position_subsampling_or_multiresolution_map": False,
            "negative_observation_supports_hard_stop_or_c1": False,
            "positive_observation_supports_acceptance": False,
            "saved_path_or_historical_witness_read": False,
            "online": False,
        },
        "policy": asdict(policy),
        "policy_hash": canonical_hash(asdict(policy)),
        "peak_rss_mib": _peak_rss_mib(),
    }
    arrays = {
        "forward_keys": _keys_array(forward_keys),
        "backward_keys": _keys_array(backward_keys),
        "observed_key_intersection": chain_array,
        "observed_target_key_intersection": target_intersection_array,
        "exact_join_keys": exact_join_array,
        "exact_target_join_keys": exact_target_array,
        "observed_target_intersection_cells": target_cells_array,
        "representative_join_position_residual_m": np.asarray(join_residuals, dtype=np.float64),
    }
    return result, arrays


def _write_json(path: Path, value) -> None:
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def _artifact_hashes(output: Path) -> dict[str, str]:
    result = {}
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "artifact_hashes.json":
            result[str(path.relative_to(output))] = sha256_file(path)
    return result


def _file_hash_snapshot(paths: Mapping[str, Path]) -> dict[str, str | None]:
    snapshot: dict[str, str | None] = {}
    for name, path in paths.items():
        try:
            snapshot[str(name)] = sha256_file(Path(path))
        except (OSError, ValueError):
            snapshot[str(name)] = None
    return snapshot


def run(inputs: Path, query: str, output: Path, policy: ReachabilityPolicy | None = None) -> dict:
    policy = policy or ReachabilityPolicy()
    inputs = Path(inputs).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=False, exist_ok=False)
    started = time.monotonic()
    source = Path(__file__).resolve()
    dependencies = [
        source,
        source.with_name("se2_semantic_guide.py"),
        source.with_name("semantic_constraint_core.py"),
        source.with_name("semantic_transition_goal_plane_topology.py"),
        source.with_name("semantic_transition_ordered_corridor.py"),
    ]
    source_paths = {str(path): path for path in dependencies}
    input_paths = {
        "input_json": inputs / f"{query}.json",
        "input_npz": inputs / f"{query}.npz",
    }
    source_hashes_before = _file_hash_snapshot(source_paths)
    input_hashes_before = _file_hash_snapshot(input_paths)
    protocol = {
        "protocol_id": PROTOCOL_ID,
        "method_id": METHOD_ID,
        "query_id": query,
        "analysis_disposition": "NON_DECISION_EXPLORATORY",
        "hard_stop_eligible": False,
        "acceptance_or_c1_decision_eligible": False,
        "policy": asdict(policy),
        "policy_hash": canonical_hash(asdict(policy)),
        "used_historical_path_or_witness": False,
        "input_kind": "request_constraint_grids_and_request_route_polyline",
        "online": False,
        "first_representative_underapproximation": True,
        "smac_analytic_expansion_implemented": False,
        "replayable_parent_chain_certificate_emitted": False,
        "positive_or_negative_observation_is_nondecisive": True,
        "input_hashes_at_start": input_hashes_before,
        "source_hashes_at_start": source_hashes_before,
    }
    arrays: dict[str, np.ndarray] = {
        "forward_keys": np.empty((0, 3), dtype=np.int32),
        "backward_keys": np.empty((0, 3), dtype=np.int32),
        "observed_key_intersection": np.empty((0, 3), dtype=np.int32),
        "observed_target_key_intersection": np.empty((0, 3), dtype=np.int32),
    }
    exception_record = None
    try:
        _write_json(output / "protocol.json", protocol)
        snapshot = output / "source_snapshot"
        snapshot.mkdir()
        for path in dependencies:
            shutil.copy2(path, snapshot / path.name)
        world = ConstraintWorld(inputs, query)
        result, arrays = analyze_world(world, policy)
    except Exception as error:  # Preserve every partial directory as excluded evidence.
        exception_record = {
            "exception_type": type(error).__name__,
            "exception_message": str(error),
            "domain_failure_code": getattr(error, "code", None),
        }
        with (output / "exception.txt").open("x", encoding="utf-8") as stream:
            stream.write(traceback.format_exc())
        result = {
            "protocol_id": PROTOCOL_ID,
            "method_id": METHOD_ID,
            "query_id": query,
            "result_code": "EXCLUDED_EXCEPTION",
            "analysis_disposition": "EXCLUDED",
            "hard_stop_eligible": False,
            "acceptance_evidence": False,
            "c1_infeasibility_evidence": False,
            "exception": exception_record,
            "traversals_open_exhausted_within_underapproximation": False,
            "observed_target_key_intersection_nondecisive": False,
            "observed_exact_representative_target_join_nondecisive": False,
            "continuous_space_witness_proved": False,
            "replayable_witness_generated": False,
            "parent_chain_certificate_emitted": False,
            "smac_analytic_expansion_implemented": False,
            "r2_resource_analysis_status": "NOT_RUN_EXCLUDED_EXCEPTION",
            "used_historical_path_or_witness": False,
            "online": False,
        }

    command = (
        f"PYTHONPATH={Path(__file__).parents[1]} /usr/bin/python3 -m "
        f"arena_evaluation.semantic_transition_se2_chain_reachability "
        f"--inputs {inputs} --query {query} --output {output}\n"
    )
    with (output / "reproduction_command.txt").open("x", encoding="utf-8") as stream:
        stream.write(command)
    source_hashes_after = _file_hash_snapshot(source_paths)
    input_hashes_after = _file_hash_snapshot(input_paths)
    drift = {
        "input_changed": input_hashes_after != input_hashes_before,
        "source_changed": source_hashes_after != source_hashes_before,
    }
    if any(drift.values()):
        prior_code = result.get("result_code")
        result["result_code"] = "EXCLUDED_INPUT_OR_SOURCE_DRIFT"
        result["analysis_disposition"] = "EXCLUDED"
        result["pre_exclusion_exploratory_result_code"] = prior_code
    result.update({
        "hard_stop_eligible": False,
        "acceptance_evidence": False,
        "c1_infeasibility_evidence": False,
        "wall_s": time.monotonic() - started,
        "python": platform.python_version(),
        "input_hashes_at_start": input_hashes_before,
        "input_hashes_at_end": input_hashes_after,
        "source_hashes_at_start": source_hashes_before,
        "source_hashes_at_end": source_hashes_after,
        "hash_drift": drift,
    })
    disposition = result["analysis_disposition"]
    status = {
        "status": disposition,
        "result_code": result["result_code"],
        "hard_stop_eligible": False,
        "acceptance_or_c1_decision_eligible": False,
        "exception": exception_record,
        "input_hashes_at_start": input_hashes_before,
        "input_hashes_at_end": input_hashes_after,
        "source_hashes_at_start": source_hashes_before,
        "source_hashes_at_end": source_hashes_after,
        "hash_drift": drift,
    }
    np.savez_compressed(output / "reachability_sets.npz", **arrays)
    _write_json(output / "result.json", result)
    _write_json(output / "STATUS.json", status)
    _write_json(output / "artifact_hashes.json", _artifact_hashes(output))
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--query", default=QUERY_ID)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-per-direction", type=float, default=120.0)
    parser.add_argument("--maximum-states-per-direction", type=int, default=1_000_000)
    args = parser.parse_args(argv)
    try:
        policy = ReachabilityPolicy(
            timeout_per_direction_s=args.timeout_per_direction,
            maximum_states_per_direction=args.maximum_states_per_direction,
        )
    except ValueError as error:
        parser.error(str(error))
    result = run(args.inputs, args.query, args.output, policy)
    print(json.dumps({
        "query_id": args.query,
        "result_code": result["result_code"],
        "analysis_disposition": result["analysis_disposition"],
        "hard_stop_eligible": result["hard_stop_eligible"],
        "observed_target_key_intersection_nondecisive": result.get(
            "observed_target_key_intersection_nondecisive", False,
        ),
        "replayable_witness_generated": result.get("replayable_witness_generated", False),
        "wall_s": result["wall_s"],
    }, indent=2, sort_keys=True))
    # This probe is non-decision by contract.  A future implementation may
    # return success only after emitting and independently replaying a complete
    # witness; the current module never does so.
    return 3


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "HARD_MAXIMUM_CURVATURE_1PM",
    "METHOD_ID",
    "PROTOCOL_ID",
    "QUERY_ID",
    "ReachabilityFailure",
    "ReachabilityPolicy",
    "analyze_world",
]
