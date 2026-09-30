"""Exact-pose Dubins graph edges and independent constrained-path audit.

Raster values are immutable. Cropping only changes storage; cell lookup always
uses the original map's origin and dimensions to preserve boundary rounding.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from .planner_benchmark.map_utils import HospitalMap
from .planner_benchmark.models import Query
from .path_audit import PathAuditor
from .se2_semantic_guide import _dubins_words, _advance, wrap_angle
from .semantic_map import canonical_hash, sha256_file
from .semantic_path_audit import SemanticPathAuditor
from .semantic_rasterizer import grid_hash

SPACING = 0.025
PADDED_FOOTPRINT = ((.265, .225), (.265, -.225), (-.265, -.225), (-.265, .225))


class CropMap(HospitalMap):
    def world_to_cell(self, x, y):
        col = math.floor((float(x) - self.full_origin[0]) / self.resolution) - self.col0
        row = self.full_height - 1 - math.floor((float(y) - self.full_origin[1]) / self.resolution) - self.row0
        return (row, col) if 0 <= row < self.height and 0 <= col < self.width else None

    def cell_to_world(self, cell):
        row, col = cell
        return (self.full_origin[0] + (col + self.col0 + .5) * self.resolution,
                self.full_origin[1] + (self.full_height - row - self.row0 - .5) * self.resolution)


@dataclass
class Edge:
    word: str
    params: tuple
    radius: float
    start: tuple
    goal: tuple
    length: float
    samples: np.ndarray
    n: int = 0
    correct: int = 0
    target: int = 0

    @property
    def side_resource(self):
        return 5 * self.correct - 4 * self.n

    @property
    def target_resource(self):
        return 2 * self.target - self.n

    def certificate(self):
        return {"word": self.word, "params": self.params, "radius_m": self.radius,
                "start": self.start, "goal": self.goal, "arc_length_m": self.length,
                "sample_count": len(self.samples), "lane_samples": self.n,
                "correct_samples": self.correct, "target_samples": self.target,
                "sample_hash": hashlib.sha256(self.samples.tobytes()).hexdigest()}


def dubins_choices(start, goal, radius=.401):
    start, goal = tuple(map(float, start)), tuple(map(float, goal))
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    direction = math.atan2(dy, dx)
    return sorted(_dubins_words((start[2]-direction) % (2*math.pi),
                               (goal[2]-direction) % (2*math.pi), math.hypot(dx, dy)/radius),
                  key=lambda item: (sum(item[1]), item[0]))


def dubins_edge(start, goal, radius=.401, choice=0):
    """Exact arc construction, no coordinate/yaw snapping and no zero segments."""
    start, goal = tuple(map(float, start)), tuple(map(float, goal))
    choices = dubins_choices(start, goal, radius)
    if choice >= len(choices):
        return None
    word, params = choices[choice]
    return dubins_edge_from_parameters(start, goal, radius, word, params)


def dubins_edge_from_parameters(start, goal, radius, word, params):
    """Build an exact edge from one already-solved Dubins word.

    Enumeration callers can reject choices from analytic length before
    allocating samples.  The resulting accepted edge is byte-identical to
    :func:`dubins_edge`.
    """
    start, goal = tuple(map(float, start)), tuple(map(float, goal))
    word = str(word)
    params = tuple(float(value) for value in params)
    pieces, pose = [], start
    for kind, parameter in zip(word, params):
        length = parameter * radius
        if length < 1e-10:
            continue
        # Strictly less than the auditor spacing avoids ceil(1+roundoff) adding
        # an unplanned semantic sample on exact .025-m straight intervals.
        count = max(1, math.ceil(length / (SPACING * (1-1e-10))))
        t = np.arange(1, count+1, dtype=np.float64) * (parameter / count)
        x, y, yaw = pose
        if kind == "S":
            samples = np.column_stack((x+t*radius*math.cos(yaw), y+t*radius*math.sin(yaw), np.full(count, yaw)))
        else:
            sign = 1 if kind == "L" else -1
            angle = yaw + sign*t
            samples = np.column_stack((x+sign*radius*(np.sin(angle)-math.sin(yaw)),
                                       y+sign*radius*(-np.cos(angle)+math.cos(yaw)),
                                       (angle+math.pi) % (2*math.pi)-math.pi))
        pieces.append(samples)
        pose = tuple(samples[-1])
    if not pieces:
        return None
    samples = np.concatenate(pieces)
    error = np.array((samples[-1, 0]-goal[0], samples[-1, 1]-goal[1], wrap_angle(samples[-1, 2]-goal[2])))
    if np.max(np.abs(error)) > 1e-8:
        raise ValueError(f"analytic endpoint construction error: {error}")
    # Only floating-point reconstruction residue is replaced. All goal values
    # are the requested exact pose; this is not an endpoint attachment change.
    samples[-1] = goal
    return Edge(word, params, radius, start, goal, radius*sum(params), samples)


def dense_interpolate(path):
    result = [tuple(path[0])]
    for a, b in zip(path, path[1:]):
        delta = np.array((b[0]-a[0], b[1]-a[1], wrap_angle(b[2]-a[2])))
        steps = max(1, math.ceil(math.hypot(delta[0], delta[1])/.025), math.ceil(abs(delta[2])/math.radians(1)))
        for i in range(1, steps+1):
            p = np.asarray(a)+delta*(i/steps)
            p[2] = wrap_angle(p[2])
            result.append(tuple(p))
    return np.asarray(result)


class ConstraintWorld:
    def __init__(
        self, input_dir, query_id, *, master_override=None,
        expected_master_hash=None, meta_override=None, arrays_override=None,
    ):
        input_dir = Path(input_dir)
        self.meta = (
            dict(meta_override) if meta_override is not None
            else json.loads((input_dir / f"{query_id}.json").read_text())
        )
        data_path = input_dir / f"{query_id}.npz"
        if arrays_override is None and sha256_file(data_path) != self.meta["npz_sha256"]:
            raise ValueError("input archive hash mismatch")
        self.input_path = data_path if arrays_override is None else None
        desc = self.meta["map"]
        self.selected = self.meta["selected_lane_labels"]
        def load(data):
            labels = data["labels"]
            rows, cols = np.where(np.isin(labels, self.selected))
            if not len(rows):
                raise ValueError("empty selected lane")
            r0, r1 = max(0, int(rows.min())-20), min(desc["height"], int(rows.max())+21)
            c0, c1 = max(0, int(cols.min())-20), min(desc["width"], int(cols.max())+21)
            self.grids = {"labels": labels[r0:r1, c0:c1].copy()}
            del labels, rows, cols
            for name in ("master", "occupancy", "allowed", "error", "correct", "right", "left", "hard", "no_stopping"):
                source = data[name]
                if name == "master" and master_override is not None:
                    source = np.asarray(master_override, dtype=np.uint8)
                    if source.shape != (int(desc["height"]), int(desc["width"])):
                        raise ValueError("verified master override shape mismatch")
                    actual_hash = grid_hash(source)
                    bound_hash = str(expected_master_hash or self.meta["expected_master_hash"])
                    if actual_hash != bound_hash or actual_hash != self.meta["expected_master_hash"]:
                        raise ValueError("verified master override hash mismatch")
                self.grids[name] = source[r0:r1, c0:c1].copy()
            self._full_occupancy = np.asarray(data["occupancy"])
            self._full_allowed = np.asarray(data["allowed"])
            return r0, r1, c0, c1
        if arrays_override is None:
            with np.load(data_path) as data:
                r0, r1, c0, c1 = load(data)
        else:
            r0, r1, c0, c1 = load(arrays_override)
        self.master = self.grids["master"]
        self.obstacle = self.master >= 254
        distance = cv2.distanceTransform((~self.obstacle).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*desc["resolution"]
        self.map = CropMap(Path(desc["image_path"]).with_suffix(".yaml"), Path(desc["image_path"]),
                           desc["resolution"], tuple(desc["origin"]), c1-c0, r1-r0,
                           self.grids["occupancy"], distance)
        self.map.full_origin, self.map.full_height = desc["origin"], desc["height"]
        self.map.row0, self.map.col0 = r0, c0
        self.query = SimpleNamespace(**self.meta["query"])
        self.start, self.goal = tuple(self.query.start), tuple(self.query.goal)
        self.bound_length = max(40., 4*math.dist(self.start[:2], self.goal[:2]))
        self.pose_checks = self.exact_checks = 0
        self.fast_dense_edge_certificates = 0
        self.dense_edge_fallbacks = 0
        self._canonical_auditor = None
        self._canonical_allowed = None
        self.safe_threshold = math.hypot(.265, .225) + math.sqrt(2)*desc["resolution"]

    def fast_dense_edge_certificate(self, samples):
        """Conservatively prove a dense swept check cannot add a failure.

        Edge samples are already closer than 0.025 m in translation.  When
        their full 3x3 grid neighborhood stays inside the same lane/ROI and
        the obstacle distance exceeds the orientation-independent footprint
        circumradius plus cell and inter-sample margins, interpolation cannot
        reach a new hard cell or obstacle.  Anything uncertain falls back to
        the unchanged dense check.
        """
        samples = np.asarray(samples, dtype=np.float64)
        rows, cols, inside = self.cells(samples)
        if not np.all(inside):
            return False
        neighbor_rows = rows[:, None] + np.asarray((-1, 0, 1), dtype=np.int64)[None, :]
        neighbor_cols = cols[:, None] + np.asarray((-1, 0, 1), dtype=np.int64)[None, :]
        rr = np.repeat(neighbor_rows[:, :, None], 3, axis=2).reshape(-1)
        cc = np.repeat(neighbor_cols[:, None, :], 3, axis=1).reshape(-1)
        if (
            np.any(rr < 0) or np.any(rr >= self.map.height)
            or np.any(cc < 0) or np.any(cc >= self.map.width)
        ):
            return False
        selected = np.isin(self.grids["labels"][rr, cc], self.selected)
        grid_safe = (
            (self.master[rr, cc] < 253)
            & self.grids["allowed"][rr, cc].astype(bool)
            & ~self.grids["hard"][rr, cc].astype(bool)
            & selected
        )
        if not np.all(grid_safe):
            return False
        clearance = self.map.distance_m[rows, cols]
        return bool(np.all(clearance > self.safe_threshold + SPACING))

    def cells(self, samples):
        samples = np.asarray(samples)
        cols = np.floor((samples[:, 0]-self.map.full_origin[0])/self.map.resolution).astype(np.int64)-self.map.col0
        rows = self.map.full_height-1-np.floor((samples[:, 1]-self.map.full_origin[1])/self.map.resolution).astype(np.int64)-self.map.row0
        inside = (rows>=0)&(rows<self.map.height)&(cols>=0)&(cols<self.map.width)
        return rows, cols, inside

    def semantic_counts(self, samples):
        rows, cols, inside = self.cells(samples)
        if not np.all(inside):
            return None
        lane = np.isin(self.grids["labels"][rows, cols], self.selected)
        finite = np.isfinite(self.grids["right"][rows, cols])
        if not np.all(lane & finite):
            return None
        correct = self.grids["correct"][rows, cols].astype(bool)
        target = correct & (self.grids["error"][rows, cols] <= .50)
        return len(rows), int(correct.sum()), int(target.sum())

    def collision_free(self, samples):
        samples = np.asarray(samples)
        self.pose_checks += len(samples)
        rows, cols, inside = self.cells(samples)
        if not np.all(inside):
            return False
        if np.any(self.master[rows, cols] >= 253):
            return False
        if np.any(~self.grids["allowed"][rows, cols]) or np.any(self.grids["hard"][rows, cols]):
            return False
        near = np.where(self.map.distance_m[rows, cols] <= self.safe_threshold)[0]
        half = self.map.resolution/2
        for index in near:
            self.exact_checks += 1
            x, y, yaw = samples[index]
            row, col = rows[index], cols[index]
            span = math.ceil(math.hypot(.265, .225)/self.map.resolution)+2
            r0, r1 = max(0, row-span), min(self.map.height, row+span+1)
            c0, c1 = max(0, col-span), min(self.map.width, col+span+1)
            rr, cc = np.where(self.obstacle[r0:r1, c0:c1])
            if not len(rr):
                continue
            rr, cc = rr+r0, cc+c0
            dx = self.map.full_origin[0]+(cc+self.map.col0+.5)*self.map.resolution-x
            dy = self.map.full_origin[1]+(self.map.full_height-rr-self.map.row0-.5)*self.map.resolution-y
            c, s = math.cos(yaw), math.sin(yaw)
            intersects = ((np.abs(dx) <= .265*abs(c)+.225*abs(s)+half) &
                          (np.abs(dy) <= .265*abs(s)+.225*abs(c)+half) &
                          (np.abs(c*dx+s*dy) <= .265+half*(abs(c)+abs(s))) &
                          (np.abs(-s*dx+c*dy) <= .225+half*(abs(c)+abs(s))))
            if np.any(intersects):
                return False
        return True

    def validate_edge(self, edge, *, dense=False):
        if edge is None:
            return False
        counts = self.semantic_counts(edge.samples)
        if counts is None:
            return False
        if dense:
            coarse = np.vstack((edge.start, edge.samples))
            if self.fast_dense_edge_certificate(coarse):
                self.fast_dense_edge_certificates += 1
            else:
                self.dense_edge_fallbacks += 1
                if not self.collision_free(dense_interpolate(coarse)):
                    return False
        elif not self.collision_free(edge.samples):
            return False
        edge.n, edge.correct, edge.target = counts
        return True

    def audit(self, edges):
        if not edges:
            return {"gate_passed": False, "failure_code": "EMPTY_PATH"}, np.empty((0, 3))
        path = np.vstack((edges[0].start, *(edge.samples for edge in edges)))
        points = [{"x": float(x), "y": float(y), "yaw": float(yaw), "source": "kinematic",
                   "motion_direction": "forward", "steering": 0., "planner_backend": "constrained_dubins_graph",
                   "backend_version": "feasibility_r0"} for x,y,yaw in path]
        if self._canonical_auditor is None:
            # PathAuditor vectorizes world-to-cell using origin/width/height.
            # Give it the original full map, not the storage-only crop.
            desc = self.meta["map"]
            if self.input_path is None:
                occupancy = self._full_occupancy
                self._canonical_allowed = self._full_allowed
            else:
                with np.load(self.input_path) as data:
                    occupancy = data["occupancy"]
                    self._canonical_allowed = data["allowed"]
            raw_distance = cv2.distanceTransform((occupancy == 0).astype(np.uint8), cv2.DIST_L2,
                                                 cv2.DIST_MASK_PRECISE)*desc["resolution"]
            full_map = HospitalMap(self.map.yaml_path, self.map.image_path, desc["resolution"],
                                   tuple(desc["origin"]), desc["width"], desc["height"], occupancy, raw_distance)
            self._canonical_auditor = PathAuditor(SimpleNamespace(hospital_map=full_map), source_commit="stage0_bound_source")
        canonical = self._canonical_auditor.audit(self.query, points, self._canonical_allowed)
        # Invoke the unchanged semantic sampler, preserving the historical
        # sample-based score. Extra <=1-degree poses are safety-only.
        semantic_samples = np.asarray(SemanticPathAuditor._samples(SimpleNamespace(hospital_map=self.map), points))
        counts = self.semantic_counts(semantic_samples)
        dense = dense_interpolate(path)
        collision_free = self.collision_free(dense)
        rows, cols, inside = self.cells(semantic_samples)
        all_lane = counts is not None
        safe_rows, safe_cols = rows[inside], cols[inside]
        errors = self.grids["error"][safe_rows, safe_cols]
        median = float(np.median(errors)) if len(errors) else None
        n, c, t = counts or (0,0,0)
        exact = bool(np.array_equal(path[0], self.start) and np.array_equal(path[-1], self.goal))
        continuity = all(np.array_equal(a.samples[-1], b.start) for a,b in zip(edges, edges[1:]))
        goalcell = self.map.world_to_cell(*self.goal[:2])
        no_stopping = bool(goalcell is None or self.grids["no_stopping"][goalcell])
        max_control_curvature = max((1/e.radius if any(k != "S" and p > 1e-10 for k,p in zip(e.word,e.params)) else 0) for e in edges)
        trace_replay = all(np.array_equal(replay_edge(e.certificate()).samples, e.samples) for e in edges)
        gate = bool(canonical.final_valid_success and collision_free and exact and continuity and trace_replay
                    and all_lane and n and 5*c >= 4*n and 2*t > n and median <= .5
                    and not no_stopping and max_control_curvature <= 2.50
                    and sum(e.length for e in edges) <= self.bound_length)
        return {"gate_passed": gate, "canonical": canonical.metrics, "canonical_diagnostics": canonical.diagnostics(),
                "exact_endpoint_xy_yaw": exact, "edge_continuity": continuity, "trace_replay_exact": trace_replay,
                "padded_effective_master_collision_free": collision_free,
                "dense_safety_pose_count": len(dense), "same_lane_instance": all_lane,
                "lane_sample_count": n, "correct_sample_count": c, "target_sample_count": t,
                "lane_correct_side_ratio": c/n if n else None, "lane_target_band_ratio": t/n if n else None,
                "lane_target_error_p50_m": median, "side_resource": 5*c-4*n, "target_resource": 2*t-n,
                "no_stopping_goal_violation": no_stopping, "maximum_control_curvature_1pm": max_control_curvature,
                "maximum_geometric_curvature_1pm": canonical.metrics.get("maximum_curvature"),
                "path_length_m": canonical.metrics.get("path_length_m"), "arc_length_m": sum(e.length for e in edges),
                "pose_checks": self.pose_checks, "exact_rectangle_checks": self.exact_checks,
                "path_sha256": hashlib.sha256(path.tobytes()).hexdigest()}, path


def replay_edge(certificate):
    start, goal, radius = certificate["start"], certificate["goal"], certificate["radius_m"]
    choices = dubins_choices(start, goal, radius)
    for index, (word, params) in enumerate(choices):
        if word == certificate["word"] and np.array_equal(params, certificate["params"]):
            return dubins_edge(start, goal, radius, index)
    raise ValueError("Dubins certificate word/parameters cannot be replayed")
