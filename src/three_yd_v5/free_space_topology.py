"""V7 map-only L1: coarse, obstacle-aware guidance without traffic semantics.

The generated empty semantic document is an adapter for the existing map and
vehicle loader.  It does not claim that a free pixel is a road or infer any
direction or right-hand preference.
"""
from __future__ import annotations

import heapq
import hashlib
import math
import os
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from arena_evaluation.semantic_map import SemanticMapV1, canonical_hash
from arena_evaluation.planner_benchmark.map_utils import HospitalMap, _distance_to_occupied
from arena_3d_v1.multires_topology import TopologyRoute
from arena_3d_v1.semantic_world import SemanticWorld, SweptChecker, json_write


class DeferredOccupancyMap(HospitalMap):
    """Map-server-equivalent trinary PGM loading; distance field is on demand."""

    world_to_cell = HospitalMap.world_to_cell
    cell_to_world = HospitalMap.cell_to_world

    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path).resolve()
        config = yaml.safe_load(self.yaml_path.read_text())
        image = Path(config['image'])
        self.image_path = (image if image.is_absolute() else self.yaml_path.parent / image).resolve()
        pixels = np.asarray(Image.open(self.image_path).convert('L'))
        self.height, self.width = pixels.shape
        self.resolution = float(config['resolution'])
        self.origin = tuple(map(float, config['origin']))
        sha = hashlib.sha256()
        with self.image_path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                sha.update(block)
        self._sha256 = sha.hexdigest()
        occupied_threshold = float(config.get('occupied_thresh', .65))
        free_threshold = float(config.get('free_thresh', .196))
        negate = bool(config.get('negate', 0))
        self.occupancy = np.full(pixels.shape, -1, np.int8)
        if negate:
            self.occupancy[pixels > 255. * occupied_threshold] = 100
            self.occupancy[pixels < 255. * free_threshold] = 0
        else:
            self.occupancy[pixels < 255. * (1. - occupied_threshold)] = 100
            self.occupancy[pixels > 255. * (1. - free_threshold)] = 0
        self._distance_m = None
        self._component_labels = {}

    @property
    def sha256(self):
        return self._sha256

    @property
    def distance_m(self):
        if self._distance_m is None:
            self._distance_m = _distance_to_occupied(self.occupancy == 100, self.resolution)
        return self._distance_m


class MapOnlyWorld:
    """Neutral world adapter; large distance/label fields are lazy for L1."""

    cells = SemanticWorld.cells
    world_points = SemanticWorld.world_points
    audit = SemanticWorld.audit

    def __init__(self, map_yaml):
        started = time.perf_counter()
        self.map = DeferredOccupancyMap(map_yaml)
        self.semantic = SemanticMapV1('map', self.map.resolution, self.map.origin,
                                      self.map.width, self.map.height, self.map.sha256)
        self.features = {}
        self.rules = {}
        self.vehicle = (.265, .225, .40)
        self.hard = self.map.occupancy != 0
        self.binding = {'schema': '3YD-V7-map-only-world-v1', 'map': self.map.sha256,
                        'semantic': self.semantic.semantic_map_hash,
                        'resolution': self.map.resolution, 'origin': self.map.origin,
                        'vehicle': self.vehicle}
        self.key = canonical_hash(self.binding)
        self._safe = None
        self._labels = None
        self._checker = None
        self.v7_map_only = True
        self.preparation_ms = (time.perf_counter()-started)*1000

    @property
    def safe(self):
        if self._safe is None:
            distance = ndimage.distance_transform_edt(~self.hard)
            clearance = math.hypot(*self.vehicle[:2]) + .05 + math.sqrt(2)*self.map.resolution/2
            self._safe = distance*self.map.resolution >= clearance
        return self._safe

    @property
    def labels(self):
        if self._labels is None:
            self._labels = np.zeros(self.hard.shape, np.int16)
        return self._labels

    @property
    def checker(self):
        if self._checker is None:
            self._checker = SweptChecker(self.hard, self.map.resolution, self.map.origin,
                                         *self.vehicle[:2])
        return self._checker

    def sample_labels(self, poses):
        row, col = self.cells(poses)
        result = np.zeros(len(row), int)
        result[(row < 0) | (col < 0) | (row >= self.map.height) | (col >= self.map.width)] = -1
        return result


def load_map_only_world(map_yaml, cache_root):
    """Load a 0.05 m occupancy map without requiring semantic inputs."""
    world = MapOnlyWorld(map_yaml)
    if not math.isclose(world.map.resolution, .05, rel_tol=0., abs_tol=1e-9):
        raise ValueError('V7_MAP_ONLY_REQUIRES_0_05_M_RESOLUTION')
    return world


class FreeSpaceTopology:
    """One neutral free-space region plus per-query coarse guides and scopes."""

    SCHEMA = '3YD-V7-r0-map-only-l1-v1'
    mode = 'free_space'

    def __init__(self, world, cache_root, *, coarse_factor=24, expansion_budget=400000):
        started = time.perf_counter()
        if not math.isclose(world.map.resolution, .05, rel_tol=0., abs_tol=1e-9):
            raise ValueError('V7_MAP_ONLY_REQUIRES_0_05_M_RESOLUTION')
        self.world = world
        self.key = canonical_hash([self.SCHEMA, world.map.sha256, world.key, coarse_factor])
        self.regions = {1: {'id': 1, 'kind': 'junction_area', 'name': 'free_space_adapter'}}
        self._subregions = None
        self.tasks, self.contexts, self._cache = {}, {}, {}
        self.factor = int(coarse_factor)
        self.expansion_budget = int(expansion_budget)
        self.goal_stop = os.environ.get('THREEYD_V7_GOAL_STOP', '1') == '1'
        # L1 only establishes coarse free-space connectivity.  Vehicle
        # clearance belongs to the existing L2/L3 checks, so do not erode
        # doorways by the Jackal circle here.
        self.coarse_free = self._coarsen(~world.hard, self.factor)
        self._free_scales = {self.factor: self.coarse_free}
        self._graphs = {}
        # Compile small/medium coarse graphs once at startup.  The former
        # Python heap A* spent most query time expanding thousands of cells;
        # compiled sparse Dijkstra is quicker even without a heuristic.
        for scale in dict.fromkeys((self.factor, 12, 6)):
            free = self._free_scales.setdefault(scale, self._coarsen(~world.hard, scale))
            if free.size <= 500000:
                self._graphs[scale] = self._compile_graph(free)
        self._components = {}
        self.edges_count = 0
        self.portal_count = 0
        self.preparation_ms = (time.perf_counter() - started) * 1000
        self._probe_stats = {}
        self._last_first_phase_resume = False
        json_write(Path(cache_root) / 'v7_map_only_l1.json', {
            'schema': self.SCHEMA, 'map_sha256': world.map.sha256,
            'source_resolution_m': world.map.resolution,
            'coarse_resolution_m': world.map.resolution * self.factor,
            'traffic_rules': None, 'right_side_rule': None,
            'coarse_cells': int(self.coarse_free.size),
            'coarse_free_cells': int(self.coarse_free.sum()),
        })

    @property
    def subregions(self):
        if self._subregions is None:
            self._subregions = np.asarray(~self.world.hard, np.int16)
        return self._subregions

    @staticmethod
    def _coarsen(safe, factor):
        height, width = safe.shape
        pad_h = (-height) % factor
        pad_w = (-width) % factor
        padded = np.pad(safe, ((0, pad_h), (0, pad_w)), constant_values=False)
        return padded.reshape(padded.shape[0] // factor, factor,
                              padded.shape[1] // factor, factor).all(axis=(1, 3))

    @staticmethod
    def _compile_graph(free):
        """Eight-connected free-cell graph; diagonal corner cutting forbidden."""
        height, width = free.shape
        ids = np.arange(free.size, dtype=np.int32).reshape(free.shape)
        sources, targets, weights = [], [], []
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            r0, r1 = max(0, -dr), min(height, height-dr)
            c0, c1 = max(0, -dc), min(width, width-dc)
            valid = (free[r0:r1, c0:c1] &
                     free[r0+dr:r1+dr, c0+dc:c1+dc])
            if dr and dc:
                valid &= (free[r0+dr:r1+dr, c0:c1] &
                          free[r0:r1, c0+dc:c1+dc])
            sources.append(ids[r0:r1, c0:c1][valid])
            targets.append(ids[r0+dr:r1+dr, c0+dc:c1+dc][valid])
            weights.append(np.full(int(valid.sum()),
                                   math.sqrt(2) if dr and dc else 1.,
                                   dtype=np.float32))
        return csr_matrix((np.concatenate(weights),
                           (np.concatenate(sources), np.concatenate(targets))),
                          shape=(free.size, free.size))

    def _cell(self, xy, factor):
        row, col = self.world.cells(np.asarray(xy, float).reshape(1, 2))
        return int(row[0] // factor), int(col[0] // factor)

    def _xy(self, cells, factor):
        cells = np.asarray(cells, int)
        fine_r = np.minimum(cells[:, 0] * factor + factor // 2, self.world.map.height - 1)
        fine_c = np.minimum(cells[:, 1] * factor + factor // 2, self.world.map.width - 1)
        return self.world.world_points(fine_r, fine_c)

    def _clear_line(self, a, b):
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        n = max(2, int(math.ceil(np.linalg.norm(b - a) / self.world.map.resolution * 2)) + 1)
        sample = a + np.linspace(0., 1., n)[:, None] * (b - a)
        rr, cc = self.world.cells(sample)
        good = ((rr >= 0) & (cc >= 0) & (rr < self.world.map.height)
                & (cc < self.world.map.width))
        return bool(good.all() and (~self.world.hard[rr, cc]).all())

    @staticmethod
    def _snap(cell, free, max_radius=4):
        r, c = cell
        height, width = free.shape
        if 0 <= r < height and 0 <= c < width and free[r, c]:
            return cell
        for radius in range(1, max_radius + 1):
            r0, r1 = max(0, r-radius), min(height, r+radius+1)
            c0, c1 = max(0, c-radius), min(width, c+radius+1)
            candidate = np.argwhere(free[r0:r1, c0:c1])
            if len(candidate):
                candidate += (r0, c0)
                picked = candidate[np.argmin(np.sum((candidate - (r, c)) ** 2, axis=1))]
                return tuple(map(int, picked))
        return None

    def _astar(self, start, goal, free, factor):
        if start == goal:
            return [start], 0
        height, width = free.shape
        graph = self._graphs.get(factor)
        if graph is not None:
            src, dst = start[0]*width+start[1], goal[0]*width+goal[1]
            if self.goal_stop:
                from .goal_search import goal_dijkstra
                distance, predecessors, expanded = goal_dijkstra(graph, src, dst)
            else:
                distances, predecessors = dijkstra(graph, indices=src,
                                                   return_predecessors=True)
                expanded = int(np.isfinite(distances).sum())
                distance = distances[dst]
            if not np.isfinite(distance):
                return None, expanded
            out = []
            at = dst
            while at != src:
                out.append(divmod(at, width))
                at = int(predecessors[at])
                if at < 0:
                    raise RuntimeError('V7_L1_GRAPH_PREDECESSOR_BROKEN')
            out.append(start)
            return out[::-1], expanded
        dist = np.full(free.shape, np.inf, np.float64)
        closed = np.zeros(free.shape, bool)
        parent = np.full((height, width), -1, np.int32)
        dist[start] = 0.
        octile = lambda r, c: max(abs(r-goal[0]), abs(c-goal[1])) + (math.sqrt(2)-1)*min(abs(r-goal[0]), abs(c-goal[1]))
        # A bounded, mildly weighted A* is appropriate for a soft guide: L2
        # owns the final grid route and may depart from this line.
        weight = 1.5
        heap = [(weight*octile(*start), 0., start[0], start[1])]
        expanded = 0
        neighbors = [(-1,0,1.), (1,0,1.), (0,-1,1.), (0,1,1.),
                     (-1,-1,math.sqrt(2)), (-1,1,math.sqrt(2)),
                     (1,-1,math.sqrt(2)), (1,1,math.sqrt(2))]
        while heap and expanded < self.expansion_budget:
            _, cost, r, c = heapq.heappop(heap)
            if closed[r, c] or cost > float(dist[r, c]) + 1e-5:
                continue
            closed[r, c] = True
            expanded += 1
            if (r, c) == goal:
                out = []
                at = r * width + c
                while at >= 0:
                    out.append(divmod(at, width))
                    at = int(parent[out[-1]])
                return out[::-1], expanded
            for dr, dc, step in neighbors:
                nr, nc = r+dr, c+dc
                if nr < 0 or nc < 0 or nr >= height or nc >= width or not free[nr, nc] or closed[nr, nc]:
                    continue
                if dr and dc and (not free[r+dr, c] or not free[r, c+dc]):
                    continue
                new = cost + step
                if new + 1e-5 < dist[nr, nc]:
                    dist[nr, nc] = new
                    parent[nr, nc] = r * width + c
                    heapq.heappush(heap, (new + weight*octile(nr, nc), new, nr, nc))
        return None, expanded

    def _guide(self, start_xy, goal_xy):
        if self._clear_line(start_xy, goal_xy):
            return np.asarray([start_xy, goal_xy], float), {'search': 'direct', 'expanded': 0}
        total_expanded = 0
        for factor in dict.fromkeys((self.factor, 12, 6, 3)):
            if factor not in self._free_scales:
                self._free_scales[factor] = self._coarsen(~self.world.hard, factor)
            free = self._free_scales[factor]
            start = self._snap(self._cell(start_xy, factor), free)
            goal = self._snap(self._cell(goal_xy, factor), free)
            if start is None or goal is None:
                continue
            if factor not in self._components:
                self._components[factor] = ndimage.label(free)[0]
            components = self._components[factor]
            if components[start] == 0 or components[start] != components[goal]:
                continue
            start_center, goal_center = self._xy([start, goal], factor)
            if not self._clear_line(start_xy, start_center) or not self._clear_line(goal_center, goal_xy):
                continue
            cells, expanded = self._astar(start, goal, free, factor)
            total_expanded += expanded
            if cells is None:
                continue
            coords = self._xy(cells, factor)
            # Compress grid staircases without crossing a blocked fine cell.
            turns = [coords[0]]
            last = None
            for a, b in zip(cells, cells[1:]):
                direction = (b[0]-a[0], b[1]-a[1])
                if last is not None and direction != last:
                    turns.append(self._xy([a], factor)[0])
                last = direction
            turns.append(coords[-1])
            points = [np.asarray(start_xy, float), *turns, np.asarray(goal_xy, float)]
            simplified = [points[0]]
            index = 0
            while index < len(points)-1:
                farthest = index+1
                for j in range(index+2, min(len(points), index+42)):
                    if self._clear_line(points[index], points[j]):
                        farthest = j
                    else:
                        break
                simplified.append(points[farthest])
                index = farthest
            line = np.asarray(simplified, float)
            keep = np.r_[True, np.linalg.norm(np.diff(line, axis=0), axis=1) > 1e-6]
            search = ('goal_stop_dijkstra' if self.goal_stop else 'full_component_dijkstra') if factor in self._graphs else 'bounded_weighted_astar'
            return line[keep], {'search': search, 'factor': factor,
                                'expanded': total_expanded, 'raw_cells': len(cells)}
        raise RuntimeError('V7_L1_NO_FREE_SPACE_ROUTE')

    def routes(self, query, limit=1):
        tick = time.perf_counter()
        key = (query.query_id, tuple(query.start), tuple(query.goal))
        if key in self._cache:
            self._probe_stats = {'cache_hit': True, 'wall_ms': (time.perf_counter()-tick)*1000}
            return self._cache[key][:limit]
        start, goal = np.asarray(query.start[:2], float), np.asarray(query.goal[:2], float)
        for point in (start, goal):
            rr, cc = self.world.cells(point[None, :])
            if not (0 <= rr[0] < self.world.map.height and 0 <= cc[0] < self.world.map.width
                    and not self.world.hard[rr[0], cc[0]]):
                raise ValueError('V7_L1_ENDPOINT_NOT_CLEAR')
        guide, diagnostics = self._guide(start, goal)
        task = {'schema': self.SCHEMA, 'mode': self.mode,
                'right_reference_polyline': guide.tolist(),  # legacy interface name; neutral geometry
                'region_sequence': [1], 'start_xy': start.tolist(), 'goal_xy': goal.tolist(),
                'frame_id': 'map', 'map_hash': self.world.map.sha256,
                'rules_hash': None, 'right_side_rule': None,
                'selected_route_has_branches': False, 'scope_half_width_m': 2.0}
        signature = canonical_hash(task)
        task['signature'] = signature
        self.tasks[signature] = task
        self.contexts[signature] = {'roads': [], 'unique_roads': [], 'endpoint_access': {
            role: {'semantic_region_id': 1, 'road_id': 1, 'allow_local_cross_road': False}
            for role in ('start', 'goal')}}
        route = TopologyRoute((1,), (), (), tuple(query.start), tuple(query.goal),
                              signature, {'l1_expanded': diagnostics['expanded'], **diagnostics},
                              {'world_key': self.world.key, 'topology_key': self.key, **self.world.binding})
        self._cache[key] = [route]
        self._probe_stats = {**diagnostics, 'cache_hit': False,
                             'candidate_count': 1, 'wall_ms': (time.perf_counter()-tick)*1000}
        return [route]

    def task(self, route):
        return self.tasks[route.signature]

    def channel_plan(self, route):
        return self.task(route)

    def scope_mask(self, route):
        """Fine-grid query scope; only a search hint, never obstacle clearance."""
        task = self.task(route)
        line = np.asarray(task['right_reference_polyline'], float)
        factor = 3
        shape = (math.ceil(self.world.map.height/factor), math.ceil(self.world.map.width/factor))
        coarse = np.zeros(shape, np.uint8)
        rr, cc = self.world.cells(line)
        points = np.column_stack((cc//factor, rr//factor)).astype(np.int32)
        cv2.polylines(coarse, [points], False, 1, 1)
        radius = math.ceil(task['scope_half_width_m'] / (factor*self.world.map.resolution))
        yy, xx = np.mgrid[-radius:radius+1, -radius:radius+1]
        coarse = cv2.dilate(coarse, ((xx*xx+yy*yy) <= radius*radius).astype(np.uint8))
        fine = np.repeat(np.repeat(coarse, factor, axis=0), factor, axis=1)
        return np.asarray(fine[:self.world.map.height, :self.world.map.width], bool)
