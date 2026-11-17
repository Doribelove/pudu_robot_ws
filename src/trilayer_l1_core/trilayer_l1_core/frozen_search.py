# SPDX-License-Identifier: Apache-2.0
"""Unchanged map-only L1 search methods extracted from frozen P6 source.

This module is a search kernel, not the semantic topology/traffic planner.
"""
import heapq
import math
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


class _FrozenFreeSpaceSearch:
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
