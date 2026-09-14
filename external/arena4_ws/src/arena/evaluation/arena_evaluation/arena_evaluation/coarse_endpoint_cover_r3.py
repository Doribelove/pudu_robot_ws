"""Preserve every endpoint alternative's coarse component during tile selection.

This selects existing, certified seam links only. It proves coarse connectivity,
not endpoint SE(2) reachability; the endpoint selector and final audit still decide.
"""
from collections import deque
import math

import numpy as np

from .reachable_endpoint_r3 import digest
from .tiled_topology_r3 import NEIGHBORS, check_deadline, file_hash

VERSION = 'coarse-endpoint-component-anchor-cover-v1'


def anchor_cover(coarse, starts, goals, *, deadline=None):
    """One deterministic tree per shared component connects all its seeds.

    A multi-source nearest-seed forest does not provide this property: its
    disconnected trees can omit the detour required by the actual safe connector.
    Storage is O(coarse nodes + edges), independent of map cell count.
    """
    selected = set()
    groups = []
    visited = set()
    for seed in sorted(starts):
        check_deadline(deadline, 'coarse_cover_component')
        if seed in visited or seed not in coarse:
            continue
        component = {seed}
        queue = deque([seed])
        visited.add(seed)
        while queue:
            check_deadline(deadline, 'coarse_cover_component_search')
            node = queue.popleft()
            for nxt, _ in sorted(coarse[node]):
                if nxt not in visited:
                    visited.add(nxt)
                    component.add(nxt)
                    queue.append(nxt)
        ss = sorted(component.intersection(starts))
        gs = sorted(component.intersection(goals))
        if not gs:
            continue
        anchor = min(gs, key=lambda n: (goals[n], n))
        parents = {anchor: None}
        queue = deque([anchor])
        pending = set(ss + gs) - {anchor}
        while queue and pending:
            check_deadline(deadline, 'coarse_cover_anchor_tree')
            node = queue.popleft()
            for nxt, _ in sorted(coarse[node]):
                if nxt not in parents:
                    parents[nxt] = node
                    pending.discard(nxt)
                    queue.append(nxt)
        if pending:
            raise ValueError('coarse component has asymmetric or broken adjacency')
        edges = set()
        covered = {anchor}
        for node in sorted(set(ss + gs)):
            while node not in covered:
                check_deadline(deadline, 'coarse_cover_trace')
                parent = parents[node]
                if parent is None:
                    raise ValueError('endpoint tree lost its anchor')
                edges.add(tuple(sorted((node, parent))))
                covered.add(node)
                node = parent
        selected.update(covered)
        groups.append({'component_root': min(component), 'anchor': anchor,
                       'starts': ss, 'goals': gs, 'edges': sorted(edges)})
    certificate = {'algorithm': VERSION, 'groups': groups,
                   'covered_nodes': sorted(selected),
                   'semantics': 'coarse component preservation; no SE2 acceptance'}
    certificate['hash'] = digest(certificate)
    return selected, certificate


def endpoint_components(tile, pose, radius_m, *, deadline=None):
    check_deadline(deadline, 'coarse_endpoint_start')
    cell = tile.map.world_to_cell(*pose[:2])
    if cell is None:
        return {}
    radius = math.ceil(radius_m / tile.map.resolution)
    size = tile.config.tile_cells
    result = {}
    for tr in range(max(0, (cell[0]-radius)//size),
                    min(tile.rows, (cell[0]+radius)//size+1)):
        for tc in range(max(0, (cell[1]-radius)//size),
                        min(tile.cols, (cell[1]+radius)//size+1)):
            check_deadline(deadline, 'coarse_endpoint_tile')
            t = (tr, tc)
            r0, _, c0, _ = tile.bounds(t)
            labels = tile.tile(t)['labels']
            rr, cc = np.ogrid[:labels.shape[0], :labels.shape[1]]
            distance = (rr+r0-cell[0])**2 + (cc+c0-cell[1])**2
            valid = (distance <= radius**2) & (labels > 0)
            for label in sorted(map(int, np.unique(labels[valid]))):
                result[(*t, label)] = math.sqrt(float(
                    distance[valid & (labels == label)].min())) / size
    return result


def candidate_tiles(tile, start, goal, endpoint_radius_m=25., *, deadline=None):
    if not math.isfinite(endpoint_radius_m) or endpoint_radius_m <= 0:
        raise ValueError('positive finite endpoint radius required')
    tile.last_selection_certificate = None
    starts = endpoint_components(tile, start, endpoint_radius_m, deadline=deadline)
    goals = endpoint_components(tile, goal, endpoint_radius_m, deadline=deadline)
    nodes, certificate = anchor_cover(tile.coarse, starts, goals, deadline=deadline)
    selected = set()
    if nodes:
        selected.update(n[:2] for n in starts)
        selected.update(n[:2] for n in goals)
        for r, c, _ in nodes:
            selected.add((r, c))
            for dr, dc in NEIGHBORS:
                if (r+dr, c+dc) in tile.portal_by_tile:
                    selected.add((r+dr, c+dc))
    certificate = {'algorithm': VERSION, 'implementation_sha256': file_hash(__file__),
                   'tile_binding': tile.key, 'start': list(start), 'goal': list(goal),
                   'endpoint_radius_m': endpoint_radius_m, 'cover': certificate,
                   'selected_tiles': sorted(selected)}
    certificate['hash'] = digest(certificate)
    check_deadline(deadline, 'coarse_cover_complete')
    tile.last_selection_certificate = certificate
    return sorted(selected)
