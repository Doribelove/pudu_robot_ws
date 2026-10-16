"""Map-derived portal corridors plus continuous Dubins waypoint optimization.

Input consists solely of the immutable query-conditioned map fields and exact
request poses. This bounded offline prototype neither reads a saved witness
nor makes an online or continuous-space completeness claim.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import platform
import resource
import time

import cv2
import numpy as np
import scipy
from scipy.optimize import differential_evolution
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from .semantic_constraint_core import ConstraintWorld, dubins_edge, dense_interpolate
from .semantic_map import SemanticMapV1, sha256_file
from .semantic_path_revisit_audit import audit_revisits
from .semantic_transition_r2_preflight import semantic_metrics, explicit_hard_audit
from .semantic_transition_preflight import write_json


def grid_routes(world):
    """Map-cell graph used only to generate optimization bounds and seeds."""
    legal = ((world.master < 253) & world.grids['allowed'] & ~world.grids['hard']
             & np.isin(world.grids['labels'], world.selected))
    rr, cc = np.indices(legal.shape)
    x = world.map.full_origin[0] + (cc+world.map.col0+.5)*world.map.resolution
    y = world.map.full_origin[1] + (world.map.full_height-rr-world.map.row0-.5)*world.map.resolution
    legal &= (np.hypot(x-world.start[0], y-world.start[1])
              + np.hypot(x-world.goal[0], y-world.goal[1]) <= world.bound_length)
    # Conservative circular clearance is only for seeding, not acceptance.
    legal &= world.map.distance_m > math.hypot(.265, .225)+.05
    src, dst = (world.map.world_to_cell(*p[:2]) for p in (world.start, world.goal))
    if not legal[src] or not legal[dst]:
        return [], {'reason': 'CONSERVATIVE_SEED_ENDPOINT_BLOCKED'}
    count, labels = cv2.connectedComponents(legal.astype(np.uint8), 8)
    legal &= labels == labels[src]
    if not legal[dst]:
        return [], {'reason': 'CONSERVATIVE_SEED_DISCONNECTED'}
    cells = np.argwhere(legal)
    indices = np.full(legal.shape, -1, dtype=np.int32)
    indices[legal] = np.arange(len(cells))
    rows, cols, costs = [], [], []
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        other = cells + (dr, dc)
        valid = ((other[:, 0] >= 0) & (other[:, 0] < legal.shape[0])
                 & (other[:, 1] >= 0) & (other[:, 1] < legal.shape[1]))
        a = np.flatnonzero(valid)
        b = indices[other[valid, 0], other[valid, 1]]
        valid = b >= 0
        a, b = a[valid], b[valid]
        clearance = np.minimum(world.map.distance_m[tuple(cells[a].T)],
                               world.map.distance_m[tuple(cells[b].T)])
        cost = math.hypot(dr, dc)*world.map.resolution*(1+.12/clearance)
        rows.extend((a, b)); cols.extend((b, a)); costs.extend((cost, cost))
    graph = csr_matrix((np.concatenate(costs), (np.concatenate(rows), np.concatenate(cols))),
                       shape=(len(cells), len(cells)))
    distances, previous = dijkstra(graph, directed=False, indices=[indices[src], indices[dst]],
                                  return_predecessors=True)
    target = legal & world.grids['correct'] & (world.grids['error'] <= .50)
    component_count, target_labels = cv2.connectedComponents(target.astype(np.uint8), 8)
    portals = []
    for component in range(1, component_count):
        pool = indices[target_labels == component]
        pool = pool[pool >= 0]
        if not len(pool):
            continue
        detour = distances[0, pool]+distances[1, pool]
        pool = pool[np.isfinite(detour) & (detour <= world.bound_length)]
        if not len(pool):
            continue
        # Farthest and median geodesic positions sample the actual reachable
        # target component, not a hand-selected world-coordinate box.
        ordered = pool[np.argsort(distances[0, pool])]
        for fraction in (1., .75, .5):
            portals.append((len(pool), int(ordered[int((len(pool)-1)*fraction)])))
    portals.sort(reverse=True)

    def backtrace(which, endpoint):
        chain = [endpoint]
        root = int(indices[src if which == 0 else dst])
        while chain[-1] != root:
            parent = int(previous[which, chain[-1]])
            if parent < 0:
                return []
            chain.append(parent)
        return list(reversed(chain))

    routes = []
    for _, portal in portals[:6]:
        front = backtrace(0, portal)
        back = list(reversed(backtrace(1, portal)))
        chain = front+back[1:]
        xy = np.column_stack((x[tuple(cells[chain].T)], y[tuple(cells[chain].T)]))
        xy[0], xy[-1] = world.start[:2], world.goal[:2]
        length = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        pivot = float(length[len(front)-1])
        # Five geometrically distributed local controls include a U-turn
        # neighborhood where endpoint heading requires a return approach.
        stations = np.array([max(.5, pivot-5), max(.75, pivot-2), pivot,
                             min(length[-1]-.75, pivot+2), min(length[-1]-.5, pivot+5)])
        stations = np.unique(stations)
        poses = []
        for s in stations:
            center = np.array([np.interp(s, length, xy[:, k]) for k in range(2)])
            a = np.array([np.interp(max(0, s-.5), length, xy[:, k]) for k in range(2)])
            b = np.array([np.interp(min(length[-1], s+.5), length, xy[:, k]) for k in range(2)])
            if np.linalg.norm(b-a) < .05:
                delta = center-a
                yaw = math.atan2(delta[1], delta[0])+math.pi/2
            else:
                yaw = math.atan2(b[1]-a[1], b[0]-a[0])
            poses.append((float(center[0]), float(center[1]), yaw))
        routes.append(poses)
    return routes, {'seed_components': count-1, 'reachable_target_components': component_count-1,
                    'target_cells': int(target.sum()), 'graph_nodes': len(cells),
                    'route_seed_count': len(routes), 'map_resolution_m': world.map.resolution}


def run(inputs, query, output, timeout=90., seed=20260907):
    output.mkdir(parents=False, exist_ok=False)
    started, cpu = time.monotonic(), time.process_time()
    source_hash = sha256_file(Path(__file__))
    world = ConstraintWorld(inputs, query)
    semantic_map = SemanticMapV1.load(inputs.parent/'conversion_v1/semantic_map_v1.json')
    if semantic_map.semantic_map_hash != world.meta['semantic_map_hash']:
        raise ValueError('semantic map binding mismatch')
    seeds, preparation = grid_routes(world)
    write_json(output/'map_derived_seeds.json', {'preparation': preparation, 'seeds': seeds})
    best, best_score = None, math.inf
    counts = {'candidates': 0, 'collision_free': 0, 'semantic_pass': 0, 'revisit_free': 0,
              'canonical_audits': 0}
    progress = []
    deadline = started+timeout

    def objective(values):
        nonlocal best, best_score
        if time.monotonic() >= deadline:
            raise TimeoutError
        counts['candidates'] += 1
        knots = [world.start, *map(tuple, np.asarray(values).reshape((-1, 3))), world.goal]
        edges = [dubins_edge(a, b) for a, b in zip(knots, knots[1:])]
        if any(e is None for e in edges):
            return 1e9
        length = sum(e.length for e in edges)
        path = np.vstack((world.start, *(e.samples for e in edges)))
        rows, cols, inside = world.cells(path)
        if not np.all(inside):
            return 1e8 + np.count_nonzero(~inside)
        clearance = world.map.distance_m[rows, cols]
        lane = np.isin(world.grids['labels'][rows, cols], world.selected)
        illegal = (~lane | ~world.grids['allowed'][rows, cols] | world.grids['hard'][rows, cols]
                   | (world.master[rows, cols] >= 253))
        # A smooth conservative deficit helps optimization approach feasible
        # geometry; the actual padded rectangle is still checked separately.
        deficit = np.mean(np.maximum(0., .37-clearance))
        points = [dict(x=float(x), y=float(y), yaw=float(a)) for x, y, a in path]
        metric = semantic_metrics(world, points)
        lane_metric = metric['active_window']['classes']['lane']
        side = lane_metric.get('correct_side_ratio', 0.)
        band = lane_metric.get('target_band_ratio', 0.)
        error = lane_metric.get('lateral_error_p50_m', 10.)
        score = (10000*np.mean(illegal)+20000*deficit+1000*max(0., length-world.bound_length)
                 +1000*max(0., .8-side)+1000*max(0., .500001-band)+100*max(0., error-.5)+.05*length)
        safe = not np.any(illegal) and world.collision_free(dense_interpolate(path))
        counts['collision_free'] += int(safe)
        if not safe:
            return float(score+1000)
        counts['semantic_pass'] += int(metric['semantic_gate_passed'])
        revisit = audit_revisits(path)
        no_revisit = revisit['revisit_screen_passed']
        counts['revisit_free'] += int(no_revisit)
        score += 500*len(revisit['intersection_events']) + 500*bool(revisit['nonlocal_collinear_segment_pairs'])
        if score < best_score:
            counts['canonical_audits'] += 1
            audit, path = world.audit(edges)
            hard = explicit_hard_audit(world, points, semantic_map)
            passed = bool(metric['semantic_gate_passed'] and no_revisit and audit['canonical']['final_valid_success']
                          and audit['padded_effective_master_collision_free'] and audit['exact_endpoint_xy_yaw']
                          and audit['edge_continuity'] and audit['trace_replay_exact'] and audit['same_lane_instance']
                          and audit['maximum_control_curvature_1pm'] <= 2.5 and length <= world.bound_length
                          and hard['hard_feature_gate_passed'])
            best = (edges, path, {'canonical': audit, 'hard_features': hard, 'semantic': metric,
                                  'revisit': revisit, 'gate_passed': passed})
            best_score = float(score)
            progress.append({'candidate': counts['candidates'], 'score': best_score,
                             'side': side, 'band': band, 'p50_m': error, 'length_m': length,
                             'revisits': len(revisit['intersection_events']), 'gate_passed': passed})
        return float(score)

    stop = 'NO_MAP_DERIVED_SEED' if not seeds else 'BUDGET_EXHAUSTED'
    try:
        for index, poses in enumerate(seeds):
            values = np.asarray(poses).ravel()
            bounds = [(float(value-margin), float(value+margin))
                      for i, value in enumerate(values) for margin in [math.pi if i % 3 == 2 else 1.75]]
            objective(values)
            local_deadline = min(deadline, time.monotonic()+timeout/max(1, len(seeds)))
            differential_evolution(objective, bounds, seed=seed+index, popsize=7, maxiter=100000,
                                   tol=0., polish=False, x0=values,
                                   callback=lambda x, convergence: (time.monotonic() >= local_deadline
                                       or bool(best and best[2]['gate_passed'])))
            if best and best[2]['gate_passed']:
                stop = 'STRICT_CANDIDATE_FOUND'
                break
    except TimeoutError:
        pass
    result = {'method': 'map_portals_continuous_dubins_optimization', 'query_id': query,
              'contract_revision': 'semantic-endpoint-transition-short-full-6m-r2',
              'architecture_id': 'UNNAMED_OFFLINE_CANDIDATE', 'gate_passed': bool(best and best[2]['gate_passed']),
              'stop_reason': stop, 'counts': counts, 'best': best[2] if best else None, 'progress': progress,
              'preparation': preparation, 'budget_s': timeout, 'seed': seed,
              'wall_s': time.monotonic()-started, 'cpu_s': time.process_time()-cpu,
              'peak_rss_bytes': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
              'source_sha256_at_start': source_hash, 'python': platform.python_version(),
              'scipy': scipy.__version__, 'numpy': np.__version__,
              'input_meta_sha256': sha256_file(inputs/f'{query}.json'),
              'input_npz_sha256': world.meta['npz_sha256'], 'map_hash': world.meta['map_hash'],
              'semantic_map_hash': world.meta['semantic_map_hash'],
              'used_historical_paths': False, 'online': False,
              'proof_scope': 'bounded map-derived waypoint family only; not an infeasibility proof'}
    if best:
        edges, path, _ = best
        write_json(output/'path.json', [dict(x=float(x), y=float(y), yaw=float(a)) for x, y, a in path])
        write_json(output/'controls.json', {'edges': [e.certificate() for e in edges]})
    write_json(output/'result.json', result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--query', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--seed', type=int, default=20260907)
    args = parser.parse_args(argv)
    result = run(args.inputs, args.query, args.output, args.timeout, args.seed)
    print(json.dumps({k: v for k, v in result.items() if k not in ('best', 'progress')}, indent=2))
    return 0 if result['gate_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
