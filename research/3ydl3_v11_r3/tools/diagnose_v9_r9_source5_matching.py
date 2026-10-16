"""Read-only reconstruction of original segment matching on sealed shared32.

No ROS, solver, planner edits, threshold changes, or performance acceptance.
The original propose/store functions must reproduce each saved cache result.
"""
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace

import numpy as np
from scipy.spatial import cKDTree

from arena_3d_v1.semantic_world import SemanticWorld, wrap
from arena_evaluation.planner_benchmark.models import Query
from three_yd_v4.local_l3 import reference_poses
from three_yd_v5.free_space_topology import FreeSpaceTopology, load_map_only_world
from three_yd_v5.topology_default import DefaultTopology
from three_yd_v5.v9_segments import cache_for, propose, store
from run_free_full import registered_input

ROOT = Path(__file__).resolve().parents[1]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def split_runs(okay):
    starts = np.flatnonzero(okay & np.r_[True, ~okay[:-1]])
    ends = np.flatnonzero(okay & np.r_[~okay[1:], True])
    return list(zip(starts, ends))


def inspect_matches(cache, p):
    """Describe the original filters; the original function decides matches."""
    tree = cKDTree(p[:, :2])
    entries = []
    for key, value in cache._data.items():
        q = value['poses']
        distance, index = tree.query(q[:, :2])
        near = distance <= .30
        yaw_ok = np.abs(wrap(q[:, 2] - p[index, 2])) <= math.radians(10)
        pose_ok = near & yaw_ok
        okay = pose_ok.copy()
        order_ok = (np.diff(index) >= 0) & (np.diff(index) <= 8)
        okay[1:] &= order_ok
        runs = []
        for a, b in split_runs(okay):
            if b - a < 2:
                continue
            x, y = int(index[a]), int(index[b])
            length = float(np.linalg.norm(np.diff(q[a:b+1, :2], axis=0), axis=1).sum())
            runs.append({'samples': int(b-a+1), 'length_m': length,
                         'reference_start_index': x, 'reference_end_index': y,
                         'original_span_gate_pass': bool(x >= 20 and y <= len(p)-21 and y > x and length >= 4.)})
        before_order_lengths = [float(np.linalg.norm(np.diff(q[a:b+1, :2], axis=0), axis=1).sum())
                                for a, b in split_runs(pose_ok) if b-a >= 2]
        entries.append({'key': key, 'samples': len(q), 'distance_eligible': int(near.sum()),
                        'distance_and_yaw_eligible': int(pose_ok.sum()),
                        'eligible_pairs_index_gap_over8': int(((np.diff(index) > 8) & pose_ok[1:] & pose_ok[:-1]).sum()),
                        'eligible_pairs_reverse_index': int(((np.diff(index) < 0) & pose_ok[1:] & pose_ok[:-1]).sum()),
                        'max_pose_eligible_run_m_before_order': max(before_order_lengths, default=0.),
                        'original_order_runs': runs})
    return {'reference_points': len(p), 'cache_entries': len(entries), 'entries': entries,
            'original_span_candidates': sum(r['original_span_gate_pass'] for v in entries for r in v['original_order_runs']),
            'index_gap_over8_total': sum(v['eligible_pairs_index_gap_over8'] for v in entries)}


def main():
    protocol_path = ROOT/'report/SOURCE5_MATCHING_PROBE_PROTOCOL.json'
    protocol = json.loads(protocol_path.read_text())
    assert protocol['read_only'] and protocol['no_ROS_or_performance_acceptance']
    assert sorted(os.sched_getaffinity(0)) == [14]
    for path, expected in protocol['files'].items():
        assert sha(ROOT/path) == expected, path
    results = {}
    with tempfile.TemporaryDirectory(prefix='v9_source5_matching_') as scratch:
        scratch = Path(scratch)
        for mid in protocol['maps']:
            base = ROOT/'results/shared_source5/shared/ABP/PLMX'/mid
            rows = [json.loads(v) for v in (base/'runs.jsonl').read_text().splitlines()]
            if mid == 'mengjia':
                world = SemanticWorld(ROOT/'inputs/corroborated_map.yaml', ROOT/'inputs/semantic.json', cache_root=base/'world')
                topology = DefaultTopology(world, ROOT/'inputs/topology.json', scratch/mid/'graph')
            else:
                directory, _, _ = registered_input(mid)
                world = load_map_only_world(directory/'map.yaml', scratch/mid/'world')
                topology = FreeSpaceTopology(world, scratch/mid/'graph')
            c = SimpleNamespace(world=world, topology=topology, views={}, generic_mode=mid != 'mengjia',
                                max_search_cells=400000, max_map_fraction=.10)
            records = []
            for row in rows:
                query = Query(row['query_id'], tuple(row['start']), tuple(row['goal']), seed=20260916 if mid == 'mengjia' else 20260929)
                case = base/'cases'/(query.query_id+'_1')
                l1 = json.loads((case/'l1_output.json').read_text())
                routes = topology.routes(query, limit=3 if mid == 'mengjia' else 1)
                matching_routes = [r for r in routes if r.signature == l1['signature']]
                assert len(matching_routes) == 1, (mid, query.query_id, 'original L1 signature')
                c.topology_route = matching_routes[0]
                p = reference_poses(np.load(case/'l2_reference_xy.npy'), query.start, query.goal)
                with np.load(case/'l3_scope.npz', allow_pickle=False) as d:
                    mask = d['mask']
                analysis = inspect_matches(cache_for(c), p)
                _, actual = propose(c, p, mask)
                saved = row['v9_local']['segments']
                assert actual == saved, (mid, query.query_id, 'original proposal differs', actual, saved)
                returned = np.load(case/'path.npy')
                points = [dict(zip(('x', 'y', 'yaw'), map(float, v))) for v in returned]
                after = store(c, {'success': True, 'result': SimpleNamespace(points=points)})
                assert after == row['v9_segment_cache'], (mid, query.query_id, 'original store differs')
                records.append({'query_id': query.query_id, 'original_propose_exact': True, 'original_store_exact': True,
                                'original_matches': actual, 'filters': analysis})
            results[mid] = records
            del c, topology, world
            gc.collect()
    for path, expected in protocol['files'].items():
        assert sha(ROOT/path) == expected, path
    report = {'source_cycle': 5, 'protocol_sha256': sha(protocol_path), 'read_only': True,
              'original_thresholds_functions_and_saved_inputs_unchanged': True,
              'all8_original_cache_proposals_and_stores_exact': True,
              'no_ROS_or_performance_acceptance': True, 'maps': results}
    with (ROOT/'report/SOURCE5_MATCHING_PROBE.json').open('x') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
        f.write('\n')
    print('ORIGINAL_MATCHING_RECONSTRUCTION8_EXACT', flush=True)
    for mid, rows in results.items():
        for r in rows[1:]:
            a = r['filters']
            print(mid, r['query_id'], 'original_span_candidates', a['original_span_candidates'],
                  'eligible_index_gaps_over8', a['index_gap_over8_total'],
                  'accepted_segments', r['original_matches']['matched_segments'], flush=True)


if __name__ == '__main__':
    main()
