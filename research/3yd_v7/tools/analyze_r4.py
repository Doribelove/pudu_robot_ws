"""Compare the V6-r4 L1 transition-table experiment with saved V6-r3 data."""

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OLD = Path('/home/robot/workspaces/3yd_v6_r3_20260924')
MAPS = (
    'pudu_wanda_3f', 'mengjia', '3yd_synth_multiloop_a_v1',
    '3yd_synth_twinwing_b_v1', '3yd_synth_trizone_c_v2',
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def values(items):
    seq = np.asarray(items, dtype=float)
    return dict(n=int(seq.size), median=float(np.median(seq)),
                p95=float(np.quantile(seq, .95)), maximum=float(np.max(seq)))


def pct(old, new):
    return 100.0 * (new / old - 1.0)


def main():
    formal = {}
    l1_only = {}
    old_l1, new_l1, old_rss, new_rss = [], [], [], []
    old_full, new_full, old_init, new_init, old_only_rss, new_only_rss = [], [], [], [], [], []
    exact = dict(strict=0, l1=0, soft=0, l2=0, final=0, ack=0, l1_only=0)
    formal_inputs = {}
    for map_id in MAPS:
        old_dir = OLD / 'results/formal_r3_final' / map_id
        new_dir = ROOT / 'results/formal_r4' / map_id
        old_protocol, new_protocol = read(old_dir/'protocol.json'), read(new_dir/'protocol.json')
        source_hashes = read(new_dir/'source_snapshot/hashes.json')
        for filename in ('topology_astar.py', 'topology.py'):
            source = ROOT/'src/three_yd_v5'/filename
            assert source_hashes[str(source)] == sha(source), (map_id, filename, 'snapshot')
        identity = ('map_sha256', 'semantic_hash', 'queries_sha256',
                    'parking_access_sha256', 'vehicle', 'CPU', 'query_ids', 'repeats')
        for key in identity:
            assert old_protocol.get(key) == new_protocol.get(key), (map_id, key)
        formal_inputs[map_id] = {key: old_protocol.get(key) for key in identity}
        former = {row['query_id']: row for row in map(json.loads, (old_dir/'runs.jsonl').read_text().splitlines())}
        current = {row['query_id']: row for row in map(json.loads, (new_dir/'runs.jsonl').read_text().splitlines())}
        assert set(former) == set(current) == {f'Q{i:02}' for i in range(1, 11)}
        m_old, m_new = [], []
        queries = {}
        for query_id in sorted(former):
            a, b = former[query_id], current[query_id]
            same_strict = a['success'] == b['success'] and a['failure_code'] == b['failure_code']
            assert same_strict, (map_id, query_id, 'strict')
            exact['strict'] += 1
            a_ms, b_ms = a['recovery']['l1_actual_ms'], b['recovery']['l1_actual_ms']
            old_l1.append(a_ms); new_l1.append(b_ms); m_old.append(a_ms); m_new.append(b_ms)
            old_rss.append(a['rss_peak_bytes']/2**30); new_rss.append(b['rss_peak_bytes']/2**30)
            queries[query_id] = dict(strict_success=b['success'], r3_l1_ms=a_ms,
                                     r4_l1_ms=b_ms, change_pct=pct(a_ms, b_ms))
            for label, filename in [('l1', 'l1_output.json'), ('soft', 'l2_soft_target.json')]:
                x = old_dir/'cases'/f'{query_id}_1'/filename
                y = new_dir/'cases'/f'{query_id}_1'/filename
                assert x.exists() and y.exists() and sha(x) == sha(y), (map_id, query_id, label)
                exact[label] += 1
            for label, filename in [('l2', 'l2_reference_xy.npy'), ('final', 'path.npy')]:
                x = old_dir/'cases'/f'{query_id}_1'/filename
                y = new_dir/'cases'/f'{query_id}_1'/filename
                if not x.exists() and not y.exists():
                    continue
                assert x.exists() and y.exists() and np.array_equal(np.load(x), np.load(y)), (map_id, query_id, label)
                exact[label] += 1
            assert a['l3']['costmap_update_acknowledged'] == b['l3']['costmap_update_acknowledged'], (map_id, query_id, 'ack')
            exact['ack'] += 1
        formal[map_id] = dict(strict_success=sum(row['success'] for row in current.values()),
                              r3_l1_ms=values(m_old), r4_l1_ms=values(m_new), queries=queries,
                              r3_runs_sha256=sha(old_dir/'runs.jsonl'),
                              r4_runs_sha256=sha(new_dir/'runs.jsonl'))

        old_only_dir = OLD/'results/l1_only_r3'/map_id
        new_only_dir = ROOT/'results/l1_only_r4'/map_id
        rows_a, rows_b, inits_a, inits_b, rss_a, rss_b = [], [], [], [], [], []
        for round_id in (1, 2, 3):
            first = old_only_dir/f'round_{round_id}'
            second = new_only_dir/f'round_{round_id}'
            x, y = read(first/'batch.json'), read(second/'batch.json')
            assert len(x['queries']) == len(y['queries']) == 10
            for u, v in zip(x['queries'], y['queries']):
                assert u['query_id'] == v['query_id'] and u['output_sha256'] == v['output_sha256']
                exact['l1_only'] += 1
                rows_a.append(u['wall_ms']); rows_b.append(v['wall_ms'])
            inits_a.append(read(first/'initialization.json')['wall_ms'])
            inits_b.append(read(second/'initialization.json')['wall_ms'])
            rss_a.append(max(item['resources_after']['peak_rss_bytes'] for item in x['queries'])/2**20)
            rss_b.append(max(item['resources_after']['peak_rss_bytes'] for item in y['queries'])/2**20)
        old_full += rows_a; new_full += rows_b
        old_init += inits_a; new_init += inits_b
        old_only_rss += rss_a; new_only_rss += rss_b
        l1_only[map_id] = dict(r3_query_ms=values(rows_a), r4_query_ms=values(rows_b),
                               r3_initialization_ms=values(inits_a), r4_initialization_ms=values(inits_b),
                               r3_peak_rss_mib=values(rss_a), r4_peak_rss_mib=values(rss_b))
    overall = dict(r3_l1_ms=values(old_l1), r4_l1_ms=values(new_l1),
                   r3_query_tree_peak_rss_gib=values(old_rss),
                   r4_query_tree_peak_rss_gib=values(new_rss),
                   r3_l1_only_forced_three_ms=values(old_full),
                   r4_l1_only_forced_three_ms=values(new_full),
                   r3_l1_only_initialization_ms=values(old_init),
                   r4_l1_only_initialization_ms=values(new_init),
                   r3_l1_only_peak_rss_mib=values(old_only_rss),
                   r4_l1_only_peak_rss_mib=values(new_only_rss))
    overall['l1_query_reduction_pct'] = {
        key: -pct(overall['r3_l1_ms'][key], overall['r4_l1_ms'][key])
        for key in ('median', 'p95', 'maximum')}
    report = dict(schema='3YD-V6-r4-L1-search-v1', comparison='r3 vs r4',
                  transition_table=True, unchanged_first_budget=50000,
                  unchanged_alternate_budget=10000, exact_checks=exact,
                  configuration=dict(
                      transition_table='THREEYD_V6_TRANSITION_TABLE=1',
                      alternate_goal_potential='off, experimental branch rejected',
                      fast_label_insert='absent, experimental branch rejected',
                      r4_env_sha256=sha(ROOT/'env.bash'),
                      source_snapshots_match_live_code=True),
                  formal_inputs=formal_inputs, formal=formal, l1_only=l1_only,
                  overall=overall, source_sha256={
                      'topology_astar.py': sha(ROOT/'src/three_yd_v5/topology_astar.py'),
                      'topology.py': sha(ROOT/'src/three_yd_v5/topology.py'),
                  })
    destination = ROOT/'report/PERFORMANCE.json'
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(exact_checks=exact, overall=overall), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
