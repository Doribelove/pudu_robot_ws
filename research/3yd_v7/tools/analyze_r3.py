"""Compare the V6-r3 five-map formal run with the saved V6-r2 run."""

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
R2 = Path('/home/robot/workspaces/3yd_v6_r2_20260924')
MAPS = {
    'pudu_wanda_3f': '万达2w地库',
    'mengjia': '万达3w地库',
    '3yd_synth_multiloop_a_v1': '合成地库A',
    '3yd_synth_twinwing_b_v1': '合成地库B',
    '3yd_synth_trizone_c_v2': '合成地库C',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(folder):
    rows = [json.loads(line) for line in (folder / 'runs.jsonl').read_text().splitlines()]
    assert len(rows) == 10
    assert sorted(row['query_id'] for row in rows) == [f'Q{i:02}' for i in range(1, 11)]
    return {row['query_id']: row for row in rows}


def stats(values):
    x = np.asarray(values, float)
    return {'n': len(x), 'median': float(np.median(x)),
            'p95': float(np.quantile(x, .95)), 'max': float(x.max())}


def path_checks(old, new, query):
    a, b = old / 'cases' / f'{query}_1', new / 'cases' / f'{query}_1'
    return {
        'l1_output': json.loads((a / 'l1_output.json').read_text()) == json.loads((b / 'l1_output.json').read_text()),
        'l2_soft_target': json.loads((a / 'l2_soft_target.json').read_text()) == json.loads((b / 'l2_soft_target.json').read_text()),
        'l2_reference_xy': bool(np.array_equal(np.load(a / 'l2_reference_xy.npy'), np.load(b / 'l2_reference_xy.npy'))),
        'final_path': bool(np.array_equal(np.load(a / 'path.npy'), np.load(b / 'path.npy'))),
    }


def metric(rows, key):
    values = list(rows.values())
    if key == 'l1_ms':
        return stats([row['recovery']['l1_actual_ms'] for row in values])
    if key == 'end_to_end_ms':
        return stats([row['wall_ms'] for row in values])
    if key == 'peak_rss_gib':
        return stats([row['rss_peak_bytes'] / 2**30 for row in values])
    raise KeyError(key)


def main():
    report = {'schema': '3YD-V6-r3-L1-tail-v1', 'maps': {}, 'totals': {},
              'scope': 'five fixed maps x 10 queries x 1 run; historical sequential r2 comparison'}
    all_rows = {'r2': {}, 'r3': {}}
    for map_id, name in MAPS.items():
        old = R2 / 'results/formal_r2_combined' / map_id
        new = ROOT / 'results/formal_r3_final' / map_id
        a, b = load(old), load(new)
        p, q = (json.loads((folder / 'protocol.json').read_text()) for folder in (old, new))
        for field in ('map_sha256', 'semantic_hash', 'queries_sha256', 'parking_access_sha256',
                      'vehicle', 'CPU', 'l2_resolution_m', 'l3_resolution_m'):
            assert p.get(field) == q.get(field), (map_id, field)
        assert all(a[query]['success'] == b[query]['success']
                   and a[query]['failure_code'] == b[query]['failure_code'] for query in a), map_id
        exact = {query: path_checks(old, new, query) for query in a if a[query]['success']}
        assert all(all(check.values()) for check in exact.values()), map_id
        all_rows['r2'].update({(map_id, query): row for query, row in a.items()})
        all_rows['r3'].update({(map_id, query): row for query, row in b.items()})
        report['maps'][map_id] = {
            'name': name, 'r2_source': str(old), 'r3_source': str(new),
            'r2_runs_sha256': sha(old / 'runs.jsonl'), 'r3_runs_sha256': sha(new / 'runs.jsonl'),
            'success': sum(row['success'] for row in b.values()),
            'exact_successful_paths': {key: sum(check[key] for check in exact.values())
                                       for key in ('l1_output', 'l2_soft_target', 'l2_reference_xy', 'final_path')},
            'certified_queries': [query for query, row in b.items() if
                                  (row['recovery'].get('l1_search_diagnostics', {})
                                   .get('after_rejection') or {}).get('single_lane_candidate_certificate')],
            'metrics': {key: {'r2': metric(a, key), 'r3': metric(b, key)}
                        for key in ('l1_ms', 'end_to_end_ms', 'peak_rss_gib')},
        }
    for arm, rows in all_rows.items():
        report['totals'][arm] = {
            'strict_success': sum(row['success'] for row in rows.values()),
            'acknowledged_success': sum(bool(row['success']) and bool(row['l3'].get('costmap_update_acknowledged'))
                                        for row in rows.values()),
            'metrics': {key: metric(rows, key) for key in ('l1_ms', 'end_to_end_ms', 'peak_rss_gib')},
            'right_side_success_median': float(np.median([row['right_side_fraction'] for row in rows.values()
                                                          if row['success']])),
            'length_success_median_m': float(np.median([row['length_m'] for row in rows.values()
                                                       if row['success']])),
        }
    out = ROOT / 'report/PERFORMANCE.json'
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report['totals'], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
