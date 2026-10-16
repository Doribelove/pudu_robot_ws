"""Summarize the V6-r0 L1 and five-map full-plan validation from raw records."""
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HISTORICAL = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_r13_l1_astar_five_20260923')
L1_HISTORICAL = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_r13_astar_experiment_record_20260923')
MAPS = {
    'pudu_wanda_3f': ('万达2w地库', 'formal_wanda2w'),
    'mengjia': ('万达3w地库', 'formal_wanda3w'),
    '3yd_synth_multiloop_a_v1': ('合成地库A', 'formal_3yd_synth_multiloop_a_v1'),
    '3yd_synth_twinwing_b_v1': ('合成地库B', 'formal_3yd_synth_twinwing_b_v1'),
    '3yd_synth_trizone_c_v2': ('合成地库C', 'formal_3yd_synth_trizone_c_v2'),
}

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def series(items):
    values = np.asarray(items, float)
    return {'median': float(np.median(values)), 'p95': float(np.quantile(values, .95)),
            'max': float(np.max(values)), 'n': len(values)}

def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]

def main():
    historical_summary = json.loads((HISTORICAL / 'summary.json').read_text())['maps']
    output = {'version': '3YD-V6-r0', 'maps': {}, 'strict_success': 0,
              'strict_total': 50, 'l1_only_valid': 0, 'l1_only_total': 150}
    for map_id, (name, folder) in MAPS.items():
        old_batch = Path(historical_summary[name]['batch'])
        new_batch = ROOT / 'results' / folder
        old_rows = {r['query_id']: r for r in rows(old_batch / 'runs.jsonl')}
        new_rows = {r['query_id']: r for r in rows(new_batch / 'runs.jsonl')}
        assert set(old_rows) == set(new_rows) == {f'Q{i:02}' for i in range(1, 11)}
        old_success = {q for q, r in old_rows.items() if r['success']}
        new_success = {q for q, r in new_rows.items() if r['success']}
        identical_paths = []
        for query in old_success & new_success:
            path_old = old_batch / 'cases' / f'{query}_1' / 'path.npy'
            path_new = new_batch / 'cases' / f'{query}_1' / 'path.npy'
            if path_old.exists() and path_new.exists() and digest(path_old) == digest(path_new):
                identical_paths.append(query)
        l1_old, l1_new = [], []
        initial_old, initial_new, peak_old, peak_new = [], [], [], []
        hashes_match = []
        for round_id in (1, 2, 3):
            base = L1_HISTORICAL / 'l1_runs' / map_id / f'round_{round_id}'
            candidate = ROOT / 'results/l1_only' / map_id / f'round_{round_id}'
            old = json.loads((base / 'batch.json').read_text())
            new = json.loads((candidate / 'batch.json').read_text())
            initial_old.append(json.loads((base / 'initialization.json').read_text())['wall_ms'] / 1000)
            initial_new.append(json.loads((candidate / 'initialization.json').read_text())['wall_ms'] / 1000)
            peak_old.append(old['final_resources']['peak_rss_bytes'] / 2**20)
            peak_new.append(new['final_resources']['peak_rss_bytes'] / 2**20)
            for before, after in zip(old['queries'], new['queries']):
                assert before['query_id'] == after['query_id']
                hashes_match.append(before['output_sha256'] == after['output_sha256'])
                l1_old.append(before['wall_ms'])
                l1_new.append(after['wall_ms'])
                assert after['success']
        assert all(hashes_match), f'L1 output mismatch on {map_id}'
        item = {
            'name': name, 'map_id': map_id,
            'strict_success_old': len(old_success), 'strict_success_v6': len(new_success),
            'lost_success': sorted(old_success - new_success),
            'new_success': sorted(new_success - old_success),
            'identical_final_path_count': len(identical_paths),
            'identical_final_path_queries': sorted(identical_paths),
            'l1_only': {
                'valid_outputs': len(hashes_match), 'output_hashes_identical': True,
                'old_build_s': series(initial_old), 'v6_build_s': series(initial_new),
                'old_query_ms': series(l1_old), 'v6_query_ms': series(l1_new),
                'old_peak_rss_mib': max(peak_old), 'v6_peak_rss_mib': max(peak_new),
            },
            'three_layer': {
                'old_end_to_end_s': series([r['wall_ms'] / 1000 for r in old_rows.values()]),
                'v6_end_to_end_s': series([r['wall_ms'] / 1000 for r in new_rows.values()]),
                'old_query_peak_rss_gib': series([r['rss_peak_bytes'] / 2**30 for r in old_rows.values()]),
                'v6_query_peak_rss_gib': series([r['rss_peak_bytes'] / 2**30 for r in new_rows.values()]),
                'old_l1_build_s': json.loads((old_batch / 'preparation.json').read_text())['l1_build_ms'] / 1000,
                'v6_l1_build_s': json.loads((new_batch / 'preparation.json').read_text())['l1_build_ms'] / 1000,
            },
            'source': {'historical_runs': str(old_batch / 'runs.jsonl'),
                       'v6_runs': str(new_batch / 'runs.jsonl'),
                       'historical_sha256': digest(old_batch / 'runs.jsonl'),
                       'v6_sha256': digest(new_batch / 'runs.jsonl')},
        }
        output['maps'][map_id] = item
        output['strict_success'] += len(new_success)
        output['l1_only_valid'] += len(hashes_match)
    dest = ROOT / 'report/COMPARISON.json'
    dest.write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n')
    print(dest)
    print('strict:', output['strict_success'], '/', output['strict_total'])
    print('L1:', output['l1_only_valid'], '/', output['l1_only_total'])

if __name__ == '__main__':
    main()
