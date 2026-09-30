"""Raw-run comparison for V6-r2 query-time L1 optimization."""
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
R0 = Path('/home/robot/workspaces/3yd_v6_r0_20260924')
R1 = Path('/home/robot/workspaces/3yd_v6_r1_20260924')
MAPS = {
    'pudu_wanda_3f': ('万达2w地库', 'formal_wanda2w'),
    'mengjia': ('万达3w地库', 'formal_wanda3w'),
    '3yd_synth_multiloop_a_v1': ('合成地库A', 'formal_3yd_synth_multiloop_a_v1'),
    '3yd_synth_twinwing_b_v1': ('合成地库B', 'formal_3yd_synth_twinwing_b_v1'),
    '3yd_synth_trizone_c_v2': ('合成地库C', 'formal_3yd_synth_trizone_c_v2'),
}


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def stats(values):
    v = np.asarray(values, float)
    return {'n': len(v), 'median': float(np.median(v)),
            'p95': float(np.quantile(v, .95)), 'max': float(v.max())}


def records(folder):
    values = [json.loads(line) for line in (folder / 'runs.jsonl').read_text().splitlines()]
    assert len(values) == 10 and sorted(v['query_id'] for v in values) == [f'Q{i:02}' for i in range(1, 11)]
    return {v['query_id']: v for v in values}


def path_checks(reference, tested, query_id):
    a, b = reference / 'cases' / f'{query_id}_1', tested / 'cases' / f'{query_id}_1'
    return {name: (json.loads((a / f'{name}.json').read_text()) ==
                   json.loads((b / f'{name}.json').read_text()))
            for name in ('l1_output', 'l2_soft_target')} | {
        'l2_reference_xy': np.array_equal(np.load(a / 'l2_reference_xy.npy'),
                                          np.load(b / 'l2_reference_xy.npy')),
        'final_path': np.array_equal(np.load(a / 'path.npy'), np.load(b / 'path.npy')),
    }


def main():
    locations = {'r0': {}, 'r0_current': {}, 'r1_A': {}, 'r2_resume_only': {}, 'r2_combined': {}}
    by_arm = {name: [] for name in locations}
    results = {name: {} for name in locations}
    for map_id, (name, r0_dir) in MAPS.items():
        locations['r0'][map_id] = R0 / 'results' / r0_dir
        locations['r0_current'][map_id] = (ROOT / 'results/r0_recheck_C' if map_id == '3yd_synth_trizone_c_v2'
                                           else ROOT / 'results/r0_recheck_five' / map_id)
        locations['r1_A'][map_id] = R1 / 'results/formal_A' / map_id
        locations['r2_resume_only'][map_id] = ROOT / 'results/formal_r2' / map_id
        locations['r2_combined'][map_id] = ROOT / 'results/formal_r2_combined' / map_id
        base_dir = locations['r0'][map_id]
        base_records = records(base_dir)
        base_protocol = json.loads((base_dir / 'protocol.json').read_text())
        for arm in locations:
            folder = locations[arm][map_id]
            rows = records(folder)
            protocol = json.loads((folder / 'protocol.json').read_text())
            for key in ('map_sha256', 'semantic_hash', 'queries_sha256',
                        'parking_access_sha256', 'vehicle', 'CPU', 'l2_resolution_m', 'l3_resolution_m'):
                assert protocol.get(key) == base_protocol.get(key), (arm, map_id, key)
            by_arm[arm].extend(rows.values())
            outcome_differences, exact_paths = [], []
            if arm != 'r0':
                for q in base_records:
                    old, new = base_records[q], rows[q]
                    if old['success'] != new['success'] or old['failure_code'] != new['failure_code']:
                        outcome_differences.append(q)
                    if old['success'] and new['success']:
                        exact_paths.append(path_checks(base_dir, folder, q))
            series = list(rows.values())
            preparation = json.loads((folder / 'preparation.json').read_text())
            results[arm][map_id] = {
                'name': name, 'source': str(folder), 'runs_sha256': sha(folder / 'runs.jsonl'),
                'strict_success': sum(bool(row['success']) for row in series),
                'outcome_differences': outcome_differences,
                'exact_successful_paths': {key: f'{sum(row[key] for row in exact_paths)}/{len(exact_paths)}'
                                           for key in ('l1_output', 'l2_soft_target', 'l2_reference_xy', 'final_path')},
                'l1_query_ms': stats([row['recovery']['l1_actual_ms'] for row in series]),
                'end_to_end_ms': stats([row['wall_ms'] for row in series]),
                'peak_tree_rss_gib': stats([row['rss_peak_bytes'] / 2**30 for row in series]),
                'l1_build_ms': preparation['l1_build_ms'],
                'preparation_ms': preparation['wall_ms'],
                'alternate_search_triggered': sum(bool(row['recovery'].get('alternate_search_triggered')) for row in series),
                'first_phase_resumed': sum(bool(row['recovery'].get('first_phase_resumed')) for row in series),
                'acknowledged_success': sum(bool(row['success']) and bool(row['l3'].get('costmap_update_acknowledged')) for row in series),
                'right_side_success_median': float(np.median([row['right_side_fraction'] for row in series if row['success']])),
                'length_success_median_m': float(np.median([row['length_m'] for row in series if row['success']])),
            }
    totals = {}
    for arm, series in by_arm.items():
        totals[arm] = {
            'strict_success': sum(bool(row['success']) for row in series),
            'l1_query_ms': stats([row['recovery']['l1_actual_ms'] for row in series]),
            'end_to_end_ms': stats([row['wall_ms'] for row in series]),
            'peak_tree_rss_gib': stats([row['rss_peak_bytes'] / 2**30 for row in series]),
            'l1_build_ms': stats([results[arm][m]['l1_build_ms'] for m in MAPS]),
            'preparation_ms': stats([results[arm][m]['preparation_ms'] for m in MAPS]),
            'first_phase_resumed': sum(results[arm][m]['first_phase_resumed'] for m in MAPS),
            'acknowledged_success': sum(results[arm][m]['acknowledged_success'] for m in MAPS),
        }
    report = {'schema': '3YD-V6-r2-query-L1-v1', 'maps': results, 'totals': totals,
              'l1_exact_result': str(ROOT / 'results/lazy_l1_exact.json')}
    (ROOT / 'report/PERFORMANCE.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(totals, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
