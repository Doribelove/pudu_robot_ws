"""Summarize saved, immutable experiments without invoking planning."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'V4-r5-soft': Path('/home/robot/workspaces/3yd_v4_r5_soft_ring_20260920/results/formal_soft_ring_controlled'),
    'V4-r6-B': Path('/home/robot/workspaces/3yd_v4_r6_20260920/results/formal_r6_B_01'),
    'V5-r0': ROOT/'results/formal_v5_r0',
    'V5-uniform-0.8-ablation': ROOT/'results/ablation_uniform_08',
}

def read_rows(directory):
    return [json.loads(line) for line in (directory/'runs.jsonl').read_text().splitlines()]

def stats(rows):
    successes = [r for r in rows if r['success']]
    return {
        'runs': len(rows), 'strict_success': len(successes),
        'l2_reference_runs': sum(bool(r.get('l2')) for r in rows),
        'successful_queries': sorted({r['query_id'] for r in successes}),
        'median_s': float(np.median([r['wall_ms']/1000 for r in rows])),
        'p95_s': float(np.percentile([r['wall_ms']/1000 for r in rows], 95)),
        'rss_peak_gib': max(r['rss_peak_bytes'] for r in rows)/2**30,
        'successful_query_median_s': float(np.median([r['wall_ms']/1000 for r in successes])) if successes else None,
    }

def main():
    protocols = {name: json.loads((root/'protocol.json').read_text()) for name, root in SOURCES.items()}
    fields = ['map_sha256', 'semantic_hash', 'queries_sha256', 'parking_access_sha256', 'vehicle', 'CPU']
    agreement = {field: all(p[field] == protocols['V5-r0'][field] for p in protocols.values()) for field in fields}
    assert all(agreement.values()), agreement
    result = {'input_agreement': agreement, 'sources': {}, 'queries': [], 'ablation': []}
    for name, root in SOURCES.items():
        rows = read_rows(root)
        result['sources'][name] = {'directory': str(root), 'all_runs': stats(rows),
                                  'first_repeat': stats([r for r in rows if r['repeat'] == 1])}
    rows = read_rows(SOURCES['V5-r0'])
    base = read_rows(SOURCES['V4-r6-B'])
    uniform = read_rows(SOURCES['V5-uniform-0.8-ablation'])
    exported = {'version': '3YD-V5-r0', 'frame_id': 'map', 'units': 'm', 'queries': []}
    for qid in sorted({r['query_id'] for r in rows}):
        qr = [r for r in rows if r['query_id'] == qid]
        r = qr[0]; case = SOURCES['V5-r0']/'cases'/f'{qid}_1'
        l1 = json.loads((case/'l1_output.json').read_text())
        item = {'query_id': qid, **stats(qr), 'failure': r['failure_code'],
                'L1_points': len(l1['right_reference_polyline']), 'region_sequence': r['region_sequence']}
        layer = {'query_id': qid, 'L1': l1, 'strict_success': r['success'], 'failure': r['failure_code'],
                 'L2_reference_xy': None, 'L3_accepted_xyyaw': None}
        if r.get('l2'):
            item.update(L2_raw_nodes=r['l2']['v5']['raw_nodes'], L2_nodes=r['l2']['v5']['straightened_nodes'],
                        L2_backend=r['l2']['selected_backend'], L2_expanded=r['l2']['dstar_expanded'],
                        corner_count=r['bounded_l3']['corner_count'], L3_cells=r['bounded_l3']['search_cells'],
                        L3_map_fraction=r['bounded_l3']['map_fraction'],
                        length_m=r['length_m'], right_side_fraction=r['right_side_fraction'],
                        steering_jump_deg=r['constraints']['max_raw_steering_jump_deg'])
            baseline = next(x for x in base if x['query_id']==qid)
            item['length_change_vs_r6_percent'] = 100*(r['length_m']/baseline['length_m']-1)
            item['right_change_vs_r6_percentage_points'] = 100*(r['right_side_fraction']-baseline['right_side_fraction'])
            layer['L2_reference_xy'] = np.load(case/'l2_reference_xy.npy').tolist()
            layer['L2_corners'] = json.loads((case/'l2_corners.json').read_text())
            layer['L3_scope_file'] = str(case/'l3_scope.npz')
            layer['L3_accepted_xyyaw'] = np.load(case/'path.npy').tolist() if r['success'] else None
            ur = [x for x in uniform if x['query_id'] == qid]
            # The scope ablation must use exactly the same L1/L2 output.
            other = SOURCES['V5-uniform-0.8-ablation']/'cases'/f'{qid}_1'
            assert l1 == json.loads((other/'l1_output.json').read_text())
            assert np.array_equal(np.load(case/'l2_reference_xy.npy'),np.load(other/'l2_reference_xy.npy'))
            result['ablation'].append({'query_id':qid, 'same_L1_L2': True,
                'uniform_passes':sum(x['success'] for x in ur), 'variable_passes':sum(x['success'] for x in qr),
                'repeats':len(qr), 'uniform_cells':ur[0]['bounded_l3']['search_cells'],
                'variable_cells':r['bounded_l3']['search_cells'],
                'uniform_failure':ur[0]['failure_code'],
                'uniform_steering_jump_deg':ur[0]['constraints']['max_raw_steering_jump_deg'],
                'variable_steering_jump_deg':r['constraints']['max_raw_steering_jump_deg']})
            assert all(np.array_equal(np.load(SOURCES['V5-r0']/'cases'/f'{qid}_{i}'/'path.npy'),np.load(case/'path.npy')) for i in (2,3))
        result['queries'].append(item); exported['queries'].append(layer)
    source_hashes=json.loads((SOURCES['V5-r0']/'source_snapshot/hashes.json').read_text())
    result['planning_sources_match_snapshot']=all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha for path,sha in source_hashes.items())
    assert result['planning_sources_match_snapshot']
    result['cleanup']={name:json.loads(root.with_suffix('.supervisor.json').read_text())['cleanup']['complete'] for name,root in SOURCES.items() if name.startswith('V5')}
    (ROOT/'report/comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    (ROOT/'report/layer_outputs.json').write_text(json.dumps(exported,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__ == '__main__':
    main()
