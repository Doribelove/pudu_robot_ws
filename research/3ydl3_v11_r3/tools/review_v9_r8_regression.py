"""Original full10 progress/perturbation and event acceptance for source4."""
import argparse
from pathlib import Path
import numpy as np
from compare_v9 import read, parts, summary
from compare_v9_r8 import exact_compare
from compare_v9_review import load, save, sha
from review_v9_r8_stage3 import source_review
from analyze_v8 import MAPS

ROOT = Path(__file__).resolve().parents[1]
MODES = ('control', 'OGR')


def progress(root):
    out = {}
    for kind in ('moving', 'perturbed'):
        out[kind] = {}
        for mid, label in MAPS:
            dirs = {m: root/kind/m/mid for m in MODES}; rows = {m: read(d) for m, d in dirs.items()}
            assert all(len(v) == 40 for v in rows.values())
            for m, d in dirs.items():
                p = load(d/'protocol.json'); assert p['workload'] == 'moving' and not p['diagnostic_subset'] and p['repeats'] == 1
                queries = load(Path(p['query_file'])); queries = queries.get('scenarios', queries.get('queries'))
                ids = {q['id'] for q in queries}; assert len(ids) == 10
                assert {r['query_id'] for r in rows[m]} == ids
                for qid in ids:
                    assert sorted(r['moving_start']['step'] for r in rows[m] if r['query_id'] == qid) == ['initial', 'p20', 'p50', 'p80']
            arms = {m: {k: summary(v) for k, v in parts(rs, kind).items()} for m, rs in rows.items()}
            assert all(v['initial']['success'] == 10 and v['moving']['success'] == 30 for v in arms.values())
            parity = exact_compare(dirs['control'], dirs['OGR'])
            changes = {}
            for metric in ('e2e_s', 'tree_rss_gib'):
                changes[metric] = {stat: 100*(arms['OGR']['moving']['metrics'][metric][stat]/
                    arms['control']['moving']['metrics'][metric][stat]-1) for stat in ('median', 'p95')}
            gates = {stat+'_not_over5': changes['e2e_s'][stat] <= 5 for stat in ('median', 'p95')}
            out[kind][mid] = {'label': label, 'arms': arms, 'exact_parity': parity,
                             'source_native_cleanup': {m: source_review(d, m, cycle=4) for m, d in dirs.items()},
                             'changed_request_change_pct': changes, 'incremental_gates': gates}
    return out


def events(root):
    out = {}
    for mid, label in MAPS:
        dirs = {m: root/'events'/m/mid for m in MODES}; rows = {m: load(d/'events.json') for m, d in dirs.items()}
        outcomes = lambda rs: [(r['label'], r.get('success'), r.get('trajectory_available')) for r in rs]
        assert outcomes(rows['control']) == outcomes(rows['OGR'])
        evidence = {}; paths = []
        for mode, directory in dirs.items():
            s = load(directory/'summary.json'); assert s['assertions_passed'] and s['events'] == len(rows[mode]) == 11
            evidence[mode] = source_review(directory, mode, cycle=4, protocol=False)
            for row in rows[mode]:
                if row.get('trajectory_available'):
                    assert row['whole_suffix_body_audit']['valid'] and row['original_task_traffic_audit']['valid']
                if row.get('success') and 'constraint_audit' in row:
                    assert row['constraint_audit']['steering_jump_valid'] and row['independent_audit']['valid']
                d = row.get('diagnostics', {})
                if d.get('costmap_update_acknowledged'):
                    assert d['costmap_ack_mismatch_cells'] == 0
                    if d.get('semantic_expected_server_content_hash'):
                        assert d['server_costmap_content_hash'] == d['semantic_expected_server_content_hash']
                        assert d['semantic_exact_stable_observations'] >= 2
        for row in rows['control']:
            name = row['label']+'_path.npy'; a, b = [d/name for d in dirs.values()]
            assert a.exists() == b.exists()
            if a.exists():
                assert np.array_equal(np.load(a), np.load(b)), (mid, row['label'])
                paths.append({'label': row['label'], 'exact_physical_path': True, 'sha256': sha(a)})
        indexed = {x['label']: x for x in rows['OGR']}
        assert not indexed['current_block']['success'] and not indexed['current_block']['trajectory_available']
        assert indexed['current_clear']['success'] and indexed['current_clear']['trajectory_available']
        if mid == 'mengjia':
            assert not indexed['future_add']['success'] and not indexed['future_add']['trajectory_available']
            assert indexed['future_clear']['success'] and indexed['future_clear']['trajectory_available']
        out[mid] = {'label': label, 'assertions': 22, 'same_outcomes': True, 'same_physical_paths': paths,
                    'source_native_cleanup': evidence, 'static_event_replay': True, 'live_execution': False}
    return out


def main():
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True)
    p.add_argument('--phase', choices=['progress', 'events', 'all'], default='all'); a = p.parse_args()
    if a.phase == 'events':
        out = load(ROOT/'report/SOURCE4_PROGRESS_REVIEW.json')
        assert out['source_cycle'] == 4 and out['strict_progress_total'] == 320 and out['candidate'] == 'OGR'
    else:
        out = {'source_cycle': 4, 'candidate': 'OGR', 'adopted': False, 'regression': progress(a.results_root)}
        out['strict_progress_total'] = 320
        out['incremental_progress_gates_pass'] = all(v for maps in out['regression'].values() for x in maps.values()
                                                    for v in x['incremental_gates'].values())
    if a.phase != 'progress':
        out['events'] = events(a.results_root); out['event_assertions'] = 44
    save(ROOT/'report'/('SOURCE4_PROGRESS_REVIEW.json' if a.phase == 'progress' else 'SOURCE4_REGRESSION_REVIEW.json'), out)
    print('PROGRESS_STRICT', 320, 'EXACT', True, 'INCREMENTAL_GATES', out['incremental_progress_gates_pass'])
    for kind, maps in out['regression'].items():
        for mid, x in maps.items(): print(kind, mid, x['changed_request_change_pct'])
    if a.phase != 'progress': print('EVENT_ASSERTIONS', 44, 'OUTCOME_AND_PATH_PARITY_PASS')


if __name__ == '__main__': main()
