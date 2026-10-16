"""Contemporaneous original AS reference; never combine historical timings."""
import argparse
from pathlib import Path
import numpy as np
from compare_v9 import read, parts, summary
from compare_v9_r7 import compare
from compare_v9_review import load, save
from review_v9_r8_stage3 import source_review
from analyze_v8 import MAPS

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(); p.add_argument('--reference-root', type=Path, required=True)
    p.add_argument('--candidate-root', type=Path, required=True); a = p.parse_args()
    out = {'source_cycle': 4, 'candidate': 'OGR', 'contemporary_reference': 'AS',
           'historical_timing_used': False, 'adopted': False, 'maps': {}}
    for mid, label in MAPS:
        dirs = {'AS': a.reference_root/'fixed/AS'/mid, 'OGR': a.candidate_root/'fixed/OGR'/mid}
        rs = {m: read(d) for m, d in dirs.items()}
        assert all(len(rows) == 30 for rows in rs.values())
        arms = {m: {k: summary(v) for k, v in parts(rows, 'fixed').items()} for m, rows in rs.items()}
        assert all(v['all']['success'] == 30 for v in arms.values())
        for m, d in dirs.items():
            protocol = load(d/'protocol.json')
            assert protocol['workload'] == 'fixed' and protocol['repeats'] == 3 and not protocol['diagnostic_subset']
            original = load(Path(protocol['query_file']))
            ids = [q['id'] for q in original.get('scenarios', original.get('queries'))]
            assert len(ids) == len(set(ids)) == 10 and {r['query_id'] for r in rs[m]} == set(ids)
            assert all(sorted(r['repeat'] for r in rs[m] if r['query_id'] == qid) == [1, 2, 3] for qid in ids)
        evidence = {'AS': source_review(dirs['AS'], 'control', cycle=4, storage_mode='control'),
                    'OGR': source_review(dirs['OGR'], 'OGR', cycle=4)}
        # Native reference actually loaded the original global geometry. The
        # candidate's local hashes remain local and must match its source.
        mask = np.load(dirs['AS']/'cases'/f'{ids[0]}_1/l3_scope.npz')['mask']; h, w = mask.shape
        log = next((dirs['AS']/'smac/logs').glob('*.log')).read_text()
        assert f': {w} X {h} map @ 0.05 m/cell' in log
        evidence['AS']['actual_original_full_map_shape'] = [h, w]
        parity = compare(dirs['AS'], dirs['OGR'])
        assert parity['quality_gate'] and parity['upstream_gate']
        def change(group, metric, stat):
            x = arms['AS'][group]['metrics'][metric][stat]; y = arms['OGR'][group]['metrics'][metric][stat]
            return 100*(y/x-1)
        changes = {'first_median': change('first', 'e2e_s', 'median'),
                   'repeat_median': change('repeat', 'e2e_s', 'median'),
                   'first_p95': change('first', 'e2e_s', 'p95'),
                   'repeat_p95': change('repeat', 'e2e_s', 'p95'),
                   'tree_rss_all_median': change('all', 'tree_rss_gib', 'median')}
        gates = {'first_benefit_at_least3': changes['first_median'] <= -3,
                 'rss_benefit_at_least5': changes['tree_rss_all_median'] <= -5,
                 'repeat_median_not_over5': changes['repeat_median'] <= 5,
                 'first_p95_not_over5': changes['first_p95'] <= 5,
                 'repeat_p95_not_over5': changes['repeat_p95'] <= 5,
                 'strict_upstream_quality': True}
        out['maps'][mid] = {'label': label, 'arms': arms, 'parity': parity,
                            'source_native_cleanup': evidence, 'change_pct': changes, 'gates': gates}
    out['fixed_combined_gates_pass'] = all(v for x in out['maps'].values() for v in x['gates'].values())
    save(ROOT/'report/SOURCE4_AS_REFERENCE_REVIEW.json', out)
    print('CONTEMPORARY_AS_FIXED_COMBINED_GATES', out['fixed_combined_gates_pass'])
    for mid, item in out['maps'].items(): print(mid, item['change_pct'], item['gates'])


if __name__ == '__main__': main()
