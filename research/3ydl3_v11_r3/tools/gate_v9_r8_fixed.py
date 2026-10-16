"""Predeclared incremental fixed gates; never implies full adoption readiness."""
import argparse
from pathlib import Path
from compare_v9_review import load, save

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(); p.add_argument('--source-cycle', type=int, choices=[3, 4], required=True)
    p.add_argument('--candidate', choices=['OG', 'OGR'], required=True); a = p.parse_args()
    review = load(ROOT/'report'/f'SOURCE{a.source_cycle}_FIXED_REVIEW.json')
    assert review['source_cycle'] == a.source_cycle and review['candidate'] == a.candidate
    assert review['strict_total'] == 120 and not review['diagnostic_only'] and not review['profiler']
    out = {'source_cycle': a.source_cycle, 'candidate': a.candidate, 'adopted': False,
           'cold_not_run': True, 'maps': {}}
    for mid, item in review['maps'].items():
        group = item['groups']['fixed']; arms = group['arms']
        def values(part, field, statistic):
            return [arms[m][part]['metrics'][field][statistic] for m in ('control', a.candidate)]
        v = {'first_median': values('first', 'e2e_s', 'median'),
             'repeat_median': values('repeat', 'e2e_s', 'median'),
             'first_p95': values('first', 'e2e_s', 'p95'),
             'repeat_p95': values('repeat', 'e2e_s', 'p95'),
             'tree_rss_all_median': values('all', 'tree_rss_gib', 'median')}
        changes = {k: 100*(y/x-1) for k, (x, y) in v.items()}
        parity = group['exact_parity']
        gates = {'first_benefit_at_least3': changes['first_median'] <= -3,
                 'repeat_median_not_over5': changes['repeat_median'] <= 5,
                 'first_p95_not_over5': changes['first_p95'] <= 5,
                 'repeat_p95_not_over5': changes['repeat_p95'] <= 5,
                 'rss_not_over3': changes['tree_rss_all_median'] <= 3,
                 'strict_and_exact': parity['quality_gate'] and parity['upstream_gate']
                        and parity['all_physical_outputs_and_effective_maps_identical']}
        assert gates['strict_and_exact']
        out['maps'][mid] = {'values': v, 'change_pct': changes, 'gates': gates}
    out['both_fixed_first_gates_pass'] = all(x['gates']['first_benefit_at_least3'] for x in out['maps'].values())
    out['all_remaining_fixed_gates_pass'] = all(v for x in out['maps'].values()
        for k, v in x['gates'].items() if k != 'first_benefit_at_least3')
    out['incremental_first_gate_incomplete'] = not out['both_fixed_first_gates_pass']
    out['reason'] = ('Fixed incremental gates pass; contemporaneous original AS reference, full precise/perturbed, '
                     '44events, cold/default/source/native/resource audits still required.'
                     if out['both_fixed_first_gates_pass'] and out['all_remaining_fixed_gates_pass'] else
                     'One or more incremental fixed gates not met; cold missing. No adoption or rerun solely to obtain a favorable sample.')
    save(ROOT/'report'/f'SOURCE{a.source_cycle}_FIXED_GATES.json', out)
    print('FIXED_FIRST_GATE', out['both_fixed_first_gates_pass'], 'OTHER_FIXED_GATES', out['all_remaining_fixed_gates_pass'])
    for mid, item in out['maps'].items(): print(mid, item['change_pct'], item['gates'])


if __name__ == '__main__': main()
