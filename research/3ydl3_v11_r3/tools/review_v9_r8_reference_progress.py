"""Report full original-AS progress recovery from contemporaneous records."""
import argparse
from pathlib import Path
from compare_v9 import read, parts, summary, tag
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
    for kind in ('moving', 'perturbed'):
        out['maps'][kind] = {}
        for mid, label in MAPS:
            dirs = {'AS': a.reference_root/kind/'AS'/mid, 'OGR': a.candidate_root/kind/'OGR'/mid}
            rows = {m: read(d) for m, d in dirs.items()}
            assert all(len(v) == 40 for v in rows.values())
            arms = {m: {k: summary(v) for k, v in parts(rs, kind).items()} for m, rs in rows.items()}
            assert all(v['initial']['success'] == 10 and v['moving']['success'] == 30 for v in arms.values())
            for mode, d in dirs.items():
                p = load(d/'protocol.json'); assert p['workload'] == 'moving' and not p['diagnostic_subset']
                q = load(Path(p['query_file'])); ids = {x['id'] for x in q.get('scenarios', q.get('queries'))}
                assert len(ids) == 10 and {r['query_id'] for r in rows[mode]} == ids
                assert all(sorted(r['moving_start']['step'] for r in rows[mode] if r['query_id'] == qid)
                           == ['initial', 'p20', 'p50', 'p80'] for qid in ids)
            parity = compare(dirs['AS'], dirs['OGR'], report_upstream_mismatch=True)
            evidence = {'AS': source_review(dirs['AS'], 'control', cycle=4, storage_mode='control'),
                        'OGR': source_review(dirs['OGR'], 'OGR', cycle=4)}
            changes = {metric: {stat: 100*(arms['OGR']['moving']['metrics'][metric][stat]/
                arms['AS']['moving']['metrics'][metric][stat]-1) for stat in ('median', 'p95')}
                       for metric in ('e2e_s', 'tree_rss_gib')}
            out['maps'][kind][mid] = {'label': label, 'arms': arms, 'parity': parity,
                                     'source_native_cleanup': evidence, 'changed_request_change_pct': changes,
                                     'original_AS_recovery_assessed_on_full10': True}
    save(ROOT/'report/SOURCE4_AS_PROGRESS_REVIEW.json', out)
    for kind, maps in out['maps'].items():
        for mid, x in maps.items(): print(kind, mid, x['changed_request_change_pct'],
                                        'QUALITY', x['parity']['quality_gate'], 'UPSTREAM', x['parity']['upstream_gate'])


if __name__ == '__main__': main()
