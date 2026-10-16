"""Review source-specific R8 records without promoting diagnostic subsets."""
import argparse
import json
from pathlib import Path
from compare_v9 import read, tag, parts, summary
from compare_v9_r8 import mode_evidence, exact_compare, compare
from compare_v9_review import load, save, sha
from compare_v8_r6 import verify_cleanup
from analyze_v8 import MAPS

ROOT = Path(__file__).resolve().parents[1]


def source_review(directory, expected_mode, arm='A', cycle=3, storage_mode='CHNT', protocol=True):
    p = load(directory/('protocol.json' if protocol else 'summary.json'))
    evidence = mode_evidence(directory, storage_mode, protocol=protocol, expected_v9_mode='A' if arm == 'A' else 'AB',
                             expected_r2_mode='control' if arm == 'A' else 'P', expected_r8_mode=expected_mode)
    archive = ROOT/'report/source_cycles'/f'source{cycle}'
    lock = load(archive/'PLANNER_SOURCE_LOCK.json'); assert lock['source_cycle'] == cycle
    files = load(directory/'source_snapshot/hashes.json'); planning = lock['planning_source_files']
    for original, expected in files.items():
        relative = Path(original).relative_to(ROOT)
        assert sha(directory/'source_snapshot'/str(relative).replace('/', '_')) == expected
        if str(relative) in planning:
            assert expected == planning[str(relative)] == sha(archive/'planning_source'/relative)
    assert {str(Path(f).relative_to(ROOT)) for f in files if str(Path(f).relative_to(ROOT)) in planning} == set(planning)
    if protocol: assert p['queries_sha256'] == sha(Path(p['query_file']))
    return {'planning_files': len(planning), 'copied_files': len(files), 'native': evidence,
            'cleanup': verify_cleanup(directory), 'runs_sha256': sha(directory/('runs.jsonl' if protocol else 'events.json'))}


def main():
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True)
    p.add_argument('--phase', choices=['diagnostic', 'shared', 'fixed'], required=True)
    p.add_argument('--source-cycle', type=int, choices=[3, 4], default=3)
    p.add_argument('--candidate', choices=['OG', 'OGR'], default='OG')
    a = p.parse_args(); root = a.results_root.resolve()
    assert (a.source_cycle, a.candidate) in ((3, 'OG'), (4, 'OGR'))
    modes = ('control', a.candidate)
    out = {'source_cycle': a.source_cycle, 'candidate': a.candidate, 'version': '3YD-V9-r8', 'phase': a.phase,
           'diagnostic_only': a.phase != 'fixed', 'profiler': False, 'adopted': False, 'maps': {}}
    for mid, label in MAPS:
        item = {'label': label, 'groups': {}}
        kinds = ('moving', 'perturbed') if a.phase == 'diagnostic' else ('A', 'ABP') if a.phase == 'shared' else ('fixed',)
        for kind in kinds:
            def directory(mode):
                if a.phase == 'diagnostic': return root/'diagnostic'/kind/mode/mid
                if a.phase == 'shared': return root/'shared'/mode/kind/mid
                return root/'fixed'/mode/mid
            dirs = {m: directory(m) for m in modes}
            rs = {m: read(d) for m, d in dirs.items()}
            expected = 30 if a.phase == 'fixed' else 4
            assert all(len(rows) == expected for rows in rs.values())
            if a.phase == 'fixed':
                for mode, directory_ in dirs.items():
                    protocol = load(directory_/'protocol.json')
                    assert protocol['workload'] == 'fixed' and not protocol['diagnostic_subset'] and protocol['repeats'] == 3
                    original = load(Path(protocol['query_file']))
                    queries = original.get('scenarios', original.get('queries'))
                    ids = [q['id'] for q in queries]; assert len(ids) == len(set(ids)) == 10
                    assert {r['query_id'] for r in rs[mode]} == set(ids)
                    assert all(sorted(r['repeat'] for r in rs[mode] if r['query_id'] == qid) == [1, 2, 3] for qid in ids)
            evidence = {m: source_review(d, m, kind if a.phase == 'shared' else 'A', a.source_cycle) for m, d in dirs.items()}
            if a.phase == 'shared':
                definition = load(ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json'))
                assert not definition['fixed10'] and sha(Path(definition['source_path'])) == definition['source_sha256']
                ids = [q['id'] for q in definition['queries']]
                assert all([r['query_id'] for r in rows] == ids for rows in rs.values())
                assert all(not r['fullroute'].get('reused') and not r['task_progress'].get('reused')
                           for rows in rs.values() for r in rows)
                groups = {m: {'initial': summary(rows[:1]), 'new_tasks': summary(rows[1:])} for m, rows in rs.items()}
            else:
                groups = {m: {k: summary(v) for k, v in parts(rows, 'fixed' if a.phase == 'fixed' else 'moving').items()}
                          for m, rows in rs.items()}
            for mode, rows in rs.items():
                for row in rows:
                    # Initial/new tasks exercise the actual generation hook;
                    # continuation resets timing metadata but retains the field.
                    if (not row.get('moving_start') and row['repeat'] == 1) or (row.get('moving_start') and row['moving_start']['kind'] == 'initial'):
                        assert row['timing']['r8_owned_search_mask_immutable'] == (mode != 'control')
                    detail = load(dirs[mode]/'cases'/tag(row)/'outcome.json').get('reference_diagnostics', {})
                    if detail:
                        assert not detail.get('r8_immutable_auxiliary_fields')
                        assert bool(detail.get('r8_sparse_occupancy_used')) == (mode != 'control' and mid != 'mengjia')
                        assert bool(detail.get('r8_exact_tile_reduction_enabled')) == (mode == 'OGR')
            parity = exact_compare(dirs['control'], dirs[a.candidate])
            selected = 'new_tasks' if a.phase == 'shared' else 'first' if a.phase == 'fixed' else 'moving'
            before = groups['control'][selected]['metrics']; after = groups[a.candidate][selected]['metrics']
            change = {k: 100*(after[k]['median']/before[k]['median']-1) for k in ('e2e_s', 'tree_rss_gib')}
            item['groups'][kind] = {'arms': groups, 'exact_parity': parity, 'source_native_cleanup': evidence,
                                    'selected_comparison_group': selected, 'median_change_pct': change}
        if a.phase == 'shared':
            item['BP_relative_to_A'] = {}
            for mode in modes:
                d1 = root/'shared'/mode/'A'/mid; d2 = root/'shared'/mode/'ABP'/mid
                q = compare(d1, d2); assert q['quality_gate'] and q['upstream_gate']
                x = item['groups']['A']['arms'][mode]['new_tasks']['metrics']['e2e_s']['median']
                y = item['groups']['ABP']['arms'][mode]['new_tasks']['metrics']['e2e_s']['median']
                item['BP_relative_to_A'][mode] = {'median_change_pct': 100*(y/x-1), 'quality_and_upstream': q}
        out['maps'][mid] = item
    out['strict_total'] = sum(group['arms'][mode]['all' if a.phase != 'shared' else 'initial']['success']
        + (group['arms'][mode]['new_tasks']['success'] if a.phase == 'shared' else 0)
        for item in out['maps'].values() for group in item['groups'].values() for mode in modes)
    save(ROOT/'report'/(f'SOURCE{a.source_cycle}_'+a.phase.upper()+'_REVIEW.json'), out)
    print(f'SOURCE{a.source_cycle}', a.phase, 'STRICT', out['strict_total'], 'EXACT_PARITY_PASS')
    for mid, item in out['maps'].items():
        print(mid, {k: v['median_change_pct'] for k, v in item['groups'].items()})
        if 'BP_relative_to_A' in item:
            print('BP/A', {k: v['median_change_pct'] for k, v in item['BP_relative_to_A'].items()})


if __name__ == '__main__': main()
