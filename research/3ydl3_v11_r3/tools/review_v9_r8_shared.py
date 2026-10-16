"""Original shared-v2 diagnostics, with O/F exact-output and scope gates."""
import argparse
import json
from pathlib import Path
from compare_v9 import read, summary
from compare_v9_r8 import mode_evidence, exact_compare, compare
from compare_v9_review import load, save, sha
from compare_v8_r6 import verify_cleanup
from analyze_v8 import MAPS

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True)
    a = p.parse_args(); root = a.results_root.resolve()
    out = {'version': '3YD-V9-r8', 'source_cycle': 2, 'diagnostic_only': True,
           'fixed10': False, 'new_tasks_per_group': 3, 'profiler': False,
           'results_root': str(root), 'modes': ['control', 'O', 'F', 'OF'], 'maps': {}}
    archive = ROOT/'report/source_cycles/source2'
    lock = load(archive/'PLANNER_SOURCE_LOCK.json')['planning_source_files']
    for mid, label in MAPS:
        definition_file = ROOT/'datasets/v9_shared_segments_v2'/(mid+'.json')
        definition = load(definition_file); queries = definition['queries']
        assert len(queries) == 4 and not definition['fixed10']
        assert sha(Path(definition['source_path'])) == definition['source_sha256']
        assert len({json.dumps(q['start'], sort_keys=True) for q in queries}) == 4
        assert len({json.dumps(q['goal'], sort_keys=True) for q in queries}) == 4
        item = {'label': label, 'query_file_sha256': sha(definition_file), 'arms': {}, 'incremental': {}, 'BP_relative_to_A': {}}
        dirs = {}
        for arm in ('A', 'ABP'):
            item['arms'][arm] = {}; dirs[arm] = {}
            for mode in out['modes']:
                d = root/mode/arm/mid; dirs[arm][mode] = d; rows = read(d)
                assert len(rows) == 4 and [r['query_id'] for r in rows] == [q['id'] for q in queries]
                protocol = load(d/'protocol.json'); assert protocol['queries_sha256'] == sha(definition_file)
                evidence = mode_evidence(d, 'CHNT', expected_v9_mode='A' if arm == 'A' else 'AB',
                                        expected_r2_mode='control' if arm == 'A' else 'P', expected_r8_mode=mode)
                assert not any(r['fullroute'].get('reused') or r['task_progress'].get('reused') for r in rows)
                assert all(not r['l2']['reused_search_result'] and r['l2']['response_ms'] > 0 for r in rows)
                for row in rows:
                    detail = load(d/'cases'/(row['query_id']+'_1')/'outcome.json')['reference_diagnostics']
                    assert bool(detail.get('r8_sparse_occupancy_used')) == ('O' in mode and mode != 'control')
                    assert bool(detail.get('r8_immutable_auxiliary_fields')) == ('F' in mode and mode != 'control')
                copied = load(d/'source_snapshot/hashes.json')
                for original, expected in copied.items():
                    relative = Path(original).relative_to(ROOT)
                    assert sha(d/'source_snapshot'/str(relative).replace('/', '_')) == expected
                    if str(relative) in lock:
                        assert expected == lock[str(relative)] == sha(archive/'planning_source'/relative), (d, relative)
                assert {str(Path(f).relative_to(ROOT)) for f in copied if str(Path(f).relative_to(ROOT)) in lock} == set(lock)
                item['arms'][arm][mode] = {'initial': summary(rows[:1]), 'new_tasks': summary(rows[1:]),
                    'raw_wall_ms': [r['wall_ms'] for r in rows[1:]], 'mode_and_native': evidence,
                    'source_files': len(copied), 'planning_source_files': len(lock),
                    'cleanup': verify_cleanup(d), 'runs_sha256': sha(d/'runs.jsonl')}
            item['incremental'][arm] = {}
            base = item['arms'][arm]['control']['new_tasks']['metrics']['e2e_s']['median']
            for mode in ('O', 'F', 'OF'):
                parity = exact_compare(dirs[arm]['control'], dirs[arm][mode])
                after = item['arms'][arm][mode]['new_tasks']['metrics']['e2e_s']['median']
                item['incremental'][arm][mode] = {'exact_parity': parity, 'median_change_pct': 100*(after/base-1)}
        for mode in out['modes']:
            quality = compare(dirs['A'][mode], dirs['ABP'][mode])
            assert quality['quality_gate'] and quality['upstream_gate']
            first = item['arms']['A'][mode]['new_tasks']['metrics']['e2e_s']['median']
            second = item['arms']['ABP'][mode]['new_tasks']['metrics']['e2e_s']['median']
            item['BP_relative_to_A'][mode] = {'median_change_pct': 100*(second/first-1), 'quality_and_upstream': quality}
        out['maps'][mid] = item
    out['strict_success_total'] = sum(v['arms'][arm][mode]['initial']['success']+v['arms'][arm][mode]['new_tasks']['success']
                                    for v in out['maps'].values() for arm in ('A', 'ABP') for mode in out['modes'])
    assert out['strict_success_total'] == 64
    out['n3_points_do_not_support_fixed10_or_tail_or_significance_claims'] = True
    out['adopted'] = False
    save(ROOT/'report/SHARED_SOURCE2_REVIEW.json', out)
    print('SHARED_SOURCE2_STRICT', out['strict_success_total'], 'EXACT_O_F_PARITY_PASS')
    for mid, v in out['maps'].items():
        print(mid, {arm: {m: round(z['median_change_pct'], 3) for m, z in modes.items()} for arm, modes in v['incremental'].items()})
        print('BP_relative_to_A', {m: round(z['median_change_pct'], 3) for m, z in v['BP_relative_to_A'].items()})


if __name__ == '__main__': main()
