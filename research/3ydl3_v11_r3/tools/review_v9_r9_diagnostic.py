"""Audit the original diagnostic subset; keep every raw step and source copy."""
import argparse
from copy import deepcopy
from pathlib import Path
import numpy as np
from compare_v9 import read, tag, pair, parts, summary
from compare_v9_review import load, save, sha
from compare_v9_r9 import mode_evidence, exact_compare
from compare_v8_r6 import verify_cleanup
from moving_starts import load as load_fixture

ROOT = Path(__file__).resolve().parents[1]
MAPS = ('mengjia', 'arena_large_250x300_2d_005')


def source_proof(directory, mode, cycle=1, profile=False, arm='A', storage_mode='CHNT', r8_mode='OGR', r2_mode=None):
    protocol = load(directory / 'protocol.json')
    assert protocol['queries_sha256'] == sha(Path(protocol['query_file']))
    native = mode_evidence(directory, storage_mode, expected_r8_mode=r8_mode,
                           expected_r9_mode=mode, expected_profile=profile,
                           expected_v9_mode='A' if arm == 'A' else 'AB',
                           expected_r2_mode=('control' if arm == 'A' else 'P') if r2_mode is None else r2_mode)
    archive = ROOT / 'report/source_cycles' / f'source{cycle}'
    planning = load(archive / 'PLANNER_SOURCE_LOCK.json')['planning_source_files']
    copies = load(directory / 'source_snapshot/hashes.json')
    found = set()
    for original, expected in copies.items():
        relative = Path(original).relative_to(ROOT)
        assert sha(directory / 'source_snapshot' / str(relative).replace('/', '_')) == expected
        if str(relative) in planning:
            assert expected == planning[str(relative)] == sha(archive / 'planning_source' / relative)
            found.add(str(relative))
    assert found == set(planning)
    return {'planning_files': len(planning), 'copied_files': len(copies), 'native': native,
            'cleanup': verify_cleanup(directory), 'runs_sha256': sha(directory / 'runs.jsonl')}


def expected_queries(protocol, mid):
    selected = 'Q09' if mid == 'mengjia' else 'A2B-08'
    queries = [deepcopy(q) for q in load_fixture(Path(protocol['query_file']), mid) if q['id'] == selected]
    for query in queries:
        start = query['start']
        values = np.asarray([start[k] for k in ('x', 'y', 'yaw_rad')] if isinstance(start, dict) else start, float)
        if query['moving_start']['kind'] == 'moving' and (protocol['pose_offset_m'] or protocol['yaw_offset_deg']):
            sign = -1 if query['moving_start']['step'] == 'p50' else 1
            values[:2] += sign * protocol['pose_offset_m'] * np.array([-np.sin(values[2]), np.cos(values[2])])
            values[2] = (values[2] + sign * np.deg2rad(protocol['yaw_offset_deg']) + np.pi) % (2 * np.pi) - np.pi
        query['start'] = values.tolist()
        goal = query['goal']
        query['goal'] = [goal[k] for k in ('x', 'y', 'yaw_rad')] if isinstance(goal, dict) else goal
    return queries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, required=True)
    args = parser.parse_args()
    root = args.results_root.resolve()
    report = {'version': '3YD-V9-r9', 'source_cycle': 1, 'candidate': 'P',
              'diagnostic_only': True, 'profiler': False, 'adopted': False,
              'full_fixed10_performance_acceptance': False, 'maps': {}}
    for kind in ('moving', 'perturbed'):
        report['maps'][kind] = {}
        for mid in MAPS:
            directories = {mode: root / kind / mode / mid for mode in ('control', 'P')}
            data = {mode: read(directory) for mode, directory in directories.items()}
            evidence = {}
            for mode, directory in directories.items():
                protocol = load(directory / 'protocol.json')
                assert protocol['diagnostic_subset'] and protocol['workload'] == 'moving' and protocol['repeats'] == 1
                assert (protocol['pose_offset_m'], protocol['yaw_offset_deg']) == ((0., 0.) if kind == 'moving' else (.03, 2.))
                expected = expected_queries(protocol, mid)
                rows = data[mode]
                assert len(rows) == len(expected) == 4
                assert [r['moving_start']['step'] for r in rows] == ['initial', 'p20', 'p50', 'p80']
                for row, query in zip(rows, expected):
                    assert row['start'] == query['start'] and row['goal'] == query['goal']
                    assert row['query_id'] == query['id']
                    observed = row['r9_record_storage']
                    assert observed['mode_enabled'] == (mode == 'P')
                    assert observed['internal_points_packed'] == (mode == 'P')
                    assert not observed['audit_or_ACK_results_cached']
                    if mode == 'P':
                        assert observed['float_buffers_immutable']
                evidence[mode] = source_proof(directory, mode)
            parity = exact_compare(directories['control'], directories['P'])
            parent = Path(load(ROOT / 'PARENT.json')['parent_workspace']) / 'results/regression_source4' / kind / 'OGR' / mid
            inherited_parity = pair(parent, directories['control'], strict=True)
            assert len(inherited_parity) == 4
            groups = {mode: {key: summary(value) for key, value in parts(rows, 'moving').items()}
                      for mode, rows in data.items()}
            before = groups['control']['moving']['metrics']
            after = groups['P']['moving']['metrics']
            change = {metric: {key: 100 * (after[metric][key] / before[metric][key] - 1)
                               for key in ('median', 'p95')} for metric in ('e2e_s', 'tree_rss_gib')}
            steps = [{'tag': tag(x), 'before_s': x['wall_ms'] / 1000, 'after_s': y['wall_ms'] / 1000,
                      'before_RSS_GiB': x['rss_peak_bytes'] / 2**30, 'after_RSS_GiB': y['rss_peak_bytes'] / 2**30,
                      'match': [x['task_progress'].get('match'), y['task_progress'].get('match')],
                      'connector_trials_before': x['task_progress'].get('connector', {}).get('trials') if x['task_progress'].get('connector') else None,
                      'connector_trials_after': y['task_progress'].get('connector', {}).get('trials') if y['task_progress'].get('connector') else None}
                     for x, y in zip(data['control'], data['P'])]
            assert all(step['match'][0] == step['match'][1] and step['connector_trials_before'] == step['connector_trials_after'] for step in steps)
            report['maps'][kind][mid] = {'groups': groups, 'changed_request_change_pct': change,
                                         'exact_parity': parity, 'parent_control_subset_exact': inherited_parity,
                                         'source_native_cleanup': evidence, 'per_step': steps}
    report['strict_total'] = 32
    save(ROOT / 'report/SOURCE1_DIAGNOSTIC_REVIEW.json', report)
    print('SOURCE1 ORIGINAL DIAGNOSTIC: 32/32 STRICT; PHYSICAL OUTPUT/MAP/PARENT SUBSET EXACT')
    for kind, maps in report['maps'].items():
        for mid, info in maps.items():
            print(kind, mid, info['changed_request_change_pct'])


if __name__ == '__main__':
    main()
