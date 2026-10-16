"""Verify the original Native120 owner diagnostic without accepting its timing."""
from pathlib import Path
import numpy as np
from compare_v9 import read, tag, pair
from compare_v9_review import load, save, sha
from compare_v9_r9 import exact_compare, compare
from review_v9_r9_full_progress import expected, source
from review_v9_r9_source3 import features
from audit_v9_r9_source import verify

ROOT = Path(__file__).resolve().parents[1]


def main():
    protocol = load(ROOT/'report/SOURCE3_STORAGE_DIAGNOSTIC_PROTOCOL.json')
    root = Path(protocol['output']); mid = protocol['map_id']
    assert sha(Path(protocol['fixture'])) == protocol['fixture_sha256']
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == protocol['planning_lock_sha256']
    assert sha(ROOT/'report/SOURCE3_FULL_PROGRESS_REVIEW.json') == protocol['reference_failed_review_sha256']
    result = {'source_cycle': 3, 'diagnostic_only': True, 'adopted': False,
              'timing_used_for_formal_acceptance': False, 'strict_total': 0,
              'fixture_sha256': protocol['fixture_sha256'], 'modes': {},
              'owner_ledger_limits': protocol['limits']}
    directories = {m: root/'moving'/m/mid for m in protocol['modes']}
    for mode, directory in directories.items():
        p = load(directory/'protocol.json'); rows = read(directory); queries = expected(p, mid)
        assert len(rows) == len(queries) == 40
        assert (p['pose_offset_m'], p['yaw_offset_deg'], p['profiler']) == (0., 0., False)
        for row, query in zip(rows, queries):
            assert row['success'] and row['query_id'] == query['id']
            assert row['start'] == query['start'] and row['goal'] == query['goal']
        assert len(pair(ROOT/'results/full_progress_source3/moving'/mode/mid, directory, strict=True)) == 40
        ledgers = [('prepared', load(directory/'storage_prepared.json'))]
        ledgers += [(tag(row), load(directory/'cases'/tag(row)/'storage.json')) for row in rows]
        raw = []
        duplicate_pairs = []
        for label, ledger in ledgers:
            assert ledger['schema'] == 'ndarray-owner-ledger-v1'
            assert ledger['diagnostic_only'] and not ledger['properties_evaluated'] and not ledger['truncated']
            assert ledger['unique_array_buffer_bytes'] == sum(v['bytes'] for v in ledger['buffers'])
            paths = {v['path']: v for v in ledger['arrays']}
            for left, right in [('world.hard', 'service.dynamic.static_hard'),
                                ('service.controller.priority', 'service.controller.search_legal')]:
                if left in paths and right in paths:
                    a, b = paths[left], paths[right]
                    assert a['dtype'] == b['dtype'] == 'bool' and a['shape'] == b['shape'] == [6000, 5000]
                    assert a['owner'] != b['owner']
                    duplicate_pairs.append({'case': label, 'paths': [left, right],
                                            'each_buffer_bytes': a['logical_bytes'],
                                            'distinct_owners': True,
                                            'equality_contract': 'original constructor/copy assignment; candidate must still verify its own guard'})
            raw.append({'case': label, 'unique_array_buffer_bytes': ledger['unique_array_buffer_bytes'],
                        'objects_visited': ledger['objects_visited'], 'ledger_sha256': sha(
                            directory/'storage_prepared.json' if label == 'prepared' else directory/'cases'/label/'storage.json')})
        actual = features(rows, 'control' if mode == 'AS' else mode, mid)
        result['modes'][mode] = {'strict': 40, 'source_native_cleanup': source(directory, mode, rows, 3),
                                'physical_exact_to_prior_same_source_batch': True,
                                'features': actual, 'ledger_raw': raw, 'duplicate_pairs': duplicate_pairs,
                                'prepared_reachable_array_MiB': raw[0]['unique_array_buffer_bytes']/2**20,
                                'request_reachable_array_MiB_median': float(np.median([v['unique_array_buffer_bytes']/2**20 for v in raw[1:]]))}
        result['strict_total'] += len(rows)
    result['candidate_control_exact'] = exact_compare(directories['control'], directories['PLM'])
    result['original_AS_quality'] = {m: compare(directories['AS'], directories[m]) for m in ('control', 'PLM')}
    assert all(v['upstream_gate'] and v['quality_gate'] for v in result['original_AS_quality'].values())
    result['protected'] = verify(3)
    assert result['strict_total'] == protocol['expected_records'] == 120
    save(ROOT/'report/SOURCE3_STORAGE_DIAGNOSTIC_REVIEW.json', result)
    print('Native owner diagnostic120/120 strict; original40 poses each, prior same-mode outputs exact, AS original quality, source/Native/cleanup verified.')
    print('Reachable array median MiB', {m: round(v['request_reachable_array_MiB_median'], 2) for m, v in result['modes'].items()})
    print('Distinct full-map static and priority/search-legal buffers identified; no timing or peak memory adoption claim.')


if __name__ == '__main__':
    main()
