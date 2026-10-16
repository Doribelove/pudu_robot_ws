"""Audit diagnostic profiling; cumulative clocks are never formal speed proof."""
import json
from pathlib import Path
import pstats
from compare_v9 import read, tag
from compare_v9_review import load, save, sha
from compare_v9_r9 import mode_evidence
from compare_v8_r6 import verify_cleanup
from moving_starts import load as load_fixture
from compare_v8_r3 import libraries
from quality_v9_r6 import quality
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def profile_output_review(before, after):
    """Check output quality without letting profiles enter the formal pair gate."""
    assert {Path(p).name: h for p, h in libraries(before).items()} == {Path(p).name: h for p, h in libraries(after).items()}
    old = {tag(row): row for row in read(before)}
    new = {tag(row): row for row in read(after)}
    assert old.keys() == new.keys()
    reviewed = []
    for key, row in new.items():
        previous = old[key]
        for name in ('l1_output.json', 'l2_reference_xy.npy', 'l2_soft_target.json'):
            assert sha(before / 'cases' / key / name) == sha(after / 'cases' / key / name)
        left = np.load(before / 'cases' / key / 'l3_scope.npz')['mask']
        right = np.load(after / 'cases' / key / 'l3_scope.npz')['mask']
        assert np.array_equal(left, right)
        for field in ('actual_cost_hash', 'path_cost', 'dstar_expanded', 'state_memory_bytes'):
            assert previous['l2'][field] == row['l2'][field]
        detail = row['l3']
        for field, original in [('r5_source_full_grid_hash', 'semantic_expected_grid_hash'),
                                ('r5_source_full_master_hash', 'semantic_expected_master_hash')]:
            assert detail[field] == previous['l3'][original]
        assert detail['r5_source_full_mask_hash'] == sha_array(right)
        assert detail['costmap_update_acknowledged'] and detail['costmap_ack_mismatch_cells'] == 0
        assert detail['semantic_exact_stable_observations'] >= 2
        assert detail['server_costmap_content_hash'] == detail['semantic_expected_server_content_hash']
        assert detail['r5_complete_scope_not_rolling'] and not detail['r5_original_map_loaded_in_nav2']
        box = detail['r5_complete_task_bounds']
        h, w = detail['r5_native_shape']
        assert [h, w] == [box[1] - box[0], box[3] - box[2]]
        assert not (right[:box[0]].any() or right[box[1]:].any() or right[:, :box[2]].any() or right[:, box[3]:].any())
        values = [load(directory / 'cases' / key / 'outcome.json')['metrics'] for directory in (before, after)]
        q = quality(previous['length_m'], row['length_m'], values[0]['minimum_clearance_m'], values[1]['minimum_clearance_m'])
        assert q['quality_gate']
        reviewed.append({'tag': key, 'upstream_exact': True, 'current_ACK_checked': True, 'quality': q,
                         'physical_final_path_equal': sha(before / 'cases' / key / 'path.npy') == sha(after / 'cases' / key / 'path.npy')})
    return reviewed


def sha_array(value):
    import hashlib
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def main():
    results = ROOT / 'results/profile_source0'
    archive = ROOT / 'report/source_cycles/source0'
    planning = load(archive / 'PLANNER_SOURCE_LOCK.json')['planning_source_files']
    report = {'source_cycle': 0, 'diagnostic_only': True, 'profiler': True,
              'formal_performance_claim': False, 'arms': {}}
    for arm, storage, r8 in [('AS', 'control', 'control'), ('OGR', 'CHNT', 'OGR')]:
        directory = results / arm / 'arena_large_250x300_2d_005'
        protocol = load(directory / 'protocol.json')
        native = mode_evidence(directory, storage, expected_r8_mode=r8, expected_profile=True)
        assert protocol['diagnostic_subset'] and protocol['workload'] == 'moving'
        assert protocol['queries_sha256'] == sha(Path(protocol['query_file']))
        original = [q for q in load_fixture(Path(protocol['query_file']), protocol['map_id']) if q['id'] == 'A2B-08']
        rows = read(directory)
        assert len(rows) == len(original) == 4
        assert [r['moving_start']['step'] for r in rows] == ['initial', 'p20', 'p50', 'p80']
        for row, query in zip(rows, original):
            assert row['query_id'] == query['id'] == 'A2B-08'
            assert row['start'] == query['start'] and row['goal'] == query['goal']
            assert row['success'] and row['audit']['valid']
            assert row['constraints']['steering_jump_valid'] and row['constraints']['traffic']['valid']
        copies = load(directory / 'source_snapshot/hashes.json')
        found = set()
        for original_file, expected in copies.items():
            relative = Path(original_file).relative_to(ROOT)
            assert sha(directory / 'source_snapshot' / str(relative).replace('/', '_')) == expected
            if str(relative) in planning:
                assert expected == planning[str(relative)] == sha(archive / 'planning_source' / relative)
                found.add(str(relative))
        assert found == set(planning)
        if arm == 'AS':
            logs = '\n'.join(p.read_text() for p in (directory / 'smac/logs').glob('*.log'))
            assert '5000 X 6000 map @ 0.05 m/cell' in logs
            native['actual_full_map_loaded'] = [6000, 5000]
        profiles = []
        for row in rows:
            profile_path = directory / 'cases' / tag(row) / 'query.prof'
            stats = pstats.Stats(str(profile_path))
            functions = [{'file': file, 'line': line, 'function': name,
                          'primitive_calls': cc, 'calls': nc,
                          'self_ms': tt * 1000, 'cumulative_ms': ct * 1000}
                         for (file, line, name), (cc, nc, tt, ct, callers) in stats.stats.items()]
            functions.sort(key=lambda item: item['cumulative_ms'], reverse=True)
            profiles.append({'tag': tag(row), 'profile_sha256': sha(profile_path),
                             'profiled_request_wall_ms': row['wall_ms'],
                             'match': row['task_progress'].get('match'),
                             'connector': row['task_progress'].get('connector'),
                             'functions': functions})
        report['arms'][arm] = {'strict': 4, 'planning_files': len(planning), 'copied_files': len(copies),
                               'native': native, 'cleanup': verify_cleanup(directory),
                               'runs_sha256': sha(directory / 'runs.jsonl'), 'profiles': profiles}
    parity = profile_output_review(results / 'AS/arena_large_250x300_2d_005', results / 'OGR/arena_large_250x300_2d_005')
    report['upstream_and_quality'] = parity
    report['strict_total'] = 8
    report['finding'] = 'Additional original connector audits plus record deepcopy/resident_bytes traversal; no EDT in original SweptChecker.'
    save(ROOT / 'report/PROFILE_SOURCE0_REVIEW.json', report)
    print('SOURCE0 ORIGINAL A2B-08 PROFILE: 8/8 STRICT; ORIGINAL INPUT/SOURCE/NATIVE/QUALITY/CLEANUP PASS')
    for arm, data in report['arms'].items():
        for profile in data['profiles'][1:]:
            top = [f for f in profile['functions'] if f['function'] in ('_retain', 'resident_bytes', 'propose')]
            print(arm, profile['tag'], profile['match'], [(Path(f['file']).name, f['function'], round(f['cumulative_ms'], 3)) for f in top])


if __name__ == '__main__':
    main()
