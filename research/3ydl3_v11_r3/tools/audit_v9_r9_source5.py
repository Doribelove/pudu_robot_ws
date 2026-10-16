"""Source5 preserves source4 except one L2 data hook and clock-excluded metadata."""
from pathlib import Path
from compare_v9_review import load, save, sha
from compare_v9_r7_all import verify as frozen_resources

ROOT = Path(__file__).resolve().parents[1]
OLD_OPTIONS = "('control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM', 'U', 'PLU', 'PLMU')"
NEW_OPTIONS = "('control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM', 'U', 'PLU', 'PLMU', 'X', 'PLMX', 'PLMUX')"
OLD_CHOICES = "choices=['control','P','L','PL','M','PM','LM','PLM','U','PLU','PLMU']"
NEW_CHOICES = "choices=['control','P','L','PL','M','PM','LM','PLM','U','PLU','PLMU','X','PLMX','PLMUX']"
SCOPE_METADATA = """                scope_after=scope_stats()
                row['r9_scope_aggregation']={**scope_after,
                    'exact_scope_calls_this_request':scope_after['exact_scope_calls']-scope_before['exact_scope_calls'],
                    'original_fallback_calls_this_request':scope_after['original_fallback_calls']-scope_before['original_fallback_calls'],
                    'disabled_calls_this_request':scope_after['disabled_calls']-scope_before['disabled_calls'],
                    'fine_cells_processed_this_request':scope_after['fine_cells_processed']-scope_before['fine_cells_processed'],
                    'logical_padded_bool_buffer_bytes_avoided_this_request':scope_after['logical_padded_bool_buffer_bytes_avoided']-scope_before['logical_padded_bool_buffer_bytes_avoided'],
                    'inspection_outside_request_timing':True}
"""


def once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new)


def verify(formal_root=None):
    spec = load(ROOT/'report/SOURCE5_PROTOCOL.json')
    assert spec['source_cycle'] == 5 and spec['declared_before_source5_implementation_and_native']
    archive4 = ROOT/'report/source_cycles/source4'
    assert sha(archive4/'PLANNER_SOURCE_LOCK.json') == spec['parent_source4_planner_lock_sha256']
    assert sha(ROOT/'report/SOURCE4_REFERENCE_FIXED_REVIEW_LOCK.json') == spec['source4_completed_reference_review_lock_sha256']
    for lockname in ('SOURCE4_REFERENCE_FIXED_REVIEW_LOCK.json', 'SOURCE4_SCOPE_AGGREGATION_PROBE_LOCK.json'):
        for name, expected in load(ROOT/'report'/lockname).items():
            assert sha(ROOT/'report'/name) == expected, name
    before = load(archive4/'PLANNER_SOURCE_LOCK.json')['planning_source_files']
    current = load(ROOT/'report/PLANNER_SOURCE_LOCK.json')
    assert current['source_cycle'] == 5
    after = current['planning_source_files']
    assert set(after)-set(before) == {spec['new_planning_file']}
    assert set(before)-set(after) == set()
    changed = {name for name, expected in before.items() if sha(ROOT/name) != expected}
    assert changed == set(spec['allowed_planning_file_changes_from_source4'])
    for name, expected in after.items():
        assert sha(ROOT/name) == expected, name
    archived = archive4/'planning_source'
    controller = (archived/'src/three_yd_v5/controller.py').read_text()
    controller = once(controller, 'from .v9_r9_membership import semantic_membership\n',
                      'from .v9_r9_membership import semantic_membership\nfrom .v9_r9_scope import aggregate_scope\n')
    controller = once(controller, '        mask=aggregate(self.search_legal,3)\n',
                      '        mask=aggregate_scope(self.search_legal,3)\n')
    assert (ROOT/'src/three_yd_v5/controller.py').read_text() == controller
    options = once((archived/'src/three_yd_v5/v9_r9_options.py').read_text(), OLD_OPTIONS, NEW_OPTIONS)
    assert (ROOT/'src/three_yd_v5/v9_r9_options.py').read_text() == options
    runner = once((archived/'tools/run_task_progress.py').read_text(), OLD_CHOICES, NEW_CHOICES)
    runner = once(runner, '        from three_yd_v5.v9_r9_buffers import buffer_stats\n',
                  '        from three_yd_v5.v9_r9_buffers import buffer_stats\n        from three_yd_v5.v9_r9_scope import scope_stats\n')
    runner = once(runner, '                buffers_before=buffer_stats()\n',
                  '                buffers_before=buffer_stats()\n                scope_before=scope_stats()\n')
    runner = once(runner, '                if not rows:\n', SCOPE_METADATA+'                if not rows:\n')
    assert (ROOT/'tools/run_task_progress.py').read_text() == runner
    archives = {}
    for cycle in range(6):
        folder = ROOT/'report/source_cycles'/f'source{cycle}'
        data = load(folder/'PLANNER_SOURCE_LOCK.json')
        assert data['source_cycle'] == cycle
        for name, expected in data['planning_source_files'].items():
            assert sha(folder/'planning_source'/name) == expected, (cycle, name)
        archives[f'source{cycle}'] = len(data['planning_source_files'])
    original = load(ROOT/'report/SOURCE4_SCOPE_AGGREGATION_PROBE.json')
    assert sha(Path(original['original_module'])) == original['original_module_sha256']
    parent_spec = load(ROOT/'PARENT.json')
    parent = Path(parent_spec['parent_workspace'])
    assert sha(parent/'RELEASE.json') == parent_spec['parent_release_sha256']
    files = load(parent/'RELEASE.json')['files']
    for name, expected in files.items():
        assert sha(parent/name) == expected, name
    resources = frozen_resources(formal_root or ROOT/'results/diagnostic_source5')
    resources['inherited_generic_test_summary'] = resources['tests']
    tests = {}
    for mode in ('control', 'PLMX'):
        path = ROOT/'report'/f'tests_source5_{mode}_all.txt'
        last = path.read_text().splitlines()[-1]
        assert last.startswith('620 passed,'), (mode, last)
        tests[mode] = {'path': str(path), 'sha256': sha(path), 'summary': last}
    resources['current_source5_tests'] = tests
    resources['tests'] = tests['PLMX']['summary']
    return {'source_cycle': 5, 'planning_files': len(after), 'source4_to5_changed_files': sorted(changed),
        'exact_one_L2_data_hook_only': True, 'original_circle_aggregation_GridView_and_L1_unchanged': True,
        'original_runner_clocks_sampling_and_all_prior_metadata_unchanged': True,
        'new_metadata_outside_original_request_timing': True, 'all_source_archives_verified': archives,
        'original_external_aggregate_source_sha256': original['original_module_sha256'],
        'parent_release_files': len(files), 'parent_release_sha256': parent_spec['parent_release_sha256'],
        'frozen_native_L1_L2_audit_production_and_owned_resources': resources}


if __name__ == '__main__':
    result = verify()
    save(ROOT/'report/PROTECTED_SOURCE5_REVIEW.json', result)
    print('SOURCE5 protected/planning124/originalonehook/clocks/external/frozen/native/parent/archives/cleanup verified')
