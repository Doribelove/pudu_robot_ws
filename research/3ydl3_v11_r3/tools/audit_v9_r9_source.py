"""Verify R9, the sealed parent, and original source/input/native/audit guards."""
import argparse
from pathlib import Path
from compare_v9_review import load, save
from compare_v9_r7_all import sha, FROZEN
from compare_v8_r3 import audit_ast
from audit_v9_r6_connector import audit_connector

ROOT = Path(__file__).resolve().parents[1]


def verify(cycle):
    assert cycle in (0, 1, 2, 3, 4)
    parent_info = load(ROOT / 'PARENT.json')
    parent = Path(parent_info['parent_workspace'])
    assert sha(parent / 'RELEASE.json') == parent_info['parent_release_sha256']
    files = load(parent / 'RELEASE.json')['files']
    assert all(sha(parent / f) == h for f, h in files.items())
    out = {'source_cycle': cycle, 'parent_release_files': len(files),
           'parent_release_sha256': parent_info['parent_release_sha256']}
    for name in ('PLANNER_SOURCE_LOCK.json', 'PRODUCTION_ENTRY_LOCK.json', 'EXTERNAL_AUDIT_LOCK.json'):
        data = load(ROOT / 'report' / name)
        if name == 'PLANNER_SOURCE_LOCK.json':
            assert data['source_cycle'] == cycle
        hashes = data.get('planning_source_files', data)
        assert all(sha(Path(f) if Path(f).is_absolute() else ROOT / f) == h for f, h in hashes.items())
        out[name] = len(hashes)
    lock = load(ROOT / 'L1_LOCK.json')
    for section in ('source_files', 'input_files', 'pixel_input_files'):
        assert all(sha(ROOT / f) == h for f, h in lock[section].items())
        out[section] = len(lock[section])
    for f in ('src/three_yd_v4/controller.py', 'src/three_yd_v4/r5_controller.py', 'src/three_yd_v5/controller.py'):
        assert audit_ast(ROOT / f) == audit_ast(FROZEN / f)
    out['original_audit_AST'] = True
    for f in ('src/three_yd_v5/task_progress.py', 'native/libcold_dstar.so', 'native/cold_dstar.cpp'):
        assert sha(ROOT / f) == sha(FROZEN / f)
    out['TaskProgress_L2_native_original'] = True
    out['original_connector_guards'] = audit_connector(ROOT, FROZEN)
    natives = [f for f in (parent / 'native').rglob('*') if f.is_file()]
    natives += [f for f in (parent / 'install').rglob('*.so') if f.is_file()]
    assert all(sha(ROOT / f.relative_to(parent)) == sha(f) for f in natives)
    out['native_files'] = len(natives)
    for current in range(cycle + 1):
        archive = ROOT / 'report/source_cycles' / f'source{current}'
        data = load(archive / 'PLANNER_SOURCE_LOCK.json')
        assert data['source_cycle'] == current
        assert all(sha(archive / 'planning_source' / f) == h for f, h in data['planning_source_files'].items())
        out[f'archive{current}'] = len(data['planning_source_files'])
    if cycle == 0:
        before = load(ROOT / 'report/PARENT_PLANNER_SOURCE_LOCK.json')['planning_source_files']
        changed = [f for f, h in before.items() if sha(ROOT / f) != h]
        assert set(changed) == {'env.bash', 'tools/run_task_progress.py', 'tools/run_progress_events.py'}
        for f in changed:
            expected = (parent / f).read_text().replace(str(parent), str(ROOT))
            expected = expected.replace('3YD-V9-r8', '3YD-V9-r9').replace('V9-r8-audit-L3', 'V9-r9-audit-L3')
            assert (ROOT / f).read_text() == expected
        out['only_inherited_changes_environment_version'] = changed
    else:
        before = load(ROOT / 'report/PARENT_PLANNER_SOURCE_LOCK.json')['planning_source_files']
        changed = [f for f, h in before.items() if sha(ROOT / f) != h]
        expected_changed = {'env.bash', 'src/three_yd_v5/fullroute.py', 'tools/run_task_progress.py',
                            'tools/run_progress_events.py', 'tools/run_v9_selected.py'}
        if cycle >= 2:
            expected_changed.add('src/three_yd_v5/v9_storage.py')
        if cycle >= 3:
            expected_changed.add('src/three_yd_v5/controller.py')
        if cycle == 4:
            expected_changed.add('src/three_yd_v5/fullroute_service.py')
        assert set(changed) == expected_changed
        expected = (parent / 'src/three_yd_v5/fullroute.py').read_text()
        expected = expected.replace('        else:value=deepcopy(value)',
                                    '        else:\n            from .v9_r9_records import copy_for_cache\n            value=copy_for_cache(value)')
        expected = expected.replace("    return {'result':result,'bounded_l3':out.get('bounded_l3',{}),'v5':out.get('v5',{})}",
                                    "    from .v9_r9_records import pack_internal\n    result=replace(result,points=pack_internal(result.points))\n    return {'result':result,'bounded_l3':out.get('bounded_l3',{}),'v5':out.get('v5',{})}")
        assert (ROOT / 'src/three_yd_v5/fullroute.py').read_text() == expected
        expected_new = {'src/three_yd_v5/v9_r9_options.py', 'src/three_yd_v5/v9_r9_records.py'}
        if cycle >= 2:
            expected_new.add('src/three_yd_v5/v9_r9_lookup.py')
            original = (parent / 'src/three_yd_v5/v9_storage.py').read_text()
            expected_static = original.replace(
                '    return table[occupancy.view(np.uint8)]',
                '    from .v9_r9_lookup import exact_lookup\n'
                '    result=exact_lookup(occupancy,table)\n'
                '    if result is not None:return result\n'
                '    return table[occupancy.view(np.uint8)]')
            assert (ROOT / 'src/three_yd_v5/v9_storage.py').read_text() == expected_static
            # The entire source matches the one lookup hook; the auditor body
            # and every other declaration are thereby unchanged.
            out['original_storage_except_exact_lookup_hook'] = True
        if cycle >= 3:
            expected_new.add('src/three_yd_v5/v9_r9_membership.py')
            archive2 = ROOT / 'report/source_cycles/source2/planning_source'
            previous = (archive2 / 'src/three_yd_v5/controller.py').read_text()
            expected_controller = previous.replace(
                'from .v9_options import enabled as v9_enabled\n',
                'from .v9_options import enabled as v9_enabled\nfrom .v9_r9_membership import semantic_membership\n')
            for target in ('np.isin(self.topology.subregions,parking_ids)',
                           'np.isin(self.topology.subregions,selected_ids)',
                           'np.isin(self.topology.subregions,lane_junction_ids)',
                           'np.isin(topology.subregions,sorted(selected_physical))',
                           'np.isin(topology.subregions,[rid for rid,r in topology.regions.items()',
                           'np.isin(topology.subregions,route.region_ids)'):
                assert expected_controller.count(target) == 1
                expected_controller = expected_controller.replace(target, target.replace('np.isin', 'semantic_membership'))
            if cycle == 4:
                expected_controller = expected_controller.replace(
                    '        self.search_legal=self.priority.copy()\n',
                    '        from .v9_r9_buffers import initial_search_scope\n'
                    '        self.priority,self.search_legal=initial_search_scope(self.priority,self.generic_mode)\n')
            assert (ROOT / 'src/three_yd_v5/controller.py').read_text() == expected_controller
            previous_options = (archive2 / 'src/three_yd_v5/v9_r9_options.py').read_text()
            mode_tuple = ("('control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM', 'U', 'PLU', 'PLMU')" if cycle == 4
                          else "('control', 'P', 'L', 'PL', 'M', 'PM', 'LM', 'PLM')")
            assert (ROOT / 'src/three_yd_v5/v9_r9_options.py').read_text() == previous_options.replace(
                "('control', 'P', 'L', 'PL')", mode_tuple)
            assert sha(ROOT / 'src/three_yd_v5/v9_r9_records.py') == sha(archive2 / 'src/three_yd_v5/v9_r9_records.py')
            assert sha(ROOT / 'src/three_yd_v5/v9_r9_lookup.py') == sha(archive2 / 'src/three_yd_v5/v9_r9_lookup.py')
            previous_runner = (archive2 / 'tools/run_task_progress.py').read_text()
            choices = ("choices=['control','P','L','PL','M','PM','LM','PLM','U','PLU','PLMU']" if cycle == 4
                       else "choices=['control','P','L','PL','M','PM','LM','PLM']")
            expected_runner = previous_runner.replace("choices=['control','P','L','PL']", choices)
            expected_runner = expected_runner.replace(
                '        from three_yd_v5.v9_r9_lookup import lookup_stats\n',
                '        from three_yd_v5.v9_r9_lookup import lookup_stats\n        from three_yd_v5.v9_r9_membership import membership_stats\n')
            expected_runner = expected_runner.replace('                lookup_before=lookup_stats()\n',
                '                lookup_before=lookup_stats()\n                membership_before=membership_stats()\n')
            new_metadata = """                membership_after=membership_stats()
                row['r9_semantic_membership']={**membership_after,
                    'native_membership_calls_this_request':membership_after['native_membership_calls']-membership_before['native_membership_calls'],
                    'original_fallback_calls_this_request':membership_after['original_fallback_calls']-membership_before['original_fallback_calls'],
                    'disabled_calls_this_request':membership_after['disabled_calls']-membership_before['disabled_calls'],
                    'lookup_cells_this_request':membership_after['lookup_cells']-membership_before['lookup_cells'],
                    'inspection_outside_request_timing':True}
"""
            expected_runner = expected_runner.replace('                if not rows:\n', new_metadata + '                if not rows:\n')
            if cycle == 4:
                expected_runner = expected_runner.replace(
                    '        from three_yd_v5.v9_r9_membership import membership_stats\n',
                    '        from three_yd_v5.v9_r9_membership import membership_stats\n'
                    '        from three_yd_v5.v9_r9_buffers import buffer_stats\n')
                expected_runner = expected_runner.replace(
                    '                membership_before=membership_stats()\n',
                    '                membership_before=membership_stats()\n'
                    '                buffers_before=buffer_stats()\n')
                buffer_metadata = """                buffers_after=buffer_stats()
                row['r9_buffer_storage']={**buffers_after,
                    'scope_shared_calls_this_request':buffers_after['scope_shared_calls']-buffers_before['scope_shared_calls'],
                    'scope_duplicate_buffer_bytes_avoided_this_request':buffers_after['scope_duplicate_buffer_bytes_avoided']-buffers_before['scope_duplicate_buffer_bytes_avoided'],
                    'inspection_outside_request_timing':True}
"""
                expected_runner = expected_runner.replace('                if not rows:\n', buffer_metadata + '                if not rows:\n')
                archive3 = ROOT / 'report/source_cycles/source3/planning_source'
                previous_service = (archive3 / 'src/three_yd_v5/fullroute_service.py').read_text()
                expected_service = previous_service.replace(
                    '            self.dynamic.static_hard=seal(self.dynamic.static_hard)\n',
                    '            from .v9_r9_buffers import initial_static_hard\n'
                    '            self.dynamic.static_hard=initial_static_hard(world.hard,self.dynamic.static_hard)\n')
                assert (ROOT / 'src/three_yd_v5/fullroute_service.py').read_text() == expected_service
                expected_new.add('src/three_yd_v5/v9_r9_buffers.py')
                unchanged3 = load(ROOT / 'report/source_cycles/source3/PLANNER_SOURCE_LOCK.json')['planning_source_files']
                allowed = {'src/three_yd_v5/controller.py', 'src/three_yd_v5/fullroute_service.py',
                           'src/three_yd_v5/v9_r9_options.py', 'tools/run_task_progress.py'}
                assert {f for f,h in unchanged3.items() if sha(ROOT / f) != h} == allowed
                out['source3_to4_exact_two_storage_hooks_and_outside_clock_metadata'] = sorted(allowed)
            assert (ROOT / 'tools/run_task_progress.py').read_text() == expected_runner
            out['source3_six_data_mask_hooks_preserved' if cycle == 4 else 'exact_six_data_mask_calls_and_import_only'] = True
            out['original_runner_clocks_and_sampling_unchanged'] = True
        assert set(load(ROOT / 'report/PLANNER_SOURCE_LOCK.json')['planning_source_files']) - set(before) == expected_new
        assert sha(ROOT / 'src/three_yd_v5/progress_connector.py') == sha(parent / 'src/three_yd_v5/progress_connector.py')
        out['only_data_representation_hooks'] = changed
    save(ROOT / 'report' / f'PROTECTED_SOURCE{cycle}_REVIEW.json', out)
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-cycle', type=int, required=True)
    print(verify(parser.parse_args().source_cycle))
