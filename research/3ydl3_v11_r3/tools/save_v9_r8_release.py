"""Seal R8's reviewed static candidate and retain unmet overall goal work."""
from pathlib import Path
import json

from compare_v9_review import load, save
from compare_v9_r7_all import sha
from compare_v8_r6 import verify_cleanup

ROOT = Path(__file__).resolve().parents[1]


def read_report(name):
    return load(ROOT / 'report' / name)


def strict_records(root, expected):
    records = []
    for path in sorted(root.rglob('runs.jsonl')):
        rows = [json.loads(line) for line in path.read_text().splitlines() if line]
        assert all(row['success'] and row['audit']['valid'] for row in rows), path
        records.extend(rows)
    assert len(records) == expected, (root, len(records), expected)
    return len(records)


def main():
    assert not (ROOT / 'RELEASE.json').exists(), 'sealed releases are immutable'
    baseline = load(ROOT / 'BASELINE.json')
    adoption = read_report('R8_FINAL_ADOPTION_REVIEW.json')
    assert baseline['selected_mode'] == adoption['selected_mode'] == 'ASCHNT+OGR'
    assert baseline['adopted'] and baseline['reviewed_default_entry']
    assert adoption['adoption_eligible'] and all(adoption['adoption_gates'].values())
    assert adoption['B_P_default_disabled'] and adoption['F_not_adopted']
    modes = dict(v9_mode='A', review_mode='S', r2_mode='control', r3_mode='control',
                 r5_mode='C', r6_mode='H', r7_mode='NT', r8_mode='OGR')
    entry_configuration = read_report('ENTRY_CONFIGURATION_SOURCE5.json')
    assert all(baseline[k] == entry_configuration['BASELINE'][k] == v for k, v in modes.items())
    assert entry_configuration['source4_to5_changed_files'] == ['tools/run_v9_selected.py']
    fixed = read_report('SOURCE4_FIXED_GATES.json')
    assert fixed['both_fixed_first_gates_pass'] and fixed['all_remaining_fixed_gates_pass']
    reference = read_report('SOURCE4_AS_REFERENCE_REVIEW.json')
    assert reference['fixed_combined_gates_pass'] and not reference['historical_timing_used']
    progress = read_report('SOURCE4_REGRESSION_REVIEW.json')
    assert progress['strict_progress_total'] == 320 and progress['incremental_progress_gates_pass']
    assert progress['event_assertions'] == 44
    assert all(d['same_outcomes'] for d in progress['events'].values())
    assert read_report('SOURCE4_PROGRESS_MEMORY_REVIEW.json')['all_request_rss_gates_pass']
    original_progress = read_report('SOURCE4_AS_PROGRESS_REVIEW.json')
    assert not original_progress['historical_timing_used']
    for maps in original_progress['maps'].values():
        for data in maps.values():
            assert data['parity']['quality_gate'] and data['parity']['upstream_gate']
    tail = original_progress['maps']['moving']['arena_large_250x300_2d_005']['changed_request_change_pct']['e2e_s']['p95']
    assert tail == adoption['original_AS_exact_arena_P95_regression_pct'] and tail > 5
    cold = read_report('SOURCE4_COLD_REVIEW.json')
    assert cold['strict_total'] == 12 and all(d['each_matches_fixed_first'] for d in cold['maps'].values())
    assert not cold['both_cold_first_medians_improve_at_least3']
    default = read_report('SOURCE5_DEFAULT_ENTRY_REVIEW.json')
    assert default['actual_run_sh_default'] and default['strict_total'] == 2
    assert default['BASELINE_sha256'] == entry_configuration['BASELINE_sha256']
    assert all(d['actual_selected_entry'] and d['strict_success'] == 1 for d in default['maps'].values())
    tests = (ROOT / 'report/tests_final.txt').read_text().splitlines()[-1]
    assert '423 passed' in tests and 'failed' not in tests
    protected = read_report('PROTECTED_SOURCE5_REVIEW.json')
    assert protected['source_cycle'] == 5 and protected['checks']['R7_release_files'] == 16559
    frozen = read_report('ALL_FROZEN_AND_RESOURCE_SOURCE5_REVIEW.json')
    assert frozen['original_strict_audit_AST_unchanged'] and frozen['task_progress_L2_native_unchanged']
    assert frozen['frozen_archive_unchanged'] and frozen['original_native_files_unchanged'] == 16
    assert '423 passed' in frozen['tests']
    planning = read_report('PLANNER_SOURCE_LOCK.json')
    assert planning['source_cycle'] == 5 and len(planning['planning_source_files']) == 118
    assert all(sha(ROOT / f) == h for f, h in planning['planning_source_files'].items())
    archived4 = read_report('source_cycles/source4/PLANNER_SOURCE_LOCK.json')['planning_source_files']
    assert set(archived4) == set(planning['planning_source_files'])
    changed = [f for f, h in planning['planning_source_files'].items() if archived4[f] != h]
    assert changed == ['tools/run_v9_selected.py'], changed
    manifests = []
    for path in sorted((ROOT / 'results').rglob('source_snapshot/hashes.json')):
        files = load(path)
        for original, expected in files.items():
            relative = Path(original).relative_to(ROOT)
            assert sha(path.parent / str(relative).replace('/', '_')) == expected, (path, relative)
        manifests.append({'manifest': str(path), 'copied_files_verified': len(files)})
    cleanup = [verify_cleanup(Path(str(path)[:-len('.supervisor.json')]))
               for path in sorted((ROOT / 'results').rglob('*.supervisor.json'))]
    assert len(manifests) == baseline['original_source_copy_manifests'] == 96
    assert len(cleanup) == baseline['cleanup_batches'] == 97
    manifest_roots = {str(Path(d['manifest']).parent.parent) for d in manifests}
    cleanup_roots = {str(Path(p[:-len('.supervisor.json')])) for p in cleanup}
    missing = cleanup_roots - manifest_roots
    exception = read_report('CLEANUP_WITHOUT_SNAPSHOT_REVIEW.json')
    assert exception['cleanup_complete'] and len(exception['only_missing_snapshot']) == 1
    documented = exception['only_missing_snapshot'][0]
    assert missing == {documented['directory']} and documented['state'] == 'complete'
    assert not documented['has_runs'] and not (Path(documented['directory']) / 'runs.jsonl').exists()
    counts = {
        'fixed_parent_and_candidate': strict_records(ROOT / 'results/formal_source4', 120),
        'fixed_original_AS': strict_records(ROOT / 'results/reference_source4', 60),
        'moving_parent_and_candidate': strict_records(ROOT / 'results/regression_source4/moving', 160),
        'perturbed_parent_and_candidate': strict_records(ROOT / 'results/regression_source4/perturbed', 160),
        'moving_and_perturbed_original_AS': strict_records(ROOT / 'results/reference_progress_source4', 160),
        'cold': strict_records(ROOT / 'results/cold_source4', 12),
    }
    assert sum(counts.values()) == baseline['formal_planning_records'] == 672
    assert strict_records(ROOT / 'results/default_source5', 2) == baseline['entry_smoke_records']
    evidence = {
        'selected_mode': baseline['selected_mode'], 'source_cycle': 5,
        'planning_files_verified': 118, 'source4_to5_changed_files': changed,
        'original_copy_manifests': manifests, 'cleanup_batches': 97,
        'cleanup_without_snapshot': documented, 'formal_records_by_phase': counts,
        'formal_strict_planning_records': 672, 'entry_smoke_strict_records': 2,
        'diagnostic_progress_records': 32, 'diagnostic_shared_records': 32,
        'event_assertions_separate': 44, 'tests': 423,
        'all_declared_adoption_gates_passed': True,
        'cold_benefit_at_least3': False, 'first_or_cold_gate_met_by_fixed_first': True,
        'B_P_default_disabled': True, 'F_not_adopted': True,
        'original_AS_exact_arena_P95_regression_pct': tail,
        'remaining_overall_goal_work': adoption['remaining_overall_goal_work'],
        'goal_remains_active': True,
        'frozen_resource_review_sha256': sha(ROOT / 'report/ALL_FROZEN_AND_RESOURCE_SOURCE5_REVIEW.json'),
        'parent_source_review_sha256': sha(ROOT / 'report/PROTECTED_SOURCE5_REVIEW.json'),
        'historical_production_entry_unchanged': True,
    }
    save(ROOT / 'report/RELEASE_EVIDENCE.json', evidence)
    baseline.update(status='selected_static_fulltask_candidate_sealed', goal_remains_active=True)
    save(ROOT / 'BASELINE.json', baseline)
    progress_path = ROOT / 'report/PROGRESS.md'
    progress_path.write_text(progress_path.read_text().replace('已完成采用审核，准备封存', '已采用并封存'))
    state_path = ROOT / 'report/CONTINUATION_STATE.md'
    state_path.write_text(state_path.read_text().replace('R8_FINAL_REVIEW_COMPLETE', 'R8_RELEASE_SEALED')
                          .replace('采用审核完成；', '已采用并封存；'))
    files = {str(path.relative_to(ROOT)): sha(path) for path in sorted(ROOT.rglob('*'))
             if path.is_file() and not any(part in path.parts for part in ('__pycache__', '.pytest_cache', '.git'))
             and path.suffix != '.pyc' and path.name not in ('RELEASE.json', 'VERIFY.json')}
    save(ROOT / 'RELEASE.json', {**baseline, 'files': files})
    assert all(sha(ROOT / f) == h for f, h in files.items())
    verification = {
        'all_match': True, 'files': len(files), 'release_sha256': sha(ROOT / 'RELEASE.json'),
        'selected_mode': baseline['selected_mode'], 'source_cycle': 5,
        'formal_strict_planning_records': 672, 'entry_smoke_strict_records': 2,
        'original_copy_manifests': 96, 'cleanup_batches': 97, 'tests': 423,
        'goal_remains_active': True,
    }
    save(ROOT / 'VERIFY.json', verification)
    print(json.dumps(verification, ensure_ascii=False))


if __name__ == '__main__':
    main()
