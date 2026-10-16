"""Seal R5 with parent AS retained; preserve C's successful and failed gates."""
from pathlib import Path
from compare_v9_review import load, save
from compare_v9_r5_all import sha
from compare_v8_r6 import verify_cleanup

ROOT = Path(__file__).resolve().parents[1]

def main():
    assert not (ROOT / 'RELEASE.json').exists(), 'Completed release is immutable'
    baseline = load(ROOT / 'BASELINE.json')
    review = load(ROOT / 'report/R5_ALL_COMPARISON.json')
    assert baseline['r5_mode'] == 'control' and baseline['selected_mode'] == 'AS'
    assert baseline['v9_mode'] == 'A' and baseline['review_mode'] == 'S'
    assert baseline['r2_mode'] == baseline['r3_mode'] == 'control'
    assert not review['candidate_C_eligible_before_default_entry_check']
    assert not review['regression_predeclared_gates']
    assert review['default_entry_verified'] and review['selected_r5_mode'] == 'control'
    assert all(review['fixed']['predeclared_fixed_gates'].values())
    assert '280 passed' in review['verification']['tests']
    planning = load(ROOT / 'report/PLANNER_SOURCE_LOCK.json')['planning_source_files']
    assert all(sha(ROOT / f) == h for f, h in planning.items())
    assert len(planning) == 109
    snapshots = []
    for manifest in sorted((ROOT / 'results').rglob('source_snapshot/hashes.json')):
        hashes = load(manifest)
        for source, expected in hashes.items():
            relative = str(Path(source).relative_to(ROOT))
            saved = manifest.parent / relative.replace('/', '_')
            assert sha(saved) == expected, (manifest, source)
        snapshots.append({'manifest': str(manifest), 'copied_files_verified': len(hashes)})
    cleanup = [verify_cleanup(Path(str(p)[:-len('.supervisor.json')]))
               for p in sorted((ROOT / 'results').rglob('*.supervisor.json'))]
    assert len(cleanup) == baseline['cleanup_batches'] == 72
    selection = load(ROOT / 'report/SELECTED_DATA.json')
    assert selection['formal_root'] == review['results_root'] == baseline['formal_data']
    evidence = {'all_original_snapshot_copies_match_saved_hashes': True,
                'snapshot_manifests': snapshots, 'snapshot_count': len(snapshots),
                'planning_files_verified': len(planning), 'cleanup_batches': len(cleanup),
                'selected_r5_mode': 'control', 'C_not_adopted': True}
    save(ROOT / 'report/RELEASE_EVIDENCE.json', evidence)
    # The calling task's state document is archived with the finished release;
    # future work must use another directory and leave these files untouched.
    state = ROOT / 'report/CONTINUATION_STATE.md'
    state.write_text('R5_RELEASE_SEALED: default AS/control, candidate C rejected. '
                     'This completed workspace and records must not be edited. '
                     'The overall goal remains active; continue in a new workspace.\n\n' + state.read_text())
    files = {str(p.relative_to(ROOT)): sha(p) for p in sorted(ROOT.rglob('*'))
             if p.is_file() and not any(x in p.parts for x in ('__pycache__', '.pytest_cache', '.git'))
             and p.suffix != '.pyc' and p.name not in ('RELEASE.json', 'VERIFY.json')}
    save(ROOT / 'RELEASE.json', {**baseline, 'files': files})
    assert all(sha(ROOT / f) == h for f, h in files.items())
    result = {'files': len(files), 'all_match': True,
              'release_sha256': sha(ROOT / 'RELEASE.json'),
              'selected_r5_mode': 'control', 'C_not_adopted': True,
              'snapshot_copies_verified': len(snapshots), 'cleanup_batches': len(cleanup)}
    save(ROOT / 'VERIFY.json', result)
    print(result)

if __name__ == '__main__': main()
