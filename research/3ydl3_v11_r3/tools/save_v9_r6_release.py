"""Seal rejected combined CH performance candidate, preserving default AS."""
from pathlib import Path
from compare_v9_review import load, save
from compare_v9_r6_all import sha
from compare_v8_r6 import verify_cleanup
ROOT=Path(__file__).resolve().parents[1]


def main():
    assert not (ROOT/'RELEASE.json').exists(), 'Completed release is immutable'
    baseline=load(ROOT/'BASELINE.json');review=load(ROOT/'report/R6_ALL_COMPARISON.json')
    assert baseline['selected_mode']=='AS' and baseline['v9_mode']=='A' and baseline['review_mode']=='S'
    assert all(baseline[k]=='control' for k in ('r2_mode','r3_mode','r5_mode','r6_mode'))
    assert not review['candidate_CH_eligible_before_default_entry_check']
    assert not review['fixed']['predeclared_fixed_gates']['both_query_tree_RSS_medians_improve_at_least_5_percent']
    assert not review['fixed']['predeclared_fixed_gates']['first_and_repeat_P95_regress_at_most_5_percent']
    assert review['regression_predeclared_gates']
    assert review['default_entry_verified'] and review['selected_mode']=='AS'
    assert review['selected_r5_mode']==review['selected_r6_mode']=='control'
    assert review['cold_performed'] is False
    assert '286 passed' in review['verification']['tests']
    planning=load(ROOT/'report/PLANNER_SOURCE_LOCK.json')['planning_source_files']
    assert len(planning)==110 and all(sha(ROOT/f)==h for f,h in planning.items())
    snapshots=[]
    for manifest in sorted((ROOT/'results').rglob('source_snapshot/hashes.json')):
        files=load(manifest)
        for source,expected in files.items():
            saved=manifest.parent/str(Path(source).relative_to(ROOT)).replace('/','_')
            assert sha(saved)==expected,(manifest,source)
        snapshots.append({'manifest':str(manifest),'copied_files_verified':len(files)})
    cleanup=[verify_cleanup(Path(str(p)[:-len('.supervisor.json')]))
             for p in sorted((ROOT/'results').rglob('*.supervisor.json'))]
    assert len(cleanup)==baseline['cleanup_batches']==30
    selection=load(ROOT/'report/SELECTED_DATA.json')
    assert selection['formal_root']==review['results_root']==baseline['formal_data']
    save(ROOT/'report/RELEASE_EVIDENCE.json',{'all_original_snapshot_copies_match_saved_hashes':True,
         'snapshot_manifests':snapshots,'snapshot_count':len(snapshots),'planning_files_verified':len(planning),
         'cleanup_batches':len(cleanup),'selected_mode':'AS','CH_not_adopted':True,
         'functional_connector_regression_pass':True,'cold_not_performed_on_ineligible_candidate':True})
    state=ROOT/'report/CONTINUATION_STATE.md'
    state.write_text('R6_RELEASE_SEALED: AS retained; combined CH RSS/P95 gates failed. '
                     'H functional regressions passed. This workspace and raw data must not be edited. '
                     'The overall goal remains active; continue in an independent version.\n\n'+state.read_text())
    files={str(p.relative_to(ROOT)):sha(p) for p in sorted(ROOT.rglob('*')) if p.is_file()
           and not any(v in p.parts for v in ('__pycache__','.pytest_cache','.git'))
           and p.suffix!='.pyc' and p.name not in ('RELEASE.json','VERIFY.json')}
    save(ROOT/'RELEASE.json',{**baseline,'files':files})
    assert all(sha(ROOT/f)==h for f,h in files.items())
    out={'files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json'),
         'selected_mode':'AS','CH_not_adopted':True,'H_functional_regression_pass':True,
         'snapshot_copies_verified':len(snapshots),'cleanup_batches':len(cleanup)}
    save(ROOT/'VERIFY.json',out);print(out)
if __name__=='__main__':main()
