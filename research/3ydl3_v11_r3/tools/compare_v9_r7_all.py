"""Full R7 review. Local server hashes are never treated as global readback."""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from compare_v9 import read, tag, pair, parts, summary
from compare_v9_review import load, save
from compare_v9_r7 import compare, mode_evidence
from compare_v8_r3 import audit_ast
from compare_v8_r6 import verify_cleanup
from analyze_v8 import MAPS, stats

ROOT = Path(__file__).resolve().parents[1]
FROZEN = Path('/home/robot/workspaces/3yd_v8_r11_firstcost_20261009')

def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()

def verify(formal):
    out = {}
    lock = load(ROOT / 'L1_LOCK.json')
    for section in ('source_files', 'input_files', 'pixel_input_files'):
        assert all(sha(ROOT / f) == h for f, h in lock[section].items()), section
        out[section] = len(lock[section])
    for f in ('src/three_yd_v4/controller.py', 'src/three_yd_v4/r5_controller.py', 'src/three_yd_v5/controller.py'):
        assert audit_ast(ROOT / f) == audit_ast(FROZEN / f), f
    out['original_strict_audit_AST_unchanged'] = True
    for f in ('src/three_yd_v5/task_progress.py',
              'native/libcold_dstar.so', 'native/cold_dstar.cpp'):
        assert sha(ROOT / f) == sha(FROZEN / f), f
    out['task_progress_L2_native_unchanged'] = True
    from audit_v9_r6_connector import audit_connector
    out['candidate_filter_only_original_connector_audits'] = audit_connector(ROOT, FROZEN)
    parents = (
        (FROZEN, '3e19256a3c09c95cf3f42dc9ce3bc73a5d396580445f12e7bb34fe5cae867bbf'),
        (Path('/home/robot/workspaces/3yd_v9_r0_l3_20261009'), '3e5aee3c83587a166226c0b2d1764aae6024da9a0b0382b86f18dd847d481a76'),
        (Path('/home/robot/workspaces/3yd_v9_r1_audit_20261009'), '23a20dff44bfea224d977f0581b695ed28aeb6786c34afd7c0289a37ed5a9518'),
        (Path('/home/robot/workspaces/3yd_v9_r2_firstplan_20261009'), 'a8bb54df366fb0e75ff8b4aa85f321eb217d8e14e3fc1537b1f25719aa6cdc19'),
        (Path('/home/robot/workspaces/3yd_v9_r3_mapflow_20261009'), '22deaa385268f2671b39a5b5752dc3e0291d2cd56dd9bdb24e1c61c86aa1ecb2'),
        (Path('/home/robot/workspaces/3yd_v9_r4_schedule_20261009'), '1ea849d520209b90b0fe94d879e0af9b9a4e99ea729a82af6cae92526da86191'),
        (Path('/home/robot/workspaces/3yd_v9_r5_fullstorage_20261009'), '713ae7efac1f91370c69ed06a95f05bbd1e88ade04c4230c37581bcdb58d4499'),
        (Path('/home/robot/workspaces/3yd_v9_r6_connection_20261009'), '2fb639196f278cec0d7c96b1914edacfeb240caebc07cd26e55f37fe2776ccaf'),
    )
    for parent, expected in parents:
        assert sha(parent / 'RELEASE.json') == expected, parent
        files = load(parent / 'RELEASE.json')['files']
        assert all(sha(parent / f) == h for f, h in files.items()), parent
        out[parent.name] = {'files_verified': len(files), 'release_sha256': expected}
    archive = Path('/home/robot/workspaces/3yd_frozen_baselines/3YD-V8-r11-AB_20261009/snapshot.tar.gz')
    assert sha(archive) == 'df7efcd136edb48e452e60dd14a40899c360ee30572fa3f0a37bf040b6f42d3c'
    out['frozen_archive_unchanged'] = True
    for manifest in ('PRODUCTION_ENTRY_LOCK.json', 'PLANNER_SOURCE_LOCK.json', 'EXTERNAL_AUDIT_LOCK.json'):
        files = load(ROOT / 'report' / manifest)
        files = files.get('planning_source_files', files)
        assert all(sha(Path(f) if Path(f).is_absolute() else ROOT / f) == h for f, h in files.items()), manifest
        out[manifest] = {'files_verified': len(files)}
    snapshots = []
    # Only this source cycle's formal records are compared with current code.
    # Earlier cycles retain their original source snapshots and raw records.
    planning = load(ROOT / 'report/PLANNER_SOURCE_LOCK.json')['planning_source_files']
    for manifest in sorted(formal.rglob('source_snapshot/hashes.json')):
        files = load(manifest)
        relevant = {f: h for f, h in files.items() if str(Path(f).relative_to(ROOT)) in planning}
        assert all(sha(f) == h for f, h in relevant.items()), manifest
        assert set(str(Path(f).relative_to(ROOT)) for f in relevant) == set(planning), manifest
        snapshots.append({'manifest': str(manifest), 'planning_files': len(relevant)})
    out['formal_source_snapshots'] = snapshots
    out['cleanup'] = [verify_cleanup(Path(str(p)[:-len('.supervisor.json')]))
                      for p in sorted((ROOT / 'results').rglob('*.supervisor.json'))]
    copies=[]
    for manifest in sorted((ROOT/'results').rglob('source_snapshot/hashes.json')):
        files=load(manifest)
        for original, expected in files.items():
            relative=Path(original).relative_to(ROOT)
            copy=manifest.parent/str(relative).replace('/','_')
            assert sha(copy)==expected,(manifest,relative)
        copies.append({'manifest':str(manifest),'copied_files_verified':len(files)})
    out['all_original_snapshot_copies_verified']=copies
    parent=Path('/home/robot/workspaces/3yd_v9_r5_fullstorage_20261009')
    native=[f for f in (parent/'native').rglob('*') if f.is_file()]
    native += [f for f in (parent/'install').rglob('*') if f.is_file() and f.suffix=='.so']
    for f in native:
        relative=f.relative_to(parent)
        assert sha(ROOT/relative)==sha(f),relative
    out['original_native_files_unchanged']=len(native)
    out['tests'] = (ROOT / 'report/tests_final.txt').read_text().splitlines()[-1]
    assert 'passed' in out['tests'] and 'failed' not in out['tests']
    return out

def cold_review(root):
    out = {}
    for mid, label in MAPS:
        arms = {}
        parities = []
        for mode in ('control', 'CHNT'):
            raw = []
            for rep in range(1, 4):
                directory = root / 'cold' / f'r{rep}' / mode / mid
                rows = read(directory)
                assert len(rows) == 1 and summary(rows)['success'] == 1
                mode_evidence(directory, mode)
                q = rows[0]
                raw.append({'round': rep, 'cold_first_response_ms': q['cold_first_response_ms'],
                            'cold_tree_rss_peak_bytes': q['cold_tree_rss_peak_bytes'],
                            'first_query_wall_ms': q['wall_ms'],
                            'startup_wall_ms': load(directory / 'session_startup.json')['wall_ms'],
                            'runs_sha256': sha(directory / 'runs.jsonl')})
                # Same query and native geometry must retain the fixed first output.
                pair(root / 'fixed' / mode / mid, directory, True)
                verify_cleanup(directory)
            arms[mode] = {'raw': raw, 'metrics': {k: stats([r[k] for r in raw])
                         for k in raw[0] if k not in ('round', 'runs_sha256')}}
        for rep in range(1, 4):
            item = compare(root / 'cold' / f'r{rep}' / 'control' / mid,
                           root / 'cold' / f'r{rep}' / 'CHNT' / mid)
            assert item['quality_gate']
            parities.append(item)
        out[mid] = {'label': label, 'arms': arms, 'n_independent_processes_per_arm': 3,
                    'parity': parities, 'each_matches_fixed_first': True}
    return out

def regressions(root):
    out = {}
    for kind in ('moving', 'perturbed'):
        out[kind] = {}
        for mid, label in MAPS:
            directories = {m: root / kind / m / mid for m in ('control', 'CHNT')}
            rows = {m: read(d) for m, d in directories.items()}
            assert all(len(v) == 40 for v in rows.values())
            arms = {m: {k: summary(v) for k, v in parts(r, kind).items()} for m, r in rows.items()}
            assert all(v['initial']['success'] == 10 and v['moving']['success'] == 30 for v in arms.values())
            parity = compare(directories['control'], directories['CHNT'], report_upstream_mismatch=True)
            out[kind][mid] = {'label': label, 'arms': arms, 'parity': parity,
                             'mode_evidence': {m: mode_evidence(d, m) for m, d in directories.items()},
                             'cleanup': {m: verify_cleanup(d) for m, d in directories.items()}}
    return out

def events_review(root):
    out = {}
    for mid, label in MAPS:
        records = {m: load(root / 'events' / m / mid / 'events.json') for m in ('control', 'CHNT')}
        outcome = lambda r: [(x['label'], x.get('success'), x.get('trajectory_available')) for x in r]
        assert outcome(records['control']) == outcome(records['CHNT']), mid
        arms = {}
        for mode, rows in records.items():
            directory = root / 'events' / mode / mid
            s = load(directory / 'summary.json')
            assert s['assertions_passed'] and s['events'] == 11
            assert s['optimization_mode'] == 'AB' and s['v9_mode'] == 'A' and s['review_mode'] == 'S'
            assert s['static_event_replay'] and not s['live_execution']
            for row in rows:
                if row.get('trajectory_available'):
                    assert row['whole_suffix_body_audit']['valid'] and row['original_task_traffic_audit']['valid']
                if row.get('success') and 'constraint_audit' in row:
                    assert row['constraint_audit']['steering_jump_valid'] and row['independent_audit']['valid']
                d = row.get('diagnostics', {})
                if d.get('costmap_update_acknowledged'):
                    assert d['costmap_ack_mismatch_cells'] == 0
                    if d.get('semantic_expected_server_content_hash'):
                        assert d['server_costmap_content_hash'] == d['semantic_expected_server_content_hash']
                        assert d['semantic_exact_stable_observations'] >= 2
            # Events have no protocol file; the actual bootstrap/resize log and
            # complete-task geometry manifest still provide mode evidence.
            event_geometry = load(directory / 'smac/lifecycle_evidence/r5_complete_geometry.json')
            assert bool(event_geometry) == (mode == 'CHNT')
            previous = {}
            for e in event_geometry:
                key = json.dumps(e['task'], sort_keys=True); box = e['bounds']
                assert e['complete_task_scope']
                if key in previous:
                    a = previous[key]
                    assert box[0] <= a[0] and box[1] >= a[1] and box[2] <= a[2] and box[3] >= a[3]
                previous[key] = box
            arms[mode] = {'summary': s, 'outcomes': outcome(rows), 'complete_geometry_events': event_geometry,
                          'mode_evidence': mode_evidence(directory, mode, protocol=False),
                          'cleanup': verify_cleanup(directory)}
        semantic = mid == 'mengjia'
        if semantic:
            indexed = {x['label']: x for x in records['CHNT']}
            assert not indexed['future_add']['success'] and not indexed['future_add']['trajectory_available']
            assert indexed['future_clear']['success'] and indexed['future_clear']['trajectory_available']
        out[mid] = {'label': label, 'arms': arms, 'same_outcomes': True,
                    'path_bytes_may_differ_due_to_native_origin': True}
    return out

def cold_plot(data):
    import matplotlib; matplotlib.use('Agg')
    from matplotlib import pyplot as plt, font_manager as fm
    font = '/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
    fm.fontManager.addfont(font); assert fm.FontProperties(fname=font).get_name() == 'SimSun'
    plt.rcParams.update({'font.family': 'SimSun', 'font.size': 11, 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for rr, (mid, label) in enumerate(MAPS):
        for cc, (field, title, scale) in enumerate((('cold_first_response_ms', '准备至首次响应 / s', 1000),
                                                  ('cold_tree_rss_peak_bytes', '冷进程树峰值RSS / GiB', 2**30),
                                                  ('startup_wall_ms', 'ROS服务启动 / s', 1000))):
            ax = axes[rr, cc]
            for i, mode in enumerate(('control', 'CHNT')):
                vals = [v[field] / scale for v in data[mid]['arms'][mode]['raw']]
                ax.scatter(i + np.linspace(-.07, .07, len(vals)), vals, s=45)
            ax.set_xticks([0, 1], ['control n=3', 'CHNT n=3']); ax.set_title(label + '\n' + title)
            ax.grid(axis='y', alpha=.25)
    fig.suptitle('V9-r7：独立冷进程逐次原始记录', fontsize=17)
    fig.text(.5, .025, '每图每组3次独立启动，显示逐点值；n=3不足以评估分布；线性坐标。\n'
             '冷首条包括地图/审计/源码导出/ROS启动与首条查询，排除模块导入和结果导出；RSS包含Python与ROS子进程，20ms采样。', ha='center')
    fig.tight_layout(rect=(0, .1, 1, .93)); fig.savefig(ROOT / 'report/cold_r7.png', dpi=160); plt.close(fig)

def main():
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True)
    p.add_argument('--phase', choices=('cold', 'all'), default='all'); a = p.parse_args(); root = a.results_root.resolve()
    fixed = load(ROOT / 'report/R7_FIXED_COMPARISON.json')
    assert Path(fixed['results_root']) == root and fixed['phase'] == 'fixed'
    out = {'version': '3YD-V9-r7', 'source_cycle': 2, 'results_root': str(root),
           'fixed': fixed, 'cold': cold_review(root), 'default_entry_verified': False}
    cold_plot(out['cold'])
    out['both_cold_first_medians_improve_at_least_3_percent'] = all(
        v['arms']['CHNT']['metrics']['cold_first_response_ms']['median'] <= .97 * v['arms']['control']['metrics']['cold_first_response_ms']['median']
        for v in out['cold'].values())
    if a.phase == 'all':
        out['regression'] = regressions(root); out['events'] = events_review(root)
        out['verification'] = verify(root)
        gates = fixed['predeclared_fixed_gates']
        benefit = gates['both_first_medians_improve_at_least_3_percent'] or out['both_cold_first_medians_improve_at_least_3_percent']
        remaining = [v for k, v in gates.items() if k != 'both_first_medians_improve_at_least_3_percent']
        out['regression_predeclared_gates'] = all(v['parity']['quality_gate'] and v['parity']['upstream_gate']
                                                for maps in out['regression'].values() for v in maps.values())
        out['candidate_CHNT_eligible_before_default_entry_check'] = bool(benefit and all(remaining) and out['regression_predeclared_gates'])
        out['default_entry_verified'] = False
        out['remaining_step'] = 'real selected-entry check and release review before default selection'
    save(ROOT / 'report' / ('R7_ALL_COMPARISON.json' if a.phase == 'all' else 'R7_COLD_COMPARISON.json'), out)
    print('COLD_BENEFIT', out['both_cold_first_medians_improve_at_least_3_percent'])
    if a.phase == 'all': print('ELIGIBLE_BEFORE_ENTRY', out['candidate_CHNT_eligible_before_default_entry_check'])

if __name__ == '__main__': main()
