"""Compare real r5 control calls with independent r6 copy/transport experiments."""
import json
from pathlib import Path
import psutil
from compare_v8_r3 import arm_summary, common_protocol, libraries, read, sha, audit_ast, metricset
from analyze_v8 import measure, stats

ROOT = Path(__file__).resolve().parents[1]
PARENT = Path('/home/robot/workspaces/3yd_v8_r5_20261008')
MAPS = [('mengjia', '万达3w地库'), ('arena_large_250x300_2d_005', 'arena_large'),
        ('3yd_synth_multiloop_a_v1', '合成地库A'), ('arena_medium_200x200_2d_005', 'arena_medium')]

def load(p):
    return json.loads(p.read_text())

def transport(d):
    root = d/'smac/lifecycle_evidence'
    reads = load(root/'readbacks.json')
    tiles = load(root/'publication_tiles.json')
    good = [v for v in reads if v.get('success')]
    deltas = [v for v in good if v.get('r6_delta_transport')]
    assert all(v.get('r6_complete_snapshot_verified') for v in deltas)
    def pub(items):
        return {'messages': len(items), 'payload_bytes': sum(v['bytes'] for v in items),
                'pacing_wait_s': sum(v.get('pacing_wait_s', 0) for v in items)}
    return {'scope': 'whole batch including startup; readback payload excludes DDS framing',
            'readback': {'requests': len(reads), 'success': len(good),
                'payload_bytes': sum(v['bytes'] for v in good),
                'elapsed_s': sum(v['elapsed_s'] for v in reads),
                'delta_requests': len(deltas),
                'delta_patches': sum(not v['r6_readback_full'] for v in deltas),
                'delta_full_resyncs': sum(v['r6_readback_full'] for v in deltas),
                'empty_delta_patches': sum(not v['r6_readback_full'] and not v['r6_readback_patch_count'] for v in deltas),
                'legacy_full_reads': len(good)-len(deltas),
                'errors': [v for v in reads if not v.get('success')]},
            'publication': pub(tiles),
            'query_publication': pub([v for v in tiles if v.get('query')]),
            'startup_publication': pub([v for v in tiles if not v.get('query')])}

def compare(a, b):
    old = {(r['query_id'], r['repeat']): r for r in read(a)}
    pairs = []
    for row in read(b):
        key = row['query_id'], row['repeat']; prev = old[key]; tag = f'{key[0]}_{key[1]}'
        assert not prev['success'] or row['success'], (b, tag, 'success regression')
        p = {'query': tag, 'before': prev['success'], 'after': row['success']}
        for f in ('l1_output.json', 'l2_reference_xy.npy', 'l2_soft_target.json', 'path.npy'):
            x, y = a/'cases'/tag/f, b/'cases'/tag/f
            p[f] = sha(x) == sha(y) if x.exists() and y.exists() else None
        assert all(p[f] for f in ('l1_output.json', 'l2_reference_xy.npy', 'l2_soft_target.json')), (b, p)
        if prev['success']:
            assert p['path.npy'], (b, tag, 'final path changed')
        for f in ('actual_cost_hash', 'path_cost', 'dstar_expanded', 'corridor_cells', 'state_memory_bytes'):
            assert prev.get('l2', {}).get(f) == row.get('l2', {}).get(f), (b, tag, f)
        pairs.append(p)
    return pairs

def verify_native(a, b, mode):
    left = {Path(p).name: h for p, h in libraries(a).items()}
    right = {Path(p).name: h for p, h in libraries(b).items()}
    assert left.keys() == right.keys()
    for name, value in left.items():
        if name == 'libnav2_costmap_2d_core.so' and 'B' in mode:
            assert right[name] == load(ROOT/'native/DELTA_BUILD.json')['binary_sha256']
        else:
            assert right[name] == value, (b, name)
    return right

def verify_cleanup(d):
    p = d.with_name(d.name+'.supervisor.json'); j = load(p)
    assert j['state'] == 'complete' and j['cleanup']['complete'], p
    for v in j['processes'].values():
        try:
            live = psutil.Process(v['pid'])
            assert abs(live.create_time()-v['created']) >= .02, (p, v, 'owned process remains')
        except psutil.NoSuchProcess:
            pass
    for name, identity in j['resources'].items():
        p2 = Path('/dev/shm')/name
        if p2.exists():
            s = p2.stat()
            assert [s.st_dev, s.st_ino, s.st_uid] != identity, (p, name, 'owned SHM remains')
    return str(p)

def main():
    result = {'version': '3YD-V8-r6', 'comparison': 'new r5 control; CPU12 serial, historical batch order not randomized', 'maps': {}}
    datasets = {}; journals = []; snapshots = []
    for i, (mid, label) in enumerate(MAPS):
        dirs = {'r5': ROOT/'results/control'/mid,
                'ABC': ROOT/('results/formal' if i < 2 else 'results/additional')/mid}
        if i < 2:
            dirs.update(A=ROOT/'results/ablation/A'/mid, AB=ROOT/'results/ablation/AB'/mid)
        arms = {}; pairs = {}; protocols = {}; native = {}
        for arm, d in dirs.items():
            arms[arm] = arm_summary(d)
            assert arms[arm]['n'] == (30 if i < 2 and arm in ('r5', 'ABC') else 10)
            arms[arm]['transport'] = transport(d)
            arms[arm]['startup_s'] = load(d/'session_startup.json')['wall_ms']/1000
            if arm != 'r5':
                pairs[arm] = compare(dirs['r5'], d)
                protocols[arm] = common_protocol(dirs['r5'], d)
                native[arm] = verify_native(dirs['r5'], d, arm)
            hashes = load(d/'source_snapshot/hashes.json')
            sources = [(p, h) for p, h in hashes.items() if '/src/' in p]
            assert all(sha(Path(p)) == h for p, h in sources), (d, 'source changed after test')
            snapshots.append({'path': str(d), 'source_files_verified': len(sources)})
            journals.append(verify_cleanup(d))
        result['maps'][mid] = {'label': label, 'arms': arms, 'parity': pairs, 'protocols': protocols, 'native_libraries': native}
        datasets[mid] = {arm: read(d) for arm, d in dirs.items()}
    lock = load(ROOT/'L1_LOCK.json')
    for kind in ('source_files', 'input_files', 'pixel_input_files'):
        assert all(sha(ROOT/f) == h for f, h in lock.get(kind, {}).items()), kind
    for f in ('src/three_yd_v4/controller.py', 'src/three_yd_v4/r5_controller.py', 'src/three_yd_v5/controller.py'):
        assert audit_ast(ROOT/f) == audit_ast(PARENT/f), f
    parent = load(PARENT/'RELEASE.json')
    assert all(sha(PARENT/f) == h for f, h in parent['files'].items()), 'parent release changed'
    for f in ('native/cold_dstar.cpp', 'native/libcold_dstar.so'):
        assert sha(ROOT/f) == sha(PARENT/f)
    result['verification'] = {'L1_and_inputs_unchanged': True, 'strict_audits_unchanged': True,
        'native_L2_and_Smac_and_layers_unchanged': True, 'only_costmap_readback_core_changed': True,
        'parent_files_unchanged': len(parent['files']), 'supervisors_and_resources_clean': journals,
        'source_snapshots': snapshots, 'test_count': 165}
    (ROOT/'report/COMPARISON.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    render(datasets)
    print(json.dumps({m: {a: v['success'] for a, v in x['arms'].items()} for m, x in result['maps'].items()}))

def render(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font = '/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
    font_manager.fontManager.addfont(font)
    assert font_manager.FontProperties(fname=font).get_name() == 'SimSun'
    plt.rcParams.update({'font.family': 'SimSun', 'axes.unicode_minus': False, 'font.size': 11})
    panels = [('e2e_s', '端到端 / s'), ('map_sync_s', '地图更新与确认 / s'), ('tree_rss_gib', '进程树峰值RSS / GiB')]
    for filename, selected, first in [('r5_r6_boxplots.png', MAPS, False), ('ablation_first_boxplots.png', MAPS[:2], True)]:
        fig, axes = plt.subplots(len(selected), 3, figsize=(16, 4.4*len(selected)))
        for line, (mid, label) in enumerate(selected):
            arms = ['r5', 'A', 'AB', 'ABC'] if first else ['r5', 'ABC']
            for ax, (metric, title) in zip(axes[line], panels):
                groups = [[measure(r, metric) for r in data[mid][a] if not first or r['repeat'] == 1] for a in arms]
                names = [('r6 '+a if a != 'r5' else 'r5 对照')+'\nn='+str(len(v)) for a, v in zip(arms, groups)]
                ax.boxplot(groups, labels=names, whis=1.5, showfliers=True)
                ax.set_title(label+'：'+title, fontweight='bold'); ax.grid(axis='y', alpha=.2)
        fig.suptitle('V8-r6 分步对照：各图首次10次' if first else 'V8-r6 四图对照：查询耗时、地图同步与内存', fontsize=18, fontweight='bold')
        fig.text(.5, .012, '箱体Q1～Q3；中线中位数；须为1.5 IQR内最远点；圆点为离群值；线性坐标。\n失败查询计入。计时排除一次性加载、启动和导出；各层包含关系不能相加。', ha='center')
        fig.tight_layout(rect=[0, .055 if first else .035, 1, .94 if first else .96])
        fig.savefig(ROOT/'report'/filename, dpi=150); plt.close(fig)

if __name__ == '__main__':
    main()
