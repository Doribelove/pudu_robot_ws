"""Independent cold processes: raw points, exact current Native/source proof."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager as fm
import numpy as np
from compare_v9 import read, summary, pair
from compare_v9_r8 import exact_compare
from compare_v9_review import load, save, sha
from review_v9_r8_stage3 import source_review
from analyze_v8 import MAPS, stats

ROOT = Path(__file__).resolve().parents[1]
FONT = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc')
MODES = ('control', 'OGR')


def main():
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True)
    p.add_argument('--fixed-root', type=Path, required=True); a = p.parse_args()
    out = {'source_cycle': 4, 'candidate': 'OGR', 'adopted': False, 'maps': {}}
    for mid, label in MAPS:
        arms = {}; parities = []
        for mode in MODES:
            raw = []
            for rep in range(1, 4):
                d = a.results_root/'cold'/f'r{rep}'/mode/mid
                rows = read(d); assert len(rows) == 1 and summary(rows)['success'] == 1
                q = rows[0]; protocol = load(d/'protocol.json')
                assert protocol['workload'] == 'fixed' and protocol['diagnostic_subset'] and not protocol['profiler']
                assert q['query_id'] == ('Q01' if mid == 'mengjia' else 'A2B-01') and q['repeat'] == 1
                evidence = source_review(d, mode, cycle=4)
                pair(a.fixed_root/'fixed'/mode/mid, d, strict=True)
                raw.append({'round': rep, 'cold_first_response_s': q['cold_first_response_ms']/1000,
                            'cold_tree_rss_peak_GiB': q['cold_tree_rss_peak_bytes']/2**30,
                            'first_query_wall_s': q['wall_ms']/1000,
                            'startup_wall_s': load(d/'session_startup.json')['wall_ms']/1000,
                            'runs_sha256': sha(d/'runs.jsonl'), 'source_native_cleanup': evidence})
            arms[mode] = {'raw': raw, 'metrics': {k: stats([r[k] for r in raw]) for k in
                ('cold_first_response_s', 'cold_tree_rss_peak_GiB', 'first_query_wall_s', 'startup_wall_s')}}
        for rep in range(1, 4):
            ds = [a.results_root/'cold'/f'r{rep}'/mode/mid for mode in MODES]
            parities.append(exact_compare(*ds))
        changes = {k: 100*(arms['OGR']['metrics'][k]['median']/arms['control']['metrics'][k]['median']-1)
                   for k in ('cold_first_response_s', 'cold_tree_rss_peak_GiB', 'first_query_wall_s', 'startup_wall_s')}
        out['maps'][mid] = {'label': label, 'arms': arms, 'parity': parities,
                           'n_independent_processes_per_arm': 3, 'each_matches_fixed_first': True,
                           'median_change_pct': changes}
    out['strict_total'] = 12
    out['both_cold_first_medians_improve_at_least3'] = all(
        x['median_change_pct']['cold_first_response_s'] <= -3 for x in out['maps'].values())
    fm.fontManager.addfont(str(FONT)); assert fm.FontProperties(fname=str(FONT)).get_name() == 'SimSun'
    plt.rcParams.update({'font.family': 'SimSun', 'font.size': 11, 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for rr, (mid, label) in enumerate(MAPS):
        for cc, (field, title) in enumerate((('cold_first_response_s', '准备至首次响应 / s'),
                ('cold_tree_rss_peak_GiB', '冷启动进程树峰值RSS / GiB'), ('startup_wall_s', 'ROS服务启动 / s'))):
            ax = axes[rr, cc]
            for i, mode in enumerate(MODES):
                values = [r[field] for r in out['maps'][mid]['arms'][mode]['raw']]
                ax.scatter(i+np.linspace(-.07, .07, 3), values, s=45)
            ax.set_xticks([0, 1], ['control n=3', 'OGR n=3']); ax.set_title(label+'\n'+title); ax.grid(axis='y', alpha=.25)
    fig.suptitle('V9-r8 OGR：三个独立冷进程的逐次原始记录', fontsize=17, y=.965)
    fig.text(.5, .035, '每图每组n=3独立进程，样本不足以支持分布或尾部结论，显示逐点值；线性坐标。\n'
             '冷首条包括地图/审计准备、源码导出、ROS启动及首次查询；排除模块导入和结果导出。RSS含Python与ROS后代，20 ms采样。',
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .12, 1, .93)); fig.savefig(ROOT/'report/cold_source4.png', dpi=160); plt.close(fig)
    save(ROOT/'report/SOURCE4_COLD_REVIEW.json', out)
    print('COLD_STRICT', 12, 'SOURCE_NATIVE_OUTPUT_EXACT_PASS')
    for mid, x in out['maps'].items(): print(mid, x['median_change_pct'])


if __name__ == '__main__': main()
