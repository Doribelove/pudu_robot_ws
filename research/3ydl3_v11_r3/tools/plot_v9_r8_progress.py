"""Full10 progress distributions with contemporary AS, parent and candidate."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager as fm
from compare_v9 import read, parts
from compare_v9_review import load, save, sha
from analyze_v8 import MAPS, measure
from plot_v9_r8_fixed import distribution, FONT

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True)
    p.add_argument('--reference-root', type=Path, required=True); a = p.parse_args()
    reviewed = load(ROOT/'report/SOURCE4_REGRESSION_REVIEW.json')
    reference = load(ROOT/'report/SOURCE4_AS_PROGRESS_REVIEW.json')
    assert reviewed['strict_progress_total'] == 320 and reviewed['event_assertions'] == 44
    assert reference['source_cycle'] == 4 and not reference['historical_timing_used']
    fm.fontManager.addfont(str(FONT)); assert fm.FontProperties(fname=str(FONT)).get_name() == 'SimSun'
    plt.rcParams.update({'font.family': 'SimSun', 'font.size': 11, 'axes.unicode_minus': False})
    modes = ('AS', 'control', 'OGR')
    evidence = {'source_cycle': 4, 'font': str(FONT), 'font_name': 'SimSun', 'linear_axes': True,
                'whiskers': 'furthest original observations within1.5*IQR of Q1/Q3', 'groups': {}}
    for kind in ('moving', 'perturbed'):
        fig, axes = plt.subplots(2, 3, figsize=(15, 8)); evidence['groups'][kind] = {}
        for rr, (mid, label) in enumerate(MAPS):
            dirs = {m: (a.reference_root/kind/m/mid if m == 'AS' else a.results_root/kind/m/mid) for m in modes}
            rows = {m: read(d) for m, d in dirs.items()}
            assert all(len(v) == 40 and all(r['success'] for r in v) for v in rows.values())
            item = {'strict': {m: '40/40' for m in modes}, 'series': {},
                    'runs_sha256': {m: sha(d/'runs.jsonl') for m, d in dirs.items()}}
            for cc, (part, metric, title, n) in enumerate((('moving', 'e2e_s', '变化请求端到端耗时 / s', 30),
                    ('all', 'tree_rss_gib', '全40请求整树RSS峰值 / GiB', 40),
                    ('moving', 'tree_rss_gib', '30条变化请求整树RSS峰值 / GiB', 30))):
                ax = axes[rr, cc]; values = []
                for mode in modes:
                    series = [measure(r, metric) for r in parts(rows[mode], kind)[part]]
                    assert len(series) == n; values.append(series)
                    item['series'].setdefault(part+'/'+metric, {})[mode] = distribution(series)
                box = ax.boxplot(values, labels=[m+f' n={n}' for m in modes], whis=1.5,
                                 patch_artist=True, showfliers=True,
                                 flierprops={'marker': 'o', 'markersize': 4, 'markerfacecolor': 'none'})
                for patch, color in zip(box['boxes'], ('#dac6b2', '#b2c6da', '#a9d6b6')): patch.set_facecolor(color)
                ax.set_title(label+'\n'+title); ax.grid(axis='y', alpha=.25)
            evidence['groups'][kind][mid] = item
        title = '精确变化起点' if kind == 'moving' else '3 cm／2°偏差起点'
        fig.suptitle('V9-r8 OGR：原固定10组'+title+'完整回放', fontsize=17, y=.965)
        fig.text(.5, .035, '箱体Q1至Q3，横线为中位数；须为1.5×IQR内最远原始值；圆点为离群点；线性坐标。\n'
                 '变化请求n=30（每查询p20/p50/p80），同任务相关；全请求n=40含初始10条。RSS为Python与ROS后代20 ms采样峰值，非独立Python内存。',
                 ha='center', fontsize=10)
        fig.tight_layout(rect=(0, .12, 1, .93)); fig.savefig(ROOT/'report'/f'{kind}_full_source4.png', dpi=160); plt.close(fig)
    save(ROOT/'report/FULL_PROGRESS_SOURCE4_BOXPLOT_DATA.json', evidence)
    print('SIMSUN_FULL_PROGRESS_RAW_BOXPLOTS', 480)


if __name__ == '__main__': main()
