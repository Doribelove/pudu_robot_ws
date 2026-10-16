"""Formal raw-record boxplots: first/repeat latency and complete process tree RSS."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager as fm
from compare_v9 import read, parts
from compare_v9_review import load, save, sha
from analyze_v8 import MAPS, measure

ROOT = Path(__file__).resolve().parents[1]
FONT = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc')


def distribution(values):
    values = np.asarray(values, float)
    q1, median, q3 = np.percentile(values, [25, 50, 75]); iqr = q3-q1
    inside = values[(values >= q1-1.5*iqr) & (values <= q3+1.5*iqr)]
    return {'n': len(values), 'raw': values.tolist(), 'q1': q1, 'median': median, 'q3': q3,
            'whisker_low': float(inside.min()), 'whisker_high': float(inside.max()),
            'outliers': values[(values < q1-1.5*iqr) | (values > q3+1.5*iqr)].tolist()}


def main():
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True)
    p.add_argument('--source-cycle', type=int, choices=[3, 4], required=True)
    p.add_argument('--candidate', choices=['OG', 'OGR'], required=True)
    p.add_argument('--reference-root', type=Path); a = p.parse_args()
    review = load(ROOT/'report'/f'SOURCE{a.source_cycle}_FIXED_REVIEW.json')
    assert review['strict_total'] == 120 and not review['diagnostic_only'] and not review['profiler']
    modes = ('AS', 'control', a.candidate) if a.reference_root else ('control', a.candidate)
    suffix = '_with_AS' if a.reference_root else ''
    if a.reference_root:
        reference = load(ROOT/'report/SOURCE4_AS_REFERENCE_REVIEW.json')
        assert a.source_cycle == 4 and reference['fixed_combined_gates_pass']
    fm.fontManager.addfont(str(FONT)); assert fm.FontProperties(fname=str(FONT)).get_name() == 'SimSun'
    plt.rcParams.update({'font.family': 'SimSun', 'font.size': 11, 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    evidence = {'source_cycle': a.source_cycle, 'candidate': a.candidate, 'font': str(FONT),
                'font_name': 'SimSun', 'linear_axes': True, 'quartiles': 'numpy linear percentiles',
                'whiskers': 'furthest actual observations within1.5*IQR of Q1/Q3', 'maps': {}}
    for rr, (mid, label) in enumerate(MAPS):
        dirs = {m: a.results_root/'fixed'/m/mid for m in modes}
        if a.reference_root: dirs['AS'] = a.reference_root/'fixed/AS'/mid
        rows = {m: read(d) for m, d in dirs.items()}
        assert all(len(v) == 30 and all(r['success'] for r in v) for v in rows.values())
        evidence['maps'][mid] = {'strict': {m: '30/30' for m in modes}, 'series': {},
                                'runs_sha256': {m: sha(d/'runs.jsonl') for m, d in dirs.items()}}
        columns = (('first', 'e2e_s', '首次请求耗时 / s', 10),
                   ('repeat', 'e2e_s', '重复请求耗时 / s', 20),
                   ('all', 'tree_rss_gib', '整套规划进程树峰值RSS / GiB', 30))
        for cc, (group, metric, title, n) in enumerate(columns):
            ax = axes[rr, cc]; values = []
            for mode in modes:
                selected = parts(rows[mode], 'fixed')[group]
                series = [measure(r, metric) for r in selected]
                assert len(series) == n
                values.append(series)
                evidence['maps'][mid]['series'].setdefault(group, {})[mode] = distribution(series)
            plot = ax.boxplot(values, labels=[m+f' n={n}' for m in modes], whis=1.5,
                              patch_artist=True, showfliers=True,
                              flierprops={'marker': 'o', 'markersize': 4, 'markerfacecolor': 'none'})
            colors = ('#dac6b2', '#b2c6da', '#a9d6b6') if a.reference_root else ('#b2c6da', '#a9d6b6')
            for box, color in zip(plot['boxes'], colors): box.set_facecolor(color)
            ax.set_title(label+'\n'+title); ax.grid(axis='y', alpha=.25)
    fig.suptitle(f'V9-r8 {a.candidate}：固定10组正式同批对照（source{a.source_cycle}）', fontsize=17, y=.965)
    fig.text(.5, .035, '箱体Q1至Q3，横线为中位数；须为1.5×IQR范围内最远原始值；圆点为离群点；线性坐标。\n'
             '首次每组n=10，重复n=20，RSS n=30；同查询重复相关。RSS为Python与ROS后代的20 ms采样峰值，非Python独立内存。',
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .12, 1, .93))
    fig.savefig(ROOT/'report'/f'fixed_source{a.source_cycle}{suffix}.png', dpi=160); plt.close(fig)
    save(ROOT/'report'/f'FIXED_SOURCE{a.source_cycle}{suffix.upper()}_BOXPLOT_DATA.json', evidence)
    print('SIMSUN_RAW_FIXED_BOXPLOTS', a.source_cycle, 60*len(modes))


if __name__ == '__main__': main()
