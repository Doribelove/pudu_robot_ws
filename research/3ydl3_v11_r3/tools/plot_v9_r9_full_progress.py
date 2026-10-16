"""Actual full40 distributions and separate memory subgroups, true SimSun."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, cbook
from compare_v9 import read, parts
from compare_v9_review import load, save, sha
from plot_v9_r9_source2 import setup, FONT, MAPS

ROOT = Path(__file__).resolve().parents[1]
MODES = ('AS', 'control', 'PLM')


def plot(memory_only=False):
    spec = ([('all', 'rss_peak_bytes', 'GiB', '全部请求进程树RSS', 40),
             ('initial', 'rss_peak_bytes', 'GiB', '初始请求进程树RSS', 10),
             ('moving', 'rss_peak_bytes', 'GiB', '变化请求进程树RSS', 30)] if memory_only else
            [('initial', 'wall_ms', 's', '初始请求耗时', 10),
             ('moving', 'wall_ms', 's', '变化请求耗时', 30),
             ('all', 'rss_peak_bytes', 'GiB', '全部请求进程树RSS', 40)])
    fig, axes = plt.subplots(4, 3, figsize=(18, 13))
    metadata = {'source_cycle': 3, 'profiler': False, 'diagnostic_subset': False,
                'font': FONT, 'font_name': 'SimSun', 'linear_axes': True,
                'review_sha256': sha(ROOT / 'report/SOURCE3_FULL_PROGRESS_REVIEW.json'),
                'box': 'Q1 to Q3, median line, furthest raw value within1.5IQR whiskers, outliers',
                'each_query_three_changed_poses_are_related': True, 'groups': []}
    for kind_index, kind in enumerate(('moving', 'perturbed')):
        for map_index, (mid, name) in enumerate(MAPS):
            row = 2*kind_index + map_index
            data = {}
            for mode in MODES:
                records = read(ROOT / 'results/full_progress_source3' / kind / mode / mid)
                assert len(records) == 40 and all(record['success'] for record in records)
                data[mode] = parts(records, 'moving')
            for col, (split, metric, unit, caption, n) in enumerate(spec):
                raw = [[float(record[metric]) / (1000 if unit == 's' else 2**30)
                        for record in data[mode][split]] for mode in MODES]
                assert all(len(values) == n for values in raw)
                ax = axes[row, col]
                ax.boxplot(raw, labels=[f'{mode} n={n}' for mode in MODES], whis=1.5, showfliers=True)
                ax.set_title(name + ('：精确起点' if kind == 'moving' else '：3cm／2°偏差') + f'\n{caption} / {unit}')
                ax.grid(axis='y', alpha=.25)
                for mode, values in zip(MODES, raw):
                    stat = cbook.boxplot_stats(values, whis=1.5)[0]
                    metadata['groups'].append({'kind': kind, 'map_id': mid, 'mode': mode,
                        'split': split, 'unit': unit, 'n': n, 'raw': values,
                        'quartiles': [float(stat['q1']), float(stat['med']), float(stat['q3'])],
                        'whiskers': [float(stat['whislo']), float(stat['whishi'])],
                        'outliers': stat['fliers'].tolist()})
    fig.suptitle('V9-r9 source3：原完整10组推进' + ('，内存子组分别保留' if memory_only else '，原AS同批对照'), fontsize=20)
    fig.text(.5, .023, '箱体Q1至Q3；中线中位数；须为1.5×IQR内最远原始值；点为离群点；线性坐标。\n变化请求每原查询三次，相关性保留；RSS是Python与ROS后代20ms采样峰值，独立进程PSS另列。', ha='center')
    fig.tight_layout(rect=(0, .067, 1, .96))
    stem = 'full_progress_memory_source3' if memory_only else 'full_progress_source3'
    fig.savefig(ROOT / 'report' / (stem + '.png'), dpi=160)
    plt.close(fig)
    save(ROOT / 'report' / (stem.upper() + '_BOXPLOT_DATA.json'), metadata)


if __name__ == '__main__':
    review = load(ROOT / 'report/SOURCE3_FULL_PROGRESS_REVIEW.json')
    assert review['strict_progress_total'] == 480 and not review['profiler'] and not review['adopted']
    setup()
    plot()
    plot(memory_only=True)
    print('true SimSun,72 raw subgroup distributions; no pooled first/changed memory')
