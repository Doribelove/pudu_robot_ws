"""Original per-request source4 distributions, explicit true SimSun."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, cbook
from compare_v9 import read, parts
from compare_v9_review import load, save, sha
from plot_v9_r9_source2 import setup, FONT, MAPS

ROOT = Path(__file__).resolve().parents[1]
MODES = ('control', 'PL', 'PLM', 'PLU', 'PLMU')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-root', type=Path, default=ROOT / 'results/formal_source4')
    parser.add_argument('--review-path', type=Path, default=ROOT / 'report/SOURCE4_FIXED_REVIEW.json')
    parser.add_argument('--modes', nargs='+', default=MODES, choices=MODES)
    parser.add_argument('--output-stem', default='fixed_source4')
    parser.add_argument('--title', default='V9-r9 source4：原固定10组×3，五模式同批对照')
    args = parser.parse_args()
    modes = tuple(args.modes)
    assert len(set(modes)) == len(modes)
    review = load(args.review_path)
    assert review['strict_total'] == 60 * len(modes) and not review['profiler'] and not review['adopted']
    setup()
    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    evidence = {'source_cycle': 4, 'font': FONT, 'font_name': 'SimSun', 'linear_axes': True,
                'formal_review_sha256': sha(args.review_path),
                'raw_results_root': str(args.results_root.resolve()),
                'box': 'Q1 to Q3, median line, furthest raw value within1.5IQR whiskers, outliers',
                'related_repeats_of_each_original_query': True, 'groups': []}
    for row, (mid, caption) in enumerate(MAPS):
        data = {}
        for mode in modes:
            directory = args.results_root / 'fixed' / mode / mid
            records = read(directory)
            assert len(records) == 30 and all(record['success'] for record in records)
            data[mode] = parts(records, 'fixed')
        for col, (split, metric, unit, title, n) in enumerate([
                ('first', 'wall_ms', 's', '首次端到端耗时', 10),
                ('repeat', 'wall_ms', 's', '重复请求端到端耗时', 20),
                ('all', 'rss_peak_bytes', 'GiB', '整套进程树RSS峰值', 30)]):
            values = [[float(record[metric]) / (1000 if unit == 's' else 2**30)
                       for record in data[mode][split]] for mode in modes]
            assert all(len(raw) == n for raw in values)
            ax = axes[row, col]
            ax.boxplot(values, labels=[f'{mode}\nn={n}' for mode in modes], whis=1.5, showfliers=True)
            ax.tick_params(axis='x', labelsize=10)
            ax.set_title(f'{caption}\n{title} / {unit}')
            ax.grid(axis='y', alpha=.25)
            for mode, raw in zip(modes, values):
                stat = cbook.boxplot_stats(raw, whis=1.5)[0]
                evidence['groups'].append({'map_id': mid, 'mode': mode, 'split': split,
                    'unit': unit, 'n': n, 'raw': raw,
                    'quartiles': [float(stat['q1']), float(stat['med']), float(stat['q3'])],
                    'whiskers': [float(stat['whislo']), float(stat['whishi'])],
                    'outliers': stat['fliers'].tolist()})
    fig.suptitle(args.title, fontsize=20)
    fig.text(.5, .025, '箱体Q1至Q3；中线中位数；须为1.5×IQR内最远原始值；点为离群点；线性坐标。\n重复来自同一查询，相关性保留。RSS含Python与ROS后代20ms采样峰值；严格成功次数另表。', ha='center')
    fig.tight_layout(rect=(0, .085, 1, .94))
    fig.savefig(ROOT / 'report' / f'{args.output_stem}.png', dpi=160)
    plt.close(fig)
    save(ROOT / 'report' / f'{args.output_stem.upper()}_BOXPLOT_DATA.json', evidence)
    print('SimSun raw', len(evidence['groups']), 'groups source4 n10/20/30; fixed', review['strict_total'], 'formal only')


if __name__ == '__main__':
    main()
