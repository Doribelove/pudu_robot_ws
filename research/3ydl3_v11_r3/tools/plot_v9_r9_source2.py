"""True SimSun; fixed raw distributions and explicitly correlated diagnostic points."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager as fm, cbook
from compare_v9 import read, parts
from compare_v9_review import save

ROOT = Path(__file__).resolve().parents[1]
FONT = '/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
MAPS = [('mengjia', '万达3w地库'), ('arena_large_250x300_2d_005', 'arena_large（无语义）')]


def setup():
    fm.fontManager.addfont(FONT)
    assert fm.FontProperties(fname=FONT).get_name() == 'SimSun'
    plt.rcParams.update({'font.family': 'SimSun', 'font.size': 12, 'axes.unicode_minus': False})


def fixed(results_root=None, output_stem='fixed_source2', title=None):
    results_root = Path(results_root) if results_root is not None else ROOT / 'results/formal_source2/fixed'
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    metadata = {'source_cycle': 2, 'font': FONT, 'font_name': 'SimSun', 'linear_axes': True,
                'box': 'Q1 to Q3; median line; whiskers furthest original values within1.5IQR; points are outliers',
                'same_query_repeats_are_related': True, 'results_root': str(results_root.resolve()), 'groups': []}
    for row, (mid, label) in enumerate(MAPS):
        groups = {}
        for mode in ('control', 'P', 'L', 'PL'):
            records = read(results_root / mode / mid)
            assert len(records) == 30
            groups[mode] = parts(records, 'fixed')
        for col, (split, metric, unit, panel_title) in enumerate([
                ('first', 'wall_ms', 's', '首次端到端耗时'),
                ('repeat', 'wall_ms', 's', '重复请求端到端耗时'),
                ('all', 'rss_peak_bytes', 'GiB', '整套进程树RSS峰值')]):
            values = [[float(record[metric]) / (1000 if unit == 's' else 2**30)
                       for record in groups[mode][split]] for mode in ('control', 'P', 'L', 'PL')]
            ax = axes[row, col]
            ax.boxplot(values, labels=[f'{mode} n={len(raw)}' for mode, raw in zip(('对照', 'P', 'L', 'PL'), values)], whis=1.5, showfliers=True)
            ax.set_title(f'{label}\n{panel_title} / {unit}')
            ax.grid(axis='y', alpha=.25)
            for mode, raw in zip(('control', 'P', 'L', 'PL'), values):
                stat = cbook.boxplot_stats(raw, whis=1.5)[0]
                metadata['groups'].append({'map_id': mid, 'mode': mode, 'split': split, 'unit': unit, 'n': len(raw),
                                           'raw': raw, 'quartiles': [float(stat['q1']), float(stat['med']), float(stat['q3'])],
                                           'whiskers': [float(stat['whislo']), float(stat['whishi'])], 'outliers': stat['fliers'].tolist()})
    fig.suptitle(title or 'V9-r9 P／L／PL：原固定10组×3同批对照', fontsize=20)
    fig.text(.5, .025, '箱体Q1至Q3；中线中位数；须为1.5×IQR内最远原始值；点为离群点；线性坐标。\n重复来自同一查询，相关性保留。RSS含Python与ROS后代20ms采样峰值，非独立进程内存。严格结果另表。', ha='center')
    fig.tight_layout(rect=(0, .085, 1, .93))
    fig.savefig(ROOT / 'report' / f'{output_stem}.png', dpi=160)
    plt.close(fig)
    save(ROOT / 'report' / f'{output_stem.upper()}_BOXPLOT_DATA.json', metadata)


def diagnostic():
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    raw = []
    for row, (mid, label) in enumerate(MAPS):
        for col, kind in enumerate(('moving', 'perturbed')):
            ax = axes[row, col]
            for mode, caption in [('control', '对照'), ('P', 'P列存储'), ('L', 'L查表'), ('PL', 'P+L组合')]:
                records = read(ROOT / 'results/diagnostic_source2' / kind / mode / mid)
                records = parts(records, 'moving')['moving']
                assert len(records) == 3
                values = [record['wall_ms'] / 1000 for record in records]
                steps = [record['moving_start']['step'] for record in records]
                assert steps == ['p20', 'p50', 'p80']
                ax.plot([20, 50, 80], values, 'o-', label=caption)
                raw.append({'map_id': mid, 'kind': kind, 'mode': mode, 'n': 3,
                            'steps': steps, 'request_wall_s': values})
            ax.set_title(label + ('\n精确起点诊断' if kind == 'moving' else '\n3cm／2°偏差诊断'))
            ax.set_xticks([20, 50, 80], ['20%', '50%', '80%'])
            ax.set_ylabel('请求端到端耗时 / s')
            ax.grid(alpha=.25)
            ax.legend()
    fig.suptitle('V9-r9 P／L／PL：原Q09／A2B-08诊断子集，逐点保留', fontsize=19)
    fig.text(.5, .03, '每点对应原查询的一个实际变化请求；每模式每图每工作负载n=3，属于同一查询的相关样本。\n线性坐标；不作为完整固定10组、尾部或更多地图性能验收。', ha='center')
    fig.tight_layout(rect=(0, .09, 1, .93))
    fig.savefig(ROOT / 'report/diagnostic_source2.png', dpi=160)
    plt.close(fig)
    save(ROOT / 'report/DIAGNOSTIC_SOURCE2_POINT_DATA.json', {'font': FONT, 'font_name': 'SimSun', 'groups': raw})


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--fixed-results-root', type=Path)
    parser.add_argument('--fixed-output-stem', default='fixed_source2')
    parser.add_argument('--fixed-title')
    parser.add_argument('--fixed-only', action='store_true')
    args = parser.parse_args()
    setup()
    if not args.fixed_only:
        diagnostic()
    fixed(args.fixed_results_root, args.fixed_output_stem, args.fixed_title)
