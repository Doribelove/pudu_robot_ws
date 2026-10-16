"""True SimSun; only the original related three changed-request raw points."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from plot_v9_r9_source2 import setup, FONT, MAPS
from compare_v9 import read, parts
from compare_v9_review import load, save, sha

ROOT = Path(__file__).resolve().parents[1]


if __name__ == '__main__':
    assert load(ROOT/'report/SOURCE5_DIAGNOSTIC_REVIEW.json')['strict_total'] == 64
    setup()
    modes = ('control', 'PLM', 'PLMX', 'PLMUX')
    root = ROOT/'results/diagnostic_source5'
    data = {'source_cycle': 5, 'font': FONT, 'font_name': 'SimSun', 'linear_axes': True,
        'related_changed_n3_diagnostic_not_full10_or_tail_or_adoption': True,
        'review_sha256': sha(ROOT/'report/SOURCE5_DIAGNOSTIC_REVIEW.json'), 'groups': []}
    fig, axes = plt.subplots(2, 4, figsize=(20, 8))
    for rr, (mid, label) in enumerate(MAPS):
        for cc, (kind, metric, divisor, unit, title) in enumerate([
            ('moving', 'wall_ms', 1000, 's', '精确起点变化请求'),
            ('perturbed', 'wall_ms', 1000, 's', '3cm／2°偏差变化请求'),
            ('moving', 'rss_peak_bytes', 2**30, 'GiB', '精确起点进程树RSS峰值'),
            ('perturbed', 'rss_peak_bytes', 2**30, 'GiB', '偏差起点进程树RSS峰值')]):
            ax = axes[rr, cc]
            for i, mode in enumerate(modes):
                rows = parts(read(root/kind/mode/mid), 'moving')['moving']
                assert len(rows) == 3
                values = [float(row[metric])/divisor for row in rows]
                ax.scatter(i+np.linspace(-.07, .07, 3), values, s=35)
                data['groups'].append({'map_id': mid, 'workload': kind, 'mode': mode,
                    'metric': metric, 'unit': unit, 'n': 3, 'raw': values,
                    'steps': [row['moving_start']['step'] for row in rows]})
            ax.set_xticks(range(4), [f'{mode}\nn=3' for mode in modes])
            ax.set_title(label+'\n'+title+' / '+unit); ax.grid(axis='y', alpha=.25)
    fig.suptitle('V9-r9 source5：原Q09／A2B-08诊断子集逐点记录', fontsize=20)
    fig.text(.5, .03, '每组n=3是同一查询的p20／p50／p80相关变化请求；样本不足以支持分布或尾部结论。\n'
        '仅诊断，不代表完整固定10组；线性坐标；RSS是Python及ROS后代20ms采样进程树峰值，非独立PSS；严格结果另表。', ha='center')
    fig.tight_layout(rect=(0, .11, 1, .93)); fig.savefig(ROOT/'report/diagnostic_source5.png', dpi=160); plt.close(fig)
    save(ROOT/'report/DIAGNOSTIC_SOURCE5_POINT_DATA.json', data)
    print('SOURCE5 SimSun relatedn3 raw diagnostic points saved')
