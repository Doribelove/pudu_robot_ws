"""Raw diagnostic points with explicit true SimSun; no inferred distributions."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager as fm
from compare_v9 import read
from analyze_v8 import MAPS, measure

ROOT = Path(__file__).resolve().parents[1]
FONT = Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc')


def main():
    p = argparse.ArgumentParser(); p.add_argument('--results-root', type=Path, required=True)
    p.add_argument('--phase', choices=['diagnostic', 'shared'], required=True)
    p.add_argument('--source-cycle', type=int, choices=[3, 4], default=3)
    p.add_argument('--candidate', choices=['OG', 'OGR'], default='OG'); a = p.parse_args()
    fm.fontManager.addfont(str(FONT)); assert fm.FontProperties(fname=str(FONT)).get_name() == 'SimSun'
    plt.rcParams.update({'font.family': 'SimSun', 'font.size': 11, 'axes.unicode_minus': False})
    groups = ('moving', 'perturbed') if a.phase == 'diagnostic' else ('shared',)
    for kind in groups:
        fig, axes = plt.subplots(2, 3, figsize=(15, 8))
        for rr, (mid, label) in enumerate(MAPS):
            for cc, (metric, title) in enumerate((('e2e_s', '请求端到端耗时 / s'),
                    ('tree_rss_gib', '整套进程树峰值RSS / GiB'), ('python_pss_gib', 'Python独立进程PSS快照 / GiB'))):
                ax = axes[rr, cc]; tags = []
                combinations = [(m, 'A') for m in ('control', a.candidate)] if a.phase == 'diagnostic' else [(m, arm) for m in ('control', a.candidate) for arm in ('A', 'ABP')]
                for i, (mode, arm) in enumerate(combinations):
                    directory = (a.results_root/'diagnostic'/kind/mode/mid if a.phase == 'diagnostic'
                                 else a.results_root/'shared'/mode/arm/mid)
                    rows = read(directory)
                    selected = [r for r in rows if r['moving_start']['kind'] == 'moving'] if a.phase == 'diagnostic' else rows[1:]
                    assert len(selected) == 3 and all(r['success'] for r in selected)
                    values = [measure(r, metric) for r in selected]
                    ax.scatter(i+np.linspace(-.08, .08, len(values)), values, s=45)
                    tags.append((mode if a.phase == 'diagnostic' else mode+'/'+arm)+' n=3')
                ax.set_xticks(range(len(tags)), tags); ax.set_title(label+'\n'+title)
                ax.grid(axis='y', alpha=.25)
        title = {'moving': '精确起点子集', 'perturbed': '3 cm／2°偏差起点子集', 'shared': '原共享任务v2子集'}[kind]
        fig.suptitle('V9-r8 '+a.candidate+'：'+title+f'（source{a.source_cycle}）', fontsize=17, y=.965)
        note = ('Q09／A2B-03，各含p20、p50、p80三条变化请求；初始请求单列；同进程、同任务相关。'
                if a.phase == 'diagnostic' else '每组一条原种子后、三个不同起点与终点的新任务；种子单列；同进程相关。')
        fig.text(.5, .035, note+'\n逐点原始记录；n=3不支持分布、尾部或完整固定10组结论；线性坐标。RSS含Python与ROS后代，20 ms采样峰值；PSS仅Python快照。', ha='center', fontsize=10)
        fig.tight_layout(rect=(0, .12, 1, .93)); fig.savefig(ROOT/'report'/(kind+f'_source{a.source_cycle}.png'), dpi=160); plt.close(fig)
        print('SIMSUN_RAW_POINTS', kind)


if __name__ == '__main__': main()
