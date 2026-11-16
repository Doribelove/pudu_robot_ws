"""Publication plots: actual map area order, absolute and length-normalized totals."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def main():
    data = json.loads((ROOT/'report/REGRESSION_RESULTS.json').read_text())
    rows = data['records']
    maps = sorted({r['map_id'] for r in rows}, key=lambda mid: next(r['area_m2'] for r in rows if r['map_id'] == mid))
    groups = [[r for r in rows if r['map_id'] == mid] for mid in maps]
    labels = [f"{g[0]['map_name']}\n{g[0]['area_m2']:,.0f} ㎡" for g in groups]
    plt.rcParams.update({'font.sans-serif': ['Noto Sans CJK SC', 'WenQuanYi Zen Hei', 'DejaVu Sans'],
                         'axes.unicode_minus': False, 'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'figure.facecolor': '#ffffff'})
    out = ROOT/'report/figures'; out.mkdir(exist_ok=True)
    for name, keys, titles, footer in [
        ('p6_by_map_absolute', ['wall_s','CPU_s','RSS_MiB'],
         ['首次导航完整请求（秒）','规划进程树 CPU（秒）','规划进程树峰值 RSS（MiB）'],
         '各图 10 个原配套任务 × 3 轮；失败与恢复保留；地图加载及服务启动不计入。'),
        ('p6_by_map_normalized', ['wall_s_per_reference_m','CPU_s_per_reference_m','RSS_MiB_per_reference_m'],
         ['首次导航 / 固定 L1 长度（秒/米）','CPU / 固定 L1 长度（秒/米）','峰值 RSS / 固定 L1 长度（MiB/米）'],
         '分母为同图同 Q 的 B 首轮固定 L1 长度；绝对总量另图保留。RSS/米含基础驻留。')]:
        fig, axes = plt.subplots(3, 1, figsize=(14.5, 11), sharex=True)
        for ax, key, title in zip(axes, keys, titles):
            values = [[r[key] for r in g] for g in groups]
            b = ax.boxplot(values, patch_artist=True, widths=.55, showfliers=True,
                           medianprops={'color':'#14324b', 'linewidth':1.7},
                           flierprops={'marker':'.','markersize':4,'markerfacecolor':'#bb6746','markeredgecolor':'#bb6746'})
            for patch in b['boxes']:
                patch.set_facecolor('#83c5c3'); patch.set_edgecolor('#417c85')
            ax.set_ylabel(title); ax.grid(axis='y', alpha=.22); ax.set_ylim(bottom=0)
        axes[-1].set_xticks(range(1,len(labels)+1),labels,fontsize=9)
        fig.suptitle('P6 · 十图首次导航分布（按地图矩形覆盖面积升序）', fontsize=17, y=.985)
        fig.text(.02,.014,footer+'  箱体 Q1–Q3，中线中位数，须 1.5IQR，离群点保留。',fontsize=10)
        fig.tight_layout(rect=(0,.045,1,.965))
        fig.savefig(out/(name+'.png'),dpi=170); fig.savefig(out/(name+'.pdf')); plt.close(fig)
    fig,ax=plt.subplots(figsize=(14.5,4.7))
    exact=[sum(r['exact'] for r in g) for g in groups]
    tolerant=[sum(r['tolerant'] for r in g) for g in groups]
    failure=[sum(not r['success'] for r in g) for g in groups]
    x=np.arange(len(groups))
    ax.bar(x,exact,color='#3d928f',label='精确端点成功')
    ax.bar(x,tolerant,bottom=exact,color='#e4ae50',label='失败后容差成功')
    ax.bar(x,failure,bottom=np.array(exact)+tolerant,color='#c85c50',label='失败')
    ax.set_xticks(x,labels,fontsize=9);ax.set_ylim(0,33);ax.set_ylabel('请求数 / 每图 30 次')
    ax.grid(axis='y',alpha=.2);ax.legend(ncol=3,loc='upper right')
    ax.set_title('P6 · 各地图成功与失败构成',fontsize=16)
    fig.tight_layout();fig.savefig(out/'p6_outcomes.png',dpi=170);fig.savefig(out/'p6_outcomes.pdf');plt.close(fig)
    print(out)


if __name__=='__main__': main()
