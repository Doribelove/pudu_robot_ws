"""Independent source5 original fixed raw distributions, true SimSun."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,cbook
from compare_v9 import read,parts
from compare_v9_review import load,save,sha
from plot_v9_r9_source2 import setup,FONT,MAPS
ROOT=Path(__file__).resolve().parents[1]
MODES=('control','PLMX','PLMUX')

def plot(root, review_path):
    setup()
    fig, axes = plt.subplots(2, 3, figsize=(20, 9))
    raw = {'source_cycle': 5, 'font': FONT, 'font_name': 'SimSun', 'linear_axes': True,
        'same_query_repeats_are_related': True, 'review_sha256': sha(review_path),
        'box': 'Q1 to Q3; median; whiskers furthest original values within1.5IQR; outliers displayed', 'groups': []}
    for rr, (mid, label) in enumerate(MAPS):
        groups = {mode: parts(read(root/'fixed'/mode/mid), 'fixed') for mode in MODES}
        for cc, (split, metric, divisor, unit, title) in enumerate([
            ('first', 'wall_ms', 1000, 's', '首次端到端耗时'),
            ('repeat', 'wall_ms', 1000, 's', '重复请求端到端耗时'),
            ('all', 'rss_peak_bytes', 2**30, 'GiB', '整套规划进程树峰值RSS')]):
            values = [[float(r[metric])/divisor for r in groups[mode][split]] for mode in MODES]
            ax = axes[rr, cc]
            ax.boxplot(values, labels=[f'{mode}\nn={len(v)}' for mode, v in zip(MODES, values)], whis=1.5, showfliers=True)
            ax.set_title(label+'\n'+title+' / '+unit); ax.grid(axis='y', alpha=.25)
            for mode, values in zip(MODES, values):
                stats = cbook.boxplot_stats(values, whis=1.5)[0]
                raw['groups'].append({'map_id': mid, 'mode': mode, 'split': split, 'unit': unit,
                    'n': len(values), 'raw': values,
                    'quartiles': [float(stats['q1']), float(stats['med']), float(stats['q3'])],
                    'whiskers': [float(stats['whislo']), float(stats['whishi'])], 'outliers': stats['fliers'].tolist()})
    fig.suptitle('V9-r9 source5：独立反序确认，原固定10组×3', fontsize=20)
    fig.text(.5, .03, '箱体Q1至Q3；中线中位数；须为1.5×IQR内最远原始值；点为离群点；线性坐标。\n'
        '首次n=10，重复n=20来自同一查询；整树RSS n=30包含Python与ROS后代20ms采样峰值；严格结果另表。', ha='center')
    fig.tight_layout(rect=(0, .10, 1, .93)); fig.savefig(ROOT/'report/confirmation_source5.png', dpi=160); plt.close(fig)
    save(ROOT/'report/CONFIRMATION_SOURCE5_BOXPLOT_DATA.json', raw)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--results-root',type=Path,required=True);args=parser.parse_args()
    path=ROOT/'report/SOURCE5_CONFIRMATION_FIXED_REVIEW.json';review=load(path)
    assert review['source_cycle']==5 and review['strict_total']==180 and review['independent_reversed_confirmation']
    assert not review['adopted'] and not review['profiler']
    assert Path(review['results_root']).resolve()==args.results_root.resolve()
    assert not (ROOT/'report/confirmation_source5.png').exists()
    plot(args.results_root.resolve(),path)
    print('SOURCE5_CONFIRM trueSimSun original raw18groups first10 repeat20 RSS30 Q1Q3/1.5IQR/outliers')
