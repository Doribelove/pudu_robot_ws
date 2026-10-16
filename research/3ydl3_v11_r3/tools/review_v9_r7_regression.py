"""Complete functional review after fixed performance gate rejection.

Cold measurements cannot cure the failed RSS/P95 gates; no cold claim is made.
Continue the original movement/event audit to certify the connector's scope.
"""
import argparse
from pathlib import Path
from compare_v9_r7_all import regressions, events_review, verify
from compare_v9_review import load, save
from compare_v9 import read, parts
from analyze_v8 import MAPS, measure
ROOT=Path(__file__).resolve().parents[1]


def plots(root):
    import matplotlib;matplotlib.use('Agg')
    from matplotlib import pyplot as plt,font_manager as fm
    font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
    fm.fontManager.addfont(font);assert fm.FontProperties(fname=font).get_name()=='SimSun'
    plt.rcParams.update({'font.family':'SimSun','font.size':11,'axes.unicode_minus':False})
    for kind,title in (('moving','精确变化起点'),('perturbed','3cm/2°偏差起点')):
        fig,axes=plt.subplots(2,3,figsize=(15,8))
        for rr,(mid,label) in enumerate(MAPS):
            for cc,(metric,text) in enumerate((('e2e_s','实际请求端到端 / s'),
                          ('tree_rss_gib','查询进程树峰值RSS / GiB'),
                          ('python_pss_gib','独立Python进程PSS快照 / GiB'))):
                vals=[]
                for arm in ('control','CHNT'):
                    rs=parts(read(root/kind/arm/mid),kind)['moving']
                    assert len(rs)==30;vals.append([measure(r,metric) for r in rs])
                ax=axes[rr,cc];ax.boxplot(vals,labels=['control n=30','CHNT n=30'],whis=1.5,showfliers=True)
                ax.set_title(label+'\n'+text);ax.grid(axis='y',alpha=.25)
        fig.suptitle('V9-r7：'+title,fontsize=17)
        fig.text(.5,.025,'每图每臂10任务×3推进起点，逐次原始值；同一任务内相关；初始10条单独列入表格。\n'
                 '箱体Q1–Q3，中线中位数；须为1.5×IQR内最远值；圆点离群值；线性坐标；RSS含ROS子进程，PSS为Python快照。',ha='center')
        fig.tight_layout(rect=(0,.1,1,.93));fig.savefig(ROOT/'report'/(kind+'_r7.png'),dpi=160);plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('--results-root',type=Path,required=True);a=p.parse_args()
    root=a.results_root.resolve();fixed=load(ROOT/'report/R7_FIXED_COMPARISON.json')
    assert Path(fixed['results_root'])==root
    gates=fixed['predeclared_fixed_gates']
    assert not gates['both_query_tree_RSS_medians_improve_at_least_5_percent'] or not gates['first_and_repeat_P95_regress_at_most_5_percent']
    out={'version':'3YD-V9-r7','source_cycle':2,'results_root':str(root),'fixed':fixed,
         'regression':regressions(root),'events':events_review(root),'verification':verify(root),
         'cold_performed':False,'cold_skip_reason':'fixed RSS/P95 mandatory adoption gates failed; cold first-plan benefit cannot cure those failures',
         'candidate_CHNT_eligible_before_default_entry_check':False,'selected_mode':'AS','selected_r5_mode':'control',
         'selected_r6_mode':'control','selected_r7_mode':'control','default_entry_verified':False,'production_entry_unchanged':True}
    out['regression_predeclared_gates']=all(v['parity']['quality_gate'] and v['parity']['upstream_gate']
                  for maps in out['regression'].values() for v in maps.values())
    plots(root);save(ROOT/'report/R7_ALL_COMPARISON.json',out)
    print('FUNCTIONAL_REGRESSION_GATE',out['regression_predeclared_gates'])
    print('CHNT_NOT_ADOPTED',gates,'cleanup',len(out['verification']['cleanup']))
if __name__=='__main__':main()
