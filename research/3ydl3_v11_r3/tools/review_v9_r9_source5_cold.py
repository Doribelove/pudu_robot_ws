"""Original cold-response scope; n3 independent raw points, no tail claims."""
import argparse
from pathlib import Path
import statistics
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np
from compare_v9 import read,summary,pair
from compare_v9_r9 import compare,exact_compare
from compare_v9_review import load,save,sha
from review_v9_r9_source2 import pose
from review_v9_r9_source5 import features
from review_v9_r9_full_progress import source
from plot_v9_r9_source2 import setup,FONT,MAPS

ROOT=Path(__file__).resolve().parents[1]
MODES=('AS','control','PLMX','PLMUX')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--protocol',type=Path,default=ROOT/'report/SOURCE5_COLD_PROTOCOL.json')
    parser.add_argument('--report-path',type=Path,default=ROOT/'report/SOURCE5_COLD_REVIEW.json')
    args=parser.parse_args();spec=load(args.protocol);root=Path(spec['root'])
    assert spec['source_cycle']==5 and spec['declared_before_cold_native']
    assert tuple(spec['modes'])==MODES and spec['expected_records']==24
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json')==spec['planner_lock_sha256']
    for name,expected in spec['analysis_tools'].items():
        assert sha(ROOT/name)==expected,name
    assert sha(ROOT/'report/SOURCE5_FULL_PROGRESS_REVIEW_LOCK.json')==spec['full_progress_review_lock_sha256']
    parent=Path(load(ROOT/'PARENT.json')['parent_workspace'])
    report={'source_cycle':5,'adopted':False,'profiler':False,'full_fixed10_acceptance':False,
        'cold_protocol_sha256':sha(args.protocol),'historical_timing_pooled':False,
        'n_independent_processes_per_map_mode':3,'no_distribution_or_tail_claim':True,
        'RSS_is_20ms_python_ROS_tree_peak_not_afterquery_PSS':True,'maps':{}}
    identities=set();count=0
    for mid,query in spec['original_queries'].items():
        qfile=Path(query['source']);assert sha(qfile)==query['sha256']
        definition=load(qfile);queries=definition.get('scenarios',definition.get('queries'))
        assert len(queries)==10
        original=next(q for q in queries if q['id']==query['id'])
        arms={};parities={};as_quality={}
        for mode in MODES:
            raw=[]
            for rep in range(1,4):
                directory=root/'cold'/f'r{rep}'/mode/mid
                rows=read(directory);assert len(rows)==1 and summary(rows)['success']==1
                row=rows[0];protocol=load(directory/'protocol.json')
                assert protocol['workload']=='fixed' and protocol['repeats']==1 and protocol['diagnostic_subset'] and not protocol['profiler']
                assert Path(protocol['query_file'])==qfile and protocol['queries_sha256']==query['sha256']
                assert row['query_id']==query['id'] and row['repeat']==1
                assert row['start']==pose(original['start']) and row['goal']==pose(original['goal'])
                proof=source(directory,mode,rows,5)
                observed=features(directory,rows,'control' if mode=='AS' else mode,mid)
                assert not row['fullroute'].get('reused') and not row['task_progress'].get('reused')
                old=(parent/'results/reference_source4/fixed/AS'/mid if mode=='AS' else
                     ROOT/'results/formal_source5/fixed'/mode/mid)
                fixed=pair(old,directory,strict=True);assert len(fixed)==1
                memory=load(directory/'cold_memory.json')
                assert memory['sampling_interval_ms']==20 and not memory['includes_module_imports']
                expected_scope='batch map preparation + auditor/service preparation + source exports + ROS startup + first request; before case exports'
                assert row['cold_first_scope']==memory['scope']==expected_scope
                assert row['cold_tree_rss_peak_bytes']==memory['tree_rss_peak_bytes']>0
                ready=load(directory/'cold_stack_ready.json')['wall_ms'];assert row['cold_first_response_ms']>=ready+row['wall_ms']>0
                journal=load(Path(str(directory)+'.supervisor.json'));assert '--cold-memory' in journal['command']
                identity=(journal['supervisor']['pid'],journal['supervisor']['created']);assert identity not in identities
                identities.add(identity)
                raw.append({'round':rep,'cold_first_response_s':row['cold_first_response_ms']/1000,
                    'cold_tree_rss_peak_GiB':row['cold_tree_rss_peak_bytes']/2**30,
                    'query_tree_rss_peak_GiB':row['rss_peak_bytes']/2**30,
                    'first_query_wall_s':row['wall_ms']/1000,
                    'startup_wall_s':load(directory/'session_startup.json')['wall_ms']/1000,
                    'independent_supervisor_identity':identity,'source_native_cleanup':proof,
                    'actual_features':observed,'same_original_fixed_first_output':fixed,
                    'cold_scope':row['cold_first_scope']})
                count+=1
            metrics={field:{'n':3,'median':statistics.median(x[field] for x in raw),
                'min':min(x[field] for x in raw),'max':max(x[field] for x in raw),
                'raw':[x[field] for x in raw]}
                for field in ('cold_first_response_s','cold_tree_rss_peak_GiB','query_tree_rss_peak_GiB','first_query_wall_s','startup_wall_s')}
            arms[mode]={'raw':raw,'metrics':metrics}
        for rep in range(1,4):
            parities[f'r{rep}']={mode:exact_compare(root/'cold'/f'r{rep}'/'control'/mid,
                root/'cold'/f'r{rep}'/mode/mid) for mode in ('PLMX','PLMUX')}
            as_quality[f'r{rep}']={mode:compare(root/'cold'/f'r{rep}'/'AS'/mid,
                root/'cold'/f'r{rep}'/mode/mid) for mode in ('control','PLMX','PLMUX')}
            assert all(x['quality_gate'] and x['upstream_gate'] for x in as_quality[f'r{rep}'].values())
        changes={baseline:{mode:{field:100*(arms[mode]['metrics'][field]['median']/arms[baseline]['metrics'][field]['median']-1)
                 for field in arms[mode]['metrics']}
                 for mode in ('PLMX','PLMUX')} for baseline in ('control','AS')}
        report['maps'][mid]={'arms':arms,'candidate_control_exact':parities,
            'original_AS_upstream_quality':as_quality,'median_change_pct':changes,
            'no_fixed_repeat_or_progress_gate_replacement':True}
    assert count==len(identities)==24
    report['strict_total']=count
    report['both_cold_first_control_medians_improve_at_least3']={mode:all(
        item['median_change_pct']['control'][mode]['cold_first_response_s']<=-3
        for item in report['maps'].values()) for mode in ('PLMX','PLMUX')}
    report['both_cold_RSS_control_medians_regress_at_most3']={mode:all(
        item['median_change_pct']['control'][mode]['cold_tree_rss_peak_GiB']<=3
        for item in report['maps'].values()) for mode in ('PLMX','PLMUX')}
    report['fixed_reverse_failures_and_full_progress_memory_P95_failures_still_preserved']=True
    assert not args.report_path.exists()
    save(args.report_path,report)
    setup();fig,axes=plt.subplots(2,3,figsize=(18,8))
    point_data={'source_cycle':5,'font':FONT,'font_name':'SimSun','linear_axes':True,
        'n3_independent_processes_no_boxplot_or_tail_claim':True,'review_sha256':sha(args.report_path),'groups':[]}
    for rr,(mid,name) in enumerate(MAPS):
        for cc,(field,caption) in enumerate([('cold_first_response_s','准备至首次响应 / s'),
                ('cold_tree_rss_peak_GiB','冷启动进程树峰值RSS / GiB'),('first_query_wall_s','首条查询耗时 / s')]):
            ax=axes[rr,cc]
            for i,mode in enumerate(MODES):
                values=report['maps'][mid]['arms'][mode]['metrics'][field]['raw']
                ax.scatter(i+np.linspace(-.07,.07,3),values,s=42)
                point_data['groups'].append({'map_id':mid,'mode':mode,'metric':field,'n':3,'raw':values})
            ax.set_xticks(range(4),[f'{mode}\nn=3' for mode in MODES]);ax.set_title(name+'\n'+caption);ax.grid(axis='y',alpha=.25)
    fig.suptitle('V9-r9 source5：每组三个独立冷进程的原始点',fontsize=18)
    fig.text(.5,.035,'每图每模式n=3独立进程，样本不足以支持分布或尾部结论；线性坐标。\n'
        '冷首条包括地图/审计/服务准备、源码导出、ROS启动与首请求，排除模块导入及结果导出；RSS为20ms整树采样峰值。',ha='center')
    fig.tight_layout(rect=(0,.12,1,.93));fig.savefig(ROOT/'report/cold_source5.png',dpi=160);plt.close(fig)
    save(ROOT/'report/COLD_SOURCE5_RAW_POINT_DATA.json',point_data)
    print('COLD24 strict/source/native/originalquery/clockscope/fixedoutput/cleanup checked')
    print('cold_first3',report['both_cold_first_control_medians_improve_at_least3'],
          'coldRSS3',report['both_cold_RSS_control_medians_regress_at_most3'])
    for mid,item in report['maps'].items():print(mid,item['median_change_pct'])


if __name__=='__main__':main()
