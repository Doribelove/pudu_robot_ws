"""Strict original shared32 and the original two-map new-task median benefit."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np
from compare_v9 import read, summary, tag, pair
from compare_v9_review import load, save, sha
from compare_v9_r9 import exact_compare, compare
from review_v9_r9_diagnostic import source_proof
from review_v9_r9_source2 import pose
from review_v9_r9_source5 import features
from plot_v9_r9_source2 import setup, FONT, MAPS

ROOT = Path(__file__).resolve().parents[1]
MODES = ('control', 'PLMX')

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--protocol', type=Path, default=ROOT/'report/SOURCE5_SHARED_PROTOCOL.json')
    p.add_argument('--report-path', type=Path, default=ROOT/'report/SOURCE5_SHARED_REVIEW.json')
    args = p.parse_args(); spec = load(args.protocol); root = Path(spec['root'])
    assert spec['source_cycle'] == 5 and spec['declared_before_shared_native']
    assert tuple(spec['modes']) == MODES and tuple(spec['arms']) == ('A', 'ABP')
    assert not spec['profiler'] and not spec['array_ledger'] and spec['expected_records'] == 32
    assert sha(ROOT/'report/PLANNER_SOURCE_LOCK.json') == spec['planner_lock_sha256']
    for name, digest in spec['required_review_locks'].items():
        assert sha(ROOT/'report'/name) == digest, name
    for name, digest in spec['analysis_tools'].items(): assert sha(ROOT/name) == digest, name
    report = {'source_cycle':5, 'adopted':False, 'profiler':False, 'historical_timing_pooled':False,
        'n3_related_new_tasks_not_fixed10_or_tail_acceptance':True, 'protocol_sha256':sha(args.protocol),
        'original_B_P_eachmap_newtask_median_reduction_min_pct':3, 'maps':{}}
    count = 0
    parent = Path(load(ROOT/'PARENT.json')['parent_workspace'])
    for mid, fixture_spec in spec['original_queries'].items():
        fixture = Path(fixture_spec['fixture']); definition = load(fixture)
        assert sha(fixture) == fixture_spec['fixture_sha256']
        assert sha(Path(definition['source_path'])) == definition['source_sha256']
        queries = definition['queries']; ids = [q['id'] for q in queries]
        assert len(queries) == 4 and not definition['fixed10'] and ids == fixture_spec['ids']
        info = {'arms':{}, 'B_P_relative_to_same_mode_A':{}}
        for arm, configuration in spec['arms'].items():
            paths = {mode:root/'shared'/arm/mode/mid for mode in MODES}
            data = {}; proofs = {}; observed = {}; raw = {}; parent_parity = {}
            for mode, directory in paths.items():
                rows = read(directory); protocol = load(directory/'protocol.json')
                assert protocol['queries_sha256'] == sha(fixture) and Path(protocol['query_file']) == fixture
                assert protocol['workload'] == 'fixed' and protocol['repeats'] == 1 and not protocol['profiler']
                assert (protocol['pose_offset_m'], protocol['yaw_offset_deg']) == (0, 0)
                assert len(rows) == summary(rows)['success'] == 4
                assert [row['query_id'] for row in rows] == ids
                for row, query in zip(rows, queries):
                    assert row['start'] == pose(query['start']) and row['goal'] == pose(query['goal'])
                    assert not row['fullroute'].get('reused') and not row['task_progress'].get('reused')
                proofs[mode] = source_proof(directory, mode, cycle=5, arm=arm, r2_mode=configuration['r2_mode'])
                observed[mode] = features(directory, rows, mode, mid)
                if mode == 'control':
                    old = parent/'results/shared_source4/shared/OGR'/arm/mid
                    parent_parity[mode] = pair(old, directory, strict=True)
                    assert len(parent_parity[mode]) == 4
                raw[mode] = [{'tag':tag(row), 'wall_s':row['wall_ms']/1000,
                    'tree_RSS_GiB':row['rss_peak_bytes']/2**30,
                    'local_accepted_in_strict_task':bool((row.get('v9_local') or {}).get('success')),
                    'segment_accepted_in_strict_task':bool((row.get('v9_local') or {}).get('success') and
                        (row.get('v9_local') or {}).get('segments',{}).get('matched_segments',0)>0),
                    'backend_calls':(row.get('l3') or {}).get('backend_call_count',0)} for row in rows]
                data[mode] = rows; count += 4
            groups = {mode:{'initial':summary(rows[:1]),'new_tasks':summary(rows[1:])} for mode, rows in data.items()}
            info['arms'][arm] = {'groups':groups,'candidate_control_exact':exact_compare(paths['control'],paths['PLMX']),
                'source_native_cleanup':proofs,'actual_features':observed,'raw_all4':raw,'sealed_parent_same_arm_control_exact':parent_parity}
        for mode in MODES:
            before = root/'shared'/'A'/mode/mid; after = root/'shared'/'ABP'/mode/mid
            quality = compare(before, after)
            assert quality['upstream_gate'] and quality['quality_gate']
            a = info['arms']['A']['groups'][mode]['new_tasks']['metrics']
            b = info['arms']['ABP']['groups'][mode]['new_tasks']['metrics']
            delta = 100*(b['e2e_s']['median']/a['e2e_s']['median']-1)
            info['B_P_relative_to_same_mode_A'][mode] = {'new_tasks_median_change_pct':delta,
                'benefit_at_least3':delta<=-3,'original_upstream_and_quality':quality,
                'memory_median_change_pct':100*(b['tree_rss_gib']['median']/a['tree_rss_gib']['median']-1),
                'not_a_fixed10_tail_or_fullmap_performance_claim':True}
        report['maps'][mid] = info
    assert count == 32; report['strict_total'] = count
    report['B_P_two_map_newtask_benefit_pass'] = {mode:all(
        info['B_P_relative_to_same_mode_A'][mode]['benefit_at_least3'] for info in report['maps'].values()) for mode in MODES}
    report['B_P_default_remains_disabled_pending_whole_objective'] = True
    assert not args.report_path.exists(); save(args.report_path, report)
    setup(); fig, axes = plt.subplots(2,2,figsize=(14,8))
    points = {'source_cycle':5,'font':FONT,'font_name':'SimSun','n3_related_raw_points_no_tail_or_boxplot_claim':True,
        'review_sha256':sha(args.report_path),'linear_axes':True,'groups':[]}
    for rr,(mid,name) in enumerate(MAPS):
        for cc,(field,unit,caption) in enumerate((('wall_s','s','新任务请求耗时'),('tree_RSS_GiB','GiB','新任务进程树RSS'))):
            ax=axes[rr,cc];labels=[];i=0
            for mode in MODES:
                for arm in ('A','ABP'):
                    raw=report['maps'][mid]['arms'][arm]['raw_all4'][mode][1:]
                    values=[row[field] for row in raw];assert len(values)==3
                    ax.scatter(i+np.linspace(-.07,.07,3),values,s=42); labels.append(mode+'/'+arm+'\nn=3')
                    points['groups'].append({'map_id':mid,'mode':mode,'arm':arm,'metric':field,'unit':unit,
                        'n':3,'query_ids':[row['tag'] for row in raw],'raw':values});i+=1
            ax.set_xticks(range(4),labels);ax.set_title(name+'\n'+caption+' / '+unit);ax.grid(axis='y',alpha=.25)
    fig.suptitle('V9-r9 source5：原种子后3个共享任务的逐次原值',fontsize=18)
    fig.text(.5,.03,'每组n=3相关新任务，不支持完整固定10组或尾部结论；线性坐标。\nRSS含Python与ROS后代20ms采样峰值，非独立进程PSS；严格成功另列32/32。',ha='center')
    fig.tight_layout(rect=(0,.1,1,.93)); fig.savefig(ROOT/'report/shared_source5.png',dpi=160);plt.close(fig)
    save(ROOT/'report/SHARED_SOURCE5_RAW_POINT_DATA.json',points)
    print('SOURCE5_SHARED32 strict/exact/source/native/currentACK/upstream/quality/cleanup verified')
    print('BP two-map original3pct gate',report['B_P_two_map_newtask_benefit_pass'])
    for mid,info in report['maps'].items():print(mid,{m:v['new_tasks_median_change_pct'] for m,v in info['B_P_relative_to_same_mode_A'].items()})

if __name__ == '__main__': main()
