"""Record the completed source4 block without pooling or hiding failed gates."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / 'report'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(name):
    return json.loads((REPORT / name).read_text())


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    review = read('SOURCE4_FULL_PROGRESS_REVIEW.json')
    assert review['source_cycle'] == 4 and review['strict_progress_total'] == 640
    assert not review['profiler'] and not review['diagnostic_subset'] and not review['adopted']
    assert sha(REPORT/'SOURCE4_FULL_PROGRESS_PROTOCOL.json') == review['predeclared_protocol_sha256']
    assert review['current_control_all_memory_subgroup_median_P95_pass'] == {'PLM': False, 'PLMU': True}
    assert review['original_AS_all40_RSS_median_at_least5_pass'] == {'PLM': True, 'PLMU': True}
    for name in ('SOURCE4_INITIAL_REVIEW_LOCK.json', 'SOURCE4_CONFIRMED_REVIEW_LOCK.json',
                 'SOURCE3_FULL_PROGRESS_REVIEW_LOCK.json', 'SOURCE3_INITIAL_REVIEW_LOCK.json'):
        if (REPORT/name).exists():
            for file_name, expected in read(name).items():
                assert sha(REPORT/file_name) == expected, file_name
    for name, expected in read('PLANNER_SOURCE_LOCK.json')['planning_source_files'].items():
        assert sha(ROOT/name) == expected, name

    resource_path = REPORT/'SOURCE4_FULL_PROGRESS_FROZEN_RESOURCE_REVIEW.json'
    resource = read(resource_path.name)
    assert len(resource['cleanup']) == len(resource['all_original_snapshot_copies_verified']) == 172
    assert len(resource['formal_source_snapshots']) == 16
    # The inherited generic verifier still reads the historical source3 summary.
    # Keep that provenance, and cite the existing complete tests for current source4.
    resource['inherited_generic_test_summary'] = resource['tests']
    actual_tests = {}
    for mode, name in [('control','tests_source4_control_all.txt'), ('PLMU','tests_source4_PLMU_all.txt')]:
        path = REPORT/name
        summary = path.read_text().splitlines()[-1]
        assert summary.startswith('573 passed,')
        actual_tests[mode] = {'path':str(path), 'sha256':sha(path), 'summary':summary}
    resource['current_source4_tests'] = actual_tests
    resource['tests'] = actual_tests['PLMU']['summary']
    save(resource_path, resource)

    failed = []
    for kind, maps in review['maps'].items():
        for mid, data in maps.items():
            for mode in ('PLM','PLMU'):
                relative = data['relative_to_control'][mode]['change_pct_by_group']
                for group in ('all','initial','moving'):
                    for metric in ('median','p95'):
                        value = relative[group]['tree_rss_gib'][metric]
                        if value > 3:
                            failed.append({'mode':mode,'workload':kind,'map_id':mid,'group':group,
                                'metric':'tree_rss_gib_'+metric,'change_pct':value,
                                'required_change_pct_at_most':3.0})
    assert len(failed) == 1 and failed[0]['mode'] == 'PLM'
    adoption = {
        'version':'3YD-V9-r9','source_cycle':4,'adopted':False,'goal_remains_active':True,
        'full_progress_strict_records':640,'review_sha256':sha(REPORT/'SOURCE4_FULL_PROGRESS_REVIEW.json'),
        'full_progress_incremental_gates_pass':review['incremental_gates_pass'],
        'original_AS_exact_arena_tail_recovery_pass':review['original_AS_exact_arena_recovery_pass'],
        'original_AS_all40_memory_each_map_workload_pass':review['original_AS_all40_RSS_median_at_least5_pass'],
        'all_current_control_memory_subgroup_median_P95_flags_pass':review['current_control_all_memory_subgroup_median_P95_pass'],
        'failed_current_control_memory_gates':failed,
        'fixed_reverse_confirmation_pass':{'PLM':True,'PLMU':False},
        'PLMU_reverse_arena_first_change_pct':-1.985326728,
        'U_semantic_fixed_repeat_negative_preserved_pct':[7.087016256899492,21.788382456370936],
        'old_source3_original_AS_arena_all40_memory_failure_preserved_pct':-4.335112866697521,
        'PLMU_all_arena_exact_timing_median_vs_PLM_pct':review['maps']['moving']['arena_large_250x300_2d_005']['U_incremental_vs_same_batch_PLM']['change_pct_by_group']['all']['e2e_s']['median'],
        'all_arena_timing_is_reported_not_a_new_predeclared_gate':True,
        'fixed_negative_not_replaced_by_progress':True,'historical_performance_not_pooled':True,
        'remaining':review['adoption_still_requires'] + ['PLM_changed_RSS_P95_control_gate','stable_U_fixed_repeat_evidence']}
    adoption_path = REPORT/'SOURCE4_FULL_PROGRESS_ADOPTION_REVIEW.json'
    assert not adoption_path.exists()
    save(adoption_path, adoption)

    names = {'mengjia':'万达3w地库','arena_large_250x300_2d_005':'arena_large（无语义）'}
    kinds = {'moving':'精确变化起点','perturbed':'3cm/2°偏差'}
    lines = ['# V9-r9 source4：完整推进640/640严格，性能验收尚未全部通过（2026-10-10）', '',
        '本批原AS、当前control、PLM与PLMU同批对照。万达3w地库（mengjia）及arena_large各保留原10组，每组初始、p20、p50、p80四请求，分别采用精确起点和3cm/2°偏差，共640/640三层静态严格成功。每个地图/模式/条件全部n=40、初始n=10、变化n=30；同一查询的三次变化有相关性。CPU12串行、线程数1，无剖析器或数组账本，原计时及20ms进程树RSS采样边界保持。', '',
        '**候选仍未采用，目标继续active。** 本批两组合相对原AS的四组全部40条RSS中位均下降至少5%，arena精确变化请求耗时中位/P95恢复门槛通过。但PLM的arena偏差变化组整树RSS P95相对control增加3.36766829%，严格超过3%。PLMU本批内存门槛通过，其此前反序固定首次arena仅下降1.98532673%，未达3%，以及万达3w相对PLM重复中位+21.7884%的负向结果仍保留。完整推进通过不能替代固定首次与独立冷首条验收。', '',
        '## 相对同批当前control的耗时与内存', '',
        '负值表示下降。RSS为整套Python及ROS后代进程树的采样峰值；单进程查询后PSS快照另列，不等同于整树峰值。', '',
        '| 条件 | 地图 | 组合 | 初始耗时中位/P95变化 | 变化耗时中位/P95变化 | 全部RSS中位/P95变化 |',
        '|---|---|---|---:|---:|---:|']
    def pair(values):
        return f"{values['median']:+.2f}% / {values['p95']:+.2f}%"
    for kind, maps in review['maps'].items():
        for mid, data in maps.items():
            for mode in ('PLM','PLMU'):
                r = data['relative_to_control'][mode]['change_pct_by_group']
                lines.append(f"| {kinds[kind]} | {names[mid]} | {mode} | {pair(r['initial']['e2e_s'])} | {pair(r['moving']['e2e_s'])} | {pair(r['all']['tree_rss_gib'])} |")
    lines += ['', '## 相对原AS的完整40条内存与变化请求恢复', '',
        '原AS内存门槛逐地图、逐条件采用全部40条RSS峰值中位，不用初始或变化子组替代。AS实际全图存储与候选完整任务范围存储的Native地图哈希分别核验，未将二者强写为相同。', '',
        '| 条件 | 地图 | 组合 | 全部RSS中位/P95变化 | 初始RSS中位/P95变化 | 变化RSS中位/P95变化 | 变化耗时中位/P95变化 |',
        '|---|---|---|---:|---:|---:|---:|']
    for kind, maps in review['maps'].items():
        for mid, data in maps.items():
            for mode in ('PLM','PLMU'):
                r=data['relative_to_AS'][mode]['change_pct_by_group']
                lines.append(f"| {kinds[kind]} | {names[mid]} | {mode} | {pair(r['all']['tree_rss_gib'])} | {pair(r['initial']['tree_rss_gib'])} | {pair(r['moving']['tree_rss_gib'])} | {pair(r['moving']['e2e_s'])} |")
    lines += ['', '## 当前control内存子组的独立审核', '',
        '| 条件 | 地图 | 组合 | 全部RSS中位/P95变化 | 初始RSS中位/P95变化 | 变化RSS中位/P95变化 |',
        '|---|---|---|---:|---:|---:|']
    for kind, maps in review['maps'].items():
        for mid, data in maps.items():
            for mode in ('PLM','PLMU'):
                r=data['relative_to_control'][mode]['change_pct_by_group']
                lines.append(f"| {kinds[kind]} | {names[mid]} | {mode} | {pair(r['all']['tree_rss_gib'])} | {pair(r['initial']['tree_rss_gib'])} | {pair(r['moving']['tree_rss_gib'])} |")
    lines += ['', 'PLM的arena偏差变化RSS P95 +3.36766829%未过，其全部40条P95 +2.782555%不能替代该子组。PLMU的全部/初始/变化RSS中位及P95在本批均通过最多3%退化门槛。', '',
        '## U相对同批PLM的附加变化', '',
        '| 条件 | 地图 | 初始耗时中位/P95变化 | 变化耗时中位/P95变化 | 全部RSS中位/P95变化 |',
        '|---|---|---:|---:|---:|']
    for kind, maps in review['maps'].items():
        for mid, data in maps.items():
            r=data['U_incremental_vs_same_batch_PLM']['change_pct_by_group']
            lines.append(f"| {kinds[kind]} | {names[mid]} | {pair(r['initial']['e2e_s'])} | {pair(r['moving']['e2e_s'])} | {pair(r['all']['tree_rss_gib'])} |")
    lines += ['', 'arena精确全部请求耗时中位相对PLM增加5.529985%，如实报告；该全部耗时混合组不属于已声明的初始/变化5%门槛，未另造有利门槛。U对本批RSS有附加下降，但首批万达3w重复中位/P95 +7.0870%/+9.0876%、反序重复中位+21.7884%、PLMU反序arena首次未达3%均独立保留。因此不能据本批宣布U稳定提速。source3原AS arena全部40条RSS仅下降4.3351%的失败、M相对PL重复P95 +10.5955%/+7.8123%也不被本批覆盖；性能不跨批拼接。', '',
        '## 输出、保护与剩余工作', '',
        '候选与同批control的L1、L2、软目标、完整范围、最终路径及有效costmap精确一致；当前AS/control与封存父r8同模式原输出精确一致。相对原AS的上游和原严格质量门槛通过，真实逐格ACK、原车辆参数、搜索预算和地图/停车绑定保持。没有移动窗口或无约束全图自由搜索。', '',
        '123项当前规划锁、source0–4归档、16份本批规划快照、全部172份源码副本及172批拥有进程/共享资源清理通过。父r8全35058项及V8-r11/V9-r0至r6冻结、L1源码与输入、原L2搜索/Native/TaskProgress/连接算法/原硬审计、生产入口与外部审计保持。V5 controller数据准备和初始化存储已有改动，不能称所有L2源码未变。当前源码既有control/PLMU各573项完整检查和43项共享专项通过；通用资源审核原读到历史source3的530项摘要，现保留其来源并另附当前两份573项原测试记录及哈希，没有重跑或改写旧测试。', '',
        'source4已审核1184条＝诊断64＋正式固定300＋反序固定180＋完整推进640，其中正式1120；source3实际1308，各源码周期累计3318条。累计次数只作记录清单，不合并性能分布。', '',
        '下一项为预声明的48条跨任务剖析诊断，检查局部连接与段复用中的checker构造开销，测试已有R2-PM配置；剖析耗时不能用于采用判断。AS固定/独立冷首条、44项事件、实际选定入口、B/P跨任务两图提速及最终保护仍待完成。默认r9=control、继承ASCHNT+OGR、B/P关闭。Q09前方障碍失败撤回/清除恢复仍须保留，本批未完成事件验收。执行闭环不属于本任务验收，静态回放不代表连续行驶或实车验收。', '',
        f'![原始耗时与整树RSS分布]({REPORT}/full_progress_source4.png)', '',
        f'![全部、初始及变化内存分别展示]({REPORT}/full_progress_memory_source4.png)', '',
        '两图均显式加载真实SimSun并完成中文及布局视觉检查。线性坐标；箱体Q1–Q3、中线中位数、须为1.5×IQR内最远原始值，点为离群点，各组标注样本数与单位。两份原始数据合计96组，四分位数、须和离群点单独保存。严格成功数单列。', '',
        f'审核：[SOURCE4_FULL_PROGRESS_REVIEW.json]({REPORT}/SOURCE4_FULL_PROGRESS_REVIEW.json)；采用审核：[SOURCE4_FULL_PROGRESS_ADOPTION_REVIEW.json]({adoption_path})。']
    result_path = REPORT/'SOURCE4_FULL_PROGRESS_RESULTS.md'
    assert not result_path.exists()
    result_path.write_text('\n'.join(lines)+'\n')
    lock_names = ['SOURCE4_FULL_PROGRESS_PROTOCOL.json','SOURCE4_FULL_PROGRESS_REVIEW.json',
        adoption_path.name,resource_path.name,'SOURCE4_FULL_PROGRESS_PROTECTED_REVIEW.json',result_path.name,
        'FULL_PROGRESS_SOURCE4_BOXPLOT_DATA.json','FULL_PROGRESS_MEMORY_SOURCE4_BOXPLOT_DATA.json',
        'full_progress_source4.png','full_progress_memory_source4.png']
    lock_path=REPORT/'SOURCE4_FULL_PROGRESS_REVIEW_LOCK.json'
    assert not lock_path.exists()
    save(lock_path,{name:sha(REPORT/name) for name in lock_names})

    status=read('SOURCE4_STATUS.json')
    status.update(stage='source4_full640_reviewed_unadopted_profile48_next',formal_record_count=1120,
        source4_verified_record_count=1184,all_source_cycles_verified_records=3318,
        source_copies_cleanup_verified=172,live_native_session_id=None,
        source4_full_progress_verified=True,source4_full_progress_review_running=False,
        full_progress_review=str(REPORT/'SOURCE4_FULL_PROGRESS_REVIEW.json'),
        full_progress_resource_review=str(resource_path),full_progress_report=str(result_path),
        full_progress_adoption_review=str(adoption_path),full_progress_review_lock=str(lock_path),
        full_progress_AS_all40_memory_pass=review['original_AS_all40_RSS_median_at_least5_pass'],
        full_progress_control_all_memory_subgroups_pass=review['current_control_all_memory_subgroup_median_P95_pass'],
        full_progress_current_control_failed_memory_gates=failed,
        next_work='Predeclared source4 shared profiler48 diagnostic only; PLM changed RSSP95 fail and PLMU reverse fixedfirst/repeat negatives preserved. ASfixed/cold/events44/entry/BP speed/final remain.')
    save(REPORT/'SOURCE4_STATUS.json',status)
    selected=read('SELECTED_DATA.json')
    selected.update(source4_actual_verified_records=1184,source4_formal_strict_records=1120,
        source4_fixed_strict_records=300,all_source_cycles_actual_records=3318,
        source4_full_progress_verified=True,source4_full_progress_review_running=False,
        latest_verified_formal_review=str(REPORT/'SOURCE4_FULL_PROGRESS_REVIEW.json'),
        latest_formal_report=str(result_path),latest_full_progress_report=str(result_path),
        full_progress_adoption_review=str(adoption_path),full_progress_root=review['results_root'],
        source4_full_progress_all_AS_memory_pass=review['original_AS_all40_RSS_median_at_least5_pass'],
        source4_full_progress_control_memory_pass=review['current_control_all_memory_subgroup_median_P95_pass'],
        current_plots=[str(REPORT/'full_progress_source4.png'),str(REPORT/'full_progress_memory_source4.png')])
    save(REPORT/'SELECTED_DATA.json',selected)
    baseline=json.loads((ROOT/'BASELINE.json').read_text())
    baseline.update(status='source4_full640_verified_PLM_memory_P95_failed_PLMU_fixedfirst_failed_unadopted',
        latest_full_progress_strict=640,latest_full_progress_source_cycle=4,
        latest_full_progress_all_AS_memory_pass=review['original_AS_all40_RSS_median_at_least5_pass'],
        latest_full_progress_control_memory_pass=review['current_control_all_memory_subgroup_median_P95_pass'])
    assert baseline['r9_mode']=='control' and not baseline['adopted'] and baseline['r2_mode']=='control'
    save(ROOT/'BASELINE.json',baseline)
    old=(REPORT/'RESULTS.md').read_text()
    (REPORT/'RESULTS.md').write_text(result_path.read_text()+'\n---\n\n以下为保留的历史独立实验块，状态和计数代表当时批次。\n\n'+old)
    progress='## 最新：source4完整640/640审核完成，尚未采用（2026-10-10）\n\n两图原10组精确及3cm/2°完整推进，AS/control/PLM/PLMU640严格且候选与control路径/地图精确。PLMU相对control初始中位下降5.44%/4.97%（精确）及6.21%/4.09%（偏差）；全部RSS相对原AS下降12.70%/10.96%及13.90%/9.85%。PLM偏差arena变化RSSP95+3.36766829%超过3%；PLMU反序首次arena仅−1.9853%和万达3wU重复+21.7884%负向保留。两组合原AS全部内存门槛本批过，旧4.3351%失败不覆盖。123规划锁、172副本/清理、当前两模式573测试、图形真实宋体QA及十文件报告锁完成。source4实际1184（正式1120+诊断64），各周期3318，不混性能。无Native在跑；下一预声明跨任务profile48诊断，冷首条/AS固定/事件44/入口/BP提速/最终验收仍待。默认control/BP关闭/goal active。旧运行头均属历史。\n\n'
    (REPORT/'PROGRESS.md').write_text(progress+(REPORT/'PROGRESS.md').read_text())
    continuation='# CURRENT: source4 full640 fully audited/locked; no Native; profile48 NEXT\n\nNative11393/review25763/resource75536/protected82242/plot2876 terminal0. SOURCE4_FULL_PROGRESS_RESULTS.md+ADOPTION_REVIEW+10-file REVIEW_LOCK saved. Original640 strict/exact; allAS40RSS5% gates bothpass, ASarenaexact recoverybothpass, currentcontrol allRSSsubmedian/P95 PLM FALSE (perturbedarena changedP95+3.36766829), PLMUtrue. PLMU reversearena first−1.98532673 FAIL3 and semanticUrepeat+21.78838246% (initial+7.087016/P95+9.087565) preserved; source3ASall40arena−4.33511287% historicalfail preserved. TrueSimSun both3040x2240PNG viewed,96rawgroups. Generic resource verifier read historical530 testsummary; fullresource now preserves oldsummary and adds actualexisting control/PLMU573 summary+hash each, no test rerun. 16full formal123snapshots; all172copies/cleanup. Verifiedsource4=1184=formal1120+diag64; allcycles3318, source3=1308. Planner123/allarchives unchanged, noadopt/defaultcontrol/BPoff/goalactive. Next CPU12serial tools/run_v9_r9_shared_profile.py predeclared48 originalshared2maps PLM/PLMU ×A/ABP/ABPM; diagnosticprofileonly, noformal timing/tail/RSS claims, originaldeadlines/audits/clocks retained. Review tools/review_v9_r9_shared_profile.py; then fullresources+protection+diagnosticreport/lock. ASfixed/cold/events44/actualentry/BPtwo-map speed/finalscope remain. No agents. Older headers are historical.\n\n'
    (REPORT/'CONTINUATION_STATE.md').write_text(continuation+(REPORT/'CONTINUATION_STATE.md').read_text())
    print('FULL640_RECONCILED source4=1184 formal1120 allcycles3318 copies_cleanup172; unadopted, failed gates preserved, lock10')


if __name__=='__main__':
    main()
