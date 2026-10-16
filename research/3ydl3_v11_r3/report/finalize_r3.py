"""Finalize from the full same-batch experiment, without changing measured code."""
from pathlib import Path
import json,csv,hashlib
import numpy as np
R=Path(__file__).resolve().parents[1];O=R/'report'
read=lambda p:json.loads(p.read_text())
def save(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
D=read(O/'PAIRED_RESULTS.json');P=read(O/'FINAL_PROTECTION.json')
assert read(R/'results/checks_final/COMPLETE.json')['complete']
assert '1442 passed' in (O/'project_tests_final.log').read_text()
events=[read(p) for p in (R/'results/checks_final/events').glob('*/summary.json')]
assert len(events)==6 and all(s['assertions_passed'] for s in events) and sum(s['events'] for s in events)==49
assert read(O/'DEFAULT_ENTRY_CHECK.json')['complete']
assert read(O/'FONT_CHECK.json')['resolved_family'].lower()=='simsun'
same={key:sum(r['same'][key] for r in D['original_output_checks']) for key in D['original_output_checks'][0]['same']}
rows=list(csv.DictReader((O/'paired_rows.csv').open()));permap={}
for m in read(R/'MAPS.json')['maps']:
 mid=m['map_id'];a=[r for r in rows if r['map_id']==mid and r['original_99']=='True'];permap[mid]={}
 for key in ['navigation_s','tree_rss_mib']:
  x=np.asarray([float(r[key]) for r in a if r['arm']=='control']);y=np.asarray([float(r[key]) for r in a if r['arm']=='candidate'])
  permap[mid][key]={'n_per_arm':len(x),'control_median':float(np.median(x)),'candidate_median':float(np.median(y)),'change_percent':float(100*(np.median(y)/np.median(x)-1))}
review={'all_predeclared_gates_passed':D['all_gates_passed'],'original_99_metrics':D['cohorts']['original_99'],'per_map':permap,'map_equal_geomean_median_change_percent':{k:100*(np.exp(np.mean([np.log(m[k]['candidate_median']/m[k]['control_median']) for m in permap.values()]))-1) for k in ['navigation_s','tree_rss_mib']},'interpretation':'One complete same-batch three-round fixed-query static experiment. No claim that every map improves or these measurements establish live execution performance.'}
save(O/'PERFORMANCE_REVIEW.json',review)
accepted=D['all_gates_passed']
acceptance={'version':'3YDL3-V11-r3','status':'static_functional_and_three_5percent_performance_accepted' if accepted else 'performance_not_accepted','goal_complete':accepted,'functional_goal_complete':D['gates']['all_300_policy_success'] and D['gates']['original_99_exact_preserved'],'full_performance_acceptance':accepted,'formal':'results/paired_final','control':'3YDL3-V11-r2','original99_exact_successes':sum(x['candidate']['exact_success'] for x in D['maps'].values()),'tolerant_successes':sum(x['candidate']['tolerant_success'] for x in D['maps'].values()),'policy_successes':sum(x['candidate']['success'] for x in D['maps'].values()),'total_requests_per_arm':300,'gates':D['gates'],'project_tests':1442,'event_assertions':49,'default_entry_checks':2,'owned_batches_cleaned':P['owned_batches'],'unchanged_original_outputs':same,'measured_source_lock':'results/paired_final/source.json','production_entry_replaced':False,'cloud_visualization_updated':False,'gitlab_published':False}
save(R/'ACCEPTANCE.json',acceptance)
task=read(R/'TASK.json');task.update(status='complete' if accepted else 'active',phase='accepted static candidate' if accepted else 'formal performance gate failed',goal_complete=accepted,functional_goal_complete=acceptance['functional_goal_complete'],performance_goal_complete=accepted,active_processes=[],next_work=[] if accepted else ['Independent next candidate; preserve this formal batch'],acceptance_result='ACCEPTANCE.json',checks={'project_tests':1442,'events':49});task.pop('previous_revision_result',None);save(R/'TASK.json',task)
names={'mengjia':'万达3w地库','pudu_wanda_3f':'万达2w地库','3yd_synth_multiloop_a_v1':'合成地库A','3yd_synth_twinwing_b_v1':'合成地库B','3yd_synth_trizone_c_v2':'合成地库C','arena_small_080x100_2d_005':'Arena小图','arena_medium_200x200_2d_005':'Arena中图','arena_large_250x300_2d_005':'非语义大地图','mentor_map_20260825_005':'导师原图','mentor_map_20260825_005_4x_area':'导师四倍面积图'}
s=[];add=s.append
add('# 3YDL3-V11-r3 首次导航性能验收\n')
add('日期：2026-10-16。独立工作区：`'+str(R)+'`。\n')
add('**'+('预设三个至少5%性能门槛全部通过，十图300/300政策内成功保留。' if accepted else '预设性能门槛未全部通过，目标未完成。')+'** 对照为已验收V11-r2。十图各自固定10Q×3轮，每版300次；双方均297次精确端点成功、3次授权容差成功。\n')
add('计时是地图及服务就绪后各Q每轮首次实际导航：包含失败尝试、候选恢复、L1/L2/L3、任务同步、真实ACK、严格终验及请求结束前的堆回收。地图加载、通用初始化、服务启动、源码快照和请求后导出排除。不以重复任务缓存命中替代首次请求。\n')
add('主指标固定原99组精确成功查询，每版297次；整树RSS是规划进程及ROS子进程的逐请求采样峰值，不能解释成累计分配字节或单独规划器内存。\n')
add('| 指标 | 同批V11-r2 | V11-r3 | 相对变化 |\n|---|---:|---:|---:|')
for label,key,stat in [('导航中位（s）','navigation_s','median'),('导航P95（s）','navigation_s','p95'),('整树RSS中位（MiB）','tree_rss_mib','median'),('整树RSS P95（MiB）','tree_rss_mib','p95'),('导航最大观测（s）','navigation_s','maximum'),('整树RSS最大观测（MiB）','tree_rss_mib','maximum')]:
 v=D['cohorts']['original_99'][key];add(f"| {label} | {v['control'][stat]:.6f} | {v['candidate'][stat]:.6f} | {v['change_percent'][stat]:+.2f}% |")
add('\n上述前三项分别以至少降低5%为门槛；其余指标同时披露，不以任一改善替代未通过的门槛。\n')
add('| 全部请求，含容差回退（每版300次） | 同批V11-r2 | V11-r3 | 相对变化 |\n|---|---:|---:|---:|')
for label,key,stat in [('导航中位（s）','navigation_s','median'),('导航P95（s）','navigation_s','p95'),('RSS中位（MiB）','tree_rss_mib','median'),('RSS最大观测（MiB）','tree_rss_mib','maximum')]:
 v=D['cohorts']['all_requests'][key];add(f"| {label} | {v['control'][stat]:.6f} | {v['candidate'][stat]:.6f} | {v['change_percent'][stat]:+.2f}% |")
add('\n![正式原始分布]('+str(O/'paired_distributions.png')+')\n')
add('图中箱体Q1–Q3、中位数线、1.5×IQR内最远样本为须，散点为离群值。时间使用标明的对数坐标，各组注明样本数，加载真实宋体。\n')
add('## 三轮复核与CPU\n')
add('| 轮次，原99组每版99次 | 导航中位变化 | 导航P95变化 | RSS中位变化 |\n|---|---:|---:|---:|')
for rnd in range(1,4):
    selected=[r for r in rows if int(r['round'])==rnd and r['original_99']=='True'];delta=[]
    for key,fun in [('navigation_s',np.median),('navigation_s',lambda a:np.quantile(a,.95)),('tree_rss_mib',np.median)]:
        a=[float(r[key]) for r in selected if r['arm']=='control'];b=[float(r[key]) for r in selected if r['arm']=='candidate'];delta.append(float(100*(fun(b)/fun(a)-1)))
    add(f'| {rnd} | {delta[0]:+.2f}% | {delta[1]:+.2f}% | {delta[2]:+.2f}% |')
I=read(O/'IMPLEMENTATION_REVIEW.json');C=I['tree_cpu_s'];H=I['heap_release']
add(f"\n全部请求的整树CPU时间中位数{C['control']['median']:.6f}→{C['candidate']['median']:.6f}s，P95 {C['control']['p95']:.6f}→{C['candidate']['p95']:.6f}s。它与导航墙钟耗时及RSS分别统计，不作为替换门槛。堆检查/回收每请求成本中位{H['call_wall_ms']['median']:.3f}ms、P95 {H['call_wall_ms']['p95']:.3f}ms，已包含在导航时间中。\n")
add('## 十图结果\n')
add('| 地图 | 双方政策成功 | 候选精确/容差 | 原99组耗时中位变化 | RSS中位变化 |\n|---|---:|---:|---:|---:|')
for mid,v in D['maps'].items():
 t=permap[mid];add(f"| {names[mid]} | {v['control']['success']}/30 → {v['candidate']['success']}/30 | {v['candidate']['exact_success']}/{v['candidate']['tolerant_success']} | {t['navigation_s']['change_percent']:+.2f}% | {t['tree_rss_mib']['change_percent']:+.2f}% |")
add('\n导师原图性能列是原9组×3轮=27次/版，其余30次/版。总体门槛通过不表示各地图都改善5%。\n')
add('![逐图箱线图]('+str(O/'per_map_distributions.png')+')\n')
add('![L1长度归一化分布]('+str(O/'normalized_distributions.png')+')\n')
add('归一化分母是各次正式L1输出right_reference_polyline的折线长度，单位米；RSS/米表示比值，并非每米新增内存或累计内存消耗。原总耗时和总RSS数据保留，性能门槛仍使用原指标。\n')
add('## 实现与代价\n')
add('1. L2只在已确定ROI内组合掩码，用轴向归约代替全图坐标列表。语义绑定每次仍哈希当前内容，已知结构省去dataclass深拷贝，其他类型回退原方法。\n2. 对原初次求解已选中双端初始化的语义任务，先给1秒原求解时间片，再提前尝试现有恢复；若失败且同一预算尚有余额，继续原调度。没有增加总预算或放宽约束。\n3. 路线缓存未命中时提前释放旧代价图所有权；L3邻域只扫描保守包围范围，构造字节0/1掩码直接转为布尔视图，常零惩罚场采用只读标量广播。\n4. 请求计算和终验完成后、实际返回前，若glibc空闲堆至少64MiB则执行malloc_trim，保留32MiB顶部余量。回收调用计入当前请求，后续重新分配成本也计入对应首次查询；全局分配器阈值和子进程分配器不变。主要作用是减少不同Q间的空闲堆滞留，不能将其表述为首个Q必然等比例改善。\n')
add('配置、原生库、冻结L1、地图/Q绑定、车体、交通规则、0.40m半径与15秒L3/30秒总请求预算保持。调度调整了原有候选的尝试时间，原严格连续曲线/车体/交通/端点审计仍逐条执行；本轮原297对最终路径及分层输出的实际一致性由下述哈希核验给出。\n')
add('另外，语义图静态代价场从已封存占用栅格按原LUT还原，无语义图保留原密集静态代价数组；大图原硬障碍布尔场按位无损存储。启用前逐格确认静态场与原LUT一致；本地发送及ACK字段仍是原始普通数组。无语义背景转换只处理非硬障碍单元，原硬障碍100、未知值、重叠软代价片顺序和预绑定哈希均保持。\n')
add('无语义可行驶与物理掩码逐格证明相同后共享不可变存储。目标范围使用单临时数组组合；不可变的全零广播动态层按恒等式处理，其余动态层按原顺序处理。新求解路线仍经过完整同步、真实ACK和严格终验。\n')
add('粗栅格障碍场只在不可变且逐格互补验证通过后复用，避免求解视图额外持有一份全粗栅格hard数组；可变字段及异常引用回退原取反。常假歧义场改用不可变标量广播，原逐格值不变。\n')
add('## 校验与范围\n')
add(f"1442项项目检查、49项受控事件断言、三组导师查询顺序及两个真实默认入口通过。双方{D['certificates_checked']}条成功证书/ACK核验，{P['owned_batches']}批拥有资源清理完成；原297对相同输出数量："+'，'.join(f'{k} {n}/297' for k,n in same.items())+'。\n')
add('导师A2B-07仍为先精确失败、后授权容差成功；实际姿态偏差单列如下。\n')
add('| 版本/轮次 | 完整导航（s） | 起点偏差m/° | 终点偏差m/° | 曲率上界（1/m） |\n|---|---:|---:|---:|---:|')
for x in D['new_tolerant_successes']:
 a=x['offsets']['start'];b=x['offsets']['goal'];add(f"| {'r2' if x['arm']=='control' else 'r3'}/{x['round']} | {x['navigation_s']:.6f} | {a['position_m']:.3f}/{a['yaw_deg']:.2f} | {b['position_m']:.3f}/{b['yaw_deg']:.2f} | {x['curvature_bound']:.6f} |")
add('\n仅静态全程规划和指定受控事件验收。万达3w Q09前方障碍仍预期撤回并在清除后恢复；事件通过不等于全部绕障成功，也不代表连续控制或实车验收。容差起点不代表已认证从实时定位到该点的执行连接。\n')
add('正式数据只取`results/paired_final`，早期A–J及本候选一轮试验全部排除。A内存改善不足；8/20MiB全局分配阈值诊断未采用；B构造前回收、C/D较高结束阈值均未采用。C首次回收试验存在导入遗漏，30次候选异常原样保留，已修复并补充完整请求流程测试，错误批不参与性能比较。\n')
add('完整三轮按地图交替版本顺序，CPU14串行，无同核心重负载；不选取最快轮、不删除慢请求。结论是本固定样本和运行环境中的观测验收，不推广到所有查询顺序或平台。\n')
add('## 可追溯结果\n')
for title,name in [('逐条统计与门槛','PAIRED_RESULTS.json'),('原始逐次CSV','paired_rows.csv'),('性能逐图核对','PERFORMANCE_REVIEW.json'),('实现和路径变化','IMPLEMENTATION_REVIEW.json'),('源锁及资源清理','FINAL_PROTECTION.json'),('正式数据选择','SELECTED_DATA.json')]:add(f'- [{title}]({O/name})')
(O/'RESULTS.md').write_text('\n'.join(s)+'\n')
(R/'README.md').write_text('# 3YDL3-V11-r3\n\n独立首次导航性能候选，同批对照V11-r2。\n\n- 结果：[report/RESULTS.md](report/RESULTS.md)\n- 验收：[ACCEPTANCE.json](ACCEPTANCE.json)\n- 正式数据：`results/paired_final`\n- 入口：`./run.sh --map-id <MAPS.json中的ID> --output <新目录>`\n')
print(json.dumps(acceptance,ensure_ascii=False,indent=2))
