"""Publish measured constraints without rewriting the historical acceptance."""
import json,csv,hashlib,math
from pathlib import Path
W=Path(__file__).resolve().parents[1];R=W/'report/constraints_20260915_01'
d=json.loads((R/'comparison.json').read_text());ref=json.loads((R/'refinements.json').read_text());rows=json.loads((R/'rows.json').read_text());a=d['arms']['baseline'];b=d['arms']['candidate'];details={r['query']:r for r in ref['rows']}
def tab(h,rr):return '\n'.join(['| '+' | '.join(h)+' |','| '+' | '.join(['---']*len(h))+' |',*['| '+' | '.join(map(str,r))+' |' for r in rr]])
def n(v,k=3):return f'{v:.{k}f}'
minimum=min(r['certified_body_clearance_lower_bound_m'] for r in ref['rows']);checks=[]
for label,key in [('静态障碍及未知格：车体连续扫掠','static_occupancy_swept_valid'),('已确认语义禁行区：车体连续扫掠','semantic_hard_swept_valid'),('车体全程包含于允许走廊','body_and_corridor_swept_valid'),('严格起终点位姿误差','strict_endpoint_valid'),('曲率 / 最小转弯半径','curvature_valid'),('有效主地图精确 ACK','exact_ack_accepted'),('软代价有限、非负且不超过 140','soft_cost_valid')]:
 checks.append([label,f"{a['passes'][key]}/120",f"{b['passes'][key]}/120",'一致，通过'])
checks += [['倒车距离 / 原地旋转','0 m / 0 次','0 m / 0 次','通过'],['位置 / 航向记录不连续','0 / 0','0 / 0','按冻结阈值通过'],['原转角元数据相邻变化 ≤15°','120/120','120/120','字段被限幅，不足以证明几何连续性'],['几何回算未限幅转角相邻变化 ≤15°','0/120','0/120','两组共有缺口；20/20 个不同查询均超限'],['完整经过 L1 指定区域顺序','96/120（16/20 查询）','96/120（16/20 查询）','4 个不同查询不一致'],['单行 / 禁转规则','N/A','N/A','本次真实地图未配置，不能报 100%'],['速度 / 加速度 / 禁停 / 减速带速度','未测','未测','没有时间参数或执行速度；velocity=0 为占位值']]
worst=max(rows,key=lambda r:r['steering']['raw_max_step_deg']);example=worst['steering']['worst_example'];seqfails=[r for r in ref['rows'] if not r['ordered_regions_verified']]
lines=['# 约束指标结果对比：原 ACK 实现与语义 ACK / 退出清理修复','',
'**结论：两组约束表现相同，但原“120/120 成功”不能解释为所有几何约束已经满足。** 碰撞、走廊包含、曲率、端点及地图 ACK 均通过；未限幅转角连续性和 L1 区域顺序的一致性存在两组共有缺口。时序修复未新增这些问题，也没有修复它们。','',
'本报告仅分析既有静态实验轨迹，未重跑规划、未修改算法、阈值或原始结果。A 为上一轮掉头/启动 ACK 实现的同日重测（`final_1_baseline`、`final_4_baseline`），B 为本轮语义 ACK/退出清理修复（`final_2_combined`、`final_3_combined`）。各 20 个不同查询 ×3 次重复 ×2 批＝120 次，总计 240 条记录。位置/航向、参考路、走廊与数据哈希逐条核对；按相同数据复用 20 份离线几何检查结果，不将重复当作独立场景。','',
'## 1. 约束结果总表','',tab(['检查项','A：原实现重测','B：本轮修复','判断'],checks),'',
'上表区域顺序采用“计划区域依次作为实际经过区域的子序列”的标准，与现有掉头修复终验一致，允许额外经过区域；若要求实际序列完全等于计划，只有 84/120（14/20）一致。它与“仍在膨胀后的允许走廊内”是两个检查。原直接成功分支没有统一执行此区域顺序终验；本次将其扩展为全量离线复核指标。','',
'## 2. 约束值与剩余余量','',tab(['指标','固定阈值 / 含义','A','B'],[
 ['全组最大曲率','≤2.50 m⁻¹；数值容差 1e-6',n(a['maximum_curvature_1pm']['max'],12),n(b['maximum_curvature_1pm']['max'],12)],
 ['最小等效转弯半径','≥0.40 m；同一数值容差','0.400000 m','0.400000 m'],
 ['起终点位置误差最大值','≤0.125 m',n(a['position_error_m']['max'],6)+' m',n(b['position_error_m']['max'],6)+' m'],
 ['起终点航向误差最大值','≤5°',n(a['heading_error_deg']['max'],6)+'°',n(b['heading_error_deg']['max'],6)+'°'],
 ['最大相邻位置间隔','≤0.15 m',n(a['max_pose_spacing_m']['max'],6)+' m',n(b['max_pose_spacing_m']['max'],6)+' m'],
 ['最大相邻航向变化','≤25°',n(a['max_yaw_step_deg']['max'],6)+'°',n(b['max_yaw_step_deg']['max'],6)+'°'],
 ['原转角记录最大相邻变化','≤15°','15.000°','15.000°'],
 ['未限幅几何转角最大相邻变化','同一个 ≤15° 指标',n(a['raw_steering_max_step_deg']['max'],6)+'°',n(b['raw_steering_max_step_deg']['max'],6)+'°'],
 ['旧净空字段最小值','路径点中心到已占据格中心的近似距离，不是车体净空',n(a['legacy_center_clearance_m']['min'],3)+' m',n(b['legacy_center_clearance_m']['min'],3)+' m'],
 ['认证的车体净空下界最小值','矩形扩张＋连续扫掠，搜索分辨率 ≤2 mm',n(minimum,6)+' m',n(minimum,6)+' m']]),'',
'多数轨迹的曲率到达 2.50 m⁻¹ 上限，不能说转弯余量很大。表中的微小浮点超出约 1e-12，在原 1e-6 容差内。车体使用加 padding 后半长 0.265 m、半宽 0.225 m。','',
'车体净空下界通过同时扩大车体半长和半宽，再使用原连续扫掠检查器认证得到；扩大后的矩形包含“原车体与半径为该余量的圆盘的 Minkowski 和”，通过时能保守证明至少该净空。失败的更大矩形不能作为真实净空上界，也未新增 0.162 m 硬门槛。原配置的 0.50 m 是靠右评价的安全边界设定，不能直接当作全路线车体净空阈值。初步外接圆下界在两条路径上为负，仅说明该估计过松，未据此认定碰撞；矩形扩张复核全部得到正下界。','',
'## 3. 重要缺口一：转角元数据掩盖了连续性问题','',
'冻结 `_annotate_smac_points` 先从位置/航向计算转角，再逐点将相邻转角差限幅到 ±15°。后续 `PathAuditor` 读取该字段检查转角连续性；因此原记录的“0 次转角跳变”不是未修改几何的连续性证据。源码位于 [unified_four_backends_smoke.py](../../../external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/unified_four_backends_smoke.py:1356)，检查位于 [path_audit.py](../../../external/arena4_ws/src/arena/evaluation/arena_evaluation/arena_evaluation/path_audit.py:250)。','',
'本次保持 x/y/yaw 不变，按原注释器限幅前的同一公式 `δᵢ = atan(0.50 × wrap(ψᵢ₊₁−ψᵢ) / chordᵢ)` 回算转角，且仅在内存副本中替换 steering，再运行原 PathAuditor：两组各 **120/120 均因 STEERING_DISCONTINUITY 拒绝**，其余原检查仍通过。每组有 468 个超限转角边界，对应 20 个不同查询的 78 个边界各重复 6 次。用带符号圆弧曲率替代 yaw/chord 后结论仍不变，最大变化约 77.182°，排除了仅由弦长/弧长近似导致超限的解释。','',
f"具体例子：`{worst['query']}`，轨迹零基索引 {example['index_zero_based']} 附近（CSV 及 JSON 保留原案例路径）。限幅前相邻转角 {n(example['raw_steering_deg'][0],3)}° → {n(example['raw_steering_deg'][1],3)}°，变化 **{n(worst['steering']['raw_max_step_deg'],3)}°**；元数据为 {n(example['metadata_steering_deg'][0],3)}° → {n(example['metadata_steering_deg'][1],3)}°。",'',
'该指标是“每相邻轨迹样本的转角差”，不是转向角速度或真实车辆执行能力；当前数据没有时间参数，不能据此断言实机转向速率超限。但按项目已有的 15° 连续性检查要求，不能继续将限幅后的元数据通过当作几何已经满足。','',
'## 4. 重要缺口二：4 条查询不遵守计划区域顺序','',
'从保存的物理 portal 序列恢复计划区域链，结合冻结拓扑分区复核实际轨迹。先按原 0.025 m/1° 插值检查，再枚举每条 XY 线段穿过的所有栅格边界，检查每段具有正长度的格子访问；两种方法对 20 条查询的通过/失败判定完全相同，排除固定间隔采样遗漏造成这 4 条失败的解释。','',
tab(['查询','计划语义顺序','实际经过语义顺序'],[[r['query'],' → '.join(r['required_semantic_sequence']),' → '.join(r['visited_semantic_sequence'])] for r in seqfails]),'',
'路口转弯两条查询用“路口4”替代了 L1 指定的“车位区9”；停车区内部两条没有经过计划中的未标注语义段，并再次经过车道5。它们没有越出允许走廊或发生碰撞，但不能声称严格实现了 L1 指定访问意图。现有走廊经过膨胀，落在其并集内不等价于依次访问计划区域。','',
'## 5. 靠右与路径质量','',
f"两组全部几何轨迹和 L2 参考均一致。公共评价场的靠右目标带占比中位数均为 **{100*b['right_band_ratio']['p50']:.2f}%**；按有效评价弧长汇总为 **{100*b['right_band_arc_weighted_ratio']:.2f}%**。20 条中有 {b['unique_queries_at_least_80_percent']} 条达到 80%，两条停车区内部仍为 0%。80% 是冻结控制场景的验收门槛；在这组复杂真实地图上只作描述，不把其余 7 条全部计为硬约束失败。",'',
'延用原公共评价场、每 0.025 m 弧长积分、两端各排除 5 m。来源：原 [path_quality.json](../path_quality.json)。不将“路径靠右率”冒充“L2→L3 条件偏好保持率”；后者本次没有新增计算。原控制直路与旋转场景各 9/9 通过、目标带比例 100%、右→左→右切回轨迹复原，见原 [performance.md](../performance.md)。','',
'## 6. 逐查询对照（两组数值相同）','',
tab(['查询','κ 最大（m⁻¹）','未限幅转角最大变化','区域顺序','认证净空下界 m','靠右率'],[[p['query'],n(p['curvature_max_1pm'],3),n(p['raw_steering_max_step_deg'],2)+'°','通过' if details[p['query']]['ordered_regions_verified'] else '不一致',n(details[p['query']]['certified_body_clearance_lower_bound_m'],3),n(100*p['right_ratio'],1)+'%'] for p in d['paired_queries']]),'',
'## 7. 未覆盖项与工作优先级','',
'本次真实地图没有激活单行规则或禁转组合，只配置靠右偏好；不能将它们的 0 次违规写成规则验证通过。1 个已确认禁行语义区域新增 4,682 个硬格，参与了车体检查；14 个围挡区域按原冻结协议未确认实体不可通行、保留非硬语义，不提供强制绕开这些围挡的证明。没有时间化轨迹，也没有速度、加速度、制动、禁停或实机跟踪误差结果。','',
'推荐优先顺序：', '',
'1. **P0：转角记录、几何和验收一致。** 取消用元数据限幅冒充几何修复；让未经限幅的几何转角进入验收，再通过真正改变轨迹的局部平滑/运动学连接满足原连续性要求。失败应如实返回，不放宽 0.40 m 半径或碰撞约束。',
'2. **P0：明确并统一验证 L1 区域访问意图。** 将需要保持的有序访问要求接入直接成功和修复后的所有路径分支，处理上述四条不一致；不能仅凭走廊掩码通过。',
'3. **P1：处理剩余靠右不足。** 在上述一致性问题解决后，修复停车区和路口偏好，继续报告长度、净空和靠右率，避免仅提高评价分数。',
'4. **再跑同一固定集的性能与约束联合对照。** 本轮 P95 改善仍是已有输出与原验收口径下的测量；不能预先承诺修复几何后仍保持相同耗时。','',
'本请求完成了结果比较与复核，没有修改规划实现。原实验 120/120、原验收 JSON 及原耗时记录保留；本报告补充了它们的适用边界，不能把两种验收解释混为一谈。','',
'## 8. 证据与复现','',
'- [比较汇总 JSON](comparison.json)：冻结阈值、两组全部统计、语义覆盖和完整性。',
'- [逐运行复核 JSON](rows.json)：240 条记录、原验收、几何回算复验和最坏位置证据。',
'- [栅格穿越与净空复核](refinements.json)：20 条不同轨迹的解析栅格访问和逐级扩张检查。',
'- [逐查询 CSV](per_query_final.csv)：两组相同结果的便捷导出。',
'- [输入哈希](source_manifest.json)：1,226 个原始数据/配置/源码文件核对；原冻结运行时代码未变。','',
'复现时使用新的空输出目录：','',
'```bash','cd /home/robot/workspaces/3yd_v1_semantic_ack_20260915','source env.bash','OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tools/compare_constraints.py --output report/constraints_new','```','',
'`refine_constraint_diagnostics.py` 和 `write_constraint_report.py` 当前绑定本次输出目录；它们只读原结果并写派生分析，不执行 ROS 规划。','']
# Resolve artifact code references directly; these are real local file links.
text='\n'.join(lines).replace('../../../external/',str(W/'external')+'/')
(R/'comparison.md').write_text(text)
final=[]
for p in d['paired_queries']:
 final.append({**p,'certified_body_clearance_lower_bound_m':details[p['query']]['certified_body_clearance_lower_bound_m'],'grid_crossing_ordered_pass':details[p['query']]['ordered_regions_verified']})
with (R/'per_query_final.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=list(final[0]));writer.writeheader();writer.writerows(final)
quality_checks={'240_rows':len(rows)==240,'20_different_queries':len(d['paired_queries'])==20,'all_artifacts_unchanged':d['integrity']['source_hashes_unchanged'] and not d['integrity']['frozen_runtime_changes'],'same_saved_poses':all(p['same_poses_all_12'] for p in d['paired_queries']), 'all_original_gates_reproduced':all(r['replay_matches_recorded_original_gate'] for r in rows),'steering_isolated_failure':all(r['unclipped_only_steering_failure'] for r in rows),'independent_grid_crossing_agrees':all(r['matches_sampled_subsequence_verdict'] for r in ref['rows']),'certified_positive_body_margin':all(r['certified_body_clearance_lower_bound_m']>0 for r in ref['rows'])}
(R/'analysis_verification.json').write_text(json.dumps({'analysis_checks':quality_checks,'analysis_complete':all(quality_checks.values()),'all_constraints_pass':False,'not_a_planner_change':True},indent=2)+'\n')
assert all(quality_checks.values())
files=[R/'comparison.md',R/'comparison.json',R/'rows.json',R/'refinements.json',R/'per_query_final.csv',R/'analysis_verification.json',*[(W/'tools'/name) for name in ['compare_constraints.py','refine_constraint_diagnostics.py','write_constraint_report.py']]]
(R/'delivery_manifest.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2)+'\n')
print('Constraint comparison written; analysis checks pass; all-constraints-pass = false.')
