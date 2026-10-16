"""Build an auditable report only from completed experiment files."""
import json,collections,datetime,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
W=Path('/home/robot/workspaces/semantic_dual_map_r1_20260914');R=W/'results';P=W/'report';F=R/'real_formal_03';C=R/'controlled_formal_04';D=R/'dynamic_formal_01'
def load(p):return json.loads(p.read_text())
def jl(p):return [json.loads(x) for x in p.read_text().splitlines()]
def link(path,label=None):return f'[{label or path.name}]({path})'
def num(x,d=2):return '未提供' if x is None else f'{x:.{d}f}'
def qpair(d,scale=1):return f"{num(None if d['p50'] is None else d['p50']/scale)} / {num(None if d['p95'] is None else d['p95']/scale)}"
def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join('---' for _ in headers)+' |',*['| '+' | '.join(map(str,r))+' |' for r in rows]])
s=load(P/'statistics.json');s['by_arm']={a:s['by_arm'][a] for a in ['original_r1','A','B','C']};real=jl(F/'runs.jsonl');control=jl(C/'runs.jsonl');dynamic=load(D/'runs.json');prep=load(R/'preparation_profile_01/summary.json');timing=load(R/'preparation_profile_01/timings.json');prov=load(P/'provenance_verification.json');retention=load(F/'derived/preference_retention.json')
# Every attempt is preserved separately; no pooled rate across changing versions.
attempts=[]
for path in sorted(R.iterdir()):
 if not path.is_dir():continue
 rows=None;unit='case';source=None
 for name in ['runs.jsonl','runs.json','results.json']:
  f=path/name
  if f.exists():
   rows=jl(f) if name.endswith('jsonl') else load(f);source=f;break
 if rows is None and (path/'result.json').exists():rows=[load(path/'result.json')];source=path/'result.json'
 if isinstance(rows,list):
  rows=[r for r in rows if isinstance(r,dict)];success=sum(bool(r.get('success',r.get('ok',False))) for r in rows)
  errors=collections.Counter(str(r.get('failure_code',r.get('error',r.get('reason','unspecified')))) for r in rows if not r.get('success',r.get('ok',False)))
  a={'attempt':path.name,'count':len(rows),'success':success,'failure':len(rows)-success,'failure_codes':dict(errors),'source':str(source)}
 else:a={'attempt':path.name,'count':0,'success':0,'failure':0,'source':str(path),'status':'no completed case rows; inspect startup or preparation logs'}
 if (path/'interruption.json').exists():a['interruption']=load(path/'interruption.json')
 if (path/'startup_failure.json').exists():
  a['startup_failure']=load(path/'startup_failure.json');a['status']='启动失败：'+a['startup_failure']['reason']
 if path.name=='original_r1_strict_regression_01' and (path/'summary.json').exists():
  summary=load(path/'summary.json');a.update(count=6,success=6 if summary['l3_pathaudit_gate']['all_valid'] else 0,failure=0,status='6 次 L3，另有 10 对 L2；严格回归通过')
 if path.name=='preparation_profile_01' and (path/'summary.json').exists():a['status']='完整构建与恢复计时完成；不属于查询分母'
 if path.name=='control_dev_C_01':a.update(count=1,success=0,failure=1,status='早期调用缺少检查器参数；1 次预备调用失败，未形成求解输出，保留 L2/走廊，原始栈未保存')
 if path.name=='diagnostic_graph_initial':a['status']='初始过密构图诊断主动停止，详见 status.json'
 if (path/'cases').exists() and isinstance(rows,list):a['uncompleted_case_directories']=max(0,len(list((path/'cases').iterdir()))-len(rows))
 attempts.append(a)
(P/'attempt_manifest.json').write_text(json.dumps(attempts,ensure_ascii=False,indent=2))
# Strict frozen r1 computation uses its own historical rules, separately from
# the new 2.500001 continuous-body acceptance applied above.
strictpath=R/'original_r1_strict_regression_01';strict=load(strictpath/'summary.json') if (strictpath/'summary.json').exists() else None
historical_hash='f046456bff28cc32f79e26b9eeadab644b495d9cad2cb00d5d553083943aee35'
l3rows=jl(strictpath/'l3_runs.jsonl') if (strictpath/'l3_runs.jsonl').exists() else []
if not l3rows:
 import csv
 if (strictpath/'l3_pathaudit_gates.csv').exists():l3rows=list(csv.DictReader((strictpath/'l3_pathaudit_gates.csv').open()))
strict_hashes=sorted(set(r.get('canonical_path_hash','') for r in l3rows))
strict_hash_match=bool(strict_hashes) and strict_hashes==[historical_hash]
historical_dir=Path('/home/robot/pudu_robot_ws/experiments/layered_planner_benchmark/nav2_3d_v1_r1_l2_strict_ab_cmp2_04_20260908T194000_SGT/l3_paths')
geometry_comparison=[{'file':p.name,'current_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'historical_sha256':hashlib.sha256((historical_dir/p.name).read_bytes()).hexdigest(),'bytes_equal':p.read_bytes()==(historical_dir/p.name).read_bytes()} for p in (strictpath/'l3_paths').glob('*.csv')]
strict_path_bytes_match=len(geometry_comparison)==6 and all(r['bytes_equal'] for r in geometry_comparison)
(P/'strict_historical_comparison.json').write_text(json.dumps({'historical_canonical_path_hash':historical_hash,'observed_canonical_path_hashes':strict_hashes,'canonical_hash_match':strict_hash_match,'trajectory_bytes_match':strict_path_bytes_match,'geometry_comparison':geometry_comparison,'canonical_hash_definition':'hash over every annotated point field except path_hash, including source_commit; trajectory coordinates separately compared byte for byte','row_count':len(l3rows),'strict_summary_present':strict is not None},indent=2))
audit_bad=[r for r in control if r['success'] and not r.get('outcome',{}).get('independent_audit',{}).get('valid')]+[r for r in dynamic if r['success'] and not r.get('outcome',{}).get('independent_audit',{}).get('valid')]
gates={**s['gates'],'controls_and_dynamic_accepted_unsafe_zero':not audit_bad,'dynamic_all_three_valid':len(dynamic)==3 and all(r['success'] for r in dynamic),'49_unit_tests':len(prov['unit_tests'])==49 and prov['all_unit_tests_passed'],'seven_original_core_files_unchanged':all(x['equal'] for x in prov['frozen_r1_core'].values()),'original_662_files_unchanged':prov['original_source_unchanged'],'frozen_inputs_unchanged':all(x['equal'] for x in prov['frozen_inputs'].values()),'frozen_r1_strict_pass':bool(strict and strict.get('strict_ab_pass')),'frozen_r1_historical_path_bytes':strict_path_bytes_match}
(P/'acceptance_results.json').write_text(json.dumps({'gates':gates,'all_pass':all(gates.values()),'performance_is_not_pass_gate':True},ensure_ascii=False,indent=2))
rr=retention['rows'];den=sum(r['reference_preferred_matched_arc_m'] for r in rr);kept=sum(r['retained_preferred_arc_m'] for r in rr);conditional=kept/den if den else None
valid_cond=[r['conditional_retention'] for r in rr if r['conditional_retention'] is not None]
base=Path('/home/robot/pudu_robot_ws');corepath=W/'external/arena4_ws/src/arena/three_d_v1/arena_3d_v1'
lines=[f'# 语义双地图与 L2 参考路径：隔离实现验收报告\n\n作者：李永祺；汇报对象：袁梦佳。日期：2026-09-14。',
f"最终版本真实地图实验完成 {len(real)}/240 次，成功 {sum(r['success'] for r in real)} 次；受控实验 {sum(r['success'] for r in control)}/{len(control)} 次成功；障碍增删实验 {sum(r['success'] for r in dynamic)}/{len(dynamic)} 次成功。验收门槛{'全部通过' if all(gates.values()) else '仍有未通过项，见下表'}。首轮只判断偏好、安全与可达性，不作提速结论。",
'## 1 冻结条件与统计口径',
'严格接入 3D-V1-r1。原七个核心源文件保持不变，新增模块显式继承 r1 控制器；默认生产 factory 没有切换。L1 为同源语义接口位姿图 A*，L2 为 compact D* Lite 与同代价 A* 回退，L3 为固定 Nav2 Smac Hybrid/DUBIN。没有以 r2、STL、骨架边加 cost 替代。',
'本轮是基于真实地图的离线路径规划实验，未进行实体机器人试跑。真实地图是万达三层现有人工标注，2,138 × 4,020 格，分辨率 0.05 m，原点 (−81.025999, −84.258104, 0)。20 组合法起终点、4 组算法、每组 3 次，共 240 次；16 组开发查询与 4 组预留查询预先固定。预留查询首次正式输入时发生 ROS 整数类型错误，修复只将数值转换为 float；因此最终重跑称为冻结查询复测，不宣称它们仍未暴露。旋转受控留出场景在权重冻结后首次使用，后续仅通用机制与传输修复，没有按场景调权重。',
'车体带 padding 半长/半宽为 0.265/0.225 m，最小转弯半径 0.40 m，禁倒车和原地旋转，48 个朝向档。位置容差 0.125 m、朝向容差 5°。Smac 规划 5 s，action 7 s；内容 ACK 每次 3 s；D* 增量 500 ms / 20,000 次扩展；新 L2 初始/回退 20 s、L1 查询 5 s、总查询防护 40 s。原 r1 初始与回退没有内建时限，不把新边界写成旧行为。',
'全局权重始终是 L2=2.0、L3 上限 140、2 m 饱和距离，低于预设最高 200；不按查询调整。四组分别为 original_r1（原图、无偏好）、A（新图、无偏好）、B（同新图、仅 L2 偏好）、C（同新图、L2 偏好与 L3 参考）。A/B/C 允许走廊相同，参考没有变成狭窄硬通道。',
'依据：'+', '.join(link(W/'config'/n) for n in ['acceptance.yaml','queries.yaml','FREEZE.json','SELECTED_WEIGHTS.json','IMPLEMENTATION_FREEZE_03.json'])+'；'+link(F/'protocol.json','实际加载库及实验协议')+'。',
'## 2 验收结论',table(['冻结门槛','结果'],[(k,'通过' if v else '**未通过**') for k,v in gates.items()]),
'所有数值只支持本真实地图、冻结车辆、冻结查询与受控场景；不代表所有地库、全部规则或生产上线验证。',
'## 3 同源地图、节点和连接',
'同一 SemanticMapV1 输入生成标签/硬障碍/安全栅格和位姿图。版本绑定包含实际占据内容、地图 YAML、语义、规则、坐标和车体；L2 与参考额外绑定有序路线、方向、权重、实际代价和动态版本。空白语义区仍可走，但不凭多边形顺序推断单行。',
'图的节点来自实际区域共享接口与长区域必要内部位姿。每个方向分别枚举并验证前进 Dubins 连接，保留合法平行候选；共享接口位置、朝向一致才拼接。禁转保留跨内部节点的语义历史，未知/超时连接从可用图排除。查询起终点作为临时节点，不永久改图。',
f"当前图 {prep['nodes']:,} 个节点、{prep['edges']:,} 条验证通过的有向连接、{prep['rejected_or_unverified']:,} 条拒绝或未验证记录。全图缓存 {prep['cache_bytes']/1024**3:.3f} GiB，仍然偏大；当前不是性能优化后的生产构图。来源：{link(R/'preparation_profile_01/summary.json')}。",
f"![实际语义和接口路线]({P/'figures/semantic_pose_graph.png'})",
'图：真实地图局部；橙色为已验证的 L1 连接，蓝色箭头为内部或临时端点位姿，紫色为接口位姿；浅蓝/米色/浅橙分别表示车道/停车区/交叉区。几何均使用地图米制坐标。图中未嵌入文字，原因见第 10 节。',
'## 4 偏好进入最终轨迹',
'目标带固定为到右侧安全边界的归一化距离 [0.125, 0.375]；左右安全边界内缩 0.50 m。按路线方向取右侧，首尾各排除 5 m，按 0.025 m 弧长积分并保留末尾不足一格的权重。受控有效长度要求至少 20 m、宽至少 4 m。左右对照要求方向正确且横向位移至少 0.50 m，禁止回退或绕圈堆高占比。',
f"![受控消融真实轨迹]({P/'figures/controlled_abc.png'})",
'图：40 × 6 m 受控直道，从上到下为 A、B、C；蓝色虚线为 L2，绿色为最终 L3，黄色为右侧目标带。黑圆为起点、黑菱形为终点。展示第 1 次重复，门槛使用全部重复。',
 table(['场景/重复','A 带内占比','B 带内占比','C 靠右占比','C 靠左占比','镜像横移 (m)'],[(f"{r['scene']} / {r['repetition']}",f"{r['A_band_ratio']:.1%}",f"{r['B_band_ratio']:.1%}",'/'.join(f'{v:.1%}' for v in r['C_right_band_ratios']),f"{r['C_left_band_ratio']:.1%}",num(r['mirror_signed_arc_lateral_shift_m'],3)) for r in s['controlled']]),
'表：来源 controlled_formal_04 全部 30 次；C 两个靠右值分别是切换前与切回后。受控首测 controlled_formal_01 的 30 次也完整保留。',
f"![左右与旋转对照]({P/'figures/controlled_mirror.png'})",
'图：左为 40 × 6 m 校准场景，右为 8 × 48 m 旋转场景；绿色靠右、紫色靠左。方向由路线决定。每次只改变参考策略，同一会话验证旧代价移除与新代价到达，全部切回轨迹哈希是否一致见 statistics.json。',
'参考代价实际写入服务器有效 master 栅格。冻结 Nav2 的 NodeHybrid::getTraversalCost 使用 child->getCost()/252，并乘 cost_penalty；障碍启发式同样消费该代价。精确 ACK 验证最终 master 的硬约束、软代价、旧值移除和版本，要求两次新鲜一致观测；相同完整键可复用先前确认。不是仅传参或可视化证据。',
'源码证据：'+link(base/'external/arena4_ws/src/deps/nav2/navigation2/nav2_smac_planner/src/node_hybrid.cpp','Smac 代价入口（304 行）')+'；'+link(corepath/'semantic_reference.py')+'；'+link(corepath/'semantic_pipeline.py')+'；'+link(C/'runs.jsonl','同会话策略切换与 ACK 原始记录')+'。',
'## 5 真实地图可达性与路径质量',
 table(['算法','成功/全部','端到端 P50/P95 (s)','峰值 RSS 最大 (GiB)','路径长度 P50/P95 (m)'],[(a,f"{v['success']}/{v['count']}",qpair(v['end_to_end_wall_ms'],1000),num(v['peak_process_tree_rss_bytes']['max']/1024**3,3),qpair(v['path_length_m'])) for a,v in s['by_arm'].items()]),
'表：real_formal_03，CPU 15，四组共享相同 Smac 会话，奇偶重复反转算法执行顺序。耗时包含成功与失败；每组 20 个独立查询 × 3 次相关重复，p95 只是样本描述，不是稳定的总体尾延迟估计。共享 L1 路线计算单独计时并计入每组端到端。',
f"原 r1 成功而 C 失败：{len(s['regressions'])} 次。报告为成功但独立检查失败：{len(s['accepted_invalid_trajectories'])} 次。失败明细：{link(P/'statistics.json')}。",
f"![真实地图的最终轨迹]({P/'figures/real_routes.png'})",
'图：依次为多路口、车道到停车区、停车区内部，均来自第 1 次 C；蓝色虚线 L2、绿色 L3，黑色障碍，白色可走。完整 240 次轨迹在 cases 中，不用代表图代替统计。',
f"真实场景 C 的 L2→L3 条件保持率按弧长汇总为 {num(None if conditional is None else 100*conditional)}%；分母 {den:.3f} m，是 L3 上能够无歧义匹配到 L2 目标带的弧长。分子为这些位置的 L3 仍处于目标带的弧长。{len(valid_cond)}/{len(rr)} 次有非零分母；其余不定义为 0% 或 100%。来源：{link(F/'derived/preference_retention.json')}。",
table(['算法','L2 带内比例 P50/P95 (%)','L3 带内比例 P50/P95 (%)','L3 有效样本'],[(a,qpair(x['l2_right_band_ratio'],.01),qpair(x['l3_right_band_ratio'],.01),x['l3_right_band_ratio']['n']) for a,x in s['by_arm'].items()]),
'表：真实复杂场景，仅计该路线可偏好且首尾各去掉 5 m 后的弧长；没有把停车区、路口和歧义区硬算为偏好失败。本表是逐轨迹的比例分位数，不将真实复杂场景套用受控直道门槛。',
'这不是 L3 总带内比例除以 L2 总带内比例；原始 reference_metrics 中的比值只作为补充，不冒称条件保持率。真实道路不要求全程达到受控直道的 80%，强制转弯与安全绕障可以偏离参考。',
 table(['算法','到 L2 的平均偏离 P50/P95 (m)','单路径偏离 P95 的跨查询 P50/P95 (m)'],[(a,qpair(v['reference_deviation_mean_m']),qpair(v['reference_deviation_path_p95_m'])) for a,v in s['by_arm'].items()]),
'表：L3 按 0.025 m 均匀弧长采样后取到 L2 栅格路径顶点的最短距离。不是到连续线段的精确距离；由 0.05 m 八连通栅格采样带来的距离高估上限约 0.0354 m。各查询全字段见原始 runs.jsonl。',
'## 6 障碍增加与恢复',
f"![受控动态障碍恢复]({P/'figures/dynamic_recovery.png'})",
'图从上到下为初始、障碍增加、障碍移除；同一个 40 × 6 m 受控场景。红色表示本次关闭单元，灰色表示原来不可走，绿色为最终轨迹。障碍位于原 L2 中段，连续两次观测确认，移除也连续两次确认。',
 table(['阶段','成功','端到端 (ms)','确认阻塞格数','L3 重评估','失败原因'],[(r['stage'],r['success'],num(r['wall_ms']),r.get('update',{}).get('blocked_count',0),r.get('update',{}).get('l3_required','初始调用'),r.get('failure_code','')) for r in dynamic]),
'表：dynamic_formal_01 单条控制路线的一次增加/移除序列。控制器先建立，initial 行只计 L3 请求与校验，不能当成冷启动；增删行包含两次观测应用、L2 更新/回退、参考与 L3 校验。它证明更新链路，不证明所有动态交通场景。完整 L2 增量/回退、版本与 ACK 见每阶段 result.json。参考单独改变的切换验证与本实验分开。',
'## 7 连续检查与机制证据',
'独立检查对轨迹每段按平移 ≤0.025 m、朝向 ≤1° 分段，以中点矩形加平移/旋转界构造保守连续包络，使用 SAT 与闭合占据格检测。未知区域、地图外及允许走廊之外视为障碍。额外验证前进、禁止原地旋转、曲率 ≤2.500001 m⁻¹、起终点及明确的语义规则。保留原 PathAudit，并且独立检查不能被复用的旧 audit 替代。',
'曲率使用航向变化对应的圆弧曲率 2 sin(|Δyaw|/2)/弦长，并交叉检查三点外接圆曲率。早期误用 Δyaw/弦长导致合法半径 0.40 m 的圆弧被拒绝，修复数学表达式后阈值保持不变，早期失败仍在记录中。',
f"最终自动检查 {len(prov['unit_tests'])} 项通过：原 r1 30 项、新机制 19 项。包括相同栅格的禁转与单向、内部节点和接口拼接、未验证连接排除、临时端点、平行/交叉/重复匹配歧义、D*/A* 同代价与增减、缓存污染与失效、连续平移/旋转车体检查。来源：{link(R/'final_unit_regression.xml')}、{link(corepath.parent/'test/test_semantic_dual_map.py')}。",
f"原 r1 严格回归完成状态：{bool(strict and strict.get('strict_ab_pass'))}；新轨迹 CSV 与历史逐字节一致：{strict_path_bytes_match}（{len(l3rows)} 次 L3）。历史综合审计哈希 {historical_hash}，当前 {strict_hashes}，二者不同；综合哈希包含 source_commit 等运行来源字段，不是纯轨迹哈希。本次没有使用浮点容差或事后等价阈值，六份轨迹输出要求精确字节一致。来源：{link(strictpath)} 与 {link(P/'strict_historical_comparison.json')}。历史四倍地图的内存门槛已失败，本轮不把它写成已通过，也不将历史 STL 数字计入本方案。",
'## 8 完整开销与缓存',
 table(['阶段','墙钟 (s)','CPU (s)','峰值进程 RSS (GiB)'],[(r['phase'],num(r['wall_ms']/1000),num(r['cpu_ms']/1000),num(r['peak_rss_bytes']/1024**3,3)) for r in timing]),
'表：preparation_profile_01，从空缓存开始；连接验证与序列化完整计入。复用现有人工标注，历史人工标注工时未测量；导入、栅格生成、节点生成、连接验证、序列化与缓存恢复分别计数。恢复在同一进程中测量，峰值 RSS 可包含分配器保留页。',
 table(['算法','冷 L2 缓存首轮 P50/P95 (s)','后续重复 P50/P95 (s)','状态命中次数','几何命中次数'],[(a,qpair(s['by_arm_cold_first_repeat'][a]['end_to_end_wall_ms'],1000),qpair(s['by_arm_later_repeats'][a]['end_to_end_wall_ms'],1000),s['by_arm'][a]['state_cache_hits'],s['by_arm'][a]['geometry_cache_hits']) for a in s['by_arm']]),
'表：每个算法独立空 L2 缓存启动，首轮是查询状态冷启动；它不是每条查询都重启 ROS 进程。ROS 首次启动单独见 protocol.json。L1 图已恢复，其离线构建不得算作免费。RSS 使用 20 ms 采样，进程树 RSS 求和可能重复计共享页，不能当成独占物理内存。',
 table(['分层墙钟 P50/P95 (ms)','原 r1','A','B','C'],[(label,*[qpair(s['by_arm'][a][key]) for a in s['by_arm']]) for label,key in [('L1','l1_wall_ms'),('L2 含偏好','l2_including_preference_wall_ms'),('方向偏好场','preference_field_wall_ms'),('参考代价适配','reference_adapter_wall_ms'),('全部内容更新','all_content_update_wall_ms'),('其中 ACK 等待','content_ack_wall_ms'),('Smac 搜索+平滑','smac_search_plus_smoothing_wall_ms'),('独立检查','independent_audit_wall_ms')]]),
table(['分层 CPU P50/P95 (ms)','原 r1','A','B','C'],[(label,*[qpair(s['by_arm'][a][key]) for a in s['by_arm']]) for label,key in [('L1','l1_cpu_ms'),('L2 含偏好','l2_including_preference_cpu_ms'),('参考适配','reference_adapter_cpu_ms'),('Smac 进程','smac_process_cpu_ms'),('独立检查','independent_audit_cpu_ms'),('全查询进程树','end_to_end_process_tree_cpu_ms')]]),
'表：分项存在包含关系，不能把所有行再相加。冻结后端不暴露搜索与平滑的独立时间，按合计报告，不虚构拆分。L1、L2、适配、审计 CPU 与进程树 CPU 全字段在 statistics.json，Smac CPU 来自进程采样。',
f"主图完整缓存 {prep['cache_bytes']/1024**3:.3f} GiB；正式四组 L2 缓存 {s['formal_l2_cache_bytes']/1024**2:.2f} MiB；正式运行中 {s['ack_attempt_failures']} 次初始内容确认失败，均保留原始确认日志；是否最终成功以每行结果为准。受控复测另有一次启动读取失败，零完成样本，记录在 controlled_formal_02；改用每场景独立 ROS 通信域后重新测量，不能宣称启动可靠性已全面解决。旧 full replay 仅在精确内容最终一致后继续，超时预算没有放宽。",
 table(['最慢查询','算法/重复','总墙钟 (s)','成功'],[(r['query_id'],f"{r['arm']}/{r['repetition']}",num(r['end_to_end_wall_ms']/1000),r['success']) for r in s['worst_cases'][:5]]),
table(['算法','L2 状态+几何最大 (MiB)','重新初始化','回退','缓存拒绝','活动状态命中'],[(a,num(x['l2_state_memory_bytes']['max']/1024**2),x['l2_reinitializations'],x['l2_fallbacks'],x['cache_invalidations'],x['active_cache_hits']) for a,x in s['by_arm'].items()]),
'表：正式静态查询的缓存与 L2 计数；缺失缓存首次构建可记为拒绝原因，实际字段见 statistics.json。每次请求后清理活动控制器，所以活动状态命中为零是实验设置；后续重复从磁盘状态恢复。版本变化与损坏由单独机制测试验证。L2 对象占用不包括共享全图、偏好场及进程库，整体内存以进程树 RSS 单列。',
'下一阶段优先研究连接路径序列化体积、重复方向场/参考栅格计算和大图传输。先按这些实测结果冻结性能目标，再优化；本轮没有速度提升门槛，也没有承诺百分比或毫秒级改善。',
'## 9 全部尝试与失败',
'各版本和诊断条件不同，以下分别报告分母，不混成一个总体成功率。被中止的进行中个例不冒充完成样本，目录和中止原因仍保留。没有将旧失败删除后宣称整个研发过程零失败。',
 table(['目录/实验','成功/完成','状态或失败类别'],[(a['attempt'],f"{a['success']}/{a['count']}",('中止；' if 'interruption' in a else '')+('; '.join(f'{k}: {v}' for k,v in a.get('failure_codes',{}).items()) or a.get('status','已记录'))) for a in attempts]),
'初期大图完整发布触发内容 ACK 失败；独立只读检查确认实际 Fast DDS 默认共享内存容量与额外 localhost 通道问题，隔离配置改用足够容量及显式本机传输，再进行 36 次压力复测。图接入遗漏的长停车区域通过必要内部节点修复。正式第 1 轮在预留查询前停止；第 2 轮记录整数坐标错误后停止；第 3 轮完整重跑，权重与所有验收阈值未变。',
'受控 controlled_formal_03 的校准 15 次通过，旋转场景因通信域 233 超出当前 DDS 端口范围而未启动；改为 220/221 后 controlled_formal_04 完整 30 次通过。它是实验配置错误，未改变求解算法。派生指标第一次计算因整数/浮点 JSON 表达导致路线签名不一致而拒绝继续，按正式 runner 的浮点转换重算后全部 60 条 C 路线签名一致。',
'来源：'+link(P/'attempt_manifest.json')+'；'+link(R/'transport_sources.md')+'；每次 runs.jsonl、outcome.json、exception.json、interruption.json 与 console.log。搜索失败、超时和未验证可行分别保留，不解释成物理不可达。',
'## 10 交付边界和复现入口',
'遗留问题：只有一张真实主图，未证实跨地图泛化；真实来源没有明确单行，14 条 fence 的硬约束属性待确认，不能用本次结果证明这些未知规则。图与序列化缓存偏大；数值门槛通过不等于生产性能和动态交通验证完成。当前车体固定，配置变化必须创建匹配的新 L3 会话。',
'未找到 SimSun 字体；按工作区约束未采用替代字体，所有证据图均不含内嵌文字，尺寸、方向、图例、数值与来源写在正文。原 r1 回归的可选绘图因同一原因省略，数值计算未改。',
f"原源文件复核 {prov['original_manifest_files_checked']} 个，内容变化 {len(prov['original_changed_files'])} 个；原七个核心文件与隔离副本完全一致。原工作区没有合并、清理、重置或提交。本交付未向导师发消息、未发布正式软件版本。来源：{link(P/'provenance_verification.json')}。",
'复现说明：'+link(W/'README.md')+'；输入示例：'+link(W/'data/calibration/semantic.json')+'；原始输出：'+link(F)+'、'+link(C)+'、'+link(D)+'；初始核对：'+link(Path('/home/robot/reports/SEMANTIC_DUAL_MAP_PHASE0_20260914/phase0_report.md'))+'。',
'方案依据：[主文档第 1 节](https://pudutech.feishu.cn/docx/QSYzd4kcmoVlktx5Co6caOhTnab)；[节点与连接详细设计](https://pudutech.feishu.cn/docx/YSazdsoP5oTzy4xePtpcJ4D1nAe)。主文档历史 STL 小节不是本方案验证证据。']
# Remove accidental literal brace after Markdown media path, if any.
text='\n\n'.join(lines).replace('png})','png)')+'\n';(P/'full_report.md').write_text(text)
summary={'gates':gates,'real_success':sum(r['success'] for r in real),'controlled_success':sum(r['success'] for r in control),'dynamic_success':sum(r['success'] for r in dynamic),'retention_weighted':conditional,'retention_defined_n':len(valid_cond),'retention_n':len(rr),'prep':prep,'preparation_timings':timing,'strict_path_bytes_match':strict_path_bytes_match,'strict_canonical_hash_match':strict_hash_match}
(P/'report_values.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print(json.dumps(summary['gates'],ensure_ascii=False))
