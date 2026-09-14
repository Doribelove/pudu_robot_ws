"""Render the measured result without inventing unmeasured speedups."""
import json,hashlib
from pathlib import Path
from collections import Counter
from arena_3d_v1.semantic_world import WORK
S=json.loads((WORK/'report/performance.json').read_text());arms=['baseline','topology_005','topology_015'];names={'baseline':'当前 3YD-V0-r0','topology_005':'轻量 L1＋0.05 m L2','topology_015':'轻量 L1＋0.15 m L2'}
def num(x,d=3):return '未测得' if x is None else f'{x:,.{d}f}'
def pair(v,scale=1.,digits=3):return num(v['p50']/scale,digits)+' / '+num(v['p95']/scale,digits) if v['p50'] is not None else '未测得'
def table(header,rows):return '\n'.join(['| '+' | '.join(header)+' |','| '+' | '.join(['---']*len(header))+' |',*['| '+' | '.join(map(str,r))+' |' for r in rows]])
def absolute(label,p):return f'[{label}]({WORK/p})'
A=S['arms'];out=['# 3YD 静态多分辨率改造与性能对照','',
'基线名称为 **3YD-V0-r0**。新实现是独立静态实验，未另行指定正式版本号。本文数据来自本轮同条件复测，不将此前报告的数据直接拼接为速度对比。','',
'## 已实现的链路','',
'topology → 0.15 m 参考规划 → 世界坐标参考线 → 0.05 m 软代价输入 → 原 Smac → 双重轨迹检查。','',
'L1 用语义子区域和有向接口做按需搜索，取消成对 Dubins 连接及详细轨迹预计算。L2 保守聚合原精细障碍/安全可通行性，保留方向规则，生成偏好参考线。粗图端点不可表示、搜索无路或超时时可切到所选路线区域的 0.05 m L2；完整回退成本计入查询。','',
'L3 原始占据图、Smac、车辆和规划参数保持不变；新的输入仍通过原有软参考适配、实际代价内容确认、原 PathAudit 及独立连续车体检查。','',
'接口定义见 '+absolute('INTERFACES.md','INTERFACES.md')+'。','',
'## 查询性能','',
'相同真实地图、20 条冻结查询、每条 3 次，三组独立进程在 CPU 15 顺序运行。下表耗时单位为秒，单元格为 **P50 / P95**。所有查询均保留在成功分母和总耗时统计中。','',
table(['指标',*[names[a] for a in arms]],[
 ['成功 / 总次数',*[f"{A[a]['success']} / {A[a]['count']}" for a in arms]],
 ['查询总耗时',*[pair(A[a]['all']['wall_ms'],1000) for a in arms]],
 ['首次查询总耗时（20 次）',*[pair(A[a]['cold']['wall_ms'],1000) for a in arms]],
 ['重复查询总耗时（40 次）',*[pair(A[a]['warm']['wall_ms'],1000) for a in arms]],
 ['L1 路线请求',*[pair(A[a]['all']['l1_ms'],1000,5) for a in arms]],
 ['首次 L2 初始化',*[pair(A[a]['cold']['l2_ms'],1000) for a in arms]],
 ['重复 L2 初始化',*[pair(A[a]['warm']['l2_ms'],1000) for a in arms]],
 ['L3 参考代价适配',*[pair(A[a]['reference_adapter_ms'],1000) for a in arms]],
 ['地图更新与内容确认',*[pair(A[a]['l3']['local_map_update_ms'],1000) for a in arms]],
 ['Smac 搜索与平滑',*[pair(A[a]['l3']['l3_planning_time_ms'],1000) for a in arms]],
 ['地图内容校验后重发修复次数',*[str(A[a]['map_update_repair_count']) for a in arms]],
 ['进程树 CPU 时间',*[pair(A[a]['all']['process_tree_cpu_ms'],1000) for a in arms]],
 ]),'',
'首次查询指本查询的 L2 状态和参考缓存尚未命中，L1 图已加载；它不代表操作系统冷启动。查询计时包含 L1、L2、L3 和轨迹检查，路径质量计算、轨迹与统计文件落盘发生在计时之后。原有内容校验的重发修复及其耗时均保留，因此部分长耗时也来自地图更新。各阶段分位数不能直接相加。',
'新接口将精细区域通道的生成计入 L2 初始化，而基线在 L1 中生成走廊。因此 L1 分项不能单独用作整条链路提速倍数，主结论依据完整查询计时。','']
b=A['baseline'];n=A['topology_015']
for label,part in [('全部查询','all'),('首次查询','cold'),('重复查询','warm')]:
 before=b[part]['wall_ms']['p50'];after=n[part]['wall_ms']['p50'];out.append(f"{label}的总耗时中位数从 {before/1000:.3f} s 变为 {after/1000:.3f} s，变化 {(after/before-1)*100:+.1f}%；中位数之比为 {before/after:.2f} 倍。")
out+=['','`topology_005` 同时包含移除 Dubins 引导后新增的栅格引导路与偏好缓存，因此它与基线的差值代表轻量静态链路整体改造的收益。两个新接口分组之间才用于观察 L2 分辨率的影响。','',
'## 粗图回退与实际收益边界','',
f"0.15 m 分组中，{n['count']-n['fallback_count']} / {n['count']} 次直接使用粗图，{n['fallback_count']} / {n['count']} 次回退到精细 L2；涉及 {len(n['fallback_queries'])} / 20 条不同查询。",'',
'回退查询：'+('、'.join(n['fallback_queries']) or '无')+'。','',
'当前回退在整个选定路线区域使用 0.05 m L2，尚未压缩成局部精细桥接。它能保留窄连接的可达性，也会保留部分原有搜索成本。所有回退与先行失败搜索都已计时。','',
'同面积总格数从 **8,594,760** 降至 **955,420**；每类同型密集 L2 数组约减少 **88.9%**。L3 全图、软代价适配、地图内容确认及服务内存仍存在，整机收益不按九分之一推算。','']
co=S['paired']['coarse_only_cold'];out += [f"在 {co['n']} 条无精细回退的首次查询中，同一新接口的 L2 中位数为：0.05 m {co['fine_l2_ms']['p50']/1000:.3f} s，0.15 m {co['coarse_l2_ms']['p50']/1000:.3f} s。",'',
'## 构图与内存','',
'构图使用空的应用缓存；恢复使用另一个新进程。文件系统页缓存没有人为清空。两种图存储的对象不同：基线是已验证车辆连接，新图是待后续验证的有向语义接口。','']
prep=[]
for a in arms:
 d=S['preparation'][a];build=d['build'];restore=d['restore'];br=build['rows'];rr=restore['rows'];graph=lambda rows:sum(x['wall_ms'] for x in rows if x['phase'].startswith('graph_'))
 prep.append([names[a],num(graph(br)/1000),num(graph(rr)/1000),num(build['graph_cache_bytes']/1024**2),num(max(x['rss_peak_bytes'] for x in br)/1024**3),str(build['graph_nodes'])+' / '+str(build['graph_edges'])])
out += [table(['分组','L1 构建与落盘 s','L1 缓存恢复 s','图缓存 MiB','准备阶段峰值 RSS GiB','节点 / 连接或接口'],prep),'',
table(['资源指标',*[names[a] for a in arms]],[
 ['查询进程树峰值 RSS（GiB）',*[num(A[a]['all']['rss_peak_bytes']['max']/1024**3) for a in arms]],
 ['查询结束进程树 PSS 最大值（GiB）',*[num(A[a]['all']['post_query_tree_pss_bytes']['max']/1024**3) for a in arms]],
 ['Smac 进程峰值 RSS（GiB）',*[num(A[a]['l3']['planner_rss_peak_bytes']['max']/1024**3) for a in arms]],
 ['L2 状态＋几何最大值（MiB）',*[num(A[a]['l2_state_bytes']['max']/1024**2) for a in arms]]
 ]),'','进程树 RSS 按约 20 ms 间隔采样，会重复计入共享页；PSS 按比例分摊共享页。此处 PSS 是每次查询结束的采样最大值，不是连续峰值。L2 状态项不包含全图、偏好场、L3 与 ROS 服务。',
'精细回退时，新拓扑区域通道比基线的轨迹走廊更宽，L2 状态最大值反而增加；总进程内存的下降主要来自移除旧图的详细连接数据，不能解释为所有内存项都减少。','',
'## 轨迹与机制检查','',
table(['指标',*[names[a] for a in arms]],[
 ['被接收但独立检查不通过',*[str(A[a]['accepted_invalid']) for a in arms]],
 ['路径长度 P50 / P95（m）',*[pair(A[a]['all']['length_m']) for a in arms]],
 ['公共细图靠右目标带占比 P50 / P95',*[pair(A[a]['right_band_ratio'],.01,1)+'%' for a in arms]],
 ['参与偏好统计的轨迹次数',*[str(A[a]['right_band_ratio']['n']) for a in arms]],
 ['去除两端后有效语义长度 P50（m）',*[num(A[a]['eligible_arc_m']['p50']) for a in arms]]
 ]),'',
'真实地图偏好统一用基线的 0.05 m 语义参考评估场衡量，按 0.025 m 弧长采样，排除两端各 5 m 与歧义/不适用区域。各组使用同一评价定义，并报告有效长度；不同路线的数值不能替代受控偏好因果验证。目标带为安全归一化横向区间 [0.125, 0.375]。','',
f"新粗图组与基线成对比较，路径长度比中位数为 {S['paired']['topology_015']['length_ratio']['p50']:.4f}，P95 为 {S['paired']['topology_015']['length_ratio']['p95']:.4f}，最大为 {S['paired']['topology_015']['length_ratio']['max']:.4f}；部分查询的路径增加约 9.5%，因此成功率与速度改善并不代表每条路线都更短。",'',
f"两个新接口分组的精细硬通道哈希在 {S['paired']['new_arms_same_hard_corridor']} / 60 个对应查询上相同。",'']
controls=S['controls'];ct=[]
for scene in ['calibration','rotated']:
 for a in arms:
  rows=[x for x in controls if x['scene']==scene and x['arm']==a];ratios=[r.get('target_band_ratio',0) for r in rows];same=[]
  for rep in [1,2,3]:
   group=[r for r in rows if r.get('repetition')==rep];same.append(len({r.get('hard_corridor_hash') for r in group})==1)
  ct.append([scene,names[a],f"{sum(r['success'] for r in rows)}/{len(rows)}",num(min(ratios)*100,1)+'%' if ratios else '缺失',str(all(same))])
out += [table(['受控场景','分组','成功次数','左右目标带最低占比','切换期间硬通道一致'],ct),'',
f"窄通道最终 L3 轨迹通过 {sum(r['success'] for r in S['narrow'])} / {len(S['narrow'])} 次，全部保留精细回退原因及连续车体检查记录。最终回归测试通过 51 项，覆盖坐标、边界补齐、保守障碍、方向/禁转、缓存完整性、加权搜索一致性和窄通道回退。",'',
'开发试跑曾发现语义边界截断车体转弯余量，独立检查拒绝了这些结果；随后统一改为由车辆与最小转弯半径计算 0.85 m 区域余量。开发失败记录保留在 `real_pilot_01`、`turn_fix_probe_01`，正式数据来自冻结后运行，未删除失败重算分母。','',
'最终跨进程恢复检查还发现区域字典整数键经 JSON 读取后排序不同，导致缓存完整性误报。修正仅位于拓扑构造函数的缓存读取分支，正式查询组在空图缓存上构图，没有执行该分支；查询、通道生成与 L2/L3 算法未改变。修复后重新测量两组新图的构建/恢复，并对缓存恢复后的两组各 20 条查询逐一核对：拓扑签名、精细硬通道、L2 参考点列与实际分辨率均和原计时样本相同。详情见 '+absolute('缓存恢复一致性验证','report/cache_restore_validation.json')+'。','',
'## 后续优先事项','',
'下一步优先把停车区的完整精细回退改成局部精细连接，减少剩余冷查询开销；再接入公共动态障碍事件、L2/L3 同源更新与版本确认。L1 通道受阻状态、安全切换位置以及失败后自动尝试其他拓扑路线尚未在本轮实现。','',
'本轮结果证明这些固定静态查询和受控场景中的链路与偏好响应，不代表动态避障、任意起终点完备性或实机控制验收。','',
'## 可追溯文件','',
'- '+absolute('原始机器可读统计','report/performance.json'),
'- '+absolute('逐轨迹偏好数据','report/path_quality.json'),
'- '+absolute('正式运行脚本','tools/run_static_formal.bash'),
'- '+absolute('正式查询冻结哈希','report/IMPLEMENTATION_FREEZE.json'),
'- '+absolute('缓存修正后的交付哈希','report/FINAL_IMPLEMENTATION_FREEZE.json'),
'- '+absolute('源文件与输入一致性核查','report/integrity.json'),
'- '+absolute('最终回归测试结果','report/regression_cache_fix.xml'),
'- 三组正式逐次记录位于 `results/formal_01_baseline`、`results/formal_01_topology_005`、`results/formal_01_topology_015`。','']
(WORK/'report/performance.md').write_text('\n'.join(out))
print(WORK/'report/performance.md')
