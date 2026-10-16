"""Render the measured candidate report; no frozen parent writes."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def num(x,n=3):return '未测得' if x is None else f'{x:.{n}f}'
def main():
 c=json.loads((ROOT/'report/COMPARISON.json').read_text());lines=['# 3YD-V8-r8：首次完整规划与变化起点重规划','',
 '本轮从新 V8-r7 完整路线分支独立开展。原 r7、r6 主版本、封存 L1 和历史生产入口保留；不采用旧移动窗口方案。所有时间均为当前调用实际耗时，初始化、启动和导出单独记录，不计入查询。','',
 '## 结果概览','',
 '两张原固定地图，各 10 组 × 3 次；对照和优化版都完成 60/60 严格成功。L1、L2、软目标和最终轨迹全部逐点一致。优化改变计算方式，未修改障碍、车辆、交通、搜索范围或终验要求。','',
 '| 地图 | 指标 | r7 同批对照 | r8 AB | 变化 |','|---|---|---:|---:|---:|']
 for mid,m in c['results']['fixed'].items():
  for metric,subset,label,v in [('e2e_s','first','首次完整规划中位 / s','median'),('e2e_s','first','首次完整规划 P95 / s','p95'),('e2e_s','repeat','相同起终点重复中位 / s','median'),('e2e_s','all','全部查询 P95 / s','p95'),('tree_rss_gib','all','查询峰值 RSS 中位 / GiB','median'),('tree_rss_gib','all','查询峰值 RSS 最大 / GiB','max')]:
   a=m['arms']['control'][subset]['metrics'][metric][v];b=m['arms']['candidate'][subset]['metrics'][metric][v];p=m['change_percent'][subset][metric][v]
   lines.append(f"| {m['label']} | {label} | {num(a)} | {num(b)} | {p:+.1f}% |")
 lines+=['','![固定集逐次耗时和资源箱线图](fixed_boxplots.png)','',
 '首次中位数每图 n=10，重复 n=20；全部查询 n=30。这里的首次指每条不同路线首次请求，地图及进程已初始化；不是只有 Q01，也不是重复执行地图加载。百分比负值表示降低。','',
 '## 各层实际耗时与内存','',
 '| 地图 | 首次查询指标 | r7 中位 / P95 | r8 中位 / P95 |','|---|---|---:|---:|']
 for mid,m in c['results']['fixed'].items():
  for metric,label in [('l1_ms','L1 / ms'),('l2_s','L2 全部候选准备 / s'),('dstar_s','L2 各候选当前搜索累计 / s'),('adapter_s','层间适配 / s'),('l3_s','L3 全部候选流水线 / s'),('map_sync_s','地图同步及 ACK / s'),('smac_action_s','Smac action / s'),('curvature_repair_s','曲率及局部几何修复 / s'),('strict_audit_s','登记的严格终验 / s')]:
   a=m['arms']['control']['first']['metrics'][metric];b=m['arms']['candidate']['first']['metrics'][metric]
   lines.append(f"| {m['label']} | {label} | {num(a['median'])} / {num(a['p95'])} | {num(b['median'])} / {num(b['p95'])} |")
 lines+=['','L2 为所有候选的控制器准备时间。L3 包含层间适配、地图同步、搜索、修复与终验；分项有嵌套，不能相加各行中位数得到端到端。分段车辆规划的部分子调用未被顶层 Smac/修复子时钟完整覆盖，以 L3 总流水线为完整口径。','',
 '| 地图 | 内存指标（全部固定查询） | r7 中位 / 最大 | r8 中位 / 最大 |','|---|---|---:|---:|']
 for mid,m in c['results']['fixed'].items():
  for metric,label in [('l2_state_mib','L2 登记状态 / MiB'),('planner_pss_gib','planner PSS / GiB'),('stack_pss_gib','登记 ROS 栈 PSS / GiB'),('python_pss_gib','导出后 Python PSS / GiB'),('tree_rss_gib','查询进程树峰值 RSS / GiB')]:
   a=m['arms']['control']['all']['metrics'][metric];b=m['arms']['candidate']['all']['metrics'][metric]
   lines.append(f"| {m['label']} | {label} | {num(a['median'])} / {num(a['max'])} | {num(b['median'])} / {num(b['max'])} |")
 lines+=['','L1 没有可独立归属的完整内存采样；Python 同时持有 L1/L2、地图、缓存及部分 L3 处理数据。L2 登记状态不是 L2 所有内存。登记 ROS 栈为 planner、map_server、lifecycle_manager，已经包含 planner。PSS 与进程树 RSS、导出后快照与查询峰值不能相加。缓存命中不重新测车辆搜索组件峰值，组件表仅统计实际有记录的样本，样本数在 COMPARISON.json 中。','',
 '## 目标固定、起点沿路线变化','',
 '每条父路线使用同一目标，依次请求原起点、原已验轨迹累计弧长约 20%、50%、80% 处的现有位置和朝向。控制与候选共用 r7 保存轨迹生成的数据，不根据新结果换点。每图 10 次初始请求与 30 次变化起点请求，分别统计。','',
 '| 地图 | 阶段 | r7 严格通过 | r8 严格通过 | r7 中位 / P95（s） | r8 中位 / P95（s） | 中位变化 |','|---|---|---:|---:|---:|---:|---:|']
 for mid,m in c['results']['moving'].items():
  for subset,label in [('initial','10 次初始请求'),('moving','30 次起点变化')]:
   a=m['arms']['control'][subset];b=m['arms']['candidate'][subset];x=a['metrics']['e2e_s'];y=b['metrics']['e2e_s']
   p=m['change_percent'][subset]['e2e_s']['median']
   lines.append(f"| {m['label']} | {label} | {a['success']}/{a['n']} | {b['success']}/{b['n']} | {num(x['median'])} / {num(x['p95'])} | {num(y['median'])} / {num(y['p95'])} | {p:+.1f}% |")
 lines+=['','![变化起点逐次耗时和资源箱线图](moving_boxplots.png)','',
 '上表和箱线图包含失败请求耗时。为避免快速失败拉低中位数，再单列成功请求与内存：','',
 '| 地图 | 成功重规划 r7→r8 中位 / P95（s） | r7→r8 RSS 中位（GiB） | r7→r8 RSS 最大（GiB） | 整轨迹精确缓存命中（r8） |','|---|---:|---:|---:|---:|']
 for mid,m in c['results']['moving'].items():
  a=m['arms']['control']['moving'];b=m['arms']['candidate']['moving'];x=a['success_e2e_s'];y=b['success_e2e_s'];rm=a['metrics']['tree_rss_gib'];rn=b['metrics']['tree_rss_gib']
  lines.append(f"| {m['label']} | {num(x['median'])}/{num(x['p95'])} → {num(y['median'])}/{num(y['p95'])} | {num(rm['median'])} → {num(rn['median'])} | {num(rm['max'])} → {num(rn['max'])} | {b['exact_route_hits']}/{b['n']} |")
 lines+=['','### 新发现的两个失效点','',
 '| 查询 | 首个失效层级 | 已核实原因 |','|---|---|---|',
 '| Q09，50% | L1 | 当前点仍在车道2，旧终点接触点已在车后约 7.84 m；重新发起的 L1 请求被终点接入方向规则拒绝，额外按最多三条候选诊断仍无合法路线。 |',
 '| Q09，80% | L1 | 当前点已在车位区3。现有绑定描述 Q09 的终点归属，没有该位置作为当前起点的停车归属，因此在选路前拒绝。 |','',
 '两版失败一致；没有进入 L2/L3，不能归因为 L3 搜索慢或障碍导致物理不可达。本轮保留冻结的 L1 和原绑定，没有改点、扩大搜索或放宽规则。详细诊断见 `Q09_L1_DIAGNOSIS/diagnosis.json`。','',
 '本测试是静止规划回放，尚未模拟跟踪误差和实时执行。后续请求仍取固定父轨迹，未沿每次新规划结果运行控制器。不能将测试结果写成连续导航闭环已通过。','',
 '## 本轮改动与剩余瓶颈','',
 '1. 停车接入膨胀只计算有效像素包围范围及完整核半径。空种子直接得到空结果；局部结果与原全图膨胀逐格一致。','2. L2 紧凑状态坐标用数组计算；地图变化范围用行列统计，避免为数百万变化格分配逐格坐标。','3. 同一次 ACK 等待只计算一次硬障碍、软代价、旧内容等预期分类。每份实际读回仍逐格比较，并核验实际内容、发布序列和时间戳。临时分类占一字节/格，等待结束即释放。','',
 '本轮没有将查询工作移到未计时的初始化，也未提前输入实验答案。L3 仍要计算完整路线，在不同起点下整轨迹缓存没有直接命中。进一步降低重规划成本，应先支持任务上下文中的道路经过阶段与停车接入，再尝试验证和复用仍有效的剩余完整路线；真正受影响或偏离原路线时才有界重算。首次规划的剩余瓶颈仍包括 L3 搜索、曲率连接修复及地图同步。','',
 '## 验证与复现','',
 '- 190 项测试通过，新增检查覆盖局部膨胀边界、空种子、紧凑坐标、变化范围、逐类 ACK 等价与篡改读回、新起点数据一致性。','- 固定集 L1、L2、软目标和最终轨迹 60/60 相同；各变化起点输出对比见 COMPARISON.json。','- 封存 L1、原输入、Smac/原生 L2、原独立终验保持。父版本登记文件未改。','- 两种模式各 11 个障碍增删/乱序/清除确认回放事件均满足原预期，端点封堵正确拒绝、清除后恢复；这属于功能回归，不是连续实车验证。','- 所有批次进程和共享内存清理证据在各目录相邻的 `.supervisor.json`。','',
 '协议：`../EXPERIMENT_PROTOCOL.md`；接口：`../INTERFACES.md`；代码：`../src/three_yd_v5/firstplan.py`、`first_ack.py`；原始逐次记录：`../results/fixed/` 与 `../results/moving/`；新数据集：`../datasets/moving_start_v1/`。','',
 '观察值来自同机串行的独立对照批次，不声称随机化统计显著性。箱线图使用逐次原始记录，宋体；箱体 Q1–Q3、须为 1.5IQR、显示离群点；不把成功率画成箱线图。','']
 (ROOT/'report/RESULTS.md').write_text('\n'.join(lines))
if __name__=='__main__':main()
