"""Render validated performance JSON to the review report."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
data=json.loads((ROOT/'report/PERFORMANCE.json').read_text())
a=data['r8_isolated_10x3'];b=data['r9_balanced_10x3']
def v(d,scope,metric,stat='median'):return d[scope][metric][stat]
def change(x,y):return (y/x-1)*100 if x else 0.
def table_row(label,scope,metric,stat='median',unit='s'):
 x=v(a,scope,metric,stat);y=v(b,scope,metric,stat)
 return f'| {label} | {x:.3f} {unit} | {y:.3f} {unit} | {change(x,y):+.1f}% |'
lines=['# 3YD-V5-r9 实验结果与缓存计时核实（2026-09-22）','',
 f"固定 Q01–Q10 各重复三次：r8 同环境对照 {a['success']}/30，r9 {b['success']}/30。r9 从冻结 r8 独立构建，仅优化计算、缓存和内存配置；导航规则与终验保持原样。",'',
 '## 先更正计时解释','',
 f"r8 的 `adapter_ms` 存在于缓存对象中，命中后仍返回历史创建时间，原先“每次固定约 0.52 秒”的解释错误。本轮直接测量：对照的缓存命中实际中位 {a['cache_hit_adapter_ms']['median']:.2f} ms，未命中完整合成中位 {a['cache_miss_adapter_ms']['median']:.2f} ms。r9 对应为 {b['cache_hit_adapter_ms']['median']:.2f} ms 和 {b['cache_miss_adapter_ms']['median']:.2f} ms。",'',
 '本次同时区分 L2 历史搜索计时与本次查询计时，记录所有候选（包括失败候选）及完整 L3 流程。L3 的 Smac 动作不包含后续局部曲率修复；不能再把 0.3 秒动作时间视为完整 L3 时间。', '',
 '## 正式对照','',
 '两批使用相同 CPU 12、固定地图/车辆/查询和预算，串行运行，不启用 cProfile。r8 对照仅补计时，r9 开启优化。原 2026-09-21 r8 30/30 结果保留，未用不同日期/核心的历史时间充当本轮直接性能对照。', '',
 '| 指标 | r8 同环境复测 | r9 | 变化 |','|---|---:|---:|---:|',f"| 严格成功 | {a['success']}/30 | {b['success']}/30 | — |",
 table_row('端到端中位','all','wall_s'),table_row('端到端 P95','all','wall_s','p95'),table_row('最大耗时','all','wall_s','max'),
 table_row('首次查询中位','first','wall_s'),table_row('重复查询中位','warm','wall_s'),table_row('重复查询 P95','warm','wall_s','p95'),
 table_row('查询树 RSS 中位','all','rss_gib',unit='GiB'),table_row('查询树 RSS 峰值','all','rss_gib','max','GiB'),
 table_row('靠右比例中位','all','right_fraction',unit='比例'),table_row('长度中位','all','length_m',unit='m'),'',
 '## 各层实际耗时','',
 '| 阶段 | r8 首次中位 | r9 首次中位 | r8 重复中位 | r9 重复中位 |','|---|---:|---:|---:|---:|']
for label,metric,unit in [('L1 选路','l1_ms','ms'),('全部候选 L2/控制器准备','controller_all_s','s'),('L2→L3 适配','adapter_all_ms','ms'),('L3 完整流程（包含适配）','l3_pipeline_all_s','s'),('其中地图更新/ACK','map_update_all_s','s'),('其中车辆规划动作','l3_action_all_s','s'),('其中局部曲率修复','curvature_repair_all_s','s'),('完整终验累计','strict_audit_all_ms','ms')]:
 values=[v(d,scope,metric) for scope in ('first','warm') for d in (a,b)]
 lines.append('| '+label+' | '+' | '.join(f'{x:.3f} {unit}' for x in values)+' |')
lines+=['','各阶段分别取中位数，不可相加还原总中位数；L3 完整流程已包含其子项。审计是控制器内完整审计调用的累计，局部连接器内部的检查仍包含在修复时间中。', '',
 '## 内存','', '| 指标 | r8 中位 | r9 中位 | 变化 |','|---|---:|---:|---:|']
for label,metric,unit in [('L2 专属状态','l2_states_mib','MiB'),('L3 规划器 PSS','planner_pss_gib','GiB'),('L3 栈 PSS','stack_pss_gib','GiB'),('L3 栈 RSS','stack_rss_gib','GiB'),('输出落盘后进程树 PSS快照','post_export_tree_pss_gib','GiB')]:
 lines.append(table_row(label,'all',metric,unit=unit))
lines+=['','L2 预计算为每个状态增加 8 个 float64 邻接代价和 1 个 float64 启发式值（72字节/状态），以少量专属内存换取搜索加速。总内存仍可下降，因为移除了无用中间图并减少通信缓冲。PSS 按共享页分摊；不能把各层 RSS 直接相加。输出后 PSS 是快照，查询 RSS 峰值来自查询过程采样，两者不混用。', '',
 '## 已实施修改','',
 '1. 缓存命中报告本次实际耗时，历史构建耗时单独保留；所有候选准备、L3、修复和审计累计记录。',
 '2. L2 预计算静态有向邻接代价和起点启发式。仍为二维 D*，保留原邻居顺序、精确 float64 代价、方向与对角格规则及预算。',
 '3. 路线缓存验证后提前恢复，避免重新生成随后被覆盖的全图语义掩码；缓存键含地图、语义、车辆、动态信息、路线、权重和完整端点姿态。',
 '4. 直接构造最终有界 costmap，一次 pinned Nav2 膨胀替代先无界再有界的两次膨胀，移除无用中间地图。',
 '5. 本工作区私有 Fast DDS 缓冲从 128 MiB 调为 64 MiB；默认保留原 glibc 分配器。保留可靠传输、消息大小、内容 ACK 与原进程清理。', '',
 '没有裁剪地图坐标系、改变 Smac/车辆模型、放宽终验、扩大搜索预算或启用全图搜索。', '',
 '## 等价性与验证','']
for key,label in [('l1_output.json','L1 折线'),('l2_soft_target.json','L2 0.30 m 软目标'),('l2_reference_xy.npy','L2 参考'),('path.npy','L3 最终轨迹')]:
 n=sum(p[key] for p in data['path_parity']);lines.append(f'- {label}逐点一致：{n}/30。')
lines += [f"- r8 首次查询最终选中参考的 A* 回退次数 {a['fallback_runs']}；r9 为 {b['fallback_runs']}，时间与展开预算保持原样。",
 f"- 所有最终地图 ACK：r8={a['l3_ack_all']}，r9={b['l3_ack_all']}；全部返回路径在搜索掩码内：r8={a['within_mask_all']}，r9={b['within_mask_all']}。",
 '- 26项回归通过，包含逐边精确代价、D* 队列结果、动态阻塞/潜在代价更新、costmap 所有数组及策略哈希等价、缓存计时来源隔离。',
 '- 12份输入文件哈希与 r8 完全一致。运行代码与协议逐批保存于 source_snapshot 和 protocol.json。', '',
 '## 内存配置取舍','',
 'compact 配置（results/r9_isolated_10x3）也取得30/30和全路径逐点一致，RSS中位约2.769 GiB、P95约9.994 s，但总体中位2.263 s，慢于同环境r8的1.954 s。因此本轮默认选择 balanced 配置，不启用激进分配器参数。compact 的单独数据保存在 report/PERFORMANCE_COMPACT.json；不能把它的最低内存和 balanced 的最快耗时拼成一个配置的成绩。','',
 '## 试验局限和后续','',
 '早期 CPU 14 批次与另一工作区的万达地图试验争用核心，已停止本任务受干扰批次并保留证据（report/INTERFERENCE.json）。不停止其他任务、不挑选受干扰批次中的快速样本。正式结论只取随后 CPU 12 的两批完整串行结果。', '',
 '本轮是同一静态地图10条查询的回归，没有证明跨地图、动态障碍或实机运动性能。CPU隔离无法消除整个桌面系统的所有背景波动；数个百分点变化不作为普遍性能保证。', '',
 '下一步优先针对仍占耗时的地图 ACK 与局部曲率修复做有归因的试验。L3 仍持有原尺寸 costmap，进一步裁剪存储需单独验证地图原点、膨胀边界和内容 ACK。当前不建议继续优化 L1，也不应继续将旧 0.52 秒字段当作热查询优化目标。', '',
 '## 原始资料','',
 '- `results/r8_isolated_10x3/runs.jsonl`：同环境 r8 对照。',
 '- `results/r9_balanced_10x3/runs.jsonl`：r9 正式结果。',
 '- `report/PERFORMANCE.json`：全部统计及30次逐点比较。',
 '- `report/INPUT_HASHES.json`、`report/CODE_CHANGES.diff`、`report/tests.txt`：输入、改动及回归证据。',
 '- `README.md`：复现命令；`INTERFACES.md`：接口与计时口径。',
 '- `results/*.supervisor.json`：进程与共享内存清理证据。','']
(ROOT/'report/RESULTS.md').write_text('\n'.join(lines))
