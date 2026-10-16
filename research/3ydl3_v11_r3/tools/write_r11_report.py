"""Reproducible final report from raw aggregate evidence."""
import json
from pathlib import Path
import numpy as np
from analyze_r11 import read
ROOT=Path(__file__).resolve().parents[1]
D=json.loads((ROOT/'report/PERFORMANCE.json').read_text())
base=D['r10_control_10x3'];final=D['r11_AB_formal_10x3']
def val(d,g,k,s='median'):return d[g][k][s]
def f(x):return f'{x:.3f}'
def ms(x):return f'{x:.3f}' if abs(x)<.1 else f'{x:.1f}'
def reduction(k,s='median',g='all'):return (1-val(final,g,k,s)/val(base,g,k,s))*100
rows=[]
for key,label in [('r10_control_10x3','r10 本轮对照'),('r11_A_formal_10x3','A 内存优化＋ACK'),('r11_B_formal_10x3','B 支持域计算＋ACK'),('r11_AB_formal_10x3','r11 默认 A+B')]:
 d=D[key];rows.append(f"| {label} | {d['strict_success']}/{d['runs']} | {f(val(d,'all','wall_s'))} | {f(val(d,'all','wall_s','p95'))} | {f(val(d,'first','wall_s'))} | {f(val(d,'repeat','wall_s'))} | {f(val(d,'all','rss_gib'))} / {f(val(d,'all','rss_gib','max'))} |")
phase=[]
for k,label in [('l1_ms','L1 区域选择与折线'),('l2_all_ms','L2 全部候选初始化与参考'),('adapter_ms','L2→L3 代价图适配'),('l3_all_ms','L3 完整流程（含适配、同步、规划、终验）')]:
 phase.append(f"| {label} | {ms(val(base,'first',k))} → {ms(val(final,'first',k))} | {ms(val(base,'repeat',k))} → {ms(val(final,'repeat',k))} |")
stress=[]
for key,label in [('r11_control_interleaved','r10 顺序打乱'),('r11_AB_interleaved','r11 顺序打乱'),('r11_control_perturbed','r10 端点扰动'),('r11_AB_perturbed','r11 端点扰动')]:
 d=D[key];stress.append(f"| {label} | {d['strict_success']}/{d['runs']} | {f(val(d,'all','wall_s'))} | {f(val(d,'all','wall_s','p95'))} | {f(val(d,'all','rss_gib','max'))} |")
crop=D.get('r11_ABC_crop_fixed10');cdata=''
if crop:
 cdata=f"固定10组裁剪诊断：**{crop['strict_success']}/{crop['runs']}**严格成功，中位{val(crop,'all','wall_s'):.3f}s，P95 {val(crop,'all','wall_s','p95'):.3f}s，RSS中位/峰值{val(crop,'all','rss_gib'):.3f}/{val(crop,'all','rss_gib','max'):.3f}GiB。该批各一次，不作为30次正式发布结果。"
cleanup=json.loads((ROOT/'report/CLEANUP.json').read_text())
text=f'''# 3YD-V5-r11 实验结果

完成日期：2026-09-22。基于冻结V5-r10，新工作区独立构建。**默认采用A+B，固定10×3保持30/30；存储裁剪C保留为独立实验，不默认启用。**

## 1. 实际修改与内存原因

- A：优化独立Nav2 InflationLayer的零距离障碍队列。原实现将大量致命格先入队，并将以后必定跳过的相邻障碍继续入队；清空后仍保留大容量。新实现先标记种子，再按原顺序入队非种子邻居。正距离传播、未知格语义、局部更新范围、膨胀代价均不变。
- A同时将占据代价转换按96行处理，避免多份全图float64临时数组；全真affected_mask以只读广播表达。
- B：软参考距离场与预期膨胀仅计算有限支持包围框，外部使用原饱和值/致命值。输出仍是完整地图，不改变原点或尺寸。
- 稀疏发布补齐了r10遗漏的订阅发现、发布序号和旧服务器时间绑定。新策略即使字节相同也重申一格，再取得两次精确有效代价图观测。保留原内容ACK与有限补发。

独立1600万格全致命测试中，膨胀队列保留容量从1,573,132,000字节降至276,000字节；这是队列容量，不是整机RSS。800组原生随机/未知格/局部更新比较全部逐格相同。实际L3规划器PSS的下降见下表。Smac二进制哈希与r10相同，原内容快照锁保留；原生源码只修改inflation_layer.cpp。

只读数组盘点中，Q06控制进程可达NumPy底层分配约499.85MiB，其中有全图距离场、语义标签、软场和多份掩码。不能把这些共享数组机械分摊为L1/L2/L3独占内存；原生膨胀队列也不在NumPy盘点中。

## 2. 固定10组×3，同环境独立对照

CPU12、串行、数值库单线程、不加profiler。首次=每组第1次，重复=该组第2/3次；查询计时不包含一次性世界构建与常驻栈启动。r11 AB世界准备时间另见preparation.json，常驻栈启动约7.439s。C每次重建费用计入查询。

| 配置 | 严格成功 | 总体中位s | P95 s | 首次中位s | 重复中位s | RSS中位/峰值GiB |
|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

默认AB总体中位下降{reduction('wall_s'):.1f}%，首次中位下降{reduction('wall_s',g='first'):.1f}%，P95下降{reduction('wall_s','p95'):.1f}%，RSS峰值下降{reduction('rss_gib','max'):.1f}%。重复查询仅改善{reduction('wall_s',g='repeat'):.1f}%，基本持平。B单独没有稳定总体提速；其与A的组合才取得本轮最佳折中，不把各项最好值拼接成结果。

本表r10是本轮重新运行的直接r10代码对照。冻结r10原报告为1.491s/7.006s/3.710GiB；保留历史数值，不与本轮列混用。

## 3. 层级计时与内存

下表为毫秒中位数，箭头为r10→r11；统计覆盖本次查询的全部候选。L3流程包含适配，不能把各行简单相加。

| 环节 | 首次ms | 重复ms |
|---|---:|---:|
{chr(10).join(phase)}

首次地图更新中位约1637.4→930.0ms。L1/L2搜索没有架构修改，普通波动不算算法收益；主要收益来自适配与L3地图处理。

| 内存口径 | r10 GiB | r11 GiB |
|---|---:|---:|
| L3规划器PSS中位 | {val(base,'all','planner_pss_gib'):.3f} | {val(final,'all','planner_pss_gib'):.3f} |
| L3整套ROS栈PSS中位 | {val(base,'all','stack_pss_gib'):.3f} | {val(final,'all','stack_pss_gib'):.3f} |
| 查询进程树RSS峰值 | {val(base,'all','rss_gib','max'):.3f} | {val(final,'all','rss_gib','max'):.3f} |

RSS为工作进程和ROS子进程相加，共享页可能重复；PSS用于归属分析。RSS中位数指30次“查询峰值”的中位数，不是持续采样的常驻中位。L3规划器PSS减少约{reduction('planner_pss_gib'):.1f}%。L2专用搜索状态和车辆/语义约束未变。

## 4. 结果质量和缓存验证

A、B、AB各30/30。全部90次与本轮r10相应查询的L1折线、0.30m软目标、L2参考、L3最终轨迹逐点一致；路径长度、靠右比例完全相同。实际内容ACK、有限范围终验全部通过。r11发布序号不再停留在旧序号，旧时间戳或进行中更换发布均不能通过测试。

L1不改；L2仍0.15m XY D*和0.30m软引导；L3仍原0.05m地图、Jackal、Smac、0.8/1.2m范围与完整连续碰撞/转向/曲率/交通终验。没有全图回退。

## 5. 次序打乱与端点扰动

固定种子611，每轮打乱Q01–Q10，共5轮50次；另对全部10组起终点平移0.05m并转动2°，保留原停车归属。各方案使用同一数据，不过滤失败。交错后的第2–5轮不能称为热缓存，是否命中以实际记录为准。

| 测试 | 严格成功 | 中位s | P95 s | 峰值RSS GiB |
|---|---:|---:|---:|---:|
{chr(10).join(stress)}

交错与扰动分别检查同条件路径、软目标、L1和靠右一致性，见PERFORMANCE.json。r11交错第2–5轮导出后进程树PSS中位约2.106、2.092、2.127、2.114GiB，未观察到持续增长；50次有限试验不能证明长期无泄漏。

## 6. C：存储裁剪的取舍

保持原允许格，增加膨胀和车体边界；原点用整数格偏移。每次切换尺寸/原点重启一个规划栈、清空ACK；裁剪后有效代价图与原全图相应区域逐格一致，再执行原全局终验。绝不同时保留两个已启动规划栈。

先跑Q06/Q09各2次，4/4通过。Q06存储163,400格、Q09为307,326格（原图16,036,124格），规划器PSS约0.119GiB。但每次切换的启动/退出/绑定约4.75–5.07s计入了查询。Q06路径长度40.956→40.930m；Q09为60.731→60.909m，最终轨迹不再逐点相同。有效代价相同并不足以保证改变尺寸、原点后的离散规划结果相同。

{cdata}

**C不作为默认：**内存更低，但尺寸切换延迟与路径变化需要单独治理。当前AB已取得主要原生内存收益，保留全图坐标接口更稳定。

## 7. 完整性、限制和后续

- 41项Python测试通过，最终日志：report/tests.log；原生800组与全致命测试通过，证据：report/native_equivalence.log。
- 输入哈希全部保持；修改清单、源码哈希、原生补丁、逐批执行快照与已加载二进制哈希均保存。
- 所属实验批次清理：{len(cleanup['batches'])}批，全部状态见report/CLEANUP.json。正式AB批次驱动收到SIGTERM后，30条查询、summary和正常栈退出已完整落盘；进程代际清理日志随后安全恢复，GATE重新通过。该事件保留于supervisor_recovery.json，不补写虚构退出码。
- 本轮仍是静态仿真规划，不等于动态障碍实机或多地图部署完成。

下一步优先用新固定地图验证r11等价优化的迁移性，再对Python端大临时数组与全图内容转换做进一步定位；如继续裁剪，先实现安全的常驻栈尺寸切换，再做坐标、边界、ACK与全固定集验收。不能直接把本轮重启式裁剪设为默认。

## 8. 文件与复现

- 正式AB：results/r11_AB_formal_10x3/runs.jsonl
- 直接r10对照：results/r10_control_10x3/runs.jsonl
- 独立A/B：results/r11_A_formal_10x3、results/r11_B_formal_10x3
- 鲁棒性：results/r11_control_interleaved、r11_AB_interleaved、r11_control_perturbed、r11_AB_perturbed
- 裁剪：results/r11_ABC_crop_pilot、results/r11_ABC_crop_fixed10
- 完整统计：report/PERFORMANCE.json；接口/协议/命令：INTERFACES.md、EXPERIMENT_PROTOCOL.md、README.md。

本轮未发布或覆盖网页历史模块。
'''
(ROOT/'report/RESULTS.md').write_text(text)
