"""Render verified JSON into the delivery report; never manufacture missing runs."""
import json,math,statistics,xml.etree.ElementTree as ET
from pathlib import Path
from arena_3d_v1.semantic_world import WORK

def rows(name):
 return [json.loads(s) for s in (WORK/'results'/name/'runs.jsonl').read_text().splitlines()]
def main():
 d=json.loads((WORK/'report/performance.json').read_text());old=d['main']['3YD-V1-r1'];new=d['main']['candidate']
 assert old['n']==new['n']==60, 'Incomplete fixed query batch'
 assert sum(s['cycles'] for s in d['startup'].values())==100, 'Incomplete startup batch'
 startup=sum(s['startup_success'] for s in d['startup'].values());complete=sum(s['complete_success'] for s in d['startup'].values());updates=sum(s['updates_success'] for s in d['startup'].values())
 control=d['controls'];hold=d['holdouts'];sealed=d['sealed_holdouts']
 repaired_rows=rows('final_v3_return_bulb')+rows('final_v2_return_bulb_rotated')
 for k in [*hold,*sealed]:
  if k.endswith('_candidate'):repaired_rows+=rows(k)
 repair_times=[r['local_curvature_repair']['wall_ms'] for r in repaired_rows if r['success']]
 checks=ET.parse(WORK/'report/regression_v3.xml').getroot().find('testsuite').attrib
 info=[]
 for name,v in [('首次查询中位数',('first','p50')),('重复查询中位数',('repeat','p50')),('全部查询 P95',('all','p95')),('最大查询耗时',('all','max'))]:
  a,b=[arm['timing_s'][v[0]][v[1]] for arm in [old,new]];info.append(f'| {name} | {a:.3f} s | {b:.3f} s | {(b/a-1)*100:+.1f}% |')
 table='\n'.join(info)
 hsucc=sum(v['success'] for k,v in hold.items() if k.endswith('_candidate'));hsum=sum(v['n'] for k,v in hold.items() if k.endswith('_candidate'))
 ssucc=sum(v['success'] for k,v in sealed.items() if k.endswith('_candidate'));ssum=sum(v['n'] for k,v in sealed.items() if k.endswith('_candidate'))
 origin=control['final_v3_return_bulb'];rot=control['final_v2_return_bulb_rotated']
 status='本批启动与后续掩码更新均通过。' if complete==100 else '本批仍有失败，未达到 100 次完整启停与更新全部通过的验收目标。'
 maintext=f'''# 基于 3YD-V1-r1 的跨区域掉头与启动 ACK 实验

日期：2026-09-14。3YD-V1 已定义为 **3YD-V1-r0**；上一轮停车区/路口、局部精细连接与候选恢复版定义为 **3YD-V1-r1**。本目录是基于 r1 的独立增量实验，未另设正式版本名。原 r0/r1 运行代码及历史结果保留。

## 结果与适用范围

本轮完成了掉头与启动链路的功能修复，但最终批次 P95 明显高于对照，暂不能判定性能准入通过。本轮增加局部运动学修复，使新增掉头样例获得通过原有两套检查的轨迹；同时修复单次读回耗尽 ACK 时限、大块掩码缺口不补发，以及会话释放依赖垃圾回收的问题。最终 100 次启动中，启动成功 {startup}/100，包含三轮地图更新和退出的完整周期 {complete}/100，地图更新成功 {updates}/300（300 是预定数，提前失败未执行的后续更新也不能算成功）。{status}

| 样本组 | 原 r1 | 本轮 | 分母与说明 |
| --- | ---: | ---: | --- |
| 固定真实地图查询 | {old['success']}/60 | {new['success']}/60 | 20 条不同查询，各重复三次 |
| 原跨区域掉头 | 0/6 | {origin['success']+rot['success']}/6 | 两个场景各三次；原 r1 历史最终复测为 0/6 |
| 掉头扩展回归 | 0/24 | {hsucc}/{hsum} | 四种位置/朝向/空间变体及旋转，共八组；曾用于迭代 |
| 新保留掉头场景 | 0/12 | {ssucc}/{ssum} | 两种新定义及旋转，共四组，几何修复冻结后首次运行 |
| 标定图与旋转图的右→左→右 | — | {control['final_calibration_v3']['success']+control['final_v2_rotated']['success']}/18 | 两组各三轮偏好切换；右侧路径恢复检查另列于 JSON |
| 窄通道 | — | {control['final_v2_narrow']['success']}/3 | 允许必要精细回退 |
| 双通道常规/错误候选注入 | — | {control['final_v2_two_routes']['success']}/3、{control['final_v2_two_routes_optimistic_edge']['success']}/3 | 注入结果不是自然故障发生率 |
| 自动回归 | — | {checks['tests']} 项 | 失败 {checks['failures']}、错误 {checks['errors']} |

掉头合计是 14 组输入各三次，不能解读为任意掉头均可完成。新增几何变体的算法对照共同使用稳定会话，分开研究轨迹恢复和通信问题。扩展/保留场景及带 `final_v2` 的控制组使用最终几何修复代码；之后仅更改非语义掩码 ACK，交付会话另经 `final_v3` 的启动、控制场景和固定查询复测。未下发车辆速度，结果属于静态规划验收。

## 失败发生在哪里，如何修复

原场景 L1 要求“车道→掉头区→车道”。转换成二维区域并集后，L2 参考只访问车道，确实丢失重复访问意图；但固定 Smac 的输出已经进入掉头区，最终拒绝的直接原因是曲率。

在相同输入与检查口径下，Smac 原始搜索输出最大曲率约 **2.500092 m⁻¹**，最终输出约 **4.146846 m⁻¹**；车辆理论上限为 **2.5 m⁻¹**。原始路径仅有小量超限，仍未通过原独立检查；不能通过直接接收原始路径或放宽数值容差宣称修复。原 PathAudit 对原始路径的容差结果与独立检查不同，系统继续要求两者都通过。

新增分支只处理“地图已确认、存在轨迹、独立检查仅曲率失败”。对超限及实际转向突变附近的局部段，用前进圆弧积分和有界曲率变化约束求可行连接，实际改变位置和航向。整个拼接结果重新通过原车体扫掠、端点、方向规则、曲率和转向连续性检查。新增段明确标为 `l3_local_curvature_repair`，没有裁剪转向元数据换取通过。

原掉头场景修复后的路径约 **{origin['length_m']['p50']:.3f} m**，最大曲率约 **{origin['curvature_1pm']['max']:.3f} m⁻¹**。原先被拒绝的最终输出长 12.778 m；多出的长度用于获得合法转弯，不能把被拒绝路径当成可执行基准来宣称质量最优。42 次掉头验证中，成功局部修复耗时中位数 **{statistics.median(repair_times):.1f} ms**、最大 **{max(repair_times):.1f} ms**。

约束上限为局部窗口 6 m、24 个候选、128 个积分状态、0.25 s 协作预算；每次迭代和接收前检查时间。这不是硬实时界限。达到独立核验的可行解即停止优化，不声称局部或全局最优。Smac 二进制、参数、0.05 m 原底图和车辆模型不变；**L3 输出处理增加了修复分支，因此最终轨迹并非始终是原封不动的 Smac 输出**。

当前只对修复后的最终轨迹核对有序区域访问，尚未实现按访问阶段搜索的 L2；对于任意复杂重复区域序列仍没有完备性保证。

## 与原 r1 的性能与路径质量对照

同一机器掉电恢复后，固定同一 CPU 15、同一地图和参数，BLAS 线程数为 1；每个实验使用独立 L2 缓存，20 条查询各三次。首次是每条查询在该实验中的第一次调用，重复是其后两次；不是每条查询都冷启动整个系统。查询计时包含候选尝试、地图修补、规划与轨迹检查，启动/构图另计，结果写盘在计时之后。

| 指标 | 原 3YD-V1-r1 本轮复测 | 本轮最终实现 | 变化 |
| --- | ---: | ---: | ---: |
{table}
| 查询进程树峰值 RSS | {old['peak_rss_gib']:.3f} GiB | {new['peak_rss_gib']:.3f} GiB | {(new['peak_rss_gib']/old['peak_rss_gib']-1)*100:+.1f}% |
| 成功 | {old['success']}/60 | {new['success']}/60 | 按实际结果统计 |
| 纯粗图 / 局部精细辅助 | 48 / 12 | {new['completion_modes'].get('coarse_only',0)} / {new['completion_modes'].get('local_fine_assisted',0)} | 完成方式按每次查询统计 |
| 语义地图完整补发次数 | {old['map_repair_count']} | {new['map_repair_count']} | 已计入耗时 |

固定集 {d['exact_same_query_paths']}/20 条查询的首次输出位置和航向数组与原 r1 完全相同；逐查询长度及冻结公共评价场中的靠右占比见下表。固定集未触发新的曲率修复分支。这轮首次中位数约增加 4.9%，重复中位数约增加 0.3%；P95 从 3.321 s 升至 6.864 s，约增加 106.7%，是必须报告的性能回退。最慢四条均发生语义地图内容不一致后的完整重发，已计入耗时；但仅凭批次对照不能断言新增代码是全部波动的原因。本轮主要收益是困难掉头的可执行性和启动可靠性，不宣称固定集提速或性能准入通过。

''' 
 maintext+='| 查询 | 原长度 m | 本轮长度 m | 原靠右目标带占比 | 本轮靠右目标带占比 |\n| --- | ---: | ---: | ---: | ---: |\n'
 for q in d['paired_queries']:
  a=q['3YD-V1-r1'];b=q['candidate'];maintext+=f"| {q['query']} | {a['length_m']:.3f} | {b['length_m']:.3f} | {a['right_band_ratio']*100:.2f}% | {b['right_band_ratio']*100:.2f}% |\n"
 maintext+='''
公共评价场沿用 V0 固定参考下的可评价车道段，所有版本共享，未换指标或删除低分段。两条停车区内部查询的靠右目标带占比仍为 0%，本轮没有修复这个遗留偏好问题。每次查询的完整评价见 `path_quality.json`。

## ACK 与资源释放证据

1. 单次 GetCostmap 最多占用 0.75 s，超时请求取消并从 pending 队列移除，仍共享原 3 s ACK 时限。
2. 冷启动和普通掩码更新中，同一缺口被重复读到后，可在原时限内最多补发两次，不再因为超过旧 5%/5000 格限制而一直等待。启动全封闭图要求两次全图为 254；普通掩码继续要求受影响关闭格为 254、开放格低于 254，以保留原有膨胀处理。
3. 语义软代价仍走原精确有效主地图、版本与稳定观测检查。普通掩码的确认不等于语义软代价逐字节相等。
4. 正常退出先通知 launch，由其停止子进程；超时才升级信号。显式释放 ActionClient、Node、Context，并仅清理观察到属于本会话且无可见占用的 FastDDS 文件。已有无所有权证据的历史文件不删除。

| 压力批次 | 启动 | 完整周期 | 成功地图更新 | 强制退出注入 |
| --- | ---: | ---: | ---: | ---: |
'''
 for name,s in d['startup'].items():
  maintext+=f"| {name} | {s['startup_success']}/{s['cycles']} | {s['complete_success']}/{s['cycles']} | {s['updates_success']}/{s['cycles']*3} | {s['abrupt_exits_injected']} |\n"
 maintext+='''
小地图循环包含两张掉头图及两张标定图（72,000、134,400 和 200,000 格），覆盖约 128 KB 地图块边界；另有实际约 859 万格地图。每批复用同一隔离 ROS 域，循环内重新创建会话，每次执行全封闭初始化、开放安全区域、关闭半区、恢复安全区域、退出。强制退出发生在指定周期更新完成后；末轮注入只验证退出与清理，不能算一次后续重启验证。

`/dev/shm` 的读数是整机共享内存，包含其他或历史进程资源；查询 RSS 是实验进程树，两者不能相加或混称本算法内存。逐周期资源、仍占用文件和清理跳过原因均保存于 `lifecycle_evidence`。这 100 次是本矩阵的有限实验，不能推导生产环境零故障。

两批压力子进程退出后各留下一个约 128 MiB 的已归属共享段。测试工具在确认子进程退出、无可见映射或文件描述符占用后，删除这两个段；记录见 `post_process_cleanup.json`。`StableSmacSession.close()` 本身不能保证进程退出后的零残留，长期服务的进程监督与退出后清理仍需补齐；此次测试收尾不替代生产生命周期验证。

受控故障注入使用真实 ROS 服务，但故障由客户端代理显式制造：

| 注入 | 原 r1 | 本轮 | 解释 |
| --- | --- | --- | --- |
| 丢弃第一个读回响应 | 启动失败 | 取消旧请求后启动成功 | 仍在原 ACK 总时限内 |
| 丢弃第一个启动地图块 | 启动失败 | 缺口补发后启动成功 | 依据服务端内容确认 |
| 丢弃启动后第一个掩码块 | 约 3.029 s，经完整重发恢复 | 约 0.064 s，局部补发恢复 | 单次注入对照，不是自然延迟分布 |
| 持续篡改读回内容 | 未重复对照 | 正确拒绝启动 | 失败是预期保护结果 |

数据文件：`results/ack_fault_injection_v3/results.json`。上述响应丢失不等同于网络层抓包；本轮没有证实所有历史丢包或共享内存问题都属于同一根因。

## 保留的失败与迭代

- 初版局部修复在扩展掉头矩阵只通过 17/24；两个较小掉头区均失败，另一次在预算内未找到结果。发现几何修复窗口之外还有真实转向突变后，扩大需修复的事件范围，优先使用一倍最小转弯半径的端部余量，并在找到可行解时结束优化。随后扩展集 24/24；新保留场景 12/12。旧失败没有删除。
- 第一轮 100 次启动及 300 次更新通过之后，额外标定图启动仍出现 96,000/134,400 格内容未生效；旧规则因缺口过大没有补发。本轮先补齐冷启动大块缺口恢复。
- 第二轮启动依然 100/100，但只有 98/100 完整周期、294/300 预定更新成功。第 11、67 周期均在同一 200,000 格图的第一次开放更新留下 127,872 格不一致；完整重发后仍有 88,128 格不一致。一个最终状态显示 readback_error，但仅是总截止前最后一次极短读回超时，先前持续的内容缺口才是主要证据。由此将有界补发覆盖到普通掩码更新，最后另跑本报告主压力批次。
- 固定查询的第一轮增量实现 P95 为 6.962 s，第二轮为 3.205 s；相应语义地图完整重发分别为四次、一次。所有历史耗时保留在 `performance.json`。最终表按最后一次预先安排的完整复测统计，不能挑选最快一轮；语义发布与完整重发仍可能影响尾部耗时，最大值也不是实时上限。

## 下一步安排

优先处理 **语义软代价更新的发布与尾部延迟**，并将已验证的退出后归属清理接入服务进程监督：保留精确内容确认，建立右/左切换、大地图块边界、连续更新和陈旧内容注入的单独矩阵，记录发布块、服务端生效与 ACK 的对应关系，减少可避免的全图重发。普通掩码稳定不等于这一条链路已经稳定。

随后修复两条停车区内部查询的靠右目标带 0% 问题，并增加较大航向变化、连续两次掉头和不可行空间等机动对照；若重复区域意图在此类场景仍丢失，再实现按区域访问阶段组织的 L2 搜索。每项保留原公共评价与轨迹检查，避免用更长路径或更宽松碰撞条件换取单项指标。

在静态准入保持后，动态阶段先接同源世界坐标障碍、L2/L3 粗细投影与版本失效，再接 L1 有向通道状态和真实位姿衔接。形态学子区域重新切割继续不列为当前前置条件。

## 可复查材料

- `performance.json`：所有分组、历史失败、启动批次、命令及源文件校验。
- `raw_vs_final.json`、`ordered_intent_diagnosis.json`：曲率与访问顺序诊断。
- `results/*/cases/*/outcome.json`、`path_points.json`：检查、实际轨迹、修复来源和候选失败记录。
- `regression_v3.xml`：最终自动回归。
- `IMPLEMENTATION_FREEZE.json`、`R1_BASELINE_MANIFEST.json`：当前源文件冻结与 727 个原 r1 运行文件检查。

'''
 integ=d['integrity'];maintext+=f"最终完整性检查：原 r1 运行文件变更 {len(integ['baseline_runtime_changed'])}，冻结文件意外变更 {len(integ['changed_frozen_files'])}；Smac 参数一致为 {integ['identical_smac_parameters']}，车辆扫掠二进制一致为 {integ['identical_vehicle_sweep_binary']}。\n"
 (WORK/'report/performance.md').write_text(maintext)
 print('Wrote report/performance.md')
if __name__=='__main__':main()
