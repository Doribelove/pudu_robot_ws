import json
from pathlib import Path
R=Path(__file__).resolve().parents[1];d=json.loads((R/'report/performance.json').read_text());a=d['main']['baseline'];b=d['main']['candidate']
def table(head,rows):return '\n'.join(['| '+' | '.join(head)+' |','| '+' | '.join(['---']*len(head))+' |',*['| '+' | '.join(map(str,r))+' |' for r in rows]])
def num(v,n=3):return '无分母' if v is None else f'{v:.{n}f}'
def change(old,new):return f'{100*(new/old-1):+.1f}%'
metric=[]
for title,k,q in [('首次查询中位数','first','p50'),('重复查询中位数','repeat','p50'),('全部查询 P95','all','p95'),('最大查询耗时','all','max')]:
 x=a['timing_s'][k][q];y=b['timing_s'][k][q];metric.append([title,num(x)+' s',num(y)+' s',change(x,y)])
metric += [['查询峰值 RSS',num(a['peak_rss_gib'])+' GiB',num(b['peak_rss_gib'])+' GiB',change(a['peak_rss_gib'],b['peak_rss_gib'])],['成功/执行',f"{a['success']}/{a['n']}",f"{b['success']}/{b['n']}",'相同固定集'],['触发整图回退的查询',a['map_repair_count'],b['map_repair_count'],'计入查询耗时'],['语义缺口局部补发',a['gap_repairs'],b['gap_repairs'],'正常主查询，无故障注入']]
lines=['# 基于 3YD-V1-r1：语义地图重发、尾部延迟与退出清理','',
'本轮已完成服务端地图快照一致性修复、真实发送节拍、同版本有界补发和进程退出后归属清理。仍是基于 **3YD-V1-r1** 的独立增量实验，未新命名正式版本，未替换生产安装。', '',
f"完整 A/B/B/A 对照中两组均为 **120/120**；20 个固定查询的全部重复输出轨迹一致。查询 P95 从 **{a['timing_s']['all']['p95']:.3f} s 降至 {b['timing_s']['all']['p95']:.3f} s**，变化 **{change(a['timing_s']['all']['p95'],b['timing_s']['all']['p95'])}**；正常主查询整图回退从 **{a['map_repair_count']} 降至 {b['map_repair_count']}**。100 次语义更新、30 次控制场景查询、12 组进程生命周期及 82 项回归均通过。",'',
'## 1. 根因与采用的修复','',
'实际运行的是工作区 Nav2 1.1.19 的 costmap 库。`GetCostmap` 服务原回调没有持有 `Costmap2D` 更新锁，可能在 StaticLayer 已写入、InflationLayer 尚未完成时复制主地图。精确 ACK 因此读到中间内容，误判未生效并触发重发；单纯提高发送频率或增加补发不能保证解决。', '',
'对同一个真实 ROS 服务测试程序，在写线程持有更新锁、格值暂为 76 时发起读取，随后写入最终值 253 并解锁：', '',
table(['库','受控测试','更新中间态被读取','收到非最终内容'],[['原工作区库','10 次','10/10','10/10'],['本地加锁库','10 次','0/10','0/10']]),'',
'证据：[并发测试源码](../tests/native_snapshot/snapshot_consistency.cpp)、[真实运行日志](native_snapshot/consistency.log)、[原包与补丁来源](native_snapshot/local_provenance.json)。这是针对可复现并发窗口的验证，10 次不是所有调度情况的概率估计。','',
'最终采用三个配合措施：', '',
'1. 服务端复制尺寸、原点和格子时持有现有更新锁，保证一次读回对应完成的地图状态。包源码只改 `costmap_2d_publisher.cpp` 此处。',
'2. 用单调时钟保证相邻地图块实际间隔至少 15 ms。原 `spin_once(timeout=15 ms)` 在回调就绪时会提前返回；开发观测最短约 0.294 ms，不能作为节拍保证。',
'3. 正常精确确认先运行；0.4 s 后仍连续两次观察到相同缺口，最多补发两次缺口及膨胀邻域的原占据内容。补发保留原逻辑序列，仍受原 ACK 时限约束，补发动作本身不算确认。','',
'`metadata.update_time` 是服务读回时的时间，不能当作地图更新代数。最终接受仍检查原目标内容/策略、发布序列、有效主地图精确内容和稳定观测；内容错误不能通过换拓扑路线掩盖。','',
'代价是读取期间短暂占用更新锁，可能等待当前膨胀结束；本轮验证静态串行请求，未证明高频、多写入者下的吞吐或硬实时上限。底图、车辆、Smac 搜索/平滑/参数及 L1/L2 几何逻辑保持原样；L3 周边的地图服务读回确实做了改动。','',
'## 2. 同条件性能与结果','',
'对照 A 为上轮“掉头与启动 ACK 修复”实现，本轮重新测量；B 为本轮最终修复。四个独立进程与缓存批次按 A/B/B/A 运行，每批同一地图的 20 条查询、每条 3 次。共 240 次，两组各 120 次；只有 20 个不同查询，重复测量相关，不能解释成 120 个独立场景。CPU、地图、参数、输入和监督流程一致。','',
table(['指标','上轮实现重测 A','本轮修复 B','变化/口径'],metric),'',
'查询耗时包含 L1、L2、代价准备、地图更新/ACK、重试、Smac 和终验；图准备、服务启动和写出结果单列。每批查询峰值 RSS 包含查询进程及 ROS 子进程，不含监督。', '',
f"监督进程峰值 RSS：A **{a['supervisor_peak_rss_mib']:.2f} MiB**，B **{b['supervisor_peak_rss_mib']:.2f} MiB**。本轮主要解决重发与退出后资源积累，不能将其描述为显著降低运行时地图内存。",'',
table(['批次','成功','首次中位 s','重复中位 s','P95 s','最大 s','整图回退查询'],[[k,f"{s['success']}/{s['n']}",num(s['timing_s']['first']['p50']),num(s['timing_s']['repeat']['p50']),num(s['timing_s']['all']['p95']),num(s['timing_s']['all']['max']),s['map_repair_count']] for k,s in d['batches'].items()]),'',
'以上包含全部预定批次，没有挑选最快一轮。旧任务最终 P95 6.864 s 可作历史参考，但本轮改善幅度以上表同条件 A/B 重测为准；本轮样本 P95/最大值不是稳定总体分位数或最坏执行上限。','',
table(['阶段（毫秒，中位 / P95）','A','B'],[[k,f"{num(a['stage_ms'][k]['p50'],1)} / {num(a['stage_ms'][k]['p95'],1)}",f"{num(b['stage_ms'][k]['p50'],1)} / {num(b['stage_ms'][k]['p95'],1)}"] for k in a['stage_ms']]),'',
'这些阶段字段来自最终 L3 调用，不能相加代替包含全部尝试的端到端耗时。','',
'## 3. 成功率与路径质量','',
f"两组均 120/120，接受后独立检查失败为 0；{d['identical_query_paths']}/20 个查询在两组全部重复中位置/航向数组完全相同。因此这批路径长度、靠右表现、曲率与碰撞检查没有因时序修复改变。L2 完成模式也一并保留：A {a['completion_modes']}；B {b['completion_modes']}。",'',
'靠右评价继续使用原公共评价场及弧长统计，不能因时序优化重新定义评价区域。两条停车区内部查询的历史 0% 偏好问题仍存在；本轮未修复路径质量问题。', '',
table(['固定查询','A/B 成功','长度 A/B（m）','目标带比例 A/B','全部轨迹一致'],[[p['query'],f"{p['baseline']['success']}/6 · {p['candidate']['success']}/6",f"{num(p['baseline']['length_m'])} / {num(p['candidate']['length_m'])}",f"{num(None if p['baseline']['right_band_ratio'] is None else 100*p['baseline']['right_band_ratio'],1)}% / {num(None if p['candidate']['right_band_ratio'] is None else 100*p['candidate']['right_band_ratio'],1)}%",p['all_paths_identical']] for p in d['paired_queries']]),'',
'补充场景单列，不与主查询成功率混合：', '',
table(['场景','成功','有效轨迹终验','右→左→右恢复','候选路线恢复成功'],[[k,f"{s['success']}/{s['n']}",'通过' if not s['accepted_invalid'] else '失败',str(s.get('right_left_right_pose_restored','不适用')),s['candidate_recoveries']] for k,s in d['controls'].items()]),'',
'直路及旋转场景的目标带弧长占比均为 100%，切回靠右后轨迹复原。窄通道、跨区域掉头及首选候选故障恢复通过。没有增加实机执行或动态障碍能力。','',
'## 4. 语义更新与故障注入','',
table(['地图','更新成功','耗时中位 / P95（s）','局部补发','整图回退'],[[k,f"{s['success']}/{s['requested_updates']}",f"{num(s['elapsed_s']['p50'])} / {num(s['elapsed_s']['p95'])}",s['gap_repairs'],s['full_fallbacks']] for k,s in d['stress'].items()]),'',
'共 100 次：校准小图 40、旋转小图 40、真实大图 20。使用生产固定的 `pinned_nav2_effective_master` 生成期望主地图；没有将服务返回值反用为期望答案。','',
table(['注入','暖机后注入结果','注入更新耗时','说明'],[[k.removeprefix('final_fault_'),'恢复成功' if v['updates'][1]['success'] else '正确拒绝',num(v['updates'][1]['elapsed_s'])+' s',f"局部补发 {v['updates'][1]['gap_repairs']} 次" if v['updates'][1]['success'] else 'ACK 拒绝；本测试不调用规划器'] for k,v in d['faults'].items()]),'',
'缺块与旧块均在暖机成功后明确注入 1 次，靠 1 次局部补发恢复，约 0.53 s。持续错误内容、旧时间戳及序列改变均被拒绝；约 6.07 s 包含原 ROI 3 s ACK 窗口和完整回退后的另一个 3 s 窗口，不应称为 3 s 端到端上限。','',
'## 5. 进程退出后的资源清理','',
'监督进程记录本次进程创建时间及实际使用的新 FastDDS 文件身份（设备号/inode/uid）。正常退出或工作进程异常退出后，先确认所属 ROS 子进程终止，再清理无人使用的归属文件。监督进程被 SIGKILL 时，由下次同结果父目录的监督启动依据日志恢复。原有文件、其他进程使用的文件、同名不同 inode 的新文件均不删除；无法核实占用时保留并报告。','',
table(['类型','次数','结果'],[[m,sum(r['mode']==m for r in d['lifecycle']['rows']),'全部最终清理完成'] for m in dict.fromkeys(r['mode'] for r in d['lifecycle']['rows'])]),'',
'12/12 达到预期，最终记录中仍存活的所属进程及残留归属文件均为 0。进程被 SIGKILL 不能执行自己的 finally，外层监督负责收尾；监督本身被 SIGKILL 时依赖下次启动恢复。不能保证删除在崩溃前尚未被归属日志观察到的文件，也不自动清理全机历史共享段。这是入口监督集成，不是已部署的常驻系统服务。','',
'## 6. 保留的失败、验证与复现','',
'- Python 侧第一版组合：60/60，但正常主查询发生 40 次缺口补发、尾延迟未改善；第二版延迟探测小批次仅 1/4 成功。均保留在 `results/formal_2_combined` 和 `results/pilot_v2_combined`，未合并冒充最终通过。',
'- 早期压力测试：两组小图 80 次通过，真实大图首次失败。曾误按流程退出码报告“100 次通过”，已纠正；最终 100/100 来自上面三组明确计数的新结果。',
'- 旧压力夹具的 EDT 模型与生产模型差 723 格，但旧真实失败有 946,952 格失配，不能归因于夹具。夹具修正和服务端快照加锁分别记录。',
'- 初步按系统 apt 包构建的 1.1.20 仅为探索；发现实际动态加载的是工作区 1.1.19 后，最终使用其源码副本，避免版本差异混入对照。','',
f"82 项回归通过。基线 {d['integrity']['baseline_files']} 个文件与生产 costmap 源码均未变；最终冻结源文件、参数、车辆扫掠库核对通过；实际 Smac 插件哈希两组相同。完整结果见 [performance.json](performance.json)、[路径评价](path_quality.json)、[验收条件](acceptance.json)、[预定执行记录](final_dispatch.jsonl)。",'',
'复现命令、补丁构建方式见 [README](../README.md)，接口边界见 [INTERFACES](../INTERFACES.md)。下一步推荐处理停车区剩余偏好和困难机动覆盖，同时进入长期压力与目标服务入口集成；动态障碍贯通按后续计划推进，见 [NEXT_STAGE_PLAN](../NEXT_STAGE_PLAN.md)。','']
(R/'report/performance.md').write_text('\n'.join(lines))
