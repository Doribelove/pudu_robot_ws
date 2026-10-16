"""Render the frozen run analysis as reviewable Markdown, without graphics."""
import json
from pathlib import Path
from collections import defaultdict
from arena_3d_v1.semantic_world import WORK
s=json.loads((WORK/'report/performance.json').read_text());a=s['arms']['3YD-V1'];b=s['arms']['recovery']
f=lambda n: '—' if n is None else f'{n:.2f}'
def ratio_change(x,y):return f'{(y/x-1)*100:+.1f}%'
lines=['# 基于 3YD-V1 的静态质量与失败恢复实验结果','',
'已实现接口多位置采样、路口附近的偏好方向修正、局部精细连接和有界的 L1 候选失败恢复。L3 原始底图、Smac、车辆及最终检查保持不变。本目录为独立实验副本，未另行赋予新的正式版本名。','',
'本轮真实地图固定查询全部通过，停车区额外绕行和完整精细重搜减少。新增跨区域掉头场景仍存在固定 Smac 平滑后曲率超限，结果继续拒绝；不能把该限制包含在“全部成功”的说法中。','',
'## 同条件真实地图对照','',
'同一地图、20 条固定查询、每条 3 次；三次重复不是独立场景。原 3YD-V1 使用原目录原脚本复测，修复版使用冻结后的新脚本。CPU 固定为 15、串行运行、查询缓存分别独立。首次查询表示本查询的应用缓存未命中，L1 图已准备；所有失败尝试、回退、地图修复和双重轨迹检查包含在查询耗时内。','',
'| 指标 | 3YD-V1 本轮复测 | 修复版 | 变化 |','| --- | --- | --- | --- |',
f"| 最终成功 / 次数 | {a['success']}/{a['n']} | {b['success']}/{b['n']} | 固定集成功率保持 |",
f"| 被接收但独立检查不通过 | {a['accepted_invalid']} | {b['accepted_invalid']} | 检查门槛不变 |"]
for name,key in [('首次查询中位数','first'),('重复查询中位数','repeat')]:
 x=a['timing_s'][key]['p50'];y=b['timing_s'][key]['p50'];lines.append(f'| {name} | {x:.2f} s | {y:.2f} s | {ratio_change(x,y)} |')
for name,key in [('全部查询中位数','p50'),('全部查询 P95','p95'),('全部查询最大值','max')]:
 x=a['timing_s']['all'][key];y=b['timing_s']['all'][key];lines.append(f'| {name} | {x:.2f} s | {y:.2f} s | {ratio_change(x,y)} |')
x,y=a['peak_rss_gib'],b['peak_rss_gib'];lines.append(f'| 查询进程树峰值 RSS | {x:.2f} GiB | {y:.2f} GiB | {ratio_change(x,y)} |')
for title,mode in [('纯粗图 L2 完成','coarse_only'),('局部精细辅助完成','local_fine_assisted'),('完整精细回退完成','full_fine_fallback')]:
 lines.append(f"| {title} | {a['completion_modes'].get(mode,0)}/60 | {b['completion_modes'].get(mode,0)}/60 | L3 始终为 0.05 m |")
lines+=['',f"本轮地图重发修复次数：3YD-V1 为 {a['map_repair_count']}，修复版为 {b['map_repair_count']}，均已计入耗时。进程树 RSS 包含共享页重复计数，不等同于整机增量占用。系统波动和传输修复仍影响尾部耗时，不将单批中位数之比视为稳定实时上限。",'',
'## 路径质量','',f"公共细图靠右目标带占比 P50：{a['right_band_ratio']['p50']*100:.2f}% → {b['right_band_ratio']['p50']*100:.2f}%。按相同查询配对，路径长度比中位数为 {s['paired_length_ratio']['p50']:.4f}，最大为 {s['paired_length_ratio']['max']:.4f}。以下指标均以同一份原 0.05 m 评估场计算，沿弧长采样、去掉两端各 5 m，并排除不适用或歧义段；不能用改变评价场提高分数。",'',
'| 查询 | V1 长度 m | 修复后长度 m | V1 靠右占比 | 修复后靠右占比 | 完成方式 |','| --- | --- | --- | --- | --- | --- |']
mode_names={'coarse_only':'粗图','local_fine_assisted':'局部精细辅助','full_fine_fallback':'完整精细回退'}
for item in s['paired_queries']:
 x,y=item['3YD-V1'],item['recovery'];lines.append(f"| {item['query']} | {f(x['length_m'])} | {f(y['length_m'])} | {f(100*x['band_ratio'])}% | {f(100*y['band_ratio'])}% | {mode_names[y['mode']]} |")
worst=min(s['paired_queries'],key=lambda x:x['band_delta_pp']);best=max(s['paired_queries'],key=lambda x:x['band_delta_pp'])
lines+=['',f"偏好逐查询变化最大改善为 {best['query']} 的 {best['band_delta_pp']:+.2f} 个百分点；最大下降为 {worst['query']} 的 {worst['band_delta_pp']:+.2f} 个百分点。两条停车区内部查询的有效车道段占比仍为 0%，与 V1 一样，尚未解决这两条查询的低偏好指标；这不表示整个停车区都应适用靠右规则。",'',
'两条停车区内部查询在 3YD-V0-r0 中分别约为 49.02 m 和 49.17 m。V1 的约 9.5% 额外绕行主要由单个接口代表位置导致的候选选择偏差引起；增加接口位置后，恢复了更合适的区域序列。局部细图本身并未单独解决这一绕行，首轮仅加局部细图的对照记录保留在 `results/local_pilot_01`。','',
'首轮把车道主轴稳定施加到整条细长车道时，出现了普通车道偏好下降。最终将处理收窄到实际访问路口前后约 3 m；前一轮正式结果与源码分别保留在 `report/performance_iteration_01.json`、`report/ITERATION_01_FREEZE.json` 和 `results/formal_recovery`，最终版本数据位于 `results/formal_recovery_final`。','',
'## 局部精细连接','',
'六条旧回退查询的起终点在保守粗图中有效，断连在途中。先用允许“粗格内存在细安全格”的图定位候选局部区域，再在原细安全格上验证所有连接，定位图不能直接输出给 L3。混合图中的 A* 保留同一非负偏好加权形式，细格只在受限区域展开；速度收益包含搜索范围缩小与混合 A*，不能全部归因于栅格数量。','',
'| 原回退查询 | 当前完成方式 | 局部细安全格数 |','| --- | --- | --- |']
for item in s['paired_queries']:
 if item['3YD-V1']['mode']=='full_fine_fallback':
  y=item['recovery'];lines.append(f"| {item['query']} | {mode_names[y['mode']]} | {y['local_fine_cells'] or 0} |")
lines+=['','局部细安全格上限为 32,000；超过预算、局部无路或无法保留粗图跨步时，仍允许进入原完整精细回退。没有移除这个兜底，也没有把局部辅助算作纯粗图成功。','',
'## L1 的额外成本','',
'物理接口数和语义子区域切分保持不变，每个接口由一个位置增加到最多三个位置；短接口去重后，不是所有节点数都恰好乘三。候选边仍按需展开，未恢复离线 Dubins 轨迹预计算和存储。','',
'| 指标 | V1 | 修复版 |','| --- | --- | --- |']
for title,key in [('接口状态节点','nodes'),('有向候选搜索边','candidate_directed_edges'),('物理接口','physical_portals')]:lines.append(f"| {title} | {a['graph'][key]:,} | {b['graph'][key]:,} |")
lines.extend([f"| 构建与落盘 | {a['graph']['build_and_write_ms']/1000:.3f} s | {b['graph']['build_and_write_ms']/1000:.3f} s |",f"| 图缓存 | {a['graph']['graph_cache_bytes']/2**20:.3f} MiB | {b['graph']['graph_cache_bytes']/2**20:.3f} MiB |",'',
'## 失败恢复与补充场景','',
'候选失败会触发请求内备选搜索，最多 4 条实际候选、64 次 L1 搜索。不能定位的失败不关闭通道；明确处于静态硬障碍内的接口样本只在本请求内排除。地图内容、版本和缓存完整性失败不会被换路掩盖。备选路线仍必须通过固定 Smac 和两套轨迹检查。','',
'| 场景 | V1 成功 / 实际执行 | 最终修复版成功 / 实际执行 | 解释 |','| --- | --- | --- | --- |'])
agg=defaultdict(lambda:defaultdict(lambda:{'n':0,'success':0,'recoveries':0,'min_band':1.,'restored':True}))
for item in s['controls'].values():
 t=agg[item['scene']][item['arm']];t['n']+=item['n'];t['success']+=item['success'];t['recoveries']+=item['recoveries'];t['min_band']=min(t['min_band'],item.get('minimum_target_band_ratio',1.));t['restored']&=item.get('right_left_right_pose_restored',True)
explanations={'calibration':'右→左→右','rotated':'旋转后的右→左→右','narrow':'0.95 m 窄通道，保留完整细图兜底','two_routes':'自然生成的双通道地图','two_routes_optimistic_edge':'显式候选图错误注入，不是自然生成错误率','return_bulb':'邻近目标需要跨区域掉头','return_bulb_rotated':'旋转后的跨区域掉头'}
for scene,desc in explanations.items():
 x=agg[scene]['topology_015'];y=agg[scene]['recovery'];lines.append(f"| {scene} | {x['success']}/{x['n']} | {y['success']}/{y['n']} | {desc} |")
lines+=['',f"左右偏好受控场景的最终修复版最低目标带占比为 {min(agg[k]['recovery']['min_band'] for k in ['calibration','rotated'])*100:.1f}%，右→左→右恢复检查为 {all(agg[k]['recovery']['restored'] for k in ['calibration','rotated'])}。错误候选注入场景中，最终修复版实际完成失败后换路 {agg['two_routes_optimistic_edge']['recovery']['recoveries']} 次；V1 两次已执行的基线运行共 6 次失败均保留，因此该行分母与最终修复版不同。",'',
'跨区域掉头两个场景中，候选恢复能切换到包含掉头空间的区域，但固定 Smac 最终输出轨迹的曲率超过 2.5 m⁻¹，最终检查拒绝了这些路径。保留失败是当前正确行为；此轮不能宣称已解决任意跨区域掉头或运动学困难场景。也未实现完整动态障碍闭环与行驶中的路线切换。','',
f"另有 {len(s['startup_failures'])} 次补充场景启动在地图 ACK 阶段失败，尚未进入查询循环，单列记录于 `performance.json/startup_failures`。前期存在共享内存耗尽，清理后同一域仍出现一次启动失败；换用新的隔离 ROS 域后完成测试，启动问题的全部根因尚未隔离。各失败目录、清理范围和重试日志均保留，未修改 ACK 检查标准。主地图两组正式查询均成功完成。",'',
'## 校验与复现','',
'最终原有及新增回归为 60 项通过；另以新进程核对 20 条真实查询的缓存恢复后参考路与精细通道。',
f"源文件核查：原 V1 {s['source_integrity']['base_files_checked']} 个文件变化列表为 `{s['source_integrity']['base_changed']}`；最终冻结 {s['source_integrity']['implementation_files_checked']} 个文件变化列表为 `{s['source_integrity']['implementation_changed']}`；Smac 参数一致：{s['source_integrity']['same_smac_parameters']}；原世界/软代价适配/L3 模块与车辆扫掠库保持一致：{s['source_integrity']['original_world_reference_l3_unchanged'] and s['source_integrity']['sweep_binary_unchanged']}。",'',
'- [接口定义](../INTERFACES.md)',
'- [最终统计与全部分母](performance.json)',
'- [逐轨迹偏好指标](path_quality.json)',
'- [最终回归检查](regression_final_03.xml)',
'- [缓存恢复核对](cache_restore_validation.json)',
'- [复现与文件说明](../README.md)','',
'后续应优先诊断跨区域掉头时的参考意图与固定 Smac 平滑后曲率问题，以及启动地图 ACK 的稳定性；再推进同源动态障碍和 L1 动态通道状态。']
(WORK/'report/performance.md').write_text('\n'.join(lines)+'\n')
print('wrote',WORK/'report/performance.md')
