"""Populate the initialized Feishu candidate after the complete evidence exists."""
import json,html
from pathlib import Path
W=Path('/home/robot/workspaces/semantic_dual_map_r1_20260914');P=W/'report'
s=json.loads((P/'statistics.json').read_text());s['by_arm']={a:s['by_arm'][a] for a in ['original_r1','A','B','C']};v=json.loads((P/'report_values.json').read_text());dynamic=json.loads((W/'results/dynamic_formal_01/runs.json').read_text());gates=v['gates']
def esc(s):return html.escape(str(s),quote=True)
def p(s):return '<p>'+esc(s)+'</p>'
def h(s):return '<h1 seq="auto">'+esc(s)+'</h1>'
def img(n,caption):return f'<img path="@./workspaces/semantic_dual_map_r1_20260914/report/figures/{n}.png" caption="{esc(caption)}"/>'
def tab(headers,rows):return '<table><thead><tr>'+''.join('<th><p>'+esc(c)+'</p></th>' for c in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td><p>'+esc(c)+'</p></td>' for c in r)+'</tr>' for r in rows)+'</tbody></table>'
def val(x):return '未定义' if x is None else f'{x:.2f}'
cal=[r for r in s['controlled'] if r['scene']=='calibration'];hold=[r for r in s['controlled'] if r['scene']=='heldout']
first=next(r for r in v['preparation_timings'] if r['phase']=='cold_connection_validation_and_serialization');rest=next(r for r in v['preparation_timings'] if r['phase']=='graph_cache_restore')
parts=['<title>[实验记录] 语义双地图与参考路径偏好验证 - 李永祺</title>',
p(f"首轮已完成：严格接入 3D-V1-r1，真实地图 {v['real_success']}/240 次、受控对照 {v['controlled_success']}/30 次、障碍增删 {v['dynamic_success']}/3 次成功，冻结验收{'全部通过' if all(gates.values()) else '存在未通过项，详见记录'}。靠右偏好能够从 L2 传到 L3 最终轨迹，已接受轨迹的独立安全校验违规为 {len(s['accepted_invalid_trajectories'])} 次，原 r1 成功而 C 失败为 {len(s['regressions'])} 次。本结论限于当前车辆与冻结场景，构图缓存较大，尚未进入耗时优化阶段。"),
p('作者：李永祺；汇报对象：袁梦佳。本轮为真实地图离线规划实验，未进行实体机器人试跑。来源：real_formal_03、controlled_formal_04、dynamic_formal_01；完整逐次证据与失败记录见文末。'),
h('实验条件'),p('表 1　冻结配置；来源 acceptance.yaml、queries.yaml、SELECTED_WEIGHTS.json。'),
tab(['项目','本轮配置'],[['真实主图','万达三层，现有人工标注；分辨率 0.05 m'],['查询与重复','20 条真实查询 × 4 组 × 3 次；受控校准直道与旋转留出场景'],['车辆与预算','带 padding 半长/半宽 0.265/0.225 m；最小转弯半径 0.40 m；前进；Smac 5 s，action 7 s'],['固定权重','L2=2.0；L3 参考代价上限 140；距离 2 m 饱和，全局统一'],['偏好门槛','目标带弧长占比 ≥80%；比 A 提升 ≥20 个百分点；左右横移 ≥0.50 m']]),
h('同源地图与通行连接'),
img('semantic_pose_graph','图 1　真实地图局部的语义与位姿连接；橙色为 L1 路线，蓝色内部或临时端点位姿，紫色接口位姿。'),
p('想做什么：让图与栅格来自同一份语义标注，并把方向和转弯规则落实到连接可用性。方案：接口位姿负责区域拼接，必要内部位姿负责长区域接入，每个方向分别验证车体扫掠与 Dubins 转弯。'),
p('采用：有向节点表与连接表，图 A* 输出路线；同源安全栅格供 r1 的 compact D* Lite 使用，回退 A* 共用相同边代价。禁转连接不可用，未验证连接不进入搜索。'),
p(f"结果：当前 {v['prep']['nodes']:,} 个节点、{v['prep']['edges']:,} 条已验证有向连接。规则、接口、缓存与独立检查等 19 项新机制测试全部通过，原 r1 的 30 项测试也通过。来源：preparation_profile_01、final_unit_regression.xml。"),
h('偏好从栅格传到最终轨迹'),
img('controlled_abc','图 2　40 × 6 m 直道，从上到下 A、B、C；蓝色虚线 L2，绿色 L3，黄色右侧目标带；黑圆起点、黑菱形终点。'),
p('想做什么：区分 L2 路径靠右与最终轨迹靠右。方案：A 无软偏好，B 仅 L2 偏好，C 加入 L3 参考；三组使用相同允许走廊。'),
p('采用：将 L2 偏离距离转成有限、非负、有上限的软代价，写入 Smac 实际搜索读取的代价地图；精确确认服务器内容后再规划。目标带按路线方向计算，首尾各去掉 5 m，以 0.025 m 弧长积分。'),
p(f"结果：校准场景 3 次重复中，A/B 最终目标带占比均为 {cal[0]['A_band_ratio']:.0%}/{cal[0]['B_band_ratio']:.0%}，C 为 {min(min(r['C_right_band_ratios']) for r in cal):.0%}，{'达到' if all(r['pass'] for r in cal) else '未达到'}冻结门槛。来源：controlled_formal_04；图示为第 1 次重复，结论使用全部重复。"),
h('方向响应与旧代价移除'),
img('controlled_mirror','图 3　绿色靠右、紫色靠左；左为 40 × 6 m 校准场景，右为 8 × 48 m 旋转场景。'),
p('想做什么：确认偏好随行驶方向旋转，并能在障碍不变时触发 L3。方案：同一会话执行靠右→靠左→靠右，核对参考版本与旧代价移除。'),
p('采用：有序路线匹配；平行、交叉或重复路段出现歧义时停止吸引。仅参考变化也重新评估 L3，保留原硬走廊。'),
p(f"结果：校准/旋转场景的左右横移分别为 {cal[0]['mirror_signed_arc_lateral_shift_m']:.2f}/{hold[0]['mirror_signed_arc_lateral_shift_m']:.2f} m；全部 3 次重复方向正确。切回轨迹哈希{'全部一致' if all(r['right_switchback_pose_hash_matches'] for r in s['controlled']) else '存在差异'}，软代价内容确认记录完整。来源：controlled_formal_04。"),
h('真实地图转场验证'),
img('real_routes','图 4　多路口、车道到停车区、停车区内部；蓝色虚线 L2、绿色最终轨迹，黑色障碍，白色可走。'),
p('想做什么：检查新图和软参考是否损失原有可达性。方案：保留原 r1，配对运行 A/B/C；每条接受轨迹都经过原 PathAudit 和独立连续车体、运动学、规则、端点检查。'),
p('采用：固定同一车辆、地图和求解预算；连续包络检查的分段平移 ≤0.025 m、朝向 ≤1°，曲率上限 2.500001 m⁻¹。软参考允许因转弯和安全避障偏离。'),
p('表 2　真实地图全部 240 次；端到端包含 L1、L2、适配、内容确认、Smac 与检查，单位秒。p95 是 20 条查询及其重复的描述值，不能作为总体尾延迟保证。来源 real_formal_03。'),
tab(['算法','有效成功/全部','总耗时 P50/P95 (s)'],[[a,f"{x['success']}/{x['count']}",f"{x['end_to_end_wall_ms']['p50']/1000:.2f}/{x['end_to_end_wall_ms']['p95']/1000:.2f}"] for a,x in s['by_arm'].items()]),
p(f"结果：原 r1 成功→C 失败 {len(s['regressions'])} 次；已接受的违规轨迹 {len(s['accepted_invalid_trajectories'])} 次。复杂场景 C 的条件偏好保持率为 {val(None if v['retention_weighted'] is None else v['retention_weighted']*100)}%：只统计能匹配到 L2 目标带的 L3 弧长，歧义处排除并另记；它不等于所有道路都能持续靠右。来源：derived/preference_retention.json。"),
h('障碍增加与恢复'),
img('dynamic_recovery','图 5　从上到下为初始、障碍增加、障碍移除；红色新关闭格、灰色原不可走区域、绿色最终轨迹。'),
p('想做什么：验证确认障碍变化能到达 L2 与 L3，恢复后旧硬代价可移除。方案：在原 L2 中段添加障碍，再移除，各经过两次观测确认。'),
p('采用：保留 r1 更新调度，增量 D* 未完成时走同代价 A* 回退；硬走廊或参考变化都触发 L3，服务器精确内容确认后接收结果。'),
p(f"结果：{v['dynamic_success']}/3 个阶段成功，新增与移除后均触发 L3。单次更新序列只证明链路，不代表已覆盖动态交通。来源：dynamic_formal_01。"),
h('开销与遗留问题'),
p(f"空缓存构建中，连接验证及序列化用时 {first['wall_ms']/1000:.2f} s，图缓存恢复 {rest['wall_ms']/1000:.2f} s；完整主图缓存 {v['prep']['cache_bytes']/1024**3:.2f} GiB。这是实测成本，未隐藏到免费预处理，也不作提速结论。来源：preparation_profile_01。"),
p('优先处理连接路径存储、重复偏好/参考计算和大地图内容传输；性能目标在下一阶段开始前冻结。当前只有一张真实主图；原标注没有明确单行方向，14 条围栏的硬约束属性待确认；单行与禁转机制以明确受控规则验证。'),
p('受控复测另有一次启动内容读取失败，记录在 controlled_formal_02，改用每场景独立 ROS 通信域复测；controlled_formal_03 的旋转场景另因通信域配置超出范围未启动，修正后完整重跑。正式真实查询有 6 次初始内容确认失败，经既有重发并精确确认后成功；启动和传输可靠性仍需关注。两次较早真实地图正式尝试分别因内容传输、ROS 整数坐标类型错误中止，失败与进行中目录均保留；最终版修复输入类型后完整重跑，数值坐标、权重和验收阈值未变。原预留查询已发生过输入尝试，最终轮称为复测，不再宣称未暴露。受控旋转留出首次运行在权重冻结之后，没有按其结果调权重。'),
p(f"原 r1 严格回归的 6 份轨迹文件与历史结果{'逐字节一致' if v['strict_path_bytes_match'] else '尚未一致'}；包含运行来源信息的综合审计哈希不同，已单列说明；历史四倍地图内存门槛本来未通过，与本轮分开。环境未找到 SimSun，所有证据图不放内嵌文字，图例、尺度与数据写在正文，未冒用其他字体。"),
h('记录入口'),
p('隔离目录：/home/robot/workspaces/semantic_dual_map_r1_20260914。完整报告 report/full_report.md；门槛 report/acceptance_results.json；统计 report/statistics.json；各版本失败 report/attempt_manifest.json；源码与运行开关 README.md。完整原始轨迹、日志、独立校验与 source_snapshot 均在对应 results 目录。'),
p('原工作区 662 个已核对源文件内容保持不变，七个 r1 核心文件哈希一致。本轮完成隔离实验与汇报材料，未合并回原仓库、未发布正式软件版本、未向导师发送消息。'),
'<p>设计来源：<a href="https://pudutech.feishu.cn/docx/QSYzd4kcmoVlktx5Co6caOhTnab">语义双地图方案第 1 节</a>；<a href="https://pudutech.feishu.cn/docx/YSazdsoP5oTzy4xePtpcJ4D1nAe">节点与连接详细设计</a>。历史 STL 结果不是本轮证据。</p>']
out=Path('/home/robot/draft_ea02fa5b_folder/draft.xml');out.write_text('\n'.join(parts));(P/'feishu_candidate.xml').write_text(out.read_text());print(out)
