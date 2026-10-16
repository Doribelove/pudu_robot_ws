"""Render the checked raw comparison into a reproducible local report."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def f(x,d=3):return '不适用' if x is None else f'{x:.{d}f}'
def delta(a,b):return '不适用' if a in (None,0) or b is None else f'{100*(b/a-1):+.1f}%'
def main():
    c=json.loads((ROOT/'report/COMPARISON.json').read_text());maps=c['maps']
    text=['# 3YD-V8-r3 首次恢复成本优化与跨地图验证','',
      '采用A＋B作为本轮静态候选：L2批量调用原车体检查，并对少数复杂长路线提前使用原有界分段策略。L1继续封存，车辆、地图解释、L2参考及软目标、L3范围和严格终验不变。原两图固定10×3仍为60/60；额外地图结果单独列出，历史生产入口不替换。','',
      '## 两项独立改动','',
      '| 改动 | 实现与作用 | 保留的约束 |','|---|---|---|',
      '| A：批量精确姿态检查 | 对一个候选格的9个子位置×24个朝向，以原顺序在原生循环中调用原碰撞函数；复用此前已计算的ROI世界坐标索引。减少Python逐次调用及重复坐标转换。 | 每个姿态独立检查，不把替代姿态组成轨迹；第一个合法姿态、证书与缓存规则不变。 |',
      '| B：复杂长路线先分段 | 无语义参考≥250 m、矩形桥接格≥64、≥4处30°转角时，先同步并确认同一costmap，再使用原分段方法。 | 这是调度启发式，不是不可达证明；最多4段共享8秒，失败保留一次原有界整段动作，不再重复已失败分段。 |','',
      'B不按地图或查询编号选策略。准备与ACK计入当前端到端；8秒分段恢复窗在准备之后开始，与原先整段动作完成后启动恢复的语义一致。整段动作原7秒限额不改，最多5次车辆动作、每次0.8/1.2 m且≤40万格和全图10%。若新分段策略失败，清除部分轨迹，并缓存此输入的拒绝结果（最多16条）；相同输入不重复预先分段。各段通过仍须整条拼接路径终验。','',
      '## 同配置正式对照','',
      '各组独立新进程、空查询缓存，CPU12串行、BLAS单线程、无剖析器。control关闭本轮A/B，保留r2 AB；它是本批主对照。万达3w地库使用原Q01–Q10，arena_large使用V7登记A2B-01–10，每组连续3次。首次10次／重复20次分别统计。一次性地图加载、建图、ROS启动和导出不计入查询。','',
      '| 地图 | 严格成功 对照→r3 | 查询中位 / s | P95 / s | 最大 / s | 首次中位 / s | 重复中位 / s |','|---|---:|---:|---:|---:|---:|---:|']
    for mid,v in maps.items():
      a,b=[v['arms'][arm] for arm in ('control','AB')];am,bm=a['metrics']['e2e_s'],b['metrics']['e2e_s']
      cells=[f"{f(am[s][k])}→{f(bm[s][k])} ({delta(am[s][k],bm[s][k])})" for s,k in [('all','median'),('all','p95'),('all','max'),('first','median'),('repeat','median')]]
      text.append('| '+v['label']+f" | {a['success']}/30→{b['success']}/30 | "+' | '.join(cells)+' |')
    text+=['','计时包含层间适配、地图确认、车辆动作、修复和终验；L3流水线已包含适配，各行中位数不能相加。查询中位数受到20次重复样本的影响，不能代替首次查询表现。','',
      '![正式对照与独立消融箱线图](r2_r3_boxplots.png)','',
      '## 路径质量','',
      '| 地图 | 靠右比例中位 对照→r3 | 最终路径长度中位 对照→r3 / m | 路径配对 |','|---|---:|---:|---:|']
    for mid,v in maps.items():
      a,b=v['arms']['control'],v['arms']['AB'];right='不适用' if b['right']['n']==0 else f"{f(a['right']['median']*100,2)}%→{f(b['right']['median']*100,2)}%"
      text.append(f"| {v['label']} | {right} | {f(a['length']['median'])}→{f(b['length']['median'])} | {v['parity']['AB']['counts']['path.npy']}/30逐点相同 |")
    text+=['','## 独立A与B的收益','',
      '四组arena_large均完整30次，不能把A或B诊断子集当成完整验收。','',
      '| 开关 | 严格成功 | 查询中位 / s | P95 / s | 最大 / s | 峰值RSS中位／最大 / GiB |','|---|---:|---:|---:|---:|---:|']
    free=maps['arena_large_250x300_2d_005']
    for arm in ('control','A','B','AB'):
      x=free['arms'][arm];m=x['metrics']['e2e_s']['all'];r=x['metrics']['tree_rss_gib']['all']
      text.append(f"| {arm} | {x['success']}/30 | {f(m['median'])} | {f(m['p95'])} | {f(m['max'])} | {f(r['median'])}／{f(r['max'])} |")
    text+=['','A的主要收益位于矩形断连恢复，普通路线和重复命中并不保证加速；B只改变符合条件的复杂长路线首次策略。全部组峰值与中位变化保留，不把计时波动归因于没有改动的L1或Smac算法。','',
      '## A2B03首次恢复案例','',
      '| 组 | 首次端到端 / s | 首次L2处理 / s | 首次L3流水线 / s | 首次Smac动作数 |','|---|---:|---:|---:|---:|']
    for arm,folder in [('control','control'),('A','ablation_A'),('B','ablation_B'),('AB','formal')]:
      rs=[json.loads(x) for x in (ROOT/'results'/folder/'arena_large_250x300_2d_005/runs.jsonl').read_text().splitlines()]
      r=next(x for x in rs if x['query_id']=='A2B-03' and x['repeat']==1);d=r['recovery']
      text.append(f"| {arm} | {f(r['wall_ms']/1000)} | {f(d['all_candidates_controller_ms']/1000)} | {f(d['all_candidates_l3_pipeline_ms']/1000)} | {r['l3']['backend_call_count']:.0f} |")
    text+=['','对照先整段搜索，再执行4段恢复；r3先做相同内容ACK，再执行4段并完整终验，少一次整段失败动作。后两次仍执行真实4段规划，不复用最终轨迹。此查询的L1、L2、软目标及最终轨迹相同；每组只有3次，案例展示使用逐点图。','',
      '![首次与重复恢复样本](A03_points.png)','',
      '## 各层耗时','',
      '以下每格为“control→r3”，首次／重复中位分别有10／20个样本；L1为本次routes调用。','',
      '| 阶段 | 万达3w首次 | 万达3w重复 | arena_large首次 | arena_large重复 |','|---|---:|---:|---:|---:|']
    for metric,label in [('l1_ms','L1 / ms'),('l2_s','L2整体 / s'),('dstar_s','L2实际D* / s'),('adapter_s','层间适配 / s'),('l3_s','L3流水线 / s'),('map_sync_s','地图处理与ACK / s'),('smac_action_s','Smac动作累计 / s'),('curvature_repair_s','局部修复累计 / s'),('strict_audit_s','完整终验累计 / s')]:
      vals=[]
      for mid in ('mengjia','arena_large_250x300_2d_005'):
        for split in ('first','repeat'):
          vals.append('→'.join(f(maps[mid]['arms'][arm]['metrics'][metric][split]['median']) for arm in ('control','AB')))
      text.append('| '+label+' | '+' | '.join(vals)+' |')
    text+=['','重复D*时间为0表示复用参考，控制器、适配和终验仍按当前调用计时。全部层级P95、最大值和逐次样本见COMPARISON.json。','',
      '## 分层资源','',
      '| 指标 | 万达3w control中位／最大 | 万达3w r3中位／最大 | arena_large control中位／最大 | arena_large r3中位／最大 |','|---|---:|---:|---:|---:|']
    for metric,label in [('l2_state_mib','L2注册状态 / MiB'),('python_pss_gib','Python工作进程PSS快照 / GiB'),('planner_pss_gib','Smac进程峰值PSS / GiB'),('stack_pss_gib','监测服务栈峰值PSS / GiB'),('tree_rss_gib','查询进程树峰值RSS / GiB')]:
      vals=[]
      for mid in ('mengjia','arena_large_250x300_2d_005'):
        for arm in ('control','AB'):
          s=maps[mid]['arms'][arm]['metrics'][metric]['all'];vals.append(f(s['median'])+'／'+f(s['max']))
      text.append('| '+label+' | '+' | '.join(vals)+' |')
    text+=['','L2登记状态不等于L2完整内存；Python包含地图、L1/L2和部分L3处理。服务栈仅含planner、map、lifecycle且已含planner；PSS快照、PSS采样峰值和完整进程树RSS峰值不同，不能相加。L1独立内存未重新隔离测量，L1本轮未改。A单项的原生调用更少，不代表整套进程分配器RSS必降。','',
      '## 额外两张地图','',
      '各图固定10组各1次，同配置新进程control和AB对照；合成地库A由fixed10_binding.resolve加载并验证，arena_medium由V7登记核验。额外地图与原两图重复样本分别统计。','',
      '| 地图 | 严格成功 对照→r3 | 端到端中位 / s | P95 / s | 峰值RSS中位／最大 / GiB |','|---|---:|---:|---:|---:|']
    for mid,v in c.get('additional',{}).items():
      a,b=v['control'],v['AB'];ma,mb=a['metrics']['e2e_s']['all'],b['metrics']['e2e_s']['all'];ra,rb=a['metrics']['tree_rss_gib']['all'],b['metrics']['tree_rss_gib']['all']
      text.append(f"| {v['label']} | {a['success']}/10→{b['success']}/10 | {f(ma['median'])}→{f(mb['median'])} | {f(ma['p95'])}→{f(mb['p95'])} | {f(ra['median'])}/{f(ra['max'])}→{f(rb['median'])}/{f(rb['max'])} |")
      for failure in b['failures']:
        stage=failure.get('section_failure') or '；'.join(failure.get('final_failures') or []) or failure['failure']
        text.append(f"\n{v['label']} {failure['query']}：L2参考{'已生成' if failure['L2_reference_generated'] else '未生成'}，车辆阶段拒绝原因 {stage}。")
        if failure.get('final_steering_jump_deg') is not None:text.append(f"\n最大转向跳变{f(failure['final_steering_jump_deg'],2)}°，超过15°；各段通过不能替代拼接后终验。")
    if any(v['AB']['failures'] for v in c.get('additional',{}).values()):
      text+=['','arena_medium的4条失败在control中同样存在。本轮保住原通过案例，但未修复这些新的跨地图车辆连接问题：A2B-03/04第一段被连续车体／范围检查拒绝，A2B-06/08各段可通过而拼接后转向跳变超过15°。ACK已确认，开放格约18.5–19.4万且远低于空间预算，因此不能简单归因为地图未同步或开放面积超限，也不能据此证明原环境物理不可达。具体片段坐标与恢复证据保存于COMPARISON.json。',
      '', '新提前分段策略在四图只触发两个首次案例：arena_large A2B-03成功，arena_medium A2B-03第一段失败后按预算执行一次整段备选，仍失败。后者说明结构条件不能保证分段可行，不能将这项调度当成普遍成功率提升。']
    text+=['','![额外地图逐次分布](additional_boxplots.png)','',
      '## 验收与范围','',
      '89项检查通过，包括原碰撞函数的逐姿态等价性、静态证书次序、候选不能作为连续轨迹、冻结L1及输入、预算、失败后清除部分轨迹和保留一次整段备选。正式两图60次L1、L2、软目标和最终轨迹与新control逐项核对，原生Smac与costmap库、独立终验实现不变。每次车辆动作都记录ACK；监督器仅清理本轮拥有的进程和共享资源。','',
      '仅覆盖本次四图固定40组静态任务。更多地图、动态变化、真实执行及启发式误触发仍需独立验收。A2B-03之外普通查询的变化较小，不承诺普遍提速。早期A/AB诊断采用中间源码，仅保留排查记录，不作为最终版本性能证据。','',
      '## 下一步','',
      '保持本版静态候选，下一轮优先修复arena_medium A2B-06/08的整路径转向跳变，再定位A2B-03/04首个连续车体／范围失败片段。继续封存L1、不增宽通道、不放宽15°终验，以有界局部连接提高跨地图通过率。普通首次查询的L2场计算和层间适配作为独立性能对照，不与成功率修复混合。地图或障碍变化后重新验证，不能直接复用旧轨迹或跳过ACK。大规模存储裁剪与L1改造暂不开展。','',
      '## 交付与复现','',
      '- src/three_yd_v5/pose_batch.py、rectangle_bridge.py、controller.py：批量调用与坐标复用；native/POSE_BATCH_BUILD.json：原生内核编译记录。',
      '- src/three_yd_v5/early_sections.py、bounded_segments.py：独立调度与有界失败恢复。',
      '- results/control、ablation_A、ablation_B、formal、additional：原始记录、源文件、原生库、轨迹、范围与监督清理日志。',
      '- report/COMPARISON.json：原始计时统计、输入／路径／搜索状态配对、各层资源和额外地图结果。',
      '- README.md、INTERFACES.md、EXPERIMENT_PROTOCOL.md：复现、接口与验收；L1_LOCK.json：封存校验；RELEASE.json：最终文件校验。','']
    (ROOT/'report/RESULTS.md').write_text('\n'.join(text))
if __name__=='__main__':main()
