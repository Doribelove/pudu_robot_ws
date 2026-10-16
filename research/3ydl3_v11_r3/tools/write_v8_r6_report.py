"""Write findings directly from recorded, verified r6 comparisons."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

def fmt(x, digits=3):
    return '—' if x is None else f'{x:.{digits}f}'

def pair(a, b):
    if a is None or b is None:
        return fmt(a)+' → '+fmt(b)
    return fmt(a)+' → '+fmt(b)+f'（{100*(b/a-1):+.1f}%）' if a else fmt(a)+' → '+fmt(b)

def main():
    c = json.loads((ROOT/'report/COMPARISON.json').read_text())
    text = [
        '# 3YD-V8-r6：整图副本、层间传递与L3局部更新', '',
        '本轮从V8-r5建立独立工作区，选择ABC组合为新的静态实验候选。原两图仍60/60严格通过；额外合成地库A为10/10，arena_medium为8/10。四图40组不同任务仍38/40。80次L1、L2参考和软目标全部相同，原来成功的78次最终轨迹逐点相同，靠右及路径长度不变。', '',
        '主要收益是减少地图传输和临时内存，首次地图同步变快；整套规划尚未获得大幅提速。主两图首次端到端约快4%，P95约降7%；arena_large全部查询中位数反而升2.8%，重复查询基本持平。不能把读回量减少80%解释成规划快80%。', '',
        '## 改了什么', '',
        '| 项目 | 实现 | 保留的校验 |', '|---|---|---|',
        '| A：减少整图副本 | 已生成并关闭范围外格子的数组以只读视图传给发布环节；确认结果引用不可变数组。内容计算和精确ACK改为64行临时工作区 | 小范围恢复仍关闭多余格子；原硬、软、旧区及普通格逐格确认全部保留 |',
        '| B：层间变化块传输 | 原生costmap服务在锁内查看完整实际地图，发送相对上次收到版本的128×128变化块；客户端重建完整快照 | 版本、坐标、内容检查后继续完整精确ACK；丢失、损坏、重启和错版本须重同步 |',
        '| C：合并局部更新消息 | 原稀疏发布方式的单包上限128,000→512,000字节，减少消息数量；原15 ms发送间隔不变 | 扩大的是发送矩形，不是允许搜索范围；多发送的未变化格也必须与源图一致 |', '',
        'L1继续封存；L2搜索、软目标、原Smac、车辆模型、Static/InflationLayer库和严格终验不变。唯一修改的Nav2库是独立costmap核心的读回回调，普通GetCostmap接口仍正常返回完整地图。未改生产入口。', '',
        '## 实验口径', '',
        '重新运行未改动的r5作为对照。主两图各10组×3次；A、AB为各图10组首次独立对照；ABC正式组各30次。额外两图每图各10组一次，失败查询也计入耗时与内存分布。所有导航串行固定CPU12，BLAS/OMP/MKL单线程，构建和测试在其他核进行；不是整机无其他负载的随机交错试验。下列百分比是这批观测变化。', '',
        '端到端不含一次性加载、服务启动及结果导出；包含当次层间转换、传递、ACK、车辆规划和终验。首次是每个固定查询首次调用，重复是之后两次。各层存在包含关系，L3含适配，各行中位数及P95不能相加。', '',
        '## 完整规划结果：r5 → r6 ABC', '',
        '| 地图 | 严格成功 | 首次中位 / s | 重复中位 / s | 全部中位 / s | 全部P95 / s | 最慢 / s |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for v in c['maps'].values():
        a, b = v['arms']['r5'], v['arms']['ABC']
        values = [pair(a['metrics']['e2e_s'][part][s], b['metrics']['e2e_s'][part][s])
                  for part, s in [('first', 'median'), ('repeat', 'median'), ('all', 'median'), ('all', 'p95'), ('all', 'max')]]
        text.append(f"| {v['label']} | {a['success']}/{a['n']} → {b['success']}/{b['n']} | "+' | '.join(values)+' |')
    text += ['', '## 内存：r5 → r6 ABC', '',
        '进程树RSS是每次查询的采样峰值，表中再对这些峰值求中位或最大；不是单独L3内存。PSS与RSS不相加。', '',
        '| 地图 | 进程树峰值RSS中位 / GiB | 峰值RSS最大 / GiB | Python PSS快照中位 / GiB | planner峰值PSS中位 / GiB |',
        '|---|---:|---:|---:|---:|']
    for v in c['maps'].values():
        a, b = [v['arms'][k]['metrics'] for k in ('r5', 'ABC')]
        values = [pair(a[m]['all'][s], b[m]['all'][s]) for m, s in [('tree_rss_gib', 'median'), ('tree_rss_gib', 'max'), ('python_pss_gib', 'median'), ('planner_pss_gib', 'median')]]
        text.append('| '+v['label']+' | '+' | '.join(values)+' |')
    text += ['',
        '减少副本主要降低Python内存。B在服务器和客户端各保留一个完整字节快照，因此planner PSS有所增加；不能说每个进程都省内存。A单独使用时比ABC更省内存，ABC为减少传输接受了这项有限代价。缓存每端至多一份，不随查询数增长。', '',
        '![四图原始分布](r5_r6_boxplots.png)', '',
        '## 各层实际耗时', '',
        '每格为r5 → r6 ABC中位数。L1单位ms，其余单位s；L2和L1未改算法，细小变化属于本批运行波动。', '',
        '| 地图 / 调用 | L1 / ms | L2整体 / s | L2→L3适配 / s | L3流水线 / s | 地图更新与ACK / s |',
        '|---|---:|---:|---:|---:|---:|']
    for v in c['maps'].values():
        a, b = [v['arms'][k]['metrics'] for k in ('r5', 'ABC')]
        for part, label in [('first', '首次'), ('repeat', '重复')]:
            if not b['e2e_s'][part]['n']:
                continue
            values = [fmt(a[m][part]['median'])+' → '+fmt(b[m][part]['median']) for m in ['l1_ms', 'l2_s', 'adapter_s', 'l3_s', 'map_sync_s']]
            text.append('| '+v['label']+' / '+label+' | '+' | '.join(values)+' |')
    text += ['', '## 层间传输到底省了多少', '',
        '下面是整批（含一次性启动）的消息有效载荷，不包含DDS协议开销。读回为L3实际costmap传回Python；发布为Python送入L3，两者不能混淆。由于观察更新中的次数不同，读回次数不强制相等。', '',
        '| 地图 | 读回载荷 / MiB | 发布消息数 | 发布载荷 / MiB | 两方向总载荷 / MiB |',
        '|---|---:|---:|---:|---:|']
    for v in c['maps'].values():
        a, b = [v['arms'][k]['transport'] for k in ('r5', 'ABC')]
        values = [pair(a['readback']['payload_bytes']/2**20, b['readback']['payload_bytes']/2**20),
                  f"{a['publication']['messages']} → {b['publication']['messages']}",
                  pair(a['publication']['payload_bytes']/2**20, b['publication']['payload_bytes']/2**20),
                  pair((a['publication']['payload_bytes']+a['readback']['payload_bytes'])/2**20,
                       (b['publication']['payload_bytes']+b['readback']['payload_bytes'])/2**20)]
        text.append('| '+v['label']+' | '+' | '.join(values)+' |')
    text += ['',
        'C把相近更新合并，消息少了，但会多发送一些未变化格子。主两图查询阶段发布量分别4.57→9.19 MiB、18.10→31.53 MiB，不能称两个方向的流量都下降。扣除不变的启动整图后，查询更新消息为83→35、295→148。合并减少调度和处理次数，最终内容仍逐格确认。', '',
        '读回环节本身也未提速：主两图整批服务调用累计时间0.724→1.863 s、1.456→3.566 s，包含新增比较、完整内容计算及更多观察次数。虽然少传约80%，本机共享内存原本传整图已经很快。收益主要体现为少分配、少消息及整个地图同步流程，而不是读回函数更快。', '',
        '## 分步对照与选型', '',
        '各行只比较每图首次10条，避免把A/AB首次分布与ABC的30次混在一起。C未单独跑，仅评估AB→ABC。', '',
        '| 地图 | 组别 | 首次中位 / s | 首次地图同步中位 / s | 首次进程树RSS中位 / GiB |',
        '|---|---|---:|---:|---:|']
    for v in list(c['maps'].values())[:2]:
        for key in ['r5', 'A', 'AB', 'ABC']:
            m = v['arms'][key]['metrics']
            text.append('| '+v['label']+' | '+key+' | '+' | '.join(fmt(m[k]['first']['median']) for k in ['e2e_s', 'map_sync_s', 'tree_rss_gib'])+' |')
    text += ['', '![分步对照首次分布](ablation_first_boxplots.png)', '',
        '本轮选择ABC：保持既有成功和路径，主两图P95及内存下降，同时建立了有内容确认的增量读回接口。B单独叠加后没有稳定的耗时优势；C在大图降低地图同步时间，但端到端仍被其他L3处理限制。保留A、AB开关供后续对照，不把ABC描述成每项指标都最优。', '',
        '一次性服务启动也有下降（未计入前述查询耗时）：', '',
        '| 地图 | 服务启动 r5 → r6 / s |', '|---|---:|']
    for v in c['maps'].values():
        text.append('| '+v['label']+' | '+pair(v['arms']['r5']['startup_s'], v['arms']['ABC']['startup_s'])+' |')
    text += ['', '## 路径质量与失败', '',
        '| 地图 | 成功路径靠右比例中位 | 成功路径长度中位 / m | 既有成功最终路径完全相同 |', '|---|---:|---:|---:|']
    for v in c['maps'].values():
        b = v['arms']['ABC']; pairs = v['parity']['ABC']; before = sum(x['before'] for x in pairs)
        text.append('| '+v['label']+' | '+fmt(b['right']['median'])+' | '+fmt(b['length']['median'])+f" | {sum(x['path.npy'] is True for x in pairs)}/{before} |")
    text += ['',
        '无语义地图不定义靠右指标，表中“—”不是0%。所有成功的碰撞、转向、交通、内容ACK及0.8/1.2 m搜索范围均通过；未触发全图Smac回退。arena_medium A2B-03/04仍在L3车体碰撞／允许范围阶段失败，本轮没有改变这两条的规划策略。', '',
        '## 检查与边界', '',
        '165项测试通过，包括完整与增量快照重建、远离路线位置的变化、旧区域关闭、丢失回复、错误基版本、几何及内容损坏、只读数组、两次真实快照和原精确ACK分类等。四图正式对照与候选，以及A/AB分步批次的监督器均完成退出；逐个核对其拥有的进程和共享内存已不存在。父版2013个登记文件、L1封存、原生L2、Smac及层库均保持。', '',
        '诊断diag_*目录不进入正式分布。早期诊断中的立即回退实现仅作为开发记录保留；最终允许正常地图更新在0.6秒内收敛，之后仅用原ACK总截止时间的剩余额度进入既有恢复，不增加总等待预算。正式实验显式使用ABC；验收后仅将env.bash启动默认值改为同一ABC模式，规划源码不再改动。', '',
        'Nav2地图仍完整驻留。服务器仍比较完整master，客户端仍重建完整地图并按原范围逐格检查；这是增量传输，不是稀疏Nav2存储，也不是只检查新路线。当前验证仅为四图固定静态任务；故障注入单测不代表动态导航、并发多机器人或实车验收。', '',
        '## 下一步', '',
        '优先把实际脏块从地图更新端传到读回端，在版本和失效规则完整的前提下减少重复扫描；同时统计L3剩余转换、终验和局部修复的真实调用成本。若要明显降低常驻内存，再独立验证Nav2局部存储／裁剪，必须覆盖原点切换、旧范围关闭、地图边缘和跨块膨胀，不能直接在本轮结果上宣称已完成。', '',
        '## 交付', '',
        '- 代码与接口：README.md、INTERFACES.md、src/three_yd_v5/r6_*.py、performance_session.py。',
        '- 独立原生库与构建：native/DELTA_BUILD.json、native/delta_snapshot.hpp、external/nav2_costmap_2d、install/nav2_snapshot_local/lib。',
        '- 原始结果：results/control、results/ablation/A、results/ablation/AB、results/formal、results/additional。',
        '- 统计与检查：report/COMPARISON.json、report/tests_final.txt、每批source_snapshot和supervisor.json。',
        '- 复现协议：EXPERIMENT_PROTOCOL.md；重新统计运行python3 tools/compare_v8_r6.py和python3 tools/write_v8_r6_report.py。',
    ]
    (ROOT/'report/RESULTS.md').write_text('\n'.join(text)+'\n')

if __name__ == '__main__':
    main()
