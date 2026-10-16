from pathlib import Path
import json,csv,collections
import numpy as np
E=Path(__file__).resolve().parents[1];R=E/'report';d=json.loads((R/'COMPARISON.json').read_text());s=json.loads((R/'SUMMARY.json').read_text());raw=list(csv.DictReader((R/'comparison_raw.csv').open()))
fixes=sorted(set((v['map_id'],v['query_id']) for v in d['fixed_requests']));name={m['map_id']:m['name'] for m in d['maps']};common=[v for v in raw if v['old_success']=='True' and v['success']=='True']
lines=['# 3YDL3-V9：十图固定 Q 修复回归','',f"本轮严格成功 **{d['strict']}/300（{d['strict']/3:.1f}%）**，V8 为 **{d['old_strict']}/300（{d['old_strict']/3:.1f}%）**。新增 {len(fixes)} 组固定查询通过，三轮共新增 {len(d['fixed_requests'])} 次严格成功；原成功查询退化 {len(d['regressions'])} 次。",'',
'独立候选目录：`'+str(E)+'`。保留 V8 及全部冻结基线，未改写历史生产入口。当前结果仅为这十张地图的固定静态完整规划回归，不代表动态障碍、连续控制或实车验收。','',
'## 本轮采用的修复','',
'1. **近端接入初始化**：旧实现只尝试距端点 1–5 m 的锚点；这些候选全失败时，现在追加 0.25、0.5、0.75 m 的候选。狭窄通道的短距离回转可以进入 B 样条优化。原有可行候选仍优先。解析曲线只用于初始化，最终交付仍是通过连续曲率、车体扫掠和完整三层审计的 B 样条。',
'2. **交通拒绝后的有限重试**：几何证书通过但道路顺序/方向检查失败时，在同一目标范围、同一 15 s 新 L3 预算及 30 s 请求预算内，将参考线跟随权重提高到原来的 2 倍，最多重试一次。原始交通审计不变；失败时仍撤回轨迹。','',
'车辆模型、地图障碍、原 Q 起终点、连续证书、交通审计和 L3 的 0.8/1.2 m 范围及 400000 格/10% 空间上限均未放宽。没有调用旧 L3、全图 Smac 或移动窗口。','',
'## 新增通过的固定查询','', '| 地图 | 查询 | 本轮结果 |','|---|---|---|']
for mid,q in fixes:lines.append(f'| {name[mid]} | {q} | 3/3 三层严格成功 |')
lines+=['','## 十图结果','',f"L1 输出文件留存 {sum(m['l1_outputs_saved'] for m in s['maps'])}/300，L2 参考线文件留存 {sum(m['l2_references_saved'] for m in s['maps'])}/300；文件存在不等于三层严格成功，严格结果以完整终验为准。",'',
'各图都使用其配套 Q，10 组 × 3 个独立轮次；第二轮倒序地图。每轮每图新进程、每个 Q 仅执行一次，未使用即时重复复用。CPU 14 串行运行。下表耗时和内存统计包含成功与失败请求，各版本每图 n=30。', '',
'| 地图 | V8 严格通过 | V9 严格通过 | V8→V9 耗时中位（s） | V8→V9 进程树 RSS 中位（MiB） | 仍失败的 Q |','|---|---:|---:|---:|---:|---|']
for m in d['maps']:
 t=m['wall_s']['all_attempts'];mem=m['tree_rss_mib']['all_attempts'];lines.append(f"| {m['name']} | {m['old_success']}/30 | {m['success']}/30 | {t['old_median']:.3f} → {t['median']:.3f} | {mem['old_median']:.0f} → {mem['median']:.0f} | {', '.join(m['remaining']) or '—'} |")
lines+=['','![请求耗时逐次分布](time_boxplots.png)','','![进程树内存逐次分布](memory_boxplots.png)','',
'箱线图来自 `comparison_raw.csv` 中逐次原始值。图内明确标注样本数、单位、四分位数、1.5×IQR 须与离群点，耗时使用对数横轴；实际加载宋体 SimSun。离散严格成功率仅列表，不画箱线图。','',
'## 性能与输出回归','',f"原 V8 成功样本中，{d['original_successes_all_four_outputs_identical']}/{d['original_successes']} 的 L1 输出、L2 参考线、软目标和最终路径四项逐文件哈希全部一致。原成功查询退化 {len(d['regressions'])} 次；本轮成功查询跨三轮路径一致：{d['stable_successful_paths_all_rounds']}。",'']
for key,title,unit in [('wall_s','共同成功样本请求耗时','s'),('tree_cpu_s','共同成功样本进程树 CPU 时间','s'),('tree_rss_mib','共同成功样本进程树 RSS 采样峰值','MiB')]:
 a=np.median([float(v['old_'+key]) for v in common]);b=np.median([float(v[key]) for v in common]);lines.append(f'- {title}：n={len(common)}，中位 {a:.3f} → {b:.3f} {unit}（{(b/a-1)*100:+.2f}%）。')
lines+=['','这是与 V8 已存档三轮数据的描述性比较，并非本轮交错运行的同期性能试验。本轮重点是失败修复；**不据此宣称又完成“耗时和资源各降低 5%”的性能门槛**。新成功案例和原失败案例不是同等工作量，因此另列共同成功样本。',
'请求耗时排除地图准备、审计器准备、节点启动和导出，启动数据在 `SUMMARY.json` 的 `separate_startup` 字段单列。RSS 是整套 Python/ROS 规划进程树的采样峰值，不是单进程常驻量或连续无遗漏的硬件峰值；PSS 返回后快照和 worker RSS 在 `raw_metrics.csv` 中另列。','',
'## 尚未解决及未采用方案','',
'其余稳定失败未隐藏、未替换 Q、未降低验收门槛。L2 局部扩展 4 m 虽能让导师原图 A2B-03 生成参考线，但 L3 仍不通过且耗时增加；恢复正式默认 L2。扩大端点初始化范围、较大半径和局部位姿搜索也未新增严格成功，保留为诊断材料，不接入默认方案。',
'后续应先对剩余端点回转失败构造可验证的局部接入参考线，再处理语义转弯的方向约束；L2 桥接与其后续 L3 必须一起通过才能采用。预算耗尽或局部搜索未找到解，不等于已证明物理不可行。','',
'## 校验与交付','',
'- 699 项继承工程检查通过，2 项新增端点回归检查通过，合计 701 项；保留 1 个继承的 NumPy 比较警告。早期新增检查曾错误地从 XY 差分重建解析曲线朝向，已改为核验解析曲线的实际朝向，失败日志和修正后日志均保留。',
 f"- {d['parent_files_unchanged']} 个父版登记文件哈希未变；L1_LOCK 的源码、原生库、输入及像素输入全部核验。控制器/交通审计文件与父版逐字相同，连续证书、原优化器和复用校验函数 AST 相同。",
 f"- 正式 30 批完整运行，全部固定 Q 位姿/输入指纹一致；{d['strict']} 个成功交付均验证严格审计及实际 costmap 内容 ACK；失败请求无可执行 path.npy。",
 f"- {len(d['all_cleanup'])} 批拥有进程与共享资源清理完成，含诊断及参数错误的调用；正式统计仅 `results/formal/`。",
 '- `REMAINING_FAILURE_DIAGNOSTICS.json`：首轮六组未解决案例的逐候选失败原因；`COMPARISON.json`：回归、输出哈希、锁定文件和清理审计；`SUMMARY.json`：十图统计与准备/启动开销；`comparison_raw.csv`：新旧逐次对照；`raw_metrics.csv`：本轮完整指标；`FIXED_QUERY_COORDINATES.csv`：原始地图配套起终点。','',
'复现方式见上级 `README.md`。所有正式结果与诊断结果分开保存，不混用。','']
(R/'RESULTS.md').write_text('\n'.join(lines))
print('Report written',R/'RESULTS.md')
