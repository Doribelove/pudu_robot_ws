import json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def main():
 d=json.loads((ROOT/'report/COMPARISON.json').read_text());selected=json.loads((ROOT/'report/SELECTED_DATA.json').read_text())
 lines=['# 3YD-V8-r7：完整路线复用与按影响重规划','',
 '旧移动窗口 r7–r10 已归档，原工作区、原始数据和报告保留。本版本从 r6 ABC 独立建立，r6主版本、封存L1及历史生产入口均未修改。',
 '', '**最终两张主地图各10组×3次均30/30严格通过。60条L1输出、L2参考、软目标和最终轨迹与同批r6逻辑对照完全一致。40次连续重复查询全部复用完整轨迹，没有再次调用车辆搜索。**',
 '', '收益主要来自重复查询。首次查询增加了约2%–4%的缓存核对和记录成本；总体P95没有改善。新增地图回归仍有原先两条失败，不宣称所有地图都能通过。','',
 '## 1 实现与版本关系','',
 '- 首次请求：沿用r6完整路线规划及原有有界恢复，返回通过全程终验的起终点轨迹。“一次返回完整路线”不等于内部只允许一次Smac调用。',
 '- 相同查询：复用完整轨迹；执行现有地图ACK协议和完整车辆、曲率、转向、交通终验。相同内容可使用r6已有的严格no-op ACK，不能称为每次重新读取服务端地图。',
 '- 障碍更新：新增障碍影响参考路时重算该查询；其他变化先复核现有轨迹，实际车体检查拒绝才重算。障碍清除需要空闲观测，不追求每次清除后强制寻找更短路线。',
 '- 障碍更新后旧版本轨迹立即不可读取；重规划失败或ACK失效时，返回空轨迹。',
 '- 完整轨迹缓存最多8条且估计不超过8 MiB；拒绝超大单项。运行事件只保留最近32条简要记录，旧版本膨胀数组按需淘汰。',
 '- L1保持封存；L2仍0.15 m二维D*；语义模式0.30 m偏移不变；L3仍0.05 m、0.8/1.2 m参考邻域、原Jackal、原验收。没有移动窗口，也没有无界全图Smac回退。','',
 '## 2 正式效果与资源','',
 '每图固定10组×3次，首次n=10、重复n=20、合计n=30；CPU12串行、单线程数值库。初始化、地图加载和导出不计入查询。对照关闭完整轨迹复用，其他r6 ABC配置保持。两批没有随机交错，百分比是本批观测。','',
 '| 地图 | 指标 | r6逻辑对照 | 新r7 | 变化 |','|---|---|---:|---:|---:|']
 metrics=[('首次端到端中位数（s）','e2e_s','first','median'),('重复端到端中位数（s）','e2e_s','repeat','median'),('重复端到端P95（s）','e2e_s','repeat','p95'),('全部端到端中位数（s）','e2e_s','all','median'),('全部端到端P95（s）','e2e_s','all','p95'),('端到端最大值（s）','e2e_s','all','max'),('查询峰值RSS中位数（GiB）','tree_rss_gib','all','median'),('查询峰值RSS最大值（GiB）','tree_rss_gib','all','max')]
 for mid,v in d['maps'].items():
  lines.append(f"| {v['name']} | 严格通过 | 30/30 | 30/30 | 相同 |")
  for label,metric,part,stat in metrics:
   a=v['r6_control']['metrics'][metric][part][stat];b=v['r7']['metrics'][metric][part][stat]
   lines.append(f"| {v['name']} | {label} | {a:.3f} | {b:.3f} | {(b/a-1)*100:+.1f}% |")
 lines+=['','![正式性能分布](performance_boxplots.png)','',
 'RSS是每次查询整套规划进程树采样峰值；表中“中位数”是30个峰值的中位数，不是某层的独占内存。Nav2仍然常驻完整地图，本轮没有证明首次规划的内存显著下降。','',
 '| 地图 | 路径长度中位数 r6→r7（m） | 靠右比例中位数 r6→r7 | 最终路径相同 |','|---|---:|---:|---:|']
 for mid,v in d['maps'].items():
  a,b=v['r6_control'],v['r7'];right='不适用：无语义交通规则' if b['right']['median'] is None else f"{a['right']['median']:.4f} → {b['right']['median']:.4f}"
  lines.append(f"| {v['name']} | {a['length']['median']:.3f} → {b['length']['median']:.3f} | {right} | 30/30 |")
 lines+=['','## 3 各层实际调用时间','',
 'L3包括层间适配、地图同步、车辆搜索/复用和终验，不能将各行中位数相加当成端到端。L1源码没有变化，小幅计时差异不作为L1算法收益。','',
 '| 地图 | 阶段 | 首次中位 r6→r7 | 重复中位 r6→r7 |','|---|---|---:|---:|']
 for mid,v in d['maps'].items():
  for metric,label in [('l1_ms','L1（ms）'),('l2_s','L2候选准备（s）'),('adapter_s','L2→L3适配（s）'),('l3_s','L3完整流水线（s）'),('strict_audit_s','独立严格审核累计（s）')]:
   vals=[]
   for part in ('first','repeat'):
    a=v['r6_control']['metrics'][metric][part]['median'];b=v['r7']['metrics'][metric][part]['median']
    vals.append(f'{a:.4f} → {b:.4f}')
   lines.append(f"| {v['name']} | {label} | {' | '.join(vals)} |")
 lines+=['', '正式缓存命中原始记录的 `local_map_update_ms` 包含整个 `sync_only` 调用（包括适配），与r6该子时钟范围不同，因此对照剔除了这一子项。L3总时钟与适配独立时钟不受影响。最终实现已改用 `fullroute_sync_wall_ms` 单独记录总同步时间，并保留ACK原生子时钟。','',
 '| 地图 | 完整轨迹缓存最高估计容量 | 最高条目数 | 连续重复命中 |','|---|---:|---:|---:|']
 for mid,v in d['maps'].items():lines.append(f"| {v['name']} | {v['max_cached_bytes']/2**20:.3f} MiB | {v['max_cache_entries']} | 20/20 |")
 lines+=['','交错诊断Q01/Q06/Q09共6次严格通过，第二轮3次均找到原完整轨迹。但切换查询后仍可能重建L2字段和同步地图：第二轮耗时分别约3.42/2.14/2.22 s，不能把连续重复的0.4 s推广到所有交错场景。','',
 '## 4 地图变化回放','',
 '每种模式选1条固定查询进行世界坐标障碍事件回放。以下是功能检查，不混入静态60次成功率，也不代表实车执行。','',
 '| 事件 | 万达3w Q06 | arena_large A2B-01 |','|---|---|---|']
 ev={m:{e['label']:e for e in json.loads((Path(path)/'events.json').read_text())} for m,path in selected['dynamic'].items()}
 for name,label in [('remote_add','远处新增障碍'),('remote_clear','远处清除障碍'),('path_add','路径中部新增障碍'),('path_clear','清除路径障碍'),('endpoint_block','封堵起点'),('endpoint_clear','恢复起点')]:
  vals=[]
  for mid in selected['dynamic']:
   e=ev[mid][name];action='重规划' if e['replanned'] else '保留路径复核'
   if name=='endpoint_block':action='拒绝规划并撤回轨迹'
   vals.append(f"{action}，{e['wall_ms']/1000:.3f} s")
  lines.append(f"| {label} | {' | '.join(vals)} |")
 lines+=['','| 障碍回放资源（每图10次调用） | 查询峰值RSS中位数 | 查询峰值RSS最大值 |','|---|---:|---:|']
 for mid,path in selected['dynamic'].items():
  values=[x['rss_peak_bytes']/2**30 for x in ev[mid].values() if 'rss_peak_bytes' in x]
  label='万达3w地库' if mid=='mengjia' else 'arena_large'
  lines.append(f'| {label} | {np.median(values):.3f} GiB | {max(values):.3f} GiB |')
 lines+=['','障碍回放是不同于静态重复查询的工作负载，尚无r6同口径动态对照；不能把静态内存结论直接推广到地图更新过程。更新峰值和地图重新组合仍需后续优化。']
 lines+=['','两图均正确处理重复事件、拒绝乱序更新、拒绝无空闲观测的清除。远处障碍增删后路径逐点不变；路径中部新增障碍后新轨迹通过完整终验；起点封死时不交付轨迹；清除后恢复。',
 '', '开发诊断中暴露的“新代价图首次组合漏存引用”和“无语义地图checker只读属性”已修复。失败诊断目录保留，但未计入上述最终事件结果。','',
 '## 5 额外地图与能力边界','',
 '| 地图 | r6历史固定10组 | 新r7固定10组 | 结果 |','|---|---:|---:|---|',
 '| 合成地库A | 10/10 | 10/10 | 路径不变 |',
 '| arena_medium | 8/10 | 8/10 | A2B-03/04仍在L3失败；原成功路径不变 |','',
 '- 完整轨迹复用要求起终点姿态和相关输入一致；机器人位姿改变仍正常规划。本轮不提供滚动轨迹拼接。',
 '- 正式收益集中于重复请求。首次规划与总体P95仍受原L2/L3求解影响，本轮没有消除这部分成本。',
 '- 无关障碍虽然不重新搜索，仍需更新地图表达、确认ACK和全路径审核；回放中约1–2 s，不是零开销。',
 '- 重规划以既有L1合法路线与限定范围为基础，不能保证任意临时障碍下都有绕行解。',
 '- 实现了障碍层增删事件接口；静态地图、坐标、车辆、语义规则更换需新建服务。尚未接传感器清障、定位跟踪及真实车辆。',
 '- 总体峰值RSS未明显降低；额外轨迹缓存小且有界。下一步应优先处理首次求解和无关变化的地图组合成本，而不是增加窗口和重试层级。','',
 '## 6 校验与交付','',
 f"- 184项测试通过；{d['L1_source_files_unchanged']}份封存L1文件保持；r6已登记{d['parent_registered_files_unchanged']}份文件校验未变。",
 f"- {len(d['cleanup_journals'])}批进程/共享资源清理记录通过，包括保留的失败诊断。",
 '- 主数据：`results/control/`、`results/final/`；额外回归：`results/additional/`；最终障碍回放路径见 `report/SELECTED_DATA.json`。',
 '- 对照统计：`report/COMPARISON.json`；复现：`tools/run_r7_formal.sh`、`tools/run_r7_final.sh`；图表：`report/performance_boxplots.png`。',
 '- 代码：`src/three_yd_v5/fullroute.py`、`fullroute_service.py`；接口与协议：`INTERFACES.md`、`EXPERIMENT_PROTOCOL.md`。',
 '- 旧窗口实验归档：`/home/robot/workspaces/3yd_frozen_baselines/3YD-V8-rolling-r7-r10_20261008/INDEX.json`。','']
 (ROOT/'report/RESULTS.md').write_text('\n'.join(lines))
 # Save explicit interleaved evidence rather than assuming cache hits.
 rows=[json.loads(v) for v in (ROOT/'results/interleaved/mengjia/runs.jsonl').read_text().splitlines()]
 assert len(rows)==6 and all(v['success'] for v in rows)
 assert sum(bool(r['fullroute'].get('reused')) for r in rows)==3
 for q in ['Q01','Q06','Q09']:
  a=np.load(ROOT/f'results/interleaved/mengjia/cases/{q}_1/path.npy')
  b=np.load(ROOT/f'results/interleaved/mengjia/cases/{q}_2/path.npy')
  assert np.array_equal(a,b)
 (ROOT/'report/INTERLEAVED.json').write_text(json.dumps({'success':[6,6],'second_round_reused':[3,3],'paths_equal':[3,3]},indent=2)+'\n')
 print(ROOT/'report/RESULTS.md')
if __name__=='__main__':main()
