import argparse,json,hashlib
from pathlib import Path
from analyze_v8 import measure
ROOT=Path(__file__).resolve().parents[1]
def main():
 parser=argparse.ArgumentParser()
 parser.add_argument('--results-root',type=Path,default=ROOT/'results')
 parser.add_argument('--report-dir',type=Path,default=ROOT/'report')
 args=parser.parse_args();report=args.report_dir.resolve();results=args.results_root.resolve()
 d=json.loads((report/'COMPARISON.json').read_text())
 lines=['# 3YD-V8-r11：首次完整规划数据处理优化','',
 '从新V8-r10完整路线续接版独立建立；L1封存、L2/软引导/Smac/Jackal、原严格终验和有界搜索均保持。不是旧移动窗口实验。',
 '', '## 本轮采用的方法','',
 '- A：无语义模式只保留其真实需要的占据与范围处理，跳过停车采样、停车标签判定和不存在的停车接入膨胀。保留原适配器的区域标签生成时机与静态归属；障碍增删后可通行掩膜仍按当前硬障碍更新。',
 '- B：ACK准备直接读取稀疏软场的soft>0，并使用明确的字节运算编码原分类。没有减少实际地图读回、逐格比较、发布时间与版本确认，也没有复用旧ACK证据。',
 '- 第一轮语义区域查表和未限定字节类型的编码没有证明收益，未采用。原始探索结果完整保留，排除正式性能结论。',
 '', '## 正式固定查询：同批r10对照与r11','',
 '| 地图 | 严格通过（对照→r11） | 首次中位（s） | 首次变化 | 首次P95（s） | 重复中位（s） | 全部P95（s） |',
 '|---|---:|---:|---:|---:|---:|---:|']
 for mid,x in d['fixed'].items():
  old,new=x['arms']['control'],x['arms']['candidate']
  def v(a,s,m,k='median'):return a[s]['metrics'][m][k]
  a,b=v(old,'first','e2e_s'),v(new,'first','e2e_s')
  lines.append(f"| {x['label']} | {old['all']['success']}/{old['all']['n']} → {new['all']['success']}/{new['all']['n']} | {a:.3f} → {b:.3f} | {100*(b/a-1):+.1f}% | {v(old,'first','e2e_s','p95'):.3f} → {v(new,'first','e2e_s','p95'):.3f} | {v(old,'repeat','e2e_s'):.3f} → {v(new,'repeat','e2e_s'):.3f} | {v(old,'all','e2e_s','p95'):.3f} → {v(new,'all','e2e_s','p95'):.3f} |")
 lines+=['','正式60/60，各层输出、L2代价与展开、最终路径、L3范围及有效costmap均与同批对照逐条一致。不是60条独立路线，是两图各10组重复3次。',
 '', '**主要收益在无语义大图；语义地图首次中位没有改善，本批增加约3.2%。ACK分类更快不等于端到端必然更快。**',
 '', '## 分层首次耗时与资源','',
 '| 地图 | 指标 | r10对照 | r11 | 变化 |','|---|---|---:|---:|---:|']
 for mid,x in d['fixed'].items():
  for key,label,unit,split in [('l1_ms','L1中位','ms','first'),('l2_s','L2整体中位','s','first'),('dstar_s','D*搜索中位','s','first'),('adapter_s','层间适配中位','s','first'),('map_sync_s','地图更新与ACK中位','s','first'),('l3_s','L3流水线中位','s','first'),('l2_state_mib','L2状态数组中位','MiB','first'),('tree_rss_gib','查询进程树峰值RSS中位','GiB','all'),('tree_rss_gib','查询进程树峰值RSS最大','GiB','all'),('python_pss_gib','Python工作进程PSS中位','GiB','all'),('planner_pss_gib','planner PSS中位（有观测的首次）','GiB','first')]:
   stat='max' if label.endswith('最大') else 'median'
   a=x['arms']['control'][split]['metrics'][key][stat];b=x['arms']['candidate'][split]['metrics'][key][stat]
   if a is None or b is None:continue
   lines.append(f"| {x['label']} | {label} | {a:.3f} {unit} | {b:.3f} {unit} | {100*(b/a-1):+.1f}% |")
 lines+=['','计时排除一次性准备、ROS启动和导出；L3包含适配、ACK及终验，不能把各项中位数相加。RSS是worker与ROS后代逐查询采样峰值；PSS是另外的快照口径。L2状态数组不是完整L2内存，L1没有独立完整内存测量。',
 'L1、D*搜索内核和L3搜索/恢复逻辑未改；这些阶段的耗时涨跌只是本批观测，不能归因于算法优化。百分比很大但绝对差仅数毫秒的指标，应按绝对时间理解。',
 '', '## 路径质量与剩余慢案例','',
 '| 地图 | 最终路径长度中位（m，对照→r11） | 靠右比例中位（对照→r11） |','|---|---:|---:|']
 for mid,x in d['fixed'].items():
  a,b=[x['arms'][arm]['all'] for arm in ('control','candidate')]
  right=[v['right_fraction']['median'] for v in (a,b)]
  right_text='不适用，无语义交通规则' if any(v is None for v in right) else f'{right[0]:.2%} → {right[1]:.2%}'
  lines.append(f"| {x['label']} | {a['length_m']['median']:.3f} → {b['length_m']['median']:.3f} | {right_text} |")
 lines+=['','| 地图 | 最慢首次查询 | 端到端（s） | L2（s） | L3流水线（s） | 搜索调用（s） | 曲率/几何恢复（s） |','|---|---|---:|---:|---:|---:|---:|']
 for mid,x in d['fixed'].items():
  rows=[json.loads(s) for s in (results/'accepted_fixed/candidate'/mid/'runs.jsonl').read_text().splitlines()]
  row=max((r for r in rows if r['repeat']==1),key=lambda r:r['wall_ms'])
  values=[measure(row,k) for k in ('e2e_s','l2_s','l3_s','smac_action_s','curvature_repair_s')]
  lines.append('| '+x['label']+' | '+row['query_id']+' | '+' | '.join(f'{v:.3f}' for v in values)+' |')
 lines+=['','慢案例的主要开销仍在L3；本轮没有优化搜索、连接和恢复算法，不能把数据准备的改进写成L3算法提速。','',
 '## 单项对照','',
 '| 地图 | 指标（首次中位） | control | A | B | AB |','|---|---|---:|---:|---:|---:|']
 for mid,x in d['fixed'].items():
  for m,label in [('e2e_s','端到端 / s'),('l2_s','L2整体 / s'),('map_sync_s','地图同步ACK / s')]:
   vals=[x['arms']['control']['first']['metrics'][m]['median'],d['ablation'][mid]['A']['first']['metrics'][m]['median'],d['ablation'][mid]['B']['first']['metrics'][m]['median'],x['arms']['candidate']['first']['metrics'][m]['median']]
   lines.append('| '+x['label']+' | '+label+' | '+' | '.join(f'{v:.3f}' for v in vals)+' |')
 lines+=['','A/B各图10次首次，control/AB固定10×3的首次各10次；不混合不同样本数或重复查询。收益以实际观测为准，不能把分项百分比相加。','',
 '![独立消融](ablation_boxplots.png)','', '![固定查询](fixed_boxplots.png)']
 for w,title in [('moving','沿路线变化起点'),('perturbed','起点偏差3cm/2°')]:
  if w not in d:continue
  lines+=['',f'## {title}回归','', '| 地图 | 变化请求严格通过（对照→r11） | 中位耗时（s） | P95（s） |','|---|---:|---:|---:|']
  for mid,x in d[w].items():
   a,b=x['arms']['control']['moving'],x['arms']['candidate']['moving']
   lines.append(f"| {x['label']} | {a['success']}/{a['n']} → {b['success']}/{b['n']} | {a['metrics']['e2e_s']['median']:.3f} → {b['metrics']['e2e_s']['median']:.3f} | {a['metrics']['e2e_s']['p95']:.3f} → {b['metrics']['e2e_s']['p95']:.3f} |")
  lines+=['','每图另有10次初始规划，两图20次均通过。相同20条任务的不同起点，不是新的独立路线。实际位姿、各层输出、完整剩余路径与r10对照一致。','',f'![{title}]({w}_boxplots.png)']
 lines+=['','## 验证与能力边界','',
 f"- {d['verification']['tests']}；拥有进程与共享资源清理通过，共{len(d['verification']['cleanup'])}批。",
 '- 两模式原障碍事件各11项通过；前方小障碍无语义案例可绕，Q09停车区前方障碍仍未求得路线，会撤回轨迹并在清除后恢复。没有宣称动态改道已全部成功。',
 '- 本轮只完成首次数据准备优化，未修改L3连接/恢复算法。Nav2仍完整存图；未完成连续定位/跟踪、受阻期间持续变化位姿、到达处理或实车验收。',
 '- 封存L1、r10父版、r6和历史生产入口保持；本版为独立测试候选，尚未替换生产基线。',
 '- CPU12逐批串行、OMP/BLAS单线程、无profiler；批次顺序固定，不作随机化显著性结论。',
 '', '## 复现','', '以下输出目录必须尚不存在；每次复测更换目录，避免覆盖正式记录。','', '```bash','source env.bash','python3 tools/run_firstcost_batches.py --output-root results/reproduce_new','python3 tools/run_firstcost_regressions.py --output-root results/reproduce_new','python3 -m pytest -q tests','python3 tools/compare_firstcost.py --results-root results/reproduce_new --report-dir report/reproduce_new','python3 tools/write_firstcost_report.py --results-root results/reproduce_new --report-dir report/reproduce_new','```','',
 '正式与排除目录登记在SELECTED_DATA.json；COMPARISON.json保留逐组统计及原始文件校验。使用新输出目录，不覆盖本轮正式记录。',
 '', '## 下一步','', '保持完整任务续接收益，优先诊断L3首次曲率/连接恢复的慢案例；随后单独补Q09前方障碍的有限L2参考修复与任务上下文恢复。成功率、性能和动态/实车验收分别报告。']
 (report/'RESULTS.md').write_text('\n'.join(lines)+'\n')
 print(report/'RESULTS.md')
if __name__=='__main__':main()
