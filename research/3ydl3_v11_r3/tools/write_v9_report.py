"""Write the verified V9 conclusions from COMPARISON.json only."""
import argparse,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def fmt(v,n=3):return '—' if v is None else f'{v:.{n}f}'
def change(a,b):return '—' if not a or b is None else f'{100*(b/a-1):+.1f}%'
def main():
 p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,default=ROOT/'report');a=p.parse_args();d=json.loads((a.report_dir/'COMPARISON.json').read_text())
 lines=['# 3YD-V9-r0：L3地图复用与局部轨迹实验','',
 '从封存V8-r11 AB独立建立。最终默认保持control。A地图处理复用降低重复耗时，但未达到降内存目标，保留为延迟优先实验；B跨任务道路段缓存、C直段验证与局部车辆连接已实现并保留独立实验开关，未整体采用。完整路线一次返回，L1封存、二维0.15 m D*、语义0.30 m软引导、原0.05 m地图/Smac/Jackal、0.8/1.2 m范围及原完整严格终验保持。','',
 'A第一版把旧数组与不可变副本同时留在细图视图和路线缓存，导致RSS上升，已否决。最终版在参考路线和缓存绑定前准备不可变数据，并重新绑定细图视图，避免重复持有。范围缓存只保留弱引用，生命周期清理时释放，不跨查询额外持有整图掩膜。只有底层不可写bytes的数组可复用哈希；可变对象仍实时检查。每次请求仍实际服务端读回、完整内容一致和发布时间/发布序列新鲜性确认；没有缓存ACK替代读回。','',
 'B缓存只有位姿，无审计或ACK证书，容量32条／4 MiB估计驻留字节，LRU、内容失效和close释放；每次匹配重新绑定地图、车辆、规则与当前搜索范围，重新验证连接及完整轨迹。C直接验证参考直段，只在风险/端点做真实几何拟合或原Smac局部连接，批次共用当前地图及实际ACK；失败只能回到原有界完整路线求解。困难场景多次局部拟合及容差端点重拟合后仍回退，引入明显成本，不能作为默认优化。','',
 '## 最终源码固定集同批对照','',
 '| 地图 | 三层严格通过（control→A） | 首次中位/P95（s，control→A） | 首次中位变化 | 重复中位/P95（s，control→A） | 全部P95（s） |',
 '|---|---:|---|---:|---|---|']
 for mid,x in d['fixed'].items():
  c,b=x['arms']['control'],x['arms']['A'];f0,f1=[z['first']['metrics']['e2e_s'] for z in (c,b)];r0,r1=[z['repeat']['metrics']['e2e_s'] for z in (c,b)]
  lines.append(f"| {x['label']} | {c['all']['success']}/{c['all']['n']} → {b['all']['success']}/{b['all']['n']} | {fmt(f0['median'])}/{fmt(f0['p95'])} → {fmt(f1['median'])}/{fmt(f1['p95'])} | {change(f0['median'],f1['median'])} | {fmt(r0['median'])}/{fmt(r0['p95'])} → {fmt(r1['median'])}/{fmt(r1['p95'])} | {fmt(c['all']['metrics']['e2e_s']['p95'])} → {fmt(b['all']['metrics']['e2e_s']['p95'])} |")
 lines+=['','两图各固定10组重复3次，首次每图n=10、重复n=20、整体n=30；不是60条独立路线。A所有60次L1输出、L2参考、软目标、有效地图、L3范围与最终轨迹和同批control一致。L1有效输出与三层严格成功分开核验；不会把地图端点/连通性验证当作完整规划通过。','',
 '![固定集分布](fixed_boxplots.png)','', '![重复查询分布](repeat_boxplots.png)','',
 '## L3分项和内存','',
 '| 地图 | 指标 | control | A | 变化 |','|---|---|---:|---:|---:|']
 for mid,x in d['fixed'].items():
  c,b=x['arms']['control'],x['arms']['A']
  metrics=[('首次L3中位 / s','l3_s','first','median'),('首次L3 P95 / s','l3_s','first','p95'),('首次适配中位 / s','adapter_s','first','median'),('首次实际地图同步＋ACK中位 / s','map_sync_s','first','median'),('首次Smac调用中位 / s','smac_action_s','first','median'),('首次原恢复中位 / s','curvature_repair_s','first','median'),('首次严格审计中位 / s','strict_audit_s','first','median'),('RSS采样峰值中位 / GiB（n=30）','tree_rss_gib','all','median'),('RSS采样峰值最大 / GiB（n=30）','tree_rss_gib','all','max'),('Python PSS快照中位 / GiB','python_pss_gib','all','median'),('首次planner PSS中位 / GiB','planner_pss_gib','first','median'),('首次stack PSS中位 / GiB','stack_pss_gib','first','median')]
  for label,key,part,stat in metrics:
   u,v=c[part]['metrics'][key][stat],b[part]['metrics'][key][stat]
   lines.append(f"| {x['label']} | {label} | {fmt(u)} | {fmt(v)} | {change(u,v)} |")
 lines+=['','L3流水线包含层间适配、实际地图同步与ACK、搜索/连接/恢复和终验；各分项中位数不能相加为端到端。查询RSS是worker及ROS后代的采样峰值；PSS是另一次快照。stack PSS只包括planner/map_server/lifecycle_manager，不能当全部进程树；Python PSS也不能全部归给L2。L1、D*和原Smac内核未改，毫秒级涨跌不解释为算法提速。','',
 'A的RSS中位在两图分别增加约1.4%／2.7%，RSS最大值也未改善。重复请求中位下降约22%／25%，不能写成耗时与内存同时下降；本轮内存目标未达成，因此未采用A为默认配置，也未替换生产入口。','',
 'A仍保留Python与Nav2全图，没有分块Nav2存储，也没有修改原生启发式。原生A*使用稀疏状态容器，不把全图基础代价数组与全(x,y,theta)密集状态混为一谈。','',
 '## A/B/C独立对照','',
 '| 地图 | 模式 | 严格通过 | 首次端到端中位/P95（s） | 首次L3中位（s） | 局部完整通过/尝试 | 有界回退 |',
 '|---|---|---:|---|---:|---:|---:|']
 for mid,x in d['fixed'].items():
  for m in ('control','A','B','C'):
   z=x['arms'][m]['first'];e=z['metrics']['e2e_s'];lines.append(f"| {x['label']} | {m} | {z['success']}/{z['n']} | {fmt(e['median'])}/{fmt(e['p95'])} | {fmt(z['metrics']['l3_s']['median'])} | {z['local_success']}/{z['local_attempts']} | {z['bounded_fallbacks']} |")
 lines+=['','每臂首次n=10。control/A另有20次重复，B/C没有混入重复请求。B在自然不同任务间可提出道路段复用，但新接缝及其他参考缺陷仍可能需要局部连接，失败回退成本全部计入。B/C最终通过率不能解释为局部策略本身全部通过；成功与回退分别列出。','',
 '![单项消融](ablation_boxplots.png)','',
 '## 路径质量','',
 '| 地图 | 模式 | 最终长度中位（m） | 靠右比例中位 | 相对control最大绝对长度变化 |','|---|---|---:|---:|---:|']
 for mid,x in d['fixed'].items():
  for m in ('control','A','B','C'):
   z=x['arms'][m]['first'];right=z['right_fraction']['median'];records=x['parity'].get(m,[]);changes=[abs(v['length_change_pct']) for v in records if v['length_change_pct'] is not None]
   lines.append(f"| {x['label']} | {m} | {fmt(z['length_m']['median'])} | {'不适用' if right is None else fmt(100*right,2)+'%'} | {fmt(max(changes),3)+'%' if changes else '—'} |")
 lines+=['','B/C轨迹变化按真实几何和原审计评估，不强求逐点一致。全部成功结果均通过连续车体、曲率、转向接缝、方向、道路序列/停车规则和地图ACK。每个局部拟合的锚点位置/航向残差和最终转向/曲率证据保存在case outcome及COMPARISON.json；拒绝候选不作为可执行轨迹。','',
 '## 不同任务共享道路段诊断','',
 '| 地图 | 模式 | 新任务严格通过 | 段复用完整通过 | 整路径/续接命中 | 新任务实际中位/P95（s） |','|---|---|---:|---:|---:|---|']
 for mid,x in d['shared'].items():
  for m in ('control','B','C','ABC'):
   z=x['arms'][m]['new_tasks'];e=z['metrics']['e2e_s'];lines.append(f"| {x['label']} | {m} | {z['success']}/{z['n']} | {z['segment_reuse_success']}/{z['n']} | {z['whole_route_hits']} | {fmt(e['median'])}/{fmt(e['p95'])} |")
 lines+=['','每图先运行一条原始任务，再运行3条明确不同任务ID、起点和目标的固定诊断任务。原始严格轨迹取自封存父版，地图/查询定义与父轨迹哈希在datasets/v9_shared_segments_v2。每图只有3个相关任务，使用逐点图，不伪造分布，不并入固定10组。匹配后仍重新求L1/L2、绑定当前地图/规则/车辆/搜索范围；只有最终完整通过且段匹配大于零才计入段复用成功。','',
 '![共享道路段逐点](shared_points.png)','',
 '| 地图 | 模式/任务 | 严格通过 | 段匹配数 | 局部完整通过 | 本次端到端 / s | 本次局部阶段 / ms |','|---|---|---|---:|---|---:|---:|']
 for mid,x in d['shared'].items():
  for m in ('B','C','ABC'):
   for r in x['per_task'][m]:lines.append(f"| {x['label']} | {m}/{r['id']} | {r['strict']} | {r['segment_match'].get('matched_segments',0)} | {r['local_success']} | {fmt(r['wall_s'])} | {fmt(r['stage_wall_ms'],1)} |")
 lines+=['','这里列的是本次调用实际耗时，不是被缓存轨迹的历史求解时间。容量上限、驻留估计、命中/失效/淘汰与拟合记录保留在原始JSON；缓存只解决几何复用问题，不能据这3条道路任务推广所有地图提速。','',
 '## 变化起点和偏差回归','',
 '| 工作负载 | 地图 | 变化请求严格通过（control→A） | 中位（s） | P95（s） |','|---|---|---:|---|---|']
 for kind,xs in d['regression'].items():
  for mid,x in xs.items():
   c,b=x['arms']['control']['moving'],x['arms']['A']['moving'];u,v=c['metrics']['e2e_s'],b['metrics']['e2e_s']
   lines.append(f"| {'精确变化起点' if kind=='moving' else '侧偏3 cm／航向2°'} | {x['label']} | {c['success']}/{c['n']} → {b['success']}/{b['n']} | {fmt(u['median'])} → {fmt(v['median'])} | {fmt(u['p95'])} → {fmt(v['p95'])} |")
 lines+=['','每工作负载每图另有10次初始规划，全部单独计数；变化请求来自同10条任务，不是新的独立路线。A各层输出、实际位姿、完整剩余轨迹和有效地图均与control一致。','',
 '![变化起点](moving_boxplots.png)','', '![偏差起点](perturbed_boxplots.png)','',
 '## 验证、否决记录与边界','']
 lines += [f"- {d['verification']['tests']}；原严格审计AST、L2原生库与续接实现核验一致。",f"- 本轮拥有进程和共享资源清理通过，共{len(d['verification']['cleanup'])}批，含被排除的探索批次。",f"- 父版{d['verification']['parent_files_unchanged']}个登记文件、release哈希、封存包哈希及L1_LOCK的源码/内核/地图/查询均核验未改变。历史生产入口和V8指针不写。",'- A与ABC均完成两模式各11项事件断言，包含乱序、重复、遮挡/无观测清除拒绝、过去障碍、前方和当前位置阻塞、清除恢复。Q09前方停车障碍仍失败安全撤回，清除后恢复；不写成已成功绕行。','- 首轮A不可变副本重复持有等探索数据保留并排除正式性能统计；最终C困难路线连接及回退的退化按同批正式数据列出并否决默认采用。A/B/C均保留实验开关。','- CPU12逐批串行、OMP/BLAS单线程、无profiler；固定批次顺序，不宣称随机化显著性。初始化和导出排除，准备开销单独保存在preparation.json/v9_preparation.json。','- 仅本轮两图静态完整规划及事件回放。不推广为九图、连续执行或实车验收；执行闭环不在本轮任务书范围。','',
 '## 原始数据与复现','',
 '正式固定集、变化起点、偏差与事件使用results/accepted_v9_final3对应子目录；共享道路诊断使用results/accepted_v9_shared_final/shared。旧final3/shared因第三组端点重复而排除并保留；所有探索与排除理由登记在SELECTED_DATA.json。COMPARISON.json保留输入/记录哈希、逐任务对照、路径质量及资源清理证据。任务批次的每case有轨迹、L1/L2输出、范围与完整outcome；任务批次有source_snapshot及已载入原生库快照。事件批次保留事件结果、轨迹、已载入原生库和真实ROS发布/读回/ACK日志，未单独复制逐批源码；事件运行期间与完成后均核验全局91项规划源码锁不变。','',
 '```bash','source env.bash','python3 tools/run_v9_protocol.py --output-root results/reproduce_new --phase fixed --final-mode A','python3 tools/run_v9_protocol.py --output-root results/reproduce_new --phase shared --final-mode A','python3 tools/run_v9_protocol.py --output-root results/reproduce_new --phase regression --final-mode A','taskset -c 14 python3 -m pytest -q tests','python3 tools/compare_v9.py --results-root results/reproduce_new --shared-root results/reproduce_new/shared --report-dir report/reproduce_new','python3 tools/write_v9_report.py --report-dir report/reproduce_new','```','',
 '输出目录必须尚不存在；复测更换目录，不覆盖正式记录。接口定义见INTERFACES.md，协议见EXPERIMENT_PROTOCOL.md。']
 (a.report_dir/'RESULTS.md').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()
