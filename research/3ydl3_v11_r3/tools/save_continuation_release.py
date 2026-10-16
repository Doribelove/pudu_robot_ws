import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PROJECT=Path('/home/robot/文档/ChatGPT/dddd-任务')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def main():
 d=json.loads((ROOT/'report/COMPARISON.json').read_text())
 assert d['verification']['tests'].startswith('207 passed')
 for w,maps in d['results'].items():
  for mid,x in maps.items():assert x['arms']['candidate']['all']['success']==(30 if w=='fixed' else 40)
 lines=['# 3YD-V8-r10：完整任务续接与小位姿偏差修复','',
 '独立继承新V8-r9 task_progress与新r8 firstplan；与封存的移动窗口r10无关。L1封存、L2二维D*、0.30m语义软引导、Smac/Jackal和原严格终验保持。',
 '', '本轮完成两步迭代：r9保留同任务已经经过的道路/停车接入上下文，复用并重新验收剩余完整路线；r10进一步处理相同请求的合法端点残差、小位姿偏差和经过路段的障碍变化。',
 '', '## 结果', '',
 '| 地图 / 工作负载 | 对照严格通过 | r10严格通过 | 中位耗时（对照→r10） | 中位降低 | P95（对照→r10） |',
 '|---|---:|---:|---:|---:|---:|']
 for w,maps in d['results'].items():
  for mid,x in maps.items():
   ss=('first','repeat') if w=='fixed' else ('initial','moving')
   for s in ss:
    a=x['arms']['control'][s];b=x['arms']['candidate'][s];u=a['metrics']['e2e_s'];v=b['metrics']['e2e_s']
    name={'first':'固定首次','repeat':'相同请求重复','initial':'本批初始任务','moving':'变化起点'}[s]
    if w=='perturbed' and s=='moving':name='起点偏差3cm/2°'
    lines.append(f"| {x['label']} / {name} | {a['success']}/{a['n']} | {b['success']}/{b['n']} | {u['median']:.3f}→{v['median']:.3f} s | {100*(1-v['median']/u['median']):.1f}% | {u['p95']:.3f}→{v['p95']:.3f} s |")
 lines+=['','原固定两图10组×3次仍60/60；精确变化起点60/60，对照58/60；偏差起点60/60，对照58/60。两个变化集各另有20次初始规划，均20/20。三个工作负载合计220次候选请求全部通过，但包含重复和相同20条路线，不能称为220条独立场景。',
 '原固定60条最终路径、L1输出、L2参考和软目标与同批对照完全相同；精确变化起点60条均为初始已验证轨迹的精确剩余部分。偏差情况下只修改短接回段，后续部分与初始轨迹完全相同。',
 'Q09 p50/p80的两处旧失败已在同任务续接中解决，不修改冻结L1，不猜测新的停车归属。首次规划本轮基本持平；r8约18%的首次优化继续保留，本轮主要收益在重复与变化起点。',
 '', '## 耗时、资源与路径质量','',
 '计时排除一次性地图/拓扑准备、ROS启动和结果导出；包含本次所有实际规划、ACK及严格终验。对照关闭任务续接，但保留r8 AB和r7整路线缓存。CPU12逐批串行、BLAS/OMP单线程、无profiler；不是随机化统计显著性研究。',
 '失败耗时也进入总体分布，成功耗时与双方共有成功样本在COMPARISON.json中单列。',
 '', '| 地图 / 固定10×3 | RSS中位（对照→r10） | RSS最大（对照→r10） | 最大值变化 |',
 '|---|---:|---:|---:|']
 for mid,x in d['results']['fixed'].items():
  a=x['arms']['control']['all']['metrics']['tree_rss_gib'];b=x['arms']['candidate']['all']['metrics']['tree_rss_gib']
  lines.append(f"| {x['label']} | {a['median']:.3f}→{b['median']:.3f} GiB | {a['max']:.3f}→{b['max']:.3f} GiB | {100*(b['max']/a['max']-1):+.1f}% |")
 lines+=['','固定集峰值RSS约增加1%–2%，换取重复及变化起点的额外复用；未出现大幅资源增长。Nav2仍完整存图，不能把停止重复搜索写成已经裁剪了Nav2地图。',
 '', '| 地图 / r10请求类型 | L1中位 ms | L2控制器中位 s | 层间适配中位 s | L3流水线中位 s | 端到端中位 s |',
 '|---|---:|---:|---:|---:|---:|']
 for mid,x in d['results']['fixed'].items():
  for w,s,label in [('fixed','first','首次'),('fixed','repeat','重复'),('moving','moving','变化起点'),('perturbed','moving','偏差起点')]:
   m=d['results'][w][mid]['arms']['candidate'][s]['metrics']
   fmt=lambda k: '—' if m[k]['median'] is None else f"{m[k]['median']:.3f}"
   lines.append(f"| {x['label']} / {label} | {fmt('l1_ms')} | {fmt('l2_s')} | {fmt('adapter_s')} | {fmt('l3_s')} | {fmt('e2e_s')} |")
 lines+=['','L3包含适配和终验；这些中位数不能相加。复用时L1/L2的0表示本次没有调用搜索，不代表首次计算免费。L2状态数组不是L2全部内存；L1无独立进程内存测量，不能把Python进程全部算给L1。RSS为worker与ROS子进程树逐次采样峰值，组件PSS为另外的观测口径。',
 '', '| 地图 / 起点偏差集 | 相对初始名义剩余轨迹的长度变化范围 |',
 '|---|---:|']
 for mid,x in d['results']['perturbed'].items():
  v=[p['length_delta_vs_nominal_suffix_percent'] for p in x['paired_paths'] if 'length_delta_vs_nominal_suffix_percent' in p]
  lines.append(f"| {x['label']} | {min(v):+.4f}% ～ {max(v):+.4f}% |")
 lines+=['','长度、靠右比例、终验及各层P95逐组统计保存在COMPARISON.json。相同原始请求路径不变；偏差请求的新增几何范围由task_progress.connector记录。点级source字段不作为局部连接长度或本次Smac调用次数的依据。',
 '', '![固定查询](fixed_boxplots.png)','', '![精确变化起点](moving_boxplots.png)','', '![小偏差起点](perturbed_boxplots.png)',
 '', '## 实现与验证','',
 '- 同query ID和同目标保留一项活动任务。精确位姿和访问唯一性匹配后，取完整剩余路径；明显离轨、反向或访问歧义回到原规划入口。',
 '- 小偏差候选限于原轨迹附近0.15m、航向差10°以内，尝试0.8/1.2/2/3m前视位置的短五次曲线；对实际新位姿和拼接点进行原车体、曲率、转向及交通检查，再执行ACK与完整终验。',
 '- 本次正式偏差覆盖3cm及2°，没有把0.15m/10°的所有组合宣称为已验证。',
 '- 交通审核仍用原规则：已走前缀与当前剩余路线共同检查经过顺序、方向、禁转和停车接入；车体审核只针对从当前实际起点到目标的轨迹。',
 '- 每次仍确认实际地图内容；地图变化立即撤回旧版本可执行轨迹。经过路段变化先复核剩余部分，前方受影响时在原有限空间内尝试L3；失败不返回旧路径。',
 '- 207项检查通过；冻结L1/输入、原终验和L2原生库保持。所有监督批次的拥有进程和共享资源均已清理。',
 '', '## 障碍事件与能力边界','',
 '两种模式各11项事件检查通过：已走路段新增障碍不重选路线；重复/乱序消息拒绝；没有空闲观测不能清除；当前位姿被障碍覆盖时停止输出；清除后保留任务上下文恢复。',
 '无语义A2B-01在原范围内绕过前方小障碍；Q09停车区前方障碍仍未找到可行路线，会撤回轨迹并等待，清除后恢复。不能把搜索失败断言为物理不可达，也不能声称动态避障全部成功。',
 '当前受影响后仍使用原L2软参考与有限范围进行L3修复；超过原参考范围的动态改道、受阻期间持续变化位姿的请求、连续定位/跟踪、到达处理及实车2m/s执行均未验收。调用者应串行调用plan/update，通过current_trajectory取得当前可用结果。',
 '图例与连续指标遵守宋体和逐次原始记录箱线图约束。固定路线与小偏差静态回放成绩不能推广为所有地图或实车成功率。',
 '', '## 复现与后续','', '```bash','source env.bash','python3 tools/run_continuation_batches.py','python3 tools/compare_continuation.py','python3 -m pytest -q tests','```',
 '', '下一步优先保持本轮已取得的续接收益，再研究新任务首次规划的地图扫描/层间处理开销；动态改道和定位跟踪另立验收，不混用本轮静态成功率。']
 (ROOT/'report/RESULTS.md').write_text('\n'.join(lines)+'\n')
 (ROOT/'README.md').write_text('# 3YD-V8-r10完整任务续接候选\n\n从新r9 task_progress独立继承；与旧移动窗口r10无关。详见report/RESULTS.md、INTERFACES.md、EXPERIMENT_PROTOCOL.md。r6主版本、冻结L1及历史生产入口保持。\n\n注册实验入口：source env.bash 后运行tools/run_continuation_batches.py；单任务API：three_yd_v5.task_progress.TaskProgressService。\n')
 (ROOT/'INTERFACES.md').write_text('''# V8-r10任务服务

## 输入

TaskProgressService(world, topology, cache_root, views, session, auditor, spec)，语义/无语义两种模式使用相同API；plan(Query)输入任务ID、当前真实XY/yaw和固定目标。地图/车辆/规则在实例生命周期内保持，障碍通过update(sequence,upsert,clear,observed_free)修改。

## 输出

plan返回当前起点至目标的完整可执行路径与原严格验收结果。task_progress说明是否复用、原起点/当前起点、匹配位置和局部连接；candidate_recovery记录本次实际各层时钟，未调用搜索为0。复用的L1/L2是原任务输出，不伪装为当前点新搜索。局部接回范围看connector日志，不以点级source推断。

## 状态

单活动任务，串行plan/update；goal或任务ID变化走原冻结L1。旧轨迹和上下文只在当前ACK、完整碰撞/曲率/转向、原交通规则审核通过后才能重新交付。current_trajectory()在地图版本变化/失败时为None，不将保留的候选几何视为可执行结果。清除障碍需要有效空闲观测。

0.8/1.2m有限L3、40万格及全图10%上限继续保留。新的短连接在已有空间内，禁止全图Smac回退。不是移动窗口、实车跟踪或完整动态改道服务。
''')
 (ROOT/'EXPERIMENT_PROTOCOL.md').write_text('''# 正式协议

两图：万达3w地库（mengjia），arena_large_250x300_2d_005。固定输入/L1保持。CPU12，单线程OMP/BLAS，控制与候选逐批串行，profiler关闭。

固定10组×3；不可变父版轨迹20/50/80%位姿各30次，目标不变，另各10次初始；相同派生点横向交替±3cm、航向±2°各30次，另各10次初始，actual_queries.json保存实际端点。对照关闭任务续接，保留所有r8/r7优化。两个变化工作负载各40次每图，不当作固定组重复或实车执行。

原验收保持、ACK本次确认；轨迹与层级输出保存。请求计时排除准备/启动/导出，包含所有本次计算。失败耗时、成功耗时和共同成功请求分别统计。进程树RSS峰值采样，PSS快照另外报告。

正式仅results/fixed、moving、perturbed和events；probe、smoke均排除性能。所有批次监督清理。独立历史迭代r9存于3yd_v8_r9_progress_20261008，不覆写父版结果。
''')
 (ROOT/'task.sh').write_text('#!/usr/bin/env bash\nset -eo pipefail\nworkspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"\nsource "$workspace_dir/env.bash"\nexec python3 "$workspace_dir/tools/run_task_progress.py" "$@"\n')
 (ROOT/'task.sh').chmod(0o755)
 base={'version':'3YD-V8-r10','branch':'fullroute_continuation','workspace':str(ROOT),'status':'tested_independent_candidate_not_production',
  'parent':'/home/robot/workspaces/3yd_v8_r9_progress_20261008','L1_frozen':True,'moving_window':False,
  'fixed_strict_success':[60,60],'moving_strict_success':[60,60],'pose_perturbed_strict_success':[60,60],
  'moving_and_perturbed_initials':[40,40],'tests':207,'events':[11,11],'cleanup_batches':len(d['verification']['cleanup']),
  'report':str(ROOT/'report/RESULTS.md'),'comparison':str(ROOT/'report/COMPARISON.json'),
  'limitations':['new-task first planning unchanged from r8','Nav2 full map remains','semantic future obstacle case failed safely and recovered on clear','no continuous tracking or vehicle acceptance']}
 write(ROOT/'BASELINE.json',base)
 write(ROOT/'report/SELECTED_DATA.json',{'formal':['results/fixed','results/moving','results/perturbed','results/events'],
  'excluded':['report/connector_probe_mengjia','report/connector_probe_arena_large_250x300_2d_005','results/smoke_perturbed_Q09','results/events_semantic_smoke','results/events_free_smoke']})
 files={str(p.relative_to(ROOT)):sha(p) for p in sorted(ROOT.rglob('*')) if p.is_file() and not any(x in p.parts for x in ('__pycache__','.pytest_cache','.git')) and p.suffix!='.pyc' and p.name not in ('RELEASE.json','VERIFY.json')}
 write(ROOT/'RELEASE.json',{**base,'files':files});assert all(sha(ROOT/f)==v for f,v in files.items())
 verify={'files':len(files),'all_match':True,'release_sha256':sha(ROOT/'RELEASE.json')};write(ROOT/'VERIFY.json',verify)
 pointer=PROJECT/'3YD_V8_CURRENT.json';state=json.loads(pointer.read_text())
 state['latest_task_progress_candidate']={**json.loads(Path(base['parent'],'BASELINE.json').read_text()),'release_sha256':sha(Path(base['parent'],'RELEASE.json'))}
 state['latest_continuation_candidate']={**base,'release_sha256':verify['release_sha256']}
 state['next_work']='preserve tested full-route continuation; optimize new-task first planning without losing success; dynamic route changes/live tracking require separate acceptance'
 write(pointer,state)
 section='''## 2026-10-08 新V8-r9/r10：完整任务进度续接

r9独立目录 `/home/robot/workspaces/3yd_v8_r9_progress_20261008`；r10独立目录 `/home/robot/workspaces/3yd_v8_r10_continuation_20261008`，不是封存的移动窗口r9/r10。冻结L1、r6主版本、历史入口保持。

r9解决Q09变化起点的停车/方向上下文丢失；r10补相同请求复用、3cm/2°实际位姿短连接及已走路段影响判断。原两图60/60，精确变化起点60/60，偏差变化起点60/60（两变化集另40次初始也通过）。对照变化集各58/60。重复中位降低37.5%/39.2%，精确变化起点降低87.0%/88.8%，偏差起点降低86.9%/87.4%；首次基本持平，固定峰值RSS约增1%/2%。

207检查、两模式22事件、17批拥有资源清理通过。前方小障碍无语义案例可绕；Q09停车区前方障碍仍失败并安全撤回，清除后恢复。不可宣称动态改道、连续定位跟踪或实车验收。报告 `report/RESULTS.md`、原始索引 `report/SELECTED_DATA.json`。

'''
 for name in ('3YD_CURRENT_PROGRESS.md','3YD_CURRENT_VERSION.md'):
  p=PROJECT/name;prior=p.read_text() if p.exists() else ''
  if section not in prior:p.write_text(section+prior)
 rule='\n- 2026-10-08新V8-r9任务进度与r10完整续接候选完成；最新目录`/home/robot/workspaces/3yd_v8_r10_continuation_20261008`，不是旧移动窗口r10。两图固定60/60、精确变化起点60/60、小偏差起点60/60；207检查、22事件和17批清理通过。首次基本持平、固定RSS最大约增1%–2%；Q09前方动态障碍仍未求得路线，会撤回轨迹并在清除后恢复。L1封存、r6和历史生产入口保持；连续定位/跟踪及实车未验收。\n'
 p=PROJECT/'AGENTS.md'
 if rule not in p.read_text():p.write_text(p.read_text()+rule)
 print(json.dumps(verify))
if __name__=='__main__':main()
