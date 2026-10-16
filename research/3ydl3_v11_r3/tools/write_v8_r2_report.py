import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MAPS=[('mengjia','万达3w地库（语义）'),('arena_large_250x300_2d_005','arena_large（无语义）')]

def main():
 c=json.loads((ROOT/'report/COMPARISON.json').read_text());s=json.loads((ROOT/'report/STATISTICS.json').read_text());v=json.loads((ROOT/'report/VERIFICATION.json').read_text());abl=json.loads((ROOT/'report/ABLATIONS.json').read_text())
 text='''# 3YD-V8-r2：保留效果，降低恢复成本

2026-10-08。从V8-r1建立独立工作区，L1继续封存，生产入口保留。优化组两张地图固定10组×3次，**60/60严格通过**，全部L1、L2参考、软目标和最终轨迹与控制组及已保存的r1逐项一致。

## 改了什么

| 优化 | 原来多做的工作 | 最终采用的方法 | 保留的检查 |
|---|---|---|---|
| A：L2矩形恢复 | 在整张精细地图上做12次矩形形态学筛选，用于调整候选格检查次序；随后仍检查全部候选 | 删除只影响顺序的预筛。路径上的姿态优先检查，其余候选按格顺序检查 | 候选集合不减少；9个子位置×24个姿态的原精确矩形检查、证书链与L3连续车辆终验不变 |
| B：L3范围变化处理 | 每次分段调用提取全图变化坐标、复制“已应用”范围，再扫描残留；即使范围没变化也做这套处理 | 每次比较真实数组；无变化直接返回零变化。存在变化时只统计变化格。省去可从布尔逻辑证明冗余的全图副本 | 每次重算内容摘要，原地修改可被识别；原地图有效代价ACK、逐段检查及整条轨迹终验不变 |

没有修改L1、D*搜索状态、障碍解释、软引导、Smac参数或车辆模型；没有复用最终轨迹、放宽验收或增加恢复次数。L3普通0.8 m／拐角1.2 m，开放格最多40万且不超过全图10%，禁止全图回退。r1最多4段、共享8秒恢复截止和16条策略缓存保持。

## 对照方法与效果核验

先保存带cProfile的A2B-03定位记录，只用于查开销，不能把含剖析器的耗时与正式数据比较。A、B、AB分别在该案例各跑3次，均严格通过且路线相同；它们是诊断子集，不能代替完整地图验收。

主要对照使用同一新工作区：`control`关闭A/B，规划逻辑与r1等价；`AB`为优化组。两组各两张地图10×3，共120次完整调用，串行CPU12、单线程数值库、独立新进程和空查询缓存。每条首次后紧接两次重复。地图加载、建图、ROS启动和结果导出排除。不是随机交叉测试，几个百分点的波动应谨慎解释。

| 地图 | 控制组严格通过 | r2严格通过 | 参考／软目标／最终轨迹 |
|---|---:|---:|---|
| 万达3w地库 | 30/30 | 30/30 | 全部相同，靠右指标也相同 |
| arena_large | 30/30 | 30/30 | 全部相同，无语义不计算靠右 |
| 合计 | 60/60 | **60/60** | 与历史r1的60条最终轨迹也相同 |

77项检查全部通过，包含输入封存、变化格统计等价、原地修改识别、非连续数组、矩形候选检查次序不影响证书、恢复预算及原有回归。严格终验函数未修改，原生Smac及costmap库保持同一版本。

## 端到端耗时与资源

“首次”是每组第一轮，“重复”是第二、三轮。全部P95依据每图30条；RSS是查询期间整个工作进程树的采样峰值。变化负值表示降低。

| 地图／指标 | r1逻辑控制组 | r2 AB | 变化 |
|---|---:|---:|---:|
'''
 specs=[('e2e_s','all','median','端到端中位数 / s'),('e2e_s','first','median','首次中位数 / s'),('e2e_s','repeat','median','重复中位数 / s'),('e2e_s','all','p95','端到端P95 / s'),('e2e_s','all','max','最慢查询 / s'),('tree_rss_gib','all','median','查询峰值RSS中位 / GiB'),('tree_rss_gib','all','max','查询峰值RSS最大 / GiB')]
 for mid,label in MAPS:
  for key,split,stat,title in specs:
   q=c[mid]['metrics'][key][split]
   text+=f"| {label}：{title} | {q['control'][stat]:.3f} | {q['r2'][stat]:.3f} | {q['change_percent'][stat]:+.1f}% |\n"
 text+='\n![逐次对照箱线图](r1_r2_boxplots.png)\n\n图中文字为宋体SimSun。每组n=30；箱体Q1～Q3，中线中位数，须为1.5×IQR内最远样本，圆点离群值，线性坐标。图中每个样本均严格成功。\n\n## A2B-03恢复成本\n\n'
 a=[json.loads(x) for x in (ROOT/'results/control/arena_large_250x300_2d_005/runs.jsonl').read_text().splitlines() if x.strip()];b=[json.loads(x) for x in (ROOT/'results/formal/arena_large_250x300_2d_005/runs.jsonl').read_text().splitlines() if x.strip()]
 a=[r for r in a if r['query_id']=='A2B-03'];b=[r for r in b if r['query_id']=='A2B-03']
 text+='| 指标 | 控制组 | r2 | 变化 |\n|---|---:|---:|---:|\n'
 for label,getter in [('首次端到端 / s',lambda r:r[0]['wall_ms']/1000),('重复端到端中位 / s',lambda r:sum(x['wall_ms'] for x in r[1:])/2000),('首次L2控制器 / s',lambda r:r[0]['recovery']['all_candidates_controller_ms']/1000),('首次L3流水线 / s',lambda r:r[0]['recovery']['all_candidates_l3_pipeline_ms']/1000),('首次分段恢复 / s',lambda r:r[0]['r1_segmented_recovery']['wall_ms']/1000),('重复分段恢复中位 / s',lambda r:sum(x['r1_segmented_recovery']['wall_ms'] for x in r[1:])/2000)]:
  x,y=getter(a),getter(b);text+=f'| {label} | {x:.3f} | {y:.3f} | {(y/x-1)*100:+.1f}% |\n'
 text+='''
首次仍包含先失败的整段Smac及随后四段恢复；后两次按原r1策略缓存跳过已知失败的整段动作，各自仍执行四段真实规划、逐段检查和整路径终验。本轮没有把首次请求伪装成缓存命中，也没有根据查询编号选择恢复策略。A2B-06的原成功修复能力保持。

![恢复案例逐点对照](A03_points.png)

该案例每组只有3次（一次首次、两次重复），用逐点图，不能据此承诺所有长路线的收益。

## 各层实际耗时

L3流水线含层间适配、地图确认、Smac、修复和终验；各行相互包含，不能把中位数相加。每格为“控制组→r2”，首次／重复分别是10／20次的中位数。L1是本次routes调用，不是独立L1整体任务计时。

| 阶段 | 万达3w首次 | 万达3w重复 | arena_large首次 | arena_large重复 |
|---|---:|---:|---:|---:|
'''
 for key,label in [('l1_ms','L1 routes / ms'),('l2_s','L2控制器处理 / s'),('dstar_s','L2实际D* / s'),('adapter_s','L2→L3适配 / s'),('l3_s','L3流水线 / s'),('map_sync_s','地图处理与ACK / s'),('smac_action_s','Smac动作累计 / s'),('curvature_repair_s','局部修复累计 / s'),('strict_audit_s','完整终验累计 / s')]:
  values=[]
  for mid,_ in MAPS:
   for split in ('first','repeat'):
    q=c[mid]['metrics'][key][split];values.append(f"{q['control']['median']:.3f}→{q['r2']['median']:.3f}")
  text+='| '+label+' | '+' | '.join(values)+' |\n'
 text+='\nD*重复搜索为0表示原参考复用，控制器和适配仍按当前调用计时。L1和Smac搜索没有优化，计时波动不能归因于本轮提高其算法性能。各层全部P95及逐次统计保存于`COMPARISON.json`。\n\n## 分层资源\n\n| 指标 | 万达3w控制组中位／最大 | 万达3w r2中位／最大 | arena_large控制组中位／最大 | arena_large r2中位／最大 |\n|---|---:|---:|---:|---:|\n'
 for key,label in [('l2_state_mib','L2登记状态 / MiB'),('python_pss_gib','Python工作进程PSS / GiB'),('planner_pss_gib','planner_server峰值PSS / GiB'),('stack_pss_gib','监测服务栈峰值PSS / GiB'),('tree_rss_gib','查询进程树峰值RSS / GiB')]:
  values=[]
  for mid,_ in MAPS:
   for version in ('control','r2'):
    q=c[mid]['metrics'][key]['all'][version];values.append(f"{q['median']:.3f} / {q['max']:.3f}")
  text+='| '+label+' | '+' | '.join(values)+' |\n'
 text+='''
本批arena_large的原生planner峰值PSS由0.477升至0.510 GiB（增加约34 MiB），监测服务栈峰值PSS也随之增加；因此不能称各层内存都下降。完整进程树峰值RSS仍略降，L2登记状态不变。\n\nL2登记状态不是整个L2；Python PSS是查询后快照，包含地图、L1/L2及部分L3适配／修复；服务栈只监测planner、map、lifecycle三类进程且包含planner；进程树RSS是完整查询采样峰值。不能相加不同口径。L1独立完整内存本轮未隔离测量。删除预筛会减少其数组和计算中间量，但总RSS不按该数组大小直接推算。

## 独立诊断对照

| 开关 | A2B-03严格通过 | 首次 / s | 重复2 / s | 重复3 / s |
|---|---:|---:|---:|---:|
'''
 for mode,q in abl.items():text+=f"| {mode} | 3/3 | {q['first_ms']/1000:.3f} | {q['repeat_ms'][0]/1000:.3f} | {q['repeat_ms'][1]/1000:.3f} |\n"
 text+='''
诊断只执行A2B-03，因此它要独自承担此前未建立的共享代价数组；正式地图按十组固定顺序执行，不能混用两者的首次耗时。A/B单项没有完整地图30次样本，合并AB已做完整两图验收。

## 限制与下一步

验证范围仍是两张地图20组静态任务，不代表九张地图、动态闭环或实车。第一条陌生路线仍需一次整段搜索才能确认是否触发分段，相关失败成本未消除；本轮避免的是重复计算，不以新增重试换成功。

后续可单独评估是否按路线结构选择整段／分段，必须保留同一有界范围、共享预算和备选处理，并在更多地图验证后才能采用。当前先保留已验收的r2作为候选，不修改历史生产入口。

## 交付与复现

- `report/STATISTICS.json`：优化组各层、首次／重复、P95与资源。
- `report/COMPARISON.json`：同批控制／优化对照及原始记录校验。
- `report/VERIFICATION.json`：60条L1／L2／软目标／最终轨迹与控制、历史r1配对，输入封存、终验源码、监督退出核验。
- `report/ABLATIONS.json`：独立A/B/AB诊断数据；`report/tests.txt`：77项检查。
- `results/control/<map>/runs.jsonl`、`results/formal/<map>/runs.jsonl`：完整逐次原始记录、源文件与原生库快照、轨迹和范围。
- `results/diagnostic/r1_profile_A03`：优化前剖析记录，非正式性能数据。
- 复现命令与开关见`README.md`，数据口径见`EXPERIMENT_PROTOCOL.md`；封存核对见`L1_LOCK.json`。

所有本轮监督器及planner、map、lifecycle退出与共享资源清理已核对。
'''
 (ROOT/'report/RESULTS.md').write_text(text)
 print('r2 report written')

if __name__=='__main__':main()
