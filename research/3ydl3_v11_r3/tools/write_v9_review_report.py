"""Write V9-r1 conclusions only from registered raw comparisons."""
import json
from pathlib import Path
from compare_v9 import read,summary,pair
from analyze_v8 import MAPS,measure,stats
ROOT=Path(__file__).resolve().parents[1]

def main():
 d=json.loads((ROOT/'report/COMPARISON.json').read_text());candidate=d['candidate'];lines=[]
 def add(s=''):lines.append(s)
 def metric(arm,part,key):return arm[part]['metrics'][key]['median']
 def pct(a,b):return 100*(b/a-1)
 add('# 3YD-V9-r1 持续审核迭代')
 add()
 add('独立继承V9-r0，封存L1、V8-r11、r6及历史生产入口保留。当前采用组合为'+candidate+'：A不可变地图哈希复用、S精确审计判定存储与有界转换'+('、K紧凑不可变整轨迹记录。' if candidate=='ASK' else '。K紧凑记录保留为独立实验，尚未满足本轮额外采用条件。')+'J解析局部连接和B跨任务道路段复用仍单独评估，不能把有界回退后的完整成功写成局部策略成功。')
 add()
 add('完整路线一次返回；原0.05 m地图、二维0.15 m D*、语义内移0.30 m、Jackal、Smac48、正常0.8 m/转角1.2 m有界范围、40万格及地图10%上限保持，无移动窗口和全图自由搜索。')
 add()
 add('## 固定集同批结果')
 add()
 add('| 地图 | 配置 | L1有效输出 | 三层严格成功 | 首次中位 / s | 重复中位 / s | 全部请求进程树RSS中位 / GiB | Python PSS中位 / GiB |')
 add('|---|---|---|---|---:|---:|---:|---:|')
 for mid,data in d['fixed'].items():
  for m,arm in data['arms'].items():
   add(f"| {data['label']} | {m} | 30/30 | {arm['all']['success']}/30 | {metric(arm,'first','e2e_s'):.3f} | {metric(arm,'repeat','e2e_s'):.3f} | {metric(arm,'all','tree_rss_gib'):.3f} | {metric(arm,'all','python_pss_gib'):.3f} |")
 add()
 for mid,data in d['fixed'].items():
  a,b=data['arms']['control'],data['arms'][candidate]
  add(f"{data['label']}：首次中位变化{pct(metric(a,'first','e2e_s'),metric(b,'first','e2e_s')):+.1f}%，重复中位{pct(metric(a,'repeat','e2e_s'),metric(b,'repeat','e2e_s')):+.1f}%，全部请求进程树RSS中位{pct(metric(a,'all','tree_rss_gib'),metric(b,'all','tree_rss_gib')):+.1f}%。")
 add()
 add('每图固定10组×3次，首次n=10、重复n=20、全部n=30；两图60次并非60条独立路线。S及候选的L1/L2/软目标/有效地图/范围/完整轨迹、独立车体审计、规范路径指标及精确车体检查次数与同批control全部相同。')
 add()
 add('![首次原始分布](fixed_boxplots.png)')
 add()
 add('![重复原始分布](repeat_boxplots.png)')
 add()
 add('| 地图 | 配置 | 首次P95 / s | 首次最大 / s | 重复P95 / s | 进程树RSS P95 / GiB | RSS最大 / GiB |')
 add('|---|---|---:|---:|---:|---:|---:|')
 for mid,data in d['fixed'].items():
  for m,arm in data['arms'].items():
   f=arm['first']['metrics']['e2e_s'];r=arm['repeat']['metrics']['e2e_s'];mem=arm['all']['metrics']['tree_rss_gib']
   add(f"| {data['label']} | {m} | {f['p95']:.3f} | {f['max']:.3f} | {r['p95']:.3f} | {mem['p95']:.3f} | {mem['max']:.3f} |")
 add()
 add('## 实际减少了哪些开销')
 add()
 add('S保存的是原审计对每个栅格“需要精确多边形碰撞检查”的完全相同布尔判定，逐格验证后按位存储；它不提供数值距离，不声称近似距离可替代原距离。最小间隙仍读取原地图距离场，每次路径交付仍执行完整原审计，近障精确检查及拒绝阈值不变。')
 add()
 for mid,data in d['fixed'].items():
  v=data['initialization'][candidate]['audit_storage']
  add(f"{data['label']}：审计判定字段由{v['original_bytes']/2**20:.2f} MiB减少到{v['stored_bytes']/2**20:.2f} MiB，压缩与逐格验证准备耗时{v['preparation_ms']:.2f} ms。")
 add()
 add('S只在原硬障碍闭合后会保持自由的格子上计算相同占据量化，并用逐值完全相同的StaticLayer查表；原距离变换域、软成本及膨胀保持。内容哈希采用相同字节流而避免整图临时bytes；仅真正不可变bytes支持的地图才缓存哈希，实际服务器读回仍重新计算。')
 add()
 add('K将符合已知平坦字段的成功轨迹点保存为不可变数值列和常量字段；每次取出恢复独立点字典，未知/嵌套字段回到原深拷贝。缓存只含几何及原元数据，没有ACK或审计证书，8项/8 MiB上限保留。')
 add()
 add('| 地图 | 配置 | 地图准备 / s | 地图准备进程树RSS峰值 / GiB | 额外A准备 / ms | 审计压缩准备 / ms | ROS启动 / s |')
 add('|---|---|---:|---:|---:|---:|---:|')
 for mid,data in d['fixed'].items():
  for m,x in data['initialization'].items():
   add(f"| {data['label']} | {m} | {x['preparation']['wall_ms']/1000:.3f} | {x['preparation']['rss_peak_bytes']/2**30:.3f} | {x['v9_preparation']['immutable_world_preparation_ms']:.2f} | {x['audit_storage'].get('preparation_ms',0):.2f} | {x['session_startup']['wall_ms']/1000:.3f} |")
 add()
 add('查询计时排除一次性初始化与导出。S当前仍先构建原float64审计距离场，再验证和压缩，因此尚未消除该初始化峰值；数组账本不等同进程RSS/PSS，不把查询驻留下降写成启动峰值下降。Nav2和Python的原完整地图仍常驻。')
 add('原审计距离场构建耗时与峰值本轮没有单独计量，上表的地图准备不包含全部审计初始化；压缩准备字段只记录压缩及逐格验证。')
 add()
 selection=json.loads((ROOT/'report/SELECTION.json').read_text())
 add('K的额外工程采用条件为：两图重复中位均至少比AS下降3%、首次中位增幅不超过2%、进程树RSS中位增幅不超过0.5%，并保持完整输出一致。这些是工程取舍门槛，不是显著性检验。实际选中'+selection['selected']+'，逐项结果保存在SELECTION.json。')
 add()
 add('## 变化起点、偏差与障碍事件')
 add()
 add('| 场景 | 地图 | 配置 | 初始严格成功 | 变化起点严格成功 | 变化请求中位 / s |')
 add('|---|---|---|---|---|---:|')
 for kind,maps in d['regression'].items():
  for mid,data in maps.items():
   for m,x in data['arms'].items():add(f"| {kind} | {data['label']} | {m} | {x['initial']['success']}/10 | {x['moving']['success']}/30 | {metric(x,'moving','e2e_s'):.3f} |")
 add()
 add('两图精确变化起点共60/60，3 cm／2°偏差共60/60；同批control与候选输出和完整审计判定均一致。障碍事件control与候选各两图11项，共44项断言，状态/可用轨迹与输出一致。Q09前方停车障碍仍未求得合法路线，会撤回轨迹并在清除后恢复；这不是新增的动态避障通过案例。')
 add()
 add('| 地图 | 配置 | 事件断言 | 前方障碍求得路线 | 过去障碍触发重规划 |')
 add('|---|---|---|---|---|')
 for m,maps in d['events'].items():
  for mid,x in maps.items():
   label=next(label for key,label in MAPS if key==mid)
   add(f"| {label} | {m} | 11/11 | {x['future_obstacle_success']} | {x['past_obstacle_replanned']} |")
 add()
 add('44项事件断言不是44条避障路线成功。两个场景都检查当前起点被挡时撤回、清除后恢复，以及旧版本/重复更新和无观测清除的处理。')
 add()
 add('![精确变化起点](moving_boxplots.png)')
 add()
 add('![偏差起点](perturbed_boxplots.png)')
 add()
 add('## J局部连接独立诊断')
 add()
 diag=ROOT/'results/review_diagnostic_v2';info={}
 add('| 地图 | 配置 | 完整严格成功 | 局部独立成功 | 有界完整回退 | 首次实际中位 / s |')
 add('|---|---|---|---|---|---:|')
 for mid,label in MAPS:
  info[mid]={}
  for m in ('C','CJ'):
   rows=read(diag/m/mid);s=summary(rows);info[mid][m]=s
   add(f"| {label} | {m} | {s['success']}/{s['n']} | {s['local_success']}/{s['n']} | {s['bounded_fallbacks']}/{s['n']} | {s['metrics']['e2e_s']['median']:.3f} |")
  info[mid]['parity']=pair(diag/'C'/mid,diag/'CJ'/mid)
 add()
 add('J使用有限候选的五次曲线，解析匹配原锚点位置、真实切线及相邻曲率；采样步长受导数凸包界约束，不改yaw伪装接缝。当前范围、连续车体及完整交通/曲率/转向审计仍决定是否可交付。C/CJ每图各10条一次，属于连接诊断；其中有单元检查并行执行的时间段，耗时仅作诊断，不用于正式性能采用结论。局部输出允许改变，逐任务长度变化、接缝与完整审计记录在CONNECTION_DIAGNOSTIC.json。J未作为默认完整规划策略。')
 add()
 add('## 跨任务段缓存的独立归因')
 add()
 add('| 地图 | 配置 | 新任务严格成功 | 真正段复用成功 | 新任务中位 / s | 全路径缓存命中 |')
 add('|---|---|---|---|---:|---|')
 for mid,data in d['shared'].items():
  for m,x in data['arms'].items():
   s=x['new_tasks'];add(f"| {data['label']} | {m} | {s['success']}/3 | {s['segment_reuse_success']}/3 | {s['metrics']['e2e_s']['median']:.3f} | {s['whole_route_hits']} |")
 add()
 add('每图先一条原始任务建几何缓存，再三条起点、终点和任务ID均不同的新任务；各组启用相同S+K。A对AB和ACJ对ABCJ分别隔离B，后一对同时启用J。每组新任务仅n=3且相关，不并入固定10验收；匹配段只是候选，当前范围、车体及完整任务交通审计重新执行。')
 add()
 add('![共享道路逐点诊断](shared_points.png)')
 add()
 add('## 审核与记录')
 add()
 v=d['verification'];add(f"{v['tests']}；{len(v['cleanup'])}批拥有进程/共享资源清理通过。L1源码、输入、像素输入、原严格审计AST、L2原生库、任务续接源码、V9-r0/V8-r11完整release清单、封存归档和生产入口均核验保持。")
 add()
 add('正式数据仅'+d['data_selection']['formal']+'；profile、storage_diagnostic_v1和review_diagnostic_v2均为诊断且排除正式性能结论。源锁、逐case L1/L2/软目标/轨迹/范围/原始结果、实际ACK日志、载入原生库及清理日记保留，正式箱线图只由原始逐次记录生成并加载宋体。')
 add()
 add('验收范围仍为本轮两图静态完整规划、起点变化和障碍事件回放；没有更多地图全部通过、连续控制、动态实车或实车执行验收。')
 (ROOT/'report/RESULTS.md').write_text('\n'.join(lines)+'\n')
 (ROOT/'report/CONNECTION_DIAGNOSTIC.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':main()
