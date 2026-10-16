import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def f(x):return '—' if x is None else f'{x:.3f}'
def main():
 c=json.loads((ROOT/'report/COMPARISON.json').read_text());t=['# 3YD-V8-r5 转向修复触发纠正','','继承V8-r4 ABC，保持L1、L2参考、原生Smac、车辆模型、搜索范围和严格验收。修复两个检查器同时报告同一转向缺陷时未进入既有修复分支的问题。','',
 '## 原因与改动','','arena_medium A2B-06/08的分段分别通过，但拼接后最大真实转向跳变约16.4°。几何检查报告RAW_STEERING_JUMP，规范轨迹检查同时报告STEERING_DISCONTINUITY。原代码要求错误列表恰好只有前者，于是跳过了原有局部连续曲率修复。','',
 '新代码只在非空错误集合完全属于这两个转向错误时启动既有拟合。拟合实际改变位置和航向，随后仍执行全部原终验；不删除错误、不裁剪转向值、不放宽15°门槛。碰撞、越界、交通、姿态或其他错误仍拒绝。继续共享原8秒分段预算，未增加全图回退或重试。','',
 '## 同配置固定集','','原两图各10×3，额外两图各10×1；与紧邻的V8-r4 ABC正式记录比较。CPU12串行、单线程数设置相同，地图加载/启动/导出不计入端到端。主两图的重复样本和额外图分别统计。','',
 '| 地图 | 样本数 | 严格成功 r4→r5 | 端到端中位 / s | P95 / s | 峰值RSS中位 / GiB | 峰值RSS最大 / GiB |','|---|---:|---:|---:|---:|---:|---:|']
 for mid,v in c['maps'].items():
  a,b=v['r4'],v['r5'];vals=[]
  for m,stat in [('e2e_s','median'),('e2e_s','p95'),('tree_rss_gib','median'),('tree_rss_gib','max')]:
   x=a['metrics'][m]['all'][stat];y=b['metrics'][m]['all'][stat];vals.append(f(x)+'→'+f(y)+f' ({100*(y/x-1):+.1f}%)')
  t.append(f"| {v['label']} | {b['n']} | {a['success']}→{b['success']} | "+' | '.join(vals)+' |')
 t+=['','![四图耗时与资源分布](r4_r5_boxplots.png)','','## 首次／重复和分层计时','','L3包含适配，中位数不能相加。下表为r5首次／重复中位；没有重复实验的地图标为“—”。全部r4/r5分层统计和逐次数据见COMPARISON.json。','',
 '| 地图 | 首次／重复端到端 / s | L1 / ms | L2整体 / s | L3流水线 / s | 适配 / s | 地图与ACK / s |','|---|---:|---:|---:|---:|---:|---:|']
 for v in c['maps'].values():
  s=v['r5']['metrics'];vals=['／'.join(f(s[m][split]['median']) for split in ('first','repeat')) for m in ['e2e_s','l1_ms','l2_s','l3_s','adapter_s','map_sync_s']]
  t.append('| '+v['label']+' | '+' | '.join(vals)+' |')
 t+=['','## 分层内存','','各格是r5中位／最大。','',
 '| 地图 | L2注册状态 / MiB | Python PSS快照 / GiB | Smac峰值PSS / GiB | 服务栈峰值PSS / GiB |','|---|---:|---:|---:|---:|']
 for v in c['maps'].values():
  s=v['r5']['metrics'];vals=['／'.join(f(s[m]['all'][stat]) for stat in ('median','max')) for m in ['l2_state_mib','python_pss_gib','planner_pss_gib','stack_pss_gib']]
  t.append('| '+v['label']+' | '+' | '.join(vals)+' |')
 t+=['','L2注册数组不等于整个L2内存。Python包含地图、L1/L2与部分L3处理；服务栈仅planner/map/lifecycle且已含Smac，PSS快照、PSS峰值和进程树RSS不能相加。','','## 路径质量与剩余失败','',
 '| 地图 | 靠右中位 r4→r5 | 长度中位 r4→r5 / m | 既有成功最终轨迹相同 |','|---|---:|---:|---:|']
 notes=[]
 for v in c['maps'].values():
  a,b=v['r4'],v['r5'];same=sum(x['path.npy'] is True for x in v['parity']);before=sum(x['before'] for x in v['parity'])
  t.append(f"| {v['label']} | {f(a['right']['median'])}→{f(b['right']['median'])} | {f(a['length']['median'])}→{f(b['length']['median'])} | {same}/{before} |")
  for x in v['parity']:
   if x['after'] and not x['before']:notes.append(f"新增严格成功：{v['label']} {x['query']}。")
  for x in b['failures']:notes.append(f"仍失败：{v['label']} {x['query']}，{x.get('section_failure') or x.get('final_failures') or x['failure']}。")
 for note in notes:t+=['',note]
 t+=['','所有L1、L2参考及软目标与r4逐项相同；既有成功最终轨迹不变。新增成功改变了成功路径统计的样本集合，因此额外图成功路径长度中位数变化不能直接解释为路径变短或变长。','',
 '## 验证、局限与下一步','','141项测试通过。独立测试覆盖两个检查器共同拒绝的触发条件及碰撞、越界、曲率、未知错误的排除；原几何、交通、ACK和预算测试继续通过。源码快照、封存L1、原生Nav2库和退出清理均检查。结果仅代表四张固定地图的静态规划，不代表实车或完整动态验收。','',
 '本轮先修复了已经存在但被条件漏掉的局部求解机会，没有仓促引入新的全局轨迹优化器。剩余连续碰撞／范围问题需要针对首个片段提供位置和航向反馈。更大的内存收益仍依赖L3服务内真正按块存储和共享地图；现有完整地图ACK仍保留。','',
 '交付：src/three_yd_v5/steering_rejection.py、bounded_segments.py、src/three_yd_v4/r5_controller.py；README.md、INTERFACES.md、EXPERIMENT_PROTOCOL.md、report/COMPARISON.json及results/formal、results/additional原始记录。']
 (ROOT/'report/RESULTS.md').write_text('\n'.join(t)+'\n')
if __name__=='__main__':main()
