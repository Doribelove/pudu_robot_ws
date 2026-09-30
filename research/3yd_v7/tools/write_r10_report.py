"""Build a concise report directly from independent raw comparisons."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
 d=json.loads((ROOT/'report/PERFORMANCE.json').read_text());b=d['batches']
 names=[('r9_control_10x3','r9 同环境对照'),('r10_A_formal_10x3','A：仅地图同步'),('r10_B_formal_10x3','B：仅L2预检'),('r10_C_formal_10x3','C：仅L3修复复用'),('r10_ABC_formal_10x3','r10：A+B+C')]
 missing=[k for k,_ in names if k not in b]
 if missing:raise ValueError(missing)
 lines=['# 3YD-V5-r10 三项独立优化与正式实验','',
 '从冻结的 V5-r9 balanced 独立构建；地图、车辆、L1、0.15m XY D*、0.30m软目标、L3原0.05m Smac和0.8/1.2m搜索范围、严格终验保持原样。禁止全图 Smac 回退。','',
 '## 正式结果','',
 '全部批次串行运行，相同CPU12、固定Q01–Q10各重复三次、单线程数值库、不启用profiler。对照来自本次原r9工作区重跑，不混用上次日期的耗时。首次指每条查询在当前批次第一次执行，重复指第2/3次；不是30次独立冷启动。','',
 '| 配置 | 严格成功 | 总体中位 s | P95 s | 首次中位 s | 重复中位 s | RSS中位 GiB | RSS峰值 GiB |',
 '|---|---:|---:|---:|---:|---:|---:|---:|']
 for key,title in names:
  e=b[key];all=e['all']
  lines.append(f"| {title} | {e['success']}/{e['runs']} | {all['wall_s']['median']:.3f} | {all['wall_s']['p95']:.3f} | {e['first']['wall_s']['median']:.3f} | {e['warm']['wall_s']['median']:.3f} | {all['rss_gib']['median']:.3f} | {all['rss_gib']['max']:.3f} |")
 base=b[names[0][0]];final=b[names[-1][0]]
 def change(a,z):return 100*(z/a-1)
 lines+=['',f"组合相对本次r9：总体中位 {change(base['all']['wall_s']['median'],final['all']['wall_s']['median']):+.1f}%；P95 {change(base['all']['wall_s']['p95'],final['all']['wall_s']['p95']):+.1f}%；首次中位 {change(base['first']['wall_s']['median'],final['first']['wall_s']['median']):+.1f}%；重复中位 {change(base['warm']['wall_s']['median'],final['warm']['wall_s']['median']):+.1f}%；RSS中位 {change(base['all']['rss_gib']['median'],final['all']['rss_gib']['median']):+.1f}%；RSS峰值 {change(base['all']['rss_gib']['max'],final['all']['rss_gib']['max']):+.1f}%。",'',
 '## 分层实际耗时','',
 '| 阶段 | r9 首次 s | r10 首次 s | r9 重复 s | r10 重复 s |','|---|---:|---:|---:|---:|']
 for key,title,scale in [('controller_all_s','所有候选L2/控制器准备',1),('adapter_all_ms','L2→L3实际适配',.001),('map_update_all_s','地图更新/ACK',1),('l3_action_all_s','车辆规划动作',1),('curvature_repair_all_s','局部曲率修复',1),('strict_audit_all_ms','完整终验累计',.001),('l3_pipeline_all_s','完整L3（包含适配及以上子项）',1)]:
  v=[e[group][key]['median']*scale for group in ('first','warm') for e in (base,final)]
  lines.append('| '+title+' | '+' | '.join(f'{x:.3f}' for x in v)+' |')
 lines+=['','各列独立取中位，不能相加得到端到端中位；L3子项不能重复计入。修复器内部候选检查计入修复耗时。缓存命中计本次实际耗时，历史求解元数据不计入当前求解。','',
 '## 路径、靠右与ACK','',
 '| 配置 | L1/软目标/L2/L3全部逐点一致 | 靠右逐次一致 | 全部ACK | 全部在搜索范围内 | 长度中位 m | 靠右比例中位 |',
 '|---|---|---|---|---|---:|---:|']
 for key,title in names:
  e=b[key];lines.append(f"| {title} | {e['all_paths_equal']} | {e['right_metric_equal']} | {e['actual_maps_acknowledged']} | {e['within_mask']} | {e['all']['length_m']['median']:.3f} | {e['all']['right_fraction']['median']:.6f} |")
 lines+=['','## 三项改动的归因','',
 '**A：地图更新。** 用确定性的稀疏矩形覆盖全部变化格，替代包含大量未变格的单个大矩形。保留消息上限、原节拍、有效膨胀图内容ACK、两次一致观测及有界补发。地图数组与Smac均未裁剪。','']
 for key,title in names[1:]:
  e=b[key];ts=e['tile_updates']
  if ts:
   lines.append(f"- {title}：逻辑变化 {sum(t['changed_cells'] for t in ts):,} 格；原矩形需发布 {sum(t['baseline_cells'] for t in ts):,} 格/{sum(t['baseline_messages'] for t in ts)}条，实际 {sum(t['published_cells'] for t in ts):,} 格/{sum(t['message_count'] for t in ts)}条。该统计只含正常发布，缺块修复保持原代码另有日志。")
 lines+=['','**B：首次候选预检。** 外接圆图失败后，先对包含原矩形恢复全部可能开放格的宽松有向图做必要条件检查，方向和不穿角规则保持原样。仅确定不连通才跳过；不以超时判永久不可达。Q10首选候选由该预检排除，实际采用路线与对照相同；这不构成物理封路证据。','',
 '**C：曲率连接器。** 保留原优化器与精确参数，复用纯数学连接结果及固定矩阵。实际地图的车体碰撞、语义/交通和最终完整终验每次执行。缓存只对完全相同的数值问题生效，不等于机器人移动后的任意新查询都能获得同样收益。','']
 for key,title in names[1:]:
  e=b[key]
  if e['connector_calls']:lines.append(f"- {title}：连接器调用 {e['connector_calls']} 次，精确命中 {e['connector_hits']} 次；失败记录不包含超时。")
 lines+=['','## 内存','',
 '| 指标 | r9 | r10 |','|---|---:|---:|']
 for key,title in [('l2_states_mib','L2专属状态 MiB'),('planner_pss_gib','L3规划器PSS GiB'),('stack_pss_gib','L3栈PSS GiB'),('post_export_tree_pss_gib','输出后进程树PSS GiB')]:
  lines.append(f"| {title} | {base['all'][key]['median']:.3f} | {final['all'][key]['median']:.3f} |")
 lines+=['','本轮没有实施全图存储裁剪，因此不承诺大幅降低常驻内存。预检图和消息划分有临时数组，几何缓存也占少量内存；RSS峰值与输出后PSS属于不同采样时刻，不能混为一谈。L2专属状态不包含共享地图和全部代价场。','',
 '## 验证范围与取舍','',
 '单项先验证代表查询，再各10组预检，随后独立10×3正式实验，最后组合另做10×3。性能差异来自这些固定静态查询；没有开展动态实机闭环、多地图泛化或随机查询顺序实验。后台系统负载仍可能导致小幅抖动。','',
 '一次A试运行误选ROS域233，DDS端口越界导致启动失败，尚未执行任何查询。保留日志并完成资源清理，修正为合法域后重新测试，不将该错误混为算法导航失败。','',
 '## 复现与证据','',
 '- `README.md`、`EXPERIMENT_PROTOCOL.md`、`INTERFACES.md`：运行方式、对照口径和接口。',
 '- `report/PERFORMANCE.json`：全部统计、逐点对照、发布量和连接器命中。',
 '- `results/r9_control_10x3/runs.jsonl`：同环境原r9。',
 '- `results/r10_{A,B,C,ABC}_formal_10x3/runs.jsonl`：各单项及组合原始数据。',
 '- 每批 `GATE.json`、`source_snapshot`、`protocol.json`：验收与源版本。',
 '- `report/tests.txt`、`CODE_CHANGES.diff`、`INPUT_HASHES.json`、`FINAL_CODE_HASHES.json`、`CLEANUP.json`：测试、变更、输入、源版本及资源清理。','']
 (ROOT/'report/RESULTS.md').write_text('\n'.join(lines))
 print(ROOT/'report/RESULTS.md')
if __name__=='__main__':main()
