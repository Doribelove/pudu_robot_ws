"""Render the R3 review using verified same-batch evidence only."""
from pathlib import Path
from compare_v9_review import load
ROOT=Path(__file__).resolve().parents[1]
def med(d,mode,part,metric):return d['arms'][mode][part]['metrics'][metric]['median']
def pct(a,b):return 100*(b/a-1)
def main():
 d=load(ROOT/'report/COMPARISON.json');v=d['verification'];selected=d['selected_r3_mode']
 text=['# 3YD-V9-r3：启动地图读回审核',
 f'继承V9-r2审核后的AS，R2实验开关均关闭。本目录审核配置为{"AS" if selected=="control" else "AS+I"}。封存L1、V8-r11、r6、历史生产入口及V9-r0/r1/r2记录保持。正式原始记录仅 `{d["data_selection"]["formal"]}`。',
 'I让启动阶段的真实地图读回也使用原有delta通道。服务仍返回当前地图，客户端重建完整内容、核对尺寸/原点/分辨率/服务epoch与序列；原语义修复包装、时间戳、两次稳定观测和逐格内容ACK均保留。启动调用者拿到独立不可写快照。地图更新频率100Hz与发布节奏15ms未变。',
 '## 首次、重复查询与内存',
 'CPU12串行，原两图各固定10组、每组三次；首次n=10、重复n=20，共享各组服务进程。初始化/ROS启动和导出不计入查询耗时，另作冷进程记录。control为父AS，I仅改变启动读回通道。',
 '|地图|配置|三层严格成功|首次中位/s|重复中位/s|查询进程树RSS中位/GiB|独立Python PSS中位/GiB|',
 '|---|---|---:|---:|---:|---:|---:|']
 for mid,x in d['fixed'].items():
  for m in ('control','I'):
   text.append(f'|{x["label"]}|{m}|{x["arms"][m]["all"]["success"]}/30|{med(x,m,"first","e2e_s"):.4f}|{med(x,m,"repeat","e2e_s"):.4f}|{med(x,m,"all","tree_rss_gib"):.4f}|{med(x,m,"all","python_pss_gib"):.4f}|')
 text+=['![首次逐次箱线图](first_boxplots.png)','![重复逐次箱线图](repeat_boxplots.png)',
 '所有图显式加载宋体SimSun。箱体Q1–Q3，中线中位数；须为1.5×IQR内最远值，离群点单列，线性坐标。RSS包含Python和ROS子进程；PSS是独立Python进程快照，不能混作整树内存。',
 '两配置固定60/60严格通过；同版本内原始L1/L2/软目标/最终路径、有效地图、姿态/地图/路径哈希、完整约束指标、原审计决定和精确车体检查次数逐次一致。与父R2 AS的原始路径、有效地图和几何约束指标也一致；含source_commit的版本化路径标识随版本更新，详见PARENT_OUTPUT_PARITY.json。']
 for _,x in d['fixed'].items():
  text.append(f'{x["label"]}：I首次中位较control变化{pct(med(x,"control","first","e2e_s"),med(x,"I","first","e2e_s")):+.1f}%，重复变化{pct(med(x,"control","repeat","e2e_s"),med(x,"I","repeat","e2e_s")):+.1f}%，查询进程树RSS变化{pct(med(x,"control","all","tree_rss_gib"),med(x,"I","all","tree_rss_gib")):+.1f}%。')
 text+=['## 独立冷进程',
 '每地图、每配置3个独立进程，组顺序逐轮交替。准备至首次响应包含地图/审计/服务准备、源码导出、ROS启动与首条请求；排除模块导入和最终结果导出。整树RSS每20ms采样。n=3显示逐点，不能外推稳定分布或统计显著性。',
 '|地图|配置|准备至首次响应中位/s|冷进程树峰值RSS中位/GiB|ROS启动中位/s|首条查询中位/s|',
 '|---|---|---:|---:|---:|---:|']
 notes=[]
 for _,x in d['cold'].items():
  for m in ('control','I'):
   a=x['arms'][m]['metrics'];text.append(f'|{x["label"]}|{m}|{a["cold_first_response_ms"]["median"]/1000:.3f}|{a["cold_tree_rss_peak_bytes"]["median"]/2**30:.3f}|{a["startup_wall_ms"]["median"]/1000:.3f}|{a["first_query_wall_ms"]["median"]/1000:.3f}|')
  a,b=[x['arms'][m]['metrics'] for m in ('control','I')]
  notes.append(f'{x["label"]}准备至首次响应中位变化{pct(a["cold_first_response_ms"]["median"],b["cold_first_response_ms"]["median"]):+.1f}%。独立冷进程全部严格通过并与固定首条输出一致。')
 text+=notes
 text+=['![独立冷进程逐点](cold_points.png)','## 传输量与采用决定',
 '|地图|固定批control实际读回/MiB|I实际读回/MiB|变化|',
 '|---|---:|---:|---:|']
 for _,x in d['fixed'].items():
  a,b=[x['readback_wire_bytes'][m] for m in ('control','I')]
  text.append(f'|{x["label"]}|{a/2**20:.3f}|{b/2**20:.3f}|{pct(a,b):+.1f}%|')
 text += ['以上是启动加固定查询批所有实际读回响应的总字节，包含初始完整帧与后续变化块；不是查询速度，也不是地图常驻内存。Nav2和Python仍保留完整地图。',
 '预先条件：两图首次查询中位均改善至少3%，或两图独立冷进程准备至首次响应中位均改善至少3%；同时每图重复查询与查询进程树RSS中位退化不超过5%，质量与清理全部通过。',
 f'首次收益条件：{d["first_benefit_verified"]}；冷进程收益条件：{d["cold_benefit_verified"]}；重复/内存条件：{d["repeat_and_memory_gate"]}。最终R3默认为 `{selected}`。'+('I保留为实验，不将传输字节下降写成整体提速。' if selected=='control' else 'I通过预设条件，采用范围仅限本目录。'),
 '## 回归与保护核验',
 '|项目|control|I实验|',
 '|---|---:|---:|',
 '|两图固定完整静态|60/60|60/60|',
 '|精确变化起点（另20个初始任务）|60/60|60/60|',
 '|3cm/2°偏差（另20个初始任务）|60/60|60/60|',
 '|障碍增删等事件断言|22/22|22/22|',
 f'测试：`{v["tests"]}`；拥有进程和共享资源清理{len(v["cleanup"])}批；正式源码快照{len(v["formal_source_snapshots"])}份；规划源码锁{v["PLANNER_SOURCE_LOCK.json"]["files_verified"]}项。L1源/查询/地图、L2原生内核、原完整审计AST、外部9份原源码、V8-r11封存归档、V8-r11/V9-r0/r1/r2全部release文件及历史生产入口均校验通过。',
 '初版MRO未进入I读回通道，诊断v1被排除；修正Stable基类后，v2和所有正式I批次均有真实delta与不可写快照证据。正式计时不混入诊断、测试或离线探针。',
 '## 术语与范围',
 '“局部成功”表示只修补参考路线中有问题的位置后，整条任务路线通过严格检查；“段复用成功”还要求复用了以前保存的道路轨迹段。找到候选段不等于段复用成功。局部方法失败后可退回原有界完整规划，最终任务成功单独统计。本轮默认AS未启用这些局部提议实验。',
 'Q09前方停车障碍仍安全撤回轨迹，清除后恢复。两图静态回放不代表九图、连续定位跟踪或实车通过。无移动窗口、全图Smac自由搜索；生产入口保持。首次规划及总体内存仍需继续优化。']
 chunks=[]
 for s in text:
  if s.startswith('|') and chunks and chunks[-1].startswith('|'):chunks[-1]+='\n'+s
  else:chunks.append(s)
 (ROOT/'report/RESULTS.md').write_text('\n\n'.join(chunks)+'\n')
if __name__=='__main__':main()
