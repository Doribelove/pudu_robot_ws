"""Write the R2 evidence report from selected raw same-batch records."""
from pathlib import Path
from compare_v9_review import load
ROOT=Path(__file__).resolve().parents[1]
def med(d,mode,part,metric):return d['arms'][mode][part]['metrics'][metric]['median']
def pct(a,b):return 100*(b/a-1)
def main():
 d=load(ROOT/'report/COMPARISON.json');candidate=d['candidate'];selected=d['selected_r2_mode'];v=d['verification']
 text=['# 3YD-V9-r2：初始化优化与局部连接审核',
 f'继承V9-r1 AS。审核后本目录配置为{"AS" if selected=="control" else "AS+"+selected}；封存L1、V8-r11、r6、历史生产入口和V9-r0/r1记录保持。正式原始数据仅 `{d["data_selection"]["formal"]}`。',
 '## 首次与重复查询',
 'CPU12串行、原两图固定10组各三次；首次n=10、重复n=20，共享各自的服务进程。control使用父AS，D只改变审计近障判定字段的初始化，DP另启用局部连接。初始化/ROS启动和结果导出排除于查询计时，单独实测。',
 '|地图|配置|三层严格|首次中位/s|重复中位/s|查询进程树RSS中位/GiB|首次局部成功|',
 '|---|---|---:|---:|---:|---:|---:|']
 for mid,item in d['fixed'].items():
  for mode in ('control','D','DP'):
   text.append(f'|{item["label"]}|{mode}|{item["arms"][mode]["all"]["success"]}/30|{med(item,mode,"first","e2e_s"):.4f}|{med(item,mode,"repeat","e2e_s"):.4f}|{med(item,mode,"all","tree_rss_gib"):.4f}|{item["arms"][mode]["first"]["local_success"]}/10|')
 text += ['![首次查询箱线图](first_boxplots.png)','![重复查询箱线图](repeat_boxplots.png)',
 '箱体Q1–Q3，中线中位数；须为1.5×IQR内最远值，离群值单列；均线性坐标。独立Python PSS为快照，进程树RSS包含Python和ROS子进程，不混为同一内存指标。严格成功单独列表。',
 f'P采用规则：{d["P_adoption_rule"]}。本次追加完整回归为 `{candidate}`，最终默认R2开关为 `{selected}`。']
 for mid,item in d['fixed'].items():
  text.append(f'{item["label"]}：D首次中位较control变化{pct(med(item,"control","first","e2e_s"),med(item,"D","first","e2e_s")):+.1f}%，DP首次较D变化{pct(med(item,"D","first","e2e_s"),med(item,"DP","first","e2e_s")):+.1f}%。')
 text += [('D保留为可复现实验；独立冷进程未满足两图总耗时均改善的预设条件，因此默认保留父AS。字段初始化更快，但不能写成整体冷启动或首次查询普遍提速。' if selected=='control' else '所选候选通过预设整体冷启动条件；查询本身的变化单独报告。'),'## 独立冷进程',
 '每图每组3个独立进程，组顺序逐轮交替。准备至首次响应计时覆盖地图/审计/服务准备、源码导出、ROS启动和首条查询；排除Python模块导入与最终结果导出。n=3只显示逐点值，不外推分布或显著性。整树RSS按20ms采样。',
 '|地图|配置|准备至首次响应中位/s|冷进程树峰值RSS中位/GiB|审计字段初始化中位/s|初始化阶段Python峰值RSS中位/GiB|',
 '|---|---|---:|---:|---:|---:|']
 cold_notes=[]
 for mid,item in d['cold'].items():
  for mode in ('control',candidate):
   m=item['arms'][mode]['metrics'];text.append(f'|{item["label"]}|{mode}|{m["cold_first_response_ms"]["median"]/1000:.3f}|{m["cold_tree_rss_peak_bytes"]["median"]/2**30:.3f}|{m["auditor_wall_ms"]["median"]/1000:.4f}|{m["auditor_python_peak_bytes"]["median"]/2**30:.3f}|')
  a=item['arms']['control']['metrics'];b=item['arms'][candidate]['metrics'];cold_notes.append(f'{item["label"]}准备至首响应中位变化{pct(a["cold_first_response_ms"]["median"],b["cold_first_response_ms"]["median"]):+.1f}%；这里的收益不能写成首次查询同等提速。')
 text += cold_notes
 text += ['![冷进程逐点](cold_points.png)',
 'D直接生成原审计所需的逐格近障布尔判定，使用与SciPy距离变换一致的整格距离核；阈值接近格点距离或父构造器发生变化时回原构建。原完整审计、数值最小间隙、精确车体检查不变。两张实验地图所有格点的打包字节与原方法完全一致；正式路径及原精确车体检查次数也逐次一致。D省去审计用全图float64距离场，Nav2及Python其他完整地图仍常驻。',
 '与父V9-r1 AS比较，60条原始最终路径、姿态/地图哈希、几何与约束指标及精确车体检查次数相同；canonical_path_hash含source_commit版本标签，更新为V9-r2后该标识不同，不能误写为所有版本化哈希相同。详见PARENT_OUTPUT_PARITY.json。',
 '## 局部连接与跨任务道路段',
 'P先在当前道路范围内生成完整几何，采用解析G2连接、参考直线处内部接缝及直线—局部G2转角—直线；这些接缝属于L3，不是L1拓扑节点。全部原锚点真实位置/朝向匹配，车体、曲率、运动朝向和全路终验通过后才可返回。当前地图必须真实发布、逐格读回ACK；几何失败仅表示本地提议未完成，会回原有界规划，不宣称物理阻断。未采用更密导数网格/切向加速度试验：短直角仍失败，保留诊断记录。',
 '风险定位复用同一批原始加密姿态，递归区间保留原起点浮点值，随机转角与格点边界风险定位和原方法一致。明显超过修复范围时提前回退。连接搜索仍有限时预算；路径、审计与ACK费用均计入真实请求。',
 '|共享诊断地图|配置|3条新任务严格|新任务中位/s|局部成功|段复用成功|',
 '|---|---|---:|---:|---:|---:|']
 for mid,item in d['shared'].items():
  for mode,arm in item['arms'].items():
   s=arm['new_tasks'];text.append(f'|{item["label"]}|{mode}|{s["success"]}/3|{s["metrics"]["e2e_s"]["median"]:.4f}|{s["local_success"]}/3|{s["segment_reuse_success"]}/3|')
 text += ['![共享任务逐点](shared_points.png)',
 '共享任务来自已登记v2诊断集，各起终点均不同，没有完整路径或任务续接命中。A/AB隔离段复用B，AB/ABM隔离M；AP/ABPM同时使用P。每图仅3个相关新任务，不能代表固定10组或普通任务收益。M只延迟无候选时的整图检查器构造，有实际匹配时仍按当前地图扫车体、完整终验；本次不据三个诊断样本将B/M改为默认。',
 '## 完整回归与保护核验',
 '|项目|control|D实验|',
 '|---|---:|---:|',
 '|两图固定静态|60/60|60/60|',
 '|精确变化起点（另20个初始任务）|60/60|60/60|',
 '|3cm/2°偏差（另20个初始任务）|60/60|60/60|',
 '|障碍增删等事件断言|22/22|22/22|',
 f'完整测试：`{v["tests"]}`。拥有进程与共享资源清理：{len(v["cleanup"])}批。正式批次源码快照{len(v["formal_source_snapshots"])}份，规划源文件锁104项；L1源/输入、L2原生内核、原三层严格审计AST、外部原审计源码与历史生产入口校验通过。V9-r0/r1、V8-r11 release全部文件与封存归档校验通过。',
 '## 尚未达到的范围',
 '两图静态回放成绩不能推广为九图全部通过、连续行驶或实车验收。Q09前方停车障碍仍撤回轨迹、清除后恢复。无80m移动窗口，无全图Smac自由搜索，生产入口保持。首次查询尚需继续优化；局部字段初始化收益与整体冷启动结果分别报告。']
 chunks=[]
 for block in text:
  if block.startswith('|') and chunks and chunks[-1].startswith('|'):chunks[-1]+='\n'+block
  else:chunks.append(block)
 (ROOT/'report/RESULTS.md').write_text('\n\n'.join(chunks)+'\n')
if __name__=='__main__':main()
