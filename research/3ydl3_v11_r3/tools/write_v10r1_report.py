from pathlib import Path
import json,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import fontManager,FontProperties
from matplotlib.patches import Patch
from matplotlib.ticker import FuncFormatter,NullFormatter
E=Path(__file__).resolve().parents[1];R=E/'report';d=json.loads((R/'COMPARISON.json').read_text());review=json.loads((R/'SOURCE_REVIEW.json').read_text());events=json.loads((R/'EVENTS.json').read_text());raw=list(csv.DictReader((R/'raw_paired_metrics.csv').open()));cold=list(csv.DictReader((R/'raw_cold_metrics.csv').open()))
font=Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc');assert font.exists();fontManager.addfont(str(font));fp=FontProperties(fname=str(font));plt.rcParams.update({'font.family':fp.get_name(),'axes.unicode_minus':False})
colors={'r0':'#8c9aad','r1':'#248473'};boxdata=[]
for metric,title,unit,logscale in [('wall_s','共同严格成功请求耗时','秒',True),('tree_rss_mib','共同严格成功请求的规划进程树 RSS 采样峰值','MiB',False)]:
 fig,ax=plt.subplots(figsize=(12,8));labels=[];positions=[]
 for i,m in enumerate(d['maps']):
  group=[r for r in raw if r['map_id']==m['map_id'] and r['r0_success']=='True' and r['r1_success']=='True'];y=len(d['maps'])-i;positions.append(y);labels.append(f"{m['name']}（每版 n={len(group)}）")
  for arm,dy in [('r0',.17),('r1',-.17)]:
   v=np.array([float(r[arm+'_'+metric]) for r in group]);q1,med,q3=np.quantile(v,[.25,.5,.75]);iqr=q3-q1;lo=v[v>=q1-1.5*iqr].min();hi=v[v<=q3+1.5*iqr].max();boxdata.append({'metric':metric,'map_id':m['map_id'],'arm':arm,'n':len(v),'unit':unit,'q1':q1,'median':med,'q3':q3,'whisker_low':lo,'whisker_high':hi,'outliers':v[(v<lo)|(v>hi)].tolist(),'raw':v.tolist()})
   b=ax.boxplot([v],positions=[y+dy],widths=.27,vert=False,patch_artist=True,whis=1.5,showfliers=True,manage_ticks=False,medianprops={'color':'black'},flierprops={'marker':'o','markersize':3});b['boxes'][0].set_facecolor(colors[arm])
 ax.set_yticks(positions);ax.set_yticklabels(labels,fontproperties=fp);ax.set_xlabel(unit+('（对数坐标）' if logscale else '（线性坐标）'),fontproperties=fp);ax.set_title('3YDL3-V10-r0 与 r1：'+title,fontproperties=fp)
 if logscale:ax.set_xscale('log');ax.xaxis.set_major_formatter(FuncFormatter(lambda v,p:f'{v:g}'));ax.xaxis.set_minor_formatter(NullFormatter())
 for t in [*ax.get_xticklabels(),*ax.get_yticklabels()]:t.set_fontproperties(fp)
 ax.legend(handles=[Patch(color=colors[a],label='V10-'+a) for a in colors],prop=fp,loc='lower right');ax.grid(axis='x',alpha=.2)
 fig.text(.5,.02,'箱体 Q1–Q3；中线为中位数；须为 1.5×IQR 内最远原始值；圆点为离群点。固定 Q × 3 轮，重复样本相关。',ha='center',fontproperties=fp,fontsize=10);fig.tight_layout(rect=(0,.06,1,1));fig.savefig(R/(metric+'_boxplots.png'),dpi=180);plt.close(fig)
(R/'BOXPLOT_DATA.json').write_text(json.dumps(boxdata,ensure_ascii=False,indent=2))
for metric,title,unit in [('first_response_s','加载至首条响应','秒'),('cold_tree_rss_mib','加载至首条响应的规划进程树 RSS 采样峰值','MiB')]:
 fig,ax=plt.subplots(figsize=(12,8));positions=[];labels=[]
 for i,m in enumerate(d['maps']):
  y=len(d['maps'])-i;positions.append(y);labels.append(m['name']+'（每版 n=3）');group=[r for r in cold if r['map_id']==m['map_id']]
  for j,row in enumerate(group):
   dy=(j-1)*.12;v=[float(row[a+'_'+metric]) for a in ('r0','r1')];ax.plot(v,[y+dy,y+dy],color='#cbd2d8',linewidth=1,zorder=1)
   for arm,x in zip(('r0','r1'),v):ax.scatter(x,y+dy,s=25,color=colors[arm],zorder=2)
 ax.set_yticks(positions);ax.set_yticklabels(labels,fontproperties=fp);ax.set_xlabel(unit+'（线性坐标）',fontproperties=fp);ax.set_title('3YDL3-V10-r0 与 r1：'+title,fontproperties=fp);ax.legend(handles=[Patch(color=colors[a],label='V10-'+a) for a in colors],prop=fp,loc='lower right');ax.grid(axis='x',alpha=.2)
 for t in [*ax.get_xticklabels(),*ax.get_yticklabels()]:t.set_fontproperties(fp)
 fig.text(.5,.02,'每点为一次独立进程的首条固定 Q；灰线连接同轮配对值。每图每版仅 3 个样本，采用逐点图，不估造分布。',ha='center',fontproperties=fp,fontsize=10);fig.tight_layout(rect=(0,.06,1,1));fig.savefig(R/(metric+'_paired_points.png'),dpi=180);plt.close(fig)
def vals(group,key,unit):
 z=group[key];return f"{z['r0_median']:.3f} → {z['r1_median']:.3f} {unit}（{z['median_change_pct']:+.2f}%）"
lines=['# 3YDL3-V10-r1：首次准备优化与失败定位修正','',f"十图各自固定 Q × 3 轮，同批 V10-r0 **{d['r0_strict']}/300**、V10-r1 **{d['r1_strict']}/300**。新增严格成功 **{len(d['new_successes'])}** 次、原成功退化 **{len(d['regressions'])}** 次。本轮改善首次准备成本，剩余六组失败尚未解决。",'',
'| 指标与样本 | r0 → r1 |','|---|---:|',f"| 加载至首条响应中位（每版 30 次独立进程） | {vals(d['cold_common'],'first_response_s','秒')} |",f"| 加载至首条响应整树 RSS 采样峰值中位（每版 30 次） | {vals(d['cold_common'],'cold_tree_rss_mib','MiB')} |",f"| 共同成功请求耗时中位（每版 {d['common_success_n']} 次） | {vals(d['common_success'],'wall_s','秒')} |",f"| 共同成功请求整树 RSS 采样峰值中位 | {vals(d['common_success'],'tree_rss_mib','MiB')} |",'',
f"按地图等权的首条响应中位比几何平均变化 **{d['cold_map_equal_weight']['first_response_s']:+.2f}%**。本轮观测门槛：保留全部原成功查询；请求耗时中位回退不超过 5%、RSS 中位回退不超过 2%；地图等权首条响应改善至少 5%。判定为：{'全部通过' if all(d['gates'].values()) else '存在未通过项，详见 COMPARISON.json'}。这些是本固定地图集上的描述统计，不是未见地图保证。",'',
 '## 本轮采用的改动','',
'1. **无拷贝二值栅格存储。** 原栅格化只产生 0/1 字节，加载阶段改为同一缓冲区的 bool 视图，消除每个语义区域后的整图类型转换。像素坐标、OpenCV 填充方式、区域覆盖优先级完全保持。适配仅在串行构造期间生效，结束或异常时恢复原函数；上游代码身份不符时使用原实现。冻结 L1 源码未改，实际加载结果另做逐格核对。',
'2. **直接构造等价的审计判定表。** 启用已有 D 初始化选项，依据同一网格距离阈值生成审计的近障碍判定，省去全图浮点距离场临时构造。几何阈值落在数值歧义边界或原构造函数发生变化时自动回到原构造。原审计和精确车体检查仍执行。',
'3. **修正 L2 断点诊断。** 连通性分析现在使用实际规划范围；原实现可能把范围外绕行算入连通性，错误报告“没有第二连通分量”。这只修正诊断，不改变 L2 搜索、参考线或接受标准。该定位采用无向位置连通性，不代替方向约束、L2 边可通行性或 L3 连续运动证明。','',
'## 逐图结果','',
'每图每版 30 次静态请求；首条响应只统计三个独立进程各自的第一组 Q，每版 n=3。第二轮地图倒序，轮次/地图交替版本顺序，CPU 14 串行。所有地图均读取本图原始固定查询，未重新抽样。','',
'| 地图 | 严格成功 r0→r1 | 首条响应中位（秒） | 首条响应变化 | 请求中位变化 | 请求整树 RSS 变化 |','|---|---:|---:|---:|---:|---:|']
z=d['common_success']['wall_s'];cpu=d['common_success']['tree_cpu_s'];i=lines.index('## 本轮采用的改动')
lines[i:i]=[f"共同成功请求的 P95 为 **{z['r0_p95']:.3f} → {z['r1_p95']:.3f} 秒**（{(z['r1_p95']/z['r0_p95']-1)*100:+.2f}%）；规划进程树 CPU 时间中位为 **{cpu['r0_median']:.3f} → {cpu['r1_median']:.3f} 秒**（{cpu['median_change_pct']:+.2f}%）。请求阶段资源与首次加载资源分开比较。",'']
for m in d['maps']:
 z=m['cold']['first_response_s'];lines.append(f"| {m['name']} | {m['r0_success']}/30 → {m['r1_success']}/30 | {z['r0_median']:.2f} → {z['r1_median']:.2f} | {z['median_change_pct']:+.2f}% | {m['common_success']['wall_s']['median_change_pct']:+.2f}% | {m['common_success']['tree_rss_mib']['median_change_pct']:+.2f}% |")
lines+=['',f'![加载至首条响应逐点图]({R/"first_response_s_paired_points.png"})','',f'![冷首条进程树内存逐点图]({R/"cold_tree_rss_mib_paired_points.png"})','',f'![逐次请求耗时箱线图]({R/"wall_s_boxplots.png"})','',f'![逐次请求进程树内存箱线图]({R/"tree_rss_mib_boxplots.png"})','',
'图中文字均加载实际宋体 SimSun。请求指标箱线图来自逐次原始记录，箱体为 Q1–Q3，中线为中位数，须为 1.5×IQR 内最远数据，离群点保留；耗时使用对数坐标。首条响应每组仅三次，用配对逐点图，未伪造分布。','',
'## 首次准备分项','',
'| 地图 | 地图/拓扑/视图准备中位 r0→r1（秒） | 审计器准备中位 r0→r1（秒） |','|---|---:|---:|']
for m in d['maps']:
 a=m['cold']['preparation_s'];b=m['cold']['auditor_s'];lines.append(f"| {m['name']} | {a['r0_median']:.2f} → {a['r1_median']:.2f} | {b['r0_median']:.2f} → {b['r1_median']:.2f} |")
lines+=['','“加载至首条响应”从地图加载开始，包含审计/服务准备、源码快照导出、ROS 节点启动和首个请求，排除 Python 模块导入及首个案例结果导出。它不等于纯算法请求耗时，也不等于完整应用从点击启动的总延迟。两版均开启相同的冷进程内存采样；每批使用新进程和新缓存目录，但没有清空操作系统文件缓存，因此不代表系统重启后的首次磁盘冷读。',
'RSS 为规划进程树每 20 ms 采样的峰值，可能漏掉更短的峰值；不是单一 Python 进程内存。PSS 在返回后取快照并另存于 COMPARISON.json，不当作峰值。首条 Q 的重复样本与整组重复样本均有关联，不将 300 次请求称为 300 个独立场景。','',
'## 失败诊断与未采用方案','',
'- Arena 中图 A2B-04、导师原图 A2B-07：目标姿态本身无碰撞，但指定朝向的局部驶入困难。更细反向候选搜索在原局部范围内未找到可行接入，未接入正式规划。搜索失败不构成物理不可行证明。',
'- 导师原图 A2B-03：在实际 L2 范围内，矩形姿态认证后的起点/终点不连通。扩大诊断认证带也未恢复连通性；不能通过取消车体检查或修改地图/Q 解决。范围外连通性造成的误导性诊断已修正。',
'- 万达2w地库 Q08/Q09、合成地库 C Q10：仍存在有序走廊、车体/曲率约束下的求解失败或预算耗尽。本轮没有把失败缩短、诊断改正或 L1 产物存在写成三层成功。','',
f'![导师原图 L2 断连诊断]({E/"results/bridge_geometry/gap.png"})', '', '图中红点为起点所属 L2 分量，蓝点为其他可用 L2 单元，绿线为 L1 引导；这是静态位置诊断，不是可执行轨迹。', '', '后续应针对真实断连位置构造局部接入证据，再处理狭窄终点的连续曲率候选；保留当前首条准备优化作为对照。', '',
'## 验证与数据口径','',
f"项目检查 **738 项通过**，保留一个继承的 NumPy 警告。最初未限定目录的测试发现额外运行了依赖包的可选 MKL 测试，因环境缺少 libmkl_rt.so 失败；该原始日志和范围说明保留在 TEST_SCOPE.json，未将其计入项目通过数。正式后端为 Clarabel。",'',
f"十图共 **{review['audit_gate_equivalence_cells']:,} 个栅格**的原审计近障碍判定逐格一致。所有正式语义地图批次的标签、障碍、安全区域一致；共同成功的 {review['common_success_L1_L2_soft_output_hashes_equal']} 次请求中，L1、L2、软目标文件哈希均一致，有效 costmap 内容及原 ACK 审计保持。最终轨迹有 {review['common_success_final_path_hashes_equal']}/282 次哈希一致。",'',
'唯一轨迹差异是第一轮导师四倍面积图 A2B-05：r0 的二次候选认证耗时 55.46 ms，触及继承的 50 ms 小预算并转入原 SQP 回退；r1 在 24.50 ms 内通过候选。其余两轮的 r0/r1 均与这条 r1 路径哈希相同，说明同版也可能受该时间预算影响。该配对路径长度为 265.22886 → 265.20722 m（−0.00816%），连续曲率上界为 1.88387 → 2.28796 m⁻¹，均低于 2.5 m⁻¹ 限制并通过原严格审计。原始记录完整保留，不宣称全部路径逐点不变，也不把该分支差异当作新增算法收益；详细证据见 OUTPUT_VARIATION.json。','',
f"父版本 {review['parent_registered_files_unchanged']:,} 个登记文件未变；冻结 L1 源文件、地图与固定 Q、原生库、曲线配置、连续证书和原严格审计保持。正式 60 批均退出码 0、拥有进程及共享资源清理完成。",'',
f"额外两图受控障碍事件断言 **{events['passed_events']} 项通过**；其中万达3w Q09 的前方障碍仍是预期失败撤回、清除后恢复，不是成功绕行。只代表本批受控事件，不能推广为十图动态、连续控制或实车验收。",'',
'正式统计仅使用 SELECTED_DATA.json 指定的 results/paired_final。剖析、端点搜索、L2 诊断、审计等价验证及事件回放均不混入性能统计。原始请求在 raw_paired_metrics.csv，冷首条数据在 raw_cold_metrics.csv，箱线图数据在 BOXPLOT_DATA.json。',
'本版为独立静态候选，历史生产入口未替换，V10-r0 代码与正式记录保留。']
(R/'RESULTS.md').write_text('\n'.join(lines)+'\n')
print('REPORT_AND_PLOTS_WRITTEN')
