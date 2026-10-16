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
colors={'r1':'#8c9aad','r2':'#248473'};boxdata=[]
for metric,title,unit,logscale in [('wall_s','共同严格成功请求耗时','秒',True),('tree_rss_mib','共同严格成功请求的规划进程树 RSS 采样峰值','MiB',False)]:
 fig,ax=plt.subplots(figsize=(12,8));labels=[];positions=[]
 for i,m in enumerate(d['maps']):
  group=[r for r in raw if r['map_id']==m['map_id'] and r['r1_success']=='True' and r['r2_success']=='True'];y=len(d['maps'])-i;positions.append(y);labels.append(f"{m['name']}（每版 n={len(group)}）")
  for arm,dy in [('r1',.17),('r2',-.17)]:
   v=np.array([float(r[arm+'_'+metric]) for r in group]);q1,med,q3=np.quantile(v,[.25,.5,.75]);iqr=q3-q1;lo=v[v>=q1-1.5*iqr].min();hi=v[v<=q3+1.5*iqr].max();boxdata.append({'metric':metric,'map_id':m['map_id'],'arm':arm,'n':len(v),'unit':unit,'q1':q1,'median':med,'q3':q3,'whisker_low':lo,'whisker_high':hi,'outliers':v[(v<lo)|(v>hi)].tolist(),'raw':v.tolist()})
   b=ax.boxplot([v],positions=[y+dy],widths=.27,vert=False,patch_artist=True,whis=1.5,showfliers=True,manage_ticks=False,medianprops={'color':'black'},flierprops={'marker':'o','markersize':3});b['boxes'][0].set_facecolor(colors[arm])
 ax.set_yticks(positions);ax.set_yticklabels(labels,fontproperties=fp);ax.set_xlabel(unit+('（对数坐标）' if logscale else '（线性坐标）'),fontproperties=fp);ax.set_title('3YDL3-V10-r1 与 r2：'+title,fontproperties=fp)
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
   dy=(j-1)*.12;v=[float(row[a+'_'+metric]) for a in ('r1','r2')];ax.plot(v,[y+dy,y+dy],color='#cbd2d8',linewidth=1,zorder=1)
   for arm,x in zip(('r1','r2'),v):ax.scatter(x,y+dy,s=25,color=colors[arm],zorder=2)
 ax.set_yticks(positions);ax.set_yticklabels(labels,fontproperties=fp);ax.set_xlabel(unit+'（线性坐标）',fontproperties=fp);ax.set_title('3YDL3-V10-r1 与 r2：'+title,fontproperties=fp);ax.legend(handles=[Patch(color=colors[a],label='V10-'+a) for a in colors],prop=fp,loc='lower right');ax.grid(axis='x',alpha=.2)
 for t in [*ax.get_xticklabels(),*ax.get_yticklabels()]:t.set_fontproperties(fp)
 fig.text(.5,.02,'每点为一次独立进程的首条固定 Q；灰线连接同轮配对值。每图每版仅 3 个样本，采用逐点图，不估造分布。',ha='center',fontproperties=fp,fontsize=10);fig.tight_layout(rect=(0,.06,1,1));fig.savefig(R/(metric+'_paired_points.png'),dpi=180);plt.close(fig)
def change(group,key,unit):
 z=group[key];return f"{z['r1_median']:.3f} → {z['r2_median']:.3f} {unit}（{z['median_change_pct']:+.2f}%）"
big=next(m for m in d['maps'] if m['map_id']=='3yd_synth_trizone_c_v2')
lines=['# 3YDL3-V10-r3：减少从零加载至首条响应的等待','',
f"相对同批 V10-r1，地图等权的首条响应中位比几何平均变化为 **{d['cold_map_equal_weight']['first_response_s']:+.2f}%**。合成地库 C 首条响应中位为 **{change(big['cold'],'first_response_s','秒')}**。这是地图加载、启动、首请求全部计时的结果，不使用预热任务结果或持久缓存。",'',
f"十张地图各自固定 Q × 3 轮，r1 **{d['r1_strict']}/300**，r2 **{d['r2_strict']}/300**；新增严格成功 **{len(d['new_successes'])}** 次，原成功退化 **{len(d['regressions'])}** 次。原有失败逐组核对，L1 输出存在不等于三层严格成功。",'',
'| 指标及样本 | r1 → r2 |','|---|---:|',
f"| 首条响应中位（每版 30 个独立进程） | {change(d['cold_common'],'first_response_s','秒')} |",
f"| 地图准备阶段进程树 RSS 采样峰值中位（每版 30 次） | {change(d['cold_common'],'preparation_tree_rss_mib','MiB')} |",
f"| 加载至首条响应进程树 RSS 采样峰值中位（每版 30 次） | {change(d['cold_common'],'cold_tree_rss_mib','MiB')} |",
f"| 共同严格成功请求耗时中位（每版 {d['common_success_n']} 次） | {change(d['common_success'],'wall_s','秒')} |",
f"| 共同严格成功请求进程树 RSS 采样峰值中位 | {change(d['common_success'],'tree_rss_mib','MiB')} |",'',
f"共同成功请求 P95 为 {d['common_success']['wall_s']['r1_p95']:.3f} → {d['common_success']['wall_s']['r2_p95']:.3f} 秒；进程树 CPU 时间中位为 {change(d['common_success'],'tree_cpu_s','秒')}。准备阶段、请求阶段和整个首条期间的峰值分别统计，不能用准备阶段节约替代整体资源改善。",'',
'## 采用的方法','',
'1. **小区域按精确像素写入。** 原构造器对每个小语义区域扫描整图布尔掩码。现在使用同样的坐标取整、边界裁剪和 OpenCV 填充，提取小包围框内的像素索引，按原顺序写入标签、硬障碍和拓扑区域。大包围框仍使用 r1 的稠密掩码，限制临时索引内存。冻结拓扑源码未改，构造结束或异常时恢复所有临时适配。',
'2. **直接计算等价安全区域。** 原表达式只使用“EDT 距离乘分辨率是否达到阈值”的真假结果。现在用同一整数格点距离构造精确形态学判定，避免生成全图浮点距离场。临界阈值、全自由特殊情形或无法识别的构造器均回退原计算。没有为路径求解提供近似距离，也未放宽安全半径。',
'3. **节点就绪后进入实际 ACK。** 保留进程存活、组件创建和生命周期激活检查，移除之后固定的 1 秒等待，将就绪轮询间隔从 200 ms 改为 20 ms。后续真实地图订阅者等待、启动 costmap 验证及每次完整有效地图 ACK 继续执行；没有用“进程启动”代替内容就绪。','',
'## 逐图首条响应','',
'每图每版三个独立进程，每进程按本图原始固定 Q 运行十次请求。第二轮地图倒序，各地图/轮次交替版本顺序，CPU 14 串行；未与性能剖析或其他本任务重负载并行。','',
'| 地图 | 严格成功 r1→r2 | 首条响应中位（秒） | 首条响应变化 | 请求耗时中位变化 | 请求整树 RSS 变化 |','|---|---:|---:|---:|---:|---:|']
for m in d['maps']:
 z=m['cold']['first_response_s'];lines.append(f"| {m['name']} | {m['r1_success']}/30 → {m['r2_success']}/30 | {z['r1_median']:.2f} → {z['r2_median']:.2f} | {z['median_change_pct']:+.2f}% | {m['common_success']['wall_s']['median_change_pct']:+.2f}% | {m['common_success']['tree_rss_mib']['median_change_pct']:+.2f}% |")
lines+=['',f'![首条响应配对点图]({R/"first_response_s_paired_points.png"})','',f'![首条期间进程树内存配对点图]({R/"cold_tree_rss_mib_paired_points.png"})','',f'![逐次请求耗时箱线图]({R/"wall_s_boxplots.png"})','',f'![逐次请求进程树内存箱线图]({R/"tree_rss_mib_boxplots.png"})','',
'图中文字使用实际宋体 SimSun。请求箱线图使用逐次原始记录，箱体为 Q1–Q3，中线为中位数，须为 1.5×IQR 内最远原始值，离群点保留；耗时图使用对数坐标。首条每图每版仅 n=3，使用配对点图，不估造分布。重复固定 Q 不是独立新场景。','',
'## 剩余等待花在哪里','',
'| 地图 | 地图准备中位 r1→r2（秒） | 服务启动中位 r1→r2（秒） | 首请求本身中位 r1→r2（秒） |','|---|---:|---:|---:|']
for m in d['maps']:
 row=[m['name']]
 for key in ('preparation_s','startup_s','first_query_s'):
  z=m['cold'][key];row.append(f"{z['r1_median']:.2f} → {z['r2_median']:.2f}")
 lines.append('| '+' | '.join(row)+' |')
lines+=['',
f"合成 C 的 r2 地图准备中位为 **{big['cold']['preparation_s']['r2_median']:.2f} 秒**，首请求本身仍为 **{big['cold']['first_query_s']['r2_median']:.2f} 秒**。地图准备已明显减少，后续应继续分解 L2 场/状态构造、首次搜索和全任务 costmap 传输与核验，不能仅靠减少服务启动等待达到低延迟。",'',
'各分项分别取中位数，且还存在审计器、服务对象构造、源码快照导出等小项，因此表内中位数之和不要求等于首条响应中位数。',
'首条响应从地图加载开始计时，包含地图准备、审计/服务准备、源码快照导出、ROS 启动和首请求，排除 Python 模块导入及案例结果导出。每批使用新进程、新缓存目录；操作系统文件缓存未清空，不能解释为系统重启后的首次磁盘冷读，也不等于用户点击启动的完整应用延迟。',
'RSS 为整套规划进程树每 20 ms 采样的峰值，可能漏掉更短峰值；PSS 是请求返回后快照，另存 COMPARISON.json，不把独立 Python RSS 或 PSS 快照当作整树峰值。','',
'## 正确性、失败与证据','',
f"五张语义地图共 {review['preparation_equivalence_cells']:,} 个单元的标签、硬障碍、安全区域，以及多分辨率视图和拓扑区域/接触/邻接/静态转移等数据，与 r1 构造结果一致。正式十五个语义地图配对批次再次核对原始数组相同。",'',
f"共同成功的 {review['common_success_L1_L2_soft_and_effective_costmap_equal']} 次请求中，L1、L2、软目标和有效 costmap 内容一致；最终路径 {review['common_success_path_equal']}/{d['common_success_n']} 次哈希相同。所有成功输出均通过原道路、车体、方向、曲率、连续证书及实际 ACK 检查。",'']
variations=json.loads((R/'OUTPUT_VARIATIONS.json').read_text())
if variations:
 lines+=['最终轨迹差异保留在 OUTPUT_VARIATIONS.json。原优化器及其候选认证小预算没有修改；不宣称全部路径逐点相同。','',
 '| 轮次/地图/查询 | 长度变化 | 曲率上界 r1→r2（m⁻¹） | 候选日志 |','|---|---:|---:|---|']
 for v in variations:
  a=v['r1_quadratic'] or {};b=v['r2_quadratic'] or {};lines.append(f"| {v['round']}/{v['map_id']}/{v['query_id']} | {v['length_change_pct']:+.5f}% | {v['r1_curvature_bound']:.5f} → {v['r2_curvature_bound']:.5f} | r1 selected={a.get('selected')} {a.get('certificate_failure','')}; r2 selected={b.get('selected')} {b.get('certificate_failure','')} |")
 lines+=['']
remaining='；'.join(m['name']+' '+ '/'.join(m['remaining']) for m in d['maps'] if m['remaining']) or '无'
lines+=[f'剩余固定失败：{remaining}。没有把诊断修正或失败退出写成新增导航成功。','',
f"项目测试 **749 项通过**，保留一个继承的 NumPy 警告。三次独立快速启动诊断均严格成功并完成清理。正式 **{review['formal_clean_batches']} 批**及两图事件进程全部退出码 0、清理完成；额外 **{events['passed_events']} 项**受控事件断言通过，其中万达3w Q09 前方障碍仍是预期失败撤回和清除恢复，不代表成功绕行。",'',
f"父版本 {review['parent_registered_files_unchanged']:,} 个登记文件校验未变，冻结 L1、地图/Q、曲线配置、原生库和原严格审计保持。r2 仍是静态候选，历史生产入口未替换，不推广为十图动态、连续控制或实车验收。",'',
f"本轮工程观测门槛：保留原成功；共同成功请求耗时中位回退≤5%、整树 RSS 中位回退≤2%；地图等权首条响应改善≥10%。结果：{'全部通过' if all(d['gates'].values()) else '有未通过项，详见 COMPARISON.json'}。这不是各图同幅改善或未见地图保证。",'',
'正式统计仅使用 SELECTED_DATA.json 的 results/paired_final。地图等价诊断、早期 pilot_c、三次 ready_probe 和事件回放不混入正式性能；不替换任何路径差异或失败批次。逐次请求与首条原始指标分别在 raw_paired_metrics.csv 和 raw_cold_metrics.csv，箱线分位数和原始值在 BOXPLOT_DATA.json。']
(R/'RESULTS.md').write_text('\n'.join(lines)+'\n');print('REPORT_AND_PLOTS_WRITTEN')
