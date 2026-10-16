from pathlib import Path
import json,csv
import numpy as np
E=Path(__file__).resolve().parents[1];R=E/'report';d=json.loads((R/'COMPARISON.json').read_text());raw=list(csv.DictReader((R/'raw_paired_metrics.csv').open()))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties,fontManager
font=Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc');assert font.exists();fontManager.addfont(str(font));fp=FontProperties(fname=str(font));plt.rcParams['font.family']=fp.get_name();plt.rcParams['axes.unicode_minus']=False
boxdata=[]
for metric,title,unit,logscale in [('wall_s','完整请求耗时','秒',True),('tree_rss_mib','规划进程树 RSS 采样峰值','MiB',False)]:
 fig,ax=plt.subplots(figsize=(12,8));labels=[];positions=[]
 for i,m in enumerate(d['maps']):
  group=[r for r in raw if r['map_id']==m['map_id'] and r['v9_success']=='True' and r['v10_success']=='True']
  y=len(d['maps'])-i;labels.append(f"{m['name']}（每版 n={len(group)}）");positions.append(y)
  for arm,dy,color in [('v9',.17,'#8c9aad'),('v10',-.17,'#248473')]:
   values=np.array([float(r[arm+'_'+metric]) for r in group]);q1,q2,q3=np.quantile(values,[.25,.5,.75]);iqr=q3-q1;low=values[values>=q1-1.5*iqr].min();high=values[values<=q3+1.5*iqr].max()
   boxdata.append({'metric':metric,'map_id':m['map_id'],'arm':arm,'n':len(values),'units':unit,'q1':q1,'median':q2,'q3':q3,'whisker_low':low,'whisker_high':high,'outliers':values[(values<low)|(values>high)].tolist(),'raw':values.tolist()})
   bp=ax.boxplot([values],positions=[y+dy],widths=.27,vert=False,patch_artist=True,whis=1.5,showfliers=True,manage_ticks=False,medianprops={'color':'black'},flierprops={'marker':'o','markersize':3})
   bp['boxes'][0].set_facecolor(color)
 ax.set_yticks(positions);ax.set_yticklabels(labels,fontproperties=fp);ax.set_xlabel(unit+('（对数坐标）' if logscale else '（线性坐标）'),fontproperties=fp);ax.set_title('3YDL3-V9 与 V10-r0：共同严格成功样本的'+title,fontproperties=fp)
 if logscale:
  ax.set_xscale('log')
  from matplotlib.ticker import FuncFormatter
  ax.xaxis.set_major_formatter(FuncFormatter(lambda value,position:f'{value:g}'))
 from matplotlib.patches import Patch
 ax.legend(handles=[Patch(color='#8c9aad',label='3YDL3-V9'),Patch(color='#248473',label='V10-r0')],prop=fp,loc='lower right');ax.grid(axis='x',alpha=.2)
 for text_item in [*ax.get_xticklabels(), *ax.get_yticklabels()]:text_item.set_fontproperties(fp)
 fig.text(.5,.018,'箱体：Q1–Q3；中线：中位数；须：1.5×IQR 范围内最远原始值；圆点：离群点。每图固定 Q × 3 轮，重复样本相关。',ha='center',fontproperties=fp,fontsize=10)
 fig.tight_layout(rect=(0,.055,1,1));fig.savefig(R/(metric+'_boxplots.png'),dpi=180);plt.close(fig)
(R/'BOXPLOT_DATA.json').write_text(json.dumps(boxdata,ensure_ascii=False,indent=2))
c=d['common_success'];v=lambda k:f"{c[k]['v9_median']:.3f} → {c[k]['v10_median']:.3f}（{c[k]['median_change_pct']:+.2f}%）"
lines=['# 3YDL3-V10-r0：多地图性能与表现优化','',f"同批十图各自固定 Q、每版三轮：V9 **{d['v9_strict']}/300**，V10-r0 **{d['v10_strict']}/300**；原成功请求退化 **{len(d['regressions'])}** 次。共同成功样本 n={d['common_success_n']}。",'', '对照版本为 3YDL3-V9（独立 L3、多地图修复版），目录 /home/robot/workspaces/3ydl3_v9_crossmapfix_20261011；与历史 3YD-V9-r9 的命名区分。','',
'| 指标 | V9 → V10-r0 |','|---|---:|',f"| 共同成功请求耗时中位（s） | {v('wall_s')} |",f"| 共同成功整树 RSS 采样峰值中位（MiB） | {v('tree_rss_mib')} |",f"| 共同成功整树 CPU 时间中位（s） | {v('tree_cpu_s')} |",f"| 整树 PSS 返回后快照中位（MiB） | {v('tree_pss_mib')} |",'',
f"门槛判定：保持原成功查询 {'通过' if d['gates']['correctness'] else '未通过'}；耗时中位至少降低 5% {'通过' if d['gates']['wall_5pct'] else '未通过'}；整树 RSS 中位至少降低 5% {'通过' if d['gates']['tree_rss_5pct'] else '未通过'}。",'',
'## 方案与边界','',
'1. 在拓扑和多分辨率视图创建前封装不可变障碍数据，拓扑、动态障碍静态底图和原生碰撞检查共享同一份只读存储。动态障碍增删仍生成当前版本地图。',
'2. 一次曲线求解复用同一份障碍与范围约束图；原生检查使用相同布尔字节的 uint8 视图，避免重复整图转换。',
'3. 三次 B 样条利用每个区间最多四个非零基函数，用四个周期系数通道求值并直接组装稀疏矩阵。随机非均匀节点、端点、外推点及 0–3 阶导数与原稠密实现逐元素相等。',
'4. 控制器清理回调采用弱引用，消除回调对控制器自身的强引用；空动态障碍层用单个只读常量表示，当前同值范围掩码共享存储。',
'5. 先尝试固定端点的二次目标候选，经过快速预筛及原连续证书、完整三层审计后才交付；拒绝则在同一预算内使用原 SQP 流程。原型在语义图上的昂贵证书拒绝未被采用。',
'6. 利用原信赖域步长上限，计算每一条线性化约束在本次 QP 中可达到的最坏值；只剔除可证明冗余的行，非线性连续证书及最终交通/车体审计保持。','',
'L1 封存源码、地图及原始 Q 保持；L2 搜索、道路方向/停车规则、连续曲率与车体扫掠证书、实际 costmap 内容 ACK、0.8/1.2 m 局部范围和原预算均未放宽。新版本在独立目录开发，历史生产入口保持。',
'本轮仅代表十图固定查询的静态完整路线回放，不代表动态障碍全面通过、连续控制或实车验收。','',
'## 每张地图','',
'每版每图 30 次，使用本图原 Q01–Q10 或其登记 A2B 查询。第二轮倒序地图，按地图与轮次交替 V9/V10 先后顺序，CPU 14 串行、无剖析器和存储账本。统计成功与失败分别保留；下列耗时及内存只用共同成功查询。','',
'| 地图 | V9/V10 严格成功 | 请求中位变化 | 整树 RSS 中位变化 | 剩余失败 Q |','|---|---:|---:|---:|---|']
for m in d['maps']:
 lines.append(f"| {m['name']} | {m['v9_success']}/30 → {m['v10_success']}/30 | {m['common_success']['wall_s']['median_change_pct']:+.2f}% | {m['common_success']['tree_rss_mib']['median_change_pct']:+.2f}% | {', '.join(m['remaining']) or '—'} |")
lines+=['',f"地图等权几何平均中位比：耗时 {(d['map_equal_weight']['wall_s']['geometric_mean_median_ratio']-1)*100:+.2f}%，整树 RSS {(d['map_equal_weight']['tree_rss_mib']['geometric_mean_median_ratio']-1)*100:+.2f}%。总体门槛通过不意味着所有地图分别改善 5%。",'',
f'![请求耗时箱线图]({R/"wall_s_boxplots.png"})' ,'',f'![规划进程树内存箱线图]({R/"tree_rss_mib_boxplots.png"})' ,'',
'箱线图均来自逐次原始记录，使用真实宋体 SimSun；原始值、四分位数、1.5×IQR 须和离群值在 BOXPLOT_DATA.json 中留存。耗时对数坐标，内存线性坐标。严格成功次数单独列表。','',
'## 长尾、准备与失败','',
f"全部 300 次尝试总耗时变化 {d['all']['wall_s']['sum_change_pct']:+.2f}%；全部请求 P95 {d['all']['wall_s']['v9_p95']:.3f} → {d['all']['wall_s']['v10_p95']:.3f} s；共同成功请求 P95 {c['wall_s']['v9_p95']:.3f} → {c['wall_s']['v10_p95']:.3f} s。",'',
'请求计时包含真实规划调用，排除一次性地图/审计准备、节点启动和结果导出。整树 RSS 是每 20 ms 采样的峰值，并非单一 Python 进程或无遗漏的硬件峰值。PSS 是返回后的快照，不能当成峰值。准备与首条响应另列；首条响应使用基准启动计时，包含源文件快照导出等实验记录开销：','',
'| 地图 | 地图准备中位 V9→V10（s） | 准备至首条响应中位 V9→V10（s） |','|---|---:|---:|']
for m in d['maps']:
 vals=lambda arm,k:np.median([x[k] for x in d['cold'] if x['map_id']==m['map_id'] and x['arm']==arm])
 lines.append(f"| {m['name']} | {vals('v9','preparation_s'):.2f} → {vals('v10','preparation_s'):.2f} | {vals('v9','first_response_s'):.2f} → {vals('v10','first_response_s'):.2f} |")
lines+=['','原剩余失败逐组保留。Arena medium A2B-04、导师原图 A2B-07 的局部反向构造搜索未找到可行接入，未接入正式入口。局部搜索失败或预算耗尽不构成物理不可行证明。','',
'## 验证与原始记录','',
f"共同成功的 {d['common_success_n']} 次请求中，L1 输出、L2 参考线及软目标全部逐文件一致。最终轨迹均有数值变化，不能宣称路径逐点不变。新增严格成功 {len(d['new_successes'])} 次，退化 {len(d['regressions'])} 次；变化明细见 COMPARISON.json。",'',
'正式数据只按 SELECTED_DATA.json 指定，早期源码诊断、剖析和中断批次不混入门槛统计。SOURCE_REVIEW.json 记录源文件、冻结 L1、父版清单、原生库、测试和拥有资源清理审计。raw_paired_metrics.csv 保留每次计量，COMPARISON.json 保留每图、总体和准备分项。']
review=json.loads((R/'SOURCE_REVIEW.json').read_text());detail=json.loads((R/'DETAILS.json').read_text())
lines+=['',f"731 项检查通过（1 个继承的 NumPy 警告）。父版 {review['parent_files_unchanged']} 个登记文件哈希未变；正式 60 批与全部诊断批次均完成拥有资源清理。连续证书的核心区间验证循环 AST 与父版相同，QP 求解器实现未变。",'',
 f"共同成功请求的有效 costmap 及 L1/L2/软目标逐文件一致；最终路径长度变化范围 {review['min_path_length_change_pct']:+.6f}% 至 {review['max_path_length_change_pct']:+.6f}%。所有成功轨迹均通过原连续曲率和车体检查，最小转弯半径要求保持 0.4 m。",'',
 f"{review['certified_quadratic_deliveries']} 次成功交付直接来自通过全部审计的二次候选。线性化几何约束累计从 {review['linear_constraints_original_vs_retained'][0]:,} 行降至 {review['linear_constraints_original_vs_retained'][1]:,} 行；只对本次 QP 中可证明冗余的行做筛除。",'',
 f"每版留存 L1 输出 300 份、L2 参考线 297 份；三层严格成功仍为每版 282 次。文件存在不代表三层严格成功。",'',
 '## 剩余失败的耗时','', '| 地图 ID / 查询 | V9 中位（s） | V10 中位（s） | 变化 |','|---|---:|---:|---:|']
for item in detail['failure_queries']:
 lines.append(f"| {item['map_id']} / {item['query_id']} | {item['v9_median_s']:.3f} | {item['v10_median_s']:.3f} | {item['change_pct']:+.2f}% |")
lines+=['','未解决查询继续失败安全退出，没有可执行轨迹。下一阶段仍需处理端点接入、道路方向与有序走廊冲突、导师原图的 L2 桥接，并优化首次地图准备；本轮未宣称这些问题已经解决。','',
 '统计不把同一 Q 的三轮重复当成独立地图样本。固定地图内按 Q 成组配对重采样 3000 次，保留三轮相关性，耗时中位变化的 95% 区间为 '+'['+', '.join(f'{v:.2f}' for v in detail['bootstrap']['wall_ms']['change_pct_95pct_interval'])+']'+'%，RSS 为 '+'['+', '.join(f'{v:.2f}' for v in detail['bootstrap']['rss_peak_bytes']['change_pct_95pct_interval'])+']'+'%。耗时区间包含不足 5% 的改善，因此本轮通过的是观测值门槛，尚不能断言以 95% 置信度超过 5%。这是本固定地图集内的估计，不是未见地图的保证。','',
 '## 未采用与范围限制','',
 '第一轮存储/基函数优化的十图对照为双方 94/100，耗时中位 -3.25%、RSS -2.96%，未达到 5% 门槛，原始记录保留。第二轮二次候选原型在语义图中产生较贵的证书拒绝；正式方案加入快速预筛与有界尝试，并采用约束筛除。早期调度器暂停导致一批清理日志未正常收尾，按拥有资源日志恢复清理后保留原批次并重新运行，该中断批次不计入正式结果。',
 '正式选择仅 results/paired_final；所有前期诊断、剖析、未完成与中断批次均排除。启动准备未被本轮整体解决，不能把请求中位提速写成首次启动同比提速。']
events=R/'EVENTS.json'
if events.exists():
 ev=json.loads(events.read_text());lines+=['',f"额外受控障碍事件回放：{ev['passed_events']} 项事件断言通过；仅覆盖列出的两张地图及本批测试，不能推广为十图动态或实车验收。万达3w地库 Q09 的前方障碍仍未找到绕行路径，预期行为是撤回轨迹并在清除后恢复；Arena 大图本次前方障碍重规划成功。"]
lines+=['', '调度结束复核：外层工具会话回报退出码 143，原因未确定；此前日志已写出全部完成标记，60 个正式工作进程均为退出码 0 且清理完成。随后以同一源码锁只读恢复检查全部批次，退出码 0，未重跑或替换任何正式请求。详见 ORCHESTRATION_COMPLETION.json。']
(R/'RESULTS.md').write_text('\n'.join(lines)+'\n')
