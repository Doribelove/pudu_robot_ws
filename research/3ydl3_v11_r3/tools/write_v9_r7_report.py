"""Final R7 record: adopted static candidate and explicitly unmet speed goals."""
from pathlib import Path
from compare_v9_review import load
ROOT=Path(__file__).resolve().parents[1]

def main():
 d=load(ROOT/'report/R7_ALL_COMPARISON.json');s=load(ROOT/'report/R7_SHARED_COMPARISON.json')
 assert d['default_entry_verified'] and d['selected_mode']=='ASCHNT'
 lines=['# 3YD-V9-r7：完整任务存图与中性标签存储','','采用本独立工作区的 **ASCHNT**：父V8-r11 AB、V9 A、审计S、完整任务Native存图C、推进连接H、中性标签N、全局/Native批次绑定T。B/P默认关闭。历史生产入口、封存L1/V8-r11/V9-r0至r6保持；L1/L2算法、原Native和三层硬审计不改。','','## 正式同批固定10×3','','两图每臂60/60三层严格，AS控制与父R6物理路径/上游/有效地图60/60相同；候选原L1/L2/软目标/global mask/source一致。Native只保存完整逻辑任务矩形，未加载原全图，无移动窗口或全图Smac自由搜索，Python仍保留原全图供L1/L2与审计。','','|地图|首次中位AS→ASCHNT /s|变化|全部30次进程树RSS中位 /GiB|变化|重复中位变化|重复P95变化|','|---|---:|---:|---:|---:|---:|---:|']
 for mid,v in d['fixed']['maps'].items():
  def val(a,split,m,t='median'):return v['arms'][a][split]['metrics'][m][t]
  x,y=val('control','first','e2e_s'),val('CHNT','first','e2e_s');a,b=val('control','all','tree_rss_gib'),val('CHNT','all','tree_rss_gib')
  repeat=100*(val('CHNT','repeat','e2e_s')/val('control','repeat','e2e_s')-1)
  tail=100*(val('CHNT','repeat','e2e_s','p95')/val('control','repeat','e2e_s','p95')-1)
  lines.append(f"|{v['label']}|{x:.6f}→{y:.6f}|{100*(y/x-1):+.2f}%|{a:.6f}→{b:.6f}|{100*(b/a-1):+.2f}%|{repeat:+.2f}%|{tail:+.2f}%|")
 lines+=['','固定门槛全部通过：两图首次或冷首条中位>=3%、query-tree RSS中位>=5%、重复中位及首次/重复P95退化<=5%、严格/上游/相对质量通过。首次每图每臂n10、重复n20，共同进程/任务内相关；无随机化显著性结论，慢样本与离群点保留。RSS为worker+ROS后代20ms采样峰值，PSS为独立快照，不能将Python PSS整体归为L2。','',f'![首次箱线图]({ROOT}/report/first_r7.png)',f'![重复箱线图]({ROOT}/report/repeat_r7.png)','','## 独立冷进程与完整起点/事件','','独立冷进程每图每臂n3，12/12严格，各轮同臂物理输出与固定首条相同；冷准备至首响应包括地图/审计/源码导出/ROS启动/首请求，排除模块导入和结果导出。','','|地图|冷准备至首响应中位AS→ASCHNT /s|变化|','|---|---:|---:|']
 for mid,v in d['cold'].items():
  a=v['arms']['control']['metrics']['cold_first_response_ms']['median']/1000;b=v['arms']['CHNT']['metrics']['cold_first_response_ms']['median']/1000
  lines.append(f"|{v['label']}|{a:.6f}→{b:.6f}|{100*(b/a-1):+.2f}%|")
 lines+=['',f'![独立冷进程原始点]({ROOT}/report/cold_r7.png)','','精确起点/3cm与2°偏差各每图每臂10初始+30变化；候选分别60/60变化+20/20初始，两臂总320/320严格。两种workload的原上游/global source/mask/相对质量及原审计通过。变化请求不是新独立路线。','','|工作负载|地图|变化请求耗时中位AS→ASCHNT /s|变化|','|---|---|---:|---:|']
 for kind,maps in d['regression'].items():
  for mid,v in maps.items():
   a=v['arms']['control']['moving']['metrics']['e2e_s']['median'];b=v['arms']['CHNT']['moving']['metrics']['e2e_s']['median']
   lines.append(f"|{kind}|{v['label']}|{a:.6f}→{b:.6f}|{100*(b/a-1):+.2f}%|")
 lines+=['','**精确推进请求中位增加22.31%/17.84%，这项未改善。** Native原点造成初始路径微差，部分原精确起点需要短连接，不能将首次性能收益写成所有请求都提速；后续独立版本继续优化此处。偏差起点本批中位改善11.35%/6.63%。','','两处arena A2B-01_p20（moving/perturbed）原净空0.7090698480606079→0.659069836139679m，raw delta=-0.050000011920928955m；quality_gate_direct=False，沿用实验前R6声明的float32输入半ULP和5.960464477539063e-8m（封顶1e-6）得到quality_gate=True。原车体/曲率/转向/交通硬审计无任何豁免，原值与两种结果分列于QUALITY_BOUNDARIES_SOURCE2.json。诊断console曾用了错误字段名显示为空列表，正式JSON判定一直使用正确helper；修正记录保留。','',f'![精确起点箱线图]({ROOT}/report/moving_r7.png)',f'![偏差起点箱线图]({ROOT}/report/perturbed_r7.png)','','四批共44事件检查与原预期一致：重复/乱序/无观测清除被拒绝；过去障碍不重规划；前方和当前阻塞触发影响重规划，失败撤回；清除恢复。Q09前方障碍仍失败撤回，arena前方障碍可重规划；不能写全部动态障碍通过。','','## 跨任务段缓存联合诊断','','原shared v2每图每臂一组种子+三个不同起点/终点的新任务，A/AB/ABP均共用CHNT，24/24严格。每个新任务有新L1/L2绑定，无整路径或任务续接命中；原上游/global来源/Native完整task/质量通过。','','|地图|臂|新任务严格|局部成功任务|段复用成功任务|端到端中位变化相对A|','|---|---|---:|---:|---:|---:|']
 for mid,v in s['maps'].items():
  for arm,w in v['arms'].items():
   q=w['new_tasks'];pct='—' if arm=='A' else f"{v['new_task_median_change_pct'][arm]:+.2f}%"
   lines.append(f"|{v['label']}|{arm}|{q['success']}/{q['n']}|{q['local_success']}|{q['segment_reuse_success']}|{pct}|")
 lines+=['','局部/段成功要求最终完整任务严格通过且候选接受；段成功计任务数，arena每任务匹配2段不能写成6个任务。B/P没有两图实际提速证据，保留显式诊断，**不默认采用**；n3仅原始点，不能推广fixed10、更多地图或尾部。T真实内部Smac动作在原A2B-01单独完成；其局部候选仍拒绝、最终由原受限回退严格通过，不混计为局部成功。','',f'![联合缓存诊断原始点]({ROOT}/report/shared_r7.png)','','## 存储、源与复核','','N只对精确原MapOnlyWorld/property SHA/空features和rules/全部原标签0 guard使用原shape/dtype的2byte只读broadcast；语义地图保持原标签，任一guard不满足回原。真实arena原60MB owner→2byte，离线owner数据不直接当RSS。T捕获全球与Native mask/source/hard/soft/internal/affected、task、frame、origin、resolution、vehicle、policy、publication sequence、trust和实际两稳定零差异ACK，沿用原parent batch的当前Native identity/content/sequence/trust检查；变化拒绝、finally释放，稀疏字段不展开全图float。普通更新保持原刷新/读回。','','source1 32组原Q09/A2B03来源诊断、原真实global/native批次错误与全部原复制source保留，不混正式统计。source2正式120固定+12cold+320起点=452/452严格；另有24共享诊断、2真实默认入口，事件单列。原探索中的绑定失败保留，不参与正式性能。','','完整345测试通过；当前规划锁114项；原L1源16/输入9/pixel8、原三层审计AST、TaskProgress/L2 Native、16原Native文件、9外部硬审计、8生产入口、父冻结归档及V9-r0至r6全部release文件校验通过。47份原复制source、28正式当前source快照、47批拥有进程/共享资源清理验证通过。真实run.sh未传候选覆盖开关，两图2/2与选中正式首条物理输出相同；仅本独立工作区默认改变，历史生产保持。','','所有图显式加载真SimSun并看图；连续原始n10/n20/n30箱线图标明Q1–Q3、中线中位、1.5IQR须和离群点/单位/线性轴；n3仅点图。完整正式记录仅results/accepted_r7_v2，联合诊断results/shared_joint_v2，当前默认入口results/selected_entry_v2。数据选择见SELECTED_DATA.json；每case的轨迹/全局scope/L1/L2/soft/outcome及ROS原读回/ACK、source/native快照保存。','','采用范围仅这两图完整静态规划与停止式障碍回放，不是更多图、连续定位/控制或实车验收。整体优化goal仍active：精确推进耗时退化和B/P跨任务实际提速尚未解决，后续另建R8，不覆盖本版。']
 (ROOT/'report/RESULTS.md').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()
