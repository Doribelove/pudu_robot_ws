from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.ticker import FuncFormatter, NullFormatter
R=Path(__file__).resolve().parents[1]
font=Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc')
assert font.is_file();fm.fontManager.addfont(str(font));name=fm.FontProperties(fname=str(font)).get_name();assert name.lower()=='simsun',name
plt.rcParams.update({'font.family':name,'axes.unicode_minus':False,'font.size':11})
data=list(csv.DictReader((R/'report/paired_rows.csv').open()))
fig,axes=plt.subplots(1,3,figsize=(12,5))
for ax,col,cohort,title,unit in [(axes[0],'navigation_s',True,'原99组：首次导航','秒'),(axes[1],'tree_rss_mib',True,'原99组：整树内存','MiB'),(axes[2],'navigation_s',False,'全部请求：含容差回退','秒')]:
 selected=[r for r in data if not cohort or r['original_99']=='True']
 groups=[[float(r[col]) for r in selected if r['arm']==arm] for arm in ('control','candidate')]
 bp=ax.boxplot(groups,labels=[f'V11-r2\nn={len(groups[0])}',f'V11-r3\nn={len(groups[1])}'],patch_artist=True,showfliers=True,whis=1.5,medianprops={'color':'#172b4d','linewidth':1.5},flierprops={'marker':'.','markersize':3,'markeredgecolor':'#7b8190','alpha':.45})
 for box,color in zip(bp['boxes'],('#bdd2ea','#bce1cf')):box.set_facecolor(color)
 ax.set_title(title);ax.set_ylabel(unit);ax.grid(axis='y',alpha=.2)
 if col=='navigation_s':
  ax.set_yscale('log');ax.set_ylabel('秒（对数坐标）')
  ax.yaxis.set_major_formatter(FuncFormatter(lambda value,position:f'{value:g}'));ax.yaxis.set_minor_formatter(NullFormatter())
fig.suptitle('3YDL3-V11-r3｜同批十图固定查询 × 三轮',fontsize=17)
fig.text(.5,.02,'箱体：Q1—Q3；中线：中位数；须：1.5×IQR范围内最远值；散点：离群值。\n每Q每轮首次导航；包含精确尝试、容差恢复、同步、真实ACK及终验。',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.13,1,.93));fig.savefig(R/'report/paired_distributions.png',dpi=180)
plt.close(fig)
names={'mengjia':'万达3w地库','pudu_wanda_3f':'万达2w地库','3yd_synth_multiloop_a_v1':'合成A','3yd_synth_twinwing_b_v1':'合成B','3yd_synth_trizone_c_v2':'合成C','arena_small_080x100_2d_005':'Arena小图','arena_medium_200x200_2d_005':'Arena中图','arena_large_250x300_2d_005':'非语义大地图','mentor_map_20260825_005':'导师原图','mentor_map_20260825_005_4x_area':'导师四倍图'}
selected=[r for r in data if r['original_99']=='True']
maps=list(dict.fromkeys(r['map_id'] for r in selected))
fig,axes=plt.subplots(2,1,figsize=(14,8),sharex=True)
from matplotlib.patches import Patch
for ax,col,title,unit in [(axes[0],'navigation_s','原99组：各地图首次导航','秒（对数坐标）'),(axes[1],'tree_rss_mib','原99组：各地图整树采样峰值RSS','MiB')]:
 for offset,arm,color in [(-.17,'control','#bdd2ea'),(.17,'candidate','#bce1cf')]:
  groups=[[float(r[col]) for r in selected if r['map_id']==m and r['arm']==arm] for m in maps]
  bp=ax.boxplot(groups,positions=np.arange(len(maps))+offset,widths=.28,patch_artist=True,showfliers=True,whis=1.5,manage_ticks=False,medianprops={'color':'#172b4d','linewidth':1.2},flierprops={'marker':'.','markersize':3,'markeredgecolor':'#7b8190','alpha':.5})
  for box in bp['boxes']:box.set_facecolor(color)
 ax.set_title(title);ax.set_ylabel(unit);ax.grid(axis='y',alpha=.2)
 if col=='navigation_s':
  ax.set_yscale('log');ax.yaxis.set_major_formatter(FuncFormatter(lambda value,position:f'{value:g}'));ax.yaxis.set_minor_formatter(NullFormatter())
 ax.legend(handles=[Patch(facecolor='#bdd2ea',label='V11-r2'),Patch(facecolor='#bce1cf',label='V11-r3')],loc='upper left')
counts=[sum(r['map_id']==m and r['arm']=='candidate' for r in selected) for m in maps]
axes[1].set_xticks(np.arange(len(maps)),[names[m]+f'\n每版n={n}' for m,n in zip(maps,counts)])
fig.suptitle('3YDL3-V11-r3｜固定原成功集的地图分布',fontsize=17)
fig.text(.5,.018,'同批三轮，每Q每轮首次请求；导师原图仅原9组，共27次/版。\n箱体Q1—Q3，中位数线，须为1.5×IQR范围内最远样本，散点为离群值；耗时采用对数坐标。',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.08,1,.94));fig.savefig(R/'report/per_map_distributions.png',dpi=180)
plt.close(fig)
(R/'report/FONT_CHECK.json').write_text(json.dumps({'font_file':str(font),'resolved_family':name,'font_loaded_explicitly':True},indent=2)+'\n')

# Per-map normalization is a supplement; the original total-time charts remain.
fig,axes=plt.subplots(2,1,figsize=(14,8),sharex=True)
for ax,col,title,unit in [(axes[0],'navigation_s_per_l1_m','首次导航 / L1参考长度','秒/米（对数坐标）'),(axes[1],'tree_rss_mib_per_l1_m','整树采样峰值RSS / L1参考长度','MiB/米（对数坐标）')]:
 for offset,arm,color in [(-.17,'control','#bdd2ea'),(.17,'candidate','#bce1cf')]:
  groups=[[float(r[col]) for r in selected if r['map_id']==m and r['arm']==arm] for m in maps]
  bp=ax.boxplot(groups,positions=np.arange(len(maps))+offset,widths=.28,patch_artist=True,showfliers=True,whis=1.5,manage_ticks=False,medianprops={'color':'#172b4d'},flierprops={'marker':'.','markersize':3,'markeredgecolor':'#7b8190','alpha':.5})
  for box in bp['boxes']:box.set_facecolor(color)
 ax.set_title(title);ax.set_ylabel(unit);ax.set_yscale('log');ax.grid(axis='y',alpha=.2)
 ax.yaxis.set_major_formatter(FuncFormatter(lambda value,position:f'{value:g}'));ax.yaxis.set_minor_formatter(NullFormatter())
 ax.legend(handles=[Patch(facecolor='#bdd2ea',label='V11-r2'),Patch(facecolor='#bce1cf',label='V11-r3')],loc='upper left')
axes[1].set_xticks(np.arange(len(maps)),[names[m]+f'\n每版n={n}' for m,n in zip(maps,counts)])
fig.suptitle('3YDL3-V11-r3｜原99组：按各次L1参考长度归一化',fontsize=17)
fig.text(.5,.018,'箱体Q1—Q3，中位数线，须为1.5×IQR内最远样本，散点为离群值；两轴均为对数坐标。\nRSS/米是归一化比值，不是每米新增内存或内存累计分配量。',ha='center',fontsize=10)
fig.tight_layout(rect=(0,.08,1,.94));fig.savefig(R/'report/normalized_distributions.png',dpi=180)
plt.close(fig)
