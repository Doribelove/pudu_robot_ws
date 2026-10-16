from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.text import Text
from matplotlib.patches import Patch
from matplotlib.ticker import FuncFormatter
E=Path(__file__).resolve().parents[1];font=Path('/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc');assert font.exists()
fm.fontManager.addfont(str(font));prop=fm.FontProperties(fname=str(font));assert prop.get_name()=='SimSun'
plt.rcParams.update({'font.family':'SimSun','font.size':11,'axes.unicode_minus':False})
rows=list(csv.DictReader((E/'report/comparison_raw.csv').open()));maps=json.loads((E/'MAPS.json').read_text())['maps'];assert len(rows)==300
for key,label,log,filename in [('wall_s','请求耗时（秒；对数坐标）',True,'time_boxplots.png'),('tree_rss_mib','整套规划进程树 RSS 采样峰值（MiB）',False,'memory_boxplots.png')]:
 fig,ax=plt.subplots(figsize=(13,9));fig.subplots_adjust(left=.22,right=.97,bottom=.16,top=.90)
 colors=['#b5c6d4','#6eac95']
 for version,(prefix,offset,color) in enumerate([('old_',-.18,colors[0]),('',.18,colors[1])]):
  data=[[float(r[prefix+key]) for r in rows if r['map_id']==m['map_id']] for m in maps];assert all(len(v)==30 for v in data)
  bp=ax.boxplot(data,positions=np.arange(len(maps))+offset,vert=False,widths=.28,whis=1.5,showfliers=True,patch_artist=True,boxprops={'facecolor':color,'edgecolor':'#33434d'},medianprops={'color':'#8b3120','linewidth':1.6},flierprops={'marker':'o','markersize':3,'markerfacecolor':color,'markeredgecolor':'#526069'})
 ax.set_yticks(np.arange(len(maps)));ax.set_yticklabels([m['name']+'（各 n=30）' for m in maps]);ax.invert_yaxis();ax.set_xlabel(label);ax.grid(axis='x',alpha=.2)
 if log:ax.set_xscale('log');ax.xaxis.set_major_formatter(FuncFormatter(lambda x,pos:f'{x:g}'))
 ax.legend(handles=[Patch(facecolor=colors[0],label='V8 原十图记录'),Patch(facecolor=colors[1],label='V9 本轮修复')],loc='lower right')
 fig.suptitle('3YDL3-V9 十图固定查询回归：V8 / V9 原始逐次分布',fontsize=17)
 fig.text(.22,.085,'每组 10 个固定 Q × 3 个独立轮次；包括成功和失败请求，非即时复用。',fontsize=11)
 fig.text(.22,.056,'箱体 Q1–Q3；中线中位数；须为 1.5×IQR 内最远观测；圆点为离群点。',fontsize=11)
 fig.text(.22,.027,'准备、节点启动与导出不计入请求耗时；对照为历史批次，不作同期性能门槛验收。',fontsize=10)
 for t in fig.findobj(Text):t.set_fontproperties(fm.FontProperties(fname=str(font),size=t.get_fontsize()))
 fig.savefig(E/'report'/filename,dpi=170);plt.close(fig)
print('2 raw-record boxplots; font verified SimSun')
