import json,hashlib
from pathlib import Path
from compare_v8_r3 import arm_summary,common_protocol,libraries,read,sha,audit_ast
from analyze_v8 import measure,stats
ROOT=Path(__file__).resolve().parents[1];PARENT=Path('/home/robot/workspaces/3yd_v8_r4_20261008')
MAPS=[('mengjia','万达3w地库'),('arena_large_250x300_2d_005','arena_large'),('3yd_synth_multiloop_a_v1','合成地库A'),('arena_medium_200x200_2d_005','arena_medium')]
def main():
 result={'version':'3YD-V8-r5','maps':{}};data={};journals=[]
 for i,(mid,label) in enumerate(MAPS):
  selection=json.loads((PARENT/'report/SELECTED_DATA.json').read_text())
  a=PARENT/selection[('ABC/' if i<2 else 'extra/ABC/')+mid]
  b=ROOT/('results/formal' if i<2 else 'results/additional')/mid
  old,new=arm_summary(a),arm_summary(b);assert old['n']==new['n']==(30 if i<2 else 10)
  left={(r['query_id'],r['repeat']):r for r in read(a)};pairs=[]
  for r in read(b):
   key=r['query_id'],r['repeat'];previous=left[key];tag=f'{key[0]}_{key[1]}'
   assert not previous['success'] or r['success'],(mid,tag,'regression')
   p={'query':tag,'before':previous['success'],'after':r['success']}
   for f in ['l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy']:
    x,y=a/'cases'/tag/f,b/'cases'/tag/f;p[f]=sha(x)==sha(y) if x.exists() and y.exists() else None
   assert all(p[f] for f in ['l1_output.json','l2_reference_xy.npy','l2_soft_target.json'])
   if previous['success']:assert p['path.npy'],(mid,tag,'existing path changed')
   pairs.append(p)
  common_protocol(a,b);assert libraries(a)==libraries(b)
  result['maps'][mid]={'label':label,'r4':old,'r5':new,'parity':pairs};data[mid]={'r4':read(a),'r5':read(b)}
  h=json.loads((b/'source_snapshot/hashes.json').read_text())
  assert all(sha(Path(p))==x for p,x in h.items() if '/src/' in p)
  jpath=b.with_name(b.name+'.supervisor.json');j=json.loads(jpath.read_text());assert j['state']=='complete' and j['cleanup']['complete']
  journals.append(str(jpath))
 lock=json.loads((ROOT/'L1_LOCK.json').read_text())
 for k in ('source_files','input_files','pixel_input_files'):
  assert all(sha(ROOT/f)==h for f,h in lock.get(k,{}).items()),k
 for f in ['src/three_yd_v4/r5_controller.py','src/three_yd_v5/controller.py']:
  assert audit_ast(ROOT/f)==audit_ast(PARENT/f)
 parent=json.loads((PARENT/'RELEASE.json').read_text())
 assert all(sha(PARENT/f)==h for f,h in parent['files'].items()),'parent release changed'
 for f in ('native/cold_dstar.cpp','native/libcold_dstar.so'):
  assert sha(ROOT/f)==sha(PARENT/f),f
 result['verification']={'L1_unchanged':True,'audits_unchanged':True,'native_nav2_unchanged':True,'native_L2_unchanged':True,'parent_files_unchanged':len(parent['files']),'supervisors_complete':journals}
 (ROOT/'report/COMPARISON.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib import font_manager
 font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
 font_manager.fontManager.addfont(font);assert font_manager.FontProperties(fname=font).get_name()=='SimSun'
 plt.rcParams.update({'font.family':'SimSun','axes.unicode_minus':False,'font.size':11})
 fig,axes=plt.subplots(4,3,figsize=(16,17))
 for row,(mid,label) in enumerate(MAPS):
  for ax,(metric,title) in zip(axes[row],[('e2e_s','端到端 / s'),('l3_s','L3流水线 / s'),('tree_rss_gib','进程树峰值RSS / GiB')]):
   values=[[measure(r,metric) for r in rows] for rows in data[mid].values()]
   ax.boxplot(values,labels=[a+'\nn='+str(len(v)) for a,v in zip(data[mid],values)],whis=1.5,showfliers=True)
   ax.set_title(label+'：'+title,fontweight='bold');ax.grid(axis='y',alpha=.2)
 fig.suptitle('V8-r5转向修复触发纠正：四图独立结果',fontsize=18,fontweight='bold')
 fig.text(.5,.012,'箱体Q1～Q3；中线中位数；须为1.5 IQR内最远点；圆点离群值。线性坐标。\n各图的样本数和重复次数分别标注；失败查询计入分布，L3含层间适配。',ha='center')
 fig.tight_layout(rect=[0,.055,1,.96]);fig.savefig(ROOT/'report/r4_r5_boxplots.png',dpi=150);plt.close(fig)
 print({m:(v['r4']['success'],v['r5']['success']) for m,v in result['maps'].items()})
if __name__=='__main__':main()
