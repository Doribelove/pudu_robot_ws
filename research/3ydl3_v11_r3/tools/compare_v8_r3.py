"""Raw-call, fixed-input A/B comparison; never edits frozen parents."""
import ast,hashlib,json
from pathlib import Path
import numpy as np
from analyze_v8 import measure,stats,METRICS,MAPS
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r2_20261008')
FREE='arena_large_250x300_2d_005'
FILES=('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy')
ARM_LABELS={'control':'r2逻辑对照','A':'A 批量检查','B':'B 提前分段','AB':'r3 A+B'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(d):return [json.loads(x) for x in (d/'runs.jsonl').read_text().splitlines() if x.strip()]
def indexed(rs):return {(r['query_id'],r['repeat']):r for r in rs}
def metricset(rows):
    return {m:{s:stats([measure(r,m) for r in rs]) for s,rs in
      [('all',rows),('first',[r for r in rows if r['repeat']==1]),('repeat',[r for r in rows if r['repeat']>1])]}
      for m in [*METRICS,'strict_audit_s']}
def failure_detail(r):
    log=r.get('r1_segmented_recovery') or r.get('r3_early_sections',{}).get('section_log',{})
    bad=next((s for s in log.get('sections',[]) if not s['success']),None)
    return {'query':r['query_id'],'repeat':r['repeat'],'failure':r.get('failure_code'),
      'L2_reference_generated':bool(r.get('l2',{}).get('v5',{}).get('success')),
      'whole_termination':r.get('l3',{}).get('r1_whole_route_termination_reason') or r.get('l3',{}).get('termination_reason'),
      'section_failure':log.get('failure'),'final_failures':log.get('final_failures'),
      'final_steering_jump_deg':log.get('final_steering_jump_deg'),
      'first_failed_section':None if bad is None else {k:bad[k] for k in ('index','start','goal','failure')},
      'map_ACK':r.get('l3',{}).get('costmap_update_acknowledged'),
      'scope':r.get('bounded_l3',{}),
      'attempts':[{'failure':a.get('failure'),'l3_termination':a.get('l3_diagnostics',{}).get('termination_reason')} for a in r.get('recovery',{}).get('attempts',[])]}
def checked_success(row):
    if not row['success']:return
    assert row['audit']['valid'] and row['constraints']['steering_jump_valid'],row['query_id']
    assert row['constraints']['traffic']['valid'],row['query_id']
    assert row['l3']['costmap_update_acknowledged'] is True and row['l3']['costmap_ack_mismatch_cells']==0
    b=row['bounded_l3'];assert b['global_fallback'] is False and b['max_sample_radius_m']<=1.2
    assert b['search_cells']<=400000 and b['map_fraction']<=.1
def arm_summary(d):
    rows=read(d)
    for r in rows:checked_success(r)
    return {'directory':str(d),'runs_sha256':sha(d/'runs.jsonl'),'n':len(rows),
      'success':sum(r['success'] for r in rows),'metrics':metricset(rows),
      'right':stats([r.get('right_side_fraction') for r in rows if r['success']]),
      'length':stats([r.get('length_m') for r in rows if r['success']]),
      'backend_calls':sum(r.get('l3',{}).get('backend_call_count',1) for r in rows),
      'early_sections':[{'query':r['query_id'],'repeat':r['repeat'],**r['r3_early_sections']} for r in rows if r.get('r3_early_sections')],
      'failures':[failure_detail(r) for r in rows if not r['success']]}
def parity(a,b):
    left,right=indexed(read(a)),indexed(read(b));assert left.keys()==right.keys()
    records=[]
    for key,r in right.items():
      tag=f'{key[0]}_{key[1]}';item={'query':tag,'same_success':left[key]['success']==r['success']}
      for f in FILES:
        pa,pb=a/'cases'/tag/f,b/'cases'/tag/f
        item[f]=sha(pa)==sha(pb) if pa.exists() and pb.exists() else None
      records.append(item)
    return {'n':len(records),'counts':{f:sum(x[f] is True for x in records) for f in FILES},'records':records}
def changes(a,b):
    return {m:{split:{s:None if a[m][split][s] in (None,0) or b[m][split][s] is None else
      100*(b[m][split][s]/a[m][split][s]-1) for s in ('median','p95','max')}
      for split in ('all','first','repeat')} for m in a}
def common_protocol(a,b):
    pa,pb=[json.loads((d/'protocol.json').read_text()) for d in (a,b)]
    fields=('map_sha256','input_sha256','queries_sha256','parking_access_sha256','query_ids','vehicle','CPU',
      'L2_resolution_m','l2_resolution_m','L3_normal_radius_m','l3_normal_radius_m',
      'L3_corner_radius_m','l3_corner_radius_m','max_search_cells','max_map_fraction','unknown_is_collision')
    check={f:pa[f]==pb.get(f) for f in fields if f in pa};assert all(check.values()),check
    return check
def libraries(d):
    v=json.loads((d/'loaded_nav2_libraries.json').read_text())
    return {k:x for p in v['processes'] for k,x in p['libraries'].items()}
def audit_ast(p):
    return [ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(p.read_text()))
      if isinstance(n,ast.FunctionDef) and n.name=='_audit']
def additional():
    result={};datasets={}
    for mid,label in [('3yd_synth_multiloop_a_v1','合成地库A（语义）'),('arena_medium_200x200_2d_005','arena_medium（无语义）')]:
      a=ROOT/'results/additional/control'/mid;b=ROOT/'results/additional/AB'/mid
      if not (b/'runs.jsonl').exists():continue
      sa,sb=arm_summary(a),arm_summary(b);assert sa['n']==sb['n']==10
      pa=parity(a,b);assert pa['counts']['l1_output.json']==10,(mid,pa)
      assert all(r['same_success'] for r in pa['records']),(mid,'regression')
      common_protocol(a,b);assert libraries(a)==libraries(b)
      result[mid]={'label':label,'control':sa,'AB':sb,'parity':pa,
        'change_percent':changes(sa['metrics'],sb['metrics'])}
      datasets[mid]={'control':read(a),'AB':read(b)}
    if datasets:
      import matplotlib
      matplotlib.use('Agg')
      import matplotlib.pyplot as plt
      from matplotlib import font_manager
      font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
      font_manager.fontManager.addfont(font);assert font_manager.FontProperties(fname=font).get_name()=='SimSun'
      plt.rcParams.update({'font.family':'SimSun','axes.unicode_minus':False,'font.size':11})
      fig,axes=plt.subplots(2,3,figsize=(15,9))
      for rr,(mid,arms) in enumerate(datasets.items()):
        for ax,(metric,title) in zip(axes[rr],[('e2e_s','端到端 / s'),('l2_s','L2整体处理 / s'),('tree_rss_gib','查询进程树峰值RSS / GiB')]):
          values=[[measure(r,metric) for r in rows] for rows in arms.values()]
          values=[[v for v in group if v is not None] for group in values]
          ax.boxplot(values,labels=[ARM_LABELS[name]+'\nn='+str(len(vals)) for name,vals in zip(arms,values)],whis=1.5,showfliers=True)
          ax.set_title(result[mid]['label']+'\n'+title,fontweight='bold');ax.grid(axis='y',alpha=.2)
      fig.suptitle('V8-r3额外地图：每图固定10组各1次，失败也计入查询分布',fontsize=17,fontweight='bold')
      fig.text(.5,.02,'箱体Q1～Q3；中线中位数；须为1.5 IQR内最远点；圆点离群值。线性坐标。\n新增地图成绩单独报告，不与原两图重复实验混作成功率。',ha='center')
      fig.tight_layout(rect=[0,.08,1,.93]);fig.savefig(ROOT/'report/additional_boxplots.png',dpi=180);plt.close(fig)
    return result
def main():
    result={'schema':'3YD-V8-r3-raw-comparison-v1','maps':{}}
    data={}
    for mid,label in MAPS:
      paths={'control':ROOT/'results/control'/mid,'AB':ROOT/'results/formal'/mid,'r2_archive':PARENT/'results/formal'/mid}
      if mid==FREE:paths.update(A=ROOT/'results/ablation_A'/mid,B=ROOT/'results/ablation_B'/mid)
      arms={arm:arm_summary(d) for arm,d in paths.items()}
      assert all(a['n']==30 and a['success']==30 for a in arms.values())
      compare={arm:parity(paths['control'],d) for arm,d in paths.items() if arm!='control'}
      state_checks={}
      base=indexed(read(paths['control']))
      for arm,d in paths.items():
        if arm in ('control','r2_archive'):continue
        checks=[]
        for key,row in indexed(read(d)).items():
          old=base[key];tag=f'{key[0]}_{key[1]}'
          for field in ('actual_cost_hash','path_cost','dstar_expanded','corridor_cells','state_memory_bytes'):
            assert old.get('l2',{}).get(field)==row.get('l2',{}).get(field),(mid,arm,tag,field)
          la=old.get('timing',{}).get('activation_attempts',[]);ra=row.get('timing',{}).get('activation_attempts',[])
          for field in ('rectangle_band_candidates','rectangle_band_certified','rectangle_rejected_cells','rectangle_certified_cells'):
            assert [x.get(field) for x in la]==[x.get(field) for x in ra],(mid,arm,tag,field)
          f='l2_rectangle_bridge_certificates.json';p,q=paths['control']/'cases'/tag/f,d/'cases'/tag/f
          if p.exists() and q.exists():assert sha(p)==sha(q),(mid,arm,tag,'pose certificate')
          checks.append(tag)
        state_checks[arm]={'same_search_cost_expansion_and_certificates':len(checks)}
      for arm,p in compare.items():
        assert p['counts']['l1_output.json']==30 and p['counts']['l2_reference_xy.npy']==30 and p['counts']['l2_soft_target.json']==30,(mid,arm,p)
        if arm in ('A','AB'):assert p['counts']['path.npy']==30
      proto={arm:common_protocol(paths['control'],d) for arm,d in paths.items() if arm!='control'}
      assert all(libraries(paths['control'])==libraries(d) for d in paths.values())
      result['maps'][mid]={'label':label,'arms':arms,'parity':compare,'common_protocol':proto,'L2_state_checks':state_checks,
        'change_percent':{arm:changes(arms['control']['metrics'],v['metrics']) for arm,v in arms.items() if arm!='control'}}
      data[mid]={arm:read(paths[arm]) for arm in ('control','A','B','AB') if arm in paths}
    result['additional']=additional()
    lock=json.loads((ROOT/'L1_LOCK.json').read_text())
    checks=[{'file':f,'same':sha(ROOT/f)==expected} for k in ('source_files','input_files','pixel_input_files') for f,expected in lock[k].items()]
    assert all(x['same'] for x in checks)
    for f in ('src/three_yd_v4/controller.py','src/three_yd_v4/r5_controller.py','src/three_yd_v5/controller.py'):
      assert audit_ast(ROOT/f)==audit_ast(PARENT/f)
    snapshots=[]
    for d in [ROOT/'results/control'/mid for mid,_ in MAPS]+[ROOT/'results/formal'/mid for mid,_ in MAPS]+[ROOT/'results/ablation_A'/FREE,ROOT/'results/ablation_B'/FREE]:
      hashes=json.loads((d/'source_snapshot/hashes.json').read_text());src=[(p,x) for p,x in hashes.items() if '/src/' in p]
      assert all(sha(Path(p))==x for p,x in src)
      snapshots.append({'directory':str(d),'planning_source_count':len(src),'matched':True})
    journals=[]
    for p in sorted((ROOT/'results').rglob('*.supervisor.json')):
      v=json.loads(p.read_text());assert v['state']=='complete' and v['cleanup']['complete'],p
      journals.append({'path':str(p),'complete':True})
    result['verification']={'L1_locked_files':checks,'strict_audit_AST_unchanged':True,
      'native_planner_unchanged':True,'source_snapshots':snapshots,'supervisors':journals,
      'private_pose_batch_build':json.loads((ROOT/'native/POSE_BATCH_BUILD.json').read_text())}
    (ROOT/'report/COMPARISON.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    render(data)
    print(json.dumps({mid:{arm:{'success':a['success'],'e2e':a['metrics']['e2e_s'],'rss':a['metrics']['tree_rss_gib']['all']} for arm,a in v['arms'].items() if arm!='r2_archive'} for mid,v in result['maps'].items()},ensure_ascii=False,indent=2))
def render(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
    font_manager.fontManager.addfont(font);assert font_manager.FontProperties(fname=font).get_name()=='SimSun'
    plt.rcParams.update({'font.family':'SimSun','axes.unicode_minus':False,'font.size':11})
    panels=[('e2e_s','端到端 / s'),('l2_s','L2整体处理 / s'),('l3_s','L3流水线，含适配 / s'),('tree_rss_gib','查询进程树峰值RSS / GiB')]
    fig,axes=plt.subplots(2,4,figsize=(19,10))
    for rr,(mid,label) in enumerate(MAPS):
      arms=data[mid]
      for ax,(metric,title) in zip(axes[rr],panels):
        ax.boxplot([[measure(r,metric) for r in rows] for rows in arms.values()],labels=[ARM_LABELS[a]+'\nn=30' for a in arms],whis=1.5,showfliers=True)
        ax.set_title(label+'\n'+title,fontweight='bold');ax.grid(axis='y',alpha=.2)
    fig.suptitle('V8-r3：新对照及独立A/B实验（每图固定10组×3次）',fontsize=18,fontweight='bold')
    fig.text(.5,.02,'箱体Q1～Q3；中线中位数；须为1.5 IQR内最远点；圆点离群值。线性坐标。\n计入所有实际查询调用；排除一次性加载、启动及导出。各层计时存在包含关系。',ha='center')
    fig.tight_layout(rect=[0,.08,1,.93]);fig.savefig(ROOT/'report/r2_r3_boxplots.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    arms=data[FREE]
    for ax,(metric,title) in zip(axes,[('e2e_s','A2B-03端到端 / s'),('l2_s','A2B-03 L2处理 / s')]):
      for x,(arm,rows) in enumerate(arms.items()):
        rs=[r for r in rows if r['query_id']=='A2B-03']
        for j,r in enumerate(rs):
          ax.scatter(x+[0,-.07,.07][j],measure(r,metric),marker=['o','s','D'][j],s=45,
            color=['#476c9b','#6c9165','#c98b49','#9a75af'][x])
      ax.set_xticks(range(len(arms)));ax.set_xticklabels([ARM_LABELS[a]+'\n(n=3)' for a in arms]);ax.set_title(title,fontweight='bold');ax.grid(axis='y',alpha=.2)
      ax.set_xlim(-.4,len(arms)-.6)
    fig.suptitle('长路线恢复案例：逐点对照，首次与重复分开标记',fontsize=16,fontweight='bold')
    fig.text(.5,.02,'圆点：首次；方块：第2次；菱形：第3次。每组仅3次，不据此声称普遍收益。',ha='center')
    fig.tight_layout(rect=[0,.08,1,.9]);fig.savefig(ROOT/'report/A03_points.png',dpi=180);plt.close(fig)
if __name__=='__main__':main()
