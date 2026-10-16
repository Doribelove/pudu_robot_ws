"""Paired fixed-input verification and distributions, control vs AB vs archive."""
import ast,hashlib,json
from pathlib import Path
import numpy as np
from analyze_v8 import measure,stats,METRICS,MAPS
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r1_20261008')


def read(p):return [json.loads(x) for x in p.read_text().splitlines()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    summaries={};checks=[];datasets={}
    for mid,label in MAPS:
        control=ROOT/'results/control'/mid;formal=ROOT/'results/formal'/mid;archive=PARENT/'results/formal'/mid
        a,b,c=[read(base/'runs.jsonl') for base in (control,formal,archive)]
        for rows in (a,b,c):assert len(rows)==30 and all(r['success'] for r in rows)
        assert [(r['query_id'],r['repeat']) for r in a]==[(r['query_id'],r['repeat']) for r in b]
        datasets[mid]=(a,b)
        metrics={}
        for key in [*METRICS,'strict_audit_s']:
            metrics[key]={}
            for split,pred in [('all',lambda r:True),('first',lambda r:r['repeat']==1),('repeat',lambda r:r['repeat']>1),('A03',lambda r:r['query_id']=='A2B-03')]:
                av=stats([measure(r,key) for r in a if pred(r)]);bv=stats([measure(r,key) for r in b if pred(r)])
                pct={stat:None if av[stat] in (None,0) or bv[stat] is None else 100*(bv[stat]/av[stat]-1) for stat in ('median','p95','max')}
                metrics[key][split]={'control':av,'r2':bv,'change_percent':pct}
        for left,right in zip(a,b):
            la=left.get('timing',{}).get('activation_attempts',[])
            ra=right.get('timing',{}).get('activation_attempts',[])
            if la and ra:
                for key in ('rectangle_band_candidates','rectangle_band_certified','rectangle_rejected_cells','rectangle_certified_cells'):
                    assert [x.get(key) for x in la]==[x.get(key) for x in ra],(mid,left['query_id'],key)
        for r in b:
            case=f"{r['query_id']}_{r['repeat']}";item={'map':mid,'case':case}
            for name,base in [('control',control),('r1',archive)]:
                for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy'):
                    item[name+'_'+f+'_same']=sha(base/'cases'/case/f)==sha(formal/'cases'/case/f)
                    assert item[name+'_'+f+'_same'],item
            checks.append(item)
        pa,pb=[json.loads((d/'protocol.json').read_text()) for d in (control,formal)]
        for key in ('map_sha256','vehicle','L2_resolution_m','L3_normal_radius_m','L3_corner_radius_m','max_search_cells','max_map_fraction','CPU'):
            # semantic protocol uses lowercase l2 keys
            if key in pa:assert pa[key]==pb[key],(mid,key)
        for key in ('input_sha256','queries_sha256','parking_access_sha256','query_ids'):
            if key in pa:assert pa[key]==pb[key],(mid,key)
        def libs(directory):
            value=json.loads((directory/'loaded_nav2_libraries.json').read_text())
            return {k:v for process in value['processes'] for k,v in process['libraries'].items()}
        assert libs(control)==libs(formal)==libs(archive),(mid,'native planner/costmap libraries changed')
        for r in b:
            log=r.get('r1_segmented_recovery',{})
            if log.get('attempted'):
                assert log['success'] and log['wall_ms']<=log['budget_s']*1000
                assert len(log['sections'])<=4 and not log['global_fallback']
                assert all(s['success'] and s['diagnostics']['costmap_update_acknowledged'] and s['diagnostics']['costmap_ack_mismatch_cells']==0 for s in log['sections'])
        snapshots=[]
        for directory in (control,formal):
            hashes=json.loads((directory/'source_snapshot/hashes.json').read_text())
            sources=[(f,v) for f,v in hashes.items() if '/src/' in f]
            assert all(sha(Path(f))==v for f,v in sources)
            snapshots.append({'directory':str(directory),'planning_source_count':len(sources),'source_snapshot_sha256':sha(directory/'source_snapshot/hashes.json')})
        summaries[mid]={'label':label,'control_success':30,'r2_success':30,'metrics':metrics,
            'control_right':stats([r.get('right_side_fraction') for r in a]),'r2_right':stats([r.get('right_side_fraction') for r in b]),
            'control_length':stats([r.get('length_m') for r in a]),'r2_length':stats([r.get('length_m') for r in b]),
            'runs_sha256':{label:sha(d/'runs.jsonl') for label,d in [('control',control),('r2',formal),('r1',archive)]},
            'source_snapshot_checks':snapshots,
            'actual_backend_calls':{name:sum(r.get('l3',{}).get('backend_call_count',1) for r in rows) for name,rows in [('control',a),('r2',b)]}}
    lock=json.loads((ROOT/'L1_LOCK.json').read_text())
    locked_files=[{'file':f,'same':sha(ROOT/f)==value} for part in ('source_files','input_files','pixel_input_files') for f,value in lock[part].items()]
    assert all(x['same'] for x in locked_files)
    def audit_src(path):
        return [ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='_audit']
    for f in ['src/three_yd_v4/controller.py','src/three_yd_v4/r5_controller.py','src/three_yd_v5/controller.py']:
        assert audit_src(ROOT/f)==audit_src(PARENT/f)
    journals=[]
    for p in sorted((ROOT/'results').rglob('*.supervisor.json')):
        v=json.loads(p.read_text());assert v['state']=='complete' and v['cleanup']['complete'],str(p)
        journals.append({'file':str(p),'state':v['state'],'cleanup':v['cleanup']})
    diagnostics={}
    ref=read(ROOT/'results/control/arena_large_250x300_2d_005/runs.jsonl')
    for mode in ('A','B','AB'):
        folder=ROOT/'results/diagnostic'/f'{mode}_A03';rows=read(folder/'runs.jsonl')
        assert len(rows)==3 and all(r['success'] for r in rows)
        for r in rows:
            tag=f"A2B-03_{r['repeat']}"
            for f in ('l1_output.json','l2_reference_xy.npy','l2_soft_target.json','path.npy'):
                assert sha(folder/'cases'/tag/f)==sha(ROOT/'results/control/arena_large_250x300_2d_005/cases'/tag/f)
        diagnostics[mode]={'strict_success':3,'sample_n':3,'complete_map_benchmark':False,'source_sha256':sha(folder/'runs.jsonl'),
                          'first_ms':rows[0]['wall_ms'],'repeat_ms':[r['wall_ms'] for r in rows[1:]],
                          'first_L2_ms':rows[0]['recovery']['all_candidates_controller_ms'],
                          'recovery_ms':[r['r1_segmented_recovery']['wall_ms'] for r in rows]}
    verification={'L1_locked_files':locked_files,'outputs':checks,'L1_equal':60,'L2_equal':60,'soft_equal':60,'final_path_equal':60,
                  'archive_r1_all_outputs_equal':True,'strict_audit_AST_unchanged':True,'supervisors':journals}
    (ROOT/'report/COMPARISON.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2)+'\n')
    (ROOT/'report/VERIFICATION.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n')
    (ROOT/'report/ABLATIONS.json').write_text(json.dumps(diagnostics,ensure_ascii=False,indent=2)+'\n')
    render(datasets)
    print(json.dumps({mid:{'e2e':v['metrics']['e2e_s']['all'],'RSS':v['metrics']['tree_rss_gib']['all']} for mid,v in summaries.items()},indent=2))


def render(data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
    font_manager.fontManager.addfont(font)
    assert font_manager.FontProperties(fname=font).get_name()=='SimSun'
    plt.rcParams.update({'font.family':'SimSun','axes.unicode_minus':False,'font.size':12})
    panels=[('e2e_s','端到端耗时 / s'),('l2_s','L2控制器处理 / s'),('l3_s','L3流水线（含适配）/ s'),('tree_rss_gib','查询进程树峰值RSS / GiB')]
    fig,axes=plt.subplots(2,4,figsize=(19,10))
    for row,(mid,label) in enumerate(MAPS):
        a,b=data[mid]
        for col,(metric,title) in enumerate(panels):
            ax=axes[row,col];ax.boxplot([[measure(r,metric) for r in x] for x in (a,b)],labels=['r1逻辑\nn=30','r2 AB\nn=30'],whis=1.5,showfliers=True)
            ax.set_title(label+'\n'+title);ax.grid(axis='y',alpha=.2)
    fig.suptitle('V8-r2：同配置控制组与优化组，两图各10组×3次',fontsize=18,fontweight='bold')
    fig.text(.5,.02,'箱体Q1～Q3；中线中位数；须为1.5×IQR内最远样本；圆点离群值。线性坐标。\n排除地图加载、ROS启动和导出；各阶段相互包含，不能相加中位数。',ha='center',fontsize=12)
    fig.tight_layout(rect=[0,.08,1,.93]);fig.savefig(ROOT/'report/r1_r2_boxplots.png',dpi=180);plt.close(fig)
    # Three formal samples of the bottleneck cannot support strong generalization.
    a,b=data['arena_large_250x300_2d_005'];fig,axes=plt.subplots(1,2,figsize=(11,5))
    for ax,(key,title) in zip(axes,[('e2e_s','A2B-03端到端 / s'),('l3_s','A2B-03 L3流水线 / s')]):
        for x,rows,label,color in [(0,a,'r1逻辑','#476c9b'),(1,b,'r2 AB','#c27828')]:
            rs=[r for r in rows if r['query_id']=='A2B-03'];vals=[measure(r,key) for r in rs]
            for j,(r,v) in enumerate(zip(rs,vals)):
                xpos=x+([0.,-.045,.045][j])
                ax.scatter([xpos],[v],color=color,marker=['o','s','D'][j],s=55)
                ax.annotate('首次' if r['repeat']==1 else '重复'+str(r['repeat']), (xpos,v),xytext=(7,[3,12,-16][j]),textcoords='offset points',fontsize=10)
        allvals=[measure(r,key) for rows in (a,b) for r in rows if r['query_id']=='A2B-03']
        ax.set_ylim(min(allvals)*.7,max(allvals)*1.1)
        ax.set_xticks([0,1]);ax.set_xticklabels(['r1逻辑（n=3）','r2 AB（n=3）']);ax.set_xlim(-.45,1.75);ax.set_title(title);ax.grid(axis='y',alpha=.2)
    fig.suptitle('恢复案例逐点对照：首次与重复分别标记',fontsize=17,fontweight='bold')
    fig.text(.5,.02,'每组n=3，含1次首次和2次重复；横向错开仅用于辨认样本。',ha='center')
    fig.tight_layout(rect=[0,.08,1,.9]);fig.savefig(ROOT/'report/A03_points.png',dpi=180);plt.close(fig)

if __name__=='__main__':main()
