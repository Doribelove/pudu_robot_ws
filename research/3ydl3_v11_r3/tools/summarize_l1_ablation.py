"""Read formal individual calls; compare output and same-mode memory scope."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from run_l1_ablation import SEM, FREE

ROOT = Path(__file__).resolve().parents[1]
NAMES = dict(zip(SEM+FREE, ['万达2w地库','万达3w地库','合成地库A','合成地库B','合成地库C',
                          'arena_medium','mentor','mentor_4x','arena_large']))


def stats(values):
    x = np.asarray(values, float)
    return {k: float(v) for k, v in {
        'n': len(x), 'median': np.median(x), 'p95': np.quantile(x,.95),
        'q1': np.quantile(x,.25), 'q3': np.quantile(x,.75), 'max': x.max()}.items()}


def collect(root, cohort, map_id):
    mode = 'semantic' if map_id in SEM else 'free_space'
    wall, cpu, rss, peak, init, first, valid, records, outputs, digests = [],[],[],[],[],[],[],{}, {},{}
    for i in range(1,4):
        folder = root/cohort/mode/map_id/f'round_{i}'
        if mode == 'semantic':
            b = json.loads((folder/'batch.json').read_text())
            setup = json.loads((folder/'initialization.json').read_text())
            rows=b['queries'];init.append(setup['wall_ms'])
            peak.append(b['final_resources']['peak_rss_bytes']/2**20)
        else:
            b=json.loads((folder/'runs.json').read_text());rows=b['queries']
            init.append(b['world_init_ms']+b['l1_graph_init_ms'])
            peak.append(b['memory_final']['peak_rss_bytes']/2**20)
        first.append(rows[0]['wall_ms'])
        for row in rows:
            wall.append(row['wall_ms']);cpu.append(row['cpu_ms']);valid.append(row['success'])
            rss.append((row['resources_after'] if mode=='semantic' else row['memory_after'])['rss_bytes']/2**20)
            key=f'{i}/{row["query_id"]}';records[key]=row
            outputs[key]=json.loads((folder/(row['query_id']+'_output.json')).read_text())['candidates']
        for p in folder.glob('*.json'):
            digests[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
    assert len(wall)==30
    return {'map_id':map_id,'map_name':NAMES[map_id],'mode':mode,'cohort':cohort,
            'valid':sum(valid),'n':len(valid),'wall_ms':stats(wall),'cpu_ms':stats(cpu),
            'after_query_rss_mib':stats(rss),'peak_rss_mib':stats(peak),'init_ms':stats(init),
            'first_call_ms':stats(first),'raw_wall_ms':wall,'raw_rss_mib':rss,'raw_peak_mib':peak,
            'records':records,'outputs':outputs,'data_sha256':digests}


def reduction(old, new):
    return (old-new)/old*100 if old else 0


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rows=[];changes=[];valid_total=0;equal_total=0
    for mid in SEM+FREE:
        mode='semantic' if mid in SEM else 'free_space'
        control=collect(args.input,'control',mid)
        candidates=['control','B','AB'] if mode=='semantic' else ['control','A','AB']
        groups=[]
        for c in candidates:
            new=collect(args.input,c,mid)
            equal=sum(new['outputs'][k]==control['outputs'][k] for k in control['outputs'])
            if equal != 30: changes.append({'map_id':mid,'cohort':c,'changed':[
                k for k in control['outputs'] if new['outputs'][k]!=control['outputs'][k]]})
            new['same_output_as_control']=equal
            if c=='AB':valid_total+=new['valid'];equal_total+=equal
            new['reduction_vs_control_percent']={
                'median':reduction(control['wall_ms']['median'],new['wall_ms']['median']),
                'p95':reduction(control['wall_ms']['p95'],new['wall_ms']['p95']),
                'peak_rss':reduction(control['peak_rss_mib']['median'],new['peak_rss_mib']['median'])}
            # Detailed outputs remain in originals; keep report JSON compact.
            new.pop('outputs');groups.append(new)
        rows.append({'map_id':mid,'map_name':NAMES[mid],'groups':groups})
    result={'final_valid':valid_total,'final_output_equal':equal_total,'total':270,
            'changes':changes,'maps':rows}
    (args.output/'STATISTICS.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    lines=['# 3YD-V7-r1 L1 独立对照','',
           f'组合版 L1 有效输出 {valid_total}/270；候选顺序、语义区域序列和完整引导线与原版一致 {equal_total}/270。', '',
           '每张地图固定10组，每组在3个独立进程中各查询一次；各组串行执行，固定CPU12。计时为L1路线搜索及输出几何生成，排除初始化、读图和输出验收。一次调用按原协议获取至多3条候选；无语义模式返回1条。', '',
           '| 地图 | 查询中位数 r0→r1（ms） | 下降 | P95 r0→r1（ms） | 下降 | 独立进程峰值RSS中位数 r0→r1（MiB） |',
           '|---|---:|---:|---:|---:|---:|']
    for r in rows:
        old,new=r['groups'][0],r['groups'][-1];d=new['reduction_vs_control_percent']
        lines.append(f"| {r['map_name']} | {old['wall_ms']['median']:.3f} → {new['wall_ms']['median']:.3f} | {d['median']:.1f}% | {old['wall_ms']['p95']:.3f} → {new['wall_ms']['p95']:.3f} | {d['p95']:.1f}% | {old['peak_rss_mib']['median']:.1f} → {new['peak_rss_mib']['median']:.1f} |")
    lines+=['','正数表示下降，负数表示增加。中位数和P95来自30次调用；峰值RSS来自3次独立进程启动，包含解释器、地图和图缓存，不能直接理解为算法净内存。语义模式只加载语义几何，无语义模式需要加载完整PGM，因此不以两种模式的RSS差异评价算法效率。','','## 单独优化对照','',
            '| 地图 | 原版 | 单独优化 | 组合版 | 单位 |','|---|---:|---:|---:|---|']
    for r in rows:
        a,b,c=r['groups']
        lines.append(f"| {r['map_name']} | {a['wall_ms']['p95']:.3f} | {b['wall_ms']['p95']:.3f}（{b['cohort']}） | {c['wall_ms']['p95']:.3f} | P95 ms |")
    lines+=['','A：无语义CSR目标停止搜索。B：语义备选搜索共享路径前缀与静态坐标，仅在选中候选时生成几何数组。AB：两项同时启用；两项分属不同输入模式，各自仅影响对应模式。','','## 初始化和首条调用','',
            '| 地图 | 初始化 r0→r1（ms） | 首条调用 r0→r1（ms） |','|---|---:|---:|']
    for r in rows:
        a,c=r['groups'][0],r['groups'][-1]
        lines.append(f"| {r['map_name']} | {a['init_ms']['median']:.1f} → {c['init_ms']['median']:.1f} | {a['first_call_ms']['median']:.3f} → {c['first_call_ms']['median']:.3f} |")
    lines+=['','首条调用是固定Q01/A2B-01，样本数3，不能代表全部冷查询。不同起终点不当作缓存命中重查。本轮不改变缓存协议。', '',
            '## 验收范围','',
            'L1有效输出只代表区域路线和引导线通过既定L1验收；本轮九图270次为L1-only，不计作完整三层导航成功率。路径输出没有变化时，下游收到的几何和语义任务保持相同，但本轮不重新宣称三层全量验收。', '',
            '## 原始记录','',str(args.input),'',
            '逐次时间、RSS、候选输出、输入校验、源文件记录与运行日志全部保留；精确统计见同目录STATISTICS.json。']
    (args.output/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    print('valid',valid_total,'equal',equal_total,'changes',changes)
    for r in rows:
        a,b=r['groups'][0],r['groups'][-1]
        print(r['map_name'],a['wall_ms']['median'],b['wall_ms']['median'],a['wall_ms']['p95'],b['wall_ms']['p95'])


if __name__=='__main__':main()
