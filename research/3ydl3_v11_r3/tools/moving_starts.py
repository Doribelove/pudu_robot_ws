"""Immutable diagnostic queries: fixed goal, start pose on the parent route.

Each fixed route has its initial request followed by 20/50/80% arc positions.
These are different queries, NOT fixed-query repeats or vehicle execution.
"""
import copy
import hashlib
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r7_fullroute_20261008')
MAPS=('mengjia','arena_large_250x300_2d_005')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def build():
    target=ROOT/'datasets/moving_start_v1';target.mkdir(parents=True,exist_ok=True)
    for mid in MAPS:
        source=ROOT/'inputs/queries.json' if mid=='mengjia' else ROOT/'inputs'/mid/'queries.json'
        key='scenarios' if mid=='mengjia' else 'queries'
        original=json.loads(source.read_text())[key];rows=[];paths={}
        for item in original:
            p=PARENT/'results/final'/mid/'cases'/f"{item['id']}_1"/'path.npy'
            points=np.load(p);arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1))]
            paths[str(p)]=sha(p)
            for step,fraction in [('initial',0.),('p20',.2),('p50',.5),('p80',.8)]:
                index=0 if fraction==0 else int(np.searchsorted(arc,fraction*arc[-1]))
                row=copy.deepcopy(item)
                if fraction:
                    row['start']=dict(zip(('x','y','yaw_rad'),points[index].tolist())) if mid=='mengjia' else points[index].tolist()
                row['moving_start']={'step':step,'kind':'initial' if fraction==0 else 'moving',
                    'parent_query':item['id'],'fraction_requested':fraction,'parent_index':index,
                    'actual_arc_m':float(arc[index]),'parent_length_m':float(arc[-1]),
                    'parent_path':str(p),'goal_fixed':True,'yaw_from_saved_trajectory':fraction!=0}
                rows.append(row)
        document={'schema':'3YD-V8-moving-start-v1','map_id':mid,'scenarios':rows,
            'fixed_queries_file':str(source),'fixed_queries_sha256':sha(source),'parent_paths_sha256':paths,
            'purpose':'different starts with fixed goal; stationary planning replay; no real tracking'}
        f=target/(mid+'.json')
        if f.exists() and f.read_text()!=json.dumps(document,ensure_ascii=False,indent=2)+'\n':raise ValueError('REFUSE_DATASET_OVERWRITE')
        f.write_text(json.dumps(document,ensure_ascii=False,indent=2)+'\n')
        print(f,len(rows))


def load(path,map_id):
    d=json.loads(Path(path).read_text())
    assert d['schema']=='3YD-V8-moving-start-v1' and d['map_id']==map_id
    assert sha(d['fixed_queries_file'])==d['fixed_queries_sha256']
    for p,h in d['parent_paths_sha256'].items():assert sha(p)==h
    originals=json.loads(Path(d['fixed_queries_file']).read_text())
    originals={q['id']:q for q in originals.get('scenarios',originals.get('queries'))}
    assert len(d['scenarios'])==40
    for row in d['scenarios']:
        meta=row['moving_start'];orig=originals[row['id']]
        assert row['goal']==orig['goal']
        if meta['kind']=='initial':assert row['start']==orig['start']
        else:
            pose=np.load(meta['parent_path'])[meta['parent_index']]
            start=row['start'];start=list(start[k] for k in ('x','y','yaw_rad')) if isinstance(start,dict) else start
            assert np.array_equal(pose,np.asarray(start))
    return d['scenarios']

if __name__=='__main__':build()
