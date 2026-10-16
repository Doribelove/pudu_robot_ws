"""Explicit diagnostic tasks on a shared road, distinct starts/goals and IDs."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r11_firstcost_20261009')
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=ROOT/'datasets/v9_shared_segments_v2')
 folder=parser.parse_args().output;folder.mkdir(parents=True,exist_ok=False)
 for mid,qid in [('mengjia','Q03'),('arena_large_250x300_2d_005','A2B-01')]:
  source=PARENT/'results/accepted_fixed/candidate'/mid/'cases'/(qid+'_1')/'path.npy'
  p=np.load(source);d=np.diff(p,axis=0);ds=np.linalg.norm(d[:,:2],axis=1)
  yaw=(d[:,2]+np.pi)%(2*np.pi)-np.pi
  okay=(ds>1e-8)&(abs(yaw)/np.maximum(ds,1e-8)<.025)
  starts=np.flatnonzero(okay&np.r_[True,~okay[:-1]]);ends=np.flatnonzero(okay&np.r_[~okay[1:],True])+1
  a,b=max(zip(starts,ends),key=lambda v:ds[v[0]:v[1]].sum())
  arc=np.r_[0.,np.cumsum(ds)];length=arc[b]-arc[a];assert length>6,(mid,length)
  original=json.loads((ROOT/('inputs/queries.json' if mid=='mengjia' else 'inputs/'+mid+'/queries.json')).read_text())
  rows=original['scenarios'] if isinstance(original,dict) and 'scenarios' in original else original
  if isinstance(rows,dict):rows=rows.get('queries',[])
  q=next(v for v in rows if v['id']==qid);queries=[q]
  def pose(v):return dict(zip(('x','y','yaw_rad'),v.tolist())) if mid=='mengjia' else v.tolist()
  pairs=[]
  for j in range(3):
   x=int(np.searchsorted(arc,arc[a]+.2+j*.3));y=int(np.searchsorted(arc,arc[b]-.2-j*.3))
   assert y>x
   assert (x,y) not in pairs; pairs.append((x,y))
   queries.append({'id':f'SHARED-{j+1:02d}','start':pose(p[x]),'goal':pose(p[y]),
                  'diagnostic':'distinct task along previously verified road; not fixed10 acceptance'})
  assert len({json.dumps(v['start'],sort_keys=True) for v in queries})==4
  assert len({json.dumps(v['goal'],sort_keys=True) for v in queries})==4
  data={'schema':'V9-shared-road-diagnostic-v2','map_id':mid,'source_path':str(source),
        'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'straight_length_m':float(length),
        'source_indices':[int(a),int(b)],'task_source_indices':pairs,'queries':queries,'fixed10':False}
  (folder/(mid+'.json')).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
  print(mid,length)
if __name__=='__main__':main()
