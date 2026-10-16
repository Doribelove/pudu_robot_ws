"""Offline geometric diagnostics only: never a full navigation success."""
import argparse,json,os,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np


def load(path):
    path=Path(path);meta=json.loads(path.with_suffix('.json').read_text())
    with np.load(path) as data:arrays={k:data[k] for k in data.files}
    world=SimpleNamespace(hard=arrays['hard'],vehicle=tuple(meta['vehicle']),
          map=SimpleNamespace(resolution=meta['resolution'],origin=tuple(meta['origin'])))
    return world,arrays,meta


def run(path,changes,budget=15.):
    from three_yd_v5.curve_l3 import optimize
    world,arrays,meta=load(path)
    config=dict(meta['config'],**changes)
    start=time.monotonic()
    result=optimize(world,arrays['mask'],arrays['guide'],arrays['start'],arrays['goal'],start+budget,config)
    return result,{'source':str(path),'config_changes':changes,'geometric_success':result[0] is not None,
                   'full_navigation_verified':False,'elapsed_s':time.monotonic()-start,'log':result[2]}


if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('input',type=Path);a.add_argument('--changes',default='{}')
    a.add_argument('--budget',type=float,default=15.);a.add_argument('--output',required=True,type=Path)
    args=a.parse_args();os.sched_setaffinity(0,{14})
    args.output.mkdir(parents=True,exist_ok=False)
    result,record=run(args.input,json.loads(args.changes),args.budget)
    (args.output/'result.json').write_text(json.dumps(record,indent=2)+'\n')
    if result[1] is not None:(args.output/'curve.json').write_text(json.dumps(result[1])+'\n')
    if result[0] is not None:np.save(args.output/'path.npy',result[0])
    print({k:v for k,v in record.items() if k!='log'},flush=True)
