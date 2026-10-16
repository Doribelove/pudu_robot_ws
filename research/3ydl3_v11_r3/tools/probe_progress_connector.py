"""Offline actual-map connector diagnostics; excluded from formal timing."""
import argparse,json
from pathlib import Path
from types import SimpleNamespace as S
from dataclasses import replace
import numpy as np
from arena_3d_v1.semantic_world import SemanticWorld,json_write
from arena_evaluation.planner_benchmark.models import Query
from three_yd_v5.free_space_topology import FreeSpaceTopology,load_map_only_world
from three_yd_v5.topology_default import DefaultTopology
from three_yd_v5.controller import V5Traffic,FreeSpaceTraffic
from three_yd_v5.progress_connector import propose
from run_free_full import registered_input
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/home/robot/workspaces/3yd_v8_r9_progress_20261008')
def main():
 a=argparse.ArgumentParser();a.add_argument('--map-id',required=True);args=a.parse_args();mid=args.map_id
 out=ROOT/'report'/('connector_probe_'+mid);out.mkdir(exist_ok=False)
 if mid=='mengjia':
  w=SemanticWorld(ROOT/'inputs/corroborated_map.yaml',ROOT/'inputs/semantic.json',cache_root=out/'world')
  g=DefaultTopology(w,ROOT/'inputs/topology.json',out/'graph');qs=json.loads((ROOT/'inputs/queries.json').read_text())['scenarios']
 else:
  d,_,qs=registered_input(mid);w=load_map_only_world(d/'map.yaml',out/'world');g=FreeSpaceTopology(w,out/'graph')
 records=[]
 for qd in qs:
  def pose(v):return tuple(v[k] for k in ('x','y','yaw_rad')) if isinstance(v,dict) else tuple(v)
  q=Query(qd['id'],pose(qd['start']),pose(qd['goal']))
  case=PARENT/'results/moving/candidate'/mid/'cases'/f'{q.query_id}_initial_1'
  p=np.load(case/'path.npy');mask=np.load(case/'l3_scope.npz')['mask']
  saved=json.loads((case/'l1_output.json').read_text())
  routes=g.routes(q,limit=3 if mid=='mengjia' else 1)
  route=next((r for r in routes if r.signature==saved.get('signature',saved.get('route_signature'))),None)
  if route is None:
   # Exact chosen reference is the decisive observable for frozen L1 output.
   route=next(r for r in g.routes(q,limit=3 if mid=='mengjia' else 1) if g.channel_plan(r)['right_reference_polyline']==saved['right_reference_polyline'])
  traffic=V5Traffic(w,g,route) if mid=='mengjia' else FreeSpaceTraffic(w,g,route)
  arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))]
  for step,fraction in [('p20',.2),('p50',.5),('p80',.8)]:
   i=int(np.searchsorted(arc,arc[-1]*fraction));v=p[i].copy();sign=-1 if step=='p50' else 1
   v[:2]+=sign*.03*np.array([-np.sin(v[2]),np.cos(v[2])]);v[2]=(v[2]+sign*np.deg2rad(2)+np.pi)%(2*np.pi)-np.pi
   service=S(remaining=p[i:],prefix=p[:i],base_traffic=traffic,world=w,controller=S(_target_mask=lambda:mask))
   value,reason=propose(service,replace(q,start=tuple(v)))
   records.append({'query':q.query_id,'step':step,'success':value is not None,'reason':reason,'details':None if value is None else value[2]})
   if value is not None:np.save(out/(q.query_id+'_'+step+'.npy'),value[1])
 json_write(out/'summary.json',{'records':records,'success':sum(v['success'] for v in records),'n':len(records),'formal_performance':False})
 print(mid,sum(v['success'] for v in records),len(records),[v for v in records if not v['success']])
if __name__=='__main__':main()
