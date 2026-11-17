#!/usr/bin/env python3
"""One real P6 request on a generated map, no robot drivers. External 60s bound."""
import os,sys,time,json,hashlib,traceback,tempfile,shutil,argparse
from pathlib import Path
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert os.environ.get('ROS_DOMAIN_ID')=='179'
from trilayer_p6_runtime import activate,RuntimeConfig
ap=argparse.ArgumentParser()
ap.add_argument('output',nargs='?')
ap.add_argument('--case',choices=['straight','shifted','small_budget','obstacle','forced_sqp'],default='straight')
ap.add_argument('--solver-root',type=Path,required=True)
args=ap.parse_args()
out=Path(args.output) if args.output else Path(tempfile.mkdtemp(prefix='trilayer-p6-'+args.case+'-'))/'run'
out.mkdir(parents=True,exist_ok=False)
assert args.solver_root.is_absolute() and args.solver_root.is_dir()
sys.path.insert(0,str(args.solver_root))
import osqp,qdldl,clarabel
solver_sources={n:str(Path(m.__file__).resolve()) for n,m in [('osqp',osqp),('qdldl',qdldl),('clarabel',clarabel)]}
assert all(Path(f).is_relative_to(args.solver_root.resolve()) for f in solver_sources.values()),solver_sources
print('SOLVER_SOURCES',json.dumps(solver_sources),flush=True)
resource=None
if args.case=='forced_sqp':
 from ament_index_python.packages import get_package_share_directory
 resource=out/'test-only-resources'
 shutil.copytree(Path(get_package_share_directory('trilayer_p6_runtime'))/'resources',resource)
 cfgfile=resource/'p6/base/runtime/base/config/l3.json'
 cfg=json.loads(cfgfile.read_text());assert cfg['certified_quadratic_start'] is True
 cfg['certified_quadratic_start']=False
 cfgfile.write_text(json.dumps(cfg,indent=2)+'\n')
 print('TEST_ONLY_OVERRIDE certified_quadratic_start=false; default unchanged',flush=True)
import numpy as np
from PIL import Image
import yaml
size=160 if args.case=='small_budget' else 240
pixels=np.full((size,size),254,np.uint8);pixels[:4,:]=0;pixels[-4:,:]=0;pixels[:,:4]=0;pixels[:,-4:]=0
if args.case=='obstacle': pixels[145:175,65:75]=0
Image.fromarray(pixels).save(out/'map.pgm')
mp=out/'map.yaml';mp.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
from trilayer_p6_runtime.api import MapOnlyRequest,plan_map
start,goal=((3.,6.,0.),(7.,6.,0.)) if args.case=='shifted' else ((2.,4.,0.),(5.,4.,0.))
request=MapOnlyRequest(mp,start,goal,out/'runtime',query_id=args.case)
data=plan_map(request,resource_root=resource,solver_root=args.solver_root)
result=data['result'];failure=data['exception'];errors=data['cleanup_errors']
data.update(case=args.case,test_overrides={'certified_quadratic_start':False} if args.case=='forced_sqp' else {})
(out/'result.json').write_text(json.dumps(data,indent=2))
print('PUBLIC_API_RESULT',data['success'],'points',len(data['path']),'worker_exit',data['worker_exit'],flush=True)
if failure or errors:raise SystemExit(1)
if args.case=='small_budget':
 assert result.get('success') is False and result.get('failure_code')=='L3_SPATIAL_BUDGET'
 assert data['worker_exit']==0
 raise SystemExit(0)
if not result.get('success'):raise SystemExit(1)
if args.case in ('forced_sqp','obstacle'):
 curve=result.get('curve_l3',{})
 assert curve.get('iterations'), 'SQP branch was not executed'
 assert all(i.get('qp_solver')=='clarabel' and i.get('status')=='Solved' for i in curve['iterations'])
 assert curve.get('certificate',{}).get('max_curvature_bound_1pm',float('inf'))<=2.5
 assert curve.get('accepted_quadratic_initialization') is False
assert data['worker_exit']==0
assert result.get('independent_audit',{}).get('valid') is True
assert data['success'] and data['path']
assert all(len(p)==3 for p in data['path'])
assert np.allclose(data['path'][0],start,atol=1e-5)
assert np.allclose(data['path'][-1],goal,atol=1e-5)
assert any(a.get('costmap_ack_status')=='cold_exact_content_verified' and a.get('costmap_ack_stable_observations',0)>=2 for a in data['ack_events'])
assert any(a.get('costmap_ack_status')=='exact_effective_master_content_verified' and a.get('costmap_ack_mismatch_cells')==0 and a.get('semantic_exact_stable_observations',0)>=2 for a in data['ack_events'])
