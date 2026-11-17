# SPDX-License-Identifier: Apache-2.0
import argparse,json,math,os,sys,tempfile
from pathlib import Path
from .demo import make_demo
from .process import run_bounded
from .environment import check_environment


def parser():
    p=argparse.ArgumentParser(description='TriLayer P6 map-only global planning. Starts an isolated Nav2 worker, never a robot controller.')
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--map',type=Path,dest='map_yaml')
    source.add_argument('--demo',choices=['straight','obstacle','small_budget'])
    p.add_argument('--start',nargs=3,type=float,metavar=('X','Y','YAW'),default=None)
    p.add_argument('--goal',nargs=3,type=float,metavar=('X','Y','YAW'),default=None)
    p.add_argument('--domain-id',type=int,required=True,help='Dedicated unused ROS DDS domain (1..232); localhost only')
    p.add_argument('--output',type=Path,required=True,help='New output directory; existing directories are rejected')
    p.add_argument('--timeout',type=float,default=60.,help='Whole worker bound, 1..120s plus bounded cleanup')
    p.add_argument('--solver-root',type=Path,help='Optional explicit pre-provisioned solver directory; origins are recorded')
    p.add_argument('--frame',choices=['map'],default='map')
    return p


def main(argv=None):
    p=parser();args=p.parse_args(argv)
    try:check_environment(os.environ)
    except ValueError as exc:p.error(str(exc))
    if args.map_yaml and (args.start is None or args.goal is None):p.error('--map requires explicit --start and --goal')
    args.start=args.start or (2.,4.,0.);args.goal=args.goal or (5.,4.,0.)
    if not 1<=args.domain_id<=232:p.error('domain-id must be 1..232 and reserved for this invocation')
    if not math.isfinite(args.timeout) or not 1<=args.timeout<=120:p.error('timeout must be 1..120 seconds')
    if not all(math.isfinite(v) for v in (*args.start,*args.goal)):p.error('start/goal must be finite')
    if args.map_yaml and not args.map_yaml.is_file():p.error('map YAML does not exist')
    if args.solver_root and not args.solver_root.is_dir():p.error('solver root does not exist')
    out=args.output.resolve()
    try:out.mkdir(parents=True,exist_ok=False)
    except FileExistsError:p.error('output directory already exists; choose a new directory')
    except OSError as exc:p.error('cannot create output directory: '+str(exc))
    mp=make_demo(out/'input',args.demo) if args.demo else args.map_yaml.resolve()
    request={'map_yaml':str(mp),'start':args.start,'goal':args.goal,'output_dir':str(out/'run'),
             'master_service':'/global_costmap/trilayer_get_costmap','vehicle':[.265,.225,.4],
             'solver_root':str(args.solver_root.resolve()) if args.solver_root else None,
             'frame':'map','profile':'p6','arm':'H-I','demo':args.demo}
    request_path=out/'request.json';request_path.write_text(json.dumps(request,indent=2)+'\n')
    env=dict(os.environ);env['ROS_LOCALHOST_ONLY']='1';env['ROS_DOMAIN_ID']=str(args.domain_id)
    env['PYTHONNOUSERSITE']='1';env['PYTHONDONTWRITEBYTECODE']='1'
    env['OMP_NUM_THREADS']='1';env['OPENBLAS_NUM_THREADS']='1'
    execution=run_bounded([sys.executable,'-m','trilayer_global_planner.worker',str(request_path),str(out/'runtime_result.json')],
                          out/'worker.log',timeout=args.timeout,env=env,cwd=out)
    (out/'execution.json').write_text(json.dumps(execution,indent=2)+'\n')
    runtime_result_path=out/'runtime_result.json'
    result=json.loads(runtime_result_path.read_text()) if runtime_result_path.exists() else {}
    success=bool(result.get('success')) and execution['returncode']==0 and not execution['remaining_processes'] and not execution['timed_out'] and not execution['interrupted']
    result['runtime_success']=bool(result.get('success'))
    result['success']=success;result['execution']=execution
    if not success and result.get('path'):
        result['unaccepted_path']=result['path'];result['path']=[]
    if execution['timed_out']:result['failure_code']='WORKER_TIMEOUT'
    elif execution['interrupted']:result['failure_code']='CANCELLED'
    elif execution['remaining_processes']:result['failure_code']='CLEANUP_INCOMPLETE'
    elif not runtime_result_path.exists():result['failure_code']='WORKER_EXIT_WITHOUT_RESULT'
    result.setdefault('path',[]);result.setdefault('frame_id','map')
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    summary={'success':success,'output':str(out),'path_points':len(result.get('path',[])),
             'failure_code':result.get('failure_code'),**execution}
    print(json.dumps(summary,ensure_ascii=False))
    if execution['interrupted']:return 130
    if execution['timed_out']:return 124
    return 0 if success else 1
