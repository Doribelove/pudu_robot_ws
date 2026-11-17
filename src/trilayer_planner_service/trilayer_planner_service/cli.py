# SPDX-License-Identifier: Apache-2.0
import argparse,json,os,sys
from pathlib import Path
from .process import run_bounded

def main(argv=None):
    p=argparse.ArgumentParser(description='Local persistent P6 snapshot service. No robot controller.')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,required=True)
    p.add_argument('--solver-root',type=Path)
    p.add_argument('--request-timeout',type=float,default=55.)
    p.add_argument('--lifetime',type=float,default=300.,help='Overall seconds, or explicit 0 for persistent service; request deadline always enforced')
    p.add_argument('--min-free-mib',type=int,default=512,help='Refuse new requests below this free-space floor; minimum 512 MiB')
    a=p.parse_args(argv)
    if a.min_free_mib<512:p.error('min-free-mib must be at least 512')
    if not 1<=a.domain_id<=232:p.error('Reserve a nondefault dedicated child DDS domain')
    if not 1<=a.request_timeout<=60 or not (a.lifetime==0 or 5<=a.lifetime<=3600):p.error('request timeout 1..60; lifetime 0 (persistent) or 5..3600 seconds')
    for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI','ROS_DISCOVERY_SERVER','ROS_STATIC_PEERS','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_FASTRTPS_USE_QOS_FROM_XML','ROS_SECURITY_ENABLE','ROS_SECURITY_STRATEGY','ROS_SECURITY_KEYSTORE','ROS_SECURITY_ENCLAVE_OVERRIDE','ROS_NAMESPACE','LD_PRELOAD','PYTHONHOME'):
        if os.environ.get(key):p.error('Unsupported environment override: '+key)
    out=a.output.resolve()
    if len(str(out/'service.sock').encode())>=104:p.error('output path too long for Unix socket')
    if a.solver_root and not a.solver_root.is_dir():p.error('solver root missing')
    try:out.mkdir(mode=0o700,parents=True,exist_ok=False)
    except OSError as exc:p.error(str(exc))
    config={'output':str(out),'solver_root':str(a.solver_root.resolve()) if a.solver_root else None,'request_timeout':a.request_timeout,'min_free_mib':a.min_free_mib}
    cfg=out/'config.json';cfg.write_text(json.dumps(config))
    env=dict(os.environ,ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(a.domain_id),PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    result=run_bounded([sys.executable,'-m','trilayer_planner_service.worker',str(cfg)],out/'worker.log',timeout=a.lifetime,env=env,cwd=out,deadline_file=out/'deadline.json')
    (out/'execution.json').write_text(json.dumps(result,indent=2))
    # Remove a socket left by forced termination. This is a directory we exclusively created.
    (out/'service.sock').unlink(missing_ok=True)
    return 124 if result['timed_out'] else (0 if result['returncode']==0 and not result['remaining_processes'] else 1)
if __name__=='__main__':raise SystemExit(main())
