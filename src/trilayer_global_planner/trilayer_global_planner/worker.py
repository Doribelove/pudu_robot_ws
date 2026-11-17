# SPDX-License-Identifier: Apache-2.0
"""One request in a fresh interpreter; the parent owns the whole-process bound."""
import json,sys,traceback
from pathlib import Path


def main():
    request_path,result_path=map(Path,sys.argv[1:3])
    request=json.loads(request_path.read_text())
    try:
        from trilayer_p6_runtime.api import MapOnlyRequest,plan_map
        result=plan_map(MapOnlyRequest(map_yaml=Path(request['map_yaml']),start=tuple(request['start']),
                        goal=tuple(request['goal']),output_dir=Path(request['output_dir']),
                        master_service=request['master_service'],vehicle=tuple(request['vehicle'])),
                        solver_root=Path(request['solver_root']) if request.get('solver_root') else None)
        result['failure_code']=result.get('failure_code') or result.get('result',{}).get('failure_code')
    except BaseException as exc:
        result={'success':False,'path':[],'frame_id':'map','failure_code':type(exc).__name__,
                'exception':traceback.format_exc()}
    temp=result_path.with_suffix('.tmp')
    temp.write_text(json.dumps(result,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x))+'\n')
    temp.replace(result_path)
    return 0 if result.get('success') and not result.get('cleanup_errors') else 1


if __name__=='__main__':raise SystemExit(main())
