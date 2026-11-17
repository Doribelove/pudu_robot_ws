"""Real successful planning, then deliberate post-close diagnostic failure.

Only cleanup failure is injected. The original close completes first; planning,
ACK, path and native worker lifecycle are real. No live child is left behind.
"""
from pathlib import Path
import os,sys,tempfile,json
import numpy as np
import psutil
from PIL import Image
from trilayer_p6_runtime import activate,RuntimeConfig
from trilayer_p6_runtime.api import MapOnlyRequest,plan_map
assert os.environ['ROS_LOCALHOST_ONLY']=='1' and os.environ['ROS_DOMAIN_ID']=='179'
root=Path(tempfile.mkdtemp(prefix='trilayer-cleanup-failure-'));out=root/'runtime'
pixels=np.full((240,240),254,np.uint8);pixels[:4,:]=0;pixels[-4:,:]=0;pixels[:,:4]=0;pixels[:,-4:]=0
Image.fromarray(pixels).save(root/'map.pgm')
mp=root/'map.yaml';mp.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
info=activate(RuntimeConfig('/global_costmap/trilayer_get_costmap',out/'cache'))
cls=info['CurveSession'];real_close=cls.close;closed=[]
def close_then_report_error(self):
 real_close(self)
 closed.append(self.stack.process.returncode)
 raise RuntimeError('TEST_ONLY_AFTER_REAL_CLOSE_FAILURE')
cls.close=close_then_report_error
try:
 data=plan_map(MapOnlyRequest(mp,(2.,4.,0.),(5.,4.,0.),out),solver_root=Path(sys.argv[1]))
finally:cls.close=real_close
assert closed==[0],closed
assert data['result']['success'] is True, 'the actual original planner did not succeed'
assert data['result']['independent_audit']['valid'] is True
assert data['plan_result']['points'], 'diagnostic candidate lost'
assert data['success'] is False and data['path']==[]
assert data['worker_exit']==0 and data['exception'] is None
assert len(data['cleanup_errors'])==1 and 'TEST_ONLY_AFTER_REAL_CLOSE_FAILURE' in data['cleanup_errors'][0]
assert any(a.get('costmap_ack_status')=='exact_effective_master_content_verified' for a in data['ack_events'])
saved=json.loads((out/'result.json').read_text())
assert saved['success'] is False and saved['path']==[] and saved['plan_result']['points']
assert not psutil.Process().children(recursive=True),'test left live descendants'
print('PASS real successful candidate retained only diagnostically; public path empty after post-close error;',out)
