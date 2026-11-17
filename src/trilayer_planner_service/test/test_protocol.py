import socket,unittest
from trilayer_planner_service.protocol import validate,strict_json,ProtocolError,read_line,encode
class ProtocolTests(unittest.TestCase):
 def test_duplicates_nonfinite(self):
  for raw in ['{"v":1,"v":1}','{"x":NaN}','{"x":Infinity}']:
   with self.assertRaises(ProtocolError):strict_json(raw)
 def test_strict_ops(self):
  self.assertEqual(validate({'v':1,'id':'s','op':'status'})['op'],'status')
  for req in [{'v':1,'id':'s','op':'reset'},{'v':True,'id':'s','op':'status'},{'v':1,'id':'s','op':'status','extra':0}]:
   with self.assertRaises(ProtocolError):validate(req)
 def test_socket_read(self):
  a,b=socket.socketpair()
  try:
   a.sendall(encode({'v':1,'id':'a','op':'close'}))
   self.assertEqual(validate(read_line(b))['op'],'close')
   a.sendall(b'{}\n{}\n')
   with self.assertRaises(ProtocolError):read_line(b)
  finally:a.close();b.close()
 def test_context_and_pose(self):
  c={'frame_id':'map','width':240,'height':240,'resolution':.05,'origin':[0,0],'hard_mask_sha256':'0'*64,'costmap_sha256':'1'*64}
  r={'v':1,'id':'a','op':'plan','context':c,'snapshot':'AA==','start':[2,4,0],'goal':[5,4,0]}
  validate(r)
  for v in [float('nan'),True,'2']:
   r['start']=[v,4,0]
   with self.assertRaises(ProtocolError):validate(r)
 def test_update_api_bounds_and_numeric_identity(self):
  c={'frame_id':'map','width':240,'height':240,'resolution':.05,'origin':[0,0],'hard_mask_sha256':'0'*64,'costmap_sha256':'1'*64}
  r={'v':1,'id':'u','op':'update','context':c,'sequence':-9,'upsert':{'box':[[1,1],[2.0,1],[2,2]]}}
  self.assertIs(validate(r),r)
  self.assertIs(type(r['upsert']['box'][0][0]),int)
  self.assertIs(type(r['upsert']['box'][1][0]),float)
  for changes in [{'sequence':True},{'upsert':{'x'*65:[[1,1],[2,1],[2,2]]}},{'upsert':{'box':[[1,1]]*129}}, {'upsert':{str(i):[[1,1],[2,1],[2,2]] for i in range(65)}}, {'clear':['x']*2}, {'clear':['box']}, {'clear':['../bad']}]:
   with self.assertRaises(ProtocolError):validate({**r,**changes})
 def test_cli_deadline_invalid_before_launch(self):
  import contextlib,io,tempfile
  from pathlib import Path
  from trilayer_planner_service.cli import main
  with tempfile.TemporaryDirectory() as d:
   out=Path(d)/'out'
   for flag,value in [('--lifetime','nan'),('--lifetime','inf'),('--lifetime','-1'),('--request-timeout','0'),('--request-timeout','nan'),('--request-timeout','inf')]:
    with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as caught:
     main(['--output',str(out),'--domain-id','186',flag,value])
    self.assertEqual(caught.exception.code,2)
    self.assertFalse(out.exists())
 def test_disk_floor_rejects_before_dispatch_without_writes(self):
  from collections import namedtuple
  from unittest.mock import Mock,patch
  from trilayer_planner_service.resources import dispatch_checked,require_space,DiskSpaceLow
  Usage=namedtuple('Usage','total used free')
  backend=Mock()
  with patch('trilayer_planner_service.resources.shutil.disk_usage',return_value=Usage(10**9,10**9,512*1024*1024-1)) as usage:
   with self.assertRaises(DiskSpaceLow):dispatch_checked(backend,{'op':'plan'},'/only-this-output',512)
   usage.assert_called_once_with('/only-this-output');backend.dispatch.assert_not_called()
  with patch('trilayer_planner_service.resources.shutil.disk_usage',return_value=Usage(10**9,0,512*1024*1024)):
   require_space('/only-this-output',512)
   with self.assertRaises(DiskSpaceLow):require_space('/only-this-output',513)
  with patch('trilayer_planner_service.resources.shutil.disk_usage') as usage:
   dispatch_checked(backend,{'op':'close'},'/only-this-output',512)
   dispatch_checked(backend,{'op':'status'},'/only-this-output',512);usage.assert_not_called()
  for invalid in (0,511,True,float('inf')):
   with self.assertRaises(ValueError):require_space('/only-this-output',invalid)
if __name__=='__main__':unittest.main(verbosity=2)
