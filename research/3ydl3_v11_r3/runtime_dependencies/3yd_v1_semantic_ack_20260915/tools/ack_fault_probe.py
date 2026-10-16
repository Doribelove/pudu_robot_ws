"""Explicit client-side fault injection against a real ROS costmap service."""
import argparse,time,os,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from arena_3d_v1.semantic_world import SemanticWorld,json_write
from arena_3d_v1.semantic_pipeline import ReferenceSmacSession
from arena_3d_v1.stable_session import StableSmacSession
from arena_evaluation.unified_four_backends_smoke import MapContext
from recovery_scenes import return_bulb_inputs

class SuppressedReply:
 def __init__(self,future):self.future=future
 def done(self):return False
 def cancel(self):return self.future.cancel()

class ChangedReply:
 def __init__(self,future):self.future=future
 def done(self):return self.future.done()
 def result(self):
  result=self.future.result();data=list(result.map.data);data[0]=0 if data[0]==254 else 254;result.map.data=data;return result
 def cancel(self):return self.future.cancel()

class InjectedClient:
 def __init__(self,client,mode):self.client=client;self.mode=mode;self.calls=0;self.removed=0
 def wait_for_service(self,**kwargs):return self.client.wait_for_service(**kwargs)
 def call_async(self,request):
  self.calls+=1;future=self.client.call_async(request)
  if self.mode=='drop_first' and self.calls==1:return SuppressedReply(future)
  if self.mode=='corrupt_all':return ChangedReply(future)
  return future
 def remove_pending_request(self,future):
  self.removed+=1;return self.client.remove_pending_request(getattr(future,'future',future))

class DropFirstTile:
 def __init__(self,publisher):self.publisher=publisher;self.calls=0
 def __getattr__(self,name):return getattr(self.publisher,name)
 def publish(self,message):
  self.calls+=1
  if self.calls>1:self.publisher.publish(message)

class InjectFault:
 def _publish_dirty_roi(self,expected,changed):
  if (self.injection_mode=='drop_first_tile' or (self.injection_mode=='drop_first_mask_tile' and getattr(self,'inject_mask_now',False))) and not isinstance(self._local_update_publisher,DropFirstTile):
   self._local_update_publisher=DropFirstTile(self._local_update_publisher)
  return super()._publish_dirty_roi(expected,changed)
 def _server_costmap_snapshot(self,deadline):
  if not isinstance(self._get_costmap_client,InjectedClient):
   self._get_costmap_client=InjectedClient(self._get_costmap_client,self.injection_mode)
   self.injected_client=self._get_costmap_client
  return super()._server_costmap_snapshot(deadline)
 def close(self):
  if isinstance(self._local_update_publisher,DropFirstTile):self._local_update_publisher=self._local_update_publisher.publisher
  return super().close()

class Baseline(InjectFault,ReferenceSmacSession):pass
class Candidate(InjectFault,StableSmacSession):pass

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--domain',type=int,default=98);args=p.parse_args()
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False);os.environ['ROS_DOMAIN_ID']=str(args.domain);os.sched_setaffinity(0,{15})
 mp,sp,_=return_bulb_inputs(out/'inputs');w=SemanticWorld(mp,sp)
 ctx=MapContext('fault',w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp);rows=[]
 for arm,cls,mode in [('r1',Baseline,'drop_first'),('candidate',Candidate,'drop_first'),('candidate',Candidate,'corrupt_all'),
                      ('r1',Baseline,'drop_first_tile'),('candidate',Candidate,'drop_first_tile'),
                      ('r1',Baseline,'drop_first_mask_tile'),('candidate',Candidate,'drop_first_mask_tile')]:
  session=cls(ctx,out/(arm+'_'+mode),map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',
   smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,
   planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
  session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0;session.injection_mode=mode
  row={'arm':arm,'injection':mode,'real_service_with_client_side_fault':True,'startup_success':False};started=time.monotonic()
  try:
   session.start();row['startup_success']=True
   if mode=='drop_first_mask_tile':
    session.inject_mask_now=True
    started_update=time.monotonic();info=session.update_local_mask(w.safe.copy())
    row['update_success']=bool(info['costmap_update_acknowledged']);row['update_elapsed_s']=time.monotonic()-started_update
    row['update_ack']=info
  except Exception as error:row['error']=str(error)
  finally:
   row['elapsed_s']=time.monotonic()-started
   client=getattr(session,'injected_client',None);row['snapshot_requests']=getattr(client,'calls',0);row['requests_removed']=getattr(client,'removed',0)
   session.close();rows.append(row);json_write(out/'results.json',rows);print(row,flush=True)
