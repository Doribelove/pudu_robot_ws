"""Real exact-master ACK updates and explicit semantic-content fault injection."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
from native_snapshot import configure
configure()
import argparse,os,time,hashlib,copy,traceback
from pathlib import Path
import numpy as np,yaml
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write
from arena_3d_v1.semantic_ack_session import SemanticAckSession
from arena_3d_v1.stable_session import StableSmacSession
from arena_evaluation.semantic_costmap_composer import SemanticCostmap,occupancy_to_static_layer
from arena_evaluation.unified_four_backends_smoke import MapContext
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master
from controlled_probe import controlled_inputs

class FaultPublisher:
 def __init__(self,publisher,mode):self.publisher=publisher;self.mode=mode;self.armed=False;self.injected=0;self.saved={}
 def __getattr__(self,name):return getattr(self.publisher,name)
 def publish(self,msg):
  key=(msg.x,msg.y,msg.width,msg.height)
  if self.armed and not self.injected and self.mode=='drop_tile':self.injected+=1;return
  if self.armed and not self.injected and self.mode=='stale_tile' and key in self.saved:
   self.injected+=1;return self.publisher.publish(self.saved[key])
  self.saved[key]=copy.deepcopy(msg);return self.publisher.publish(msg)

class FaultMixin:
 def _server_costmap_snapshot(self,deadline):
  values,stamp=super()._server_costmap_snapshot(deadline)
  if self._semantic_costmap is not None:
   if self.fault=='corrupt_reads':values=values.copy();values[0,0]=0 if values[0,0]==254 else 254
   if self.fault=='stale_timestamp':stamp=0
   if self.fault=='sequence_change':self._semantic_publication_sequence+=1
  return values,stamp
class Candidate(FaultMixin,SemanticAckSession):pass
class Baseline(FaultMixin,StableSmacSession):pass

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--scene',choices=['calibration','rotated','real'],default='rotated')
 p.add_argument('--updates',type=int,default=12);p.add_argument('--domain',type=int,default=103)
 p.add_argument('--publication',choices=['baseline','paced','repair','combined'],default='combined')
 p.add_argument('--fault',choices=['none','drop_tile','stale_tile','corrupt_reads','stale_timestamp','sequence_change'],default='none')
 args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False);os.sched_setaffinity(0,{15});os.environ['ROS_DOMAIN_ID']=str(args.domain)
 if args.scene=='real':
  cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());mp,sp=map(Path,[cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path']])
 else:mp,sp=controlled_inputs(out/'inputs',args.scene=='rotated')
 world=SemanticWorld(mp,sp);ctx=MapContext('semantic_stress',world.map,world.safe,world.map.distance_m,world.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
 cls=Baseline if args.publication=='baseline' else Candidate
 opts={} if cls is Baseline else {'tile_interval_s':.015 if args.publication in ['paced','combined'] else 0.,'enable_semantic_repair':args.publication in ['repair','combined']}
 session=cls(ctx,out/'smac',map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.,**opts)
 session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0;session.fault='none'
 params=yaml.safe_load(session.params_file.read_text())['global_costmap']['global_costmap']['ros__parameters'];inf=params['inflation_layer']
 # The frozen footprint is +/-0.255 m by +/-0.215 m plus 0.01 m padding.
 inscribed=min(abs(p[1]) for p in yaml.safe_load(params['footprint']))+float(params.get('footprint_padding',.01))
 fields=[];xx=np.linspace(5,65,world.map.width).astype(np.int8)
 for side,q in [('right',xx),('left',xx[::-1])]:
  occ=np.where(world.safe,np.broadcast_to(q,world.safe.shape),100).astype(np.int8)
  master=pinned_nav2_effective_master(occupancy_to_static_layer(occ),resolution=world.map.resolution,inflation_radius_m=float(inf['inflation_radius']),cost_scaling_factor=float(inf['cost_scaling_factor']),inscribed_radius_m=inscribed)
  fields.append(SemanticCostmap(master.copy(),occ,master,np.where(world.safe,1,0).astype(np.uint8),~world.safe,np.ones(world.safe.shape,bool),'stress_'+side,world.semantic.semantic_map_hash,side,True))
 protocol={'expected_master_model':'pinned_nav2_effective_master','args':vars(args),'map_sha256':world.map.sha256,'map_shape':world.safe.shape,'vehicle':world.vehicle,'ack_timeout_s':3.,'inscribed_radius_m':inscribed,'parameter_sha256':hashlib.sha256(session.params_file.read_bytes()).hexdigest(),'source_hashes':{str(f.relative_to(WORK)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__),*WORK.glob('external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/*session.py')]}}
 json_write(out/'protocol.json',protocol);rows=[];original=None;started=time.monotonic();startup=False
 try:
  session.start();__import__('native_snapshot').record(out);startup=True;startup_s=time.monotonic()-started;original=session._local_update_publisher;publisher=FaultPublisher(original,args.fault);session._local_update_publisher=publisher
  for i in range(args.updates):
   session.current_query_id=f'update_{i}';field=fields[i%2];session.set_semantic_costmap(field)
   # Warm the previous content before stale-tile injection; all faults are
   # confined to the first nontrivial switch and subsequent repair handling.
   if i>=1:publisher.armed=True;session.fault=args.fault
   began=time.monotonic();row={'index':i,'side':'right' if i%2==0 else 'left','success':False}
   try:
    ack=session.update_local_mask(world.safe);row.update(success=ack['costmap_update_acknowledged'] is True,ack=ack)
   except Exception as error:row['error']=str(error);row['traceback']=traceback.format_exc()
   row['elapsed_s']=time.monotonic()-began;row['injected_tiles']=publisher.injected;rows.append(row)
   with (out/'updates.jsonl').open('a') as f:f.write(__import__('json').dumps(row)+'\n')
   print(i,row['success'],round(row['elapsed_s'],3),row.get('ack',{}).get('semantic_gap_repair_count',0),row.get('error',''),flush=True)
   if not row['success']:break
 finally:
  if original is not None:session._local_update_publisher=original
  session.close()
  json_write(out/'summary.json',{'startup_success':startup,'requested_updates':args.updates,'executed':len(rows),'success':sum(r['success'] for r in rows),'failures':[r for r in rows if not r['success']],'fault':args.fault})
if __name__=='__main__':main()
