"""Count every startup separately from planning; reuse a single isolated domain."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
import argparse, hashlib, json, os, time, traceback
from pathlib import Path
import numpy as np
import yaml
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write
from arena_3d_v1.stable_session import StableSmacSession
from arena_3d_v1.diagnostic_session import DiagnosticSession
from arena_evaluation.unified_four_backends_smoke import MapContext
from recovery_scenes import return_bulb_inputs
from controlled_probe import controlled_inputs

def shm_used():
    v=os.statvfs('/dev/shm');return (v.f_blocks-v.f_bfree)*v.f_frsize

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--cycles',type=int,default=10)
    p.add_argument('--arm',choices=['r1','stable'],required=True);p.add_argument('--domain',type=int,default=95)
    p.add_argument('--real',action='store_true');p.add_argument('--abrupt-every',type=int,default=0)
    args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    os.sched_setaffinity(0,{15});os.environ['ROS_DOMAIN_ID']=str(args.domain)
    worlds=[]
    if args.real:
        cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());inputs=[(Path(cfg['inputs']['map_yaml']),Path(cfg['inputs']['semantic_path']))]
    else:
        inputs=[return_bulb_inputs(out/f'inputs_{r}',bool(r))[:2] for r in range(2)]
        inputs += [controlled_inputs(out/f'calibration_{r}',bool(r)) for r in range(2)]
    for mp,sp in inputs:
        w=SemanticWorld(mp,sp,cache_root=out/'world_cache')
        ctx=MapContext('startup',w.map,w.safe,w.map.distance_m,w.map.sha256,hashlib.sha256(mp.read_bytes()).hexdigest(),mp)
        worlds.append((w,ctx,mp))
    json_write(out/'protocol.json',{'cycles':args.cycles,'arm':args.arm,'domain':args.domain,'real':args.real,
               'ack_timeout_s':3.,'map_hashes':[w.map.sha256 for w,_,_ in worlds],
               'checks_per_startup':['cold_all_lethal','open_safe_region','close_half','restore_safe_region'],
               'abrupt_every':args.abrupt_every,'cpu':15,
               'source_hashes':{str(p.relative_to(WORK)):hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in [Path(__file__),*WORK.glob('external/arena4_ws/src/arena/three_d_v1/arena_3d_v1/*session.py')]}})
    rows=[]
    for index in range(args.cycles):
        w,ctx,mp=worlds[index%len(worlds)];root=out/f'cycle_{index:03d}'
        row={'cycle':index,'startup_success':False,'updates':[],'shm_before':shm_used()};session=None;t=time.monotonic()
        try:
            cls=StableSmacSession if args.arm=='stable' else DiagnosticSession
            session=cls(ctx,root,map_yaml=mp,local_mask_updates=True,optimization_profile='v7_candidate',
                smac_parameter_profile='lighter_smoother',optimization_stage='step3_delta_map',enable_mask_reuse_noop=True,
                planner_parameter_overrides={'angle_quantization_bins':48},costmap_ack_timeout_s=3.)
            session.local_map_update_strategy='roi_ack';session.full_grid_settle_cycles=0
            session.start();row['startup_success']=True;row['startup_ms']=(time.monotonic()-t)*1000
            masks=[w.safe.copy(),w.safe.copy(),w.safe.copy()];masks[1][:,:w.map.width//2]=False
            for mask in masks:
                info=session.update_local_mask(mask)
                if info.get('costmap_update_acknowledged') is not True:raise AssertionError('UNACKED_MAP')
                row['updates'].append({k:v for k,v in info.items() if k.startswith(('costmap_ack','costmap_update_ack'))})
            if args.abrupt_every and (index+1)%args.abrupt_every==0:
                # Kill only this stack's planner to exercise resource release
                # and the following same-domain cold startup, not robot control.
                import signal
                os.kill(session.planner_pid,signal.SIGKILL);row['injected_planner_kill']=True
        except Exception as e:
            row['error']=str(e);row['traceback']=traceback.format_exc()
        finally:
            if session is not None:
                try:session.close()
                except Exception as e:row['close_error']=str(e)
            row['shm_after']=shm_used();row['wall_s']=time.monotonic()-t
            row['success']=row['startup_success'] and len(row['updates'])==3 and 'error' not in row and 'close_error' not in row
            rows.append(row);json_write(root/'cycle.json',row)
            with (out/'cycles.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
            print(index,row['success'],row.get('error',''),round(row.get('startup_ms',0)),round(row['shm_after']/2**20),flush=True)
    json_write(out/'summary.json',{'cycles':len(rows),'startup_success':sum(r['startup_success'] for r in rows),
        'complete_success':sum(r['success'] for r in rows),'update_successes':sum(len(r['updates']) for r in rows),
        'shm_start':rows[0]['shm_before'],'shm_end':rows[-1]['shm_after'],'failures':[r for r in rows if not r['success']]})

if __name__=='__main__':main()
