"""Experimental storage crop; restart costs remain inside the timed query.

All acceptance stays on the original global world. A cropped server never
inherits ACK evidence from a different origin or shape. Not enabled by default.
"""
from dataclasses import replace
from pathlib import Path
import math,time,json,hashlib
import numpy as np
import yaml
from PIL import Image
from .performance_session import PerformanceSession
from .r11_fields import bbox,bounded_inflation

def crop_origin(origin,height,bounds,resolution):
    a,b,c,d=bounds
    return (origin[0]+c*resolution,origin[1]+(height-b)*resolution,origin[2])

class StorageCropSession:
    def __init__(self,ctx,output,**kwargs):
        self.global_ctx=ctx;self.output=Path(output);self.kwargs=dict(kwargs)
        self.template=PerformanceSession(ctx,self.output/'template',**kwargs)
        self.delegate=None;self.bounds=None;self.current_query_id='';self.events=[];self.counter=0
        self.local_map_update_strategy='roi_ack';self.full_grid_settle_cycles=0
        self.r10_tile_events=[]
    def __getattr__(self,name):
        return getattr(self.delegate if self.delegate is not None else self.template,name)
    def start(self):pass  # lazily bind when the first bounded map is available
    def reset_query_state(self,query_id,**kwargs):
        self.current_query_id=query_id
        if self.delegate is not None:return self.delegate.reset_query_state(query_id,**kwargs)
        return {}
    def set_semantic_costmap(self,grid):
        began=time.monotonic();m=self.global_ctx.hospital_map
        bounds=bbox(~grid.hard_semantic_mask,math.ceil((.55+.35)/m.resolution)+2)
        if bounds is None:raise ValueError('EMPTY_STORAGE_CROP')
        a,b,c,d=bounds;sl=(slice(a,b),slice(c,d));switched=bounds!=self.bounds
        shutdown=0.;startup=0.
        if switched:
            if self.delegate is not None:
                t=time.monotonic();self.delegate.close();shutdown=(time.monotonic()-t)*1000
            self.counter+=1;folder=self.output/f'crop_{self.counter:03d}';folder.mkdir(parents=True,exist_ok=False)
            origin=crop_origin(m.origin,m.height,bounds,m.resolution)
            config=yaml.safe_load(Path(self.kwargs['map_yaml']).read_text());original=Image.open(m.image_path)
            image_path=folder/'map.png';original.crop((c,a,d,b)).save(image_path)
            config.update(image='map.png',origin=list(origin));mp=folder/'map.yaml';mp.write_text(yaml.safe_dump(config))
            local=replace(m,yaml_path=mp,image_path=image_path,width=d-c,height=b-a,origin=origin,
                occupancy=m.occupancy[sl].copy(),distance_m=m.distance_m[sl].copy())
            ctx=replace(self.global_ctx,hospital_map=local,free_mask=self.global_ctx.free_mask[sl].copy(),
                distance_m=self.global_ctx.distance_m[sl].copy(),map_sha256=local.sha256,
                map_yaml_sha256=hashlib.sha256(mp.read_bytes()).hexdigest(),map_yaml=mp)
            args={**self.kwargs,'map_yaml':mp}
            self.delegate=PerformanceSession(ctx,folder/'smac',**args)
            self.delegate.local_map_update_strategy='roi_ack';self.delegate.full_grid_settle_cycles=0
            self.delegate.r10_tile_events=self.r10_tile_events
            t=time.monotonic();self.delegate.start();startup=(time.monotonic()-t)*1000
            self.delegate.reset_query_state(self.current_query_id,restore_base_map=False)
            from native_snapshot import record
            record(folder)
            self.bounds=bounds
        fields={name:np.ascontiguousarray(getattr(grid,name)[sl]) for name in (
            'internal_cost','occupancy_grid','expected_master_cost','soft_cost','hard_semantic_mask','affected_mask')}
        check,_=bounded_inflation(fields['internal_cost'],m.resolution)
        if not np.array_equal(check,fields['expected_master_cost']):raise RuntimeError('CROP_EFFECTIVE_MAP_BINDING_MISMATCH')
        crop=replace(grid,**fields,policy_hash=hashlib.sha256((grid.policy_hash+repr(bounds)+repr(self.delegate.ctx.hospital_map.origin)).encode()).hexdigest())
        self.delegate.set_semantic_costmap(crop)
        self.events.append({'query':self.current_query_id,'bbox_rc':list(bounds),'cells':(b-a)*(d-c),
            'global_cells':m.width*m.height,'switched':switched,'shutdown_ms':shutdown,
            'startup_ms':startup,'bind_ms':(time.monotonic()-began)*1000,'effective_master_exact':True,
            'origin':list(self.delegate.ctx.hospital_map.origin),'global_audit_retained':True})
    def plan(self,query,*args,allowed_mask=None,**kwargs):
        a,b,c,d=self.bounds
        mask=np.asarray(allowed_mask,bool)
        if np.count_nonzero(mask)!=np.count_nonzero(mask[a:b,c:d]):raise RuntimeError('CROP_DROPPED_ALLOWED_CELLS')
        result=self.delegate.plan(query,*args,allowed_mask=mask[a:b,c:d],**kwargs)
        result.diagnostics['r11_crop']=dict(self.events[-1])
        return result
    def update_local_mask(self,mask,**kwargs):
        a,b,c,d=self.bounds
        return self.delegate.update_local_mask(mask[a:b,c:d],**kwargs)
    def close(self):
        t=time.monotonic()
        if self.delegate is not None:self.delegate.close()
        (self.output/'crop_events.json').write_text(json.dumps({'events':self.events,'final_shutdown_ms':(time.monotonic()-t)*1000},indent=2))
