"""Diagnostic-only exception-local capture. Never part of formal execution."""
from pathlib import Path
import json,numpy as np

def install(directory):
 from three_yd_v5.controller import V5Controller
 original=V5Controller._activate_reference;out=Path(directory);out.mkdir(parents=True,exist_ok=True)
 def activate(self,*args,**kwargs):
  try:return original(self,*args,**kwargs)
  except (RuntimeError,ValueError) as error:
   tb=error.__traceback__;local=None
   while tb:
    if tb.tb_frame.f_code.co_name=='_activate_reference':local=tb.tb_frame.f_locals
    tb=tb.tb_next
   if local and 'circle_view' in local:
    v=local['circle_view'];data={k:local[k] for k in ('mask','point_safe','opened','point_raw','band','fine_hard','direction_local') if k in local};data['circle_safe']=v.safe
    np.savez_compressed(out/'failure.npz',**data)
    record={'error':str(error),'record':local.get('record'),'start':list(self.topology_route.start),'goal':list(self.topology_route.goal),'resolution':self.world.map.resolution,'origin':self.world.map.origin,'shape':self.world.hard.shape,'vehicle':self.world.vehicle,'bbox':local.get('bbox'),'certificates':local.get('certificates',[])}
    (out/'failure.json').write_text(json.dumps(record,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x),indent=2))
   raise
 V5Controller._activate_reference=activate
