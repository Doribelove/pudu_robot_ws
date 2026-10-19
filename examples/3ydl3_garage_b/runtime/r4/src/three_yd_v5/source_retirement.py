"""Retire obsolete global composition owners after trajectory withdrawal.

Keep byte-identical native publication/readback state for the next real ACK.
A small native view must not keep an obsolete full-map source allocation alive.
This is called only between actions, before constructing the next controller.
"""
from dataclasses import replace
import numpy as np
from .r4_tiles import TiledSoft


def detach(a):
 if not isinstance(a,np.ndarray):return a
 owner=a
 while isinstance(getattr(owner,'base',None),np.ndarray):owner=owner.base
 owner=getattr(owner,'base',None) if isinstance(getattr(owner,'base',None),(bytes,memoryview)) else owner
 owned=owner.nbytes if isinstance(owner,np.ndarray) else len(owner)
 if owned<=max(65536,2*a.nbytes):return a
 return np.frombuffer(a.tobytes(),dtype=a.dtype).reshape(a.shape)


def retire(session):
 if (not getattr(session,'r5_enabled',False) or getattr(session,'_r5_global_grid',None) is None):return False
 if (getattr(session,'_action_in_progress',False) or getattr(session,'_r7_batch',None) is not None
     or getattr(session,'_v9_batch',None) is not None):raise RuntimeError('R6_RETIRE_DURING_ACTIVE_PUBLICATION')
 local=getattr(session,'_semantic_costmap',None)
 if local is not None:
  values={name:detach(getattr(local,name)) for name in ('internal_cost','occupancy_grid','expected_master_cost','hard_semantic_mask','affected_mask')}
  soft=local.soft_cost
  if type(soft) is TiledSoft:soft=TiledSoft(soft.shape,soft.cap,detach(soft.drivable),[(box,detach(v)) for box,v in soft.pieces])
  elif isinstance(soft,np.ndarray):soft=detach(soft)
  else:return False
  session._semantic_costmap=replace(local,**values,soft_cost=soft)
 for name in ('_current_grid','_current_allowed_mask','_last_exact_expected_master'):
  if hasattr(session,name):setattr(session,name,detach(getattr(session,name)))
 session._r5_global_grid=None;session._r5_grid_cache=None;session._r5_mask_cache=None
 return True
