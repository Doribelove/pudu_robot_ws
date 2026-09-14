"""Ordered-route-conditioned preference and finite L2-to-Smac soft adapter."""
from dataclasses import dataclass
import math,time
import numpy as np
from scipy.spatial import cKDTree
from arena_evaluation.semantic_map import canonical_hash
from arena_evaluation.semantic_costmap_composer import (SemanticCostmap,occupancy_to_static_layer,internal_soft_to_occupancy)
from arena_evaluation.semantic_costmap_r2 import pinned_nav2_effective_master
from .semantic_world import digest,wrap

@dataclass
class RoutePreference:
 potential:np.ndarray
 normalized_right:np.ndarray
 eligible:np.ndarray
 ambiguous:np.ndarray
 key:str
 diagnostics:dict

def exact_baseline_costmap(session,allowed_mask):
 """Verify the original r1 grid exactly without adding any preference."""
 started=time.monotonic();cpu=time.process_time()
 if session._semantic_costmap is not None:raise ValueError('baseline must start with the original source grid')
 _,server_values=session._grid_for_mask(allowed_mask);occupancy=np.ascontiguousarray(np.flipud(server_values))
 static=occupancy_to_static_layer(occupancy)
 master=pinned_nav2_effective_master(static,resolution=session.ctx.hospital_map.resolution,inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
 key=canonical_hash(['original_r1_unmodified_source_grid',digest(occupancy)])
 return SemanticCostmap(static,occupancy,master,np.zeros(occupancy.shape,np.float32),occupancy==100,np.ones(occupancy.shape,bool),key,'','',False,
  {'original_grid_unchanged':True,'adapter_ms':(time.monotonic()-started)*1000,'adapter_cpu_ms':(time.process_time()-cpu)*1000})

def _route_samples(world,route):
 poses=[];progress=[];arc=0.
 for e in route.edges:
  if e.path is None:raise ValueError('route paths must be resolved')
  p=e.path;ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);s=np.r_[0,np.cumsum(ds)]
  ids=np.unique(np.r_[np.searchsorted(s,np.arange(0,s[-1],.20)),len(p)-1]).astype(int)
  poses.extend(p[ids]);progress.extend(s[ids]+arc);arc+=s[-1]
 p=np.asarray(poses);return p,np.asarray(progress),world.sample_labels(p)

def ordered_match(world,route,rows,cols,reference=None):
 p,progress,labels=_route_samples(world,route)
 if reference is not None:
  p=np.asarray(reference);length=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);progress=np.r_[0,np.cumsum(length)];labels=world.sample_labels(p)
  ids=np.unique(np.r_[np.searchsorted(progress,np.arange(0,progress[-1],.20)),len(p)-1]).astype(int)
  p=p[ids];progress=progress[ids];labels=labels[ids]
 xy=world.world_points(rows,cols);cell_labels=world.labels[rows,cols]
 indices=np.full(len(rows),-1,int);ambiguous=np.zeros(len(rows),bool);distances=np.full(len(rows),np.inf)
 for label in np.unique(cell_labels):
  hits=np.flatnonzero(cell_labels==label);refs=np.flatnonzero(labels==label)
  if not len(refs):continue
  tree=cKDTree(p[refs,:2]);d,ix=tree.query(xy[hits],k=min(8,len(refs)))
  if d.ndim==1:d=d[:,None];ix=ix[:,None]
  globalix=refs[ix];indices[hits]=globalix[:,0];distances[hits]=d[:,0]
  # Adjacent samples/edges are compatible; nonlocal occurrences at the same
  # spatial support are ambiguous. We abstain instead of attracting across
  # a parallel return lane or the wrong visit to an intersection.
  remote=abs(progress[globalix]-progress[globalix[:,0]][:,None])>3.
  ambiguous[hits]=np.any(remote & (d<d[:,0,None]+.25),axis=1)
  # Eight nearest samples are only the fast path. Dense sampling must not
  # conceal a nearby second occurrence behind eight samples of the first.
  pending=(d[:,-1]<d[:,0]+.25)&~ambiguous[hits]
  for j in np.flatnonzero(pending):
   candidates=refs[tree.query_ball_point(xy[hits[j]],float(d[j,0]+.25))]
   if np.any(abs(progress[candidates]-progress[globalix[j,0]])>3.):ambiguous[hits[j]]=True
 return p,progress,indices,distances,ambiguous

def preference_field(world,route,*,side='right'):
 started=time.monotonic();cpu=time.process_time();shape=world.labels.shape
 laneids=[k for k,f in world.features.items() if f.semantic_class=='lane']
 selected=world.safe&route.plan.corridor_mask&np.isin(world.labels,laneids)
 r,c=np.nonzero(selected);pot=np.zeros(shape,np.float32);qgrid=np.full(shape,np.nan,np.float32)
 eligible=np.zeros(shape,bool);ambiguity=np.zeros(shape,bool)
 p,s,ix,d,amb=ordered_match(world,route,r,c)
 valid=(ix>=0)&~amb
 if len(r) and np.any(valid):
  rv,cv=r[valid],c[valid];xy=world.world_points(rv,cv);theta=p[ix[valid],2]
  n=np.column_stack((np.sin(theta),-np.cos(theta)))
  label=world.labels[rv,cv];limits=[]
  for sign in [1,-1]:
   length=np.zeros(len(rv));active=np.ones(len(rv),bool)
   for step in np.arange(.025,16.001,.05):
    indices=np.flatnonzero(active)
    if not len(indices):break
    points=xy[indices]+sign*step*n[indices];rr,cc=world.cells(points)
    inside=(rr>=0)&(cc>=0)&(rr<shape[0])&(cc<shape[1]);same=np.zeros(len(indices),bool)
    good=indices[inside];same[inside]=(world.labels[rr[inside],cc[inside]]==label[good])&~world.hard[rr[inside],cc[inside]]
    length[indices[same]]=step;active[indices[~same]]=False
   limits.append(length)
  right,left=limits;width=right+left;safe_width=width-1.
  normalized=np.divide(right-.5,safe_width,out=np.full(len(right),.5),where=safe_width>0)
  target=.25 if side=='right' else .75
  potential=np.minimum(1.,abs(normalized-target)/.75)
  good=width>=4.
  pot[rv[good],cv[good]]=potential[good];qgrid[rv,cv]=normalized
  eligible[rv[good],cv[good]]=True
 ambiguity[r,c]=amb
 key=canonical_hash([world.key,route.plan.route_signature,side,digest(pot),digest(ambiguity)])
 return RoutePreference(pot,qgrid,eligible,ambiguity,key,{'build_ms':(time.monotonic()-started)*1000,'cpu_ms':(time.process_time()-cpu)*1000,'eligible_cells':int(eligible.sum()),'ambiguous_cells':int(ambiguity.sum()),'side':side})

def reference_costmap(world,route,l2_path,*,enabled=True,cap=140,saturation=2.,side='right',blocked=()):
 started=time.monotonic();cpu=time.process_time();mask=route.plan.corridor_mask.copy()
 for cell in blocked:mask[cell]=False
 soft=np.zeros(world.labels.shape,np.float32);ambiguity=np.zeros(mask.shape,bool)
 if enabled:
  if not (0<=cap<=200) or saturation<=0:raise ValueError('reference cost must be bounded and nonnegative')
  rc=np.asarray(l2_path,dtype=int);xy=world.world_points(rc[:,0],rc[:,1]);delta=np.diff(xy,axis=0)
  yaw=np.arctan2(delta[:,1],delta[:,0]);poses=np.column_stack((xy,np.r_[yaw,yaw[-1]]))
  r,c=np.nonzero(mask&~world.hard);p,arc,ix,d,amb=ordered_match(world,route,r,c,reference=poses)
  good=(ix>=0)&~amb;soft[r[good],c[good]]=cap*np.minimum(1.,d[good]/saturation);ambiguity[r,c]=amb
 base=np.asarray(world.map.occupancy,dtype=np.int16).copy();base[world.hard|~mask]=100
 occupancy=np.maximum(base,internal_soft_to_occupancy(soft)).astype(np.int8)
 occupancy[world.map.occupancy<0]=-1;occupancy[world.hard|~mask]=100
 static=occupancy_to_static_layer(occupancy)
 master=pinned_nav2_effective_master(static,resolution=world.map.resolution,inflation_radius_m=.55,cost_scaling_factor=3.,inscribed_radius_m=.225)
 key=canonical_hash([world.key,route.plan.route_signature,digest(soft),enabled,cap,saturation,side,digest(ambiguity)])
 return SemanticCostmap(static,occupancy,master,soft,world.hard|~mask,np.ones(mask.shape,bool),key,world.semantic.semantic_map_hash,
  digest(np.asarray(l2_path)),enabled,{'reference_key':key,'route_signature':route.plan.route_signature,'finite_nonnegative':bool(np.all(np.isfinite(soft)) and np.all(soft>=0)),
   'max_soft_cost':float(soft.max()),'ambiguous_cells':int(ambiguity.sum()),'adapter_ms':(time.monotonic()-started)*1000,'adapter_cpu_ms':(time.process_time()-cpu)*1000,
   'hard_mask_sha256':digest(world.hard|~mask),'corridor_sha256':digest(mask)})
