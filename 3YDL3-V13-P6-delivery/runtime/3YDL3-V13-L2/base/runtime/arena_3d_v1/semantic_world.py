"""Single semantic source, conservative body checks, no skeleton topology."""
from __future__ import annotations
import ctypes, hashlib, json, math, os, time
from pathlib import Path
from dataclasses import dataclass
import numpy as np
import cv2
from scipy import ndimage
from arena_evaluation.planner_benchmark.map_utils import HospitalMap
from arena_evaluation.semantic_map import SemanticMapV1, canonical_hash
from arena_evaluation.semantic_rasterizer import rasterize_feature

WORK=Path(__file__).resolve().parents[6]
HALF_LENGTH=.265
HALF_WIDTH=.225
def wrap(a): return np.arctan2(np.sin(a),np.cos(a))
def digest(a): return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def json_write(path,value):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False));os.replace(tmp,path)
def densify(poses,ds=.025,dyaw=math.pi/180):
 p=np.asarray(poses,dtype=float);out=[p[0]]
 for a,b in zip(p,p[1:]):
  d=b-a;d[2]=wrap(d[2]);n=max(1,int(math.ceil(np.linalg.norm(d[:2])/ds)),int(math.ceil(abs(d[2])/dyaw)))
  out.extend(a+d*np.arange(1,n+1)[:,None]/n)
 return np.asarray(out)

class SweptChecker:
 def __init__(self,hard,resolution,origin,half_length=HALF_LENGTH,half_width=HALF_WIDTH):
  self.hard=np.ascontiguousarray(hard,dtype=np.uint8);self.res=float(resolution);self.origin=origin
  self.half_length=float(half_length);self.half_width=float(half_width)
  self.lib=ctypes.CDLL(str(WORK/'build/libsemantic_sweep.so'))
  self.fn=self.lib.swept_rect
  self.fn.argtypes=[np.ctypeslib.ndpointer(dtype=np.uint8,flags='C_CONTIGUOUS'),ctypes.c_int,ctypes.c_int,
    np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS'),ctypes.c_int,*([ctypes.c_double]*5)]
  self.fn.restype=ctypes.c_long
 def check(self,poses):
  p=np.ascontiguousarray(poses,dtype=np.float64)
  if p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.all(np.isfinite(p)): return False
  return self.fn(self.hard,*self.hard.shape,p,len(p),self.res,*self.origin[:2],self.half_length,self.half_width)==0

class SemanticWorld:
 def __init__(self,map_yaml,semantic_path,*,rules=None,cache_root=None,vehicle=(HALF_LENGTH,HALF_WIDTH,.40)):
  started=time.monotonic();self.map=HospitalMap.load(map_yaml);self.semantic=SemanticMapV1.load(semantic_path)
  m=self.map;s=self.semantic
  if (m.width,m.height,m.resolution,m.origin)!=(s.width,s.height,s.resolution,s.origin): raise ValueError('semantic/map coordinate mismatch')
  self.rules=dict(rules or {});self.features={i+1:f for i,f in enumerate(s.features)}
  self.vehicle=tuple(map(float,vehicle))
  if len(self.vehicle)!=3 or not all(math.isfinite(v) and v>0 for v in self.vehicle):raise ValueError('invalid vehicle dimensions/radius')
  self.binding={'schema':'dual-map-semantic-base-v1','map':m.sha256,'semantic':s.semantic_map_hash,
    'rules':canonical_hash(self.rules),'resolution':m.resolution,'origin':m.origin,'vehicle':self.vehicle,
    'map_yaml':hashlib.sha256(Path(map_yaml).read_bytes()).hexdigest(),'occupancy':digest(m.occupancy)}
  self.key=canonical_hash(self.binding);self.cache_hit=False
  cache=None if cache_root is None else Path(cache_root)/('semantic_'+self.key+'.npz')
  if cache and cache.exists():
   try:
    meta=json.loads(cache.with_suffix('.json').read_text())
    if meta['key']!=self.key or meta['sha256']!=hashlib.sha256(cache.read_bytes()).hexdigest(): raise ValueError('corrupt semantic cache')
    with np.load(cache,allow_pickle=False) as d:self.labels=d['labels'];self.hard=d['hard'];self.safe=d['safe']
    if self.labels.shape!=m.occupancy.shape:raise ValueError('shape mismatch')
    self.cache_hit=True
   except Exception:
    raise ValueError('semantic base cache failed integrity verification')
  else:
   self.labels=np.zeros(m.occupancy.shape,np.int16);self.hard=m.occupancy!=0
   # The broad final parking region is a fallback label; explicit narrow
   # parking regions, lanes, and junctions override it in that order.
   area=lambda f: abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(f.coordinates,f.coordinates[1:]+f.coordinates[:1])))/2
   priority=lambda f: {'parking_area':10,'lane':20,'junction_area':30}.get(f.semantic_class,-1)
   for i,f in sorted(self.features.items(),key=lambda item:(priority(item[1]),-area(item[1]))):
    mask=rasterize_feature(s,f)
    if f.hard:self.hard|=mask
    if priority(f)>=0:self.labels[mask]=i
   # Unlabelled raw free regions are explicit unknown-semantic regions. No
   # traffic direction is invented. They remain visible in provenance.
   self.labels[self.hard]=0
   self.safe=ndimage.distance_transform_edt(~self.hard)*m.resolution>=math.hypot(*self.vehicle[:2])+.05+math.sqrt(2)*m.resolution/2
   if cache:
    cache.parent.mkdir(parents=True,exist_ok=True);tmp=cache.with_suffix('.tmp.npz')
    np.savez_compressed(tmp,labels=self.labels,hard=self.hard,safe=self.safe);os.replace(tmp,cache)
    json_write(cache.with_suffix('.json'),{'key':self.key,'sha256':hashlib.sha256(cache.read_bytes()).hexdigest()})
  self.hard=np.asarray(self.hard,bool);self.checker=SweptChecker(self.hard,m.resolution,m.origin,*self.vehicle[:2])
  self.preparation_ms=(time.monotonic()-started)*1000
 def cells(self,p):
  p=np.asarray(p);c=np.floor((p[:,0]-self.map.origin[0])/self.map.resolution).astype(int)
  r=self.map.height-1-np.floor((p[:,1]-self.map.origin[1])/self.map.resolution).astype(int)
  return r,c
 def world_points(self,r,c):
  return np.column_stack((self.map.origin[0]+(np.asarray(c)+.5)*self.map.resolution,
   self.map.origin[1]+(self.map.height-np.asarray(r)-.5)*self.map.resolution))
 def sample_labels(self,p):
  r,c=self.cells(p);valid=(r>=0)&(c>=0)&(r<self.map.height)&(c<self.map.width)
  out=np.full(len(r),-1);out[valid]=self.labels[r[valid],c[valid]];return out
 def audit(self,query,poses,allowed_mask,*,rule_checker=None):
  started=time.monotonic();cpu=time.process_time();p=np.asarray(poses,dtype=float);fail=[]
  if p.ndim!=2 or p.shape[1]!=3 or len(p)<2 or not np.all(np.isfinite(p)):
   return {'valid':False,'failures':['EMPTY_OR_NONFINITE_PATH']}
  dense=densify(p);swept=SweptChecker(self.hard|~np.asarray(allowed_mask,bool),self.map.resolution,self.map.origin,*self.vehicle[:2])
  if not swept.check(dense):fail.append('SWEPT_BODY_COLLISION_OR_CORRIDOR_EXIT')
  d=np.diff(p,axis=0);lengths=np.linalg.norm(d[:,:2],axis=1);dyaw=wrap(d[:,2]);mid=p[:-1,2]+dyaw/2
  forward=d[:,0]*np.cos(mid)+d[:,1]*np.sin(mid)
  if np.any(forward< -1e-6):fail.append('REVERSE')
  if np.any((lengths<1e-8)&(abs(dyaw)>1e-6)):fail.append('IN_PLACE_ROTATION')
  # Curvature of the heading interpolation, plus circumcircle curvature of
  # geometric samples. This does not assume reported steering metadata.
  # Pose samples of a circular primitive are separated by a chord, not arc
  # length. 2*sin(delta_yaw/2)/chord is its curvature; delta_yaw/chord would
  # falsely reject a mathematically exact 0.40 m-radius Dubins primitive.
  curvature=np.divide(2*np.sin(abs(dyaw)/2),lengths,out=np.zeros_like(lengths),where=lengths>1e-8)
  a=d[:-1,:2];b=d[1:,:2];cross=np.abs(a[:,0]*b[:,1]-a[:,1]*b[:,0]);den=np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)*np.linalg.norm(a+b,axis=1)
  geom=np.divide(2*cross,den,out=np.zeros_like(cross),where=den>1e-12)
  max_curvature=max(float(curvature.max(initial=0)),float(geom.max(initial=0)))
  if max_curvature>1./self.vehicle[2]+.000001:fail.append('CURVATURE')
  endpoint={}
  for role,pose in [('start',p[0]),('goal',p[-1])]:
   target=np.asarray(getattr(query,role));pe=float(np.linalg.norm(pose[:2]-target[:2]));ye=float(abs(wrap(pose[2]-target[2])))
   endpoint[role]={'position_error_m':pe,'yaw_error_rad':ye}
   if pe>.125+1e-9 or ye>math.pi/36+1e-9:fail.append(role.upper()+'_POSE_ERROR')
  labels=self.sample_labels(dense);sequence=labels[np.r_[True,labels[1:]!=labels[:-1]]]
  names=[self.features[int(k)].semantic_id if int(k) in self.features else 'unlabelled' for k in sequence]
  for rule in self.rules.get('forbidden_transitions',[]):
   for i in range(max(0,len(sequence)-2)):
    if list(rule)==names[i:i+3] or list(rule)==list(sequence[i:i+3]):fail.append('FORBIDDEN_TRANSITION')
  dense_delta=np.diff(dense[:,:2],axis=0)
  for rule in self.rules.get('one_way',[]):
   k=next((k for k,f in self.features.items() if f.semantic_id==rule['semantic_id']),None)
   if k is not None:
    projection=dense_delta@np.array([math.cos(rule['yaw']),math.sin(rule['yaw'])])
    if np.any((labels[:-1]==k)&(labels[1:]==k)&(projection < -1e-6)):fail.append('ONE_WAY_VIOLATION')
  if rule_checker:fail.extend(rule_checker(dense))
  return {'valid':not fail,'failures':fail,'swept_body_checked':True,'continuous_enclosure':'midpoint_rectangle_translation_and_rotation_bound',
    'dense_pose_count':len(dense),'max_curvature_1pm':max_curvature,'length_m':float(lengths.sum()),'endpoints':endpoint,
    'audit_wall_ms':(time.monotonic()-started)*1000,'audit_cpu_ms':(time.process_time()-cpu)*1000}
