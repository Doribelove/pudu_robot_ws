"""Unchanged extracted geometry/rasterization/route contract helpers."""
from __future__ import annotations
from dataclasses import dataclass,field
from pathlib import Path
from typing import Any
import hashlib,json,os,math
import numpy as np
# Type names occur only as postponed annotations; callers provide validated fields.
SemanticMapV1=object
SemanticFeature=object
import cv2
def canonical_hash(value: Any) -> str:
    """Return a stable SHA-256 for JSON-compatible policy/provenance data."""
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False, default=str,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()

def _world_polygon_cells(semantic_map: SemanticMapV1, feature: SemanticFeature) -> np.ndarray:
    result = []
    for x, y in feature.coordinates:
        col = math.floor((float(x) - semantic_map.origin[0]) / semantic_map.resolution)
        row_from_bottom = math.floor((float(y) - semantic_map.origin[1]) / semantic_map.resolution)
        row = semantic_map.height - 1 - row_from_bottom
        result.append([
            int(np.clip(col, 0, semantic_map.width - 1)),
            int(np.clip(row, 0, semantic_map.height - 1)),
        ])
    return np.asarray(result, dtype=np.int32)

def rasterize_feature(semantic_map: SemanticMapV1, feature: SemanticFeature) -> np.ndarray:
    mask = np.zeros((semantic_map.height, semantic_map.width), dtype=np.uint8)
    points = _world_polygon_cells(semantic_map, feature)
    if feature.geometry_type == "polygon" and len(points) >= 3:
        cv2.fillPoly(mask, [points], 1, lineType=cv2.LINE_8)
    elif feature.geometry_type == "line" and len(points) >= 2:
        cv2.polylines(mask, [points], False, 1, thickness=1, lineType=cv2.LINE_8)
    elif feature.geometry_type == "point" and len(points):
        mask[points[0, 1], points[0, 0]] = 1
    return mask.astype(bool)

@dataclass
class TopologyRoute:
    region_ids: tuple
    interface_ids: tuple
    semantic_ids: tuple
    start: tuple
    goal: tuple
    signature: str
    diagnostics: dict
    binding: dict = field(default_factory=dict)

def json_write(path,value):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False));os.replace(tmp,path)

def cross_section(poly,d,points):
    points=np.asarray(points);right=np.array([d[1],-d[0]]);t=points@d
    low=np.full(len(points),np.inf);high=np.full(len(points),-np.inf)
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        span=(b-a)@d
        if abs(span)<1e-9:continue
        f=(t-a@d)/span;ok=(f>=-1e-8)&(f<=1+1e-8);lat=a@right+f*((b-a)@right)
        low[ok]=np.minimum(low[ok],lat[ok]);high[ok]=np.maximum(high[ok],lat[ok])
    width=high-low;normal=np.divide(high-points@right,width,out=np.full(len(points),np.nan),where=np.isfinite(width)&(width>1e-8))
    return normal,low,high

class R5EndpointRules:
    def endpoint_options(self,query,role):
        pose=getattr(query,role);rc=self.world.map.world_to_cell(*pose[:2]);rid=int(self.subregions[rc]) if rc is not None else 0
        if not rid:raise ValueError('ENDPOINT_NO_REGION')
        if self.regions[rid]['kind']=='parking_area':
            bs=[b for b in self.access['bindings'] if b.get('query_id')==query.query_id and b['endpoint']==role and b['parking_region_id']==rid]
            if len(bs)!=1:raise ValueError('PARKING_OWNERSHIP_UNRESOLVED')
            b=bs[0]
            if math.dist(pose[:2],b['anchor_xy'])>b.get('anchor_tolerance_m',.25):raise ValueError('PARKING_BINDING_POSITION_MISMATCH')
            roads=b.get('allowed_road_ids',[b.get('road_id')])
            return [(rid,x,b) for x in roads if (rid,x) in self.contacts]
        return [(rid,rid,{})]

    def _allowed_direction(self,rid,d,binding):
        feature=self.router.features[self.regions[rid]['name']]
        for rule in self.router.rules.get('one_way',[]):
            if rule['semantic_id']==feature['semantic_id'] and d@np.array([math.cos(rule['yaw']),math.sin(rule['yaw'])])<0:return False
        axis=self.router.axes[rid];vertical=abs(axis[1])>=abs(axis[0]);right=np.array([d[1],-d[0]])
        side=('right' if right[0]>0 else 'left') if vertical else ('top' if right[1]>0 else 'bottom')
        return not binding.get('travel_sides') or side in binding['travel_sides']

