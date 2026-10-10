"""Ordered original-region/gate contract; no extra permanent topology nodes."""
import math
import numpy as np
from scipy.spatial import cKDTree
from arena_3d_v1.semantic_world import densify

def distances_to_gate(xy, gate):
    xy=np.asarray(xy,float).reshape(-1,2)
    if gate.get('temporary'):
        points=np.asarray(gate['contact_band_world_xy'],float)
        return cKDTree(points).query(xy)[0]
    line=np.asarray(gate['geometry'],float)
    result=np.full(len(xy),np.inf)
    for a,b in zip(line,line[1:]):
        delta=b-a; t=np.clip((xy-a)@delta/max(float(delta@delta),1e-20),0,1)
        result=np.minimum(result,np.linalg.norm(xy-a-t[:,None]*delta,axis=1))
    return result

class Channel:
    def __init__(self,world,topology,route):
        self.world=world;self.labels=topology.subregions;self.route=route
        self.plan=topology.channel_plan(route);self.regions=list(route.region_ids)
        self.gates=self.plan['gates']
        # One snapped fine cell plus half diagonal for raster transition detection.
        # This is a gate location tolerance, never a center/obstacle dilation.
        self.gate_tolerance=world.map.resolution*(1+math.sqrt(2)/2)
    def sample(self,xy):
        r,c=self.world.cells(xy);valid=(r>=0)&(c>=0)&(r<self.labels.shape[0])&(c<self.labels.shape[1])
        out=np.zeros(len(r),np.int16);out[valid]=self.labels[r[valid],c[valid]];return out
    def audit(self,poses):
        poses=np.asarray(poses,float)
        if poses.ndim!=2 or poses.shape[1]!=3 or len(poses)<2 or not np.all(np.isfinite(poses)):
            return {'valid':False,'failure':'CHANNEL_INVALID_PATH','crossed_gates':[]}
        dense=densify(poses,ds=self.world.map.resolution/4);labels=self.sample(dense)
        changes=np.flatnonzero(labels[1:]!=labels[:-1])+1
        sequence=[int(labels[0])]+[int(labels[i]) for i in changes]
        expected=self.regions;crossings=[];failure=''
        if sequence!=expected:failure='CHANNEL_REGION_SEQUENCE'
        if not failure:
            for j,i in enumerate(changes):
                p=(dense[i-1,:2]+dense[i,:2])/2;distance=float(distances_to_gate(p[None,:],self.gates[j])[0])
                ok=distance<=self.gate_tolerance
                crossings.append({'gate_id':self.gates[j]['gate_id'],'xy':p.tolist(),'distance_m':distance,'valid':ok})
                if not ok:failure='CHANNEL_WRONG_GATE'
        return {'valid':not failure,'failure':failure,'expected_regions':expected,'visited_regions':sequence,'crossed_gates':crossings,'gate_tolerance_m':self.gate_tolerance,'center_scope':'exact original-region raster; no body-envelope permission','sample_step_m':self.world.map.resolution/4}
