"""Audit interior travel derived from side edges, never crossing side edges."""
import copy
import numpy as np
from arena_3d_v1.semantic_world import densify
from .channel import Channel


class EdgeChannel(Channel):
    def __init__(self,world,topology,route):
        super().__init__(world,topology,route)
        self.stage_directions=[s.get('travel_direction_xy') for s in self.plan['stages']]
        self.center_mask=topology.center_mask(route)
        self.cache_contract=copy.deepcopy(self.plan)
        self.cache_contract['edge_channel'].pop('diagnostics',None)
    def audit(self,poses):
        result=super().audit(poses)
        result['contract']='interior travel + derived interfaces + selected lane direction; side edges are not crossing gates'
        if not result['valid']:return result
        dense=densify(np.asarray(poses,float),ds=self.world.map.resolution/4)
        r,c=self.world.cells(dense)
        if not np.all(self.center_mask[r,c]):
            result.update(valid=False,failure='EDGE_CHANNEL_OUTSIDE_SELECTED_REGIONS')
            return result
        labels=self.sample(dense);changes=np.r_[0,np.flatnonzero(labels[1:]!=labels[:-1])+1,len(labels)]
        violations=[]
        for stage,(a,b) in enumerate(zip(changes,changes[1:])):
            direction=self.stage_directions[stage]
            if direction is None:continue
            p=dense[a:b];delta=np.diff(p[:,:2],axis=0)
            # Lane travel may move laterally around obstacles, but cannot run
            # backwards along the selected directed road side.
            dots=delta@np.asarray(direction);negative=float(np.maximum(-dots,0).sum())
            if negative>self.world.map.resolution/4:
                violations.append({'stage':stage,'road_id':self.regions[stage],'reverse_progress_m':negative})
            # Fixed endpoint yaw is retained and must also obey traffic direction.
            for index in [a,b-1]:
                if index not in (0,len(dense)-1):continue
                heading=np.array([np.cos(dense[index,2]),np.sin(dense[index,2])])
                if heading@direction < -1e-6:violations.append({'stage':stage,'endpoint_heading_against_traffic':True})
        result['direction_violations']=violations
        if violations:result.update(valid=False,failure='EDGE_CHANNEL_WRONG_TRAVEL_DIRECTION')
        return result
