"""Only owned parking access is staged; recommended road order stays soft."""
import numpy as np

class TerminalContract:
    def __init__(self,world,topology,task):
        self.world=world;self.topology=topology;self.task=task
        self.junctions={i for i,r in topology.regions.items() if r['kind']=='junction_area'}
        self.known=set(topology.regions)
        self.endpoints={role:a for role,a in task['endpoint_access'].items()
            if topology.regions[a['semantic_region_id']]['kind']=='parking_area'}
        self.regions=[]
        if 'start' in self.endpoints:self.regions.append('start')
        self.regions.append('transit')
        if 'goal' in self.endpoints:self.regions.append('goal')
        self.plan={'contract':'terminal-access-v1','task':task['signature'],'stages':self.regions}
    def sample(self,points):
        r,c=self.world.cells(points);ok=(r>=0)&(c>=0)&(r<self.world.map.height)&(c<self.world.map.width)
        labels=np.zeros(len(r),np.int16);labels[ok]=self.topology.subregions[r[ok],c[ok]]
        return labels
    def contains(self,stage,labels):
        role=self.regions[stage]
        if role=='transit':
            parking={a['semantic_region_id'] for a in self.endpoints.values()}
            return np.isin(labels,list(self.known-parking))
        return np.isin(labels,list(self.junctions|{self.endpoints[role]['semantic_region_id']}))
    def crossing(self,a,b,source,target):
        if a==b:return np.ones(len(source),bool)
        if a>b:a,b,source,target=b,a,target,source
        if self.regions[a]=='start':return target==self.endpoints['start']['road_id']
        if self.regions[b]=='goal':return source==self.endpoints['goal']['road_id']
        raise ValueError('UNEXPECTED_TERMINAL_TRANSITION')
