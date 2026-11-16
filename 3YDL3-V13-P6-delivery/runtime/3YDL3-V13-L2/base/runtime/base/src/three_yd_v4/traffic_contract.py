"""r1 separates source traffic prohibitions from L1's route recommendation.

No new map geometry or obstacle clearing. The prior all-step route-order audit
is retained as a separate comparison, never silently relabelled as equivalent.
"""
import math
import numpy as np
from arena_3d_v1.semantic_world import densify
from .guidance import TrafficAudit as NominalRouteAudit

class TrafficAudit(NominalRouteAudit):
    def audit(self,poses):
        nominal=super().audit(poses)
        p=densify(np.asarray(poses,float),ds=.025);r,c=self.world.cells(p)
        if np.any((r<0)|(c<0)|(r>=self.world.map.height)|(c>=self.world.map.width)):
            return {'valid':False,'failure':'OUTSIDE_MAP','failures':['OUTSIDE_MAP'],'nominal_route_audit':nominal}
        labels=self.topology.subregions[r,c];sequence=list(map(int,labels[np.r_[True,labels[1:]!=labels[:-1]]]))
        rules=self.topology.router.rules;failures=[];lane_ids={i for i,v in self.topology.regions.items() if v['kind']=='lane'}
        features=self.topology.router.features
        semantic=lambda rid:features[self.topology.regions[rid]['name']]['semantic_id'] if rid in self.topology.regions else None
        deltas=np.diff(p[:,:2],axis=0)
        for rule in rules.get('one_way',[]):
            ids=[i for i in lane_ids if semantic(i)==rule['semantic_id']]
            direction=np.array([math.cos(rule['yaw']),math.sin(rule['yaw'])])
            if np.any(np.isin(labels[:-1],ids)&np.isin(labels[1:],ids)&(deltas@direction < -1e-6)):failures.append('EXPLICIT_ONE_WAY_VIOLATION')
        for a,j,b in zip(sequence,sequence[1:],sequence[2:]):
            if [semantic(a),semantic(j),semantic(b)] in rules.get('forbidden_transitions',[]):failures.append('EXPLICIT_FORBIDDEN_TURN')
        # A parking endpoint is still tied to its explicitly owned road. L1's
        # nearest projection does not force the exact physical entrance point.
        access=[]
        for role,a in self.task['endpoint_access'].items():
            rid=a['semantic_region_id']
            if rid not in self.endpoint_parking:continue
            seq=sequence if role=='start' else sequence[::-1]
            first_road=next((i for i in seq[1:] if i in lane_ids),None)
            allowed=(seq and seq[0]==rid and first_road==a['road_id'])
            # Only junctions may bridge an annotated parking/road contact.
            before=[]
            for i in seq[1:]:
                if i in lane_ids:break
                before.append(i)
            allowed &= all(self.topology.regions.get(i,{}).get('kind')=='junction_area' for i in before)
            if not allowed:failures.append('PARKING_OWNED_ROAD_ACCESS')
            access.append({'role':role,'owned_road':a['road_id'],'actual_access_road':first_road,'valid':bool(allowed)})
        unknown=labels==0
        if np.any(unknown) and np.linalg.norm(deltas[unknown[:-1]|unknown[1:]],axis=1).sum()>.30:failures.append('UNBOUND_SEMANTIC_TRAVEL')
        failures=sorted(set(failures))
        return {'valid':not failures,'failure':';'.join(failures),'failures':failures,
            'contract':'r1_source_prohibitions_hard_topology_and_right_preference_soft',
            'visited_roads':[i for i in sequence if i in lane_ids],'endpoint_access':access,
            'nominal_route_audit':nominal,'nominal_route_valid':nominal.get('valid',False),
            'source_parking_semantics':'soft:true, hard:false; no parking topology edges are added'}
