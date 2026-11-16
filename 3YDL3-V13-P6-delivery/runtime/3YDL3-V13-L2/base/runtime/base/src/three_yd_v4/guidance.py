"""Finite edge guidance with owned-road matching and inward vehicle offset."""
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.semantic_world import digest, densify
from arena_3d_v1.semantic_reference import RoutePreference


def sample_uses(task, spacing=.15):
    samples = []
    for order, use in enumerate(task['ordered_uses']):
        if use['kind'] != 'lane':
            continue
        p = np.asarray(use['geometry'], float)
        for a, b in zip(p, p[1:]):
            length = float(np.linalg.norm(b-a))
            if length < 1e-8:
                continue
            tangent = (b-a)/length
            for t in np.linspace(0., 1., max(2, math.ceil(length/spacing)+1)):
                samples.append([*(a+t*(b-a)), *tangent, use['region_id'], order])
    return np.asarray(samples, float).reshape((-1,6))


def road_groups(task):
    groups=[];last_kind=None
    for use in task['ordered_uses']:
        if use['kind']=='lane':
            if last_kind=='lane' and groups[-1][-1]['region_id']==use['region_id']:groups[-1].append(use)
            else:groups.append([use])
        last_kind=use['kind']
    return groups


def edge_field(world, view, topology, task):
    started = time.monotonic()
    shape = view.safe.shape
    pot = np.zeros(shape, np.float32)
    normalized = np.full(shape, np.nan, np.float32)
    eligible = np.zeros(shape, bool)
    ambiguous = np.zeros(shape, bool)
    direction = np.zeros((*shape,2), np.float32)
    progress = np.full(shape, -1, np.int16)
    rr, cc = np.indices(shape)
    xy = view.world_points(rr.ravel(), cc.ravel())
    fr, fc = world.cells(xy)
    valid = (fr>=0)&(fc>=0)&(fr<world.map.height)&(fc<world.map.width)
    labels = np.zeros(len(xy), np.int16)
    labels[valid] = topology.subregions[fr[valid],fc[valid]]
    labels = labels.reshape(shape)
    samples = sample_uses(task)
    # This offset is guidance, not a safety certificate. Full rectangular swept
    # checks and the frozen vehicle model still determine path acceptance.
    offset = world.vehicle[1] + .05 + math.sqrt(2)*world.map.resolution/2
    for rid in np.unique(samples[:,4]).astype(int) if len(samples) else []:
        ids = np.flatnonzero(labels.ravel()==rid)
        refs = samples[samples[:,4]==rid]
        points = xy[ids]
        distance, ix = cKDTree(refs[:,:2]).query(points)
        nearest = refs[ix]
        left = np.column_stack((-nearest[:,3], nearest[:,2]))
        signed = np.sum((points-nearest[:,:2])*left, axis=1)
        # Competing visits on the same physical road need progress state. If
        # their attraction is indistinguishable, abstain and audit road order.
        amb = np.zeros(len(ids), bool)
        for order in np.unique(refs[:,5]):
            other = refs[refs[:,5]==order]
            d = cKDTree(other[:,:2]).query(points)[0]
            opposite = other[0,2:4] @ nearest[:,2:4].T < -.5
            amb |= (nearest[:,5]!=order) & opposite & (d<=distance+.2)
        good = (signed>=-.05)&~amb
        rows, cols = np.unravel_index(ids,shape)
        target_distance = np.linalg.norm(points-(nearest[:,:2]+offset*left),axis=1)
        pot[rows[good],cols[good]] = np.minimum(1.,target_distance[good]/2.)
        direction[rows[good],cols[good]] = nearest[good,2:4]
        progress[rows[good],cols[good]] = nearest[good,5].astype(np.int16)
        ambiguous[rows,cols] = amb
        # Physical screen-side classification is normalized using the original
        # polygon cross-section, independent of occupied cars. No width>=4m rule.
        poly = topology.router.polygons[rid]
        axis = topology.router.axes[rid]
        lateral = np.array([-axis[1],axis[0]])
        lat = poly@lateral
        width = float(lat.max()-lat.min())
        if width > 2*offset:
            normalized[rows[good],cols[good]] = signed[good]/width
            eligible[rows[good],cols[good]] = True
    key = canonical_hash(['v4-edge-field-v1',task['signature'],view.key,offset,digest(pot),digest(direction)])
    preference = RoutePreference(pot,normalized,eligible,ambiguous,key,
        {'build_ms':(time.monotonic()-started)*1000,'inward_target_offset_m':offset,
         'source':'selected original edge uses, owned road, travel direction',
         'eligible_cells':int(eligible.sum()),'ambiguous_cells':int(ambiguous.sum()),
         'junction_edge_attraction':False,'offset_is_not_collision_certificate':True})
    return preference,direction,progress,labels


class TrafficAudit:
    """Road/access legality; no compulsory gate, no exact junction polygon order."""
    def __init__(self, world, topology, route):
        self.world=world;self.topology=topology;self.route=route;self.task=topology.task(route)
        self.roads=[]
        self.groups=road_groups(self.task)
        for group in self.groups:
            u=group[0]
            v=sum((np.diff(np.asarray(x['geometry']),axis=0).sum(axis=0) for x in group),np.zeros(2))
            axis=topology.router.axes[u['region_id']]
            d=axis*(1 if v@axis>=0 else -1)
            self.roads.append((u['region_id'],d))
        self.endpoint_parking={a['semantic_region_id'] for a in self.task['endpoint_access'].values()
                               if topology.regions[a['semantic_region_id']]['kind']=='parking_area'}

    def audit(self, poses):
        p=densify(np.asarray(poses,float),ds=.025)
        r,c=self.world.cells(p)
        if np.any((r<0)|(c<0)|(r>=self.world.map.height)|(c>=self.world.map.width)):
            return {'valid':False,'failure':'OUTSIDE_MAP'}
        labels=self.topology.subregions[r,c]
        lane_ids={rid for rid,x in self.topology.regions.items() if x['kind']=='lane'}
        failures=[];visits=[];stage=0;previous=None;between=False
        for i,rid in enumerate(labels):
            rid=int(rid)
            if rid not in lane_ids:
                between=True
                if rid in self.topology.regions and self.topology.regions[rid]['kind']=='parking_area':
                    if rid not in self.endpoint_parking:failures.append('UNAUTHORIZED_PARKING_TRANSIT')
                # Unknown semantic gaps never become invented physical walls;
                # nontrivial excursions are rejected below as unbound traffic.
                continue
            if rid!=previous or between:
                returning=previous is not None
                visits.append(rid);previous=rid
                if (returning and between and stage+1<len(self.roads) and rid==self.roads[stage+1][0]):stage+=1
                elif stage<len(self.roads) and rid!=self.roads[stage][0]:
                    if stage+1<len(self.roads) and rid==self.roads[stage+1][0]:stage+=1
                    else:failures.append('ROAD_SEQUENCE_OR_OWNERSHIP')
                between=False
            if stage>=len(self.roads) or rid!=self.roads[stage][0]:continue
            if i and labels[i-1]==rid:
                delta=p[i,:2]-p[i-1,:2]
                if delta@self.roads[stage][1]<-math.sin(math.radians(5))*np.linalg.norm(delta)-1e-6:
                    failures.append('WRONG_TRAVEL_DIRECTION')
        if stage!=len(self.roads)-1:failures.append('ROAD_SEQUENCE_INCOMPLETE')
        # Parking may only be the initial/final region; endpoint-specific owned
        # road must immediately follow/precede it, irrespective of distance.
        compressed=list(map(int,labels[np.r_[True,labels[1:]!=labels[:-1]]]))
        rules=getattr(self.topology.router,'rules',{})
        features=getattr(self.topology.router,'features',{})
        if features:
            def semantic_id(rid):
                return features[self.topology.regions[rid]['name']]['semantic_id']
            for a,j,b in zip(compressed,compressed[1:],compressed[2:]):
                if a in lane_ids and b in lane_ids and self.topology.regions.get(j,{}).get('kind')=='junction_area':
                    if [semantic_id(a),semantic_id(j),semantic_id(b)] in rules.get('forbidden_transitions',[]):
                        failures.append('FORBIDDEN_TURN')
                    if a==b and j not in rules.get('allowed_u_turn_junctions',[]):
                        # Check whether this re-entry actually reverses the road
                        # direction; traversing a painted junction straight is fine.
                        hits=np.flatnonzero(labels==j)
                        if len(hits) and hits[0]>0 and hits[-1]+1<len(p):
                            before=np.array([math.cos(p[hits[0]-1,2]),math.sin(p[hits[0]-1,2])])
                            after=np.array([math.cos(p[hits[-1]+1,2]),math.sin(p[hits[-1]+1,2])])
                            if before@after<-.5:failures.append('UNAUTHORIZED_U_TURN')
        for role in ('start','goal'):
            a=self.task['endpoint_access'][role];rid=a['semantic_region_id']
            if rid not in self.endpoint_parking:continue
            seq=compressed if role=='start' else compressed[::-1]
            if len(seq)<2 or seq[0]!=rid or seq[1]!=a['road_id']:
                failures.append('PARKING_OWNED_ROAD_ACCESS')
            if rid in seq[1:]:failures.append('PARKING_REENTRY_TRANSIT')
        unknown=labels==0
        if np.any(unknown):
            ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
            if float(ds[unknown[:-1]|unknown[1:]].sum())>.30:failures.append('UNBOUND_SEMANTIC_TRAVEL')
        failures=sorted(set(failures))
        return {'valid':not failures,'failure':';'.join(failures),'failures':failures,
                'visited_roads':visits,'expected_roads':[r for r,_ in self.roads],
                'exact_region_stages_required':False,'mandatory_gates_required':False}
