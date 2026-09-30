"""R5 L1: ordered regions, direction, gates, and local right boundaries."""
import json,time,heapq,itertools,math
from pathlib import Path
from types import SimpleNamespace
from collections import defaultdict
import numpy as np
import cv2
from arena_evaluation.semantic_map import canonical_hash
from arena_3d_v1.multires_topology import TopologyRoute
from three_yd_v3.topology import OutlineTopology

class R5Topology(OutlineTopology):
    SCHEMA='3YD-V4-r5-region-gates-right-boundary-v1'
    def __init__(self,world,graph_path,cache_root):
        super().__init__(world,graph_path,cache_root)
        s=json.loads((Path(graph_path).parent/'semantic.json').read_text())
        self.access=json.loads((Path(graph_path).parent/'parking_access.json').read_text())
        features={f['properties']['source_name']:f for f in s['features']};axes={};polygons={}
        for rid,r in self.regions.items():
            if r['kind']!='lane':continue
            p=np.array(features[r['name']]['coordinates']);polygons[rid]=p
            box=cv2.boxPoints(cv2.minAreaRect(p.astype(np.float32)));v=np.roll(box,-1,axis=0)-box;l=np.linalg.norm(v,axis=1);axis=v[np.argmax(l)]/max(l)
            if axis[np.argmax(abs(axis))]<0:axis=-axis
            axes[rid]=axis
        self.router=SimpleNamespace(axes=axes,polygons=polygons,features=features,rules=s.get('traffic_rules',{}),key=canonical_hash([self.SCHEMA,s,self.access]))
        self.contacts=defaultdict(list)
        for e in self.graph['edges']:
            if e['kind']=='shared_outline' and len(e['regions'])==2:
                a,b=e['regions'];self.contacts[a,b].extend(e['geometry']);self.contacts[b,a].extend(e['geometry'])
        for (a,b),rc in self.parking_contacts.items():
            xy=world.world_points(rc[:,0],rc[:,1]);self.contacts[a,b].extend(xy.tolist());self.contacts[b,a].extend(xy.tolist())
        self.adj=defaultdict(set)
        for a,b in self.contacts:self.adj[a].add(b)
        # A lane/lane contact overlapped by a declared common junction must
        # route through that junction, not silently skip its semantic stage.
        for a in list(self.adj):
            for b in list(self.adj[a]):
                if self.regions[a]['kind']==self.regions[b]['kind']=='lane':
                    common=self.adj[a]&self.adj[b]
                    if any(self.regions[j]['kind']=='junction_area' for j in common):self.adj[a].discard(b)
        self.key=self.router.key;self.tasks={};self._cache={}
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
    def routes(self,query,limit=2):
        key=(query.query_id,tuple(query.start),tuple(query.goal))
        if key in self._cache:return self._cache[key]
        heap=[];serial=itertools.count();found=[];seen=set();expansions=0
        for sr,road,b in self.endpoint_options(query,'start'):
            seq=(sr,) if sr==road else (sr,road)
            contact=np.asarray(self.contacts.get((sr,road),[query.start[:2]]));entry=np.array(query.start[:2]) if sr==road else contact[np.argmin(np.linalg.norm(contact-np.asarray(query.start[:2]),axis=1))]
            for gr,groad,gb in self.endpoint_options(query,'goal'):heapq.heappush(heap,(0.,next(serial),seq,entry,[],(sr,road,b,gr,groad,gb)))
        while heap and len(found)<limit and expansions<10000:
            cost,_,seq,entry,visits,ends=heapq.heappop(heap);expansions+=1
            rid=seq[-1];sr,sroad,sb,gr,groad,gb=ends
            targets=([None] if rid==groad else [])+sorted(self.adj[rid])
            for nxt in targets:
                terminal=nxt is None
                if nxt is not None:
                    if self.regions[nxt]['kind']=='parking_area':continue
                    if seq.count(nxt)>=2 or len(seq)>18:continue
                    if len(seq)>1 and nxt==seq[-2] and self.regions[rid]['kind']=='junction_area' and rid not in self.router.rules.get('allowed_u_turn_junctions',[]):continue
                if terminal and gr==groad:exit=np.array(query.goal[:2])
                elif terminal:
                    contact=np.asarray(self.contacts[rid,gr]);exit=contact[np.argmin(np.linalg.norm(contact-np.asarray(query.goal[:2]),axis=1))]
                else:exit=np.mean(self.contacts[rid,nxt],axis=0)
                d=None
                if self.regions[rid]['kind']=='lane':
                    axis=self.router.axes[rid];dot=(exit-entry)@axis
                    if abs(dot)<.01:continue
                    d=axis*(1 if dot>=0 else -1)
                    if not self._allowed_direction(rid,d,sb if rid==sroad and not visits else {}):continue
                    if terminal and not self._allowed_direction(rid,d,gb):continue
                    if not visits and sr==sroad and d@np.array([math.cos(query.start[2]),math.sin(query.start[2])])<0:continue
                    if terminal and gr==groad and d@np.array([math.cos(query.goal[2]),math.sin(query.goal[2])])<0:continue
                if len(seq)>1 and self.regions[rid]['kind']=='junction_area' and nxt is not None:
                    sem=lambda x:self.router.features[self.regions[x]['name']]['semantic_id']
                    if [sem(seq[-2]),sem(rid),sem(nxt)] in self.router.rules.get('forbidden_transitions',[]):continue
                visit={'region_id':rid,'kind':self.regions[rid]['kind'],'occurrence':len(visits),'direction':None if d is None else d.tolist(),'entry_region':seq[-2] if len(seq)>1 else None,'exit_region':gr if terminal and gr!=groad else nxt,'entry_area':self.contacts.get((seq[-2],rid),[]) if len(seq)>1 else [],'exit_area':self.contacts.get((rid,gr if terminal else nxt),[]) if nxt is not None or gr!=groad else [],'entry_hint':entry.tolist(),'exit_hint':exit.tolist(),'hints_are_not_mandatory':True}
                vv=visits+[visit];newcost=cost+float(np.linalg.norm(exit-entry))
                if terminal:
                    sig=tuple((v['region_id'],tuple(v['direction'] or [])) for v in vv)
                    if sig in seen:continue
                    seen.add(sig);regions=list(seq)+([gr] if gr!=groad else [])
                    if sr!=sroad:vv=[{'region_id':sr,'kind':'parking_area','direction':None,'occurrence':-1}]+vv
                    if gr!=groad:vv.append({'region_id':gr,'kind':'parking_area','direction':None,'occurrence':len(vv)})
                    task={'schema':self.SCHEMA,'visits':vv,'priority_region_ids':list(dict.fromkeys(regions)),'start':list(query.start),'goal':list(query.goal),'endpoint_access':{role:{'semantic_region_id':a,'road_id':b,'original_pose':list(p),'xy':list(p[:2]),'allow_local_cross_road':bool(binding.get('allow_local_cross_road'))} for role,a,b,p,binding in [('start',sr,sroad,query.start,sb),('goal',gr,groad,query.goal,gb)]},'map_hash':self.world.map.sha256,'rules_hash':self.router.key,'directed_outline_dependencies':False,'ordered_outline_uses':False,'legal_transitions':self.router.rules,'raster_corridor_produced':False,'l2_contract':'local right boundary is soft; junction and parking are free; reverse road motion forbidden'}
                    task['visits']=self.expand_visits(task['visits']);regions=[v['region_id'] for v in task['visits']];task['priority_region_ids']=list(dict.fromkeys(regions))
                    from .r5_guidance import local_right_boundary
                    for visit in task['visits']:
                        if visit['kind']=='lane':visit['right_boundary']=local_right_boundary(self,visit).tolist()
                        else:visit['right_boundary']=[]
                    signature=canonical_hash(task);task['signature']=signature;self.tasks[signature]=task
                    found.append(TopologyRoute(tuple(regions),(),tuple(self.parent_semantics[x] for x in regions),tuple(query.start),tuple(query.goal),signature,{'region_task':task,'region_search_expansions':expansions},{'world_key':self.world.key,'topology_key':self.key,**self.world.binding}))
                else:heapq.heappush(heap,(newcost,next(serial),seq+(nxt,),exit,vv,ends))
        self._cache[key]=found;return found
    def expand_visits(self,visits):
        expanded=[]
        for visit in visits:
            if visit['kind']!='lane':expanded.append(dict(visit));continue
            rid=visit['region_id'];d=np.asarray(visit['direction']);a=np.asarray(visit['entry_hint']);b=np.asarray(visit['exit_hint']);crossings=[]
            for j in self.adj[rid]:
                if self.regions[j]['kind']!='junction_area' or j in (visit['entry_region'],visit['exit_region']):continue
                pts=np.asarray(self.contacts[rid,j]);center=pts.mean(axis=0);t=float(center@d)
                if a@d+.3<t<b@d-.3:crossings.append((t,j,pts,center))
            current=dict(visit);current['entry_hint']=a.tolist()
            for _,j,pts,center in sorted(crossings):
                # Area extents provide hints only, retaining the whole contact.
                lo,hi=float(np.min(pts@d)),float(np.max(pts@d));before=center+(lo-center@d)*d;after=center+(hi-center@d)*d
                first={**current,'exit_region':j,'exit_area':pts.tolist(),'exit_hint':before.tolist()};expanded.append(first)
                expanded.append({'region_id':j,'kind':'junction_area','direction':None,'entry_region':rid,'exit_region':rid,'entry_area':pts.tolist(),'exit_area':pts.tolist(),'entry_hint':before.tolist(),'exit_hint':after.tolist(),'hints_are_not_mandatory':True})
                current={**visit,'entry_region':j,'entry_area':pts.tolist(),'entry_hint':after.tolist()}
            expanded.append(current)
        for i,v in enumerate(expanded):v['occurrence']=i
        return expanded

    def plan(self,query,**kw):
        routes=self.routes(query)
        if not routes:raise RuntimeError('NO_LEGAL_REGION_ROUTE')
        return routes[0]
    def task(self,route):return self.tasks[route.signature]
    def channel_plan(self,route):return self.task(route)
