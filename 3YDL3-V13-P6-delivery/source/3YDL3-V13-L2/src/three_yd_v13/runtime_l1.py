"""Runtime component graph; unchanged L1 search, original downstream semantics."""
import copy,math,time,types
from dataclasses import replace
from collections import OrderedDict,defaultdict
import cv2
import numpy as np
from scipy import ndimage
from arena_evaluation.semantic_map import canonical_hash


def components(region,world,polygon,obstacle_mask):
    rr,cc=world.cells(np.asarray(polygon,float))
    r0=max(0,int(rr.min())-2);r1=min(world.map.height,int(rr.max())+3)
    c0=max(0,int(cc.min())-2);c1=min(world.map.width,int(cc.max())+3)
    inside=np.zeros((r1-r0,c1-c0),np.uint8)
    cv2.fillPoly(inside,[np.column_stack((cc-c0,rr-r0)).astype(np.int32)],1)
    static=np.asarray(region.static_hard[r0:r1,c0:c1],bool)
    hard=np.asarray(world.hard[r0:r1,c0:c1],bool)
    dynamic=np.asarray(obstacle_mask[r0:r1,c0:c1],bool)
    before=ndimage.label(inside.astype(bool)&~static)[0]
    after=ndimage.label(inside.astype(bool)&~hard&~dynamic)[0]
    return r0,c0,before,after


def sample_port(world,points):
    p=np.asarray(points,float)
    if len(p)<2:return p
    delta=np.diff(p,axis=0);counts=np.maximum(1,np.ceil(np.linalg.norm(delta,axis=1)/world.map.resolution).astype(int))
    ix=np.repeat(np.arange(len(counts)),counts);offset=np.arange(int(counts.sum()))-np.repeat(np.cumsum(counts)-counts,counts)
    return np.vstack((p[ix]+(delta/counts[:,None])[ix]*offset[:,None],p[-1]))


def labels_at(world,data,points,after=True):
    r0,c0,before,current=data;lab=current if after else before
    rr,cc=world.cells(points);rr=rr-r0;cc=cc-c0
    values=np.zeros(len(points),np.int32)
    for dr,dc in [(0,0),(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
        r=rr+dr;c=cc+dc;valid=(values==0)&(r>=0)&(c>=0)&(r<lab.shape[0])&(c<lab.shape[1])
        values[valid]=lab[r[valid],c[valid]]
    return values


def port_labels(world,data,points,after=True):
    points=sample_port(world,points);r0,c0,before,current=data;lab=current if after else before
    rr,cc=world.cells(points);rr=rr-r0;cc=cc-c0;result=set()
    for dr in (-1,0,1):
        for dc in (-1,0,1):
            r=rr+dr;c=cc+dc;valid=(r>=0)&(c>=0)&(r<lab.shape[0])&(c<lab.shape[1])
            result.update(int(x) for x in np.unique(lab[r[valid],c[valid]]) if x)
    return result


def fresh(base):
    t=copy.copy(base);t.tasks={};t.contexts={};t._cache={};t._first_phase_cache=OrderedDict();t.router=copy.copy(base.router)
    return t


class RuntimeSemanticTopology:
    def __init__(self,base,world,dynamic):
        tick=time.monotonic();self.base=base;self.world=world;self.top=fresh(base)
        self.top.adj=defaultdict(set,{a:set(v) for a,v in base.adj.items()})
        self.top._static_transitions={k:dict(v) for k,v in base._static_transitions.items()}
        self.data={};self.ports={};self.blocked=[];self.query_tops={};self.node_map={};self.reverse_nodes={}
        boxes=[(p.min(axis=0),p.max(axis=0)) for p in dynamic.obstacles.values()]
        for rid,region in base.regions.items():
            if region['kind']=='parking_area':continue # frozen L1 never transits parking areas
            poly=np.asarray(base.router.features[region['name']]['coordinates'],float)
            lo=poly.min(axis=0);hi=poly.max(axis=0)
            if not any(np.all(hi>=a) and np.all(lo<=b) for a,b in boxes):continue
            data=components(base,world,poly,dynamic.mask);self.data[rid]=data
            for other in base.adj[rid]:
                points=base.contacts[rid,other]
                self.ports[rid,other]=(port_labels(world,data,points,False),port_labels(world,data,points,True))
            for previous in base.adj[rid]:
                for nxt in base.adj[rid]:
                    a0,a1=self.ports[rid,previous];b0,b1=self.ports[rid,nxt]
                    if not(a0&b0) or (a1&b1):continue
                    prepared=self.top._static_transitions.get((previous,rid),{}).get(nxt)
                    if prepared is not None and prepared[4] is None:
                        self.top._static_transitions[previous,rid][nxt]=(*prepared[:4],'runtime_obstacle_disconnected')
                        self.blocked.append([int(previous),int(rid),int(nxt)])
        self.top.key=canonical_hash([base.key,'V13-P7-r1-runtime-components',dynamic.version,dynamic.content_hash])
        self.top.router.key=self.top.key
        self.detail={'schema':'V13-P7-r1-runtime-L1','obstacle_version':dynamic.version,'affected_regions':sorted(self.data),
            'blocked_transitions':self.blocked,'frozen_L1_search_unchanged':True,'routes':[],
            'routing_model':'4-connected region component graph; vehicle/curvature feasibility remains downstream'}
        self.graph=self._component_graph()
        self.detail['preparation_ms']=(time.monotonic()-tick)*1000
        self.top.v13_runtime_l1=self.detail

    def _component_graph(self):
        t=fresh(self.top);t.regions=copy.deepcopy(self.base.regions);t.parent_semantics=dict(self.base.parent_semantics)
        t.router.features=dict(self.base.router.features);t.router.polygons=dict(self.base.router.polygons);t.router.axes=dict(self.base.router.axes)
        next_id=max(t.regions)+1
        for rid,data in self.data.items():
            # A split is represented by separate graph identities; unchanged
            # L1's no-revisit rule now applies to components, not the whole lane.
            labels=np.unique(data[3]);labels=labels[labels>0]
            if len(labels)<=1:continue
            for label in labels:
                node=next_id;next_id+=1;self.node_map[rid,int(label)]=node;self.reverse_nodes[node]=rid
                name=self.base.regions[rid]['name']+'#component'+str(int(label))
                t.regions[node]={**self.base.regions[rid],'id':node,'name':name}
                t.parent_semantics[node]=self.base.parent_semantics[rid]
                t.router.features[name]=self.base.router.features[self.base.regions[rid]['name']]
                if rid in self.base.router.axes:t.router.axes[node]=self.base.router.axes[rid];t.router.polygons[node]=self.base.router.polygons[rid]
        contacts=defaultdict(list);adj=defaultdict(set)
        for (a,b),raw in self.base.contacts.items():
            if b not in self.base.adj[a]:continue
            if a not in self.data and b not in self.data:
                contacts[a,b]=list(raw);adj[a].add(b);continue
            points=sample_port(self.world,raw)
            def identities(rid):
                if rid not in self.data:return np.full(len(points),rid,int)
                ls=labels_at(self.world,self.data[rid],points)
                return np.array([self.node_map.get((rid,int(l)),rid) if l else -1 for l in ls],int)
            aa=identities(a);bb=identities(b)
            for na,nb in np.unique(np.column_stack((aa,bb)),axis=0):
                if na<0 or nb<0:continue
                keep=(aa==na)&(bb==nb);contacts[int(na),int(nb)].extend(points[keep].tolist());adj[int(na)].add(int(nb))
        # Preserve original contact direction symmetry, allowing only edges
        # whose source and destination free components actually touch a portal.
        t.contacts=contacts;t.adj=adj
        t.contact_centers={key:tuple(map(float,np.asarray(p).mean(axis=0))) for key,p in contacts.items() if p}
        # Execute the frozen on-the-fly transition/rule checks on this graph.
        t._static_transitions={};t._directed_distances=None;t._directed_state_index={}
        t.edges_count=sum(map(len,adj.values()))//2
        self.detail['virtual_component_to_original']={str(k):v for k,v in self.reverse_nodes.items()}
        self.detail['runtime_graph_nodes']=len(adj);self.detail['runtime_graph_edges']=t.edges_count
        return t

    def _options(self,q,role,heading):
        out=[];pose=getattr(q,role)
        for region,road,binding in self.base.endpoint_options(q,role):
            roads=[road];new_region=region
            if region==road and road in self.data:
                label=int(labels_at(self.world,self.data[road],np.asarray([pose[:2]],float))[0])
                if not label:continue
                roads=[self.node_map.get((road,label),road)];new_region=roads[0]
            elif region!=road and any(x==road for x in self.reverse_nodes.values()):
                roads=[n for n,original in self.reverse_nodes.items() if original==road and (region,n) in self.graph.contacts]
            for node in roads:
                binding=dict(binding)
                if heading and region==road and self.base.regions[road]['kind']=='lane':
                    h=np.array([math.cos(pose[2]),math.sin(pose[2])]);axis=self.base.router.axes[road];dot=float(axis@h)
                    if abs(dot)>.5:
                        d=axis*(1 if dot>0 else -1);right=np.array([d[1],-d[0]]);vertical=abs(axis[1])>=abs(axis[0])
                        side=('right' if right[0]>0 else 'left') if vertical else ('top' if right[1]>0 else 'bottom')
                        allowed=binding.get('travel_sides')
                        if allowed and side not in allowed:continue
                        binding['travel_sides']=[side]
                out.append((new_region,node,binding))
        return out

    def __getattr__(self,name):return getattr(self.top,name)

    def release_workspaces(self):
        self.data.clear();self.ports.clear();self.query_tops.clear()

    def routes(self,query,limit=1):
        tick=time.monotonic();key=(query.query_id,tuple(query.start),tuple(query.goal));heading=True
        def get_top(prefer):
            k=(key,prefer)
            if k not in self.query_tops:
                top=fresh(self.graph)
                top.endpoint_options=types.MethodType(lambda t,q,role:self._options(q,role,prefer),top)
                self.query_tops[k]=top
            return self.query_tops[k]
        t=get_top(True);raw=t.routes(query,limit=limit)
        if not raw:heading=False;t=get_top(False);raw=t.routes(query,limit=limit)
        routes=[]
        for r in raw:
            original=lambda rid:self.reverse_nodes.get(int(rid),int(rid))
            task=copy.deepcopy(t.tasks[r.signature]);task.pop('signature',None)
            task['region_sequence']=[original(x) for x in task['region_sequence']]
            task['runtime_component_sequence']=list(r.region_ids)
            for j in task.get('junction_joins',[]):
                for f in ('junction_region_id','incoming_road_id','outgoing_road_id'):
                    if j.get(f) is not None:j[f]=original(j[f])
            signature=canonical_hash(task);task['signature']=signature
            context=copy.deepcopy(t.contexts[r.signature])
            for k in ('roads','unique_roads'):context[k]=[(original(rid),d) for rid,d in context[k]]
            for v in context['endpoint_access'].values():
                for k in ('semantic_region_id','road_id'):v[k]=original(v[k])
            self.top.tasks[signature]=task;self.top.contexts[signature]=context
            routes.append(replace(r,region_ids=tuple(original(x) for x in r.region_ids),signature=signature,
                diagnostics={**r.diagnostics,'runtime_component_sequence':list(r.region_ids),'L1_runtime_components':True}))
        self.detail['routes'].append({'start':list(query.start),'goal':list(query.goal),'limit':limit,'heading_preferred':heading,
            'wall_ms':(time.monotonic()-tick)*1000,'region_sequences':[list(r.region_ids) for r in routes],
            'component_sequences':[list(r.region_ids) for r in raw],'search':copy.deepcopy(getattr(t,'_probe_stats',{}))})
        return routes
