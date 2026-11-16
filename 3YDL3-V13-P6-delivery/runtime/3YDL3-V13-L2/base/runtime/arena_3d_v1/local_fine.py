"""Bounded fine patches joined to conservative coarse centers in one search.

Fine patches are selected by a *proposal* on any-fine-safe coarse blocks.
The proposal is never an accepted reference. Every hybrid edge is checked
against original fine safe cells, corner clearance, and direction rules.
"""
import heapq,itertools,math,time
from types import SimpleNamespace
import numpy as np
from scipy.ndimage import binary_dilation
from .multires_grid import aggregate,make_roi,guide_astar

STEPS=[(r,c) for r in (-1,0,1) for c in (-1,0,1) if r or c]

class LocalFineGraph:
    def __init__(self,world,view,mask,start,goal,signature,*,max_fine_cells=32000,patch_radius_m=.6,waypoints=()):
        self.world=world;self.view=view;self.safe=world.safe&mask;self.start=tuple(start);self.goal=tuple(goal);self.waypoints=[tuple(p) for p in waypoints]
        self.coarse=view.safe&view.corridor(mask)
        loose=aggregate(self.safe,3);proposal=SimpleNamespace(**view.__dict__);proposal.safe=loose
        proposal.cell=view.cell;proposal.world_points=view.world_points
        st,go=view.cell(world.world_points([start[0]],[start[1]])[0]),view.cell(world.world_points([goal[0]],[goal[1]])[0])
        anchors=[st]+[view.cell(world.world_points([p[0]],[p[1]])[0]) for p in self.waypoints]+[go]
        pieces=[]
        for a,b in zip(anchors,anchors[1:]):
            roi=make_roi(proposal,loose,a,b,signature)
            piece,_=guide_astar(proposal,roi,timeout=3.);pieces.append(piece)
        path=np.vstack(pieces)
        bad=path[~self.coarse[path[:,0],path[:,1]]]
        seeds=np.zeros(loose.shape,bool)
        if len(bad):seeds[bad[:,0],bad[:,1]]=True
        # Endpoints retain their original fine coordinates, even inside a
        # free coarse block; the short endpoint collars bridge the two grids.
        for anchor in anchors:seeds[anchor]=True
        patch=binary_dilation(seeds,iterations=max(1,int(math.ceil(patch_radius_m/view.map.resolution))))&loose
        # Aligned inverse block lookup, respecting padded top/right borders.
        rr,cc=np.nonzero(patch);fine_patch=np.zeros(self.safe.shape,bool)
        for dr in range(3):
            for dc in range(3):
                fr=world.map.height-1-((view.map.height-1-rr)*3+dr);fc=cc*3+dc
                valid=(fr>=0)&(fr<world.map.height)&(fc<world.map.width)
                fine_patch[fr[valid],fc[valid]]=True
        fine_patch&=self.safe
        if fine_patch.sum()>max_fine_cells:raise RuntimeError('LOCAL_FINE_BUDGET_EXCEEDED')
        cr,cc=np.nonzero(self.coarse);xy=view.world_points(cr,cc);fr,fc=world.cells(xy)
        active=fine_patch.copy();active[fr,fc]=True
        self.patch=fine_patch;self.active=active;self.coarse_centers=np.zeros(active.shape,bool);self.coarse_centers[fr,fc]=True
        self.stats={'local_fine_cells':int(fine_patch.sum()),'coarse_nodes':len(cr),'active_nodes':int(active.sum()),'partial_blocks':len(bad),'patch_radius_m':patch_radius_m,'max_fine_cells':max_fine_cells}
        self.directions=[]
        for rule in world.rules.get('one_way',[]):
            labels=[k for k,f in world.features.items() if f.semantic_id==rule['semantic_id']]
            self.directions.append((np.isin(world.labels,labels),float(rule['yaw'])))

    def _edge(self,u,v):
        h,w=self.safe.shape;r,c=v
        if not(0<=r<h and 0<=c<w and self.active[r,c]):return False
        dr,dc=r-u[0],c-u[1];steps=max(abs(dr),abs(dc))
        if any((m[u] or m[v]) and dc*math.cos(yaw)-dr*math.sin(yaw)<-1e-6 for m,yaw in self.directions):return False
        for i in range(1,steps+1):
            a=(u[0]+dr*(i-1)//steps,u[1]+dc*(i-1)//steps);b=(u[0]+dr*i//steps,u[1]+dc*i//steps)
            if not self.safe[b]:return False
            if any((m[a] or m[b]) and dc*math.cos(yaw)-dr*math.sin(yaw)<-1e-6 for m,yaw in self.directions):return False
            if a[0]!=b[0] and a[1]!=b[1] and not(self.safe[a[0],b[1]] and self.safe[b[0],a[1]]):return False
        return True

    def search(self,potential=None,weight=2.,*,timeout=10.,heuristic=True):
        tick=time.monotonic();points=[self.start]+self.waypoints+[self.goal];pieces=[];stats={'expanded':0,'cost_cells':0.}
        for start,goal in zip(points,points[1:]):
            piece,item=self._search(start,goal,potential,weight,timeout=max(.001,timeout-(time.monotonic()-tick)),heuristic=heuristic)
            pieces.append(piece if not pieces else piece[1:]);stats['expanded']+=item['expanded'];stats['cost_cells']+=item['cost_cells']
        stats['search_ms']=(time.monotonic()-tick)*1000
        return np.vstack(pieces),stats

    def _search(self,start,goal,potential=None,weight=2.,*,timeout=10.,heuristic=True):
        tick=time.monotonic()
        if not(self.active[start] and self.active[goal]):raise RuntimeError('LOCAL_FINE_ENDPOINT_UNCONNECTED')
        dist={start:0.};parent={};serial=itertools.count();expanded=0
        h=lambda u:math.dist(u,goal) if heuristic else 0.
        queue=[(h(start),0.,next(serial),start)]
        while queue:
            if time.monotonic()-tick>timeout:raise TimeoutError('LOCAL_FINE_TIMEOUT')
            _,cost,_,u=heapq.heappop(queue)
            if cost!=dist.get(u):continue
            if u==goal:
                path=[u]
                while u!=start:u=parent[u];path.append(u)
                path.reverse()
                return np.asarray(path,int),{'expanded':expanded,'search_ms':(time.monotonic()-tick)*1000,'cost_cells':cost}
            expanded+=1
            neighbors=[(u[0]+r,u[1]+c) for r,c in STEPS]
            if self.coarse_centers[u]:neighbors.extend((u[0]+3*r,u[1]+3*c) for r,c in STEPS)
            for v in neighbors:
                if not self._edge(u,v):continue
                p=0. if potential is None else (float(potential[u])+float(potential[v]))*.5
                ng=cost+math.dist(u,v)*(1+weight*p)
                if ng<dist.get(v,math.inf):
                    dist[v]=ng;parent[v]=u;heapq.heappush(queue,(ng+h(v),ng,next(serial),v))
        raise RuntimeError('LOCAL_FINE_NO_PATH')
