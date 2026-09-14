"""A bounded real-lane rule experiment, NOT the original full R2 acceptance.

Preserve the frozen query poses. Both arms use the same original lane strip.
The rule arm adds a hard center-right predicate over a named longitudinal slab.
The source map's soft route_tangent_right is inspiration, not equivalent to this
experimental predicate. Only a straight two-boundary polygon strip is supported.
"""
import copy
import math
import numpy as np
from .bezier import evaluate

LANE_ID='374682499F8A4131783406408835'
RULE_ID='STL-V0-R1-LOCAL-RIGHT-BAND'


def strip_edges(polygon,ylo,yhi):
    pts=np.asarray(polygon,float)
    if np.any((pts[:,1]>ylo)&(pts[:,1]<yhi)):
        raise ValueError('SEMANTIC_FIXTURE_REQUIRES_STRAIGHT_BOUNDARY_STRIP')
    middle=(ylo+yhi)/2;edges=[]
    for a,b in zip(pts[:-1],pts[1:]):
        if min(a[1],b[1])<middle<max(a[1],b[1]):
            m=(b[0]-a[0])/(b[1]-a[1]);edges.append((float(m),float(a[0]-m*a[1])))
    if len(edges)!=2:raise ValueError('SEMANTIC_FIXTURE_REQUIRES_TWO_BOUNDARIES')
    return sorted(edges,key=lambda e:e[0]*middle+e[1])


def clip_polygon(vertices,halfspaces):
    points=[np.asarray(p,float) for p in vertices]
    for a,b,c in halfspaces:
        normal=np.array([a,b]);out=[]
        for p,q in zip(points,points[1:]+points[:1]):
            fp=float(p@normal-c);fq=float(q@normal-c)
            if fp<=1e-9:out.append(p)
            if (fp<0<fq) or (fq<0<fp):out.append(p+(q-p)*fp/(fp-fq))
        points=out
        if len(points)<3:return []
    return points


def make_lane_fixture(base,semantic,enabled):
    if base['metadata']['query_id']!='cmp2-01-lane-north':raise ValueError('WRONG_FROZEN_QUERY_FOR_FIXTURE')
    feature=next(f for f in semantic['features'] if f['semantic_id']==LANE_ID)
    if feature['direction_rule']!='route_tangent_right':raise ValueError('SOURCE_RULE_CHANGED')
    ylo=base['start'][1]-1.;yhi=base['goal'][1]+1.
    left,right=strip_edges(feature['coordinates'],ylo,yhi)
    center=[(a+b)/2 for a,b in zip(left,right)];band_lo=-20.;band_hi=8.;offset=.35
    corridor=[[-1,left[0],-left[1]],[1,-right[0],right[1]]]
    rule=[-1,center[0],-center[1]-offset]
    xmin=min(left[0]*ylo+left[1],left[0]*yhi+left[1]);xmax=max(right[0]*ylo+right[1],right[0]*yhi+right[1])
    d=copy.deepcopy(base);regions=[]
    for r in base['regions']:
        for part,low,high in [('entry',ylo,band_lo),('band',band_lo-.5,band_hi+.5),('exit',band_hi,yhi)]:
            a,b,c,e=r['bounds'];bounds=[max(a,xmin),max(b,low),min(c,xmax),min(e,high)]
            if bounds[2]-bounds[0]<1e-5 or bounds[3]-bounds[1]<1e-5:continue
            a,b,c,e=bounds;hs=copy.deepcopy(corridor)
            if enabled and part=='band':hs.append(rule)
            polygon=clip_polygon([[a,b],[c,b],[c,e],[a,e]],hs)
            if not polygon:continue
            pts=np.asarray(polygon);bounds=np.r_[pts.min(axis=0),pts.max(axis=0)].tolist()
            item={'id':r['id']+'_'+part,'bounds':bounds,'halfspaces':hs,'labels':[part]+(['sparse_seed_cover'] if 'sparse_seed_cover' in r.get('labels',[]) else [])}
            if enabled and part=='band':item['direction']=[math.pi/2,math.pi/4]
            regions.append(item)
    # Remove redundant boxes only within the same part, predicates and family.
    # Their convex sets are contained, so this preserves each family's union.
    before_pruning=len(regions);kept=[]
    for item in sorted(regions,key=lambda r:-(r['bounds'][2]-r['bounds'][0])*(r['bounds'][3]-r['bounds'][1])):
        b0=item['bounds']
        if any(a['labels']==item['labels'] and a['halfspaces']==item['halfspaces'] and
               a['bounds'][0]<=b0[0]+1e-10 and a['bounds'][1]<=b0[1]+1e-10 and
               a['bounds'][2]>=b0[2]-1e-10 and a['bounds'][3]>=b0[3]-1e-10 for a in kept):continue
        kept.append(item)
    regions=kept;d['metadata']['semantic_regions_before_pruning']=before_pruning
    d['regions']=regions;d['name']='STL-V0 real lane '+('right rule' if enabled else 'no right rule')
    d['task']['ordered_eventually']=[{'label':'band','window_s':[0,200.]}]
    d['metadata'].update(phase='real-lane-local-rule-experiment',semantic_acceptance='LOCAL_PREDICATE_ONLY',
        local_rule={'id':RULE_ID,'enabled':enabled,'source_semantic_id':LANE_ID,'source_rule':'route_tangent_right',
                    'experimental_interpretation':'hard center offset in a bounded straight northbound strip',
                    'not_equivalent_to_original_soft_R2':True,'centerline_x_of_y':center,'right_offset_m':offset,
                    'active_y_interval_m':[band_lo,band_hi],'source_polygon':feature['coordinates'],
                    'fixture_y_interval_m':[ylo,yhi],'source_lane_edges': [left,right]},region_count=len(regions))
    return d


def audit_local_rule(rows,trajectory,rule):
    if not rows:return {'status':'NO_PATH','valid':None,'full_R2_acceptance':None}
    pts=np.array([[r['x'],r['y']] for r in rows]);yaw=np.array([r['yaw'] for r in rows]);m,b=rule['centerline_x_of_y']
    lo,hi=rule['active_y_interval_m'];selected=(pts[:,1]>=lo)&(pts[:,1]<=hi)
    offset=pts[:,0]-(m*pts[:,1]+b);margins=offset[selected]-rule['right_offset_m']
    heading=np.abs(np.arctan2(np.sin(yaw[selected]-math.pi/2),np.cos(yaw[selected]-math.pi/2)))
    # Independent continuous check: isolate every Bézier part that enters the
    # active slab; enforce a Bernstein lower bound, with recursive subdivision.
    from .bezier import split
    nodes=0
    def check(cp,depth=0):
        nonlocal nodes
        nodes+=1;ys=cp[:,1]
        if ys.max()<lo-1e-9 or ys.min()>hi+1e-9:return True
        margin=cp[:,0]-m*cp[:,1]-b-rule['right_offset_m']
        dv=3*np.diff(cp,axis=0)
        if margin.min()>=-1e-7 and np.all(dv[:,1]>=np.abs(dv[:,0])-1e-7):return True
        samples,vel,_=evaluate(cp,np.linspace(0,1,9));inside=(samples[:,1]>=lo)&(samples[:,1]<=hi)
        if np.any(samples[inside,0]-m*samples[inside,1]-b-rule['right_offset_m']< -1e-6):return False
        if np.any(vel[inside,1]<np.abs(vel[inside,0])-1e-6):return False
        if depth>=14:return False
        a,c=split(cp);return check(a,depth+1) and check(c,depth+1)
    # Split at actual cubic roots of the predicate's activation boundaries.
    # This avoids recursively mixing inactive points with active boundary points.
    pieces=[]
    for cp in trajectory['control_points']:
        cp=np.asarray(cp,float);y=cp[:,1]
        coefficient=[y[3]-3*y[2]+3*y[1]-y[0],3*(y[2]-2*y[1]+y[0]),3*(y[1]-y[0]),y[0]]
        cuts=[0.,1.]
        for bound in [lo,hi]:
            poly=coefficient.copy();poly[-1]-=bound
            for root in np.roots(np.trim_zeros(poly,'f')):
                if abs(root.imag)<1e-8 and 0<root.real<1:cuts.append(float(root.real))
        cuts=sorted(set(cuts))
        for u0,u1 in zip(cuts[:-1],cuts[1:]):
            if u1-u0<1e-12:continue
            middle=evaluate(cp,np.array([(u0+u1)/2]))[0][0,1]
            if not lo-1e-9<=middle<=hi+1e-9:continue
            pos,vel,_=evaluate(cp,np.array([u0,u1]));du=u1-u0
            pieces.append(np.array([pos[0],pos[0]+du*vel[0]/3,pos[1]-du*vel[1]/3,pos[1]]))
    continuous=bool(pieces) and all(check(piece) for piece in pieces)
    valid=bool(len(margins) and margins.min()>=-1e-6 and heading.max()<=math.pi/4+1e-6 and continuous)
    return {'status':'PASS' if valid else ('LOCAL_RIGHT_RULE_VIOLATION' if (len(margins) and (margins.min()< -1e-6 or heading.max()>math.pi/4+1e-6)) else 'LOCAL_RULE_NOT_CERTIFIED'),'valid':valid,
            'active_samples':int(selected.sum()),'mean_right_offset_m':float(offset[selected].mean()) if selected.any() else None,
            'minimum_right_margin_m':float(margins.min()) if len(margins) else None,
            'sample_violation_fraction':float(np.mean(margins< -1e-6)) if len(margins) else None,
            'continuous_linear_predicate_check':continuous,'continuous_check_nodes':nodes,
            'method':'independent whole-curve recursive Bernstein predicate plus dense audit',
            'scope':RULE_ID,'full_R2_acceptance':None}
