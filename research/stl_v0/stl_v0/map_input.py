"""Independent fixed-resolution map adapter; no legacy planner imports.

Real-map r0 is a physical feasibility preflight. Semantic source is bound and
preserved; its route-dependent R2 preferences are NOT claimed implemented.
"""
from pathlib import Path
import hashlib
import json
import math
import time
import numpy as np
import yaml
from PIL import Image,ImageDraw
from scipy.ndimage import distance_transform_edt,binary_dilation,label
from .output import sha256


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def load_dataset(map_yaml,semantic_json,query_yaml):
    mp=Path(map_yaml).resolve();sp=Path(semantic_json).resolve();qp=Path(query_yaml).resolve()
    config=yaml.safe_load(mp.read_text());semantic=json.loads(sp.read_text());queries=yaml.safe_load(qp.read_text())
    image=Path(config['image']);image=image if image.is_absolute() else mp.parent/image
    raw=np.asarray(Image.open(image).convert('L'))
    if float(config['resolution'])!=0.05 or float(config['origin'][2])!=0:
        raise ValueError('UNSUPPORTED_MAP_FRAME_OR_RESOLUTION')
    if raw.shape!=(semantic['height'],semantic['width']) or not np.allclose(config['origin'],semantic['origin'],rtol=0,atol=1e-9):
        raise ValueError('MAP_SEMANTIC_FRAME_MISMATCH')
    actual=sha256(image)
    if actual!=queries['map_hash']:
        raise ValueError('MAP_HASH_MISMATCH')
    sem_payload={k:v for k,v in semantic.items() if k!='semantic_map_hash'}
    if canonical_hash(sem_payload)!=semantic['semantic_map_hash'] or semantic['semantic_map_hash']!=queries['semantic_map_hash']:
        raise ValueError('SEMANTIC_HASH_MISMATCH')
    # Pin exact default source file too: metadata hashes alone are insufficient.
    if sha256(qp)!='b3307d4578447131e71db16156cd2f72e9f5042f98bdce0d75d21fe81300738d':
        raise ValueError('FROZEN_SELECTED8_FILE_MISMATCH')
    if len(queries['queries'])!=8 or len({q['query_id'] for q in queries['queries']})!=8:
        raise ValueError('INVALID_SELECTED8')
    for info in queries['intent_validation']:
        ev=info['verification']
        if ev['minimum_endpoint_clearance_m']<1.5 or ev['topology_route_length_m']<=50:
            raise ValueError('QUERY_SELECTION_CONTRACT_MISMATCH')
    prob=raw.astype(np.float32)/255
    if not config.get('negate',0):prob=1-prob
    free=prob<float(config.get('free_thresh',0.196))
    h,w=free.shape;res=0.05;ox,oy,_=config['origin']
    forbidden=np.zeros(free.shape,bool);no_stopping=np.zeros(free.shape,bool)
    semantic_counts={};directions=[]
    for feature in semantic['features']:
        cls=feature['semantic_class'];semantic_counts[cls]=semantic_counts.get(cls,0)+1
        dr=feature.get('direction_rule')
        if dr not in [None,'','none','route_tangent_right']:
            directions.append(feature['semantic_id'])
        if feature.get('hard') or cls in ['forbidden','no_go'] or feature.get('non_stopping'):
            if feature['geometry_type']!='polygon':
                raise ValueError('UNSUPPORTED_HARD_SEMANTIC_GEOMETRY')
            mask=Image.new('1',(w,h));draw=ImageDraw.Draw(mask)
            draw.polygon([((x-ox)/res,h-(y-oy)/res) for x,y in feature['coordinates']],fill=1)
            # Enlarge raster conversion by one cell before body inflation.
            painted=binary_dilation(np.asarray(mask),iterations=1)
            if feature.get('hard') or cls in ['forbidden','no_go']:forbidden|=painted
            if feature.get('non_stopping'):no_stopping|=painted
    if directions:
        raise ValueError('EXPLICIT_DIRECTION_NOT_IMPLEMENTED_IN_REAL_ADAPTER:'+str(directions))
    free &= ~forbidden
    circ=math.hypot(.265,.225)
    # EDT refers to pixel centers. Subtract two half diagonals to cover both
    # the blocked pixel square and every point of an accepted center pixel.
    padded=np.pad(free,1,constant_values=False)
    clearance=distance_transform_edt(padded)[1:-1,1:-1]*res
    safe=clearance>circ+math.sqrt(2)*res+0.002
    evidence={'map_sha256':actual,'map_yaml_sha256':sha256(mp),'semantic_file_sha256':sha256(sp),
              'semantic_content_hash':semantic['semantic_map_hash'],'query_file_sha256':sha256(qp),
              'query_content_hash':queries['query_hash'],'query_set_id':queries['schema_version'],
              'resolution_m':res,'shape':list(raw.shape),'origin':config['origin'],
              'semantic_feature_counts':semantic_counts,'source_paths':[str(mp),str(image),str(sp),str(qp)],
              'cover_body_model':'circumscribed disk of padded rectangle plus pixel-square margin',
              'cover_radius_m':circ,'conservative_margin_m':math.sqrt(2)*res+0.002,
              'raw_semantic_preferences_preserved':True,'r2_preferences_implemented':False,
              'dynamic_obstacles':False,'historical_path_inputs':False,
              'source_file_hashes':{str(p):sha256(p) for p in [mp,image,sp,qp]}}
    return free,safe,no_stopping,config,semantic,queries,evidence


def world_cell(config,shape,point):
    x,y=point[:2];ox,oy,_=config['origin'];r=config['resolution'];h,w=shape
    col=math.floor((x-ox)/r);row=h-1-math.floor((y-oy)/r)
    if not 0<=row<h or not 0<=col<w:raise ValueError('ENDPOINT_OUTSIDE_MAP')
    return row,col


def grow_rectangle(mask,row,col,cap=400,vertical_first=False):
    if vertical_first:
        a,b,c,d=grow_rectangle(mask.T,col,row,cap,False)
        return b,a,d,c
    h,w=mask.shape
    x0=col;x1=col+1
    while x0>max(0,col-cap) and mask[row,x0-1]:x0-=1
    while x1<min(w,col+cap+1) and mask[row,x1]:x1+=1
    # Each included row must be entirely free; no averaging or resampling.
    full=np.all(mask[:,x0:x1],axis=1)
    y0=row;y1=row+1
    while y0>max(0,row-cap) and full[y0-1]:y0-=1
    while y1<min(h,row+cap+1) and full[y1]:y1+=1
    return x0,y0,x1,y1


def make_scene(safe,no_stopping,config,query,evidence,margin_m=20.,seed_stride_m=0.5,cover_mode="repaired"):
    phase_start=time.monotonic();stage_timing={}
    if not math.isfinite(margin_m) or margin_m<=0:
        raise ValueError('INVALID_ROI_MARGIN')
    h,w=safe.shape;r=float(config['resolution']);ox,oy,_=config['origin']
    start=world_cell(config,safe.shape,query['start']);goal=world_cell(config,safe.shape,query['goal'])
    for cell in [start,goal]:
        if no_stopping[cell]:raise ValueError('NO_STOPPING_ENDPOINT')
        if not safe[cell]:raise ValueError('ENDPOINT_REJECTED_BY_CONSERVATIVE_COVER')
    m=int(math.ceil(margin_m/r))
    y0=max(0,min(start[0],goal[0])-m);y1=min(h,max(start[0],goal[0])+m+1)
    x0=max(0,min(start[1],goal[1])-m);x1=min(w,max(start[1],goal[1])+m+1)
    mask=safe[y0:y1,x0:x1];components,_=label(mask)
    connected=bool(components[start[0]-y0,start[1]-x0]==components[goal[0]-y0,goal[1]-x0])
    stride=max(1,int(round(seed_stride_m/r)))
    seeds=[(a-y0,b-x0) for a,b in [start,goal]]
    seeds += [(a,b) for a in range(0,mask.shape[0],stride) for b in range(0,mask.shape[1],stride) if mask[a,b]]
    stage_timing["roi_and_seeds_s"]=time.monotonic()-phase_start;phase_start=time.monotonic()
    rects=set()
    for row,col in seeds:
        for vertical in [False,True]:
            box=grow_rectangle(mask,row,col,vertical_first=vertical)
            if (box[2]-box[0])*r>.12 and (box[3]-box[1])*r>.12:rects.add(box)
    stage_timing["axis_growth_s"]=time.monotonic()-phase_start;phase_start=time.monotonic()
    # Preserve a freshly generated sparse cover as one candidate family.
    # This uses original safe pixels, never historical routes or legacy planners.
    sparse=[]
    for b in sorted(rects,key=lambda b:(-(b[2]-b[0])*(b[3]-b[1]),b)):
        if not any(a[0]<=b[0] and a[1]<=b[1] and a[2]>=b[2] and a[3]>=b[3] for a in sparse):sparse.append(b)
    sparse_set=set(sparse)
    stage_timing["sparse_pruning_s"]=time.monotonic()-phase_start;phase_start=time.monotonic()
    balanced_added=0
    if cover_mode=="repaired":
        from .cover import grow_core
        before=len(rects)
        for row,col in seeds:rects.add(grow_core(mask,(col,row,col+1,row+1)))
        balanced_added=len(rects)-before
    stage_timing["balanced_growth_s"]=time.monotonic()-phase_start;phase_start=time.monotonic()
    # Remove only fully contained boxes, retaining an under-approximate cover.
    ordered=sorted(rects,key=lambda b:(-(b[2]-b[0])*(b[3]-b[1]),b))
    kept=[]
    for b in ordered:
        if any(a[0]<=b[0] and a[1]<=b[1] and a[2]>=b[2] and a[3]>=b[3] for a in kept):continue
        kept.append(b)
    kept=sparse+[b for b in kept if b not in sparse_set]
    if cover_mode not in ["legacy","repaired"]:raise ValueError("INVALID_COVER_MODE")
    stage_timing["combined_pruning_s"]=time.monotonic()-phase_start;phase_start=time.monotonic()
    repair_info=None
    if cover_mode=="repaired":
        from .cover import repair_cover
        kept,repair_info=repair_cover(mask,kept)
    stage_timing["repair_and_verification_s"]=time.monotonic()-phase_start
    regions=[]
    for i,(a,b,c,d) in enumerate(kept):
        regions.append({'id':f'free_{i:04d}','bounds':[ox+(a+x0)*r,oy+(h-(d+y0))*r,ox+(c+x0)*r,oy+(h-(b+y0))*r],
                        'labels':['certified_static_free']+(['sparse_seed_cover'] if (a,b,c,d) in sparse_set else [])})
    return {'name':'STL-V0 real physical preflight '+query['query_id'],'start':query['start'],'goal':query['goal'],
            'regions':regions,'task':{'horizon_s':240.,'always':[{'predicate':'safe'}],'ordered_eventually':[]},
            'metadata':{**evidence,'query_id':query['query_id'],'query_category':query['category'],
                        'roi_cells':[x0,y0,x1,y1],'roi_margin_m':margin_m,'roi_safe_grid_connected':connected,'region_seed_spacing_m':seed_stride_m,
                        'cover_stage_timing':stage_timing,'sparse_cover_regions':len(sparse),'cover_mode':cover_mode,'balanced_regions_added_before_pruning':balanced_added,'cover_repair':repair_info,'cover_is_complete':False,'phase':'physical-preflight-only',
                        'semantic_acceptance':'NOT_EVALUATED','region_count':len(regions)}}
